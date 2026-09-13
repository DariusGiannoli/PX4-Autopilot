// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "ControlAllocator.hpp"
#include <uORB/Publication.hpp>
#include <uORB/topics/morphing_allocation_matrix.h>

class ControlAllocatorMorphingTestPeer
{
public:
	static int publishTestAngles(ControlAllocator &module, int argc, char *argv[])
	{
		return module.publishTestAngles(argc, argv);
	}
	static void reload(ControlAllocator &module)
	{
		module.updateParams();
		module.parameters_updated();
	}
	static const ActuatorEffectiveness::Configuration &configuration(ControlAllocator &module)
	{
		return module._effectiveness_update.config;
	}
};

TEST(ControlAllocatorMorphing, SelectingAndReconfiguringUsesFreshBuffers)
{
	param_control_autosave(false);
	int32_t airframe = 0, count = 4;
	ASSERT_EQ(param_set(param_find("CA_AIRFRAME"), &airframe), 0);
	ASSERT_EQ(param_set(param_find("CA_ROTOR_COUNT"), &count), 0);
	ControlAllocator module;
	uORB::Subscription applied_sub{ORB_ID(morphing_allocation_matrix)};
	uORB::Publication<morphing_arm_state_s> input{ORB_ID(morphing_arm_state)};
	morphing_arm_state_s state{};
	morphing_allocation_matrix_s applied{}, neutral{};

	for (int cycle = 0; cycle < 3; ++cycle) {
		airframe = 13;
		ASSERT_EQ(param_set(param_find("CA_AIRFRAME"), &airframe), 0);
		state.timestamp = 100 + cycle * 10;
		state.angle[1] = 0.f;
		ASSERT_TRUE(input.publish(state));
		ControlAllocatorMorphingTestPeer::reload(module);
		ASSERT_TRUE(applied_sub.update(&neutral));
		EXPECT_EQ(neutral.timestamp_angle, state.timestamp);
		const auto &config = ControlAllocatorMorphingTestPeer::configuration(module);
		EXPECT_EQ(config.num_actuators_matrix[0], 4);
		EXPECT_EQ(config.num_actuators_matrix[1], 0);
		EXPECT_EQ(config.num_actuators[0], 4);
		EXPECT_EQ(config.num_actuators[1], 0);
		for (int row = 0; row < ActuatorEffectiveness::NUM_AXES; ++row) {
			for (int motor = 0; motor < ActuatorEffectiveness::NUM_ACTUATORS; ++motor) {
				EXPECT_FLOAT_EQ(config.effectiveness_matrices[1](row, motor), 0.f);
			}
		}

		++state.timestamp;
		state.angle[1] = 0.3f;
		ASSERT_TRUE(input.publish(state));
		ControlAllocatorMorphingTestPeer::reload(module);
		ASSERT_TRUE(applied_sub.update(&applied));
		EXPECT_EQ(applied.timestamp_angle, state.timestamp);
		EXPECT_EQ(applied.matrix_generation, neutral.matrix_generation + 1);
		EXPECT_NE(applied.effectiveness[3], neutral.effectiveness[3]);
		EXPECT_EQ(config.num_actuators_matrix[0], 4); // Must not accumulate on reuse.

		airframe = 0;
		ASSERT_EQ(param_set(param_find("CA_AIRFRAME"), &airframe), 0);
		ControlAllocatorMorphingTestPeer::reload(module);
		EXPECT_FALSE(applied_sub.update(&applied));
	}
}

TEST(ControlAllocatorMorphing, ConsoleTestPublishesAnglesAndRejectsUnsafeInputs)
{
	param_control_autosave(false);
	int32_t airframe = 13;
	ASSERT_EQ(param_set(param_find("CA_AIRFRAME"), &airframe), 0);
	uORB::Publication<vehicle_status_s> status_pub{ORB_ID(vehicle_status)};
	uORB::Subscription input_sub{ORB_ID(morphing_arm_state)};
	vehicle_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	ASSERT_TRUE(status_pub.publish(status));
	ControlAllocator module;
	char command[] = "morphing_test", a[] = "10", b[] = "-30", c[] = "30", d[] = "0";
	char *args[] = {command, a, b, c, d};
	ASSERT_EQ(ControlAllocatorMorphingTestPeer::publishTestAngles(module, 5, args), 0);
	morphing_arm_state_s state{};
	ASSERT_TRUE(input_sub.update(&state));
	EXPECT_EQ(orb_exists(ORB_ID(morphing_arm_state), 0), PX4_OK);
	EXPECT_GT(state.timestamp, 0u);
	EXPECT_NEAR(state.angle[0], 0.174532925f, 1e-7f);
	EXPECT_NEAR(state.angle[1], -0.523598776f, 1e-7f);
	EXPECT_NEAR(state.angle[2], 0.523598776f, 1e-7f);
	EXPECT_FLOAT_EQ(state.angle[3], 0.f);

	char invalid[][8] = {"31", "nan", "inf", "10junk", ""};
	for (auto &value : invalid) {
		args[1] = value;
		EXPECT_NE(ControlAllocatorMorphingTestPeer::publishTestAngles(module, 5, args), 0);
		EXPECT_FALSE(input_sub.update(&state));
	}
	args[1] = a;
	EXPECT_NE(ControlAllocatorMorphingTestPeer::publishTestAngles(module, 4, args), 0);
	status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	ASSERT_TRUE(status_pub.publish(status));
	EXPECT_NE(ControlAllocatorMorphingTestPeer::publishTestAngles(module, 5, args), 0);
	EXPECT_FALSE(input_sub.update(&state));
	status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	status.timestamp = 0;
	ASSERT_TRUE(status_pub.publish(status));
	EXPECT_NE(ControlAllocatorMorphingTestPeer::publishTestAngles(module, 5, args), 0);
	EXPECT_FALSE(input_sub.update(&state));

	status.timestamp = hrt_absolute_time();
	ASSERT_TRUE(status_pub.publish(status));
	char *neutral_args[] = {command, d, d, d, d};
	ASSERT_EQ(ControlAllocatorMorphingTestPeer::publishTestAngles(module, 5, neutral_args), 0);
	ASSERT_TRUE(input_sub.update(&state));
	EXPECT_EQ(orb_exists(ORB_ID(morphing_arm_state), 0), PX4_OK);
	for (float angle : state.angle) {
		EXPECT_FLOAT_EQ(angle, 0.f);
	}
	const param_t publisher_enable = param_find("MORPH_PUB_EN");
	if (publisher_enable != PARAM_INVALID) {
		int32_t enabled = 1;
		ASSERT_EQ(param_set(publisher_enable, &enabled), 0);
		EXPECT_NE(ControlAllocatorMorphingTestPeer::publishTestAngles(module, 5, neutral_args), 0);
		EXPECT_FALSE(input_sub.update(&state));
		enabled = 0;
		ASSERT_EQ(param_set(publisher_enable, &enabled), 0);
	}
}

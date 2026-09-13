// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "MorphingArmPublisher.hpp"
#include <drivers/drv_hrt.h>
#include <ActuatorEffectivenessMorphingQuad.hpp>

class MorphingArmPublisherTestPeer
{
public:
	static bool configure(MorphingArmPublisher &m) { return m.configure(); }
	static void run(MorphingArmPublisher &m) { m.Run(); }
	static void process(MorphingArmPublisher &m, const vehicle_command_s &c, uint64_t now) { m.process(c, now); }
	static void timeout(MorphingArmPublisher &m, uint64_t now) { m.checkTimeout(now); }
	static void resetTime(MorphingArmPublisher &m) { m._start_time = 1; m._last_seen = 1; }
	static uint8_t mask(const MorphingArmPublisher &m) { return m._state.initialized_mask; }
	static const morphing_publisher_status_s &status(const MorphingArmPublisher &m) { return m._status; }
};

class MorphingArmPublisherTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		param_control_autosave(false);
		setInt("CA_AIRFRAME", 13);
		setInt("CA_ROTOR_COUNT", 4);
		setInt("MORPH_PUB_EN", 1);
		setFloat("MORPH_PUB_TMO", 1.f);
		for (const char *arm : {"FR", "RR", "RL", "FL"}) {
			char name[17];
			snprintf(name, sizeof(name), "MARM_%s_NEG", arm); setFloat(name, -30.f);
			snprintf(name, sizeof(name), "MARM_%s_ZERO", arm); setFloat(name, 0.f);
			snprintf(name, sizeof(name), "MARM_%s_CTR", arm); setFloat(name, 0.f);
			snprintf(name, sizeof(name), "MARM_%s_POS", arm); setFloat(name, 30.f);
		}
	}
	void TearDown() override { setInt("MORPH_PUB_EN", 0); }
	static void setInt(const char *name, int32_t value) { ASSERT_EQ(param_set(param_find(name), &value), 0); }
	static void setFloat(const char *name, float value) { ASSERT_EQ(param_set(param_find(name), &value), 0); }
	static vehicle_command_s command(uint64_t timestamp, float a, float b, float c, float d)
	{
		vehicle_command_s out{};
		out.timestamp = timestamp;
		out.command = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;
		out.param1 = a; out.param2 = b; out.param3 = c; out.param4 = d;
		return out;
	}
};

TEST_F(MorphingArmPublisherTest, CalibrationGateAndMonotonicCurves)
{
	MorphingArmPublisher module;
	setInt("MORPH_PUB_EN", 0);
	EXPECT_FALSE(MorphingArmPublisherTestPeer::configure(module));
	setInt("MORPH_PUB_EN", 1);
	setFloat("MARM_FR_POS", 0.f);
	EXPECT_FALSE(MorphingArmPublisherTestPeer::configure(module));
	setFloat("MARM_FR_POS", 31.f);
	EXPECT_FALSE(MorphingArmPublisherTestPeer::configure(module));
	setFloat("MARM_FR_POS", NAN);
	EXPECT_FALSE(MorphingArmPublisherTestPeer::configure(module));
	setFloat("MARM_FR_POS", 30.f);
	setFloat("MARM_RR_NEG", 25.f);
	setFloat("MARM_RR_ZERO", 5.f);
	setFloat("MARM_RR_POS", -15.f);
	ASSERT_TRUE(MorphingArmPublisherTestPeer::configure(module));
	MorphingArmPublisherTestPeer::resetTime(module);
	uORB::Subscription sub{ORB_ID(morphing_arm_state)};
	const auto c = command(10, 1.f, -.5f, -1.f, .5f);
	MorphingArmPublisherTestPeer::process(module, c, 11);
	morphing_arm_state_s state{};
	ASSERT_TRUE(sub.update(&state));
	EXPECT_NEAR(state.angle[0], .523598776f, 1e-7f);
	EXPECT_NEAR(state.angle[1], .261799388f, 1e-7f);
	EXPECT_NEAR(state.angle[2], -.523598776f, 1e-7f);
	EXPECT_NEAR(state.angle[3], .261799388f, 1e-7f);
}

TEST_F(MorphingArmPublisherTest, PartialUpdatesInvalidInputsAndIncreasingTimestamp)
{
	MorphingArmPublisher module;
	ASSERT_TRUE(MorphingArmPublisherTestPeer::configure(module));
	MorphingArmPublisherTestPeer::resetTime(module);
	uORB::Subscription sub{ORB_ID(morphing_arm_state)};
	morphing_arm_state_s state{};
	sub.copy(&state);
	auto c = command(10, 1.f / 3.f, NAN, NAN, NAN);
	MorphingArmPublisherTestPeer::process(module, c, 20);
	EXPECT_EQ(MorphingArmPublisherTestPeer::mask(module), 1);
	EXPECT_FALSE(sub.update(&state));
	c = command(11, NAN, 0.f, -.5f, .5f);
	MorphingArmPublisherTestPeer::process(module, c, 20);
	ASSERT_TRUE(sub.update(&state));
	EXPECT_NEAR(state.angle[0], .174532925f, 1e-7f);
	const uint64_t first = state.timestamp;
	c = command(12, NAN, .5f, NAN, NAN);
	MorphingArmPublisherTestPeer::process(module, c, 20);
	ASSERT_TRUE(sub.update(&state));
	EXPECT_GT(state.timestamp, first);
	EXPECT_NEAR(state.angle[0], .174532925f, 1e-7f);
	EXPECT_NEAR(state.angle[1], .261799388f, 1e-7f);
	for (float bad : {1.01f, INFINITY, -INFINITY}) {
		c = command(++c.timestamp, 0.f, bad, NAN, NAN);
		MorphingArmPublisherTestPeer::process(module, c, 20);
		EXPECT_FALSE(sub.update(&state));
	}
	c = command(18, NAN, NAN, NAN, NAN);
	MorphingArmPublisherTestPeer::process(module, c, 20);
	EXPECT_FALSE(sub.update(&state));
	c = command(19, 0.f, NAN, NAN, NAN); c.param7 = 1.f;
	MorphingArmPublisherTestPeer::process(module, c, 20);
	EXPECT_FALSE(sub.update(&state));
	c.param7 = 0.f; c.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	MorphingArmPublisherTestPeer::process(module, c, 20);
	EXPECT_FALSE(sub.update(&state));
	c = command(12, 0.f, 0.f, 0.f, 0.f); // replay
	MorphingArmPublisherTestPeer::process(module, c, 20);
	EXPECT_FALSE(sub.update(&state));
	EXPECT_EQ(MorphingArmPublisherTestPeer::status(module).rejected_commands, 4u);
}

TEST_F(MorphingArmPublisherTest, TimeoutRequiresFreshInitializationAndDoesNotRepublish)
{
	MorphingArmPublisher module;
	ASSERT_TRUE(MorphingArmPublisherTestPeer::configure(module));
	MorphingArmPublisherTestPeer::resetTime(module);
	uORB::Subscription sub{ORB_ID(morphing_arm_state)};
	morphing_arm_state_s state{};
	MorphingArmPublisherTestPeer::process(module, command(10, 0.f, 0.f, 0.f, 0.f), 11);
	ASSERT_TRUE(sub.update(&state));
	MorphingArmPublisherTestPeer::timeout(module, 1000011);
	EXPECT_TRUE(MorphingArmPublisherTestPeer::status(module).stale);
	EXPECT_EQ(MorphingArmPublisherTestPeer::mask(module), 0);
	EXPECT_FALSE(sub.update(&state));
	MorphingArmPublisherTestPeer::process(module, command(1000012, 1.f, NAN, NAN, NAN), 1000013);
	EXPECT_FALSE(sub.update(&state));
	MorphingArmPublisherTestPeer::timeout(module, 2000013);
	EXPECT_EQ(MorphingArmPublisherTestPeer::mask(module), 0); // Partial initialization also expires.
	MorphingArmPublisherTestPeer::process(module, command(2000014, 0.f, 0.f, 0.f, 0.f), 2000015);
	ASSERT_TRUE(sub.update(&state));
	EXPECT_FALSE(MorphingArmPublisherTestPeer::status(module).stale);
}

TEST_F(MorphingArmPublisherTest, RealCommandSubscriptionAndDisableGate)
{
	MorphingArmPublisher module;
	ASSERT_TRUE(MorphingArmPublisherTestPeer::configure(module));
	MorphingArmPublisherTestPeer::resetTime(module);
	uORB::Publication<vehicle_command_s> pub{ORB_ID(vehicle_command)};
	uORB::Subscription sub{ORB_ID(morphing_arm_state)};
	morphing_arm_state_s state{};
	sub.copy(&state);
	ASSERT_TRUE(pub.publish(command(hrt_absolute_time(), 1.f / 3.f, 0.f, 0.f, 0.f)));
	MorphingArmPublisherTestPeer::run(module);
	ASSERT_TRUE(sub.update(&state));
	EXPECT_NEAR(state.angle[0], .174532925f, 1e-7f);
	for (unsigned i = 0; i < 4; ++i) {
		char name[17];
		snprintf(name, sizeof(name), "CA_ROTOR%u_CT", i);
		setFloat(name, 6.5f);
	}
	ActuatorEffectivenessMorphingQuad provider(nullptr);
	ActuatorEffectiveness::Configuration configuration{};
	ASSERT_TRUE(provider.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
	EXPECT_EQ(configuration.num_actuators_matrix[0], 4);
	EXPECT_NEAR(configuration.effectiveness_matrices[0](0, 0), -.39194f, 1e-5f);
	EXPECT_NEAR(configuration.effectiveness_matrices[0](1, 0), .48254f, 1e-5f);
	setInt("MORPH_PUB_EN", 0);
	ASSERT_TRUE(pub.publish(command(hrt_absolute_time(), 0.f, 0.f, 0.f, 0.f)));
	MorphingArmPublisherTestPeer::run(module);
	EXPECT_FALSE(sub.update(&state));
	EXPECT_EQ(MorphingArmPublisherTestPeer::mask(module), 0);
}

TEST_F(MorphingArmPublisherTest, QueueLossInvalidatesRetainedTargets)
{
	MorphingArmPublisher module;
	ASSERT_TRUE(MorphingArmPublisherTestPeer::configure(module));
	MorphingArmPublisherTestPeer::resetTime(module);
	uORB::Publication<vehicle_command_s> pub{ORB_ID(vehicle_command)};
	uORB::Subscription sub{ORB_ID(morphing_arm_state)};
	morphing_arm_state_s state{};
	const uint64_t stamp = hrt_absolute_time();
	ASSERT_TRUE(pub.publish(command(stamp, 0.f, 0.f, 0.f, 0.f)));
	MorphingArmPublisherTestPeer::run(module);
	ASSERT_TRUE(sub.update(&state));
	for (unsigned i = 0; i < 12; ++i) {
		ASSERT_TRUE(pub.publish(command(stamp, .5f, NAN, NAN, NAN)));
	}
	MorphingArmPublisherTestPeer::run(module);
	EXPECT_FALSE(sub.update(&state));
	EXPECT_GT(MorphingArmPublisherTestPeer::status(module).queue_overruns, 0u);
	EXPECT_EQ(MorphingArmPublisherTestPeer::mask(module), 0);
}

TEST_F(MorphingArmPublisherTest, ShiftedCenterPublishesNeutralAndCorrectTravel)
{
    setFloat("MARM_RR_CTR", -.16f);
    setFloat("MARM_RL_CTR", .1f);
    MorphingArmPublisher module;
    ASSERT_TRUE(MorphingArmPublisherTestPeer::configure(module));
    MorphingArmPublisherTestPeer::resetTime(module);
    uORB::Subscription sub{ORB_ID(morphing_arm_state)};
    morphing_arm_state_s state{};
    MorphingArmPublisherTestPeer::process(module, command(10, 0.f, -.16f, .1f, 0.f), 11);
    ASSERT_TRUE(sub.update(&state));
    for (float angle : state.angle) { EXPECT_NEAR(angle, 0.f, 1e-7f); }
    // 40% of each center-to-positive-endpoint span is +12 degrees.
    MorphingArmPublisherTestPeer::process(module, command(12, .4f, .304f, .46f, .4f), 13);
    ASSERT_TRUE(sub.update(&state));
    for (float angle : state.angle) { EXPECT_NEAR(angle, .20943951f, 1e-7f); }
    MorphingArmPublisherTestPeer::process(module, command(14, -1.f, -1.f, -1.f, -1.f), 15);
    ASSERT_TRUE(sub.update(&state));
    for (float angle : state.angle) { EXPECT_NEAR(angle, -.523598776f, 1e-7f); }
    setFloat("MARM_RR_CTR", 1.f);
    EXPECT_FALSE(MorphingArmPublisherTestPeer::configure(module));
    setFloat("MARM_RR_CTR", NAN);
    EXPECT_FALSE(MorphingArmPublisherTestPeer::configure(module));
}

// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <map>
#include <string>

// SITL has no hardware MIN/MAX/DIS parameters. Substitute their storage only;
// execute the production configuration validator and command/model path.
static std::map<std::string, int32_t> measured_parameters;
static int measured_read(const char *name, int32_t *value)
{
    auto found = measured_parameters.find(name);
    if (found == measured_parameters.end()) { return -1; }
    *value = found->second; return 0;
}

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
	static void advance(MorphingArmPublisher &m, uint64_t now) { m.advanceMeasured(now); m.publishMeasured(now); }
	static bool measured(MorphingArmPublisher &m) { m._measured = m.configureMeasured(measured_read); return m._measured; }
	static bool fault(const MorphingArmPublisher &m) { return m._model_fault; }
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
		setInt("MARM_MODEL", 0);
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
	static void measuredConfig()
	{
		measured_parameters.clear();
		measured_parameters["MARM_CAL_ID"] = morphing_publisher::measured::CALIBRATION_ID;
		measured_parameters["PWM_MAIN_REV"] = 0;
		measured_parameters["PWM_MAIN_TIM1"] = 50;
		measured_parameters["PWM_MAIN_TIM2"] = 50;
		const unsigned pins[] = {7, 6, 8, 5};
		for (unsigned i = 0; i < 4; ++i) {
			char name[17];
			snprintf(name, sizeof(name), "PWM_MAIN_FUNC%u", pins[i]); measured_parameters[name] = 301+i;
			snprintf(name, sizeof(name), "PWM_MAIN_MIN%u", pins[i]); measured_parameters[name] = 1150;
			snprintf(name, sizeof(name), "PWM_MAIN_MAX%u", pins[i]); measured_parameters[name] = 1750;
			snprintf(name, sizeof(name), "PWM_MAIN_DIS%u", pins[i]); measured_parameters[name] = 1450;
		}
		for (const char *arm : {"FR", "RR", "RL", "FL"}) {
			char name[17]; snprintf(name, sizeof(name), "MARM_%s_DIR", arm); measured_parameters[name] = 1;
		}
	}
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

TEST_F(MorphingArmPublisherTest, MeasuredRoutingAndFiniteMotion)
{
    measuredConfig();
    MorphingArmPublisher module;
    ASSERT_TRUE(MorphingArmPublisherTestPeer::configure(module));
    ASSERT_TRUE(MorphingArmPublisherTestPeer::measured(module));
    MorphingArmPublisherTestPeer::resetTime(module);
    uORB::Subscription sub{ORB_ID(morphing_arm_state)};
    morphing_arm_state_s state{};
    MorphingArmPublisherTestPeer::process(module, command(1000, 0.f, 0.f, 0.f, 0.f), 1000);
    MorphingArmPublisherTestPeer::advance(module, 1000);
    ASSERT_TRUE(sub.update(&state));
    for (float angle : state.angle) { EXPECT_NEAR(angle, 0.f, 1e-6f); }
    // Set2 is RL. New target must not instantly change estimated geometry.
    MorphingArmPublisherTestPeer::process(module, command(10000, 0.f, .3f, 0.f, 0.f), 10000);
    MorphingArmPublisherTestPeer::advance(module, 10000);
    ASSERT_TRUE(sub.update(&state));
    for (float angle : state.angle) { EXPECT_NEAR(angle, 0.f, 1e-6f); }
    MorphingArmPublisherTestPeer::advance(module, 80000);
    ASSERT_TRUE(sub.update(&state));
    EXPECT_GT(state.angle[2], .1f);
    EXPECT_NEAR(state.angle[0], 0.f, 1e-6f);
    EXPECT_NEAR(state.angle[1], 0.f, 1e-6f);
    EXPECT_NEAR(state.angle[3], 0.f, 1e-6f);
    EXPECT_FALSE(MorphingArmPublisherTestPeer::fault(module));
}

TEST_F(MorphingArmPublisherTest, MeasuredStaleHistoryRequiresRestart)
{
    measuredConfig();
    MorphingArmPublisher module;
    ASSERT_TRUE(MorphingArmPublisherTestPeer::configure(module));
    ASSERT_TRUE(MorphingArmPublisherTestPeer::measured(module));
    MorphingArmPublisherTestPeer::resetTime(module);
    MorphingArmPublisherTestPeer::process(module, command(1000, 0.f, 0.f, 0.f, 0.f), 1000);
    MorphingArmPublisherTestPeer::timeout(module, 2000000);
    EXPECT_TRUE(MorphingArmPublisherTestPeer::fault(module));
    MorphingArmPublisherTestPeer::process(module, command(2000001, 0.f, 0.f, 0.f, 0.f), 2000001);
    EXPECT_TRUE(MorphingArmPublisherTestPeer::status(module).stale);
    EXPECT_EQ(MorphingArmPublisherTestPeer::mask(module), 0);
}

TEST_F(MorphingArmPublisherTest, MeasuredConfigurationAndInitialNeutralGate)
{
    measuredConfig();
    MorphingArmPublisher module;
    measured_parameters["PWM_MAIN_DIS5"] = 1300;
    EXPECT_FALSE(MorphingArmPublisherTestPeer::measured(module));
    measured_parameters["PWM_MAIN_DIS5"] = 1450;
    ASSERT_TRUE(MorphingArmPublisherTestPeer::configure(module));
    ASSERT_TRUE(MorphingArmPublisherTestPeer::measured(module));
    MorphingArmPublisherTestPeer::resetTime(module);
    MorphingArmPublisherTestPeer::process(module, command(1000, .2f, 0.f, 0.f, 0.f), 1000);
    EXPECT_TRUE(MorphingArmPublisherTestPeer::fault(module));
    EXPECT_EQ(MorphingArmPublisherTestPeer::mask(module), 0);
}

TEST_F(MorphingArmPublisherTest, MeasuredAllFourChannelsAndDirectionSigns)
{
    measuredConfig();
    measured_parameters["MARM_FL_DIR"] = -1;
    MorphingArmPublisher module;
    ASSERT_TRUE(MorphingArmPublisherTestPeer::configure(module));
    ASSERT_TRUE(MorphingArmPublisherTestPeer::measured(module));
    MorphingArmPublisherTestPeer::resetTime(module);
    uORB::Subscription sub{ORB_ID(morphing_arm_state)};
    MorphingArmPublisherTestPeer::process(module, command(1000, 0.f, 0.f, 0.f, 0.f), 1000);
    MorphingArmPublisherTestPeer::process(module, command(10000, .1f, .2f, .3f, .4f), 10000);
    for (uint64_t now = 60000; now <= 260000; now += 50000) {
        MorphingArmPublisherTestPeer::advance(module, now);
    }
    morphing_arm_state_s state{};
    ASSERT_TRUE(sub.update(&state));
    // Measured lookup expectations for PWM 1480,1570,1510,1540 in canonical order.
    const float degrees[] = {3.141499f, 13.020228f, 6.550470f, -9.800842f};
    for (unsigned i = 0; i < 4; ++i) {
        EXPECT_NEAR(state.angle[i], degrees[i] * .01745329252f, .00035f);
    }
}

TEST_F(MorphingArmPublisherTest, MeasuredSchedulerGapStopsGeometry)
{
    measuredConfig();
    MorphingArmPublisher module;
    ASSERT_TRUE(MorphingArmPublisherTestPeer::configure(module));
    ASSERT_TRUE(MorphingArmPublisherTestPeer::measured(module));
    MorphingArmPublisherTestPeer::resetTime(module);
    MorphingArmPublisherTestPeer::process(module, command(1000, 0.f, 0.f, 0.f, 0.f), 1000);
    MorphingArmPublisherTestPeer::advance(module, 102000);
    EXPECT_TRUE(MorphingArmPublisherTestPeer::fault(module));
    EXPECT_TRUE(MorphingArmPublisherTestPeer::status(module).stale);
    EXPECT_EQ(MorphingArmPublisherTestPeer::mask(module), 0);
}

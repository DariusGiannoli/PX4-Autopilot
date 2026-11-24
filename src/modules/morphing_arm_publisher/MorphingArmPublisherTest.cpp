// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <map>
#include <string>

// SITL has no hardware MIN/MAX/DIS parameters. Substitute their storage only;
// execute the production configuration validator and command/model path.
static std::map<std::string, int32_t> measured_parameters;
static bool trimmed_commands = false;
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
	static void setAngle(MorphingArmPublisher &m, float degrees) { m._models[0].angle = degrees; }
	static const morphing_publisher_status_s &status(const MorphingArmPublisher &m) { return m._status; }
};

class MorphingArmPublisherTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		param_control_autosave(false);
        trimmed_commands = false;
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
        trimmed_commands = true;
		measured_parameters["MARM_CAL_ID"] = morphing_publisher::measured::CALIBRATION_ID;
		measured_parameters["PWM_MAIN_REV"] = 0;
		measured_parameters["PWM_MAIN_TIM1"] = 50;
		measured_parameters["PWM_MAIN_TIM2"] = 50;
		for (unsigned i = 0; i < 4; ++i) {
            const unsigned pin = morphing_publisher::measured::OUTPUT_PINS[i];
            char name[17];
            snprintf(name, sizeof(name), "PWM_MAIN_FUNC%u", pin); measured_parameters[name] = morphing_publisher::measured::OUTPUT_FUNCTIONS[i];
            snprintf(name, sizeof(name), "PWM_MAIN_MIN%u", pin); measured_parameters[name] = 1150;
            snprintf(name, sizeof(name), "PWM_MAIN_MAX%u", pin); measured_parameters[name] = 1750;
            snprintf(name, sizeof(name), "PWM_MAIN_DIS%u", pin); measured_parameters[name] = morphing_publisher::measured::NEUTRAL_PWM[i];
        }
		for (const char *arm : {"FR", "RR", "RL", "FL"}) {
			char name[17]; snprintf(name, sizeof(name), "MARM_%s_DIR", arm); measured_parameters[name] = 1;
		}
	}
	static vehicle_command_s command(uint64_t timestamp, float a, float b, float c, float d)
	{
		vehicle_command_s out{};
        if (trimmed_commands) {
            // Keep fixture arguments in historical FR/RL/FL/RR logical order.
            const float canonical[] = {a, d, b, c};
            float channels[4]{};
            for (unsigned i = 0; i < 4; ++i) {
                channels[morphing_publisher::measured::COMMAND_INDEX[i]] = canonical[i]
                    + (morphing_publisher::measured::NEUTRAL_PWM[i] - 1450.f) / 300.f;
            }
            a = channels[0]; b = channels[1]; c = channels[2]; d = channels[3];
        }
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
    // Fixture second argument selects RL; generated routing sends it to set1.
    // New target must not instantly change estimated geometry.
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
    // Fresh parameter storage must select the same contract as the generated table.
    int32_t default_calibration = 0;
    ASSERT_EQ(param_get_system_default_value(param_find("MARM_CAL_ID"), &default_calibration), PX4_OK);
    EXPECT_EQ(default_calibration, morphing_publisher::measured::CALIBRATION_ID);
    measuredConfig();
    MorphingArmPublisher module;
    measured_parameters["PWM_MAIN_DIS5"] = 1300;
    EXPECT_FALSE(MorphingArmPublisherTestPeer::measured(module));
    measured_parameters["PWM_MAIN_DIS5"] = 1500;
    ASSERT_TRUE(MorphingArmPublisherTestPeer::configure(module));
    ASSERT_TRUE(MorphingArmPublisherTestPeer::measured(module));
    MorphingArmPublisherTestPeer::resetTime(module);
    MorphingArmPublisherTestPeer::process(module, command(1000, .2f, 0.f, 0.f, 0.f), 1000);
    EXPECT_TRUE(MorphingArmPublisherTestPeer::fault(module));
    EXPECT_EQ(MorphingArmPublisherTestPeer::mask(module), 0);
}

TEST_F(MorphingArmPublisherTest, MeasuredPwmRefreshRequiresMatchedSupportedGroups)
{
    for (int32_t rate : {50, 200}) {
        measuredConfig();
        measured_parameters["PWM_MAIN_TIM1"] = rate;
        measured_parameters["PWM_MAIN_TIM2"] = rate;
        MorphingArmPublisher module;
        EXPECT_TRUE(MorphingArmPublisherTestPeer::measured(module)) << "rate=" << rate;
    }
    const int32_t rejected[][2] = {{50, 200}, {200, 50}, {100, 100},
                                 {400, 400}, {0, 0}, {-1, -1}, {200, 400}};
    for (const auto &rates : rejected) {
        measuredConfig();
        measured_parameters["PWM_MAIN_TIM1"] = rates[0];
        measured_parameters["PWM_MAIN_TIM2"] = rates[1];
        MorphingArmPublisher module;
        EXPECT_FALSE(MorphingArmPublisherTestPeer::measured(module))
                << "rates=" << rates[0] << "," << rates[1];
    }
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
    // RR-reference table at 1530,1620,1560,1590; physical channel trims
    // and FL direction are independent of the table reference neutral.
    const float degrees[] = {1.373548f, 11.517634f, 4.810225f, -8.267678f};
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

TEST_F(MorphingArmPublisherTest, TrimmedTwentyDegreeReversalsPublishUnclippedGeometry)
{
    measuredConfig();
    MorphingArmPublisher module;
    ASSERT_TRUE(MorphingArmPublisherTestPeer::configure(module));
    ASSERT_TRUE(MorphingArmPublisherTestPeer::measured(module));
    MorphingArmPublisherTestPeer::resetTime(module);
    uORB::Subscription sub{ORB_ID(morphing_arm_state)};
    MorphingArmPublisherTestPeer::process(module, command(1000, 0.f, 0.f, 0.f, 0.f), 1000);
    uint64_t now = 1000;
    float peak = 0.f;
    // Virtual reference-table endpoints near +20 / -20 deg; command() adds trims.
    // Full reversals stress the dynamics more than the normal 3-deg ramp.
    for (int pwm : {1698, 1310, 1698, 1310}) {
        const float wire = (pwm - morphing_publisher::measured::REFERENCE_NEUTRAL_PWM) / 300.f;
        for (unsigned tick = 0; tick < 60; ++tick) {
            now += 10000;
            if (tick % 10 == 0) {
                // RL stays neutral just as in the current L1 test.
                MorphingArmPublisherTestPeer::process(module, command(now, wire, 0.f, wire, wire), now);
            }
            MorphingArmPublisherTestPeer::advance(module, now);
            ASSERT_FALSE(MorphingArmPublisherTestPeer::fault(module));
            morphing_arm_state_s state{};
            ASSERT_TRUE(sub.update(&state));
            EXPECT_NEAR(state.angle[2], 0.f, 1e-6f);
            peak = fmaxf(peak, fabsf(state.angle[0]));
            morphing_quad::GeometryResult geometry{};
            const morphing_quad::RotorValues angles{{state.angle[0], state.angle[1], state.angle[2], state.angle[3]}};
            EXPECT_TRUE(morphing_quad::compute(angles, {{1.f, 1.f, 1.f, 1.f}},
                                              {{-.05f, -.05f, .05f, .05f}}, {}, geometry));
        }
    }
    EXPECT_GT(peak, .34f); // Overshoot was published, not hidden by clipping.
    EXPECT_LT(peak, morphing_quad::MAX_ANGLE_RAD);
}

TEST_F(MorphingArmPublisherTest, PredictionOutsideEnvelopeStillFaults)
{
    measuredConfig();
    MorphingArmPublisher module;
    ASSERT_TRUE(MorphingArmPublisherTestPeer::configure(module));
    ASSERT_TRUE(MorphingArmPublisherTestPeer::measured(module));
    MorphingArmPublisherTestPeer::resetTime(module);
    MorphingArmPublisherTestPeer::process(module, command(1000, 0.f, 0.f, 0.f, 0.f), 1000);
    MorphingArmPublisherTestPeer::setAngle(module, 36.f);
    MorphingArmPublisherTestPeer::advance(module, 1000);
    EXPECT_TRUE(MorphingArmPublisherTestPeer::fault(module));
    EXPECT_TRUE(MorphingArmPublisherTestPeer::status(module).stale);
}

TEST_F(MorphingArmPublisherTest, RRLowPwmAndReversalsKeepProductionPublisherAndAllocatorUpdating)
{
    measuredConfig();
    MorphingArmPublisher module;
    ASSERT_TRUE(MorphingArmPublisherTestPeer::configure(module));
    ASSERT_TRUE(MorphingArmPublisherTestPeer::measured(module));
    MorphingArmPublisherTestPeer::resetTime(module);
    uORB::Subscription sub{ORB_ID(morphing_arm_state)};
    uint64_t now = 1000;
    auto input = command(now, 0.f, 0.f, 0.f, 0.f);
    MorphingArmPublisherTestPeer::process(module, input, now);
    MorphingArmPublisherTestPeer::advance(module, now);
    morphing_arm_state_s state{};
    ASSERT_TRUE(sub.update(&state));
    float low = 0.f, high = 0.f;
    for (unsigned motor = 0; motor < 4; ++motor) {
        char name[17]; snprintf(name, sizeof(name), "CA_ROTOR%u_CT", motor); setFloat(name, 6.5f);
    }
    ActuatorEffectivenessMorphingQuad provider(nullptr);
    uORB::Subscription geometry_sub{ORB_ID(morphing_geometry)};
    uORB::Subscription matrix_sub{ORB_ID(morphing_allocation_matrix)};
    int previous_pwm = 1500;
    // Actual physical RR PWM, not pre-translated table coordinates. Both
    // branch reversals cross the old erroneous 1200-us command gate.
    for (int target : {1199, 1180, 1740, 1180, 1500}) {
        for (unsigned tick = 1; tick <= 700; ++tick) {
            now += 10000;
            if (tick % 10 == 0) {
                const float fraction = fminf(tick / 500.f, 1.f);
                const float pwm = lroundf(previous_pwm + (target - previous_pwm) * fraction);
                input = command(now, 0.f, 0.f, 0.f, 0.f);
                input.param3 = (pwm - 1450.f) / 300.f;
                MorphingArmPublisherTestPeer::process(module, input, now);
            }
            MorphingArmPublisherTestPeer::advance(module, now);
            ASSERT_FALSE(MorphingArmPublisherTestPeer::fault(module)) << "RR PWM " << target << " tick " << tick;
            ASSERT_TRUE(sub.update(&state));
            EXPECT_EQ(state.timestamp, now);
            EXPECT_EQ(MorphingArmPublisherTestPeer::mask(module), 15);
            EXPECT_FALSE(MorphingArmPublisherTestPeer::status(module).stale);
            for (unsigned other : {0u, 2u, 3u}) { EXPECT_NEAR(state.angle[other], 0.f, 1e-6f); }
            low = fminf(low, state.angle[1]); high = fmaxf(high, state.angle[1]);
            ActuatorEffectiveness::Configuration configuration{};
            ASSERT_TRUE(provider.getEffectivenessMatrix(configuration, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
            morphing_geometry_s geometry{};
            ASSERT_TRUE(geometry_sub.update(&geometry));
            EXPECT_EQ(geometry.timestamp_input, now);
            EXPECT_EQ(geometry.timestamp_angle, now);
            EXPECT_EQ(geometry.input_result, static_cast<uint8_t>(morphing_geometry_s::INPUT_ACCEPTED));
            EXPECT_NEAR(geometry.angle[1], state.angle[1], 1e-7f);
            provider.effectivenessMatrixApplied(0, configuration.effectiveness_matrices[0], 0);
            morphing_allocation_matrix_s applied{};
            ASSERT_TRUE(matrix_sub.update(&applied));
            EXPECT_EQ(applied.timestamp_angle, now);
            EXPECT_GT(applied.matrix_generation, 0u);
        }
        previous_pwm = target;
    }
    EXPECT_LT(low, -.58f); EXPECT_GT(high, .4f);
    EXPECT_EQ(MorphingArmPublisherTestPeer::status(module).rejected_commands, 0u);
    // A truly out-of-table request must still latch the fault.
    input = command(now + 1000, 0.f, 0.f, 0.f, 0.f);
    input.param3 = (1149.f - 1450.f) / 300.f;
    MorphingArmPublisherTestPeer::process(module, input, now + 1000);
    EXPECT_TRUE(MorphingArmPublisherTestPeer::fault(module));
    EXPECT_EQ(MorphingArmPublisherTestPeer::mask(module), 0);
}

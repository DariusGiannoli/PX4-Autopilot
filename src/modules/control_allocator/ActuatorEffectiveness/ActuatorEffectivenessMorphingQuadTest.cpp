// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "ActuatorEffectivenessMorphingQuad.hpp"
#include <ControlAllocationSequentialDesaturation.hpp>
#include <uORB/Publication.hpp>
#include <limits>

namespace
{
class TestEffectiveness : public ActuatorEffectivenessMorphingQuad
{
public:
	TestEffectiveness() : ActuatorEffectivenessMorphingQuad(nullptr) {}
	using ActuatorEffectivenessMorphingQuad::updateParams;
};

class TestAllocator : public ControlAllocationSequentialDesaturation
{
public:
	const matrix::Vector<float, NUM_AXES> &scale() const { return _control_allocation_scale; }
};

class MorphingAllocationTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		// Functional-test runner does not start the autosave work queue.
		param_control_autosave(false);
		int32_t count = 4;
		ASSERT_EQ(param_set(param_find("CA_ROTOR_COUNT"), &count), 0);
		int32_t airmode = 2;
		ASSERT_EQ(param_set(param_find("MC_AIRMODE"), &airmode), 0);

		for (int motor = 0; motor < 4; ++motor) {
			char name[17];
			float ct = 1.f;
			float km = motor < 2 ? 0.006f : -0.006f;
			snprintf(name, sizeof(name), "CA_ROTOR%d_CT", motor);
			ASSERT_EQ(param_set(param_find(name), &ct), 0);
			snprintf(name, sizeof(name), "CA_ROTOR%d_KM", motor);
			ASSERT_EQ(param_set(param_find(name), &km), 0);
		}

		float zero = 0.f;
		ASSERT_EQ(param_set(param_find("CA_MORPH_COGX"), &zero), 0);
		ASSERT_EQ(param_set(param_find("CA_MORPH_COGY"), &zero), 0);
		ASSERT_EQ(param_set(param_find("CA_MORPH_COGZ"), &zero), 0);
		publish({}); // replace any state retained from a preceding test
	}

	void publish(const morphing_quad::RotorValues &angles, uint64_t stamp = 0)
	{
		morphing_arm_state_s state{};
		state.timestamp = stamp ? stamp : ++_timestamp;
		for (size_t i = 0; i < 4; ++i) { state.angle[i] = angles[i]; }
		ASSERT_TRUE(_publication.publish(state));
	}

	static uint64_t _timestamp;
	uORB::Publication<morphing_arm_state_s> _publication{ORB_ID(morphing_arm_state)};
};
uint64_t MorphingAllocationTest::_timestamp = 0;

TEST_F(MorphingAllocationTest, NeutralConfigurationAndDefaultPolicies)
{
	TestEffectiveness provider;
	ActuatorEffectiveness::Configuration config{};
	ASSERT_TRUE(provider.getEffectivenessMatrix(config, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
	EXPECT_EQ(config.num_actuators[0], 4);
	EXPECT_EQ(config.num_actuators[1], 0);
	EXPECT_EQ(config.num_actuators_matrix[0], 4);
	const auto &b = config.effectiveness_matrices[0];
	EXPECT_NEAR(b(0, 0), -0.06783734f, 1e-7f);
	EXPECT_NEAR(b(1, 0), 0.06792713f, 1e-7f);
	EXPECT_NEAR(b(2, 0), 0.006f, 1e-7f);
	EXPECT_GT(fabsf(b(2, 0)), provider.minimumControlAuthority());
	EXPECT_FLOAT_EQ(b(5, 0), -1.f);
	EXPECT_TRUE(b.isAllFinite());
	EXPECT_FLOAT_EQ((b.slice<6, 12>(0, 4).norm()), 0.f);
	EXPECT_EQ(provider.effectivenessUpdateIntervalUs(), 10000u);
	ActuatorEffectivenessMultirotor standard(nullptr);
	EXPECT_EQ(standard.effectivenessUpdateIntervalUs(), 100000u);
	EXPECT_FLOAT_EQ(standard.minimumControlAuthority(), 0.05f);
	ActuatorEffectiveness::EffectivenessMatrix reference;
	EXPECT_FALSE(standard.getNormalizationReference(0, reference));
	ASSERT_TRUE(provider.getNormalizationReference(0, reference));
	EXPECT_TRUE(reference == b);
}

TEST_F(MorphingAllocationTest, AcceptedStatesAndMotorOrdering)
{
	TestEffectiveness provider;
	ActuatorEffectiveness::Configuration neutral{};
	ASSERT_TRUE(provider.getEffectivenessMatrix(neutral, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
	ActuatorEffectiveness::Configuration no_change{};
	EXPECT_FALSE(provider.getEffectivenessMatrix(no_change, EffectivenessUpdateReason::NO_EXTERNAL_UPDATE));
	// Input RR is index 1, but its motor column is 3.
	publish({{0.f, 0.3f, 0.f, 0.f}});
	ActuatorEffectiveness::Configuration moved{};
	ASSERT_TRUE(provider.getEffectivenessMatrix(moved, EffectivenessUpdateReason::NO_EXTERNAL_UPDATE));
	for (int motor = 0; motor < 3; ++motor) {
		for (int row = 0; row < 6; ++row) {
			EXPECT_FLOAT_EQ(moved.effectiveness_matrices[0](row, motor), neutral.effectiveness_matrices[0](row, motor));
		}
	}
	EXPECT_GT(fabsf(moved.effectiveness_matrices[0](0, 3) - neutral.effectiveness_matrices[0](0, 3)), 0.001f);

	for (float bad : {0.6f, std::numeric_limits<float>::quiet_NaN()}) {
		publish({{bad, 0.f, 0.f, 0.f}});
		ActuatorEffectiveness::Configuration invalid{};
		EXPECT_FALSE(provider.getEffectivenessMatrix(invalid, EffectivenessUpdateReason::NO_EXTERNAL_UPDATE));
	}
	// A replay cannot return the arms to neutral.
	publish({}, 1);
	ActuatorEffectiveness::Configuration replay{};
	EXPECT_FALSE(provider.getEffectivenessMatrix(replay, EffectivenessUpdateReason::NO_EXTERNAL_UPDATE));
	ActuatorEffectiveness::Configuration forced{};
	ASSERT_TRUE(provider.getEffectivenessMatrix(forced, EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE));
	EXPECT_TRUE(forced.effectiveness_matrices[0] == moved.effectiveness_matrices[0]);
	ActuatorEffectiveness::EffectivenessMatrix reference;
	ASSERT_TRUE(provider.getNormalizationReference(0, reference));
	EXPECT_TRUE(reference == neutral.effectiveness_matrices[0]);
}

TEST_F(MorphingAllocationTest, CoGAndInvalidRotorCount)
{
	TestEffectiveness provider;
	float cog_x = 0.01f;
	ASSERT_EQ(param_set(param_find("CA_MORPH_COGX"), &cog_x), 0);
	provider.updateParams();
	ActuatorEffectiveness::Configuration config{};
	ASSERT_TRUE(provider.getEffectivenessMatrix(config, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
	EXPECT_NEAR(config.effectiveness_matrices[0](1, 0), 0.05792713f, 1e-7f);
	int32_t count = 6;
	ASSERT_EQ(param_set(param_find("CA_ROTOR_COUNT"), &count), 0);
	provider.updateParams();
	ActuatorEffectiveness::Configuration invalid{};
	EXPECT_FALSE(provider.getEffectivenessMatrix(invalid, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
	EXPECT_EQ(invalid.totalNumActuators(), 0);
}

TEST_F(MorphingAllocationTest, MovingGeometryUsesNeutralNormalization)
{
	TestEffectiveness provider;
	ActuatorEffectiveness::Configuration neutral{};
	ASSERT_TRUE(provider.getEffectivenessMatrix(neutral, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
	TestAllocator allocator;
	allocator.updateParameters();
	allocator.setNormalizeRPY(true);
	ControlAllocation::ActuatorVector trim;
	allocator.setEffectivenessMatrix(neutral.effectiveness_matrices[0], trim, trim, 4, true);
	ActuatorEffectiveness::EffectivenessMatrix reference;
	ASSERT_TRUE(provider.getNormalizationReference(0, reference));
	allocator.setNormalizationReference(reference);
	matrix::Vector<float, 6> command;
	command(0) = 0.03f;
	command(1) = -0.02f;
	command(2) = 0.01f;
	command(5) = -0.5f;
	allocator.setControlSetpoint(command);
	allocator.allocate();
	const auto original_scale = allocator.scale();
	const auto neutral_outputs = allocator.getActuatorSetpoint();

	publish({{0.3f, -0.2f, 0.1f, -0.4f}});
	ActuatorEffectiveness::Configuration moved{};
	ASSERT_TRUE(provider.getEffectivenessMatrix(moved, EffectivenessUpdateReason::NO_EXTERNAL_UPDATE));
	allocator.setEffectivenessMatrix(moved.effectiveness_matrices[0], trim, trim, 4, false);
	allocator.allocate();
	EXPECT_TRUE(allocator.scale() == original_scale);
	const ControlAllocation::ActuatorVector output_change(allocator.getActuatorSetpoint() - neutral_outputs);
	EXPECT_GT(output_change.norm(), 0.001f);
	for (int axis = 0; axis < 6; ++axis) {
		EXPECT_NEAR(allocator.getAllocatedControl()(axis), command(axis), 1e-4f);
	}

	// Reconfiguration while folded must still normalize against neutral, not folded geometry.
	allocator.setEffectivenessMatrix(moved.effectiveness_matrices[0], trim, trim, 4, true);
	allocator.setNormalizationReference(reference);
	allocator.allocate();
	EXPECT_TRUE(allocator.scale() == original_scale);
	for (int axis = 0; axis < 6; ++axis) {
		EXPECT_NEAR(allocator.getAllocatedControl()(axis), command(axis), 1e-4f);
	}

	command(0) = 2.f;
	command(2) = 1.f;
	allocator.setControlSetpoint(command);
	allocator.allocate();
	allocator.clipActuatorSetpoint();
	for (int motor = 0; motor < 4; ++motor) {
		EXPECT_GE(allocator.getActuatorSetpoint()(motor), 0.f);
		EXPECT_LE(allocator.getActuatorSetpoint()(motor), 1.f);
	}
	// Normalization stays fixed when the existing failure handling removes a motor.
	allocator.setHadActuatorFailure(true);
	moved.effectiveness_matrices[0].col(0) = 0.f;
	allocator.setEffectivenessMatrix(moved.effectiveness_matrices[0], trim, trim, 4, false);
	allocator.setNormalizationReference(reference);
	allocator.allocate();
	EXPECT_TRUE(allocator.scale() == original_scale);
	EXPECT_TRUE(allocator.getActuatorSetpoint().isAllFinite());
}
} // namespace

TEST_F(MorphingAllocationTest, DiagnosticsRecordGeometryAndAssignedMatrix)
{
	TestEffectiveness provider;
	uORB::Subscription geometry_sub{ORB_ID(morphing_geometry)};
	uORB::Subscription applied_sub{ORB_ID(morphing_allocation_matrix)};
	publish({{0.f, 0.3f, 0.f, 0.f}});
	ActuatorEffectiveness::Configuration config{};
	ASSERT_TRUE(provider.getEffectivenessMatrix(config, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
	morphing_geometry_s log{};
	ASSERT_TRUE(geometry_sub.update(&log));
	EXPECT_TRUE(log.geometry_valid);
	EXPECT_TRUE(log.matrix_updated);
	EXPECT_EQ(log.matrix_generation, 1u);
	EXPECT_EQ(log.timestamp_angle, _timestamp);
	EXPECT_EQ(log.timestamp_input, _timestamp);
	EXPECT_EQ(log.input_result, static_cast<int>(morphing_geometry_s::INPUT_ACCEPTED));
	EXPECT_FLOAT_EQ(log.angle[1], 0.3f);
	EXPECT_FLOAT_EQ(log.input_angle[1], 0.3f);

	for (int motor = 0; motor < 4; ++motor) {
		const float x = log.rotor_position[3 * motor] - log.cog[0];
		const float y = log.rotor_position[3 * motor + 1] - log.cog[1];
		EXPECT_FLOAT_EQ(log.effectiveness[motor], -log.ct[motor] * y);
		EXPECT_FLOAT_EQ(log.effectiveness[4 + motor], log.ct[motor] * x);
		EXPECT_FLOAT_EQ(log.effectiveness[8 + motor], log.ct[motor] * log.km[motor]);
		EXPECT_FLOAT_EQ(log.effectiveness[20 + motor], -log.ct[motor]);
		for (int row = 0; row < 6; ++row) {
			EXPECT_FLOAT_EQ(log.effectiveness[4 * row + motor], config.effectiveness_matrices[0](row, motor));
		}
	}

	// The assignment callback must log the supplied matrix, not recompute raw geometry.
	auto assigned = config.effectiveness_matrices[0];
	assigned.col(3) = 0.f;
	assigned.row(2) = 0.f;
	provider.effectivenessMatrixApplied(0, assigned, 8);
	morphing_allocation_matrix_s applied{};
	ASSERT_TRUE(applied_sub.update(&applied));
	EXPECT_EQ(applied.matrix_generation, log.matrix_generation);
	EXPECT_EQ(applied.timestamp_angle, log.timestamp_angle);
	EXPECT_EQ(applied.handled_motor_failure_mask, 8);
	for (int row = 0; row < 6; ++row) {
		for (int motor = 0; motor < 4; ++motor) {
			EXPECT_FLOAT_EQ(applied.effectiveness[4 * row + motor], assigned(row, motor));
		}
	}
	provider.effectivenessMatrixApplied(1, assigned, 0);
	EXPECT_FALSE(applied_sub.update(&applied));
}

TEST_F(MorphingAllocationTest, DiagnosticsDistinguishRejectedAndUnchangedInputs)
{
	TestEffectiveness provider;
	uORB::Subscription subscription{ORB_ID(morphing_geometry)};
	ActuatorEffectiveness::Configuration config{};
	ASSERT_TRUE(provider.getEffectivenessMatrix(config, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
	morphing_geometry_s initial{};
	ASSERT_TRUE(subscription.update(&initial));
	publish({{0.7f, 0.f, 0.f, 0.f}});
	EXPECT_FALSE(provider.getEffectivenessMatrix(config, EffectivenessUpdateReason::NO_EXTERNAL_UPDATE));
	morphing_geometry_s rejected{};
	ASSERT_TRUE(subscription.update(&rejected));
	EXPECT_EQ(rejected.input_result, static_cast<int>(morphing_geometry_s::INPUT_INVALID));
	EXPECT_EQ(rejected.rejected_inputs, 1u);
	EXPECT_FALSE(rejected.matrix_updated);
	EXPECT_TRUE(rejected.geometry_valid); // Held geometry is still valid.
	EXPECT_EQ(rejected.timestamp_angle, initial.timestamp_angle);
	EXPECT_FLOAT_EQ(rejected.input_angle[0], 0.7f);
	EXPECT_FLOAT_EQ(rejected.angle[0], 0.f);
	for (int i = 0; i < 24; ++i) { EXPECT_FLOAT_EQ(rejected.effectiveness[i], initial.effectiveness[i]); }

	publish({}, initial.timestamp_angle);
	EXPECT_FALSE(provider.getEffectivenessMatrix(config, EffectivenessUpdateReason::NO_EXTERNAL_UPDATE));
	ASSERT_TRUE(subscription.update(&rejected));
	EXPECT_EQ(rejected.input_result, static_cast<int>(morphing_geometry_s::INPUT_OLD));
	EXPECT_EQ(rejected.rejected_inputs, 2u);

	publish({});
	EXPECT_FALSE(provider.getEffectivenessMatrix(config, EffectivenessUpdateReason::NO_EXTERNAL_UPDATE));
	ASSERT_TRUE(subscription.update(&rejected));
	EXPECT_EQ(rejected.input_result, static_cast<int>(morphing_geometry_s::INPUT_ACCEPTED));
	EXPECT_EQ(rejected.timestamp_angle, _timestamp);
	EXPECT_EQ(rejected.matrix_generation, initial.matrix_generation);
	EXPECT_FALSE(rejected.matrix_updated);
}

TEST_F(MorphingAllocationTest, InvalidConfigurationLogsUnusableGeometry)
{
	TestEffectiveness provider;
	int32_t count = 3;
	ASSERT_EQ(param_set(param_find("CA_ROTOR_COUNT"), &count), 0);
	provider.updateParams();
	uORB::Subscription subscription{ORB_ID(morphing_geometry)};
	ActuatorEffectiveness::Configuration config{};
	EXPECT_FALSE(provider.getEffectivenessMatrix(config, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
	morphing_geometry_s log{};
	ASSERT_TRUE(subscription.update(&log));
	EXPECT_FALSE(log.geometry_valid);
	EXPECT_FALSE(log.matrix_updated);
	for (float value : log.effectiveness) { EXPECT_TRUE(std::isnan(value)); }
	for (float value : log.rotor_position) { EXPECT_TRUE(std::isnan(value)); }
}

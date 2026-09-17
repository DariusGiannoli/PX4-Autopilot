// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "ActuatorEffectivenessMorphingQuad.hpp"
#include <ControlAllocationSequentialDesaturation.hpp>
#include <uORB/Publication.hpp>
#include <limits>
#include <random>

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
	EXPECT_TRUE(provider.useRowScaledInverse());
	EXPECT_TRUE(provider.useDirectionPreservingSaturation());
	ActuatorEffectivenessMultirotor standard(nullptr);
	EXPECT_EQ(standard.effectivenessUpdateIntervalUs(), 100000u);
	EXPECT_FLOAT_EQ(standard.minimumControlAuthority(), 0.05f);
	EXPECT_FALSE(standard.useRowScaledInverse());
	EXPECT_FALSE(standard.useDirectionPreservingSaturation());
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
	allocator.setUseRowScaledInverse(provider.useRowScaledInverse());
	allocator.setUseDirectionPreservingSaturation(provider.useDirectionPreservingSaturation());
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
TEST_F(MorphingAllocationTest, RecoversFeasibleMotorCommandsAcrossFoldingRange)
{
	std::mt19937 rng(20260916);
	std::uniform_real_distribution<float> angle(-morphing_quad::MAX_ANGLE_RAD, morphing_quad::MAX_ANGLE_RAD);
	std::uniform_real_distribution<float> motor_value(0.2f, 0.7f);

	for (int profile = 0; profile < 3; ++profile) {
		const float thrust = profile == 1 ? 6.5f : 1.f;
		const float moment = profile == 1 ? 0.05f : 0.006f;
		morphing_quad::RotorValues ct{{thrust, thrust, thrust, thrust}};
		morphing_quad::RotorValues km{{moment, -moment, moment, -moment}};
		matrix::Vector3f cog;

		if (profile == 2) {
			ct = {{0.8f, 1.1f, 1.2f, 0.9f}};
			cog = matrix::Vector3f(0.009f, -0.007f, 0.f);
		}

		auto make_matrix = [&](const morphing_quad::RotorValues &q) {
			morphing_quad::GeometryResult geometry{};
			EXPECT_TRUE(morphing_quad::compute(q, ct, km, {{cog(0), cog(1), cog(2)}}, geometry));
			ActuatorEffectiveness::EffectivenessMatrix b;

			for (int row = 0; row < 6; ++row) {
				for (int motor = 0; motor < 4; ++motor) { b(row, motor) = geometry.effectiveness[row][motor]; }
			}

			return b;
		};

		const auto reference = make_matrix({});
		TestAllocator allocator;
		allocator.updateParameters();
		allocator.setNormalizeRPY(true);
		allocator.setUseRowScaledInverse(true);
		allocator.setUseDirectionPreservingSaturation(true);
		ControlAllocation::ActuatorVector trim;
		allocator.setEffectivenessMatrix(reference, trim, trim, 4, true);
		allocator.setNormalizationReference(reference);
		const auto neutral_scale = allocator.scale();
		float max_error = 0.f;

		// All 81 combinations of -30/0/+30 degrees, then 5,000 seeded random geometries.
		for (int sample = 0; sample < 5081; ++sample) {
			morphing_quad::RotorValues q{};

			for (int arm = 0, index = sample; arm < 4; ++arm, index /= 3) {
				q[arm] = sample < 81 ? (index % 3 - 1) * morphing_quad::MAX_ANGLE_RAD : angle(rng);
			}

			const auto b = make_matrix(q);
			ControlAllocation::ActuatorVector expected;

			for (int motor = 0; motor < 4; ++motor) { expected(motor) = motor_value(rng); }

			// A known feasible motor vector is an independent oracle: B*u must recover u.
			const matrix::Vector<float, 6> command = (b * expected).emult(neutral_scale);
			allocator.setEffectivenessMatrix(b, trim, trim, 4, false);
			allocator.setControlSetpoint(command);
			allocator.allocate();
			ASSERT_TRUE(allocator.getActuatorSetpoint().isAllFinite());
			EXPECT_TRUE(allocator.scale() == neutral_scale);

			for (int motor = 0; motor < 4; ++motor) {
				max_error = fmaxf(max_error, fabsf(allocator.getActuatorSetpoint()(motor) - expected(motor)));
			}

			// Infeasible commands still use the existing desaturation and clipping.
			auto saturated = command;
			saturated(0) += 3.f;
			saturated(2) -= 2.f;
			allocator.setControlSetpoint(saturated);
			allocator.allocate();
			allocator.clipActuatorSetpoint();
			ASSERT_TRUE(allocator.getActuatorSetpoint().isAllFinite());

			for (int motor = 0; motor < 4; ++motor) {
				ASSERT_GE(allocator.getActuatorSetpoint()(motor), 0.f);
				ASSERT_LE(allocator.getActuatorSetpoint()(motor), 1.f);
			}
		}

		EXPECT_LT(max_error, 1e-4f) << "coefficient profile " << profile;
		RecordProperty("max_motor_error_ppm_profile_" + std::to_string(profile), std::to_string(max_error * 1e6f));
	}
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

// Recorded bench configuration: CT=6.5, KM in motor order [-.05,-.05,+.05,+.05],
// fixed CAD-origin CoG, CA_METHOD=Auto (sequential desaturation), MC_AIRMODE=0.
TEST_F(MorphingAllocationTest, RecordedConfigurationRollPitchDemandsAndTransitions)
{
	int32_t airmode = 0;
	ASSERT_EQ(param_set(param_find("MC_AIRMODE"), &airmode), 0);

	for (int motor = 0; motor < 4; ++motor) {
		char name[17];
		float ct = 6.5f;
		float km = motor < 2 ? -0.05f : 0.05f;
		snprintf(name, sizeof(name), "CA_ROTOR%d_CT", motor);
		ASSERT_EQ(param_set(param_find(name), &ct), 0);
		snprintf(name, sizeof(name), "CA_ROTOR%d_KM", motor);
		ASSERT_EQ(param_set(param_find(name), &km), 0);
	}

	TestEffectiveness provider;
	ActuatorEffectiveness::Configuration neutral{};
	ASSERT_TRUE(provider.getEffectivenessMatrix(neutral, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
	TestAllocator allocator;
	allocator.updateParameters();
	bool normalize[ActuatorEffectiveness::MAX_NUM_MATRICES]{};
	provider.getNormalizeRPY(normalize);
	allocator.setNormalizeRPY(normalize[0]);
	allocator.setUseRowScaledInverse(provider.useRowScaledInverse());
	allocator.setUseDirectionPreservingSaturation(provider.useDirectionPreservingSaturation());
	ControlAllocation::ActuatorVector trim;
	allocator.setEffectivenessMatrix(neutral.effectiveness_matrices[0], trim, trim, 4, true);
	ActuatorEffectiveness::EffectivenessMatrix reference;
	ASSERT_TRUE(provider.getNormalizationReference(0, reference));
	allocator.setNormalizationReference(reference);
	const auto scale = allocator.scale();
	// Independent neutral normalization: RMS roll/pitch inverse, largest yaw entry,
	// and average vertical-thrust inverse for four identical vertical rotors.
	const double x = 0.028 + 0.03992713;
	const double y = 0.028 + 0.03983734;
	const double expected_scale[] = {
		1.0 / (2.0 * 6.5 * std::sqrt(x * x + y * y)),
		1.0 / (2.0 * 6.5 * std::sqrt(x * x + y * y)),
		1.0 / (4.0 * 6.5 * 0.05), 1.0 / (4.0 * 6.5),
	};
	const int active_axes[] = {0, 1, 2, 5};

	for (int i = 0; i < 4; ++i) { ASSERT_NEAR(scale(active_axes[i]), expected_scale[i], 1e-5); }

	const float demands[][2] = {
		{0.f, 0.f}, {0.05f, 0.f}, {-0.05f, 0.f}, {0.15f, 0.f}, {-0.15f, 0.f},
		{0.f, 0.05f}, {0.f, -0.05f}, {0.f, 0.15f}, {0.f, -0.15f},
		{0.05f, 0.05f}, {0.05f, -0.05f}, {-0.05f, 0.05f}, {-0.05f, -0.05f},
	};
	double worst_tracking = 0.0;
	double worst_cross_axis = 0.0;
	float smallest_motor = 1.f;
	float largest_motor = 0.f;
	int feasible_count = 0;
	int saturated_count = 0;
	int geometry_count = 0;
	ActuatorEffectiveness::Configuration current = neutral;
	morphing_quad::RotorValues previous_angles{};

	auto exercise = [&](const morphing_quad::RotorValues &angles) {
		SCOPED_TRACE(::testing::Message() << "geometry " << geometry_count << ": "
			     << angles[0] << "," << angles[1] << "," << angles[2] << "," << angles[3]);
		publish(angles);
		ActuatorEffectiveness::Configuration config{};
		const bool updated = provider.getEffectivenessMatrix(config, EffectivenessUpdateReason::NO_EXTERNAL_UPDATE);
		ASSERT_EQ(updated, angles != previous_angles);

		if (updated) { current = config; }

		previous_angles = angles;
		const auto &b = current.effectiveness_matrices[0];
		allocator.setEffectivenessMatrix(b, trim, trim, 4, false);

		// Independently checked in double precision: these demands fit inside 0..1
		// for every tested geometry; higher collective is not always feasible.
		for (float collective : {0.25f, 0.4f, 0.5f}) {
			for (const auto &demand : demands) {
				matrix::Vector<float, 6> command;
				command(0) = demand[0];
				command(1) = demand[1];
				command(5) = -collective;
				allocator.setControlSetpoint(command);
				allocator.allocate();
				const auto output = allocator.getActuatorSetpoint();
				ASSERT_TRUE(output.isAllFinite());
				ASSERT_TRUE(allocator.scale() == scale);

				for (int motor = 0; motor < 4; ++motor) {
					// A feasible test must not quietly pass because clipping hid a bad inverse.
					ASSERT_GT(output(motor), 0.f);
					ASSERT_LT(output(motor), 1.f);
					smallest_motor = fminf(smallest_motor, output(motor));
					largest_motor = fmaxf(largest_motor, output(motor));
				}

				// Forward multiply in double precision, independently of getAllocatedControl().
				for (int axis = 0; axis < 6; ++axis) {
					double achieved = 0.0;

					for (int motor = 0; motor < 4; ++motor) {
						achieved += static_cast<double>(b(axis, motor)) * static_cast<double>(output(motor));
					}

					achieved *= static_cast<double>(scale(axis));
					const double error = std::fabs(achieved - static_cast<double>(command(axis)));
					worst_tracking = fmax(worst_tracking, error);

					if (axis < 3 && command(axis) == 0.f) { worst_cross_axis = fmax(worst_cross_axis, error); }

					EXPECT_NEAR(achieved, command(axis), 1e-5) << "axis " << axis;
				}

				++feasible_count;
			}

			// Impossible requests: validate bounds, requested torque sign, and recovery.
			for (int axis = 0; axis < 2; ++axis) {
				for (float sign : {-1.f, 1.f}) {
					matrix::Vector<float, 6> command;
					command(axis) = sign;
					command(5) = -collective;
					allocator.setControlSetpoint(command);
					allocator.allocate();
					allocator.clipActuatorSetpoint();
					ASSERT_TRUE(allocator.getActuatorSetpoint().isAllFinite());

					for (int motor = 0; motor < 4; ++motor) {
						EXPECT_GE(allocator.getActuatorSetpoint()(motor), 0.f);
						EXPECT_LE(allocator.getActuatorSetpoint()(motor), 1.f);
					}

					EXPECT_GT(allocator.getAllocatedControl()(axis) * sign, 0.f)
						<< "axis=" << axis << " demand=" << sign << " collective=" << collective
						<< " motors=" << allocator.getActuatorSetpoint()(0) << ","
						<< allocator.getActuatorSetpoint()(1) << "," << allocator.getActuatorSetpoint()(2)
						<< "," << allocator.getActuatorSetpoint()(3);
					command(axis) = 0.f;
					allocator.setControlSetpoint(command);
					allocator.allocate();

					for (int i = 0; i < 6; ++i) {
						EXPECT_NEAR(allocator.getAllocatedControl()(i), command(i), 1e-5);
					}

					++saturated_count;
				}
			}
		}

		++geometry_count;
	};

	// Includes all four requested bench poses, every individual arm, both diagonals,
	// opposite-signed folds, and every other -30/0/+30 combination.
	for (int index = 0; index < 81; ++index) {
		morphing_quad::RotorValues angles{};

		for (int arm = 0, digits = index; arm < 4; ++arm, digits /= 3) {
			angles[arm] = (digits % 3 - 1) * morphing_quad::MAX_ANGLE_RAD;
		}

		exercise(angles);
	}

	// Repeated updates in 3-degree increments: FR alone, FR/RL diagonal, all arms.
	for (int pattern = 0; pattern < 3; ++pattern) {
		for (int step = 0; step <= 20; ++step) {
			const float angle = (step <= 10 ? step : 20 - step) * (morphing_quad::MAX_ANGLE_RAD / 10.f);
			morphing_quad::RotorValues angles{};

			for (int arm = 0; arm < 4; ++arm) {
				if (arm == 0 || pattern == 2 || (pattern == 1 && arm == 2)) { angles[arm] = angle; }
			}

			exercise(angles);
		}
	}

	EXPECT_EQ(geometry_count, 144);
	EXPECT_EQ(feasible_count, 5616);
	EXPECT_EQ(saturated_count, 1728);
	RecordProperty("roll_pitch_geometry_count", geometry_count);
	RecordProperty("roll_pitch_feasible_count", feasible_count);
	RecordProperty("roll_pitch_saturated_count", saturated_count);
	RecordProperty("max_tracking_error_ppm", std::to_string(worst_tracking * 1e6));
	RecordProperty("max_cross_axis_error_ppm", std::to_string(worst_cross_axis * 1e6));
	RecordProperty("min_feasible_motor", std::to_string(smallest_motor));
	RecordProperty("max_feasible_motor", std::to_string(largest_motor));
}

// Regression for saturated pitch reversal, with both inverse policies.
// This compares both inverse policies using a fresh allocator for every demand.
TEST_F(MorphingAllocationTest, FoldedSaturatedPitchMustNotReverseDirection)
{
	int32_t airmode = 0;
	ASSERT_EQ(param_set(param_find("MC_AIRMODE"), &airmode), 0);
	auto matrix_at = [](float angle) {
		morphing_quad::GeometryResult geometry{};
		const bool valid = morphing_quad::compute({{angle, angle, angle, angle}},
			{{6.5f, 6.5f, 6.5f, 6.5f}}, {{-0.05f, 0.05f, -0.05f, 0.05f}}, {}, geometry);
		EXPECT_TRUE(valid);
		ActuatorEffectiveness::EffectivenessMatrix b;

		for (int axis = 0; axis < 6; ++axis) {
			for (int motor = 0; motor < 4; ++motor) { b(axis, motor) = geometry.effectiveness[axis][motor]; }
		}

		return b;
	};
	const auto neutral = matrix_at(0.f);

	for (bool row_scaled : {false, true}) {
		for (int degrees : {0, 15, 30}) {
			const auto b = matrix_at(degrees * (morphing_quad::MAX_ANGLE_RAD / 30.f));

			for (float pitch : {0.15f, 1.f}) {
				TestAllocator allocator;
				allocator.updateParameters();
				allocator.setNormalizeRPY(true);
				allocator.setUseRowScaledInverse(row_scaled);
				allocator.setUseDirectionPreservingSaturation(true);
				ControlAllocation::ActuatorVector zero;
				allocator.setEffectivenessMatrix(neutral, zero, zero, 4, true);
				allocator.setNormalizationReference(neutral);
				allocator.setEffectivenessMatrix(b, zero, zero, 4, false);
				matrix::Vector<float, 6> command;
				command(1) = pitch;
				command(5) = -0.25f;
				allocator.setControlSetpoint(command);
				allocator.allocate();
				allocator.clipActuatorSetpoint();
				const auto &motors = allocator.getActuatorSetpoint();
				const auto achieved = allocator.getAllocatedControl();
				printf("SATURATION_REPRO scaled=%d arms=%d pitch=%.6f achieved_roll=%.9f achieved_pitch=%.9f thrust=%.9f motors=%.9f,%.9f,%.9f,%.9f\n",
				       row_scaled, degrees, static_cast<double>(pitch), static_cast<double>(achieved(0)),
				       static_cast<double>(achieved(1)), static_cast<double>(achieved(5)),
				       static_cast<double>(motors(0)), static_cast<double>(motors(1)),
				       static_cast<double>(motors(2)), static_cast<double>(motors(3)));
				EXPECT_GT(achieved(1), 0.f) << "row_scaled=" << row_scaled << " arms=" << degrees << " pitch=" << pitch;
			}
		}
	}
}

TEST_F(MorphingAllocationTest, SaturationPreservesAxesThrustAndLimitsAcrossAirmodes)
{
	std::mt19937 rng(20260917);
	std::uniform_real_distribution<float> random_angle(-morphing_quad::MAX_ANGLE_RAD, morphing_quad::MAX_ANGLE_RAD);
	std::uniform_real_distribution<float> random_torque(-1.f, 1.f);
	float worst_rp_direction_error = 0.f, worst_zero_axis = 0.f;
	int checked = 0;

	for (int profile = 0; profile < 2; ++profile) {
		const morphing_quad::RotorValues ct = profile == 0 ? morphing_quad::RotorValues{{6.5f, 6.5f, 6.5f, 6.5f}}
				: morphing_quad::RotorValues{{6.5f, 7.1f, 5.8f, 6.2f}};
		const morphing_quad::Vector3 cog = profile == 0 ? morphing_quad::Vector3{} : morphing_quad::Vector3{{0.009f, -0.007f, 0.f}};
		auto make_matrix = [&](const morphing_quad::RotorValues &angles) {
			morphing_quad::GeometryResult geometry{};
			EXPECT_TRUE(morphing_quad::compute(angles, ct, {{-0.05f, 0.05f, -0.05f, 0.05f}}, cog, geometry));
			ActuatorEffectiveness::EffectivenessMatrix b;

			for (int axis = 0; axis < 6; ++axis) {
				for (int motor = 0; motor < 4; ++motor) { b(axis, motor) = geometry.effectiveness[axis][motor]; }
			}

			return b;
		};
		const auto neutral = make_matrix({});

		for (int32_t mode = 0; mode <= 2; ++mode) {
			ASSERT_EQ(param_set(param_find("MC_AIRMODE"), &mode), 0);
			TestAllocator allocator;
			allocator.updateParameters();
			allocator.setNormalizeRPY(true);
			allocator.setUseRowScaledInverse(true);
			allocator.setUseDirectionPreservingSaturation(true);
			ControlAllocation::ActuatorVector zero;
			allocator.setEffectivenessMatrix(neutral, zero, zero, 4, true);
			allocator.setNormalizationReference(neutral);

			for (int sample = 0; sample < 181; ++sample) {
				morphing_quad::RotorValues angles{};

				for (int arm = 0, index = sample; arm < 4; ++arm, index /= 3) {
					angles[arm] = sample < 81 ? (index % 3 - 1) * morphing_quad::MAX_ANGLE_RAD : random_angle(rng);
				}

				allocator.setEffectivenessMatrix(make_matrix(angles), zero, zero, 4, false);

				for (float thrust : {0.f, 0.01f, 0.25f, 0.5f, 0.95f, 1.f}) {
					for (int demand = 0; demand < 8; ++demand) {
						SCOPED_TRACE(::testing::Message() << "profile=" << profile << " mode=" << mode << " sample=" << sample
							     << " thrust=" << thrust << " demand=" << demand);
						matrix::Vector<float, 6> command;
						command(0) = demand == 0 || demand == 1 || demand == 4 ? 0.f : random_torque(rng);
						command(1) = demand == 0 || demand == 2 || demand == 4 ? 0.f : random_torque(rng);
						command(2) = demand < 4 ? 0.f : random_torque(rng);
						command(5) = -thrust;
						allocator.setControlSetpoint(command);
						allocator.allocate();
						allocator.clipActuatorSetpoint();
						const auto &outputs = allocator.getActuatorSetpoint();
						const auto achieved = allocator.getAllocatedControl();
						ASSERT_TRUE(outputs.isAllFinite());

						for (int motor = 0; motor < 4; ++motor) {
							ASSERT_GE(outputs(motor), 0.f);
							ASSERT_LE(outputs(motor), 1.f);
						}

						for (int axis = 0; axis < 3; ++axis) {
							EXPECT_GE(achieved(axis) * command(axis), -2e-5f);
							EXPECT_LE(fabsf(achieved(axis)), fabsf(command(axis)) + 2e-5f);

							if (command(axis) == 0.f) {
								worst_zero_axis = fmaxf(worst_zero_axis, fabsf(achieved(axis)));
								EXPECT_NEAR(achieved(axis), 0.f, 2e-5f);
							}
						}

						const float direction_error = fabsf(achieved(0) * command(1) - achieved(1) * command(0));
						worst_rp_direction_error = fmaxf(worst_rp_direction_error, direction_error);
						EXPECT_LT(direction_error, 2e-5f);
						EXPECT_LE(achieved(5), 2e-5f);

						if (mode == 0 || (mode == 1 && command(0) == 0.f && command(1) == 0.f)) {
							EXPECT_LE(-achieved(5), thrust + 2e-5f);
						}

						++checked;
					}
				}
			}
		}
	}

	EXPECT_EQ(checked, 52128);
	RecordProperty("saturation_property_cases", checked);
	RecordProperty("max_rp_direction_error_ppm", std::to_string(worst_rp_direction_error * 1e6f));
	RecordProperty("max_zero_axis_error_ppm", std::to_string(worst_zero_axis * 1e6f));
}

TEST_F(MorphingAllocationTest, UnsupportedSaturationCasesRetainOriginalResults)
{
	morphing_quad::GeometryResult geometry{};
	ASSERT_TRUE(morphing_quad::compute({{0.5f, 0.5f, 0.5f, 0.5f}}, {{6.5f, 6.5f, 6.5f, 6.5f}},
		{{-0.05f, 0.05f, -0.05f, 0.05f}}, {}, geometry));

	for (int scenario = 0; scenario < 9; ++scenario) {
		TestAllocator allocator;
		allocator.updateParameters();
		allocator.setNormalizeRPY(true);
		allocator.setUseRowScaledInverse(true);
		ActuatorEffectiveness::EffectivenessMatrix b;

		for (int axis = 0; axis < 6; ++axis) {
			for (int motor = 0; motor < 4; ++motor) { b(axis, motor) = geometry.effectiveness[axis][motor]; }
		}

		ControlAllocation::ActuatorVector trim;
		int count = 4;

		if (scenario == 0) { allocator.setHadActuatorFailure(true); b.col(0) = 0.f; }
		if (scenario == 1) { b.col(0) = 0.f; } // Rank loss without the failure flag.
		if (scenario == 2) { count = 3; }
		if (scenario == 3) { trim(0) = 0.1f; }
		if (scenario == 4) { b(3, 0) = 0.2f; }
		if (scenario == 5) { b(0, 4) = 0.2f; count = 5; }
		if (scenario == 6) { auto minimum = allocator.getActuatorMin(); minimum(0) = -1.f; allocator.setActuatorMin(minimum); }
		if (scenario == 7) { b.row(2) = 0.f; }
		if (scenario == 8) { auto maximum = allocator.getActuatorMax(); maximum(0) = 0.f; allocator.setActuatorMax(maximum); }

		allocator.setEffectivenessMatrix(b, trim, {}, count, true);
		matrix::Vector<float, 6> command;
		command(0) = 0.9f; command(1) = -0.8f; command(2) = 0.7f; command(5) = -0.25f;
		allocator.setControlSetpoint(command);
		allocator.allocate();
		allocator.clipActuatorSetpoint();
		const auto legacy = allocator.getActuatorSetpoint();
		allocator.setUseDirectionPreservingSaturation(true);
		allocator.allocate();
		allocator.clipActuatorSetpoint();
		ASSERT_TRUE(legacy.isAllFinite());
		ASSERT_TRUE(allocator.getActuatorSetpoint().isAllFinite());

		for (int motor = 0; motor < count; ++motor) {
			EXPECT_FLOAT_EQ(allocator.getActuatorSetpoint()(motor), legacy(motor)) << scenario;
		}
	}
}

TEST_F(MorphingAllocationTest, SaturationMatchesIndependentVertexEnumeration)
{
	// Independent double-precision oracle: enumerate all intersections of the
	// actuator/thrust/scale bounds, rather than production's interval elimination.
	auto optimum = [](const double base[4], const double direction[4], const double thrust[4],
			  double thrust_max, double preferred) {
		double a[12][2] {}, bound[12] {};

		for (int i = 0; i < 4; ++i) {
			a[2 * i][0] = direction[i]; a[2 * i][1] = thrust[i]; bound[2 * i] = 1.0 - base[i];
			a[2 * i + 1][0] = -direction[i]; a[2 * i + 1][1] = -thrust[i]; bound[2 * i + 1] = base[i];
		}

		a[8][0] = 1.0; bound[8] = 1.0;
		a[9][0] = -1.0;
		a[10][1] = 1.0; bound[10] = thrust_max;
		a[11][1] = -1.0;
		double best = -1.0;

		for (int i = 0; i < 12; ++i) {
			for (int j = i + 1; j < 12; ++j) {
				const double determinant = a[i][0] * a[j][1] - a[j][0] * a[i][1];

				if (std::fabs(determinant) < 1e-12) { continue; }

				const double s = (bound[i] * a[j][1] - bound[j] * a[i][1]) / determinant;
				const double t = (a[i][0] * bound[j] - a[j][0] * bound[i]) / determinant;
				bool feasible = true;

				for (int k = 0; k < 12; ++k) { feasible &= a[k][0] * s + a[k][1] * t <= bound[k] + 1e-9; }

				if (feasible) { best = fmax(best, s); }
			}
		}

		EXPECT_GE(best, -1e-9);
		best = fmin(1.0, fmax(0.0, best));
		double lo = 0.0, hi = thrust_max;

		for (int motor = 0; motor < 4; ++motor) {
			lo = fmax(lo, (-base[motor] - direction[motor] * best) / thrust[motor]);
			hi = fmin(hi, (1.0 - base[motor] - direction[motor] * best) / thrust[motor]);
		}

		return std::make_pair(best, fmin(fmax(preferred, lo), hi));
	};
	auto make_matrix = [](const morphing_quad::RotorValues &angles) {
		morphing_quad::GeometryResult geometry{};
		EXPECT_TRUE(morphing_quad::compute(angles, {{6.5f, 6.5f, 6.5f, 6.5f}},
			{{-0.05f, 0.05f, -0.05f, 0.05f}}, {}, geometry));
		ActuatorEffectiveness::EffectivenessMatrix b;

		for (int axis = 0; axis < 6; ++axis) {
			for (int motor = 0; motor < 4; ++motor) { b(axis, motor) = geometry.effectiveness[axis][motor]; }
		}

		return b;
	};
	std::mt19937 rng(20260918);
	std::uniform_real_distribution<float> torque(-1.f, 1.f);
	const auto neutral = make_matrix({});
	double max_motor_error = 0.0;
	int checked = 0;

	for (int32_t mode = 0; mode <= 2; ++mode) {
		ASSERT_EQ(param_set(param_find("MC_AIRMODE"), &mode), 0);
		TestAllocator allocator;
		allocator.updateParameters();
		allocator.setNormalizeRPY(true);
		allocator.setUseRowScaledInverse(true);
		allocator.setUseDirectionPreservingSaturation(true);
		ControlAllocation::ActuatorVector zero;
		allocator.setEffectivenessMatrix(neutral, zero, zero, 4, true);
		allocator.setNormalizationReference(neutral);

		for (int sample = 0; sample < 81; ++sample) {
			morphing_quad::RotorValues angles{};

			for (int arm = 0, index = sample; arm < 4; ++arm, index /= 3) {
				angles[arm] = (index % 3 - 1) * morphing_quad::MAX_ANGLE_RAD;
			}

			const auto b = make_matrix(angles);
			allocator.setEffectivenessMatrix(b, zero, zero, 4, false);
			matrix::SquareMatrix<double, 4> normalized, inverse;
			const int axes[] = {0, 1, 2, 5};

			for (int axis = 0; axis < 4; ++axis) {
				for (int motor = 0; motor < 4; ++motor) {
					normalized(axis, motor) = static_cast<double>(b(axes[axis], motor)) * static_cast<double>(allocator.scale()(axes[axis]));
				}
			}

			ASSERT_TRUE(matrix::inv(normalized, inverse));

			for (float collective : {0.f, 0.01f, 0.25f, 0.5f, 0.95f, 1.f}) {
				for (int demand = 0; demand < 6; ++demand) {
					matrix::Vector<float, 6> command;
					command(0) = demand == 0 || demand == 1 ? 0.f : torque(rng);
					command(1) = demand == 0 || demand == 2 ? 0.f : torque(rng);
					command(2) = demand < 3 ? 0.f : torque(rng);
					command(5) = -collective;
					double rp[4], yaw[4], thrust[4], base[4] {}, expected[4];
					bool feasible = true;

					for (int motor = 0; motor < 4; ++motor) {
						rp[motor] = inverse(motor, 0) * static_cast<double>(command(0)) + inverse(motor, 1) * static_cast<double>(command(1));
						yaw[motor] = inverse(motor, 2) * static_cast<double>(command(2));
						thrust[motor] = -inverse(motor, 3);
						expected[motor] = rp[motor] + yaw[motor] + thrust[motor] * static_cast<double>(collective);
						feasible &= expected[motor] >= 0.0 && expected[motor] <= 1.0;
					}

					if (!feasible) {
						const double maximum = mode == 0 ? static_cast<double>(collective) : 1.0;
						const auto rp_result = optimum(base, rp, thrust, maximum, static_cast<double>(collective));

						for (int motor = 0; motor < 4; ++motor) { base[motor] = rp[motor] * rp_result.first; }

						const auto yaw_result = optimum(base, yaw, thrust, mode == 2 ? maximum : rp_result.second, rp_result.second);

						for (int motor = 0; motor < 4; ++motor) {
							expected[motor] = base[motor] + yaw[motor] * yaw_result.first + thrust[motor] * yaw_result.second;
						}
					}

					allocator.setControlSetpoint(command);
					allocator.allocate();
					allocator.clipActuatorSetpoint();

					for (int motor = 0; motor < 4; ++motor) {
						const double error = std::fabs(static_cast<double>(allocator.getActuatorSetpoint()(motor)) - expected[motor]);
						max_motor_error = fmax(max_motor_error, error);
						EXPECT_LT(error, 4e-5) << "mode=" << mode << " geometry=" << sample << " demand=" << demand;
					}

					++checked;
				}
			}
		}
	}

	EXPECT_EQ(checked, 8748);
	RecordProperty("independent_vertex_cases", checked);
	RecordProperty("max_vertex_motor_error_ppm", std::to_string(max_motor_error * 1e6));
}

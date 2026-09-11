// SPDX-License-Identifier: BSD-3-Clause
#include "ActuatorEffectivenessMorphingQuad.hpp"
#include <drivers/drv_hrt.h>

bool ActuatorEffectivenessMorphingQuad::computeMatrix(const morphing_quad::RotorValues &angles,
		EffectivenessMatrix &matrix, morphing_quad::GeometryResult *geometry_result) const
{
	const auto &rotors = _mc_rotors.geometry();

	if (rotors.num_rotors != 4) {
		return false;
	}

	morphing_quad::RotorValues ct{}, km{};

	for (size_t motor = 0; motor < 4; ++motor) {
		const size_t arm = morphing_quad::MOTOR_TO_ARM[motor];
		// Reuse CA_ROTOR0..3_CT/KM in standard PX4 motor order.
		ct[arm] = rotors.rotors[motor].thrust_coef;
		km[arm] = rotors.rotors[motor].moment_ratio;
	}

	morphing_quad::GeometryResult geometry;
	const morphing_quad::Vector3 cog{{_param_cog_x.get(), _param_cog_y.get(), _param_cog_z.get()}};

	if (!morphing_quad::compute(angles, ct, km, cog, geometry)) {
		return false;
	}

	if (geometry_result) { *geometry_result = geometry; }
	matrix.setZero();

	for (size_t row = 0; row < 6; ++row) {
		for (size_t motor = 0; motor < 4; ++motor) {
			matrix(row, motor) = geometry.effectiveness[row][motor];
		}
	}

	return true;
}

bool ActuatorEffectivenessMorphingQuad::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason reason)
{
	morphing_quad::RotorValues next_angles = _angles;
	uint64_t next_timestamp = _last_timestamp;
	bool changed = false;
	morphing_arm_state_s state;

	const bool received = _arm_state_sub.update(&state);

	if (received) {
		_diagnostics.timestamp_input = state.timestamp;
		for (size_t arm = 0; arm < 4; ++arm) { _diagnostics.input_angle[arm] = state.angle[arm]; }
		_diagnostics.input_result = morphing_geometry_s::INPUT_OLD;
	}

	if (received && state.timestamp > _last_timestamp) {
		for (size_t arm = 0; arm < 4; ++arm) {
			next_angles[arm] = state.angle[arm];
		}

		// Validate the complete message before changing stored state.
		EffectivenessMatrix candidate;

		if (computeMatrix(next_angles, candidate)) {
			_diagnostics.input_result = morphing_geometry_s::INPUT_ACCEPTED;
			changed = next_angles != _angles;
			next_timestamp = state.timestamp;

		} else {
			_diagnostics.input_result = morphing_geometry_s::INPUT_INVALID;
			next_angles = _angles;
		}
	}

	if (received && _diagnostics.input_result != morphing_geometry_s::INPUT_ACCEPTED) {
		++_diagnostics.rejected_inputs;
	}

	if (!changed && reason == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		_last_timestamp = next_timestamp;
		if (received) { publishGeometry(false); }
		return false;
	}

	EffectivenessMatrix effectiveness;

	if (!computeMatrix(next_angles, effectiveness)) {
		publishGeometry(false);
		PX4_ERR("Morphing quad requires 4 rotors and valid CT/KM/CoG parameters");
		return false;
	}

	configuration.effectiveness_matrices[configuration.selected_matrix] = effectiveness;
	configuration.actuatorsAdded(ActuatorType::MOTORS, 4);
	_angles = next_angles;
	_last_timestamp = next_timestamp;
	++_diagnostics.matrix_generation;
	publishGeometry(true);
	return true;
}

bool ActuatorEffectivenessMorphingQuad::getNormalizationReference(int matrix_index,
		EffectivenessMatrix &reference) const
{
	return matrix_index == 0 && computeMatrix(morphing_quad::RotorValues{}, reference);
}

void ActuatorEffectivenessMorphingQuad::publishGeometry(bool matrix_updated)
{
	_diagnostics.timestamp = hrt_absolute_time();
	_diagnostics.timestamp_angle = _last_timestamp;
	_diagnostics.matrix_updated = matrix_updated;
	_diagnostics.cog[0] = _param_cog_x.get();
	_diagnostics.cog[1] = _param_cog_y.get();
	_diagnostics.cog[2] = _param_cog_z.get();
	for (size_t arm = 0; arm < 4; ++arm) { _diagnostics.angle[arm] = _angles[arm]; }

	const auto &rotors = _mc_rotors.geometry();
	for (size_t motor = 0; motor < 4; ++motor) {
		_diagnostics.ct[motor] = rotors.num_rotors == 4 ? rotors.rotors[motor].thrust_coef : NAN;
		_diagnostics.km[motor] = rotors.num_rotors == 4 ? rotors.rotors[motor].moment_ratio : NAN;
	}

	EffectivenessMatrix matrix;
	morphing_quad::GeometryResult geometry;
	_diagnostics.geometry_valid = computeMatrix(_angles, matrix, &geometry);
	for (size_t motor = 0; motor < 4; ++motor) {
		for (size_t axis = 0; axis < 3; ++axis) {
			_diagnostics.rotor_position[3 * motor + axis] = _diagnostics.geometry_valid
					? geometry.rotor_position_body[motor][axis] : NAN;
		}
		for (size_t row = 0; row < 6; ++row) {
			_diagnostics.effectiveness[4 * row + motor] = _diagnostics.geometry_valid ? matrix(row, motor) : NAN;
		}
	}
	_geometry_pub.publish(_diagnostics);
}

void ActuatorEffectivenessMorphingQuad::effectivenessMatrixApplied(int matrix_index,
		const EffectivenessMatrix &matrix, uint16_t failure_mask)
{
	if (matrix_index != 0) { return; }
	morphing_allocation_matrix_s applied{};
	applied.timestamp = hrt_absolute_time();
	applied.timestamp_angle = _last_timestamp;
	applied.matrix_generation = _diagnostics.matrix_generation;
	applied.handled_motor_failure_mask = failure_mask;
	for (size_t row = 0; row < 6; ++row) {
		for (size_t motor = 0; motor < 4; ++motor) {
			applied.effectiveness[4 * row + motor] = matrix(row, motor);
		}
	}
	_matrix_pub.publish(applied);
}

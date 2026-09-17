/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ControlAllocationSequentialDesaturation.cpp
 *
 * @author Roman Bapst <bapstroman@gmail.com>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include "ControlAllocationSequentialDesaturation.hpp"

namespace
{
/** Maximize s in [0,1] with 0 <= base[i] + direction[i]*s + thrust[i]*t <= maximum[i].
 * All thrust coefficients are positive. Eliminate t by requiring every lower
 * bound on t to be below every upper bound, including 0 <= t <= thrust_max.
 * s=0 must be feasible. This is a fixed 25-inequality calculation, not an
 * iterative optimizer. Among equal s, choose t nearest the preferred thrust.
 */
bool maximizeDirectionScale(const float base[4], const float direction[4], const float thrust[4],
			    const float maximum[4], float thrust_max, float preferred_thrust, float &scale, float &collective)
{
	float lower[5] {}, upper[5] {}, slope[5] {};
	upper[4] = thrust_max;

	for (int motor = 0; motor < 4; ++motor) {
		lower[motor] = -base[motor] / thrust[motor];
		upper[motor] = (maximum[motor] - base[motor]) / thrust[motor];
		slope[motor] = -direction[motor] / thrust[motor];
	}

	scale = 1.f;

	for (int i = 0; i < 5; ++i) {
		for (int j = 0; j < 5; ++j) {
			const float available = upper[j] - lower[i];
			const float change = slope[i] - slope[j];

			if (available < -2e-5f) { return false; }

			if (change > FLT_EPSILON) {
				scale = fminf(scale, fmaxf(0.f, available) / change);
			}
		}
	}

	// Leave a few float ULPs at an active boundary so final clipping is roundoff only.
	if (scale < 1.f) { scale = fmaxf(0.f, scale - 4.f * FLT_EPSILON); }

	float low = 0.f, high = thrust_max;

	for (int motor = 0; motor < 4; ++motor) {
		low = fmaxf(low, lower[motor] + slope[motor] * scale);
		high = fminf(high, upper[motor] + slope[motor] * scale);
	}

	if (!PX4_ISFINITE(low) || !PX4_ISFINITE(high) || low > high + 2e-5f) { return false; }

	collective = fminf(fmaxf(preferred_thrust, fminf(low, high)), high);
	return PX4_ISFINITE(scale) && PX4_ISFINITE(collective);
}
} // namespace

bool ControlAllocationSequentialDesaturation::allocateDirectionPreserving()
{
	// Preserve the existing least-squares/failure behavior outside the supported
	// full-rank vertical four-motor map. In particular, do not reweight a failed motor.
	if (_num_actuators != 4 || _had_actuator_failure || !_mix.isAllFinite()
	    || !_effectiveness.isAllFinite() || !_control_sp.isAllFinite()
	    || fabsf(_control_sp(THRUST_X)) > 0.f || fabsf(_control_sp(THRUST_Y)) > 0.f) {
		return false;
	}

	for (int motor = 0; motor < NUM_ACTUATORS; ++motor) {
		for (int axis = 0; axis < NUM_AXES; ++axis) {
			if ((motor >= 4 || axis == THRUST_X || axis == THRUST_Y) && fabsf(_effectiveness(axis, motor)) > 0.f) {
				return false;
			}
		}
	}

	constexpr int axes[4] {ROLL, PITCH, YAW, THRUST_Z};

	for (int row : axes) {
		if (!PX4_ISFINITE(_control_allocation_scale(row)) || _control_allocation_scale(row) <= 0.f
		    || fabsf(_control_trim(row)) > 0.f) { return false; }

		for (int column : axes) {
			float product = 0.f;

			for (int motor = 0; motor < 4; ++motor) { product += _effectiveness(row, motor) * _mix(motor, column); }

			product *= _control_allocation_scale(row);

			if (!PX4_ISFINITE(product) || fabsf(product - (row == column ? 1.f : 0.f)) > 2e-4f) { return false; }
		}
	}

	float thrust[4], rp[4], yaw[4], maximum[4], result[4];
	float thrust_capacity = 0.f, zero_torque_capacity = INFINITY;
	bool feasible = true;

	for (int motor = 0; motor < 4; ++motor) {
		thrust[motor] = -_mix(motor, THRUST_Z);
		maximum[motor] = _actuator_max(motor);

		if (!PX4_ISFINITE(_actuator_min(motor)) || !PX4_ISFINITE(_actuator_trim(motor))
		    || fabsf(_actuator_min(motor)) > 0.f || fabsf(_actuator_trim(motor)) > 0.f
		    || !PX4_ISFINITE(maximum[motor]) || maximum[motor] <= 0.f
		    || thrust[motor] < 1e-6f || _effectiveness(THRUST_Z, motor) >= 0.f) { return false; }

		rp[motor] = _mix(motor, ROLL) * _control_sp(ROLL) + _mix(motor, PITCH) * _control_sp(PITCH);
		yaw[motor] = _mix(motor, YAW) * _control_sp(YAW);
		result[motor] = rp[motor] + yaw[motor] - thrust[motor] * _control_sp(THRUST_Z);

		if (!PX4_ISFINITE(result[motor]) || !PX4_ISFINITE(rp[motor]) || !PX4_ISFINITE(yaw[motor])) { return false; }

		feasible &= result[motor] >= 0.f && result[motor] <= maximum[motor];
		thrust_capacity -= _effectiveness(THRUST_Z, motor) * maximum[motor] * _control_allocation_scale(THRUST_Z);
		zero_torque_capacity = fminf(zero_torque_capacity, maximum[motor] / thrust[motor]);
	}

	if (!PX4_ISFINITE(thrust_capacity) || !PX4_ISFINITE(zero_torque_capacity)) { return false; }

	if (!feasible) {
		const float requested = fmaxf(0.f, -_control_sp(THRUST_Z));
		const int airmode = _param_mc_airmode.get();
		const float thrust_max = (airmode == 1 || airmode == 2) ? thrust_capacity : fminf(requested, thrust_capacity);
		float base[4] {};
		float rp_scale = 0.f, collective = fminf(thrust_max, zero_torque_capacity);

		if (!maximizeDirectionScale(base, rp, thrust, maximum, thrust_max, requested, rp_scale, collective)) {
			// Numerical rejection: retain feasible zero-torque thrust, not the legacy
			// desaturation sequence that can reverse the requested torque.
			rp_scale = 0.f;
			collective = fminf(thrust_max, zero_torque_capacity);
		}

		for (int motor = 0; motor < 4; ++motor) { base[motor] = rp[motor] * rp_scale; }

		const float yaw_thrust_max = airmode == 2 ? thrust_max : collective;
		float yaw_scale = 0.f, yaw_collective = collective;

		if (!maximizeDirectionScale(base, yaw, thrust, maximum, yaw_thrust_max, collective, yaw_scale, yaw_collective)) {
			yaw_scale = 0.f;
			yaw_collective = collective;
		}

		for (int motor = 0; motor < 4; ++motor) {
			result[motor] = base[motor] + yaw[motor] * yaw_scale + thrust[motor] * yaw_collective;
		}
	}

	_actuator_sp.setZero();

	for (int motor = 0; motor < 4; ++motor) {
		_actuator_sp(motor) = fminf(fmaxf(result[motor], 0.f), maximum[motor]);
	}

	return true;
}

void
ControlAllocationSequentialDesaturation::allocate()
{
	//Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	if (_direction_preserving_saturation && allocateDirectionPreserving()) { return; }

	switch (_param_mc_airmode.get()) {
	case 1:
		mixAirmodeRP();
		break;

	case 2:
		mixAirmodeRPY();
		break;

	default:
		mixAirmodeDisabled();
		break;
	}
}

void ControlAllocationSequentialDesaturation::desaturateActuators(
	ActuatorVector &actuator_sp,
	const ActuatorVector &desaturation_vector, bool increase_only)
{
	float gain = computeDesaturationGain(desaturation_vector, actuator_sp);

	if (increase_only && gain < 0.f) {
		return;
	}

	for (int i = 0; i < _num_actuators; i++) {
		actuator_sp(i) += gain * desaturation_vector(i);
	}

	gain = 0.5f * computeDesaturationGain(desaturation_vector, actuator_sp);

	for (int i = 0; i < _num_actuators; i++) {
		actuator_sp(i) += gain * desaturation_vector(i);
	}
}

float ControlAllocationSequentialDesaturation::computeDesaturationGain(const ActuatorVector &desaturation_vector,
		const ActuatorVector &actuator_sp)
{
	float k_min = 0.f;
	float k_max = 0.f;

	for (int i = 0; i < _num_actuators; i++) {
		// Do not use try to desaturate using an actuator with weak effectiveness to avoid large desaturation gains
		if (fabsf(desaturation_vector(i)) < 0.2f) {
			continue;
		}

		if (actuator_sp(i) < _actuator_min(i)) {
			float k = (_actuator_min(i) - actuator_sp(i)) / desaturation_vector(i);

			if (k < k_min) { k_min = k; }

			if (k > k_max) { k_max = k; }
		}

		if (actuator_sp(i) > _actuator_max(i)) {
			float k = (_actuator_max(i) - actuator_sp(i)) / desaturation_vector(i);

			if (k < k_min) { k_min = k; }

			if (k > k_max) { k_max = k; }
		}
	}

	// Reduce the saturation as much as possible
	return k_min + k_max;
}

void
ControlAllocationSequentialDesaturation::mixAirmodeRP()
{
	// Airmode for roll and pitch, but not yaw

	// Mix without yaw
	ActuatorVector thrust_z;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) = _actuator_trim(i) +
				  _mix(i, ControlAxis::ROLL) * (_control_sp(ControlAxis::ROLL) - _control_trim(ControlAxis::ROLL)) +
				  _mix(i, ControlAxis::PITCH) * (_control_sp(ControlAxis::PITCH) - _control_trim(ControlAxis::PITCH)) +
				  _mix(i, ControlAxis::THRUST_X) * (_control_sp(ControlAxis::THRUST_X) - _control_trim(ControlAxis::THRUST_X)) +
				  _mix(i, ControlAxis::THRUST_Y) * (_control_sp(ControlAxis::THRUST_Y) - _control_trim(ControlAxis::THRUST_Y)) +
				  _mix(i, ControlAxis::THRUST_Z) * (_control_sp(ControlAxis::THRUST_Z) - _control_trim(ControlAxis::THRUST_Z));
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
	}

	desaturateActuators(_actuator_sp, thrust_z);

	// Mix yaw independently
	mixYaw();
}

void
ControlAllocationSequentialDesaturation::mixAirmodeRPY()
{
	// Airmode for roll, pitch and yaw

	// Do full mixing
	ActuatorVector thrust_z;
	ActuatorVector yaw;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) = _actuator_trim(i) +
				  _mix(i, ControlAxis::ROLL) * (_control_sp(ControlAxis::ROLL) - _control_trim(ControlAxis::ROLL)) +
				  _mix(i, ControlAxis::PITCH) * (_control_sp(ControlAxis::PITCH) - _control_trim(ControlAxis::PITCH)) +
				  _mix(i, ControlAxis::YAW) * (_control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW)) +
				  _mix(i, ControlAxis::THRUST_X) * (_control_sp(ControlAxis::THRUST_X) - _control_trim(ControlAxis::THRUST_X)) +
				  _mix(i, ControlAxis::THRUST_Y) * (_control_sp(ControlAxis::THRUST_Y) - _control_trim(ControlAxis::THRUST_Y)) +
				  _mix(i, ControlAxis::THRUST_Z) * (_control_sp(ControlAxis::THRUST_Z) - _control_trim(ControlAxis::THRUST_Z));
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
		yaw(i) = _mix(i, ControlAxis::YAW);
	}

	desaturateActuators(_actuator_sp, thrust_z);

	// Unsaturate yaw (in case upper and lower bounds are exceeded)
	// to prioritize roll/pitch over yaw.
	desaturateActuators(_actuator_sp, yaw);
}

void
ControlAllocationSequentialDesaturation::mixAirmodeDisabled()
{
	// Airmode disabled: never allow to increase the thrust to unsaturate a motor

	// Mix without yaw
	ActuatorVector thrust_z;
	ActuatorVector roll;
	ActuatorVector pitch;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) = _actuator_trim(i) +
				  _mix(i, ControlAxis::ROLL) * (_control_sp(ControlAxis::ROLL) - _control_trim(ControlAxis::ROLL)) +
				  _mix(i, ControlAxis::PITCH) * (_control_sp(ControlAxis::PITCH) - _control_trim(ControlAxis::PITCH)) +
				  _mix(i, ControlAxis::THRUST_X) * (_control_sp(ControlAxis::THRUST_X) - _control_trim(ControlAxis::THRUST_X)) +
				  _mix(i, ControlAxis::THRUST_Y) * (_control_sp(ControlAxis::THRUST_Y) - _control_trim(ControlAxis::THRUST_Y)) +
				  _mix(i, ControlAxis::THRUST_Z) * (_control_sp(ControlAxis::THRUST_Z) - _control_trim(ControlAxis::THRUST_Z));
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
		roll(i) = _mix(i, ControlAxis::ROLL);
		pitch(i) = _mix(i, ControlAxis::PITCH);
	}

	// only reduce thrust
	desaturateActuators(_actuator_sp, thrust_z, true);

	// Reduce roll/pitch acceleration if needed to unsaturate
	desaturateActuators(_actuator_sp, roll);
	desaturateActuators(_actuator_sp, pitch);

	// Mix yaw independently
	mixYaw();
}

void
ControlAllocationSequentialDesaturation::mixYaw()
{
	// Add yaw to outputs
	ActuatorVector yaw;
	ActuatorVector thrust_z;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) += _mix(i, ControlAxis::YAW) * (_control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW));
		yaw(i) = _mix(i, ControlAxis::YAW);
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
	}

	// Change yaw acceleration to unsaturate the outputs if needed (do not change roll/pitch),
	// and allow some yaw response at maximum thrust
	ActuatorVector max_prev = _actuator_max;
	_actuator_max += (_actuator_max - _actuator_min) * 0.15f;
	desaturateActuators(_actuator_sp, yaw);
	_actuator_max = max_prev;

	// reduce thrust only
	desaturateActuators(_actuator_sp, thrust_z, true);
}

void
ControlAllocationSequentialDesaturation::updateParameters()
{
	updateParams();
}

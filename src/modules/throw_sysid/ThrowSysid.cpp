/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "ThrowSysid.hpp"

#include <math.h>

static const char *phase_name(uint8_t p)
{
	switch (p) {
	case throw_sysid_status_s::PHASE_IDLE: return "IDLE";

	case throw_sysid_status_s::PHASE_EXCITE: return "EXCITE";

	case throw_sysid_status_s::PHASE_DONE: return "DONE";

	default: return "?";
	}
}

static const char *end_name(uint8_t r)
{
	switch (r) {
	case throw_sysid_status_s::END_NONE: return "none";

	case throw_sysid_status_s::END_COMPLETE: return "sequence complete";

	case throw_sysid_status_s::END_DROP: return "drop limit";

	case throw_sysid_status_s::END_RATE: return "body rate limit";

	case throw_sysid_status_s::END_DISARMED: return "disarmed";

	case throw_sysid_status_s::END_CANCEL: return "cancel";

	case throw_sysid_status_s::END_TILT: return "tilt limit";

	default: return "?";
	}
}

ThrowSysid::ThrowSysid() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
}

ThrowSysid::~ThrowSysid()
{
	perf_free(_cycle_perf);
}

bool ThrowSysid::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void ThrowSysid::start(hrt_abstime now)
{
	const float amp = _param_amp.get();
	PX4_INFO("hover pulses +-%.2f on top of the current command (now %.3f %.3f %.3f %.3f)", (double)amp,
		 (double)_base[0], (double)_base[1], (double)_base[2], (double)_base[3]);

	const float base_min = fminf(fminf(_base[0], _base[1]), fminf(_base[2], _base[3]));

	if (amp > base_min) {
		PX4_WARN("THRSID_AMP %.2f > lowest motor command %.3f: the negative pulses will clip at 0", (double)amp,
			 (double)base_min);
	}

	_pulses.configure(amp, _param_slot.get(), _param_rate_lim.get());
	_end_reason = throw_sysid_status_s::END_NONE;
	_start = now;
	_z_start = _z;
	_phase = throw_sysid_status_s::PHASE_EXCITE;
	PX4_INFO("EXCITE");
}

void ThrowSysid::finish(uint8_t reason, hrt_abstime now)
{
	for (int i = 0; i < 4; i++) { _offset[i] = 0.f; }

	_end_reason = reason;
	_phase = throw_sysid_status_s::PHASE_DONE;
	publishStatus(now);
	PX4_INFO("pulses ended: %s, aborted motors 0x%x", end_name(reason), _pulses.abortedMask());
}

void ThrowSysid::publishStatus(hrt_abstime now)
{
	throw_sysid_status_s s{};
	s.phase = _phase;

	for (int i = 0; i < 4; i++) { s.offset[i] = _offset[i]; }

	s.aborted_motors = _pulses.abortedMask();
	s.end_reason = _end_reason;
	s.timestamp = now;
	_status_pub.publish(s);
	_last_status = now;
}

void ThrowSysid::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s pu;
		_parameter_update_sub.copy(&pu);
		updateParams();
	}

	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	}

	vehicle_land_detected_s land;

	if (_vehicle_land_detected_sub.update(&land)) {
		_landed = land.landed;
	}

	vehicle_local_position_s lpos;

	if (_vehicle_local_position_sub.update(&lpos)) {
		_pos_z_valid = lpos.z_valid;
		_z = lpos.z;
	}

	vehicle_attitude_s att;

	if (_vehicle_attitude_sub.update(&att)) {
		const float c = 1.f - 2.f * (att.q[1] * att.q[1] + att.q[2] * att.q[2]);  // cos(tilt)
		_tilt_deg = acosf(fminf(fmaxf(c, -1.f), 1.f)) * 57.2958f;
	}

	actuator_motors_s motors;

	if (_phase != throw_sysid_status_s::PHASE_EXCITE && _actuator_motors_sub.update(&motors)) {
		const bool finite = PX4_ISFINITE(motors.control[0]) && PX4_ISFINITE(motors.control[1])
				    && PX4_ISFINITE(motors.control[2]) && PX4_ISFINITE(motors.control[3]);

		for (int i = 0; i < 4; i++) {
			_base[i] = (finite && _base_valid) ? 0.98f * _base[i] + 0.02f * motors.control[i] : motors.control[i];
		}

		_base_valid = finite && _armed;
	}

	vehicle_angular_velocity_s av;

	if (!_vehicle_angular_velocity_sub.update(&av)) {
		perf_end(_cycle_perf);
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (_arm_request.load()) {
		_arm_request.store(false);

		if (_phase == throw_sysid_status_s::PHASE_EXCITE) {
			PX4_WARN("already running");

		} else if (!_armed || _landed || !_base_valid) {
			PX4_WARN("take off and hover first");

		} else {
			start(now);
		}
	}

	if (_phase == throw_sysid_status_s::PHASE_EXCITE) {
		const float t = (now - _start) * 1e-6f;
		const float rate_max = _param_rate_max.get();

		if (_cancel_request.load()) {
			finish(throw_sysid_status_s::END_CANCEL, now);

		} else if (!_armed) {
			finish(throw_sysid_status_s::END_DISARMED, now);

		} else if (fabsf(av.xyz[0]) > rate_max || fabsf(av.xyz[1]) > rate_max || fabsf(av.xyz[2]) > rate_max) {
			finish(throw_sysid_status_s::END_RATE, now);

		} else if (_tilt_deg > _param_tilt_max.get()) {
			finish(throw_sysid_status_s::END_TILT, now);

		} else if (_pos_z_valid && (_z - _z_start) > _param_max_drop.get()) {
			finish(throw_sysid_status_s::END_DROP, now);

		} else if (_pulses.done(t)) {
			finish(throw_sysid_status_s::END_COMPLETE, now);

		} else {
			_pulses.step(t, av.xyz, _offset);
			publishStatus(now);
		}
	}

	_cancel_request.store(false);

	if (now - _last_status > 1_s) {
		publishStatus(now);
	}

	perf_end(_cycle_perf);
}

int ThrowSysid::print_status()
{
	PX4_INFO("phase: %s, last end: %s, aborted motors: 0x%x", phase_name(_phase), end_name(_end_reason),
		 _pulses.abortedMask());
	PX4_INFO("armed %d, landed %d, tilt %.0f deg, motor commands %.3f %.3f %.3f %.3f", _armed, _landed,
		 (double)_tilt_deg, (double)_base[0], (double)_base[1], (double)_base[2], (double)_base[3]);
	PX4_INFO("pulses +-%.2f, %.2f s per motor (%.2f s), limits: rate margin %.0f / max %.0f rad/s, tilt %.0f deg, drop %.1f m",
		 (double)_param_amp.get(), (double)_param_slot.get(), (double)(4.f * _param_slot.get()),
		 (double)_param_rate_lim.get(), (double)_param_rate_max.get(), (double)_param_tilt_max.get(),
		 (double)_param_max_drop.get());
	perf_print_counter(_cycle_perf);
	return 0;
}

int ThrowSysid::task_spawn(int argc, char *argv[])
{
	ThrowSysid *instance = new ThrowSysid();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ThrowSysid::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	if (argc >= 1 && !strcmp(argv[0], "arm")) {
		get_instance()->requestArm();
		return PX4_OK;
	}

	if (argc >= 1 && !strcmp(argv[0], "cancel")) {
		get_instance()->requestCancel();
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int ThrowSysid::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Hover pulses for kf_b1. `throw_sysid arm` while hovering adds a +THRSID_AMP / -THRSID_AMP doublet to
each motor in turn (4 x THRSID_SLOT), on top of the command of whatever controller flies; control_allocator
adds the offsets. No mode change. kf_b1 fits B1 on the pulses and restarts its filter from that fit.
Rate, tilt and drop limits, `cancel` and disarming stop the pulses.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("throw_sysid", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("arm", "start the pulses (vehicle hovering)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("cancel", "stop the pulses");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int throw_sysid_main(int argc, char *argv[])
{
	return ThrowSysid::main(argc, argv);
}

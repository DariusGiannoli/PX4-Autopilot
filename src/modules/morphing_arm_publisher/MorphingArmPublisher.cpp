// SPDX-License-Identifier: BSD-3-Clause
#include "MorphingArmPublisher.hpp"
#include <drivers/drv_hrt.h>
#include <cstdio>

MorphingArmPublisher::MorphingArmPublisher() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool MorphingArmPublisher::enabled() const
{
	int32_t value = 0;
	return _enable_param != PARAM_INVALID && param_get(_enable_param, &value) == PX4_OK && value == 1;
}

bool MorphingArmPublisher::configure()
{
	_enable_param = param_find("MORPH_PUB_EN");
	int32_t airframe = 0, rotors = 0;
	param_get(param_find("CA_AIRFRAME"), &airframe);
	param_get(param_find("CA_ROTOR_COUNT"), &rotors);

	if (!enabled() || airframe != 13 || rotors != 4) {
		PX4_ERR("Requires MORPH_PUB_EN=1, CA_AIRFRAME=13, CA_ROTOR_COUNT=4");
		return false;
	}

	const char *arms[] = {"FR", "RR", "RL", "FL"};
	int32_t model = 0;
	if (param_get(param_find("MARM_MODEL"), &model) != PX4_OK || (model != 0 && model != 1)) { return false; }
	_measured = model == 1;
	if (_measured && !configureMeasured()) { return false; }
	const char *points[] = {"NEG", "ZERO", "POS", "CTR"};
	float *destinations[] = {_calibration.negative, _calibration.zero, _calibration.positive, _calibration.center};

	for (unsigned arm = 0; arm < 4; ++arm) {
		for (unsigned point = 0; point < 4; ++point) {
			char name[17];
			snprintf(name, sizeof(name), "MARM_%s_%s", arms[arm], points[point]);
			if (param_get(param_find(name), &destinations[point][arm]) != PX4_OK) { return false; }
		}
	}

	float timeout = 0.f;
	param_get(param_find("MORPH_PUB_TMO"), &timeout);
	_status.calibration_valid = _measured || _calibration.valid();

	if (!_status.calibration_valid || !PX4_ISFINITE(timeout) || timeout < 0.1f || timeout > 10.f) {
		PX4_ERR("Set measured monotonic calibration in +/-30 deg and valid timeout before starting");
		return false;
	}

	_timeout_us = static_cast<uint64_t>(timeout * 1e6f);
	_start_time = hrt_absolute_time();
	_last_seen = _start_time;
	_previous_enabled = true;
	return true;
}

int MorphingArmPublisher::readInteger(const char *name, int32_t *value)
{
	return param_get(param_find(name), value);
}

bool MorphingArmPublisher::configureMeasured(int (*read_int)(const char *, int32_t *))
{
	int32_t id = 0, reverse = 0;
	if (read_int("MARM_CAL_ID", &id) != PX4_OK || id != morphing_publisher::measured::CALIBRATION_ID
	    || read_int("PWM_MAIN_REV", &reverse) != PX4_OK || reverse != 0) { return false; }
	int32_t rate1 = 0, rate2 = 0;
	if (read_int("PWM_MAIN_TIM1", &rate1) != PX4_OK || read_int("PWM_MAIN_TIM2", &rate2) != PX4_OK
	    || rate1 != 50 || rate2 != 50) { return false; }
	const unsigned pins[] = {7, 5, 6, 8}; // canonical FR RR RL FL
	const int32_t functions[] = {301, 304, 302, 303};
	const char *arms[] = {"FR", "RR", "RL", "FL"};
	const char *fields[] = {"FUNC", "MIN", "MAX", "DIS"};
	for (unsigned i = 0; i < 4; ++i) {
		const int32_t expected[] = {functions[i], 1150, 1750, 1450};
		for (unsigned j = 0; j < 4; ++j) {
			char name[17]; snprintf(name, sizeof(name), "PWM_MAIN_%s%u", fields[j], pins[i]);
			int32_t value = 0;
			if (read_int(name, &value) != PX4_OK || value != expected[j]) {
				PX4_ERR("Measured arm calibration requires %s=%ld", name, (long)expected[j]);
				return false;
			}
		}
		char name[17]; snprintf(name, sizeof(name), "MARM_%s_DIR", arms[i]);
		if (read_int(name, &_directions[i]) != PX4_OK
		    || (_directions[i] != -1 && _directions[i] != 1)) { return false; }
	}
	return true;
}

void MorphingArmPublisher::advanceMeasured(uint64_t now)
{
	if (!_model_started || _model_fault || now <= _model_time) { return; }
	const float dt = (now - _model_time) * 1e-6f;
	if (dt > .1f) {
		_model_fault = true; _state.invalidate(); _status.stale = true;
		PX4_ERR("Arm model scheduling gap: restart from physical neutral");
		return;
	}
	for (auto &model : _models) { model.advance(dt); }
	_model_time = now;
}

void MorphingArmPublisher::processMeasured(const vehicle_command_s &command, uint64_t now)
{
	// MAVLink actuator sets FR RL FL RR -> canonical allocator FR RR RL FL.
	const float values[] = {command.param1, command.param4, command.param2, command.param3};
	bool valid = !_model_fault;
	for (float value : values) {
		valid = valid && PX4_ISFINITE(value) && fabsf(value) <= 1.f;
		if (!_model_started) { valid = valid && fabsf(value) <= 0.f; }
	}
	if (!valid) {
		++_status.rejected_commands; _state.invalidate(); _status.stale = true;
		_model_fault = true; // Servo path is independent: history can no longer be trusted.
		return;
	}
	if (!_model_started) {
		_model_started = true; _model_time = command.timestamp;
	} else {
		advanceMeasured(command.timestamp);
		if (_model_fault) { return; }
	}
	for (unsigned i = 0; i < 4; ++i) { _models[i].accept(values[i]); }
	_state.initialized_mask = 15;
	_status.timestamp_command = command.timestamp;
	_status.stale = false;
	advanceMeasured(now);
}

void MorphingArmPublisher::publishMeasured(uint64_t now)
{
	if (!_model_started || _model_fault || _status.stale || _state.initialized_mask != 15) { return; }
	morphing_arm_state_s state{};
	state.timestamp = now;
	for (unsigned i = 0; i < 4; ++i) {
		state.angle[i] = _models[i].angle * _directions[i] * 0.017453292519943295f;
		// Existing allocator supports +/-30 degrees; do not silently clip geometry.
		if (!PX4_ISFINITE(state.angle[i]) || fabsf(state.angle[i]) > 0.523598776f) {
			_model_fault = true; _state.invalidate(); _status.stale = true;
			PX4_ERR("Predicted arm angle outside allocator range; restart from neutral");
			return;
		}
	}
	if (_angles.publish(state)) {
		_status.timestamp_angle = now;
		++_status.published_messages;
	}
}

bool MorphingArmPublisher::init()
{
	if (!configure()) { return false; }
	ScheduleOnInterval(10000); // Drain the command queue at 100 Hz, outside rate_ctrl.
	return true;
}

void MorphingArmPublisher::checkTimeout(uint64_t now)
{
	const uint64_t reference = _status.timestamp_command ? _status.timestamp_command : _start_time;

	if (now > reference && now - reference > _timeout_us) {
		if (!_status.stale) {
			PX4_WARN("Arm commands stale; allocator holds last angles. %s", _measured ? "Restart model from neutral" : "Resend all four targets");
		}
		_status.stale = true;
		_state.invalidate();
		if (_measured && _model_started) { _model_fault = true; }
	}
}

void MorphingArmPublisher::process(const vehicle_command_s &command, uint64_t now)
{
	if (command.command != vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR || !PX4_ISFINITE(command.param7)
	    || command.param7 < 0.f || command.param7 > 0.f) { return; }

	if (command.timestamp <= _last_seen || command.timestamp > now || now - command.timestamp > _timeout_us) {
		++_status.rejected_commands;
		if (_measured) { _model_fault = true; _state.invalidate(); _status.stale = true; }
		return;
	}

	_last_seen = command.timestamp;
	if (_measured) { processMeasured(command, now); return; }
	const float values[] = {command.param1, command.param2, command.param3, command.param4};
	const auto result = _state.accept(values, _calibration);

	if (result == morphing_publisher::ArmCommandState::Result::Invalid) {
		++_status.rejected_commands;
		// The output driver may have accepted other fields: require a full resync.
		_state.invalidate();
		_status.stale = true;
		if (_last_invalid_warning == 0 || now - _last_invalid_warning >= 1000000) {
			PX4_WARN("Invalid arm command; geometry unchanged (servo output path is independent)");
			_last_invalid_warning = now;
		}
		return;
	}

	if (result == morphing_publisher::ArmCommandState::Result::Unchanged) { return; }
	_status.timestamp_command = command.timestamp;

	if (result != morphing_publisher::ArmCommandState::Result::Complete) { return; }

	morphing_arm_state_s state{};
	// Never refresh stale geometry without a new accepted command.
	state.timestamp = now > _status.timestamp_angle ? now : _status.timestamp_angle + 1;
	for (unsigned i = 0; i < 4; ++i) { state.angle[i] = _state.angles[i]; }

	if (_angles.publish(state)) {
		_status.timestamp_angle = state.timestamp;
		_status.stale = false;
		++_status.published_messages;
	}
}

void MorphingArmPublisher::publishStatus(uint64_t now)
{
	_status.timestamp = now;
	_status.enabled = enabled();
	_status.initialized_mask = _state.initialized_mask;
	_status_pub.publish(_status);
	_last_status = now;
}

void MorphingArmPublisher::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	const uint64_t now = hrt_absolute_time();
	const bool active = enabled();

	if (!active || !_previous_enabled) {
		if (_measured && _model_started) { _model_fault = true; }
		_state.invalidate();
		_status.stale = true;
		_last_seen = now; // Do not replay commands sent while disabled.
	}

	_previous_enabled = active;
	if (active) { checkTimeout(now); }
	vehicle_command_s command{};

	// Bound queue draining so a busy command source cannot monopolize the work queue.
	for (unsigned i = 0; i < vehicle_command_s::ORB_QUEUE_LENGTH; ++i) {
		const unsigned generation = _commands.get_last_generation();
		if (!_commands.update(&command)) { break; }

		if (_commands.get_last_generation() - generation > 1) {
			if (_measured && _model_started) { _model_fault = true; }
			++_status.queue_overruns;
			_state.invalidate();
			_status.stale = true;
		}

		if (active) { process(command, now); }
	}
	if (active && _measured) { advanceMeasured(now); publishMeasured(now); }

	if (now - _last_status >= 100000) { publishStatus(now); }
}

int MorphingArmPublisher::print_status()
{
	PX4_INFO("MAVLink group 0: %s; no position feedback", _measured ? "FR RL FL RR (measured model)" : "FR RR RL FL (legacy)");
	if (_measured) { PX4_INFO("calibration=%d model_fault=%d; host/mocap delay excluded", morphing_publisher::measured::CALIBRATION_ID, _model_fault); }
	PX4_INFO("enabled=%d calibration=%d initialized=0x%x stale=%d published=%lu rejected=%lu",
		 enabled(), _status.calibration_valid, _state.initialized_mask, _status.stale,
		 (unsigned long)_status.published_messages, (unsigned long)_status.rejected_commands);
	return PX4_OK;
}

int MorphingArmPublisher::task_spawn(int argc, char *argv[])
{
	auto *instance = new MorphingArmPublisher();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		if (instance->init()) { return PX4_OK; }
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int MorphingArmPublisher::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }
	PRINT_MODULE_USAGE_NAME("morphing_arm_publisher", "controller");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start calibrated MAVLink arm-angle bridge (restart after calibration changes)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return PX4_OK;
}

extern "C" __EXPORT int morphing_arm_publisher_main(int argc, char *argv[])
{
	return MorphingArmPublisher::main(argc, argv);
}

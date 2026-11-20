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

#include "KfB1.hpp"

// B1 column order [front-left, rear-left, rear-right, front-right] as actuator_motors.control indices
// (quad X: 0 front-right, 1 rear-left, 2 front-left, 3 rear-right).
static constexpr int PX4_MOTOR_INDEX[4] = {2, 1, 3, 0};

KfB1::KfB1() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_sample_dt_mean.setAlpha(0.01f);

	updateParams();
	_kf.reset(_g0);
}

KfB1::~KfB1()
{
	perf_free(_cycle_perf);
	perf_free(_kf_interval_perf);
}

bool KfB1::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void KfB1::updateParams()
{
	ModuleParams::updateParams();

	const float p0[4] {_param_kfb1_p0_0.get(), _param_kfb1_p0_1.get(), _param_kfb1_p0_2.get(), _param_kfb1_p0_3.get()};
	const float q[4] {_param_kfb1_q0.get(), _param_kfb1_q1.get(), _param_kfb1_q2.get(), _param_kfb1_q3.get()};
	const float r[4] {_param_kfb1_r0.get(), _param_kfb1_r1.get(), _param_kfb1_r2.get(), _param_kfb1_r3.get()};

	// New noise values apply from the next step; new P0 and G0 at the next reset (arming).
	_kf.setParameters(Vec(p0), Vec(q), Vec(r), _param_kfb1_pcap.get());

	const float g0[16] {
		_param_kfb1_g0_00.get(), _param_kfb1_g0_01.get(), _param_kfb1_g0_02.get(), _param_kfb1_g0_03.get(),
		_param_kfb1_g0_10.get(), _param_kfb1_g0_11.get(), _param_kfb1_g0_12.get(), _param_kfb1_g0_13.get(),
		_param_kfb1_g0_20.get(), _param_kfb1_g0_21.get(), _param_kfb1_g0_22.get(), _param_kfb1_g0_23.get(),
		_param_kfb1_g0_30.get(), _param_kfb1_g0_31.get(), _param_kfb1_g0_32.get(), _param_kfb1_g0_33.get()};

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			_g0(i, j) = g0[4 * i + j];
		}
	}

	// Re-derive the input filters at the next sample.
	_filter_sample_rate_hz = 0.f;
}

void KfB1::updateAppliedCommand(hrt_abstime t)
{
	// the pending command becomes the applied one once a gyro sample newer than its source sample arrives
	auto apply_pending_if_older = [this, t]() {
		if (!_have_pending || (_u_pending.timestamp_sample >= t)) {
			return;
		}

		Vec u;
		bool finite = true;

		for (int i = 0; i < 4; i++) {
			u(i) = _u_pending.control[PX4_MOTOR_INDEX[i]];
			finite = finite && PX4_ISFINITE(u(i));
		}

		_have_u = finite;  // NaN = disarmed / stopped

		if (finite) {
			_u_applied = u;
		}

		_have_pending = false;
	};

	actuator_motors_s motors;

	if (_actuator_motors_sub.update(&motors)) {
		apply_pending_if_older();
		_u_pending = motors;
		_have_pending = true;
	}

	apply_pending_if_older();
}

void KfB1::resetFilters()
{
	for (int i = 0; i < 4; i++) {
		_u_motor_lag[i].reset(_u_applied(i));
		const float u_gyro = _u_filter_gyro[i].reset(_u_applied(i));
		_u_filter_alpha[i].reset(u_gyro);
		_u_alpha(i) = u_gyro;
		_u_az(i) = _u_filter_az[i].reset(_u_applied(i));
	}

	_filters_running = true;
	_anchored = false;
}

void KfB1::updateFilters(float dt)
{
	// Set the 2nd-order filters' sample rate to the measured gyro publication rate.
	if (_sample_dt_mean.getState() <= 0.f) {
		_sample_dt_mean.reset(dt);

	} else {
		_sample_dt_mean.update(dt);
	}

	const float rate_hz = 1.f / _sample_dt_mean.getState();

	if ((_filter_sample_rate_hz <= 0.f) || (fabsf(rate_hz - _filter_sample_rate_hz) > 0.1f * _filter_sample_rate_hz)) {
		_filter_sample_rate_hz = rate_hz;

		for (int i = 0; i < 4; i++) {
			// restart the filters from the current command
			_u_filter_az[i].set_cutoff_frequency(rate_hz, _param_imu_accel_cutoff.get());
			_u_az(i) = _u_filter_az[i].reset(_u_motor_lag[i].getState());
			_u_filter_gyro[i].set_cutoff_frequency(rate_hz, _param_imu_gyro_cutoff.get());
			_u_filter_gyro[i].reset(_u_motor_lag[i].getState());
			// cutoff 0 disables the filter (pass-through)
			_extra_nu[i].set_cutoff_frequency(rate_hz, _param_kfb1_lpf_hz.get());
			_extra_u_az[i].set_cutoff_frequency(rate_hz, _param_kfb1_lpf_hz.get());
			_extra_u_alpha[i].set_cutoff_frequency(rate_hz, _param_kfb1_lpf_hz.get());
		}

		_anchored = false;  // new delta history
	}

	// Same chain as PX4's angular acceleration: gyro low-pass (2nd order), then the 1st-order
	// derivative filter (pass-through if its cutoff is disabled).
	const float dgyro_cutoff = _param_imu_dgyro_cutoff.get();
	const float tau = (dgyro_cutoff > 0.f) ? 1.f / (2.f * M_PI_F * dgyro_cutoff) : 0.f;

	for (int i = 0; i < 4; i++) {
		// motor response model first (pass-through when KFB1_TAU_M = 0), then the measurement filters
		_u_motor_lag[i].setParameters(dt, _param_kfb1_tau_m.get());
		const float u_motor = _u_motor_lag[i].update(_u_applied(i));

		_u_filter_alpha[i].setParameters(dt, tau);
		_u_alpha(i) = _u_filter_alpha[i].update(_u_filter_gyro[i].apply(u_motor));
		_u_az(i) = _u_filter_az[i].apply(u_motor);
	}
}

void KfB1::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

		if (armed && !_armed) {
			// every flight starts from the prior
			_kf.reset(_g0);
			_init_source = b1_estimate_s::INIT_PRIOR;
			_kf_steps = 0;
			_kf_reset_since_arming = true;
			_anchored = false;
		}

		_armed = armed;
	}

	vehicle_land_detected_s land_detected;

	if (_vehicle_land_detected_sub.update(&land_detected)) {
		_landed = land_detected.landed;
	}

	vehicle_acceleration_s acceleration;

	if (_vehicle_acceleration_sub.update(&acceleration)) {
		_acc_z_flu = -acceleration.xyz[2];  // FRD -> FLU
		_acc_sample = acceleration.timestamp_sample;
		_have_acc = true;
	}

	vehicle_angular_velocity_s angular_velocity;

	if (!_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		perf_end(_cycle_perf);
		return;
	}

	const hrt_abstime t = angular_velocity.timestamp_sample;
	const float dt = ((_last_gyro_sample != 0) && (t > _last_gyro_sample)) ? (t - _last_gyro_sample) * 1e-6f : 0.f;
	_last_gyro_sample = t;

	updateAppliedCommand(t);

	// Input filters run on every gyro sample.
	if (!_have_u) {
		_filters_running = false;
		_anchored = false;

	} else if (!_filters_running || (dt <= 0.f) || (dt > MAX_SAMPLE_GAP)) {
		resetFilters();

	} else {
		updateFilters(dt);
	}

	// measurement usable: fresh accelerometer sample (within MAX_SAMPLE_GAP of the gyro) and all finite
	const float acc_age = fabsf((float)((int64_t)t - (int64_t)_acc_sample)) * 1e-6f;
	const bool meas_ok = _have_acc && (acc_age < MAX_SAMPLE_GAP) && PX4_ISFINITE(_acc_z_flu)
			     && PX4_ISFINITE(angular_velocity.xyz_derivative[0]) && PX4_ISFINITE(angular_velocity.xyz_derivative[1])
			     && PX4_ISFINITE(angular_velocity.xyz_derivative[2]);

	// Throw identification: during the hover pulses (and the low-pass settling after them), fit B1 by
	// batch least squares instead of stepping the KF; then restart the KF from that fit (batchFinish).
	throw_sysid_status_s throw_status;

	if (_throw_sysid_status_sub.update(&throw_status)) {
		_throw_phase = throw_status.phase;
	}

	if (_throw_phase == throw_sysid_status_s::PHASE_EXCITE) {
		const float lpf_hz = _param_kfb1_lpf_hz.get();
		_batch_end = t + (hrt_abstime)((lpf_hz > 0.f) ? 1.5e6f / lpf_hz : 0.f);
	}

	if (_throw_phase == throw_sysid_status_s::PHASE_EXCITE || (_batch_active && t < _batch_end)) {
		if (!_batch_active) {
			for (int o = 0; o < 4; o++) {
				_batch_A[o].setZero();
				_batch_b[o].setZero();
			}

			_batch_n = 0;
			_batch_prev_valid = false;
			_batch_active = true;
		}

		if (_have_u && meas_ok && _filters_running) {
			batchAccumulate(angular_velocity.xyz_derivative);

		} else {
			_batch_prev_valid = false;
		}

		_anchored = false;  // the KF restarts its delta history after the excitation
		perf_end(_cycle_perf);
		return;

	} else if (_batch_active) {
		_batch_active = false;
		batchFinish();
	}

	const bool active = _armed && !_landed && _kf_reset_since_arming && _have_u && meas_ok && _filters_running;

	if (active) {
		// nu = [a_z, alpha_x, alpha_y, alpha_z] in FLU
		Vec nu;
		nu(0) = _acc_z_flu;
		nu(1) = angular_velocity.xyz_derivative[0];
		nu(2) = -angular_velocity.xyz_derivative[1];
		nu(3) = -angular_velocity.xyz_derivative[2];

		// identical extra low-pass on both sides of the regression (restarted with the delta history)
		Vec u_az = _u_az;
		Vec u_alpha = _u_alpha;

		for (int i = 0; i < 4; i++) {
			if (!_anchored) {
				nu(i) = _extra_nu[i].reset(nu(i));
				u_az(i) = _extra_u_az[i].reset(u_az(i));
				u_alpha(i) = _extra_u_alpha[i].reset(u_alpha(i));

			} else {
				nu(i) = _extra_nu[i].apply(nu(i));
				u_az(i) = _extra_u_az[i].apply(u_az(i));
				u_alpha(i) = _extra_u_alpha[i].apply(u_alpha(i));
			}
		}

		if (!_anchored) {
			// start a fresh delta history (after arming, takeoff, a gap or a filter restart)
			_anchored = true;

		} else if ((t - _last_kf_sample) >= (hrt_abstime)_param_kfb1_dt_us.get()) {
			// delta nu over the interval against the delta of the (filtered) command applied in it
			Mat X;
			const Vec du_az = u_az - _u_az_prev;
			const Vec du_alpha = u_alpha - _u_alpha_prev;

			for (int j = 0; j < 4; j++) {
				X(0, j) = du_az(j);
				X(1, j) = du_alpha(j);
				X(2, j) = du_alpha(j);
				X(3, j) = du_alpha(j);
			}

			const float dt_ratio = (t - _last_kf_sample) * 1e-6f / NOMINAL_KF_DT;
			_kf.step(X, nu - _nu_prev, dt_ratio);
			_kf_steps++;
			perf_count(_kf_interval_perf);

			publish(t, true);

		} else {
			// between KF steps: keep the previous anchor
			perf_end(_cycle_perf);
			return;
		}

		_nu_prev = nu;
		_u_az_prev = u_az;
		_u_alpha_prev = u_alpha;
		_last_kf_sample = t;

	} else {
		_anchored = false;

		// not updating: publish the held estimate with valid = false at 10 Hz
		if (hrt_elapsed_time(&_last_publish) > 100_ms) {
			publish(t, false);
		}
	}

	perf_end(_cycle_perf);
}

void KfB1::publish(hrt_abstime timestamp_sample, bool valid)
{
	b1_estimate_s msg{};
	msg.timestamp_sample = timestamp_sample;

	const Mat &g = _kf.G();

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			msg.b1[4 * i + j] = g(i, j);
		}
	}

	msg.valid = valid;
	msg.init_source = _init_source;
	msg.timestamp = hrt_absolute_time();
	_b1_estimate_pub.publish(msg);
	_last_publish = msg.timestamp;

	// copy to debug_array (MAVLink DEBUG_FLOAT_ARRAY), at most 20 Hz
	if (msg.timestamp - _last_debug >= 50_ms) {
		debug_array_s dbg{};
		dbg.id = DEBUG_ARRAY_ID;
		strncpy(dbg.name, "kf_b1", sizeof(dbg.name));

		for (int k = 0; k < 16; k++) { dbg.data[k] = msg.b1[k]; }

		dbg.data[16] = valid ? 1.f : 0.f;
		dbg.data[17] = msg.init_source;
		dbg.data[18] = static_cast<float>(_kf_steps);
		dbg.data[19] = _armed ? 1.f : 0.f;
		dbg.data[20] = _landed ? 1.f : 0.f;
		dbg.timestamp = msg.timestamp;
		_debug_array_pub.publish(dbg);
		_last_debug = msg.timestamp;
	}
}

int KfB1::print_status()
{
	PX4_INFO("armed: %d, landed: %d, motor command: %d, KF steps since arming: %" PRIu32,
		 _armed, _landed, _have_u, _kf_steps);
	PX4_INFO("input filter rate: %.1f Hz", (double)_filter_sample_rate_hz);

	const Mat &g = _kf.G();
	PX4_INFO("B1 estimate (rows a_z, alpha_x, alpha_y, alpha_z; cols FL, RL, RR, FR; FLU):");

	for (int i = 0; i < 4; i++) {
		PX4_INFO("  % 9.3f % 9.3f % 9.3f % 9.3f", (double)g(i, 0), (double)g(i, 1), (double)g(i, 2), (double)g(i, 3));
	}

	if (_throw_valid) {
		PX4_INFO("last throw estimate (%" PRIu32 " samples, used to restart the KF):", _batch_n);

		for (int i = 0; i < 4; i++) {
			PX4_INFO("  % 9.3f % 9.3f % 9.3f % 9.3f", (double)_throw_G(i, 0), (double)_throw_G(i, 1),
				 (double)_throw_G(i, 2), (double)_throw_G(i, 3));
		}
	}

	PX4_INFO("filter started from: %s", _init_source == b1_estimate_s::INIT_THROW ? "throw estimate" : "prior G0");

	perf_print_counter(_cycle_perf);
	perf_print_counter(_kf_interval_perf);
	return 0;
}

void KfB1::batchAccumulate(const float alpha_frd[3])
{
	// same filters as the KF, including the KFB1_LPF_HZ low-pass (the KF restarts these after the fit)
	Vec nu;
	nu(0) = _acc_z_flu;
	nu(1) = alpha_frd[0];
	nu(2) = -alpha_frd[1];
	nu(3) = -alpha_frd[2];
	Vec u_az = _u_az;
	Vec u_alpha = _u_alpha;

	for (int i = 0; i < 4; i++) {
		if (!_batch_prev_valid) {
			nu(i) = _extra_nu[i].reset(nu(i));
			u_az(i) = _extra_u_az[i].reset(u_az(i));
			u_alpha(i) = _extra_u_alpha[i].reset(u_alpha(i));

		} else {
			nu(i) = _extra_nu[i].apply(nu(i));
			u_az(i) = _extra_u_az[i].apply(u_az(i));
			u_alpha(i) = _extra_u_alpha[i].apply(u_alpha(i));
		}
	}

	if (_batch_prev_valid) {
		const Vec dnu = nu - _batch_nu_prev;
		const Vec du_az = u_az - _batch_u_az_prev;
		const Vec du_alpha = u_alpha - _batch_u_alpha_prev;

		for (int o = 0; o < 4; o++) {
			const Vec &x = (o == 0) ? du_az : du_alpha;

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					_batch_A[o](i, j) += x(i) * x(j);
				}

				_batch_b[o](i) += x(i) * dnu(o);
			}
		}

		_batch_n++;
	}

	_batch_nu_prev = nu;
	_batch_u_az_prev = u_az;
	_batch_u_alpha_prev = u_alpha;
	_batch_prev_valid = true;
}

void KfB1::batchFinish()
{
	// Own-pattern gains (thrust, roll, pitch, yaw) of a row, in B1 column order [FL, RL, RR, FR]
	static constexpr float PATTERN[4][4] = {{1, 1, 1, 1}, {1, 1, -1, -1}, {-1, 1, 1, -1}, {1, -1, 1, -1}};

	_throw_valid = false;

	if (_batch_n < (uint32_t)_param_kfb1_thr_nmin.get()) {
		PX4_WARN("throw fit: only %" PRIu32 " samples (< KFB1_THR_NMIN): keeping the current estimate", _batch_n);
		return;
	}

	Mat G;
	Mat P[4];

	for (int o = 0; o < 4; o++) {
		Mat A_inv;

		if (!matrix::inv(_batch_A[o], A_inv)) {
			PX4_WARN("throw fit: row %d not invertible (motor not excited?)", o);
			return;
		}

		const Vec g = A_inv * _batch_b[o];

		if (!g.isAllFinite()) {
			PX4_WARN("throw fit: row %d not finite: keeping the current estimate", o);
			return;
		}

		float own = 0.f;

		for (int j = 0; j < 4; j++) {
			G(o, j) = g(j);
			own += PATTERN[o][j] * g(j) / 4.f;
		}

		// KF restart covariance: each entry known to +-KFB1_THR_STD x |own gain| of its row
		const float std = fmaxf(_param_kfb1_thr_std.get() * fabsf(own), 1e-3f);
		P[o].setZero();

		for (int j = 0; j < 4; j++) {
			P[o](j, j) = std * std;
		}
	}

	_throw_G = G;
	_throw_valid = true;

	PX4_INFO("throw fit (%" PRIu32 " samples), own gains: a_z %.2f, roll %.1f, pitch %.1f, yaw %.2f", _batch_n,
		 (double)((G(0, 0) + G(0, 1) + G(0, 2) + G(0, 3)) / 4.f),
		 (double)((G(1, 0) + G(1, 1) - G(1, 2) - G(1, 3)) / 4.f),
		 (double)((-G(2, 0) + G(2, 1) + G(2, 2) - G(2, 3)) / 4.f),
		 (double)((G(3, 0) - G(3, 1) + G(3, 2) - G(3, 3)) / 4.f));

	_kf.reset(G, P);
	_init_source = b1_estimate_s::INIT_THROW;
	PX4_INFO("throw fit used: KF restarted from it");
}

int KfB1::task_spawn(int argc, char *argv[])
{
	KfB1 *instance = new KfB1();

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

int KfB1::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int KfB1::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Online Kalman-filter estimate of the reduced control-effectiveness matrix B1
(delta nu = B1 delta u, nu = [a_z, alpha_x, alpha_y, alpha_z]). Estimator only:
publishes b1_estimate (rows FLU, columns FL, RL, RR, FR) and does not affect control.
Updates while armed and not landed; resets to the prior on arming. During throw_sysid's
hover pulses it fits B1 by least squares and restarts the filter from that fit.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("kf_b1", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int kf_b1_main(int argc, char *argv[])
{
	return KfB1::main(argc, argv);
}

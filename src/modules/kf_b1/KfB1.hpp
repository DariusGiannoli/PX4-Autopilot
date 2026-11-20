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

/**
 * @file KfB1.hpp
 * Online estimate of B1 from delta nu = B1 delta u; publishes b1_estimate, does not affect control.
 * u goes through a motor-lag model and PX4's measurement filters; both sides through KFB1_LPF_HZ.
 */

#pragma once

#include "KalmanB1.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/b1_estimate.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/throw_sysid_status.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class KfB1 : public ModuleBase<KfB1>, public ModuleParams, public px4::WorkItem
{
public:
	KfB1();
	~KfB1() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	using Vec = KalmanB1::Vec;
	using Mat = KalmanB1::Mat;

	void Run() override;

	void updateParams() override;

	/** Track the newest motor command that was computed from an earlier gyro sample than t. */
	void updateAppliedCommand(hrt_abstime t);

	/** Restart the input filters from the current command and drop the delta history. */
	void resetFilters();

	/** Run the input filters one sample; adjusts the 2nd-order filter to the measured sample rate. */
	void updateFilters(float dt);

	void publish(hrt_abstime timestamp_sample, bool valid);

	/** Throw identification: accumulate one sample of the hover pulses (with the KFB1_LPF_HZ low-pass). */
	void batchAccumulate(const float alpha_frd[3]);

	/** Throw identification: solve the batch least squares and restart the KF from it. */
	void batchFinish();

	static constexpr float NOMINAL_KF_DT = 0.002f;  ///< step length the Q parameters are defined for [s]
	static constexpr uint16_t DEBUG_ARRAY_ID = 0xB1;  ///< debug_array id: data[0..15] B1 row-major, 16 valid, 17 init_source, 18 KF steps, 19 armed, 20 landed
	static constexpr float MAX_SAMPLE_GAP = 0.02f;  ///< longer gaps between gyro samples restart the deltas [s]

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _throw_sysid_status_sub{ORB_ID(throw_sysid_status)};

	// Publication
	uORB::Publication<b1_estimate_s> _b1_estimate_pub{ORB_ID(b1_estimate)};
	uORB::Publication<debug_array_s> _debug_array_pub{ORB_ID(debug_array)};

	KalmanB1 _kf;
	Mat _g0{};

	// Vehicle state
	bool _armed{false};
	bool _landed{true};
	bool _kf_reset_since_arming{false};
	float _acc_z_flu{0.f};
	hrt_abstime _acc_sample{0};
	bool _have_acc{false};

	// Motor command actually applied (B1 column order), and a newer one not yet applied
	Vec _u_applied{};
	bool _have_u{false};
	actuator_motors_s _u_pending{};
	bool _have_pending{false};

	// Input filters on u, one per motor: matched to the a_z and alpha measurement filters
	AlphaFilter<float> _u_motor_lag[4];  ///< motor response model u / (KFB1_TAU_M s + 1)
	math::LowPassFilter2p<float> _u_filter_az[4];
	math::LowPassFilter2p<float> _u_filter_gyro[4];  ///< first stage of the alpha path (IMU_GYRO_CUTOFF)
	AlphaFilter<float> _u_filter_alpha[4];
	Vec _u_az{};     ///< latest u through the a_z-row filter
	Vec _u_alpha{};  ///< latest u through the alpha-row filter

	// Identical extra low-pass on nu and both filtered u (KFB1_LPF_HZ)
	math::LowPassFilter2p<float> _extra_nu[4];
	math::LowPassFilter2p<float> _extra_u_az[4];
	math::LowPassFilter2p<float> _extra_u_alpha[4];
	AlphaFilter<float> _sample_dt_mean{};
	float _filter_sample_rate_hz{0.f};
	bool _filters_running{false};

	// Delta bookkeeping between KF steps
	hrt_abstime _last_gyro_sample{0};
	hrt_abstime _last_kf_sample{0};
	bool _anchored{false};
	Vec _nu_prev{};
	Vec _u_az_prev{};
	Vec _u_alpha_prev{};

	// Throw identification (batch least squares over throw_sysid's EXCITE phase plus the low-pass settling), per row o:
	// normal equations A_o = sum x x^T, b_o = sum x y over consecutive-sample deltas
	uint8_t _throw_phase{0};
	bool _batch_active{false};
	bool _batch_prev_valid{false};
	hrt_abstime _batch_end{0};   ///< fit window end: last pulse sample + low-pass settling
	Mat _batch_A[4] {};
	Vec _batch_b[4] {};
	uint32_t _batch_n{0};
	Vec _batch_nu_prev{};
	Vec _batch_u_az_prev{};
	Vec _batch_u_alpha_prev{};
	Mat _throw_G{};          ///< last batch estimate
	bool _throw_valid{false};
	uint8_t _init_source{0};  ///< b1_estimate_s::INIT_*

	uint32_t _kf_steps{0};
	hrt_abstime _last_publish{0};
	hrt_abstime _last_debug{0};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _kf_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": kf interval")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::KFB1_DT_US>) _param_kfb1_dt_us,
		(ParamFloat<px4::params::KFB1_R0>) _param_kfb1_r0,
		(ParamFloat<px4::params::KFB1_R1>) _param_kfb1_r1,
		(ParamFloat<px4::params::KFB1_R2>) _param_kfb1_r2,
		(ParamFloat<px4::params::KFB1_R3>) _param_kfb1_r3,
		(ParamFloat<px4::params::KFB1_Q0>) _param_kfb1_q0,
		(ParamFloat<px4::params::KFB1_Q1>) _param_kfb1_q1,
		(ParamFloat<px4::params::KFB1_Q2>) _param_kfb1_q2,
		(ParamFloat<px4::params::KFB1_Q3>) _param_kfb1_q3,
		(ParamFloat<px4::params::KFB1_P0_0>) _param_kfb1_p0_0,
		(ParamFloat<px4::params::KFB1_P0_1>) _param_kfb1_p0_1,
		(ParamFloat<px4::params::KFB1_P0_2>) _param_kfb1_p0_2,
		(ParamFloat<px4::params::KFB1_P0_3>) _param_kfb1_p0_3,
		(ParamFloat<px4::params::KFB1_PCAP>) _param_kfb1_pcap,
		(ParamFloat<px4::params::KFB1_LPF_HZ>) _param_kfb1_lpf_hz,
		(ParamFloat<px4::params::KFB1_TAU_M>) _param_kfb1_tau_m,
		(ParamFloat<px4::params::KFB1_G0_00>) _param_kfb1_g0_00,
		(ParamFloat<px4::params::KFB1_G0_01>) _param_kfb1_g0_01,
		(ParamFloat<px4::params::KFB1_G0_02>) _param_kfb1_g0_02,
		(ParamFloat<px4::params::KFB1_G0_03>) _param_kfb1_g0_03,
		(ParamFloat<px4::params::KFB1_G0_10>) _param_kfb1_g0_10,
		(ParamFloat<px4::params::KFB1_G0_11>) _param_kfb1_g0_11,
		(ParamFloat<px4::params::KFB1_G0_12>) _param_kfb1_g0_12,
		(ParamFloat<px4::params::KFB1_G0_13>) _param_kfb1_g0_13,
		(ParamFloat<px4::params::KFB1_G0_20>) _param_kfb1_g0_20,
		(ParamFloat<px4::params::KFB1_G0_21>) _param_kfb1_g0_21,
		(ParamFloat<px4::params::KFB1_G0_22>) _param_kfb1_g0_22,
		(ParamFloat<px4::params::KFB1_G0_23>) _param_kfb1_g0_23,
		(ParamFloat<px4::params::KFB1_G0_30>) _param_kfb1_g0_30,
		(ParamFloat<px4::params::KFB1_G0_31>) _param_kfb1_g0_31,
		(ParamFloat<px4::params::KFB1_G0_32>) _param_kfb1_g0_32,
		(ParamFloat<px4::params::KFB1_G0_33>) _param_kfb1_g0_33,
		(ParamInt<px4::params::KFB1_THR_NMIN>) _param_kfb1_thr_nmin,
		(ParamFloat<px4::params::KFB1_THR_STD>) _param_kfb1_thr_std,
		(ParamFloat<px4::params::IMU_ACCEL_CUTOFF>) _param_imu_accel_cutoff,
		(ParamFloat<px4::params::IMU_GYRO_CUTOFF>) _param_imu_gyro_cutoff,
		(ParamFloat<px4::params::IMU_DGYRO_CUTOFF>) _param_imu_dgyro_cutoff
	)
};

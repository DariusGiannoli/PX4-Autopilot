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
 * @file ThrowSysid.hpp
 * Hover pulses for kf_b1: per-motor doublets added to the flying controller's motor commands
 * (by control_allocator, via throw_sysid_status.offset). kf_b1 fits B1 on PHASE_EXCITE.
 */

#pragma once

#include "Pulses.hpp"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/throw_sysid_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class ThrowSysid : public ModuleBase<ThrowSysid>, public ModuleParams, public px4::WorkItem
{
public:
	ThrowSysid();
	~ThrowSysid() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

	void requestArm() { _arm_request.store(true); }
	void requestCancel() { _cancel_request.store(true); }

private:
	void Run() override;

	void start(hrt_abstime now);
	void finish(uint8_t reason, hrt_abstime now);
	void publishStatus(hrt_abstime now);

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<throw_sysid_status_s> _status_pub{ORB_ID(throw_sysid_status)};

	HoverPulses _pulses;

	px4::atomic<bool> _arm_request{false};
	px4::atomic<bool> _cancel_request{false};

	uint8_t _phase{throw_sysid_status_s::PHASE_IDLE};
	uint8_t _end_reason{throw_sysid_status_s::END_NONE};
	hrt_abstime _start{0};
	hrt_abstime _last_status{0};
	float _offset[4] {};

	bool _armed{false};
	bool _landed{true};
	bool _flight_state_allows_pulses{false};
	uint8_t _nav_state{0};
	uint8_t _start_nav_state{0};
	bool _pos_z_valid{false};
	float _z{0.f};
	float _z_start{0.f};
	float _tilt_deg{0.f};
	float _base[4] {};        ///< motor commands averaged over ~0.2 s, for the clipping warning
	bool _base_valid{false};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::THRSID_AMP>) _param_amp,
		(ParamFloat<px4::params::THRSID_SLOT>) _param_slot,
		(ParamFloat<px4::params::THRSID_RATE_LIM>) _param_rate_lim,
		(ParamFloat<px4::params::THRSID_RATE_MAX>) _param_rate_max,
		(ParamFloat<px4::params::THRSID_TILT_MAX>) _param_tilt_max,
		(ParamFloat<px4::params::THRSID_MAX_DROP>) _param_max_drop
	)
};

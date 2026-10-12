// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include "ArmCommandState.hpp"
#include "MeasuredArmModel.hpp"
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/morphing_arm_state.h>
#include <uORB/topics/morphing_publisher_status.h>

class MorphingArmPublisher : public ModuleBase<MorphingArmPublisher>, public px4::ScheduledWorkItem
{
public:
	MorphingArmPublisher();
	~MorphingArmPublisher() override = default;
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);
	int print_status() override;
	bool init();

private:
	friend class MorphingArmPublisherTestPeer;
	bool configure();
	void Run() override;
	void process(const vehicle_command_s &command, uint64_t now);
	void checkTimeout(uint64_t now);
	void publishStatus(uint64_t now);
	bool enabled() const;
	static int readInteger(const char *name, int32_t *value);
	bool configureMeasured(int (*read_int)(const char *, int32_t *) = readInteger);
	void advanceMeasured(uint64_t now);
	void processMeasured(const vehicle_command_s &command, uint64_t now);
	void publishMeasured(uint64_t now);
	bool _measured{false};
	bool _model_started{false};
	bool _model_fault{false}; // Unknown history requires restart from neutral.
	uint64_t _model_time{0};
	int32_t _directions[4]{1, 1, 1, 1};
	morphing_publisher::MeasuredArmModel _models[4]{};
	morphing_publisher::Calibration _calibration{};
	morphing_publisher::ArmCommandState _state{};
	uORB::Subscription _commands{ORB_ID(vehicle_command)};
	uORB::Publication<morphing_arm_state_s> _angles{ORB_ID(morphing_arm_state)};
	uORB::Publication<morphing_publisher_status_s> _status_pub{ORB_ID(morphing_publisher_status)};
	morphing_publisher_status_s _status{};
	param_t _enable_param{PARAM_INVALID};
	uint64_t _start_time{0};
	uint64_t _last_seen{0};
	uint64_t _last_status{0};
	uint64_t _last_invalid_warning{0};
	uint64_t _timeout_us{1000000};
	bool _previous_enabled{false};
};

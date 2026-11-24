/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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

#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/morphing_arm_state.h>
#include <uORB/topics/morphing_publisher_status.h>
#include "../../morphing_arm_publisher/MeasuredArmCalibration.hpp"
#include "MORPHEUS_ARM_PREDICTION.hpp"

class MavlinkStreamOdometry : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamOdometry(mavlink); }

	static constexpr const char *get_name_static() { return "ODOMETRY"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ODOMETRY; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _vehicle_odometry_sub.advertised() ? MAVLINK_MSG_ID_ODOMETRY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES
		       + (_arm_state_sub.advertised() ? 60 + 5 + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0) : 0;
	}

private:
	explicit MavlinkStreamOdometry(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription _arm_state_sub{ORB_ID(morphing_arm_state)};
	uORB::Subscription _arm_status_sub{ORB_ID(morphing_publisher_status)};

	// MAP1 predicts arms at FC publication time and pairs them with this exact
	// flight-state sample. Never differentiate angles or use host joint truth.
	void send_arm_prediction(const vehicle_odometry_s &odom)
	{
		morphing_arm_state_s arm{};
		morphing_publisher_status_s status{};
		const uint64_t now = hrt_absolute_time();
		if (!_arm_state_sub.copy(&arm) || !_arm_status_sub.copy(&status)
		    || !status.enabled || !status.calibration_valid || status.stale
		    || status.initialized_mask != 15 || arm.timestamp == 0 || now < arm.timestamp
		    || now - arm.timestamp > 50000 || now < status.timestamp || now - status.timestamp > 200000
		    || now < status.timestamp_command || now - status.timestamp_command > 500000) { return; }
		mavlink_v2_extension_t msg{};
		msg.message_type = 42002;
		for (unsigned i = 0; i < 4; ++i) {
			if (!PX4_ISFINITE(arm.angle[i]) || !PX4_ISFINITE(arm.angular_velocity[i])) { return; }
		}
		morpheus_arm_prediction::encode(msg.payload, odom.reset_counter,
			morphing_publisher::measured::CALIBRATION_ID, odom.timestamp_sample,
			arm.timestamp, arm.angle, arm.angular_velocity);
		mavlink_msg_v2_extension_send_struct(_mavlink->get_channel(), &msg);
	}

	bool send() override
	{
		vehicle_odometry_s odom;

		if (_vehicle_odometry_sub.update(&odom)) {
			mavlink_odometry_t msg{};
			msg.time_usec = odom.timestamp_sample;

			// set the frame_id according to the local frame of the data
			switch (odom.pose_frame) {
			case vehicle_odometry_s::POSE_FRAME_NED:
				msg.frame_id = MAV_FRAME_LOCAL_NED;
				break;

			case vehicle_odometry_s::POSE_FRAME_FRD:
				msg.frame_id = MAV_FRAME_LOCAL_FRD;
				break;
			}

			switch (odom.velocity_frame) {
			case vehicle_odometry_s::VELOCITY_FRAME_NED:
				msg.child_frame_id = MAV_FRAME_LOCAL_NED;
				break;

			case vehicle_odometry_s::VELOCITY_FRAME_FRD:
				msg.child_frame_id = MAV_FRAME_LOCAL_FRD;
				break;

			case vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD:
				msg.child_frame_id = MAV_FRAME_BODY_FRD;
				break;
			}

			msg.x = odom.position[0];
			msg.y = odom.position[1];
			msg.z = odom.position[2];

			msg.q[0] = odom.q[0];
			msg.q[1] = odom.q[1];
			msg.q[2] = odom.q[2];
			msg.q[3] = odom.q[3];

			msg.vx = odom.velocity[0];
			msg.vy = odom.velocity[1];
			msg.vz = odom.velocity[2];

			// Current body rates
			msg.rollspeed  = odom.angular_velocity[0];
			msg.pitchspeed = odom.angular_velocity[1];
			msg.yawspeed   = odom.angular_velocity[2];

			// pose_covariance
			//  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle
			//  (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.)
			for (auto &pc : msg.pose_covariance) {
				pc = NAN;
			}

			msg.pose_covariance[0]  = odom.position_variance[0];  // X  row 0, col 0
			msg.pose_covariance[6]  = odom.position_variance[1];  // Y  row 1, col 1
			msg.pose_covariance[11] = odom.position_variance[2];  // Z  row 2, col 2

			msg.pose_covariance[15] = odom.orientation_variance[0];  // R  row 3, col 3
			msg.pose_covariance[18] = odom.orientation_variance[1];  // P  row 4, col 4
			msg.pose_covariance[20] = odom.orientation_variance[2];  // Y  row 5, col 5

			// velocity_covariance
			//  Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle
			//  (states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.)
			for (auto &vc : msg.velocity_covariance) {
				vc = NAN;
			}

			msg.velocity_covariance[0]  = odom.velocity_variance[0];   // X  row 0, col 0
			msg.velocity_covariance[6]  = odom.velocity_variance[1];   // Y  row 1, col 1
			msg.velocity_covariance[11] = odom.velocity_variance[2];   // Z  row 2, col 2

			msg.reset_counter = odom.reset_counter;

			// source: PX4 estimator
			msg.estimator_type = MAV_ESTIMATOR_TYPE_AUTOPILOT;

			msg.quality = odom.quality;

			mavlink_msg_odometry_send_struct(_mavlink->get_channel(), &msg);
			send_arm_prediction(odom);

			return true;
		}

		return false;
	}
};

#endif // ODOMETRY_HPP

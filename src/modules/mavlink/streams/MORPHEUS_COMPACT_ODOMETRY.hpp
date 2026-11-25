#pragma once

#include "MORPHEUS_ARM_PREDICTION.hpp"

namespace morpheus_compact_odometry
{
constexpr uint16_t MessageType = 42003;
constexpr unsigned PayloadSize = 106;

// MOD1 carries the state and the nine diagonal variances already populated
// by the standard stream. Keep the final byte nonzero so MAVLink 2 never
// truncates a genuine payload containing zero-valued trailing metadata.
inline void encode(uint8_t *payload, const mavlink_odometry_t &odometry)
{
	memcpy(payload, "MOD1", 4);
	morpheus_arm_prediction::put64(payload + 4, odometry.time_usec);
	const float fields[] = {
		odometry.x, odometry.y, odometry.z,
		odometry.q[0], odometry.q[1], odometry.q[2], odometry.q[3],
		odometry.vx, odometry.vy, odometry.vz,
		odometry.rollspeed, odometry.pitchspeed, odometry.yawspeed,
		odometry.pose_covariance[0], odometry.pose_covariance[6], odometry.pose_covariance[11],
		odometry.pose_covariance[15], odometry.pose_covariance[18], odometry.pose_covariance[20],
		odometry.velocity_covariance[0], odometry.velocity_covariance[6], odometry.velocity_covariance[11],
	};

	for (unsigned i = 0; i < sizeof(fields) / sizeof(fields[0]); ++i) {
		morpheus_arm_prediction::put_float(payload + 12 + sizeof(float) * i, fields[i]);
	}

	payload[100] = odometry.frame_id;
	payload[101] = odometry.child_frame_id;
	payload[102] = odometry.reset_counter;
	payload[103] = odometry.estimator_type;
	payload[104] = static_cast<uint8_t>(odometry.quality);
	payload[105] = 0xa5;
}
}

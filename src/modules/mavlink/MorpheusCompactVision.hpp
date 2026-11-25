#pragma once

#include <lib/timesync/MorpheusVisionTiming.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>

namespace morpheus_compact_vision
{
constexpr uint16_t MessageType = 42004;
constexpr unsigned WirePayloadSize = 75;
constexpr unsigned PoseDiagonals[] = {0, 6, 11, 15, 18, 20};

inline uint64_t read_integer(const uint8_t *p, unsigned bytes)
{
	uint64_t value = 0;
	for (unsigned i = 0; i < bytes; ++i) {
		value |= static_cast<uint64_t>(p[i]) << (8 * i);
	}
	return value;
}

inline float read_float(const uint8_t *p)
{
	const uint32_t bits = read_integer(p, 4);
	float value;
	static_assert(sizeof(value) == sizeof(bits), "compact vision uses IEEE float32");
	memcpy(&value, &bits, sizeof(value));
	return value;
}

inline bool decode(const mavlink_message_t &msg, uint8_t system, uint8_t component, mavlink_odometry_t &odom)
{
	if (msg.msgid != MAVLINK_MSG_ID_V2_EXTENSION || msg.len != WirePayloadSize
	    || mavlink_msg_v2_extension_get_message_type(&msg) != MessageType) {
		return false;
	}

	mavlink_v2_extension_t extension{};
	mavlink_msg_v2_extension_decode(&msg, &extension);
	const auto *p = extension.payload;
	if (extension.target_network != 0 || extension.target_system != system
	    || extension.target_component != component || memcmp(p, "MVI1", 4) != 0 || p[69] != 0xa5
	    || !MorpheusVisionTiming::strict_input(msg.sysid, msg.compid, p[67], p[64], p[65])) {
		return false;
	}

	mavlink_odometry_t result{};
	result.time_usec = read_integer(p + 4, 8);
	if (!result.time_usec) { return false; }
	result.x = read_float(p + 12);
	result.y = read_float(p + 16);
	result.z = read_float(p + 20);
	if (!std::isfinite(result.x) || !std::isfinite(result.y) || !std::isfinite(result.z)) { return false; }
	for (unsigned i = 0; i < 4; ++i) {
		result.q[i] = read_float(p + 24 + 4 * i);
		if (!std::isfinite(result.q[i])) { return false; }
	}
	for (unsigned i = 0; i < sizeof(PoseDiagonals) / sizeof(PoseDiagonals[0]); ++i) {
		const float value = read_float(p + 40 + 4 * i);
		if (!std::isfinite(value) || value < 0.f) { return false; }
		result.pose_covariance[PoseDiagonals[i]] = value;
	}
	result.vx = result.vy = result.vz = NAN;
	result.rollspeed = result.pitchspeed = result.yawspeed = NAN;
	for (unsigned i = 0; i < sizeof(result.velocity_covariance) / sizeof(float); ++i) {
		result.velocity_covariance[i] = NAN;
	}
	result.frame_id = p[64];
	result.child_frame_id = p[65];
	result.reset_counter = p[66];
	result.estimator_type = p[67];
	memcpy(&result.quality, p + 68, 1);
	odom = result;
	return true;
}
}

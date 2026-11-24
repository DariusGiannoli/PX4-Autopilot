#pragma once

// MAP1 is an explicitly little-endian wire format. V2_EXTENSION.payload starts
// at an unaligned offset: never cast its bytes to float/uint32_t/uint64_t*.
#include <cstdint>
#include <cstring>

namespace morpheus_arm_prediction {
static constexpr unsigned PAYLOAD_SIZE = 60;

inline void put32(uint8_t *p, uint32_t value)
{
	for (unsigned i = 0; i < 4; ++i) { p[i] = static_cast<uint8_t>(value >> (8 * i)); }
}

inline void put64(uint8_t *p, uint64_t value)
{
	for (unsigned i = 0; i < 8; ++i) { p[i] = static_cast<uint8_t>(value >> (8 * i)); }
}

inline void put_float(uint8_t *p, float value)
{
	static_assert(sizeof(float) == sizeof(uint32_t), "MAP1 requires float32");
	uint32_t bits;
	memcpy(&bits, &value, sizeof(bits));
	put32(p, bits);
}

inline void encode(uint8_t *payload, uint8_t reset, uint32_t calibration,
		   uint64_t odometry_stamp, uint64_t arm_stamp, const float *angle, const float *velocity)
{
	memcpy(payload, "MAP1", 4);
	payload[4] = 1; payload[5] = 15; payload[6] = reset; payload[7] = 1;
	put32(payload + 8, calibration);
	put64(payload + 12, odometry_stamp);
	put64(payload + 20, arm_stamp);
	for (unsigned i = 0; i < 4; ++i) {
		put_float(payload + 28 + 4 * i, angle[i]);
		put_float(payload + 44 + 4 * i, velocity[i]);
	}
}
} // namespace morpheus_arm_prediction

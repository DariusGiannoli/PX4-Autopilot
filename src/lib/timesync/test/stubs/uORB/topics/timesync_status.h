#pragma once
#include <stdint.h>
struct timesync_status_s {
 static constexpr uint8_t SOURCE_PROTOCOL_UNKNOWN = 0;
 uint64_t timestamp{}, remote_timestamp{};
 int64_t observed_offset{}, estimated_offset{};
 uint32_t round_trip_time{};
 uint8_t source_protocol{};
 bool vision_clock_ready{};
 uint8_t vision_clock_fault{};
 uint16_t vision_clock_samples{};
 int64_t vision_clock_offset{};
 uint32_t vision_clock_uncertainty_us{}, vision_clock_age_us{}, vision_clock_generation{};
};
extern timesync_status_s test_last_status;

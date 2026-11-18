/* SPDX-License-Identifier: BSD-3-Clause */
#pragma once
#include <stdint.h>

/** Existing private host protocol identities; these are not authentication.
 * Flight/bench uses 245:191; disarmed configuration housekeeping uses 246:191.
 * Both use the same host CLOCK_MONOTONIC epoch on the same MAVLink instance.
 * Other peers, frames and legacy callers retain their existing conversion.
 */
namespace MorpheusVisionTiming
{
inline bool clock_peer(uint8_t system, uint8_t component)
{
	return (system == 245 || system == 246) && component == 191;
}

inline bool strict_input(uint8_t system, uint8_t component, uint8_t estimator, uint8_t frame, uint8_t child_frame)
{
	// MAV_ESTIMATOR_TYPE_VISION=2, MAV_FRAME_LOCAL_FRD=20, MAV_FRAME_BODY_FRD=12.
	return system == 245 && component == 191 && estimator == 2 && frame == 20 && child_frame == 12;
}
}

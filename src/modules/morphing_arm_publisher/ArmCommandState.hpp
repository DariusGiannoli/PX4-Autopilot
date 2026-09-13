// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cmath>
#include <cstdint>

namespace morphing_publisher
{
struct Calibration {
	float negative[4]{}; // measured degrees at normalized -1
	float zero[4]{};     // measured degrees at center[arm]
	float positive[4]{}; // measured degrees at normalized +1
	float center[4]{};   // normalized command at middle calibration point

	bool valid() const
	{
		for (unsigned i = 0; i < 4; ++i) {
			if (!std::isfinite(center[i]) || std::fabs(center[i]) >= 1.f
			    || !std::isfinite(negative[i]) || !std::isfinite(zero[i]) || !std::isfinite(positive[i])
			    || std::fabs(negative[i]) > 30.f || std::fabs(zero[i]) > 30.f || std::fabs(positive[i]) > 30.f) {
				return false;
			}

			if (!((negative[i] < zero[i] && zero[i] < positive[i])
			      || (negative[i] > zero[i] && zero[i] > positive[i]))) {
				return false;
			}
		}

		return true;
	}

	float radians(unsigned arm, float value) const
	{
		const float degrees = value < center[arm]
			? zero[arm] + (center[arm] - value) / (center[arm] + 1.f) * (negative[arm] - zero[arm])
			: zero[arm] + (value - center[arm]) / (1.f - center[arm]) * (positive[arm] - zero[arm]);
		return degrees * 0.017453292519943295f;
	}
};

class ArmCommandState
{
public:
	enum class Result { Invalid, Unchanged, Partial, Complete };

	Result accept(const float (&values)[4], const Calibration &calibration)
	{
		bool changed = false;

		for (float value : values) {
			if (std::isnan(value)) { continue; }
			if (!std::isfinite(value) || std::fabs(value) > 1.f) { return Result::Invalid; }
			changed = true;
		}

		if (!calibration.valid()) { return Result::Invalid; }
		if (!changed) { return Result::Unchanged; }

		// Commit only after validation of the entire command. NaN retains state.
		for (unsigned i = 0; i < 4; ++i) {
			if (std::isfinite(values[i])) {
				angles[i] = calibration.radians(i, values[i]);
				initialized_mask |= 1u << i;
			}
		}

		return initialized_mask == 15 ? Result::Complete : Result::Partial;
	}

	void invalidate() { initialized_mask = 0; }
	float angles[4]{};
	uint8_t initialized_mask{0};
};
} // namespace morphing_publisher

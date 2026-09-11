// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cmath>
#include <cstddef>

/** Pure geometry calculation; no parameter, uORB, or output-driver dependencies.
 *
 * Input arrays: front right, rear right, rear left, front left (CAD/model order).
 * Angles: radians from CAD neutral, positive about CAD +Z (CCW viewed from above).
 * PX4 body frame: forward/right/down, inferred from the CAD link names:
 *   (x, y, z)_PX4 = (-y, -x, -z)_CAD.
 * Thus positive input angles rotate about PX4 -Z, not PX4 +Z.
 * Output columns: FR, RL, FL, RR (standard PX4 quad motor order).
 * Output rows: torque X/Y/Z, thrust X/Y/Z. This is B, not its inverse.
 */
namespace morphing_quad
{
/** Fixed aggregate storage: NuttX's minimal C++ library does not provide std::array. */
template<typename T, std::size_t N>
struct FixedArray {
	T elements[N];
	constexpr T &operator[](std::size_t i) { return elements[i]; }
	constexpr const T &operator[](std::size_t i) const { return elements[i]; }
	constexpr T *begin() { return elements; }
	constexpr T *end() { return elements + N; }
	constexpr const T *begin() const { return elements; }
	constexpr const T *end() const { return elements + N; }
	// Exact equality detects unchanged targets; ordered comparisons retain NaN != NaN.
	static constexpr bool equalElement(float a, float b) { return a <= b && a >= b; }
	template<typename U>
	static constexpr bool equalElement(const U &a, const U &b) { return a == b; }
	constexpr bool operator==(const FixedArray &other) const
	{
		for (std::size_t i = 0; i < N; ++i) {
			if (!equalElement(elements[i], other.elements[i])) { return false; }
		}
		return true;
	}
	constexpr bool operator!=(const FixedArray &other) const { return !(*this == other); }
};
using Vector3 = FixedArray<float, 3>;
using RotorValues = FixedArray<float, 4>;
using EffectivenessMatrix = FixedArray<FixedArray<float, 4>, 6>;

constexpr float MAX_ANGLE_RAD = 0.5235987755982988f; // 30 degrees
constexpr FixedArray<std::size_t, 4> MOTOR_TO_ARM{{0, 2, 3, 1}};

struct GeometryResult {
	FixedArray<Vector3, 4> rotor_position_body{}; // metres, model origin, motor order
	EffectivenessMatrix effectiveness{};       // torque about supplied CoG
};

/** Geometry extracted from the corrected morphing_quadrotor.urdf.
 * Hinge and neutral hinge-to-thrust-site offsets, in PX4 FRD metres.
 * The thrust site includes the rotor collision-origin offset used by the
 * reference dynamics model. Nanometre-scale horizontal export noise is omitted.
 * The nominal arm length in the horizontal plane is 0.05640203337 m.
 */
constexpr FixedArray<Vector3, 4> HINGES{{
	{{ 0.028f,  0.028f, -0.012f}},
	{{-0.028f,  0.028f, -0.012f}},
	{{-0.028f, -0.028f, -0.012f}},
	{{ 0.028f, -0.028f, -0.012f}},
}};
constexpr FixedArray<Vector3, 4> OFFSETS{{
	{{ 0.03992713f,  0.03983734f, -0.013f}},
	{{-0.03983734f,  0.03992713f, -0.013f}},
	{{-0.03992713f, -0.03983734f, -0.013f}},
	{{ 0.03983734f, -0.03992713f, -0.013f}},
}};

/** Calculate B for four vertical rotors. No heap allocation or side effects.
 *
 * ct: nonnegative effectiveness/thrust coefficients, in INPUT arm order.
 * km: signed reaction-torque/thrust ratios using PX4 convention, in INPUT order.
 *     No spin direction or measured propulsion coefficient is assumed here.
 * cog: centre of gravity in PX4 FRD metres relative to the CAD model origin.
 *
 * For physical rotor thrust inputs use ct=1; for actuator-effectiveness inputs
 * supply the appropriate coefficients. Allocation normalization is downstream.
 * Returns false for invalid inputs, leaving result unchanged. The command
 * interface should enforce limits and publish only accepted angles; this
 * function deliberately does not silently clamp a command.
 */
inline bool compute(const RotorValues &angles, const RotorValues &ct, const RotorValues &km,
		    const Vector3 &cog, GeometryResult &result)
{
	for (const float value : cog) {
		if (!std::isfinite(value)) {
			return false;
		}
	}

	for (std::size_t arm = 0; arm < 4; ++arm) {
		if (!std::isfinite(angles[arm]) || std::fabs(angles[arm]) > MAX_ANGLE_RAD
		    || !std::isfinite(ct[arm]) || ct[arm] < 0.f || !std::isfinite(km[arm])) {
			return false;
		}
	}

	GeometryResult next{};

	for (std::size_t motor = 0; motor < 4; ++motor) {
		const std::size_t arm = MOTOR_TO_ARM[motor];
		const float c = std::cos(angles[arm]);
		const float s = std::sin(angles[arm]);
		auto &position = next.rotor_position_body[motor];
		// Rz(-angle): CAD +Z becomes PX4 -Z.
		position[0] = HINGES[arm][0] + c * OFFSETS[arm][0] + s * OFFSETS[arm][1];
		position[1] = HINGES[arm][1] - s * OFFSETS[arm][0] + c * OFFSETS[arm][1];
		position[2] = HINGES[arm][2] + OFFSETS[arm][2];
		next.effectiveness[0][motor] = -ct[arm] * (position[1] - cog[1]);
		next.effectiveness[1][motor] =  ct[arm] * (position[0] - cog[0]);
		next.effectiveness[2][motor] =  ct[arm] * km[arm];
		next.effectiveness[5][motor] = -ct[arm];
	}

	for (const auto &row : next.effectiveness) {
		for (const float value : row) {
			if (!std::isfinite(value)) {
				return false; // Reject arithmetic overflow as well as nonfinite inputs.
			}
		}
	}

	result = next;
	return true;
}
} // namespace morphing_quad

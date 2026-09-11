// SPDX-License-Identifier: BSD-3-Clause
// Host-only test executable: compile the exact header intended for firmware.
#include "MorphingQuadGeometry.hpp"
#include <cassert>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

using namespace morphing_quad;

static void check_invalid_inputs()
{
	const RotorValues zero{};
	const RotorValues unit{{1.f, 1.f, 1.f, 1.f}};
	GeometryResult result;
	assert(compute(zero, unit, zero, Vector3{}, result));
	const GeometryResult saved = result;
	const auto reject = [&](const RotorValues &angles, const RotorValues &ct,
				const RotorValues &km, const Vector3 &cog) {
		assert(!compute(angles, ct, km, cog, result));
		assert(result.effectiveness == saved.effectiveness);
		assert(result.rotor_position_body == saved.rotor_position_body);
	};

	for (std::size_t i = 0; i < 4; ++i) {
		for (const float bad : {MAX_ANGLE_RAD + 0.001f, -MAX_ANGLE_RAD - 0.001f,
				       std::numeric_limits<float>::quiet_NaN(),
				       std::numeric_limits<float>::infinity()}) {
			RotorValues angles{};
			angles[i] = bad;
			reject(angles, unit, zero, Vector3{});
		}

		for (const float bad : {-1.f, std::numeric_limits<float>::quiet_NaN(),
				       std::numeric_limits<float>::infinity()}) {
			RotorValues ct = unit;
			ct[i] = bad;
			reject(zero, ct, zero, Vector3{});
		}

		RotorValues km{};
		km[i] = std::numeric_limits<float>::quiet_NaN();
		reject(zero, unit, km, Vector3{});
		km[i] = std::numeric_limits<float>::infinity();
		reject(zero, unit, km, Vector3{});
	}

	for (std::size_t i = 0; i < 3; ++i) {
		for (const float bad : {std::numeric_limits<float>::quiet_NaN(),
				       std::numeric_limits<float>::infinity()}) {
			Vector3 cog{};
			cog[i] = bad;
			reject(zero, unit, zero, cog);
		}
	}

	RotorValues enormous = unit;
	enormous[0] = std::numeric_limits<float>::max();
	reject(zero, enormous, unit, Vector3{{10.f, 10.f, 0.f}});
	assert(compute(zero, zero, zero, Vector3{}, result));
	assert(result.effectiveness == EffectivenessMatrix{});

	// Positive CAD angle on FR moves its rotor forward and towards the centreline.
	RotorValues angles{};
	angles[0] = MAX_ANGLE_RAD;
	assert(compute(angles, unit, zero, Vector3{}, result));
	assert(result.rotor_position_body[0][0] > saved.rotor_position_body[0][0]);
	assert(result.rotor_position_body[0][1] < saved.rotor_position_body[0][1]);
	for (std::size_t motor = 1; motor < 4; ++motor) {
		assert(result.rotor_position_body[motor] == saved.rotor_position_body[motor]);
	}
}

int main()
{
	check_invalid_inputs();
	std::cout << std::setprecision(9);
	std::string token;
	while (std::cin >> token) {
		RotorValues angles{}, ct{}, km{};
		Vector3 cog{};
		angles[0] = std::stof(token);
		for (std::size_t i = 1; i < 4; ++i) { std::cin >> angles[i]; }
		for (float &value : ct) { std::cin >> value; }
		for (float &value : km) { std::cin >> value; }
		for (float &value : cog) { std::cin >> value; }
		if (!std::cin) { return 2; }
		GeometryResult result;
		if (!compute(angles, ct, km, cog, result)) {
			std::cerr << "Unexpected rejected test input\n";
			return 3;
		}
		for (const auto &position : result.rotor_position_body) {
			for (const float value : position) { std::cout << value << ' '; }
		}
		for (const auto &row : result.effectiveness) {
			for (const float value : row) { std::cout << value << ' '; }
		}
		std::cout << '\n';
	}
}

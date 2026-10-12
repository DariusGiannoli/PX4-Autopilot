// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include "MeasuredArmCalibration.hpp"
#include <cmath>

namespace morphing_publisher {
// Same generalized-play/deadband and midpoint speed-limited model as the bench fit.
// Starts at initial neutral. Commands retain state; no resets at direction changes.
class MeasuredArmModel {
public:
	MeasuredArmModel() { reset(); }
	void reset()
	{
		_pwm = 1450.f; _held = -measured::BIAS / measured::GAIN;
		_play = _held - measured::ELASTIC * .5f * (lookup(_pwm, measured::LOWER) + lookup(_pwm, measured::UPPER));
		angle = 0.f; velocity = 0.f;
	}
	bool accept(float wire)
	{
		if (!std::isfinite(wire) || std::fabs(wire) > 1.f) { return false; }
		const float pwm = static_cast<float>(lroundf(1450.f + 300.f * wire));
		if (static_cast<int>(pwm) == static_cast<int>(_pwm)) { return true; }
		const float lo = lookup(pwm, measured::LOWER), hi = lookup(pwm, measured::UPPER);
		const float elastic = measured::ELASTIC * .5f * (lo + hi);
		_play = clip(_play, lo - elastic, hi - elastic);
		const float raw = _play + elastic;
		if (std::fabs(raw - _held) > measured::DEADBAND) { _held = raw; }
		_pwm = pwm;
		return true;
	}
	float target() const { return _held * measured::GAIN + measured::BIAS; }
	bool advance(float dt)
	{
		// A longer scheduler gap leaves the estimate unqualified; caller marks stale.
		if (!std::isfinite(dt) || dt < 0.f || dt > .1f) { return false; }
		const unsigned count = static_cast<unsigned>(ceilf(dt / .002f));
		if (count == 0) { return true; }
		const float h = dt / count;
		for (unsigned i = 0; i < count; ++i) {
			const float a = measured::OMEGA * measured::OMEGA * (target() - angle)
				- 2.f * measured::DAMPING * measured::OMEGA * velocity;
			const float vm = clip(velocity + .5f*h*a, -measured::VNEG, measured::VPOS);
			const float qm = angle + .5f*h*velocity;
			const float am = measured::OMEGA * measured::OMEGA * (target() - qm)
				- 2.f * measured::DAMPING * measured::OMEGA * vm;
			const float vn = clip(velocity + h*am, -measured::VNEG, measured::VPOS);
			angle += .5f*h*(velocity + vn); velocity = vn;
		}
		return true;
	}
	float angle{0.f}, velocity{0.f}; // degrees, degrees/s, increasing-PWM convention
private:
	static float clip(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
	static float lookup(float pwm, const float *values)
	{
		for (unsigned i = 1; i < measured::COUNT; ++i) {
			if (pwm <= measured::PWM[i]) {
				const float w = (pwm - measured::PWM[i-1]) / (measured::PWM[i] - measured::PWM[i-1]);
				return values[i-1] + w * (values[i] - values[i-1]);
			}
		}
		return values[measured::COUNT-1];
	}
	float _pwm{1450.f}, _held{0.f}, _play{0.f};
};
} // namespace morphing_publisher

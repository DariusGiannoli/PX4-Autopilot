// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include "MeasuredArmCalibration.hpp"
#include <cmath>

namespace morphing_publisher {
// Frozen two-play/deadband mixture and midpoint speed-limited loaded-hover model.
// Starts at initial neutral. Commands retain state; no resets at direction changes.
class MeasuredArmModel {
public:
	MeasuredArmModel() { reset(); }
	void reset(float neutral_pwm = measured::REFERENCE_NEUTRAL_PWM, unsigned arm = 0)
	{
		_arm = arm;
		_trim = neutral_pwm - measured::REFERENCE_NEUTRAL_PWM;
		_pwm = measured::REFERENCE_NEUTRAL_PWM;
		const float initial = (measured::INITIAL_ANGLE_DEG-measured::BIAS) / measured::GAIN;
		const float mid = .5f * (lookup(_pwm, measured::LOWER) + lookup(_pwm, measured::UPPER));
		for (unsigned i = 0; i < 2; ++i) {
			_held[i] = initial; _play[i] = initial-measured::COMPONENT_ELASTIC[i]*mid;
		}
		angle = measured::INITIAL_ANGLE_DEG; velocity = 0.f;
		_clock = 0.; _head = _size = 0; _drive = _last_target = angle;
	}
	bool accept(float wire)
	{
		if (!std::isfinite(wire) || std::fabs(wire) > 1.f) { return false; }
		float pwm = 0.f;
		if (!decode(wire, _arm, pwm)) { return false; }
		if (static_cast<int>(pwm) == static_cast<int>(_pwm)) { return true; }
		const float lo = lookup(pwm, measured::LOWER), hi = lookup(pwm, measured::UPPER);
		for (unsigned i = 0; i < 2; ++i) {
			const float elastic = measured::COMPONENT_ELASTIC[i] * .5f * (lo + hi);
			_play[i] = clip(_play[i], lo-elastic, hi-elastic);
			const float raw = _play[i]+elastic;
			if (std::fabs(raw-_held[i]) > measured::DEADBAND) { _held[i] = raw; }
		}
		_pwm = pwm;
		const float next = target();
		if (std::fabs(next-_last_target) > 1.e-7f) {
			if (_size == QUEUE_CAPACITY) { return false; }
			const unsigned slot = (_head+_size) % QUEUE_CAPACITY;
			_due[slot] = _clock + measured::RESPONSE_DELAY_S;
			_pending[slot] = next; ++_size; _last_target = next;
		}
		return true;
	}
	static __attribute__((noinline, optimize("Os"))) bool decode(float wire, unsigned arm, float &pwm)
	{
		if (arm >= 4 || !std::isfinite(wire) || std::fabs(wire) > 1.f) { return false; }
		const float physical = lroundf(measured::PWM_MIN[arm] + .5f*(wire+1.f)*(measured::PWM_MAX[arm]-measured::PWM_MIN[arm]));
		pwm = measured::REFERENCE_NEUTRAL_PWM + measured::PWM_SIGN[arm]*(physical-measured::NEUTRAL_PWM[arm]);
		return physical >= measured::PWM_MIN[arm] && physical <= measured::PWM_MAX[arm]
			&& pwm >= measured::PWM[0] && pwm <= measured::PWM[measured::COUNT-1];
	}

	static __attribute__((noinline, optimize("Os"))) bool safeWire(float wire, unsigned arm, int direction = 1)
	{
		float pwm = 0.f;
		if (!decode(wire, arm, pwm) || (direction != -1 && direction != 1)) { return false; }
		const float lo = lookup(pwm, measured::LOWER)*measured::GAIN+measured::BIAS;
		const float hi = lookup(pwm, measured::UPPER)*measured::GAIN+measured::BIAS;
		return std::fabs(lo) <= measured::SAFETY_ANGLE_DEG && std::fabs(hi) <= measured::SAFETY_ANGLE_DEG;
	}

	float target() const { return (measured::FIRST_COMPONENT_WEIGHT*_held[0]+(1.f-measured::FIRST_COMPONENT_WEIGHT)*_held[1])*measured::GAIN+measured::BIAS; }
	bool advance(float dt)
	{
		// A longer scheduler gap leaves the estimate unqualified; caller marks stale.
		if (!std::isfinite(dt) || dt < 0.f || dt > .1f) { return false; }
		const double finish = _clock + static_cast<double>(dt);
		while (_clock < finish) {
			while (_size && _due[_head] <= _clock + 1.e-10) {
				_drive = _pending[_head]; _head = (_head+1) % QUEUE_CAPACITY; --_size;
			}
			double until = finish;
			if (_size && _due[_head] < until) { until = _due[_head]; }
			const float h = static_cast<float>((until-_clock) < .0005 ? (until-_clock) : .0005);
			step(h); _clock += static_cast<double>(h);
		}
		return true;
	}

	float angle{0.f}, velocity{0.f}; // degrees, degrees/s, increasing-PWM convention
private:
	void step(float h) {
		const float a = measured::OMEGA*measured::OMEGA*(_drive-angle)-2.f*measured::DAMPING*measured::OMEGA*velocity;
		const float vm = clip(velocity+.5f*h*a, -measured::VNEG, measured::VPOS);
		const float qm = angle+.5f*h*velocity;
		const float am = measured::OMEGA*measured::OMEGA*(_drive-qm)-2.f*measured::DAMPING*measured::OMEGA*vm;
		const float vn = clip(velocity+h*am, -measured::VNEG, measured::VPOS);
		angle += .5f*h*(velocity+vn); velocity = vn;
	}
	static constexpr unsigned QUEUE_CAPACITY = 32;
	double _clock{0.}, _due[QUEUE_CAPACITY]{};
	float _pending[QUEUE_CAPACITY]{}, _drive{0.f}, _last_target{0.f};
	unsigned _head{0}, _size{0};
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
	unsigned _arm{0};
	float _pwm{1500.f}, _held[2]{}, _play[2]{}, _trim{0.f};
};
} // namespace morphing_publisher

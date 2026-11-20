/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Pulses.hpp
 * Hover pulses: a +amp / -amp doublet on one motor at a time, with a per-motor body-rate abort.
 */

#pragma once

#include <math.h>
#include <stdint.h>

class HoverPulses
{
public:
	static constexpr int N = 4;

	void configure(float amp, float slot_duration, float rate_limit)
	{
		_amp = amp;
		_slot = slot_duration;
		_rate_limit = rate_limit;
		reset();
	}

	void reset()
	{
		for (int i = 0; i < N; i++) {
			_aborted[i] = false;
			_entry_recorded[i] = false;
		}
	}

	bool done(float t) const { return t >= _slot * N; }

	/** Bit i set: motor i's pulse was cut by the rate margin. */
	uint8_t abortedMask() const
	{
		uint8_t m = 0;

		for (int i = 0; i < N; i++) { if (_aborted[i]) { m |= (1u << i); } }

		return m;
	}

	/**
	 * Offsets at time t since the start: motor k (actuator_motors order) gets +amp then -amp in slot k.
	 * A slot is cut to 0 once a body rate exceeds its entry rate + (rate_limit - |entry|) / motors left.
	 */
	void step(float t, const float rate[3], float offset[N])
	{
		for (int i = 0; i < N; i++) { offset[i] = 0.f; }

		if (t < 0.f || done(t)) {
			return;
		}

		const int motor = static_cast<int>(t / _slot);

		if (!_entry_recorded[motor]) {
			for (int k = 0; k < 3; k++) { _entry_rate[motor][k] = rate[k]; }

			_entry_recorded[motor] = true;
		}

		const int n_left = N - 1 - motor;
		const float divisor = (n_left > 0) ? static_cast<float>(n_left) : 1.f;

		for (int k = 0; k < 3; k++) {
			const float entry = fabsf(_entry_rate[motor][k]);

			if (fabsf(rate[k]) > entry + fmaxf(_rate_limit - entry, 0.f) / divisor) {
				_aborted[motor] = true;
			}
		}

		if (!_aborted[motor]) {
			offset[motor] = (t - motor * _slot < 0.5f * _slot) ? _amp : -_amp;
		}
	}

private:
	float _amp{0.08f};
	float _slot{0.5f};
	float _rate_limit{20.f};
	bool _aborted[N] {};
	bool _entry_recorded[N] {};
	float _entry_rate[N][3] {};
};

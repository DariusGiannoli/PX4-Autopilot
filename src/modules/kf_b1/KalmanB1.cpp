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

#include "KalmanB1.hpp"

void KalmanB1::setParameters(const Vec &p0_std, const Vec &q_std, const Vec &r_std, float p_cap_factor)
{
	for (int o = 0; o < N; o++) {
		_p0_var(o) = p0_std(o) * p0_std(o);
		_q_var(o) = q_std(o) * q_std(o);
		// r is floored above 0
		const float r = (r_std(o) > 1e-6f) ? r_std(o) : 1e-6f;
		_r_var(o) = r * r;
	}

	_p_cap_factor_sq = p_cap_factor * p_cap_factor;
}

void KalmanB1::reset(const Mat &g0)
{
	_G = g0;

	for (int o = 0; o < N; o++) {
		_P[o].setZero();

		for (int i = 0; i < N; i++) {
			_P[o](i, i) = _p0_var(o);
		}
	}
}

void KalmanB1::reset(const Mat &g0, const Mat p0[N])
{
	_G = g0;

	for (int o = 0; o < N; o++) {
		_P[o] = p0[o];
	}
}

void KalmanB1::step(const Mat &X, const Vec &y, float dt_ratio)
{
	const Mat identity = matrix::eye<float, N>();

	for (int o = 0; o < N; o++) {
		Vec x;

		for (int j = 0; j < N; j++) {
			x(j) = X(o, j);
		}

		// Predict: G unchanged, P inflated by Q (diagonal), unless already above the cap.
		Mat p = _P[o];
		const float q = _q_var(o) * dt_ratio;
		const float cap = _p_cap_factor_sq * _p0_var(o);

		for (int i = 0; i < N; i++) {
			if (p(i, i) < cap) {
				p(i, i) += q;
			}
		}

		// Update with the scalar measurement y_o.
		float y_hat = 0.f;

		for (int j = 0; j < N; j++) {
			y_hat += _G(o, j) * x(j);
		}

		const float e = y(o) - y_hat;
		const Vec px = p * x;
		const float s = x.dot(px) + _r_var(o);

		if (!matrix::Vector2f(e, s).isAllFinite() || !(s > 0.f)) {
			continue;  // keep G_o and P_o
		}
		const Vec k = px / s;

		for (int j = 0; j < N; j++) {
			_G(o, j) += k(j) * e;
		}

		// Joseph form: P = (I - k x^T) p (I - k x^T)^T + r^2 k k^T, outer products written out
		Mat a = identity;
		Mat kk;

		for (int i = 0; i < N; i++) {
			for (int j = 0; j < N; j++) {
				a(i, j) -= k(i) * x(j);
				kk(i, j) = k(i) * k(j);
			}
		}

		_P[o] = a * p * a.transpose() + kk * _r_var(o);
	}
}

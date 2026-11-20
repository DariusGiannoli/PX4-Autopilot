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
 * @file KalmanB1.hpp
 * Per-row Kalman filter on B1 (random-walk parameters, scalar measurement per row, Joseph update),
 * with per-row regressors, dt-scaled Q and a P cap.
 */

#pragma once

#include <matrix/math.hpp>

class KalmanB1
{
public:
	static constexpr int N = 4; ///< number of rows (outputs) and regressor dimension

	using Vec = matrix::Vector<float, N>;
	using Mat = matrix::SquareMatrix<float, N>;

	/**
	 * Set the noise model. All values are standard deviations, one per row.
	 * @param p0_std        prior std of each entry of row o at reset
	 * @param q_std         random-walk std of each entry of row o per nominal step
	 * @param r_std         measurement-noise std of output o (must be > 0)
	 * @param p_cap_factor  stop inflating a variance once it exceeds (p_cap_factor * p0_std)^2
	 */
	void setParameters(const Vec &p0_std, const Vec &q_std, const Vec &r_std, float p_cap_factor);

	/** Reset G to g0 and every row's covariance to its prior. */
	void reset(const Mat &g0);

	/** Reset G to g0 and row o's covariance to p0[o]. */
	void reset(const Mat &g0, const Mat p0[N]);

	/**
	 * One predict + update.
	 * @param X         row o is the regressor (delta u) for output o
	 * @param y         measurements (delta nu), one per row
	 * @param dt_ratio  step length / nominal step length, scales the process noise
	 */
	void step(const Mat &X, const Vec &y, float dt_ratio = 1.f);

	const Mat &G() const { return _G; }
	const Mat &P(int row) const { return _P[row]; }

private:
	Mat _G{};
	Mat _P[N] {};

	Vec _p0_var{};
	Vec _q_var{};
	Vec _r_var{};
	float _p_cap_factor_sq{9.f};
};

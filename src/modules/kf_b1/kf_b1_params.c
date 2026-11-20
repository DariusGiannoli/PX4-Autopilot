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
 * @file kf_b1_params.c
 * kf_b1 parameters. Suffix n = row (0 a_z, 1 alpha_x, 2 alpha_y, 3 alpha_z); Q is per 2 ms step.
 */

/**
 * KF step period
 *
 * Minimum time between KF steps; the input filters run on every gyro sample.
 *
 * @unit us
 * @min 1000
 * @max 20000
 * @group KF B1 estimator
 */
PARAM_DEFINE_INT32(KFB1_DT_US, 2000);

/**
 * Measurement noise std, a_z row
 *
 * @unit m/s^2
 * @min 0.001
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_R0, 0.2f);

/**
 * Measurement noise std, alpha_x row
 *
 * @unit rad/s^2
 * @min 0.001
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_R1, 10.0f);

/**
 * Measurement noise std, alpha_y row
 *
 * @unit rad/s^2
 * @min 0.001
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_R2, 10.0f);

/**
 * Measurement noise std, alpha_z row
 *
 * @unit rad/s^2
 * @min 0.001
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_R3, 2.0f);

/**
 * Random-walk std per 2 ms step, a_z row
 *
 * @unit m/s^2
 * @min 0
 * @decimal 4
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_Q0, 0.005f);

/**
 * Random-walk std per 2 ms step, alpha_x row
 *
 * @unit rad/s^2
 * @min 0
 * @decimal 4
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_Q1, 0.5f);

/**
 * Random-walk std per 2 ms step, alpha_y row
 *
 * @unit rad/s^2
 * @min 0
 * @decimal 4
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_Q2, 0.5f);

/**
 * Random-walk std per 2 ms step, alpha_z row
 *
 * @unit rad/s^2
 * @min 0
 * @decimal 4
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_Q3, 0.05f);

/**
 * Prior std of each entry at reset, a_z row
 *
 * @unit m/s^2
 * @min 0.001
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_P0_0, 0.5f);

/**
 * Prior std of each entry at reset, alpha_x row
 *
 * @unit rad/s^2
 * @min 0.001
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_P0_1, 100.0f);

/**
 * Prior std of each entry at reset, alpha_y row
 *
 * @unit rad/s^2
 * @min 0.001
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_P0_2, 100.0f);

/**
 * Prior std of each entry at reset, alpha_z row
 *
 * @unit rad/s^2
 * @min 0.001
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_P0_3, 10.0f);

/**
 * Motor time constant
 *
 * First-order lag on the motor command before the regression. 0 = none.
 *
 * @unit s
 * @min 0
 * @max 0.2
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_TAU_M, 0.01f);

/**
 * Identification low-pass cutoff
 *
 * 2nd-order low-pass applied identically to nu and u before differencing, in the KF and the pulse fit. 0 = off.
 *
 * @unit Hz
 * @min 0
 * @max 50
 * @decimal 1
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_LPF_HZ, 5.0f);

/**
 * Covariance growth cap
 *
 * Q is no longer added once a std exceeds this factor times KFB1_P0_n.
 *
 * @min 1
 * @max 100
 * @decimal 1
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_PCAP, 3.0f);

/**
 * Throw fit: minimum number of samples
 *
 * Minimum number of samples for the throw fit to be used.
 *
 * @min 20
 * @max 5000
 * @group KF B1 estimator
 */
PARAM_DEFINE_INT32(KFB1_THR_NMIN, 40);

/**
 * Pulse fit: trust at the KF restart
 *
 * After the fit, each B1 entry is taken as known to +- this fraction of its row's own gain.
 *
 * @min 0.01
 * @max 2
 * @decimal 2
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_THR_STD, 0.2f);

/**
 * Start kf_b1 at boot
 *
 * 1: rc.mc_apps starts kf_b1 at boot.
 *
 * @boolean
 * @reboot_required true
 * @group KF B1 estimator
 */
PARAM_DEFINE_INT32(KFB1_EN, 1);

/**
 * Prior B1, row 0 (a_z), column 0 (FL)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_00, 12.0f);

/**
 * Prior B1, row 0 (a_z), column 1 (RL)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_01, 12.0f);

/**
 * Prior B1, row 0 (a_z), column 2 (RR)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_02, 12.0f);

/**
 * Prior B1, row 0 (a_z), column 3 (FR)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_03, 12.0f);

/**
 * Prior B1, row 1 (alpha_x), column 0 (FL)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_10, 350.0f);

/**
 * Prior B1, row 1 (alpha_x), column 1 (RL)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_11, 350.0f);

/**
 * Prior B1, row 1 (alpha_x), column 2 (RR)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_12, -350.0f);

/**
 * Prior B1, row 1 (alpha_x), column 3 (FR)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_13, -350.0f);

/**
 * Prior B1, row 2 (alpha_y), column 0 (FL)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_20, -350.0f);

/**
 * Prior B1, row 2 (alpha_y), column 1 (RL)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_21, 350.0f);

/**
 * Prior B1, row 2 (alpha_y), column 2 (RR)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_22, 350.0f);

/**
 * Prior B1, row 2 (alpha_y), column 3 (FR)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_23, -350.0f);

/**
 * Prior B1, row 3 (alpha_z), column 0 (FL)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_30, 40.0f);

/**
 * Prior B1, row 3 (alpha_z), column 1 (RL)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_31, -40.0f);

/**
 * Prior B1, row 3 (alpha_z), column 2 (RR)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_32, 40.0f);

/**
 * Prior B1, row 3 (alpha_z), column 3 (FR)
 *
 * Used at arming; rows FLU, columns in rotor order FL, RL, RR, FR.
 *
 * @decimal 3
 * @group KF B1 estimator
 */
PARAM_DEFINE_FLOAT(KFB1_G0_33, -40.0f);

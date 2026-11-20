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
 * Pulse amplitude
 *
 * Doublet +amp then -amp added to each motor's command in turn.
 *
 * @min 0.02
 * @max 0.4
 * @decimal 2
 * @group Throw sysid
 */
PARAM_DEFINE_FLOAT(THRSID_AMP, 0.08f);

/**
 * Pulse slot per motor
 *
 * Time per motor (half +amp, half -amp); all pulses take 4 x this.
 *
 * @unit s
 * @min 0.02
 * @max 1
 * @decimal 3
 * @group Throw sysid
 */
PARAM_DEFINE_FLOAT(THRSID_SLOT, 0.5f);

/**
 * Body-rate budget for the per-motor abort
 *
 * A motor's pulse is cut when a rate exceeds entry rate + (limit - |entry|) / motors left.
 *
 * @unit rad/s
 * @min 2
 * @max 35
 * @decimal 1
 * @group Throw sysid
 */
PARAM_DEFINE_FLOAT(THRSID_RATE_LIM, 20.0f);

/**
 * Hard body-rate limit
 *
 * Stops the pulses.
 *
 * @unit rad/s
 * @min 5
 * @max 40
 * @decimal 1
 * @group Throw sysid
 */
PARAM_DEFINE_FLOAT(THRSID_RATE_MAX, 30.0f);

/**
 * Tilt limit
 *
 * Stops the pulses when the tilt from level exceeds this.
 *
 * @unit deg
 * @min 10
 * @max 180
 * @decimal 0
 * @group Throw sysid
 */
PARAM_DEFINE_FLOAT(THRSID_TILT_MAX, 60.0f);

/**
 * Drop limit
 *
 * Stops the pulses after this height loss since their start (needs a local position).
 *
 * @unit m
 * @min 0.1
 * @max 20
 * @decimal 1
 * @group Throw sysid
 */
PARAM_DEFINE_FLOAT(THRSID_MAX_DROP, 1.0f);

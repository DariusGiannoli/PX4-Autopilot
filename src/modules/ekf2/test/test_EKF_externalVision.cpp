/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * Test the external vision functionality
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"


class EkfExternalVisionTest : public ::testing::Test
{
public:

	EkfExternalVisionTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	static constexpr float _tilt_align_time = 7.f;

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		// Init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}
};

TEST_F(EkfExternalVisionTest, checkVisionFusionLogic)
{
	_sensor_simulator.runSeconds(_tilt_align_time); // Let the tilt align
	_ekf_wrapper.enableExternalVisionPositionFusion();
	_sensor_simulator._vio.setPositionFrameToLocalNED();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(2);

	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());

	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	_ekf_wrapper.enableExternalVisionVelocityFusion();
	_sensor_simulator.runSeconds(2);

	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());

	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	_ekf_wrapper.enableExternalVisionHeadingFusion();
	_sensor_simulator.runSeconds(2);

	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());

	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());
}

TEST_F(EkfExternalVisionTest, visionYawNoiseUsesVarianceUnits)
{
	_sensor_simulator.runSeconds(_tilt_align_time);
	_ekf_wrapper.enableExternalVisionHeadingFusion();
	_sensor_simulator._vio.setPositionFrameToLocalFRD();
	_sensor_simulator._vio.setAngularVariance(0.0009f);
	_ekf->getParamHandle()->ev_att_noise = 0.1f;
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(2);

	ASSERT_TRUE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());
	// The configured standard deviation is in radians; aid-source variance is rad^2.
	EXPECT_NEAR(_ekf->aid_src_ev_yaw().observation_variance, 0.01f, 1e-6f);

	// A larger supplied measurement variance must still take precedence.
	_sensor_simulator._vio.setAngularVariance(0.04f);
	_sensor_simulator.runSeconds(1);
	EXPECT_NEAR(_ekf->aid_src_ev_yaw().observation_variance, 0.04f, 1e-6f);
}

TEST_F(EkfExternalVisionTest, visionVelocityReset)
{
	_sensor_simulator.runSeconds(_tilt_align_time);
	ResetLoggingChecker reset_logging_checker(_ekf);
	reset_logging_checker.capturePreResetState();

	const Vector3f simulated_velocity(0.3f, -1.0f, 0.4f);

	_sensor_simulator._vio.setVelocity(simulated_velocity);
	_ekf_wrapper.enableExternalVisionVelocityFusion();
	_sensor_simulator.startExternalVision();
	// Note: test duration needs to allow time for tilt alignment to complete
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.runMicroseconds(2e5);

	// THEN: a reset to Vision velocity should be done
	// Note: velocity will drift after reset due to INAV errors so the tolerance needs to allow for this
	const Vector3f estimated_velocity = _ekf->getVelocity();
	EXPECT_TRUE(isEqual(estimated_velocity, simulated_velocity, 0.01f));

	// AND: the reset in velocity should be saved correctly
	reset_logging_checker.capturePostResetState();
	EXPECT_TRUE(reset_logging_checker.isHorizontalVelocityResetCounterIncreasedBy(1));
	EXPECT_TRUE(reset_logging_checker.isVerticalVelocityResetCounterIncreasedBy(1));
	EXPECT_TRUE(reset_logging_checker.isVelocityDeltaLoggedCorrectly(0.01f));
}

TEST_F(EkfExternalVisionTest, visionVelocityResetWithAlignment)
{
	_sensor_simulator.runSeconds(_tilt_align_time);
	ResetLoggingChecker reset_logging_checker(_ekf);
	reset_logging_checker.capturePreResetState();
	// GIVEN: Drone is pointing north, and we use mag (ROTATE_EV)
	//        Heading of drone in EKF frame is 0°

	// WHEN: Vision frame is rotate +90°. The reported heading is -90°
	Quatf vision_to_ekf(Eulerf(0.0f, 0.0f, math::radians(-90.0f)));
	_sensor_simulator._vio.setOrientation(vision_to_ekf.inversed());

	const Vector3f simulated_velocity_in_vision_frame(0.3f, -1.0f, 0.4f);
	const Vector3f simulated_velocity_in_ekf_frame =
		Dcmf(vision_to_ekf) * simulated_velocity_in_vision_frame;
	_sensor_simulator._vio.setVelocity(simulated_velocity_in_vision_frame);
	_ekf_wrapper.enableExternalVisionVelocityFusion();
	_sensor_simulator.startExternalVision();
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.runMicroseconds(2e5);

	// THEN: a reset to Vision velocity should be done
	const Vector3f estimated_velocity_in_ekf_frame = _ekf->getVelocity();
	EXPECT_TRUE(isEqual(estimated_velocity_in_ekf_frame, simulated_velocity_in_ekf_frame, 0.01f));
	// And: the frame offset should be estimated correctly
	// Quatf estimatedExternalVisionFrameOffset = _ekf->getVisionAlignmentQuaternion();
	// EXPECT_TRUE(matrix::isEqual(vision_to_ekf.canonical(),
	// 			    estimatedExternalVisionFrameOffset.canonical()));

	// AND: the reset in velocity should be saved correctly
	reset_logging_checker.capturePostResetState();
	EXPECT_TRUE(reset_logging_checker.isHorizontalVelocityResetCounterIncreasedBy(1));
	EXPECT_TRUE(reset_logging_checker.isVerticalVelocityResetCounterIncreasedBy(1));
	EXPECT_TRUE(reset_logging_checker.isVelocityDeltaLoggedCorrectly(0.01f));
}

TEST_F(EkfExternalVisionTest, visionHorizontalPositionReset)
{
	_sensor_simulator.runSeconds(_tilt_align_time);
	const Vector3f simulated_position(8.3f, -1.0f, 0.0f);

	_sensor_simulator._vio.setPosition(simulated_position);
	_sensor_simulator._vio.setPositionFrameToLocalNED();
	_ekf_wrapper.enableExternalVisionPositionFusion();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runMicroseconds(2e5);

	// THEN: a reset to Vision velocity should be done
	const Vector3f estimated_position = _ekf->getPosition();
	EXPECT_TRUE(isEqual(estimated_position, simulated_position, 1e-5f));
}

TEST_F(EkfExternalVisionTest, visionHorizontalPositionResetWithAlignment)
{
	_sensor_simulator.runSeconds(_tilt_align_time);
	// GIVEN: Drone is pointing north, and we use mag (ROTATE_EV)
	//        Heading of drone in EKF frame is 0°

	// WHEN: Vision frame is rotate +90°. The reported heading is -90°
	Quatf vision_to_ekf(Eulerf(0.0f, 0.0f, math::radians(-90.0f)));
	_sensor_simulator._vio.setOrientation(vision_to_ekf.inversed());

	const Vector3f simulated_position_in_vision_frame(8.3f, -1.0f, 0.0f);
	const Vector3f simulated_position_in_ekf_frame =
		Dcmf(vision_to_ekf) * simulated_position_in_vision_frame;
	_sensor_simulator._vio.setPosition(simulated_position_in_vision_frame);
	_ekf_wrapper.enableExternalVisionPositionFusion();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runMicroseconds(2e5);

	// THEN: a reset to Vision velocity should be done
	const Vector3f estimated_position_in_ekf_frame = _ekf->getPosition();
	EXPECT_TRUE(isEqual(estimated_position_in_ekf_frame, simulated_position_in_ekf_frame, 1e-2f));
}

TEST_F(EkfExternalVisionTest, visionVarianceCheck)
{
	_sensor_simulator.runSeconds(_tilt_align_time);
	const Vector3f velVar_init = _ekf->getVelocityVariance();
	EXPECT_NEAR(velVar_init(0), velVar_init(1), 0.0001);

	_sensor_simulator._vio.setVelocityVariance(Vector3f{2.0f, 0.01f, 0.01f});
	_ekf_wrapper.enableExternalVisionVelocityFusion();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(4);

	const Vector3f velVar_new = _ekf->getVelocityVariance();
	EXPECT_TRUE(velVar_new(0) > velVar_new(1));
}

TEST_F(EkfExternalVisionTest, visionAlignment)
{
	_sensor_simulator.runSeconds(_tilt_align_time);
	// GIVEN: Drone is pointing north, and we use mag (ROTATE_EV)
	//        Heading of drone in EKF frame is 0°

	// WHEN: Vision frame is rotate +90°. The reported heading is -90°
	Quatf externalVisionFrameOffset(Eulerf(0.0f, 0.0f, math::radians(90.0f)));
	_sensor_simulator._vio.setOrientation(externalVisionFrameOffset.inversed());

	// Simulate high uncertainty on vision x axis which is in this case
	// the y EKF frame axis
	_sensor_simulator._vio.setVelocityVariance(Vector3f{2.0f, 0.01f, 0.01f});
	_ekf_wrapper.enableExternalVisionVelocityFusion();
	_sensor_simulator.startExternalVision();

	const Vector3f velVar_init = _ekf->getVelocityVariance();
	EXPECT_NEAR(velVar_init(0), velVar_init(1), 0.0001);

	_sensor_simulator.runSeconds(4);

	// THEN: velocity uncertainty in y should be bigger
	const Vector3f velVar_new = _ekf->getVelocityVariance();
	EXPECT_TRUE(velVar_new(1) > velVar_new(0));

	// THEN: the frame offset should be estimated correctly
	// Quatf estimatedExternalVisionFrameOffset = _ekf->getVisionAlignmentQuaternion();
	// EXPECT_TRUE(matrix::isEqual(externalVisionFrameOffset.canonical(),
	// 			    estimatedExternalVisionFrameOffset.canonical()));
}

TEST_F(EkfExternalVisionTest, velocityFrameBody)
{
	// GIVEN: Drone is turned 90 degrees
	const Quatf quat_sim(Eulerf(0.0f, 0.0f, math::radians(90.0f)));
	_sensor_simulator.simulateOrientation(quat_sim);
	_sensor_simulator.runSeconds(_tilt_align_time);

	_ekf->set_vehicle_at_rest(false);

	// Without any measurement x and y velocity variance are close
	const Vector3f velVar_init = _ekf->getVelocityVariance();
	EXPECT_NEAR(velVar_init(0), velVar_init(1), 0.0001);

	// WHEN: measurement is given in BODY-FRAME and
	//       x variance is bigger than y variance
	_sensor_simulator._vio.setVelocityFrameToBody();

	const Vector3f vel_cov_body(2.0f, 0.01f, 0.01f);
	const Vector3f vel_body(1.0f, 0.0f, 0.0f);
	_sensor_simulator._vio.setVelocityVariance(vel_cov_body);
	_sensor_simulator._vio.setVelocity(vel_body);
	_ekf_wrapper.enableExternalVisionVelocityFusion();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(4);

	// THEN: As the drone is turned 90 degrees, velocity variance
	//       along local y axis is expected to be bigger
	const Vector3f velVar_new = _ekf->getVelocityVariance();
	EXPECT_NEAR(velVar_new(1) / velVar_new(0), 30.f, 15.f);

	const Vector3f vel_earth_est = _ekf->getVelocity();
	EXPECT_NEAR(vel_earth_est(0), 0.0f, 0.1f);
	EXPECT_NEAR(vel_earth_est(1), 1.0f, 0.1f);
}

TEST_F(EkfExternalVisionTest, velocityFrameLocal)
{
	// GIVEN: Drone is turned 90 degrees
	const Quatf quat_sim(Eulerf(0.0f, 0.0f, math::radians(90.0f)));
	_sensor_simulator.simulateOrientation(quat_sim);
	_sensor_simulator.runSeconds(_tilt_align_time);

	// Without any measurement x and y velocity variance are close
	const Vector3f velVar_init = _ekf->getVelocityVariance();
	EXPECT_NEAR(velVar_init(0), velVar_init(1), 0.0001);

	// WHEN: measurement is given in LOCAL-FRAME and
	//       x variance is bigger than y variance
	_sensor_simulator._vio.setVelocityFrameToLocalNED();

	const Vector3f vel_cov_earth{2.f, 0.01f, 0.01f};
	const Vector3f vel_earth(1.0f, 0.0f, 0.0f);
	_sensor_simulator._vio.setVelocityVariance(vel_cov_earth);
	_sensor_simulator._vio.setVelocity(vel_earth);
	_ekf_wrapper.enableExternalVisionVelocityFusion();
	_sensor_simulator.startExternalVision();
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.runSeconds(4);

	// THEN: Independently on drones heading, velocity variance
	//       along local x axis is expected to be bigger
	const Vector3f velVar_new = _ekf->getVelocityVariance();
	EXPECT_NEAR(velVar_new(0) / velVar_new(1), 30.f, 15.f);

	const Vector3f vel_earth_est = _ekf->getVelocity();
	EXPECT_NEAR(vel_earth_est(0), 1.0f, 0.1f);
	EXPECT_NEAR(vel_earth_est(1), 0.0f, 0.1f);
}

TEST_F(EkfExternalVisionTest, positionFrameLocal)
{
	// GIVEN: Drone is turned 90 degrees
	const Quatf quat_sim(Eulerf(0.0f, 0.0f, math::radians(90.0f)));
	_sensor_simulator.simulateOrientation(quat_sim);
	_sensor_simulator.runSeconds(_tilt_align_time);

	_ekf->set_vehicle_at_rest(false);

	// WHEN: using EV yaw fusion and rotate EV is set
	Quatf vision_to_ekf(Eulerf(0.0f, 0.0f, 0.f));
	_sensor_simulator._vio.setOrientation(vision_to_ekf.inversed());
	_ekf_wrapper.enableExternalVisionHeadingFusion(); // EV_YAW

	// AND WHEN: using EV position aiding
	const Vector3f pos_var_earth(0.01f, 0.01f, 0.01f);
	Vector3f pos_earth(0.0f, 0.0f, 0.0f);
	_sensor_simulator._vio.setPositionVariance(pos_var_earth);
	_sensor_simulator._vio.setPosition(pos_earth);
	_ekf_wrapper.enableExternalVisionPositionFusion(); // EV_POS
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(4);

	// THEN: the fusions should start and the position should not drift
	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());
	Vector3f pos_earth_est = _ekf->getPosition();
	EXPECT_NEAR(pos_earth_est(0), pos_earth(0), 0.01f);
	EXPECT_NEAR(pos_earth_est(1), pos_earth(1), 0.01f);

	// WHEN: the measurement in EV FRD frame changes
	pos_earth = Vector3f(0.3f, 0.0f, 0.0f);
	_sensor_simulator._vio.setPosition(pos_earth);

	_sensor_simulator.runSeconds(12);

	// THEN: the position should converge to the EV position
	// Note that the estimate is now in the EV frame because it is
	// the reference frame
	pos_earth_est = _ekf->getPosition();
	EXPECT_NEAR(pos_earth_est(0), pos_earth(0), 0.1f);
	EXPECT_NEAR(pos_earth_est(1), pos_earth(1), 0.1f);
}

// Append to src/modules/ekf2/test/test_EKF_externalVision.cpp in an isolated
// test worktree. This snippet never accesses hardware or transport sockets.
// It tests the real EKF core, with direct parameter access as existing EKF unit
// tests do. It does NOT test EKF2's uORB parameter_update subscription lifecycle.
TEST(EkfExternalVisionStartupLifecycleTest, delayedFrdUnknownQualityRuntimeEnableAndResume)
{
	auto ekf = std::make_shared<Ekf>();
	auto *params = ekf->getParamHandle();
	params->delay_max_ms = 200.f; // Set before init: determines buffer allocation.
	params->ev_delay_ms = 0.f; // Capture time already supplied; no extra delay.
	params->ev_quality_minimum = 0;
	params->ev_ctrl = 0;
	params->height_sensor_ref = static_cast<int32_t>(HeightSensor::BARO);
	ASSERT_TRUE(ekf->init(0));
	ekf->set_in_air_status(false);
	ekf->set_vehicle_at_rest(true);
	SensorSimulator sim(ekf);
	EkfWrapper wrapper(ekf);
	sim._mag.stop(); // Match failed-run yaw_align=false, no North reference.
	sim.stopExternalVision(); // The lambda below owns the EV stream.
	sim.runSeconds(7.f);
	ASSERT_TRUE(ekf->control_status_flags().tilt_align);
	ASSERT_FALSE(ekf->control_status_flags().yaw_align);

	extVisionSample sample{};
	sample.pos_frame = PositionFrame::LOCAL_FRAME_FRD;
	sample.vel_frame = VelocityFrame::LOCAL_FRAME_FRD;
	sample.pos = Vector3f{-0.367f, 0.716f, 0.156f};
	sample.quat = Quatf{1.f, 0.f, 0.f, 0.f};
	sample.position_var = Vector3f{0.01f, 0.01f, 0.01f};
	sample.orientation_var = Vector3f{0.01f, 0.01f, 0.01f};
	sample.quality = -1; // Unknown quality accepted iff EKF2_EV_QMIN <= 0.
	sample.reset_counter = 0;

	// At rest, fixed state is also correct at the earlier capture time. Feed
	// 100 Hz EV with a real 20 ms capture-to-insertion delay, preserving stamps.
	// Simulator's default IMU rate is 200 Hz; the core filter interval is 10 ms.
	auto run_with_ev = [&](uint32_t duration_us) {
		ASSERT_EQ(duration_us % 10000, 0u);
		for (uint32_t elapsed = 0; elapsed < duration_us; elapsed += 10000) {
			sim.runMicroseconds(10000);
			sample.time_us = sim.getTime() - 20000;
			ekf->setExtVisionData(sample);
		}
	};
	auto expect_ev_active = [&]() {
		EXPECT_TRUE(wrapper.isIntendingExternalVisionPositionFusion());
		EXPECT_TRUE(wrapper.isIntendingExternalVisionHeightFusion());
		EXPECT_TRUE(wrapper.isIntendingExternalVisionHeadingFusion());
		EXPECT_FALSE(wrapper.isIntendingExternalVisionVelocityFusion());
		EXPECT_GT(ekf->aid_src_ev_pos().time_last_fuse, 0u);
		EXPECT_GT(ekf->aid_src_ev_hgt().time_last_fuse, 0u);
		EXPECT_GT(ekf->aid_src_ev_yaw().time_last_fuse, 0u);
	};

	// Pose traffic can precede runtime enable, as it did in timestamp probing.
	run_with_ev(20000000);
	EXPECT_FALSE(wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_FALSE(wrapper.isIntendingExternalVisionHeightFusion());
	EXPECT_FALSE(wrapper.isIntendingExternalVisionHeadingFusion());
	params->height_sensor_ref = static_cast<int32_t>(HeightSensor::EV);
	params->ev_ctrl = static_cast<int32_t>(EvCtrl::HPOS)
			  | static_cast<int32_t>(EvCtrl::VPOS)
			  | static_cast<int32_t>(EvCtrl::YAW);
	ekf->updateParameters();
	run_with_ev(2000000);
	expect_ev_active();

	// A multi-second sender handover must not permanently prevent restart.
	sim.runSeconds(3.f);
	EXPECT_FALSE(wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_FALSE(wrapper.isIntendingExternalVisionHeightFusion());
	EXPECT_FALSE(wrapper.isIntendingExternalVisionHeadingFusion());
	run_with_ev(2000000);
	expect_ev_active();

	// Also exercise a second runtime 0 -> 11 transition with poses continuing.
	params->ev_ctrl = 0;
	params->height_sensor_ref = static_cast<int32_t>(HeightSensor::BARO);
	ekf->updateParameters();
	run_with_ev(2000000);
	EXPECT_FALSE(wrapper.isIntendingExternalVisionHeadingFusion());
	params->ev_ctrl = 11;
	params->height_sensor_ref = static_cast<int32_t>(HeightSensor::EV);
	ekf->updateParameters();
	run_with_ev(2000000);
	expect_ev_active();
}

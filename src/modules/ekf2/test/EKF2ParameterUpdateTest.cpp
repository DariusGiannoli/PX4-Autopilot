// SPDX-License-Identifier: BSD-3-Clause
// Host-only module test: real EKF2 Run, uORB, parameter storage and wrappers.
// Does not start MAVLink, aircraft I/O, the selector or work-queue threads.
#define MODULE_NAME "ekf2_parameter_test"
#include <gtest/gtest.h>
#include "EKF2.hpp"
#include <memory>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_magnetometer.h>

// The backup timeout cannot schedule work in this deterministic harness.
// IMU callbacks still register through the production code. No parameter,
// subscription clock, sensor conversion or estimator method is stubbed.
extern "C" void __wrap_hrt_call_after(struct hrt_call *, hrt_abstime, hrt_callout, void *) {}

namespace
{
bool low_boot_clock = false;
hrt_abstime low_boot_time_us = 0;
}

extern "C" hrt_abstime __real_hrt_absolute_time();
extern "C" hrt_abstime __wrap_hrt_absolute_time()
{
	return low_boot_clock ? low_boot_time_us : __real_hrt_absolute_time();
}

class EKF2ParameterUpdateTestPeer
{
public:
	static void run(EKF2 &module) { module.Run(); }
	static const parameters &params(const EKF2 &module) { return *module._params; }
	static const Ekf &ekf(const EKF2 &module) { return module._ekf; }
	static bool callbackRegistered(const EKF2 &module) { return module._callback_registered; }
};

class EKF2ParameterUpdateTest : public ::testing::Test
{
protected:
	virtual bool startBeforeFirstSecond() const { return false; }

	void SetUp() override
	{
		if (startBeforeFirstSecond()) {
			low_boot_clock = true;
			_sample_us = low_boot_time_us = 100'000;
		}

		param_control_autosave(false);
		setInt("EKF2_EV_CTRL", 0);
		setInt("EKF2_HGT_REF", 1);
		setInt("EKF2_EV_QMIN", 0);
		setInt("EKF2_LOG_VERBOSE", 1);
		setInt("EKF2_MAG_TYPE", 5);
		setInt("EKF2_GPS_CTRL", 0);
		setInt("EKF2_PREDICT_US", 10000);
		setFloat("EKF2_DELAY_MAX", 200.f);
		setFloat("EKF2_EV_DELAY", 0.f);
		setFloat("EKF2_EVP_NOISE", .1f);
		setFloat("EKF2_EV_POS_X", 0.f);
		setFloat("EKF2_EV_POS_Y", 0.f);
		setFloat("EKF2_EV_POS_Z", 0.f);
		ASSERT_TRUE(_imu0.advertise());
		ASSERT_TRUE(_imu1.advertise());
		ASSERT_EQ(_imu0.get_instance(), 0);
		ASSERT_EQ(_imu1.get_instance(), 1);
		vehicle_magnetometer_s mag{};
		mag.timestamp_sample = mag.timestamp = _sample_us;
		ASSERT_TRUE(_mag.publish(mag));
		_first = std::make_unique<EKF2>(true, px4::wq_configurations::INS0, false);
		_second = std::make_unique<EKF2>(true, px4::wq_configurations::INS1, false);
		ASSERT_TRUE(_first->multi_init(0, 0));
		ASSERT_TRUE(_second->multi_init(1, 0));
		runSamples(8'000'000);
		for (EKF2 *module : {_first.get(), _second.get()}) {
			ASSERT_TRUE(EKF2ParameterUpdateTestPeer::callbackRegistered(*module));
			ASSERT_TRUE(EKF2ParameterUpdateTestPeer::ekf(*module).control_status_flags().tilt_align);
			ASSERT_EQ(EKF2ParameterUpdateTestPeer::params(*module).ev_ctrl, 0);
			ASSERT_EQ(EKF2ParameterUpdateTestPeer::params(*module).ev_quality_minimum, 0);
		}
		// Preserve the real 1 Hz subscription interval; settle initial notices.
		spinWall(1'100'000);
	}

	void TearDown() override
	{
		_first.reset();
		_second.reset();
		low_boot_clock = false;
	}

	static void setInt(const char *name, int32_t value)
	{
		ASSERT_NE(param_find(name), PARAM_INVALID) << name;
		ASSERT_EQ(param_set(param_find(name), &value), 0) << name;
	}

	static void setFloat(const char *name, float value)
	{
		ASSERT_NE(param_find(name), PARAM_INVALID) << name;
		ASSERT_EQ(param_set(param_find(name), &value), 0) << name;
	}

	void runSamples(uint32_t duration_us, bool pose = true)
	{
		const uint64_t end = _sample_us + duration_us;
		while (_sample_us < end) {
			_sample_us += 2500;
			if (low_boot_clock) { low_boot_time_us = _sample_us; }
			++_ticks;
			if (_ticks % 20 == 0) {
				vehicle_status_s status{};
				status.timestamp = _sample_us;
				status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
				status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
				_status.publish(status);
				vehicle_land_detected_s land{};
				land.timestamp = _sample_us;
				land.at_rest = land.landed = land.ground_contact = true;
				_land.publish(land);
				vehicle_air_data_s air{};
				air.timestamp_sample = air.timestamp = _sample_us;
				air.baro_device_id = 123;
				air.baro_alt_meter = 0.f;
				air.rho = 1.225f;
				_air.publish(air);
			}
			if (pose && _ticks % 4 == 0) {
				vehicle_odometry_s odom{};
				odom.timestamp = _sample_us;
				odom.timestamp_sample = _sample_us - 20000;
				odom.pose_frame = vehicle_odometry_s::POSE_FRAME_FRD;
				odom.position[0] = -.367f;
				odom.position[1] = .716f;
				odom.position[2] = .156f;
				odom.q[0] = 1.f;
				odom.quality = -1;
				for (int i = 0; i < 3; ++i) {
					odom.position_variance[i] = .01f;
					odom.orientation_variance[i] = .0009f;
					odom.velocity[i] = odom.velocity_variance[i] = NAN;
				}
				_pose.publish(odom);
			}
			vehicle_imu_s imu{};
			imu.timestamp_sample = imu.timestamp = _sample_us;
			imu.delta_angle_dt = imu.delta_velocity_dt = 2500;
			imu.delta_velocity[2] = -CONSTANTS_ONE_G * .0025f;
			imu.accel_device_id = 1;
			imu.gyro_device_id = 11;
			_imu0.publish(imu);
			imu.accel_device_id = 2;
			imu.gyro_device_id = 12;
			_imu1.publish(imu);
			EKF2ParameterUpdateTestPeer::run(*_first);
			EKF2ParameterUpdateTestPeer::run(*_second);
		}
	}

	void spinWall(uint64_t duration_us)
	{
		const uint64_t end = hrt_absolute_time() + duration_us;
		do {
			runSamples(20'000);
			px4_usleep(1000);
		} while (hrt_absolute_time() < end);
	}

	void expectControl(int32_t value)
	{
		for (const EKF2 *module : {_first.get(), _second.get()}) {
			EXPECT_EQ(EKF2ParameterUpdateTestPeer::params(*module).ev_ctrl, value);
			const auto health = publishedHealth(*module);
			EXPECT_EQ(health.effective_ev_ctrl, value);
			EXPECT_EQ(health.effective_ev_qmin, EKF2ParameterUpdateTestPeer::params(*module).ev_quality_minimum);
			EXPECT_EQ(health.effective_hgt_ref, EKF2ParameterUpdateTestPeer::params(*module).height_sensor_ref);
			EXPECT_EQ(health.ev_startup_ev_ctrl, health.effective_ev_ctrl);
			EXPECT_EQ(health.ev_startup_ev_qmin, health.effective_ev_qmin);
			EXPECT_EQ(health.ev_startup_hgt_ref, health.effective_hgt_ref);
			EXPECT_GT(health.ev_startup_checked_at_us, 0u);
			EXPECT_LE(health.ev_startup_checked_at_us, health.timestamp_sample);
			EXPECT_EQ(health.ev_startup_failed_checks, 0);
		}
	}

	static morpheus_estimator_health_s publishedHealth(const EKF2 &module)
	{
		uORB::Subscription sub{ORB_ID(morpheus_estimator_health), uint8_t(module.instance())};
		morpheus_estimator_health_s health{};
		EXPECT_TRUE(sub.copy(&health));
		return health;
	}

	void expectFusion(bool active)
	{
		for (const EKF2 *module : {_first.get(), _second.get()}) {
			const auto &ekf = EKF2ParameterUpdateTestPeer::ekf(*module);
			EXPECT_EQ(ekf.control_status_flags().ev_pos, active);
			EXPECT_EQ(ekf.control_status_flags().ev_hgt, active);
			EXPECT_EQ(ekf.control_status_flags().ev_yaw, active);
			if (active) {
				EXPECT_GT(ekf.aid_src_ev_pos().time_last_fuse, 0u);
				EXPECT_GT(ekf.aid_src_ev_hgt().time_last_fuse, 0u);
				EXPECT_GT(ekf.aid_src_ev_yaw().time_last_fuse, 0u);
			}
		}
	}

	uint64_t _sample_us{1'000'000};
	unsigned _ticks{0};
	uORB::PublicationMulti<vehicle_imu_s> _imu0{ORB_ID(vehicle_imu)}, _imu1{ORB_ID(vehicle_imu)};
	uORB::Publication<vehicle_magnetometer_s> _mag{ORB_ID(vehicle_magnetometer)};
	uORB::Publication<vehicle_status_s> _status{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_land_detected_s> _land{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_air_data_s> _air{ORB_ID(vehicle_air_data)};
	uORB::Publication<vehicle_odometry_s> _pose{ORB_ID(vehicle_visual_odometry)};
	std::unique_ptr<EKF2> _first, _second;
};

class EKF2EarlyBootParameterUpdateTest : public EKF2ParameterUpdateTest
{
protected:
	bool startBeforeFirstSecond() const override { return true; }
};

TEST_F(EKF2EarlyBootParameterUpdateTest, NotifyingWritesReachBothInstancesAfterEarlyInitialization)
{
	expectFusion(false);
	setInt("EKF2_HGT_REF", 3);
	setInt("EKF2_EV_CTRL", 11);
	spinWall(1'200'000);
	expectControl(11);
	for (const EKF2 *module : {_first.get(), _second.get()}) {
		EXPECT_EQ(EKF2ParameterUpdateTestPeer::params(*module).height_sensor_ref, 3);
	}
	expectFusion(true);
	setInt("EKF2_EV_CTRL", 0);
	setInt("EKF2_HGT_REF", 1);
	spinWall(1'200'000);
	expectControl(0);
	expectFusion(false);
}

TEST_F(EKF2ParameterUpdateTest, NotifyingWritesEnableBothRunningInstances)
{
	expectFusion(false);
	setInt("EKF2_HGT_REF", 3);
	spinWall(40'000);
	setInt("EKF2_EV_CTRL", 11);
	spinWall(1'200'000);
	expectControl(11);
	expectFusion(true);

	setInt("EKF2_EV_CTRL", 0);
	setInt("EKF2_HGT_REF", 1);
	spinWall(1'200'000);
	expectControl(0);
	expectFusion(false);
	runSamples(3'000'000, false);
	setInt("EKF2_HGT_REF", 3);
	setInt("EKF2_EV_CTRL", 11);
	spinWall(1'200'000);
	expectControl(11);
	expectFusion(true);
}

TEST_F(EKF2ParameterUpdateTest, MissingNoticeAndSameValueWriteLeaveRuntimeStale)
{
	setInt("EKF2_HGT_REF", 3);
	spinWall(1'200'000);
	expectControl(0);
	const int32_t enabled = 11;
	ASSERT_EQ(param_set_no_notification(param_find("EKF2_EV_CTRL"), &enabled), 0);
	int32_t stored = -1;
	ASSERT_EQ(param_get(param_find("EKF2_EV_CTRL"), &stored), 0);
	ASSERT_EQ(stored, 11);
	spinWall(1'200'000);
	expectControl(0);
	expectFusion(false);
	setInt("EKF2_EV_CTRL", 11); // Same storage value intentionally emits no notice.
	spinWall(1'200'000);
	expectControl(0);
	expectFusion(false);
	setFloat("EKF2_EVP_NOISE", .11f); // A genuinely changed notifying write reloads all fields.
	spinWall(1'200'000);
	expectControl(11);
	expectFusion(true);
}

TEST_F(EKF2ParameterUpdateTest, DiagnosticsExposeQualityGateWithoutChangingFusion)
{
	setInt("EKF2_HGT_REF", 3);
	setInt("EKF2_EV_CTRL", 11);
	setInt("EKF2_EV_QMIN", 1);
	spinWall(1'200'000);
	expectFusion(false);
	for (const EKF2 *module : {_first.get(), _second.get()}) {
		const auto health = publishedHealth(*module);
		EXPECT_EQ(health.effective_ev_ctrl, 11);
		EXPECT_EQ(health.effective_ev_qmin, 1);
		EXPECT_EQ(health.ev_startup_ev_ctrl, 11);
		EXPECT_EQ(health.ev_startup_ev_qmin, 1);
		EXPECT_GT(health.ev_startup_checked_at_us, 0u);
		// All three quality checks must fail for unknown-quality (-1) poses.
		EXPECT_EQ(health.ev_startup_failed_checks & 13, 13);
	}
	setInt("EKF2_EV_QMIN", 0);
	spinWall(1'200'000);
	expectControl(11);
	expectFusion(true);
}

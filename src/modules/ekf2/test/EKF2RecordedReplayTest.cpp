// SPDX-License-Identifier: BSD-3-Clause
// Offline-only EKF2/uORB replay. No work queues, MAVLink or aircraft I/O.
#define MODULE_NAME "ekf2_recorded_replay"
#include <gtest/gtest.h>
#include "EKF2.hpp"
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

// All users of the monotonic clock (including parameter subscriptions) see
// the explicit replay delivery clock. Only backup scheduling is suppressed.
static uint64_t replay_clock_us = 1'000'000;
extern "C" hrt_abstime __wrap_hrt_absolute_time() { return replay_clock_us; }
extern "C" void __wrap_hrt_call_after(struct hrt_call *, hrt_abstime, hrt_callout, void *) {}

class EKF2ParameterUpdateTestPeer
{
public:
	static void run(EKF2 &module) { module.Run(); }
	static const Ekf &ekf(const EKF2 &module) { return module._ekf; }
};

static std::vector<std::string> split(const std::string &line)
{
	std::vector<std::string> values;
	std::stringstream stream(line);
	std::string value;
	while (std::getline(stream, value, ',')) { values.push_back(value); }
	return values;
}

TEST(EKF2RecordedReplay, ReplayFile)
{
	const char *input_path = std::getenv("MORPHEUS_REPLAY_INPUT");
	const char *output_path = std::getenv("MORPHEUS_REPLAY_OUTPUT");
	if (!input_path || !output_path) { GTEST_SKIP() << "Explicit offline input/output paths required"; }
	std::ifstream input(input_path);
	std::ofstream output(output_path);
	ASSERT_TRUE(input.good());
	ASSERT_TRUE(output.good());
	param_control_autosave(false);
	uORB::Publication<sensor_combined_s> imu_pub{ORB_ID(sensor_combined)};
	uORB::Publication<vehicle_odometry_s> vision_pub{ORB_ID(vehicle_visual_odometry)};
	uORB::Publication<vehicle_air_data_s> baro_pub{ORB_ID(vehicle_air_data)};
	uORB::Publication<vehicle_land_detected_s> land_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_status_s> status_pub{ORB_ID(vehicle_status)};
	uORB::Subscription health_sub{ORB_ID(morpheus_estimator_health)};
	uORB::Subscription odom_sub{ORB_ID(vehicle_odometry)};
	std::unique_ptr<EKF2> module;
	uint64_t next_tick = 0;
	size_t imu_count = 0, line_number = 0;
	output << std::setprecision(10);
	output << "time_us,sample_us,health_us,horizon_us,control_flags,fault_flags,innovation_flags,"
	       "pos_fuse_us,hgt_fuse_us,yaw_fuse_us,heading_variance,solution_flags,reset_counter,"
	       "px,py,pz,vx,vy,vz,qw,qx,qy,qz,pcx,pcy,pcz,vcx,vcy,vcz,acx,acy,acz,"
	       "gyro_bx,gyro_by,gyro_bz,accel_bx,accel_by,accel_bz,"
	       "pos_innov_x,pos_innov_y,pos_ratio_x,pos_ratio_y,hgt_innov,yaw_innov,yaw_observation_variance,"
	       "effective_ev_ctrl,effective_ev_qmin,startup_mask,startup_checked_us,"
	       "heading_good,ev_pose_frame,pose_frame,velocity_frame,rollspeed,pitchspeed,yawspeed\n";
	std::string line;
	ASSERT_TRUE(bool(std::getline(input, line)));
	ASSERT_EQ(line, "#MORPHEUS_EKF_REPLAY_V1");
	++line_number;
	while (std::getline(input, line)) {
		++line_number;
		if (line.empty() || line[0] == '#') { continue; }
		const auto fields = split(line);
		ASSERT_GE(fields.size(), 3u) << line_number;
		const auto number = [&](size_t i) { return std::stod(fields.at(i)); };
		const auto integer = [&](size_t i) { return std::stoull(fields.at(i)); };
		const std::string &kind = fields[0];
		const uint64_t delivery = integer(1);
		if (delivery == 0) {
			ASSERT_EQ(kind, "P");
			ASSERT_FALSE(bool(module));
		}
		// Zero-time parameter rows configure the module before construction.
		if (delivery != 0) {
			ASSERT_GE(delivery, replay_clock_us) << line_number;
			replay_clock_us = delivery;
		}
		if (kind == "P") {
			ASSERT_EQ(fields.size(), 4u) << line_number;
			const param_t handle = param_find(fields[2].c_str());
			ASSERT_NE(handle, PARAM_INVALID) << fields[2];
			if (param_type(handle) == PARAM_TYPE_INT32) {
				const int32_t value = std::stoi(fields[3]);
				ASSERT_EQ(param_set(handle, &value), 0);
			} else {
				const float value = std::stof(fields[3]);
				ASSERT_EQ(param_set(handle, &value), 0);
			}
			continue;
		}
		ASSERT_GT(delivery, 0u);
		if (!module) {
			// Single-instance sensor_combined path. The saved individual IMUs
			// are decimated: this cannot reproduce the two-IMU hardware process.
			module = std::make_unique<EKF2>(false, px4::wq_configurations::INS0, false);
			next_tick = delivery;
		}
		if (kind == "I") {
			ASSERT_EQ(fields.size(), 13u) << line_number;
			sensor_combined_s imu{};
			imu.timestamp = delivery;
			imu.gyro_integral_dt = integer(2);
			imu.accelerometer_integral_dt = integer(3);
			ASSERT_GT(imu.gyro_integral_dt, 0u);
			ASSERT_GT(imu.accelerometer_integral_dt, 0u);
			for (int i = 0; i < 3; ++i) {
				imu.gyro_rad[i] = number(4 + i);
				imu.accelerometer_m_s2[i] = number(7 + i);
			}
			imu.accelerometer_clipping = integer(10);
			imu.accel_calibration_count = integer(11);
			imu.gyro_calibration_count = integer(12);
			ASSERT_TRUE(imu_pub.publish(imu));
			EKF2ParameterUpdateTestPeer::run(*module);
			++imu_count;
			if (delivery >= next_tick) {
				next_tick += ((delivery - next_tick) / 10000 + 1) * 10000;
				morpheus_estimator_health_s health{};
				vehicle_odometry_s odom{};
				health_sub.copy(&health);
				odom_sub.copy(&odom);
				const auto &ekf = EKF2ParameterUpdateTestPeer::ekf(*module);
				output << delivery << ',' << odom.timestamp_sample << ',' << health.timestamp << ','
				       << health.fusion_horizon_us << ',' << health.control_flags << ',' << health.fault_flags << ','
				       << health.innovation_flags << ',' << health.ev_pos_last_fuse << ',' << health.ev_hgt_last_fuse << ','
				       << health.ev_yaw_last_fuse << ',' << health.heading_variance << ',' << health.solution_flags << ','
				       << int(odom.reset_counter);
				for (float v : odom.position) { output << ',' << v; }
				for (float v : odom.velocity) { output << ',' << v; }
				for (float v : odom.q) { output << ',' << v; }
				for (float v : odom.position_variance) { output << ',' << v; }
				for (float v : odom.velocity_variance) { output << ',' << v; }
				for (float v : odom.orientation_variance) { output << ',' << v; }
				for (int i = 0; i < 3; ++i) { output << ',' << ekf.getGyroBias()(i); }
				for (int i = 0; i < 3; ++i) { output << ',' << ekf.getAccelBias()(i); }
				output << ',' << ekf.aid_src_ev_pos().innovation[0] << ',' << ekf.aid_src_ev_pos().innovation[1]
				       << ',' << ekf.aid_src_ev_pos().test_ratio[0] << ',' << ekf.aid_src_ev_pos().test_ratio[1]
				       << ',' << ekf.aid_src_ev_hgt().innovation << ',' << ekf.aid_src_ev_yaw().innovation
				       << ',' << ekf.aid_src_ev_yaw().observation_variance << ',' << health.effective_ev_ctrl
				       << ',' << health.effective_ev_qmin << ',' << int(health.ev_startup_failed_checks)
				       << ',' << health.ev_startup_checked_at_us << ',' << health.heading_good_for_control
				       << ',' << int(health.ev_pose_frame) << ',' << int(odom.pose_frame) << ',' << int(odom.velocity_frame);
				for (float v : odom.angular_velocity) { output << ',' << v; }
				output << '\n';
			}
		} else if (kind == "V") {
			ASSERT_EQ(fields.size(), 23u) << line_number;
			vehicle_odometry_s vision{};
			vision.timestamp = delivery;
			vision.timestamp_sample = integer(2);
			vision.pose_frame = integer(3);
			vision.quality = std::stoi(fields[4]);
			vision.reset_counter = integer(5);
			for (int i = 0; i < 4; ++i) { vision.q[i] = number(6 + i); }
			for (int i = 0; i < 3; ++i) {
				vision.position[i] = number(10 + i);
				vision.position_variance[i] = number(13 + i);
				vision.orientation_variance[i] = number(16 + i);
				vision.velocity[i] = NAN;
				vision.velocity_variance[i] = NAN;
			}
			// Reserved fields assert that velocity aiding was not silently lost.
			for (int i = 19; i < 23; ++i) { ASSERT_EQ(number(i), 0.); }
			ASSERT_TRUE(vision_pub.publish(vision));
		} else if (kind == "B") {
			ASSERT_EQ(fields.size(), 7u) << line_number;
			vehicle_air_data_s baro{};
			baro.timestamp = delivery;
			baro.timestamp_sample = integer(2);
			baro.baro_alt_meter = number(3);
			baro.rho = number(4);
			baro.baro_device_id = integer(5);
			baro.calibration_count = integer(6);
			ASSERT_TRUE(baro_pub.publish(baro));
		} else if (kind == "L") {
			ASSERT_EQ(fields.size(), 5u) << line_number;
			vehicle_land_detected_s land{};
			land.timestamp = delivery;
			land.at_rest = integer(2);
			land.landed = integer(3);
			land.in_ground_effect = integer(4);
			ASSERT_TRUE(land_pub.publish(land));
		} else if (kind == "S") {
			ASSERT_EQ(fields.size(), 4u) << line_number;
			vehicle_status_s status{};
			status.timestamp = delivery;
			status.arming_state = integer(2);
			status.vehicle_type = integer(3);
			ASSERT_NE(int(status.arming_state), int(vehicle_status_s::ARMING_STATE_ARMED));
			ASSERT_TRUE(status_pub.publish(status));
		} else {
			FAIL() << "Unknown replay kind " << kind << " at " << line_number;
		}
	}
	ASSERT_GT(imu_count, 1000u);
	output.flush();
	ASSERT_TRUE(output.good());
}

#pragma once

// Morpheus protocol v1: little-endian 86-byte payload inside V2_EXTENSION,
// message_type 42001, magic MHE1. Explicit encoding avoids struct padding.
#include <uORB/topics/morpheus_estimator_health.h>
#include <uORB/topics/estimator_selector_status.h>

class MavlinkStreamMorpheusEstimatorHealth : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *m) { return new MavlinkStreamMorpheusEstimatorHealth(m); }
	static constexpr const char *get_name_static() { return "MORPHEUS_ESTIMATOR_HEALTH"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_V2_EXTENSION; }
	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }
	unsigned get_size() override { return _health.advertised() ? 86 + 5 + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0; }

private:
	explicit MavlinkStreamMorpheusEstimatorHealth(Mavlink *m) : MavlinkStream(m) {}
	uORB::Subscription _selector{ORB_ID(estimator_selector_status)};
	uORB::SubscriptionMultiArray<morpheus_estimator_health_s> _health{ORB_ID::morpheus_estimator_health};
	uint64_t _last_timestamp{0};
	uint16_t _sequence{0};

	bool send() override
	{
		estimator_selector_status_s selector{};
		const uint64_t now = hrt_absolute_time();
		if (!_selector.copy(&selector) || !selector.timestamp || now < selector.timestamp
		    || now - selector.timestamp > 2500000) { return false; }

		for (auto &sub : _health) {
			morpheus_estimator_health_s h{};
			if (!sub.copy(&h) || h.estimator_instance != selector.primary_instance
			    || h.timestamp <= _last_timestamp || now < h.timestamp || now - h.timestamp > 100000) { continue; }

			mavlink_v2_extension_t msg{};
			msg.message_type = 42001;
			char *p = reinterpret_cast<char *>(msg.payload);
			memcpy(p, "MHE1", 4);
			_mav_put_uint8_t(p, 4, 1);
			_mav_put_uint8_t(p, 5, h.estimator_instance);
			_mav_put_uint16_t(p, 6, _sequence++);
			_mav_put_uint64_t(p, 8, h.timestamp);
			_mav_put_uint64_t(p, 16, h.timestamp_sample);
			_mav_put_uint64_t(p, 24, h.fusion_horizon_us);
			_mav_put_uint64_t(p, 32, h.ev_pos_last_fuse);
			_mav_put_uint64_t(p, 40, h.ev_hgt_last_fuse);
			_mav_put_uint64_t(p, 48, h.ev_yaw_last_fuse);
			_mav_put_uint32_t(p, 56, h.control_flags);
			_mav_put_uint32_t(p, 60, h.fault_flags);
			_mav_put_uint32_t(p, 64, h.innovation_flags);
			_mav_put_uint32_t(p, 68, selector.instance_changed_count);
			_mav_put_float(p, 72, h.heading_variance);
			_mav_put_uint16_t(p, 76, h.solution_flags);
			_mav_put_uint8_t(p, 78, h.heading_good_for_control);
			_mav_put_uint8_t(p, 79, h.ev_pose_frame);
			_mav_put_uint8_t(p, 80, h.reset_counter);
			_mav_put_uint8_t(p, 81, h.xy_reset_counter);
			_mav_put_uint8_t(p, 82, h.z_reset_counter);
			_mav_put_uint8_t(p, 83, h.vxy_reset_counter);
			_mav_put_uint8_t(p, 84, h.vz_reset_counter);
			_mav_put_uint8_t(p, 85, h.heading_reset_counter);
			mavlink_msg_v2_extension_send_struct(_mavlink->get_channel(), &msg);
			_last_timestamp = h.timestamp;
			return true;
		}
		return false;
	}
};

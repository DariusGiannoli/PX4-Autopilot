/* SPDX-License-Identifier: BSD-3-Clause */
#include "BoundedVisionClock.hpp"

uint64_t BoundedVisionClock::drift(uint64_t age_us)
{
	// ceil(age_us*100/1e6), without an overflowing product. Share the wide
	// division across callers to preserve the target's existing flash region.
	static_assert(DriftPpm == 100, "Update exact drift arithmetic when changing the bound");
	return age_us / 10000 + (age_us % 10000 != 0);
}

bool BoundedVisionClock::update(uint64_t now_us, int64_t remote_ns, int64_t originated_ns)
{
	const Status before = status(now_us);

	if (before.fault != Fault::None || remote_ns <= 0 || originated_ns <= 0) { return false; }

	const uint64_t originated_us = static_cast<uint64_t>(originated_ns / 1000);
	const int64_t remote_us = remote_ns / 1000;

	// Reject impossible/future echoes, duplicate replies and reordered older
	// exchanges. These checks do not authenticate a MAVLink peer.
	if (originated_us == 0 || originated_us > now_us || now_us - originated_us > MaximumRttUs
	    || originated_us <= _last_originated_us) { return false; }

	// Overlapping exchanges can be stamped out of order by a healthy remote
	// clock when paths reorder. Only nonoverlap proves physical event order.
	if (_last_remote_ns && originated_us >= _last_received_us && remote_ns <= _last_remote_ns) {
		_fault = Fault::RemoteTimeReversed;
		return false;
	}

	const int64_t margin = static_cast<int64_t>(drift(now_us - originated_us) + 1);
	int64_t low = static_cast<int64_t>(originated_us) - remote_us;
	int64_t high = static_cast<int64_t>(now_us) - remote_us;

	if (low < INT64_MIN + margin || high > INT64_MAX - margin) {
		_fault = Fault::Arithmetic;
		return false;
	}

	low -= margin;
	high += margin;

	// Even after ordinary expiry, reject a new epoch that contradicts the
	// last known interval plus documented drift. This anchor never grants
	// readiness or supplies a converted timestamp during a gap.
	if (_anchor_valid) {
		const int64_t expansion = static_cast<int64_t>(drift(now_us - _anchor_at_us));

		if (_anchor_low < INT64_MIN + expansion || _anchor_high > INT64_MAX - expansion) {
			_fault = Fault::Arithmetic;
			return false;
		}

		if (low > _anchor_high + expansion || high < _anchor_low - expansion) {
			_fault = Fault::InconsistentBounds;
			return false;
		}
	}
	_samples[_next] = {now_us, low, high};
	_next = (_next + 1) % Capacity;
	_count = _count < Capacity ? _count + 1 : Capacity;
	_last_originated_us = originated_us;
	_last_received_us = now_us;
	_last_remote_ns = remote_ns;
	return status(now_us).fault == Fault::None;
}

BoundedVisionClock::Status BoundedVisionClock::status(uint64_t now_us)
{
	Status result{};
	result.generation = _generation;

	if (now_us == 0 || now_us > static_cast<uint64_t>(INT64_MAX)
	    || (_last_now_us && now_us < _last_now_us)) {
		_fault = Fault::LocalTimeReversed;
	}

	_last_now_us = now_us;
	result.fault = _fault;

	if (_fault != Fault::None) { return result; }

	int64_t low = INT64_MIN;
	int64_t high = INT64_MAX;
	uint64_t newest = 0;

	for (unsigned i = 0; i < _count; ++i) {
		const Sample &sample = _samples[i];

		if (sample.at_us > now_us || now_us - sample.at_us > WindowUs) { continue; }

		const int64_t margin = static_cast<int64_t>(drift(now_us - sample.at_us));

		if (sample.low < INT64_MIN + margin || sample.high > INT64_MAX - margin) {
			_fault = Fault::Arithmetic;
			result.fault = _fault;
			return result;
		}

		const int64_t expanded_low = sample.low - margin;
		const int64_t expanded_high = sample.high + margin;
		low = expanded_low > low ? expanded_low : low;
		high = expanded_high < high ? expanded_high : high;
		newest = sample.at_us > newest ? sample.at_us : newest;
		++result.samples;
	}

	if (!result.samples) {
		if (_count) {
			_count = 0;
			_next = 0;
			++_generation;
			result.generation = _generation;
		}
		return result;
	}

	if (low > high || (low < 0 && high > INT64_MAX + low)) {
		_fault = low > high ? Fault::InconsistentBounds : Fault::Arithmetic;
		result.fault = _fault;
		return result;
	}

	const uint64_t width = static_cast<uint64_t>(high - low);
	result.offset_us = low + static_cast<int64_t>(width / 2);
	result.uncertainty_us = (width + 1) / 2;
	result.age_us = now_us - newest;
	result.ready = result.samples >= MinimumSamples && result.uncertainty_us <= MaximumUncertaintyUs;
	_anchor_valid = true;
	_anchor_low = low;
	_anchor_high = high;
	_anchor_at_us = now_us;
	return result;
}

bool BoundedVisionClock::convert(uint64_t remote_us, uint64_t now_us, uint64_t &converted_us, uint64_t &uncertainty_us,
		ConversionTrace *trace)
{
	const Status current = status(now_us);

	if (trace) {
		*trace = {};
		trace->clock = current;
		trace->timestamp_us = now_us;
	}

	const auto finish = [trace](ConversionResult result) {
		if (trace) { trace->result = result; }

		return result == ConversionResult::Accepted;
	};

	if (!current.ready) { return finish(ConversionResult::ClockNotReady); }

	if (remote_us == 0 || remote_us > static_cast<uint64_t>(INT64_MAX)) {
		return finish(ConversionResult::InvalidRemoteStamp);
	}

	const int64_t remote = static_cast<int64_t>(remote_us);

	if ((current.offset_us > 0 && remote > INT64_MAX - current.offset_us)
	    || (current.offset_us < 0 && remote < -current.offset_us)) {
		return finish(ConversionResult::TimestampArithmetic);
	}

	const int64_t candidate = remote + current.offset_us;

	if (candidate <= 0) { return finish(ConversionResult::InvalidRemoteStamp); }

	const uint64_t mapped = static_cast<uint64_t>(candidate);
	const uint64_t distance = mapped > now_us ? mapped - now_us : now_us - mapped;
	// The offset interval was evaluated at receipt time. Cover drift back to
	// capture time, including the unknown location inside the current bound.
	if (distance > UINT64_MAX - current.uncertainty_us) { return finish(ConversionResult::TimestampArithmetic); }

	// Solve e <= u + rho*(distance+e), rather than assuming the unknown
	// capture instant equals the centre. At100ppm, rho/(1-rho)=1/9999.
	const uint64_t lookback = distance + current.uncertainty_us;
	const uint64_t capture_margin = lookback / 9999 + (lookback % 9999 != 0);
	uncertainty_us = current.uncertainty_us + capture_margin;

	if (trace) { trace->capture_uncertainty_us = uncertainty_us; }

	if (uncertainty_us > MaximumUncertaintyUs) { return finish(ConversionResult::CaptureUncertaintyExceeded); }

	if (mapped > now_us && mapped - now_us > uncertainty_us) { return finish(ConversionResult::FutureCapture); }

	converted_us = mapped;
	return finish(ConversionResult::Accepted);
}

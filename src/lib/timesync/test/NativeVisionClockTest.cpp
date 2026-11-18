/* SPDX-License-Identifier: BSD-3-Clause */
#include "../Timesync.hpp"
#include "../MorpheusVisionTiming.hpp"
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

uint64_t hrt_test_time = 1;
timesync_status_s test_last_status{};
static unsigned checks = 0;
static void require(bool condition, const char *message)
{
	++checks;
	if (!condition) { throw std::runtime_error(message); }
}

static int64_t true_offset(uint64_t at, int drift_ppm, int64_t base = -1000000000)
{
	return base + static_cast<int64_t>(at) * drift_ppm / 1000000;
}

static void exchange(Timesync &clock, uint64_t originated, uint64_t outgoing, uint64_t incoming,
		     int drift_ppm = 0, bool eligible_peer = true, int64_t jump = 0)
{
	const uint64_t remote_at = originated + outgoing;
	const int64_t remote_us = static_cast<int64_t>(remote_at) - true_offset(remote_at, drift_ppm) + jump;
	hrt_test_time = remote_at + incoming;
	clock.update(hrt_test_time, remote_us * 1000 + 333, originated * 1000, eligible_peer);
}

static void initialize(Timesync &clock, uint64_t &at, int drift_ppm = 0)
{
	for (unsigned i = 0; i < BoundedVisionClock::MinimumSamples; ++i) {
		at += 20000;
		exchange(clock, at, 2000, 2000, drift_ppm);
	}
}

static void run_tests()
{
	{
		Timesync clock;
		uint64_t mapped = 777;
		hrt_test_time = 1000000;
		require(!clock.sync_vision_stamp(1001000000, hrt_test_time, mapped), "cold strict clock must drop");
		require(mapped == 777, "cold strict conversion must not write receipt time");
		require(clock.sync_stamp(1001000000) == hrt_test_time, "legacy receipt fallback unchanged");
		uint64_t at = 1000000;
		initialize(clock, at);
		require(!clock.sync_converged(), "legacy still requires500 samples");
		require(clock.vision_clock_status(hrt_test_time).ready, "bounded ready from8 actual samples");
		require(clock.sync_vision_stamp(hrt_test_time + 1000000000 - 30000, hrt_test_time, mapped), "qualified strict conversion");
		require(mapped >= hrt_test_time - 32010 && mapped <= hrt_test_time - 27990, "strict maps capture, not receipt");
		require(test_last_status.vision_clock_ready && test_last_status.vision_clock_samples == 8,
			"actual Timesync.cpp publishes new diagnostics");
		require(test_last_status.remote_timestamp > 0 && test_last_status.round_trip_time == 4000,
			"legacy status remains actual exchange");
	}
	for (int drift_ppm : {-100, 0, 100}) {
		Timesync clock;
		uint64_t at = 10000000;
		initialize(clock, at, drift_ppm);
		for (unsigned i = 0; i < 1000; ++i) {
			at += i < 250 ? 20000 : (i < 500 ? 200000 : (i % 2 ? 40000 : 20000));
			const uint64_t outgoing = 500 + i % 3000;
			const uint64_t incoming = 500 + (i * 7) % 3000;
			exchange(clock, at, outgoing, incoming, drift_ppm);
			const auto state = clock.vision_clock_status(hrt_test_time);
			require(state.ready && state.fault == BoundedVisionClock::Fault::None, "variable cadence remains bounded");
			const int64_t truth = true_offset(hrt_test_time, drift_ppm);
			require(truth >= state.offset_us - static_cast<int64_t>(state.uncertainty_us)
				&& truth <= state.offset_us + static_cast<int64_t>(state.uncertainty_us), "true offset lies within interval");
			const uint64_t capture_fc = hrt_test_time - 30000;
			const uint64_t capture_remote = capture_fc - true_offset(capture_fc, drift_ppm);
			uint64_t mapped = 0;
			require(clock.sync_vision_stamp(capture_remote, hrt_test_time, mapped), "capture translation ready");
			const uint64_t difference = mapped > capture_fc ? mapped - capture_fc : capture_fc - mapped;
			require(difference <= BoundedVisionClock::MaximumUncertaintyUs, "capture translation within declared cap");
		}
	}
	{
		Timesync clock;
		uint64_t at = 1000000;
		initialize(clock, at, 100);
		const auto old = clock.vision_clock_status(hrt_test_time);
		const uint64_t resumed = hrt_test_time + 90000000;
		uint64_t mapped = 777;
		require(!clock.sync_vision_stamp(resumed + 1000000000, resumed, mapped), "long holdover must drop");
		auto state = clock.vision_clock_status(resumed);
		require(!state.ready && state.samples == 0 && state.generation == old.generation + 1,
			"ordinary expiry clears samples and advances generation");
		require(state.fault == BoundedVisionClock::Fault::None, "ordinary expiry is not hard fault");
		at = resumed;
		for (unsigned i = 0; i < 8; ++i) {
			at += 20000;
			exchange(clock, at, 1000, 1000, 100);
			state = clock.vision_clock_status(hrt_test_time);
			require(state.ready == (i == 7), "reacquisition requires complete new8 sample set");
		}
		require(state.generation == 1, "idle generation advances exactly once");
	}
	{
		Timesync clock;
		uint64_t at = 1000000;
		initialize(clock, at);
		at = hrt_test_time + 3000000;
		clock.vision_clock_status(at);
		exchange(clock, at + 1000, 1000, 1000, 0, true, 100000);
		require(clock.vision_clock_status(hrt_test_time).fault == BoundedVisionClock::Fault::InconsistentBounds,
			"contradictory epoch after ordinary idle must latch");
		clock.reset_filter();
		exchange(clock, at + 21000, 1000, 1000);
		require(!clock.vision_clock_status(hrt_test_time).ready, "legacy reset cannot erase strict fault");
		Timesync reconstructed;
		initialize(reconstructed, at);
		require(reconstructed.vision_clock_status(hrt_test_time).ready, "explicit object reconstruction resets hard fault");
	}
	{
		BoundedVisionClock clock;
		uint64_t at = 1000000;
		for (unsigned i = 0; i < 8; ++i) {
			at += 30000;
			require(clock.update(at + 20000, (at + 1000010000) * 1000, at * 1000), "RTT20ms is bounded evidence");
		}
		require(!clock.status(at + 20000).ready, "wide asymmetric interval does not grant readiness");
		at += 30000;
		require(!clock.update(at + 20001, (at + 1000010000) * 1000, at * 1000), "RTT above20ms rejected");
		require(!clock.update(at + 20001, (at + 1000010000) * 1000, (at + 20002) * 1000), "future originate rejected");
		const auto count = clock.status(at + 20001).samples;
		require(!clock.update(at + 20001, (at + 1000010000) * 1000, (at - 30000) * 1000), "duplicate origin rejected");
		require(clock.status(at + 20001).samples == count, "duplicates cannot manufacture sample count");
	}
	{
		Timesync clock;
		uint64_t at = 1000000;
		initialize(clock, at);
		at += 20000;
		exchange(clock, at, 1000, 1000, 0, true, -1000000);
		require(clock.vision_clock_status(hrt_test_time).fault == BoundedVisionClock::Fault::RemoteTimeReversed,
			"remote monotonic reversal latches");
		Timesync other;
		at = 1000000;
		initialize(other, at);
		other.vision_clock_status(hrt_test_time - 1);
		require(other.vision_clock_status(hrt_test_time).fault == BoundedVisionClock::Fault::LocalTimeReversed,
			"local reversal latches");
	}
	{
		Timesync clock;
		uint64_t at = 1000000;
		for (unsigned i = 0; i < 600; ++i) {
			at += 20000;
			exchange(clock, at, 1000, 1000, 0, false);
		}
		require(clock.sync_converged(), "legacy source behavior unchanged");
		require(!clock.vision_clock_status(hrt_test_time).ready, "unapproved clock peer cannot seed strict mapping");
		require(MorpheusVisionTiming::clock_peer(245, 191) && MorpheusVisionTiming::clock_peer(246, 191), "known same-clock peer handoff");
		require(!MorpheusVisionTiming::clock_peer(255, 191) && !MorpheusVisionTiming::clock_peer(245, 190), "other peers excluded");
		const uint8_t fields[] = {245, 191, 2, 20, 12};
		require(MorpheusVisionTiming::strict_input(fields[0], fields[1], fields[2], fields[3], fields[4]), "actual private input predicate");
		for (unsigned i = 0; i < 5; ++i) {
			std::vector<uint8_t> changed(fields, fields + 5);
			++changed[i];
			require(!MorpheusVisionTiming::strict_input(changed[0], changed[1], changed[2], changed[3], changed[4]), "other legacy inputs excluded");
		}
	}
	{
		BoundedVisionClock clock;
		uint64_t mapped = 123, uncertainty = 456;
		require(!clock.convert(UINT64_MAX, UINT64_MAX, mapped, uncertainty), "invalid extreme timestamps fail closed");
		require(mapped == 123, "rejection leaves output untouched");
	}
	for (int64_t offset : {-1000000000000LL, 1000000000LL}) {
		BoundedVisionClock clock;
		uint64_t at = 10000000000ULL;
		for (unsigned i = 0; i < 16; ++i) {
			at += 20000;
			const uint64_t outgoing = i % 2 ? 13000 : 1000;
			const uint64_t incoming = i % 2 ? 1000 : 13000;
			const int64_t remote = static_cast<int64_t>(at + outgoing) - offset;
			require(clock.update(at + outgoing + incoming, remote * 1000 + 999, at * 1000),
				"positive/negative offset and alternating14ms asymmetric delays valid");
		}
		const uint64_t now = at + 14000;
		const auto state = clock.status(now);
		require(state.ready && state.uncertainty_us < 2000, "opposite asymmetric intervals intersect without equal-delay assumption");
		uint64_t mapped = 0, uncertainty = 0;
		const uint64_t capture = now - 90000;
		require(clock.convert(capture - offset, now, mapped, uncertainty), "signed offset conversion");
		require((mapped > capture ? mapped - capture : capture - mapped) <= uncertainty, "signed capture contains truth");
		require(!clock.convert(now - offset + 100000, now, mapped, uncertainty), "future capture beyond interval rejected");
	}
	{
		BoundedVisionClock clock;
		// First remote event at1018000, second at1011000: valid overlapping
		// exchanges reorder at the remote and arrive here in originate order.
		require(clock.update(1018500, 1001018000LL * 1000, 1000000LL * 1000), "first overlapping exchange");
		require(clock.update(1020000, 1001011000LL * 1000, 1010000LL * 1000), "remote reordering inside overlapping exchanges is valid");
		require(clock.status(1020000).fault == BoundedVisionClock::Fault::None, "network reordering is not a clock epoch reset");
	}
	std::cout << "PASS " << checks << " native checks (actual Timesync.cpp + bounded mapper + source policy)\n";
	std::cout << "sizeof(BoundedVisionClock)=" << sizeof(BoundedVisionClock)
		  << " sizeof(Timesync_with_test_uORB_adapter)=" << sizeof(Timesync) << '\n';
}

static void replay()
{
	Timesync clock;
	std::string line;
	std::cout << "kind,now_us,ready,fault,samples,offset_us,uncertainty_us,age_us,generation,converted_us,converted\n";
	while (std::getline(std::cin, line)) {
		std::istringstream input(line);
		char kind;
		uint64_t now;
		input >> kind >> now;
		hrt_test_time = now;
		uint64_t mapped = 0;
		bool converted = false;
		if (kind == 'U') {
			int64_t remote, originated;
			input >> remote >> originated;
			clock.update(now, remote, originated, true);
		} else if (kind == 'Q') {
			uint64_t remote;
			input >> remote;
			converted = clock.sync_vision_stamp(remote, now, mapped);
		} else { throw std::runtime_error("unknown replay command"); }
		if (!input) { throw std::runtime_error("invalid replay input"); }
		const auto status = clock.vision_clock_status(now);
		std::cout << kind << ',' << now << ',' << status.ready << ',' << unsigned(status.fault) << ','
			  << status.samples << ',' << status.offset_us << ',' << status.uncertainty_us << ','
			  << status.age_us << ',' << status.generation << ',' << mapped << ',' << converted << '\n';
	}
}

int main(int argc, char **argv)
{
	try {
		if (argc == 2 && std::string(argv[1]) == "--replay") { replay(); }
		else { run_tests(); }
		return 0;
	} catch (const std::exception &error) {
		std::cerr << "FAIL: " << error.what() << '\n';
		return 1;
	}
}

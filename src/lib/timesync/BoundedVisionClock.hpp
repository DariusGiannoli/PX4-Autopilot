/* SPDX-License-Identifier: BSD-3-Clause */
#pragma once

#include <stdint.h>
#include <limits.h>

/** Causal FC-minus-remote clock bounds, without equal-path-delay or skew fits.
 *
 * Assumptions: timestamps identify the actual exchange, nonnegative transport
 * delay, and relative clock drift <=100 ppm. Nanosecond truncation is covered
 * explicitly. Readiness is an engineering timing screen, not flight approval.
 * Contradiction/reversal faults latch until this object is reconstructed.
 * Ordinary expiry clears readiness and requires a new complete sample set.
 * Legacy exponential-filter resets must not erase contradiction evidence.
 */
class BoundedVisionClock
{
public:
	static constexpr unsigned Capacity = 128;
	static constexpr unsigned MinimumSamples = 8;
	static constexpr uint64_t WindowUs = 2000000;
	static constexpr uint64_t MaximumRttUs = 20000;
	static constexpr uint64_t MaximumUncertaintyUs = 5000;
	static constexpr unsigned DriftPpm = 100;

	enum class Fault : uint8_t { None, LocalTimeReversed, RemoteTimeReversed, InconsistentBounds, Arithmetic };
	struct Status {
		int64_t offset_us{0};
		uint64_t uncertainty_us{UINT64_MAX};
		uint64_t age_us{UINT64_MAX};
		uint32_t generation{0};
		uint16_t samples{0};
		Fault fault{Fault::None};
		bool ready{false};
	};

	bool update(uint64_t now_us, int64_t remote_ns, int64_t originated_ns);

	Status status(uint64_t now_us);

	bool convert(uint64_t remote_us, uint64_t now_us, uint64_t &converted_us, uint64_t &uncertainty_us);

private:
	struct Sample { uint64_t at_us{0}; int64_t low{0}; int64_t high{0}; };
	static uint64_t drift(uint64_t age_us) __attribute__((noinline));
	Sample _samples[Capacity]{};
	unsigned _count{0};
	unsigned _next{0};
	uint64_t _last_now_us{0};
	uint64_t _last_originated_us{0};
	uint64_t _last_received_us{0};
	int64_t _last_remote_ns{0};
	bool _anchor_valid{false};
	int64_t _anchor_low{0};
	int64_t _anchor_high{0};
	uint64_t _anchor_at_us{0};
	uint32_t _generation{0};
	Fault _fault{Fault::None};
};

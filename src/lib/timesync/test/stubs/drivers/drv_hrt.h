#pragma once
#include <stdint.h>
#include <time.h>
extern uint64_t hrt_test_time;
inline uint64_t hrt_absolute_time() { return hrt_test_time; }
namespace time_literals {
constexpr uint64_t operator"" _ms(unsigned long long value) { return value * 1000; }
}
#define PX4_WARN(...) do {} while (0)

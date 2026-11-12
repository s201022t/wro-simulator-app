#include "servo_s_curve.h"
#include <initializer_list>
using namespace servo_s_curve;

constexpr bool boundedMonotonicBothDirections() {
  for (int speed : {1, 100, 200, 255}) {
    const auto duration = durationMs(500, 2500, speed);
    int previousUp = 500, previousDown = 2500;
    for (uint32_t t = 0; t <= duration + 1; ++t) {
      const int up = pulseAt(500, 2500, t, duration);
      const int down = pulseAt(2500, 500, t, duration);
      if (up < previousUp || up > 2500 || down > previousDown || down < 500) return false;
      previousUp = up;
      previousDown = down;
    }
    if (previousUp != 2500 || previousDown != 500) return false;
  }
  return true;
}

constexpr bool gentleEndpoints() {
  // Compare pulse increments over equal time windows: gentle ends, faster middle.
  const int start = pulseAt(500, 2500, 100, 1000) - 500;
  const int middle = pulseAt(500, 2500, 550, 1000) - pulseAt(500, 2500, 450, 1000);
  const int end = 2500 - pulseAt(500, 2500, 900, 1000);
  return start == end && start < middle / 10;
}

constexpr bool elapsedTimeAndWrap() {
  const uint32_t start = UINT32_MAX - 49;
  const uint32_t now = 50;
  return pulseAt(500, 2500, uint32_t(now - start), 1000) == pulseAt(500, 2500, 100, 1000)
      && pulseAt(500, 2500, 5000, 1000) == 2500;
}

static_assert(boundedMonotonicBothDirections(), "No overshoot in either direction");
static_assert(gentleEndpoints(), "S-curve must ease in and out");
static_assert(elapsedTimeAndWrap(), "Wall time, including rollover, determines position");
static_assert(durationMs(500, 2500, 100) > durationMs(500, 2500, 200), "Speeds are distinct");
static_assert(durationMs(500, 2500, 200) > durationMs(500, 2500, 255), "Speeds are distinct");
static_assert(durationMs(500, 2500, 280) == durationMs(500, 2500, 255), "Clamp high speeds");
static_assert(durationMs(500, 2500, 0) == durationMs(500, 2500, 1), "Clamp low speeds");
static_assert(durationMs(1500, 1500, 255) == 0, "Same position completes immediately");
static_assert(durationMs(1500, 1501, 255) == 100, "Short moves retain easing time");
static_assert(pulseAt(1500, 1501, 100, 100) == 1501, "Exact short move endpoint");
static_assert(pulseAt(1200, 1200, 0, 0) == 1200, "Zero duration is defined");
static_assert(pulseAt(1234, 500, 0, 1000) == 1234, "Retarget starts at last output pulse");

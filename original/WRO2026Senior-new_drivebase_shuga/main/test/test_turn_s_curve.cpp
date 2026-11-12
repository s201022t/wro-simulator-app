#include "turn_s_curve.h"

using turn_s_curve::power;

constexpr bool startupAndDeceleration() {
  if (power(0, 360, 0, 100, 255, 0.33f, 600) != 100) return false;
  float previous = 100;
  for (int ms = 1; ms <= 400; ++ms) {
    float now = power(ms * 0.001f, 360, 0, 100, 255, 0.33f, 600);
    if (now < previous || now > 255 || now - previous > 0.601f) return false;
    previous = now;
  }
  if (previous != 255) return false;
  for (int deg = 0; deg <= 360; ++deg) {
    float now = power(1, 360, deg, 100, 255, 0.33f, 600);
    if (now > previous || now < 100) return false;
    previous = now;
  }
  return previous == 100;
}

constexpr bool shortTurnAndLowMaximum() {
  // Even while accelerating, near-target PWM must respect deceleration.
  if (power(0.2f, 5, 4.9f, 100, 255, 0.33f, 600) >= 102) return false;
  if (power(0, 90, 0, 80, 80, 0.4f, 600) != 80) return false;
  if (power(1, 90, 90, 80, 80, 0.4f, 600) != 80) return false;
  return power(1, 90, 100, 100, 255, 0.4f, 600) == 100;
}

constexpr bool samplingRateDoesNotChangeProfile() {
  // Compare common wall-clock points at 200Hz and 8kHz.
  for (int tick = 0; tick <= 80; ++tick) {
    const float slow = power((tick * 5000) * 0.000001f, 90, tick, 100, 255, 0.33f, 600);
    const float fast = power((tick * 40 * 125) * 0.000001f, 90, tick, 100, 255, 0.33f, 600);
    if (slow != fast) return false;
  }
  return true;
}

static_assert(startupAndDeceleration(), "S-curve must rise and fall, without a 200 PWM floor");
static_assert(shortTurnAndLowMaximum(), "Short turns and capped power must stay bounded");
static_assert(samplingRateDoesNotChangeProfile(), "Changing sample rate must preserve the envelope");
static_assert(turn_s_curve::clampWheel(255, 100, 120) == 120,
              "Wheel correction must not bypass the acceleration envelope");
static_assert(turn_s_curve::clampWheel(0, 100, 120) == 100,
              "Driving wheel retains its configured friction floor");

constexpr bool minimumSpeedSearchRange() {
  // 90 degree spin: deceleration starts 29.7 degrees before the target and
  // must reach the 100 PWM floor at 80 degrees (10 degrees remaining).
  if (power(1, 90, 79, 100, 255, 0.33f, 600, 10) <= 100) return false;
  if (power(1, 90, 80, 100, 255, 0.33f, 600, 10) != 100) return false;
  if (power(1, 90, 85, 100, 255, 0.33f, 600, 10) != 100) return false;
  if (power(1, 90, 89, 100, 255, 0.33f, 600, 10) != 100) return false;
  // Existing callers that omit the range keep the original curve.
  return power(1, 90, 80, 100, 255, 0.33f, 600) > 100;
}

static_assert(minimumSpeedSearchRange(),
              "Spin must enter minimum-speed search ten degrees early");

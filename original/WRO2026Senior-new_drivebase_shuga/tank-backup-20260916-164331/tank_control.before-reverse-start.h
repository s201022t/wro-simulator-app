#pragma once

#include "turn_s_curve.h"

// Pure PWM planning helpers, independent of Arduino and control-loop frequency.
namespace tank_control {
constexpr int magnitude(int value) {
  return value >= 255 || value <= -255 ? 255 : value < 0 ? -value : value;
}

constexpr int minimum(int requested, int maximum) {
  const int value = magnitude(requested);
  return value > maximum ? maximum : value;
}

constexpr float envelope(float elapsedSec, float target, float progress,
                         int minimumPower, int maximumPower) {
  return turn_s_curve::power(elapsedSec, target, progress,
                            minimumPower, maximumPower, 0.30f, 600.0f);
}

struct Wheels { int left; int right; };

// The profile is a ceiling for BOTH wheels. Make room for steering by lowering
// common power, so correction never accelerates a wheel beyond the envelope.
constexpr Wheels mix(int ceiling, int floor, int correction) {
  const int limit = (ceiling - floor) / 2;
  const int steer = correction < -limit ? -limit : correction > limit ? limit : correction;
  const int magnitude = steer < 0 ? -steer : steer;
  const int common = ceiling - magnitude;
  return {common - steer, common + steer};
}
}

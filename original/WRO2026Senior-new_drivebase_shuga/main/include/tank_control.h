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
                         int minimumPower, int maximumPower,
                         float decelerationRatio = 0.30f,
                         float minimumSpeedRange = 0.0f) {
  return turn_s_curve::power(elapsedSec, target, progress,
                            minimumPower, maximumPower,
                            decelerationRatio, 600.0f,
                            minimumSpeedRange);
}

struct Wheels { int left; int right; };

// Fade in attainable steering during reverse startup. Clamp BEFORE scaling so
// a large encoder error cannot saturate the mixer and bypass the startup ramp.
// This shapes correction amplitude, not the rate of a changing sensor signal.
constexpr int startupCorrection(int correction, int direction, float elapsedSec,
                                int ceiling, int floor) {
  if (direction >= 0) return correction;
  const int limit = (ceiling - floor) / 2;
  const int bounded = correction < -limit ? -limit : correction > limit ? limit : correction;
  if (elapsedSec >= 0.250f) return bounded;
  return static_cast<int>(bounded * turn_s_curve::smooth(elapsedSec / 0.250f));
}

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

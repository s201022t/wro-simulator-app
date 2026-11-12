#pragma once

// Hardware-independent PWM envelope. Inputs are finite; targetDeg > 0,
// 0 <= minPower <= maxPower, 0 < decRatio <= 1, riseRate > 0.
namespace turn_s_curve {
constexpr float smooth(float x) {
  return x <= 0.0f ? 0.0f : x >= 1.0f ? 1.0f : x * x * (3.0f - 2.0f * x);
}

constexpr float power(float elapsedSec, float targetDeg, float progressDeg,
                      int minPower, int maxPower, float decRatio, float riseRate,
                      float minimumSpeedRangeDeg = 0.0f) {
  // Cubic smoothstep's maximum slope is 1.5. Keep the configured peak
  // PWM rise rate while making the start/end slopes zero.
  const float span = static_cast<float>(maxPower - minPower);
  if (span <= 0.0f) return static_cast<float>(minPower);
  const float accelSec = 1.5f * span / riseRate;
  const float accel = smooth(elapsedSec / accelSec);
  const float decelRange = targetDeg * decRatio;
  // Leave a non-zero transition range for short turns. For a 90 degree spin,
  // decelRange=29.7 and a requested 10 degree search range is used unchanged.
  const float maxMinimumRange = decelRange * 0.5f;
  const float minimumRange = minimumSpeedRangeDeg <= 0.0f ? 0.0f
      : minimumSpeedRangeDeg < maxMinimumRange ? minimumSpeedRangeDeg
      : maxMinimumRange;
  const float transitionRange = decelRange - minimumRange;
  const float remaining = targetDeg - progressDeg;
  const float decel = remaining <= minimumRange ? 0.0f
      : transitionRange <= 0.0f ? 1.0f
      : smooth((remaining - minimumRange) / transitionRange);
  return minPower + span * (accel < decel ? accel : decel);
}

// Apply AFTER motor gains and wheel synchronization, so neither can bypass
// the envelope. Zero/braking and the stationary pivot wheel are handled separately.
constexpr int clampWheel(int requested, int minPower, int profileMax) {
  return requested < minPower ? minPower : requested > profileMax ? profileMax : requested;
}
}

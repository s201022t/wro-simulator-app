#pragma once

// yaw is the unwrapped displacement from resetYawDeg(), not a 0..360 heading.
// Motor direction is chosen by the caller. Gyro mounting polarity is irrelevant.
namespace turn_progress {
constexpr float angle(float yaw) {
  return yaw < 0.0f ? -yaw : yaw;
}

// d(abs(yaw))/dt: returning toward the start must reduce progress, not add travel.
constexpr float rate(float yaw, float yawRate) {
  return yaw > 0.0f ? yawRate : yaw < 0.0f ? -yawRate : 0.0f;
}

constexpr bool reached(float yaw, float target) {
  return angle(yaw) >= target;
}

// Estimate the angle travelled before braking takes effect. Reverse motion
// must not increase the prediction. The minimum/maximum keep the lead useful
// at low speed and bounded at high speed.
constexpr float brakeLead(float rateDegPerSec, float lookaheadSec,
                          float minimumDeg, float maximumDeg) {
  const float forwardRate = rateDegPerSec > 0.0f ? rateDegPerSec : 0.0f;
  const float estimated = forwardRate * lookaheadSec;
  return estimated < minimumDeg ? minimumDeg
       : estimated > maximumDeg ? maximumDeg
       : estimated;
}

constexpr bool shouldBrake(float progressDeg, float targetDeg,
                           float rateDegPerSec, float lookaheadSec,
                           float minimumDeg, float maximumDeg) {
  return progressDeg + brakeLead(rateDegPerSec, lookaheadSec,
                                 minimumDeg, maximumDeg) >= targetDeg;
}
}

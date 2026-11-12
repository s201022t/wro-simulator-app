#pragma once
#include <stdint.h>

// Pulse-position trajectory; angles are logical command units, not feedback.
namespace servo_s_curve {
constexpr int clampSpeed(int speed) {
  return speed < 1 ? 1 : speed > 255 ? 255 : speed;
}

// Quintic smoothstep: velocity and acceleration are zero at both ends.
constexpr double smooth(double t) {
  return t <= 0 ? 0 : t >= 1 ? 1 : t * t * t * (10 + t * (-15 + 6 * t));
}

constexpr uint32_t durationMs(int fromUs, int toUs, int speed) {
  const int distance = toUs > fromUs ? toUs - fromUs : fromUs - toUs;
  if (distance == 0) return 0;
  // Peak rate: 33.33..500 logical degrees/s over 500..2500us.
  const double peakDegPerSec = 1000.0 / 30.0 +
      (500.0 - 1000.0 / 30.0) * (clampSpeed(speed) - 1) / 254.0;
  // Maximum derivative of quintic smoothstep is 1.875.
  const double ms = 1.875 * distance * 0.18 * 1000.0 / peakDegPerSec;
  const uint32_t roundedUp = static_cast<uint32_t>(ms + 0.999999);
  return roundedUp < 100 ? 100 : roundedUp;
}

constexpr int pulseAt(int fromUs, int toUs, uint32_t elapsed, uint32_t duration) {
  if (duration == 0 || elapsed >= duration) return toUs;
  const double pulse = fromUs + (toUs - fromUs) * smooth(double(elapsed) / duration);
  return static_cast<int>(pulse + 0.5);
}
}

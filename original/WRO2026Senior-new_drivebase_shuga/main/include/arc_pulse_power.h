#pragma once

#include <stdint.h>

// Absolute PWM magnitudes only; the caller applies the wheel's direction.
// A 10ms slot is independent of the much faster gyro control loop.
class ArcPulsePower {
public:
  static constexpr uint32_t SLOT_MS = 10;

  constexpr explicit ArcPulsePower(uint32_t startMs)
    : lastSlotMs_(startMs - SLOT_MS) {}

  constexpr int update(uint32_t nowMs, float requestedPower,
                       int pulsePower, bool finished, float gain = 1.0f) {
    if (finished || requestedPower <= 0.0f || pulsePower <= 0 || gain <= 0.0f) {
      reset(nowMs);
      return 0;
    }

    // Gain adjusts only the low-power pulse region.
    if (requestedPower >= pulsePower) {
      accumulator_ = 0.0f;
      output_ = static_cast<int>(requestedPower);
      lastSlotMs_ = nowMs - SLOT_MS;
      return output_;
    }
    if (static_cast<uint32_t>(nowMs - lastSlotMs_) < SLOT_MS) return output_;

    // Do not emit catch-up bursts after a delayed control cycle.
    lastSlotMs_ = nowMs;
    float increment = requestedPower * gain;
    if (increment > pulsePower) increment = static_cast<float>(pulsePower);
    accumulator_ += increment;
    if (accumulator_ >= pulsePower) {
      accumulator_ -= pulsePower;
      output_ = pulsePower;
    } else {
      output_ = 0;
    }
    return output_;
  }

private:
  constexpr void reset(uint32_t nowMs) {
    accumulator_ = 0.0f;
    output_ = 0;
    lastSlotMs_ = nowMs - SLOT_MS;
  }

  uint32_t lastSlotMs_;
  float accumulator_ = 0.0f;
  int output_ = 0;
};

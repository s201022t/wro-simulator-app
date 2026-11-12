#include "turn_progress.h"
#include "turn_s_curve.h"

constexpr bool wrongInitialMotionCannotLatchDirection() {
  // Initial +0.2 degree kick, then negative rotation. Old sign latch gives
  // progress=-180 at the end, so error=360 and the motor never hits angle stop.
  const float samples[] = {0.0f, 0.2f, 0.05f, -0.1f, -90.0f, -179.9f, -180.0f};
  for (int i = 0; i < 6; ++i) {
    if (turn_progress::reached(samples[i], 180)) return false;
    if (turn_progress::reached(-samples[i], 180)) return false;
  }
  return turn_progress::reached(samples[6], 180) &&
         turn_progress::reached(-samples[6], 180);
}

constexpr bool mirroredTurnsHaveSameControlInputs() {
  for (int deg = 1; deg < 180; ++deg) {
    const float positive = turn_progress::angle(deg);
    const float negative = turn_progress::angle(-deg);
    if (positive != negative) return false;
    const float rateP = turn_progress::rate(deg, 90);
    const float rateN = turn_progress::rate(-deg, -90);
    if (rateP != rateN) return false;
    if (8 * (180 - positive) - 0.3f * rateP !=
        8 * (180 - negative) - 0.3f * rateN) return false;
    if (turn_s_curve::power(1, 180, positive, 100, 255, 0.33f, 600) !=
        turn_s_curve::power(1, 180, negative, 100, 255, 0.33f, 600)) return false;
  }
  return true;
}

static_assert(wrongInitialMotionCannotLatchDirection(), "Initial opposite kick must not prevent stopping");
static_assert(mirroredTurnsHaveSameControlInputs(), "Both gyro polarities need identical PD and S-curve inputs");
static_assert(turn_progress::rate(-90, 20) == -20, "Returning toward start reduces progress");
static_assert(turn_progress::rate(90, -20) == -20, "Positive-side return reduces progress too");
static_assert(turn_progress::rate(0, 20) == 0, "Zero displacement has no chosen radial direction");
static_assert(turn_progress::reached(-181, 180), "Overshoot must stop");
static_assert(!turn_progress::reached(-359, 360), "Do not wrap yaw at 180 or 360");
static_assert(turn_progress::reached(-360, 360), "Full negative rotation must stop");
static_assert(turn_progress::reached(720, 720), "Multiple rotations use unwrapped yaw");
static_assert(turn_progress::brakeLead(100, 0.04f, 0.8f, 4.5f) == 4.0f,
              "Brake lead follows forward angular speed");
static_assert(turn_progress::brakeLead(200, 0.04f, 0.8f, 4.5f) == 4.5f,
              "Brake lead is capped at high angular speed");
static_assert(turn_progress::brakeLead(-100, 0.04f, 0.8f, 4.5f) == 0.8f,
              "Reverse motion cannot enlarge brake lead");
static_assert(turn_progress::shouldBrake(86, 90, 100, 0.04f, 0.8f, 4.5f),
              "A 100 dps turn brakes four degrees before target");
static_assert(!turn_progress::shouldBrake(85, 90, 100, 0.04f, 0.8f, 4.5f),
              "Prediction must not brake before the calculated lead");
static_assert(turn_progress::brakeLead(100, 0.07f, 1.1f, 8.0f) == 7.0f,
              "Current spin settings predict seven degrees at 100 dps");
static_assert(turn_progress::brakeLead(200, 0.07f, 1.1f, 8.0f) == 8.0f,
              "Current spin settings cap prediction at eight degrees");

#include "arc_stall_guard.h"

// Hardware-independent regression tests, evaluated by the C++14 compiler.
// arm-none-eabi-g++ -std=c++14 -I main/include -fsyntax-only main/test/test_arc_stall_guard.cpp

constexpr bool stationaryWheelRecoversOrStops() {
  ArcStallGuard guard(0, 45, 130);
  if (guard.update(0, 0, false, 30) != 45) return false;
  if (guard.update(149, 0, false, 30) != 45) return false;
  if (guard.update(150, 0, false, 30) != 65) return false;
  if (guard.update(151, 0, false, 30) != 65) return false;
  if (guard.update(300, 0, false, 30) != 85) return false;
  guard.update(450, 0, false, 30);
  guard.update(600, 0, false, 30);
  if (guard.update(750, 0, false, 30) != 130) return false;
  if (guard.update(999, 0, false, 30) != 130 || guard.stalled()) return false;
  return guard.update(1000, 0, false, 30) == 0 && guard.stalled();
}

constexpr bool recoveryKeepsWorkingPowerAndRestartsWatchdog() {
  ArcStallGuard guard(0, 45, 255);
  guard.update(0, 0, false, 60);
  if (guard.update(150, 0, false, 60) != 80) return false;
  if (guard.update(200, 2, false, 20) != 80) return false;
  for (uint32_t now = 300; now <= 2000; now += 100) {
    if (guard.update(now, now / 50, false, 20) != 80 || guard.stalled()) return false;
  }
  guard.update(2999, 40, false, 20);
  if (guard.stalled()) return false;
  return guard.update(3000, 40, false, 20) == 0 && guard.stalled();
}

constexpr bool independentWheelsAndPivot() {
  ArcStallGuard left(0, 45, 130);
  ArcStallGuard right(0, 45, 130);
  for (uint32_t now = 0; now <= 2000; now += 100) {
    if (left.update(now, 0, true, 130) != 0 || left.stalled()) return false;
    if (right.update(now, now / 50, false, 100) != 100 || right.stalled()) return false;
  }
  return true;
}

constexpr bool completedWheelStaysOff() {
  ArcStallGuard guard(0, 45, 255);
  guard.update(0, 0, false, 45);
  guard.update(150, 0, false, 45);
  return guard.update(200, 1, true, 255) == 0 &&
         guard.update(2000, 1, true, 255) == 0 && !guard.stalled();
}

constexpr bool jitterDoesNotHideStall() {
  ArcStallGuard guard(0, 45, 255);
  for (uint32_t now = 0; now < 1000; now += 100) {
    guard.update(now, (now / 100) % 2, false, 45);
  }
  return guard.update(1000, 1, false, 45) == 0 && guard.stalled();
}

constexpr bool respectsPowerLimit() {
  ArcStallGuard guard(0, 45, 30);
  if (guard.update(0, 0, false, -100) != 30) return false;
  if (guard.update(150, 0, false, 255) != 30) return false;
  return guard.update(300, 2, false, -100) == 30 && !guard.stalled();
}

constexpr bool clockWraparound() {
  ArcStallGuard guard(UINT32_MAX - 100, 45, 255);
  guard.update(UINT32_MAX - 100, 0, false, 45);
  if (guard.update(49, 0, false, 45) != 65) return false;
  guard.update(898, 0, false, 45);
  if (guard.stalled()) return false;
  return guard.update(899, 0, false, 45) == 0 && guard.stalled();
}

static_assert(stationaryWheelRecoversOrStops(), "stall ramp and timeout");
static_assert(recoveryKeepsWorkingPowerAndRestartsWatchdog(), "recovery and subsequent stall");
static_assert(independentWheelsAndPivot(), "independent monitoring and intentional stop");
static_assert(completedWheelStaysOff(), "completed wheel must never be boosted");
static_assert(jitterDoesNotHideStall(), "encoder jitter must not reset watchdog");
static_assert(respectsPowerLimit(), "PWM must stay within configured limit");
static_assert(clockWraparound(), "millis wraparound must not break watchdog");

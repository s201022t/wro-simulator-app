#include "arc_pulse_power.h"
#include "arc_stall_guard.h"

constexpr bool halfDutyAndFastControlLoop() {
  ArcPulsePower pulse(0);
  // 15% requested, 30% on-pulse: alternate 0 and 30% every 10ms.
  for (uint32_t now = 0; now < 100; ++now) {
    if (pulse.update(now, 38.5f, 77, false) !=
        ((now / 10) % 2 == 0 ? 0 : 77)) return false;
  }
  return true;
}

constexpr bool fractionalAverageAndGain() {
  ArcPulsePower unity(0), half(0), saturated(0);
  int unityTotal = 0, halfTotal = 0;
  for (uint32_t now = 0; now < 1000; now += 10) {
    unityTotal += unity.update(now, 21, 77, false);
    halfTotal += half.update(now, 21, 77, false, 0.5f);
    if (saturated.update(now, 21, 77, false, 100.0f) != 77) return false;
  }
  // Quantization error must be less than one on-pulse over the full run.
  return unityTotal <= 2100 && unityTotal > 2100 - 77 &&
         halfTotal <= 1050 && halfTotal > 1050 - 77;
}

constexpr bool immediateStopsAndRestart() {
  ArcPulsePower pulse(0);
  pulse.update(0, 38.5f, 77, false);
  if (pulse.update(10, 38.5f, 77, false) != 77) return false;
  if (pulse.update(11, 38.5f, 77, true) != 0) return false;
  if (pulse.update(12, 38.5f, 77, false) != 0) return false;
  if (pulse.update(22, 38.5f, 77, false) != 77) return false;
  if (pulse.update(23, 0, 77, false) != 0) return false;
  return pulse.update(24, -10, 77, false) == 0;
}

constexpr bool continuousAndDelayedSlots() {
  ArcPulsePower pulse(0);
  if (pulse.update(0, 100, 100, false) != 100) return false;
  if (pulse.update(1, 38.5f, 77, false) != 0) return false;
  if (pulse.update(1001, 38.5f, 77, false) != 77) return false;
  if (pulse.update(1002, 38.5f, 77, false) != 77) return false;
  return pulse.update(1011, 38.5f, 77, false) == 0;
}

constexpr bool clockWraparound() {
  ArcPulsePower pulse(UINT32_MAX - 5);
  if (pulse.update(UINT32_MAX - 5, 38.5f, 77, false) != 0) return false;
  if (pulse.update(3, 38.5f, 77, false) != 0) return false;
  return pulse.update(4, 38.5f, 77, false) == 77;
}

constexpr bool pulsingCannotHideStall() {
  ArcStallGuard guard(0, 77, 200);
  ArcPulsePower pulse(0);
  for (uint32_t now = 0; now <= 1000; ++now) {
    int amplitude = guard.update(now, 0, false, 38);
    int output = pulse.update(now, 38, amplitude, guard.stalled());
    if (output < 0 || output > 200) return false;
    if (now < 1000 && guard.stalled()) return false;
    if (now == 1000 && (!guard.stalled() || output != 0)) return false;
  }
  return true;
}

static_assert(halfDutyAndFastControlLoop(), "Pulse timing must not depend on control loop frequency");
static_assert(fractionalAverageAndGain(), "Accumulator preserves requested average PWM and bounds gain");
static_assert(immediateStopsAndRestart(), "Finished and zero-power commands stop immediately and clear residual");
static_assert(continuousAndDelayedSlots(), "Continuous PWM and delayed slots must not produce catch-up bursts");
static_assert(clockWraparound(), "millis wrap must preserve pulse timing");
static_assert(pulsingCannotHideStall(), "Stationary inner wheel must stop after one second");

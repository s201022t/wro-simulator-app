#include "tank_control.h"
#include <limits.h>

using namespace tank_control;

static_assert(magnitude(10) == 10 && minimum(100, 10) == 10,
              "Low requested maxima must not become PWM 100");
static_assert(magnitude(0) == 0 && minimum(100, 0) == 0,
              "Zero maximum must remain stopped");
static_assert(magnitude(INT_MIN) == 255 && magnitude(INT_MAX) == 255,
              "Signed magnitude normalization must not overflow");

constexpr bool boundedSteering() {
  for (int ceiling = 0; ceiling <= 255; ++ceiling) {
    const int floor = ceiling < 100 ? ceiling / 2 : 50;
    for (int correction = -150; correction <= 150; correction += 5) {
      const Wheels wheels = mix(ceiling, floor, correction);
      const Wheels mirror = mix(ceiling, floor, -correction);
      if (wheels.left < floor || wheels.right < floor ||
          wheels.left > ceiling || wheels.right > ceiling) return false;
      if (wheels.left + wheels.right > 2 * ceiling) return false;
      if (wheels.left != mirror.right || wheels.right != mirror.left) return false;
      if (correction > 0 && wheels.left > wheels.right) return false;
    }
  }
  return true;
}
static_assert(boundedSteering(), "Steering must stay bounded and symmetric for reverse travel");
static_assert(envelope(0, 100, 0, 100, 255) == 100, "Start at minimum power");
static_assert(envelope(0.1f, 10, 0, 100, 255) < 130,
              "Short elapsed time must not jump to maximum PWM");
static_assert(envelope(0.2f, 10, 9.9f, 100, 255) < 101,
              "Deceleration must win even while acceleration is active");
static_assert(envelope(1, 10, 10, 100, 255) == 100, "End at minimum power");
static_assert(envelope(1, 100, 50, 10, 10) == 10, "Low cap must survive profile planning");

constexpr bool reverseStartup() {
  for (int correction = -150; correction <= 150; ++correction) {
    if (startupCorrection(correction, -1, 0, 100, 50) != 0) return false;
    const Wheels original = mix(100, 50, correction);
    const Wheels settled = mix(100, 50, startupCorrection(correction, -1, 0.250f, 100, 50));
    if (original.left != settled.left || original.right != settled.right) return false;
    int previous = 0;
    for (int ms = 0; ms <= 250; ++ms) {
      const float sec = ms * 0.001f;
      if (startupCorrection(correction, 1, sec, 100, 50) != correction) return false;
      const int actual = startupCorrection(correction, -1, sec, 100, 50);
      if (actual != -startupCorrection(-correction, -1, sec, 100, 50)) return false;
      const int absolute = actual < 0 ? -actual : actual;
      if (absolute < previous || absolute > 25) return false;
      previous = absolute;
    }
  }
  return true;
}
static_assert(reverseStartup(), "Reverse fade must be symmetric and preserve forward and settled steering");
static_assert(startupCorrection(150, -1, 0.125f, 100, 50) == 12,
              "Even saturated error must use only half of attainable steering at 125ms");

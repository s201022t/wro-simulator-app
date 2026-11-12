#include "arc_geometry.h"

// Differential-drive kinematics: clockwise yaw is (left - right) / track.
constexpr bool correctDirection(float radius, float deg, int power) {
  const auto wheels = arc_geometry::wheelRadii(radius, 60.0f, deg, power);
  const int drive = power > 0 ? 1 : -1;
  const float clockwise = drive * (wheels.left - wheels.right);
  return (deg > 0 ? clockwise > 0 : clockwise < 0) &&
         (wheels.left + wheels.right) / 2.0f == radius;
}

constexpr bool allDirections() {
  const float radii[] = {0.0f, 40.0f, 60.0f, 300.0f};
  const float angles[] = {-360.0f, -180.0f, -90.0f, 90.0f, 180.0f, 360.0f};
  for (float radius : radii) {
    for (float deg : angles) {
      if (!correctDirection(radius, deg, 200) ||
          !correctDirection(radius, deg, -200)) return false;
    }
  }
  return true;
}

static_assert(allDirections(), "Arc direction must hold at 180 degrees and for reverse travel");
static_assert(arc_geometry::wheelRadii(40, 60, 180, 255).right < 0,
              "Tight right turn must reverse the right inner wheel");
static_assert(arc_geometry::wheelRadii(60, 60, 180, 255).right == 0,
              "Half-track radius must stop the right inner wheel");

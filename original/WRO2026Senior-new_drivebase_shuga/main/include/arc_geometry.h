#pragma once

namespace arc_geometry {
struct WheelRadii {
  float left;
  float right;
};

// Signed radii: negative means the inner wheel must run backwards.
// Positive deg is clockwise; reversing travel swaps the inner/outer wheels.
constexpr WheelRadii wheelRadii(float radius, float halfTrack, float deg, int power) {
  return ((deg > 0) == (power >= 0))
    ? WheelRadii{radius + halfTrack, radius - halfTrack}
    : WheelRadii{radius - halfTrack, radius + halfTrack};
}
}

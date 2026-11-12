#pragma once

namespace drive_sync {
struct Speeds { float left; float right; };

// Inputs are normalized to each wheel's requested travel direction.
// Like tank_control::mix, subtract the common offset as well as steering:
// the lagging wheel stays at its base speed and only the leading wheel slows.
// Use progress ratios so unequal wheel distances can still finish together.
constexpr Speeds apply(float leftBase, float rightBase,
                       float leftPeak, float rightPeak,
                       float leftTarget, float rightTarget,
                       float leftPosition, float rightPosition,
                       float gain, bool active) {
  if (!active || leftTarget<=0 || rightTarget<=0 ||
      leftPeak<=0 || rightPeak<=0 || leftBase<=0 || rightBase<=0)
    return {leftBase,rightBase};
  const float correction=gain*(leftPosition/leftTarget-rightPosition/rightTarget);
  if (correction>0) {
    const float reduction=2*correction*leftPeak;
    return {reduction>=leftBase ? 0 : leftBase-reduction,rightBase};
  }
  const float reduction=-2*correction*rightPeak;
  return {leftBase,reduction>=rightBase ? 0 : rightBase-reduction};
}
}

#include "ff_motion_detector.h"
using ff_calibration::MotionDetector;
constexpr bool rejectsJump() {
  MotionDetector d;
  return !d.sample(20) && !d.sample(20) && !d.sample(20) && !d.sample(20);
}
constexpr bool rejectsJitter() {
  MotionDetector d;
  for(int i=0;i<20;++i) if(d.sample(i%2 ? -1 : 1)) return false;
  return true;
}
constexpr bool acceptsContinuous() {
  MotionDetector d;
  return !d.sample(4) && !d.sample(8) && d.sample(12);
}
constexpr bool requiresDistance() {
  MotionDetector d;
  return !d.sample(2) && !d.sample(4) && !d.sample(6) &&
    !d.sample(8) && !d.sample(10) && d.sample(12);
}
constexpr bool resetsAfterPause() {
  MotionDetector d;
  return !d.sample(4) && !d.sample(8) && !d.sample(8) &&
    !d.sample(12) && !d.sample(16) && d.sample(20);
}
constexpr bool rejectsWrongDirection() {
  MotionDetector d;
  return !d.sample(-4) && !d.sample(-8) && !d.sample(-12);
}
static_assert(rejectsJump(),"A single burst followed by rest is not sustained motion");
static_assert(rejectsJitter(),"Small back-and-forth counts must not count as travel");
static_assert(acceptsContinuous(),"Accept sufficient forward travel over three windows");
static_assert(requiresDistance(),"Window count alone cannot bypass minimum distance");
static_assert(resetsAfterPause(),"A pause must reset consecutive windows");
static_assert(rejectsWrongDirection(),"Opposite direction cannot pass");

#pragma once
#include <stdint.h>

namespace ff_calibration {
constexpr int MOTION_COUNTS=12;
constexpr int WINDOW_COUNTS=2;
constexpr int REQUIRED_WINDOWS=3;
constexpr unsigned long WINDOW_MS=50;

// sample() is called once per WINDOW_MS with signed progress from test start,
// normalized to the commanded direction. A one-off jump cannot pass.
struct MotionDetector {
  int64_t previous=0;
  int consecutive=0;
  constexpr bool sample(int64_t progress) {
    consecutive=progress-previous>=WINDOW_COUNTS ? consecutive+1 : 0;
    previous=progress;
    return progress>=MOTION_COUNTS && consecutive>=REQUIRED_WINDOWS;
  }
};
}

#pragma once

namespace line_sensor_normalization {
constexpr int fullScale = 1023;
struct Calibration { int white; int black; };

constexpr bool valid(Calibration c) {
  return c.white >= 0 && c.white <= fullScale &&
         c.black >= 0 && c.black <= fullScale && c.white != c.black;
}

// Supports either sensor polarity; clamps readings outside the calibrated range.
constexpr int normalize(int raw, Calibration c) {
  if (!valid(c)) return raw < 0 ? 0 : raw > fullScale ? fullScale : raw;
  const float value = (static_cast<float>(raw) - c.white) * fullScale /
                      (c.black - c.white);
  return value <= 0 ? 0 : value >= fullScale ? fullScale : static_cast<int>(value + 0.5f);
}
}

#pragma once

namespace tank_on_brake {
// Encoder counters wrap at the native long width.
constexpr long delta(long now, long previous) {
  return static_cast<long>(static_cast<unsigned long>(now) -
                           static_cast<unsigned long>(previous));
}
// Same 5ms / one-pulse tolerance as the existing encoder brake.
constexpr bool atRest(long movement) {
  return movement >= -1 && movement <= 1;
}
constexpr bool finished(long initial, long movement) {
  return atRest(movement) || (initial > 0 && movement < 0) ||
         (initial < 0 && movement > 0);
}
}

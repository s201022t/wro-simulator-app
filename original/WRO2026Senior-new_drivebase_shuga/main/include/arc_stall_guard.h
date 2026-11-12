#pragma once

#include <stdint.h>

// arcの各輪用。countは開始位置からのエンコーダ移動量の絶対値。
// 低出力で停止した輪の最低PWMを引き上げ、同じ走行中は保持する。
class ArcStallGuard {
public:
  static constexpr uint32_t BOOST_INTERVAL_MS = 150;
  static constexpr uint32_t STOP_TIMEOUT_MS = 1000;
  static constexpr long PROGRESS_PULSES = 2;
  static constexpr int BOOST_STEP = 20;

  constexpr ArcStallGuard(uint32_t startMs, int minPower, int maxPower)
    : lastProgressMs_(startMs), lastBoostMs_(startMs),
      floorPower_(minPower < maxPower ? minPower : maxPower),
      maxPower_(maxPower) {}

  constexpr int update(uint32_t nowMs, long count, bool finished, int requestedPower) {
    // 半径0の車輪や目標到達済みの車輪には出力しない。
    if (finished) {
      lastProgressMs_ = nowMs;
      lastBoostMs_ = nowMs;
      progressCount_ = count;
      lastPower_ = 0;
      stalled_ = false;
      return 0;
    }

    // 1パルスの往復ノイズでは監視時間をリセットしない。
    if (count - progressCount_ >= PROGRESS_PULSES) {
      progressCount_ = count;
      lastProgressMs_ = nowMs;
      lastBoostMs_ = nowMs;
    }

    const uint32_t stoppedMs = nowMs - lastProgressMs_;
    stalled_ = stoppedMs >= STOP_TIMEOUT_MS;
    if (stalled_) {
      lastPower_ = 0;
      return 0;
    }

    if (stoppedMs >= BOOST_INTERVAL_MS &&
        static_cast<uint32_t>(nowMs - lastBoostMs_) >= BOOST_INTERVAL_MS) {
      const int base = lastPower_ > floorPower_ ? lastPower_ : floorPower_;
      floorPower_ = base + BOOST_STEP;
      if (floorPower_ > maxPower_) floorPower_ = maxPower_;
      lastBoostMs_ = nowMs;
    }

    lastPower_ = requestedPower > floorPower_ ? requestedPower : floorPower_;
    if (lastPower_ > maxPower_) lastPower_ = maxPower_;
    return lastPower_;
  }

  constexpr bool stalled() const { return stalled_; }

private:
  uint32_t lastProgressMs_;
  uint32_t lastBoostMs_;
  long progressCount_ = 0;
  int floorPower_;
  int maxPower_;
  int lastPower_ = 0;
  bool stalled_ = false;
};

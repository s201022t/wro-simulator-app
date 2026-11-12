#pragma once
#include <Arduino.h>
#include "encoder.h"

// このロボット1台の操作窓口。setup/loopと同じコアで使用する。
// モーター・エンコーダは固定配線の共有ハードウェアなので、1個だけ作る。
class Robot {
public:
  explicit Robot(int startButtonPin = 20, const DriveConfig& config = DriveConfig{})
      : startButtonPin_(startButtonPin), config_(config) {}
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;

  void begin();
  // 出力を止め、ボタン解放→押下を待つ。各30ms安定で確定。
  // 起動時に押されていた場合も、一度離してから押す。
  void waitForStart();

  void setPower(int left, int right);
  void stop();
  // 正負で駆動方向を指定。開始位置からの差の絶対値で停止。
  bool movePulses(int32_t left, int32_t right, int power = 100,
                  unsigned long timeoutMs = 10000);
  bool moveRevolutions(float left, float right, int power = 100,
                       unsigned long timeoutMs = 10000);
  // モーター軸の相対角度（度）。360度=1周。車体の旋回角ではない。
  // 正=前進、負=後退、0=停止。エンコーダの整数カウントに四捨五入する。
  bool moveDegrees(float leftDegrees, float rightDegrees, int power = 100,
                   unsigned long timeoutMs = 10000);

  // Speed magnitudes: mm/s. Signed wheel distances: mm.
  bool drive(float leftMax, float rightMax, float leftMin, float rightMin,
             float leftMm, float rightMm, unsigned long timeoutMs = 30000);
  EncoderCounts counts() const;
  void resetCounts();
  void printResult(Print& output, bool completed) const;

private:
  const int startButtonPin_;
  const DriveConfig config_;
  bool initialized_ = false;
};

#pragma once
#include "drive_config.h"
void encoderSetup(const DriveConfig& config);
#include <stdint.h>

// A/B両相CHANGEの4逓倍後で1周620カウント（暫定）。実測で調整する。
constexpr float ENCODER_PULSES_PER_REV = 620.0f;

struct EncoderCounts {
  int32_t left;
  int32_t right;
};

// motorSetup()の後に、setup()から1回呼ぶ。同一コアで使用する。
void encoderSetup();
EncoderCounts encoderRead();
// 通常処理から呼ぶ。間隔を制限し、送信バッファ不足時は表示を省略する。
void encoderPrintSerial();
void encoderReset();

// 呼び出し位置からの相対移動。正=前進、負=後退、0=その輪は動かさない。
// motorMovePulsesの単位は4逓倍後のカウント数。
// 停止判定: abs(現在値 - 開始値) >= abs(目標)カウント方向には依存しない。
// power: 1～255の固定PWM。各輪が目標に達すると個別に出力OFF。
// 完了まで待機。成功=true、未初期化/引数不正/時間切れ=false（両輪停止）。
// 同期・速度調整・ジャイロ補正・停止位置保持は行わない。
bool motorMovePulses(int32_t left, int32_t right, int power = 100,
                     unsigned long timeoutMs = 10000);
bool motorMoveRevolutions(float left, float right, int power = 100,
                          unsigned long timeoutMs = 10000);

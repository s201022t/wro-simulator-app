#pragma once
#include <stdint.h>
struct DriveConfig {
  int leftMotorA=10, leftMotorB=11, rightMotorA=8, rightMotorB=9;
  int leftEncoderA=28, leftEncoderB=29, rightEncoderA=26, rightEncoderB=27;
  // エンコーダ方向の設定場所。前進時にカウントが増える符号にする。
  int leftEncoderDirection=1, rightEncoderDirection=-1;
  float leftWheelDiameterMm=62.4, rightWheelDiameterMm=62.4;
  float leftCountsPerRev=625, rightCountsPerRev=625;
  float speedKp=0.5f, speedKi=0.8f;
  float leftPwmPerMmS=0.6f, rightPwmPerMmS=0.6f;
  float positionKp=3, syncKp=2, rampTimeSeconds=0.5f;
  float positionToleranceMm=1;
  // Encoder serial output interval (ms). 0 disables output.
  unsigned long encoderPrintIntervalMs=100;
  // この時間内のカウント変化量の合計が閾値以下なら逆相ブレーキを切る。
  unsigned long brakeSampleMs=10;
  int32_t brakeStopCounts=5;
  unsigned long controlPeriodMs=10, settleTimeMs=200;
};

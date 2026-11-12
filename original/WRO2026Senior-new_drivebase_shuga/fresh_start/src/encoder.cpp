#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "encoder.h"
#include "motor.h"

namespace {
int LEFT_A = 26;//26
int LEFT_B = 27;//27
int RIGHT_A = 28;//28
int RIGHT_B = 29;//29
// 実行時の値。encoderSetup()が上書きするため、方向の変更は
// include/drive_config.h の left/rightEncoderDirection で行う。
int LEFT_DIR = 1;
int RIGHT_DIR = -1;
volatile uint32_t leftCount = 0;
volatile uint32_t rightCount = 0;
volatile uint8_t leftState = 0;
volatile uint8_t rightState = 0;
bool ready = false;
unsigned long printIntervalMs=0;
unsigned long lastPrintMs=0;

// AB: 00 -> 01 -> 11 -> 10 -> 00 が+方向（従来のA相判定と同符号）。
// 変化なし、または両ビット同時変化はカウントしない。
constexpr int8_t QUADRATURE_STEPS[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};

uint8_t readState(int pinA, int pinB) {
  return (digitalRead(pinA) << 1) | digitalRead(pinB);
}

void onLeftEdge() {
  const uint8_t now = readState(LEFT_A, LEFT_B);
  leftCount += QUADRATURE_STEPS[(leftState << 2) | now] * LEFT_DIR;
  leftState = now;
}

void onRightEdge() {
  const uint8_t now = readState(RIGHT_A, RIGHT_B);
  rightCount += QUADRATURE_STEPS[(rightState << 2) | now] * RIGHT_DIR;
  rightState = now;
}

int32_t relativeCount(int32_t now, int32_t start) {
  return static_cast<int32_t>(static_cast<uint32_t>(now) - static_cast<uint32_t>(start));
}

bool reached(int32_t progress, int32_t target) {
  // 開始位置からの差の絶対値で判定する。INT32_MINも扱えるよう拡張する。
  return llabs(static_cast<long long>(progress)) >=
         llabs(static_cast<long long>(target));
}
}

void encoderSetup() { encoderSetup(DriveConfig{}); }
void encoderSetup(const DriveConfig& c) {
  if (ready) {
    detachInterrupt(digitalPinToInterrupt(LEFT_A));
    detachInterrupt(digitalPinToInterrupt(LEFT_B));
    detachInterrupt(digitalPinToInterrupt(RIGHT_A));
    detachInterrupt(digitalPinToInterrupt(RIGHT_B));
  }
  printIntervalMs=c.encoderPrintIntervalMs;
  lastPrintMs=millis();
  LEFT_A=c.leftEncoderA; LEFT_B=c.leftEncoderB;
  RIGHT_A=c.rightEncoderA; RIGHT_B=c.rightEncoderB;
  LEFT_DIR=c.leftEncoderDirection; RIGHT_DIR=c.rightEncoderDirection;
  pinMode(LEFT_A, INPUT_PULLUP);
  pinMode(LEFT_B, INPUT_PULLUP);
  pinMode(RIGHT_A, INPUT_PULLUP);
  pinMode(RIGHT_B, INPUT_PULLUP);
  encoderReset();
  attachInterrupt(digitalPinToInterrupt(LEFT_A), onLeftEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_B), onLeftEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), onRightEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_B), onRightEdge, CHANGE);
  ready = true;
}

EncoderCounts encoderRead() {
  noInterrupts();
  const EncoderCounts result{static_cast<int32_t>(leftCount), static_cast<int32_t>(rightCount)};
  interrupts();
  return result;
}

void encoderPrintSerial() {
  if (!ready || !printIntervalMs || !Serial) return;
  const unsigned long now=millis();
  if (now-lastPrintMs<printIntervalMs) return;
  lastPrintMs=now;
  const EncoderCounts c=encoderRead();
  char line[64];
  const int size=snprintf(line,sizeof(line),"ENC L=%ld R=%ld\r\n",
                          static_cast<long>(c.left),static_cast<long>(c.right));
  if (size>0 && size<static_cast<int>(sizeof(line)) && Serial.availableForWrite()>=size)
    Serial.write(reinterpret_cast<const uint8_t*>(line),size);
}

void encoderReset() {
  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  leftState = readState(LEFT_A, LEFT_B);
  rightState = readState(RIGHT_A, RIGHT_B);
  interrupts();
}

bool motorMovePulses(int32_t left, int32_t right, int power, unsigned long timeoutMs) {
  motorStop();
  if (!ready || power < 1 || power > 255 || timeoutMs == 0) return false;
  const EncoderCounts start = encoderRead();
  const unsigned long startedAt = millis();
  bool leftDone = left == 0;
  bool rightDone = right == 0;
  while (true) {
    encoderPrintSerial();
    const EncoderCounts now = encoderRead();
    leftDone = leftDone || reached(relativeCount(now.left, start.left), left);
    rightDone = rightDone || reached(relativeCount(now.right, start.right), right);
    if (leftDone && rightDone) {
      motorStop();
      return true;
    }
    if (millis() - startedAt >= timeoutMs) {
      motorStop();
      return false;
    }
    motorPower(leftDone ? 0 : (left > 0 ? power : -power),
               rightDone ? 0 : (right > 0 ? power : -power));
    delay(1);
  }
}

bool motorMoveRevolutions(float left, float right, int power, unsigned long timeoutMs) {
  const double leftPulses = static_cast<double>(left) * ENCODER_PULSES_PER_REV;
  const double rightPulses = static_cast<double>(right) * ENCODER_PULSES_PER_REV;
  if (!isfinite(leftPulses) || !isfinite(rightPulses) ||
      fabs(leftPulses) > 2147483647.0 || fabs(rightPulses) > 2147483647.0) {
    motorStop();
    return false;
  }
  return motorMovePulses(static_cast<int32_t>(lround(leftPulses)),
                         static_cast<int32_t>(lround(rightPulses)), power, timeoutMs);
}

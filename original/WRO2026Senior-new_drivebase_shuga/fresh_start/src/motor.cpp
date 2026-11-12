#include <Arduino.h>
#include "motor.h"

namespace {
// 既存ロボットの配線と同じGPIO番号。
int LEFT_A = 10;
int LEFT_B = 11;
int RIGHT_A = 8;
int RIGHT_B = 9;

void writeMotor(int pinA, int pinB, int power) {
  power = constrain(power, -255, 255);
  // 反対側の出力を先に0にしてから指定方向に出力する。
  if (power >= 0) {
    analogWrite(pinB, 0);
    analogWrite(pinA, power);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, -power);
  }
}
}

void motorSetup() { motorSetup(DriveConfig{}); }
void motorSetup(const DriveConfig& c) {
  LEFT_A=c.leftMotorA; LEFT_B=c.leftMotorB;
  RIGHT_A=c.rightMotorA; RIGHT_B=c.rightMotorB;
  pinMode(LEFT_A, OUTPUT);
  pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);
  analogWriteRange(255);
  motorStop();
}
void motorPower(int left, int right) {
  writeMotor(LEFT_A, LEFT_B, left);
  writeMotor(RIGHT_A, RIGHT_B, right);
}
void motorStop() { motorPower(0, 0); }

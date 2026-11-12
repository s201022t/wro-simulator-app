#ifndef SERVO_LIB_H
#define SERVO_LIB_H

#include <Arduino.h>

void servoSetup();

// speed: 1～255。範囲外は制限。既接続サーボにS字パルス軌道を適用。
// 完了は目標パルスの出力完了であり、実機の到達確認ではない。
bool startServoMoveByPin(int pin, int targetAngle, int speed);
void updateServoTasks();
// 待機中もS字軌道を更新。停止ボタンの判定は呼び出し側で行う。
void servoDelay(unsigned long durationMs);
void cancelServoTasks();
bool servoTasksBusy();
void waitServoTasks();

void writeBothServos(int leftAngle, int rightAngle);

bool resetServoToAngleByPin(int pin, int angle);
void resetGeekServosToStartAngle();

int getServoDirByPin(int pin);
int applyServoDirByPin(int pin, int angle);

void printServoSettings();

// =====================================================
// アーム制御関数
//
// speed:
//   1～255
//
// sync:
//   1 = サーボ動作完了まで待つ
//   0 = 待たずに次の処理へ進む
// =====================================================

// GP1：物をつかむアーム
void catchArm(
  int angle,
  int speed = 255,
  int sync = 1
);

// GP0：つかむアームを持ち上げる
// 0～75を5度単位で指定する
// 0  = 最大下げ
// 75 = 最大上げ
void liftArm(
  int angle,
  int speed = 255,
  int sync = 1
);

// GP2：機体内部へ格納するアーム
//
// position:
//   0   = 最大閉じる
//   100 = 最大開く
//
// speed:
//   省略時は255
//
// sync:
//   1 = 動作完了まで待つ
//   0 = 待たずに次へ進む
void backArm(
  int position,
  int speed = 255,
  int sync = 1
);
// GP3：後方から降ろして物をつかむアーム
//
// position:
//   0   = 最大上げ
//   100 = 最大下げ
//
// speed:
//   省略時は255
//
// sync:
//   1 = 動作完了まで待つ
//   0 = 待たずに次へ進む
void circleArm(
  int position,
  int speed = 255,
  int sync = 1
);

enum FrontArmOrder {
  F_LIFT,
  F_CATCH
};

void front(
  FrontArmOrder order,
  int firstAngle,
  int secondAngle
);

#endif
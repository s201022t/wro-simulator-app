
#include <Arduino.h>
#include "motion_lib.h"
#include "servo_lib.h"
#include "robot_app.h"
#include "pi_color_lib.h"
#include "move_function.h"
#include "extra.h"

void setup() {
  servoSetup();

  motionSetup();

  piColorSetup();

  // 起動時・リセット時に指定角度へ移動
  resetGeekServosToStartAngle();

  printServoSettings();

  Serial.println("READY BUTTON MODE");
  Serial.println("GYRO I2C: Wire1 GP18=SDA GP19=SCL");

  buzzerBeep(880, 100);
  buzzerBeep(1320, 100);
}


void loop() {
  // GP20ボタンが押されるまで待つ
  waitStartButtonWithGyroCheck();

  Serial.println("BUTTON START");
  robot_reset();
  runCurrentProgram();
  tankOff();  // シーケンス終了時はオンモード走行も停止する。



}


// ここだけを書き換えて競技用シーケンスを作る
// requestColorRead();で色を読む
void runCurrentProgram() {
  motorPower(255, 255);
  // tankbSet(150);
  // tank(-50);
  // spin(90,255);
  // // delay(2000);
  // spin(-90,255);
  // // delay(2000);
  // spin(180,255);
  // // delay(2000);
  // spin(-180,255);
  // // delay(2000);
  // tank(100);
  // tank(-100);
  // tank(500);
  // tank(-500);
  // tline(100, 200, 0);
  // tank(73);
  // extra_1();
  // extra_2();
  // extra_3();
  // extra_4();
  // extra_5();
  // extra_6();
  // extra_7();
  // extra_8();
  // extra_9();
  // extra_10();
  // section_A();
  // section_B();
  // section_C();
  
}

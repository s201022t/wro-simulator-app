#ifndef MOTION_LIB_H
#define MOTION_LIB_H

#include "servo_lib.h"
#include <Arduino.h>

// =====================================================
// ピン定義
// =====================================================

// モーター
extern const int M1A;
extern const int M1B;
extern const int M2A;
extern const int M2B;

// ラインセンサー
extern const int SENSOR_R;
extern const int SENSOR_L;
extern const int SENSOR_R_END;
extern const int SENSOR_L_END;

// ボタン
extern const int START_BUTTON;
extern const int STOP_BUTTON;
extern const int BUTTON;

// エンコーダ
extern const int ENC_L_A;
extern const int ENC_L_B;
extern const int ENC_R_A;
extern const int ENC_R_B;

// MPU9265 I2C
extern const int GYRO_I2C_SDA_PIN;
extern const int GYRO_I2C_SCL_PIN;

// move.ino互換名
extern const int ENC1_A;
extern const int ENC1_B;
extern const int ENC2_A;
extern const int ENC2_B;

// =====================================================
// 初期化
// =====================================================

void motionSetup();
void calibrateGyroZ(int times);

// ジャイロ走行用。処理時間込みの目標周期（125us = 8kHz）。
// I2Cや演算が超過する場合は実行可能な速度となる。実測で判断すること。
constexpr unsigned long MOTION_CONTROL_PERIOD_US = 125;
struct MotionTimingStats {
  uint32_t controlCycles;
  uint32_t controlOverruns;
  uint32_t maxControlUs;
  uint64_t totalControlUs;
  uint32_t gyroIntervals;
  uint32_t maxGyroIntervalUs;
  uint64_t totalGyroIntervalUs;
};
// 同じコアから使用。走行前にreset、停止後にgetして集計する。
void resetMotionTimingStats();
MotionTimingStats getMotionTimingStats();

// =====================================================
// 円弧走行
//
// radiusMm:
//   ロボット中心が描く円の半径 mm
//   0以上。トレッド幅の半分で内輪停止、それ未満で内輪逆転。
//   0はその場旋回。
//   150ms進まない輪はPWMを増加し、1秒進まなければ両輪停止。
//   内輪の低出力は10ms単位のパルスで再現する（初期パルスPWMは77）。
//   最低出力とパルス補正係数はmotion_lib.cppのARC_INNER_PULSE_*で調整。
//   パルスOFF中もストール監視を続ける。平均PWMと実際の速度は一致とは限らない。
//
// deg:
//    正 = 右旋回
//    負 = 左旋回
//
// maxPower:
//   外側タイヤの最大PWM
//   正 = 前進、負 = 後退。degの旋回方向は前進・後退で共通。
//
// minPower:
//   外側タイヤの最低PWM
//
// brake:
//   0 = ブレーキなし
//   1 = 逆位相ブレーキあり
// =====================================================
void arc(
  float radiusMm,
  float deg,
  int maxPower,
  int minPower = 130,
  int brake = 1
);

// =====================================================
// 外から使う走行関数
// =====================================================

void line(float targetMm, int maxSpeed, int brake = 1);

void tline(
  float targetMm,
  int maxSpeed,
  int brake = 1,
  int initial = 100
);

void tlineL(
  float targetMm,
  int maxSpeed,
  int brake = 1,
  int initial = 100
);

void tlineR(
  float targetMm,
  int maxSpeed,
  int brake = 1,
  int initial = 100
);

void ex_tline(
  float targetMm,
  int maxSpeed,
  int brake = 1,
  int initial = 120,
  float extraMm = 65.0f
);

void checkline(int times);
enum RobotSide {
  L,
  R
};

// 両方とも前進の180度arcを2回。右移動は右180度→左180度、左移動は逆順。
// distanceMm: 横移動距離mm（0以上）。半径はdistanceMm / 4で自動計算。
// 終了時の前後位置と向きは元通り。途中では前後へ半径分ずつふくらむ。
// side: R=右、L=左。maxPower: 1〜255。
// 各arcの終端で停止する。STOP・ストール・タイムアウト時はfalseで中断。
// 例: parallelShift(250, R); // 右250mmを目標、前後変位0mm。半径62.5mm。
bool parallelShift(
  float distanceMm,
  RobotSide side,
  int maxPower = 200,
  int minPower = 130
);

//端センサーでのライントレース
void edgeLine(
  RobotSide side,
  float targetMm,
  int maxSpeed,
  int brake = 1,
  int initial = 100
);

// MPU9265基準の片輪pivot
void pivot(
  float deg,
  int Maxspeed,
  RobotSide stopSide,
  int minPower = 110
);


// =====================================================
// RGB LED系統
// =====================================================

void boardRgbSetup();
void boardRgbSet(uint8_t r, uint8_t g, uint8_t b);
void boardRgbOff();
void boardRgbSetByColor(String color);

// =====================================================
// MPU9265 ジャイロ
// =====================================================

bool gyroReady();
void resetYawDeg();
float getYawDeg();

// ジャイロ確認つきスタート待ち
bool checkGyroAlive();
void waitStartButtonWithGyroCheck(unsigned long checkIntervalMs = 500);

// =====================================================
// GP22 ブザー
// =====================================================
extern const int BUZZER_PIN;

void buzzerSetup();

void buzzerOn(unsigned int frequencyHz = 2000);

void buzzerOff();

void buzzerBeep(
  unsigned int frequencyHz = 2000,
  unsigned long durationMs = 100
);

void buzzerBeepCount(
  int count,
  unsigned int frequencyHz = 2000,
  unsigned long onMs = 100,
  unsigned long offMs = 100
);

// =====================================================
// END_L または END_R がしきい値以上になった後、
// Advance_mm 分移動して止まる
// =====================================================

void tankb(int mm, int maxpower, int Advance_mm, int Min_power = 100);

// =====================================================
// ジャイロ補正ありの直進
// 旧 gyrotank
// =====================================================

void tank(
  float mm,
  int maxPower = 200,
  int Min_power = 100,
  int brake = 1
);

// =====================================================
// 壁あて
// =====================================================

void kabe(int power);

// =====================================================
// ジャイロ基準のその場旋回
// 旧 gyroTurnDeg
// =====================================================

void spin(float deg, int maxPower = 220);

// =====================================================
// 必要なら使える補助関数
// =====================================================

bool startPressed();
bool stopPressed();
void test_encoder(unsigned long ms);

void motorBrakeTest();

// =====================================================
// マイブロック
// =====================================================


void robot_reset();



#endif

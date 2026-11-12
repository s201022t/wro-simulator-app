#include "motion_lib.h"
#include "arc_stall_guard.h"
#include "arc_pulse_power.h"
#include "arc_geometry.h"
#include "turn_s_curve.h"
#include "tank_control.h"
#include "turn_progress.h"
#include "servo_lib.h"
#include "pi_color_lib.h"
#include <Wire.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <hardware/watchdog.h>

// =====================================================
// GP22 ブザー
// =====================================================

const int BUZZER_PIN = 22;

// ブザー初期化
void buzzerSetup() {
  pinMode(BUZZER_PIN, OUTPUT);

  noTone(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
}

// 連続して鳴らし始める
// frequencyHz: 周波数 Hz
void buzzerOn(unsigned int frequencyHz) {
  if (frequencyHz < 31) {
    frequencyHz = 31;
  }

  tone(BUZZER_PIN, frequencyHz);
}

// 音を止める
void buzzerOff() {
  noTone(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
}

// 指定時間だけ鳴らす
// frequencyHz: 周波数 Hz
// durationMs : 鳴らす時間 ms
void buzzerBeep(unsigned int frequencyHz, unsigned long durationMs) {
  if (durationMs == 0) {
    return;
  }

  buzzerOn(frequencyHz);
  delay(durationMs);
  buzzerOff();
}

// 指定回数鳴らす
// count      : 鳴らす回数
// frequencyHz: 周波数 Hz
// onMs       : 1回鳴らす時間 ms
// offMs      : 音と音の間隔 ms
void buzzerBeepCount(
  int count,
  unsigned int frequencyHz,
  unsigned long onMs,
  unsigned long offMs
) {
  if (count <= 0) {
    return;
  }

  for (int i = 0; i < count; i++) {
    buzzerBeep(frequencyHz, onMs);

    if (i < count - 1 && offMs > 0) {
      delay(offMs);
    }
  }
}

// =====================================================
// MOTION2350 Pro 基板上 RGB LED
// =====================================================
#define RGB_LED_PIN 23
#define RGB_LED_COUNT 6

Adafruit_NeoPixel boardRgbLed(
  RGB_LED_COUNT,
  RGB_LED_PIN,
  NEO_GRB + NEO_KHZ800);

void boardRgbSetup() {
  boardRgbLed.begin();
  boardRgbLed.setBrightness(30);  // 0〜255。明るすぎる場合は下げる
  boardRgbLed.clear();
  boardRgbLed.show();
}

void boardRgbSet(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < RGB_LED_COUNT; i++) {
    boardRgbLed.setPixelColor(i, boardRgbLed.Color(r, g, b));
  }
  boardRgbLed.show();
}

void boardRgbOff() {
  boardRgbLed.clear();
  boardRgbLed.show();
}

void boardRgbSetByColor(String color) {
  if (color == "G") {
    boardRgbSet(0, 255, 0);  // 緑
  } else if (color == "B") {
    boardRgbSet(0, 0, 255);  // 青
  } else if (color == "W") {
    boardRgbSet(255, 255, 255);  // 白
  } else if (color == "Y") {
    boardRgbSet(255, 180, 0);  // 黄
  } else {
    boardRgbOff();  // UNKNOWNなどは消灯
  }
}
// =====================================================
// モーター
// =====================================================
const int M1A = 8;  // 右モーター
const int M1B = 9;
const int M2A = 10;  // 左モーター
const int M2B = 11;

// =====================================================
// センサー
// =====================================================
const int SENSOR_R_END = 26;  //右端
const int SENSOR_R = 27;
const int SENSOR_L = 28;
const int SENSOR_L_END = 29;  //左端

// =====================================================
// GP20 開始・停止兼用ボタン
// =====================================================
const int START_BUTTON = 20;
const int STOP_BUTTON = 20;
const int BUTTON = 20;

// =====================================================
// エンコーダ
// =====================================================

// 左エンコーダ
const int ENC_L_A = 6;
const int ENC_L_B = 7;

// 右エンコーダ
const int ENC_R_A = 5;
const int ENC_R_B = 4;

// move.ino互換名
// enc1 = 右
// enc2 = 左
const int ENC1_A = 5;
const int ENC1_B = 4;

const int ENC2_A = 6;
const int ENC2_B = 7;

// =====================================================
// エンコーダ向き補正
// =====================================================
const int ENC_L_DIR = 1;
const int ENC_R_DIR = -1;


// =====================================================
// 機体定数
// =====================================================
const float WHEEL_DIAMETER_MM = 62.4;
const float TRACK_WIDTH_MM = 167.0;
const float PULSE_PER_REV = 360.0;

// 従来の100Hz/200Hz制御から、今回500Hz/1000Hzへ上げた倍率。
// 時間刻みへ依存するIIR・I項・差分D項の実時間特性を保つために使う。
const float CONTROL_RATE_SCALE = 5.0f;

// =====================================================
// モーター向き補正
//
// motorPower() に渡す値は
//   正 = 前進
//   負 = 後退
//
// 実際のモーター取り付け方向はここだけで補正する。
// =====================================================
const int LEFT_MOTOR_DIR  = 1;  // M2
const int RIGHT_MOTOR_DIR = 1;   // M1

// =====================================================
// line 用 PID
// =====================================================
const int LINE_PHASE_ACC = 0;
const int LINE_PHASE_CRUISE = 1;
const int LINE_PHASE_DEC = 2;

float LINE_ACC_KP = 0.35f;
float LINE_ACC_KI = 0.000f;
float LINE_ACC_KD = 0.75f;

float LINE_CRUISE_KP = 0.4f;
float LINE_CRUISE_KI = 0.000f;
float LINE_CRUISE_KD = 0.75f;

float LINE_DEC_KP = 0.35f;
float LINE_DEC_KI = 0.000f;
float LINE_DEC_KD = 0.75f;

// ラインセンサー差分に掛ける一次IIRローパスの係数
// filtered = filtered + alpha * (raw - filtered)
// 小さいほど滑らかになるが、ライン変化への応答は遅くなる。
// 100Hz時の0.60とほぼ同じ時定数になる500Hz用の係数。
const float LINE_IIR_ALPHA = 0.167f;

// =====================================================
// END_L / END_R 黒線時専用 PID
// =====================================================
// ENDセンサーが黒線を読んでいる間だけ、このPIDを使う
float LINE_END_BLACK_KP = 0.00f;
float LINE_END_BLACK_KI = 0.000f;
float LINE_END_BLACK_KD = 0.00f;

// =====================================================
// line 用 台形
// =====================================================
const int LINE_MIN_SPEED = 120;
const float LINE_ACC_RATIO = 0.25f;
const float LINE_DEC_RATIO = 0.20f;

// =====================================================
// Tline 用終端判定
//閾値しきいち
//停止判定
// =====================================================
const int END_THRESHOLD = 840; //変更しよう！

// =====================================================
// tank 用 台形
// =====================================================

const float ACC_RATIO = 0.25f;
const float DEC_RATIO = 0.30f;

// =====================================================
// シンクロ制御
// =====================================================
const float SYNC_KP = 16.5f;  //モーター設定
const int SYNC_MAX = 160;//左右補正量のMax値

// =====================================================
// tank 用 ジャイロ直進 PD補正
// =====================================================
// P: 角度ズレへの補正
const float GYRO_TANK_KP = 0.3f;//0.3

// D: 回転速度への補正
// 大きくすると蛇行を抑えやすいが、大きすぎると動きが硬くなる
const float GYRO_TANK_KD = 0.4f;  //0.4

// 左右補正PWMの上限
const int GYRO_TANK_MAX_CORRECTION = 150;

// ログを見たい時だけ true
const bool GYRO_TANK_DEBUG = false;

// =====================================================
// turn / pivot 用
// =====================================================
const int TURN_MIN_POWER = 200;
const int TURN_MAX_POWER = 255;
const float TURN_ACC_RATIO = 0.45f;
const float TURN_DEC_RATIO = 0.45f;

// spinの左右輪同期制御。
// 位置差(P)に加え、1制御周期ごとの速度差(D)も使う。
float SPIN_SYNC_POSITION_KP = 8.0f;
float SPIN_SYNC_SPEED_KD = 12.0f;
const int SPIN_SYNC_MAX = 80;

// 左右モーター固有の強さの差を補うフィードフォワードゲイン。
// 弱い側を1.0より大きく、強い側を1.0より小さくする。
float SPIN_LEFT_MOTOR_GAIN = 1.0f;
float SPIN_RIGHT_MOTOR_GAIN = 1.0f;
float PIVOT_LEFT_MOTOR_GAIN = 1.0f;
float PIVOT_RIGHT_MOTOR_GAIN = 1.0f;
const float TURN_GAIN = 0.86848f;

const int PIVOT_MIN_POWER = 140;
const int PIVOT_MAX_POWER = 255;
const float PIVOT_ACC_RATIO = 0.45f;
const float PIVOT_DEC_RATIO = 0.45f;

// pivot() の速度制御に使う角度→エンコーダパルス換算値。
// これは角度補正係数ではなく、1度あたりのパルス数。
const float PIVOT_PULSE_PER_DEG = 5.0f;

// spin / pivot の角度感度補正。
// 制御目標角度 = 指定角度の絶対値 * ゲイン
// 実際の旋回角が大きい場合は1.0より小さく、
// 小さい場合は1.0より大きくする。
// spinDeg / pivotDeg のエンコーダ代替動作にも同じ値を使用する。
//ゲイン計算方法・・・新しいゲイン = 現在のゲイン × 理想角度 ÷ 実測角度
float SPIN_ANGLE_GAIN = 1.005586592178771f;
float PIVOT_ANGLE_GAIN = 1.002799181860304f;

// =====================================================
// arc 用ジャイロ停止角オフセット
//
// 指定角度からこの値を引いた角度を制御目標にする。
//
// 例:
//   ARC_GYRO_OFFSET_DEG = 3.0
//   arc(300, 90, 200)  → 87度相当を目標
//   arc(300, -90, 200) → 87度相当を目標
// =====================================================
float ARC_GYRO_OFFSET_DEG = 0.0;

// arc内輪の低出力用。PWM 77/255 ≒ 30%。10ms単位で77と0を切り替える。
// ゲインは低出力パルスの頻度を調整する。横移動距離のゲインではない。
// 逆転内輪では大きくすると逆転量が増える傾向がある。実機で調整する。
constexpr int ARC_INNER_PULSE_MIN_POWER = 77;
constexpr float ARC_INNER_PULSE_GAIN = 1.0f;

// =====================================================
// MPU9265用 I2C
//
// 接続:
//   GP18 = SDA
//   GP19 = SCL
//   Wire1を使用
// =====================================================
#define GYRO_I2C Wire1

extern const int GYRO_I2C_SDA_PIN = 18;
extern const int GYRO_I2C_SCL_PIN = 19;

static void updateGyroModeLed();

// =====================================================
// MPU9265設定
// =====================================================

// 検出されたI2Cアドレス
static uint8_t g_mpuAddress = 0x68;

static MotionTimingStats g_motionTiming = {};
static unsigned long g_lastGyroSampleUs = 0;
static bool g_hasGyroSample = false;

void resetMotionTimingStats() {
  g_motionTiming = {};
  g_hasGyroSample = false;
}

MotionTimingStats getMotionTimingStats() {
  return g_motionTiming;
}

static void finishMotionControlCycle(unsigned long startUs) {
  unsigned long elapsedUs = micros() - startUs;
  if (elapsedUs > MOTION_CONTROL_PERIOD_US) {
    ++g_motionTiming.controlOverruns;
  }
  // 処理時間を含めて待つ。超過時に追いつくための連続実行はしない。
  while (elapsedUs < MOTION_CONTROL_PERIOD_US) {
    elapsedUs = micros() - startUs;
  }
  ++g_motionTiming.controlCycles;
  g_motionTiming.totalControlUs += elapsedUs;
  if (elapsedUs > g_motionTiming.maxControlUs) {
    g_motionTiming.maxControlUs = elapsedUs;
  }
}

// MPU9265のI2Cアドレス候補
const uint8_t MPU_ADDRESS_LOW  = 0x68;
const uint8_t MPU_ADDRESS_HIGH = 0x69;

// MPU9265レジスタ
const uint8_t MPU_REG_SMPLRT_DIV   = 0x19;
const uint8_t MPU_REG_CONFIG       = 0x1A;
const uint8_t MPU_REG_GYRO_CONFIG  = 0x1B;
const uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
const uint8_t MPU_REG_INT_STATUS   = 0x3A;
const uint8_t MPU_REG_GYRO_XOUT_H  = 0x43;
const uint8_t MPU_REG_GYRO_ZOUT_H  = 0x47;
const uint8_t MPU_REG_PWR_MGMT_1   = 0x6B;
const uint8_t MPU_REG_PWR_MGMT_2   = 0x6C;
const uint8_t MPU_REG_WHO_AM_I     = 0x75;

// ±500度/秒設定時の感度
// raw値 ÷ 65.5 = degree/second
const float MPU9265_GYRO_SCALE_DPS = 65.5f;

// MPU9265として使用している基板の
// WHO_AM_Iが対応範囲に入っているか確認する
static bool isSupportedMpuWhoAmI(uint8_t whoAmI) {
  return (
    whoAmI == 0x68 ||
    whoAmI == 0x70 ||
    whoAmI == 0x71 ||
    whoAmI == 0x73
  );
}

// 旋回方向が逆の場合は -1.0f に変更する
const float GYRO_YAW_DIR = 1.0f;


// =====================================================
// ジャイロ旋回 PID設定
// =====================================================
float GYRO_TURN_KP = 3.0f;  
float GYRO_TURN_KI = 0.0f;
float GYRO_TURN_KD = 2.0f;  

float GYRO_PIVOT_KP = 5.5f;
float GYRO_PIVOT_KI = 0.0f;
float GYRO_PIVOT_KD = 2.5f;

const float GYRO_YAW_GAIN = 0.97f;

// 目標角度へ戻すための最低PWM
// 停止許容範囲の外では、PID出力が小さすぎても最低PWMを保証する。
// これにより、175度付近で PWM=10 程度になって動けなくなる症状を防ぐ。
const int GYRO_TURN_MIN_POWER = 100;  //←もともと110

// 停止判定
// 目標角 ± GYRO_TURN_TOLERANCE_DEG の範囲内に、
// GYRO_TURN_HOLD_MS 以上いたら停止する
const float GYRO_TURN_TOLERANCE_DEG = 0.7f;

// =====================================================
// ぴったり停止用
//
// 許容範囲に入ってから止めると、慣性で数度オーバーするため、
// 角速度から「今止め始めるべき距離」を予測して早めに停止する。
// =====================================================
const float GYRO_TURN_BRAKE_LEAD_SEC = 0.040f;
const float GYRO_TURN_BRAKE_LEAD_MIN_DEG = 0.8f;
const float GYRO_TURN_BRAKE_LEAD_MAX_DEG = 4.5f;
const float GYRO_TURN_CROSS_STOP_LIMIT_DEG = 2.0f;
const float GYRO_TURN_FINE_RANGE_DEG = 12.0f;
const int GYRO_TURN_FINE_MIN_POWER = 70;

// =====================================================
// 最終合わせ用
//
// 前回式の「角度 + dps + hold」で最終停止するが、
// 許容範囲が狭いと目標付近で最低PWMが強すぎてガタつく。
// そのため、目標付近だけ最低PWMと上限PWMをさらに下げる。
// =====================================================
const float GYRO_TURN_SETTLE_RANGE_DEG = 4.0f;
const int GYRO_TURN_SETTLE_MIN_POWER = 45;
const int GYRO_TURN_SETTLE_MAX_POWER = 65;

const float GYRO_TURN_ULTRA_FINE_RANGE_DEG = 1.2f;
const int GYRO_TURN_ULTRA_FINE_MIN_POWER = 25;
const int GYRO_TURN_ULTRA_FINE_MAX_POWER = 40;

// pivotは片輪旋回で低PWM時にストールしやすいため、
// spinよりも最終減速中の最低PWM/上限PWMを少し高めにする。
const int GYRO_PIVOT_START_MIN_POWER = 125;
const int GYRO_PIVOT_FINE_MIN_POWER = 85;
const int GYRO_PIVOT_SETTLE_MIN_POWER = 65;
const int GYRO_PIVOT_SETTLE_MAX_POWER = 90;
const int GYRO_PIVOT_ULTRA_FINE_MIN_POWER = 65;  //55
const int GYRO_PIVOT_ULTRA_FINE_MAX_POWER = 75;

// 高速旋回時の流れを減らすため、目標角に近づいたらPWM上限を台形的に下げる。
// 値を大きくすると減速開始が早くなる。
// spinはさらに減速開始を遅らせるため、90度なら 90 * 0.25 = 残り22.5度から減速。
const float GYRO_SPIN_DEC_RATIO = 0.33f;

// pivotは前回のまま。片輪旋回でストールしやすいため、ここは下げない。
const float GYRO_PIVOT_DEC_RATIO = 0.40f;

const float GYRO_TURN_DEC_MIN_RANGE_DEG = 12.0f;

// x秒間の設定
// 例：300ms = 0.3秒
//     500ms = 0.5秒
//    1000ms = 1.0秒
const unsigned long GYRO_TURN_HOLD_MS = 150;

// 停止判定用の角速度しきい値
// 角度だけでなく、ほぼ止まっていることも確認してから終了する。
const float GYRO_TURN_STOP_DPS = 8.0f;

const unsigned long GYRO_TURN_TIMEOUT_MS = 1000;

// 旋回中ログ。必要な時だけ true にする。
const bool GYRO_TURN_DEBUG = false;

// 積分リミット
const float GYRO_TURN_INTEGRAL_LIMIT = 130.0f;

// ジャイロ状態
bool g_gyroReady = false;

// MPU9265静止時のZ軸ジャイロraw平均値
float g_gyroZOffset = 0.0f;

// MPU9265のZ軸角速度を積分した相対Yaw角
float g_yawDeg = 0.0f;

// 現在のZ軸角速度 degree/second
float g_gyroZDps = 0.0f;

// 前回Yaw積分時刻
unsigned long g_yawPrevMicros = 0;

// =====================================================
// ブレーキ
// =====================================================
const int BRAKE_POWER = 200;
const int BRAKE_MS = 100;

// gyroTurnDeg 用のその場旋回ブレーキ時間
// PWM量は固定値ではなく、最後に出していたPWMをそのまま反転する
const int GYRO_BRAKE_MS = 75;

// =====================================================
// 安全用
// =====================================================
const unsigned long LOOP_TIMEOUT_MS = 5000;

// =====================================================
// PID用変数
// =====================================================
float integral = 0;
float lastError = 0;
float filteredLineError = 0;
bool lineIirInitialized = false;
int lineLastPhase = -1;
int lineLastEndBlack = -1;
// =====================================================
// 現在モーターPWM記録
// motorPower() / setMotor() で最後に出したPWMを保存する
// 左右それぞれのPWMを個別に保存するので、
// spinTurnDeg のように左右の向きが違う場合にも対応できる。
// pivot のように片側が 0 の場合は、動いている側だけブレーキする。
// =====================================================
static int g_currentLeftPower = 0;
static int g_currentRightPower = 0;

// =====================================================
// エンコーダカウント
// =====================================================
volatile long encL = 0;
volatile long encR = 0;
volatile long encL_total = 0;
volatile long encR_total = 0;
volatile long enc1 = 0;  // 右エンコーダ
volatile long enc2 = 0;  // 左エンコーダ

// =====================================================
// 内部関数プロトタイプ
// =====================================================
static void isrEncL();
static void isrEncR();

static float wheelCircumference();
static long mmToPulse(float mm);
static float pulseToMm(long pulse);

static void resetEncoder();
static long getEncL();
static long getEncR();
static long getTotalEncL();
static long getTotalEncR();
static long getAvgPulse();
static float getAvgMm();

static void resetPID();

static void driveLeftRaw(int pwm);
static void driveRightRaw(int pwm);
static void setMotor(int left, int right);
static void stopMotor();
static void stopMotors();
static void shortBrakeMotors();
static void shortBrakeCurrentMotors();

static int calcTrapezoidSpeed(float targetMm, float nowMm, int maxSpeed);
static int calcTurnTrapezoidPower(long targetPulse, long nowPulse);
static int calcPivotTrapezoidPower(long targetPulse, long nowPulse);
static float smoothStep01(float ratio);

static void lineControlStep(int baseSpeed,
                            float nowMm,
                            float targetMm,
                            int rightValue,
                            int leftValue,
                            int rightEndValue,
                            int leftEndValue,
                            const char* modeName,
                            bool usePhasePid);

static void motorPower(int left, int right);
static void brakeMotors(int dir);
static void brakeSpinMotors(int lastLeftPower, int lastRightPower);
static void brakeCurrentMotors();

static long degToPulse(float deg);
static long pivotDegToPulse(float deg);

static bool mpu9265WriteRegister(
  uint8_t reg,
  uint8_t value
);

static bool mpu9265ReadRegister(
  uint8_t reg,
  uint8_t* value
);

static bool mpu9265ReadRegisters(
  uint8_t startReg,
  uint8_t* buffer,
  size_t length
);

static bool mpu9265ReadRawGyroZ(
  int16_t* rawGyroZ
);

static bool setupGyroMPU9265();

void calibrateGyroZ(int times);

static void updateYawFromGyro();

static bool tryRecoverGyroForNextMove();

// =====================================================
// エンコーダ割り込み
// =====================================================
static void isrEncL() {
  int a = digitalRead(ENC_L_A);
  int b = digitalRead(ENC_L_B);

  long step;

  if (a == b) {
    step = ENC_L_DIR;
  } else {
    step = -ENC_L_DIR;
  }

  encL += step;
  encL_total++;

  // 左エンコーダは enc2
  enc2 += step;
}

static void isrEncR() {
  int a = digitalRead(ENC_R_A);
  int b = digitalRead(ENC_R_B);

  long step;

  if (a == b) {
    step = ENC_R_DIR;
  } else {
    step = -ENC_R_DIR;
  }

  encR += step;
  encR_total++;

  // 右エンコーダは enc1
  enc1 += step;
}

// =====================================================
// 補助関数
// =====================================================
static float wheelCircumference() {
  return WHEEL_DIAMETER_MM * 3.14159265f;
}

static long mmToPulse(float mm) {
  return (long)(mm * PULSE_PER_REV / wheelCircumference());
}

static float pulseToMm(long pulse) {
  return (float)pulse * wheelCircumference() / PULSE_PER_REV;
}

static void resetEncoder() {
  noInterrupts();
  encL = 0;
  encR = 0;
  encL_total = 0;
  encR_total = 0;
  enc1 = 0;
  enc2 = 0;
  interrupts();
}

static long getEncL() {
  noInterrupts();
  long v = encL;
  interrupts();
  return v;
}

static long getEncR() {
  noInterrupts();
  long v = encR;
  interrupts();
  return v;
}

static long getTotalEncL() {
  noInterrupts();
  long v = encL_total;
  interrupts();
  return v;
}

static long getTotalEncR() {
  noInterrupts();
  long v = encR_total;
  interrupts();
  return v;
}

static long getAvgPulse() {
  return (getTotalEncL() + getTotalEncR()) / 2;
}

static float getAvgMm() {
  return pulseToMm(getAvgPulse());
}

static void resetPID() {
  integral = 0;
  lastError = 0;
  filteredLineError = 0;
  lineIirInitialized = false;
  lineLastPhase = -1;
  lineLastEndBlack = -1;
}


static void driveLeftRaw(int pwm) {
  pwm = constrain(pwm, -255, 255);

  // 左モーター(M2)の物理的な取り付け方向を補正
  pwm *= LEFT_MOTOR_DIR;

  if (pwm >= 0) {
    analogWrite(M2A, pwm);
    analogWrite(M2B, 0);
  } else {
    analogWrite(M2A, 0);
    analogWrite(M2B, -pwm);
  }
}

static void driveRightRaw(int pwm) {
  pwm = constrain(pwm, -255, 255);

  // 右モーター(M1)の物理的な取り付け方向を補正
  pwm *= RIGHT_MOTOR_DIR;

  if (pwm >= 0) {
    analogWrite(M1A, pwm);
    analogWrite(M1B, 0);
  } else {
    analogWrite(M1A, 0);
    analogWrite(M1B, -pwm);
  }
}
static void setMotor(int left, int right) {
  motorPower(left, right);
}

static void stopMotor() {
  g_currentLeftPower = 0;
  g_currentRightPower = 0;

  driveLeftRaw(0);
  driveRightRaw(0);
}

static void stopMotors() {
  stopMotor();
}
// =====================================================
// ショートブレーキ
// モーター両端を同電位にして制動する
// =====================================================
static void shortBrakeMotors() {
  // 現在PWM記録を停止状態に更新
  g_currentLeftPower = 0;
  g_currentRightPower = 0;

  // 左モーター：A/B両方HIGH
  analogWrite(M2A, 255);
  analogWrite(M2B, 255);

  // 右モーター：A/B両方HIGH
  analogWrite(M1A, 255);
  analogWrite(M1B, 255);
}

static void shortBrakeCurrentMotors() {
  shortBrakeMotors();
  delay(300);
  stopMotors();
}
// =====================================================
// line
// =====================================================
void line(float targetMm, int maxSpeed, int brake) {
  resetEncoder();
  resetPID();

  long targetPulse = mmToPulse(targetMm);

  while (1) {
    updateServoTasks();
    updateGyroModeLed();

    if (stopPressed()) {
      break;
    }

    long avgPulse = getAvgPulse();
    if (avgPulse >= targetPulse) {
      break;
    }

    int rightValue = analogRead(SENSOR_R);
    int leftValue = analogRead(SENSOR_L);
    int rightEndValue = analogRead(SENSOR_R_END);
    int leftEndValue = analogRead(SENSOR_L_END);

    float nowMm = getAvgMm();
    int baseSpeed = calcTrapezoidSpeed(targetMm, nowMm, maxSpeed);

    lineControlStep(baseSpeed,
                    nowMm,
                    targetMm,
                    rightValue,
                    leftValue,
                    rightEndValue,
                    leftEndValue,
                    "LINE",
                    true);

    delay(2);
  }

  if (brake) {
    brakeCurrentMotors();
  }

  resetPID();
}
// =====================================================
// edgeLine
//
// L: 左端2個でライントレースし、右端センサーで停止
// R: 右端2個でライントレースし、左端センサーで停止
//
// 動作:
//   1. targetMm までは台形制御でライントレース
//   2. targetMm 到達後、LINE_MIN_SPEED でライントレース継続
//   3. 反対側の端センサーが黒線を読んだら停止
// =====================================================

// =====================================================
// edgeLine
//
// 引数 side によって、ライントレースに使うセンサを切り替える。
//
// R:
//   右中央 SENSOR_R と右端 SENSOR_R_END でライントレース
//   左中央 SENSOR_L が黒線を読んだら停止
//
// L:
//   左端 SENSOR_L_END と左中央 SENSOR_L でライントレース
//   右中央 SENSOR_R が黒線を読んだら停止
//
// 動作仕様は tline() と同様:
//
//   1. targetMm までは台形制御でライントレース
//   2. targetMm 到達後、直前速度を維持してライントレース継続
//   3. 停止判定センサが END_THRESHOLD 以上になったら停止
//   4. brake が 1 なら brakeCurrentMotors()
//   5. resetPID()
//
// 関数名・引数:
//   motion_lib.h 側:
//     void edgeLine(RobotSide side, float targetMm, int maxSpeed, int brake = 1);
//
//   motion_lib.cpp 側:
//     void edgeLine(RobotSide side, float targetMm, int maxSpeed, int brake)
//
// =====================================================
void edgeLine(
  RobotSide side,
  float targetMm,
  int maxSpeed,
  int brake,
  int initial
) {
  int leftTraceSensor;
  int rightTraceSensor;
  int stopSensor;

  const char* trapModeName;
  const char* endModeName;

  // =====================================================
  // 1. 引数調整
  // =====================================================
  initial = constrain(
    initial,
    1,
    255
  );

  maxSpeed = constrain(
    maxSpeed,
    initial,
    255
  );

  if (side == R) {
    // =====================================================
    // R指定:
    // 右中央 + 右端でライントレース
    //
    // センサの並び:
    //   左側        右側
    //   SENSOR_R   SENSOR_R_END
    //
    // 停止判定:
    //   左中央 SENSOR_L が黒線を読んだら停止
    // =====================================================
    leftTraceSensor = SENSOR_R;
    rightTraceSensor = SENSOR_R_END;

    stopSensor = SENSOR_L;

    trapModeName = "EDGE-LINE-R-TRAP";
    endModeName = "EDGE-LINE-R-END";

  } else if (side == L) {
    // =====================================================
    // L指定:
    // 左端 + 左中央でライントレース
    //
    // センサの並び:
    //   左側          右側
    //   SENSOR_L_END  SENSOR_L
    //
    // 停止判定:
    //   右中央 SENSOR_R が黒線を読んだら停止
    // =====================================================
    leftTraceSensor = SENSOR_L_END;
    rightTraceSensor = SENSOR_L;

    stopSensor = SENSOR_R;

    trapModeName = "EDGE-LINE-L-TRAP";
    endModeName = "EDGE-LINE-L-END";

  } else {
    stopMotors();
    return;
  }

  resetEncoder();
  resetPID();

  long targetPulse = mmToPulse(targetMm);

  // targetMm到達後に維持する速度
  int holdSpeed = initial;

  // 台形制御の加速・減速距 
  float accDist =
    targetMm * LINE_ACC_RATIO;

  float decDist =
    targetMm * LINE_DEC_RATIO;

  // =====================================================
  // 2. targetMmまでは台形制御でライントレース
  // =====================================================
  while (true) {
    updateServoTasks();
    updateGyroModeLed();

    if (stopPressed()) {
      brakeCurrentMotors();
      resetPID();
      return;
    }

    long avgPulse = getAvgPulse();

    if (avgPulse >= targetPulse) {
      break;
    }

    int leftValue =
      analogRead(leftTraceSensor);

    int rightValue =
      analogRead(rightTraceSensor);

    float nowMm =
      getAvgMm();

    float remainMm =
      targetMm - nowMm;

    int baseSpeed;

    // ===================================================
    // 加速区間
    //
    // initialからmaxSpeedへ加速
    // ===================================================
    if (
      accDist > 0.0f &&
      nowMm < accDist
    ) {
      float ratio =
        nowMm / accDist;

      ratio = smoothStep01(ratio);

      baseSpeed =
        initial +
        static_cast<int>(
          (maxSpeed - initial) *
          ratio
        );

    // ===================================================
    // 減速区間
    //
    // maxSpeedからinitialへ減速
    // ===================================================
    } else if (
      decDist > 0.0f &&
      remainMm < decDist
    ) {
      float ratio =
        remainMm / decDist;

      ratio = smoothStep01(ratio);

      baseSpeed =
        initial +
        static_cast<int>(
          (maxSpeed - initial) *
          ratio
        );

    // ===================================================
    // 巡航区間
    // ===================================================
    } else {
      baseSpeed = maxSpeed;
    }

    baseSpeed = constrain(
      baseSpeed,
      initial,
      maxSpeed
    );

    holdSpeed = baseSpeed;

    // 端寄り2個でライントレースするため、
    // lineControlStep側のEND判定は使わない。
    lineControlStep(
      baseSpeed,
      nowMm,
      targetMm,
      rightValue,
      leftValue,
      0,
      0,
      trapModeName,
      true
    );

    delay(2);
  }

  // =====================================================
  // 3. targetMm到達後、直前速度を維持
  // =====================================================
  holdSpeed = constrain(
    holdSpeed,
    initial,
    255
  );

  while (true) {
    updateServoTasks();
    updateGyroModeLed();

    if (stopPressed()) {
      brakeCurrentMotors();
      resetPID();
      return;
    }

    int leftValue =
      analogRead(leftTraceSensor);

    int rightValue =
      analogRead(rightTraceSensor);

    // edgeLine専用 補正倍率
const float EDGE_GAIN = 0.9f;

int center =
  (leftValue + rightValue) / 2;

leftValue =
  center +
  (int)((leftValue - center) * EDGE_GAIN);

rightValue =
  center +
  (int)((rightValue - center) * EDGE_GAIN);

    int stopValue =
      analogRead(stopSensor);

    // =====================================================
    // 4. 反対側のセンサーが黒線を読んだら停止
    // =====================================================
    if (stopValue >= END_THRESHOLD) {
      break;
    }

    float nowMm =
      getAvgMm();

    lineControlStep(
      holdSpeed,
      nowMm,
      targetMm,
      rightValue,
      leftValue,
      0,
      0,
      endModeName,
      false
    );

    delay(2);
  }

  // =====================================================
  // 5. ブレーキ
  // =====================================================
  if (brake) {
    stopMotors();
    delay(100);
  }

  resetPID();
}

// =====================================================
// Tline停止条件
// =====================================================
enum TlineStopMode {
  TLINE_STOP_EITHER,  // 左右どちらか
  TLINE_STOP_LEFT,    // 左だけ
  TLINE_STOP_RIGHT    // 右だけ
};

// =====================================================
// Tline共通処理
// =====================================================
static void tlineInternal(
  float targetMm,
  int maxSpeed,
  int brake,
  int initial,
  TlineStopMode stopMode
) {
  // =====================================================
  // 1. 引数調整
  // =====================================================
  initial = constrain(
    initial,
    1,
    255
  );

  maxSpeed = constrain(
    maxSpeed,
    initial,
    255
  );

  resetEncoder();
  resetPID();

  long targetPulse = mmToPulse(targetMm);

  // targetMm到達後に維持する速度
  int holdSpeed = initial;

  // 台形制御の加速・減速距離
  float accDist =
    targetMm * LINE_ACC_RATIO;

  float decDist =
    targetMm * LINE_DEC_RATIO;

  // =====================================================
  // 2. 指定距離まで台形制御でライントレース
  // =====================================================
  while (1) {
    updateServoTasks();
    updateGyroModeLed();

    int rightValue =
      analogRead(SENSOR_R);

    int leftValue =
      analogRead(SENSOR_L);

    int rightEndValue =
      analogRead(SENSOR_R_END);

    int leftEndValue =
      analogRead(SENSOR_L_END);

    if (stopPressed()) {
      brakeCurrentMotors();
      resetPID();
      return;
    }

    long avgPulse =
      getAvgPulse();

    if (avgPulse >= targetPulse) {
      break;
    }

    float nowMm =
      getAvgMm();

    float remainMm =
      targetMm - nowMm;

    int baseSpeed;

    // ===================================================
    // 加速区間
    //
    // initial から maxSpeed へ加速
    // ===================================================
    if (
      accDist > 0.0f &&
      nowMm < accDist
    ) {
      float ratio =
        nowMm / accDist;

      ratio = smoothStep01(ratio);

      baseSpeed =
        initial +
        static_cast<int>(
          (maxSpeed - initial) *
          ratio
        );

    // ===================================================
    // 減速区間
    //
    // maxSpeed から initial へ減速
    // ===================================================
    } else if (
      decDist > 0.0f &&
      remainMm < decDist
    ) {
      float ratio =
        remainMm / decDist;

      ratio = smoothStep01(ratio);

      baseSpeed =
        initial +
        static_cast<int>(
          (maxSpeed - initial) *
          ratio
        );

    // ===================================================
    // 巡航区間
    // ===================================================
    } else {
      baseSpeed = maxSpeed;
    }

    baseSpeed = constrain(
      baseSpeed,
      initial,
      maxSpeed
    );

    // 指定距離到達時点の速度を保存
    holdSpeed = baseSpeed;

    lineControlStep(
      baseSpeed,
      nowMm,
      targetMm,
      rightValue,
      leftValue,
      // tlineの端センサーは目標距離到達後の停止判定専用。
      // ここで渡すと、端センサーが黒を読んだ瞬間に
      // LINE_END_BLACK_*（現在は全て0）へ切り替わり、操舵が止まる。
      0,
      0,
      "TLINE-TRAP",
      true
    );

    delay(2);
  }

  // =====================================================
  // 3. 指定距離到達後の速度
  //
  // 台形制御終了時点の速度を維持する
  // 最低でもinitial以上
  // =====================================================
  holdSpeed = constrain(
    holdSpeed,
    initial,
    maxSpeed
  );

  // =====================================================
  // 4. 端センサーが黒線を読むまで継続
  // =====================================================
  while (1) {
    updateServoTasks();
    updateGyroModeLed();

    int rightValue =
      analogRead(SENSOR_R);

    int leftValue =
      analogRead(SENSOR_L);

    int rightEndValue =
      analogRead(SENSOR_R_END);

    int leftEndValue =
      analogRead(SENSOR_L_END);

    if (stopPressed()) {
      brakeCurrentMotors();
      resetPID();
      return;
    }

bool rightBlack =
  rightEndValue >= END_THRESHOLD;

bool leftBlack =
  leftEndValue >= END_THRESHOLD;

bool stopDetected = false;

switch (stopMode) {
  case TLINE_STOP_LEFT:
    // 左端センサーだけで判定
    stopDetected = leftBlack;
    break;

  case TLINE_STOP_RIGHT:
    // 右端センサーだけで判定
    stopDetected = rightBlack;
    break;

  case TLINE_STOP_EITHER:
  default:
    // 左右どちらかが黒なら停止
    stopDetected =
      leftBlack || rightBlack;
    break;
}

if (stopDetected) {
  break;
}

    float nowMm =
      getAvgMm();

    lineControlStep(
      holdSpeed,
      nowMm,
      targetMm,
      rightValue,
      leftValue,
      rightEndValue,
      leftEndValue,
      "TLINE-END",
      false
    );

    delay(2);
  }

  // =====================================================
  // 5. 終了時ブレーキ
  // =====================================================
  if (brake) {
    stopMotors();
  }

  resetPID();
}

// =====================================================
// 左右どちらかの端センサーで停止
//
// 既存のtline()はこの動作。
// デフォルトとして使用する。
// =====================================================
void tline(
  float targetMm,
  int maxSpeed,
  int brake,
  int initial
) {
  tlineInternal(
    targetMm,
    maxSpeed,
    brake,
    initial,
    TLINE_STOP_EITHER
  );
}

// =====================================================
// 左端センサーだけで停止
// =====================================================
void tlineL(
  float targetMm,
  int maxSpeed,
  int brake,
  int initial
) {
  tlineInternal(
    targetMm,
    maxSpeed,
    brake,
    initial,
    TLINE_STOP_LEFT
  );
}

// =====================================================
// 右端センサーだけで停止
// =====================================================
void tlineR(
  float targetMm,
  int maxSpeed,
  int brake,
  int initial
) {
  tlineInternal(
    targetMm,
    maxSpeed,
    brake,
    initial,
    TLINE_STOP_RIGHT
  );
}
// =====================================================
// move 用 モーター出力
// =====================================================
static void motorPower(int left, int right) {
  left = constrain(
    left,
    -255,
    255
  );

  right = constrain(
    right,
    -255,
    255
  );

  // 最後に出したPWMを保存
  g_currentLeftPower = left;
  g_currentRightPower = right;

  driveLeftRaw(
    left
  );

  driveRightRaw(
    right
  );
}

// =====================================================
// ex_tline
//
// 通常の tline と同じ動作を行い、
// ENDセンサーで黒線を検出した後に
// tank() で extraMm だけ追加走行する。
//
// extraMm のデフォルト値は motion_lib.h 側で 73mm。
//
// 動作:
//   1. targetMm まで通常の tline と同じ台形制御
//   2. targetMm 到達後、直前速度でライントレース継続
//   3. 左右どちらかのENDセンサーが黒線を検出
//   4. tank() で extraMm 直進
//   5. 停止
// =====================================================
void ex_tline(
  float targetMm,
  int maxSpeed,
  int brake,
  int initial,
  float extraMm
) {
  // =====================================================
  // 1. 引数調整
  // =====================================================
  initial = constrain(
    initial,
    1,
    255
  );

  maxSpeed = constrain(
    maxSpeed,
    initial,
    255
  );

  if (extraMm < 0.0f) {
    extraMm = 0.0f;
  }

  resetEncoder();
  resetPID();

  long targetPulse = mmToPulse(targetMm);

  // targetMm到達後に維持する速度
  int holdSpeed = initial;

  // 台形制御の加速・減速距離
  float accDist =
    targetMm * LINE_ACC_RATIO;

  float decDist =
    targetMm * LINE_DEC_RATIO;

  // =====================================================
  // 2. 指定距離まで台形制御でライントレース
  // =====================================================
  while (1) {
    updateServoTasks();
    updateGyroModeLed();

    int rightValue =
      analogRead(SENSOR_R);

    int leftValue =
      analogRead(SENSOR_L);

    int rightEndValue =
      analogRead(SENSOR_R_END);

    int leftEndValue =
      analogRead(SENSOR_L_END);

    if (stopPressed()) {
      brakeCurrentMotors();
      resetPID();
      return;
    }

    long avgPulse =
      getAvgPulse();

    if (avgPulse >= targetPulse) {
      break;
    }

    float nowMm =
      getAvgMm();

    float remainMm =
      targetMm - nowMm;

    int baseSpeed;

    // ===================================================
    // 加速区間
    // initial → maxSpeed
    // ===================================================
    if (
      accDist > 0.0f &&
      nowMm < accDist
    ) {
      float ratio =
        nowMm / accDist;

      ratio = smoothStep01(ratio);

      baseSpeed =
        initial +
        static_cast<int>(
          (maxSpeed - initial) *
          ratio
        );

    // ===================================================
    // 減速区間
    // maxSpeed → initial
    // ===================================================
    } else if (
      decDist > 0.0f &&
      remainMm < decDist
    ) {
      float ratio =
        remainMm / decDist;

      ratio = smoothStep01(ratio);

      baseSpeed =
        initial +
        static_cast<int>(
          (maxSpeed - initial) *
          ratio
        );

    // ===================================================
    // 巡航区間
    // ===================================================
    } else {
      baseSpeed = maxSpeed;
    }

    baseSpeed = constrain(
      baseSpeed,
      initial,
      maxSpeed
    );

    // targetMm到達時点の速度を保存
    holdSpeed = baseSpeed;

    lineControlStep(
      baseSpeed,
      nowMm,
      targetMm,
      rightValue,
      leftValue,
      rightEndValue,
      leftEndValue,
      "EX-TLINE-TRAP",
      true
    );

    delay(2);
  }

  // =====================================================
  // 3. targetMm到達後の速度
  // =====================================================
  holdSpeed = constrain(
    holdSpeed,
    initial,
    maxSpeed
  );

  // =====================================================
  // 4. 左右どちらかのENDセンサーが
  //    黒線を読むまでライントレース
  // =====================================================
  while (1) {
    updateServoTasks();
    updateGyroModeLed();

    int rightValue =
      analogRead(SENSOR_R);

    int leftValue =
      analogRead(SENSOR_L);

    int rightEndValue =
      analogRead(SENSOR_R_END);

    int leftEndValue =
      analogRead(SENSOR_L_END);

    if (stopPressed()) {
      brakeCurrentMotors();
      resetPID();
      return;
    }

    bool rightBlack =
      rightEndValue >= END_THRESHOLD;

    bool leftBlack =
      leftEndValue >= END_THRESHOLD;

    // 左右どちらかが黒なら検出
    if (rightBlack || leftBlack) {
      break;
    }

    float nowMm =
      getAvgMm();

    lineControlStep(
      holdSpeed,
      nowMm,
      targetMm,
      rightValue,
      leftValue,
      rightEndValue,
      leftEndValue,
      "EX-TLINE-END",
      false
    );

    delay(2);
  }

  // =====================================================
  // 5. 黒線検出後
  //    tankで extraMm 追加走行
  //
  // Maxpower = initial
  // Min_power = initial
  //
  // → initialを基準に直進
  // =====================================================
  if (extraMm > 0.0f) {
    tank(
      extraMm,
      initial,
      initial,
      brake
    );
  } else {
    // 追加距離0mmの場合
    if (brake) {
      stopMotors();
    }
  }

  resetPID();
}

static void brakeCurrentMotors() {
  int lastLeftPower  = g_currentLeftPower;
  int lastRightPower = g_currentRightPower;

  if (lastLeftPower == 0 && lastRightPower == 0) {
    stopMotors();
    return;
  }

  // 直前PWMと同じ強さを逆方向へ出す
  driveLeftRaw(-lastLeftPower);
  driveRightRaw(-lastRightPower);

  long prevLeft  = getEncL();
  long prevRight = getEncR();

  unsigned long startMs = millis();

  while (millis() - startMs < 100) {
    delay(5);

    long nowLeft  = getEncL();
    long nowRight = getEncR();

    long leftMove  = labs(nowLeft - prevLeft);
    long rightMove = labs(nowRight - prevRight);

    // 5ms間の変化がほぼゼロなら停止
    if (leftMove <= 1 && rightMove <= 1) {
      break;
    }

    prevLeft  = nowLeft;
    prevRight = nowRight;
  }

  stopMotors();
}
// =====================================================
// 旧 brakeMotors 互換
// =====================================================
static void brakeMotors(int dir) {
  (void)dir;
  brakeCurrentMotors();
}

// =====================================================
// spin / pivot 用 逆位相ブレーキ
// =====================================================
static void brakeSpinMotors(int lastLeftPower, int lastRightPower) {
  if (lastLeftPower == 0 && lastRightPower == 0) {
    stopMotors();
    return;
  }

  int brakeLeft = constrain(-lastLeftPower, -255, 255);
  int brakeRight = constrain(-lastRightPower, -255, 255);

  driveLeftRaw(brakeLeft);
  driveRightRaw(brakeRight);

  delay(GYRO_BRAKE_MS);

  stopMotors();
}

bool startPressed() {
  return digitalRead(START_BUTTON) == LOW;
}

bool stopPressed() {
  // GP20が押されていなければ停止要求なし
  if (digitalRead(STOP_BUTTON) != LOW) {
    return false;
  }

  // チャタリング対策
  delay(20);

  if (digitalRead(STOP_BUTTON) != LOW) {
    return false;
  }

  // モーターを直ちに停止
  stopMotors();

  // LEDは消灯
  boardRgbOff();

  // 停止音
  buzzerBeep(2000, 150);

  Serial.println();
  Serial.println("================================");
  Serial.println("GP20 STOP");
  Serial.println("REBOOT TO START WAIT MODE");
  Serial.println("================================");

  // 押したまま再起動すると、再起動後に開始ボタンとして
  // 認識される可能性があるため、離されるまで待つ
  while (digitalRead(STOP_BUTTON) == LOW) {
    delay(10);
  }

  // ボタンを離した直後のチャタリング対策
  delay(100);

  // 念のため再度モーター停止
  stopMotors();

  // 基板を再起動
  watchdog_reboot(0, 0, 0);

  // 再起動されるまで待機
  while (true) {
    delay(10);
  }

  return true;
}
// =====================================================
// MPU9265 生存確認
//
// 確認内容:
//   1. I2Cアドレス0x68へ応答するか
//   2. WHO_AM_Iレジスタが0x68か
//   3. Z軸ジャイロ値を実際に読み取れるか
//
// 正常:
//   trueを返す
//
// 異常:
//   g_gyroReady=false
//   モーター停止
//   LEDを赤
//   falseを返す
// =====================================================
bool checkGyroAlive() {
  // -----------------------------------------------------
  // 1. 検出済みアドレスへの応答確認
  // -----------------------------------------------------
  GYRO_I2C.beginTransmission(g_mpuAddress);

  uint8_t error =
    GYRO_I2C.endTransmission(true);

  if (error != 0) {
    g_gyroReady = false;
    g_gyroZDps = 0.0f;

    boardRgbSet(255, 0, 0);
    stopMotors();

    Serial.print(
      "MPU9265 I2C no response: error="
    );
    Serial.println(error);

    return false;
  }

  // -----------------------------------------------------
  // 2. WHO_AM_I確認
  // -----------------------------------------------------
  uint8_t whoAmI = 0xFF;

  if (!mpu9265ReadRegister(
        MPU_REG_WHO_AM_I,
        &whoAmI
      )) {
    g_gyroReady = false;
    g_gyroZDps = 0.0f;

    boardRgbSet(255, 0, 0);
    stopMotors();

    Serial.println(
      "MPU9265 WHO_AM_I read failed"
    );

    return false;
  }

  if (!isSupportedMpuWhoAmI(whoAmI)) {
    g_gyroReady = false;
    g_gyroZDps = 0.0f;

    boardRgbSet(255, 0, 0);
    stopMotors();

    Serial.print(
      "MPU9265 invalid WHO_AM_I: 0x"
    );

    if (whoAmI < 0x10) {
      Serial.print("0");
    }

    Serial.println(whoAmI, HEX);

    return false;
  }

  // -----------------------------------------------------
  // 3. 実際にZ軸ジャイロを読み取る
  // -----------------------------------------------------
  int16_t rawGyroZ = 0;

  if (!mpu9265ReadRawGyroZ(&rawGyroZ)) {
    g_gyroReady = false;
    g_gyroZDps = 0.0f;

    boardRgbSet(255, 0, 0);
    stopMotors();

    Serial.println(
      "MPU9265 gyro Z read failed"
    );

    return false;
  }

  return true;
}
// =====================================================
// MPU9265通信状態を定期確認して基板LEDへ表示
//
// 緑:
//   MPU9265への通信正常
//   WHO_AM_I正常
//   Z軸ジャイロ読み取り正常
//
// 黄:
//   一時的な通信失敗
//
// 赤:
//   連続して通信に失敗
//
// 250msごとにWHO_AM_IとZ軸ジャイロ値を確認する。
// =====================================================
// =====================================================
// MPU9265通信状態を定期確認して基板LEDへ表示
//
// 緑:
//   MPU9265への通信正常
//   WHO_AM_I正常
//   Z軸ジャイロ読み取り正常
//
// 黄:
//   一時的な通信失敗
//
// 赤:
//   連続して通信に失敗
//
// 250msごとにWHO_AM_IとZ軸ジャイロ値を確認する。
// =====================================================
static void updateGyroModeLed() {
  // 走行中はYaw取得自体が通信確認を兼ねる。診断I2CとLED転送の割込みを避ける。
  if (g_gyroReady && g_hasGyroSample &&
      static_cast<unsigned long>(micros() - g_lastGyroSampleUs) < 10000UL) {
    return;
  }
  static unsigned long lastCheckMs = 0;
  static int consecutiveFailCount = 0;

  const unsigned long CHECK_INTERVAL_MS = 250;
  const int FAIL_LIMIT = 3;

  unsigned long nowMs = millis();

  // 250msごとに1回だけ確認
  if (nowMs - lastCheckMs < CHECK_INTERVAL_MS) {
    return;
  }

  lastCheckMs = nowMs;

  // -----------------------------------------------------
  // 1. WHO_AM_I確認
  // -----------------------------------------------------
  uint8_t whoAmI = 0xFF;

  bool whoAmIOk =
    mpu9265ReadRegister(
      MPU_REG_WHO_AM_I,
      &whoAmI
    );

  if (!whoAmIOk ||
      !isSupportedMpuWhoAmI(whoAmI)) {

    consecutiveFailCount++;

    // 一時的な失敗は黄色
    if (consecutiveFailCount < FAIL_LIMIT) {
      boardRgbSet(255, 180, 0);
    } else {
      // 3回連続失敗で赤
      boardRgbSet(255, 0, 0);
      g_gyroReady = false;
    }

    return;
  }


  // -----------------------------------------------------
  // 2. Z軸ジャイロ値の実読取り確認
  // -----------------------------------------------------
  int16_t rawGyroZ = 0;

  if (!mpu9265ReadRawGyroZ(&rawGyroZ)) {
    consecutiveFailCount++;

    if (consecutiveFailCount < FAIL_LIMIT) {
      boardRgbSet(255, 180, 0);
    } else {
      boardRgbSet(255, 0, 0);
      g_gyroReady = false;
    }

    return;
  }

  // -----------------------------------------------------
  // 3. 正常
  // -----------------------------------------------------
  consecutiveFailCount = 0;

  if (g_gyroReady) {
    boardRgbSet(0, 255, 0);
  } else {
    // センサー自体は応答しているが、
    // 初期化またはキャリブレーション未完了
    boardRgbSet(255, 180, 0);
  }
}

void waitStartButtonWithGyroCheck(unsigned long checkIntervalMs) {

  Serial.println("WAIT START BUTTON WITH GYRO CHECK");

  unsigned long lastCheckMs = 0;

  // 一時的なI2C失敗だけで赤へ切り替えないためのカウンタ
  int consecutiveFailCount = 0;
  const int FAIL_LIMIT = 3;

  // 待機開始時の表示
  if (g_gyroReady) {
    boardRgbSet(0, 255, 0);
  } else {
    boardRgbSet(255, 180, 0);
  }

  while (!startPressed()) {
    updateServoTasks();

    unsigned long nowMs = millis();

    if (nowMs - lastCheckMs >= checkIntervalMs) {
      lastCheckMs = nowMs;

      bool alive = checkGyroAlive();

      if (alive) {
        consecutiveFailCount = 0;

        if (!g_gyroReady) {
          Serial.println("GYRO NOT READY: initialization retry");

          if (setupGyroMPU9265()) {
            calibrateGyroZ(500);
          }
        }

        if (g_gyroReady) {
          boardRgbSet(0, 255, 0);
        } else {
          boardRgbSet(255, 180, 0);
        }

      } else {
        consecutiveFailCount++;

        Serial.print("GYRO CHECK FAIL COUNT=");
        Serial.println(consecutiveFailCount);

        if (consecutiveFailCount >= FAIL_LIMIT) {
          boardRgbSet(255, 0, 0);
        }
      }
    }

    delay(10);
  }

  // チャタリング対策
  delay(20);

  Serial.println("START BUTTON PRESSED: gyro check end");

  boardRgbOff();

  // 開始操作で押したGP20が離されるまで待つ。
  // 離す前に走行を開始すると、その押下を停止操作として
  // 誤検出するため必須。
  while (startPressed()) {
    delay(10);
  }

  delay(100);
}

// =====================================================
// line 用 台形速度計算
// =====================================================
static int calcTrapezoidSpeed(float targetMm, float nowMm, int maxSpeed) {
  float accDist = targetMm * LINE_ACC_RATIO;
  float decDist = targetMm * LINE_DEC_RATIO;
  float remain = targetMm - nowMm;

  if (maxSpeed < LINE_MIN_SPEED) maxSpeed = LINE_MIN_SPEED;

  if (accDist > 0 && nowMm < accDist) {
    float ratio = nowMm / accDist;
    ratio = smoothStep01(ratio);
    return LINE_MIN_SPEED + (int)((maxSpeed - LINE_MIN_SPEED) * ratio);
  }

  if (decDist > 0 && remain < decDist) {
    float ratio = remain / decDist;
    ratio = smoothStep01(ratio);
    int speed = LINE_MIN_SPEED + (int)((maxSpeed - LINE_MIN_SPEED) * ratio);
    return constrain(speed, LINE_MIN_SPEED, maxSpeed);
  }

  return maxSpeed;
}

// =====================================================
// line / Tline 共通 1ステップ制御
// =====================================================
// =====================================================
// line / Tline 共通 1ステップ制御
//
// tank() と同じモーター方向に修正した版
//
// 前進:
//   左モーター = 負
//   右モーター = 正
//
// 今回の修正:
//   補正方向が逆だったため、error の向きを反転
//
// 注意:
//   ここでは LINE_LEFT_DIR / LINE_RIGHT_DIR は使わない。
// =====================================================

static void lineControlStep(int baseSpeed,
                            float nowMm,
                            float targetMm,
                            int rightValue,
                            int leftValue,
                            int rightEndValue,
                            int leftEndValue,
                            const char* modeName,
                            bool usePhasePid) {
  (void)modeName;  // Serial表示を削除したため、未使用警告対策

  // =====================================================
  // ラインのズレ量
  //
  // error > 0 : 右センサー値が大きい
  // error < 0 : 左センサー値が大きい
  //
  // 左センサーが黒線を読んだときは error < 0 とし、
  // 左モーターを減速させて左へ戻す。
  // =====================================================
  float rawError = (float)(rightValue - leftValue);

  // 初回だけ実測値で初期化し、開始直後にゼロから追従する遅れを防ぐ。
  if (!lineIirInitialized) {
    filteredLineError = rawError;
    lineIirInitialized = true;
  } else {
    filteredLineError += LINE_IIR_ALPHA * (rawError - filteredLineError);
  }

  float error = filteredLineError;

  int phase = LINE_PHASE_CRUISE;
  float kp = LINE_CRUISE_KP;
  float ki = LINE_CRUISE_KI;
  float kd = LINE_CRUISE_KD;

  // =====================================================
  // 加速・等速・減速フェーズごとにPIDを切り替える
  // =====================================================
  if (usePhasePid) {
    float accDist = targetMm * LINE_ACC_RATIO;
    float decDist = targetMm * LINE_DEC_RATIO;
    float remain = targetMm - nowMm;

    if (accDist > 0 && nowMm < accDist) {
      phase = LINE_PHASE_ACC;
      kp = LINE_ACC_KP;
      ki = LINE_ACC_KI;
      kd = LINE_ACC_KD;
    } else if (decDist > 0 && remain < decDist) {
      phase = LINE_PHASE_DEC;
      kp = LINE_DEC_KP;
      ki = LINE_DEC_KI;
      kd = LINE_DEC_KD;
    } else {
      phase = LINE_PHASE_CRUISE;
      kp = LINE_CRUISE_KP;
      ki = LINE_CRUISE_KI;
      kd = LINE_CRUISE_KD;
    }
  }

  // =====================================================
  // END_L または END_R が黒線を読んでいる間は専用PIDに切り替える
  // =====================================================
  bool endBlack = (rightEndValue >= END_THRESHOLD || leftEndValue >= END_THRESHOLD);

  if (endBlack) {
    kp = LINE_END_BLACK_KP;
    ki = LINE_END_BLACK_KI;
    kd = LINE_END_BLACK_KD;
  }

  int endBlackMode = endBlack ? 1 : 0;

  // =====================================================
  // フェーズまたはEND黒線状態が変わったらPID内部値をリセット
  // =====================================================
  if (lineLastPhase != phase || lineLastEndBlack != endBlackMode) {
    integral = 0;
    lastError = error;
    lineLastPhase = phase;
    lineLastEndBlack = endBlackMode;
  }

  // =====================================================
  // PID計算
  // =====================================================
  // 制御周波数を5倍にしても、実時間あたりのI項・D項を従来相当に保つ。
  integral += error / CONTROL_RATE_SCALE;
  integral = constrain(integral, -5000.0f, 5000.0f);

  float derivative =
    (error - lastError) * CONTROL_RATE_SCALE;
  float turn = kp * error + ki * integral + kd * derivative;

  lastError = error;

  // =====================================================
  // 左右の出力強さを計算
  //
  // leftAbsPower / rightAbsPower は 0〜255 の強さ
  // =====================================================
  int leftAbsPower = baseSpeed;
  int rightAbsPower = baseSpeed;

  if (turn > 0) {
    rightAbsPower = baseSpeed - (int)turn;
  } else {
    leftAbsPower = baseSpeed + (int)turn;
  }

  leftAbsPower = constrain(leftAbsPower, 0, 255);
  rightAbsPower = constrain(rightAbsPower, 0, 255);

  // =====================================================
  // tank() と同じモーター方向
  //
  // 前進:
  //   左 = 負
  //   右 = 正
  // =====================================================
  int leftMotor = leftAbsPower;
  int rightMotor = rightAbsPower;

  leftMotor = constrain(leftMotor, -255, 255);
  rightMotor = constrain(rightMotor, -255, 255);

  motorPower(leftMotor, rightMotor);
}

// =====================================================
// turn 用 台形速度計算
// =====================================================
static int calcTurnTrapezoidPower(long targetPulse, long nowPulse) {
  long accDist = lround(targetPulse * TURN_ACC_RATIO);
  long decDist = lround(targetPulse * TURN_DEC_RATIO);
  long remain = targetPulse - nowPulse;

  if (accDist < 1) accDist = 1;
  if (decDist < 1) decDist = 1;

  if (targetPulse <= 0) return TURN_MIN_POWER;

  if (nowPulse < accDist) {
    float ratio = nowPulse / (float)accDist;
    ratio = smoothStep01(ratio);
    int power = TURN_MIN_POWER + (int)((TURN_MAX_POWER - TURN_MIN_POWER) * ratio);
    return constrain(power, TURN_MIN_POWER, TURN_MAX_POWER);
  }

  if (remain < decDist) {
    float ratio = remain / (float)decDist;
    ratio = smoothStep01(ratio);
    int power = TURN_MIN_POWER + (int)((TURN_MAX_POWER - TURN_MIN_POWER) * ratio);
    return constrain(power, TURN_MIN_POWER, TURN_MAX_POWER);
  }

  return TURN_MAX_POWER;
}

// =====================================================
// pivot 用 台形速度計算
// =====================================================
static int calcPivotTrapezoidPower(long targetPulse, long nowPulse) {
  long accDist = lround(targetPulse * PIVOT_ACC_RATIO);
  long decDist = lround(targetPulse * PIVOT_DEC_RATIO);
  long remain = targetPulse - nowPulse;

  if (accDist < 1) accDist = 1;
  if (decDist < 1) decDist = 1;

  if (targetPulse <= 0) return PIVOT_MIN_POWER;

  if (nowPulse < accDist) {
    float ratio = nowPulse / (float)accDist;
    ratio = smoothStep01(ratio);
    int power = PIVOT_MIN_POWER + (int)((PIVOT_MAX_POWER - PIVOT_MIN_POWER) * ratio);
    return constrain(power, PIVOT_MIN_POWER, PIVOT_MAX_POWER);
  }

  if (remain < decDist) {
    float ratio = remain / (float)decDist;
    ratio = smoothStep01(ratio);
    int power = PIVOT_MIN_POWER + (int)((PIVOT_MAX_POWER - PIVOT_MIN_POWER) * ratio);
    return constrain(power, PIVOT_MIN_POWER, PIVOT_MAX_POWER);
  }

  return PIVOT_MAX_POWER;
}


// =====================================================
// tank: gyro heading PD with encoder synchronization.
// Both wheel commands respect the time/distance PWM envelope.
// Positive motorPower commands mean forward; dir handles reverse travel.
// =====================================================
void tank(float mm, int Maxpower, int Min_power, int brake) {
  const int maxPower = tank_control::magnitude(Maxpower);
  if (!isfinite(mm) || fabs(mm) < 0.1f || maxPower == 0) {
    stopMotors();
    return;
  }
  const int minPower = tank_control::minimum(Min_power, maxPower);
  const int wheelFloor = min(50, minPower / 2);
  const int dir = mm > 0.0f ? 1 : -1;
  const long targetPulse = mmToPulse(fabs(mm));
  if (targetPulse <= 0 || !tryRecoverGyroForNextMove()) {
    stopMotors();
    return;
  }

  const float targetYaw = getYawDeg();
  if (!g_gyroReady) {
    stopMotors();
    return;
  }

  resetEncoder();
  const unsigned long startMs = millis();
  const unsigned long timeoutMs = 3000UL + static_cast<unsigned long>(fabs(mm) * 15.0f);
  bool applyBrake = false;
  while (true) {
    const unsigned long cycleUs = micros();
    updateServoTasks();
    updateGyroModeLed();
    if (stopPressed()) {
      applyBrake = true;
      break;
    }
    if (millis() - startMs >= timeoutMs) {
      Serial.println("tank: timeout");
      applyBrake = true;
      break;
    }

    long leftCount;
    long rightCount;
    noInterrupts();
    leftCount = labs(enc2);
    rightCount = labs(enc1);
    interrupts();
    const long pos = (leftCount + rightCount) / 2;
    if (pos >= targetPulse) {
      applyBrake = brake > 0;
      break;
    }

    const float currentYaw = getYawDeg();
    if (!g_gyroReady) {
      // getYawDeg() already stops on read failure. Never reapply drive PWM.
      applyBrake = true;
      break;
    }
    const float elapsedSec = (millis() - startMs) * 0.001f;
    const int ceiling = static_cast<int>(tank_control::envelope(
        elapsedSec, static_cast<float>(targetPulse), static_cast<float>(pos),
        minPower, maxPower));

    const float yawError = currentYaw - targetYaw;
    // Restore the pre-refactor balance after curved travel was reported in
    // both directions. The physical gyro polarity still needs hardware validation.
    const int sync = constrain(
        static_cast<int>((leftCount - rightCount) * SYNC_KP), -SYNC_MAX, SYNC_MAX);
    const int heading = constrain(
        static_cast<int>((GYRO_TANK_KP * yawError + GYRO_TANK_KD * g_gyroZDps) * dir),
        -GYRO_TANK_MAX_CORRECTION, GYRO_TANK_MAX_CORRECTION);
    const int correction = constrain(sync + heading,
        -GYRO_TANK_MAX_CORRECTION, GYRO_TANK_MAX_CORRECTION);
    const tank_control::Wheels wheels = tank_control::mix(ceiling, wheelFloor, correction);
    motorPower(wheels.left * dir, wheels.right * dir);
    finishMotionControlCycle(cycleUs);
  }

  if (applyBrake) {
    // Passive short braking cannot turn into an unintended reverse drive.
    // Keep servo tasks alive during the bounded settling interval.
    shortBrakeMotors();
    const unsigned long brakeStart = millis();
    while (millis() - brakeStart < 60UL) {
      updateServoTasks();
      delay(1);
    }
  }
  stopMotors();
  delay(20);
}
void tankb(
  int mm,
  int maxPower,
  int advanceMm,
  int minPower
) {
  // =====================================================
  // 1. 引数と方向
  // =====================================================
  int dir = (mm >= 0) ? 1 : -1;
  int advanceDir = (advanceMm >= 0) ? 1 : -1;

  int targetMm = abs(mm);
  int targetAdvanceMm = abs(advanceMm);

  if (maxPower < 0) {
    maxPower = -maxPower;
  }

  if (minPower < 0) {
    minPower = -minPower;
  }

  minPower = constrain(
    minPower,
    1,
    255
  );

  maxPower = constrain(
    maxPower,
    minPower,
    255
  );

  const int TANKB_WHEEL_MIN_POWER = 50;

  int wheelMinPower = constrain(
    TANKB_WHEEL_MIN_POWER,
    0,
    maxPower
  );

  // =====================================================
  // 2. 距離をパルスへ変換
  // =====================================================
  long targetPulse = labs(
    mmToPulse(targetMm)
  );

  long advanceTargetPulse = labs(
    mmToPulse(targetAdvanceMm)
  );

  // =====================================================
  // 3. ジャイロの確認
  //
  // 復旧できなくてもtankb自体は中止せず、
  // エンコーダ同期補正だけで続行する。
  // =====================================================
  bool useGyro =
    tryRecoverGyroForNextMove();

  float targetYaw = 0.0f;

  if (useGyro) {
    targetYaw = getYawDeg();

    if (!g_gyroReady) {
      useGyro = false;
      g_gyroZDps = 0.0f;
    }
  }

  // =====================================================
  // ジャイロ補正計算
  //
  // moveDir:
  //   1  = 前進
  //  -1  = 後退
  //
  // 走行途中でMPU9265が読めなくなった場合は、
  // その後は0を返してエンコーダ補正のみで続行する。
  // =====================================================
  auto calculateGyroCorrection =
    [&](int moveDir) -> int {

    if (!useGyro) {
      return 0;
    }

    float currentYaw = getYawDeg();

    if (!g_gyroReady) {
      useGyro = false;
      g_gyroZDps = 0.0f;
      return 0;
    }

    // MPU9265の積分角なので、
    // 0～360度の折り返し処理は不要
    float yawError =
      currentYaw - targetYaw;

    float gyroPd =
      GYRO_TANK_KP * yawError +
      GYRO_TANK_KD * g_gyroZDps;

    int correction =
      static_cast<int>(
        gyroPd * moveDir
      );

    return constrain(
      correction,
      -GYRO_TANK_MAX_CORRECTION,
      GYRO_TANK_MAX_CORRECTION
    );
  };

  // =====================================================
  // 4. 第1段階
  // mmで指定した距離を台形制御で進む
  // =====================================================
  if (targetPulse > 0) {
    long accPulse = lround(
      targetPulse * ACC_RATIO
    );

    long decPulse = lround(
      targetPulse * DEC_RATIO
    );

    if (accPulse < 1) {
      accPulse = 1;
    }

    if (decPulse < 1) {
      decPulse = 1;
    }

    long cruisePulse =
      targetPulse -
      accPulse -
      decPulse;

    // 短距離で加速区間＋減速区間が
    // 目標距離を超える場合に再配分
    if (cruisePulse < 0) {
      long totalRamp =
        accPulse + decPulse;

      if (totalRamp > 0) {
        accPulse =
          static_cast<long>(
            static_cast<float>(targetPulse) *
            (
              static_cast<float>(accPulse) /
              static_cast<float>(totalRamp)
            )
          );

        decPulse =
          targetPulse - accPulse;
      }

      if (accPulse < 1) {
        accPulse = 1;
      }

      if (decPulse < 1) {
        decPulse = 1;
      }

      cruisePulse = 0;
    }

    resetEncoder();

    unsigned long startTime =
      millis();

    while (true) {

      const unsigned long cycleUs = micros();
      updateServoTasks();
      updateGyroModeLed();

      if (stopPressed()) {
        brakeCurrentMotors();
        return;
      }

      if (
        millis() - startTime >
        LOOP_TIMEOUT_MS
      ) {
        brakeCurrentMotors();
        return;
      }

      long leftPulse;
      long rightPulse;

      noInterrupts();

      leftPulse = labs(enc2);
      rightPulse = labs(enc1);

      interrupts();

      long currentPulse =
        (leftPulse + rightPulse) / 2;

      if (currentPulse >= targetPulse) {
        break;
      }

      int basePower;

      if (currentPulse < accPulse) {
        float ratio =
          currentPulse /
          static_cast<float>(accPulse);

        ratio = smoothStep01(ratio);

        basePower =
          minPower +
          static_cast<int>(
            (maxPower - minPower) *
            ratio
          );

      } else if (
        currentPulse <
        accPulse + cruisePulse
      ) {
        basePower = maxPower;

      } else {
        long remainPulse =
          targetPulse - currentPulse;

        if (remainPulse < 0) {
          remainPulse = 0;
        }

        float ratio =
          remainPulse /
          static_cast<float>(decPulse);

        ratio = smoothStep01(ratio);

        basePower =
          minPower +
          static_cast<int>(
            (maxPower - minPower) *
            ratio
          );
      }

      basePower = constrain(
        basePower,
        minPower,
        maxPower
      );

      // -------------------------------------------------
      // エンコーダ左右同期補正
      // -------------------------------------------------
      long syncError =
        leftPulse - rightPulse;

      int syncCorrection =
        static_cast<int>(
          syncError * SYNC_KP
        );

      syncCorrection = constrain(
        syncCorrection,
        -SYNC_MAX,
        SYNC_MAX
      );

      // -------------------------------------------------
      // MPU9265直進補正
      // -------------------------------------------------
      int gyroCorrection =
        calculateGyroCorrection(dir);

      int totalCorrection =
        syncCorrection +
        gyroCorrection;

      totalCorrection = constrain(
        totalCorrection,
        -GYRO_TANK_MAX_CORRECTION,
        GYRO_TANK_MAX_CORRECTION
      );

      int leftAbsPower =
       basePower - totalCorrection;

      int rightAbsPower =
       basePower + totalCorrection;

      leftAbsPower = constrain(
      leftAbsPower,
      wheelMinPower,
      maxPower
);

rightAbsPower = constrain(
  rightAbsPower,
  wheelMinPower,
  maxPower
);

      int leftPower =
        leftAbsPower * dir;

      int rightPower =
        rightAbsPower * dir;

      motorPower(
        constrain(leftPower, -255, 255),
        constrain(rightPower, -255, 255)
      );

      finishMotionControlCycle(cycleUs);
    }
  }

  // =====================================================
  // 5. 第2段階
  // ENDセンサーのどちらかが黒線を読むまで進む
  // =====================================================
  resetEncoder();

  unsigned long startTime =
    millis();

  while (true) {

    const unsigned long cycleUs = micros();
    updateServoTasks();
    updateGyroModeLed();

    if (stopPressed()) {
      brakeCurrentMotors();
      return;
    }

    if (
      millis() - startTime >
      LOOP_TIMEOUT_MS
    ) {
      brakeCurrentMotors();
      return;
    }

    int rightEndValue =
      analogRead(SENSOR_R_END);

    int leftEndValue =
      analogRead(SENSOR_L_END);

    if (
      rightEndValue >= END_THRESHOLD ||
      leftEndValue >= END_THRESHOLD
    ) {
      break;
    }

    long leftPulse;
    long rightPulse;

    noInterrupts();

    leftPulse = labs(enc2);
    rightPulse = labs(enc1);

    interrupts();

    int basePower = minPower;

    long syncError =
      leftPulse - rightPulse;

    int syncCorrection =
      static_cast<int>(
        syncError * SYNC_KP
      );

    syncCorrection = constrain(
      syncCorrection,
      -SYNC_MAX,
      SYNC_MAX
    );

    int gyroCorrection =
      calculateGyroCorrection(dir);

    int totalCorrection =
      syncCorrection +
      gyroCorrection;

    totalCorrection = constrain(
      totalCorrection,
      -GYRO_TANK_MAX_CORRECTION,
      GYRO_TANK_MAX_CORRECTION
    );

    int leftAbsPower =
      basePower - totalCorrection;

    int rightAbsPower =
      basePower + totalCorrection;

    leftAbsPower = constrain(
      leftAbsPower,
      wheelMinPower,
      maxPower
    );

    rightAbsPower = constrain(
      rightAbsPower,
      wheelMinPower,
      maxPower
    );

    int leftPower =
      leftAbsPower * dir;

    int rightPower =
      rightAbsPower * dir;

    motorPower(
      constrain(leftPower, -255, 255),
      constrain(rightPower, -255, 255)
    );

    finishMotionControlCycle(cycleUs);
  }

  // =====================================================
  // 6. advanceMmが0なら黒線位置で終了
  // =====================================================
  if (advanceTargetPulse <= 0) {
    brakeCurrentMotors();
    return;
  }

  // =====================================================
  // 7. 黒線検出後に方向が変わる場合
  //
  // 例:
  //   tankb(100, 150, -50, 100)
  //
  // 最初は前進し、黒線検出後は後退する。
  // =====================================================
  if (dir != advanceDir) {
    brakeCurrentMotors();
    delay(10);

    // 方向変更後の向きを新しい直進基準にする
    if (useGyro) {
      resetYawDeg();

      if (g_gyroReady) {
        targetYaw = getYawDeg();
      } else {
        useGyro = false;
        g_gyroZDps = 0.0f;
      }
    }
  }

  // =====================================================
  // 8. 第3段階
  // 黒線検出後、advanceMm分だけ進む
  // =====================================================
  resetEncoder();

  startTime = millis();

  while (true) {

    const unsigned long cycleUs = micros();
    updateServoTasks();
    updateGyroModeLed();

    if (stopPressed()) {
      brakeCurrentMotors();
      return;
    }

    if (
      millis() - startTime >
      LOOP_TIMEOUT_MS
    ) {
      brakeCurrentMotors();
      return;
    }

    long leftPulse;
    long rightPulse;

    noInterrupts();

    leftPulse = labs(enc2);
    rightPulse = labs(enc1);

    interrupts();

    long currentPulse =
      (leftPulse + rightPulse) / 2;

    if (
      currentPulse >=
      advanceTargetPulse
    ) {
      brakeCurrentMotors();
      break;
    }

    int basePower = minPower;

    long syncError =
      leftPulse - rightPulse;

    int syncCorrection =
      static_cast<int>(
        syncError * SYNC_KP
      );

    syncCorrection = constrain(
      syncCorrection,
      -SYNC_MAX,
      SYNC_MAX
    );

    int gyroCorrection =
      calculateGyroCorrection(
        advanceDir
      );

    int totalCorrection =
      syncCorrection +
      gyroCorrection;

    totalCorrection = constrain(
      totalCorrection,
      -GYRO_TANK_MAX_CORRECTION,
      GYRO_TANK_MAX_CORRECTION
    );

    int leftAbsPower =
      basePower - totalCorrection;

    int rightAbsPower =
      basePower + totalCorrection;

    leftAbsPower = constrain(
      leftAbsPower,
      wheelMinPower,
      maxPower
    );

    rightAbsPower = constrain(
      rightAbsPower,
      wheelMinPower,
      maxPower
    );

    int leftPower =
      leftAbsPower * advanceDir;

    int rightPower =
      rightAbsPower * advanceDir;

    motorPower(
      constrain(leftPower, -255, 255),
      constrain(rightPower, -255, 255)
    );

    finishMotionControlCycle(cycleUs);
  }

  stopMotors();
  delay(20);
}

// 0.0～1.0の直線的な進行率をS字へ変換する。
// 始端と終端で傾きが0になるため、PWMの急変を抑えられる。
static float smoothStep01(float ratio) {
  ratio = constrain(ratio, 0.0f, 1.0f);
  return ratio * ratio * (3.0f - 2.0f * ratio);
}

// =====================================================
// turn 用
// =====================================================
static long degToPulse(float deg) {
  if (TRACK_WIDTH_MM <= 0.0f) return 0;
  if (WHEEL_DIAMETER_MM <= 0.0f) return 0;

  float turnCircumference = TRACK_WIDTH_MM * 3.14159265f;
  float moveMm = turnCircumference * (fabs(deg) / 360.0f);
  float wheelCirc = WHEEL_DIAMETER_MM * 3.14159265f;

  return lround(moveMm * PULSE_PER_REV / wheelCirc);
}

// =====================================================
// pivot 用
// =====================================================
static long pivotDegToPulse(float deg) {
  // 指定角度へ比例ゲインを掛けてからパルスへ変換する。
  float targetDeg = fabs(deg) * PIVOT_ANGLE_GAIN;

  if (targetDeg < 0.1f) {
    targetDeg = 0.1f;
  }

  return lround(targetDeg * PIVOT_PULSE_PER_DEG);
}


// =====================================================
// MPU9265 レジスタ書き込み
// =====================================================
static bool mpu9265WriteRegister(uint8_t reg, uint8_t value) {
  GYRO_I2C.beginTransmission(g_mpuAddress);

  GYRO_I2C.write(reg);
  GYRO_I2C.write(value);

  uint8_t error =
    GYRO_I2C.endTransmission(true);

  return error == 0;
}
// =====================================================
// MPU9265 レジスタ1バイト読み取り
// =====================================================
static bool mpu9265ReadRegister(uint8_t reg, uint8_t* value) {
  if (value == nullptr) {
    return false;
  }

  GYRO_I2C.beginTransmission(g_mpuAddress);
  GYRO_I2C.write(reg);

  uint8_t error =
    GYRO_I2C.endTransmission(false);

  if (error != 0) {
    return false;
  }

  size_t received = GYRO_I2C.requestFrom(
    g_mpuAddress,
    static_cast<size_t>(1),
    true
  );

  if (received != 1) {
    while (GYRO_I2C.available()) {
      GYRO_I2C.read();
    }

    return false;
  }

  if (!GYRO_I2C.available()) {
    return false;
  }

  *value = static_cast<uint8_t>(
    GYRO_I2C.read()
  );

  return true;
}

// =====================================================
// MPU9265 レジスタ連続読み取り
// =====================================================
static bool mpu9265ReadRegisters(
  uint8_t startReg,
  uint8_t* buffer,
  size_t length
) {
  if (buffer == nullptr || length == 0) {
    return false;
  }

  GYRO_I2C.beginTransmission(g_mpuAddress);
  GYRO_I2C.write(startReg);

  uint8_t error =
    GYRO_I2C.endTransmission(false);

  if (error != 0) {
    return false;
  }

  size_t received = GYRO_I2C.requestFrom(
    g_mpuAddress,
    length,
    true
  );

  if (received != length) {
    while (GYRO_I2C.available()) {
      GYRO_I2C.read();
    }

    return false;
  }

  for (size_t i = 0; i < length; i++) {
    if (!GYRO_I2C.available()) {
      return false;
    }

    buffer[i] = static_cast<uint8_t>(
      GYRO_I2C.read()
    );
  }

  return true;
}

// =====================================================
// MPU9265 Z軸ジャイロraw値読み取り
//
// GYRO_ZOUT_H = 0x47
// GYRO_ZOUT_L = 0x48
//
// MPU9265は上位バイト→下位バイトの順で返す。
// =====================================================
static bool mpu9265ReadRawGyroZ(int16_t* rawGyroZ) {
  if (rawGyroZ == nullptr) {
    return false;
  }

  uint8_t data[2];

  if (!mpu9265ReadRegisters(
        MPU_REG_GYRO_ZOUT_H,
        data,
        sizeof(data)
      )) {
    return false;
  }

  *rawGyroZ =
    static_cast<int16_t>(
      (static_cast<uint16_t>(data[0]) << 8) |
      static_cast<uint16_t>(data[1])
    );

  return true;
}

// =====================================================
// MPU9265を次の走行前に確認・復旧
//
// 戻り値:
//   true  : MPU9265を使用可能
//   false : 復旧できなかった
//
// 動作:
//   1. 現在Readyなら生存確認
//   2. 異常なら再初期化
//   3. Z軸オフセットを再キャリブレーション
//   4. 実データ読み取りを確認
//   5. Yaw積分値を0へ戻す
//
// 注意:
//   再キャリブレーション中はロボットを完全に停止させる。
// =====================================================
static bool tryRecoverGyroForNextMove() {
  // 走行開始前の処理なので、復旧中は必ずモーターを止める
  stopMotors();

  // -----------------------------------------------------
  // 1. 現在Readyなら、実際に通信できるか確認
  // -----------------------------------------------------
  if (g_gyroReady) {
    if (checkGyroAlive()) {
      // 前回から長時間空いていても積分時間が飛ばないようにする
      g_gyroZDps = 0.0f;
      g_yawPrevMicros = micros();

      return true;
    }

    Serial.println(
      "MPU9265 lost: recovery start"
    );
  } else {
    Serial.println(
      "MPU9265 not ready: recovery start"
    );
  }

  // -----------------------------------------------------
  // 2. 復旧前の状態を初期化
  // -----------------------------------------------------
  g_gyroReady = false;
  g_gyroZDps = 0.0f;
  g_yawDeg = 0.0f;
  g_yawPrevMicros = 0;

  boardRgbSet(255, 180, 0);  // 黄：復旧中

  // -----------------------------------------------------
  // 3. 最大3回、再初期化を試す
  // -----------------------------------------------------
  const int RECOVERY_RETRY_COUNT = 3;

  for (int retry = 1;
       retry <= RECOVERY_RETRY_COUNT;
       retry++) {

    Serial.print("MPU9265 recovery attempt ");
    Serial.print(retry);
    Serial.print("/");
    Serial.println(RECOVERY_RETRY_COUNT);

    // 念のためI2CバスとMPU9265を再設定する
    if (!setupGyroMPU9265()) {
      Serial.println(
        "MPU9265 recovery: setup failed"
      );

      g_gyroReady = false;
      delay(100);
      continue;
    }

    // ---------------------------------------------------
    // 4. 静止時Z軸オフセットを再測定
    //
    // 500回×10msなので約5秒。
    // この間はロボットを動かさない。
    // ---------------------------------------------------
    calibrateGyroZ(500);

    if (!g_gyroReady) {
      Serial.println(
        "MPU9265 recovery: calibration failed"
      );

      delay(100);
      continue;
    }

    // ---------------------------------------------------
    // 5. WHO_AM_IとZ軸実データを再確認
    // ---------------------------------------------------
    if (!checkGyroAlive()) {
      Serial.println(
        "MPU9265 recovery: alive check failed"
      );

      g_gyroReady = false;
      delay(100);
      continue;
    }

    // ---------------------------------------------------
    // 6. 積分角度を0へ戻す
    // ---------------------------------------------------
    resetYawDeg();

    if (!g_gyroReady) {
      Serial.println(
        "MPU9265 recovery: yaw reset failed"
      );

      delay(100);
      continue;
    }

    // resetYawDeg()後にもう一度Z軸を確認
    int16_t rawGyroZ = 0;

    if (!mpu9265ReadRawGyroZ(&rawGyroZ)) {
      Serial.println(
        "MPU9265 recovery: final gyro read failed"
      );

      g_gyroReady = false;
      g_gyroZDps = 0.0f;
      g_yawPrevMicros = 0;

      delay(100);
      continue;
    }

    // ---------------------------------------------------
    // 7. 復旧成功
    // ---------------------------------------------------
    g_gyroReady = true;
    g_gyroZDps = 0.0f;
    g_yawPrevMicros = micros();

    boardRgbSet(0, 255, 0);

    Serial.print(
      "MPU9265 recovery successful: rawGyroZ="
    );
    Serial.println(rawGyroZ);

    return true;
  }

  // -----------------------------------------------------
  // 8. 全試行失敗
  // -----------------------------------------------------

  g_gyroReady = false;
  g_gyroZDps = 0.0f;
  g_yawDeg = 0.0f;
  g_yawPrevMicros = 0;

  boardRgbSet(255, 0, 0);
  stopMotors();

  Serial.println(
    "MPU9265 recovery failed"
  );

  return false;
}
// 既存コードの呼び出し名を変えないために残す
// =====================================================
// MPU9265初期化
//
// 接続:
//   SDA = GP18
//   SCL = GP19
//   VCC = 3.3V
//   GND = GND
//
// 設定:
//   クロック      : X軸ジャイロPLL
//   サンプル周期  : 8kHz（125us）
//   ローパス      : DLPF_CFG=0、GYRO_CONFIG FCHOICE_B=0
//   ジャイロ範囲  : ±500 degree/second
// =====================================================
static bool setupGyroMPU9265() {
  Serial.println("MPU9265 setup start");
  g_hasGyroSample = false;

  g_gyroReady = false;
  g_gyroZOffset = 0.0f;
  g_yawDeg = 0.0f;
  g_gyroZDps = 0.0f;
  g_yawPrevMicros = 0;

  // -----------------------------------------------------
  // 1. Wire1をGP18/GP19で開始
  // -----------------------------------------------------
  GYRO_I2C.setSDA(GYRO_I2C_SDA_PIN);
  GYRO_I2C.setSCL(GYRO_I2C_SCL_PIN);
  GYRO_I2C.begin();

  // 1000Hz制御内でジャイロを読み切れるようFast-modeへ設定する。
  GYRO_I2C.setClock(400000);

  delay(100);

  // -----------------------------------------------------
  // 2. 0x68と0x69を検索
  // -----------------------------------------------------
  const uint8_t addresses[2] = {
    MPU_ADDRESS_LOW,
    MPU_ADDRESS_HIGH
  };

  bool detected = false;
  uint8_t whoAmI = 0x00;

  for (int addressIndex = 0;
       addressIndex < 2 && !detected;
       addressIndex++) {

    g_mpuAddress = addresses[addressIndex];

    for (int retry = 0; retry < 10; retry++) {
      if (mpu9265ReadRegister(
            MPU_REG_WHO_AM_I,
            &whoAmI
          )) {

        if (isSupportedMpuWhoAmI(whoAmI)) {
          detected = true;
          break;
        }
      }

      delay(20);
    }
  }

  if (!detected) {
    Serial.println(
      "MPU9265 was not detected at 0x68 or 0x69"
    );

    Serial.print("Last WHO_AM_I=0x");

    if (whoAmI < 0x10) {
      Serial.print("0");
    }

    Serial.println(whoAmI, HEX);

    g_gyroReady = false;
    boardRgbSet(255, 0, 0);
    return false;
  }

  Serial.print("MPU9265 detected: address=0x");
  Serial.print(g_mpuAddress, HEX);

  Serial.print(" WHO_AM_I=0x");

  if (whoAmI < 0x10) {
    Serial.print("0");
  }

  Serial.println(whoAmI, HEX);

  // -----------------------------------------------------
  // 3. デバイスリセット
  // -----------------------------------------------------
  if (!mpu9265WriteRegister(
        MPU_REG_PWR_MGMT_1,
        0x80
      )) {
    Serial.println("MPU9265 reset write failed");
    boardRgbSet(255, 0, 0);
    return false;
  }

  delay(100);

  // リセット後、同じアドレスで再応答するまで少し待つ
  bool respondedAfterReset = false;

  for (int retry = 0; retry < 10; retry++) {
    GYRO_I2C.beginTransmission(g_mpuAddress);

    if (GYRO_I2C.endTransmission(true) == 0) {
      respondedAfterReset = true;
      break;
    }

    delay(20);
  }

  if (!respondedAfterReset) {
    Serial.println(
      "MPU9265 did not respond after reset"
    );

    boardRgbSet(255, 0, 0);
    return false;
  }

  // -----------------------------------------------------
  // 4. スリープ解除
  // X軸ジャイロPLLをクロックに使用
  // -----------------------------------------------------
  if (!mpu9265WriteRegister(
        MPU_REG_PWR_MGMT_1,
        0x01
      )) {
    Serial.println("MPU9265 wakeup failed");
    boardRgbSet(255, 0, 0);
    return false;
  }

  delay(10);

  // 全軸有効
  if (!mpu9265WriteRegister(
        MPU_REG_PWR_MGMT_2,
        0x00
      )) {
    Serial.println("MPU9265 axis enable failed");
    boardRgbSet(255, 0, 0);
    return false;
  }

  // DLPF_CFG=0: 対応MPU共通の8kHzモード。帯域が広がるため振動ノイズに注意。
  if (!mpu9265WriteRegister(
        MPU_REG_CONFIG,
        0x00
      )) {
    Serial.println("MPU9265 DLPF setup failed");
    boardRgbSet(255, 0, 0);
    return false;
  }

  // 分周なし。DLPF_CFG=0ではジャイロ出力8kHz。
  if (!mpu9265WriteRegister(
        MPU_REG_SMPLRT_DIV,
        0
      )) {
    Serial.println(
      "MPU9265 sample rate setup failed"
    );

    boardRgbSet(255, 0, 0);
    return false;
  }

  // FS_SEL=1 → ±500dps
  if (!mpu9265WriteRegister(
        MPU_REG_GYRO_CONFIG,
        0x08
      )) {
    Serial.println(
      "MPU9265 gyro range setup failed"
    );

    boardRgbSet(255, 0, 0);
    return false;
  }

  // 加速度±2g
  if (!mpu9265WriteRegister(
        MPU_REG_ACCEL_CONFIG,
        0x00
      )) {
    Serial.println(
      "MPU9265 accel range setup failed"
    );

    boardRgbSet(255, 0, 0);
    return false;
  }

  delay(100);

  // -----------------------------------------------------
  // 5. 設定後にWHO_AM_Iを再確認
  // -----------------------------------------------------
  whoAmI = 0x00;

  if (!mpu9265ReadRegister(
        MPU_REG_WHO_AM_I,
        &whoAmI
      ) ||
      !isSupportedMpuWhoAmI(whoAmI)) {

    Serial.print(
      "MPU9265 verification failed: WHO_AM_I=0x"
    );

    if (whoAmI < 0x10) {
      Serial.print("0");
    }

    Serial.println(whoAmI, HEX);

    g_gyroReady = false;
    boardRgbSet(255, 0, 0);
    return false;
  }

  // -----------------------------------------------------
  // 6. 初期状態
  // -----------------------------------------------------
  g_gyroZOffset = 0.0f;
  g_yawDeg = 0.0f;
  g_gyroZDps = 0.0f;
  g_yawPrevMicros = micros();

  g_gyroReady = true;

  boardRgbSet(0, 255, 0);

  Serial.print("MPU9265 setup OK: address=0x");
  Serial.print(g_mpuAddress, HEX);

  Serial.print(" WHO_AM_I=0x");

  if (whoAmI < 0x10) {
    Serial.print("0");
  }

  Serial.println(whoAmI, HEX);

  Serial.println("Gyro range: +/-500 dps");
  Serial.println("Gyro output rate: 8000 Hz; I2C: 400 kHz");
  Serial.print("Motion control target period (us): ");
  Serial.println(MOTION_CONTROL_PERIOD_US);
  Serial.println("MPU9265 READY: LED GREEN");

  return true;
}
// =====================================================
// MPU9265 Z軸ジャイロ オフセット測定
//
// times:
//   測定回数
//   通常は500回程度を推奨
//
// 注意:
//   測定中はロボットを完全に静止させる。
//   動かすと誤ったオフセットが保存される。
// =====================================================
void calibrateGyroZ(int times) {
  Serial.println("MPU9265 gyro Z calibration...");
  Serial.println("Keep robot completely still.");

  stopMotors();

  // 不正な回数が渡された場合
  if (times <= 0) {
    Serial.println(
      "MPU9265 calibration failed: invalid sample count"
    );

    g_gyroReady = false;
    g_gyroZOffset = 0.0f;
    g_yawDeg = 0.0f;
    g_gyroZDps = 0.0f;

    boardRgbSet(255, 0, 0);
    return;
  }

  // -----------------------------------------------------
  // 1. MPU9265がI2C上に存在するか確認
  // -----------------------------------------------------
  uint8_t whoAmI = 0x00;

  if (!mpu9265ReadRegister(
        MPU_REG_WHO_AM_I,
        &whoAmI
      )) {
    Serial.println(
      "MPU9265 calibration failed: WHO_AM_I read failed"
    );

    g_gyroReady = false;
    g_gyroZOffset = 0.0f;
    g_yawDeg = 0.0f;
    g_gyroZDps = 0.0f;

    boardRgbSet(255, 0, 0);
    return;
  }

  if (!isSupportedMpuWhoAmI(whoAmI)) {
    Serial.print(
      "MPU9265 calibration failed: invalid WHO_AM_I=0x"
    );

    if (whoAmI < 0x10) {
      Serial.print("0");
    }

    Serial.println(whoAmI, HEX);

    g_gyroReady = false;
    g_gyroZOffset = 0.0f;
    g_yawDeg = 0.0f;
    g_gyroZDps = 0.0f;

    boardRgbSet(255, 0, 0);
    return;
  }

  // -----------------------------------------------------
  // 2. センサー値が安定するまで少し待つ
  // -----------------------------------------------------
  delay(200);

  // 最初の数回は捨て読みする
  int16_t dummyRaw = 0;

  for (int i = 0; i < 20; i++) {
    mpu9265ReadRawGyroZ(&dummyRaw);
    delay(10);
  }

  // -----------------------------------------------------
  // 3. Z軸raw値を複数回測定
  // -----------------------------------------------------
  int64_t rawSum = 0;

  int validCount = 0;
  int readFailCount = 0;

  int16_t rawMin = 32767;
  int16_t rawMax = -32768;

  for (int i = 0; i < times; i++) {
    int16_t rawGyroZ = 0;

    if (mpu9265ReadRawGyroZ(&rawGyroZ)) {
      rawSum += rawGyroZ;
      validCount++;

      if (rawGyroZ < rawMin) {
        rawMin = rawGyroZ;
      }

      if (rawGyroZ > rawMax) {
        rawMax = rawGyroZ;
      }

    } else {
      readFailCount++;
    }

    // キャリブレーション平均を安定させるため10ms間隔
    delay(10);
  }

  // -----------------------------------------------------
  // 4. 読取り成功数を確認
  //
  // 全体の80%以上成功した場合だけ有効とする。
  // -----------------------------------------------------
  int minimumValidCount =
    static_cast<int>(times * 0.8f);

  if (minimumValidCount < 1) {
    minimumValidCount = 1;
  }

  if (validCount < minimumValidCount) {
    Serial.println(
      "MPU9265 calibration failed: too many read errors"
    );

    Serial.print("validCount=");
    Serial.println(validCount);

    Serial.print("readFailCount=");
    Serial.println(readFailCount);

    g_gyroReady = false;
    g_gyroZOffset = 0.0f;
    g_yawDeg = 0.0f;
    g_gyroZDps = 0.0f;

    boardRgbSet(255, 0, 0);
    return;
  }

  // -----------------------------------------------------
  // 5. 静止時オフセットを計算
  //
  // g_gyroZOffsetはraw単位で保存する。
  // 角速度へ変換する際に、
  //
  // (rawGyroZ - g_gyroZOffset) / 65.5
  //
  // と計算する。
  // -----------------------------------------------------
  g_gyroZOffset =
    static_cast<float>(rawSum) /
    static_cast<float>(validCount);

  // -----------------------------------------------------
  // 6. キャリブレーション後の状態を初期化
  // -----------------------------------------------------
  g_yawDeg = 0.0f;
  g_gyroZDps = 0.0f;
  g_yawPrevMicros = micros();

  g_gyroReady = true;

  boardRgbSet(0, 255, 0);

  // -----------------------------------------------------
  // 7. 結果表示
  // -----------------------------------------------------
  Serial.println("MPU9265 gyro Z calibration complete");

  Serial.print("samplesRequested=");
  Serial.println(times);

  Serial.print("validCount=");
  Serial.println(validCount);

  Serial.print("readFailCount=");
  Serial.println(readFailCount);

  Serial.print("rawMin=");
  Serial.println(rawMin);

  Serial.print("rawMax=");
  Serial.println(rawMax);

  Serial.print("rawRange=");
  Serial.println(
    static_cast<long>(rawMax) -
    static_cast<long>(rawMin)
  );

  Serial.print("gyroZOffsetRaw=");
  Serial.println(g_gyroZOffset, 3);

  Serial.print("gyroZOffsetDps=");
  Serial.println(
    g_gyroZOffset / MPU9265_GYRO_SCALE_DPS,
    4
  );

  Serial.println("MPU9265 GYRO READY: LED GREEN");
}

// =====================================================
// MPU9265のZ軸角速度からYaw角を更新
//
// MPU9265はBNO055と異なり、Yaw角そのものを出力しない。
// Z軸角速度 degree/second を経過時間で積分して、
// 相対Yaw角 g_yawDeg を作る。
//
// 必要な事前処理:
//   setupGyroMPU9265()
//   calibrateGyroZ()
//
// 使用する値:
//   g_gyroZOffset   : 静止時のZ軸raw平均値
//   g_gyroZDps      : 現在のZ軸角速度 degree/second
//   g_yawDeg        : 積分した相対Yaw角 degree
//   g_yawPrevMicros : 前回更新時刻
// =====================================================
static void updateYawFromGyro() {
  if (!g_gyroReady) {
    return;
  }

  // -----------------------------------------------------
  // 1. MPU9265のZ軸ジャイロraw値を読む
  // -----------------------------------------------------
  int16_t rawGyroZ = 0;
  bool readOk = false;

  // 一時的なI2C読取り失敗を考慮して最大3回試す
  for (int retry = 0; retry < 3; retry++) {
    if (mpu9265ReadRawGyroZ(&rawGyroZ)) {
      readOk = true;
      break;
    }

    delayMicroseconds(500);
  }

  if (!readOk) {
    g_gyroReady = false;
    g_gyroZDps = 0.0f;

    boardRgbSet(255, 0, 0);
    stopMotors();

    Serial.println(
      "MPU9265 gyro Z read failed after retry"
    );

    return;
  }

  // -----------------------------------------------------
  // 2. 現在時刻と前回からの経過時間を取得
  // -----------------------------------------------------
  unsigned long nowMicros = micros();

  if (g_hasGyroSample) {
    const unsigned long intervalUs = nowMicros - g_lastGyroSampleUs;
    // 起動・停止・キャリブレーションなどの長い空白は走行周期集計から除外。
    if (intervalUs <= 100000UL) {
      ++g_motionTiming.gyroIntervals;
      g_motionTiming.totalGyroIntervalUs += intervalUs;
      if (intervalUs > g_motionTiming.maxGyroIntervalUs) {
        g_motionTiming.maxGyroIntervalUs = intervalUs;
      }
    }
  }
  g_lastGyroSampleUs = nowMicros;
  g_hasGyroSample = true;

  // 初回だけは積分せず、時刻基準だけ登録する
  if (g_yawPrevMicros == 0) {
    g_yawPrevMicros = nowMicros;
    g_gyroZDps = 0.0f;
    return;
  }

  unsigned long elapsedMicros =
    nowMicros - g_yawPrevMicros;

  g_yawPrevMicros = nowMicros;

  float dt =
    static_cast<float>(elapsedMicros) /
    1000000.0f;

  // -----------------------------------------------------
  // 3. 異常な経過時間を除外
  //
  // ジャイロ走行ループの目標は125us。実際の経過時間で積分する。
  // 長時間呼ばれなかった後の値を積分すると、
  // Yaw角が突然大きく飛ぶため、その回は積分しない。
  // -----------------------------------------------------
  if (dt <= 0.0f || dt > 0.1f) {
    g_gyroZDps = 0.0f;
    return;
  }

  // -----------------------------------------------------
  // 4. raw値から静止時オフセットを引く
  // -----------------------------------------------------
  float correctedRaw =
    static_cast<float>(rawGyroZ) -
    g_gyroZOffset;

  // -----------------------------------------------------
  // 5. raw値をdegree/secondへ変換
  //
  // setupGyroMPU9265()で±500dpsへ設定しているため、
  // 感度は65.5 LSB/(degree/second)。
  // -----------------------------------------------------
  float gyroZDps =
    correctedRaw /
    MPU9265_GYRO_SCALE_DPS;

  // センサーの取り付け方向に応じた符号補正
  gyroZDps *= GYRO_YAW_DIR;

  g_gyroZDps = gyroZDps;

  // -----------------------------------------------------
  // 6. 角速度を時間積分してYaw角を更新
  //
  // angle += degree/second × second
  // -----------------------------------------------------
  g_yawDeg += g_gyroZDps * dt;
}
bool gyroReady() {
  return g_gyroReady;
}

// =====================================================
// MPU9265 相対Yaw角リセット
//
// MPU9265は絶対Yaw角を出力しないため、
// 積分している角度を0度へ戻す。
//
// g_gyroZOffsetはキャリブレーション値なので、
// ここでは変更しない。
// =====================================================
void resetYawDeg() {
  g_hasGyroSample = false;
  // ジャイロが使用可能でなければ、
  // 角度だけ初期化して終了する
  if (!g_gyroReady) {
    g_yawDeg = 0.0f;
    g_gyroZDps = 0.0f;
    g_yawPrevMicros = 0;

    Serial.println(
      "MPU9265 resetYawDeg skipped: gyro not ready"
    );

    return;
  }

  // -----------------------------------------------------
  // 積分角度を0度へ戻す
  // -----------------------------------------------------
  g_yawDeg = 0.0f;

  // 前回の角速度を残すと、リセット直後のPIDのD項へ
  // 古い値が使われるため0へ戻す
  g_gyroZDps = 0.0f;

  // -----------------------------------------------------
  // 積分時間の基準を現在時刻へ更新
  //
  // これを行わないと、リセット前からの待機時間が
  // 次回の角度積分へ含まれる可能性がある。
  // -----------------------------------------------------
  g_yawPrevMicros = micros();

  Serial.println("MPU9265 yaw reset: 0.00 deg");
}

float getYawDeg() {
  updateYawFromGyro();
  return g_yawDeg;
}

// spin / pivot: 角度PD + 時間基準のS字加速・残角基準のS字減速。
// 単位: KP=PWM/deg、KD=PWM/(deg/s)。実機で個別に調整する初期値。
float SPIN_PD_KP = 8.0f;
float SPIN_PD_KD = 0.3f;
float PIVOT_PD_KP = 10.0f;
float PIVOT_PD_KD = 0.8f;
const float TURN_PD_RISE_PWM_PER_SEC = 600.0f;
// 駆動下限はspinのGYRO_TURN_MIN_POWER / pivotのminPowerを使用。
// 固定200の下限は加減速を潰すため使わない。
const float TURN_PD_FINE_DEG = 12.0f;

static void runTurnPD(float deg, int maxPower, int minPower,
                      bool isPivot, RobotSide stopSide) {
  if (!isfinite(deg) || fabs(deg) < 0.1f || maxPower == 0) {
    stopMotors();
    return;
  }
  maxPower = static_cast<int>(constrain(fabs(static_cast<float>(maxPower)), 0.0f, 255.0f));
  minPower = static_cast<int>(constrain(fabs(static_cast<float>(minPower)), 0.0f, static_cast<float>(maxPower)));
  if (!tryRecoverGyroForNextMove()) {
    stopMotors();
    return;
  }
  float target = fabs(deg) * (isPivot ? PIVOT_ANGLE_GAIN : SPIN_ANGLE_GAIN);
  if (!isfinite(target) || target <= 0.0f) {
    stopMotors();
    return;
  }
  resetEncoder();
  resetYawDeg();
  if (!g_gyroReady) {
    stopMotors();
    return;
  }
  const float kp = isPivot ? PIVOT_PD_KP : SPIN_PD_KP;
  const float kd = isPivot ? PIVOT_PD_KD : SPIN_PD_KD;
  const int driveMinPower = minPower;
  const int direction = deg > 0.0f ? 1 : -1;
  const unsigned long startMs = millis();
  const unsigned long timeoutMs = 3000UL + static_cast<unsigned long>(
    constrain(fabs(deg) * 30.0f, 0.0f, 3600000.0f));
  unsigned long previousUs = micros();
  const unsigned long profileStartUs = previousUs;
  // 初動の微小な揺れで符号を固定しない。開始位置からの角変位の大きさで追跡。
  float filteredRate = 0.0f;
  float applied = 0.0f;
  long previousL = 0, previousR = 0;
  float filteredL = 0.0f, filteredR = 0.0f;
  while (true) {
    unsigned long cycleUs = micros();
    float dt = constrain((cycleUs - previousUs) * 0.000001f, 0.0001f, 0.05f);
    previousUs = cycleUs;
    updateServoTasks();
    updateGyroModeLed();
    if (stopPressed()) break;
    if (millis() - startMs >= timeoutMs) break;
    float yaw = getYawDeg();
    if (!g_gyroReady || !isfinite(yaw) || !isfinite(g_gyroZDps)) break;
    float progress = turn_progress::angle(yaw);
    float rate = turn_progress::rate(yaw, g_gyroZDps);
    filteredRate += dt / (0.02f + dt) * (rate - filteredRate);
    float error = target - progress;
    // 到達・通過したらブレーキで終了。逆転して角度を戻さない。
    if (turn_progress::reached(yaw, target)) break;
    float command = kp * error - kd * filteredRate;
    // PDが負ならブレーキ。ただし低速・停止時は目標手前で保持せず再始動。
    // D項で負になった場合にも摩擦補償を適用し、失速から復帰できるようにする。
    command = constrain(command, 0.0f, static_cast<float>(maxPower));
    if (fabs(filteredRate) < GYRO_TURN_STOP_DPS)
      command = max(command, static_cast<float>(driveMinPower));
    // 周回数に依存しないS字出力上限。短い旋回では加速中でも減速側を優先。
    const float profilePower = turn_s_curve::power(
      static_cast<unsigned long>(cycleUs - profileStartUs) * 0.000001f,
      target, progress, driveMinPower, maxPower,
      isPivot ? GYRO_PIVOT_DEC_RATIO : GYRO_SPIN_DEC_RATIO,
      TURN_PD_RISE_PWM_PER_SEC);
    applied = min(command, profilePower);
    if (command > 0.0f)
      applied = max(applied, static_cast<float>(driveMinPower));

    long totalL = getTotalEncL(), totalR = getTotalEncR();
    float alpha = dt / (0.017f + dt);
    filteredL += alpha * ((totalL - previousL) * 0.005f / dt - filteredL);
    filteredR += alpha * ((totalR - previousR) * 0.005f / dt - filteredR);
    previousL = totalL; previousR = totalR;
    if (fabs(applied) < 1.0f) {
      shortBrakeMotors();
    } else {
      int power = static_cast<int>(fabs(applied));
      int left = static_cast<int>(power * (isPivot ? PIVOT_LEFT_MOTOR_GAIN : SPIN_LEFT_MOTOR_GAIN));
      int right = static_cast<int>(power * (isPivot ? PIVOT_RIGHT_MOTOR_GAIN : SPIN_RIGHT_MOTOR_GAIN));
      // 大きく回している間は既存の左右同期を維持。終端では無効化。
      if (!isPivot && error > TURN_PD_FINE_DEG && applied > 0.0f) {
        int limit = min(SPIN_SYNC_MAX, power);
        int correction = static_cast<int>((totalL - totalR) * SPIN_SYNC_POSITION_KP +
                                         (filteredL - filteredR) * SPIN_SYNC_SPEED_KD);
        correction = constrain(correction, -limit, limit);
        // 終端へ近づくにつれて同期補正を弱め、切替時の段差を避ける。
        correction = static_cast<int>(correction * constrain(
          (error - TURN_PD_FINE_DEG) / TURN_PD_FINE_DEG, 0.0f, 1.0f));
        left -= correction; right += correction;
      }
      // 補正・モーターゲイン後もS字上限を守る。maxPowerだけの制限では
      // 同期補正が加減速を打ち消してしまう。
      // pivot固定輪はこの後0出力にしてショートブレーキで保持する。
      const int profileMax = static_cast<int>(profilePower);
      left = turn_s_curve::clampWheel(left, driveMinPower, profileMax);
      right = turn_s_curve::clampWheel(right, driveMinPower, profileMax);
      int driveDirection = direction;
      motorPower(isPivot && stopSide == L ? 0 : driveDirection * left,
                 isPivot && stopSide == R ? 0 : -driveDirection * right);
      if (isPivot) {
        if (stopSide == L) { analogWrite(M2A, 255); analogWrite(M2B, 255); }
        else { analogWrite(M1A, 255); analogWrite(M1B, 255); }
      }
    }
    finishMotionControlCycle(cycleUs);
  }
  shortBrakeMotors();
  delay(50);
  stopMotors();
}

void spin(float deg, int maxPower) {
  runTurnPD(deg, maxPower, GYRO_TURN_MIN_POWER, false, L);
}

void pivot(float deg, int Maxspeed, RobotSide stopSide, int minPower) {
  if (stopSide != L && stopSide != R) { stopMotors(); return; }
  runTurnPD(deg, Maxspeed, minPower, true, stopSide);
}

void test_encoder(unsigned long ms) {
  resetEncoder();

  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0;

  while (millis() - startTime < ms) {
    updateServoTasks();
    updateGyroModeLed();

    if (stopPressed()) {
      break;
    }

    unsigned long now = millis();

    // 100msごとに表示
    if (now - lastPrintTime >= 100) {
      lastPrintTime = now;

      long leftCount;
      long rightCount;

      noInterrupts();
      leftCount = enc2;   // 左エンコーダ
      rightCount = enc1;  // 右エンコーダ
      interrupts();

      long leftAbs = labs(leftCount);
      long rightAbs = labs(rightCount);
      long pos = (leftAbs + rightAbs) / 2;

      Serial.print("time=");
      Serial.print(now - startTime);
      Serial.print("ms");

      Serial.print(" L=");
      Serial.print(leftCount);

      Serial.print(" R=");
      Serial.print(rightCount);

      Serial.print(" pos=");
      Serial.println(pos);
    }
  }

  Serial.println("test_encoder end");
}
void kabe(int power) {
  int dir = (power >= 0) ? 1 : -1;

  int basePower = abs(power);
  basePower = constrain(basePower, 100, 255);

  const int TANK_WHEEL_MIN_POWER = 50;
  int wheelMinPower = constrain(TANK_WHEEL_MIN_POWER, 0, basePower);

  const unsigned long START_IGNORE_MS = 300;
  const unsigned long CHECK_INTERVAL_MS = 80;
  const long STALL_DELTA_PULSE = 2;
  const unsigned long TIMEOUT_MS = 3000;

  resetEncoder();

  unsigned long startMs = millis();
  unsigned long lastCheckMs = millis();

  long lastLeftAbs = 0;
  long lastRightAbs = 0;

  while (true) {
    updateServoTasks();

    if (stopPressed()) {
      break;
    }

    if (millis() - startMs > TIMEOUT_MS) {
      break;
    }

    long leftCount;
    long rightCount;

    noInterrupts();
    leftCount = labs(enc2);   // 左エンコーダ
    rightCount = labs(enc1);  // 右エンコーダ
    interrupts();

    // =====================================================
    // 左右同期補正
    //
    // 左が多く回っている:
    //   syncError > 0
    //   左を弱める
    //   右を強める
    //
    // 右が多く回っている:
    //   syncError < 0
    //   左を強める
    //   右を弱める
    // =====================================================
    long syncError = leftCount - rightCount;

    int syncCorrection = (int)(syncError * SYNC_KP);
    syncCorrection = constrain(syncCorrection, -SYNC_MAX, SYNC_MAX);

    int leftAbsPower = basePower - syncCorrection;
    int rightAbsPower = basePower + syncCorrection;

    leftAbsPower = constrain(leftAbsPower, wheelMinPower, 255);
    rightAbsPower = constrain(rightAbsPower, wheelMinPower, 255);

    int leftPower = leftAbsPower * dir;
    int rightPower = rightAbsPower * dir;

    leftPower = constrain(leftPower, -255, 255);
    rightPower = constrain(rightPower, -255, 255);

    motorPower(leftPower, rightPower);

    unsigned long nowMs = millis();

    if (nowMs - lastCheckMs >= CHECK_INTERVAL_MS) {
      lastCheckMs = nowMs;

      long leftAbs = leftCount;
      long rightAbs = rightCount;

      long leftDelta = leftAbs - lastLeftAbs;
      long rightDelta = rightAbs - lastRightAbs;

      lastLeftAbs = leftAbs;
      lastRightAbs = rightAbs;

      if (nowMs - startMs > START_IGNORE_MS) {
        bool leftStalled = leftDelta <= STALL_DELTA_PULSE;
        bool rightStalled = rightDelta <= STALL_DELTA_PULSE;

        // 壁あてなので、左右両方が止まったら終了
        if (leftStalled && rightStalled) {
          break;
        }
      }
    }

    delay(1);
  }

  stopMotors();
}

// =====================================================
// setup 相当
// =====================================================
void motionSetup() {

  boardRgbSetup();
  buzzerSetup();
  
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);

  pinMode(SENSOR_R, INPUT);
  pinMode(SENSOR_L, INPUT);
  pinMode(SENSOR_R_END, INPUT);
  pinMode(SENSOR_L_END, INPUT);

  pinMode(START_BUTTON, INPUT_PULLUP);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncR, CHANGE);

  Serial.begin(115200);
  delay(500);

  stopMotor();

  if (setupGyroMPU9265()) {
    calibrateGyroZ(500);

    // gyro ready 成功：緑
    boardRgbSet(0, 255, 0);
    Serial.println("GYRO READY: LED GREEN");
  } else {
    // gyro ready 失敗：赤
    boardRgbSet(255, 0, 0);
    Serial.println("GYRO NOT READY: LED RED");
  }

  Serial.println("READY");
}
void checkline(int times) {
  if (times <= 0) {
    return;
  }

  for (int i = 0; i < times; i++) {
    int rightEndValue = analogRead(SENSOR_R_END);
    int rightValue    = analogRead(SENSOR_R);
    int leftValue     = analogRead(SENSOR_L);
    int leftEndValue  = analogRead(SENSOR_L_END);

    Serial.print("回数=");
    Serial.print(i + 1);

    Serial.print("  L_END=");
    Serial.print(leftEndValue);

    Serial.print("  L=");
    Serial.print(leftValue);

    Serial.print("  R=");
    Serial.print(rightValue);

    Serial.print("  R_END=");
    Serial.println(rightEndValue);

    delay(100);
  }
}

// =====================================================
// arc
// ジャイロ・エンコーダ補正付き円弧走行
//
// radiusMm:
//   ロボット中心が描く円の半径 mm
//   トレッド幅の半分で内輪停止、それ未満で内輪逆転、0でその場旋回
//
// deg:
//    正 = 右旋回
//    負 = 左旋回
//
// 例:
//   arc(300, 90, 200);
//     半径300mmで右へ90度
//
//   arc(300, -90, 200);
//     半径300mmで左へ90度
//
// motorPower() の引数は左右とも正 = 前進、負 = 後退
//
// エンコーダ:
//   enc2 = 左
//   enc1 = 右
// =====================================================
static bool runArc(
  float radiusMm,
  float deg,
  int maxPower,
  int minPower,
  int brake
) {
  // =====================================================
  // 1. 引数確認
  // =====================================================
  if (!isfinite(radiusMm) || !isfinite(deg) || maxPower == 0) {
    stopMotors();
    return false;
  }
  if (fabs(deg) < 0.1) {
    stopMotors();
    return true;
  }

  float halfTrack =
    TRACK_WIDTH_MM / 2.0;

  // 半径0はその場旋回。負の半径は指定不可。
  if (radiusMm < 0.0f) {
    Serial.println(
      "ARC ERROR: radiusMm must be non-negative"
    );

    stopMotors();
    return false;
  }

  // maxPowerの符号で前進・後退を指定
//
// 正:
//   前進
//
// 負:
//   後退
int driveDir =
  maxPower >= 0 ? 1 : -1;

maxPower =
  abs(maxPower);

minPower =
  abs(minPower);
  minPower = constrain(
    minPower,
    1,
    255
  );

  maxPower = constrain(
    maxPower,
    minPower,
    255
  );

  brake = constrain(
    brake,
    0,
    1
  );

  // =====================================================
  // 2. 旋回方向
  //
  // deg > 0:
  //   右旋回
  //
  // deg < 0:
  //   左旋回
  // =====================================================
  // degの符号は機体の向きが変わる方向を表す
//
// 90:
//   前進でも後退でも、機体の向きは右へ変化
//
// -90:
//   前進でも後退でも、機体の向きは左へ変化
bool turnRight =
  deg > 0;

// 後退時は、同じ向きへ機体を回すために
// タイヤの内外関係を反転する
bool wheelCurveRight;

if (driveDir > 0) {
  wheelCurveRight =
    turnRight;
} else {
  wheelCurveRight =
    !turnRight;
}

  // =====================================================
  // 3. 左右タイヤの旋回半径
  //
  // 右旋回では左輪が外側、右輪が内側。
  // 後退時はwheelCurveRightで内外を反転する。
  // =====================================================
  const auto radii = arc_geometry::wheelRadii(
    radiusMm, halfTrack, deg, driveDir
  );
  float leftRadiusMm = radii.left;
  float rightRadiusMm = radii.right;

  // 内輪半径が負なら逆転、0なら停止する。
  // 距離と出力の大きさは絶対値で制御し、方向は出力時に適用する。
  const int leftWheelDir = leftRadiusMm < 0.0f ? -1 : 1;
  const int rightWheelDir = rightRadiusMm < 0.0f ? -1 : 1;
  leftRadiusMm = fabs(leftRadiusMm);
  rightRadiusMm = fabs(rightRadiusMm);

  // =====================================================
  // 4. 旋回角度をラジアンへ変換
  // =====================================================
  // ユーザーが指定した旋回角度
float requestedAngleDeg =
  fabs(deg);

// オフセットを引いた制御目標角度
float targetAngleDeg =
  requestedAngleDeg -
  ARC_GYRO_OFFSET_DEG;

// オフセットが指定角度以上の場合の保護
if (targetAngleDeg < 0.1) {
  Serial.println(
    "ARC ERROR: gyro offset is too large"
  );

  stopMotors();
  return false;
}

// オフセット適用後の角度から
// 左右タイヤの目標距離を計算する
float angleRad =
  targetAngleDeg *
  PI /
  180.0;
  // =====================================================
  // 5. 左右タイヤの目標距離
  // =====================================================
  float leftTargetMm =
    leftRadiusMm *
    angleRad;

  float rightTargetMm =
    rightRadiusMm *
    angleRad;

  long leftTargetPulse =
    mmToPulse(
      leftTargetMm
    );

  long rightTargetPulse =
    mmToPulse(
      rightTargetMm
    );

  if (
    leftTargetPulse <= 0 &&
    rightTargetPulse <= 0
  ) {
    stopMotors();
    return false;
  }

  // =====================================================
  // 6. 外側タイヤの情報
  // =====================================================
  float outerRadiusMm =
    max(
      leftRadiusMm,
      rightRadiusMm
    );

  float outerTargetMm =
    max(
      leftTargetMm,
      rightTargetMm
    );

  long outerTargetPulse =
    max(
      leftTargetPulse,
      rightTargetPulse
    );

  bool leftIsOuter =
    leftRadiusMm >
    rightRadiusMm;

  // =====================================================
  // 7. 左右タイヤの速度比
  //
  // 外側タイヤを1.0とする
  // =====================================================
  float leftSpeedRatio =
    leftRadiusMm /
    outerRadiusMm;

  float rightSpeedRatio =
    rightRadiusMm /
    outerRadiusMm;

  // =====================================================
  // 8. 円弧制御定数
  // =====================================================

  // 左右の進行率同期補正
  //
  // 大きくすると円弧比率の補正が強くなる
  const float ARC_SYNC_KP =
    100.0;

  // ジャイロ角度補正
  const float ARC_GYRO_KP =
    3.0;

  // ジャイロ角速度補正
  const float ARC_GYRO_KD =
    0.15;

  // 同期補正上限
  const int ARC_SYNC_MAX =
    80;

  // ジャイロ補正上限
  const int ARC_GYRO_MAX =
    60;

  // すべての補正を合成した後の上限
  const int ARC_TOTAL_MAX =
    100;

  // 外側タイヤの最低出力。内輪はARC_INNER_PULSE_MIN_POWERでパルス駆動。
  const int ARC_WHEEL_MIN_POWER =
    45;

  // 台形制御
  const float ARC_ACC_RATIO =
    0.25;

  const float ARC_DEC_RATIO =
    0.35;

  // =====================================================
  // 9. 台形制御区間
  // =====================================================
  long accPulse =
    lround(
      outerTargetPulse *
      ARC_ACC_RATIO
    );

  long decPulse =
    lround(
      outerTargetPulse *
      ARC_DEC_RATIO
    );

  long cruisePulse =
    outerTargetPulse -
    accPulse -
    decPulse;

  if (accPulse < 1) {
    accPulse = 1;
  }

  if (decPulse < 1) {
    decPulse = 1;
  }

  // 短距離で加速区間と減速区間が重なる場合
  if (cruisePulse < 0) {
    long totalRamp =
      accPulse +
      decPulse;

    if (totalRamp > 0) {
      accPulse =
        static_cast<long>(
          static_cast<float>(
            outerTargetPulse
          ) *
          (
            static_cast<float>(
              accPulse
            ) /
            static_cast<float>(
              totalRamp
            )
          )
        );

      decPulse =
        outerTargetPulse -
        accPulse;
    }

    if (accPulse < 1) {
      accPulse = 1;
    }

    if (decPulse < 1) {
      decPulse = 1;
    }

    cruisePulse = 0;
  }

  // =====================================================
  // 10. ジャイロ準備
  //
  // ジャイロ復旧に失敗しても、
  // エンコーダだけで円弧走行を続ける
  // =====================================================
  bool useGyro =
    tryRecoverGyroForNextMove();

  if (useGyro) {
    resetYawDeg();

    if (!g_gyroReady) {
      useGyro = false;
    }
  }

  // =====================================================
  // 11. エンコーダ初期化
  // =====================================================
  resetEncoder();

  unsigned long startMs =
    millis();

  ArcStallGuard leftStallGuard(startMs,
    leftIsOuter ? ARC_WHEEL_MIN_POWER : ARC_INNER_PULSE_MIN_POWER, maxPower);
  ArcStallGuard rightStallGuard(startMs,
    leftIsOuter ? ARC_INNER_PULSE_MIN_POWER : ARC_WHEEL_MIN_POWER, maxPower);
  ArcPulsePower innerPulse(startMs);

  // 円弧形状はエンコーダで制御し、全体の制限時間も設ける。
  unsigned long timeoutMs =
    3000UL +
    static_cast<unsigned long>(
      outerTargetMm *
      20.0
    );

  // =====================================================
  // 12. 円弧走行ループ
  // =====================================================
  while (true) {
    const unsigned long cycleUs = micros();
    updateServoTasks();
    updateGyroModeLed();

    // STOPボタン
    if (stopPressed()) {
      stopMotors();
      return false;
    }

    // タイムアウト
    if (
      millis() - startMs >=
      timeoutMs
    ) {
      Serial.println(
        "ARC TIMEOUT"
      );

      stopMotors();
      return false;
    }

    // ===================================================
    // 13. 左右エンコーダ取得
    //
    // enc2 = 左
    // enc1 = 右
    // ===================================================
    long leftCount;
    long rightCount;

    noInterrupts();

    leftCount =
      labs(enc2);

    rightCount =
      labs(enc1);

    interrupts();

    bool leftFinished =
      leftCount >=
      leftTargetPulse;

    bool rightFinished =
      rightCount >=
      rightTargetPulse;

    // 両輪が目標に到達
    if (
      leftFinished &&
      rightFinished
    ) {
      break;
    }

    // ===================================================
    // 14. 外側タイヤの現在位置
    // ===================================================
    long outerCount;

    if (leftIsOuter) {
      outerCount =
        leftCount;
    } else {
      outerCount =
        rightCount;
    }

    outerCount = constrain(
      outerCount,
      0L,
      outerTargetPulse
    );

    long remainPulse =
      outerTargetPulse -
      outerCount;

    if (remainPulse < 0) {
      remainPulse = 0;
    }

    // ===================================================
    // 15. 台形速度制御
    //
    // 外側タイヤの出力を基準にする
    // ===================================================
    int outerBasePower;

    // 加速
    if (outerCount < accPulse) {
      float ratio =
        outerCount /
        static_cast<float>(
          accPulse
        );

      ratio = smoothStep01(ratio);

      outerBasePower =
        minPower +
        static_cast<int>(
          (
            maxPower -
            minPower
          ) *
          ratio
        );

    // 巡航
    } else if (
      outerCount <
      accPulse +
      cruisePulse
    ) {
      outerBasePower =
        maxPower;

    // 減速
    } else {
      float ratio =
        remainPulse /
        static_cast<float>(
          decPulse
        );

      ratio = smoothStep01(ratio);

      outerBasePower =
        minPower +
        static_cast<int>(
          (
            maxPower -
            minPower
          ) *
          ratio
        );
    }

    outerBasePower = constrain(
      outerBasePower,
      minPower,
      maxPower
    );

    // ===================================================
    // 16. 半径比から左右基本出力を計算
    // ===================================================
    int leftAbsPower =
      static_cast<int>(
        outerBasePower *
        leftSpeedRatio
      );

    int rightAbsPower =
      static_cast<int>(
        outerBasePower *
        rightSpeedRatio
      );

    // ===================================================
    // 17. 左右タイヤの進行率
    //
    // 正しい円弧なら、左右の進行率が一致する
    // ===================================================
    float leftProgress = leftTargetPulse > 0 ?
      leftCount /
      static_cast<float>(
        leftTargetPulse
      ) : 0.0f;

    float rightProgress = rightTargetPulse > 0 ?
      rightCount /
      static_cast<float>(
        rightTargetPulse
      ) : 0.0f;

    leftProgress = constrain(
      leftProgress,
      0.0,
      1.2
    );

    rightProgress = constrain(
      rightProgress,
      0.0,
      1.2
    );

    // ===================================================
    // 18. エンコーダ同期補正
    //
    // 左の進行率が大きい:
    //   左を弱める
    //   右を強める
    // ===================================================
    // 片輪停止の旋回では進行率の同期を行わない。
    float syncError = (leftTargetPulse > 0 && rightTargetPulse > 0)
      ? leftProgress - rightProgress : 0.0f;

    int syncCorrection =
      static_cast<int>(
        syncError *
        ARC_SYNC_KP
      );

    syncCorrection = constrain(
      syncCorrection,
      -ARC_SYNC_MAX,
      ARC_SYNC_MAX
    );

    // ===================================================
    // 19. ジャイロ角度補正
    //
    // ジャイロ角度の正負には依存せず、
    // 絶対値で実際の旋回量を見る
    // ===================================================
    int gyroCorrection = 0;

    if (useGyro) {
      float currentYaw =
        getYawDeg();

      if (!g_gyroReady) {
        useGyro = false;
        g_gyroZDps = 0.0;
        gyroCorrection = 0;

      } else {
        // 外側タイヤの進行率を基準に
        // 現在向いているべき角度を計算
        float pathProgress =
          outerCount /
          static_cast<float>(
            outerTargetPulse
          );

        pathProgress = constrain(
          pathProgress,
          0.0,
          1.0
        );

        float expectedAngle =
          targetAngleDeg *
          pathProgress;

        float actualAngle =
          fabs(
            currentYaw
          );

        float angleError =
          expectedAngle -
          actualAngle;

        // 旋回が不足している場合は正
        float gyroPd =
          ARC_GYRO_KP *
          angleError -
          ARC_GYRO_KD *
          fabs(
            g_gyroZDps
          );

        gyroCorrection =
          static_cast<int>(
            gyroPd
          );

        gyroCorrection = constrain(
          gyroCorrection,
          -ARC_GYRO_MAX,
          ARC_GYRO_MAX
        );
      }
    }

    // ===================================================
    // 20. 補正を左右出力へ適用
    // ===================================================

    // エンコーダ進行率補正
    leftAbsPower -=
      syncCorrection;

    rightAbsPower +=
      syncCorrection;

    // ジャイロ補正
    //
    // gyroCorrection > 0:
    //   旋回量が不足している
    //
    // 実機の旋回方向と各輪の進行方向に合わせて補正する。
    // 逆転輪では、出力の大きさに対する補正方向も反転する。
    if (wheelCurveRight) {
  // 右旋回を強めるには左輪を速め、右輪を遅くする
  leftAbsPower +=
    gyroCorrection * leftWheelDir;

  rightAbsPower -=
    gyroCorrection * rightWheelDir;

} else {
  // 左旋回は左右反転
  leftAbsPower -=
    gyroCorrection * leftWheelDir;

  rightAbsPower +=
    gyroCorrection * rightWheelDir;
}

    // ===================================================
    // 21. 補正量が大きくなりすぎないよう制限
    // ===================================================
    int leftBasePower =
      static_cast<int>(
        outerBasePower *
        leftSpeedRatio
      );

    int rightBasePower =
      static_cast<int>(
        outerBasePower *
        rightSpeedRatio
      );

    leftAbsPower = constrain(
      leftAbsPower,
      leftBasePower -
      ARC_TOTAL_MAX,
      leftBasePower +
      ARC_TOTAL_MAX
    );

    rightAbsPower = constrain(
      rightAbsPower,
      rightBasePower -
      ARC_TOTAL_MAX,
      rightBasePower +
      ARC_TOTAL_MAX
    );

    // ===================================================
    // 22. 出力範囲・目標到達・ストール対策
    // ===================================================
    const uint32_t nowMs = millis();
    // 最低出力へ引き上げる前の要求値を、内輪の平均PWMとして保存する。
    const int innerRequestedPower = constrain(
      leftIsOuter ? rightAbsPower : leftAbsPower, 0, maxPower
    );
    leftAbsPower = leftStallGuard.update(
      nowMs, leftCount, leftFinished, leftAbsPower
    );
    rightAbsPower = rightStallGuard.update(
      nowMs, rightCount, rightFinished, rightAbsPower
    );

    // 復帰しなければ両輪を停止。逆位相ブレーキで追加駆動しない。
    if (leftStallGuard.stalled() || rightStallGuard.stalled()) {
      stopMotors();
      Serial.println(leftStallGuard.stalled() ? "ARC STALL: LEFT" : "ARC STALL: RIGHT");
      return false;
    }

    // 内輪だけをパルス化。guardの最低PWM（停止時は段階的に増加）を
    // パルスの高さに使う。エンコーダ監視はOFF区間も含めて継続する。
    // 目標到達や要求0は、10msの途中でも即座に出力を止める。
    if (leftIsOuter) {
      rightAbsPower = innerPulse.update(nowMs, innerRequestedPower,
        rightAbsPower, rightFinished, ARC_INNER_PULSE_GAIN);
    } else {
      leftAbsPower = innerPulse.update(nowMs, innerRequestedPower,
        leftAbsPower, leftFinished, ARC_INNER_PULSE_GAIN);
    }

    // ===================================================
    // 23. モーター出力
    //
    // 進行方向に各輪の方向を掛けて、内輪の逆転にも対応する。
    // ===================================================
int leftPower =
  leftAbsPower *
  driveDir * leftWheelDir;

int rightPower =
  rightAbsPower *
  driveDir * rightWheelDir;

    motorPower(
      constrain(
        leftPower,
        -255,
        255
      ),
      constrain(
        rightPower,
        -255,
        255
      )
    );

    finishMotionControlCycle(cycleUs);
  }

  // =====================================================
  // 24. 終了処理
  // =====================================================
  if (brake) {
    brakeCurrentMotors();
  } else {
    stopMotors();
  }

  delay(20);
  return true;
}

void arc(float radiusMm, float deg, int maxPower, int minPower, int brake) {
  runArc(radiusMm, deg, maxPower, minPower, brake);
}

bool parallelShift(float distanceMm, RobotSide side, int maxPower, int minPower) {
  if (!isfinite(distanceMm) || distanceMm < 0.0f ||
      (side != L && side != R) || maxPower <= 0 || maxPower > 255) {
    stopMotors();
    Serial.println("PARALLEL SHIFT ERROR: invalid distance, side or power");
    return false;
  }
  if (distanceMm == 0.0f) {
    stopMotors();
    return true;
  }

  // 両方とも前進し、右180度→左180度（左移動は逆順）。
  // 各半円で横に2rずつ、合計4r移動し、前後位置・方位は元通り。
  const float radiusMm = distanceMm / 4.0f;
  const float firstDeg = side == R ? 180.0f : -180.0f;
  // 1本目の異常終了時には2本目を開始しない。
  if (!runArc(radiusMm, firstDeg, maxPower, minPower, 1)) return false;
  return runArc(radiusMm, -firstDeg, maxPower, minPower, 1);
}

void motorBrakeTest() {
  // =====================================================
  // 1. 通常停止の確認
  // =====================================================
  Serial.println("TEST 1: NORMAL STOP");

  motorPower(-100, 100);
  delay(1000);

  stopMotors();
  delay(1000);

  // =====================================================
  // 2. A/B両方HIGHの確認
  // =====================================================
  Serial.println("TEST 2: BOTH HIGH");

  motorPower(-100, 100);
  delay(1000);

  analogWrite(M2A, 255);
  analogWrite(M2B, 255);

  analogWrite(M1A, 255);
  analogWrite(M1B, 255);

  delay(1000);

  stopMotors();
  delay(1000);

  // =====================================================
  // 3. 逆位相ブレーキの確認
  // =====================================================
  Serial.println("TEST 3: REVERSE BRAKE");

  motorPower(-100, 100);
  delay(1000);

  brakeCurrentMotors();
  delay(1000);

  Serial.println("BRAKE TEST FINISHED");
}

// =====================================================
//マイブロック___フォルダごと分けたい！
// =====================================================

//ロボット動作時リセット
void robot_reset() {
  calibrateGyroZ(10);
  startServoMoveByPin(0, -140, 255);  //全上げ
  startServoMoveByPin(2, 15, 255);    //閉じる
  startServoMoveByPin(3, 30, 255);    //後ろ全上げ   ★★変更★★
}

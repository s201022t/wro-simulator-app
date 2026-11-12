#ifndef PI_COLOR_LIB_H
#define PI_COLOR_LIB_H

#include <Arduino.h>

// =========================================================
// MOTION2350 Pro 基板上 RGB LED
// motion_lib.cpp 側で定義されている関数を使う
// =========================================================
void boardRgbSet(uint8_t r, uint8_t g, uint8_t b);
void boardRgbOff();

// =========================================================
// 通信ピン
// =========================================================
extern const int MOTION_TX_PIN;
extern const int MOTION_RX_PIN;

// =========================================================
// 一番最初のコードと同じ変数名
// SET0、つまり既存の12個ROI用です。
// =========================================================
extern const int ROI_COUNT;
extern const int GROUP_COUNT;

extern String Group1;
extern String Group2;
extern String Group3;

// =========================================================
// 6GROUP 最終判定結果
// =========================================================
extern String GroupA;
extern String GroupB;
extern String GroupC;
extern String GroupD;
extern String GroupE;
extern String GroupF;

extern bool g_colorReadValid;
extern bool g_manualColorMode;

extern String roiColor[];
extern String groupColor[];

// RGB値追加分
// 既存SET0のROI番号と同じ番号で使います。
extern int roiR[];
extern int roiG[];
extern int roiB[];

// =========================================================
// 追加SET用
// SET1〜SET5を使う場合だけ使います。
// 既存のroiColor[] / groupColor[] / Group1〜3はそのまま残します。
// =========================================================
#define COLOR_SET_COUNT 6
#define MAX_ROI_COUNT 20

extern String colorSetName[];
extern int roiCount[];
extern bool setColorReadValid[];

extern String setRoiColor[COLOR_SET_COUNT][MAX_ROI_COUNT + 1];
extern int setRoiR[COLOR_SET_COUNT][MAX_ROI_COUNT + 1];
extern int setRoiG[COLOR_SET_COUNT][MAX_ROI_COUNT + 1];
extern int setRoiB[COLOR_SET_COUNT][MAX_ROI_COUNT + 1];

// =========================================================
// 初期化
// =========================================================
void piColorSetup();

// =========================================================
// Piから1行読む
// =========================================================
String readLineFromPi(unsigned long timeoutMs);

// =========================================================
// パース関数
// 旧形式をそのまま残します。
// =========================================================
bool parseRoiColorLine(String line, int &index, String &color);
bool parseGroupColorLine(String line, int &index, String &color);

// RGB追加形式: RGB,1,100,120,130
bool parseRoiRgbLine(String line, int &index, int &r, int &g, int &b);

// 複数SET用形式
bool parseSetRoiColorLine(String line, int &setId, int &index, String &color);
bool parseSetRoiRgbLine(String line, int &setId, int &index, int &r, int &g, int &b);

// =========================================================
// 表示
// =========================================================
void printAllRoiColors();
void printAllRoiRgb();
void printAllGroupColors();
void checkGroupColors();

void printAllRoiColors(int setId);
void printAllRoiRgb(int setId);
void printColorSetStatus(int setId);

bool setManualGroupColors(String csv);

// =========================================================
// Piへコマンド送信
// =========================================================
void sendSimpleCommandToPi(const String &command, unsigned long timeoutMs);

// =========================================================
// 写真撮影
// =========================================================
void requestPiSnap();
void requestPiSnap(int setId);

// =========================================================
// ROI Picker起動
// =========================================================
void requestPiPicker();
void requestPiPicker(int setId);

bool readRoi();

// =========================================================
// 色読み
// =========================================================
bool requestColorRead();
bool requestColorRead(int setId);

// =========================================================
// 色読み + Group1〜3へ保存
// =========================================================
bool readColorAndSetVariable();

// =========================================================
// 既存SET0の色データ取得
// =========================================================
String getRoiColor(int index);
int getRoiR(int index);
int getRoiG(int index);
int getRoiB(int index);

String getGroupColor(int index);

// =========================================================
// 追加SET用の取得
// SET0を指定した場合は、既存のroiColor[]を返します。
// =========================================================
String getRoiColor(int setId, int index);
int getRoiR(int setId, int index);
int getRoiG(int setId, int index);
int getRoiB(int setId, int index);
int getRoiCount(int setId);
String getColorSetName(int setId);

// =========================================================
// 成功確認
// =========================================================
bool hasValidColorRead();
bool hasValidColorRead(int setId);

// =========================================================
// GROUP変数更新
// =========================================================
void setting_variable();

// =========================================================
// シリアルバッファクリア
// =========================================================
void clearPiSerialBuffer();

#endif
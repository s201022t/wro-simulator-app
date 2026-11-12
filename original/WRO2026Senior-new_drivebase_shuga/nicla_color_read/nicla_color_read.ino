#include <Arduino.h>
#include "camera.h"
#include "gc2145.h"
#include <math.h>
#include <string.h>

// ============================================================
// Nicla Vision / MOTION2350 Pro / PC ROI Picker 統合版
//
// MOTION側（Serial1、115200bps）から受け取る命令:
//   READ / READ0 / READ,0
//   SNAP / SNAP0 / SNAP,0
//   PICK / PICK0 / PICK,0
//   PING
//
// READ時にMOTIONへ返す形式:
//   ROI,0,1,G
//   RGB,0,1,100,120,80
//   ...
//   GROUP,0,1,G
//   GROUP,0,2,B
//   GROUP,0,3,Y
//   END,0
//
// 3回ともノイズを検出した場合:
//   ERR:READ_FAILED,0,NOISE_3_TIMES
//
// PC側（USB Serial）から受け取る命令:
//   CAPTURE_AND_GET
//   GET_LAST_PHOTO
//   CAPTURE
//   READ_TEST
//   GET_ROI_CONFIG
//   SET_ROI,番号,中心X,中心Y,幅,高さ
//   GET_HSV_CONFIG
//   SET_HSV,色,Hmin,Hmax,Smin,Smax,Vmin,Vmax
//   GET_COLOR_ADJUST
//   SET_COLOR_ADJUST,明るさ%,彩度%
//   GET_REPAIR_STATUS
//   STATUS
//   CLEAR_PHOTO
//   PING
//
// 重要:
//   ROI/HSV設定と保存画像はRAM保持です。
//   リセットまたは電源OFFで初期値に戻ります。
// ============================================================


// ============================================================
// 基本設定
// ============================================================

constexpr uint32_t IMAGE_WIDTH = 320;
constexpr uint32_t IMAGE_HEIGHT = 240;
constexpr uint32_t BYTES_PER_PIXEL = 2;
constexpr uint32_t ROW_SIZE = IMAGE_WIDTH * BYTES_PER_PIXEL;
constexpr uint32_t IMAGE_SIZE =
    IMAGE_WIDTH * IMAGE_HEIGHT * BYTES_PER_PIXEL;

constexpr int ROI_COUNT = 12;
constexpr int GROUP_COUNT = 3;

// ============================================================
// GROUPとROIの対応
// 配列の数値は0始まりのROIインデックス
// ============================================================
//
// GROUP1 = ROI1, ROI2, ROI5, ROI6
// GROUP2 = ROI9, ROI10, ROI11, ROI12
// GROUP3 = ROI3, ROI4, ROI7, ROI8
//
constexpr uint8_t GROUP_ROI_INDEXES[GROUP_COUNT][4] = {
  { 0, 1, 4, 5 },     // GROUP1
  { 8, 9, 10, 11 },   // GROUP2
  { 2, 3, 6, 7 }      // GROUP3
};

constexpr uint32_t USB_BAUD_RATE = 115200;
constexpr uint32_t MOTION_BAUD_RATE = 115200;

// PC表示画像は上下反転しているため、ROI判定時も論理座標へ合わせる
constexpr bool FLIP_VERTICAL = true;
constexpr bool FLIP_HORIZONTAL = false;


// ============================================================
// RGB565横帯ノイズの検出設定
//
// 対象とするノイズ:
//   確認済み画像2枚で発生していた、画面全幅にまたがる
//   RGB565の1バイト位相ずれ。
//
// 動作:
//   ノイズを検出した画像は修復せずに破棄し、再撮影する。
//   色読みでは最大3回撮影し、3回ともノイズだった場合は
//   MOTIONへエラーを返す。
//
// 注意:
//   画面の一部分だけで起きる破損や、2バイト以上のずれは
//   現在の検出対象外。
// ============================================================

constexpr bool ENABLE_FRAME_NOISE_DETECTION = false;

// 行スコア計算時の横方向サンプリング設定
constexpr int REPAIR_SAMPLE_X_START = 8;
constexpr int REPAIR_SAMPLE_X_END = 312;  // この値は含まない
constexpr int REPAIR_SAMPLE_X_STEP = 4;

// 添付された破損画像2枚から決めた初期しきい値
// 正常読みの平均隣接色差が100以上で、
// 1バイト補正後が正常読みの65%以下、かつ40以上改善した行を候補とする。
constexpr uint32_t REPAIR_MIN_NORMAL_SCORE = 200;
constexpr uint32_t REPAIR_MAX_RATIO_PERCENT = 65;
constexpr uint32_t REPAIR_MIN_IMPROVEMENT = 40;

// 単発の誤検出を避けるため、3行以上連続した場合だけノイズとする。
constexpr int REPAIR_MIN_RUN_ROWS = 3;
constexpr int REPAIR_MAX_REGIONS = 8;

// ROIの上下に余裕を持たせて検査する行数
constexpr int REPAIR_ROI_Y_MARGIN = 4;

// 色読み時の最大撮影回数
constexpr int COLOR_READ_MAX_CAPTURE_ATTEMPTS = 5;


// ============================================================
// 型定義
// ============================================================

struct RoiConfig {
  int centerX;
  int centerY;
  int width;
  int height;
};

struct HsvRange {
  int hMin;
  int hMax;
  int sMin;
  int sMax;
  int vMin;
  int vMax;
};

enum ColorCode : uint8_t {
  COLOR_UNKNOWN = 0,
  COLOR_BLACK,
  COLOR_WHITE,
  COLOR_YELLOW,
  COLOR_RED,
  COLOR_BLUE,
  COLOR_GREEN
};

struct RoiResult {
  ColorCode color;

  uint8_t averageR;
  uint8_t averageG;
  uint8_t averageB;

  uint32_t countBlack;
  uint32_t countWhite;
  uint32_t countGreen;
  uint32_t countBlue;
  uint32_t countYellow;
  uint32_t countRed;
};

struct GroupColorCounts {
  uint8_t green;
  uint8_t blue;
  uint8_t white;
  uint8_t yellow;
};

struct RepairRegion {
  int cameraStartRow;
  int cameraEndRow;
};

enum RepairScope : uint8_t {
  REPAIR_SCOPE_FULL_IMAGE = 0,
  REPAIR_SCOPE_ROI_ROWS
};


// ============================================================
// カメラと画像RAM
// ============================================================

GC2145 cameraSensor;
Camera camera(cameraSensor);
FrameBuffer cameraFrameBuffer;

static uint8_t savedImage[IMAGE_SIZE];

bool savedImageValid = false;
uint32_t savedImageNumber = 0;

// 直近フレームのノイズ検出情報（既存ステータス名は維持）
bool lastFrameRepairApplied = false;
bool lastFrameRepairRegionOverflow = false;
int lastFrameRepairRegionCount = 0;
RepairRegion lastFrameRepairRegions[REPAIR_MAX_REGIONS];
uint32_t lastFrameRepairAnalyzeUs = 0;
uint32_t lastFrameRepairApplyUs = 0;
uint32_t lastFrameRepairTotalUs = 0;
uint32_t lastFrameRepairBytes = 0;
RepairScope lastCaptureRepairScope = REPAIR_SCOPE_FULL_IMAGE;


// ============================================================
// ROI初期設定
// PC側ROI PickerからSET_ROIで変更する
// ============================================================
// ===== NiclaRoiApp 自動生成設定 =====
RoiConfig roiConfigs[ROI_COUNT] = {
  { 245, 115, 50, 50 },  // ROI1
  { 57, 89, 5, 25 },  // ROI2
  { 116, 113, 5, 25 },  // ROI3
  { 95, 114, 5, 25 },  // ROI4
  { 116, 149, 5, 25 },  // ROI5
  { 135, 139, 5, 25 },  // ROI6
  { 89, 132, 5, 25 },  // ROI7
  { 131, 128, 5, 25 },  // ROI8
  { 93, 142, 5, 25 },  // ROI9
  { 111, 138, 5, 25 },  // ROI10
  { 99, 138, 5, 25 },  // ROI11
  { 87, 83, 5, 25 }  // ROI12
};
HsvRange hsvBlack = { 0, 179, 0, 255, 0, 50 };
HsvRange hsvWhite = { 0, 179, 0, 70, 140, 255 };
HsvRange hsvYellow = { 18, 42, 70, 255, 50, 255 };
HsvRange hsvRed = { 170, 10, 100, 255, 50, 255 };
HsvRange hsvBlue = { 96, 128, 110, 255, 65, 255 };
HsvRange hsvGreen = { 43, 89, 50, 255, 50, 255 };
int brightnessPercent = 100;
int saturationPercent = 160;

// READ_ROI / READ_TEST用 判定対象色
bool checklistBlackEnabled = true;
bool checklistWhiteEnabled = true;
bool checklistYellowEnabled = true;
bool checklistRedEnabled = true;
bool checklistBlueEnabled = true;
bool checklistGreenEnabled = true;
// ===== 自動生成設定ここまで =====



// ============================================================
// 判定結果
// ============================================================

RoiResult roiResults[ROI_COUNT];

ColorCode groupResults[GROUP_COUNT] = {
  COLOR_UNKNOWN,
  COLOR_UNKNOWN,
  COLOR_UNKNOWN
};


// ============================================================
// シリアル受信バッファ
// ============================================================

String usbLineBuffer = "";
String motionLineBuffer = "";


// ============================================================
// 関数プロトタイプ
// Arduino IDEの自動プロトタイプ生成による型エラー対策
// ============================================================

int clampInt(int value, int minimum, int maximum);

uint8_t applyPercentCorrection(
  uint8_t originalValue,
  int percent
);

void applyHsvCorrection(
  uint8_t &saturation,
  uint8_t &value
);

int splitCsv(
  const String &line,
  String parts[],
  int maximumParts
);

const char* colorCodeToText(ColorCode color);

void decodeRgb565Bytes(
  uint8_t highByte,
  uint8_t lowByte,
  uint8_t &red,
  uint8_t &green,
  uint8_t &blue
);

uint32_t calculateRowPhaseScore(
  const uint8_t* imageData,
  int cameraY,
  int bytePhaseOffset
);

bool isDamagedRgb565Row(
  const uint8_t* imageData,
  int cameraY
);

void resetLastRepairStatus();

void getRepairScanCameraRowRange(
  RepairScope repairScope,
  int &cameraStartRow,
  int &cameraEndRow
);

bool detectRgb565Noise(
  const uint8_t* imageData,
  RepairScope repairScope
);

void sendRepairStatusToUsb();

void readRgb565Pixel(
  const uint8_t* imageData,
  int logicalX,
  int logicalY,
  uint8_t &red,
  uint8_t &green,
  uint8_t &blue
);

void rgbToOpenCvHsv(
  uint8_t red,
  uint8_t green,
  uint8_t blue,
  uint8_t &hue,
  uint8_t &saturation,
  uint8_t &value
);

bool hsvInRange(
  uint8_t hue,
  uint8_t saturation,
  uint8_t value,
  const HsvRange &range
);

ColorCode classifyPixel(
  uint8_t red,
  uint8_t green,
  uint8_t blue
);

ColorCode selectRoiMajorityColor(
  uint32_t countGreen,
  uint32_t countBlue,
  uint32_t countWhite,
  uint32_t countYellow
);

ColorCode selectRoiMajorityColorWithChecklist(
  uint32_t countBlack,
  uint32_t countRed,
  uint32_t countGreen,
  uint32_t countBlue,
  uint32_t countWhite,
  uint32_t countYellow
);


// ============================================================
// ROIだけ読むモード用の5色多数決
// チェックされていない色は候補から除外する。
// 同数時は BLACK -> GREEN -> BLUE -> WHITE -> YELLOW の順。
// ============================================================
ColorCode selectRoiMajorityColorWithChecklist(
  uint32_t countBlack,
  uint32_t countRed,
  uint32_t countGreen,
  uint32_t countBlue,
  uint32_t countWhite,
  uint32_t countYellow
) {
  ColorCode selectedColor = COLOR_UNKNOWN;
  uint32_t maximumCount = 0;

  if (checklistBlackEnabled && countBlack > maximumCount) {
    maximumCount = countBlack;
    selectedColor = COLOR_BLACK;
  }

  if (checklistRedEnabled && countRed > maximumCount) {
    maximumCount = countRed;
    selectedColor = COLOR_RED;
  }

  if (checklistGreenEnabled && countGreen > maximumCount) {
    maximumCount = countGreen;
    selectedColor = COLOR_GREEN;
  }

  if (checklistBlueEnabled && countBlue > maximumCount) {
    maximumCount = countBlue;
    selectedColor = COLOR_BLUE;
  }

  if (checklistWhiteEnabled && countWhite > maximumCount) {
    maximumCount = countWhite;
    selectedColor = COLOR_WHITE;
  }

  if (checklistYellowEnabled && countYellow > maximumCount) {
    maximumCount = countYellow;
    selectedColor = COLOR_YELLOW;
  }

  return selectedColor;
}

RoiResult analyzeSingleRoi(
  const uint8_t* imageData,
  const RoiConfig &roi,
  bool useColorChecklist
);

void analyzeAllRois(
  const uint8_t* imageData,
  bool useColorChecklist
);

uint8_t getColorCount(
  const GroupColorCounts &counts,
  ColorCode color
);

void makeRankedColors(
  const GroupColorCounts &counts,
  ColorCode rankedColors[4]
);

void decideGroupColors();

bool captureAndStoreImage(RepairScope repairScope);

void motionPrintLine(const String &line);

void sendResultsToMotion(int setId);

void sendResultsToUsb(int setId);

bool performColorRead(
  int setId,
  bool sendToMotion,
  bool sendToUsb,
  bool useColorChecklist
);

bool writeAllUsb(
  const uint8_t* data,
  uint32_t size
);

bool waitUsbCommand(
  const String &expected,
  uint32_t timeoutMs
);

void sendSavedImageToUsb();

void sendRoiConfigToUsb();

void printSingleHsvConfig(
  const char* name,
  const HsvRange &range
);

void sendHsvConfigToUsb();

void sendColorAdjustToUsb();

void handleSetRoiCommand(const String &command);

void handleSetHsvCommand(const String &command);

void handleSetColorAdjustCommand(const String &command);

void handleSetEnabledColorsCommand(const String &command);
void sendEnabledColorsToUsb();

void processUsbCommand(String command);

void updateUsbCommands();

String cleanMotionCommand(String command);

bool parseNamedMotionCommand(
  const String &command,
  const String &name,
  int &setId
);

void processMotionCommand(String command);

void updateMotionCommands();


// ============================================================
// 共通補助
// ============================================================

int clampInt(int value, int minimum, int maximum) {
  if (value < minimum) {
    return minimum;
  }

  if (value > maximum) {
    return maximum;
  }

  return value;
}

uint8_t applyPercentCorrection(
  uint8_t originalValue,
  int percent
) {
  percent = clampInt(
    percent,
    0,
    400
  );

  // PC側と同じ整数式。+50で100除算時に四捨五入する。
  int correctedValue =
      (
        static_cast<int>(originalValue) *
        percent +
        50
      ) /
      100;

  correctedValue = clampInt(
    correctedValue,
    0,
    255
  );

  return static_cast<uint8_t>(
    correctedValue
  );
}

void applyHsvCorrection(
  uint8_t &saturation,
  uint8_t &value
) {
  saturation =
      applyPercentCorrection(
        saturation,
        saturationPercent
      );

  value =
      applyPercentCorrection(
        value,
        brightnessPercent
      );
}

int splitCsv(
  const String &line,
  String parts[],
  int maximumParts
) {
  int count = 0;
  int startPosition = 0;

  while (count < maximumParts) {
    int commaPosition =
        line.indexOf(',', startPosition);

    if (commaPosition < 0) {
      parts[count] =
          line.substring(startPosition);

      parts[count].trim();

      count++;
      break;
    }

    parts[count] =
        line.substring(
          startPosition,
          commaPosition
        );

    parts[count].trim();

    count++;
    startPosition = commaPosition + 1;
  }

  return count;
}

const char* colorCodeToText(ColorCode color) {
  switch (color) {
    case COLOR_BLACK:
      return "BLACK";

    case COLOR_WHITE:
      return "W";

    case COLOR_YELLOW:
      return "Y";

    case COLOR_RED:
      return "R";

    case COLOR_BLUE:
      return "B";

    case COLOR_GREEN:
      return "G";

    default:
      return "UNKNOWN";
  }
}


// ============================================================
// RGB565横帯ノイズの検出
// ============================================================

void decodeRgb565Bytes(
  uint8_t highByte,
  uint8_t lowByte,
  uint8_t &red,
  uint8_t &green,
  uint8_t &blue
) {
  uint16_t pixel =
      (
        static_cast<uint16_t>(highByte) << 8
      ) |
      static_cast<uint16_t>(lowByte);

  uint8_t red5 =
      static_cast<uint8_t>(
        (pixel >> 11) & 0x1F
      );

  uint8_t green6 =
      static_cast<uint8_t>(
        (pixel >> 5) & 0x3F
      );

  uint8_t blue5 =
      static_cast<uint8_t>(
        pixel & 0x1F
      );

  red =
      static_cast<uint8_t>(
        (red5 << 3) |
        (red5 >> 2)
      );

  green =
      static_cast<uint8_t>(
        (green6 << 2) |
        (green6 >> 4)
      );

  blue =
      static_cast<uint8_t>(
        (blue5 << 3) |
        (blue5 >> 2)
      );
}

uint32_t calculateRowPhaseScore(
  const uint8_t* imageData,
  int cameraY,
  int bytePhaseOffset
) {
  if (
    imageData == nullptr ||
    cameraY < 0 ||
    cameraY >= static_cast<int>(IMAGE_HEIGHT)
  ) {
    return UINT32_MAX;
  }

  const uint32_t rowStart =
      static_cast<uint32_t>(cameraY) *
      ROW_SIZE;

  bool hasPreviousPixel = false;

  uint8_t previousRed = 0;
  uint8_t previousGreen = 0;
  uint8_t previousBlue = 0;

  uint32_t differenceSum = 0;
  uint32_t differenceCount = 0;

  for (
    int x = REPAIR_SAMPLE_X_START;
    x < REPAIR_SAMPLE_X_END;
    x += REPAIR_SAMPLE_X_STEP
  ) {
    int32_t highIndex =
        static_cast<int32_t>(rowStart) +
        static_cast<int32_t>(x * BYTES_PER_PIXEL) +
        bytePhaseOffset;

    int32_t lowIndex =
        highIndex + 1;

    if (
      highIndex < 0 ||
      lowIndex < 0 ||
      highIndex >= static_cast<int32_t>(IMAGE_SIZE) ||
      lowIndex >= static_cast<int32_t>(IMAGE_SIZE)
    ) {
      continue;
    }

    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;

    decodeRgb565Bytes(
      imageData[highIndex],
      imageData[lowIndex],
      red,
      green,
      blue
    );

    if (hasPreviousPixel) {
      differenceSum +=
          static_cast<uint32_t>(
            abs(
              static_cast<int>(red) -
              static_cast<int>(previousRed)
            )
          );

      differenceSum +=
          static_cast<uint32_t>(
            abs(
              static_cast<int>(green) -
              static_cast<int>(previousGreen)
            )
          );

      differenceSum +=
          static_cast<uint32_t>(
            abs(
              static_cast<int>(blue) -
              static_cast<int>(previousBlue)
            )
          );

      differenceCount++;
    }

    previousRed = red;
    previousGreen = green;
    previousBlue = blue;
    hasPreviousPixel = true;
  }

  if (differenceCount == 0) {
    return UINT32_MAX;
  }

  return differenceSum / differenceCount;
}

bool isDamagedRgb565Row(
  const uint8_t* imageData,
  int cameraY
) {
  // 通常解釈:
  //   [data[index], data[index + 1]]
  const uint32_t normalScore =
      calculateRowPhaseScore(
        imageData,
        cameraY,
        0
      );

  // 確認済み破損を補正した解釈:
  //   [data[index - 1], data[index]]
  // calculateRowPhaseScore()ではhighIndexに-1を加える。
  const uint32_t repairedScore =
      calculateRowPhaseScore(
        imageData,
        cameraY,
        -1
      );

  if (
    normalScore == UINT32_MAX ||
    repairedScore == UINT32_MAX
  ) {
    return false;
  }

  if (normalScore < REPAIR_MIN_NORMAL_SCORE) {
    return false;
  }

  if (
    normalScore <
      repairedScore +
      REPAIR_MIN_IMPROVEMENT
  ) {
    return false;
  }

  return (
    repairedScore * 100UL <=
    normalScore * REPAIR_MAX_RATIO_PERCENT
  );
}

void resetLastRepairStatus() {
  lastFrameRepairApplied = false;
  lastFrameRepairRegionOverflow = false;
  lastFrameRepairRegionCount = 0;
  lastFrameRepairAnalyzeUs = 0;
  lastFrameRepairApplyUs = 0;
  lastFrameRepairTotalUs = 0;
  lastFrameRepairBytes = 0;

  for (
    int i = 0;
    i < REPAIR_MAX_REGIONS;
    i++
  ) {
    lastFrameRepairRegions[i].cameraStartRow = -1;
    lastFrameRepairRegions[i].cameraEndRow = -1;
  }
}

void getRepairScanCameraRowRange(
  RepairScope repairScope,
  int &cameraStartRow,
  int &cameraEndRow
) {
  if (repairScope == REPAIR_SCOPE_FULL_IMAGE) {
    cameraStartRow = 0;
    cameraEndRow = IMAGE_HEIGHT - 1;
    return;
  }

  int logicalMinY = IMAGE_HEIGHT - 1;
  int logicalMaxY = 0;
  bool foundValidRoi = false;

  for (int i = 0; i < ROI_COUNT; i++) {
    const int roiHeight =
        clampInt(
          roiConfigs[i].height,
          1,
          IMAGE_HEIGHT
        );

    const int rawStartY =
        roiConfigs[i].centerY -
        roiHeight / 2;

    const int rawEndYExclusive =
        rawStartY + roiHeight;

    const int logicalStartY =
        clampInt(
          rawStartY,
          0,
          IMAGE_HEIGHT
        );

    const int logicalEndYExclusive =
        clampInt(
          rawEndYExclusive,
          0,
          IMAGE_HEIGHT
        );

    if (logicalEndYExclusive <= logicalStartY) {
      continue;
    }

    logicalMinY =
        min(
          logicalMinY,
          logicalStartY
        );

    logicalMaxY =
        max(
          logicalMaxY,
          logicalEndYExclusive - 1
        );

    foundValidRoi = true;
  }

  if (!foundValidRoi) {
    // ROI設定が不正な場合は安全側で全行を検査する。
    cameraStartRow = 0;
    cameraEndRow = IMAGE_HEIGHT - 1;
    return;
  }

  logicalMinY =
      clampInt(
        logicalMinY - REPAIR_ROI_Y_MARGIN,
        0,
        IMAGE_HEIGHT - 1
      );

  logicalMaxY =
      clampInt(
        logicalMaxY + REPAIR_ROI_Y_MARGIN,
        0,
        IMAGE_HEIGHT - 1
      );

  if (FLIP_VERTICAL) {
    cameraStartRow =
        IMAGE_HEIGHT - 1 - logicalMaxY;

    cameraEndRow =
        IMAGE_HEIGHT - 1 - logicalMinY;

  } else {
    cameraStartRow = logicalMinY;
    cameraEndRow = logicalMaxY;
  }

  if (cameraStartRow > cameraEndRow) {
    const int temporary = cameraStartRow;
    cameraStartRow = cameraEndRow;
    cameraEndRow = temporary;
  }
}

bool detectRgb565Noise(
  const uint8_t* imageData,
  RepairScope repairScope
) {
  resetLastRepairStatus();

  const uint32_t totalStartUs =
      micros();

  if (!ENABLE_FRAME_NOISE_DETECTION) {
    lastFrameRepairTotalUs =
        micros() -
        totalStartUs;

    return false;
  }

  if (imageData == nullptr) {
    lastFrameRepairTotalUs =
        micros() -
        totalStartUs;

    return true;
  }

  bool damagedRows[IMAGE_HEIGHT] = { false };

  int scanStartRow = 0;
  int scanEndRow = IMAGE_HEIGHT - 1;

  getRepairScanCameraRowRange(
    repairScope,
    scanStartRow,
    scanEndRow
  );

  for (
    int cameraY = scanStartRow;
    cameraY <= scanEndRow;
    cameraY++
  ) {
    damagedRows[cameraY] =
        isDamagedRgb565Row(
          imageData,
          cameraY
        );
  }

  // 1行だけ判定が抜けた場合は前後をつなぐ。
  for (
    int cameraY = scanStartRow + 1;
    cameraY < scanEndRow;
    cameraY++
  ) {
    if (
      !damagedRows[cameraY] &&
      damagedRows[cameraY - 1] &&
      damagedRows[cameraY + 1]
    ) {
      damagedRows[cameraY] = true;
    }
  }

  int cameraY = scanStartRow;

  while (
    cameraY <= scanEndRow
  ) {
    if (!damagedRows[cameraY]) {
      cameraY++;
      continue;
    }

    const int runStart =
        cameraY;

    while (
      cameraY + 1 <= scanEndRow &&
      damagedRows[cameraY + 1]
    ) {
      cameraY++;
    }

    const int runEnd =
        cameraY;

    const int runLength =
        runEnd -
        runStart +
        1;

    if (
      runLength >=
      REPAIR_MIN_RUN_ROWS
    ) {
      if (
        lastFrameRepairRegionCount <
        REPAIR_MAX_REGIONS
      ) {
        lastFrameRepairRegions[
          lastFrameRepairRegionCount
        ].cameraStartRow =
            runStart;

        lastFrameRepairRegions[
          lastFrameRepairRegionCount
        ].cameraEndRow =
            runEnd;

        lastFrameRepairRegionCount++;

      } else {
        lastFrameRepairRegionOverflow = true;
      }
    }

    cameraY++;
  }

  // 修復処理は行わない。
  // 画像データは撮影時の内容のまま保持する。
  lastFrameRepairApplied = false;
  lastFrameRepairApplyUs = 0;
  lastFrameRepairBytes = 0;

  lastFrameRepairAnalyzeUs =
      micros() -
      totalStartUs;

  lastFrameRepairTotalUs =
      lastFrameRepairAnalyzeUs;

  return (
    lastFrameRepairRegionOverflow ||
    lastFrameRepairRegionCount > 0
  );
}

void sendRepairStatusToUsb() {
  Serial.println(
    "REPAIR_STATUS_BEGIN"
  );

  Serial.print(
    "REPAIR_ENABLED,"
  );

  Serial.println(
    ENABLE_FRAME_NOISE_DETECTION ?
    1 :
    0
  );

  Serial.print(
    "REPAIR_APPLIED,"
  );

  Serial.println(
    lastFrameRepairApplied ?
    1 :
    0
  );

  Serial.print(
    "REPAIR_REGION_OVERFLOW,"
  );

  Serial.println(
    lastFrameRepairRegionOverflow ?
    1 :
    0
  );

  Serial.print(
    "REPAIR_REGION_COUNT,"
  );

  Serial.println(
    lastFrameRepairRegionCount
  );

  for (
    int regionIndex = 0;
    regionIndex < lastFrameRepairRegionCount;
    regionIndex++
  ) {
    const int cameraStart =
        lastFrameRepairRegions[
          regionIndex
        ].cameraStartRow;

    const int cameraEnd =
        lastFrameRepairRegions[
          regionIndex
        ].cameraEndRow;

    int displayStart = cameraStart;
    int displayEnd = cameraEnd;

    if (FLIP_VERTICAL) {
      displayStart =
          IMAGE_HEIGHT - 1 -
          cameraEnd;

      displayEnd =
          IMAGE_HEIGHT - 1 -
          cameraStart;
    }

    Serial.print(
      "REPAIR_REGION,"
    );

    Serial.print(
      regionIndex + 1
    );

    Serial.print(
      ",CAMERA_ROWS,"
    );

    Serial.print(cameraStart);
    Serial.print(",");
    Serial.print(cameraEnd);

    Serial.print(
      ",DISPLAY_ROWS,"
    );

    Serial.print(displayStart);
    Serial.print(",");
    Serial.println(displayEnd);
  }

  Serial.print(
    "REPAIR_BYTES,"
  );

  Serial.println(
    lastFrameRepairBytes
  );

  Serial.print(
    "REPAIR_ANALYZE_US,"
  );

  Serial.println(
    lastFrameRepairAnalyzeUs
  );

  Serial.print(
    "REPAIR_APPLY_US,"
  );

  Serial.println(
    lastFrameRepairApplyUs
  );

  Serial.print(
    "REPAIR_TOTAL_US,"
  );

  Serial.println(
    lastFrameRepairTotalUs
  );

  Serial.println(
    "REPAIR_STATUS_END"
  );
}


// ============================================================
// RGB565読取り
// ============================================================

void readRgb565Pixel(
  const uint8_t* imageData,
  int logicalX,
  int logicalY,
  uint8_t &red,
  uint8_t &green,
  uint8_t &blue
) {
  logicalX = clampInt(
    logicalX,
    0,
    IMAGE_WIDTH - 1
  );

  logicalY = clampInt(
    logicalY,
    0,
    IMAGE_HEIGHT - 1
  );

  int cameraX = logicalX;
  int cameraY = logicalY;

  if (FLIP_HORIZONTAL) {
    cameraX =
        IMAGE_WIDTH - 1 - logicalX;
  }

  if (FLIP_VERTICAL) {
    cameraY =
        IMAGE_HEIGHT - 1 - logicalY;
  }

  uint32_t pixelIndex =
      static_cast<uint32_t>(cameraY) *
      IMAGE_WIDTH +
      static_cast<uint32_t>(cameraX);

  uint32_t byteIndex =
      pixelIndex *
      BYTES_PER_PIXEL;

  // savedImageは修復していない撮影データ。
  // 通常のRGB565として読む。
  decodeRgb565Bytes(
    imageData[byteIndex],
    imageData[byteIndex + 1],
    red,
    green,
    blue
  );
}


// ============================================================
// RGB -> OpenCV互換HSV
// ============================================================

void rgbToOpenCvHsv(
  uint8_t red,
  uint8_t green,
  uint8_t blue,
  uint8_t &hue,
  uint8_t &saturation,
  uint8_t &value
) {
  float r =
      static_cast<float>(red) /
      255.0f;

  float g =
      static_cast<float>(green) /
      255.0f;

  float b =
      static_cast<float>(blue) /
      255.0f;

  float maximumValue = r;

  if (g > maximumValue) {
    maximumValue = g;
  }

  if (b > maximumValue) {
    maximumValue = b;
  }

  float minimumValue = r;

  if (g < minimumValue) {
    minimumValue = g;
  }

  if (b < minimumValue) {
    minimumValue = b;
  }

  float difference =
      maximumValue -
      minimumValue;

  float hueDegrees = 0.0f;

  if (difference == 0.0f) {
    hueDegrees = 0.0f;

  } else if (
    maximumValue == r
  ) {
    hueDegrees =
        60.0f *
        fmodf(
          (g - b) /
          difference,
          6.0f
        );

  } else if (
    maximumValue == g
  ) {
    hueDegrees =
        60.0f *
        (
          ((b - r) / difference) +
          2.0f
        );

  } else {
    hueDegrees =
        60.0f *
        (
          ((r - g) / difference) +
          4.0f
        );
  }

  if (hueDegrees < 0.0f) {
    hueDegrees += 360.0f;
  }

  float saturationNormalized = 0.0f;

  if (maximumValue > 0.0f) {
    saturationNormalized =
        difference /
        maximumValue;
  }

  hue =
      static_cast<uint8_t>(
        clampInt(
          static_cast<int>(
            roundf(
              hueDegrees / 2.0f
            )
          ),
          0,
          179
        )
      );

  saturation =
      static_cast<uint8_t>(
        clampInt(
          static_cast<int>(
            roundf(
              saturationNormalized *
              255.0f
            )
          ),
          0,
          255
        )
      );

  value =
      static_cast<uint8_t>(
        clampInt(
          static_cast<int>(
            roundf(
              maximumValue *
              255.0f
            )
          ),
          0,
          255
        )
      );
}

bool hsvInRange(
  uint8_t hue,
  uint8_t saturation,
  uint8_t value,
  const HsvRange &range
) {
  bool hueValid = false;

  if (range.hMin <= range.hMax) {
    hueValid =
        hue >= range.hMin &&
        hue <= range.hMax;

  } else {
    hueValid =
        hue >= range.hMin ||
        hue <= range.hMax;
  }

  return (
    hueValid &&
    saturation >= range.sMin &&
    saturation <= range.sMax &&
    value >= range.vMin &&
    value <= range.vMax
  );
}


// ============================================================
// 1画素の色判定
// 後に一致した色を優先する
// ============================================================

ColorCode classifyPixel(
  uint8_t red,
  uint8_t green,
  uint8_t blue
) {
  uint8_t hue = 0;
  uint8_t saturation = 0;
  uint8_t value = 0;

  rgbToOpenCvHsv(
    red,
    green,
    blue,
    hue,
    saturation,
    value
  );

  // 5色判定の直前にS/Vを補正する。
  // Hは変更しないため、色相範囲は従来どおり使用できる。
  applyHsvCorrection(
    saturation,
    value
  );

  // どの色条件にも一致しない画素はUNKNOWN。
  // BLACKをROI結果として使うため、HSVのBLACK範囲に一致した画素だけBLACKにする。
  ColorCode result = COLOR_UNKNOWN;

  if (
    hsvInRange(
      hue,
      saturation,
      value,
      hsvBlack
    )
  ) {
    result = COLOR_BLACK;
  }

  if (
    hsvInRange(
      hue,
      saturation,
      value,
      hsvWhite
    )
  ) {
    result = COLOR_WHITE;
  }

  if (
    hsvInRange(
      hue,
      saturation,
      value,
      hsvYellow
    )
  ) {
    result = COLOR_YELLOW;
  }
    if (
    hsvInRange(
      hue,
      saturation,
      value,
      hsvRed
    )
  ) {
    result = COLOR_RED;
  }
  if (
    hsvInRange(
      hue,
      saturation,
      value,
      hsvBlue
    )
  ) {
    result = COLOR_BLUE;
  }

  if (
    hsvInRange(
      hue,
      saturation,
      value,
      hsvGreen
    )
  ) {
    result = COLOR_GREEN;
  }

  return result;
}


// ============================================================
// ROI多数決
// BLACKは多数決対象外
// 同数時: G -> B -> W -> Y
// ============================================================

ColorCode selectRoiMajorityColor(
  uint32_t countGreen,
  uint32_t countBlue,
  uint32_t countWhite,
  uint32_t countYellow
) {
  uint32_t maximumCount =
      countGreen;

  ColorCode selectedColor =
      COLOR_GREEN;

  if (countBlue > maximumCount) {
    maximumCount = countBlue;
    selectedColor = COLOR_BLUE;
  }

  if (countWhite > maximumCount) {
    maximumCount = countWhite;
    selectedColor = COLOR_WHITE;
  }

  if (countYellow > maximumCount) {
    maximumCount = countYellow;
    selectedColor = COLOR_YELLOW;
  }

  if (maximumCount == 0) {
    return COLOR_UNKNOWN;
  }

  return selectedColor;
}

RoiResult analyzeSingleRoi(
  const uint8_t* imageData,
  const RoiConfig &roi,
  bool useColorChecklist
) {
  RoiResult result;

  result.color = COLOR_UNKNOWN;

  result.averageR = 0;
  result.averageG = 0;
  result.averageB = 0;

  result.countBlack = 0;
  result.countWhite = 0;
  result.countGreen = 0;
  result.countBlue = 0;
  result.countYellow = 0;
  result.countRed = 0;

  int roiWidth =
      clampInt(
        roi.width,
        1,
        IMAGE_WIDTH
      );

  int roiHeight =
      clampInt(
        roi.height,
        1,
        IMAGE_HEIGHT
      );

  int rawStartX =
      roi.centerX -
      roiWidth / 2;

  int rawStartY =
      roi.centerY -
      roiHeight / 2;

  int rawEndX =
      rawStartX +
      roiWidth;

  int rawEndY =
      rawStartY +
      roiHeight;

  int startX =
      clampInt(
        rawStartX,
        0,
        IMAGE_WIDTH
      );

  int startY =
      clampInt(
        rawStartY,
        0,
        IMAGE_HEIGHT
      );

  int endX =
      clampInt(
        rawEndX,
        0,
        IMAGE_WIDTH
      );

  int endY =
      clampInt(
        rawEndY,
        0,
        IMAGE_HEIGHT
      );

  if (
    endX <= startX ||
    endY <= startY
  ) {
    return result;
  }

  uint64_t sumRed = 0;
  uint64_t sumGreen = 0;
  uint64_t sumBlue = 0;

  uint32_t pixelCount = 0;

  for (
    int y = startY;
    y < endY;
    y++
  ) {
    for (
      int x = startX;
      x < endX;
      x++
    ) {
      uint8_t red = 0;
      uint8_t green = 0;
      uint8_t blue = 0;

      readRgb565Pixel(
        imageData,
        x,
        y,
        red,
        green,
        blue
      );

      sumRed += red;
      sumGreen += green;
      sumBlue += blue;

      pixelCount++;

      ColorCode pixelColor =
          classifyPixel(
            red,
            green,
            blue
          );

      if (
        pixelColor ==
        COLOR_BLACK
      ) {
        result.countBlack++;

      } else if (
        pixelColor ==
        COLOR_WHITE
      ) {
        result.countWhite++;

      } else if (
        pixelColor ==
        COLOR_GREEN
      ) {
        result.countGreen++;

            } else if (
        pixelColor ==
        COLOR_BLUE
      ) {
        result.countBlue++;

      } else if (
        pixelColor ==
        COLOR_YELLOW
      ) {
        result.countYellow++;

      } else if (
        pixelColor ==
        COLOR_RED
      ) {
        result.countRed++;
      }

    }
  }

  if (pixelCount > 0) {
    result.averageR =
        static_cast<uint8_t>(
          sumRed /
          pixelCount
        );

    result.averageG =
        static_cast<uint8_t>(
          sumGreen /
          pixelCount
        );

    result.averageB =
        static_cast<uint8_t>(
          sumBlue /
          pixelCount
        );
  }

    if (useColorChecklist) {
    result.color =
        selectRoiMajorityColorWithChecklist(
          result.countBlack,
          result.countRed,
          result.countGreen,
          result.countBlue,
          result.countWhite,
          result.countYellow
        );

  } else {
    result.color =
        selectRoiMajorityColor(
          result.countGreen,
          result.countBlue,
          result.countWhite,
          result.countYellow
        );
  }

  return result;
}

void analyzeAllRois(
  const uint8_t* imageData,
  bool useColorChecklist
) {
  for (
    int i = 0;
    i < ROI_COUNT;
    i++
  ) {
    roiResults[i] =
        analyzeSingleRoi(
          imageData,
          roiConfigs[i],
          useColorChecklist
        );
  }
}


// ============================================================
// GROUP判定
//
// GROUP1 = ROI1, ROI2, ROI5, ROI6
// GROUP2 = ROI9, ROI10, ROI11, ROI12
// GROUP3 = ROI3, ROI4, ROI7, ROI8
//
// 各GROUPで多い色を選び、同じ色の重複を避ける
// ============================================================

uint8_t getColorCount(
  const GroupColorCounts &counts,
  ColorCode color
) {
  switch (color) {
    case COLOR_GREEN:
      return counts.green;

    case COLOR_BLUE:
      return counts.blue;

    case COLOR_WHITE:
      return counts.white;

    case COLOR_YELLOW:
      return counts.yellow;

    default:
      return 0;
  }
}

void makeRankedColors(
  const GroupColorCounts &counts,
  ColorCode rankedColors[4]
) {
  rankedColors[0] = COLOR_GREEN;
  rankedColors[1] = COLOR_BLUE;
  rankedColors[2] = COLOR_WHITE;
  rankedColors[3] = COLOR_YELLOW;

  for (int i = 0; i < 3; i++) {
    for (
      int j = i + 1;
      j < 4;
      j++
    ) {
      uint8_t countI =
          getColorCount(
            counts,
            rankedColors[i]
          );

      uint8_t countJ =
          getColorCount(
            counts,
            rankedColors[j]
          );

      if (countJ > countI) {
        ColorCode temporary =
            rankedColors[i];

        rankedColors[i] =
            rankedColors[j];

        rankedColors[j] =
            temporary;
      }
    }
  }
}

void decideGroupColors() {
  GroupColorCounts counts[GROUP_COUNT];

  // ==========================================================
  // 各GROUPに含まれる4個のROI結果を集計
  // ==========================================================
  for (
    int group = 0;
    group < GROUP_COUNT;
    group++
  ) {
    counts[group].green = 0;
    counts[group].blue = 0;
    counts[group].white = 0;
    counts[group].yellow = 0;

    for (
      int offset = 0;
      offset < 4;
      offset++
    ) {
      int roiIndex =
          GROUP_ROI_INDEXES[group][offset];

      ColorCode color =
          roiResults[roiIndex].color;

      if (
        color ==
        COLOR_GREEN
      ) {
        counts[group].green++;

      } else if (
        color ==
        COLOR_BLUE
      ) {
        counts[group].blue++;

      } else if (
        color ==
        COLOR_WHITE
      ) {
        counts[group].white++;

      } else if (
        color ==
        COLOR_YELLOW
      ) {
        counts[group].yellow++;
      }
    }
  }

  // ==========================================================
  // 各GROUPの色順位と最多票数を作成
  // ==========================================================
  ColorCode ranked[GROUP_COUNT][4];
  uint8_t bestCount[GROUP_COUNT];

  for (
    int group = 0;
    group < GROUP_COUNT;
    group++
  ) {
    makeRankedColors(
      counts[group],
      ranked[group]
    );

    bestCount[group] =
        getColorCount(
          counts[group],
          ranked[group][0]
        );
  }

  // ==========================================================
  // 最多票数が多いGROUPから処理
  // 同数の場合はGROUP番号が小さい方を優先
  // ==========================================================
  int processingOrder[GROUP_COUNT] = {
    0,
    1,
    2
  };

  for (
    int i = 0;
    i < GROUP_COUNT - 1;
    i++
  ) {
    for (
      int j = i + 1;
      j < GROUP_COUNT;
      j++
    ) {
      int groupI =
          processingOrder[i];

      int groupJ =
          processingOrder[j];

      if (
        bestCount[groupJ] >
        bestCount[groupI]
      ) {
        int temporary =
            processingOrder[i];

        processingOrder[i] =
            processingOrder[j];

        processingOrder[j] =
            temporary;
      }
    }
  }

  // ==========================================================
  // 使用済み色
  // ==========================================================
  bool greenUsed = false;
  bool blueUsed = false;
  bool whiteUsed = false;
  bool yellowUsed = false;

  // 最初はすべてUNKNOWN
  for (
    int group = 0;
    group < GROUP_COUNT;
    group++
  ) {
    groupResults[group] =
        COLOR_UNKNOWN;
  }

  // ==========================================================
  // 1票以上の未使用色だけを割り当てる
  // ==========================================================
  for (
    int orderIndex = 0;
    orderIndex < GROUP_COUNT;
    orderIndex++
  ) {
    int group =
        processingOrder[orderIndex];

    for (
      int rank = 0;
      rank < 4;
      rank++
    ) {
      ColorCode candidate =
          ranked[group][rank];

      uint8_t candidateCount =
          getColorCount(
            counts[group],
            candidate
          );

      // 0票の色は選ばない
      if (candidateCount == 0) {
        continue;
      }

      // 他GROUPで使用済みの色は選ばない
      bool alreadyUsed =
          (
            candidate ==
            COLOR_GREEN &&
            greenUsed
          ) ||
          (
            candidate ==
            COLOR_BLUE &&
            blueUsed
          ) ||
          (
            candidate ==
            COLOR_WHITE &&
            whiteUsed
          ) ||
          (
            candidate ==
            COLOR_YELLOW &&
            yellowUsed
          );

      if (alreadyUsed) {
        continue;
      }

      groupResults[group] =
          candidate;

      if (
        candidate ==
        COLOR_GREEN
      ) {
        greenUsed = true;

      } else if (
        candidate ==
        COLOR_BLUE
      ) {
        blueUsed = true;

      } else if (
        candidate ==
        COLOR_WHITE
      ) {
        whiteUsed = true;

      } else if (
        candidate ==
        COLOR_YELLOW
      ) {
        yellowUsed = true;
      }

      break;
    }
  }
}


// ============================================================
// 撮影
// ============================================================

bool captureAndStoreImage(RepairScope repairScope) {
  int captureResult =
      camera.grabFrame(
        cameraFrameBuffer,
        3000
      );

  if (captureResult != 0) {
    if (Serial) {
      Serial.print(
        "CAPTURE_ERROR_CODE:"
      );

      Serial.println(
        captureResult
      );
    }

    return false;
  }

  uint8_t* cameraData =
      cameraFrameBuffer.getBuffer();

  uint32_t actualSize =
      camera.frameSize();

  if (cameraData == nullptr) {
    if (Serial) {
      Serial.println(
        "ERROR NULL_CAMERA_BUFFER"
      );
    }

    return false;
  }

  if (actualSize != IMAGE_SIZE) {
    if (Serial) {
      Serial.print(
        "ERROR IMAGE_SIZE:"
      );

      Serial.println(
        actualSize
      );
    }

    return false;
  }

  // カメラフレームバッファ変化対策
  memcpy(
    savedImage,
    cameraData,
    IMAGE_SIZE
  );

  // 撮影画像のRGB565横帯ノイズを検査する。
  // ノイズを検出しても、画像の修復や書き換えは行わない。
  detectRgb565Noise(
    savedImage,
    repairScope
  );

  lastCaptureRepairScope = repairScope;

  savedImageValid = true;
  savedImageNumber++;

  return true;
}


// ============================================================
// MOTIONへ送信
// ============================================================

void motionPrintLine(
  const String &line
) {
  Serial1.println(line);
}

void sendResultsToMotion(int setId) {
  // 現在のMOTION側requestColorRead()はSET0を使用する。
  // 受信コマンドに合わせてsetIdを返す。

  for (
    int i = 0;
    i < ROI_COUNT;
    i++
  ) {
    Serial1.print("ROI,");
    Serial1.print(setId);
    Serial1.print(",");
    Serial1.print(i + 1);
    Serial1.print(",");
    Serial1.println(
      colorCodeToText(
        roiResults[i].color
      )
    );

    Serial1.print("RGB,");
    Serial1.print(setId);
    Serial1.print(",");
    Serial1.print(i + 1);
    Serial1.print(",");
    Serial1.print(
      roiResults[i].averageR
    );
    Serial1.print(",");
    Serial1.print(
      roiResults[i].averageG
    );
    Serial1.print(",");
    Serial1.println(
      roiResults[i].averageB
    );
  }

  // GROUPの判定はMOTION側で行うため、ここではROI/RGBのみ送信する。

  Serial1.print("END,");
  Serial1.println(setId);

  Serial1.flush();

  if (Serial) {
    Serial.print(
      "MOTION SEND COMPLETE SET"
    );

    Serial.println(setId);
  }
}


// ============================================================
// USBへ判定結果を送信
// PC ROI Pickerの色読みテスト用
// ============================================================

void sendResultsToUsb(int setId) {
  Serial.print(
    "RESULT_BEGIN,"
  );

  Serial.println(setId);

  for (
    int i = 0;
    i < ROI_COUNT;
    i++
  ) {
    Serial.print("ROI,");
    Serial.print(setId);
    Serial.print(",");
    Serial.print(i + 1);
    Serial.print(",");
    Serial.println(
      colorCodeToText(
        roiResults[i].color
      )
    );

    Serial.print("RGB,");
    Serial.print(setId);
    Serial.print(",");
    Serial.print(i + 1);
    Serial.print(",");
    Serial.print(
      roiResults[i].averageR
    );
    Serial.print(",");
    Serial.print(
      roiResults[i].averageG
    );
    Serial.print(",");
    Serial.println(
      roiResults[i].averageB
    );

    // COUNT順: G,B,W,Y
    Serial.print("COUNT,");
    Serial.print(i + 1);
    Serial.print(",");
    Serial.print(
      roiResults[i].countGreen
    );
    Serial.print(",");
    Serial.print(
      roiResults[i].countBlue
    );
    Serial.print(",");
    Serial.print(
      roiResults[i].countWhite
    );
    Serial.print(",");
    Serial.println(
      roiResults[i].countYellow
    );
  }

  for (
    int group = 0;
    group < GROUP_COUNT;
    group++
  ) {
    Serial.print("GROUP,");
    Serial.print(setId);
    Serial.print(",");
    Serial.print(group + 1);
    Serial.print(",");
    Serial.println(
      colorCodeToText(
        groupResults[group]
      )
    );
  }

  Serial.print(
    "RESULT_END,"
  );

  Serial.println(setId);
}


// ============================================================
// 色読み一式
// ============================================================

bool performColorRead(
  int setId,
  bool sendToMotion,
  bool sendToUsb,
  bool useColorChecklist
) {
  int noiseDetectedCount = 0;
  int captureFailureCount = 0;

  // 最大3回、ノイズのない画像が得られるまで撮影する。
  for (
    int attempt = 1;
    attempt <= COLOR_READ_MAX_CAPTURE_ATTEMPTS;
    attempt++
  ) {
    if (Serial) {
      Serial.print(
        "COLOR_READ_ATTEMPT,"
      );

      Serial.print(attempt);
      Serial.print(",");
      Serial.println(
        COLOR_READ_MAX_CAPTURE_ATTEMPTS
      );
    }

    if (
      !captureAndStoreImage(
        REPAIR_SCOPE_ROI_ROWS
      )
    ) {
      captureFailureCount++;

      if (Serial) {
        Serial.print(
          "COLOR_READ_CAPTURE_FAILED,"
        );

        Serial.println(attempt);
      }

      // 撮影自体が失敗した場合も、残り回数があれば再試行する。
      continue;
    }

    const bool noiseDetected =
        lastFrameRepairRegionOverflow ||
        lastFrameRepairRegionCount > 0;

    if (noiseDetected) {
      noiseDetectedCount++;

      if (Serial) {
        Serial.print(
          "COLOR_READ_NOISE_DETECTED,"
        );

        Serial.print(attempt);

        Serial.print(
          ",REGION_COUNT,"
        );

        Serial.print(
          lastFrameRepairRegionCount
        );

        Serial.print(
          ",OVERFLOW,"
        );

        Serial.println(
          lastFrameRepairRegionOverflow ?
          1 :
          0
        );
      }

      // ノイズ画像は修復せずに破棄し、次の撮影へ進む。
      continue;
    }

    // ノイズのない画像だけを色判定に使用する。
    analyzeAllRois(
      savedImage,
      useColorChecklist
    );

    if (!useColorChecklist) {
      // 通常READだけ従来のGROUPデバッグ判定を行う。
      decideGroupColors();
    } else {
      // READ_ROI / READ_TESTではグループ化しない。
      for (int group = 0; group < GROUP_COUNT; group++) {
        groupResults[group] = COLOR_UNKNOWN;
      }
    }

    if (sendToMotion) {
      sendResultsToMotion(
        setId
      );
    }

    if (sendToUsb) {
      sendResultsToUsb(
        setId
      );
    }

    if (Serial) {
      Serial.print(
        "COLOR_READ_SUCCESS,ATTEMPT,"
      );

      Serial.println(attempt);
    }

    return true;
  }

  // 3回試しても有効な画像を取得できなかった。
  savedImageValid = false;

  if (sendToMotion) {
    Serial1.print(
      "ERR:READ_FAILED,"
    );

    Serial1.print(setId);

    if (
      noiseDetectedCount ==
      COLOR_READ_MAX_CAPTURE_ATTEMPTS
    ) {
      Serial1.println(
        ",NOISE_3_TIMES"
      );

    } else if (
      captureFailureCount ==
      COLOR_READ_MAX_CAPTURE_ATTEMPTS
    ) {
      Serial1.println(
        ",CAPTURE_FAILED_3_TIMES"
      );

    } else {
      Serial1.println(
        ",NO_VALID_FRAME_3_ATTEMPTS"
      );
    }

    Serial1.flush();
  }

  if (
    sendToUsb &&
    Serial
  ) {
    if (
      noiseDetectedCount ==
      COLOR_READ_MAX_CAPTURE_ATTEMPTS
    ) {
      Serial.println(
        "ERROR READ_NOISE_3_TIMES"
      );

    } else if (
      captureFailureCount ==
      COLOR_READ_MAX_CAPTURE_ATTEMPTS
    ) {
      Serial.println(
        "ERROR READ_CAPTURE_FAILED_3_TIMES"
      );

    } else {
      Serial.println(
        "ERROR READ_NO_VALID_FRAME"
      );
    }
  }

  if (Serial) {
    Serial.print(
      "COLOR_READ_FAILED,NOISE_COUNT,"
    );

    Serial.print(noiseDetectedCount);

    Serial.print(
      ",CAPTURE_FAILURE_COUNT,"
    );

    Serial.println(captureFailureCount);
  }

  return false;
}


// ============================================================
// USB画像転送
// PCアプリのROW/ACK方式へ合わせる
// ============================================================

bool writeAllUsb(
  const uint8_t* data,
  uint32_t size
) {
  uint32_t totalSent = 0;

  uint32_t lastProgressTime =
      millis();

  while (totalSent < size) {
    size_t sentNow =
        Serial.write(
          data +
          totalSent,
          size -
          totalSent
        );

    if (sentNow > 0) {
      totalSent += sentNow;

      lastProgressTime =
          millis();

    } else {
      delay(1);

      if (
        millis() -
        lastProgressTime >
        5000
      ) {
        return false;
      }
    }
  }

  Serial.flush();

  return true;
}

bool waitUsbCommand(
  const String &expected,
  uint32_t timeoutMs
) {
  String line = "";

  uint32_t startTime =
      millis();

  while (
    millis() -
    startTime <
    timeoutMs
  ) {
    while (
      Serial.available() >
      0
    ) {
      char character =
          static_cast<char>(
            Serial.read()
          );

      if (character == '\n') {
        line.trim();

        if (line == expected) {
          return true;
        }

        line = "";

      } else if (
        character != '\r'
      ) {
        line += character;
      }
    }

    delay(1);
  }

  return false;
}

void sendSavedImageToUsb() {
  if (!savedImageValid) {
    // PCアプリはERRORで始まる行をエラーとして処理する
    Serial.println(
      "ERROR NO_PHOTO"
    );

    return;
  }

  // READではROI行だけを検査しているため、画像をPCへ送る前に
  // 未検査部分を含む全行のノイズ検査だけを行う。
  // 画像の修復や書き換えは行わない。
  if (lastCaptureRepairScope == REPAIR_SCOPE_ROI_ROWS) {
    detectRgb565Noise(
      savedImage,
      REPAIR_SCOPE_FULL_IMAGE
    );

    lastCaptureRepairScope = REPAIR_SCOPE_FULL_IMAGE;
  }

  Serial.print("IMAGE ");
  Serial.print(IMAGE_WIDTH);
  Serial.print(" ");
  Serial.print(IMAGE_HEIGHT);
  Serial.print(" ");
  Serial.println(IMAGE_SIZE);
  Serial.flush();

  if (
    !waitUsbCommand(
      "START",
      10000
    )
  ) {
    Serial.println(
      "ERROR START_TIMEOUT"
    );

    return;
  }

  for (
    uint32_t row = 0;
    row < IMAGE_HEIGHT;
    row++
  ) {
    Serial.print("ROW ");
    Serial.print(row);
    Serial.print(" ");
    Serial.println(ROW_SIZE);
    Serial.flush();

    const uint8_t* rowData =
        savedImage +
        row *
        ROW_SIZE;

    if (
      !writeAllUsb(
        rowData,
        ROW_SIZE
      )
    ) {
      Serial.print(
        "ERROR SEND_FAILED "
      );

      Serial.println(row);
      return;
    }

    String expectedAck =
        "ACK " +
        String(row);

    if (
      !waitUsbCommand(
        expectedAck,
        10000
      )
    ) {
      Serial.print(
        "ERROR ACK_TIMEOUT "
      );

      Serial.println(row);
      return;
    }
  }

  Serial.println("DONE");
  Serial.flush();
}


// ============================================================
// PCからROI/HSV設定
// ============================================================

void sendRoiConfigToUsb() {
  Serial.println(
    "ROI_CONFIG_BEGIN"
  );

  for (
    int i = 0;
    i < ROI_COUNT;
    i++
  ) {
    Serial.print(
      "ROI_CONFIG,"
    );

    Serial.print(i + 1);
    Serial.print(",");

    Serial.print(
      roiConfigs[i].centerX
    );

    Serial.print(",");

    Serial.print(
      roiConfigs[i].centerY
    );

    Serial.print(",");

    Serial.print(
      roiConfigs[i].width
    );

    Serial.print(",");

    Serial.println(
      roiConfigs[i].height
    );
  }

  Serial.println(
    "ROI_CONFIG_END"
  );
}

void printSingleHsvConfig(
  const char* name,
  const HsvRange &range
) {
  Serial.print(
    "HSV_CONFIG,"
  );

  Serial.print(name);
  Serial.print(",");

  Serial.print(range.hMin);
  Serial.print(",");

  Serial.print(range.hMax);
  Serial.print(",");

  Serial.print(range.sMin);
  Serial.print(",");

  Serial.print(range.sMax);
  Serial.print(",");

  Serial.print(range.vMin);
  Serial.print(",");

  Serial.println(range.vMax);
}

void sendHsvConfigToUsb() {
  Serial.println(
    "HSV_CONFIG_BEGIN"
  );

  printSingleHsvConfig(
    "BLACK",
    hsvBlack
  );

  printSingleHsvConfig(
    "WHITE",
    hsvWhite
  );

  printSingleHsvConfig(
    "YELLOW",
    hsvYellow
  );

  printSingleHsvConfig(
    "RED",
    hsvRed
  );

  printSingleHsvConfig(
    "BLUE",
    hsvBlue
  );

  printSingleHsvConfig(
    "GREEN",
    hsvGreen
  );

  Serial.println(
    "HSV_CONFIG_END"
  );
}

void sendColorAdjustToUsb() {
  Serial.print(
    "COLOR_ADJUST,"
  );

  Serial.print(
    brightnessPercent
  );

  Serial.print(",");

  Serial.println(
    saturationPercent
  );
}

void handleSetRoiCommand(
  const String &command
) {
  String parts[7];

  int count =
      splitCsv(
        command,
        parts,
        7
      );

  if (count < 6) {
    Serial.println(
      "ERROR SET_ROI_FORMAT"
    );

    return;
  }

  int roiIndex =
      parts[1].toInt() -
      1;

  if (
    roiIndex < 0 ||
    roiIndex >= ROI_COUNT
  ) {
    Serial.println(
      "ERROR SET_ROI_INDEX"
    );

    return;
  }

  roiConfigs[roiIndex].centerX =
      clampInt(
        parts[2].toInt(),
        0,
        IMAGE_WIDTH - 1
      );

  roiConfigs[roiIndex].centerY =
      clampInt(
        parts[3].toInt(),
        0,
        IMAGE_HEIGHT - 1
      );

  roiConfigs[roiIndex].width =
      clampInt(
        parts[4].toInt(),
        1,
        IMAGE_WIDTH
      );

  roiConfigs[roiIndex].height =
      clampInt(
        parts[5].toInt(),
        1,
        IMAGE_HEIGHT
      );

  Serial.print(
    "ROI_SAVED,"
  );

  Serial.println(
    roiIndex + 1
  );
}

void handleSetHsvCommand(
  const String &command
) {
  String parts[9];

  int count =
      splitCsv(
        command,
        parts,
        9
      );

  if (count < 8) {
    Serial.println(
      "ERROR SET_HSV_FORMAT"
    );

    return;
  }

  String colorName =
      parts[1];

  colorName.trim();
  colorName.toUpperCase();

  HsvRange newRange;

  newRange.hMin =
      clampInt(
        parts[2].toInt(),
        0,
        179
      );

  newRange.hMax =
      clampInt(
        parts[3].toInt(),
        0,
        179
      );

  newRange.sMin =
      clampInt(
        parts[4].toInt(),
        0,
        255
      );

  newRange.sMax =
      clampInt(
        parts[5].toInt(),
        0,
        255
      );

  newRange.vMin =
      clampInt(
        parts[6].toInt(),
        0,
        255
      );

  newRange.vMax =
      clampInt(
        parts[7].toInt(),
        0,
        255
      );

  if (colorName == "BLACK") {
    hsvBlack = newRange;

  } else if (
    colorName == "WHITE"
  ) {
    hsvWhite = newRange;

  } else if (
    colorName == "YELLOW"
  ) {
    hsvYellow = newRange;

  } else if (
    colorName == "RED"
  ) {
    hsvRed = newRange;

  } else if (
    colorName == "BLUE"
  ) {
    hsvBlue = newRange;

  } else if (
    colorName == "GREEN"
  ) {
    hsvGreen = newRange;

  } else {
    Serial.println(
      "ERROR SET_HSV_COLOR"
    );

    return;
  }

  Serial.print(
    "HSV_SAVED,"
  );

  Serial.println(
    colorName
  );
}


void handleSetColorAdjustCommand(
  const String &command
) {
  String parts[4];

  int count =
      splitCsv(
        command,
        parts,
        4
      );

  if (count < 3) {
    Serial.println(
      "ERROR SET_COLOR_ADJUST_FORMAT"
    );

    return;
  }

  int newBrightnessPercent =
      clampInt(
        parts[1].toInt(),
        0,
        400
      );

  int newSaturationPercent =
      clampInt(
        parts[2].toInt(),
        0,
        400
      );

  brightnessPercent =
      newBrightnessPercent;

  saturationPercent =
      newSaturationPercent;

  Serial.print(
    "COLOR_ADJUST_SAVED,"
  );

  Serial.print(
    brightnessPercent
  );

  Serial.print(",");

  Serial.println(
    saturationPercent
  );
}


// ============================================================
// USBコマンド処理
// ============================================================


// ============================================================
// ROIだけ読むモードの有効色設定
// 例: SET_ENABLED_COLORS,BLACK,YELLOW,BLUE
// ============================================================
void handleSetEnabledColorsCommand(const String &command) {
  String parts[7];
  int count = splitCsv(command, parts, 7);

  if (count < 2 || parts[0] != "SET_ENABLED_COLORS") {
    Serial.println("ERROR ENABLED_COLORS_INVALID");
    return;
  }

  bool black = false;
  bool white = false;
  bool yellow = false;
  bool red = false;
  bool blue = false;
  bool green = false;

  int validCount = 0;

  for (int i = 1; i < count; i++) {
    String name = parts[i];
    name.trim();
    name.toUpperCase();

    if (name == "BLACK") { black = true; validCount++; }
    else if (name == "WHITE" || name == "W") { white = true; validCount++; }
    else if (name == "YELLOW" || name == "Y") { yellow = true; validCount++; }
    else if (name == "RED" || name == "R") {red = true;validCount++;}
    else if (name == "BLUE" || name == "B") { blue = true; validCount++; }
    else if (name == "GREEN" || name == "G") { green = true; validCount++; }
    else {
      Serial.print("ERROR ENABLED_COLORS_UNKNOWN,");
      Serial.println(name);
      return;
    }
  }

  if (validCount <= 0) {
    Serial.println("ERROR ENABLED_COLORS_EMPTY");
    return;
  }

  checklistBlackEnabled = black;
  checklistWhiteEnabled = white;
  checklistYellowEnabled = yellow;
  checklistRedEnabled = red;
  checklistBlueEnabled = blue;
  checklistGreenEnabled = green;

  Serial.print("ENABLED_COLORS_SAVED");
  if (black) Serial.print(",BLACK");
  if (white) Serial.print(",WHITE");
  if (yellow) Serial.print(",YELLOW");
  if (red) Serial.print(",RED");
  if (blue) Serial.print(",BLUE");
  if (green) Serial.print(",GREEN");
  Serial.println();
}

void sendEnabledColorsToUsb() {
  Serial.print("ENABLED_COLORS");
  if (checklistBlackEnabled) Serial.print(",BLACK");
  if (checklistWhiteEnabled) Serial.print(",WHITE");
  if (checklistYellowEnabled) Serial.print(",YELLOW");
  if (checklistRedEnabled) Serial.print(",RED");
  if (checklistBlueEnabled) Serial.print(",BLUE");
  if (checklistGreenEnabled) Serial.print(",GREEN");
  Serial.println();
}

void processUsbCommand(
  String command
) {
  command.trim();
  command.toUpperCase();

  if (
    command.length() ==
    0
  ) {
    return;
  }

  if (command == "PING") {
    Serial.println("PONG");
    return;
  }

  if (
    command ==
    "CAPTURE"
  ) {
    if (
      captureAndStoreImage(REPAIR_SCOPE_FULL_IMAGE)
    ) {
      Serial.println(
        "CAPTURE_OK"
      );

    } else {
      Serial.println(
        "ERROR CAPTURE_FAILED"
      );
    }

    return;
  }

  if (
    command ==
      "GET_LAST_PHOTO" ||
    command ==
      "GET_IMAGE"
  ) {
    sendSavedImageToUsb();
    return;
  }

  if (
    command ==
    "CAPTURE_AND_GET"
  ) {
    if (
      !captureAndStoreImage(REPAIR_SCOPE_FULL_IMAGE)
    ) {
      Serial.println(
        "ERROR CAPTURE_FAILED"
      );

      return;
    }

    sendSavedImageToUsb();
    return;
  }

  if (
    command ==
      "READ_TEST" ||
    command ==
      "READ"
  ) {
    performColorRead(
      0,
      false,
      true,
      true
    );

    return;
  }

  if (
    command ==
    "GET_ROI_CONFIG"
  ) {
    sendRoiConfigToUsb();
    return;
  }

  if (
    command ==
    "GET_HSV_CONFIG"
  ) {
    sendHsvConfigToUsb();
    return;
  }

  if (
    command ==
    "GET_COLOR_ADJUST"
  ) {
    sendColorAdjustToUsb();
    return;
  }

  if (
    command.startsWith(
      "SET_ENABLED_COLORS,"
    )
  ) {
    handleSetEnabledColorsCommand(command);
    return;
  }

  if (command == "GET_ENABLED_COLORS") {
    sendEnabledColorsToUsb();
    return;
  }

  if (
    command.startsWith(
      "SET_ROI,"
    )
  ) {
    handleSetRoiCommand(
      command
    );

    return;
  }

  if (
    command.startsWith(
      "SET_HSV,"
    )
  ) {
    handleSetHsvCommand(
      command
    );

    return;
  }

  if (
    command.startsWith(
      "SET_COLOR_ADJUST,"
    )
  ) {
    handleSetColorAdjustCommand(
      command
    );

    return;
  }

  if (
    command ==
    "GET_REPAIR_STATUS"
  ) {
    sendRepairStatusToUsb();
    return;
  }

  if (
    command ==
    "CLEAR_PHOTO"
  ) {
    savedImageValid = false;

    memset(
      savedImage,
      0,
      IMAGE_SIZE
    );

    Serial.println(
      "PHOTO_CLEARED"
    );

    return;
  }

  if (
    command ==
    "STATUS"
  ) {
    Serial.println(
      "STATUS_BEGIN"
    );

    Serial.print(
      "PHOTO_VALID,"
    );

    Serial.println(
      savedImageValid ?
      1 :
      0
    );

    Serial.print(
      "PHOTO_NUMBER,"
    );

    Serial.println(
      savedImageNumber
    );

    Serial.print(
      "BRIGHTNESS_PERCENT,"
    );

    Serial.println(
      brightnessPercent
    );

    Serial.print(
      "SATURATION_PERCENT,"
    );

    Serial.println(
      saturationPercent
    );

    Serial.println(
      "STATUS_END"
    );

    return;
  }

  Serial.println(
    "ERROR UNKNOWN_USB_COMMAND"
  );
}

void updateUsbCommands() {
  while (
    Serial.available() >
    0
  ) {
    char character =
        static_cast<char>(
          Serial.read()
        );

    if (character == '\n') {
      usbLineBuffer.trim();

      if (
        usbLineBuffer.length() >
        0
      ) {
        String command =
            usbLineBuffer;

        usbLineBuffer = "";

        processUsbCommand(
          command
        );

      } else {
        usbLineBuffer = "";
      }

    } else if (
      character != '\r'
    ) {
      usbLineBuffer +=
          character;

      if (
        usbLineBuffer.length() >
        250
      ) {
        usbLineBuffer = "";
      }
    }
  }
}


// ============================================================
// MOTIONコマンド処理
// ============================================================

String cleanMotionCommand(
  String command
) {
  String cleaned = "";

  for (
    unsigned int i = 0;
    i < command.length();
    i++
  ) {
    char character =
        command.charAt(i);

    // 表示可能ASCIIのみ残す
    if (
      character >= 32 &&
      character <= 126
    ) {
      cleaned += character;
    }
  }

  cleaned.trim();
  cleaned.toUpperCase();

  return cleaned;
}

bool parseNamedMotionCommand(
  const String &command,
  const String &name,
  int &setId
) {
  setId = 0;

  if (command == name) {
    return true;
  }

  String commaPrefix =
      name + ",";

  if (
    command.startsWith(
      commaPrefix
    )
  ) {
    String suffix =
        command.substring(
          commaPrefix.length()
        );

    if (suffix.length() == 0) {
      return false;
    }

    for (
      unsigned int i = 0;
      i < suffix.length();
      i++
    ) {
      if (
        suffix.charAt(i) <
          '0' ||
        suffix.charAt(i) >
          '9'
      ) {
        return false;
      }
    }

    setId =
        clampInt(
          suffix.toInt(),
          0,
          5
        );

    return true;
  }

  if (
    command.startsWith(name) &&
    command.length() >
      name.length()
  ) {
    String suffix =
        command.substring(
          name.length()
        );

    for (
      unsigned int i = 0;
      i < suffix.length();
      i++
    ) {
      if (
        suffix.charAt(i) <
          '0' ||
        suffix.charAt(i) >
          '9'
      ) {
        return false;
      }
    }

    setId =
        clampInt(
          suffix.toInt(),
          0,
          5
        );

    return true;
  }

  return false;
}

void processMotionCommand(
  String command
) {
  String cleanedCommand =
      cleanMotionCommand(
        command
      );

  if (Serial) {
    Serial.print(
      "MOTION RX:["
    );

    Serial.print(
      cleanedCommand
    );

    Serial.println("]");
  }

  if (
    cleanedCommand.length() ==
    0
  ) {
    return;
  }

  int setId = 0;


  // READ_ROI / READ_ROI0 / READ_ROI,0
  // APPで設定したチェックリストだけでROI判定し、グループ化しない。
  if (
    parseNamedMotionCommand(
      cleanedCommand,
      "READ_ROI",
      setId
    )
  ) {
    performColorRead(
      setId,
      true,
      false,
      true
    );

    return;
  }

  // READ / READ0 / READ,0
  if (
    parseNamedMotionCommand(
      cleanedCommand,
      "READ",
      setId
    )
  ) {
    performColorRead(
      setId,
      true,
      false,
      false
    );

    return;
  }

  // SNAP / SNAP0 / SNAP,0
  if (
    parseNamedMotionCommand(
      cleanedCommand,
      "SNAP",
      setId
    )
  ) {
    if (
      captureAndStoreImage(REPAIR_SCOPE_FULL_IMAGE)
    ) {
      Serial1.print(
        "DONE:SNAP,set="
      );

      Serial1.println(setId);
      Serial1.flush();

    } else {
      Serial1.print(
        "ERR:SNAP_FAILED,set="
      );

      Serial1.println(setId);
      Serial1.flush();
    }

    return;
  }

  // PICK / PICK0 / PICK,0
  //
  // Nicla単体ではPCブラウザを起動できない。
  // 代わりに写真を撮ってRAMへ保持し、
  // PCアプリの「直近の保存画像を表示」で取得できるようにする。
  if (
    parseNamedMotionCommand(
      cleanedCommand,
      "PICK",
      setId
    )
  ) {
    if (
      captureAndStoreImage(REPAIR_SCOPE_FULL_IMAGE)
    ) {
      Serial1.print(
        "DONE:PICK,set="
      );

      Serial1.println(setId);
      Serial1.flush();

    } else {
      Serial1.print(
        "ERR:PICK_FAILED,set="
      );

      Serial1.println(setId);
      Serial1.flush();
    }

    return;
  }

  if (
    cleanedCommand ==
    "PING"
  ) {
    Serial1.println("PONG");
    Serial1.flush();
    return;
  }

  if (
    cleanedCommand ==
    "STATUS"
  ) {
    Serial1.print(
      "DONE:STATUS,PHOTO="
    );

    Serial1.println(
      savedImageValid ?
      1 :
      0
    );

    Serial1.flush();
    return;
  }

  Serial1.println(
    "ERR:UNKNOWN_COMMAND"
  );

  Serial1.flush();
}

void updateMotionCommands() {
  while (
    Serial1.available() >
    0
  ) {
    char character =
        static_cast<char>(
          Serial1.read()
        );

    if (character == '\n') {
      motionLineBuffer.trim();

      if (
        motionLineBuffer.length() >
        0
      ) {
        String command =
            motionLineBuffer;

        motionLineBuffer = "";

        processMotionCommand(
          command
        );

      } else {
        motionLineBuffer = "";
      }

    } else if (
      character != '\r'
    ) {
      motionLineBuffer +=
          character;

      if (
        motionLineBuffer.length() >
        100
      ) {
        motionLineBuffer = "";
      }
    }
  }
}


// ============================================================
// setup / loop
// ============================================================

void setup() {
  Serial.begin(
    USB_BAUD_RATE
  );

  // PC未接続でも競技動作を止めない
  unsigned long usbStartMs =
      millis();

  while (
    !Serial &&
    millis() -
      usbStartMs <
      1000
  ) {
    delay(10);
  }

  // Nicla VisionのJ2 UART
  Serial1.begin(
    MOTION_BAUD_RATE
  );

  if (Serial) {
    Serial.println();
    Serial.println(
      "NICLA_MOTION_CAMERA_V3"
    );

    Serial.println(
      "CAMERA_INITIALIZING"
    );
  }

  if (
    !camera.begin(
      CAMERA_R320x240,
      CAMERA_RGB565,
      30
    )
  ) {
    if (Serial) {
      Serial.println(
        "ERROR CAMERA_INIT"
      );
    }

    while (true) {
      delay(1000);
    }
  }

  if (Serial) {
    Serial.println(
      "CAMERA_READY"
    );

    Serial.println(
      "MOTION_UART_READY"
    );

    Serial.println(
      "SYSTEM_READY"
    );
  }
}

void loop() {
  updateMotionCommands();
  updateUsbCommands();

  delay(1);
}
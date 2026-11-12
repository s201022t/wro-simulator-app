#include "pi_color_lib.h"
#include "motion_lib.h"

// =========================================================// UART設定
// =========================================================
const int MOTION_TX_PIN = 16;  // GP16 = TX → Raspberry Pi RX
const int MOTION_RX_PIN = 17;  // GP17 = RX ← Raspberry Pi TX

const int ROI_COUNT = 12;
const int GROUP_COUNT = 6;
const int MAX_PER_COLOR = 3;

// 2色×2個になる4個グループの最大数
const int MAX_MIXED_GROUPS = 2;

// =========================================================
// Pi故障中の手動GROUP色設定
// MGコマンドや setManualGroupColors() 用に残す
// readColorAndSetVariable() では使わない
// =========================================================

// =========================================================
// 6GROUP 最終判定結果
// =========================================================
String GroupA = "UNKNOWN";
String GroupB = "UNKNOWN";
String GroupC = "UNKNOWN";
String GroupD = "UNKNOWN";
String GroupE = "UNKNOWN";
String GroupF = "UNKNOWN";

// =========================================================
// 旧走行処理との互換用
// 後でGroupA～Fとの対応を整理する
// =========================================================
String Group1 = "UNKNOWN";
String Group2 = "UNKNOWN";
String Group3 = "UNKNOWN";

// 1始まりで使用する
// groupColor[1] = A
// groupColor[2] = B
// ...
// groupColor[6] = F
String groupColor[GROUP_COUNT + 1];

// =========================================================
// 各色の残り使用可能数
// =========================================================
int GreenRemaining  = MAX_PER_COLOR;
int BlueRemaining   = MAX_PER_COLOR;
int WhiteRemaining  = MAX_PER_COLOR;
int YellowRemaining = MAX_PER_COLOR;

bool g_colorReadValid = false;
bool g_manualColorMode = false;

String roiColor[ROI_COUNT + 1];

int roiR[ROI_COUNT + 1];
int roiG[ROI_COUNT + 1];
int roiB[ROI_COUNT + 1];

// =========================================================
// 追加SET用
// 今回はSET0中心。SET1以降もコンパイルエラー防止用に実体だけ用意。
// =========================================================
String colorSetName[COLOR_SET_COUNT] = {
  "SET0", "SET1", "SET2", "SET3", "SET4", "SET5"
};

int roiCount[COLOR_SET_COUNT] = {
  12, 12, 12, 12, 12, 12
};

bool setColorReadValid[COLOR_SET_COUNT];

String setRoiColor[COLOR_SET_COUNT][MAX_ROI_COUNT + 1];
int setRoiR[COLOR_SET_COUNT][MAX_ROI_COUNT + 1];
int setRoiG[COLOR_SET_COUNT][MAX_ROI_COUNT + 1];
int setRoiB[COLOR_SET_COUNT][MAX_ROI_COUNT + 1];

// =========================================================
// GROUPとROIの対応 (Nicla側 GROUP_ROI_INDEXES と同一)
// roiColor[] は1始まりなのでこちらも1始まりの番号で持つ
//
// GROUP1 = ROI1, ROI2, ROI5, ROI6
// GROUP2 = ROI9, ROI10, ROI11, ROI12
// GROUP3 = ROI3, ROI4, ROI7, ROI8
// =========================================================
// =========================================================
// GROUPとROIの対応
//
// GroupA = ROI1,5
// GroupB = ROI2,6
// GroupC = ROI3,7
// GroupD = ROI4,8
// GroupE = ROI9,10
// GroupF = ROI11,12
// =========================================================
static const int GROUP_ROI_INDEXES[GROUP_COUNT][2] = {
  { 1,  5 },   // GroupA
  { 2,  6 },   // GroupB
  { 3,  7 },   // GroupC
  { 4,  8 },   // GroupD
  { 9, 10 },   // GroupE
  {11, 12 }    // GroupF
};

// 探索時のペア
static const int GROUP_PAIRS[3][2] = {
  {0, 1},   // A-B
  {2, 3},   // C-D
  {4, 5}    // E-F
};
// =========================================================
// 色ID
// 探索処理ではStringではなく番号で管理する
// =========================================================
enum ColorId {
  COLOR_B = 0,
  COLOR_G = 1,
  COLOR_W = 2,
  COLOR_Y = 3,
  COLOR_UNKNOWN = -1
};



// =========================================================
// Groupごとの候補
// 1Groupにつき4色を候補にする
// =========================================================
struct GroupCandidates {
  ColorId colors[4];
  int count;
};


// =========================================================
// ペア探索用
// =========================================================
struct PairOption {
  ColorId first;
  ColorId second;
};

// =========================================================
// 内部補助
// =========================================================
static String normalizeColor(String color) {
  color.trim();
  color.toUpperCase();

  // GREEN
  if (
    color == "G" ||
    color == "GREEN"
  ) {
    return "G";
  }

  // BLUE
  if (
    color == "B" ||
    color == "BLUE"
  ) {
    return "B";
  }

  // WHITE
  if (
    color == "W" ||
    color == "WHITE"
  ) {
    return "W";
  }

  // YELLOW
  if (
    color == "Y" ||
    color == "YELLOW"
  ) {
    return "Y";
  }

  // RED
  if (
    color == "R" ||
    color == "RED"
  ) {
    return "R";
  }

  // BLACK
  // BLUEが"B"を使っているため、BLACKは"B"に省略しない
  if (
    color == "BLACK"
  ) {
    return "BLACK";
  }

  return "UNKNOWN";
}

static ColorId stringToColorId(String color) {
  color = normalizeColor(color);

  if (color == "B") return COLOR_B;
  if (color == "G") return COLOR_G;
  if (color == "W") return COLOR_W;
  if (color == "Y") return COLOR_Y;

  return COLOR_UNKNOWN;
}

// =========================================================
// Groupごとの色候補を作成
//
// ROIが正常に読めている場合:
//   B / G / W / Y の4色を候補にして、
//   後の全探索でROIとの一致数を評価する。
//
// ROIがUNKNOWNの場合:
//   エラー終了にはせず、
//   B / G / W / Y の4色を候補として残す。
//   UNKNOWNのROIは一致点が0点になるため、
//   他Groupとの関係から最終色を決定できる。
// =========================================================
static bool makeGroupCandidates(
  int groupIndex,
  GroupCandidates &candidates
) {

  // 全4色を候補にする
  candidates.colors[0] = COLOR_B;
  candidates.colors[1] = COLOR_G;
  candidates.colors[2] = COLOR_W;
  candidates.colors[3] = COLOR_Y;

  candidates.count = 4;

  return true;
}

// =========================================================
// このGroupが色補完を必要としているか確認
//
// Groupを構成する2つのROIが両方UNKNOWNの場合のみtrue。
//
// 片方だけ正常に読めている場合は、
// 正常に読めたROIの色を通常の一致点で優先する。
// =========================================================
static bool groupNeedsColorFallback(int groupIndex) {

  int roi1 = GROUP_ROI_INDEXES[groupIndex][0];
  int roi2 = GROUP_ROI_INDEXES[groupIndex][1];

  ColorId color1 = stringToColorId(roiColor[roi1]);
  ColorId color2 = stringToColorId(roiColor[roi2]);

  return (
    color1 == COLOR_UNKNOWN &&
    color2 == COLOR_UNKNOWN
  );
}

// =========================================================
// Group × 色 の得点を事前計算
//
// groupMatchScoreTable[group][color]
//
// group:
//   0 = A
//   1 = B
//   2 = C
//   3 = D
//   4 = E
//   5 = F
//
// color:
//   COLOR_B = 0
//   COLOR_G = 1
//   COLOR_W = 2
//   COLOR_Y = 3
//
// 得点:
//   0 = ROI一致なし
//   1 = ROI1個一致
//   2 = ROI2個一致
// =========================================================

static int groupMatchScoreTable[GROUP_COUNT][4];


static void buildGroupMatchScoreTable() {

  for (int group = 0; group < GROUP_COUNT; group++) {

    int roi1 = GROUP_ROI_INDEXES[group][0];
    int roi2 = GROUP_ROI_INDEXES[group][1];

    // String → ColorId変換はここで1回だけ行う
    ColorId roiColor1 = stringToColorId(roiColor[roi1]);
    ColorId roiColor2 = stringToColorId(roiColor[roi2]);


    // B / G / W / Y の4色について得点を計算
    for (int color = 0; color < 4; color++) {

      ColorId targetColor = (ColorId)color;

      int score = 0;

      if (roiColor1 == targetColor) {
        score++;
      }

      if (roiColor2 == targetColor) {
        score++;
      }

      groupMatchScoreTable[group][color] = score;
    }
  }
}

// =========================================================
// 指定したGroupに指定色を置いた場合の一致数を返す
//
// 戻り値
// 0 = 0個一致
// 1 = 1個一致
// 2 = 2個一致
// =========================================================
static int getGroupMatchScore(
  int groupIndex,
  ColorId color
) {

  return groupMatchScoreTable[groupIndex][color];
}



// =========================================================
// GroupA～Fの色組み合わせを全探索
//
// 制約:
// ・各GroupはB/G/W/Yの4色から選択
// ・各色は最大3Groupまで
// ・A-B / C-D / E-F のうち
//   2色構成になるペアは最大 MAX_MIXED_GROUPS 個
// ・ROI1～12との一致数が最大の組み合わせを採用
//
// A != B → 2色×2個グループ
// C != D → 2色×2個グループ
// E != F → 2色×2個グループ
// =========================================================
static bool searchGroupColors(
  int pairIndex,
  GroupCandidates candidates[GROUP_COUNT],
  ColorId results[GROUP_COUNT],
  int colorCount[4],
  int mixedGroupCount
) {

  // =====================================================
  // 全探索中の最高結果
  // =====================================================
  static int bestScore = -1;

  // UNKNOWN Groupの補完評価
  static int bestFallbackScore = -1;

  static ColorId bestResults[GROUP_COUNT];

  static int bestColorCount[4];

  static int bestMixedGroupCount = 0;

  static bool foundAny = false;


  // =====================================================
  // 最初の呼び出し時に初期化
  // =====================================================
  if (pairIndex == 0) {

    bestScore = -1;
    bestFallbackScore = -1;
    bestMixedGroupCount = 0;
    foundAny = false;

    for (int group = 0; group < GROUP_COUNT; group++) {
      bestResults[group] = COLOR_UNKNOWN;
    }

    for (int color = 0; color < 4; color++) {
      bestColorCount[color] = 0;
    }
  }


  // =====================================================
  // A-B、C-D、E-Fすべて決定済み
  // =====================================================
  if (pairIndex >= 3) {

    int score = 0;


    // ===================================================
    // ROI一致数を計算
    //
    // 1Group最大2点
    // 6Group最大12点
    // ===================================================
    for (int group = 0; group < GROUP_COUNT; group++) {

      score += getGroupMatchScore(
        group,
        results[group]
      );
    }


    // ===================================================
    // UNKNOWN Groupの補完評価
    // ===================================================
    int fallbackScore = 0;

    for (int group = 0; group < GROUP_COUNT; group++) {

      if (!groupNeedsColorFallback(group)) {
        continue;
      }

      ColorId selectedColor = results[group];

      // この色を使用しているGroupがこのGroupだけなら
      // 他Groupで使われていない色と判断する
      if (
        selectedColor >= COLOR_B &&
        selectedColor <= COLOR_Y &&
        colorCount[selectedColor] == 1
      ) {
        fallbackScore++;
      }
    }


    // ===================================================
    // 最終結果の選択
    //
    // 優先順位
    //
    // 1. ROI一致数が多い
    //
    // 2. ROI一致数が同じなら
    //    2色×2個のグループ数が少ない方を優先
    //    → 1色4個を優先
    //
    // 3. mixed数も同じなら
    //    UNKNOWN Groupへの未使用色補完を優先
    //
    // 4. それも同じなら最初の候補を残す
    // ===================================================
    if (
      !foundAny ||

      score > bestScore ||

      (
        score == bestScore &&
        mixedGroupCount < bestMixedGroupCount
      ) ||

      (
        score == bestScore &&
        mixedGroupCount == bestMixedGroupCount &&
        fallbackScore > bestFallbackScore
      )
    ) {

      bestScore = score;
      bestFallbackScore = fallbackScore;
      bestMixedGroupCount = mixedGroupCount;

      for (int group = 0; group < GROUP_COUNT; group++) {
        bestResults[group] = results[group];
      }

      for (int color = 0; color < 4; color++) {
        bestColorCount[color] = colorCount[color];
      }
    }


    foundAny = true;

    return true;
  }


  // =====================================================
  // 今回処理する2Group
  // =====================================================
  int g1 = GROUP_PAIRS[pairIndex][0];
  int g2 = GROUP_PAIRS[pairIndex][1];


  // =====================================================
  // 4色 × 4色 = 16通り
  // =====================================================
  PairOption options[16];
  int optionCount = 0;


  for (int i = 0; i < candidates[g1].count; i++) {

    for (int j = 0; j < candidates[g2].count; j++) {

      options[optionCount].first =
        candidates[g1].colors[i];

      options[optionCount].second =
        candidates[g2].colors[j];

      optionCount++;
    }
  }


  bool branchFound = false;


  // =====================================================
  // 全候補を試す
  // =====================================================
  for (int i = 0; i < optionCount; i++) {

    ColorId c1 = options[i].first;
    ColorId c2 = options[i].second;


    // ===================================================
    // 今回増える各色の使用数
    // ===================================================
    int added[4] = {
      0, 0, 0, 0
    };

    added[c1]++;
    added[c2]++;


    // ===================================================
    // 各色最大3Groupの確認
    // ===================================================
    bool valid = true;

    for (int color = 0; color < 4; color++) {

      if (
        colorCount[color] + added[color]
        > MAX_PER_COLOR
      ) {

        valid = false;
        break;
      }
    }


    if (!valid) {
      continue;
    }


    // ===================================================
    // 2色×2個グループ数の確認
    //
    // c1 != c2 なら、この4個グループは
    // 2色×2個の混色グループ
    // ===================================================
    int nextMixedGroupCount = mixedGroupCount;

    if (c1 != c2) {
      nextMixedGroupCount++;
    }


    // 最大数を超えたら使用しない
    if (
      nextMixedGroupCount
      > MAX_MIXED_GROUPS
    ) {

      continue;
    }


    // ===================================================
    // 仮決定
    // ===================================================
    results[g1] = c1;
    results[g2] = c2;


    for (int color = 0; color < 4; color++) {
      colorCount[color] += added[color];
    }


    // ===================================================
    // 次のペアへ
    // ===================================================
    if (
      searchGroupColors(
        pairIndex + 1,
        candidates,
        results,
        colorCount,
        nextMixedGroupCount
      )
    ) {

      branchFound = true;
    }


    // ===================================================
    // 仮決定を取り消す
    // ===================================================
    for (int color = 0; color < 4; color++) {
      colorCount[color] -= added[color];
    }

    results[g1] = COLOR_UNKNOWN;
    results[g2] = COLOR_UNKNOWN;
  }


  // =====================================================
  // 全探索終了
  // =====================================================
  if (pairIndex == 0) {

    if (!foundAny) {
      return false;
    }


    // 最高得点結果を復元
    for (int group = 0; group < GROUP_COUNT; group++) {
      results[group] = bestResults[group];
    }

    for (int color = 0; color < 4; color++) {
      colorCount[color] = bestColorCount[color];
    }


    // ===================================================
    // テスト確認用Serial
    // ===================================================
    Serial.print("Best group match score = ");
    Serial.print(bestScore);
    Serial.println(" / 12");

    Serial.print("Best mixed groups = ");
    Serial.print(bestMixedGroupCount);
    Serial.print(" / ");
    Serial.println(MAX_MIXED_GROUPS);


    return true;
  }


  return branchFound;
}


// =========================================================
// 各色の残り使用可能数を更新
// =========================================================
static void updateColorRemaining(
  const int colorCount[4]
) {

  BlueRemaining =
    MAX_PER_COLOR - colorCount[COLOR_B];

  GreenRemaining =
    MAX_PER_COLOR - colorCount[COLOR_G];

  WhiteRemaining =
    MAX_PER_COLOR - colorCount[COLOR_W];

  YellowRemaining =
    MAX_PER_COLOR - colorCount[COLOR_Y];
}



static String colorIdToString(ColorId color) {
  if (color == COLOR_B) return "B";
  if (color == COLOR_G) return "G";
  if (color == COLOR_W) return "W";
  if (color == COLOR_Y) return "Y";

  return "UNKNOWN";
}

static int splitCsv(String line, String parts[], int maxParts) {
  int count = 0;
  int start = 0;

  while (count < maxParts) {
    int comma = line.indexOf(',', start);

    if (comma < 0) {
      parts[count] = line.substring(start);
      parts[count].trim();
      count++;
      break;
    }

    parts[count] = line.substring(start, comma);
    parts[count].trim();
    count++;

    start = comma + 1;
  }

  return count;
}

static void blinkColorReadErrorLed() {
    boardRgbSet(255, 0, 0);  // 赤
    servoDelay(150);
    boardRgbOff();

}


// =========================================================
// 初期化
// =========================================================
void piColorSetup() {
  Serial1.setTX(MOTION_TX_PIN);
  Serial1.setRX(MOTION_RX_PIN);
  Serial1.begin(115200);

  for (int i = 0; i <= ROI_COUNT; i++) {
    roiColor[i] = "UNKNOWN";
    roiR[i] = 0;
    roiG[i] = 0;
    roiB[i] = 0;
  }

  for (int i = 0; i <= GROUP_COUNT; i++) {
    groupColor[i] = "UNKNOWN";
  }

  GroupA = "UNKNOWN";
  GroupB = "UNKNOWN";
  GroupC = "UNKNOWN";
  GroupD = "UNKNOWN";
  GroupE = "UNKNOWN";
  GroupF = "UNKNOWN";

  BlueRemaining   = MAX_PER_COLOR;
  GreenRemaining  = MAX_PER_COLOR;
  WhiteRemaining  = MAX_PER_COLOR;
  YellowRemaining = MAX_PER_COLOR;

  g_colorReadValid = false;
  g_manualColorMode = false;

  for (int setId = 0; setId < COLOR_SET_COUNT; setId++) {
    setColorReadValid[setId] = false;

    for (int i = 0; i <= MAX_ROI_COUNT; i++) {
      setRoiColor[setId][i] = "UNKNOWN";
      setRoiR[setId][i] = 0;
      setRoiG[setId][i] = 0;
      setRoiB[setId][i] = 0;
    }
  }

  Serial.println("Pi color UART setup OK");
}

// =========================================================
// シリアルバッファクリア
// =========================================================
void clearPiSerialBuffer() {
  while (Serial1.available()) {
    Serial1.read();
  }
}

// =========================================================
// Piから1行読む
// =========================================================
String readLineFromPi(unsigned long timeoutMs) {
  String line = "";
  unsigned long startMs = millis();

  while (millis() - startMs < timeoutMs) {
    // =====================================================
    // GP20停止確認
    //
    // stopPressed()は、GP20が押された場合に
    // モーター停止・停止音・基板再起動まで実行する。
    // 押されていない場合はfalseを返して通常処理を続ける。
    // =====================================================
    if (stopPressed()) {
      return "";
    }

    while (Serial1.available()) {
      // UARTデータを連続受信している最中にも停止確認
      if (stopPressed()) {
        return "";
      }

      char c = (char)Serial1.read();

      if (c == '\n') {
        line.trim();
        return line;
      }

      if (c != '\r') {
        line += c;
      }
    }

    servoDelay(1);
  }

  line.trim();
  return line;
}

// =========================================================
// パース関数
// ROI,1,G
// ROI,1,GREEN
// ROI,0,1,G
// =========================================================
bool parseRoiColorLine(String line, int &index, String &color) {
  line.trim();

  String parts[5];
  int count = splitCsv(line, parts, 5);

  if (count < 3) {
    return false;
  }

  if (parts[0] != "ROI") {
    return false;
  }

  // 旧形式: ROI,1,B
  if (count == 3) {
    index = parts[1].toInt();
    color = normalizeColor(parts[2]);
  }

  // 新形式: ROI,0,1,B
  else if (count >= 4) {
    int setId = parts[1].toInt();

    if (setId != 0) {
      return false;
    }

    index = parts[2].toInt();
    color = normalizeColor(parts[3]);
  }

  if (index < 1 || index > ROI_COUNT) {
    return false;
  }

  return true;
}

// =========================================================
// GROUP,1,G
// GROUP,0,1,G
// =========================================================
bool parseGroupColorLine(String line, int &index, String &color) {
  line.trim();

  String parts[5];
  int count = splitCsv(line, parts, 5);

  if (count < 3) {
    return false;
  }

  if (parts[0] != "GROUP") {
    return false;
  }

  // 旧形式: GROUP,1,Y
  if (count == 3) {
    index = parts[1].toInt();
    color = normalizeColor(parts[2]);
  }

  // 新形式: GROUP,0,1,Y
  else if (count >= 4) {
    int setId = parts[1].toInt();

    if (setId != 0) {
      return false;
    }

    index = parts[2].toInt();
    color = normalizeColor(parts[3]);
  }

  if (index < 1 || index > GROUP_COUNT) {
    return false;
  }

  return true;
}

// =========================================================
// RGB,1,100,120,130
// RGB,0,1,100,120,130
// =========================================================
bool parseRoiRgbLine(String line, int &index, int &r, int &g, int &b) {
  line.trim();

  String parts[7];
  int count = splitCsv(line, parts, 7);

  if (count < 5) {
    return false;
  }

  if (parts[0] != "RGB") {
    return false;
  }

  // 旧形式: RGB,1,100,120,130
  if (count == 5) {
    index = parts[1].toInt();
    r = parts[2].toInt();
    g = parts[3].toInt();
    b = parts[4].toInt();
  }

  // 新形式: RGB,0,1,100,120,130
  else if (count >= 6) {
    int setId = parts[1].toInt();

    if (setId != 0) {
      return false;
    }

    index = parts[2].toInt();
    r = parts[3].toInt();
    g = parts[4].toInt();
    b = parts[5].toInt();
  }

  if (index < 1 || index > ROI_COUNT) {
    return false;
  }

  return true;
}

// =========================================================
// 追加SET用: SROI,setId,index,color
// =========================================================
bool parseSetRoiColorLine(String line, int &setId, int &index, String &color) {
  line.trim();

  if (!line.startsWith("SROI,")) {
    return false;
  }

  int p1 = line.indexOf(',');
  int p2 = line.indexOf(',', p1 + 1);
  int p3 = line.indexOf(',', p2 + 1);

  if (p1 < 0 || p2 < 0 || p3 < 0) {
    return false;
  }

  setId = line.substring(p1 + 1, p2).toInt();
  index = line.substring(p2 + 1, p3).toInt();
  color = normalizeColor(line.substring(p3 + 1));

  if (setId < 0 || setId >= COLOR_SET_COUNT) {
    return false;
  }

  if (index < 1 || index > MAX_ROI_COUNT) {
    return false;
  }

  return true;
}

// =========================================================
// 追加SET用: SRGB,setId,index,r,g,b
// =========================================================
bool parseSetRoiRgbLine(String line, int &setId, int &index, int &r, int &g, int &b) {
  line.trim();

  if (!line.startsWith("SRGB,")) {
    return false;
  }

  int p1 = line.indexOf(',');
  int p2 = line.indexOf(',', p1 + 1);
  int p3 = line.indexOf(',', p2 + 1);
  int p4 = line.indexOf(',', p3 + 1);
  int p5 = line.indexOf(',', p4 + 1);

  if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0 || p5 < 0) {
    return false;
  }

  setId = line.substring(p1 + 1, p2).toInt();
  index = line.substring(p2 + 1, p3).toInt();
  r = line.substring(p3 + 1, p4).toInt();
  g = line.substring(p4 + 1, p5).toInt();
  b = line.substring(p5 + 1).toInt();

  if (setId < 0 || setId >= COLOR_SET_COUNT) {
    return false;
  }

  if (index < 1 || index > MAX_ROI_COUNT) {
    return false;
  }

  return true;
}

// =========================================================
// 表示
// =========================================================
void printAllRoiColors() {
  Serial.println("===== ROI COLORS =====");

  for (int i = 1; i <= ROI_COUNT; i++) {
    Serial.print("ROI");
    Serial.print(i);
    Serial.print("=");
    Serial.println(roiColor[i]);
  }
}

void printAllRoiRgb() {
  Serial.println("===== ROI RGB =====");

  for (int i = 1; i <= ROI_COUNT; i++) {
    Serial.print("ROI");
    Serial.print(i);
    Serial.print(" R=");
    Serial.print(roiR[i]);
    Serial.print(" G=");
    Serial.print(roiG[i]);
    Serial.print(" B=");
    Serial.println(roiB[i]);
  }
}

void printAllGroupColors() {
  Serial.println("===== GROUP COLORS =====");

  Serial.print("GroupA=");
  Serial.println(GroupA);

  Serial.print("GroupB=");
  Serial.println(GroupB);

  Serial.print("GroupC=");
  Serial.println(GroupC);

  Serial.print("GroupD=");
  Serial.println(GroupD);

  Serial.print("GroupE=");
  Serial.println(GroupE);

  Serial.print("GroupF=");
  Serial.println(GroupF);

  Serial.println("===== COLOR REMAINING =====");

  Serial.print("BlueRemaining=");
  Serial.println(BlueRemaining);

  Serial.print("GreenRemaining=");
  Serial.println(GreenRemaining);

  Serial.print("WhiteRemaining=");
  Serial.println(WhiteRemaining);

  Serial.print("YellowRemaining=");
  Serial.println(YellowRemaining);
}
void checkGroupColors() {
  Serial.println("===== CHECK GROUP COLORS =====");

  Serial.print("ColorReadValid=");
  Serial.println(g_colorReadValid ? "true" : "false");

  Serial.print("ManualColorMode=");
  Serial.println(g_manualColorMode ? "true" : "false");

  printAllRoiColors();
  printAllGroupColors();
}

void printAllRoiColors(int setId) {
  if (setId == 0) {
    printAllRoiColors();
    return;
  }

  if (setId < 0 || setId >= COLOR_SET_COUNT) {
    Serial.println("Invalid setId");
    return;
  }

  Serial.print("===== ");
  Serial.print(colorSetName[setId]);
  Serial.println(" ROI COLORS =====");

  for (int i = 1; i <= roiCount[setId]; i++) {
    Serial.print("ROI");
    Serial.print(i);
    Serial.print("=");
    Serial.println(setRoiColor[setId][i]);
  }
}

void printAllRoiRgb(int setId) {
  if (setId == 0) {
    printAllRoiRgb();
    return;
  }

  if (setId < 0 || setId >= COLOR_SET_COUNT) {
    Serial.println("Invalid setId");
    return;
  }

  Serial.print("===== ");
  Serial.print(colorSetName[setId]);
  Serial.println(" ROI RGB =====");

  for (int i = 1; i <= roiCount[setId]; i++) {
    Serial.print("ROI");
    Serial.print(i);
    Serial.print(" R=");
    Serial.print(setRoiR[setId][i]);
    Serial.print(" G=");
    Serial.print(setRoiG[setId][i]);
    Serial.print(" B=");
    Serial.println(setRoiB[setId][i]);
  }
}

void printColorSetStatus(int setId) {
  if (setId < 0 || setId >= COLOR_SET_COUNT) {
    Serial.println("Invalid setId");
    return;
  }

  Serial.print("SET");
  Serial.print(setId);
  Serial.print(" valid=");
  Serial.println(setColorReadValid[setId] ? "true" : "false");

  printAllRoiColors(setId);
}

// =========================================================
// Piへコマンド送信
// =========================================================
void sendSimpleCommandToPi(const String &command, unsigned long timeoutMs) {
  clearPiSerialBuffer();

  Serial.print("Sent to Pi: ");
  Serial.println(command);

  Serial1.println(command);
  Serial1.flush();

  unsigned long startMs = millis();

  while (millis() - startMs < timeoutMs) {
    String line = readLineFromPi(200);

    if (line.length() <= 0) {
      continue;
    }

    Serial.print("Received from Pi: ");
    Serial.println(line);

    if (line.startsWith("DONE:") || line.startsWith("ERR:") || line == "END") {
      return;
    }
  }

  Serial.println("Pi response timeout");
}

// =========================================================
// 写真撮影
// =========================================================
void requestPiSnap() {
  sendSimpleCommandToPi("SNAP", 8000);
}

void requestPiSnap(int setId) {
  if (setId == 0) {
    requestPiSnap();
    return;
  }

  String command = "SNAP";
  command += setId;
  sendSimpleCommandToPi(command, 8000);
}

// =========================================================
// ROI Picker起動
// =========================================================
void requestPiPicker() {
  sendSimpleCommandToPi("PICK", 8000);
}

void requestPiPicker(int setId) {
  if (setId == 0) {
    requestPiPicker();
    return;
  }

  String command = "PICK";
  command += setId;
  sendSimpleCommandToPi(command, 8000);
}

bool readRoi() {
  clearPiSerialBuffer();

  for (int i = 0; i <= ROI_COUNT; i++) {
    roiColor[i] = "UNKNOWN";
    roiR[i] = 0;
    roiG[i] = 0;
    roiB[i] = 0;
  }

  g_colorReadValid = false;

  bool roiReceived[ROI_COUNT + 1] = { false };

  Serial1.println("READ_ROI");
  Serial1.flush();

  unsigned long startMs = millis();
  const unsigned long timeoutMs = 1000;

  while (millis() - startMs < timeoutMs) {
    String line = readLineFromPi(100);

    if (line.length() <= 0) {
      continue;
    }

    if (line.startsWith("ERR:")) {
      return false;
    }

    int index = 0;
    String color = "UNKNOWN";

    if (parseRoiColorLine(line, index, color)) {
      roiColor[index] = color;
      roiReceived[index] = true;
      continue;
    }

    int r = 0;
    int g = 0;
    int b = 0;

    if (parseRoiRgbLine(line, index, r, g, b)) {
      roiR[index] = r;
      roiG[index] = g;
      roiB[index] = b;
      continue;
    }

    if (
      line == "END" ||
      line.startsWith("END,")
    ) {
      for (int i = 1; i <= ROI_COUNT; i++) {
        if (!roiReceived[i]) {
          return false;
        }
      }

      g_colorReadValid = true;
      return true;
    }
  }

  return false;
}

// =========================================================
// 色読み
// Piへ READ を送り、ROI / GROUP / RGB を受け取る
// =========================================================
bool requestColorRead() {
  clearPiSerialBuffer();

  for (int i = 0; i <= ROI_COUNT; i++) {
    roiColor[i] = "UNKNOWN";
    roiR[i] = 0;
    roiG[i] = 0;
    roiB[i] = 0;
  }

  for (int i = 0; i <= GROUP_COUNT; i++) {
    groupColor[i] = "UNKNOWN";
  }

  GroupA = "UNKNOWN";
  GroupB = "UNKNOWN";
  GroupC = "UNKNOWN";
  GroupD = "UNKNOWN";
  GroupE = "UNKNOWN";
  GroupF = "UNKNOWN";

  BlueRemaining   = MAX_PER_COLOR;
  GreenRemaining  = MAX_PER_COLOR;
  WhiteRemaining  = MAX_PER_COLOR;
  YellowRemaining = MAX_PER_COLOR;

  g_colorReadValid = false;
  g_manualColorMode = false;

  g_colorReadValid = false;
  g_manualColorMode = false;

  bool roiReceived[ROI_COUNT + 1];
  bool groupReceived[GROUP_COUNT + 1];

  for (int i = 0; i <= ROI_COUNT; i++) {
    roiReceived[i] = false;
  }

  for (int i = 0; i <= GROUP_COUNT; i++) {
    groupReceived[i] = false;
  }

  Serial.println("Sent to Pi: READ");
  Serial1.println("READ");
  Serial1.flush();

  unsigned long startMs = millis();
  const unsigned long timeoutMs = 1000;


  while (millis() - startMs < timeoutMs) {
    String line = readLineFromPi(100);

    if (line.length() <= 0) {
      continue;
    }

    Serial.print("Received from Pi: ");
    Serial.println(line);

    if (line.startsWith("ERR:")) {
      g_colorReadValid = false;
      Serial.print("Color read error: ");
      Serial.println(line);
      return false;
    }

    int index = 0;
    String color = "UNKNOWN";

    if (parseRoiColorLine(line, index, color)) {
      roiColor[index] = color;
      roiReceived[index] = true;

      Serial.print("ROI");
      Serial.print(index);
      Serial.print(" COLOR=");
      Serial.println(color);

      continue;
    }

    int r = 0;
    int g = 0;
    int b = 0;

    if (parseRoiRgbLine(line, index, r, g, b)) {
      roiR[index] = r;
      roiG[index] = g;
      roiB[index] = b;

      Serial.print("ROI");
      Serial.print(index);
      Serial.print(" RGB=");
      Serial.print(r);
      Serial.print(",");
      Serial.print(g);
      Serial.print(",");
      Serial.println(b);

      continue;
    }

    int groupIndex = 0;
    String group = "UNKNOWN";

    if (parseGroupColorLine(line, groupIndex, group)) {
      groupColor[groupIndex] = group;
      groupReceived[groupIndex] = true;

      if (groupIndex == 1) {
        GroupA = group;
       } else if (groupIndex == 2) {
  GroupB = group;
} else if (groupIndex == 3) {
  GroupC = group;
} else if (groupIndex == 4) {
  GroupD = group;
} else if (groupIndex == 5) {
  GroupE = group;
} else if (groupIndex == 6) {
  GroupF = group;
}

      Serial.print("GROUP");
      Serial.print(groupIndex);
      Serial.print(" COLOR=");
      Serial.println(group);

      continue;
    }

    if (line == "END" || line.startsWith("END,")) {
      // GROUPはNicla側からは送られてこなくなったため、
      // ROIデータが揃っているかどうかだけで判定する。
      // (groupReceived / GROUP,... のパースは後方互換のため残しているが、
      //  ここでは使用しない)
      bool allRoiReceived = true;

      for (int i = 1; i <= ROI_COUNT; i++) {
        if (!roiReceived[i]) {
          allRoiReceived = false;
          break;
        }
      }

      if (allRoiReceived) {
        setting_variable();
        g_colorReadValid = true;
        Serial.println("Color read END with ROI data");
        return true;
      }

      g_colorReadValid = false;
      Serial.println("Color read END but data incomplete");
      printAllRoiColors();
      printAllGroupColors();
      return false;
    }
  }

  g_colorReadValid = false;
  Serial.println("Color read timeout");
  return false;
}

bool requestColorRead(int setId) {
  if (setId == 0) {
    return requestColorRead();
  }

  if (setId < 0 || setId >= COLOR_SET_COUNT) {
    Serial.println("Invalid setId");
    return false;
  }

  Serial.print("requestColorRead SET");
  Serial.print(setId);
  Serial.println(" is not supported by current Raspberry Pi code.");

  setColorReadValid[setId] = false;
  return false;
}

// =========================================================
// GROUP変数更新
// GROUP_ROI_INDEXESに従って各GROUPのROI票数を集計し、
// 票数が多いGROUPから優先的に、他GROUPでまだ使われていない
// 色を割り当てる(同じ色が複数GROUPに重複しないようにする)。
// 以前はNicla側で計算していたロジックをMotion側へ移植したもの。
// =========================================================
void setting_variable() {

  GroupCandidates candidates[GROUP_COUNT];

  ColorId results[GROUP_COUNT] = {
    COLOR_UNKNOWN,
    COLOR_UNKNOWN,
    COLOR_UNKNOWN,
    COLOR_UNKNOWN,
    COLOR_UNKNOWN,
    COLOR_UNKNOWN
  };

  int colorCount[4] = {
    0, 0, 0, 0
  };


  // =====================================================
  // GroupA～Fの候補を作成
  // =====================================================
  for (int group = 0; group < GROUP_COUNT; group++) {

    if (!makeGroupCandidates(
          group,
          candidates[group]
        )) {

      Serial.print("Group candidate error: ");
      Serial.println(group);

      GroupA = "UNKNOWN";
      GroupB = "UNKNOWN";
      GroupC = "UNKNOWN";
      GroupD = "UNKNOWN";
      GroupE = "UNKNOWN";
      GroupF = "UNKNOWN";

      return;
    }
  }


  // =====================================================
  // Group×色の得点を事前計算
  // =====================================================
  buildGroupMatchScoreTable();


  // =====================================================
  // 色の組み合わせを探索
  // =====================================================
  bool success = searchGroupColors(
    0,
    candidates,
    results,
    colorCount,
    0
  );


  if (!success) {

    Serial.println(
      "Color grouping error: no valid combination"
    );

    GroupA = "UNKNOWN";
    GroupB = "UNKNOWN";
    GroupC = "UNKNOWN";
    GroupD = "UNKNOWN";
    GroupE = "UNKNOWN";
    GroupF = "UNKNOWN";

    return;
  }


  // =====================================================
  // 最終結果を保存
  // =====================================================
  GroupA = colorIdToString(results[0]);
  GroupB = colorIdToString(results[1]);
  GroupC = colorIdToString(results[2]);
  GroupD = colorIdToString(results[3]);
  GroupE = colorIdToString(results[4]);
  GroupF = colorIdToString(results[5]);

  groupColor[1] = GroupA;
  groupColor[2] = GroupB;
  groupColor[3] = GroupC;
  groupColor[4] = GroupD;
  groupColor[5] = GroupE;
  groupColor[6] = GroupF;


  // 残り個数を更新
  updateColorRemaining(colorCount);


  Serial.println(
    "Group variables updated (6-group solver)"
  );

  printAllGroupColors();
}

// =========================================================
// GROUPだけを手入力
// 形式: "Y,B,G"
// Group1, Group2, Group3 に直接入れる
// =========================================================
bool setManualGroupColors(String csv) {
  csv.trim();
  csv.toUpperCase();

  String parts[3];
  int count = splitCsv(csv, parts, 3);

  if (count < 3) {
    Serial.println("Manual group error: use Y,B,G");
    return false;
  }

  String c1 = normalizeColor(parts[0]);
  String c2 = normalizeColor(parts[1]);
  String c3 = normalizeColor(parts[2]);

  if (c1 == "UNKNOWN" || c2 == "UNKNOWN" || c3 == "UNKNOWN") {
    Serial.println("Manual group error: color must be Y/B/W/G");
    return false;
  }

  Group1 = c1;
  Group2 = c2;
  Group3 = c3;

  groupColor[1] = Group1;
  groupColor[2] = Group2;
  groupColor[3] = Group3;

  g_colorReadValid = true;
  g_manualColorMode = true;

  Serial.println("Manual GROUP colors set OK");
  printAllGroupColors();

  return true;
}

// =========================================================
// 色読み + Group1〜3へ保存
// PiへREADを送り、読み取った結果をGroup1〜3へ反映する
// =========================================================
bool readColorAndSetVariable() {
  g_manualColorMode = false;

  bool ok = requestColorRead();

  if (!ok) {
    blinkColorReadErrorLed();

    // 色読み失敗時のセーフティー
    // GroupA～F = G G B B Y Y
    GroupA = "G";
    GroupB = "G";
    GroupC = "B";
    GroupD = "B";
    GroupE = "Y";
    GroupF = "Y";

    groupColor[1] = GroupA;
    groupColor[2] = GroupB;
    groupColor[3] = GroupC;
    groupColor[4] = GroupD;
    groupColor[5] = GroupE;
    groupColor[6] = GroupF;

    g_colorReadValid = true;
    g_manualColorMode = true;

    return true;
  }

  g_colorReadValid = true;
  g_manualColorMode = false;

  return true;
}

// =========================================================
// 既存SET0の色データ取得
// =========================================================
String getRoiColor(int index) {
  if (index < 1 || index > ROI_COUNT) {
    return "UNKNOWN";
  }

  return roiColor[index];
}

int getRoiR(int index) {
  if (index < 1 || index > ROI_COUNT) {
    return 0;
  }

  return roiR[index];
}

int getRoiG(int index) {
  if (index < 1 || index > ROI_COUNT) {
    return 0;
  }

  return roiG[index];
}

int getRoiB(int index) {
  if (index < 1 || index > ROI_COUNT) {
    return 0;
  }

  return roiB[index];
}

String getGroupColor(int index) {
  if (index < 1 || index > GROUP_COUNT) {
    return "UNKNOWN";
  }

  return groupColor[index];
}

// =========================================================
// 追加SET用の取得
// =========================================================
String getRoiColor(int setId, int index) {
  if (setId == 0) {
    return getRoiColor(index);
  }

  if (setId < 0 || setId >= COLOR_SET_COUNT) {
    return "UNKNOWN";
  }

  if (index < 1 || index > MAX_ROI_COUNT) {
    return "UNKNOWN";
  }

  return setRoiColor[setId][index];
}

int getRoiR(int setId, int index) {
  if (setId == 0) {
    return getRoiR(index);
  }

  if (setId < 0 || setId >= COLOR_SET_COUNT) {
    return 0;
  }

  if (index < 1 || index > MAX_ROI_COUNT) {
    return 0;
  }

  return setRoiR[setId][index];
}

int getRoiG(int setId, int index) {
  if (setId == 0) {
    return getRoiG(index);
  }

  if (setId < 0 || setId >= COLOR_SET_COUNT) {
    return 0;
  }

  if (index < 1 || index > MAX_ROI_COUNT) {
    return 0;
  }

  return setRoiG[setId][index];
}

int getRoiB(int setId, int index) {
  if (setId == 0) {
    return getRoiB(index);
  }

  if (setId < 0 || setId >= COLOR_SET_COUNT) {
    return 0;
  }

  if (index < 1 || index > MAX_ROI_COUNT) {
    return 0;
  }

  return setRoiB[setId][index];
}

int getRoiCount(int setId) {
  if (setId < 0 || setId >= COLOR_SET_COUNT) {
    return 0;
  }

  return roiCount[setId];
}

String getColorSetName(int setId) {
  if (setId < 0 || setId >= COLOR_SET_COUNT) {
    return "UNKNOWN";
  }

  return colorSetName[setId];
}

// =========================================================
// 成功確認
// =========================================================
bool hasValidColorRead() {
  return g_colorReadValid;
}

bool hasValidColorRead(int setId) {
  if (setId == 0) {
    return hasValidColorRead();
  }

  if (setId < 0 || setId >= COLOR_SET_COUNT) {
    return false;
  }

  return setColorReadValid[setId];
}




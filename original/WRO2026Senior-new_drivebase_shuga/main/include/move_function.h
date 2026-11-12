//pivot()の動作確認用
void pivotcheck();
//spin()の動作確認用
void spincheck();
//試験的機能をつけたいときに使ってね
void test_function();
//ボウルの動き。
void bowl();

int getMosaicDepth(char color, int count);

bool moveBetweenMosaicColors(char fromColor, char toColor);

void put_mosaic_both(int openback = 1);

void move_mosaic2_3(char color1, char color2);

void move_mosaic1(char color1, char color2);

void section_A();

void section_B();

void section_C();

void robot_start(int go_point = 1);

void catch_block(int turn_side, int depth = 65, int back_offset = 115);

void motikae(int depth);


void leadoff(int to_lift = 1);

enum Entry {
  G,
  B
};

// =====================================================
// モザイクフレームを避けてセメントターゲットエリアYを中継し
// て反対側に向かう。Entryは入る側の色を指定してね。
// G,Bがつかえます''はいりません
// =====================================================
void avoidGo(Entry);
// =====================================================
// 白セメントまで行く関数。
// =====================================================
void go_white(Entry);
// =====================================================
// モザイクの初期位置の手前のブロックの左右を入れ替える
// =====================================================
void swapOnField();
// =====================================================
// 4個つかんでいる状態の左右を入れ替える
// =====================================================
void swapGrabbed();
// =====================================================
// cementをとる動作。tlineが終わった直後に入れてね
// =====================================================
void cement(int backMm = -200,int catchDeg = 0);
// =====================================================
// 4個持っているとき、前方の片方だけ置くことができる。
// sideは-1で左を離す。1で右を離す
// =====================================================
void oneput(int side);
// =====================================================
// barrierの上のオブジェクトを円形アームでつかむ。
// =====================================================
void on_barrier(int mm = 215);

void resetMosaicRemaining();

void color_EX(int ROI_index);
void color_EX_if(int ROI_Index);
// =====================================================
// モザイクフレームの中のモザイクをつかみます。
//sideで左と右を-1と1で指定
//depthは手前だけつかむなら1。4個つかむなら2
// =====================================================
void frame_mosaic_catch(int side = -1, int depth = 2);

void moveTo(float x, float y, int speed);
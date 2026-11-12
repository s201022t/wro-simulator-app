#include "program.h"

// 上から順に実行する走行手順を書く。
// 動作に失敗したらfalseで終了する。
bool runProgram() {
  return robot.drive(
    500.0f, 500.0f,   // 左右MAX速度 [mm/s]
    150.0f, 150.0f,     // 左右MIN速度 [mm/s]
    1000.0f, 1000.0f  // 左右の移動距離 [mm]
  );
  return true;
}

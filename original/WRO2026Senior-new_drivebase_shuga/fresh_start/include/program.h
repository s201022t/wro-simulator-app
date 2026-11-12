#pragma once
#include "robot.h"

// 実体と初期化・開始待ちはprogram.cppが管理する。
extern Robot robot;
// main.cppに走行手順を書く。失敗時はfalseで手順を終了する。
bool runProgram();

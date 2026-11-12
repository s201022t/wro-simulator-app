#pragma once
#include "drive_config.h"
void motorSetup(const DriveConfig& config);

// ピンを初期化し、左右の出力を0にする。
void motorSetup();

// 左右のPWMを直接指定する。範囲: -255～255。
// 正: 前進、負: 後退、0: 出力OFF。補正や自動停止は行わない。
void motorPower(int left, int right);

// 両輪を出力OFFにする（ショートブレーキではない）。
void motorStop();

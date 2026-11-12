# 実装・調査レポート

## 元コードから確認した事項

- モーター：右 M1 GP8/9、左 M2 GP10/11、PWM -255..255
- ライン：右端/右/左/左端 GP26/27/28/29
- エンコーダ：左 GP26/27、右 GP28/29。ラインと物理ピンが完全に重複
- 互換名 ENC1 GP5/4、ENC2 GP6/7 も別途定義
- 始動・停止ボタン：いずれも GP20
- MPU9265：Wire1、SDA GP18、SCL GP19、候補 0x68/0x69、±1000 dps、32.8 LSB/(deg/s)
- 制御目標周期：125 µs（8 kHz）
- 車輪径 62.4 mm、トレッド 167.0 mm、620 count/rev
- `runCurrentProgram()` の実体は `motorPower(255,255)` のみ
- カメラ UART：TX GP16、RX GP17。ROI 12、GROUP 6
- サーボ：GP0 昇降、GP1 把持、GP2 格納、GP3 円形アーム

ネイティブ層は Arduino の時刻、GPIO、PWM、割り込み、Serial、Wire、NeoPixel、watchdog を提供します。元の走行ループが `micros()` を読むたび仮想時間と物理を進め、描画速度から制御結果を分離します。

## 検証済み

- 元の `motion_lib.cpp` と仮想 HAL の GCC 14 / C++20 ビルド
- MPU 初期化、500サンプルのジャイロ校正
- 300 mm 前進、右旋回、300 mm 前進の完走
- 500 mm 後進
- 元の `servo_lib.cpp` の4サーボ S 字タスク
- 元の `pi_color_lib.cpp` の READ/ROI/RGB/END UART 処理
- JSON Lines 状態出力、ローカル HTTP UI、イベント配信
- 自動統合テスト 7 件

## 未完了

公式ルール自動採点、厳密な剛体回転、ユーザー C++ 編集、ネイティブ制御スレッドの停止・再開・ステップ、Windows 実機検証は未完了です。オブジェクト物理とカメラは調整可能な簡易モデルであり、現状を公式判定用の完成版とは扱いません。

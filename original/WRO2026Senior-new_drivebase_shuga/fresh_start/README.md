# モーター操作から始める新規プロジェクト

発進時の左右差を調べる場合は [FF用の発進PWM測定](FF_CALIBRATION.md) を参照してください。通常走行と別の `pico2_ff_calibration` 環境を用意しています。

Pico 2 / Arduino / PlatformIO用。元の `main` プロジェクトとは独立しており、元のソースやライブラリは読み込みません。

## 始め方

1. VS Codeの「フォルダーを開く」で、この `fresh_start` フォルダーを開きます。
2. `src/main.cpp` に処理を書きます。
3. PlatformIOのBuildでビルドし、UploadでPico 2に書き込みます。

PlatformIO CLIなら、このフォルダーで `pio run` がビルド、`pio run -t upload` が書き込みです。
UF2は `.pio/build/pico2/firmware.uf2` に生成されます。

現在の `main.cpp` はGPIO20のボタンが押されるまで停止状態で待機します。ボタンはGP20とGNDの間に接続し、INPUT_PULLUPで押下時LOWを検出します（30msのチャタリング対策）。
押されると左右を出力100で前進させ、各輪が開始位置から絶対値620カウント（暫定1周）に達すると個別に出力OFFにします。1回の動作は最大10秒で打ち切ります。
目標角度と出力は `LEFT_TARGET_DEG` / `RIGHT_TARGET_DEG` / `DRIVE_POWER` で変更できます。初期値は左右360度です。
停止後はシリアルモニター（115200）に `TARGET REACHED` または `FAILED / TIMEOUT` と左右のカウントを表示します。
終了後は次のボタン操作を待ち、離してから押すたびに再実行できます。起動時から押していた場合も、一度離してから押してください。押下・解放とも30msの安定を確認します。

## Robotクラスから使う

`#include "robot.h"` と `Robot robot(20);` を用意します。`robot.begin()` でモーター・エンコーダ・開始ボタンをまとめて初期化します。
共有ハードウェアのロボット1台用なので、Robotオブジェクトは1個だけ作り、同じコアで使用してください。

```cpp
void setup() {
  robot.begin();
}

void loop() {
  robot.waitForStart();
  bool ok = robot.movePulses(620, 620, 100);
  robot.stop();
  // ok == falseなら時間切れや引数不正。ここで結果に応じた処理を書く。
}
```

用意した `main.cpp` では、走行手順を `runProgram()` にまとめています。手順を増やすときはここを編集してください。

| メソッド | 用途 |
| --- | --- |
| `robot.begin()` | 初期化 |
| `robot.waitForStart()` | 停止し、ボタン解放→押下待ち |
| `robot.setPower(左, 右)` | PWM直接出力 |
| `robot.stop()` | 両輪出力OFF |
| `robot.movePulses(左, 右, 出力, タイムアウトms)` | カウント指定で移動 |
| `robot.moveRevolutions(左, 右, 出力, タイムアウトms)` | 回転数指定で移動 |
| `robot.moveDegrees(左の度数, 右の度数, 出力, タイムアウトms)` | モーター角度指定で移動 |
| `robot.counts()` | 左右の符号付きカウント取得 |
| `robot.resetCounts()` | カウントを0にする |
| `robot.printResult(Serial, ok)` | 結果とカウントを表示 |

移動の出力は省略時100、タイムアウトは省略時10000msです。以下の従来関数も引き続き利用できますが、通常はRobotクラスから呼び出せます。

### 角度指定

`moveDegrees()` は呼び出した位置からのモーター軸の回転角を指定します。車体の旋回角ではありません。

```cpp
robot.moveDegrees(90, 90, 100);   // 両輪を前進方向へ90度
robot.moveDegrees(180, 0, 100);   // 左だけ180度
robot.moveDegrees(0, -360, 100);  // 右だけ後退方向へ1周
```

360度=620カウントとして換算し、最も近い整数カウントに丸めます（1カウント約0.58度）。
停止はカウント差の絶対値で判定し、完了でtrue、時間切れ・引数不正でfalseを返します。
固定PWMで出力OFFにするため、実際の停止角度には惰性によるずれがあります。

## モーター操作

`motorSetup()` を最初に呼んでから、`motorPower(左, 右)` を使います。
出力範囲は -255～255（範囲外は制限）。正が前進、負が後退、0が出力OFFです。
指定値はPWM出力であり、回転数や距離ではありません。

```cpp
motorPower(100, 0);    // 左だけ
delay(1000);          // 1秒維持
motorStop();          // 両輪の出力をOFF

motorPower(0, 100);    // 右だけ
delay(1000);
motorStop();

motorPower(-100, -100); // 両輪を後退方向へ
delay(1000);
motorStop();
```

出力は次の指示まで続きます。`motorStop()` は出力OFFで、ショートブレーキではありません。
直接出力には自動停止処理はありません。ジャイロ・サーボの処理は含みません。GPIO20は開始待ち専用で、走行中の停止ボタンとしては動作しません。

## エンコーダで回転量を指定する

従来関数を直接使う場合は `#include "encoder.h"` を追加し、`motorSetup()` の後に `encoderSetup()` を呼びます。Robotクラスでは `robot.begin()` がこれらを行います。

```cpp
bool ok = motorMoveRevolutions(1, 1, 100); // 左右1周、PWM100
if (!ok) Serial.println("Move failed / timed out");
// 以下はそれぞれ独立した使用例
motorMovePulses(620, 0, 100);             // 左だけ620カウント（暫定1周）
motorMoveRevolutions(0, -0.5f, 100);      // 右だけ逆方向に半周
EncoderCounts counts = encoderRead();   // counts.left / counts.right
encoderReset();                         // 累積カウントを0にする
```

移動量は呼び出した時点からの相対値。正が前進、負が後退、0はその輪を動かしません。
停止判定は `abs(現在のカウント - 開始時のカウント) >= abs(目標)` です。
カウントが正負どちらへ進んでも指定量で停止します。往復分の累積距離ではなく、開始位置からの差を使います。
`encoderRead()` は診断用に符号付きの値を返します。
各輪は目標に達した時点で個別に出力OFFになります。両輪の完了まで関数内で待機します。
出力は固定PWMで、左右の速度同期や減速制御はありません。惰性で目標を超える場合があります。
目標到達で `true`、未初期化・引数不正・時間切れで両輪を停止して `false` を返します。
第4引数はタイムアウト（ミリ秒、既定10000）。例: `motorMoveRevolutions(5, 5, 100, 30000)`。
読み取り・リセット・移動関数はsetup/loopと同じコアの通常処理から呼んでください。

### 4逓倍と1周のカウント数

A/B両相の立ち上がり・立ち下がり（CHANGE）を読む4逓倍です。
AB状態の遷移から方向を判定し、変化なし・両ビット同時変化は数えません。
`motorMovePulses()` の引数と `encoderRead()` の値は4逓倍後のカウント数です。
`include/encoder.h` の `ENCODER_PULSES_PER_REV` は、4逓倍後で1周620カウントとして暫定設定しています。
620は既に4逓倍後の値なので、さらに4倍する必要はありません。
`encoderReset()` 後に対象の軸を手で正確に1周回し、`encoderRead()` の変化量の絶対値を設定してください。
表示を前進方向で増加に揃えたい場合は `src/encoder.cpp` の該当する `LEFT_DIR` / `RIGHT_DIR` の符号を反転します。絶対値で停止判定するため、停止のための符号合わせは不要です。
符号は元プログラムと同じ左+1・右-1を初期値としています。

## 配線とファイル

既存ロボットと同じGPIO番号・方向を使用しています。

| モーター | A | B |
| --- | --- | --- |
| 左（M2） | GP10 | GP11 |
| 右（M1） | GP8 | GP9 |

| エンコーダ | A | B |
| --- | --- | --- |
| 左 | GP26 | GP27 |
| 右 | GP28 | GP29 |

- `src/main.cpp`: 自分の処理を書く場所
- `include/robot.h`: Robotクラスの操作一覧
- `src/robot.cpp`: 初期化、ボタン待ち、モーター・エンコーダの操作窓口
- `include/motor.h`: モーター関数の宣言
- `src/motor.cpp`: ピン設定とPWM出力
- `include/encoder.h`: 回転あたりパルス数とエンコーダ関数の宣言
- `src/encoder.cpp`: 割り込みカウントと回転量による停止処理
- `platformio.ini`: Pico 2用ビルド設定


## mm・mm/s指定の共通走行関数

新しい距離走行は Robot::drive() を基礎関数として使用します。既存の moveDegrees 等は従来の固定PWM動作を維持します。

```cpp
DriveConfig config; // Robot生成前に設定する
// config.leftWheelDiameterMm = 56.0f; // 仮の初期値。必ず実測値へ変更
Robot robot(20, config);

// setup()でrobot.begin()、loop()でrobot.waitForStart()の後に呼ぶ
// 引数: 左MAX, 右MAX, 左MIN, 右MIN [mm/s], 左距離, 右距離 [mm]
bool ok = robot.drive(150.0f, 150.0f, 20.0f, 20.0f, 1000.0f, 1000.0f);
// 後退: robot.drive(150, 150, 20, 20, -1000, -1000);
// 片輪: robot.drive(100, 0, 10, 0, 200.5f, 0);
// 超信地旋回: robot.drive(100, 100, 10, 10, 200, -200);
```

速度は正の大きさ、距離の符号が方向です。floatに対応し、第7引数はタイムアウトms（既定30000）。完了まで待機し、成功でtrue、不正値・未初期化・時間切れでfalseを返します。終了時は両輪のPWMを0にします。開始ボタンは走行中の停止には使用しません。

左右の距離比とMAXから共通の所要時間を求め、同時到着する軌道を生成します。MINは巡航速度の下限です。左右の距離比に対してMIN/MAXを同時に満たせなければfalseを返します。S字の始動・停止では0から速度を変化させ、終点補正でもMIN未満を使用します。MINは最低PWMではありません。

smoothstepによる速度のS字立ち上げ・立ち下げ、各輪の速度PI制御、軌道位置誤差補正、左右進捗率の同期補正を行います。位置制御には必要に応じた逆方向補正を含みます。両輪が位置許容差内で低速になり、settleTimeMsだけ安定すれば完了です。出力OFF後の位置保持・ショートブレーキは行いません。

左右同期は参考プログラムのtank_control::mixと同じく、先行側だけを弱める方式です。左の進捗率が低い場合は右の目標速度だけを下げ、左は同期前の目標速度を維持します。同期前の目標速度にはS字軌道と位置誤差補正を含み、各輪のMAX以内に制限します。同期そのものによる加速・逆転は行いません。PWMを直接合わせるのではなく、補正後の目標速度を各輪の速度PI制御で追従します。実速度が上限以内になることを保証するものではありません。

比較は距離そのものではなく「走行距離/目標距離」で行うため、左右の目標距離が違う走行にも対応します。後退・超信地旋回も各輪の指令方向へ正規化して比較します。片輪走行、どちらかの目標速度が0以下の場合、時間軌道終了後は同期を外し、各輪の位置制御・停止処理を優先します。syncKpは従来と同じ進捗率に対するゲインで、0なら同期無効です。参考コードのPWM/カウント単位のSYNC_KPとは単位が異なるため、数値をそのまま移植しません。ジャイロ補正は追加していません。

### 設定と精度確認

include/drive_config.h の DriveConfig をRobot構築前に設定します。GPIOはPico 2で使用できる、開始ボタンを含めて重複のないピンを指定してください。同一ハードウェアを扱うRobotは1個だけ使用します。

- left/rightMotorA/B、left/rightEncoderA/B: 配線GPIO。
- left/rightEncoderDirection: +1か-1。手動で前進させると各カウントが増えるよう調整してください。新しいdriveの符号付きフィードバックには方向一致が必要です。
- left/rightWheelDiameterMm: 左右それぞれの実効タイヤ径。初期値56mmは仮値です。
- left/rightCountsPerRev: タイヤ軸1回転の4逓倍後カウント数。ギヤ比も含めて実測してください。
- left/rightPwmPerMmS: 定常速度に対するPWMの比例係数。speedKp/speedKiは速度PI、positionKpは軌道位置補正、syncKpは左右進捗率補正です。
- rampTimeSeconds: 加減速の時間（短距離では短縮）。controlPeriodMs: 制御周期、positionToleranceMm: 各輪終点の許容差、settleTimeMs: 安定確認時間。

旧moveRevolutions/moveDegreesは互換性のため従来どおりENCODER_PULSES_PER_REVで換算します。DriveConfigの左右別換算値はdrive専用です。

1000mm前進・後退で縦±5mm／横±10mmは実機評価の目標であり、保証値ではありません。エンコーダのみではタイヤの滑りや実際の横ずれを観測できません。実測したタイヤ径・カウント数・方向を設定し、低速度から速度ゲインと左右径を調整してください。同じ床・荷重で1000mmの前進／後退をそれぞれ複数回実施し、出力OFF後の縦ずれと横ずれを測って判定します。必要に応じてジャイロ等の車体姿勢フィードバックを追加してください。


## main.cppに走行手順だけを書く

main.cppにはprogram.hの読み込みとrunProgram()だけを置きます。既存の「左右360度を10回」の手順を維持しています。初期化、ボタン待ち、終了時の停止、結果表示はsrc/program.cppが担当します。配線・タイヤ径などの既定値はinclude/drive_config.h、個別の設定値と開始ボタンはsrc/program.cppで変更してください。

runProgram()内では動作を上から順に書きます。各移動は完了まで待機します。失敗した場合はfalseを返すことで後続手順を中止し、共通処理が停止・結果表示を行います。全手順完了時はtrueを返します。例えば1000mm前進してから1000mm後退する手順は次のとおりです。

```cpp
#include "program.h"

bool runProgram() {
  if (!robot.drive(150, 150, 20, 20, 1000, 1000)) return false;
  if (!robot.drive(150, 150, 20, 20, -1000, -1000)) return false;
  return true;
}
```

program.hで共通のrobotを宣言しています。main.cpp側でRobotを追加生成したり、setup()/loop()を定義したりする必要はありません。


## エンコーダ値のシリアル表示

シリアルモニターを115200bpsで開くと、ボタン待ち中と走行関数の実行中に約100ms間隔で左右の符号付き累積カウントを表示します。例: ENC L=625 R=625。4逓倍後の値で、resetCounts()により0に戻ります。main.cppへの表示処理追加は不要です。

DriveConfigのencoderPrintIntervalMsで表示間隔を変更できます。0にすると無効です。送信バッファが不足している場合はその回の表示を省略します。独自の待機ループやdelay()中にも表示したい場合は、通常処理からencoderPrintSerial()を定期的に呼んでください。割り込み内からは呼ばないでください。


### 逆相ブレーキの停止判定

drive()で回転方向と逆のPWMを出して減速するとき、左右それぞれのカウント変化を監視します。brakeSampleMsの判定窓内のカウント変化量合計がbrakeStopCounts以下なら、その輪の逆出力と速度積分値を0にします。累積カウントの0判定ではありません。指定した走行方向に対する逆出力は、位置補正が目的の場合も停止判定の対象です。停止判定中は毎周期出力と積分値を0にするため、停止した輪へ低い逆PWMをかけ続けません。後退指定でも同じ判定を行います。

軌道終了後、目標を許容差より越えて停止し、逆補正が上記条件で抑止されている場合、両輪の出力OFF・低速状態がsettleTimeMs継続するとfalseで終了します。この場合、表示はFAILED / TIMEOUTです。許容差外の停止を成功とは扱いません。停止後に逆方向へ再発進して位置を戻す動作は抑止されるため、行き過ぎが残る場合は減速や速度設定の調整が必要です。それ以外の未到達には従来のタイムアウトが適用されます。

停止判定時間brakeSampleMsはDriveConfigで調整できます（制御周期以上）。低速ではカウント間隔が長いため、無変化は完全停止の保証ではありません。エンコーダのカウント周期より十分長い時間で実機調整してください。main.cppへの追加は不要です。


逆相ブレーキ停止の閾値はbrakeStopCounts（既定5カウント）です。brakeSampleMsは10msです。各制御周期のカウント差の絶対値を判定窓内で合計します。0なら無変化のみ、1なら1カウント以下で逆出力を切ります。窓の始点と終点だけの差ではないため、観測できた往復移動は相殺しません。低速判定は停止保証ではなく、判定窓・閾値の実機調整が必要です。

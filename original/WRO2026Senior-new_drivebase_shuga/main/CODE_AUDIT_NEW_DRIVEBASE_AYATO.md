# new_drivebase_ayato 調査報告

## 整理実施状況

ユーザー指定により `extra_2()`〜`extra_10()` は今後使用する関数として保持（宣言・実装とも変更なし）。
参照のない内部関数8個と対応宣言、空の `main/move_function.h`、includeだけの `main/src/robot_app.cpp`、操作出力 `witch -c old-version 54a15e3` を削除。
バックアップ・PCアプリ派生版・Git作業ツリーは保管/使用状況未確認のため保持。制御ロジックのリファクタリングは未実施。
以下は削除前の調査記録であり、関数一覧とソース行番号は調査時点のもの。


調査日: 2026-09-23。基準コミット: e729886。現在の未コミット main/src/main.cpp（tankの前後進6回）も含めて調査。
調査時点ではコードの変更なし。その後の削除内容は冒頭の整理実施状況を参照。

## 結論

- 先に削除してよい可能性が高いものは、参照のない内部関数8個、空ヘッダーと処理のないcpp。
- 「現在のsetup/loopから到達しない」と「不要」は異なる。現在はtankテストだけなので、競技動作・旋回・ライン追従・色認識の多くは到達しないが、保持する。
- 制御周期は全機能で共通ではない。走行125µs目標、ライン追従2ms待機＋処理時間、オンモード最短1ms、サーボ軌道最短2ms、停止監視最短5ms。
- tankbSetには周期・協調更新・停止監視に問題があり、使用前の修正対象。削除と動作修正は別の変更として扱う。
- PlatformIOビルド成功。main/testの9本はARM C++14コンパイラの-fsyntax-onlyでstatic_assertを含めて成功。実機計測・実機動作確認は未実施。

## 調査方法と範囲

main/platformio.iniのmain/src/*.cppが本体ビルド対象。main/includeの依存先、main/test、別ファームウェアnicla_color_read、PC用NiclaRoiApp、バックアップ、Git作業ツリーも確認。
main/srcから関数定義215件を抽出し、コメント・文字列を除いた関数本体の参照、割り込み登録、ヘッダー参照を照合した。
正規表現ベースの静的調査でありC++ ASTの完全解析ではない。オーバーロードは同名をまとめて保守的に扱う。外部から利用する公開APIや手動実行されるPythonスクリプトの不要性までは断定しない。
本体はsetup/loopからの到達関数名66個。ただし協調タスクの条件分岐も到達可能として含めるため、実行履歴を意味しない。
既存UNUSED_FUNCTIONS.mdは2026-09-06時点。そこに未使用とあるarcや周期計測API等を根拠に一括削除しない。

## 1. 削除候補

### A. 現行ソースで参照のない内部関数

以下はstatic関数で、宣言・定義・コメント以外の参照がmain/src・main/includeに見つからない。削除時は対応する内部プロトタイプも同時に削除する。既存の呼び出されるブレーキ関数は削除しない。

| 関数 | 判断 |
|---|---|
| [setMotor()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:892) | motorPowerを呼ぶだけの未使用ラッパー |
| [shortBrakeCurrentMotors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:927) | 300msショートブレーキの旧ラッパー |
| [brakeMotors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:1942) | brakeCurrentMotorsの旧互換ラッパー |
| [brakeSpinMotors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:1950) | 旧逆相旋回ブレーキ。現在のrunTurnPDから参照なし |
| [calcTurnTrapezoidPower()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2497) | 旧旋回台形計算。現在はrunTurnPD＋turn_s_curve |
| [calcPivotTrapezoidPower()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2527) | 旧pivot台形計算。現在はrunTurnPD＋turn_s_curve |
| [degToPulse()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:3399) | 旧旋回角→パルス変換。現行旋回は角度PD |
| [pivotDegToPulse()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:3413) | 旧pivot角→パルス変換。参照なし |

### B. 空関数・空ファイル等

| 対象 | 根拠・扱い |
|---|---|
| [extra_2()〜extra_10()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/extra.cpp:288) | 9個とも空の関数。呼び出しはmain.cppのコメントだけ。ユーザー指定で今後使用するため、宣言・実装・コメント例をすべて保持する。 |
| [main/move_function.h](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/move_function.h) | 0バイト。実際の宣言はmain/include/move_function.h。依存のない空ファイルとして削除候補。 |
| [main/src/robot_app.cpp](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/robot_app.cpp) | includeのみで関数・変数定義がない。cppのみ削除候補。robot_app.hはrunCurrentProgramの宣言に必要なので残す。 |
| [witch -c old-version 54a15e3](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/witch -c old-version 54a15e3) | lessヘルプの文字列が保存されたファイル。本体ソース・設定から参照なし。不要な操作出力として削除候補。 |

### C. 要確認のファイル・フォルダー

| 対象 | 調査結果 |
|---|---|
| tank-backup-20260916-164331/ | 本体ビルド対象外。旧motion_libの複数コピーとundo.patch。READMEに「以前からの未コミット変更も含む」とあるため、Gitだけで復元できると断定できない。履歴・差分の保全後にアーカイブまたは削除。 |
| NiclaRoiAppの基準gui以外の派生版6本 | run_app.bat / build_exe.bat / build_test.batはnicla_roi_app_gui.pyを参照。派生版への内部参照は見つからないが、手動起動用かもしれないため使用版を確認して整理。 |
| .worktrees/new_drivebase/ | Gitに登録されたdetached HEADの作業ツリー。本体ソースとは別。不要なら変更・未追跡ファイルを確認しgit worktree removeで整理する。通常の再帰削除対象にはしない。 |
| main/.pio/ | PlatformIOの生成物・依存ライブラリ。ソースではない。再生成可能だが依存の再取得が必要な場合がある。 |
| main/UNUSED_FUNCTIONS.md | 旧調査。削除より本報告への置換・旧版明記を推奨。 |
| main/2026_weekly_schedule.xlsx | ファームウェアの依存ではないが作業計画資料。コードから未参照という理由では削除しない。 |

派生版の内訳: nicla_roi_app_gui_no_gray_test_v4.py、nicla_roi_app_gui_no_gray_test_v4_hsv_adjust.py、nicla_roi_app_gui_learning_test.py、nicla_roi_app_gui_learning_test_black_green.py、nicla_roi_app_red_checklist_fixed.py、nicla_roi_app_list.py。

### D. 残すもの

- main/includeの制御用ヘッダー9本（arc_geometry、arc_pulse_power、arc_stall_guard、line_sensor_normalization、servo_s_curve、tank_control、tank_on_brake、turn_progress、turn_s_curve）は現行コードの依存。対応する9本のtestも保持。
- main/src/move_function.cpp、extra_1、pi_color_lib.cpp、nicla_color_read、基準NiclaRoiApp。現在のtankテストで使わなくても競技シーケンス・色認識・調整に必要。
- resetMotionTimingStats / getMotionTimingStatsは今回の周期実測に使うので残す。
- setLineSensorCalibrationや診断APIは現時点で参照がなくても、校正・実機確認用として保持を推奨。
- 公開移動APIのtlineL、ex_tline、arc、parallelShift、moveTo、tankSyncOffなどは「未使用だから削除」としない。

## 2. 移動・回転と周辺処理の周期

周期は「ループ目標」「末尾の待ち時間」「センサー出力周期」「PWM搬送周期」を分ける。以下のHzは設定上の値または上限であり、実測値ではない。

| 関数・処理 | 設定／待ち時間 | 実効周期の読み方 |
|---|---|---|
| tank | MOTION_CONTROL_PERIOD_US=125µs | 最大8kHzを目標。1周期は概ねmax(125µs,処理時間)。I2C・割り込み・サーボ更新等で超過する。 |
| tankbの3段階 | 各段階125µs目標 | tank同様。途中の方向変更10ms、ブレーキは別の待機。 |
| spin / pivot → runTurnPD | 125µs目標 | ジャイロPD・S字・同期の処理込み。終了時の50msショートブレーキは周期とは別。 |
| arc → runArc | 125µs目標 | ジャイロ・左右パルス・ストール判定込み。 |
| parallelShift | 独自周期なし | 複数のrunArcに委譲する合成動作。 |
| moveTo | 独自周期なし | spin / tank等の呼び出しに従う合成動作。 |
| line | servoDelay(2) | 2ms＋読み取り・計算時間。500Hzは厳密な固定値ではなく目安の上限。 |
| edgeLine | 各走行段階servoDelay(2) | 同上。 |
| tline / tlineL / tlineR → tlineInternal | 各走行段階servoDelay(2) | 同上。 |
| ex_tline | ライン部分servoDelay(2) | 終端の追加距離はtankに委譲。 |
| tankOn / tankSyncOn → updateMotionTasks | 最短1000µs | 協調呼び出しで最大1kHz。updateServoTasks等が呼ばれないと更新されない。tankSyncOff時は同期補正なし。 |
| tankBrakeLeft / tankBrakeRight | 5msごとに片輪確認、逆相最大100ms | 制御ループ通常周期ではなく片輪制動。 |
| tankbSet | 明示周期なし | ADCを連続読み取りするbusy loop。通常時にupdateServoTasks/updateMotionTasksがなくオンモード同期が継続しない。後述。 |
| kabe | servoDelay(1)＋処理時間 | PWM/同期更新は約1ms以上。壁当て判定は80msごと。開始300msは判定除外、3秒タイムアウト。 |
| waitUntilMotorsStopped | 5msごと、内部servoDelay(1) | 両輪1pulse以下が20ms連続、待機最大500ms。 |
| brakeCurrentMotors | servoDelay(5)、最大100ms | 5ms間の変化を見る既存逆相ブレーキ。共通の20ms停止継続判定とは別。 |
| updateServoTasks | 最短2ms | 軌道演算は最大500Hz。updateMotionTasksは2ms制限の前に呼ばれる。 |
| catchArm / liftArm / backArm / circleArm / front / startServoMoveByPin | 独自の固定周期なし | 指令を登録しupdateServoTasksで更新。同期待機時はwaitServoTasksに委譲。 |
| waitServoTasks | delay(2)＋処理時間 | 軌道の完了待ち。サーボの物理的到達を測っているわけではない。 |
| servoDelay | delay(1)を挟み協調更新 | 指定時間を待つ間もサーボとオンモードを更新。 |
| waitStartButtonWithGyroCheck | ループ10ms待ち、診断既定500ms | ボタンとサーボ更新を繰り返し、ジャイロ生存確認は約2Hz。 |
| updateGyroModeLed | 生存診断の最短間隔250ms | 直近10msにジャイロ取得成功があれば追加I2C診断省略。 |
| calibrateGyroZ | 測定ごとservoDelay(10) | 100Hz以下。安定待ち200msと捨て読み20回。起動500サンプルは概ね5.4秒＋通信、robot_resetの10回は概ね0.5秒＋通信。 |
| checkline | servoDelay(100) | 最大約10Hzの診断表示。 |
| test_encoder | ループに固定待機なし、表示100ms間隔 | 表示は約10Hz。エンコーダ割り込み自体は独立。 |
| requestColorRead / readColorAndSetVariable | 要求時のみ | READ応答の外側タイムアウト1000ms、1行受信待ち100ms。周期1秒ではない。 |
| readLineFromPi | データなし時servoDelay(1) | UART受信のポーリング。通信速度115200bps。 |
| section_A/B/C、extra、その他競技シーケンス | 独自周期なし | 呼び出す移動・サーボと明示的なservoDelayで所要時間が決まる。 |
| buzzer / LED / 設定・取得・変換関数 | 呼び出し時のみ | 指定周波数や点灯時間は制御ループ周期とは別。 |

根拠: [125µsの定数](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/include/motion_lib.h:61)、[finishMotionControlCycle()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:415)、[updateMotionTasks()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2668)、[updateServoTasks()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:253)、[waitUntilMotorsStopped()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:817)。

## 3. センサーと出力の周期

| 対象 | コードで確認できる設定 | 注意 |
|---|---|---|
| ジャイロZ | CONFIG=0、SMPLRT_DIV=0、GYRO_CONFIG=0x08。コード上8kHz/±500dpsの設定意図 | 読み取りはgetYawDeg呼び出し時。独立8kHzタスク/FIFO回収ではない。実効取得は走行ループに依存。 |
| I2C | Wire1、GP18/19、400kHz。Z軸2バイト読み | 400kHzはバス速度。概算45クロックだけで112.5µs、さらにSTART/STOP・ライブラリ・演算等。125µs達成は実測が必要。失敗時最大3回、各失敗後500µs待ち。 |
| 加速度・磁気 | 加速度レンジ初期設定はあるが、現行の姿勢制御で加速度/磁気データの周期取得は見つからない | 走行の補正はZジャイロ積分。6軸/9軸融合制御ではない。 |
| 左右エンコーダ | A相CHANGE割り込み。ISR内でA/Bを読み方向判定 | 固定Hzなし。回転速度に応じて割り込み発生。制御でカウントを読む頻度とは別。 |
| ライン4センサー | readLineSensor内analogRead＋正規化 | 周期は呼び出し元次第。line系は2ms＋処理、tankbのセンサー探索段階は125µs目標、tankbSetは固定間隔なし。4chは順番に読む。 |
| GP20開始/停止ボタン | startPressed/stopPressedのポーリング | 待機中10ms、走行中は各ループ等。ハード割り込み監視ではない。停止時のチャタリング待ちは別処理。 |
| Niclaカメラ | camera.begin(R320x240, RGB565, 30) | 設定30fps≒33.3ms。READ/CAPTURE要求時撮影で、色判定が常時30Hzで届くわけではない。grabFrameは3000ms引数、色取得は最大5回の試行。 |
| Niclaコマンド受付 | loop末尾delay(1)、USB/UART各115200bps | 通常は約1ms＋処理。撮影・画像処理中のブロック時間が加わる。 |
| サーボ信号 | インストール済みServoライブラリREFRESH_INTERVAL=20000µs | 約50Hzの出力パルス。軌道演算500Hzとは異なる。サーボからの位置フィードバックはない。 |
| DCモーターPWM | プロジェクトにanalogWriteFreq変更なし。インストール済みArduino-Picoの既定analogFreq=1000 | PWM搬送波は既定約1kHz。125µsごとの指令更新とは別。 |

カメラ最大3秒の取得待ち/複数試行に対し、メイン側READ待ちは1秒なので、異常・遅延時の応答期限の整合性は今後確認すべき。正常時の撮影時間を実測してから変更する。

## 4. リファクタリング前に確認する問題

### tankbSetの協調更新不足

[tankbSet()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:5634) はtankOnを呼んだ後、無期限whileでラインADCを読む。通常経路にupdateServoTasks、updateMotionTasks、stopPressed、タイムアウトがない。tankOnは開始時に出力するだけで、同期・STOP監視の継続にはupdateMotionTasksが必要。
黒線未検出時はモーター出力が続き、ジャイロ同期・STOP確認・サーボ軌道が更新されない。黒線検出時にtankBrakeLeft/Right内のservoDelay経由で一時的に更新されるが、通常の連続更新の代わりにはならない。現在main.cppではコメントアウトされている。使用前に更新呼び出し、停止分岐、最大時間を追加し、実機確認する。

### 終了条件の仕様差

line/edgeLine/tline系のbrake=0は次の動作へつなぐため出力継続する経路がある。一方tankはbrake=0でも出力停止。spin/pivotは50msのショートブレーキ、tankは60msのショートブレーキ、arc等は逆相ブレーキを使う。単に共通の終了関数へ置換すると動作を変えるので、関数ごとの仕様表を先に固定する。

### ブロッキング処理と周期

協調式タスクなので通常delay、長いUART待ち、I2C再試行、Serial出力等で更新間隔は伸びる。tankbSetを除いても「設定したHzで常時動く」とは限らない。将来のリファクタリングでは周期の実測を比較する。

## 5. 推奨する作業順序

1. 現状のmain.cpp変更を含む基準を保存。現在の距離・速度・ゲイン・ブレーキ・引数・停止条件を記録する。
2. Aの8内部関数、Bの空cpp・空ヘッダー・操作出力だけを削除。extra関数は保持。公開APIや競技シーケンスを削除しない。依存を再確認してビルドと9テストを実行。
3. バックアップとPC用派生版は、使用版・未保存差分を確認した別の整理として行う。
4. tankbSetの動作不備を独立修正。リファクタリングと混ぜず検証する。
5. 純粋関数の重複から整理。既存tank_control/turn_s_curve/arc系ヘッダーを維持し、境界値・逆走・0距離・STOP・タイムアウトをテストする。
6. tankb/edgeLine/ex_tlineの段階処理、停止処理を、仕様差を保つ小さな関数へ分割。spin/pivotは既にrunTurnPD、tline系はtlineInternal、arcはrunArcに共通化されているため再共通化を急がない。
7. motion_lib.cppの役割分割（モーター/エンコーダ、ジャイロ、ライン、走行、旋回、円弧、診断）。一度に全移動関数を書き換えず、一機能ずつビルド・実機確認。
8. pi_color_libの受信・パース・色割当を分離。サーボはハード出力、非同期軌道、競技アーム操作を分ける。move_function/extraは呼び出しシーケンスなので距離や順序を維持。

## 6. 実機周期の測り方

既存resetMotionTimingStats()を対象の走行直前、getMotionTimingStats()を直後に呼ぶ。周期を乱さないためSerial出力は終了後にまとめる。

- 平均制御周期[µs] = totalControlUs / controlCycles（0除算を避ける）。
- 平均ジャイロ取得間隔[µs] = totalGyroIntervalUs / gyroIntervals。
- 実効Hz = 1000000 / 平均周期。
- maxControlUs、maxGyroIntervalUs、controlOverrunsも記録。
- この統計はfinishMotionControlCycleを通る通常走行周期が対象。終了分岐やブレーキ・停止待ち全体の所要時間、line系の周期まで網羅してはいない。gyroIntervalsは100ms超の間隔を除外する。
- line、オンモード、サーボ、Nicla処理時間は別の計測が必要。PWM/サーボ波形とエンコーダはロジックアナライザ等で実測するとコード設定と実出力を分離できる。

## 付録: main/src 関数一覧

「到達」は現在のsetup/loopからの静的到達可能性。「未到達」だけでは削除しない。周期欄の「呼出時」は独自タイマーを持たず、呼び出し元の周期またはイベントで実行されるという意味。

| 定義 | 現在の到達 | 周期／実行方式 |
|---|---|---|
| [extra_1()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/extra.cpp:11) | 未到達 | シーケンス／呼び出し先に従う |
| [extra_2()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/extra.cpp:288) | 未到達 | シーケンス／呼び出し先に従う |
| [extra_3()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/extra.cpp:292) | 未到達 | シーケンス／呼び出し先に従う |
| [extra_4()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/extra.cpp:296) | 未到達 | シーケンス／呼び出し先に従う |
| [extra_5()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/extra.cpp:300) | 未到達 | シーケンス／呼び出し先に従う |
| [extra_6()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/extra.cpp:306) | 未到達 | シーケンス／呼び出し先に従う |
| [extra_7()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/extra.cpp:311) | 未到達 | シーケンス／呼び出し先に従う |
| [extra_8()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/extra.cpp:317) | 未到達 | シーケンス／呼び出し先に従う |
| [extra_9()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/extra.cpp:323) | 未到達 | シーケンス／呼び出し先に従う |
| [extra_10()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/extra.cpp:328) | 未到達 | シーケンス／呼び出し先に従う |
| [setup()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/main.cpp:10) | 到達 | 起動時1回 |
| [loop()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/main.cpp:30) | 到達 | 開始待機→シーケンスの反復 |
| [runCurrentProgram()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/main.cpp:46) | 到達 | 呼出時（独自周期なし） |
| [buzzerSetup()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:24) | 到達 | 呼出時（独自周期なし） |
| [buzzerOn()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:33) | 到達 | 呼出時（独自周期なし） |
| [buzzerOff()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:42) | 到達 | 呼出時（独自周期なし） |
| [buzzerBeep()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:50) | 到達 | 呼出時（独自周期なし） |
| [buzzerBeepCount()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:65) | 未到達 | 呼出時（独自周期なし） |
| [boardRgbSetup()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:95) | 到達 | 呼出時（独自周期なし） |
| [boardRgbSet()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:102) | 到達 | 呼出時（独自周期なし） |
| [boardRgbOff()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:109) | 到達 | 呼出時（独自周期なし） |
| [boardRgbSetByColor()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:114) | 未到達 | 呼出時（独自周期なし） |
| [findLineSensor()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:155) | 未到達 | 呼出時（独自周期なし） |
| [setLineSensorCalibration()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:163) | 未到達 | 呼出時（独自周期なし） |
| [readLineSensor()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:171) | 未到達 | 呼出時にADC読取 |
| [resetMotionTimingStats()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:406) | 未到達 | 呼出時（独自周期なし） |
| [getMotionTimingStats()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:411) | 未到達 | 呼出時（独自周期なし） |
| [finishMotionControlCycle()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:415) | 到達 | 125µsまで不足時間を待つ |
| [isSupportedMpuWhoAmI()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:453) | 到達 | 呼出時（独自周期なし） |
| [cancelTankOn()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:614) | 到達 | 呼出時（独自周期なし） |
| [isrEncL()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:714) | 到達 | 左A相CHANGE割り込み |
| [isrEncR()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:733) | 到達 | 右A相CHANGE割り込み |
| [wheelCircumference()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:755) | 到達 | 呼出時（独自周期なし） |
| [mmToPulse()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:759) | 到達 | 呼出時（独自周期なし） |
| [pulseToMm()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:763) | 未到達 | 呼出時（独自周期なし） |
| [resetEncoder()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:767) | 到達 | 呼出時（独自周期なし） |
| [getEncL()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:780) | 未到達 | 呼出時（独自周期なし） |
| [getEncR()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:787) | 未到達 | 呼出時（独自周期なし） |
| [getTotalEncL()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:794) | 到達 | 呼出時（独自周期なし） |
| [getTotalEncR()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:801) | 到達 | 呼出時（独自周期なし） |
| [getAvgPulse()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:808) | 未到達 | 呼出時（独自周期なし） |
| [getAvgMm()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:812) | 未到達 | 呼出時（独自周期なし） |
| [waitUntilMotorsStopped()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:817) | 到達 | 最短5ms確認、20ms継続、500ms上限 |
| [resetPID()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:853) | 未到達 | 呼出時（独自周期なし） |
| [driveLeftRaw()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:863) | 到達 | 呼出時（独自周期なし） |
| [driveRightRaw()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:878) | 到達 | 呼出時（独自周期なし） |
| [setMotor()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:892) | 未到達 | 呼出時（独自周期なし） |
| [stopMotor()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:896) | 到達 | 呼出時（独自周期なし） |
| [stopMotors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:905) | 到達 | 呼出時（独自周期なし） |
| [shortBrakeMotors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:912) | 到達 | 呼出時（独自周期なし） |
| [shortBrakeCurrentMotors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:927) | 未到達 | 呼出時（独自周期なし） |
| [line()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:935) | 未到達 | 2ms待機＋処理 |
| [edgeLine()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:1022) | 未到達 | 2ms待機＋処理 |
| [tlineInternal()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:1308) | 未到達 | 2ms待機＋処理 |
| [tline()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:1563) | 未到達 | tlineInternalに委譲 |
| [tlineL()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:1581) | 未到達 | tlineInternalに委譲 |
| [tlineR()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:1599) | 未到達 | tlineInternalに委譲 |
| [motorPower()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:1616) | 到達 | 呼出時（独自周期なし） |
| [ex_tline()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:1660) | 未到達 | 2ms待機＋処理、後段tank |
| [brakeCurrentMotors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:1901) | 未到達 | 5ms待機、最大100ms |
| [brakeMotors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:1942) | 未到達 | 呼出時（独自周期なし） |
| [brakeSpinMotors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:1950) | 未到達 | 呼出時（独自周期なし） |
| [startPressed()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:1967) | 到達 | 呼出時（独自周期なし） |
| [stopPressed()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:1971) | 到達 | 呼出時（独自周期なし） |
| [checkGyroAlive()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2040) | 到達 | 呼出時（独自周期なし） |
| [updateGyroModeLed()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2159) | 到達 | 診断最短250ms、取得直後は省略 |
| [waitStartButtonWithGyroCheck()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2241) | 到達 | 10ms待機、診断既定500ms |
| [calcTrapezoidSpeed()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2320) | 未到達 | 呼出時（独自周期なし） |
| [lineControlStep()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2362) | 未到達 | 呼出時（独自周期なし） |
| [calcTurnTrapezoidPower()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2497) | 未到達 | 呼出時（独自周期なし） |
| [calcPivotTrapezoidPower()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2527) | 未到達 | 呼出時（独自周期なし） |
| [writeTankOnPower()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2558) | 到達 | 呼出時（独自周期なし） |
| [tankOff()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2564) | 到達 | 呼出時（独自周期なし） |
| [tankOn()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2569) | 未到達 | 呼出時（独自周期なし） |
| [tankSyncOn()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2582) | 未到達 | 呼出時（独自周期なし） |
| [tankSyncOff()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2605) | 未到達 | 呼出時（独自周期なし） |
| [tankOnEncoder()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2612) | 未到達 | 呼出時（独自周期なし） |
| [tankBrakeWheel()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2619) | 未到達 | 5ms待機、逆相最大100ms |
| [tankBrakeLeft()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2653) | 未到達 | tankBrakeWheelに委譲 |
| [tankBrakeRight()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2654) | 未到達 | tankBrakeWheelに委譲 |
| [tankStraightCorrection()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2657) | 到達 | 呼出時（独自周期なし） |
| [updateMotionTasks()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2668) | 到達 | 最短1ms・協調式 |
| [tank()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2703) | 到達 | 125µs目標 |
| [tankb()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:2795) | 未到達 | 3段階とも125µs目標 |
| [smoothStep01()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:3391) | 未到達 | 呼出時（独自周期なし） |
| [degToPulse()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:3399) | 未到達 | 呼出時（独自周期なし） |
| [pivotDegToPulse()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:3413) | 未到達 | 呼出時（独自周期なし） |
| [mpu9265WriteRegister()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:3428) | 到達 | 呼出時（独自周期なし） |
| [mpu9265ReadRegister()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:3442) | 到達 | 呼出時（独自周期なし） |
| [mpu9265ReadRegisters()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:3485) | 到達 | 呼出時（独自周期なし） |
| [mpu9265ReadRawGyroZ()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:3539) | 到達 | 呼出時（独自周期なし） |
| [tryRecoverGyroForNextMove()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:3580) | 到達 | 呼出時（独自周期なし） |
| [setupGyroMPU9265()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:3751) | 到達 | 呼出時（独自周期なし） |
| [calibrateGyroZ()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:4016) | 到達 | 10ms待機＋取得 |
| [updateYawFromGyro()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:4238) | 到達 | getYawDeg呼出時にI2C取得 |
| [gyroReady()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:4349) | 未到達 | 呼出時（独自周期なし） |
| [resetYawDeg()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:4362) | 到達 | 呼出時（独自周期なし） |
| [getYawDeg()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:4398) | 到達 | 呼出時に取得・積分 |
| [runTurnPD()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:4414) | 未到達 | 125µs目標 |
| [spin()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:4526) | 未到達 | runTurnPD:125µs目標 |
| [pivot()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:4530) | 未到達 | runTurnPD:125µs目標 |
| [test_encoder()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:4535) | 未到達 | 表示100ms、ループ周期なし |
| [kabe()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:4584) | 未到達 | 1ms待機＋処理、判定80ms |
| [motionSetup()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:4691) | 到達 | 呼出時（独自周期なし） |
| [checkline()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:4735) | 未到達 | 100ms待機＋表示 |
| [runArc()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:4779) | 未到達 | 125µs目標 |
| [arc()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:5562) | 未到達 | runArc:125µs目標 |
| [parallelShift()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:5566) | 未到達 | runArcに委譲 |
| [motorBrakeTest()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:5587) | 未到達 | 呼出時（独自周期なし） |
| [tankbSet()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:5634) | 未到達 | 周期・協調更新なし（要修正） |
| [robot_reset()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/motion_lib.cpp:5668) | 到達 | 呼出時（独自周期なし） |
| [resetMosaicRemaining()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:26) | 未到達 | シーケンス／呼び出し先に従う |
| [getMosaicRemaining()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:42) | 未到達 | シーケンス／呼び出し先に従う |
| [consumeMosaic()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:77) | 未到達 | シーケンス／呼び出し先に従う |
| [getMosaicDepth()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:167) | 未到達 | シーケンス／呼び出し先に従う |
| [moveBetweenMosaicColors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:215) | 未到達 | シーケンス／呼び出し先に従う |
| [pivotcheck()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:286) | 未到達 | シーケンス／呼び出し先に従う |
| [spincheck()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:296) | 未到達 | シーケンス／呼び出し先に従う |
| [test_function()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:307) | 未到達 | シーケンス／呼び出し先に従う |
| [bowl()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:310) | 未到達 | シーケンス／呼び出し先に従う |
| [put_mosaic_both()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:388) | 未到達 | シーケンス／呼び出し先に従う |
| [move_mosaic2_3()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:431) | 未到達 | シーケンス／呼び出し先に従う |
| [section_A()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:573) | 未到達 | シーケンス／呼び出し先に従う |
| [section_B()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:596) | 未到達 | シーケンス／呼び出し先に従う |
| [section_C()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:691) | 未到達 | シーケンス／呼び出し先に従う |
| [move_mosaic1()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:788) | 未到達 | シーケンス／呼び出し先に従う |
| [robot_start()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:949) | 未到達 | シーケンス／呼び出し先に従う |
| [catch_block()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:965) | 未到達 | シーケンス／呼び出し先に従う |
| [motikae()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:1004) | 未到達 | シーケンス／呼び出し先に従う |
| [leadoff()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:1026) | 未到達 | シーケンス／呼び出し先に従う |
| [avoidGo()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:1039) | 未到達 | シーケンス／呼び出し先に従う |
| [go_white()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:1057) | 未到達 | シーケンス／呼び出し先に従う |
| [swapOnField()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:1070) | 未到達 | シーケンス／呼び出し先に従う |
| [swapGrabbed()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:1091) | 未到達 | シーケンス／呼び出し先に従う |
| [cement()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:1118) | 未到達 | シーケンス／呼び出し先に従う |
| [oneput()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:1126) | 未到達 | シーケンス／呼び出し先に従う |
| [on_barrier()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:1146) | 未到達 | シーケンス／呼び出し先に従う |
| [color_EX()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:1156) | 未到達 | シーケンス／呼び出し先に従う |
| [color_EX_if()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:1179) | 未到達 | シーケンス／呼び出し先に従う |
| [frame_mosaic_catch()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:1186) | 未到達 | シーケンス／呼び出し先に従う |
| [moveTo()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/move_function.cpp:1202) | 未到達 | シーケンス／呼び出し先に従う |
| [normalizeColor()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:151) | 未到達 | 呼出時（独自周期なし） |
| [stringToColorId()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:206) | 未到達 | 呼出時（独自周期なし） |
| [makeGroupCandidates()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:230) | 未到達 | 呼出時（独自周期なし） |
| [groupNeedsColorFallback()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:254) | 未到達 | 呼出時（独自周期なし） |
| [buildGroupMatchScoreTable()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:296) | 未到達 | 呼出時（独自周期なし） |
| [getGroupMatchScore()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:336) | 未到達 | 呼出時（独自周期なし） |
| [searchGroupColors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:360) | 未到達 | 呼出時（独自周期なし） |
| [updateColorRemaining()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:689) | 未到達 | 呼出時（独自周期なし） |
| [colorIdToString()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:708) | 未到達 | 呼出時（独自周期なし） |
| [splitCsv()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:717) | 未到達 | 呼出時（独自周期なし） |
| [blinkColorReadErrorLed()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:741) | 未到達 | 呼出時（独自周期なし） |
| [piColorSetup()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:752) | 到達 | 呼出時（独自周期なし） |
| [clearPiSerialBuffer()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:800) | 未到達 | 呼出時（独自周期なし） |
| [readLineFromPi()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:809) | 未到達 | 空受信時1ms待機 |
| [parseRoiColorLine()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:856) | 未到達 | 呼出時（独自周期なし） |
| [parseGroupColorLine()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:899) | 未到達 | 呼出時（独自周期なし） |
| [parseRoiRgbLine()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:942) | 未到達 | 呼出時（独自周期なし） |
| [parseSetRoiColorLine()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:988) | 未到達 | 呼出時（独自周期なし） |
| [parseSetRoiRgbLine()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1021) | 未到達 | 呼出時（独自周期なし） |
| [printAllRoiColors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1058) | 未到達 | 呼出時（独自周期なし） |
| [printAllRoiRgb()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1069) | 未到達 | 呼出時（独自周期なし） |
| [printAllGroupColors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1084) | 未到達 | 呼出時（独自周期なし） |
| [checkGroupColors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1119) | 未到達 | 呼出時（独自周期なし） |
| [printAllRoiColors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1132) | 未到達 | 呼出時（独自周期なし） |
| [printAllRoiRgb()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1155) | 未到達 | 呼出時（独自周期なし） |
| [printColorSetStatus()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1182) | 未到達 | 呼出時（独自周期なし） |
| [sendSimpleCommandToPi()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1199) | 未到達 | 呼出時（独自周期なし） |
| [requestPiSnap()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1231) | 未到達 | 呼出時（独自周期なし） |
| [requestPiSnap()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1235) | 未到達 | 呼出時（独自周期なし） |
| [requestPiPicker()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1249) | 未到達 | 呼出時（独自周期なし） |
| [requestPiPicker()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1253) | 未到達 | 呼出時（独自周期なし） |
| [readRoi()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1264) | 未到達 | 呼出時（独自周期なし） |
| [requestColorRead()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1337) | 未到達 | 要求時のみ、READ待ち1000ms |
| [requestColorRead()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1504) | 未到達 | 要求時のみ、READ待ち1000ms |
| [setting_variable()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1529) | 未到達 | 呼出時（独自周期なし） |
| [setManualGroupColors()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1641) | 未到達 | 呼出時（独自周期なし） |
| [readColorAndSetVariable()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1683) | 未到達 | 呼出時（独自周期なし） |
| [getRoiColor()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1722) | 未到達 | 呼出時（独自周期なし） |
| [getRoiR()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1730) | 未到達 | 呼出時（独自周期なし） |
| [getRoiG()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1738) | 未到達 | 呼出時（独自周期なし） |
| [getRoiB()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1746) | 未到達 | 呼出時（独自周期なし） |
| [getGroupColor()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1754) | 未到達 | 呼出時（独自周期なし） |
| [getRoiColor()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1765) | 未到達 | 呼出時（独自周期なし） |
| [getRoiR()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1781) | 未到達 | 呼出時（独自周期なし） |
| [getRoiG()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1797) | 未到達 | 呼出時（独自周期なし） |
| [getRoiB()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1813) | 未到達 | 呼出時（独自周期なし） |
| [getRoiCount()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1829) | 未到達 | 呼出時（独自周期なし） |
| [getColorSetName()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1837) | 未到達 | 呼出時（独自周期なし） |
| [hasValidColorRead()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1848) | 未到達 | 呼出時（独自周期なし） |
| [hasValidColorRead()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/pi_color_lib.cpp:1852) | 未到達 | 呼出時（独自周期なし） |
| [servoSetup()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:105) | 到達 | 呼出時（独自周期なし） |
| [clampServoAngle()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:114) | 到達 | 呼出時（独自周期なし） |
| [servoAngleToPulseUs()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:120) | 到達 | 呼出時（独自周期なし） |
| [getServoDirByPin()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:131) | 到達 | 呼出時（独自周期なし） |
| [applyServoDirByPin()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:137) | 到達 | 呼出時（独自周期なし） |
| [isReservedPin()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:142) | 到達 | 呼出時（独自周期なし） |
| [findServoSlotByPin()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:164) | 到達 | 呼出時（独自周期なし） |
| [findFreeServoSlot()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:171) | 到達 | 呼出時（独自周期なし） |
| [getOrAttachServoSlotWithInitialAngle()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:178) | 到達 | 呼出時（独自周期なし） |
| [resetServoToAngleByPin()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:194) | 到達 | 呼出時（独自周期なし） |
| [resetGeekServosToStartAngle()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:217) | 到達 | 呼出時（独自周期なし） |
| [startServoMoveByPin()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:224) | 到達 | 呼出時（独自周期なし） |
| [writeBothServos()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:248) | 未到達 | 呼出時（独自周期なし） |
| [updateServoTasks()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:253) | 到達 | 最短2ms・協調式 |
| [cancelServoTasks()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:274) | 到達 | 呼出時（独自周期なし） |
| [servoDelay()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:279) | 到達 | 内部1ms待機＋協調更新 |
| [servoTasksBusy()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:288) | 未到達 | 呼出時（独自周期なし） |
| [waitServoTasks()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:295) | 未到達 | 2ms待機＋処理 |
| [printServoSettings()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:308) | 到達 | 呼出時（独自周期なし） |
| [catchArm()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:359) | 未到達 | 呼出時（独自周期なし） |
| [liftArm()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:402) | 未到達 | 呼出時（独自周期なし） |
| [backArm()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:452) | 未到達 | 呼出時（独自周期なし） |
| [circleArm()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:487) | 未到達 | 呼出時（独自周期なし） |
| [front()](C:/Users/progl/OneDrive/Desktop/WRO2026Senior/main/src/servo_lib.cpp:517) | 未到達 | 呼出時（独自周期なし） |

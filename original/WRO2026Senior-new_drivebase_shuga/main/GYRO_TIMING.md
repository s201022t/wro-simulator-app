# ジャイロと走行制御の高速化

- ジャイロ設定: `CONFIG=0`、`SMPLRT_DIV=0`、`GYRO_CONFIG=0x10`。出力8 kHz、±1000 dps。
- I2C: Wire1、GP18/GP19、400 kHz。Z軸2バイトだけを連続読み取りする。
- `tank`、`tankb`の3段階、`spin`、`pivot`、`arc`: 処理時間を含めて125 µsを目標とする。処理が超過したら追加待ちなしで次のループへ進む。
- ライントレースの周期は従来どおり。ジャイロ取得は走行ループからの呼び出し時に行い、停止中の連続収集やFIFO記録は行わない。

8 kHzはセンサー出力設定・制御目標であり、実効取得周波数の保証ではない。I2Cのアドレス送信・レジスタ指定・2バイト受信は計45クロック、400 kHzで112.5 µsを要し、さらにSTART/STOP、ライブラリ、演算、割り込みの時間が加わる。現構成では125 µsを超過する可能性が高く、その場合は処理時間が実効周期になる。全センサーサンプルの取得は保証しない。

`main/include/motion_lib.h` の `MOTION_CONTROL_PERIOD_US` を250にすると目標4 kHz、1000にすると目標1 kHzに変更できる。125未満ではセンサー更新周期より短くなるため、同じ値を再取得する可能性がある。

走行中の直近のYaw取得が成功している間はLED用の追加I2C診断を省略する。通信エラー時の再試行と停止処理は維持する。旋回のフィルター・角度積分は実時間dtを使用する。キャリブレーションの静止測定時間は維持する。

## spin / pivot のS字加減速

加速は開始からの実経過時間、減速は目標までの残角からPWM上限を計算する。周回数を基準にしないため、125 µsと5 msで同じ経過時間・角度なら同じ上限になる。左右同期補正・モーターゲイン後もこの上限を適用する。

固定の最低PWM 200を廃止し、spinは既存の`GYRO_TURN_MIN_POWER=100`、pivotは引数`minPower`（既定110）を使用する。始動時は静止摩擦対策としてこの下限から出力する。0からの連続立ち上げではない。

`spin(360, 255)`の上限は100から255まで約0.388秒でS字上昇し、残角が目標の33%に入ると100へS字下降する。pivotの減速区間は40%。短い旋回では加速中でも減速上限を優先する。角度PDによる早い減速・ブレーキ、目標通過・STOP・通信異常時の停止は優先するため、実出力そのものが常に理想的なS字になる保証はない。PWM指令の整形であり、角速度のS字軌道追従ではない。

実機では最低PWMで始動できるか、終端で振動しないか、停止角度が合うかを確認する。下限が不足する場合は`GYRO_TURN_MIN_POWER`またはpivotの`minPower`を機体に合わせて調整する。

停止・減速の進行角はリセット位置からの`abs(yaw)`を使う。初動0.15°の符号を固定する方式は、逆向きの揺れで符号を誤ると目標停止できなくなるため廃止した。D項の角速度も`abs(yaw)`の増減方向に合わせる。移動量の絶対値を毎回加算する方式ではないので、揺れの往復を旋回達成量として蓄積しない。Yawは折り返さず積分しており、360°以上も判定できる。回転方向自体はモーター指令で決まり、ジャイロの極性には依存しない。左右とも到達時にブレーキをかけ、既存のタイムアウトも維持する。

## 実機での計測

競技シーケンスの対象動作の前後で以下を実行する。Serial出力は必ず動作終了後に行う。

```cpp
resetMotionTimingStats();
spin(45, 255);  // 計測する既存の動作
const MotionTimingStats timing = getMotionTimingStats();
if (timing.gyroIntervals && timing.totalGyroIntervalUs) {
  Serial.print("Actual gyro read Hz: ");
  Serial.println(1000000.0 * timing.gyroIntervals / timing.totalGyroIntervalUs);
}
if (timing.controlCycles && timing.totalControlUs) {
  Serial.print("Timed control body Hz: ");
  Serial.println(1000000.0 * timing.controlCycles / timing.totalControlUs);
}
Serial.print("Max gyro interval us: ");
Serial.println(timing.maxGyroIntervalUs);
Serial.print("Max control body us: ");
Serial.println(timing.maxControlUs);
Serial.print("Control overruns: ");
Serial.println(timing.controlOverruns);
```

取得間隔はYaw用の成功したI2C読み取りの間隔。100 msを超える空白とYawリセットをまたぐ間隔は除外する。制御統計はループ先頭から周期待ち終了までを測定し、集計処理・ループ遷移・break/returnで終了した最終周回は含まない。厳密な実効周期は取得間隔と併せて評価する。同じコアからAPIを呼ぶこと。

DLPF_CFGを3から0に変えると帯域が広がり、振動ノイズが増える。高速化後の角度精度・出力の振れ・停止位置は実機で確認し、必要ならゲインや周期を調整する。

仕様参照: [TDK MPU-9250 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf)、[TDK MPU-9250 Product Specification](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)。コード内のMPU9265という名称だけで実チップは確定できないため、起動ログのWHO_AM_Iも確認する。

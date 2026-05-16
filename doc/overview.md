# Orion_F303_BLDC 改善方針

## 目的
- 可読性と保守性を上げる。
- リアルタイム制御性能を維持する。
- 挙動を変えないリファクタリングを優先する。

## 全体構成（現在）
- `Core/Src/main.c`: 制御ループ本体、周期同期、共通コンテキスト定義。
- `Core/Src/control_mode.c`: 実行モード判定とモードディスパッチ。
- `Core/Src/calibration.c`: エンコーダ校正・モータ校正。
- `Core/Src/protect.c`: 保護処理と異常停止。
- `Core/Src/comms.c`: CAN/UART 受信、テレメトリ送信。
- `Core/Src/startup_sequence.c`: 起動シーケンスと自己チェック。
- `Core/Src/diagnostics.c`: 周期ログ出力。
- `Core/Inc/app_context.h`: 共有状態（グローバル）の参照定義。

## 1ms制御ループの処理順
`main.c` の無限ループでは、基本的に以下の順で実行する。
1. `receiveUserSerialCommand()`（UART操作受付）
2. `updateMotorSpeedEstimate()`（速度推定）
3. `sendCanData()`（2msベースの送信を内部カウンタで間引き）
4. `runControlMode()`（モードに応じた制御実行）
5. `protect()`（保護判定）
6. `waitForNextMainCycle()`（次周期待ち）

## runControlMode と状態管理
`runControlMode()` は `control_mode.c` に実装し、次の2段で動作する。
- `getControlMode()`:
  - `fault_mode` フラグ
  - 起動中フラグ `sys.is_starting_mode`
  - 校正カウンタ `enc_calib_cnt` / `motor_calib_cnt`
  - フリーウィール状態 `sys.free_wheel_cnt`
  を参照してモードを決定する。
- モードに応じたディスパッチ:
  - `CONTROL_MODE_RUN` / `CONTROL_MODE_FREEWHEEL` / `CONTROL_MODE_STARTUP` / `CONTROL_MODE_FAULT` -> `runMode()`
  - `CONTROL_MODE_ENCODER_CALIB` -> `encoderCalibrationMode()`
  - `CONTROL_MODE_MOTOR_CALIB` -> `motorCalibrationMode()`

定義済みモード:
- `CONTROL_MODE_STARTUP`
- `CONTROL_MODE_RUN`
- `CONTROL_MODE_ENCODER_CALIB`
- `CONTROL_MODE_MOTOR_CALIB`
- `CONTROL_MODE_FREEWHEEL`
- `CONTROL_MODE_FAULT`

## キャリブレーション段階
`calib_process.motor_calib_mode` は `motor_calib_stage_t` で管理する。
- `MOTOR_CALIB_STAGE_INIT`
- `MOTOR_CALIB_STAGE_LOW_CW`
- `MOTOR_CALIB_STAGE_LOW_CCW`
- `MOTOR_CALIB_STAGE_HIGH_CW`
- `MOTOR_CALIB_STAGE_HIGH_CCW`

従来の数値ステップを enum に置き換えて、状態遷移の意図を明確化している。

## キャリブレーション手順
UART `c`、CAN `0x310`、起動時SW4でキャリブレーションを開始する。校正中はCAN速度指令を無視し、エンコーダ校正が完了してから速度係数校正へ自動で進む。

ゼロ電気角校正は `dev/simple_foc` ブランチで実機精度が良かった多点固定角方式を使う。
- 固定電気角は `6点/電気周期 * 12極対 = 72点`。
- 正順と逆順の2パスを測定する。
- 各点は250ms整定、100msサンプル。
- 各点で `fixed_angle - encoder_electrical_angle` を計算し、sin/cosの円周平均でゼロ角を求める。
- 2パス目では正順とのヒステリシスを計算し、8degを超える点はゼロ角平均から除外する。
- 完了時に `enc_offset[].zero_calib` とFlashのエンコーダ校正値を更新する。

校正ログは1kHz側のステージ完了時に出す。代表ログは次の形式である。

```text
enc cal start points 72 pass 2 settle 250 sample 100 voltage_mV +2000
cal enc p0 i00 raw 12345/23456 zero_mdeg +5796/+3067
cal enc p1 i00 raw 12340/23460 zero_mdeg +5797/+3068 hyst +1/+1 used 03
enc cal M0 zero_mrad +5796 pairs used/rej 72/0 hyst_mdeg avg/max +1/+4 samples 14400
enc cal done zero +5796/+3067 mrad
```

速度係数校正は従来どおり `+3V`, `-3V`, `+5V`, `-5V` の順にM0/M1同時に測定し、5V正転側の `rps_per_v` をFlashへ保存する。ゼロ電気角の精度を優先するため、速度係数校正の最後にCW/CCW速度差からエンコーダオフセットを再補正する処理は行わない。

## 設計ルール
- ISR と 1kHz ループの fast path では分岐/処理を増やしすぎない。
- ロジック変更を伴う最適化は、必ず計測値（周期余裕・CPU負荷）で確認する。
- 共有状態は `app_context.h` 経由で参照し、`extern` の散在を防ぐ。
- モードや校正段階は `enum` で明示する。

## 現行の出力制御経路
通常走行時の出力制御は、現時点では次の順に処理する。

```text
cmd.speed
  -> speedToOutputVoltage()
  -> cmd.out_v
  -> setFinalOutputVoltage()
  -> cmd.out_v_final / enc_offset.final
  -> as5047p.output_radian + enc_offset.final
  -> setOutputRadianMotor()
  -> PWM CCR
```

`motor.c` は速度指令から出力電圧とエンコーダ電気角オフセットを作る。`tim.c` は電気角と電圧から三相PWMのCCR値を作る。今回の整理では制御式は変更せず、legacy仕様に名前を付けて分割した。

現行仕様では、正逆転の意味はPWM振幅の符号ではなく、`getLegacyTorqueOffsetRadian()` が返す電気角オフセットで表現している。`setOutputRadianMotor()` は負の出力電圧を絶対値に変換してからPWM振幅へ使う。また、出力電圧が `output_voltage_limit` を超えた場合は飽和ではなく0Vに落とす。

左右回転差の切り分けとして、負方向トルクのlegacy電気角オフセットを正方向から厳密に `pi rad` 離す実験を行ったが、速度係数校正時の正逆回転差が大きくなり、校正エラーになることを確認した。`ROTATION_OFFSET_RADIAN = 2.00 rad` は理想q軸とは一致しないが、現行のPWM位相、エンコーダ符号、モータ配線、または速度推定符号を含むlegacy経路全体に対する実験的補償として効いている可能性が高い。そのため現時点では `2 * pi - ROTATION_OFFSET_RADIAN` の負方向オフセットを維持する。

周期ログでは `TrqOff` として、M0/M1それぞれで選択されたlegacy電気角オフセットを出力する。左右回転の比較では、`cmd.speed`, `motor_real.rps`, `cmd.out_v`, `cmd.out_v_final`, `pid.eff_voltage`, `pid.output_voltage_limitting`, `TrqOff`, `enc_offset.final` を合わせて見る。

## FOC/SinePWM診断経路
`dev/simple_foc` ブランチの実装を参考に、SimpleFOC相当の電圧FOC計算をCで追加した。通常RUNとキャリブレーション出力はまだlegacy経路のままで、今回追加したFOC経路は比較・診断用である。

- `Core/Src/foc_math.c`: 角度正規化、電気角計算、逆Park/Clarke、相電圧からPWM CCRへの変換。
- `Core/Src/foc_driver_hal.c`: FOC計算結果をTIM1/TIM8 CCRへ反映するHALブリッジ。
- `runStartupSequence()` で `focMathInit()` を呼ぶ。ただし現ブランチではRAM節約のためFOC専用sinテーブルは持たず、診断用として `sinf()` / `cosf()` を直接使う。

UART `v` でFOC計算チェックを実行する。これはPWM出力を変更せず、FOC mathセルフテスト、現在のエンコーダraw、legacy電気角、rawから直接計算したFOC電気角、`Uq=+/-2V` 相当のCCR候補をログ出力する。実機を回さず、legacy経路とFOC経路の符号・相順を比較するための前段診断である。

`v` 診断はSPI更新を行わない。TIM割り込み側が更新済みの `as5047p[]` 値を、短時間だけ割り込み禁止してローカルへコピーし、そのスナップショットから計算する。これにより、回転中にUART診断を実行してもAS5047PのSPIアクセス所有者をTIM割り込み側に一本化する。ログには `raw+` と `raw-` として、sensor direction `+1` / `-1` の両方で計算したFOC電気角を出す。

回転中に1kHzメインループを止めると、`calcMotorSpeed()` が1ms前提でraw差分をRPS換算するため、正常回転でも速度異常になる。`v` 診断では `HAL_Delay()` を使わず、FOC mathセルフテストは起動時に1回だけ実行して結果を保持する。

UART `V` でFOC診断モードを開始/停止する。開始時は速度指令を0にし、PWMを有効化する。既存の `w` / `s` で速度指令を増減すると、1kHz側で従来の速度-電圧変換を行った後、FOC診断用に `+/-2.0V` へ制限した `Uq` をTIM割り込み側でSinePWM出力する。通常RUN、CAN速度指令、キャリブレーション出力はまだlegacy経路のままである。

FOC診断モードでは、TIM割り込み内で `as5047p.output_radian + enc_offset.zero_calib + manual_offset` を電気角として使い、`Uq` には `-1.0` のトルク方向係数を掛ける。これは `dev/simple_foc` の `APP_SENSOR_TORQUE_DIRECTION = -1.0f` に合わせた初期値である。停止時は60秒フリーウィールへ戻す。

## 診断と安全チェック
UART `i` で非回転I/Oチェックを実行する。実行時は速度指令と出力電圧を0にし、PWMをフリーウィールへ落としてから、スイッチ、ADC raw/換算値、AS5047P raw/診断レジスタ、PWM CCR/CCER/BDTR、CAN/Flash状態を出力する。実機で回転指令を入れる前のベンチ確認に使う。

UART出力関数 `p()` は `vsnprintf()` でバッファ境界を確認する。キャリブレーションやI/Oチェックの連続ログでUART DMA用バッファを壊さないことを優先する。

AS5047Pは通常角度更新とは別に `updateAS5047PDiagnostics()` で診断レジスタを読む。診断読み出し中は短時間だけ割り込みを止め、PWM ISR側のSPIアクセスと競合しないようにする。通常更新では最終フレーム `last_frame` とエラーフレーム数 `spi_error_count` を保持する。

電流センス安全確認の `isNotZeroCurrent()` はM0/M1の両方を見る。温度はADC rawから直接返さず、有効範囲内の値だけを一次ローパスに通したフィルタ値を `getTempFET()` / `getTempMotor()` で返す。

## 改善項目（完了）
1. ファイル分割（通信・起動・診断）
- `comms.c`, `startup_sequence.c`, `diagnostics.c` を追加。
- `main.c` から該当ロジックを移設。

2. 状態管理の明示化
- `control_mode_t` を拡張（`startup/run/enc_calib/motor_calib/freewheel/fault`）。
- Fault 状態を `setFaultMode()` で明示的に遷移。

3. キャリブレーション段階の明示化
- `motor_calib_stage_t` を導入。
- `motor_calib_mode` のマジックナンバーを段階 enum に置換。

4. 共有依存の整理
- `Core/Inc/app_context.h` へ共有変数宣言を集約。
- `calibration/protect/control_mode/comms/startup/diagnostics` で利用。

5. 文字化けコメント修正
- `main.c`, `calibration.c`, `doc/overview.md` の文字化けを解消。

## ビルド手順（CLI）
### 前提
- STM32CubeIDE 1.17.0 がインストール済み。
- GNU Tools for STM32 が利用可能。

### Debug
```powershell
cd Debug
& "C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin\make.exe" -B -j4 all
```

### Release
```powershell
cd Release
& "C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin\make.exe" -B -j4 all
```

## 書き込み手順（CLI）
### 接続確認
```powershell
& "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -l stlink
& "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD mode=UR -rst
```

### Debugを書き込み
```powershell
& "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD mode=UR -w "Debug\Orion_F303_BLDC.elf" -v -rst
```

### Releaseを書き込み
```powershell
& "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD mode=UR -w "Release\Orion_F303_BLDC.elf" -v -rst
```

### 補足
- `mode=UR` は Under Reset 接続。起動直後にSWDが不安定な場合に有効。
- `-v` は書き込み後ベリファイ。
- `-rst` は書き込み後にリセットを実行。

## スクリプト
`Script` 配下に、ビルドと書き込みをまとめた PowerShell スクリプトを配置している。

### ビルド
```powershell
powershell -ExecutionPolicy Bypass -File .\Script\build.ps1 -Configuration Debug
powershell -ExecutionPolicy Bypass -File .\Script\build.ps1 -Configuration Release -Rebuild
```

### 接続確認
```powershell
powershell -ExecutionPolicy Bypass -File .\Script\flash.ps1 -List
powershell -ExecutionPolicy Bypass -File .\Script\flash.ps1 -ConnectOnly
```

### 書き込み
```powershell
powershell -ExecutionPolicy Bypass -File .\Script\flash.ps1 -Configuration Debug
powershell -ExecutionPolicy Bypass -File .\Script\flash.ps1 -Configuration Release
```

### ビルドしてから書き込み
```powershell
powershell -ExecutionPolicy Bypass -File .\Script\build_and_flash.ps1 -Configuration Debug
powershell -ExecutionPolicy Bypass -File .\Script\build_and_flash.ps1 -Configuration Release -Rebuild
```

### UARTログ受信
```powershell
powershell -ExecutionPolicy Bypass -File .\Script\monitor_uart.ps1
powershell -ExecutionPolicy Bypass -File .\Script\monitor_uart.ps1 -Port COM60 -BaudRate 2000000 -DurationSec 5
powershell -ExecutionPolicy Bypass -File .\Script\monitor_uart.ps1 -Port COM60 -BaudRate 2000000 -MaxLines 100 -LogPath .\uart_log.txt
```

## 出力成果物
- `Orion_F303_BLDC.elf`
- `Orion_F303_BLDC.map`
- `Orion_F303_BLDC.list`

## 注意点
- リンカの RWX 警告は現行リンカ設定由来で、今回のリファクタリング由来ではない。
- 実機評価では、起動シーケンス・校正シーケンス・保護動作・通信周期を必ず回帰確認する。

## 参考ドキュメント
- ハードウェア仕様（コード推定）: `doc/hardware_spec.md`

# Orion_F303_BLDC 改善方針

## 現行方針（2026-05-02 アプリ層刷新）
古いアプリケーション実装はビルド対象から外し、CubeMX生成コードとHAL周辺操作だけを流用する構成へ切り替えた。
制御状態、1kHz処理、TIM割り込み、UART/CANコマンド、保護処理は `bldc_app.c` に集約する。

現行のビルド対象:
- `Core/Src/main.c`: MCU初期化と `bldc_app` への委譲。
- `Core/Src/bldc_app.c`: 新アプリ層。状態管理、起動、1kHz周期処理、TIM割り込み処理、UART/CAN受信、保護、テレメトリを担当。
- `Core/Src/foc_math.c`: SimpleFOC風の角度正規化、電気角、SinePWM相電圧、CCR変換。
- `Core/Src/foc_driver_hal.c`: `foc_math` の結果をTIM1/TIM8 CCRへ反映する薄いHALブリッジ。
- `Core/Src/io_check.c`: モータを回さない実機I/Oチェック。
- `adc.c`, `spi.c`, `tim.c`, `can.c`, `gpio.c`, `usart.c`, `flash.c`: CubeMX/HAL周辺操作と既存の低レベル補助関数。

ビルド対象から外した旧アプリ層:
- `calibration.c`
- `comms.c`
- `control_mode.c`
- `diagnostics.c`
- `motor.c`
- `protect.c`
- `startup_sequence.c`

旧ソースファイルと旧ヘッダは削除済みで、`Debug/Core/Src/subdir.mk`, `Release/Core/Src/subdir.mk`, `objects.list` からも除外済み。
刷新後の通常経路では `app_context.h` の共有グローバル状態を使わない。

## 新アプリ層の状態
`bldc_app.c` は以下のモードで動作する。

- `BLDC_APP_MODE_BOOT`: 起動初期化中。
- `BLDC_APP_MODE_FREEWHEEL`: PWMチャネルを無効化し、相出力をフリーウィールにする。
- `BLDC_APP_MODE_READY`: 入出力は初期化済みで、出力指令待ち。
- `BLDC_APP_MODE_RUN`: PWM出力を有効化し、SimpleFOC風SinePWMで電圧指令を出す。
- `BLDC_APP_MODE_FAULT`: 保護検出後。PWMはフリーウィール固定。

起動直後は安全側として60秒のフリーウィールに入る。
UART `n` でRUNへ入れるが、目標速度は0のままなので即時回転はしない。
CANまたはUARTで速度指令が入ると、`target_rps * voltage_per_rps` の開ループ電圧指令をSinePWMへ渡す。

## 新1msループ
`bldcAppTick1kHz()` の処理順:

1. UART受信コマンド処理。
2. CANから要求されたI/Oチェック処理。
3. エンコーダ差分から速度推定。
4. フリーウィールタイマ更新。
5. 速度指令から `voltage_q` を計算。
6. 電圧、電流、温度、エンコーダ異常の保護判定。
7. ゼロ出力が続く場合のPWMフリーウィール化。
8. CANテレメトリ送信。
9. UART診断ログ出力。
10. TIM1割り込み20回分を待って1ms周期に同期。

TIM1更新割り込みではM0/M1を交互に処理し、ADC注入変換値の取得、AS5047P更新、必要時のSinePWM CCR更新だけを行う。
割り込み内でUART出力やCAN送信は行わない。

## UART操作（刷新後）
```text
i  非回転I/Oチェック。PWMを60秒フリーウィールへ落としてADC/SPI/GPIO/CAN/Flash/PWM状態を表示。
m  FOC計算セルフテストのみ実行。PWM出力は変更しない。
n  RUN許可。目標速度は0のまま、PWM出力だけ有効化。
x  60秒フリーウィール。
0  60秒フリーウィール。
w  両モータの目標速度を +0.5 rps。
s  両モータの目標速度を -0.5 rps。
space  目標速度0、60秒フリーウィール。
enter  診断ログページ切り替え。
```

## CAN操作（刷新後）
- `0x100` / `0x102`: M0速度指令。
- `0x101` / `0x103`: M1速度指令。
- `0x110`: `data[0] == 3` のときフリーウィール要求。
- `0x320`: 次回1kHzループで非回転I/Oチェック。
- `0x010`: `data[0] == 0 && data[1] == 0` のときリセット。

CANフィルタは既存の `CAN_Filter_Init()` を流用している。

## 安全上の注意
- 起動直後はフリーウィール固定で、意図せず回転しない。
- `n` だけでは目標速度が0なので回転しない。
- `w/s` またはCAN速度指令で初めて電圧指令が非ゼロになる。
- 現状は電流FOCではなく、速度指令から電圧を作る開ループ寄りのSinePWMである。
- 実回転前に必ずUART `i` でADC、エンコーダ、PWM CCER、Flash校正値を確認する。

## 旧設計メモ
以下は刷新前の設計メモであり、現行ビルド対象ではない旧アプリ層の記録として残す。

## 目的
- 可読性と保守性を上げる。
- リアルタイム制御性能を維持する。
- 挙動を変えないリファクタリングを優先する。

## 全体構成（旧実装）
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

## 設計ルール
- ISR と 1kHz ループの fast path では分岐/処理を増やしすぎない。
- ロジック変更を伴う最適化は、必ず計測値（周期余裕・CPU負荷）で確認する。
- 共有状態は `app_context.h` 経由で参照し、`extern` の散在を防ぐ。
- モードや校正段階は `enum` で明示する。

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

## SimpleFOC流用方針
CubeMX/HAL環境は維持し、SimpleFOCライブラリ本体はリンクしない。
C++/Arduino依存を避けるため、SimpleFOCの考え方のうち以下だけをC実装として段階的に取り込む。

- `setPhaseVoltage()` 相当の電圧FOC計算
- SinePWM の Inverse Park + Clarke 変換
- 角度正規化と電気角変換
- HAL TIM1/TIM8 へCCRを書き込む薄いドライバ層

追加済みファイル:
- `Core/Inc/foc_math.h`, `Core/Src/foc_math.c`
  - SimpleFOC風の角度正規化、電気角変換、SinePWM相電圧生成、CCR変換、オフラインセルフテスト。
- `Core/Inc/foc_driver_hal.h`, `Core/Src/foc_driver_hal.c`
  - TIM1/TIM8のCCRへSimpleFOC風の相電圧計算結果を書き込むHALブリッジ。
- `Core/Inc/io_check.h`, `Core/Src/io_check.c`
  - 実機でモータを回す前の非回転I/Oチェック。

現時点では通常制御パスの `setOutputRadianMotor()` は差し替えていない。
SimpleFOC風SinePWMは、UARTコマンドでの計算確認と将来の段階移行用として追加している。

参考:
- SimpleFOC FOC workflow: https://docs.simplefoc.com/foc_implementation
- SimpleFOC source structure: https://docs.simplefoc.com/source_code

## 実機回転前I/Oチェック
モータを実際に回す前に、UARTまたはCANから非回転チェックを行う。

### UART
```text
i
```

実行内容:
- PWM CCRをセンター値へ戻す。
- PWM出力をフリーウィール状態へ落とし、60秒間またはUART `n` まで保持する。
- 速度指令と出力電圧指令を0にする。
- ADC値、換算電圧/電流/温度を表示する。
- AS5047Pのraw値、電気角、差分、起動時取得レジスタを表示する。
- スイッチ入力状態を表示する。
- TIM1/TIM8のCCR/CCER/BDTRを表示する。
- CAN受信数、CANエラー、Flash校正値を表示する。
- SimpleFOC風SinePWM計算のオフラインセルフテストを実行する。

### UART FOC計算チェックのみ
```text
m
```

SimpleFOC風SinePWMの計算だけを行い、代表CCR値を表示する。PWM出力は変更しない。

### CAN
標準ID `0x320` を受信すると、次回メインループ側で `i` と同じ非回転I/Oチェックを実行する。
CAN割り込み内では実行せず、要求フラグだけを立てる。

### フリーウィール解除
```text
n
```

校正状態と手動オフセットを解除し、`free_wheel_cnt` を0へ戻してPWM出力を再開する。
モータ回転前のチェックだけを行う場合は、`n` は送信しない。

## 実機確認の順序
1. 電源投入後、通常ログで起動シーケンスが完了することを確認する。
2. UART `i` を実行し、PWMがフリーウィール状態のまま、ADC/SPI/GPIO/CAN/Flash値を確認する。
3. UART `m` を実行し、FOC計算セルフテストが `OK` になることを確認する。
4. PWM出力を有効にする変更を入れる前に、TIM1/TIM8とM0/M1の対応を再確認する。
5. 最初にSimpleFOC風SinePWMを接続する場合は片モータ、低電圧、無負荷で確認する。

## 実装上の注意
- 現状の通常制御は従来の `setOutputRadianMotor()` のまま。
- SimpleFOC風SinePWMは `voltage_power_supply/2` を中心に相電圧を作るため、従来の `TIM_PWM_CENTER + sin * scale` 方式と振幅スケールが異なる。
- 本番経路へ接続する前に、同一入力で旧CCRと新CCRを比較する検証ステップを追加する。
- 電流FOCは今回の対象外。F303のADC注入変換、2モータ、保護監視と強く結合しているため、別フェーズで扱う。

## 追加修正
- `isNotZeroCurrent()` がM0を2回見ていたため、M0/M1の両方を見るよう修正。
- 起動時の `output_voltage_limit` 計算をFlash生値の直接割り算から、検証済みの `motor_param[].voltage_per_rps` ベースへ変更。
- UART改行コマンドの `print idx` 表示で不足していた引数を追加。


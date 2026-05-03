# Orion_F303_BLDC 改善方針

## 現行方針（2026-05-02 アプリ層刷新）
古いアプリケーション実装はビルド対象から外し、CubeMX生成コードとHAL周辺操作だけを流用する構成へ切り替えた。
制御状態、1kHz処理、TIM割り込み、UART/CANコマンド、保護処理は `bldc_app.c` に集約する。

## 2026-05-02 電圧モデル推定方式の更新
電流センスは電源ラインのローパス後の値のみで、相電流制御や電流ベースのモーターパラメーター推定には使わない。電流値は過電流保護と状態監視に限定する。

通常運転は引き続き `velocity_openloop` を標準とし、速度指令から電圧指令を作る。ただし低速域では起電力だけでなく摩擦、デッドゾーン、静止抵抗的な要素が支配的になるため、電圧モデルを次の一次式として扱う。

```text
Vq = sign(rps) * (voltage_offset + voltage_per_rps * abs(rps))
```

- `voltage_per_rps` は速度に比例して必要になる電圧成分。
- `voltage_offset` は低速域で支配的になる回転開始・維持のための固定電圧成分。
- Flash 保存では既存の `rps_per_v_cw` 領域に `1 / voltage_per_rps` を保存する。
- 旧 `rps_per_v_ccw` 用だった未使用領域は `voltage_offset` の保存先として再利用する。

UART `k` / `K` のモーターパラメーター推定は、目標速度を達成できる最低 Vq を探す方式から、Vq 掃引で実測回転数を観測する方式へ変更した。

推定手順:
1. M0、M1 の順に実行する。
2. 各モーターで SimpleFOC 風の固定電気角アライメントを行い、RAM 上の `zero_electric_angle` を更新する。
3. センサ角を使う電圧トルク診断モードに切り替え、`0.60 V` から `2.00 V` まで `0.20 V` 刻みで Vq を印加する。
4. 各 Vq で 600 ms 待ってから 900 ms の実測 rps 平均を取る。
5. `0.30 rps` 以上で回った点だけを使い、`Vq = offset + K * rps` を最小二乗で推定する。
6. 推定値には 10% の電圧マージンを加える。
7. `k` はRAM反映のみ、`K` は推定後にFlashへ保存する。

この推定はセンサ角電圧モードで実際に回転できることを前提にしている。十分な観測点が得られないモーターは推定失敗として、既存の `voltage_per_rps` と `voltage_offset` を保持する。

## 最終的な完成形
現時点の完成形は、SimpleFOCのC++/Arduino依存を持ち込まず、CubeMX/HAL環境のままSimpleFOC相当のSinePWMと `velocity_openloop` をC実装として取り込む構成とする。

- 通常RUN経路は `velocity_openloop` を標準とし、CAN/UARTの速度指令から回転磁界を生成する。
- 電圧指令は `target_rps * voltage_per_rps` で作り、Flashに保存済みの `rps_per_v_cw` が妥当な場合はその逆数を使う。
- 外部から受けた速度指令は `command_rps` として保持し、1kHz周期で `target_rps` を `0.01 rps/ms` 以下の変化量に制限して追従させる。
- 極対数は12、センサ角を使う診断/校正経路の符号は実機診断結果に合わせて `APP_SENSOR_DIRECTION = +1` とする。
- センサ角を使う電圧FOCは通常経路ではなく、zero angle、相順、センサ方向を詰めるための診断モードとして残す。
- センサ方向はSimpleFOC風の電気角スキャンで確認し、現ハードでは `APP_SENSOR_DIRECTION = +1`。open-loop速度指令の符号は別に `APP_OPEN_LOOP_DIRECTION = -1` として扱う。
- 起動直後は60秒フリーウィール、`n` だけでは目標速度0、`space`/`x`/`0` で即フリーウィールへ戻る。
- 電流FOCと速度PI閉ループは今回の完成範囲外。ADC注入変換、2モータ、保護監視、電流センス相順を別フェーズで確認してから扱う。
- 電流センスは電源ラインのローパスありの値だけを使う。制御には使わず、過電流保護と状態監視に限定する。

## モーターパラメーター推定
電流制御に必要な相電流は観測できないため、推定対象は電圧次元の速度フィードフォワード係数に限定する。

- 推定コマンドはUART `k` / `K`。
- `k` はRAM反映のみ、`K` は推定後にFlashの `rps_per_v_cw` へ保存する。
- 推定はM0、M1の順に実行する。
- 各モーターで固定電気角アライメントを行ってから、センサ角電圧モードで `0.60 V` から `2.00 V` までVqを掃引する。
- 各Vqで実測rpsを観測し、`Vq = offset + voltage_per_rps * rps` を最小二乗で推定する。
- 推定値には10%の電圧マージンを乗せる。
- Flashへ保存する値は既存形式に合わせて `rps_per_v = 1 / voltage_per_rps` とする。
- 推定中に `space`、Fault、電圧/温度/電流保護が入った場合は中断し、フリーウィールへ戻す。

## 実装・デバッグ手順
1. 非回転I/O確認: UART `i` でADC、AS5047P、PWM CCER/BDTR、Flash校正値、FOC計算を確認する。
2. ゼロ出力RUN確認: UART `n` でRUNへ入り、目標速度0、Faultなし、電源電圧正常を確認する。
3. 回転磁界確認: `o` を有効にして、M0は `q/a`、M1は `e/d` で片軸ずつ低速から確認する。
4. 極対数・符号確認: 目標 `+2.5 rps` に対して推定速度が `+2.5 rps` 付近になることを確認する。
5. 両軸確認: `w/s` で両軸同時に低速確認し、電流、温度、Faultを監視する。
6. 通常経路化: `velocity_openloop` を起動時標準にし、CAN速度指令でも同じ経路を通す。
7. 回帰確認: Debug/Releaseビルド、Debug書き込み、起動ログ、I/Oチェック、片軸、両軸、停止を確認する。
8. スルーレート制限確認: UARTログの `cmd` と `tgt` を見て、急な指令に対して `target_rps` が段階的に追従することを確認する。

## 実機確認結果（2026-05-02）
- Debug/Releaseビルド成功。既存の linker RWX 警告のみ残る。
- Debug書き込みとVerify成功。
- 起動後ログは `Mode 1 OL 1`、Fault `0x0000`、バッテリ約24V、ゲートドライバDCDC約8.3V。
- UART `i` はADC、AS5047P、PWM CCER/BDTR、Flash校正値、FOC計算セルフテストを正常に表示した。
- UART `m` のFOC計算セルフテストは `OK`。
- UART `n` のゼロ出力RUNでは `Mode 3 OL 1`、Faultなし、目標速度0。
- 標準RUN経路のまま、M0単独 `+2.5 rps` は推定 `+2.48〜+2.52 rps` で安定。
- 標準RUN経路のまま、M1単独 `+2.5 rps` は推定 `+2.48〜+2.52 rps` で安定。
- 標準RUN経路のまま、M0/M1同時 `+2.5 rps` は両軸とも推定 `+2.48〜+2.52 rps` で安定。
- 各回とも `space` でフリーウィールへ戻し、最終状態は `Mode 1 OL 1`、Fault `0x0000`。

## 実機確認結果（2026-05-02 追加）
- 12極対設定でSimpleFOC風センサアラインを再実行し、RAM上のzero angleを更新できた。
- センサ角を使う電圧診断は、センサ方向を `APP_SENSOR_DIRECTION = +1` に修正した後、Vq掃引で持続回転できることを確認した。ただし通常経路は引き続き速度open-loopを使う。
- 速度指令スルーレート制限を追加し、UARTログに `cmd` と `tgt` を分けて表示するようにした。
- M0単独 `cmd +2.5 rps` では `tgt` が段階的に上がり、最終的に推定速度 `+2.48〜+2.52 rps` で安定した。
- M1単独 `cmd +2.5 rps` でも同様に推定速度 `+2.48〜+2.52 rps` で安定した。
- M0/M1同時 `cmd +2.5 rps` でも両軸が推定速度 `+2.48〜+2.52 rps` で安定し、停止後Faultなし。

## 実機確認結果（2026-05-02 パラメーター推定）
- UART `k` でRAM上のモーターパラメーター推定を実行し、Faultなしで完了した。
- UART `K` で同じ推定を実行し、Flashへ保存した。
- 推定結果は M0 `voltage_offset = +0.18 V`, `voltage_per_rps = +0.190 V/rps`。
- 推定結果は M1 `voltage_offset = +0.20 V`, `voltage_per_rps = +0.186 V/rps`。
- Flash保存形式では M0 `rps_per_v_cw = +5.268 rps/V`、M1 `rps_per_v_cw = +5.367 rps/V`。
- 保存後の通常RUNでM0/M1同時 `cmd +2.5 rps` を確認し、`vq` はM0約 `+0.66 V`、M1約 `+0.67 V`、推定速度は `+2.49〜+2.52 rps` で安定した。
- 最終状態は `Mode 1 OL 1`、Fault `0x0000`。

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
q  M0のみ目標速度を +0.5 rps。
a  M0のみ目標速度を -0.5 rps。
e  M1のみ目標速度を +0.5 rps。
d  M1のみ目標速度を -0.5 rps。
y  M0のSimpleFOC風センサアラインをRAM上で実行し、zero_electric_angleを更新して60秒フリーウィールへ戻す。
h  M1のSimpleFOC風センサアラインをRAM上で実行し、zero_electric_angleを更新して60秒フリーウィールへ戻す。
o  通常RUN経路の velocity_openloop を有効化する。
c  センサ角を使う電圧FOC診断へ切り替える。通常運用では使わない。
v  SimpleFOC風センサ方向診断をM0/M1の順に実行する。電気角を正逆へ動かし、raw差分、推定方向、設定との一致、zero angleを表示する。
k  モーターパラメーター推定をRAM上で実行する。Flashへは保存しない。
K  モーターパラメーター推定を実行し、推定した `rps_per_v_cw` をFlashへ保存する。
space  目標速度0、60秒フリーウィール。
1  診断ログを状態ページへ固定。
2  診断ログをM0ページへ固定。
3  診断ログをM1ページへ固定。
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
- 極対数は旧 `spi.c` の `enc_raw % 5461` 実装から12極対として扱う。65536 / 5461 は約12で、1機械回転あたり12電気角回転の前提になっていた。
- `velocity_openloop` 診断では、既存の速度推定符号に合わせて正指令時に電気角を負方向へ進める。

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

## 2026-05-02 追記: SimpleFOC風Vq電圧制御の前提確認と実測結果
SimpleFOCの `voltage` トルク制御相当では、センサ角から電気角を作り、その角度に対して `Uq` を印加する。したがって、Vq掃引で回らない場合は電圧量より先に、センサ方向、zero electric angle、極対数、PWM相順の前提を疑う。

今回の確認では、SimpleFOC風の電気角スキャン診断 `v` により、現ハードのセンサ方向は M0/M1 とも `+1` と判定された。通常のopen-loop速度指令の符号とは分離し、設定は次の通りにした。

- センサ角を使う診断/校正経路: `APP_SENSOR_DIRECTION = +1`
- 通常の速度open-loop経路: `APP_OPEN_LOOP_DIRECTION = -1`

この修正後、UART `k` / `K` のVq掃引は正常に回転し、`K` でFlash保存した値は次の通り。

- M0: `voltage_offset = +0.18 V`, `voltage_per_rps = +0.190 V/rps`, `rps_per_v = +5.268 rps/V`
- M1: `voltage_offset = +0.20 V`, `voltage_per_rps = +0.186 V/rps`, `rps_per_v = +5.367 rps/V`

通常速度指令では、`Vq = sign(rps) * (voltage_offset + voltage_per_rps * abs(rps))` を使う。速度指令が入った場合は、60秒フリーウィール保持中でも `FREEWHEEL` から `RUN` へ戻す。これにより、校正直後やI/Oチェック直後に `w/s/q/a/e/d` またはCAN速度指令を送っても通常経路の確認へ進められる。

UART受信は1バイト保持では連続入力を取りこぼすため、32バイトのリングバッファへ変更した。`wwww3` のような連続コマンドでも順次処理できる。

## 2026-05-03 追記: 高回転確認中断と処理余裕計測
高回転確認では `+10 rps` まではM0/M1同時に追従したが、`+15 rps` 以上で脱調/ロックが発生した。ロック時に電源ライン電流が約 `1.0 A` 以上まで上がったため、実機保護を優先して `THR_MOTOR_OVER_CURRENT` を `0.70 A` に下げた。

制御処理の余裕確認のため、DWT cycle counter を有効化し、診断ページ0に次の値を追加した。

- `loop current/max us`: 1kHzメイン処理の待ち時間を除いた処理時間。
- `slack us`: 1ms周期に対する、直近100ms内の最悪処理時間から見た残り時間。
- `isr current/max us`: TIM1割り込み内の処理時間。

高回転側の再試験は、まず処理余裕ログと電流保護動作を確認してから行う。

追加計測の結果、`+10 rps` 動作中はTIM1 ISRが最大約 `34 us` で50us周期内に収まっていた。一方、1kHzメイン処理は診断出力周期に最大 `1.4〜1.8 ms` まで伸び、1ms周期を超過していた。通常open-loopの電気角更新を1kHzメイン処理に置くと、高回転時に角度更新が粗く、診断出力の遅延で位相も乱れるため、open-loop電気角更新はTIM1 ISR内のPWM更新タイミングへ移した。

固定50us仮定でISR内積分した場合、実測速度が指令より約5%高くなったため、DWT cycle counter の実測差分から各モーターの更新dtを計算し、そのdtで電気角を積分する方式へ変更した。

同じ約5%のずれは速度推定側にもあり、1kHz待ちが `<= 20` 判定で実質21割り込み待ちになっていた。待ち条件を20割り込みちょうどへ修正し、速度推定も固定1msではなくDWT実測dtで計算する。

修正後の実機確認では、M0/M1同時に `+20 rps` まで追従した。代表値は次の通り。

- `+5 rps`: M0 `+4.99 rps`, M1 `+5.01 rps`, Faultなし。
- `+10 rps`: M0 `+9.98 rps`, M1 `+10.01 rps`, Faultなし。
- `+12.5 rps`: M0/M1 とも `+12.50 rps`, Faultなし。
- `+15 rps`: M0/M1 とも `+15.00 rps`, Faultなし。
- `+20 rps`: M0 `+20.01 rps`, M1 `+19.99 rps`, Faultなし。

ただし診断出力を含む1kHzメイン処理は、出力周期に最大約 `1.7〜2.1 ms` まで伸びる。PWM位相更新はISR側へ移したため高回転追従は改善したが、保護処理やコマンド処理の周期余裕を詰めるには、診断出力を制御ループ外へ逃がす、またはログ周期/内容を減らす必要がある。

## 2026-05-03 追記: 左右/正逆確認前の保護調整
正逆比較の再開直後に `BLDC_UNDER_VOLTAGE` が一度ラッチしたが、ログ上のバッテリ表示は24V台であり、単発ADC値による誤検出の可能性が高い。電圧/電流保護は20ms連続検出でFaultにする。Fault解除用にUART `R` でシステムリセットできるようにし、停止中の電流オフセット再取得用にUART `u` を追加した。

電流オフセット再取得後、M0/M1同時に正逆方向で `5/10/15/20 rps` を確認した。左右差、正逆差はいずれも表示分解能レベルで、Faultなし。

| 指令 | M0実測 | M1実測 | M0 Vq | M1 Vq |
|---:|---:|---:|---:|---:|
| `+5 rps` | `+5.00` | `+5.01` | `+1.13 V` | `+1.13 V` |
| `+10 rps` | `+10.00` | `+9.98` | `+2.08 V` | `+2.06 V` |
| `+15 rps` | `+14.99` | `+14.99` | `+3.03 V` | `+3.00 V` |
| `+20 rps` | `+19.98` | `+20.01` | `+3.98 V` | `+3.93 V` |
| `-5 rps` | `-4.99` | `-5.02` | `-1.13 V` | `-1.13 V` |
| `-10 rps` | `-9.98` | `-9.99` | `-2.08 V` | `-2.06 V` |
| `-15 rps` | `-15.00` | `-14.99` | `-3.03 V` | `-3.00 V` |
| `-20 rps` | `-20.01` | `-19.99` | `-3.98 V` | `-3.93 V` |

## 2026-05-03 追記: 60rps試験に向けた制限緩和
`APP_OUTPUT_VOLTAGE_LIMIT = 6.0 V`, `THR_MOTOR_OVER_CURRENT = 0.70 A` では、M0/M1同時に `+/-32.5 rps` までは追従したが、`+/-35 rps` で過電流Faultになった。`32.5 rps` 時点でVqは両軸とも `6.0 V` に張り付いている。

安全側の再試験として、出力電圧上限を `8.0 V`、過電流しきい値を `1.00 A` に一時的に緩める。再試験は `35 rps` から段階的に行い、過電流または脱調が見えた時点で中断する。

再試験結果:

| 指令 | M0実測 | M1実測 | M0 Vq | M1 Vq | 結果 |
|---:|---:|---:|---:|---:|---|
| `+30 rps` | `+30.02` | `+29.99` | `+5.88 V` | `+5.79 V` | Faultなし |
| `+35 rps` | `+34.98` | `+34.99` | `+6.83 V` | `+6.72 V` | Faultなし |
| `+40 rps` | - | - | - | - | `BLDC_OVER_CURRENT`, `+1.19 A` |
| `-30 rps` | `-29.99` | `-30.01` | `-5.88 V` | `-5.79 V` | Faultなし |
| `-35 rps` | `-35.00` | `-34.99` | `-6.83 V` | `-6.72 V` | Faultなし |
| `-40 rps` | - | - | - | - | `BLDC_OVER_CURRENT`, `+1.15 A` |

現時点で安全に確認できた上限は M0/M1同時の `+/-35 rps`。`40 rps` 以上は、電流リミットをさらに上げる必要があり、ロック時の熱/電流リスクが上がるため今回の安全範囲では進めない。

## 2026-05-03 追記: 制御ループとデバッグ出力の分離
電気レベルのモーター制御、つまりopen-loop電気角積分、ADC/SPI更新、PWM CCR更新はTIM1 ISR側で行う。速度指令、スルーレート、電圧モデル、保護判定、CAN処理、パラメーター更新は1kHz側で行う。

デバッグ用UART出力は `vsprintf` とDMA送信を含み、1kHz周期を大きく乱すため通常RUNの周期出力から外した。UART `1` / `2` / `3` / Enter は連続ログではなく、要求されたページを1回だけ出すスナップショットにする。UARTコマンド受信時は要求フラグを立て、1kHzループ内で保護判定と通常更新を終えた後にだけデバッグ出力を処理する。

- `1`: 状態/処理時間ページを1回出力。
- `2`: M0ページを1回出力。
- `3`: M1ページを1回出力。
- Enter: ページを進めて1回出力。
- `i`: 先にフリーウィールへ落としてから、後段で非回転I/Oチェックを実行。
- `m`: 後段でFOC計算セルフテストを1回実行。

この方式では、デバッグコマンドを打った瞬間だけ次周期が伸びる可能性は残るが、周期デバッグ出力によってRUN中に継続的に1kHzを破ることは避ける。

## 2026-05-03 追記: ISR処理時間の再測定と三角関数負荷の削減
TIM1 ISRは20kHzで呼ばれるため、1回あたりの周期は約50usである。M0/M1は割り込みごとに交互更新しているため、各モーターのPWM更新は約10kHzとなる。

当初の10rps RUN確認では、ISR内で `fmodf`、`sinf`、`cosf` を毎回呼んでいたため、TIM1 ISRの最大処理時間が約38usまで伸びた。50us周期に対する残りは約12usしかなく、高回転時の追加処理や割り込み干渉に対して余裕が小さい。

対策として、起動時に `focMathInit()` で1024点のsinテーブルを作成し、ISR中のSinePWM計算ではテーブル線形補間を使用する。角度正規化も、通常の微小増分で重い剰余演算を使わない実装にした。これによりSimpleFOC風の電圧次元SinePWMの構造は維持しつつ、ISR内のlibm呼び出しを避ける。

再測定結果:

| 条件 | TIM1 ISR current/max | 50us周期に対する最大余裕 | 1kHz loop current/max | 備考 |
|---|---:|---:|---:|---|
| 起動後、Freewheel | `16/16 us` | 約`34 us` | `109/110 us` | 連続デバッグ出力なし |
| M0/M1 `+10 rps` RUN | `24/27 us` | 約`23 us` | `116/213 us` | M0 `+9.98 rps`, M1 `+10.02 rps` |
| M0/M1 `+30 rps` RUN | `27/27 us` | 約`23 us` | `116/211 us` | M0 `+30.03 rps`, M1 `+29.99 rps` |

速度を10rpsから30rpsへ上げてもISR最大時間は増えていないため、現在の主要なCPU負荷は回転数そのものではなく、ADC/SPI更新とSinePWM計算の固定コストである。現状のISR最大27usは50us周期内に収まっているが、余裕は約23usなので、今後ISR内へ保護判定、UART、Flash、CAN、複雑な推定処理を入れないこと。

## 2026-05-03 追記: PWM/ISR周期の引き上げ確認
TIM1 ISR最大処理時間は約27usであるため、CPU時間だけを見ると30kHz付近まで引き上げ可能に見える。ただし実機ではPWM周波数を上げるとモーター/ドライバ側の実効条件も変わるため、段階的に確認した。

設定はTIM1/TIM8を同じPeriodにし、`TIM_PWM_CENTER` はPeriodの半分、`APP_PWM_ISR_PER_1MS` は1msあたりのTIM1更新回数に合わせる。

| TIM1/TIM8周期 | Period | `TIM_PWM_CENTER` | `APP_PWM_ISR_PER_1MS` | 結果 |
|---:|---:|---:|---:|---|
| 約30kHz | `1200` | `600` | `30` | 10rpsで実回転ほぼ0rps、電流約0.5A。採用不可 |
| 25kHz | `1440` | `720` | `25` | 10rpsで実回転ほぼ0rps、電流約0.5A。採用不可 |
| 24kHz | `1500` | `750` | `24` | 10rpsで実回転ほぼ0rps、電流約0.6A。採用不可 |
| 約21kHz | `1714` | `857` | `21` | 10rpsで正常追従 |
| 約22kHz | `1636` | `818` | `22` | 10rpsで正常追従 |
| 約23kHz | `1566` | `783` | `23` | 10rps/30rpsで正常追従 |

最終的に、実機で正常動作を確認できた最大側として約23kHz設定を採用する。23kHz時の30rps確認結果は、M0 `+29.97 rps`、M1 `+30.02 rps`、Faultなし。TIM1 ISRは最大 `27us`、周期は約`43.5us`なので、CPU時間余裕は約`16.5us`、比率では約38%である。

24kHz以上ではCPU時間としては周期内に収まっているにもかかわらず回転しないため、制限要因はCPU処理時間ではなくPWM周波数上昇に伴うゲートドライバ、電流応答、ADC注入変換タイミング、または実効電圧のいずれかである可能性が高い。追加調査なしに24kHz以上へ進めないこと。

## 2026-05-03 追記: TIM1/TIM8分離制御、CCRプリロード、半周期位相ずらし
TIM1をM0、TIM8をM1の制御ISRとして分離した。TIM8はTIM1に対してカウンタを半周期分ずらして開始し、2つの制御ISRが同時に走りにくいようにする。CCRはプリロードを有効にし、PWM周期途中でのCCR即時反映を避ける。ゲートドライバ側にデッドタイム設定があるため、TIM1/TIM8のDeadTimeは0にした。

この構成では各モーターを各PWM周期で更新するため、以前の「TIM1 ISRでM0/M1を交互更新」構成より総ISR負荷が約2倍になる。23kHzのままでは10rps RUN時にUART応答が止まり、CPU過負荷またはISR詰まりが疑われた。HALの汎用 `HAL_TIM_IRQHandler()` 経由も負荷になるため、TIM1/TIM8 update IRQではupdate flagを直接確認して `bldcAppOnTimerElapsed()` を呼ぶ経路に変更した。

CPU余裕を見て、最終的にTIM1/TIM8を15kHzへ下げた。

| 項目 | 値 |
|---|---:|
| TIM1/TIM8 Period | `2400` |
| `TIM_PWM_CENTER` | `1200` |
| `APP_PWM_ISR_PER_1MS` | `15` |
| TIM DeadTime | `0` |
| CCR preload | 有効 |
| TIM8 phase | TIM1から半周期ずらし |

15kHz確認結果:

| 指令 | M0実測 | M1実測 | M0 Vq | M1 Vq | 結果 |
|---:|---:|---:|---:|---:|---|
| `+10 rps` | `+10.00` | `+10.00` | `+2.08 V` | `+2.06 V` | Faultなし |
| `+20 rps` | `+19.98` | `+20.01` | `+3.98 V` | `+3.93 V` | Faultなし |
| `+25 rps` | `+24.99` | `+25.00` | `+4.93 V` | `+4.86 V` | Faultなし |
| `+30 rps` | - | - | - | - | `BLDC_OVER_CURRENT`, M0 `+1.22 A` |

15kHz/25rps時のTIM1/TIM8 ISR単体最大は約`27us`、1kHz側の通常処理最大は約`578us`だった。30rpsでは過電流が再現したため、現構成で安全に確認できた上限はM0/M1同時の`+25 rps`である。30rps以上へ進める場合は、PWM周波数ではなく電流保護、Vqモデル、加速ランプ、または電流オフセット/過渡電流検出の扱いを先に見直す。

## 2026-05-03 追記: TIM1単独ISRへの復帰とTIM1/TIM8同期
TIM1/TIM8の両方で制御ISRを走らせる構成は、割り込み負荷とデバッグ時の扱いが複雑になるため廃止した。制御ISRはTIM1 updateのみへ戻し、TIM1 ISR内でM0/M1を交互更新する。TIM8はPWM出力タイマーとして使用するが、base interruptは開始しない。

TIM1/TIM8のカウンタ位相は同期させる。起動時に両タイマーのカウンタを0へ戻し、update eventを発行してプリロード値を反映してから、update flagをクリアする。CCR preload、ARR preload、TIM DeadTime 0は維持する。

最終設定:

| 項目 | 値 |
|---|---:|
| 制御ISR | TIM1 updateのみ |
| TIM1/TIM8 Period | `1566` |
| `TIM_PWM_CENTER` | `783` |
| `APP_PWM_ISR_PER_1MS` | `23` |
| TIM1/TIM8位相 | 同期 |
| TIM8 base interrupt | 未使用 |
| TIM DeadTime | `0` |
| CCR preload | 有効 |

実機確認:

| 指令 | M0実測 | M1実測 | M0 Vq | M1 Vq | 結果 |
|---:|---:|---:|---:|---:|---|
| `+10 rps` | `+10.01` | `+9.98` | `+2.08 V` | `+2.06 V` | Faultなし |
| `+30 rps` | `+29.97` | `+30.00` | `+5.88 V` | `+5.79 V` | Faultなし |

30rps時のTIM1 ISR最大は約`27us`、1kHz loop最大は約`236us`で、TIM1/TIM8分離ISR構成より1kHz側の余裕が大きく改善した。

## 2026-05-03 追記: 60rpsへ向けた再試験結果
TIM1単独ISR、TIM1/TIM8同期、CCR preload有効、TIM DeadTime 0、約23kHz設定で60rpsへ向けた段階試験を行った。

結果:

| 指令 | M0実測 | M1実測 | M0 Vq | M1 Vq | 結果 |
|---:|---:|---:|---:|---:|---|
| `+30 rps` | `+30.00` | `+30.04` | `+5.88 V` | `+5.79 V` | Faultなし |
| `+32.5 rps` | - | - | - | - | `BLDC_ENC_ERROR`, M0推定速度 `+242.30 rps` |
| `+35 rps` | - | - | - | - | 32.5rpsでFault済みのため未実施 |
| `+60 rps` | - | - | - | - | 32.5rpsでFault済みのため未実施 |

32.5rpsで出たFaultは過電流ではなくエンコーダ速度推定異常である。`updateSpeed()` は1kHz側で、最後にISRで更新されたAS5047P raw値との差分とDWT実測dtから速度を計算している。M0で一度 `APP_RPS_LIMIT * 2` を超える速度に見えるスパイクが発生した。30rpsまでは正常に追従しているため、60rpsへ進める前に、AS5047P読み取りタイミング、速度推定の外れ値処理、SPI読み取り失敗時の検出、1kHz速度推定とISR内SPI更新のデータ競合を切り分ける必要がある。

試験後はST-Link経由でリセットし、Freewheel起動状態へ戻した。
## 2026-05-03 追記: 電流上昇原因の切り分け

電流制限は `THR_MOTOR_OVER_CURRENT = 1.00 A` のまま維持し、電流上昇を許容して高回転化する方針は取らない。電流が大きく増える場合は、制御前提の不一致や同期外れを疑う。

確認結果:

| 条件 | 結果 |
|---|---|
| 修正前のセンサー角Vq制御 M0 `+1 rps` | 実測 `-1.57 rps`。指令符号と実回転符号が逆。 |
| 修正前のセンサー角Vq制御 M0 `-1 rps` | 実測 `+1.54 rps`。同じく逆。 |
| 修正前のセンサー角Vq制御 M1 `+1 rps` | 実測 `-1.55 rps`。M0/M1共通の符号問題。 |
| センサー診断 `v` | 電気角正方向掃引に対してエンコーダー差分はM0/M1とも正。`APP_SENSOR_DIRECTION = 1` は妥当。 |
| オープンループ M0 `+1 rps` | 実測 `+1.09 rps`。`APP_OPEN_LOOP_DIRECTION = -1` により指令符号は合っている。 |
| オープンループ M0 `-1 rps` | 実測 `-0.94 rps`。 |

原因は、エンコーダー方向ではなく、センサー角を使って `Uq` を印加したときのトルク符号がユーザー指令の符号と逆になっていたこと。オープンループでは `APP_OPEN_LOOP_DIRECTION = -1` によって電気角進行方向を補正していたため低速では動いていたが、センサー角Vq制御では同等の補正がなく、正方向指令で逆方向トルクを出していた。

対策として `APP_SENSOR_TORQUE_DIRECTION = -1.0f` を追加し、センサー角Vq制御時だけ `Uq` 符号を反転する。オープンループの角度進行方向補正は変更しない。

修正後の確認結果:

| 条件 | 実測 | Vq | 電流 | 結果 |
|---:|---:|---:|---:|---|
| センサー角Vq M0 `+1 rps` | `+1.58 rps` | `+0.37 V` | `-0.03 A` | 符号正常 |
| センサー角Vq M0 `-1 rps` | `-1.53 rps` | `-0.37 V` | `-0.03 A` | 符号正常 |
| センサー角Vq M1 `+1 rps` | `+1.65 rps` | `+0.39 V` | `-0.01 A` | 符号正常 |
| センサー角Vq M0 `+10 rps` | `+10.78 rps` | `+2.08 V` | `-0.03 A` | Faultなし |
| センサー角Vq M0 `+20 rps` | `+19.47 rps` | `+3.98 V` | `-0.01 A` | Faultなし |
| センサー角Vq M0 `+30 rps` | `+27.12 rps` | `+5.88 V` | `+0.12 A` | Faultなし |
| センサー角Vq M0 `+40 rps` | `+33.18 rps` | `+7.78 V` | `+0.33 A` | Faultなし |
| センサー角Vq M0 `+50 rps` | `+33.69 rps` | `+8.00 V` | `+0.40 A` | 電圧制限で頭打ち |
| センサー角Vq M0 `+60 rps` | `+33.65 rps` | `+8.00 V` | `+0.43 A` | 電圧制限で頭打ち |

この確認では、8V電圧制限に当たったあとは約33.6rpsで頭打ちになるが、1A制限には達していない。したがって直近の異常な電流上昇は、正常な高回転負荷ではなく、修正前のセンサー角Vq符号不整合と、オープンループ高回転時の同期外れが主因と判断する。60rpsへ進める場合は、1A電流制限を維持したまま、センサー角Vq制御を前提に電圧制限と電圧モデルを段階的に見直す。

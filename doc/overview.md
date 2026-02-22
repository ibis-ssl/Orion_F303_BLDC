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

## 出力成果物
- `Orion_F303_BLDC.elf`
- `Orion_F303_BLDC.map`
- `Orion_F303_BLDC.list`

## 注意点
- リンカの RWX 警告は現行リンカ設定由来で、今回のリファクタリング由来ではない。
- 実機評価では、起動シーケンス・校正シーケンス・保護動作・通信周期を必ず回帰確認する。

## ハードウェア仕様（コードからの推定）
以下はコード上の設定値から推測した内容であり、回路図/部品表での最終確認が必要。

### MCU/クロック
- MCU: STM32F303 系（`STM32F303xC` 定義、startup ファイル名より）
- コア: Cortex-M4F
- システムクロック: HSE + PLL x9（`SystemClock_Config`）で約 72MHz 構成
- 主制御周期: 1ms（20kHz 割り込みを 20 回で 1ms 相当として運用）

### モータ駆動
- 対象: 2モータ（M0/M1）
- PWM: TIM1 と TIM8 の 3相コンプリメンタリ出力（CH1-3 + CH1N-3N）
- PWM 設定: `Period=1800`, `DeadTime=10`
- 推定PWM周波数: 約 20kHz（コメントと設定値より）
- フリーウィール制御: CH/CHN の有効無効を直接切り替え

### エンコーダ
- 種別: AS5047P（SPI1, 16bit フレームアクセス）
- センサ数: 2個（CS: PB6/PB7 で個別選択）
- 角度分解能扱い: 14bit 読み出し値を内部で 16bit スケールへ展開

### アナログ計測
- ADC: ADC1/ADC2/ADC3 を併用
- 計測対象:
  - バッテリ電圧
  - ゲートドライバ DCDC 電圧
  - モータ電流 x2
  - モータ温度 x2
  - FET温度 x2
- 電流センサ想定:
  - ZXCT1084（オフセット 0）
  - INA199（オフセット 2048）
  を起動時の生ADC値で切り替える実装

### 通信
- CAN: 1ch 使用（PA11/PA12）
  - 設定値: Prescaler=4, BS1=4TQ, BS2=4TQ（コード上）
  - フィルタで速度指令、電源有効、キャリブ開始などを受信
- UART1: 2,000,000 bps（PC4/PC5）
  - DMA送信バッファを使ったデバッグ出力

### GPIO（主な用途）
- スイッチ入力: PC0-PC3（Pull-up, Low active）
- LED出力: PC13-PC15
- エンコーダCS: PB6/PB7

### 電源系のしきい値（ソフト保護）
- 過電流/過熱/低電圧/過電圧/過負荷を監視し、異常時はPWM停止＋リセット
- しきい値実体は `control_limits.h` を参照

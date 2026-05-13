# Orion_F303_BLDC 概要

この文書は、今後の開発に必要な現行仕様、制約、操作手順をまとめる。過去の実装・評価ログは [work_log.md](work_log.md) に分離した。

## 現行方針

古い独自アプリケーション層はビルド対象から外し、CubeMX生成コードとHAL周辺操作だけを流用する。アプリケーション状態、1kHz処理、PWM ISR、UART/CANコマンド、保護処理は `Core/Src/bldc_app.c` に集約する。

SimpleFOCのC++/Arduino実装は直接取り込まず、SimpleFOC相当の電圧FOC/SinePWMをCで実装する。通常運転は速度PIや電流FOCではなく、速度指令から電圧指令を生成する電圧次元の制御とする。

## ハードウェア制約

- MCU: STM32F303、Cortex-M4F、72MHz想定。
- モーター数: 2軸、M0/M1。
- PWM: TIM1/TIM8 の3相コンプリメンタリ出力。
- PWM周期: 40kHz。TIM1 ISRでM0/M1を交互に更新するため、各モーターの実効更新は20kHz。
- 制御ターゲット・パラメーター更新: 1kHz。
- エンコーダー: AS5047P、SPI1、14bit角度を16bitスケールへ展開。CSはPB6/PB7で、起動直後からHighにする。
- AS5047PのSPI1は9MHz設定とする。18MHz相当では診断レジスタ読み出しが不安定になった。
- 電流センス: 電源ラインのローパス後の値のみ。相電流は観測できない。
- 電流値の用途: 過電流保護と状態監視のみ。電流制御、抵抗推定、dq電流推定には使わない。
- UART1: 2,000,000bps。デバッグ出力は高負荷になり得るため、ISR内では出力しない。

## 制御構成

通常RUN経路は `velocity_openloop` を標準とする。CAN/UARTの速度指令から `command_rps` を保持し、1kHz周期で `target_rps` をスルーレート制限しながら追従させる。

```text
target_rps += clamp(command_rps - target_rps, +/-0.01 rps/ms)
Vq = target_rps * voltage_per_rps
```

現行校正では `voltage_offset` は使わず、Flash保存値も `0.0 V` とする。Flashには既存形式に合わせ、`rps_per_v = 1 / voltage_per_rps` を保存する。

センサ角を使う電圧FOCは通常経路ではなく、ゼロ電気角、相順、センサ方向、進角、速度係数を詰めるための診断・校正経路として扱う。

## 符号規約

現ハードウェアの実機確認結果に基づき、符号は以下で固定する。

```c
APP_SENSOR_DIRECTION      = +1
APP_OPEN_LOOP_DIRECTION   = -1
APP_SENSOR_TORQUE_DIRECTION = -1.0f
APP_POLE_PAIRS            = 12
```

この値を単独で変更してはいけない。相順、エンコーダ配線、FOC変換、ゼロ角計算を変更した場合は、UART `v` のセンサ方向診断と `c` センサ角電圧診断で符号を再確認する。

正の速度指令では、センサ角Vq診断で実測 `rps` が正になること、電気角正方向スキャンでAS5047P raw差分が正になることを確認する。

## 進角モデル

40rpsまでの正逆評価から、進角は正逆共通の速度比例モデルを使う。

```text
phase_advance_rad = -0.003457 * abs(target_rps)
```

12極対では約46usの等価遅れ補償に相当する。UART `[` / `]` は進角トリムを1度単位で調整し、`p` でトリムを0度へ戻す。

## ゼロ電気角校正

ゼロ電気角校正はUART `k` / `K` の一部として実行する。

- `k`: RAM反映のみ。
- `K`: 校正後にFlashへ保存。
- M0/M1を並列に実行する。
- 各電気角ステップのサンプル完了時に、M0/M1それぞれのAS5047P raw値、機械角、計算ゼロ角をUARTへ出力する。2パス目は往復ヒステリシスと採用/棄却も出力する。
- 固定角はUqではなくUdへ印加する。
- `6点/電気周期 * 12極対 = 72点` を正順と逆順で測定し、機械1回転分を平均する。
- 各点は250ms整定、100msサンプル。
- 正順/逆順の同一点ペアからヒステリシスを計算する。
- ヒステリシスが8degを超える点はゼロ角平均から除外する。
- 校正ログには `pairs used/rej` と `hyst avg/max` を出す。

2026-05-04時点の保存値:

```text
zero_electric_angle:
  M0 +5.795 rad
  M1 +3.067 rad
voltage_per_rps:
  M0 +0.203 V/rps
  M1 +0.203 V/rps
voltage_offset:
  M0 +0.00 V
  M1 +0.00 V
```

保存後の40rps簡易確認では、M0/M1が `+42.04/+42.06 rps`、`-41.74/-41.91 rps` で、Faultは発生していない。

2026-05-04の並列ゼロ電気角校正確認では、保存なし `k` でM0/M1のゼロ角校正が同時に進行し、両方とも `72/0` ペア採用/除外、ヒステリシス平均/最大はM0 `0.1/0.4 deg`、M1 `0.1/0.3 deg` だった。RAM反映結果は M0 `+5.796 rad`、M1 `+3.067 rad`、Faultなし。

## 速度係数校正

速度係数校正は、固定Vqを静止状態から直接上げる方式を使わない。低速域では静止摩擦やコギングが支配的で過電流になりやすいためである。速度係数校正は電源電圧降下や負荷の相互影響を避けるため、M0、M1の順に片側ずつ実行する。

現在は通常のセンサ角電圧制御経路で以下を順に指令し、実際に出した `abs(Vq)` と実測 `abs(rps)` から原点通過の最小二乗で `voltage_per_rps` を求める。

```text
+10, +20, +30, +40, -10, -20, -30, -40 rps
```

フィット後は5%の電圧余裕を掛ける。電流センスが相電流ではないため、速度係数以上のモーターパラメーター推定は現状の範囲外とする。

## UART操作

```text
i      非回転I/Oチェック。ADC、AS5047P、PWM、Flash、FOC計算を確認する。
m      FOC計算セルフテスト。
l      保護モデルチェック。疑似電流制限と短期I2R積算の非回転診断。
R      MCUリセット。
u      電流オフセット校正。
n      RUN許可。目標速度0のままRUNへ入り、PWM出力は非ゼロ指令までフリーウィールを維持する。
x/0    60秒フリーウィール。
space  目標速度0、60秒フリーウィール。
w/s    M0/M1両方の目標速度を +/-0.5 rps。
q/a    M0のみ目標速度を +/-0.5 rps。
e/d    M1のみ目標速度を +/-0.5 rps。
y/h    M0/M1の単発センサアライメント。
v      センサ方向診断。
o      open-loop velocityを有効化。
c      センサ角電圧診断モードを有効化。
[/]    進角トリムを -/+1deg。
p      進角トリムを0degへ戻す。
k      モーターパラメーター校正をRAM反映のみで実行。
K      モーターパラメーター校正を実行し、Flashへ保存。
1/2/3  診断ページ0/1/2を表示。
4      PWM TIM1/TIM8のCCR/CCER/BDTRを表示。
Enter  診断ページを切り替えて表示。
```

## スイッチ操作

スイッチはPC0-PC3のPull-up/Low active入力である。起動時にSW1またはSW2が押されている場合はFlashのボードIDを更新する。SW1とSW2が同時に押された場合はSW1を優先する。

```text
起動時SW1  ボードIDを0に設定してFlashへ保存。
起動時SW2  ボードIDを1に設定してFlashへ保存。
起動時SW4  モーターパラメーター校正を開始し、完了後Flashへ保存。
動作中SW1  M0/M1を +10 rps で回転。
動作中SW2  M0/M1を -10 rps で回転。
動作中SW3  M0/M1を +40 rps で回転。
動作中SW4  M0/M1を -60 rps で回転。
```

動作中スイッチ指令はCAN速度指令より優先する。スイッチを押している間だけ1kHz周期で目標速度を上書きし、離すと約2msでスイッチ指令は失効する。起動時にいずれかのスイッチが押されていた場合は、ボードID設定や校正要求だけを記録し、全スイッチが一度離されるまで初期化中に待機する。これにより、リセット直後の押しっぱなしが動作中スイッチ指令として扱われてモーターが回ることを防ぐ。

## CAN操作

- CAN bitrateは1Mbps。CANable/libusbツールではターゲット送信テレメトリ `0x202`, `0x203`, `0x212`, `0x213`, `0x232`, `0x233`, `0x502`, `0x503` を受信確認済み。
- FWの受信経路は、フィルタ設定後に `HAL_CAN_Start()` し、その後 `CAN_ActivateRxNotifications()` でFIFO0/FIFO1通知を有効化する。速度指令はFIFO0、制御系IDはFIFO1へ割り当てる。
- FIFOコールバックはFIFOが空になるまで `HAL_CAN_GetRxMessage()` でドレインする。連続CAN指令では1割り込み1フレーム処理に戻すと取りこぼしやすい。
- 2026-05-05時点の実機確認では、CANableからは送信ECHOが見えるが、ターゲット側の `CAN` 受信カウンタが増えず速度指令が反映されない。ターゲットからCANable方向の通信は確認済みのため、CANable TXからターゲットRXまでの配線、トランシーバ、終端、CANableの送信/ACK状態を優先して切り分ける。
- `0x010`: `data[0] == 0 && data[1] == 0` でMCUリセット。
- `0x100`, `0x102`: M0速度指令。
- `0x101`, `0x103`: M1速度指令。
- `0x110`: `data[0] == 3` でフリーウィール時間指定。
- `0x320`: I/Oチェック要求。

## 状態遷移

- `BLDC_APP_MODE_BOOT`: 起動初期化中。
- `BLDC_APP_MODE_FREEWHEEL`: PWMチャネルを無効化し、相出力をフリーウィール。
- `BLDC_APP_MODE_READY`: 入出力初期化済み、出力指令待ち。
- `BLDC_APP_MODE_RUN`: PWM出力有効、SinePWMで電圧指令を出力。
- `BLDC_APP_MODE_FAULT`: 保護検出後。PWMはフリーウィール固定。

起動直後は安全側として60秒フリーウィールに入る。UART `n` だけでは目標速度0のため即時回転しない。

## 1kHz処理

`bldcAppTick1kHz()` の主な処理順:

1. UART受信コマンド処理。
2. 速度推定。
3. フリーウィールタイマ更新。
4. センサアライメント、センサ診断、パラメーター校正の状態更新。
5. 速度指令から `voltage_q` を計算。
6. open-loop角度管理。
7. 電圧、電流、温度、エンコーダ異常の保護判定。
8. ゼロ出力継続時のPWMフリーウィール化。
9. CANテレメトリ送信。
10. 遅延実行のUART診断出力。
11. TIM1割り込み40回を待って1ms周期へ同期。

## PWM ISR

TIM1割り込みでM0/M1を交互に処理する。ISR内では以下のみを行う。

- ADC高速値更新。
- AS5047P更新。
- 速度推定用の差分更新。
- 必要なSinePWM CCR更新。
- ISR処理時間計測。

UART出力、CAN送信、重い診断計算はISR内で行わない。

`voltage_q == 0` の場合はSinePWM計算を省略し、PWM中心値だけを設定して戻る。40kHz動作ではゼロ出力中の三角関数計算だけでもメインループやUART応答を圧迫するためである。

AS5047P診断レジスタ読み出しは2フレームアクセスで行う。I/Oチェック中はPWM ISRの通常SPI更新と競合しないよう、短時間だけ割り込みを止めて ERRFL/PROG/DIAAGC/MAG/ANGLEUNC/ANGLECOM を読む。通常運転の角度更新では `last_frame` と `spi_error_count` を監視する。

## 保護機能

保護判定は主に `bldcAppTick1kHz()` 内の `applyProtection()` で1kHz周期に実行する。エンコーダ速度異常のみ、速度更新処理内で検出する。Fault検出時は `bldcAppForceFault()` により `BLDC_APP_MODE_FAULT` へ遷移し、校正・診断を中断し、目標速度を0へ戻し、PWMをフリーウィールにする。Fault状態ではUART `n` などによるRUN復帰はできず、復帰にはリセットが必要。

保護閾値は `Core/Inc/control_limits.h` と `Core/Src/bldc_app.c` の定数で管理する。

| 保護 | Fault ID | 閾値 | 判定条件 | 補足 |
|---|---:|---:|---|---|
| 低電圧 | `BLDC_UNDER_VOLTAGE` `0x0001` | `18.0 V` | バッテリ電圧が20ms連続で下回る | `fault_info = 0` |
| 過電圧 | `BLDC_OVER_VOLTAGE` `0x0020` | `35.0 V` | バッテリ電圧が20ms連続で上回る | `fault_info = 0` |
| 瞬間過電流 | `BLDC_OVER_CURRENT` `0x0002` | `10.00 A` | RUN中、PWM ISRで取得した電源ライン電流が1回でも上回る | 短絡相当の即時停止用 |
| 過負荷過電流 | `BLDC_OVER_LOAD` `0x0008` | `1.00 A` | 各モーターの電源ライン電流が20ms連続で上回る | 従来の平均過電流保護 |
| 推定過負荷 | `BLDC_OVER_LOAD` `0x0008` | `3.00 A相当の短期I2R損失` | 推定電流から計算した短期損失積算がしきい値を超える | Vqと実測速度から疑似判定 |
| モーター過熱 | `BLDC_MOTOR_OVER_HEAT` `0x0004` | `70 degC` | 各モーター温度が上回る | 異常値除外後のローパス値で判定 |
| FET過熱 | `BLDC_FET_OVER_HEAT` `0x0040` | `80 degC` | 各FET温度が上回る | 異常値除外後のローパス値で判定 |
| エンコーダ速度異常 | `BLDC_ENC_ERROR` `0x0010` | `160 rps` | RUN中に5回連続で推定速度の絶対値が上回る | SPI/角度飛びの検出用 |

過電流は相電流ではなく、電源ラインのローパス済み電流で判定する。そのため、PWM周期内の瞬間相電流やdq電流は保護できない。瞬間過電流はADC高速更新値をPWM ISR内で確認し、しきい値超過時に即Faultへ入れる。ただし回路側のローパス後の値であるため、ハードウェアコンパレータ相当の短絡保護ではない。

過負荷保護は2系統で行う。1つ目は従来の電源ライン電流が1Aを20ms連続で超えた場合の停止。2つ目は、Vq電圧と現在回転数から疑似電流を推定し、電圧指令を制限しつつ損失を積算する方式である。

キャリブレーション中は速度係数やゼロ電気角の取得を優先し、疑似電流制限、過負荷過電流、推定過負荷の `BLDC_OVER_LOAD` 判定は無効化する。短絡相当の瞬間過電流、低電圧/過電圧、温度、エンコーダ速度異常は通常どおり有効にする。

疑似電流推定は次の考え方とする。

```text
expected_voltage = abs(measured_rps_ave) * voltage_per_rps + voltage_offset
estimated_current = max(0, abs(Vq) - expected_voltage) / APP_EST_CURRENT_RESISTANCE_OHM
estimated_loss = estimated_current^2 * APP_EST_CURRENT_RESISTANCE_OHM
```

現状の `APP_EST_CURRENT_RESISTANCE_OHM` は実測/仕様値として `0.28 ohm` としている。負荷制限は `1.00 A` 相当で、推定電流がこの値を超えないように `Vq` を制限する。推定過負荷の検知は、制限状態の連続時間ではなく `I^2R` の短期損失積算で行う。3A相当の損失が約1秒続く量を基準にし、数秒の時定数で自然減衰させるため、制限が一瞬だけ解除されても積算値は即リセットしない。

疑似電流保護の役割は、瞬間過電流保護と温度センサーによる長期温度保護の間を埋めることである。温度上昇そのものはモーター/FET温度センサーで検知できるため、疑似電流側では長期熱モデルを持たず、ロック、脱調、急負荷など「回転数に対してVqだけが大きい」状態を数百msから数秒の範囲で抑える。この推定は実相電流の正確な推定ではない。

温度値は `adcUpdateTemperatureFilters()` で1kHz更新する。明らかな異常値はフィルタへ投入せず、正常範囲のサンプルだけを一次ローパスに通す。過熱保護、UART表示、CAN温度送信はいずれも `getTempMotor()` / `getTempFET()` のフィルタ済み値を使用する。

2026-05-04の固定モーター実機確認では、センサ角電圧診断モードで `+5 rps` を指令した状態で `lim=1`、推定電流は約 `3.6 A`、短期損失積算は `2.52 J` 付近まで増加し、`BLDC_OVER_LOAD` `0x0008` でFaultした。Fault値は `+3.61 A` であり、疑似電流制限と短期I2R過負荷検知が実機で動作することを確認した。

エンコーダ速度異常は、AS5047P raw差分と経過サイクルから計算した瞬時速度が `160 rps` を超えた場合にカウントする。5回連続で発生するとFaultにする。単発の外れ値は `speed_glitch_count` と診断ログに記録するが、連続しなければ測定値更新を止めるだけでFaultにはしない。

ゼロ出力が続く場合は保護ではなく省電力・安全動作として扱う。RUN中にM0/M1の `target_rps` が両方0の状態が5秒続くと、PWMをフリーウィールにする。

現在未実装または限定的な保護:

- 相電流ベースのハード短絡保護。
- dq電流制限。
- 速度偏差、脱調、負荷過大の判定。
- ゲートドライバFault入力の明示的な監視。
- CAN指令値の範囲外異常判定。速度指令自体は `APP_RPS_LIMIT` でクランプする。

## ビルドと書き込み

Debugビルド:

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\build.ps1 -Configuration Debug
```

書き込み:

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\flash.ps1 -Configuration Debug
```

ビルドと書き込み:

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\build_and_flash.ps1 -Configuration Debug
```

UART監視:

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\monitor_uart.ps1 -Port COM167 -BaudRate 2000000
```

CANableを使ったCAN入出力デバッグCLIの設計方針は [canable_debug_design.md](canable_debug_design.md) にまとめる。

既存のリンカ警告 `LOAD segment with RWX permissions` は残っている。

## 主要ファイル

- `Core/Src/bldc_app.c`: アプリケーション層。状態管理、1kHz処理、ISR、UART/CAN、校正、保護。
- `Core/Src/foc_math.c`: SimpleFOC風の角度正規化、電気角、SinePWM相電圧、CCR変換。
- `Core/Src/foc_driver_hal.c`: FOC計算結果をTIM1/TIM8 CCRへ反映するHALブリッジ。
- `Core/Src/io_check.c`: 非回転I/Oチェック。
- `Core/Src/adc.c`, `spi.c`, `tim.c`, `can.c`, `gpio.c`, `usart.c`, `flash.c`: CubeMX/HAL周辺操作と低レベル補助関数。
- `doc/hardware_spec.md`: ハードウェア仕様メモ。
- `doc/canable_debug_design.md`: CANable/libusbを使うCANデバッグCLIの設計方針。
- `doc/work_log.md`: 過去の実装、評価、試行錯誤ログ。

## 安全上の注意

- 実機回転前にUART `i` で非回転I/Oチェックを行う。
- 初回の回転確認は低速から開始し、電源電流、温度、Faultを監視する。
- 異常時は `space` で目標速度0とフリーウィールへ戻す。
- 電流制限は電源ライン電流に対する保護であり、相電流制御の代替ではない。
- `K` はFlashへ保存するため、保存前に `k` でRAM校正結果を確認する。

## 2026-05-10 キャリブレーション開始直後の停止対策

`k` / `K` 実行後に `param cal start multi-point save 0/1` だけが表示され、最初の強制励磁角でロックしたまま進まない場合は、過去ログの温度 Fault ではなく、キャリブレーション状態のメインループ更新が PWM ISR へ確実に見えていない可能性を優先して確認する。

`app.param_calib.active`、`stage`、`electrical_angle` は 1kHz メインループで更新し、TIM1 PWM ISR で参照する共有状態であるため `volatile` として扱う。これにより `-Ofast` 最適化時でも ISR が古い励磁角を読み続けることを避ける。

開始直後の `param cal start ...` と `param cal enc both ...` ログは出さない。開始直後の UART DMA 連続送信と float printf の負荷を下げるため、進捗ログはエンコーダサンプル完了後の短い `cal enc ...` 行から出す。

性能確認は UART 診断ページ 0 の `loop current/max us`、`slack`、`isr current/max us` を見る。`slack` が正なら 1kHz メイン周期内に収まっている。

## 2026-05-11 キャリブレーション5秒タイムアウト

`param cal enc both pass 0 point 0/72 angle_mrad +0` から進まない現象を切り分けるため、キャリブレーション各ステージに監視タイマを追加した。ゼロ電気角の強制励磁ステージは 5秒でタイムアウトし、PWM をフリーウィールへ戻す。

タイムアウト時の UART 出力例:

```text
param cal TIMEOUT stage 1 motor 0 pass 0 point 0/72 elapsed 5000ms total 5000ms angle_mrad +0
param cal timeout perf loop 10/120 us slack 880 us OK isr 3/8 us slack 17 us OK
```

`stage` は `APP_PARAM_CAL_STAGE_*` の値で、`1` は `ENC_SETTLE`、`2` は `ENC_SAMPLE` を示す。`perf` 行では 1kHz メインループの最大時間を 1000us 予算、PWM ISR の最大時間を 25us 予算で判定する。両方が `OK` なら処理時間には余裕があり、進行停止の原因は時間不足ではなく、状態更新、割り込み、PWM 出力、または UART 出力経路を疑う。

速度測定の `SPEED_SETTLE` は仕様上 6000ms かかるため、ここだけは 7000ms をタイムアウト値にする。その他のステージは 5000ms で判定する。

## 2026-05-11 実機キャリブレーション停止原因と性能確認

実機で `param cal enc both pass 0 point 0/72 angle_mrad +0` のまま進まない状態を確認した。ST-Linkで停止中のPCを読むと `Core/Src/foc_math.c` の `focPhaseVoltageToCompare()` に入っており、PWM ISR内で固定角の SinePWM 計算を毎周期繰り返していた。診断ログでも `isr 22/23 us` と25us周期にほぼ張り付き、メインループ最大時間が大きく悪化していた。

キャリブレーションの強制励磁角は `ENC_SETTLE` / `ENC_SAMPLE` 中に変わらないため、ステージ開始時に `focDriverApplySineVoltage()` を1回だけ実行し、ISRではADC/エンコーダ/速度更新だけを行う。これにより25us周期のISRから三角関数とCCR変換の繰り返しを外した。

実機確認ではキャリブレーションが `cal enc p0 i00 ...` 以降へ進行し、診断値は次の範囲に収まった。

```text
Mode 3 ... loop 101/805 us slack 195 us isr 15/18 us
```

1kHzメインループの最大805usは1000us予算内、PWM ISRの最大18usは25us予算内であり、今回の停止原因は処理時間不足そのものではなく、固定角強制励磁の計算をISRで毎回実行していたことによるISR占有だった。

## 2026-05-12 キャリブレーション完了後の停止対策

`param cal done ...` の直後に停止して見える場合は、校正完了後のフリーウィール中に PWM ISR 側の短絡電流判定だけが動き、残留回転や切替直後の電流検出で `BLDC_OVER_CURRENT` に入っていないか確認する。メイン側の保護と同じく、`freewheel_ms > 0` の間はPWM出力が無効なため、ISRの短絡電流Fault対象から外す。

校正ログは1kHzループ時間へ直接入るため、速度測定と完了サマリはfloat printfを避け、`target_mrps`、`vq_mv`、`zero_mrad`、`mv_rps` の整数ログにする。UART送信キューは `vsnprintf()` で境界チェックし、完了時の連続ログでバッファ末尾を壊さない。

実機確認では `k` が完了し、完了後診断で `Mode 1`、`Free` 残あり、`Fault 0x0000` を確認した。再書き込み直後の診断では `loop 124/149 us`、`isr 16/17 us` で、通常待機時の処理時間は予算内に収まっている。

## 2026-05-13 100ms制御余裕ログ

制御時間の余裕確認用に、100msごとに `ctrl 100ms ...` ログを出す。ログ行は直近100msの1kHzメイン処理最大時間とPWM ISR最大時間を表示し、`loop` は1000us予算、`isr` は25us予算で `slack` を判定する。UARTログを出したtickは制御時間の最大値集計から外し、文字出力自体の時間を制御処理余裕へ混ぜない。

待機中の実機確認では `ctrl 100ms mode 1 loop 125/149 slack 851 OK isr 17/18 slack 7 OK fault 0x0000` 程度で安定した。一方、UART `w` でRUNへ入れるとコマンドはRAM上で反映されるが、UARTログが読めなくなる。RUN中のPWM ISR負荷またはUART DMA送信経路の余裕不足が残っているため、RUN中の連続ログ確認は未完了。

## 2026-05-13 PWM/ISR周期を半分に落とした動作確認

RUN中に100msログが読めなくなる切り分けとして、TIM1/TIM8のPWM周期を `Period 899` から `1799`、`TIM_PWM_CENTER` を `450` から `900`、1ms同期待ちを `APP_PWM_ISR_PER_1MS 40` から `20` へ変更した。これによりPWM/ISR頻度を半分に落とす。

実機確認では待機中が `loop 27/52 slack 948 OK isr 16/17 slack 8 OK`、`w` によるRUN中が `loop 63/68 slack 932 OK isr 25/25 slack 0 NG` 程度だった。メインループは十分余裕があり、RUN中もUARTログは継続して読めるようになった。PWM ISRは25us予算ちょうどで余裕0のため、次の改善対象はRUN中ISR処理の削減である。

## 2026-05-13 PWM ISR処理の整理

`main` ブランチの構成では速度推定を1kHzメインループ側で行い、PWM ISRはADC更新、エンコーダ更新、PWM CCR更新に限定している。現行ブランチは通常RUNでオープンループ電気角を使うため、ブロッキングSPIのAS5047P読取もPWM ISR内で毎回行う必要はない。

PWM ISRから `updateAS5047P()` と速度計算を外し、既存の1kHz `updateSpeed()` へ移した。ISRに残す処理はADC高速更新、RUN中かつフリーウィール解除済みの場合の瞬間過電流判定、必要なPWM CCR更新、処理時間計測だけにする。センサ角制御や校正中の角度計算は、1kHz側で更新された最新エンコーダ値を参照する。

実機確認では待機中が `loop 61/67 slack 933 OK isr 2/2 slack 23 OK`、`n` 後に `w` でRUNへ入れた状態が `loop 77/82 slack 918 OK isr 10/10 slack 15 OK` で安定した。停止コマンド後は `mode 1` に戻り、Faultは `0x0000` のままだった。RUN中ISRが25us予算に張り付く状態は解消した。

その後、PWM/TIM周期は元の40kHz設定へ戻した。`TIM1/TIM8 Period 899`、`TIM_PWM_CENTER 450`、1ms同期待ち `APP_PWM_ISR_PER_1MS 40` とする。ISR制御周期を超過したままRUNを継続するとメインループやUARTが止まって危険なため、RUN中にPWM ISR時間が25us予算を超えた場合は `BLDC_ISR_OVERRUN` (`0x0080`) でFaultへ落とし、PWMをフリーウィールにする。

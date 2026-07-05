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
UART `c`、CAN `0x310`、起動時SW4で全キャリブレーションを開始する。校正中はCAN速度指令を無視し、エンコーダ校正が完了してから速度係数校正へ自動で進む。

UART `m` は速度係数校正だけを開始する。既存のFlash上のエンコーダゼロ角を維持し、`+3V`, `-3V`, `+5V`, `-5V` の速度係数測定だけを実行する。ゼロ電気角校正をやり直したくない実機確認では `m` を使う。

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
通常走行時の出力制御は、次の順に処理する。

```text
cmd.speed
  -> speedToOutputVoltage()
  -> cmd.out_v
  -> cmd.out_v_final
  -> focControlApplyVoltage()
  -> raw- + zero_calib + manual_offset + pi/2 + phase_advance
  -> voltage-FOC / SinePWM
  -> PWM CCR
```

`motor.c` は速度指令から出力電圧を作る。`foc_control.c` はMT6835 raw、エンコーダゼロ角、手動オフセット、q軸補正、位相進角からFOC電気角を作り、`foc_driver_hal.c` 経由で三相PWM CCRへ反映する。

FOCの角度規約は `raw- + zero + manual + pi/2`、トルク方向係数は `-1.0` に固定する。正逆転は `Uq` 符号で表し、旧実装の `ROTATION_OFFSET_RADIAN = 2.00rad` は通常RUNと速度係数校正では使わない。周期ログでは手動オフセットとFOC q軸補正 `FocAxis` を出力する。

## FOC/SinePWM経路
`dev/simple_foc` ブランチの実装を参考に、SimpleFOC相当の電圧FOC計算をCで追加した。通常RUN、FOC診断、速度係数校正は共通の `foc_control.c` を使う。

- `Core/Src/foc_math.c`: 角度正規化、電気角計算、逆Park/Clarke、相電圧からPWM CCRへの変換。
- `Core/Src/foc_driver_hal.c`: FOC計算結果をTIM1/TIM8 CCRへ反映するHALブリッジ。
- `runStartupSequence()` で `focMathInit()` を呼ぶ。ただし現ブランチではRAM節約のためFOC専用sinテーブルは持たず、診断用として `sinf()` / `cosf()` を直接使う。

FOCの三相出力はlegacy PWMと同じ相順に合わせる。legacyは `A=sin(angle)`、`B=sin(angle+120deg)`、`C=sin(angle+240deg)` の順でCCRへ入れているため、FOC側の逆ClarkeもB/Cをlegacy相順に合わせる。標準的なClarke表記のままB/Cを逆にすると、実機配線に対して電圧ベクトルの回転方向が反転し、Uq指令で回転せずロックする可能性がある。

UART `v` でFOC角度状態を出力する。これはPWM出力を変更せず、現在のエンコーダraw、`raw+`、`raw-`、`zero_calib`、legacy角、FOCで使う基準角、進角適用後の角度、速度指令、実速度、`Uq` 符号をログ出力する。シミュレーションでFOC計算式と候補規約の比較は済んだため、FW側では実行時に実際に使う `raw- + zero + manual_offset` 規約の確認に絞る。

進角ログでは `advM` が速度比例モデル由来、`advT` がUART調整トリム、`advU` が回転方向の符号まで含めて実際に電気角へ加えた値である。進角調整では同じ速度指令・同じ出力電圧で `RPS` が上がり、電流と `Eff` が悪化しない方向を探す。

`v` 診断はSPI更新を行わない。TIM割り込み側が更新済みの `mt6835[]` 値を、短時間だけ割り込み禁止してローカルへコピーし、そのスナップショットから計算する。これにより、回転中にUART診断を実行してもMT6835のSPIアクセス所有者をTIM割り込み側に一本化する。

回転中に1kHzメインループを止めると、`calcMotorSpeed()` が1ms前提でraw差分をRPS換算するため、正常回転でも速度異常になる。`v` 診断では `HAL_Delay()` を使わず、スナップショットだけを出力する。

UART `V` でFOC診断モードを開始/停止する。開始時は速度指令を0にし、PWMを有効化する。既存の `w` / `s` で速度指令を増減すると、1kHz側で速度-電圧変換を行った後、FOC診断用に `+/-2.0V` へ制限した `Uq` をTIM割り込み側でSinePWM出力する。

FOC診断モードでは、TIM割り込み内でMT6835 rawから `raw-` 電気角を作り、`enc_offset.zero_calib + manual_offset` を加えた角度を基準にする。実機確認とシミュレーションの結果から角度規約は `raw- + zero`、トルク方向係数は `-1.0` に固定する。FOCのトルク軸補正はlegacyの `ROTATION_OFFSET_RADIAN = 2.00rad` を流用せず、正逆共通の `pi/2 rad` とする。これにより、回転方向は `Uq` 符号で表し、角度補正はq軸への固定変換として扱う。候補比較用だった `B/X/Z/T/Y/H` コマンドは削除し、誤操作で規約を変えられないようにする。

FOC診断モードでは、実機調整に基づく速度比例の位相進角モデル `+0.00610865238 rad/rps`（約 `0.35deg/rps`）を入れる。実際に電気角へ加える進角は、速度指令の符号に応じて反転する。進角はモーター損失が増えすぎない範囲に抑えるため `20deg` で制限する。UART `[` / `]` で位相進角トリムを `1deg` ずつ、`{` / `}` で `0.1deg` ずつ調整し、`P` でトリムを0へ戻す。10/20/30/40rpsの実機確認では、旧モデル相当より正方向の進角でCS電流が下がり、40rpsでも `+12deg` 前後の実適用進角で停止せず回転したため、最大効率点を追い込まず内側の値としてこの係数を使う。

30kHz/96MHzオーバークロック設定へ移行した後の再調整では、20rps相当指令と40rps相当指令のどちらも追加トリム `0deg` 近傍が最小電流になった。1deg刻みでは、20rps相当は `trim 0deg` で `Avg +0.082/+0.090`、40rps相当は `trim 0deg` で `Avg +0.190/+0.197` 程度で、正負どちらへ1deg以上動かしても同等または悪化した。0.1deg刻みの再確認では、20rps相当は `-0.2..+0.5deg` が測定ノイズ内でほぼ同等、40rps相当は `+0.5deg` が `Avg +0.190/+0.193` で最小だったが、`0deg` との差は約 `0.004` と小さい。したがって30kHz条件では現行の速度比例モデルをそのまま使い、通常運用時のトリムは `0deg` とする。

UART `n` は通常RUN復帰コマンドとして、キャリブレーション状態、速度指令、出力電圧、手動オフセット、freewheel、ゼロ出力スリープを解除し、PWMを再開する。FOC診断停止後はfreewheelカウンタが残るため、進角調整などで通常RUNへ戻す場合は `n` を使う。

## FOCシミュレーション
実機だけでFOCの角度規約や速度依存の崩れを切り分けるのは難しいため、`sim/` にPC用の簡易BLDC + エンコーダシミュレータを置く。最初の目的は高精度なモータ同定ではなく、FOC数式、三相電圧、エンコーダraw、ゼロ角規約、位相進角の関係を確認することである。

`sim/bldc_sim.py` は、既存 `foc_math.c` と同じ逆Park/Clarke式で `Uq/Ud` から三相電圧を作り、実ロータ電気角でdq変換し直した `uq_effective` をトルク相当値として簡易機械モデルへ入力する。エンコーダは機械角から、ドライバがMT6835の21bit角度を縮小した後と同じ `0..65535` の制御用raw値を生成する。

実行例:

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -SelfTest
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -Scenario conventions
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -Scenario step -AngleMode raw_neg_add -OutputCsv sim\out\step_raw_neg_add.csv
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -Scenario snapshot -Raw 5108 -Zero 4.796 -LegacyElec 5.202 -UqV -2.0 -BatteryV 25.7
```

角度モードは `raw_pos_add`、`raw_pos_sub`、`raw_neg_add`、`raw_neg_sub`、`legacy` を比較できる。CSVには `time_s`、実機械角、実電気角、エンコーダraw、指令電気角、速度、`uq_cmd_v`、`ud_eff_v`、`uq_eff_v`、三相電圧を出す。実機投入前に、正しい規約で `uq_eff` が期待方向に出ること、誤った規約でトルクが落ちること、位相進角トリムで傾向が変わることを確認する。

`snapshot` シナリオは実機の `[FOC CHECK]` ログを入力し、各角度規約の指令電気角、legacy角との差分、CCR候補を出す。`-ActualElec` を指定した場合は、その実ロータ電気角に対する `ud_eff` / `uq_eff` も出力する。実機ログとシミュレータの角度候補が一致することを先に確認してから、機械モデルのステップ応答へ進む。

シミュレーション環境は、実機ではマイコンの制約でデバッグ機能を使いにくく、実時間でしか動作しない問題を回避するためのものとして扱う。そのため、FOC数式だけでなく、モータのdq電気モデル、機械慣性・摩擦・負荷、エンコーダraw量子化・遅延、将来的には30kHz ISR/1kHz main loopの周期差も模擬する。`-Model simple` は符号と角度規約の確認用、`-Model dq` は電流応答、逆起電力、位相進角不足の確認用である。

dqモデルのモーター定数は、MAD MOTOR 4006の200kVカスタムモデル相当を初期値にする。公開されているMAD 4006 EEE標準モデルは320kV/380kV/740kVであり、200kVモデルの抵抗・インダクタンスは公開値として確認できない。そのため、抵抗はMAD 4006 EEE 320kVの相間抵抗 `280mohm` を基準に、相抵抗化と巻数二乗比例で `0.3584ohm` とする。インダクタンスはMAD 4006標準モデルの公開値が見つからないため、既存シミュレータの標準値 `80uH` を320kV標準相当の仮定値として扱い、巻数二乗比例で `Ld = Lq = 204.8uH` と推定する。永久磁石鎖交磁束は200kVから `Kt = 60 / (2*pi*Kv)` を求め、`psi_f = Kt / (1.5*pole_pairs) = 0.00265Wb` とする。これらは初期推定値であり、実測できた場合は `R/L/psi_f` を優先して更新する。

機械慣性の初期値は、直径10cm、質量1kgの中実円柱が回転軸まわりに回る負荷を想定し、`J = 1/2*m*r^2 = 1.25e-3 kg m^2` とする。

`-Scenario realtime` では30kHz ISR、1kHz main loop、M0/M1交互PWM更新、PWM出力保持、エンコーダ遅延を模擬する。実機と同じく、片側のモーターは毎ISRではPWM更新されず、前回の三相電圧を保持したまま機械・電気モデルが進む。更新順やセンサ遅延の影響を見る場合はこのシナリオを使う。

GUIで操作する場合は `Script/run_sim_gui.ps1` を使う。GUIはTkinterで実装し、CLIと同じ `bldc_sim.py` のシナリオ関数を呼び出す。GUI上で条件を変えて実行し、必要に応じてCSV保存やCLIコマンドコピーで再現条件を残す。`step` / `realtime` では速度、`uq_eff`、`iq`、トルクの時系列プロットを同時表示し、M0/M1の表示対象を切り替えられる。dq軸上の `Id/Iq` 電流ベクトルに加えて、機械角と実ロータ電気角を円上のベクトルとして表示する。

GUIの `Live Start` は、実機30kHz ISR相当の `33.3us` ステップをGUI上で進める。`Live speed %` はシミュレーション時間が実時間に対して何%で進むかを指定する値で、初期値は `0.1%` である。低速ではGUI tick間隔を伸ばし、高速では1回のGUI tickで複数のステップをまとめて進める。ライブ実行中もGUI入力値を読み直すため、速度指令、角度規約、ゼロ角、エンコーダ遅延、位相トリム、再生速度を変更しながら回転、電流、dqベクトルの変化を確認できる。

シミュレータの動作確認は、`-Model dq`、MAD MOTOR 4006 200kV推定モデル、`-BatteryV 24`、`-AngleMode raw_neg_add`、`-Zero 0`、`-EncoderDirection -1`、`-EncoderDelaySteps 1`、`-PhaseTrimDeg 0` を標準条件にする。まず `±5rps` と `±20rps` で正逆差とM0/M1差を見て、次に角度規約、エンコーダ遅延、位相進角トリムを個別に振る。確認対象は `speed_rps`、`theta_mech_rad`、`theta_elec_actual_rad`、`Id/Iq`、`uq_eff_v`、`torque_nm` である。GUIではdq電流ベクトルと機械角/電気角ベクトルを合わせて見て、角度遅れ、逆起電力、`Id` 増加の関係を確認する。ゼロ角誤差は標準条件へ混ぜず、`-Zero 0.1`、`0.5`、`1.2` などを指定する感度試験として別枠で扱う。再現条件を残す場合はCSV出力付きCLIを使い、代表例として `rt_raw_neg_add_z0_p5/m5/p20/m20.csv` を保存する。

## クロック設定
本ブランチでは常温評価用のオーバークロック設定として、HSE 8MHzからPLL x12でSYSCLK/HCLKを96MHzにする。APB1は24MHz、APB2は48MHzで動作させる。USART1はAPB2由来の2Mbpsで、APB2=48MHzでは分周が整数になりやすい。

TIM1/TIM8はHCLKをクロック源にし、Prescaler=1、Period=1600、`TIM_PWM_CENTER=800` とする。これによりPWM/制御割り込みは30kHzになり、M0/M1を交互に更新するため各モーターのFOC更新は15kHzになる。1kHzメインループの基準は `INTERRUPT_KHZ_1MS=30` とする。キャリブレーションのISRカウント値は20kHz時の実時間を維持するため1.5倍へ補正する。

SPI1はAPB2=48MHz、Prescaler=/4により12MHzで動作する。MT6835のSPI上限25MHz以内であり、Mode 3、8bitデータサイズ、MSB firstで使用する。1回の角度更新はburst command 2byteと角度/status/CRC 4byteの計6byteである。

ADC34の非同期クロックはPLL直結を避け、PLL/2の48MHzにする。ADC1/2は従来通りPCLK同期/2で動作するため、APB2=48MHzでは24MHzになる。

CANはAPB1=24MHzに合わせ、Prescaler=3、BS1=5TQ、BS2=2TQ、SJW=1TQで1Mbpsにする。旧72MHz設定はAPB1=36MHz、Prescaler=4、BS1=4TQ、BS2=4TQで1Mbpsだったため、CAN設定を戻すと96MHz設定では約667kbpsへ落ちる。

## TIM割り込み高速化
30kHz制御周期への移行を見据え、TIM割り込み内の処理はHAL呼び出しを避ける。ADC injected結果は `JDRx` を直接読み、次回変換開始は `ISR` のJEOC/JEOSをクリアして `CR.JADSTART` を立てる。電流値は毎割り込み更新するが、温度、バッテリ電圧、ゲートドライバ電圧は `ADC_SLOW_SAMPLE_INTERVAL=8` で間引いてコピーし、電圧換算値はキャッシュする。

通常FOC出力は `ud=0` の電圧FOC専用パスを使う。汎用の相電圧構造体生成と角度正規化を通さず、`uq`、電気角、バッテリ電圧からPWM比較値を直接作る。`fast_sin()` は負角でもテーブルmaskで折り返せる実装にし、通常制御では `focNormalizeAngle()` を避ける。

MT6835の角度差ピーク記録は診断用であり、通常の速度計算には使わない。割り込み負荷を下げるため、`diff_enc` は毎回更新しつつ、`diff_min/max` のabs比較は16回に1回へ間引く。

## 診断と安全チェック
UART `i` で非回転I/Oチェックを実行する。実行時は速度指令と出力電圧を0にし、PWMをフリーウィールへ落としてから、スイッチ、ADC raw/換算値、MT6835 raw/status/CRC、PWM CCR/CCER/BDTR、CAN/Flash状態を出力する。実機で回転指令を入れる前のベンチ確認に使う。

UART出力関数 `p()` は `vsnprintf()` でバッファ境界を確認する。キャリブレーションやI/Oチェックの連続ログでUART DMA用バッファを壊さないことを優先する。

UARTログはUSART1 TX DMAの二重バッファで送る。送信完了割り込みと `p()` が重なった場合でも、DMA送信を開始する経路では必ず対応する `sending_first_buf` / `sending_second_buf` を立て、送信長はローカルへ退避してからバッファ長を0へ戻す。フラグと実DMA送信状態がずれると、次のログが送信中バッファへ追記され、改行欠落や行連結のような壊れ方になるため、この順序を崩さない。

電流検出は `getCurrentMotor()` が従来通り瞬時値を返し、過電流保護や起動時チェックはこの値を使う。効率計算や調整ログ用には `getCurrentMotorAverage()` を追加し、ADC更新時に `alpha=0.001` のIIR平均を更新する。通常ログの `CS` は瞬時値、`Avg` はノイズを落とした平均値である。平均値は診断用途であり、保護判定へは使わない。

MT6835は通常角度更新と診断更新のどちらもCRC付きburst readを使う。診断読み出し中は短時間だけ割り込みを止め、PWM ISR側のSPIアクセスと競合しないようにする。`last_frame`、受信CRC、計算CRC、status、CRCエラー数、statusエラー数、低電圧数を保持する。CRC不一致またはstatus非ゼロ時は角度を更新せず、直前の正常値を維持する。連続異常が30回に達した場合は `BLDC_ENC_ERROR` で保護停止する。

## MT6835エンコーダ

- AS5047Pと同じSPI1配線（SCK/MISO/MOSIおよびM0/M1個別CS）へ接続する。通信プロトコルは互換ではないため、AS5047Pを同じSPIバスへ混在させない。
- CSはPB7がM0、PB6がM1で、非選択時High、選択時Lowとする。
- burst readはcommand `0xA0`、開始レジスタ`0x03`を送信し、21bit角度、3bit status、CRC-8を受信する。
- CRC-8は多項式 `x^8 + x^2 + x + 1`（poly `0x07`）、初期値`0x00`で、角度/statusの3byteを対象とする。
- 21bit角度は下位5bitを落としてから読み出し時に反転し、既存AS5047Pと同じ回転方向の`0..65535`へ変換する。速度、CAN角度、FOC角度のスケールと符号規約は従来と同じである。
- status bitはbit0=過速度、bit1=磁界弱、bit2=電源低電圧として保持する。status非ゼロ時は角度更新を拒否し、連続異常が30回に達した場合は保護停止する。
- センサ交換後はエンコーダゼロ角が変わるため、回転指令を入れる前にUART `i` でraw変化、CRCエラー0、status 0を確認し、その後エンコーダ校正を再実行する。

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

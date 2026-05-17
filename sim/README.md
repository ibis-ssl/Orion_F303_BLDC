# BLDC + Encoder Simulation

このディレクトリは、実機へ入れる前にFOCの角度規約、電圧変換、エンコーダ入力、簡易モータ応答をPC上で確認するためのシミュレーション環境である。

## 目的

- `Uq/Ud -> 三相電圧` のFOC数式を確認する。
- AS5047P raw値、電気角、ゼロ角校正値の規約を比較する。
- 低速では回るが速度を上げると崩れるような、角度誤差・位相進角不足の症状を実機なしで再現する。
- 実機の `[FOC CHECK]` ログから、角度候補とCCR候補をPC上で再計算する。

## 実行

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -SelfTest
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -Scenario conventions
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -Scenario step -AngleMode raw_neg_add -OutputCsv sim\out\step_raw_neg_add.csv
```

実機ログのスナップショット確認:

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -Scenario snapshot -Raw 5108 -Zero 4.796 -LegacyElec 5.202 -UqV -2.0 -BatteryV 25.7
```

## 角度モード

- `raw_pos_add`: `raw+ + zero`
- `raw_pos_sub`: `raw+ - zero`
- `raw_neg_add`: `raw- + zero`
- `raw_neg_sub`: `raw- - zero`
- `legacy`: 現ブランチの `as5047p.output_radian + zero_calib` 相当

## モデル

最初の切り分けを目的に、電流モデルは持たず、FOC出力三相電圧を実ロータ電気角でdq変換し直した `uq_effective` をトルク相当値として扱う。

```text
torque = Kt_per_volt * uq_effective - damping * omega - load_torque
omega += torque / inertia * dt
theta += omega * dt
encoder_raw = theta を 0..65535 へ量子化
```

このモデルはモータ定数の精密同定ではなく、符号、位相、ゼロ角規約、進角の妥当性確認に使う。

## 実環境模擬の拡張方針

実機ではマイコン上でデバッグ機能を使いにくく、実時間でしか動作しないため、PC上では次の要素を分離して確認できるようにする。

1. FOC数式:
   - `Uq/Ud` から三相電圧を作る。
   - 実ロータ電気角でdqへ戻し、指令角と実角のずれを `ud_eff` / `uq_eff` で見る。

2. 電気モデル:
   - `R`, `Ld`, `Lq`, 永久磁石鎖交磁束 `psi_f` を持つSPMSM相当のdqモデルを使う。
   - `id` / `iq` を状態量として持ち、逆起電力と電流応答遅れを再現する。
   - 高速域で電圧が足りない、位相進角が必要になる、という現象を確認する。

3. 機械モデル:
   - 慣性 `J`, 粘性抵抗 `B`, クーロン摩擦, 負荷トルクを持つ。
   - M0/M1の個体差は、まず `R/L/psi_f/J/B/friction/load` の差として表現する。
   - 初期慣性は、直径10cm、質量1kgの中実円柱相当として `J = 1/2*m*r^2 = 1.25e-3 kg m^2` とする。

4. エンコーダモデル:
   - 機械角からAS5047P raw相当の `0..65535` を生成する。
   - 量子化、センサ方向、サンプル遅延、ノイズを持つ。
   - raw差分による速度推定や、1サンプル古い角度をFOCへ入れた場合の影響を確認する。

5. 実行周期モデル:
   - 20kHz PWM ISRと1kHz main loopを別周期として扱う。
   - 今後、M0/M1交互更新、ADC/SPI更新順、UART診断による遅延を追加する。

## dqモデル

`-Model dq` を指定すると、`uq_effective` 直結トルクではなく、dq電気方程式で `id/iq` を更新する。

```text
di_d/dt = (u_d - R*i_d + omega_e*Lq*i_q) / Ld
di_q/dt = (u_q - R*i_q - omega_e*(Ld*i_d + psi_f)) / Lq
torque = 1.5 * pole_pairs * (psi_f*i_q + (Ld - Lq)*i_d*i_q)
```

## MAD MOTOR 4006 200kVカスタムモデル

dqモデルの初期値は、MAD MOTOR 4006の200kVカスタムモデル相当として設定する。

公開されているMAD 4006 EEE標準モデルは320kV/380kV/740kVであり、200kVモデルの抵抗・インダクタンス値は公開値として確認できない。そのため、最も近い低kV標準モデルである320kVを基準に、巻数が `Kv` に反比例する前提で換算する。

抵抗はMAD 4006 EEE 320kVの相間抵抗 `280mohm` を基準にする。dqモデルで使う値は相抵抗なので、相間抵抗の半分を相抵抗とし、200kV化による巻数比で `R_phase = 0.280 / 2 * (320 / 200)^2 = 0.3584 ohm` とする。

インダクタンスはMAD 4006標準モデルの公開値が見つからないため、既存シミュレータの標準値 `80uH` を320kV標準相当の仮定値として扱い、同じ巻数二乗比例で `L_phase = 80uH * (320 / 200)^2 = 204.8uH` と推定する。SPMSM相当として `Ld = Lq = 204.8uH` とする。

永久磁石鎖交磁束は200kVからSI単位のトルク定数を求め、`psi_f = Kt / (1.5 * pole_pairs)` として `0.00265Wb` にする。この値は線間逆起電力定数の厳密な測定値ではなく、シミュレータで速度起電力とトルク定数の整合を取るための初期推定である。

実行例:

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -Scenario step -Model dq -AngleMode raw_neg_add -SpeedRps 20 -DurationSec 0.5 -OutputCsv sim\out\dq_step.csv
```

初期パラメータは仮値であり、実機同定値ではない。重要なのは、パラメータを振ったときに症状がどう変わるかを見ることである。

## 実行周期モデル

`-Scenario realtime` は、実機の制御周期に近い形で次を模擬する。

- 20kHz ISR相当の `dt = 50us`
- 1kHz main loop相当の速度電圧指令更新
- ISRごとのM0/M1交互PWM更新
- 各モーターのPWM出力保持
- エンコーダサンプル遅延

実行例:

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -Scenario realtime -Model dq -AngleMode raw_neg_add -SpeedRpsM0 20 -SpeedRpsM1 20 -DurationSec 0.1 -EncoderDelaySteps 1
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -Scenario realtime -Model dq -AngleMode raw_neg_add -SpeedRpsM0 20 -SpeedRpsM1 20 -DurationSec 0.1 -OutputCsv sim\out\realtime.csv
```

CSVは1kHz main loop相当のタイミングでM0/M1を2行ずつ出力する。`speed_rps`, `encoder_raw`, `cmd_elec_rad`, `uq_eff_v`, `id_a`, `iq_a`, `torque_nm`, 三相電圧を確認できる。

## GUI

CLIは維持したまま、TkinterベースのGUIを `sim/sim_gui.py` に置く。外部ライブラリは不要で、GUIは `bldc_sim.py` のシナリオ関数を直接呼び出す。

起動:

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\run_sim_gui.ps1
```

GUIでできること:

- `conventions` / `step` / `snapshot` / `realtime` の実行
- `simple` / `dq` モデル切替
- 角度規約、速度、ゼロ角、エンコーダ遅延、位相トリムの変更
- Snapshot用のraw/zero/legacy角/Uq/電源電圧の入力
- 結果サマリのテーブル表示
- `step` / `realtime` 結果の時系列プロット表示
- 速度、q軸有効電圧、q軸電流、トルクを同時表示
- M0/M1の表示対象切替
- dq軸上の `Id/Iq` 電流ベクトル表示
- 機械角と実ロータ電気角のベクトル表示
- `Live Start` / `Live Stop` / `Live Reset` による低速ライブ実行
- ライブ再生速度の変更
- CSV保存先指定
- 同じ条件を再現するCLIコマンドのコピー

GUIは解析の入口であり、再現性を残す場合は `Copy CLI` でCLIコマンドを保存する。

ライブ実行は、実機20kHz ISR相当の1ステップ `50us` をGUI上で進める。`Live speed %` はシミュレーション時間が実時間に対して何%で進むかを指定する値で、初期値は `0.1%` である。低速設定ではGUI tick間隔を伸ばし、高速設定では1回のGUI tickで複数ステップをまとめて進める。実行中も速度指令、角度規約、ゼロ角、エンコーダ遅延、位相トリムなどの入力値を変更できる。エンコーダは機械角を参照し、M0/M1のPWM出力は前回値を保持したまま物理モデルを進める。GUIでは機械角 `theta_mech_rad` と、極対数を掛けた実ロータ電気角 `theta_elec_actual_rad` を円上のベクトルとして表示する。

## 動作確認条件

シミュレータの動作確認は、角度規約、dq電流モデル、実機相当スケジューラの影響を分けて見る。

標準条件:

- `-Model dq`
- MAD MOTOR 4006 200kVカスタム推定モデル
- `-BatteryV 24`
- `-AngleMode raw_neg_add`
- `-Zero 0`
- `-EncoderDirection -1`
- `-EncoderDelaySteps 1`
- `-PhaseTrimDeg 0`
- M0/M1初期角は `0.3rad` / `0.6rad`
- GUIライブ再生は最初 `0.1%`、確認後に `1%`、`10%` へ上げる

確認する値:

- `speed_rps`: 指令方向と実回転方向、M0/M1差、正逆差を見る。
- `theta_mech_rad`: 機械角が連続し、エンコーダrawと対応することを見る。
- `theta_elec_actual_rad`: 機械角の12倍として周期的に回ることを見る。
- `Id/Iq`: 主に `Iq` が出て、`Id` が不自然に大きくならないことを見る。
- `uq_eff_v` / `torque_nm`: 指令方向と符号が一致し、速度上昇に伴う逆起電力の影響が出ることを見る。

代表テスト:

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -SelfTest
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -Scenario realtime -Model dq -AngleMode raw_neg_add -Zero 0 -SpeedRpsM0 5 -SpeedRpsM1 5 -DurationSec 0.2 -EncoderDelaySteps 1 -OutputCsv sim\out\rt_raw_neg_add_z0_p5.csv
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -Scenario realtime -Model dq -AngleMode raw_neg_add -Zero 0 -SpeedRpsM0 -5 -SpeedRpsM1 -5 -DurationSec 0.2 -EncoderDelaySteps 1 -OutputCsv sim\out\rt_raw_neg_add_z0_m5.csv
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -Scenario realtime -Model dq -AngleMode raw_neg_add -Zero 0 -SpeedRpsM0 20 -SpeedRpsM1 20 -DurationSec 0.2 -EncoderDelaySteps 1 -OutputCsv sim\out\rt_raw_neg_add_z0_p20.csv
powershell -ExecutionPolicy Bypass -File .\Script\run_sim.ps1 -Scenario realtime -Model dq -AngleMode raw_neg_add -Zero 0 -SpeedRpsM0 -20 -SpeedRpsM1 -20 -DurationSec 0.2 -EncoderDelaySteps 1 -OutputCsv sim\out\rt_raw_neg_add_z0_m20.csv
```

比較テスト:

- 角度規約: `raw_neg_add`、`raw_neg_sub`、`raw_pos_add`、`raw_pos_sub` を同じ速度条件で比較する。
- エンコーダ遅延: `-EncoderDelaySteps 0`, `1`, `2`, `4` を比較する。
- 位相進角: `-PhaseTrimDeg -5`, `0`, `5` を比較する。
- ゼロ角誤差: 標準条件では `-Zero 0` とし、`-Zero 0.1`, `0.5`, `1.2` は正逆差の感度確認として別枠で扱う。

GUI確認では、時系列プロットだけでなく、dq電流ベクトルと機械角/電気角ベクトルを見て、角度遅れ、逆起電力、`Id` 増加の関係を確認する。

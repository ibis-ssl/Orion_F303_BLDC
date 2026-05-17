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

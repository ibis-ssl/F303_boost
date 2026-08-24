# 昇圧シミュレーション

`boost_sim.py`は、`Core/Src/main.c`の昇圧PWMスケジュールを再現し、10 µHインダクタから680 µF×2のコンデンサへ充電する過程を計算します。

## 実行

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\run_boost_sim.ps1
```

出力:

- `sim/out/boost_simulation.png`: 電圧、コンデンサ充電電力、ピーク電流、充電時間、平均充電電力、PWMスケジュールのプロット。
- `sim/out/boost_loss_simulation.png`: FET導通損失、コイル銅損、合計モデル損失、部分効率のプロット。
- `sim/out/boost_control_comparison.png`: 段階Duty、連続Duty、±20%電圧ノイズ、平滑化制御の比較プロット。
- `sim/out/boost_control_comparison_report.md`: 連続Dutyと50 seed Monte Carloの日本語評価結果。
- `sim/out/boost_simulation_report.md`: 結果表と解釈上の注意をまとめた日本語レポート。
- `sim/out/boost_simulation_summary.csv`: 入力電圧と`PWM_CNT`を変え、解析を2秒まで延長した比較結果。
- `sim/out/boost_simulation_baseline.csv`: 22.2 V、`PWM_CNT=700`の1 ms時系列。

## モデルと制約

- タイマクロック72 MHz、固定オン時間`PWM_CNT / 72 MHz`、電圧帯別ARR係数はFWと同じです。
- 1 ms周期のFW制御と1秒の充電タイムアウトを再現します。
- 入力電圧／`PWM_CNT`比較では、1秒を超えた場合の必要充電時間を確認するため解析だけ2秒まで継続し、現行1秒タイムアウトと比較します。
- オン区間とオフ区間をRL回路として解析し、オフ区間のインダクタ電流をコンデンサへ積算します。
- 出力電力は負荷電力ではなく、1 msごとのコンデンサ蓄積エネルギー変化`d(1/2 C V^2)/dt`として計算します。
- 680 µF×2は並列（合計1,360 µF）を既定値としています。
- SiC FETのRDS(on)は0.08 Ω、コイル抵抗は0.02 Ω、その他オフ経路抵抗は0.02 Ωの仮値です。
- ダイオード順方向電圧は仮値1.5 Vです。
- インダクタンスの電流依存性、コア損、スイッチング損、ESR、電圧検出誤差、バッテリ内部抵抗は含みません。

FET損失はRDS(on)による導通損失、コイル損失は銅損だけを計算します。FETのスイッチング損、ゲート駆動損、コイルのコア損、ダイオード損は含みません。

連続Duty案は、現行の各電圧帯中央におけるDutyを線形補間します。ノイズ比較では取得電圧へ毎回一様分布の±20%ノイズを加えます。ノイズ対策案は1 msごとに32サンプルを平均し、EMA係数0.30で平滑化した電圧を使用します。平滑化後電圧が3回連続で目標以上になった場合に停止します。Monte Carlo比較は50個の乱数seedで行います。

したがって、絶対的な充電時間よりも、入力電圧、`PWM_CNT`、電圧帯切替による相対変化の確認に使用してください。実測へ合わせる場合は`-FetResistanceOhm`、`-CoilResistanceOhm`、`-OffPathResistanceOhm`、`-DiodeDropV`だけでなく、インダクタの実効インダクタンスとバッテリ電圧降下も確認してください。

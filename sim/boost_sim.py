"""昇圧チョッパのFWパラメータとコンデンサ充電挙動を検討する。"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, replace
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


@dataclass(frozen=True)
class BoostParameters:
    timer_hz: float = 72_000_000.0
    pwm_counts: int = 700
    inductance_h: float = 10e-6
    capacitance_f: float = 2 * 680e-6
    fet_resistance_ohm: float = 0.08
    coil_resistance_ohm: float = 0.02
    off_path_resistance_ohm: float = 0.02
    diode_drop_v: float = 1.5
    control_period_s: float = 1e-3
    timeout_s: float = 1.0
    target_v: float = 450.0

    @property
    def on_resistance_ohm(self) -> float:
        return self.fet_resistance_ohm + self.coil_resistance_ohm

    @property
    def off_resistance_ohm(self) -> float:
        return self.coil_resistance_ohm + self.off_path_resistance_ohm


@dataclass(frozen=True)
class ControlProfile:
    schedule: str = "stepped"
    noise_fraction: float = 0.0
    filter_alpha: float = 1.0
    stop_confirm_samples: int = 1
    stop_threshold_scale: float = 1.0
    random_seed: int = 1
    measurement_average_samples: int = 1


@dataclass
class SimulationResult:
    input_v: float
    params: BoostParameters
    time_s: np.ndarray
    capacitor_v: np.ndarray
    output_power_w: np.ndarray
    fet_loss_power_w: np.ndarray
    coil_loss_power_w: np.ndarray
    control_voltage_v: np.ndarray
    peak_current_a: np.ndarray
    frequency_hz: np.ndarray
    duty: np.ndarray
    reached_target: bool
    charge_time_s: float | None
    final_voltage_v: float
    maximum_peak_current_a: float
    average_output_power_w: float
    average_fet_loss_power_w: float
    average_coil_loss_power_w: float
    maximum_overshoot_v: float


def period_multiplier(voltage_v: float) -> float:
    """main.cのboostControl()と同じ電圧帯別ARR係数を返す。"""
    if voltage_v < 50.0:
        return 10.0
    if voltage_v < 100.0:
        return 3.0
    if voltage_v < 200.0:
        return 1.5
    if voltage_v < 300.0:
        return 1.4
    if voltage_v < 400.0:
        return 1.3
    return 1.25


def continuous_duty(voltage_v: float, params: BoostParameters) -> float:
    """現行各電圧帯の中央Dutyを線形補間した連続制御値を返す。"""
    anchor_v = np.asarray((0.0, 25.0, 75.0, 150.0, 250.0, 350.0, 425.0, 500.0))
    anchor_duty = np.asarray(tuple(
        params.pwm_counts / (int(params.pwm_counts * multiplier) + 1)
        for multiplier in (10.0, 10.0, 3.0, 1.5, 1.4, 1.3, 1.25, 1.25)
    ))
    return float(np.interp(voltage_v, anchor_v, anchor_duty))


def pwm_timing(voltage_v: float, params: BoostParameters, schedule: str = "stepped") -> tuple[float, float, float, float]:
    if schedule == "stepped":
        arr = int(params.pwm_counts * period_multiplier(voltage_v))
    elif schedule == "continuous":
        duty_request = continuous_duty(voltage_v, params)
        arr = max(params.pwm_counts, round(params.pwm_counts / duty_request) - 1)
    else:
        raise ValueError(f"Unsupported schedule: {schedule}")
    period_s = (arr + 1) / params.timer_hz
    on_s = params.pwm_counts / params.timer_hz
    off_s = period_s - on_s
    return period_s, on_s, off_s, on_s / period_s


def rl_step(current_a: float, source_v: float, duration_s: float, resistance_ohm: float, inductance_h: float) -> tuple[float, float, float]:
    """一定電圧のRL区間を進め、終了電流、電流時間積、電流二乗時間積を返す。"""
    if duration_s <= 0.0:
        return current_a, 0.0, 0.0
    decay_rate = resistance_ohm / inductance_h
    decay = math.exp(-resistance_ohm * duration_s / inductance_h)
    steady_a = source_v / resistance_ohm
    transient_a = current_a - steady_a
    end_a = steady_a + transient_a * decay
    integral_as = steady_a * duration_s + transient_a * (1.0 - decay) / decay_rate
    integral_i2 = (
        steady_a * steady_a * duration_s
        + 2.0 * steady_a * transient_a * (1.0 - decay) / decay_rate
        + transient_a * transient_a * (1.0 - decay * decay) / (2.0 * decay_rate)
    )
    return end_a, integral_as, max(0.0, integral_i2)


def simulate(input_v: float, params: BoostParameters, control: ControlProfile = ControlProfile()) -> SimulationResult:
    capacitor_v = input_v
    inductor_current_a = 0.0
    time_s = 0.0
    charge_time_s: float | None = None
    enabled = True
    rng = np.random.default_rng(control.random_seed)
    control_voltage_v = input_v
    stop_confirm_count = 0

    times = [0.0]
    voltages = [capacitor_v]
    control_voltages = [control_voltage_v]
    peaks = [0.0]
    fet_losses = [0.0]
    coil_losses = [0.0]
    frequencies = [0.0]
    duties = [0.0]

    while times[-1] < params.timeout_s - 1e-12:
        sample_time_s = times[-1]
        noise_samples = rng.uniform(
            -control.noise_fraction,
            control.noise_fraction,
            size=max(1, control.measurement_average_samples),
        )
        measured_voltage_v = capacitor_v * (1.0 + float(np.mean(noise_samples)))
        control_voltage_v += control.filter_alpha * (measured_voltage_v - control_voltage_v)
        if control_voltage_v >= params.target_v * control.stop_threshold_scale:
            stop_confirm_count += 1
        else:
            stop_confirm_count = 0
        if stop_confirm_count >= control.stop_confirm_samples:
            enabled = False
            if charge_time_s is None:
                charge_time_s = sample_time_s

        segment_end_s = min(sample_time_s + params.control_period_s, params.timeout_s)
        segment_peak_a = inductor_current_a
        segment_fet_loss_j = 0.0
        segment_coil_loss_j = 0.0
        segment_frequency_hz = 0.0
        segment_duty = 0.0

        if enabled:
            period_s, on_s, off_s, duty = pwm_timing(control_voltage_v, params, control.schedule)
            segment_frequency_hz = 1.0 / period_s
            segment_duty = duty

            # PWMタイマは1 ms制御境界でも連続動作するため、境界をまたぐ
            # 1周期を許可して周期端数を毎回捨てない。
            while time_s < segment_end_s - 1e-15:
                inductor_current_a, _, on_i2_as = rl_step(
                    inductor_current_a,
                    input_v,
                    on_s,
                    params.on_resistance_ohm,
                    params.inductance_h,
                )
                segment_fet_loss_j += on_i2_as * params.fet_resistance_ohm
                segment_coil_loss_j += on_i2_as * params.coil_resistance_ohm
                segment_peak_a = max(segment_peak_a, inductor_current_a)

                off_source_v = input_v - capacitor_v - params.diode_drop_v
                steady_off_a = off_source_v / params.off_resistance_ohm
                conduct_s = off_s
                if steady_off_a < 0.0 and inductor_current_a > 0.0:
                    ratio = -steady_off_a / (inductor_current_a - steady_off_a)
                    if 0.0 < ratio < 1.0:
                        zero_crossing_s = -math.log(ratio) * params.inductance_h / params.off_resistance_ohm
                        conduct_s = min(conduct_s, zero_crossing_s)

                end_current_a, transferred_charge_c, off_i2_as = rl_step(
                    inductor_current_a,
                    off_source_v,
                    conduct_s,
                    params.off_resistance_ohm,
                    params.inductance_h,
                )
                segment_coil_loss_j += off_i2_as * params.coil_resistance_ohm
                capacitor_v += max(0.0, transferred_charge_c) / params.capacitance_f
                inductor_current_a = 0.0 if conduct_s < off_s else max(0.0, end_current_a)
                time_s += period_s

        else:
            inductor_current_a = 0.0
            time_s = max(time_s, segment_end_s)

        times.append(segment_end_s)
        voltages.append(capacitor_v)
        control_voltages.append(control_voltage_v)
        peaks.append(segment_peak_a)
        fet_losses.append(segment_fet_loss_j / params.control_period_s)
        coil_losses.append(segment_coil_loss_j / params.control_period_s)
        frequencies.append(segment_frequency_hz)
        duties.append(segment_duty)

    voltage_array = np.asarray(voltages)
    time_array = np.asarray(times)
    capacitor_energy_j = 0.5 * params.capacitance_f * voltage_array**2
    output_power_w = np.zeros_like(capacitor_energy_j)
    output_power_w[1:] = np.diff(capacitor_energy_j) / np.diff(time_array)
    reached = charge_time_s is not None
    final_voltage_v = float(voltages[-1])
    active_duration_s = charge_time_s if charge_time_s is not None else params.timeout_s
    average_output_power_w = float((capacitor_energy_j[-1] - capacitor_energy_j[0]) / active_duration_s)
    active_samples = max(1, round(active_duration_s / params.control_period_s))
    average_fet_loss_power_w = float(np.mean(fet_losses[1 : active_samples + 1]))
    average_coil_loss_power_w = float(np.mean(coil_losses[1 : active_samples + 1]))
    return SimulationResult(
        input_v=input_v,
        params=params,
        time_s=time_array,
        capacitor_v=voltage_array,
        output_power_w=output_power_w,
        fet_loss_power_w=np.asarray(fet_losses),
        coil_loss_power_w=np.asarray(coil_losses),
        control_voltage_v=np.asarray(control_voltages),
        peak_current_a=np.asarray(peaks),
        frequency_hz=np.asarray(frequencies),
        duty=np.asarray(duties),
        reached_target=reached,
        charge_time_s=charge_time_s,
        final_voltage_v=final_voltage_v,
        maximum_peak_current_a=float(max(peaks)),
        average_output_power_w=average_output_power_w,
        average_fet_loss_power_w=average_fet_loss_power_w,
        average_coil_loss_power_w=average_coil_loss_power_w,
        maximum_overshoot_v=max(0.0, final_voltage_v - params.target_v) if reached else 0.0,
    )


def write_timeseries(path: Path, result: SimulationResult) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", "capacitor_v", "control_voltage_v", "output_power_w", "fet_conduction_loss_w", "coil_copper_loss_w", "peak_current_a", "frequency_hz", "duty_pct"))
        for row in zip(
            result.time_s,
            result.capacitor_v,
            result.control_voltage_v,
            result.output_power_w,
            result.fet_loss_power_w,
            result.coil_loss_power_w,
            result.peak_current_a,
            result.frequency_hz,
            result.duty * 100.0,
        ):
            writer.writerow(tuple(f"{value:.6f}" for value in row))


def write_summary(path: Path, results: list[SimulationResult], firmware_timeout_s: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(("input_v", "pwm_counts", "target_v", "reached_within_1s", "charge_time_s", "final_voltage_v", "average_output_power_w", "average_fet_conduction_loss_w", "average_coil_copper_loss_w", "max_peak_current_a", "overshoot_v"))
        for result in results:
            writer.writerow((
                f"{result.input_v:.1f}",
                result.params.pwm_counts,
                f"{result.params.target_v:.1f}",
                int(result.charge_time_s is not None and result.charge_time_s <= firmware_timeout_s),
                "" if result.charge_time_s is None else f"{result.charge_time_s:.6f}",
                f"{result.final_voltage_v:.3f}",
                f"{result.average_output_power_w:.3f}",
                f"{result.average_fet_loss_power_w:.3f}",
                f"{result.average_coil_loss_power_w:.3f}",
                f"{result.maximum_peak_current_a:.3f}",
                f"{result.maximum_overshoot_v:.3f}",
            ))


def write_report(path: Path, input_results: list[SimulationResult], sweep_results: list[SimulationResult], firmware_timeout_s: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# 昇圧シミュレーション結果",
        "",
        "既定条件は10 µH、1,360 µF、目標450 V、制御タイムアウト1秒です。FET RDS(on) 0.08 Ω、コイル抵抗0.02 Ω、その他オフ経路抵抗0.02 Ω、ダイオード降下1.5 Vは仮値です。",
        "",
        "## 現行PWM_CNT=700",
        "",
        "| 入力電圧 | 450 V到達時間 | 1秒後電圧 | 平均充電電力 | FET導通損失 | コイル銅損 | 最大ピーク電流 |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for result in input_results:
        charge = f"{result.charge_time_s:.3f} s" if result.charge_time_s is not None else "未到達"
        lines.append(
            f"| {result.input_v:.1f} V | {charge} | {result.final_voltage_v:.1f} V | {result.average_output_power_w:.1f} W | "
            f"{result.average_fet_loss_power_w:.1f} W | {result.average_coil_loss_power_w:.1f} W | {result.maximum_peak_current_a:.1f} A |"
        )

    lines.extend((
        "",
        "## PWM_CNT比較",
        "",
        "| PWM_CNT | 入力電圧 | 450 V到達時間 | 現行1秒以内 | 平均コンデンサ充電電力 | 最大インダクタピーク電流 |",
        "| ---: | ---: | ---: | :---: | ---: | ---: |",
    ))
    for result in sweep_results:
        charge = f"{result.charge_time_s:.3f} s" if result.charge_time_s is not None else "2秒で未到達"
        within_timeout = "○" if result.charge_time_s is not None and result.charge_time_s <= firmware_timeout_s else "×"
        lines.append(
            f"| {result.params.pwm_counts} | {result.input_v:.1f} V | {charge} | "
            f"{within_timeout} | {result.average_output_power_w:.1f} W | {result.maximum_peak_current_a:.1f} A |"
        )

    lines.extend((
        "",
        "## 解釈上の注意",
        "",
        "- 最大ピーク電流はシミュレーション上のインダクタ電流です。FWの25 A保護はADCで取得したバッテリ電流なので、直接比較できません。",
        "- 低い入力電圧で未到達になる主因は、1秒の`boost_cnt`タイムアウトです。",
        "- 絶対値を確定するには、インダクタの電流依存L値、各経路抵抗、ダイオード特性、バッテリ電圧降下を実測値へ置き換えてください。",
        "- FET損失はRDS(on)による導通損失のみ、コイル損失は銅損のみです。スイッチング損、コア損、ダイオード損は含みません。",
        "",
    ))
    path.write_text("\n".join(lines), encoding="utf-8")


def create_plot(path: Path, input_results: list[SimulationResult], sweep_results: list[SimulationResult]) -> None:
    plt.style.use("seaborn-v0_8-whitegrid")
    fig, axes = plt.subplots(3, 2, figsize=(14, 13), constrained_layout=True)

    voltage_ax, power_ax, current_ax, sweep_ax, average_power_ax, timing_ax = axes.flat
    for result in input_results:
        label = f"Vin={result.input_v:g} V"
        voltage_ax.plot(result.time_s, result.capacitor_v, linewidth=1.8, label=label)
        power_ax.plot(result.time_s, result.output_power_w, linewidth=1.4, label=label)
        current_ax.plot(result.time_s, result.peak_current_a, linewidth=1.5, label=label)
    voltage_ax.axhline(input_results[0].params.target_v, color="black", linestyle="--", linewidth=1.0, label="Target 450 V")
    voltage_ax.set(title="Capacitor charging (PWM_CNT=700)", xlabel="Time [s]", ylabel="Capacitor voltage [V]", xlim=(0, 1.0))
    voltage_ax.legend(loc="lower right")
    power_ax.set(title="Output power into capacitor (PWM_CNT=700)", xlabel="Time [s]", ylabel="d(½CV²)/dt [W]", xlim=(0, 1.0))
    power_ax.legend(loc="upper right")
    current_ax.axhline(25.0, color="black", linestyle="--", linewidth=1.0, label="25 A reference (FW sensor is not Ipk)")
    current_ax.set(title="Calculated inductor peak current", xlabel="Time [s]", ylabel="Peak current [A]", xlim=(0, 1.0))
    current_ax.legend(loc="upper right")

    pwm_values = sorted({result.params.pwm_counts for result in sweep_results})
    input_values = sorted({result.input_v for result in sweep_results})
    for pwm_counts in pwm_values:
        group = [result for result in sweep_results if result.params.pwm_counts == pwm_counts]
        group.sort(key=lambda item: item.input_v)
        final_or_time = [result.charge_time_s if result.reached_target else result.params.timeout_s for result in group]
        marker = "o" if pwm_counts == 700 else "s" if pwm_counts < 700 else "^"
        sweep_ax.plot([item.input_v for item in group], final_or_time, marker=marker, linewidth=1.7, label=f"PWM_CNT={pwm_counts}")
    sweep_ax.axhline(1.0, color="black", linestyle="--", linewidth=1.0, label="FW timeout 1.0 s")
    sweep_ax.set(title="Required charge time to 450 V", xlabel="Input voltage [V]", ylabel="Charge time [s]", ylim=(0.5, 1.45))
    sweep_ax.legend(loc="upper right")

    for pwm_counts in pwm_values:
        group = [result for result in sweep_results if result.params.pwm_counts == pwm_counts]
        group.sort(key=lambda item: item.input_v)
        marker = "o" if pwm_counts == 700 else "s" if pwm_counts < 700 else "^"
        average_power_ax.plot(
            [item.input_v for item in group],
            [item.average_output_power_w for item in group],
            marker=marker,
            linewidth=1.7,
            label=f"PWM_CNT={pwm_counts}",
        )
    average_power_ax.set(title="Average capacitor charging power to 450 V", xlabel="Input voltage [V]", ylabel="Average output power [W]")
    average_power_ax.legend(loc="upper left")

    voltages = np.linspace(0.0, 470.0, 471)
    frequencies = []
    duties = []
    params = input_results[0].params
    for voltage in voltages:
        period_s, _, _, duty = pwm_timing(float(voltage), params)
        frequencies.append(1.0 / period_s / 1000.0)
        duties.append(duty * 100.0)
    timing_ax.step(voltages, frequencies, where="post", linewidth=1.8, label="Frequency [kHz]")
    duty_axis = timing_ax.twinx()
    duty_axis.step(voltages, duties, where="post", linewidth=1.8, linestyle="--", color="tab:orange", label="Duty [%]")
    timing_ax.set(title="FW voltage-band PWM schedule", xlabel="Capacitor voltage [V]", ylabel="Frequency [kHz]", xlim=(0, 470))
    duty_axis.set_ylabel("Duty [%]")
    lines = timing_ax.get_lines() + duty_axis.get_lines()
    timing_ax.legend(lines, [line.get_label() for line in lines], loc="lower right")

    fig.suptitle("F303_boost chopper simulation — 10 µH, 1360 µF, target 450 V", fontsize=15)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def create_loss_plot(path: Path, input_results: list[SimulationResult]) -> None:
    plt.style.use("seaborn-v0_8-whitegrid")
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    fet_ax, coil_ax, total_ax, efficiency_ax = axes.flat
    for result in input_results:
        label = f"Vin={result.input_v:g} V"
        total_loss = result.fet_loss_power_w + result.coil_loss_power_w
        denominator = result.output_power_w + total_loss
        efficiency = np.divide(
            result.output_power_w * 100.0,
            denominator,
            out=np.full_like(denominator, np.nan),
            where=denominator > 1e-9,
        )
        fet_ax.plot(result.time_s, result.fet_loss_power_w, linewidth=1.5, label=label)
        coil_ax.plot(result.time_s, result.coil_loss_power_w, linewidth=1.5, label=label)
        total_ax.plot(result.time_s, total_loss, linewidth=1.5, label=label)
        efficiency_ax.plot(result.time_s, efficiency, linewidth=1.5, label=label)

    fet_ax.set(title="FET conduction loss (RDS(on)=80 mΩ)", xlabel="Time [s]", ylabel="Loss [W]", xlim=(0, 1.0))
    coil_ax.set(title="Inductor copper loss (Rcoil=20 mΩ)", xlabel="Time [s]", ylabel="Loss [W]", xlim=(0, 1.0))
    total_ax.set(title="FET + inductor modeled loss", xlabel="Time [s]", ylabel="Loss [W]", xlim=(0, 1.0))
    efficiency_ax.set(title="Partial efficiency (excludes switching/core/diode loss)", xlabel="Time [s]", ylabel="Efficiency [%]", xlim=(0, 1.0), ylim=(80, 100))
    for axis in axes.flat:
        axis.legend(loc="best")
    fig.suptitle("F303_boost modeled conduction and copper losses — PWM_CNT=700", fontsize=15)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def create_control_comparison_plot(
    path: Path,
    cases: list[tuple[str, SimulationResult]],
    raw_monte_carlo: list[SimulationResult],
    filtered_monte_carlo: list[SimulationResult],
) -> None:
    plt.style.use("seaborn-v0_8-whitegrid")
    fig, axes = plt.subplots(3, 2, figsize=(14, 13), constrained_layout=True)
    voltage_ax, duty_ax, power_ax, sensed_ax, final_ax, stop_ax = axes.flat
    for label, result in cases:
        voltage_ax.plot(result.time_s, result.capacitor_v, linewidth=1.6, label=label)
        duty_ax.plot(result.time_s, result.duty * 100.0, linewidth=1.3, label=label)
        power_ax.plot(result.time_s, result.output_power_w, linewidth=1.3, label=label)
        if "noise" in label.lower():
            sensed_ax.plot(result.time_s, result.control_voltage_v, linewidth=1.0, label=label)

    target_v = cases[0][1].params.target_v
    voltage_ax.axhline(target_v, color="black", linestyle="--", linewidth=1.0, label="Target 450 V")
    sensed_ax.axhline(target_v, color="black", linestyle="--", linewidth=1.0, label="Stop threshold")
    voltage_ax.set(title="Actual capacitor voltage", xlabel="Time [s]", ylabel="Voltage [V]", xlim=(0, 1.5))
    duty_ax.set(title="Commanded duty", xlabel="Time [s]", ylabel="Duty [%]", xlim=(0, 1.5), ylim=(0, 85))
    power_ax.set(title="Output power into capacitor", xlabel="Time [s]", ylabel="d(½CV²)/dt [W]", xlim=(0, 1.5))
    sensed_ax.set(title="Voltage used by noisy controllers", xlabel="Time [s]", ylabel="Control voltage [V]", xlim=(0, 1.5))
    for axis in (voltage_ax, duty_ax, power_ax, sensed_ax):
        axis.legend(loc="best")

    final_data = [[result.final_voltage_v for result in raw_monte_carlo], [result.final_voltage_v for result in filtered_monte_carlo]]
    stop_data = [
        [result.charge_time_s if result.charge_time_s is not None else result.params.timeout_s for result in raw_monte_carlo],
        [result.charge_time_s if result.charge_time_s is not None else result.params.timeout_s for result in filtered_monte_carlo],
    ]
    final_ax.boxplot(final_data, tick_labels=("Raw ±20%", "32× avg + EMA"), showmeans=True)
    final_ax.axhline(target_v, color="black", linestyle="--", linewidth=1.0)
    final_ax.set(title="Monte Carlo final voltage (50 seeds)", ylabel="Final voltage [V]")
    stop_ax.boxplot(stop_data, tick_labels=("Raw ±20%", "32× avg + EMA"), showmeans=True)
    stop_ax.axhline(1.0, color="black", linestyle="--", linewidth=1.0, label="FW timeout 1.0 s")
    stop_ax.set(title="Monte Carlo PWM stop time (50 seeds)", ylabel="Stop time [s]")
    stop_ax.legend(loc="best")

    fig.suptitle("Stepped vs continuous duty control at Vin=22.2 V", fontsize=15)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def write_control_report(
    path: Path,
    cases: list[tuple[str, SimulationResult]],
    raw_monte_carlo: list[SimulationResult],
    filtered_monte_carlo: list[SimulationResult],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# 連続Duty・電圧ノイズ評価",
        "",
        "条件はVin=22.2 V、PWM_CNT=700、電圧ノイズは各ADCサンプルに一様分布±20%を付加しています。",
        "",
        "| 制御方式 | PWM停止時刻 | 最終実電圧 | 平均充電電力 | 最大Duty |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for label, result in cases:
        stop = f"{result.charge_time_s:.3f} s" if result.charge_time_s is not None else "1.5秒で未停止"
        lines.append(
            f"| {label} | {stop} | {result.final_voltage_v:.1f} V | "
            f"{result.average_output_power_w:.1f} W | {result.duty.max() * 100.0:.1f}% |"
        )

    lines.extend(("", "## Monte Carlo（50 seed）", "", "| ノイズ処理 | 最終電圧 平均 | 最小 | 最大 | PWM停止時刻 平均 |", "| --- | ---: | ---: | ---: | ---: |"))
    for label, results in (("生の±20%", raw_monte_carlo), ("32サンプル平均＋EMA＋3回確認", filtered_monte_carlo)):
        final_values = np.asarray([result.final_voltage_v for result in results])
        stop_times = np.asarray([
            result.charge_time_s if result.charge_time_s is not None else result.params.timeout_s
            for result in results
        ])
        lines.append(
            f"| {label} | {np.mean(final_values):.1f} V | {np.min(final_values):.1f} V | "
            f"{np.max(final_values):.1f} V | {np.mean(stop_times):.3f} s |"
        )

    lines.extend((
        "",
        "## 評価",
        "",
        "- 連続Dutyはノイズなしでは段階制御とほぼ同じ充電時間になります。",
        "- 生の±20%電圧を停止判定へ使うと、単発の正ノイズで450 V未満の早期停止が発生します。",
        "- 32サンプル平均、EMA係数0.30、3回連続確認の組み合わせは、今回の独立一様ノイズモデルでは最終電圧のばらつきを大幅に抑えます。",
        "- 実機ノイズがスイッチング位相と相関する場合、単純平均の効果はこのモデルより小さくなるため、PWM同期サンプリングまたは短時間PWM停止後の確認測定も検討してください。",
        "",
    ))
    path.write_text("\n".join(lines), encoding="utf-8")


def comparison_metrics(result: SimulationResult) -> dict[str, float]:
    duration_s = result.charge_time_s if result.charge_time_s is not None else result.params.timeout_s
    modeled_loss_w = result.average_fet_loss_power_w + result.average_coil_loss_power_w
    denominator_w = result.average_output_power_w + modeled_loss_w
    return {
        "charge_time_s": duration_s,
        "output_power_w": result.average_output_power_w,
        "fet_loss_w": result.average_fet_loss_power_w,
        "coil_loss_w": result.average_coil_loss_power_w,
        "partial_efficiency_pct": result.average_output_power_w / denominator_w * 100.0,
        "modeled_loss_energy_j": modeled_loss_w * duration_s,
        "final_voltage_v": result.final_voltage_v,
    }


def build_method_comparison(base: BoostParameters, input_voltages: tuple[float, ...], monte_carlo_seeds: int = 20) -> dict[str, list[dict[str, float]]]:
    comparison_params = replace(base, timeout_s=2.0)
    series: dict[str, list[dict[str, float]]] = {
        "Stepped / clean": [],
        "Continuous / clean": [],
        "Stepped / filtered noise": [],
        "Continuous / filtered noise": [],
    }
    robust_profile_args = {
        "noise_fraction": 0.20,
        "measurement_average_samples": 32,
        "filter_alpha": 0.30,
        "stop_confirm_samples": 3,
    }
    for input_v in input_voltages:
        for label, schedule in (("Stepped / clean", "stepped"), ("Continuous / clean", "continuous")):
            result = simulate(input_v, comparison_params, ControlProfile(schedule=schedule))
            series[label].append({"input_v": input_v, **comparison_metrics(result)})

        for label, schedule in (("Stepped / filtered noise", "stepped"), ("Continuous / filtered noise", "continuous")):
            seed_metrics = [
                comparison_metrics(
                    simulate(
                        input_v,
                        comparison_params,
                        ControlProfile(schedule=schedule, random_seed=seed, **robust_profile_args),
                    )
                )
                for seed in range(monte_carlo_seeds)
            ]
            averaged = {
                key: float(np.mean([metrics[key] for metrics in seed_metrics]))
                for key in seed_metrics[0]
            }
            series[label].append({"input_v": input_v, **averaged})
    return series


def create_method_comparison_plot(path: Path, series: dict[str, list[dict[str, float]]]) -> None:
    plt.style.use("seaborn-v0_8-whitegrid")
    fig, axes = plt.subplots(3, 2, figsize=(14, 13), constrained_layout=True)
    definitions = (
        ("output_power_w", "Average capacitor charging power", "Power [W]"),
        ("fet_loss_w", "Average FET conduction loss", "Loss [W]"),
        ("coil_loss_w", "Average inductor copper loss", "Loss [W]"),
        ("partial_efficiency_pct", "Partial efficiency", "Efficiency [%]"),
        ("charge_time_s", "Charge time to 450 V", "Time [s]"),
        ("modeled_loss_energy_j", "Modeled loss energy per charge", "Loss energy [J]"),
    )
    styles = {
        "Stepped / clean": ("o", "-"),
        "Continuous / clean": ("s", "-"),
        "Stepped / filtered noise": ("o", "--"),
        "Continuous / filtered noise": ("s", "--"),
    }
    for axis, (metric, title, ylabel) in zip(axes.flat, definitions):
        for label, rows in series.items():
            marker, linestyle = styles[label]
            axis.plot(
                [row["input_v"] for row in rows],
                [row[metric] for row in rows],
                marker=marker,
                linestyle=linestyle,
                linewidth=1.6,
                label=label,
            )
        axis.set(title=title, xlabel="Input voltage [V]", ylabel=ylabel)
        axis.legend(loc="best")
    fig.suptitle("Current stepped output vs continuous duty — PWM_CNT=700", fontsize=15)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def write_method_comparison_csv(path: Path, series: dict[str, list[dict[str, float]]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = (
        "method",
        "input_v",
        "charge_time_s",
        "output_power_w",
        "fet_loss_w",
        "coil_loss_w",
        "partial_efficiency_pct",
        "modeled_loss_energy_j",
        "final_voltage_v",
    )
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for label, rows in series.items():
            for row in rows:
                writer.writerow({"method": label, **{key: f"{row[key]:.6f}" for key in fieldnames[1:]}})


def write_method_comparison_report(path: Path, series: dict[str, list[dict[str, float]]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# 現行段階出力と連続Dutyの比較",
        "",
        "FET損失は導通損失、コイル損失は銅損のみです。部分効率にはスイッチング損、コア損、ダイオード損を含みません。ノイズあり条件は±20%独立一様ノイズを32サンプル平均し、EMA 0.30と3回連続確認を適用した20 seed平均です。",
        "",
        "## Vin=22.2 V",
        "",
        "| 方式 | 充電時間 | 平均出力 | FET損失 | コイル損失 | 部分効率 | 充電1回のモデル損失 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for label, rows in series.items():
        row = next(item for item in rows if abs(item["input_v"] - 22.2) < 1e-6)
        lines.append(
            f"| {label} | {row['charge_time_s']:.3f} s | {row['output_power_w']:.1f} W | "
            f"{row['fet_loss_w']:.2f} W | {row['coil_loss_w']:.2f} W | "
            f"{row['partial_efficiency_pct']:.2f}% | {row['modeled_loss_energy_j']:.2f} J |"
        )

    stepped = next(item for item in series["Stepped / clean"] if abs(item["input_v"] - 22.2) < 1e-6)
    continuous = next(item for item in series["Continuous / clean"] if abs(item["input_v"] - 22.2) < 1e-6)
    lines.extend((
        "",
        "## 差分",
        "",
        f"- ノイズなし連続Dutyは現行段階制御に対し、充電時間が{(continuous['charge_time_s'] / stepped['charge_time_s'] - 1.0) * 100.0:+.2f}%、平均出力が{(continuous['output_power_w'] / stepped['output_power_w'] - 1.0) * 100.0:+.2f}%です。",
        f"- FET導通損失は{(continuous['fet_loss_w'] / stepped['fet_loss_w'] - 1.0) * 100.0:+.2f}%、コイル銅損は{(continuous['coil_loss_w'] / stepped['coil_loss_w'] - 1.0) * 100.0:+.2f}%です。",
        f"- 部分効率差は{continuous['partial_efficiency_pct'] - stepped['partial_efficiency_pct']:+.3f}ポイント、充電1回のモデル損失エネルギー差は{continuous['modeled_loss_energy_j'] - stepped['modeled_loss_energy_j']:+.3f} Jです。",
        "- このモデルでは両方式の差は小さく、連続Duty化の主な価値は効率改善よりも電力変化の平滑化です。ノイズ対策のADC処理と停止判定追加が必須になります。",
        "",
    ))
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="F303_boostの昇圧PWM・コンデンサ充電シミュレーション")
    parser.add_argument("--output-dir", type=Path, default=Path("sim/out"))
    parser.add_argument("--target-v", type=float, default=450.0)
    parser.add_argument("--capacitance-uf", type=float, default=1360.0)
    parser.add_argument("--inductance-uh", type=float, default=10.0)
    parser.add_argument("--fet-resistance-ohm", type=float, default=0.08)
    parser.add_argument("--coil-resistance-ohm", type=float, default=0.02)
    parser.add_argument("--off-path-resistance-ohm", type=float, default=0.02)
    parser.add_argument("--diode-drop-v", type=float, default=1.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    base = BoostParameters(
        target_v=args.target_v,
        capacitance_f=args.capacitance_uf * 1e-6,
        inductance_h=args.inductance_uh * 1e-6,
        fet_resistance_ohm=args.fet_resistance_ohm,
        coil_resistance_ohm=args.coil_resistance_ohm,
        off_path_resistance_ohm=args.off_path_resistance_ohm,
        diode_drop_v=args.diode_drop_v,
    )
    input_voltages = (18.0, 20.0, 22.2, 25.2)
    input_results = [simulate(input_v, base) for input_v in input_voltages]
    firmware_timeout_s = base.timeout_s
    sweep_results = [
        simulate(input_v, replace(base, pwm_counts=pwm_counts, timeout_s=2.0))
        for pwm_counts in (650, 700, 750)
        for input_v in input_voltages
    ]

    comparison_params = replace(base, timeout_s=1.5)
    control_cases = [
        ("Stepped / clean", simulate(22.2, comparison_params)),
        ("Continuous / clean", simulate(22.2, comparison_params, ControlProfile(schedule="continuous"))),
        (
            "Continuous / raw noise ±20%",
            simulate(22.2, comparison_params, ControlProfile(schedule="continuous", noise_fraction=0.20, random_seed=7)),
        ),
        (
            "Continuous / 32x avg + EMA noise ±20%",
            simulate(
                22.2,
                comparison_params,
                ControlProfile(
                    schedule="continuous",
                    noise_fraction=0.20,
                    measurement_average_samples=32,
                    filter_alpha=0.30,
                    stop_confirm_samples=3,
                    random_seed=7,
                ),
            ),
        ),
    ]
    raw_monte_carlo = [
        simulate(
            22.2,
            comparison_params,
            ControlProfile(schedule="continuous", noise_fraction=0.20, random_seed=seed),
        )
        for seed in range(50)
    ]
    filtered_monte_carlo = [
        simulate(
            22.2,
            comparison_params,
            ControlProfile(
                schedule="continuous",
                noise_fraction=0.20,
                measurement_average_samples=32,
                filter_alpha=0.30,
                stop_confirm_samples=3,
                random_seed=seed,
            ),
        )
        for seed in range(50)
    ]
    method_comparison = build_method_comparison(base, input_voltages, monte_carlo_seeds=20)

    output_dir = args.output_dir
    create_plot(output_dir / "boost_simulation.png", input_results, sweep_results)
    create_loss_plot(output_dir / "boost_loss_simulation.png", input_results)
    create_control_comparison_plot(output_dir / "boost_control_comparison.png", control_cases, raw_monte_carlo, filtered_monte_carlo)
    create_method_comparison_plot(output_dir / "boost_method_comparison.png", method_comparison)
    write_summary(output_dir / "boost_simulation_summary.csv", sweep_results, firmware_timeout_s)
    write_report(output_dir / "boost_simulation_report.md", input_results, sweep_results, firmware_timeout_s)
    write_control_report(output_dir / "boost_control_comparison_report.md", control_cases, raw_monte_carlo, filtered_monte_carlo)
    write_method_comparison_csv(output_dir / "boost_method_comparison.csv", method_comparison)
    write_method_comparison_report(output_dir / "boost_method_comparison_report.md", method_comparison)
    baseline = next(result for result in input_results if result.input_v == 22.2)
    write_timeseries(output_dir / "boost_simulation_baseline.csv", baseline)

    for result in sweep_results:
        charge = f"{result.charge_time_s:.3f}s" if result.charge_time_s is not None else f"timeout ({result.final_voltage_v:.1f}V)"
        print(f"Vin={result.input_v:4.1f}V PWM_CNT={result.params.pwm_counts}: {charge}, Ipk={result.maximum_peak_current_a:.1f}A")
    print(f"plot={output_dir / 'boost_simulation.png'}")
    for label, result in control_cases:
        stop = f"{result.charge_time_s:.3f}s" if result.charge_time_s is not None else "timeout"
        print(f"{label}: stop={stop}, final={result.final_voltage_v:.1f}V, duty_max={result.duty.max() * 100.0:.1f}%")
    print(f"loss_plot={output_dir / 'boost_loss_simulation.png'}")
    print(f"control_plot={output_dir / 'boost_control_comparison.png'}")
    print(f"method_plot={output_dir / 'boost_method_comparison.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

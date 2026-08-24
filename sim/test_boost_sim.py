"""昇圧シミュレーションのFW設定再現と基本傾向を検証する。"""

import unittest

from boost_sim import BoostParameters, ControlProfile, build_method_comparison, pwm_timing, simulate


class BoostSimulationTest(unittest.TestCase):
    def test_high_voltage_pwm_schedule_matches_firmware(self) -> None:
        params = BoostParameters(pwm_counts=700)
        period_s, on_s, off_s, duty = pwm_timing(450.0, params)
        self.assertAlmostEqual(period_s, 876.0 / 72_000_000.0)
        self.assertAlmostEqual(on_s, 700.0 / 72_000_000.0)
        self.assertAlmostEqual(off_s, 176.0 / 72_000_000.0)
        self.assertAlmostEqual(duty, 700.0 / 876.0)

    def test_voltage_band_boundaries_match_firmware(self) -> None:
        params = BoostParameters(pwm_counts=700)
        expected_arr = ((49.9, 7000), (50.0, 2100), (100.0, 1050), (200.0, 980), (300.0, 910), (400.0, 875))
        for voltage_v, arr in expected_arr:
            with self.subTest(voltage_v=voltage_v):
                period_s, _, _, _ = pwm_timing(voltage_v, params)
                self.assertAlmostEqual(period_s, (arr + 1) / params.timer_hz)

    def test_higher_input_voltage_charges_faster(self) -> None:
        params = BoostParameters()
        low = simulate(20.0, params)
        high = simulate(25.2, params)
        self.assertGreater(high.final_voltage_v, low.final_voltage_v)
        self.assertTrue(high.reached_target)
        self.assertIsNotNone(high.charge_time_s)
        self.assertGreater(high.average_output_power_w, low.average_output_power_w)
        self.assertGreater(float(high.output_power_w.max()), 0.0)
        self.assertGreater(high.average_fet_loss_power_w, 0.0)
        self.assertGreater(high.average_coil_loss_power_w, 0.0)

    def test_larger_pwm_count_increases_charge_rate_and_peak_current(self) -> None:
        low = simulate(22.2, BoostParameters(pwm_counts=650))
        high = simulate(22.2, BoostParameters(pwm_counts=750))
        self.assertIsNotNone(low.charge_time_s)
        self.assertIsNotNone(high.charge_time_s)
        self.assertLess(high.charge_time_s, low.charge_time_s)
        self.assertGreater(high.maximum_peak_current_a, low.maximum_peak_current_a)

    def test_continuous_control_noise_filter_reduces_false_stop(self) -> None:
        params = BoostParameters(timeout_s=1.5)
        raw = simulate(
            22.2,
            params,
            ControlProfile(schedule="continuous", noise_fraction=0.20, random_seed=7),
        )
        filtered = simulate(
            22.2,
            params,
            ControlProfile(
                schedule="continuous",
                noise_fraction=0.20,
                measurement_average_samples=32,
                filter_alpha=0.30,
                stop_confirm_samples=3,
                random_seed=7,
            ),
        )
        self.assertLess(raw.final_voltage_v, 420.0)
        self.assertGreater(filtered.final_voltage_v, 445.0)
        self.assertLess(filtered.final_voltage_v, 455.0)

    def test_continuous_and_stepped_clean_performance_are_close(self) -> None:
        comparison = build_method_comparison(BoostParameters(), (22.2,), monte_carlo_seeds=2)
        stepped = comparison["Stepped / clean"][0]
        continuous = comparison["Continuous / clean"][0]
        self.assertLess(abs(continuous["output_power_w"] / stepped["output_power_w"] - 1.0), 0.03)
        self.assertLess(abs(continuous["partial_efficiency_pct"] - stepped["partial_efficiency_pct"]), 0.3)


if __name__ == "__main__":
    unittest.main()

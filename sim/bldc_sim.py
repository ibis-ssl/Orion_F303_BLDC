#!/usr/bin/env python3
"""BLDC + encoder simulation for checking FOC angle and voltage conventions."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

TWO_PI = 2.0 * math.pi
SQRT3_2 = 0.8660254037844386
ENC_COUNT = 65536
POLE_PAIRS = 12


def normalize_angle(angle: float) -> float:
    return angle % TWO_PI


def angle_error(a: float, b: float) -> float:
    err = normalize_angle(a - b)
    if err > math.pi:
        err -= TWO_PI
    return err


def clamp(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def foc_set_phase_voltage_sine(uq: float, ud: float, angle_el: float, voltage_limit: float) -> tuple[float, float, float]:
    """Match Core/Src/foc_math.c:focSetPhaseVoltageSine()."""
    if voltage_limit <= 0.0:
        return 0.0, 0.0, 0.0

    half_limit = voltage_limit * 0.5
    uq = clamp(uq, half_limit)
    ud = clamp(ud, half_limit)
    angle_el = normalize_angle(angle_el)
    sin_el = math.sin(angle_el)
    cos_el = math.cos(angle_el)

    ualpha = cos_el * ud - sin_el * uq
    ubeta = sin_el * ud + cos_el * uq

    ua = ualpha + half_limit
    ub = -0.5 * ualpha + SQRT3_2 * ubeta + half_limit
    uc = -0.5 * ualpha - SQRT3_2 * ubeta + half_limit
    return ua, ub, uc


def phase_voltage_to_dq(ua: float, ub: float, uc: float, rotor_angle_el: float, voltage_supply: float) -> tuple[float, float]:
    """Convert phase voltages back to dq voltage using the actual rotor electrical angle."""
    half = voltage_supply * 0.5
    va = ua - half
    vb = ub - half
    vc = uc - half

    ualpha = va
    ubeta = (vb - vc) / math.sqrt(3.0)
    sin_el = math.sin(normalize_angle(rotor_angle_el))
    cos_el = math.cos(normalize_angle(rotor_angle_el))
    ud = cos_el * ualpha + sin_el * ubeta
    uq = -sin_el * ualpha + cos_el * ubeta
    return ud, uq


def encoder_raw_from_mech(theta_mech: float, direction: int = -1) -> int:
    sensed = theta_mech if direction >= 0 else -theta_mech
    return int(normalize_angle(sensed) * ENC_COUNT / TWO_PI) & 0xFFFF


def raw_to_mech_rad(raw: int) -> float:
    return float(raw & 0xFFFF) * TWO_PI / float(ENC_COUNT)


def raw_pos_electrical(raw: int) -> float:
    return normalize_angle(raw_to_mech_rad(raw) * POLE_PAIRS)


def raw_neg_electrical(raw: int) -> float:
    return normalize_angle(-raw_pos_electrical(raw))


def select_electrical_angle(raw: int, zero: float, mode: str, manual_offset: float = 0.0) -> float:
    raw_pos = raw_pos_electrical(raw)
    raw_neg = raw_neg_electrical(raw)
    if mode == "raw_pos_add":
        angle = raw_pos + zero
    elif mode == "raw_pos_sub":
        angle = raw_pos - zero
    elif mode == "raw_neg_add" or mode == "legacy":
        angle = raw_neg + zero
    elif mode == "raw_neg_sub":
        angle = raw_neg - zero
    else:
        raise ValueError(f"unknown angle mode: {mode}")
    return normalize_angle(angle + manual_offset)


@dataclass
class MotorSimConfig:
    voltage_supply: float = 24.0
    voltage_per_rps: float = 0.08
    voltage_limit: float = 12.0
    resistance_ohm: float = 0.28
    ld_h: float = 80.0e-6
    lq_h: float = 80.0e-6
    flux_wb: float = 0.006
    # Solid cylinder load, diameter 10cm and mass 1kg: J = 1/2*m*r^2.
    inertia: float = 1.25e-3
    damping: float = 1.0e-5
    coulomb_friction: float = 0.001
    kt_per_volt: float = 0.03
    load_torque: float = 0.0
    dt: float = 0.00005
    zero_electric_angle: float = 1.2
    encoder_direction: int = -1
    encoder_delay_steps: int = 0
    torque_sign: float = -1.0
    phase_advance_model_rad_per_rps: float = -0.003457
    phase_advance_trim_rad: float = 0.0


@dataclass
class MotorSimState:
    theta_mech: float = 0.0
    omega_rad_s: float = 0.0
    id_a: float = 0.0
    iq_a: float = 0.0

    @property
    def rps(self) -> float:
        return self.omega_rad_s / TWO_PI

    @property
    def electrical_angle(self) -> float:
        return normalize_angle(self.theta_mech * POLE_PAIRS)


class EncoderSim:
    def __init__(self, direction: int, delay_steps: int) -> None:
        self.direction = direction
        self.delay_steps = max(0, delay_steps)
        self.queue: list[int] = []

    def sample(self, theta_mech: float) -> int:
        raw_now = encoder_raw_from_mech(theta_mech, self.direction)
        self.queue.append(raw_now)
        if len(self.queue) <= self.delay_steps:
            return self.queue[0]
        return self.queue.pop(0)


@dataclass
class PwmHold:
    ua: float = 12.0
    ub: float = 12.0
    uc: float = 12.0
    cmd_elec: float = 0.0
    uq_cmd: float = 0.0


def phase_advance_for_speed(speed_rps: float, cfg: MotorSimConfig) -> float:
    model = cfg.phase_advance_model_rad_per_rps * abs(speed_rps)
    return clamp(model + cfg.phase_advance_trim_rad, math.pi * 0.25)


def coulomb_friction_torque(omega_rad_s: float, cfg: MotorSimConfig) -> float:
    if abs(omega_rad_s) < 1.0e-6:
        return 0.0
    return math.copysign(cfg.coulomb_friction, omega_rad_s)


def update_simple_motor(state: MotorSimState, cfg: MotorSimConfig, uq_eff: float) -> float:
    torque = (
        cfg.kt_per_volt * uq_eff
        - cfg.damping * state.omega_rad_s
        - coulomb_friction_torque(state.omega_rad_s, cfg)
        - cfg.load_torque
    )
    state.omega_rad_s += (torque / cfg.inertia) * cfg.dt
    state.theta_mech = normalize_angle(state.theta_mech + state.omega_rad_s * cfg.dt)
    return torque


def update_dq_motor(state: MotorSimState, cfg: MotorSimConfig, ud_eff: float, uq_eff: float) -> float:
    omega_e = state.omega_rad_s * POLE_PAIRS
    did = (ud_eff - cfg.resistance_ohm * state.id_a + omega_e * cfg.lq_h * state.iq_a) / cfg.ld_h
    diq = (
        uq_eff
        - cfg.resistance_ohm * state.iq_a
        - omega_e * (cfg.ld_h * state.id_a + cfg.flux_wb)
    ) / cfg.lq_h
    state.id_a += did * cfg.dt
    state.iq_a += diq * cfg.dt

    electromagnetic_torque = 1.5 * POLE_PAIRS * (
        cfg.flux_wb * state.iq_a + (cfg.ld_h - cfg.lq_h) * state.id_a * state.iq_a
    )
    shaft_torque = (
        electromagnetic_torque
        - cfg.damping * state.omega_rad_s
        - coulomb_friction_torque(state.omega_rad_s, cfg)
        - cfg.load_torque
    )
    state.omega_rad_s += (shaft_torque / cfg.inertia) * cfg.dt
    state.theta_mech = normalize_angle(state.theta_mech + state.omega_rad_s * cfg.dt)
    return electromagnetic_torque


def build_foc_pwm(
    cfg: MotorSimConfig,
    raw: int,
    angle_mode: str,
    speed_rps: float,
    voltage_cmd: float,
) -> PwmHold:
    electrical_cmd = select_electrical_angle(raw, cfg.zero_electric_angle, angle_mode)
    advance = phase_advance_for_speed(speed_rps, cfg)
    electrical_cmd = normalize_angle(electrical_cmd + (-advance if speed_rps < 0.0 else advance))
    uq_cmd = cfg.torque_sign * voltage_cmd
    ua, ub, uc = foc_set_phase_voltage_sine(uq_cmd, 0.0, electrical_cmd, cfg.voltage_supply)
    return PwmHold(ua=ua, ub=ub, uc=uc, cmd_elec=electrical_cmd, uq_cmd=uq_cmd)


def integrate_motor_from_pwm(state: MotorSimState, cfg: MotorSimConfig, pwm: PwmHold, model: str) -> tuple[float, float, float]:
    ud_eff, uq_eff = phase_voltage_to_dq(pwm.ua, pwm.ub, pwm.uc, state.electrical_angle, cfg.voltage_supply)
    if model == "dq":
        torque = update_dq_motor(state, cfg, ud_eff, uq_eff)
    else:
        torque = update_simple_motor(state, cfg, uq_eff)
    return ud_eff, uq_eff, torque


def simulate_step(
    cfg: MotorSimConfig,
    angle_mode: str,
    speed_rps: float,
    duration_s: float,
    model: str,
    initial_theta: float = 0.3,
) -> list[dict[str, float]]:
    state = MotorSimState(theta_mech=initial_theta, omega_rad_s=0.0)
    encoder = EncoderSim(cfg.encoder_direction, cfg.encoder_delay_steps)
    rows: list[dict[str, float]] = []
    steps = int(duration_s / cfg.dt)
    sample_interval = max(1, int(0.001 / cfg.dt))

    for step in range(steps):
        raw = encoder.sample(state.theta_mech)
        voltage_cmd = clamp(speed_rps * cfg.voltage_per_rps, cfg.voltage_limit)
        pwm = build_foc_pwm(cfg, raw, angle_mode, speed_rps, voltage_cmd)
        ud_eff, uq_eff, torque = integrate_motor_from_pwm(state, cfg, pwm, model)

        if step % sample_interval == 0:
            rows.append({
                "time_s": step * cfg.dt,
                "theta_mech_rad": state.theta_mech,
                "theta_elec_actual_rad": state.electrical_angle,
                "encoder_raw": raw,
                "cmd_elec_rad": pwm.cmd_elec,
                "speed_rps": state.rps,
                "target_rps": speed_rps,
                "uq_cmd_v": pwm.uq_cmd,
                "ud_eff_v": ud_eff,
                "uq_eff_v": uq_eff,
                "id_a": state.id_a,
                "iq_a": state.iq_a,
                "torque_nm": torque,
                "ua_v": pwm.ua,
                "ub_v": pwm.ub,
                "uc_v": pwm.uc,
            })

    return rows


def make_realtime_row(
    time_s: float,
    motor: int,
    state: MotorSimState,
    raw: int,
    pwm: PwmHold,
    ud_eff: float,
    uq_eff: float,
    torque: float,
    target_rps: float,
    voltage_cmd: float,
) -> dict[str, float]:
    return {
        "time_s": time_s,
        "motor": motor,
        "theta_mech_rad": state.theta_mech,
        "theta_elec_actual_rad": state.electrical_angle,
        "encoder_raw": raw,
        "cmd_elec_rad": pwm.cmd_elec,
        "speed_rps": state.rps,
        "target_rps": target_rps,
        "voltage_cmd_v": voltage_cmd,
        "uq_cmd_v": pwm.uq_cmd,
        "ud_eff_v": ud_eff,
        "uq_eff_v": uq_eff,
        "id_a": state.id_a,
        "iq_a": state.iq_a,
        "torque_nm": torque,
        "ua_v": pwm.ua,
        "ub_v": pwm.ub,
        "uc_v": pwm.uc,
    }


def simulate_realtime(args: argparse.Namespace) -> list[dict[str, float]]:
    cfg = MotorSimConfig(
        zero_electric_angle=args.zero,
        encoder_direction=args.encoder_direction,
        encoder_delay_steps=args.encoder_delay_steps,
        phase_advance_trim_rad=math.radians(args.phase_trim_deg),
    )
    states = [
        MotorSimState(theta_mech=args.initial_theta_m0, omega_rad_s=0.0),
        MotorSimState(theta_mech=args.initial_theta_m1, omega_rad_s=0.0),
    ]
    encoders = [
        EncoderSim(cfg.encoder_direction, cfg.encoder_delay_steps),
        EncoderSim(cfg.encoder_direction, cfg.encoder_delay_steps),
    ]
    pwms = [PwmHold(cfg.voltage_supply * 0.5, cfg.voltage_supply * 0.5, cfg.voltage_supply * 0.5),
            PwmHold(cfg.voltage_supply * 0.5, cfg.voltage_supply * 0.5, cfg.voltage_supply * 0.5)]
    target_rps = [args.speed_rps_m0, args.speed_rps_m1]
    voltage_cmd = [0.0, 0.0]
    raw_last = [encoder_raw_from_mech(states[0].theta_mech, cfg.encoder_direction),
                encoder_raw_from_mech(states[1].theta_mech, cfg.encoder_direction)]
    ud_last = [0.0, 0.0]
    uq_last = [0.0, 0.0]
    torque_last = [0.0, 0.0]
    rows: list[dict[str, float]] = []

    steps = int(args.duration_s / cfg.dt)
    isr_per_main = max(1, int(round(0.001 / cfg.dt)))
    motor_select = 0

    for step in range(steps):
        if step % isr_per_main == 0:
            for m in range(2):
                voltage_cmd[m] = clamp(target_rps[m] * cfg.voltage_per_rps, cfg.voltage_limit)

        motor_select ^= 1
        raw_last[motor_select] = encoders[motor_select].sample(states[motor_select].theta_mech)
        pwms[motor_select] = build_foc_pwm(
            cfg,
            raw_last[motor_select],
            args.angle_mode,
            target_rps[motor_select],
            voltage_cmd[motor_select],
        )

        for m in range(2):
            ud_last[m], uq_last[m], torque_last[m] = integrate_motor_from_pwm(states[m], cfg, pwms[m], args.model)

        if step % isr_per_main == 0:
            time_s = step * cfg.dt
            for m in range(2):
                rows.append(make_realtime_row(
                    time_s,
                    m,
                    states[m],
                    raw_last[m],
                    pwms[m],
                    ud_last[m],
                    uq_last[m],
                    torque_last[m],
                    target_rps[m],
                    voltage_cmd[m],
                ))

    return rows


def summarize_rows(rows: list[dict[str, float]]) -> dict[str, float]:
    tail = rows[len(rows) // 2:] if rows else []
    if not tail:
        return {"final_rps": 0.0, "avg_uq_eff": 0.0, "avg_abs_ud_eff": 0.0}
    return {
        "final_rps": rows[-1]["speed_rps"],
        "avg_uq_eff": sum(row["uq_eff_v"] for row in tail) / len(tail),
        "avg_abs_ud_eff": sum(abs(row["ud_eff_v"]) for row in tail) / len(tail),
        "avg_iq_a": sum(row["iq_a"] for row in tail) / len(tail),
        "avg_torque_nm": sum(row["torque_nm"] for row in tail) / len(tail),
    }


def write_csv(rows: list[dict[str, float]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
      writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
      writer.writeheader()
      writer.writerows(rows)


def run_snapshot(args: argparse.Namespace) -> int:
    modes = ["raw_pos_add", "raw_pos_sub", "raw_neg_add", "raw_neg_sub", "legacy"]
    actual_el = None if math.isnan(args.actual_elec) else normalize_angle(args.actual_elec)
    legacy_el = None if math.isnan(args.legacy_elec) else normalize_angle(args.legacy_elec)

    print("mode,cmd_elec_rad,err_to_legacy_rad,ud_eff_v,uq_eff_v,ccr_a,ccr_b,ccr_c")
    for mode in modes:
        cmd_el = select_electrical_angle(args.raw, args.zero, mode)
        ua, ub, uc = foc_set_phase_voltage_sine(args.uq_v, args.ud_v, cmd_el, args.battery_v)
        ccr_a = int(ua * args.pwm_period / args.battery_v)
        ccr_b = int(ub * args.pwm_period / args.battery_v)
        ccr_c = int(uc * args.pwm_period / args.battery_v)

        err = 0.0 if legacy_el is None else angle_error(cmd_el, legacy_el)
        if actual_el is None:
            ud_eff = 0.0
            uq_eff = 0.0
        else:
            ud_eff, uq_eff = phase_voltage_to_dq(ua, ub, uc, actual_el, args.battery_v)

        print(f"{mode},{cmd_el:+.6f},{err:+.6f},{ud_eff:+.6f},{uq_eff:+.6f},{ccr_a},{ccr_b},{ccr_c}")
    return 0


def run_conventions(args: argparse.Namespace) -> int:
    cfg = MotorSimConfig(
        zero_electric_angle=args.zero,
        encoder_direction=args.encoder_direction,
        encoder_delay_steps=args.encoder_delay_steps,
        phase_advance_trim_rad=math.radians(args.phase_trim_deg),
    )
    modes = ["raw_pos_add", "raw_pos_sub", "raw_neg_add", "raw_neg_sub", "legacy"]
    print("mode,final_rps,avg_uq_eff,avg_abs_ud_eff,avg_iq_a,avg_torque_nm")
    for mode in modes:
        rows = simulate_step(cfg, mode, args.speed_rps, args.duration_s, args.model)
        summary = summarize_rows(rows)
        print(
            f"{mode},{summary['final_rps']:+.4f},{summary['avg_uq_eff']:+.4f},"
            f"{summary['avg_abs_ud_eff']:+.4f},{summary['avg_iq_a']:+.4f},{summary['avg_torque_nm']:+.6f}"
        )
    return 0


def run_step(args: argparse.Namespace) -> int:
    cfg = MotorSimConfig(
        zero_electric_angle=args.zero,
        encoder_direction=args.encoder_direction,
        encoder_delay_steps=args.encoder_delay_steps,
        phase_advance_trim_rad=math.radians(args.phase_trim_deg),
    )
    rows = simulate_step(cfg, args.angle_mode, args.speed_rps, args.duration_s, args.model)
    summary = summarize_rows(rows)
    if args.output_csv:
        write_csv(rows, Path(args.output_csv))
    print(
        f"angle_mode {args.angle_mode} final_rps {summary['final_rps']:+.4f} "
        f"avg_uq_eff {summary['avg_uq_eff']:+.4f} avg_abs_ud_eff {summary['avg_abs_ud_eff']:+.4f} "
        f"avg_iq {summary['avg_iq_a']:+.4f} avg_torque {summary['avg_torque_nm']:+.6f}"
    )
    return 0


def summarize_realtime_rows(rows: list[dict[str, float]]) -> dict[int, dict[str, float]]:
    result: dict[int, dict[str, float]] = {}
    for motor in (0, 1):
        motor_rows = [row for row in rows if int(row["motor"]) == motor]
        tail = motor_rows[len(motor_rows) // 2:] if motor_rows else []
        if not tail:
            result[motor] = {"final_rps": 0.0, "avg_uq_eff": 0.0, "avg_iq_a": 0.0, "avg_torque_nm": 0.0}
            continue
        result[motor] = {
            "final_rps": motor_rows[-1]["speed_rps"],
            "avg_uq_eff": sum(row["uq_eff_v"] for row in tail) / len(tail),
            "avg_iq_a": sum(row["iq_a"] for row in tail) / len(tail),
            "avg_torque_nm": sum(row["torque_nm"] for row in tail) / len(tail),
        }
    return result


def run_realtime(args: argparse.Namespace) -> int:
    rows = simulate_realtime(args)
    if args.output_csv:
        write_csv(rows, Path(args.output_csv))

    summary = summarize_realtime_rows(rows)
    print("motor,final_rps,avg_uq_eff,avg_iq_a,avg_torque_nm")
    for motor in (0, 1):
        s = summary[motor]
        print(f"M{motor},{s['final_rps']:+.4f},{s['avg_uq_eff']:+.4f},{s['avg_iq_a']:+.4f},{s['avg_torque_nm']:+.6f}")
    return 0


def run_self_test() -> int:
    cfg = MotorSimConfig(zero_electric_angle=0.0, encoder_direction=-1, damping=0.005, coulomb_friction=0.0)
    rows_ok = simulate_step(cfg, "raw_neg_add", 20.0, 0.8, "simple")
    rows_bad = simulate_step(cfg, "raw_pos_add", 20.0, 0.8, "simple")
    rows_dq = simulate_step(cfg, "raw_neg_add", 20.0, 0.05, "dq")
    realtime_args = argparse.Namespace(
        zero=0.0,
        encoder_direction=-1,
        encoder_delay_steps=1,
        phase_trim_deg=0.0,
        initial_theta_m0=0.3,
        initial_theta_m1=0.6,
        speed_rps_m0=20.0,
        speed_rps_m1=20.0,
        duration_s=0.02,
        angle_mode="raw_neg_add",
        model="dq",
    )
    rows_rt = simulate_realtime(realtime_args)
    ok = summarize_rows(rows_ok)
    bad = summarize_rows(rows_bad)
    dq = summarize_rows(rows_dq)

    phase = foc_set_phase_voltage_sine(2.0, 0.0, 0.0, 24.0)
    ud_eff, uq_eff = phase_voltage_to_dq(*phase, rotor_angle_el=0.0, voltage_supply=24.0)
    checks = [
        ("sine_pwm_uq", uq_eff > 1.9 and abs(ud_eff) < 0.01),
        ("angle_mode_expected_response", abs(ok["final_rps"]) > 0.5),
        ("wrong_zero_convention_differs", abs(ok["avg_uq_eff"] - bad["avg_uq_eff"]) > 0.5),
        ("dq_current_response", abs(dq["avg_iq_a"]) > 0.1),
        ("realtime_scheduler_rows", len(rows_rt) >= 20),
    ]
    for name, passed in checks:
        print(f"{name}: {'OK' if passed else 'NG'}")
    return 0 if all(passed for _, passed in checks) else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true", help="run built-in simulation checks")
    parser.add_argument("--scenario", choices=["conventions", "step", "snapshot", "realtime"], default="conventions")
    parser.add_argument("--model", choices=["simple", "dq"], default="simple")
    parser.add_argument("--angle-mode", choices=["raw_pos_add", "raw_pos_sub", "raw_neg_add", "raw_neg_sub", "legacy"], default="raw_neg_add")
    parser.add_argument("--speed-rps", type=float, default=20.0)
    parser.add_argument("--speed-rps-m0", type=float, default=20.0)
    parser.add_argument("--speed-rps-m1", type=float, default=20.0)
    parser.add_argument("--duration-s", type=float, default=1.0)
    parser.add_argument("--zero", type=float, default=1.2)
    parser.add_argument("--encoder-direction", type=int, choices=[-1, 1], default=-1)
    parser.add_argument("--encoder-delay-steps", type=int, default=0)
    parser.add_argument("--phase-trim-deg", type=float, default=0.0)
    parser.add_argument("--output-csv", default="")
    parser.add_argument("--raw", type=int, default=0)
    parser.add_argument("--legacy-elec", type=float, default=float("nan"))
    parser.add_argument("--actual-elec", type=float, default=float("nan"))
    parser.add_argument("--uq-v", type=float, default=-2.0)
    parser.add_argument("--ud-v", type=float, default=0.0)
    parser.add_argument("--battery-v", type=float, default=24.0)
    parser.add_argument("--pwm-period", type=int, default=1800)
    parser.add_argument("--initial-theta-m0", type=float, default=0.3)
    parser.add_argument("--initial-theta-m1", type=float, default=0.6)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        return run_self_test()
    if args.scenario == "snapshot":
        return run_snapshot(args)
    if args.scenario == "realtime":
        return run_realtime(args)
    if args.scenario == "step":
        return run_step(args)
    return run_conventions(args)


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Tkinter GUI for the BLDC simulation scenarios."""

from __future__ import annotations

import argparse
import contextlib
import io
import math
import os
import sys
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import bldc_sim


REPO_ROOT = Path(__file__).resolve().parents[1]


def parse_float(value: str, default: float = 0.0) -> float:
    text = value.strip()
    if text == "":
        return default
    if text.lower() == "nan":
        return float("nan")
    return float(text)


def parse_int(value: str, default: int = 0) -> int:
    text = value.strip()
    if text == "":
        return default
    return int(text)


class SimGui(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("BLDC FOC Simulator")
        self.geometry("1180x760")
        self.minsize(1000, 640)

        self.vars: dict[str, tk.StringVar] = {}
        self.last_rows: list[dict[str, float]] = []
        self.live_running = False
        self.live_after_id: str | None = None
        self.live_time_s = 0.0
        self.live_step = 0
        self.live_motor_select = 0
        self.live_states: list[bldc_sim.MotorSimState] = []
        self.live_encoders: list[bldc_sim.EncoderSim] = []
        self.live_pwms: list[bldc_sim.PwmHold] = []
        self.live_raw_last = [0, 0]
        self.live_voltage_cmd = [0.0, 0.0]
        self.live_ud_last = [0.0, 0.0]
        self.live_uq_last = [0.0, 0.0]
        self.live_torque_last = [0.0, 0.0]
        self._make_vars()
        self._build_ui()
        self._refresh_cli()

    def _make_vars(self) -> None:
        defaults = {
            "scenario": "realtime",
            "model": "dq",
            "angle_mode": "raw_neg_add",
            "speed_rps": "20.0",
            "speed_rps_m0": "20.0",
            "speed_rps_m1": "20.0",
            "duration_s": "0.05",
            "zero": "0.0",
            "encoder_direction": "-1",
            "encoder_delay_steps": "1",
            "phase_trim_deg": "0.0",
            "output_csv": "",
            "raw": "0",
            "legacy_elec": "nan",
            "actual_elec": "nan",
            "uq_v": "-2.0",
            "ud_v": "0.0",
            "battery_v": "24.0",
            "pwm_period": "1800",
            "initial_theta_m0": "0.3",
            "initial_theta_m1": "0.6",
            "plot_m0": "1",
            "plot_m1": "1",
            "live_status": "Live stopped",
        }
        for key, value in defaults.items():
            var = tk.StringVar(value=value)
            var.trace_add("write", lambda *_: self._refresh_cli())
            self.vars[key] = var

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        left = ttk.Frame(self, padding=8)
        left.grid(row=0, column=0, sticky="ns")

        right = ttk.Frame(self, padding=8)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)
        right.rowconfigure(2, weight=1)
        right.rowconfigure(3, weight=1)

        self._build_settings(left)
        self._build_actions(right)
        self._build_result_table(right)
        self._build_plots(right)
        self._build_log(right)

    def _build_settings(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(1, weight=1)
        row = 0

        row = self._section(parent, row, "Common")
        row = self._combo(parent, row, "Scenario", "scenario", ["conventions", "step", "snapshot", "realtime"])
        row = self._combo(parent, row, "Model", "model", ["simple", "dq"])
        row = self._combo(parent, row, "Angle", "angle_mode", ["raw_pos_add", "raw_pos_sub", "raw_neg_add", "raw_neg_sub", "legacy"])
        row = self._entry(parent, row, "Duration s", "duration_s")
        row = self._entry(parent, row, "Zero rad", "zero")
        row = self._combo(parent, row, "Enc dir", "encoder_direction", ["-1", "1"])
        row = self._entry(parent, row, "Enc delay", "encoder_delay_steps")
        row = self._entry(parent, row, "Phase trim deg", "phase_trim_deg")

        row = self._section(parent, row, "Step")
        row = self._entry(parent, row, "Speed rps", "speed_rps")

        row = self._section(parent, row, "Realtime")
        row = self._entry(parent, row, "M0 rps", "speed_rps_m0")
        row = self._entry(parent, row, "M1 rps", "speed_rps_m1")
        row = self._entry(parent, row, "M0 theta", "initial_theta_m0")
        row = self._entry(parent, row, "M1 theta", "initial_theta_m1")

        row = self._section(parent, row, "Snapshot")
        row = self._entry(parent, row, "Raw", "raw")
        row = self._entry(parent, row, "Legacy elec", "legacy_elec")
        row = self._entry(parent, row, "Actual elec", "actual_elec")
        row = self._entry(parent, row, "Uq V", "uq_v")
        row = self._entry(parent, row, "Ud V", "ud_v")
        row = self._entry(parent, row, "Battery V", "battery_v")
        row = self._entry(parent, row, "PWM period", "pwm_period")

        row = self._section(parent, row, "Output")
        ttk.Label(parent, text="CSV").grid(row=row, column=0, sticky="w", pady=2)
        ttk.Entry(parent, textvariable=self.vars["output_csv"], width=32).grid(row=row, column=1, sticky="ew", pady=2)
        ttk.Button(parent, text="Browse", command=self._browse_csv).grid(row=row, column=2, padx=(4, 0), pady=2)

    def _section(self, parent: ttk.Frame, row: int, title: str) -> int:
        ttk.Label(parent, text=title, font=("", 10, "bold")).grid(row=row, column=0, columnspan=3, sticky="w", pady=(10, 3))
        return row + 1

    def _entry(self, parent: ttk.Frame, row: int, label: str, key: str) -> int:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        ttk.Entry(parent, textvariable=self.vars[key], width=16).grid(row=row, column=1, columnspan=2, sticky="ew", pady=2)
        return row + 1

    def _combo(self, parent: ttk.Frame, row: int, label: str, key: str, values: list[str]) -> int:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        box = ttk.Combobox(parent, textvariable=self.vars[key], values=values, state="readonly", width=16)
        box.grid(row=row, column=1, columnspan=2, sticky="ew", pady=2)
        return row + 1

    def _build_actions(self, parent: ttk.Frame) -> None:
        actions = ttk.Frame(parent)
        actions.grid(row=0, column=0, sticky="ew")
        actions.columnconfigure(5, weight=1)
        ttk.Button(actions, text="Run", command=self.run_scenario).grid(row=0, column=0, padx=(0, 4))
        ttk.Button(actions, text="Self Test", command=self.run_self_test).grid(row=0, column=1, padx=4)
        ttk.Button(actions, text="Live Start", command=self.start_live).grid(row=0, column=2, padx=4)
        ttk.Button(actions, text="Live Stop", command=self.stop_live).grid(row=0, column=3, padx=4)
        ttk.Button(actions, text="Live Reset", command=self.reset_live).grid(row=0, column=4, padx=4)
        ttk.Button(actions, text="Copy CLI", command=self.copy_cli).grid(row=0, column=5, padx=4)
        ttk.Button(actions, text="Open CSV Folder", command=self.open_csv_folder).grid(row=0, column=6, padx=4)
        ttk.Label(actions, textvariable=self.vars["live_status"]).grid(row=0, column=7, sticky="e", padx=(12, 0))

    def _build_result_table(self, parent: ttk.Frame) -> None:
        table_frame = ttk.LabelFrame(parent, text="Summary")
        table_frame.grid(row=1, column=0, sticky="nsew", pady=(8, 4))
        table_frame.rowconfigure(0, weight=1)
        table_frame.columnconfigure(0, weight=1)

        self.tree = ttk.Treeview(table_frame, show="headings")
        self.tree.grid(row=0, column=0, sticky="nsew")
        yscroll = ttk.Scrollbar(table_frame, orient="vertical", command=self.tree.yview)
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll = ttk.Scrollbar(table_frame, orient="horizontal", command=self.tree.xview)
        xscroll.grid(row=1, column=0, sticky="ew")
        self.tree.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)

    def _build_log(self, parent: ttk.Frame) -> None:
        log_frame = ttk.LabelFrame(parent, text="Log / CLI")
        log_frame.grid(row=3, column=0, sticky="nsew", pady=(4, 0))
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        self.log = tk.Text(log_frame, height=12, wrap="none")
        self.log.grid(row=0, column=0, sticky="nsew")
        yscroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        yscroll.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=yscroll.set)

    def _build_plots(self, parent: ttk.Frame) -> None:
        plot_frame = ttk.LabelFrame(parent, text="Plots")
        plot_frame.grid(row=2, column=0, sticky="nsew", pady=4)
        plot_frame.columnconfigure(0, weight=3)
        plot_frame.columnconfigure(1, weight=1)
        plot_frame.rowconfigure(1, weight=1)

        controls = ttk.Frame(plot_frame)
        controls.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 4))
        ttk.Checkbutton(controls, text="M0", variable=self.vars["plot_m0"], command=self._draw_plots).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(controls, text="M1", variable=self.vars["plot_m1"], command=self._draw_plots).grid(row=0, column=1, sticky="w", padx=(8, 16))
        ttk.Button(controls, text="Redraw", command=self._draw_plots).grid(row=0, column=2)

        self.plot_canvas = tk.Canvas(plot_frame, height=280, background="white")
        self.plot_canvas.grid(row=1, column=0, sticky="nsew", padx=(0, 4))
        self.dq_canvas = tk.Canvas(plot_frame, height=280, width=240, background="white")
        self.dq_canvas.grid(row=1, column=1, sticky="nsew")
        self.plot_canvas.bind("<Configure>", lambda _event: self._draw_plots())
        self.dq_canvas.bind("<Configure>", lambda _event: self._draw_plots())

    def build_args(self) -> argparse.Namespace:
        return argparse.Namespace(
            self_test=False,
            scenario=self.vars["scenario"].get(),
            model=self.vars["model"].get(),
            angle_mode=self.vars["angle_mode"].get(),
            speed_rps=parse_float(self.vars["speed_rps"].get(), 20.0),
            speed_rps_m0=parse_float(self.vars["speed_rps_m0"].get(), 20.0),
            speed_rps_m1=parse_float(self.vars["speed_rps_m1"].get(), 20.0),
            duration_s=parse_float(self.vars["duration_s"].get(), 0.05),
            zero=parse_float(self.vars["zero"].get(), 0.0),
            encoder_direction=parse_int(self.vars["encoder_direction"].get(), -1),
            encoder_delay_steps=parse_int(self.vars["encoder_delay_steps"].get(), 0),
            phase_trim_deg=parse_float(self.vars["phase_trim_deg"].get(), 0.0),
            output_csv=self._csv_abs_or_empty(),
            raw=parse_int(self.vars["raw"].get(), 0),
            legacy_elec=parse_float(self.vars["legacy_elec"].get(), math.nan),
            actual_elec=parse_float(self.vars["actual_elec"].get(), math.nan),
            uq_v=parse_float(self.vars["uq_v"].get(), -2.0),
            ud_v=parse_float(self.vars["ud_v"].get(), 0.0),
            battery_v=parse_float(self.vars["battery_v"].get(), 24.0),
            pwm_period=parse_int(self.vars["pwm_period"].get(), 1800),
            initial_theta_m0=parse_float(self.vars["initial_theta_m0"].get(), 0.3),
            initial_theta_m1=parse_float(self.vars["initial_theta_m1"].get(), 0.6),
        )

    def _csv_abs_or_empty(self) -> str:
        text = self.vars["output_csv"].get().strip()
        if text == "":
            return ""
        path = Path(text)
        if not path.is_absolute():
            path = REPO_ROOT / path
        return str(path)

    def run_scenario(self) -> None:
        try:
            args = self.build_args()
            output = self._capture_run(args)
            self._show_output(output)
        except Exception as exc:
            messagebox.showerror("Simulation error", str(exc))
            self._append_log(f"\nERROR: {exc}\n")

    def run_self_test(self) -> None:
        try:
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                rc = bldc_sim.run_self_test()
            output = buffer.getvalue()
            self.last_rows = []
            self._show_output(output)
            if rc != 0:
                messagebox.showwarning("Self Test", "Self test failed.")
        except Exception as exc:
            messagebox.showerror("Self test error", str(exc))
            self._append_log(f"\nERROR: {exc}\n")

    def _capture_run(self, args: argparse.Namespace) -> str:
        self.stop_live()
        self.last_rows = []
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            if args.scenario == "snapshot":
                bldc_sim.run_snapshot(args)
            elif args.scenario == "realtime":
                bldc_sim.run_realtime(args)
                self.last_rows = bldc_sim.simulate_realtime(args)
            elif args.scenario == "step":
                bldc_sim.run_step(args)
                cfg = bldc_sim.MotorSimConfig(
                    zero_electric_angle=args.zero,
                    encoder_direction=args.encoder_direction,
                    encoder_delay_steps=args.encoder_delay_steps,
                    phase_advance_trim_rad=math.radians(args.phase_trim_deg),
                )
                self.last_rows = bldc_sim.simulate_step(cfg, args.angle_mode, args.speed_rps, args.duration_s, args.model)
            else:
                bldc_sim.run_conventions(args)
        return buffer.getvalue()

    def _make_live_config(self, args: argparse.Namespace) -> bldc_sim.MotorSimConfig:
        return bldc_sim.MotorSimConfig(
            voltage_supply=args.battery_v,
            zero_electric_angle=args.zero,
            encoder_direction=args.encoder_direction,
            encoder_delay_steps=args.encoder_delay_steps,
            phase_advance_trim_rad=math.radians(args.phase_trim_deg),
        )

    def reset_live(self) -> None:
        self.stop_live()
        args = self.build_args()
        cfg = self._make_live_config(args)
        self.live_time_s = 0.0
        self.live_step = 0
        self.live_motor_select = 0
        self.live_states = [
            bldc_sim.MotorSimState(theta_mech=args.initial_theta_m0, omega_rad_s=0.0),
            bldc_sim.MotorSimState(theta_mech=args.initial_theta_m1, omega_rad_s=0.0),
        ]
        self.live_encoders = [
            bldc_sim.EncoderSim(cfg.encoder_direction, cfg.encoder_delay_steps),
            bldc_sim.EncoderSim(cfg.encoder_direction, cfg.encoder_delay_steps),
        ]
        half = cfg.voltage_supply * 0.5
        self.live_pwms = [
            bldc_sim.PwmHold(half, half, half),
            bldc_sim.PwmHold(half, half, half),
        ]
        self.live_raw_last = [
            bldc_sim.encoder_raw_from_mech(self.live_states[0].theta_mech, cfg.encoder_direction),
            bldc_sim.encoder_raw_from_mech(self.live_states[1].theta_mech, cfg.encoder_direction),
        ]
        self.live_voltage_cmd = [0.0, 0.0]
        self.live_ud_last = [0.0, 0.0]
        self.live_uq_last = [0.0, 0.0]
        self.live_torque_last = [0.0, 0.0]
        self.last_rows = []
        self.vars["live_status"].set("Live reset, 0.000000s sim")
        self._draw_plots()

    def start_live(self) -> None:
        if not self.live_states:
            self.reset_live()
        if self.live_running:
            return
        self.live_running = True
        self.vars["live_status"].set(f"Live running, {self.live_time_s:.6f}s sim")
        self._schedule_live_tick()

    def stop_live(self) -> None:
        self.live_running = False
        if self.live_after_id is not None:
            try:
                self.after_cancel(self.live_after_id)
            except tk.TclError:
                pass
            self.live_after_id = None
        if "live_status" in self.vars:
            self.vars["live_status"].set(f"Live stopped, {self.live_time_s:.6f}s sim")

    def _schedule_live_tick(self) -> None:
        # 50us simulation step at 0.1% real-time speed => 50ms wall-clock interval.
        self.live_after_id = self.after(50, self._live_tick)

    def _live_tick(self) -> None:
        if not self.live_running:
            return
        try:
            self._advance_live_one_step()
        except Exception as exc:
            self.stop_live()
            messagebox.showerror("Live simulation error", str(exc))
            self._append_log(f"\nERROR: {exc}\n")
            return
        self._schedule_live_tick()

    def _advance_live_one_step(self) -> None:
        args = self.build_args()
        cfg = self._make_live_config(args)
        if len(self.live_states) != 2:
            self.reset_live()
            return

        for encoder in self.live_encoders:
            encoder.direction = cfg.encoder_direction
            encoder.delay_steps = max(0, cfg.encoder_delay_steps)

        isr_per_main = max(1, int(round(0.001 / cfg.dt)))
        if self.live_step % isr_per_main == 0:
            self.live_voltage_cmd[0] = bldc_sim.clamp(args.speed_rps_m0 * cfg.voltage_per_rps, cfg.voltage_limit)
            self.live_voltage_cmd[1] = bldc_sim.clamp(args.speed_rps_m1 * cfg.voltage_per_rps, cfg.voltage_limit)

        target_rps = [args.speed_rps_m0, args.speed_rps_m1]
        self.live_motor_select ^= 1
        motor = self.live_motor_select
        self.live_raw_last[motor] = self.live_encoders[motor].sample(self.live_states[motor].theta_mech)
        self.live_pwms[motor] = bldc_sim.build_foc_pwm(
            cfg,
            self.live_raw_last[motor],
            args.angle_mode,
            target_rps[motor],
            self.live_voltage_cmd[motor],
        )

        for m in range(2):
            self.live_ud_last[m], self.live_uq_last[m], self.live_torque_last[m] = bldc_sim.integrate_motor_from_pwm(
                self.live_states[m],
                cfg,
                self.live_pwms[m],
                args.model,
            )

        self.live_time_s += cfg.dt
        self.live_step += 1

        for m in range(2):
            self.last_rows.append(bldc_sim.make_realtime_row(
                self.live_time_s,
                m,
                self.live_states[m],
                self.live_raw_last[m],
                self.live_pwms[m],
                self.live_ud_last[m],
                self.live_uq_last[m],
                self.live_torque_last[m],
                target_rps[m],
                self.live_voltage_cmd[m],
            ))
        if len(self.last_rows) > 2000:
            self.last_rows = self.last_rows[-2000:]

        if self.live_step % 2 == 0:
            self._draw_plots()
            self._populate_live_table()
            self.vars["live_status"].set(f"Live running, {self.live_time_s:.6f}s sim")

    def _populate_live_table(self) -> None:
        latest = self._latest_rows_by_motor()
        for item in self.tree.get_children():
            self.tree.delete(item)
        header = ["motor", "time_s", "speed_rps", "encoder_raw", "theta_mech_rad", "uq_eff_v", "iq_a", "torque_nm"]
        self.tree["columns"] = header
        for col in header:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=max(90, min(160, len(col) * 10)), anchor="w")
        for motor in sorted(latest):
            row = latest[motor]
            self.tree.insert("", "end", values=(
                "step" if motor < 0 else f"M{motor}",
                f"{row['time_s']:.6f}",
                f"{row['speed_rps']:+.4f}",
                int(row["encoder_raw"]),
                f"{row['theta_mech_rad']:+.4f}",
                f"{row['uq_eff_v']:+.4f}",
                f"{row['iq_a']:+.4f}",
                f"{row['torque_nm']:+.6f}",
            ))

    def _show_output(self, output: str) -> None:
        self._populate_table(output)
        self._draw_plots()
        self._append_log("\n" + self.build_cli_command() + "\n" + output)

    def _populate_table(self, output: str) -> None:
        lines = [line.strip() for line in output.splitlines() if line.strip()]
        csv_lines = [line for line in lines if "," in line]
        for item in self.tree.get_children():
            self.tree.delete(item)

        if not csv_lines:
            self.tree["columns"] = ("result",)
            self.tree.heading("result", text="result")
            self.tree.column("result", width=500, anchor="w")
            for line in lines:
                self.tree.insert("", "end", values=(line,))
            return

        header = csv_lines[0].split(",")
        self.tree["columns"] = header
        for col in header:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=max(90, min(180, len(col) * 10)), anchor="w")
        for line in csv_lines[1:]:
            self.tree.insert("", "end", values=line.split(","))

    def _append_log(self, text: str) -> None:
        self.log.insert("end", text)
        self.log.see("end")

    def _draw_plots(self) -> None:
        if not hasattr(self, "plot_canvas"):
            return
        self._draw_time_plot()
        self._draw_dq_plot()

    def _draw_time_plot(self) -> None:
        canvas = self.plot_canvas
        canvas.delete("all")
        width = max(10, canvas.winfo_width())
        height = max(10, canvas.winfo_height())
        metrics = ["speed_rps", "uq_eff_v", "iq_a", "torque_nm"]
        margin_l = 62
        margin_r = 12
        margin_t = 18
        margin_b = 24
        gap = 12
        plot_w = max(1, width - margin_l - margin_r)
        slot_h = max(24, (height - margin_t - margin_b - gap * (len(metrics) - 1)) / len(metrics))

        if not self.last_rows:
            canvas.create_text(width / 2, height / 2, text="Run step or realtime scenario to plot", fill="#666666")
            return

        selected_motors = self._selected_plot_motors()
        colors = {0: "#1f77b4", 1: "#d62728", -1: "#1f77b4"}
        for idx, metric in enumerate(metrics):
            top = margin_t + idx * (slot_h + gap)
            self._draw_metric_slot(canvas, metric, selected_motors, colors, margin_l, top, plot_w, slot_h)

    def _draw_metric_slot(
        self,
        canvas: tk.Canvas,
        metric: str,
        selected_motors: set[int],
        colors: dict[int, str],
        margin_l: float,
        top: float,
        plot_w: float,
        plot_h: float,
    ) -> None:
        canvas.create_rectangle(margin_l, top, margin_l + plot_w, top + plot_h, outline="#c8c8c8")
        canvas.create_text(margin_l + 4, top + 2, text=metric, anchor="nw", fill="#202020")

        if metric not in self.last_rows[0]:
            return

        series = self._series_by_motor(metric, selected_motors)
        all_points = [point for points in series.values() for point in points]
        if len(all_points) < 2:
            return

        x_min = min(p[0] for p in all_points)
        x_max = max(p[0] for p in all_points)
        y_min = min(p[1] for p in all_points)
        y_max = max(p[1] for p in all_points)
        if abs(x_max - x_min) < 1.0e-12:
            x_max = x_min + 1.0
        if abs(y_max - y_min) < 1.0e-12:
            y_min -= 1.0
            y_max += 1.0
        pad = (y_max - y_min) * 0.08
        y_min -= pad
        y_max += pad

        def sx(x: float) -> float:
            return margin_l + (x - x_min) * plot_w / (x_max - x_min)

        def sy(y: float) -> float:
            return top + plot_h - (y - y_min) * plot_h / (y_max - y_min)

        canvas.create_text(4, top + 2, text=f"{y_max:+.3g}", anchor="nw", fill="#555555")
        canvas.create_text(4, top + plot_h - 2, text=f"{y_min:+.3g}", anchor="sw", fill="#555555")
        if top + plot_h + 18 <= canvas.winfo_height():
            canvas.create_text(margin_l, top + plot_h + 2, text=f"{x_min:.3f}s", anchor="nw", fill="#555555")
            canvas.create_text(margin_l + plot_w, top + plot_h + 2, text=f"{x_max:.3f}s", anchor="ne", fill="#555555")

        for motor, points in series.items():
            if len(points) < 2:
                continue
            coords: list[float] = []
            for x, y in points:
                coords.extend([sx(x), sy(y)])
            canvas.create_line(*coords, fill=colors.get(motor, "#1f77b4"), width=2)
            label = "step" if motor < 0 else f"M{motor}"
            label_x = margin_l + plot_w - 48 + max(0, motor) * 34
            canvas.create_text(label_x, top + 12, text=label, anchor="w", fill=colors.get(motor, "#1f77b4"))

    def _series_by_motor(self, metric: str, selected_motors: set[int]) -> dict[int, list[tuple[float, float]]]:
        series: dict[int, list[tuple[float, float]]] = {}
        for row in self.last_rows:
            motor = int(row.get("motor", -1))
            if motor >= 0 and motor not in selected_motors:
                continue
            series.setdefault(motor, []).append((float(row["time_s"]), float(row[metric])))
        return series

    def _draw_dq_plot(self) -> None:
        canvas = self.dq_canvas
        canvas.delete("all")
        width = max(10, canvas.winfo_width())
        height = max(10, canvas.winfo_height())
        cx = width * 0.5
        cy = height * 0.55
        radius = min(width, height) * 0.36
        canvas.create_text(8, 6, text="dq current", anchor="nw", fill="#202020")
        canvas.create_line(cx - radius, cy, cx + radius, cy, fill="#b0b0b0")
        canvas.create_line(cx, cy + radius, cx, cy - radius, fill="#b0b0b0")
        canvas.create_text(cx + radius + 4, cy, text="d", anchor="w", fill="#555555")
        canvas.create_text(cx, cy - radius - 4, text="q", anchor="s", fill="#555555")
        canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, outline="#d0d0d0")

        latest = self._latest_rows_by_motor()
        if not latest:
            canvas.create_text(cx, cy, text="no dq data", fill="#666666")
            return

        max_abs = max(max(abs(float(row.get("id_a", 0.0))), abs(float(row.get("iq_a", 0.0)))) for row in latest.values())
        if max_abs < 1.0e-6:
            max_abs = 1.0
        scale = radius / max_abs
        colors = {0: "#1f77b4", 1: "#d62728", -1: "#1f77b4"}
        y = height - 34
        for motor, row in latest.items():
            id_a = float(row.get("id_a", 0.0))
            iq_a = float(row.get("iq_a", 0.0))
            x2 = cx + id_a * scale
            y2 = cy - iq_a * scale
            color = colors.get(motor, "#1f77b4")
            canvas.create_line(cx, cy, x2, y2, fill=color, width=3, arrow="last")
            label = "step" if motor < 0 else f"M{motor}"
            canvas.create_text(8, y, text=f"{label} Id {id_a:+.3f} Iq {iq_a:+.3f}", anchor="w", fill=color)
            y += 16

    def _latest_rows_by_motor(self) -> dict[int, dict[str, float]]:
        latest: dict[int, dict[str, float]] = {}
        selected = self._selected_plot_motors()
        for row in self.last_rows:
            motor = int(row.get("motor", -1))
            if motor >= 0 and motor not in selected:
                continue
            latest[motor] = row
        return latest

    def _selected_plot_motors(self) -> set[int]:
        selected: set[int] = set()
        if self.vars["plot_m0"].get() == "1":
            selected.add(0)
        if self.vars["plot_m1"].get() == "1":
            selected.add(1)
        if not selected:
            selected.update({0, 1})
        return selected

    def _browse_csv(self) -> None:
        path = filedialog.asksaveasfilename(
            initialdir=str(REPO_ROOT / "sim" / "out"),
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if path:
            try:
                rel = Path(path).resolve().relative_to(REPO_ROOT)
                self.vars["output_csv"].set(str(rel))
            except ValueError:
                self.vars["output_csv"].set(path)

    def open_csv_folder(self) -> None:
        csv_path = self._csv_abs_or_empty()
        folder = Path(csv_path).parent if csv_path else (REPO_ROOT / "sim" / "out")
        folder.mkdir(parents=True, exist_ok=True)
        os.startfile(str(folder))

    def build_cli_command(self) -> str:
        parts = [
            "powershell -ExecutionPolicy Bypass -File .\\Script\\run_sim.ps1",
            f"-Scenario {self.vars['scenario'].get()}",
            f"-Model {self.vars['model'].get()}",
            f"-AngleMode {self.vars['angle_mode'].get()}",
            f"-SpeedRps {self.vars['speed_rps'].get()}",
            f"-SpeedRpsM0 {self.vars['speed_rps_m0'].get()}",
            f"-SpeedRpsM1 {self.vars['speed_rps_m1'].get()}",
            f"-DurationSec {self.vars['duration_s'].get()}",
            f"-Zero {self.vars['zero'].get()}",
            f"-EncoderDirection {self.vars['encoder_direction'].get()}",
            f"-EncoderDelaySteps {self.vars['encoder_delay_steps'].get()}",
            f"-PhaseTrimDeg {self.vars['phase_trim_deg'].get()}",
            f"-Raw {self.vars['raw'].get()}",
            f"-LegacyElec {self.vars['legacy_elec'].get()}",
            f"-ActualElec {self.vars['actual_elec'].get()}",
            f"-UqV {self.vars['uq_v'].get()}",
            f"-UdV {self.vars['ud_v'].get()}",
            f"-BatteryV {self.vars['battery_v'].get()}",
            f"-PwmPeriod {self.vars['pwm_period'].get()}",
            f"-InitialThetaM0 {self.vars['initial_theta_m0'].get()}",
            f"-InitialThetaM1 {self.vars['initial_theta_m1'].get()}",
        ]
        if self.vars["output_csv"].get().strip():
            parts.append(f"-OutputCsv \"{self.vars['output_csv'].get()}\"")
        return " ".join(parts)

    def _refresh_cli(self) -> None:
        if not hasattr(self, "log"):
            return
        # Keep this light: full CLI is appended on each run/copy, not on every edit.

    def copy_cli(self) -> None:
        command = self.build_cli_command()
        self.clipboard_clear()
        self.clipboard_append(command)
        self._append_log("\nCopied CLI:\n" + command + "\n")


def main() -> int:
    app = SimGui()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

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

        self._build_settings(left)
        self._build_actions(right)
        self._build_result_table(right)
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
        ttk.Button(actions, text="Copy CLI", command=self.copy_cli).grid(row=0, column=2, padx=4)
        ttk.Button(actions, text="Open CSV Folder", command=self.open_csv_folder).grid(row=0, column=3, padx=4)

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
        log_frame.grid(row=2, column=0, sticky="nsew", pady=(4, 0))
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        self.log = tk.Text(log_frame, height=12, wrap="none")
        self.log.grid(row=0, column=0, sticky="nsew")
        yscroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        yscroll.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=yscroll.set)

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
            self._show_output(output)
            if rc != 0:
                messagebox.showwarning("Self Test", "Self test failed.")
        except Exception as exc:
            messagebox.showerror("Self test error", str(exc))
            self._append_log(f"\nERROR: {exc}\n")

    def _capture_run(self, args: argparse.Namespace) -> str:
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            if args.scenario == "snapshot":
                bldc_sim.run_snapshot(args)
            elif args.scenario == "realtime":
                bldc_sim.run_realtime(args)
            elif args.scenario == "step":
                bldc_sim.run_step(args)
            else:
                bldc_sim.run_conventions(args)
        return buffer.getvalue()

    def _show_output(self, output: str) -> None:
        self._populate_table(output)
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

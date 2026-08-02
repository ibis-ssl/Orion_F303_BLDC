"""OrionモーターをCAN経由で手動運転する安全重視の簡易GUIを提供する。"""

from __future__ import annotations

import struct
import tkinter as tk
import time
from collections import deque
from tkinter import messagebox, ttk

from serial.tools import list_ports

from .driver import CanFrame, OrionCanDriver, OrionCanError
from .telemetry import MotorTelemetry, TelemetryUpdate, angle_rad_to_legacy_raw, apply_telemetry_frame, decode_speed

MAX_SPEED_RPS = 80.0
GUI_HEARTBEAT_MS = 50
MONITOR_UPDATE_MS = 100
PLOT_UPDATE_MS = 250
MAX_RX_PER_TICK = 1024
PLOT_WINDOW_S = 10.0


def parse_target(text: str) -> float:
    """GUI入力をファームの許容速度範囲内の有限値へ変換する。"""
    try:
        value = float(text)
    except ValueError as exc:
        raise ValueError("速度は数値で入力してください") from exc
    if not -MAX_SPEED_RPS <= value <= MAX_SPEED_RPS:
        raise ValueError(f"速度は{-MAX_SPEED_RPS:g}～{MAX_SPEED_RPS:g} rpsで入力してください")
    return value


class MotorControlGui:
    """接続、目標速度更新、停止、テレメトリ表示を管理するTk画面。"""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Orion CAN Motor Control")
        self.root.resizable(False, False)
        self.driver: OrionCanDriver | None = None
        self.running = False
        self.targets = [0.0, 0.0]
        self.telemetry = [MotorTelemetry(), MotorTelemetry()]
        self.speed_history: list[deque[tuple[float, float]]] = [deque(), deque()]
        self.current_history: list[deque[tuple[float, float]]] = [deque(), deque()]
        self.next_monitor_update = 0.0
        self.next_plot_update = 0.0
        self.rx_total = 0

        self.port = tk.StringVar()
        self.board = tk.IntVar(value=1)
        self.target_vars = [tk.DoubleVar(value=0.0), tk.DoubleVar(value=0.0)]
        self.target_text_vars = [tk.StringVar(value="+0.0 rps"), tk.StringVar(value="+0.0 rps")]
        self.actual_vars = [tk.StringVar(value="--.- rps"), tk.StringVar(value="--.- rps")]
        self.encoder_vars = [tk.StringVar(value="-----"), tk.StringVar(value="-----")]
        self.voltage_vars = [tk.StringVar(value="--.-- V"), tk.StringVar(value="--.-- V")]
        self.current_vars = [tk.StringVar(value="--.-- A"), tk.StringVar(value="--.-- A")]
        self.motor_temp_vars = [tk.StringVar(value="--.- °C"), tk.StringVar(value="--.- °C")]
        self.fet_temp_vars = [tk.StringVar(value="--.- °C"), tk.StringVar(value="--.- °C")]
        self.status = tk.StringVar(value="未接続 / 停止")
        self.rx_status = tk.StringVar(value="受信: 0 frame")

        self._build()
        self._refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(GUI_HEARTBEAT_MS, self._tick)

    def _build(self) -> None:
        frame = ttk.Frame(self.root, padding=14)
        frame.grid(sticky="nsew")

        connection = ttk.LabelFrame(frame, text="CAN接続", padding=10)
        connection.grid(row=0, column=0, columnspan=4, sticky="ew")
        ttk.Label(connection, text="ポート").grid(row=0, column=0, padx=(0, 6))
        self.port_combo = ttk.Combobox(connection, textvariable=self.port, width=12)
        self.port_combo.grid(row=0, column=1, padx=(0, 8))
        ttk.Button(connection, text="更新", command=self._refresh_ports).grid(row=0, column=2, padx=(0, 12))
        ttk.Label(connection, text="Board ID").grid(row=0, column=3, padx=(0, 6))
        ttk.Spinbox(connection, from_=0, to=1, textvariable=self.board, width=4, state="readonly").grid(row=0, column=4, padx=(0, 12))
        self.connect_button = ttk.Button(connection, text="接続", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=5)

        ttk.Label(frame, text="モーター", anchor="center").grid(row=1, column=0, pady=(14, 4))
        ttk.Label(frame, text="目標速度 [-80～+80 rps]", anchor="center").grid(row=1, column=1, pady=(14, 4))
        ttk.Label(frame, text="実速度", anchor="center").grid(row=1, column=2, pady=(14, 4))
        for motor in range(2):
            ttk.Label(frame, text=f"Motor {motor}").grid(row=2 + motor, column=0, padx=8, pady=6)
            target_frame = ttk.Frame(frame)
            target_frame.grid(row=2 + motor, column=1, padx=8, pady=6)
            slider = tk.Scale(
                target_frame,
                from_=-MAX_SPEED_RPS,
                to=MAX_SPEED_RPS,
                resolution=0.5,
                orient="horizontal",
                length=300,
                showvalue=False,
                variable=self.target_vars[motor],
                command=lambda value, index=motor: self._slider_changed(index, value),
            )
            slider.pack(side="left")
            ttk.Label(target_frame, textvariable=self.target_text_vars[motor], width=11, anchor="e").pack(side="left", padx=(6, 0))
            ttk.Label(frame, textvariable=self.actual_vars[motor], width=14, anchor="e").grid(row=2 + motor, column=2, padx=8, pady=6)
            ttk.Button(frame, text="0に設定", command=lambda index=motor: self._zero_motor(index)).grid(row=2 + motor, column=3, padx=8, pady=6)

        button_row = ttk.Frame(frame)
        button_row.grid(row=4, column=0, columnspan=4, pady=(14, 8), sticky="ew")
        ttk.Label(button_row, text="接続中はスライダ値を自動送信します。", anchor="w").pack(side="left", fill="x", expand=True, padx=(0, 8))
        self.stop_button = tk.Button(
            button_row, text="停止（0 rps）", command=self._stop_motion,
            bg="#c62828", fg="white", activebackground="#8e0000", activeforeground="white",
            font=("Yu Gothic UI", 11, "bold"), padx=18, pady=5,
        )
        self.stop_button.pack(side="left", fill="x", expand=True)

        ttk.Separator(frame).grid(row=5, column=0, columnspan=4, sticky="ew", pady=6)
        ttk.Label(frame, textvariable=self.status, anchor="w").grid(row=6, column=0, columnspan=4, sticky="ew")
        ttk.Label(frame, text="速度指令は20 ms周期。GUI停止時は200 ms以内に0 rpsへ移行します。", foreground="#555555").grid(row=7, column=0, columnspan=4, sticky="w", pady=(4, 0))

        monitor = ttk.LabelFrame(frame, text="モーターテレメトリ", padding=8)
        monitor.grid(row=8, column=0, columnspan=4, sticky="nsew", pady=(12, 0))
        headers = ("モーター", "指定速度", "現在速度", "Encoder raw", "電圧", "電流", "Motor温度", "FET温度")
        for column, text in enumerate(headers):
            ttk.Label(monitor, text=text, anchor="center").grid(row=0, column=column, padx=8, pady=(0, 4))
        for motor in range(2):
            values = (self.target_text_vars[motor], self.actual_vars[motor], self.encoder_vars[motor], self.voltage_vars[motor], self.current_vars[motor], self.motor_temp_vars[motor], self.fet_temp_vars[motor])
            ttk.Label(monitor, text=f"Motor {motor}").grid(row=1 + motor, column=0, padx=8, pady=4)
            for column, variable in enumerate(values, start=1):
                ttk.Label(monitor, textvariable=variable, width=12, anchor="e").grid(row=1 + motor, column=column, padx=8, pady=4)
        ttk.Label(monitor, textvariable=self.rx_status).grid(row=3, column=0, columnspan=8, sticky="w", pady=(5, 0))

        plots = ttk.LabelFrame(frame, text="直近10秒のプロット", padding=8)
        plots.grid(row=9, column=0, columnspan=4, sticky="ew", pady=(12, 0))
        plot_tabs = ttk.Notebook(plots)
        plot_tabs.grid(row=0, column=0)
        speed_tab = ttk.Frame(plot_tabs, padding=4)
        current_tab = ttk.Frame(plot_tabs, padding=4)
        plot_tabs.add(speed_tab, text="現在速度")
        plot_tabs.add(current_tab, text="電流")
        ttk.Label(speed_tab, text="回転数 [rps]   Motor 0: 青 / Motor 1: 橙").grid(row=0, column=0, sticky="w")
        self.speed_canvas = tk.Canvas(speed_tab, width=720, height=450, bg="white", highlightthickness=1, highlightbackground="#aaaaaa")
        self.speed_canvas.grid(row=1, column=0, pady=(2, 0))
        ttk.Label(current_tab, text="電流 [A]   Motor 0: 青 / Motor 1: 橙").grid(row=0, column=0, sticky="w")
        self.current_canvas = tk.Canvas(current_tab, width=720, height=450, bg="white", highlightthickness=1, highlightbackground="#aaaaaa")
        self.current_canvas.grid(row=1, column=0, pady=(2, 0))

    def _refresh_ports(self) -> None:
        ports = list(list_ports.comports())
        names = [item.device for item in ports]
        self.port_combo["values"] = names
        preferred = next((item.device for item in ports if item.vid == 0x0483 and item.pid == 0x5740), None)
        if not self.port.get() or self.port.get() not in names:
            self.port.set(preferred or (names[0] if names else ""))

    def _toggle_connection(self) -> None:
        if self.driver is not None:
            self._disconnect()
            return
        if not self.port.get():
            messagebox.showerror("接続エラー", "CANアダプタのCOMポートを選択してください")
            return
        try:
            # 全点プロットの描画中も一時的なGUI遅延で指令を0へ落とさない。
            # プロセス停止やCAN断時はファーム側の約100 ms timeoutが機能する。
            driver = OrionCanDriver(self.port.get(), command_watchdog_s=0.500)
            driver.open()
            driver.set_speed(self.board.get(), 0, 0.0)
            driver.set_speed(self.board.get(), 1, 0.0)
        except Exception as exc:
            messagebox.showerror("接続エラー", str(exc))
            return
        self.driver = driver
        self.targets = [value.get() for value in self.target_vars]
        self.running = True
        self._clear_telemetry()
        self._send_targets()
        self.connect_button.configure(text="切断")
        self.status.set(f"{self.port.get()} / Board {self.board.get()} / 接続済み・自動反映中")

    def _disconnect(self) -> None:
        driver, self.driver = self.driver, None
        self.running = False
        self.targets = [0.0, 0.0]
        if driver is not None:
            try:
                driver.close()
            except Exception as exc:
                messagebox.showwarning("切断時エラー", str(exc))
        self.connect_button.configure(text="接続")
        self.status.set("未接続 / 停止")

    def _zero_motor(self, motor: int) -> None:
        self.target_vars[motor].set(0.0)
        self.target_text_vars[motor].set("+0.0 rps")
        self.targets[motor] = 0.0
        if self.driver is not None:
            self.driver.set_speed(self.board.get(), motor, 0.0)

    def _stop_motion(self) -> None:
        self.running = False
        self.targets = [0.0, 0.0]
        for value, text in zip(self.target_vars, self.target_text_vars):
            value.set(0.0)
            text.set("+0.0 rps")
        if self.driver is not None:
            try:
                self.driver.stop_all()
                self.running = True
                self.status.set(f"{self.port.get()} / Board {self.board.get()} / 0 rps・自動反映中")
            except OrionCanError as exc:
                self._connection_failed(exc)

    def _slider_changed(self, motor: int, value: str) -> None:
        target = parse_target(value)
        self.target_text_vars[motor].set(f"{target:+.1f} rps")
        self.targets[motor] = target
        if self.running and self.driver is not None:
            try:
                self.driver.set_speed(self.board.get(), motor, target)
            except OrionCanError as exc:
                self._connection_failed(exc)

    def _send_targets(self) -> None:
        assert self.driver is not None
        board = self.board.get()
        for motor, target in enumerate(self.targets):
            self.driver.set_speed(board, motor, target)

    def _tick(self) -> None:
        if self.driver is not None:
            try:
                if self.running:
                    self._send_targets()
                for _ in range(MAX_RX_PER_TICK):
                    frame = self.driver.receive(timeout=0)
                    if frame is None:
                        break
                    update = apply_telemetry_frame(frame, self.board.get(), self.telemetry)
                    if update is not None:
                        self.rx_total += 1
                        received_at = time.monotonic()
                        if update.kind == "speed":
                            self.speed_history[update.motor].append((received_at, update.value))
                        elif update.kind == "current":
                            self.current_history[update.motor].append((received_at, update.value))
                if time.monotonic() >= self.next_monitor_update:
                    self._render_telemetry()
                    self.next_monitor_update = time.monotonic() + MONITOR_UPDATE_MS / 1000.0
                if time.monotonic() >= self.next_plot_update:
                    self._render_plots()
                    self.next_plot_update = time.monotonic() + PLOT_UPDATE_MS / 1000.0
            except OrionCanError as exc:
                self._connection_failed(exc)
        self.root.after(GUI_HEARTBEAT_MS, self._tick)

    def _render_telemetry(self) -> None:
        now = time.monotonic()
        self._trim_histories(now)
        for motor, state in enumerate(self.telemetry):
            if state.speed_rps is not None:
                self.actual_vars[motor].set(f"{state.speed_rps:+.2f} rps")
            if state.encoder_raw is not None:
                self.encoder_vars[motor].set(f"{state.encoder_raw:5d}")
            if state.voltage_v is not None:
                self.voltage_vars[motor].set(f"{state.voltage_v:.2f} V")
            if state.current_a is not None:
                self.current_vars[motor].set(f"{state.current_a:+.2f} A")
            if state.motor_temp_c is not None:
                self.motor_temp_vars[motor].set(f"{state.motor_temp_c:.1f} °C")
            if state.fet_temp_c is not None:
                self.fet_temp_vars[motor].set(f"{state.fet_temp_c:.1f} °C")
        self.rx_status.set(f"復号済みCANテレメトリ: {self.rx_total:,} frame")

    def _render_plots(self) -> None:
        now = time.monotonic()
        self._trim_histories(now)
        self._draw_plot(self.speed_canvas, self.speed_history, MAX_SPEED_RPS, now)
        measured_current_limit = max((abs(value) for history in self.current_history for _, value in history), default=0.0)
        current_limit = max(5.0, measured_current_limit)
        self._draw_plot(self.current_canvas, self.current_history, current_limit, now)

    def _trim_histories(self, now: float) -> None:
        cutoff = now - PLOT_WINDOW_S
        for history in self.speed_history + self.current_history:
            while history and history[0][0] < cutoff:
                history.popleft()

    @staticmethod
    def _draw_plot(canvas: tk.Canvas, histories: list[deque[tuple[float, float]]], limit: float, now: float | None = None) -> None:
        canvas.delete("all")
        now = time.monotonic() if now is None else now
        width = int(canvas["width"])
        height = int(canvas["height"])
        left, right, top, bottom = 42, width - 8, 8, height - 20
        middle = (top + bottom) / 2
        canvas.create_line(left, top, left, bottom, fill="#888888")
        canvas.create_line(left, middle, right, middle, fill="#cccccc")
        canvas.create_text(left - 5, top, text=f"+{limit:.1f}", anchor="e", fill="#555555")
        canvas.create_text(left - 5, middle, text="0", anchor="e", fill="#555555")
        canvas.create_text(left - 5, bottom, text=f"-{limit:.1f}", anchor="e", fill="#555555")
        canvas.create_text(left, bottom + 10, text="-10 s", anchor="w", fill="#555555")
        canvas.create_text(right, bottom + 10, text="現在", anchor="e", fill="#555555")
        colors = ("#1976d2", "#ef6c00")
        for history, color in zip(histories, colors):
            if len(history) < 2:
                continue
            points: list[float] = []
            for received_at, value in history:
                x = left + (right - left) * max(0.0, min(PLOT_WINDOW_S, received_at - (now - PLOT_WINDOW_S))) / PLOT_WINDOW_S
                y = middle - max(-limit, min(limit, value)) * (bottom - top) / (2 * limit)
                points.extend((x, y))
            canvas.create_line(*points, fill=color, width=2)

    def _clear_telemetry(self) -> None:
        self.telemetry = [MotorTelemetry(), MotorTelemetry()]
        for history in self.speed_history + self.current_history:
            history.clear()
        self.rx_total = 0
        self.rx_status.set("復号済みCANテレメトリ: 0 frame")

    def _connection_failed(self, error: Exception) -> None:
        self._disconnect()
        messagebox.showerror("CAN通信エラー", str(error))

    def _on_close(self) -> None:
        self._disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    MotorControlGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()

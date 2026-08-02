"""OrionモーターをCAN経由で手動運転する安全重視の簡易GUIを提供する。"""

from __future__ import annotations

import struct
import tkinter as tk
from collections import deque
from tkinter import messagebox, ttk

from serial.tools import list_ports

from .driver import CanFrame, OrionCanDriver, OrionCanError

MAX_SPEED_RPS = 80.0
GUI_HEARTBEAT_MS = 50


def parse_target(text: str) -> float:
    """GUI入力をファームの許容速度範囲内の有限値へ変換する。"""
    try:
        value = float(text)
    except ValueError as exc:
        raise ValueError("速度は数値で入力してください") from exc
    if not -MAX_SPEED_RPS <= value <= MAX_SPEED_RPS:
        raise ValueError(f"速度は{-MAX_SPEED_RPS:g}～{MAX_SPEED_RPS:g} rpsで入力してください")
    return value


def decode_speed(frame: CanFrame, board_id: int) -> tuple[int, float] | None:
    """対象基板の速度テレメトリからmotor番号とrpsを取り出す。"""
    base_id = 0x200 + board_id * 2
    if frame.can_id not in (base_id, base_id + 1) or len(frame.data) != 8:
        return None
    speed_rps = struct.unpack_from("<f", frame.data)[0]
    return frame.can_id - base_id, speed_rps


class MotorControlGui:
    """接続、目標速度更新、停止、テレメトリ表示を管理するTk画面。"""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Orion CAN Motor Control")
        self.root.resizable(False, False)
        self.driver: OrionCanDriver | None = None
        self.running = False
        self.targets = [0.0, 0.0]
        self.rx_times: deque[float] = deque()

        self.port = tk.StringVar()
        self.board = tk.IntVar(value=1)
        self.target_vars = [tk.StringVar(value="0.0"), tk.StringVar(value="0.0")]
        self.actual_vars = [tk.StringVar(value="--.- rps"), tk.StringVar(value="--.- rps")]
        self.status = tk.StringVar(value="未接続 / 停止")

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
        ttk.Label(frame, text="目標速度 [rps]", anchor="center").grid(row=1, column=1, pady=(14, 4))
        ttk.Label(frame, text="実速度", anchor="center").grid(row=1, column=2, pady=(14, 4))
        for motor in range(2):
            ttk.Label(frame, text=f"Motor {motor}").grid(row=2 + motor, column=0, padx=8, pady=6)
            ttk.Entry(frame, textvariable=self.target_vars[motor], width=14, justify="right").grid(row=2 + motor, column=1, padx=8, pady=6)
            ttk.Label(frame, textvariable=self.actual_vars[motor], width=14, anchor="e").grid(row=2 + motor, column=2, padx=8, pady=6)
            ttk.Button(frame, text="0に設定", command=lambda index=motor: self._zero_motor(index)).grid(row=2 + motor, column=3, padx=8, pady=6)

        button_row = ttk.Frame(frame)
        button_row.grid(row=4, column=0, columnspan=4, pady=(14, 8), sticky="ew")
        self.run_button = ttk.Button(button_row, text="運転開始 / 目標を反映", command=self._start, state="disabled")
        self.run_button.pack(side="left", fill="x", expand=True, padx=(0, 8))
        self.stop_button = tk.Button(
            button_row, text="停止（0 rps）", command=self._stop_motion,
            bg="#c62828", fg="white", activebackground="#8e0000", activeforeground="white",
            font=("Yu Gothic UI", 11, "bold"), padx=18, pady=5,
        )
        self.stop_button.pack(side="left", fill="x", expand=True)

        ttk.Separator(frame).grid(row=5, column=0, columnspan=4, sticky="ew", pady=6)
        ttk.Label(frame, textvariable=self.status, anchor="w").grid(row=6, column=0, columnspan=4, sticky="ew")
        ttk.Label(frame, text="速度指令は20 ms周期。GUI停止時は200 ms以内に0 rpsへ移行します。", foreground="#555555").grid(row=7, column=0, columnspan=4, sticky="w", pady=(4, 0))

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
            driver = OrionCanDriver(self.port.get())
            driver.open()
            driver.set_speed(self.board.get(), 0, 0.0)
            driver.set_speed(self.board.get(), 1, 0.0)
        except Exception as exc:
            messagebox.showerror("接続エラー", str(exc))
            return
        self.driver = driver
        self.running = False
        self.connect_button.configure(text="切断")
        self.run_button.configure(state="normal")
        self.status.set(f"{self.port.get()} / Board {self.board.get()} / 接続済み・停止")

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
        self.run_button.configure(state="disabled")
        self.status.set("未接続 / 停止")

    def _start(self) -> None:
        if self.driver is None:
            return
        try:
            targets = [parse_target(value.get()) for value in self.target_vars]
        except ValueError as exc:
            messagebox.showerror("入力エラー", str(exc))
            return
        self.targets = targets
        self.running = True
        self._send_targets()
        self.status.set(f"{self.port.get()} / Board {self.board.get()} / 運転中")

    def _zero_motor(self, motor: int) -> None:
        self.target_vars[motor].set("0.0")
        self.targets[motor] = 0.0
        if self.driver is not None:
            self.driver.set_speed(self.board.get(), motor, 0.0)

    def _stop_motion(self) -> None:
        self.running = False
        self.targets = [0.0, 0.0]
        for value in self.target_vars:
            value.set("0.0")
        if self.driver is not None:
            try:
                self.driver.stop_all()
                self.status.set(f"{self.port.get()} / Board {self.board.get()} / 停止")
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
                for _ in range(100):
                    frame = self.driver.receive(timeout=0)
                    if frame is None:
                        break
                    decoded = decode_speed(frame, self.board.get())
                    if decoded is not None:
                        motor, speed = decoded
                        self.actual_vars[motor].set(f"{speed:+.2f} rps")
            except OrionCanError as exc:
                self._connection_failed(exc)
        self.root.after(GUI_HEARTBEAT_MS, self._tick)

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

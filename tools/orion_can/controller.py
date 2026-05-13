"""High-level Orion motor command controller for CLI and future GUI tools."""

from __future__ import annotations

import time
from collections.abc import Callable

from .protocol import OrionFrame, encode_freewheel_command, encode_reset_command, encode_speed_command


FrameWriter = Callable[[OrionFrame], None]


class OrionMotorController:
    def __init__(self, write_frame: FrameWriter, board: int) -> None:
        self.write_frame = write_frame
        self.board = board

    def set_speed(self, motor: int, rps: float) -> None:
        self.write_frame(encode_speed_command(self.board, motor, rps))

    def set_speeds(self, m0_rps: float | None, m1_rps: float | None) -> None:
        if m0_rps is not None:
            self.set_speed(0, m0_rps)
        if m1_rps is not None:
            self.set_speed(1, m1_rps)

    def stop(self, count: int = 5, period_s: float = 0.02) -> None:
        for index in range(count):
            self.set_speed(0, 0.0)
            self.set_speed(1, 0.0)
            if index + 1 < count and period_s > 0.0:
                time.sleep(period_s)

    def freewheel(self, ms: int) -> None:
        self.write_frame(encode_freewheel_command(ms))

    def reset(self) -> None:
        self.write_frame(encode_reset_command())


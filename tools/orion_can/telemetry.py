"""OrionのCANテレメトリをモーター単位の物理量へ復号する。"""

from __future__ import annotations

import struct
from dataclasses import dataclass

from .driver import CanFrame


@dataclass(slots=True)
class MotorTelemetry:
    speed_rps: float | None = None
    encoder_raw: int | None = None
    voltage_v: float | None = None
    current_a: float | None = None
    motor_temp_c: float | None = None
    fet_temp_c: float | None = None


@dataclass(frozen=True, slots=True)
class TelemetryUpdate:
    motor: int
    kind: str
    value: float


def decode_speed(frame: CanFrame, board_id: int) -> tuple[int, float] | None:
    base_id = 0x200 + board_id * 2
    if frame.can_id not in (base_id, base_id + 1) or len(frame.data) != 8:
        return None
    return frame.can_id - base_id, struct.unpack_from("<f", frame.data)[0]


def angle_rad_to_legacy_raw(angle_rad: float) -> int:
    raw = round(angle_rad * 65535.0 / (2.0 * 3.141592653589793))
    return max(0, min(65535, raw))


def apply_telemetry_frame(frame: CanFrame, board_id: int, telemetry: list[MotorTelemetry]) -> TelemetryUpdate | None:
    if len(frame.data) != 8:
        return None
    values = struct.unpack("<ff", frame.data)
    groups = ((0x200, "speed"), (0x210, "voltage"), (0x220, "temperature"), (0x230, "current"))
    for base, kind in groups:
        offset = frame.can_id - (base + board_id * 2)
        if offset not in (0, 1):
            continue
        state = telemetry[offset]
        if kind == "speed":
            state.speed_rps = values[0]
            state.encoder_raw = angle_rad_to_legacy_raw(values[1])
        elif kind == "voltage":
            state.voltage_v = values[0]
        elif kind == "temperature":
            state.motor_temp_c, state.fet_temp_c = values
        else:
            state.current_a = values[0]
        return TelemetryUpdate(offset, kind, values[0])
    return None

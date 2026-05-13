"""Orion_F303_BLDC CAN protocol helpers for PC-side development tools."""

from __future__ import annotations

import struct
from dataclasses import dataclass


CAN_SFF_MASK = 0x7FF
ORION_RPS_LIMIT = 80.0


@dataclass(frozen=True)
class OrionFrame:
    can_id: int
    data: bytes


def clamp_rps(rps: float) -> float:
    if rps > ORION_RPS_LIMIT:
        return ORION_RPS_LIMIT
    if rps < -ORION_RPS_LIMIT:
        return -ORION_RPS_LIMIT
    return rps


def speed_command_id(board: int, motor: int) -> int:
    if board < 0:
        raise ValueError("board must be >= 0")
    if motor not in (0, 1):
        raise ValueError("motor must be 0 or 1")
    return (0x100 + board * 2 + motor) & CAN_SFF_MASK


def encode_speed_command(board: int, motor: int, rps: float) -> OrionFrame:
    can_id = speed_command_id(board, motor)
    return OrionFrame(can_id=can_id, data=struct.pack("<ff", clamp_rps(float(rps)), 0.0))


def encode_reset_command() -> OrionFrame:
    return OrionFrame(can_id=0x010, data=bytes([0, 0]))


def encode_freewheel_command(ms: int) -> OrionFrame:
    if ms < 0 or ms > 255:
        raise ValueError("firmware CAN freewheel command currently supports 0..255 ms in data[2]")
    return OrionFrame(can_id=0x110, data=bytes([3, 0, ms & 0xFF]))


def format_frame(frame: OrionFrame) -> str:
    data_hex = " ".join(f"{byte:02X}" for byte in frame.data)
    return f"id=0x{frame.can_id:03X} dlc={len(frame.data)} data={data_hex}"


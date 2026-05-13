"""CANable gs_usb transport wrapper used by Orion PC tools."""

from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / "Script"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from canable_cli import CanFrame, CanableGsUsb, decode_target_frame, print_frame  # noqa: E402

from .protocol import OrionFrame


class OrionCanTransport:
    def __init__(
        self,
        bitrate: int = 1_000_000,
        vid_pid: tuple[int, int] | None = None,
        channel: int = 0,
        timeout_ms: int = 100,
        verbose: bool = False,
        loopback: bool = False,
    ) -> None:
        self.bitrate = bitrate
        self.loopback = loopback
        self.dev = CanableGsUsb(vid_pid, channel=channel, timeout_ms=timeout_ms, verbose=verbose)

    def __enter__(self) -> "OrionCanTransport":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def open(self) -> None:
        self.dev.open()
        self.dev.start(self.bitrate, listen_only=False, loopback=self.loopback)

    def close(self) -> None:
        self.dev.close()

    def write(self, frame: OrionFrame) -> None:
        self.dev.write_frame(CanFrame(can_id=frame.can_id, data=frame.data))

    def read(self, timeout_ms: int = 1) -> CanFrame | None:
        try:
            return self.dev.read_frame(timeout_ms=timeout_ms)
        except Exception as exc:
            if "timeout" in str(exc).lower():
                return None
            raise

"""Command line motor control tool for Orion_F303_BLDC over CANable."""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass

from .controller import OrionMotorController
from .protocol import OrionFrame, format_frame
from .transport import OrionCanTransport


@dataclass
class RunMonitor:
    board: int
    m0_last: float | None = None
    m1_last: float | None = None
    m0_max_abs: float = 0.0
    m1_max_abs: float = 0.0
    fault_id: int = 0
    fault_info: int = 0
    fault_value: float = 0.0
    rx_count: int = 0

    def update(self, can_id: int, data: bytes) -> None:
        if len(data) < 8:
            return
        sid = can_id & 0x7FF
        self.rx_count += 1
        if sid == 0x000:
            self.fault_id, self.fault_info, self.fault_value = struct.unpack("<HHf", data[:8])
            return
        m0_id = 0x200 + self.board * 2
        m1_id = m0_id + 1
        if sid == m0_id:
            self.m0_last = struct.unpack("<f", data[:4])[0]
            self.m0_max_abs = max(self.m0_max_abs, abs(self.m0_last))
        elif sid == m1_id:
            self.m1_last = struct.unpack("<f", data[:4])[0]
            self.m1_max_abs = max(self.m1_max_abs, abs(self.m1_last))

    def summary(self) -> str:
        return (
            f"monitor rx={self.rx_count} "
            f"m0_last={self.m0_last if self.m0_last is not None else 0.0:+.3f} "
            f"m0_max_abs={self.m0_max_abs:.3f} "
            f"m1_last={self.m1_last if self.m1_last is not None else 0.0:+.3f} "
            f"m1_max_abs={self.m1_max_abs:.3f} "
            f"fault=0x{self.fault_id:04X}/{self.fault_info} {self.fault_value:+.3f}"
        )


def parse_vid_pid(text: str) -> tuple[int, int]:
    if ":" not in text:
        raise argparse.ArgumentTypeError("VID:PID must be hex, for example 1d50:606f")
    vid_s, pid_s = text.split(":", 1)
    return int(vid_s, 16), int(pid_s, 16)


def validate_board_for_current_firmware(board: int) -> None:
    if board not in (0, 1):
        raise SystemExit(
            "current firmware handleCanMessage() accepts speed IDs 0x100..0x103 only; "
            "use --board 0 or --board 1"
        )


class DryRunWriter:
    def __init__(self) -> None:
        self.count = 0

    def __call__(self, frame: OrionFrame) -> None:
        self.count += 1
        print(format_frame(frame))


def open_writer(args):
    if args.dry_run:
        return DryRunWriter()
    return OrionCanTransport(
        bitrate=args.bitrate,
        vid_pid=args.vid_pid,
        channel=args.channel,
        timeout_ms=args.timeout,
        verbose=args.verbose,
        loopback=args.loopback,
    )


def run_with_writer(args, action) -> int:
    validate_board_for_current_firmware(args.board)
    writer_or_transport = open_writer(args)
    if args.dry_run:
        controller = OrionMotorController(writer_or_transport, args.board)
        action(controller, None)
        return 0

    with writer_or_transport as transport:
        controller = OrionMotorController(transport.write, args.board)
        action(controller, transport)
    return 0


def command_run(args) -> int:
    def action(controller: OrionMotorController, transport: OrionCanTransport | None) -> None:
        period_s = 1.0 / args.rate
        start = time.perf_counter()
        next_send = start
        monitor = RunMonitor(args.board) if args.monitor else None
        try:
            while True:
                controller.set_speeds(args.m0, args.m1)
                if args.once:
                    break
                if args.duration > 0.0 and time.perf_counter() - start >= args.duration:
                    break
                next_send += period_s
                while True:
                    now = time.perf_counter()
                    sleep_s = next_send - now
                    if sleep_s <= 0.0:
                        break
                    if monitor is not None and transport is not None:
                        frame = transport.read(timeout_ms=min(5, max(1, int(sleep_s * 1000.0))))
                        if frame is not None:
                            monitor.update(frame.display_id, frame.data)
                    else:
                        time.sleep(sleep_s)
                        break
                else:
                    next_send = time.perf_counter()
        except KeyboardInterrupt:
            pass
        finally:
            if not args.no_stop_on_exit:
                controller.stop(count=args.stop_count, period_s=args.stop_period)
            if monitor is not None:
                stop_until = time.perf_counter() + 0.2
                while transport is not None and time.perf_counter() < stop_until:
                    frame = transport.read(timeout_ms=5)
                    if frame is not None:
                        monitor.update(frame.display_id, frame.data)
                print(monitor.summary())

    return run_with_writer(args, action)


def command_stop(args) -> int:
    return run_with_writer(args, lambda controller, _transport: controller.stop(count=args.count, period_s=args.period))


def command_freewheel(args) -> int:
    return run_with_writer(args, lambda controller, _transport: controller.freewheel(args.ms))


def command_reset(args) -> int:
    return run_with_writer(args, lambda controller, _transport: controller.reset())


def add_common_device_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--vid-pid", type=parse_vid_pid, default=None, help="USB VID:PID, default probes CANable/candleLight IDs")
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--timeout", type=int, default=100, help="USB timeout in ms")
    parser.add_argument("--loopback", action="store_true", help="start CANable in loopback mode")
    parser.add_argument("--dry-run", action="store_true", help="print frames without opening CANable")
    parser.add_argument("-v", "--verbose", action="store_true")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Orion_F303_BLDC motor command CLI")
    sub = parser.add_subparsers(dest="command", required=True)

    p_run = sub.add_parser("run", help="send periodic target speed commands")
    add_common_device_args(p_run)
    p_run.add_argument("--board", type=int, default=1)
    p_run.add_argument("--m0", type=float, default=None, help="M0 target speed in rps")
    p_run.add_argument("--m1", type=float, default=None, help="M1 target speed in rps")
    p_run.add_argument("--rate", type=float, default=50.0, help="send rate in Hz")
    p_run.add_argument("--duration", type=float, default=0.0, help="seconds; 0 means until Ctrl+C")
    p_run.add_argument("--once", action="store_true", help="send one command set and exit")
    p_run.add_argument("--monitor", action="store_true", help="read telemetry during run and print measured speed summary")
    p_run.add_argument("--no-stop-on-exit", action="store_true")
    p_run.add_argument("--stop-count", type=int, default=5)
    p_run.add_argument("--stop-period", type=float, default=0.02)
    p_run.set_defaults(func=command_run)

    p_stop = sub.add_parser("stop", help="send zero speed to both motors several times")
    add_common_device_args(p_stop)
    p_stop.add_argument("--board", type=int, default=1)
    p_stop.add_argument("--count", type=int, default=5)
    p_stop.add_argument("--period", type=float, default=0.02)
    p_stop.set_defaults(func=command_stop)

    p_free = sub.add_parser("freewheel", help="send firmware freewheel command")
    add_common_device_args(p_free)
    p_free.add_argument("--board", type=int, default=1)
    p_free.add_argument("--ms", type=int, default=100)
    p_free.set_defaults(func=command_freewheel)

    p_reset = sub.add_parser("reset", help="send MCU reset command")
    add_common_device_args(p_reset)
    p_reset.add_argument("--board", type=int, default=1)
    p_reset.set_defaults(func=command_reset)

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "run" and args.m0 is None and args.m1 is None:
        print("error: run requires --m0 and/or --m1", file=sys.stderr)
        return 2
    if hasattr(args, "rate") and args.rate <= 0.0:
        print("error: --rate must be > 0", file=sys.stderr)
        return 2
    try:
        return args.func(args)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

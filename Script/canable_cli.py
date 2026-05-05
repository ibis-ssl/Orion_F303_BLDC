#!/usr/bin/env python3
"""Minimal CANable/candleLight gs_usb CLI for Orion_F303_BLDC debugging."""

from __future__ import annotations

import argparse
import csv
import struct
import sys
import time
from dataclasses import dataclass
from typing import Iterable

try:
    import usb.core
    import usb.util
except ImportError as exc:
    raise SystemExit("pyusb is required: python -m pip install pyusb") from exc


VID_PID_DEFAULTS = ((0x1D50, 0x606F), (0x1209, 0x2323))

USB_DIR_OUT_VENDOR_INTERFACE = 0x41
USB_DIR_IN_VENDOR_INTERFACE = 0xC1

GS_USB_BREQ_HOST_FORMAT = 0
GS_USB_BREQ_BITTIMING = 1
GS_USB_BREQ_MODE = 2
GS_USB_BREQ_BT_CONST = 4
GS_USB_BREQ_DEVICE_CONFIG = 5

GS_CAN_MODE_RESET = 0
GS_CAN_MODE_START = 1
GS_CAN_MODE_LISTEN_ONLY = 1 << 0
GS_CAN_MODE_LOOP_BACK = 1 << 1

GS_HOST_FRAME_ECHO_ID_RX = 0xFFFFFFFF
GS_MAX_TX_URBS = 10

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x7FF
CAN_EFF_MASK = 0x1FFFFFFF

FRAME_STRUCT = struct.Struct("<IIBBBB8s")
BT_CONST_STRUCT = struct.Struct("<10I")
BITTIMING_STRUCT = struct.Struct("<5I")
MODE_STRUCT = struct.Struct("<2I")


@dataclass(frozen=True)
class BtConst:
    feature: int
    fclk_can: int
    tseg1_min: int
    tseg1_max: int
    tseg2_min: int
    tseg2_max: int
    sjw_max: int
    brp_min: int
    brp_max: int
    brp_inc: int


@dataclass(frozen=True)
class BitTiming:
    prop_seg: int
    phase_seg1: int
    phase_seg2: int
    sjw: int
    brp: int

    def pack(self) -> bytes:
        return BITTIMING_STRUCT.pack(self.prop_seg, self.phase_seg1, self.phase_seg2, self.sjw, self.brp)

    @property
    def total_tq(self) -> int:
        return 1 + self.prop_seg + self.phase_seg1 + self.phase_seg2

    def bitrate(self, fclk_can: int) -> float:
        return fclk_can / (self.brp * self.total_tq)


@dataclass(frozen=True)
class CanFrame:
    can_id: int
    data: bytes
    extended: bool = False
    remote: bool = False
    error: bool = False
    echo_id: int = GS_HOST_FRAME_ECHO_ID_RX
    channel: int = 0
    flags: int = 0

    @property
    def display_id(self) -> int:
        if self.extended:
            return self.can_id & CAN_EFF_MASK
        return self.can_id & CAN_SFF_MASK

    @property
    def raw_can_id(self) -> int:
        raw = self.display_id
        if self.extended:
            raw |= CAN_EFF_FLAG
        if self.remote:
            raw |= CAN_RTR_FLAG
        if self.error:
            raw |= CAN_ERR_FLAG
        return raw


def parse_vid_pid(text: str) -> tuple[int, int]:
    if ":" not in text:
        raise argparse.ArgumentTypeError("VID:PID must be hex, for example 1d50:606f")
    vid_s, pid_s = text.split(":", 1)
    return int(vid_s, 16), int(pid_s, 16)


def format_data(data: bytes, dlc: int | None = None) -> str:
    n = len(data) if dlc is None else min(dlc, len(data))
    return " ".join(f"{b:02X}" for b in data[:n])


def decode_target_frame(can_id: int, data: bytes) -> str:
    sid = can_id & CAN_SFF_MASK
    if len(data) < 8:
        return ""
    if sid == 0x000:
        err_id, info, value = struct.unpack("<HHf", data[:8])
        return f"fault id=0x{err_id:04X} info={info} value={value:+.3f}"
    if 0x200 <= sid <= 0x20F:
        speed, angle = struct.unpack("<ff", data[:8])
        return f"speed={speed:+.3f} angle={angle:+.3f}"
    if 0x210 <= sid <= 0x21F:
        v0, _ = struct.unpack("<ff", data[:8])
        return f"voltage={v0:+.3f}"
    if 0x220 <= sid <= 0x22F:
        tm, tf = struct.unpack("<ff", data[:8])
        return f"motor_temp={tm:+.1f} fet_temp={tf:+.1f}"
    if 0x230 <= sid <= 0x23F:
        cur, _ = struct.unpack("<ff", data[:8])
        return f"current={cur:+.3f}"
    if 0x500 <= sid <= 0x50F:
        rps_per_v, zero = struct.unpack("<ff", data[:8])
        return f"rps_per_v={rps_per_v:+.3f} zero={zero:+.3f}"
    return ""


class CanableGsUsb:
    def __init__(
        self,
        vid_pid: tuple[int, int] | None = None,
        channel: int = 0,
        timeout_ms: int = 100,
        verbose: bool = False,
    ) -> None:
        self.vid_pid = vid_pid
        self.channel = channel
        self.timeout_ms = timeout_ms
        self.verbose = verbose
        self.dev = None
        self.ep_in = 0x81
        self.ep_out = 0x02
        self.tx_echo_id = 0
        self.bt_const: BtConst | None = None

    def open(self) -> None:
        candidates = [self.vid_pid] if self.vid_pid else list(VID_PID_DEFAULTS)
        last = None
        for item in candidates:
            if item is None:
                continue
            vid, pid = item
            dev = usb.core.find(idVendor=vid, idProduct=pid)
            if dev is not None:
                self.dev = dev
                break
            last = item
        if self.dev is None:
            known = ", ".join(f"{vid:04x}:{pid:04x}" for vid, pid in candidates if vid is not None)
            raise RuntimeError(f"CANable/candleLight USB device not found ({known}). Check libusb driver.")

        try:
            self.dev.set_configuration()
        except usb.core.USBError:
            pass

        cfg = self.dev.get_active_configuration()
        intf = cfg[(0, 0)]
        try:
            if self.dev.is_kernel_driver_active(0):
                self.dev.detach_kernel_driver(0)
        except (NotImplementedError, usb.core.USBError):
            pass
        usb.util.claim_interface(self.dev, 0)

        for ep in intf:
            addr = ep.bEndpointAddress
            attr = usb.util.endpoint_type(ep.bmAttributes)
            if attr != usb.util.ENDPOINT_TYPE_BULK:
                continue
            if usb.util.endpoint_direction(addr) == usb.util.ENDPOINT_IN:
                self.ep_in = addr
            else:
                self.ep_out = addr

        if self.verbose:
            print(f"opened {self.dev.idVendor:04x}:{self.dev.idProduct:04x} in=0x{self.ep_in:02x} out=0x{self.ep_out:02x}", file=sys.stderr)

    def close(self) -> None:
        if self.dev is None:
            return
        try:
            self.mode(GS_CAN_MODE_RESET)
        except Exception:
            pass
        try:
            usb.util.release_interface(self.dev, 0)
        except Exception:
            pass
        usb.util.dispose_resources(self.dev)
        self.dev = None

    def ctrl_out(self, request: int, data: bytes, value: int | None = None) -> None:
        assert self.dev is not None
        self.dev.ctrl_transfer(
            USB_DIR_OUT_VENDOR_INTERFACE,
            request,
            self.channel if value is None else value,
            0,
            data,
            self.timeout_ms,
        )

    def ctrl_in(self, request: int, length: int, value: int | None = None) -> bytes:
        assert self.dev is not None
        data = self.dev.ctrl_transfer(
            USB_DIR_IN_VENDOR_INTERFACE,
            request,
            self.channel if value is None else value,
            0,
            length,
            self.timeout_ms,
        )
        return bytes(data)

    def host_format(self) -> None:
        self.ctrl_out(GS_USB_BREQ_HOST_FORMAT, struct.pack("<I", 0x0000BEEF), value=1)

    def read_device_config(self) -> tuple[int, int, int]:
        data = self.ctrl_in(GS_USB_BREQ_DEVICE_CONFIG, 12, value=1)
        if len(data) >= 12:
            _, _, _, icount, sw, hw = struct.unpack("<BBBBII", data[:12])
            return icount, sw, hw
        if len(data) >= 4:
            _, _, _, icount = struct.unpack("<BBBB", data[:4])
            return icount, 0, 0
        raise RuntimeError(f"short device config: {len(data)} bytes")

    def read_bt_const(self) -> BtConst:
        data = self.ctrl_in(GS_USB_BREQ_BT_CONST, BT_CONST_STRUCT.size)
        if len(data) < BT_CONST_STRUCT.size:
            raise RuntimeError(f"short bt_const: {len(data)} bytes")
        self.bt_const = BtConst(*BT_CONST_STRUCT.unpack(data[: BT_CONST_STRUCT.size]))
        return self.bt_const

    def mode(self, mode: int, flags: int = 0) -> None:
        self.ctrl_out(GS_USB_BREQ_MODE, MODE_STRUCT.pack(mode, flags))

    def set_bittiming(self, timing: BitTiming) -> None:
        self.ctrl_out(GS_USB_BREQ_BITTIMING, timing.pack())

    def start(self, bitrate: int, timing: BitTiming | None = None, listen_only: bool = False, loopback: bool = False) -> BitTiming:
        self.host_format()
        self.mode(GS_CAN_MODE_RESET)
        bt = self.read_bt_const()
        chosen = timing if timing else choose_bittiming(bt, bitrate)
        self.set_bittiming(chosen)
        flags = 0
        if listen_only:
            flags |= GS_CAN_MODE_LISTEN_ONLY
        if loopback:
            flags |= GS_CAN_MODE_LOOP_BACK
        self.mode(GS_CAN_MODE_START, flags)
        return chosen

    def read_frame(self, timeout_ms: int | None = None) -> CanFrame | None:
        assert self.dev is not None
        try:
            raw = bytes(self.dev.read(self.ep_in, 64, self.timeout_ms if timeout_ms is None else timeout_ms))
        except usb.core.USBTimeoutError:
            return None
        if len(raw) < FRAME_STRUCT.size:
            return None
        echo_id, can_id_raw, dlc, channel, flags, _reserved, data = FRAME_STRUCT.unpack(raw[: FRAME_STRUCT.size])
        extended = bool(can_id_raw & CAN_EFF_FLAG)
        remote = bool(can_id_raw & CAN_RTR_FLAG)
        error = bool(can_id_raw & CAN_ERR_FLAG)
        mask = CAN_EFF_MASK if extended else CAN_SFF_MASK
        return CanFrame(
            can_id=can_id_raw & mask,
            data=data[: min(dlc, 8)],
            extended=extended,
            remote=remote,
            error=error,
            echo_id=echo_id,
            channel=channel,
            flags=flags,
        )

    def write_frame(self, frame: CanFrame) -> None:
        assert self.dev is not None
        echo_id = self.tx_echo_id
        self.tx_echo_id = (self.tx_echo_id + 1) % GS_MAX_TX_URBS
        data = frame.data[:8].ljust(8, b"\x00")
        pkt = FRAME_STRUCT.pack(echo_id, frame.raw_can_id, len(frame.data[:8]), self.channel, frame.flags, 0, data)
        self.dev.write(self.ep_out, pkt, self.timeout_ms)


def choose_bittiming(bt: BtConst, bitrate: int) -> BitTiming:
    best: tuple[float, float, BitTiming] | None = None
    brp_inc = max(bt.brp_inc, 1)
    for brp in range(bt.brp_min, bt.brp_max + 1, brp_inc):
        total_tq_float = bt.fclk_can / (bitrate * brp)
        total_tq = round(total_tq_float)
        if total_tq < 3:
            continue
        actual = bt.fclk_can / (brp * total_tq)
        bitrate_error = abs(actual - bitrate) / bitrate
        if bitrate_error > 0.005:
            continue
        for tseg2 in range(bt.tseg2_min, bt.tseg2_max + 1):
            tseg1 = total_tq - 1 - tseg2
            if tseg1 < bt.tseg1_min or tseg1 > bt.tseg1_max:
                continue
            sjw = min(bt.sjw_max, tseg2, 1)
            prop = 1 if tseg1 > 1 else 0
            phase1 = tseg1 - prop
            sample_point = (1 + tseg1) / total_tq
            sample_error = abs(sample_point - 0.875)
            timing = BitTiming(prop, phase1, tseg2, sjw, brp)
            score = (bitrate_error, sample_error)
            if best is None or score < (best[0], best[1]):
                best = (bitrate_error, sample_error, timing)
    if best is None:
        raise RuntimeError(
            f"no bit timing for bitrate={bitrate}, fclk={bt.fclk_can}, "
            f"tseg1={bt.tseg1_min}-{bt.tseg1_max}, tseg2={bt.tseg2_min}-{bt.tseg2_max}, brp={bt.brp_min}-{bt.brp_max}/{bt.brp_inc}"
        )
    return best[2]


def parse_data_hex(items: Iterable[str]) -> bytes:
    data = bytearray()
    for item in items:
        text = item.replace(":", " ").replace(",", " ")
        for part in text.split():
            if len(data) >= 8:
                raise argparse.ArgumentTypeError("classic CAN data length must be <= 8")
            data.append(int(part, 16) & 0xFF)
    return bytes(data)


def print_frame(frame: CanFrame, start_time: float, csv_writer=None) -> None:
    now = time.perf_counter() - start_time
    direction = "RX" if frame.echo_id == GS_HOST_FRAME_ECHO_ID_RX else f"ECHO{frame.echo_id}"
    id_width = 8 if frame.extended else 3
    decoded = decode_target_frame(frame.display_id, frame.data)
    if csv_writer is not None:
        row = [f"{now:.6f}", direction, f"{frame.display_id:X}", len(frame.data)]
        row.extend(frame.data[i] if i < len(frame.data) else "" for i in range(8))
        row.append(decoded)
        csv_writer.writerow(row)
    print(f"{now:9.6f} {direction:>6} {frame.display_id:0{id_width}X} {len(frame.data)} {format_data(frame.data):<23} {decoded}".rstrip())


def command_info(args) -> int:
    dev = CanableGsUsb(args.vid_pid, channel=args.channel, timeout_ms=args.timeout, verbose=args.verbose)
    try:
        dev.open()
        dev.host_format()
        icount, sw, hw = dev.read_device_config()
        bt = dev.read_bt_const()
        print(f"device {dev.dev.idVendor:04x}:{dev.dev.idProduct:04x} channels={icount} sw=0x{sw:08x} hw=0x{hw:08x}")
        print(
            f"bt_const feature=0x{bt.feature:08x} fclk={bt.fclk_can} "
            f"tseg1={bt.tseg1_min}-{bt.tseg1_max} tseg2={bt.tseg2_min}-{bt.tseg2_max} "
            f"sjw_max={bt.sjw_max} brp={bt.brp_min}-{bt.brp_max}/{bt.brp_inc}"
        )
        timing = choose_bittiming(bt, args.bitrate)
        print(
            f"timing bitrate={timing.bitrate(bt.fclk_can):.1f} "
            f"prop={timing.prop_seg} phase1={timing.phase_seg1} phase2={timing.phase_seg2} sjw={timing.sjw} brp={timing.brp}"
        )
    finally:
        dev.close()
    return 0


def command_capture(args) -> int:
    dev = CanableGsUsb(args.vid_pid, channel=args.channel, timeout_ms=args.timeout, verbose=args.verbose)
    csv_file = None
    csv_writer = None
    try:
        dev.open()
        timing = dev.start(args.bitrate, listen_only=args.listen_only, loopback=args.loopback)
        bt = dev.bt_const
        assert bt is not None
        print(
            f"# started bitrate={timing.bitrate(bt.fclk_can):.1f} "
            f"prop={timing.prop_seg} phase1={timing.phase_seg1} phase2={timing.phase_seg2} sjw={timing.sjw} brp={timing.brp}",
            flush=True,
        )
        if args.csv:
            csv_file = open(args.csv, "w", newline="", encoding="utf-8")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["time_s", "dir", "id", "dlc", "b0", "b1", "b2", "b3", "b4", "b5", "b6", "b7", "decoded"])
        start = time.perf_counter()
        while True:
            if args.duration > 0 and time.perf_counter() - start >= args.duration:
                break
            frame = dev.read_frame()
            if frame is None:
                continue
            print_frame(frame, start, csv_writer)
    except KeyboardInterrupt:
        pass
    finally:
        if csv_file:
            csv_file.close()
        dev.close()
    return 0


def command_send(args) -> int:
    dev = CanableGsUsb(args.vid_pid, channel=args.channel, timeout_ms=args.timeout, verbose=args.verbose)
    try:
        dev.open()
        timing = dev.start(args.bitrate, listen_only=False, loopback=args.loopback)
        bt = dev.bt_const
        assert bt is not None
        if args.verbose:
            print(f"started bitrate={timing.bitrate(bt.fclk_can):.1f}", file=sys.stderr)
        frame = CanFrame(can_id=args.can_id, data=parse_data_hex(args.data), extended=args.extended, remote=args.remote)
        for _ in range(args.count):
            dev.write_frame(frame)
            if args.period > 0:
                time.sleep(args.period)
    finally:
        dev.close()
    return 0


def command_send_speed(args) -> int:
    can_id = (0x100 + args.board * 2 + args.motor) & CAN_SFF_MASK
    data = struct.pack("<ff", float(args.rps), 0.0)
    args.can_id = can_id
    args.data = [format_data(data)]
    args.extended = False
    args.remote = False
    return command_send(args)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="CANable/candleLight gs_usb CLI")
    parser.add_argument("--vid-pid", type=parse_vid_pid, default=None, help="USB VID:PID, default 1d50:606f or 1209:2323")
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--timeout", type=int, default=100, help="USB timeout in ms")
    parser.add_argument("-v", "--verbose", action="store_true")
    sub = parser.add_subparsers(dest="command", required=True)

    p_info = sub.add_parser("info", help="show CANable gs_usb configuration")
    p_info.set_defaults(func=command_info)

    p_cap = sub.add_parser("capture", help="capture CAN frames")
    p_cap.add_argument("--duration", type=float, default=10.0, help="seconds, 0 means until Ctrl+C")
    p_cap.add_argument("--csv", default="", help="optional CSV output path")
    p_cap.add_argument("--listen-only", action="store_true")
    p_cap.add_argument("--loopback", action="store_true")
    p_cap.set_defaults(func=command_capture)

    p_send = sub.add_parser("send", help="send a classic CAN frame")
    p_send.add_argument("can_id", type=lambda x: int(x, 0))
    p_send.add_argument("data", nargs="*", help="hex bytes, for example 01 02 0A or 01:02:0A")
    p_send.add_argument("--extended", action="store_true")
    p_send.add_argument("--remote", action="store_true")
    p_send.add_argument("--count", type=int, default=1)
    p_send.add_argument("--period", type=float, default=0.0)
    p_send.add_argument("--loopback", action="store_true")
    p_send.set_defaults(func=command_send)

    p_speed = sub.add_parser("send-speed", help="send Orion target speed command")
    p_speed.add_argument("--board", type=int, required=True)
    p_speed.add_argument("--motor", type=int, choices=(0, 1), required=True)
    p_speed.add_argument("--rps", type=float, required=True)
    p_speed.add_argument("--count", type=int, default=1)
    p_speed.add_argument("--period", type=float, default=0.0)
    p_speed.add_argument("--loopback", action="store_true")
    p_speed.set_defaults(func=command_send_speed)

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except usb.core.USBError as exc:
        print(f"USB error: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

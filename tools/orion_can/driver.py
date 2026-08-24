"""WeAct USB2CANFDV1を介したOrion CAN送受信と安全な周期速度送信を担う。"""

from __future__ import annotations

import queue
import struct
import threading
import time
from dataclasses import dataclass
import serial


class OrionCanError(RuntimeError):
    """CANアダプタまたはプロトコルの異常を表す。"""


@dataclass(frozen=True, slots=True)
class CanFrame:
    can_id: int
    data: bytes = b""

    def __post_init__(self) -> None:
        if not 0 <= self.can_id <= 0x7FF:
            raise ValueError("標準CAN IDは0x000..0x7FFで指定してください")
        if len(self.data) > 8:
            raise ValueError("Classical CANのデータ長は最大8 byteです")

    def to_slcan(self) -> bytes:
        return f"t{self.can_id:03X}{len(self.data):X}{self.data.hex().upper()}\r".encode("ascii")

    @staticmethod
    def from_slcan(line: bytes) -> "CanFrame":
        if not line or line[:1] != b"t":
            raise OrionCanError(f"未対応のSLCANフレームです: {line!r}")
        try:
            can_id = int(line[1:4], 16)
            dlc = int(line[4:5], 16)
            payload = bytes.fromhex(line[5:].decode("ascii"))
        except (ValueError, UnicodeDecodeError) as exc:
            raise OrionCanError(f"不正なSLCANフレームです: {line!r}") from exc
        if dlc > 8 or len(payload) != dlc:
            raise OrionCanError(f"DLCとデータ長が一致しません: {line!r}")
        return CanFrame(can_id, payload)


@dataclass(slots=True)
class _TxRequest:
    frame: CanFrame
    completed: threading.Event
    error: Exception | None = None


class OrionCanDriver:
    """シリアルポートを単一ワーカーで所有するOrion用CANドライバ。"""

    def __init__(
        self,
        port: str,
        *,
        serial_baudrate: int = 1_000_000,
        bitrate_code: str = "S8",
        command_period_s: float = 0.020,
        command_watchdog_s: float = 0.200,
    ) -> None:
        if command_period_s <= 0:
            raise ValueError("command_period_sは正数で指定してください")
        if command_watchdog_s < command_period_s:
            raise ValueError("command_watchdog_sは送信周期以上にしてください")
        self._port_name = port
        self._serial_baudrate = serial_baudrate
        self._bitrate_code = bitrate_code
        self._period = command_period_s
        self._watchdog = command_watchdog_s
        self._serial: serial.Serial | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._tx_queue: queue.Queue[_TxRequest] = queue.Queue(maxsize=64)
        self._rx_queue: queue.Queue[CanFrame] = queue.Queue(maxsize=1024)
        self._target_lock = threading.Lock()
        self._targets: dict[int, tuple[float, float]] = {}
        self._periodic_tx_enabled = True
        self._error: Exception | None = None

    def open(self) -> None:
        if self._serial is not None:
            return
        port = serial.Serial(self._port_name, self._serial_baudrate, timeout=0.010, write_timeout=0.2)
        try:
            self._configure(port)
        except Exception:
            port.close()
            raise
        self._serial = port
        self._stop.clear()
        self._error = None
        self._thread = threading.Thread(target=self._worker, name="orion-can-io", daemon=True)
        self._thread.start()

    def close(self, *, send_stop: bool = True) -> None:
        port = self._serial
        if port is None:
            return
        try:
            if send_stop:
                self.stop_all(repetitions=5)
        finally:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=1.0)
            try:
                port.write(b"C\r")
                port.flush()
            finally:
                port.close()
                self._serial = None
                self._thread = None

    def __enter__(self) -> "OrionCanDriver":
        self.open()
        return self

    def __exit__(self, _type: object, _value: object, _traceback: object) -> None:
        self.close()

    def set_speed(self, board_id: int, motor: int, speed_rps: float) -> None:
        self._validate_motor(board_id, motor)
        with self._target_lock:
            self._targets[board_id * 2 + motor] = (float(speed_rps), time.monotonic())

    def clear_speed(self, board_id: int, motor: int) -> None:
        self.set_speed(board_id, motor, 0.0)

    def set_periodic_tx_enabled(self, enabled: bool) -> None:
        """速度指令の周期CAN送信を開始または完全停止する。"""
        with self._target_lock:
            self._periodic_tx_enabled = enabled

    def stop_all(self, *, repetitions: int = 5) -> None:
        if self._serial is None:
            return
        with self._target_lock:
            channels = list(self._targets)
            self._targets = {channel: (0.0, time.monotonic()) for channel in channels}
        for _ in range(repetitions):
            for channel in channels:
                self.send(self._speed_frame(channel, 0.0), timeout=0.2)
            time.sleep(self._period)

    def send(self, frame: CanFrame, *, timeout: float = 0.5) -> None:
        self._raise_if_unavailable()
        request = _TxRequest(frame, threading.Event())
        try:
            self._tx_queue.put(request, timeout=timeout)
        except queue.Full as exc:
            raise OrionCanError("CAN送信キューが満杯です") from exc
        if not request.completed.wait(timeout):
            raise OrionCanError("CAN送信がタイムアウトしました")
        if request.error is not None:
            raise OrionCanError("CAN送信に失敗しました") from request.error

    def receive(self, timeout: float | None = None) -> CanFrame | None:
        self._raise_if_unavailable()
        try:
            return self._rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _raise_if_unavailable(self) -> None:
        if self._serial is None or self._thread is None:
            raise OrionCanError("CANドライバが開かれていません")
        if self._error is not None:
            raise OrionCanError("CAN I/Oワーカーが停止しました") from self._error

    @staticmethod
    def _validate_motor(board_id: int, motor: int) -> None:
        if board_id not in (0, 1):
            raise ValueError("現行ファームで使用できるboard_idは0または1です")
        if motor not in (0, 1):
            raise ValueError("motorは0または1です")

    @staticmethod
    def _speed_frame(channel: int, speed_rps: float) -> CanFrame:
        return CanFrame(0x100 + channel, struct.pack("<f", speed_rps) + b"\x00" * 4)

    def _configure(self, port: serial.Serial) -> None:
        port.reset_input_buffer()
        for command, allow_bel in ((b"C\r", True), (b"M0\r", False), (b"A0\r", False),
                                   (self._bitrate_code.encode("ascii") + b"\r", False), (b"O\r", False)):
            self._command(port, command, allow_bel=allow_bel)

    @staticmethod
    def _command(port: serial.Serial, command: bytes, *, allow_bel: bool) -> None:
        port.write(command)
        port.flush()
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            value = port.read(1)
            if value == b"\r":
                return
            if value == b"\x07":
                if allow_bel:
                    return
                raise OrionCanError(f"アダプタがコマンドを拒否しました: {command!r}")
        raise OrionCanError(f"アダプタ応答がタイムアウトしました: {command!r}")

    def _worker(self) -> None:
        assert self._serial is not None
        port = self._serial
        rx_buffer = bytearray()
        next_periodic = time.monotonic()
        try:
            while not self._stop.is_set():
                self._process_one_tx(port)
                now = time.monotonic()
                if now >= next_periodic:
                    self._send_periodic(port, now)
                    next_periodic = now + self._period
                chunk = port.read(256)
                if chunk:
                    rx_buffer.extend(chunk)
                    self._parse_input(rx_buffer)
        except Exception as exc:
            self._error = exc
            self._stop.set()

    def _process_one_tx(self, port: serial.Serial) -> None:
        try:
            request = self._tx_queue.get_nowait()
        except queue.Empty:
            return
        try:
            port.write(request.frame.to_slcan())
            port.flush()
        except Exception as exc:
            request.error = exc
        finally:
            request.completed.set()

    def _send_periodic(self, port: serial.Serial, now: float) -> None:
        with self._target_lock:
            if not self._periodic_tx_enabled:
                return
            targets = list(self._targets.items())
        for channel, (speed, updated_at) in targets:
            effective_speed = speed if now - updated_at <= self._watchdog else 0.0
            port.write(self._speed_frame(channel, effective_speed).to_slcan())
        if targets:
            port.flush()

    def _parse_input(self, buffer: bytearray) -> None:
        while True:
            try:
                end = buffer.index(0x0D)
            except ValueError:
                return
            line = bytes(buffer[:end])
            del buffer[: end + 1]
            if not line:
                continue
            if line == b"\x07":
                raise OrionCanError("アダプタがCAN送信を拒否しました")
            if line[:1] != b"t":
                continue
            frame = CanFrame.from_slcan(line)
            try:
                self._rx_queue.put_nowait(frame)
            except queue.Full:
                self._rx_queue.get_nowait()
                self._rx_queue.put_nowait(frame)

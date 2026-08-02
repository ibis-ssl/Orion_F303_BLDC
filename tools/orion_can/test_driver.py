"""Orion CAN低層ドライバのフレーム変換と入力検証をテストする。"""

import struct
import unittest

from .driver import CanFrame, OrionCanDriver, OrionCanError
from .gui import decode_speed, parse_target


class CanFrameTest(unittest.TestCase):
    def test_slcan_round_trip(self) -> None:
        frame = CanFrame(0x123, bytes.fromhex("0102A0"))
        self.assertEqual(frame.to_slcan(), b"t12330102A0\r")
        self.assertEqual(CanFrame.from_slcan(b"t12330102A0"), frame)

    def test_rejects_invalid_dlc(self) -> None:
        with self.assertRaises(OrionCanError):
            CanFrame.from_slcan(b"t123201")

    def test_speed_frame_is_float32_little_endian(self) -> None:
        frame = OrionCanDriver._speed_frame(3, -12.5)
        self.assertEqual(frame.can_id, 0x103)
        self.assertEqual(frame.data[:4], struct.pack("<f", -12.5))
        self.assertEqual(frame.data[4:], b"\x00" * 4)

    def test_rejects_unsupported_board(self) -> None:
        with self.assertRaises(ValueError):
            OrionCanDriver._validate_motor(2, 0)

    def test_gui_target_validation(self) -> None:
        self.assertEqual(parse_target("-12.5"), -12.5)
        with self.assertRaises(ValueError):
            parse_target("81")
        with self.assertRaises(ValueError):
            parse_target("not-a-number")

    def test_speed_telemetry_decode(self) -> None:
        frame = CanFrame(0x203, struct.pack("<ff", 3.25, 1.0))
        self.assertEqual(decode_speed(frame, 1), (1, 3.25))
        self.assertIsNone(decode_speed(frame, 0))


if __name__ == "__main__":
    unittest.main()

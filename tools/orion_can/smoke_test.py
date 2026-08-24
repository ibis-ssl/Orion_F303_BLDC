"""接続実機に対して速度0送信とOrionテレメトリ受信を確認する。"""

from __future__ import annotations

import argparse
import time

from .driver import OrionCanDriver


def main() -> int:
    parser = argparse.ArgumentParser(description="Orion CAN低層ドライバ実機確認")
    parser.add_argument("--port", required=True)
    parser.add_argument("--board", type=int, choices=(0, 1), default=0)
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()

    telemetry_ids = {0x200 + args.board * 2, 0x201 + args.board * 2}
    received = 0
    with OrionCanDriver(args.port) as driver:
        driver.set_speed(args.board, 0, 0.0)
        driver.set_speed(args.board, 1, 0.0)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            frame = driver.receive(timeout=0.1)
            if frame is not None and frame.can_id in telemetry_ids:
                received += 1
        driver.stop_all()

    print(f"port={args.port} board={args.board} telemetry_frames={received} zero_tx=ok")
    if received == 0:
        print("status=failed: Orionテレメトリを受信できませんでした")
        return 1
    print("status=ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

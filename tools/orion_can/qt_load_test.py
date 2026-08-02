"""PyQtGraph版GUIで実機運転中のスレッド負荷をCSVへ記録する。"""

from __future__ import annotations

import argparse
import csv
import threading
import time
from pathlib import Path

from PySide6 import QtCore, QtWidgets

from .load_test import thread_cpu_seconds
from .qt_gui import MotorControlWindow


def main() -> int:
    parser = argparse.ArgumentParser(description="Orion PyQtGraph GUI実機負荷試験")
    parser.add_argument("--port", default="COM175")
    parser.add_argument("--board", type=int, choices=(0, 1), default=1)
    parser.add_argument("--motor", type=int, choices=(0, 1), default=0)
    parser.add_argument("--speed", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--output", type=Path, default=Path("Log/orion_qt_gui_load.csv"))
    args = parser.parse_args()
    if args.duration < 10.0:
        parser.error("--durationは10秒以上にしてください")

    qt_app = QtWidgets.QApplication([])
    window = MotorControlWindow()
    render_durations: list[float] = []
    original_render = window._update_plots

    def measured_render() -> None:
        started = time.perf_counter()
        original_render()
        render_durations.append(time.perf_counter() - started)

    window.plot_timer.timeout.disconnect()
    window.plot_timer.timeout.connect(measured_render)
    window.show()
    window.port_combo.setCurrentText(args.port)
    window.board_spin.setValue(args.board)
    window._toggle_connection()
    if window.driver is None or window.driver._thread is None or window.driver._thread.native_id is None:
        raise RuntimeError("CAN接続またはI/Oスレッド起動に失敗しました")
    window.speed_sliders[1 - args.motor].setValue(0)
    window.speed_sliders[args.motor].setValue(round(args.speed * 2.0))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "elapsed_s", "process_cpu_pct_one_core", "gui_thread_cpu_pct", "io_thread_cpu_pct",
        "plot_update_avg_ms", "plot_update_max_ms", "speed_points_m0", "speed_points_m1",
        "current_points_m0", "current_points_m1", "rx_total", "rx_queue",
    ]
    stream = args.output.open("w", newline="", encoding="utf-8-sig")
    writer = csv.DictWriter(stream, fieldnames=fieldnames)
    writer.writeheader()
    stream.flush()

    main_id = threading.get_native_id()
    io_id = window.driver._thread.native_id
    state = {
        "start": time.perf_counter(), "wall": time.perf_counter(), "process": time.process_time(),
        "main": thread_cpu_seconds(main_id), "io": thread_cpu_seconds(io_id), "render_index": 0,
    }
    safety_timer: threading.Timer | None = None

    def safety_stop() -> None:
        if window.driver is not None:
            try:
                window.driver.stop_all()
            except Exception as exc:
                print(f"SAFETY STOP ERROR: {exc}", flush=True)

    def sample() -> None:
        now = time.perf_counter()
        wall_delta = now - state["wall"]
        process_now, main_now, io_now = time.process_time(), thread_cpu_seconds(main_id), thread_cpu_seconds(io_id)
        recent = render_durations[int(state["render_index"]):]
        row = {
            "elapsed_s": now - state["start"],
            "process_cpu_pct_one_core": (process_now - state["process"]) / wall_delta * 100.0,
            "gui_thread_cpu_pct": (main_now - state["main"]) / wall_delta * 100.0,
            "io_thread_cpu_pct": (io_now - state["io"]) / wall_delta * 100.0,
            "plot_update_avg_ms": sum(recent) / len(recent) * 1000.0 if recent else 0.0,
            "plot_update_max_ms": max(recent) * 1000.0 if recent else 0.0,
            "speed_points_m0": len(window.speed_history[0]), "speed_points_m1": len(window.speed_history[1]),
            "current_points_m0": len(window.current_history[0]), "current_points_m1": len(window.current_history[1]),
            "rx_total": window.rx_total, "rx_queue": window.driver._rx_queue.qsize(),
        }
        writer.writerow(row)
        stream.flush()
        print(" ".join(f"{key}={value:.2f}" if isinstance(value, float) else f"{key}={value}" for key, value in row.items()), flush=True)
        state.update(wall=now, process=process_now, main=main_now, io=io_now, render_index=len(render_durations))
        if row["elapsed_s"] >= args.duration:
            window._stop_motion()
            window.close()
            qt_app.quit()

    timer = QtCore.QTimer()
    timer.timeout.connect(sample)
    timer.start(1000)
    safety_timer = threading.Timer(args.duration + 5.0, safety_stop)
    safety_timer.daemon = True
    safety_timer.start()
    try:
        result = qt_app.exec()
    finally:
        safety_timer.cancel()
        if window.driver is not None:
            window._stop_motion()
            window._disconnect()
        stream.close()
    print(f"log={args.output.resolve()}")
    return result


if __name__ == "__main__":
    raise SystemExit(main())

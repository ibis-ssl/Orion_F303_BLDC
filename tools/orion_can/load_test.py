"""実機モーター運転中のGUI/CANスレッド負荷と描画履歴増加をCSVへ記録する。"""

from __future__ import annotations

import argparse
import csv
import ctypes
import threading
import time
from pathlib import Path

from . import gui

THREAD_QUERY_LIMITED_INFORMATION = 0x0800


class FileTime(ctypes.Structure):
    _fields_ = (("low", ctypes.c_uint32), ("high", ctypes.c_uint32))

    def seconds(self) -> float:
        return ((self.high << 32) | self.low) / 10_000_000.0


def thread_cpu_seconds(thread_id: int) -> float:
    """Windows native threadのkernel+user CPU時間を秒で返す。"""
    kernel32 = ctypes.windll.kernel32
    handle = kernel32.OpenThread(THREAD_QUERY_LIMITED_INFORMATION, False, thread_id)
    if not handle:
        raise OSError(ctypes.get_last_error(), f"OpenThread failed: {thread_id}")
    creation, exit_time, kernel, user = FileTime(), FileTime(), FileTime(), FileTime()
    try:
        if not kernel32.GetThreadTimes(handle, ctypes.byref(creation), ctypes.byref(exit_time), ctypes.byref(kernel), ctypes.byref(user)):
            raise OSError(ctypes.get_last_error(), f"GetThreadTimes failed: {thread_id}")
        return kernel.seconds() + user.seconds()
    finally:
        kernel32.CloseHandle(handle)


def main() -> int:
    parser = argparse.ArgumentParser(description="Orion GUI実機負荷試験")
    parser.add_argument("--port", default="COM175")
    parser.add_argument("--board", type=int, choices=(0, 1), default=1)
    parser.add_argument("--motor", type=int, choices=(0, 1), default=0)
    parser.add_argument("--speed", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--output", type=Path, default=Path("Log/orion_gui_load.csv"))
    parser.add_argument("--visible", action="store_true", help="計測中のGUIウィンドウを表示する")
    args = parser.parse_args()
    if args.duration < 10.0:
        parser.error("--durationは10秒以上にしてください")

    render_durations: list[float] = []
    original_render = gui.MotorControlGui._render_plots

    def measured_render(self: gui.MotorControlGui) -> None:
        started = time.perf_counter()
        original_render(self)
        render_durations.append(time.perf_counter() - started)

    gui.MotorControlGui._render_plots = measured_render
    gui.messagebox.showerror = lambda title, message: print(f"ERROR {title}: {message}")
    gui.messagebox.showwarning = lambda title, message: print(f"WARNING {title}: {message}")

    root = gui.tk.Tk()
    if not args.visible:
        root.withdraw()
    app = gui.MotorControlGui(root)
    fieldnames = [
        "elapsed_s", "process_cpu_pct_one_core", "gui_thread_cpu_pct", "io_thread_cpu_pct",
        "plot_render_avg_ms", "plot_render_max_ms", "speed_points_m0", "speed_points_m1",
        "current_points_m0", "current_points_m1", "rx_total", "rx_queue",
    ]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    stream = args.output.open("w", newline="", encoding="utf-8-sig")
    writer = csv.DictWriter(stream, fieldnames=fieldnames)
    writer.writeheader()
    stream.flush()
    safety_timer: threading.Timer | None = None
    try:
        app.port.set(args.port)
        app.board.set(args.board)
        app._toggle_connection()
        if app.driver is None or app.driver._thread is None or app.driver._thread.native_id is None:
            raise RuntimeError("CAN接続またはI/Oスレッド起動に失敗しました")

        other_motor = 1 - args.motor
        app._slider_changed(other_motor, "0.0")
        app._slider_changed(args.motor, str(args.speed))

        main_thread_id = threading.get_native_id()
        io_thread_id = app.driver._thread.native_id
        state = {
            "start": time.perf_counter(),
            "last_wall": time.perf_counter(),
            "last_process": time.process_time(),
            "last_main": thread_cpu_seconds(main_thread_id),
            "last_io": thread_cpu_seconds(io_thread_id),
            "last_render_index": 0,
        }

        def safety_stop() -> None:
            if app.driver is not None:
                try:
                    app.driver.stop_all()
                except Exception as exc:
                    print(f"SAFETY STOP ERROR: {exc}", flush=True)

        def sample() -> None:
            now = time.perf_counter()
            wall_delta = now - state["last_wall"]
            process_now = time.process_time()
            main_now = thread_cpu_seconds(main_thread_id)
            io_now = thread_cpu_seconds(io_thread_id)
            render_index = int(state["last_render_index"])
            recent_renders = render_durations[render_index:]
            row = {
                "elapsed_s": now - state["start"],
                "process_cpu_pct_one_core": (process_now - state["last_process"]) / wall_delta * 100.0,
                "gui_thread_cpu_pct": (main_now - state["last_main"]) / wall_delta * 100.0,
                "io_thread_cpu_pct": (io_now - state["last_io"]) / wall_delta * 100.0,
                "plot_render_avg_ms": (sum(recent_renders) / len(recent_renders) * 1000.0) if recent_renders else 0.0,
                "plot_render_max_ms": (max(recent_renders) * 1000.0) if recent_renders else 0.0,
                "speed_points_m0": len(app.speed_history[0]),
                "speed_points_m1": len(app.speed_history[1]),
                "current_points_m0": len(app.current_history[0]),
                "current_points_m1": len(app.current_history[1]),
                "rx_total": app.rx_total,
                "rx_queue": app.driver._rx_queue.qsize(),
            }
            writer.writerow(row)
            stream.flush()
            print(" ".join(f"{key}={value:.2f}" if isinstance(value, float) else f"{key}={value}" for key, value in row.items()), flush=True)
            state.update(last_wall=now, last_process=process_now, last_main=main_now, last_io=io_now, last_render_index=len(render_durations))
            if row["elapsed_s"] >= args.duration:
                app._stop_motion()
                root.quit()
            else:
                root.after(1000, sample)

        safety_timer = threading.Timer(args.duration + 5.0, safety_stop)
        safety_timer.daemon = True
        safety_timer.start()
        root.after(1000, sample)
        root.mainloop()
    finally:
        if safety_timer is not None:
            safety_timer.cancel()
        app._stop_motion()
        app._disconnect()
        try:
            root.destroy()
        except gui.tk.TclError:
            pass
        stream.close()
    print(f"log={args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

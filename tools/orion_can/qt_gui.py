"""PySide6とPyQtGraphでOrionモーター制御と大量点テレメトリ表示を行う。"""

from __future__ import annotations

import sys
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets
from serial.tools import list_ports

from .driver import OrionCanDriver, OrionCanError
from .telemetry import MotorTelemetry, apply_telemetry_frame

MAX_SPEED_RPS = 80.0
PLOT_WINDOW_S = 10.0
RX_INTERVAL_MS = 20
DISPLAY_INTERVAL_MS = 100
PLOT_INTERVAL_MS = 250
MAX_RX_PER_TICK = 1024


class MotorControlWindow(QtWidgets.QMainWindow):
    """CAN制御、物理量表示、全受信点プロットを統合するメイン画面。"""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Orion CAN Motor Control - PyQtGraph")
        self.driver: OrionCanDriver | None = None
        self.running = False
        self.targets = [0.0, 0.0]
        self.telemetry = [MotorTelemetry(), MotorTelemetry()]
        self.speed_history: list[deque[tuple[float, float]]] = [deque(), deque()]
        self.command_history: list[deque[tuple[float, float]]] = [deque(), deque()]
        self.current_history: list[deque[tuple[float, float]]] = [deque(), deque()]
        self.rx_total = 0

        pg.setConfigOptions(antialias=False, useOpenGL=True)
        self._build_ui()
        self._refresh_ports()

        self.rx_timer = QtCore.QTimer(self)
        self.rx_timer.timeout.connect(self._service_can)
        self.rx_timer.start(RX_INTERVAL_MS)
        self.display_timer = QtCore.QTimer(self)
        self.display_timer.timeout.connect(self._update_display)
        self.display_timer.start(DISPLAY_INTERVAL_MS)
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self._update_plots)
        self.plot_timer.start(PLOT_INTERVAL_MS)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        connection = QtWidgets.QGroupBox("CAN接続")
        connection_layout = QtWidgets.QHBoxLayout(connection)
        connection_layout.addWidget(QtWidgets.QLabel("ポート"))
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setEditable(True)
        connection_layout.addWidget(self.port_combo)
        refresh = QtWidgets.QPushButton("更新")
        refresh.clicked.connect(self._refresh_ports)
        connection_layout.addWidget(refresh)
        connection_layout.addWidget(QtWidgets.QLabel("Board ID"))
        self.board_spin = QtWidgets.QSpinBox()
        self.board_spin.setRange(0, 1)
        self.board_spin.setValue(1)
        connection_layout.addWidget(self.board_spin)
        self.connect_button = QtWidgets.QPushButton("接続")
        self.connect_button.clicked.connect(self._toggle_connection)
        connection_layout.addWidget(self.connect_button)
        layout.addWidget(connection)

        control = QtWidgets.QGroupBox("速度指令（接続中は自動反映）")
        control_layout = QtWidgets.QGridLayout(control)
        control_layout.addWidget(QtWidgets.QLabel("モーター"), 0, 0)
        control_layout.addWidget(QtWidgets.QLabel("-80 rps"), 0, 1)
        control_layout.addWidget(QtWidgets.QLabel("スライダ"), 0, 2, alignment=QtCore.Qt.AlignmentFlag.AlignCenter)
        control_layout.addWidget(QtWidgets.QLabel("+80 rps"), 0, 3)
        control_layout.addWidget(QtWidgets.QLabel("指定速度"), 0, 4)
        self.speed_sliders: list[QtWidgets.QSlider] = []
        self.target_labels: list[QtWidgets.QLabel] = []
        for motor in range(2):
            control_layout.addWidget(QtWidgets.QLabel(f"Motor {motor}"), motor + 1, 0)
            slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
            slider.setRange(-160, 160)
            slider.setSingleStep(1)
            slider.setValue(0)
            slider.valueChanged.connect(lambda value, index=motor: self._slider_changed(index, value))
            control_layout.addWidget(slider, motor + 1, 1, 1, 3)
            label = QtWidgets.QLabel("+0.0 rps")
            label.setMinimumWidth(85)
            label.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)
            control_layout.addWidget(label, motor + 1, 4)
            zero = QtWidgets.QPushButton("0に設定")
            zero.clicked.connect(lambda _checked=False, index=motor: self.speed_sliders[index].setValue(0))
            control_layout.addWidget(zero, motor + 1, 5)
            self.speed_sliders.append(slider)
            self.target_labels.append(label)
        self.stop_button = QtWidgets.QPushButton("停止（両モーター 0 rps）")
        self.stop_button.setStyleSheet("QPushButton { background:#c62828; color:white; font-weight:bold; padding:8px; }")
        self.stop_button.clicked.connect(self._stop_motion)
        control_layout.addWidget(self.stop_button, 3, 0, 1, 6)
        layout.addWidget(control)

        telemetry_group = QtWidgets.QGroupBox("モーターテレメトリ")
        telemetry_layout = QtWidgets.QGridLayout(telemetry_group)
        headers = ("モーター", "指定速度", "現在速度", "Encoder raw", "電圧", "電流", "Motor温度", "FET温度")
        for column, header in enumerate(headers):
            telemetry_layout.addWidget(QtWidgets.QLabel(header), 0, column)
        self.value_labels: list[list[QtWidgets.QLabel]] = []
        for motor in range(2):
            telemetry_layout.addWidget(QtWidgets.QLabel(f"Motor {motor}"), motor + 1, 0)
            values = [QtWidgets.QLabel("--") for _ in range(7)]
            for column, label in enumerate(values, start=1):
                label.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)
                label.setMinimumWidth(82)
                telemetry_layout.addWidget(label, motor + 1, column)
            self.value_labels.append(values)
        self.rx_label = QtWidgets.QLabel("復号済みCANテレメトリ: 0 frame")
        telemetry_layout.addWidget(self.rx_label, 3, 0, 1, 8)
        layout.addWidget(telemetry_group)

        self.plot_tabs = QtWidgets.QTabWidget()
        self.speed_plot, self.speed_curves = self._make_plot("速度 [rps]", -MAX_SPEED_RPS, MAX_SPEED_RPS, "現在速度 M")
        command_colors = ("#64b5f6", "#ffb74d")
        self.command_curves = [
            self.speed_plot.plot(
                name=f"指令速度 M{motor}",
                pen=pg.mkPen(color, width=1, style=QtCore.Qt.PenStyle.DashLine),
                antialias=False,
            )
            for motor, color in enumerate(command_colors)
        ]
        for curve in self.command_curves:
            curve.setClipToView(True)
            curve.setSkipFiniteCheck(True)
        self.current_plot, self.current_curves = self._make_plot("電流 [A]", -5.0, 5.0, "電流 M")
        self.plot_tabs.addTab(self.speed_plot, "現在速度")
        self.plot_tabs.addTab(self.current_plot, "電流")
        layout.addWidget(self.plot_tabs, stretch=1)

        self.status_label = QtWidgets.QLabel("未接続 / 停止")
        layout.addWidget(self.status_label)
        self.resize(900, 850)

    @staticmethod
    def _make_plot(y_label: str, y_min: float, y_max: float, series_label: str) -> tuple[pg.PlotWidget, list[pg.PlotDataItem]]:
        plot = pg.PlotWidget()
        plot.setMinimumHeight(450)
        plot.setLabel("bottom", "時刻", units="s")
        plot.setLabel("left", y_label)
        plot.setXRange(-PLOT_WINDOW_S, 0.0, padding=0)
        plot.setYRange(y_min, y_max, padding=0.05)
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.addLegend()
        colors = ("#1976d2", "#ef6c00")
        curves = []
        for motor, color in enumerate(colors):
            curve = plot.plot(name=f"{series_label}{motor}", pen=pg.mkPen(color, width=1), antialias=False)
            curve.setClipToView(True)
            curve.setDownsampling(ds=1, auto=False, method="peak")
            curve.setSkipFiniteCheck(True)
            curves.append(curve)
        return plot, curves

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        ports = list(list_ports.comports())
        self.port_combo.clear()
        self.port_combo.addItems([port.device for port in ports])
        preferred = next((port.device for port in ports if port.vid == 0x0483 and port.pid == 0x5740), "")
        selected = current if current in [port.device for port in ports] else preferred
        if selected:
            self.port_combo.setCurrentText(selected)

    def _toggle_connection(self) -> None:
        if self.driver is not None:
            self._disconnect()
            return
        port = self.port_combo.currentText().strip()
        if not port:
            QtWidgets.QMessageBox.critical(self, "接続エラー", "CANアダプタのCOMポートを選択してください")
            return
        try:
            driver = OrionCanDriver(port, command_watchdog_s=0.500)
            driver.open()
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "接続エラー", str(exc))
            return
        self.driver = driver
        self.running = True
        self._clear_telemetry()
        self.targets = [slider.value() * 0.5 for slider in self.speed_sliders]
        self._send_targets()
        self.connect_button.setText("切断")
        self.status_label.setText(f"{port} / Board {self.board_spin.value()} / 接続済み・自動反映中")

    def _disconnect(self) -> None:
        driver, self.driver = self.driver, None
        self.running = False
        if driver is not None:
            try:
                driver.close()
            except Exception as exc:
                QtWidgets.QMessageBox.warning(self, "切断時エラー", str(exc))
        self.connect_button.setText("接続")
        self.status_label.setText("未接続 / 停止")

    def _slider_changed(self, motor: int, slider_value: int) -> None:
        target = slider_value * 0.5
        self.targets[motor] = target
        self.target_labels[motor].setText(f"{target:+.1f} rps")
        if self.running and self.driver is not None:
            self.driver.set_speed(self.board_spin.value(), motor, target)

    def _send_targets(self) -> None:
        if self.driver is None:
            return
        board = self.board_spin.value()
        for motor, target in enumerate(self.targets):
            self.driver.set_speed(board, motor, target)

    def _stop_motion(self) -> None:
        for slider in self.speed_sliders:
            slider.setValue(0)
        if self.driver is not None:
            try:
                self.driver.stop_all()
                self.running = True
                self.status_label.setText(f"{self.port_combo.currentText()} / Board {self.board_spin.value()} / 0 rps・自動反映中")
            except OrionCanError as exc:
                self._connection_failed(exc)

    @QtCore.Slot()
    def _service_can(self) -> None:
        if self.driver is None:
            return
        try:
            if self.running:
                self._send_targets()
                commanded_at = time.monotonic()
                for motor, target in enumerate(self.targets):
                    self.command_history[motor].append((commanded_at, target))
            for _ in range(MAX_RX_PER_TICK):
                frame = self.driver.receive(timeout=0)
                if frame is None:
                    break
                update = apply_telemetry_frame(frame, self.board_spin.value(), self.telemetry)
                if update is None:
                    continue
                self.rx_total += 1
                received_at = time.monotonic()
                if update.kind == "speed":
                    self.speed_history[update.motor].append((received_at, update.value))
                elif update.kind == "current":
                    self.current_history[update.motor].append((received_at, update.value))
        except OrionCanError as exc:
            self._connection_failed(exc)

    @QtCore.Slot()
    def _update_display(self) -> None:
        for motor, state in enumerate(self.telemetry):
            values = (
                f"{self.targets[motor]:+.1f} rps",
                "--" if state.speed_rps is None else f"{state.speed_rps:+.2f} rps",
                "--" if state.encoder_raw is None else str(state.encoder_raw),
                "--" if state.voltage_v is None else f"{state.voltage_v:.2f} V",
                "--" if state.current_a is None else f"{state.current_a:+.2f} A",
                "--" if state.motor_temp_c is None else f"{state.motor_temp_c:.1f} °C",
                "--" if state.fet_temp_c is None else f"{state.fet_temp_c:.1f} °C",
            )
            for label, value in zip(self.value_labels[motor], values):
                label.setText(value)
        self.rx_label.setText(f"復号済みCANテレメトリ: {self.rx_total:,} frame")

    @QtCore.Slot()
    def _update_plots(self) -> None:
        now = time.monotonic()
        cutoff = now - PLOT_WINDOW_S
        for history in self.speed_history + self.command_history + self.current_history:
            while history and history[0][0] < cutoff:
                history.popleft()
        self._set_curve_data(self.speed_curves, self.speed_history, now)
        self._set_curve_data(self.command_curves, self.command_history, now)
        self._set_curve_data(self.current_curves, self.current_history, now)

    @staticmethod
    def _set_curve_data(curves: list[pg.PlotDataItem], histories: list[deque[tuple[float, float]]], now: float) -> None:
        for curve, history in zip(curves, histories):
            if not history:
                curve.setData([], [])
                continue
            data = np.asarray(history, dtype=np.float64)
            curve.setData(data[:, 0] - now, data[:, 1], connect="all", skipFiniteCheck=True)

    def _clear_telemetry(self) -> None:
        self.telemetry = [MotorTelemetry(), MotorTelemetry()]
        for history in self.speed_history + self.command_history + self.current_history:
            history.clear()
        self.rx_total = 0

    def _connection_failed(self, error: Exception) -> None:
        self._disconnect()
        QtWidgets.QMessageBox.critical(self, "CAN通信エラー", str(error))

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._disconnect()
        event.accept()


def main() -> int:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    window = MotorControlWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

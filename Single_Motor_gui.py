import csv
import sys
import time
from collections import deque
from datetime import datetime

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

# Edit these 3 values for a new laptop/setup.
PORT_NAME = "/dev/ttyUSB0"
BAUDRATE = 57600
DXL_ID = 15
PROTOCOL_VERSION = 2.0

TICKS_PER_REV = 4096
STEP_REV = 0.02
MAX_REV = 10.0
MIN_REV = -10.0

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VOLTAGE = 144
ADDR_PRESENT_TEMP = 146

portHandler = None
packetHandler = None
zero = 0
goal = 0.0


def init_hardware(port_name, baudrate, protocol_version, dxl_id):
    global portHandler, packetHandler, zero, goal
    goal = 0.0

    portHandler = PortHandler(port_name)
    packetHandler = PacketHandler(protocol_version)

    if not portHandler.openPort():
        return False, f"Failed to open port: {port_name}"
    if not portHandler.setBaudRate(baudrate):
        portHandler.closePort()
        return False, f"Failed to set baudrate: {baudrate}"

    init_motor(dxl_id)
    zero = read_raw(dxl_id)
    return True, ""


def init_motor(dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.05)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, 4)
    time.sleep(0.1)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
    time.sleep(0.05)


def read_raw(dxl_id):
    val, res, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    return val if res == COMM_SUCCESS else 0


def pack_goal(rev, zero_offset):
    ticks = int(zero_offset + rev * TICKS_PER_REV) & 0xFFFFFFFF
    return ticks


def send_goal(goal_rev):
    ticks = pack_goal(goal_rev, zero)
    packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, ticks)


def read_telemetry(dxl_id, zero_offset):
    pos, r, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    vel, r2, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_VELOCITY)
    cur, r3, _ = packetHandler.read2ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_CURRENT)
    vol, r4, _ = packetHandler.read2ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_VOLTAGE)
    tmp, r5, _ = packetHandler.read1ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_TEMP)

    pos_rev = (pos - zero_offset) / TICKS_PER_REV if r == COMM_SUCCESS else 0.0
    vel_rpm = vel * 0.229 if r2 == COMM_SUCCESS else 0.0
    if vel_rpm > 2147483647 * 0.229:
        vel_rpm -= 4294967296 * 0.229
    cur_ma = ((cur - 65536) * 2.69 if cur > 32767 else cur * 2.69) if r3 == COMM_SUCCESS else 0.0
    vol_v = vol * 0.1 if r4 == COMM_SUCCESS else 0.0
    tmp_c = tmp if r5 == COMM_SUCCESS else 0

    return pos_rev, vel_rpm, cur_ma, vol_v, tmp_c


class MotorPanel(QGroupBox):
    def __init__(self, title, color):
        super().__init__(title)
        self.setStyleSheet(
            f"""
            QGroupBox {{
                color: {color};
                border: 1px solid {color};
                border-radius: 6px;
                margin-top: 10px;
                font-weight: bold;
                font-size: 13px;
            }}
            QGroupBox::title {{ subcontrol-origin: margin; left: 10px; }}
            QLabel {{ color: #c9d1d9; font-family: Consolas; font-size: 11px; }}
        """
        )

        layout = QVBoxLayout()
        grid = QGridLayout()

        def row(label, row_idx):
            lbl = QLabel(label)
            lbl.setStyleSheet("color: #8b949e;")
            val = QLabel("--")
            val.setAlignment(Qt.AlignRight)
            grid.addWidget(lbl, row_idx, 0)
            grid.addWidget(val, row_idx, 1)
            return val

        self.pos_label = row("Position (rev)", 0)
        self.deg_label = row("Angle (deg)", 1)
        self.vel_label = row("Velocity (rpm)", 2)
        self.cur_label = row("Current (mA)", 3)
        self.vol_label = row("Voltage (V)", 4)
        self.tmp_label = row("Temp (degC)", 5)
        self.goal_label = row("Goal (rev)", 6)

        layout.addLayout(grid)

        btn_layout = QHBoxLayout()
        self.btn_left = QPushButton("LEFT")
        self.btn_right = QPushButton("RIGHT")
        self.btn_zero = QPushButton("ZERO")
        for btn in [self.btn_left, self.btn_right, self.btn_zero]:
            btn.setStyleSheet(
                """
                QPushButton {
                    background: #21262d; color: #c9d1d9;
                    border: 1px solid #30363d; border-radius: 4px;
                    padding: 6px; font-family: Consolas; font-size: 11px;
                }
                QPushButton:hover { background: #30363d; }
                QPushButton:pressed { background: #161b22; }
            """
            )
        btn_layout.addWidget(self.btn_left)
        btn_layout.addWidget(self.btn_right)
        btn_layout.addWidget(self.btn_zero)
        layout.addLayout(btn_layout)

        torque_layout = QHBoxLayout()
        self.btn_ton = QPushButton("TORQUE ON")
        self.btn_toff = QPushButton("TORQUE OFF")
        self.btn_ton.setStyleSheet(
            "QPushButton { background:#238636; color:white; border:none; border-radius:4px; padding:5px; font-family:Consolas; font-size:10px; } QPushButton:hover{background:#2ea043;}"
        )
        self.btn_toff.setStyleSheet(
            "QPushButton { background:#da3633; color:white; border:none; border-radius:4px; padding:5px; font-family:Consolas; font-size:10px; } QPushButton:hover{background:#f85149;}"
        )
        torque_layout.addWidget(self.btn_ton)
        torque_layout.addWidget(self.btn_toff)
        layout.addLayout(torque_layout)

        self.setLayout(layout)

    def update_telemetry(self, pos_rev, vel_rpm, cur_ma, vol_v, tmp_c, goal_rev):
        self.pos_label.setText(f"{pos_rev:.3f}")
        self.deg_label.setText(f"{pos_rev * 360:.1f}")
        self.vel_label.setText(f"{vel_rpm:.2f}")
        self.cur_label.setText(f"{cur_ma:.1f}")
        self.vol_label.setText(f"{vol_v:.1f}")
        self.tmp_label.setText(f"{tmp_c}")
        self.goal_label.setText(f"{goal_rev:.3f}")


class GraphWindow(QWidget):
    def __init__(self, parent):
        super().__init__(parent, Qt.Window)
        self.setWindowTitle("Telemetry Graph")
        self.setMinimumSize(760, 520)

        self.field_options = {
            "Position (rev)": "position",
            "Goal Position (rev)": "goal_position",
            "Velocity (rpm)": "velocity",
            "Current (mA)": "current",
            "Voltage (V)": "voltage",
            "Temp (degC)": "temperature",
        }

        layout = QVBoxLayout()
        controls = QHBoxLayout()
        self.checkboxes = {}

        for label, key in self.field_options.items():
            cb = QCheckBox(label)
            cb.setChecked(key in ["position", "goal_position"])
            cb.stateChanged.connect(parent.update_graph)
            controls.addWidget(cb)
            self.checkboxes[key] = cb

        layout.addLayout(controls)

        self.figure = Figure(figsize=(8, 4))
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        layout.addWidget(self.canvas)

        self.setLayout(layout)

    def selected_fields(self):
        return [key for key, cb in self.checkboxes.items() if cb.isChecked()]

    def refresh_plot(self, history):
        self.ax.clear()
        x = list(range(len(history["position"])))
        selected = self.selected_fields()
        colors = {
            "position": "#58a6ff",
            "goal_position": "#ff7b72",
            "velocity": "#a5d6ff",
            "current": "#c9d1d4",
            "voltage": "#7ee787",
            "temperature": "#f4d469",
        }

        for field in selected:
            self.ax.plot(x, history[field], label=field.replace("_", " ").title(), color=colors.get(field, "#c9d1d9"))

        self.ax.set_xlabel("Samples")
        self.ax.set_ylabel("Value")
        self.ax.grid(True, linestyle=":", alpha=0.5)
        self.ax.legend(loc="upper left", fontsize="small")
        self.canvas.draw()


class MainWindow(QMainWindow):
    def __init__(self, port_name, dxl_id):
        super().__init__()
        self.setWindowTitle("Single Motor Control - IITGN")
        self.setStyleSheet("background-color: #0d1117; color: #c9d1d9;")
        self.setMinimumWidth(740)

        self.history = {
            "position": deque(maxlen=200),
            "goal_position": deque(maxlen=200),
            "velocity": deque(maxlen=200),
            "current": deque(maxlen=200),
            "voltage": deque(maxlen=200),
            "temperature": deque(maxlen=200),
        }

        central = QWidget()
        main_layout = QVBoxLayout()

        title = QLabel(f"SINGLE DYNAMIXEL CONTROL | ID {dxl_id} | {port_name}")
        title.setStyleSheet("color: #00d4ff; font-family: Consolas; font-size: 13px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)

        self.panel = MotorPanel(f"MOTOR ID {dxl_id}", "#58a6ff")
        main_layout.addWidget(self.panel)

        graph_button = QPushButton("OPEN GRAPH")
        graph_button.setStyleSheet(
            "QPushButton { background:#21262d; color:#ffd33d; border:1px solid #ffd33d; border-radius:4px; padding:10px; font-family:Consolas; font-size:11px; } QPushButton:hover{background:#30363d;}"
        )
        graph_button.clicked.connect(self.open_graph_window)
        main_layout.addWidget(graph_button)

        routine_box = QGroupBox("AUTO STEP ROUTINE")
        routine_box.setStyleSheet(
            """
            QGroupBox {
                color: #ffd33d;
                border: 1px solid #ffd33d;
                border-radius: 6px;
                margin-top: 10px;
                font-weight: bold;
                font-size: 13px;
            }
            QLabel { color: #c9d1d9; font-family: Consolas; font-size: 11px; }
        """
        )
        routine_layout = QGridLayout()

        routine_layout.addWidget(QLabel("Step size (rev):"), 0, 0)
        self.step_size_input = QDoubleSpinBox()
        self.step_size_input.setRange(0.001, 5.0)
        self.step_size_input.setSingleStep(0.01)
        self.step_size_input.setValue(STEP_REV)
        routine_layout.addWidget(self.step_size_input, 0, 1)

        routine_layout.addWidget(QLabel("Hold time (s):"), 1, 0)
        self.hold_time_input = QDoubleSpinBox()
        self.hold_time_input.setRange(0.1, 60.0)
        self.hold_time_input.setSingleStep(0.1)
        self.hold_time_input.setValue(1.0)
        routine_layout.addWidget(self.hold_time_input, 1, 1)

        btn_layout = QHBoxLayout()
        self.start_routine_btn = QPushButton("START ROUTINE")
        self.stop_routine_btn = QPushButton("STOP ROUTINE")
        self.start_routine_btn.setStyleSheet(
            "QPushButton { background:#238636; color:white; border:none; border-radius:4px; padding:8px; } QPushButton:hover{background:#2ea043;}"
        )
        self.stop_routine_btn.setStyleSheet(
            "QPushButton { background:#da3633; color:white; border:none; border-radius:4px; padding:8px; } QPushButton:hover{background:#f85149;}"
        )
        self.stop_routine_btn.setEnabled(False)
        btn_layout.addWidget(self.start_routine_btn)
        btn_layout.addWidget(self.stop_routine_btn)
        routine_layout.addLayout(btn_layout, 2, 0, 1, 2)

        self.routine_status = QLabel("Routine stopped")
        self.routine_status.setStyleSheet("color: #8b949e;")
        routine_layout.addWidget(self.routine_status, 3, 0, 1, 2)

        routine_box.setLayout(routine_layout)
        main_layout.addWidget(routine_box)

        self.status = QLabel("Running at 20Hz")
        self.status.setStyleSheet("color: #484f58; font-family: Consolas; font-size: 10px;")
        self.status.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.status)

        central.setLayout(main_layout)
        self.setCentralWidget(central)

        self.panel.btn_left.clicked.connect(lambda: self.move("LEFT"))
        self.panel.btn_right.clicked.connect(lambda: self.move("RIGHT"))
        self.panel.btn_zero.clicked.connect(self.zero)
        self.panel.btn_ton.clicked.connect(lambda: packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1))
        self.panel.btn_toff.clicked.connect(lambda: packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0))

        self.start_routine_btn.clicked.connect(self.start_routine)
        self.stop_routine_btn.clicked.connect(self.stop_routine)

        self.graph_window = None
        self.routine_active = False
        self.routine_direction = 1
        self.routine_timer = QTimer(self)
        self.routine_timer.setSingleShot(True)
        self.routine_timer.timeout.connect(self.perform_next_routine_step)
        self.csv_file = None
        self.log_writer = None

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_telemetry)
        self.timer.start(50)

    def move(self, direction):
        global goal
        step = STEP_REV if direction == "RIGHT" else -STEP_REV
        goal = max(MIN_REV, min(MAX_REV, goal + step))
        send_goal(goal)

    def zero(self):
        global zero, goal
        zero = read_raw(DXL_ID)
        goal = 0.0
        send_goal(goal)

    def open_graph_window(self):
        if self.graph_window is None:
            self.graph_window = GraphWindow(self)
        self.graph_window.show()
        self.graph_window.raise_()
        self.graph_window.activateWindow()
        self.update_graph()

    def create_log_file(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"motor_routine_{timestamp}.csv"
        self.csv_file = open(filename, "w", newline="", encoding="utf-8")
        self.log_writer = csv.writer(self.csv_file)
        self.log_writer.writerow(
            [
                "timestamp",
                "position_rev",
                "goal_rev",
                "velocity_rpm",
                "current_mA",
                "voltage_V",
                "temperature_C",
            ]
        )
        self.csv_file.flush()

    def close_log_file(self):
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
            self.log_writer = None

    def start_routine(self):
        if self.routine_active:
            return
        self.routine_active = True
        self.routine_direction = 1
        self.routine_step = self.step_size_input.value()
        self.routine_hold_time = self.hold_time_input.value()
        self.create_log_file()
        self.start_routine_btn.setEnabled(False)
        self.stop_routine_btn.setEnabled(True)
        self.routine_status.setText(
            f"Routine running: step {self.routine_step:.3f} rev, hold {self.routine_hold_time:.2f}s"
        )
        self.perform_next_routine_step()

    def stop_routine(self):
        if not self.routine_active:
            return
        self.routine_active = False
        self.routine_timer.stop()
        self.close_log_file()
        self.start_routine_btn.setEnabled(True)
        self.stop_routine_btn.setEnabled(False)
        self.routine_status.setText("Routine stopped")

    def perform_next_routine_step(self):
        if not self.routine_active:
            return
        global goal
        step = self.routine_direction * self.routine_step
        goal = max(MIN_REV, min(MAX_REV, goal + step))
        send_goal(goal)
        self.routine_direction *= -1
        self.routine_status.setText(f"Holding goal {goal:.3f} rev for {self.routine_hold_time:.2f}s")
        self.routine_timer.start(int(self.routine_hold_time * 1000))

    def update_telemetry(self):
        global goal
        pos_rev, vel_rpm, cur_ma, vol_v, tmp_c = read_telemetry(DXL_ID, zero)
        self.panel.update_telemetry(pos_rev, vel_rpm, cur_ma, vol_v, tmp_c, goal)

        self.history["position"].append(pos_rev)
        self.history["goal_position"].append(goal)
        self.history["velocity"].append(vel_rpm)
        self.history["current"].append(cur_ma)
        self.history["voltage"].append(vol_v)
        self.history["temperature"].append(tmp_c)

        if self.routine_active and self.log_writer:
            self.log_writer.writerow(
                [
                    datetime.now().isoformat(timespec="seconds"),
                    f"{pos_rev:.6f}",
                    f"{goal:.6f}",
                    f"{vel_rpm:.2f}",
                    f"{cur_ma:.2f}",
                    f"{vol_v:.2f}",
                    f"{tmp_c:.1f}",
                ]
            )
            self.csv_file.flush()

        self.status.setText(f"Position: {pos_rev:.3f} rev | Goal: {goal:.3f} rev | 20Hz")
        if self.graph_window and self.graph_window.isVisible():
            self.update_graph()

    def update_graph(self):
        if self.graph_window:
            self.graph_window.refresh_plot(self.history)

    def closeEvent(self, event):
        self.stop_routine()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    app.setFont(QFont("Consolas", 10))

    ok, err = init_hardware(PORT_NAME, BAUDRATE, PROTOCOL_VERSION, DXL_ID)
    if not ok:
        print(f"HARDWARE ERROR: {err}")
        QMessageBox.critical(None, "Hardware Error", err)
        return 1

    window = MainWindow(PORT_NAME, DXL_ID)
    window.show()

    try:
        return app.exec()
    finally:
        if packetHandler and portHandler:
            packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
            portHandler.closePort()


if __name__ == "__main__":
    sys.exit(main())

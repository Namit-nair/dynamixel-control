import sys
import time
from collections import deque
from datetime import datetime

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QKeyEvent
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
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

PORT_NAME = "/dev/ttyUSB0"
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

detected_baud = 57600
detected_id = 1


def auto_detect_motor(port_name, protocol_version):
    temp_port = PortHandler(port_name)
    temp_packet = PacketHandler(protocol_version)
    
    if not temp_port.openPort():
        return False, f"Failed to open port: {port_name}", None, None

    # Common Dynamixel baudrates
    baudrates = [57600, 1000000, 115200, 2000000, 3000000, 4000000, 9600]
    
    print(f"Scanning port {port_name} for Dynamixel motors...")
    for baud in baudrates:
        if temp_port.setBaudRate(baud):
            data_list, result = temp_packet.broadcastPing(temp_port)
            if data_list:
                found_id = list(data_list.keys())[0]
                model = data_list[found_id][0]
                print(f"Success! Found Motor ID {found_id} (Model: {model}) at {baud} bps.")
                temp_port.closePort()
                return True, "", baud, found_id
                
    temp_port.closePort()
    return False, "No Dynamixel motor detected. Check power and cables.", None, None


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
    packetHandler.write4ByteTxRx(portHandler, detected_id, ADDR_GOAL_POSITION, ticks)


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
        # Explicitly label this as Torque / Force output
        self.cur_label = row("Torque Output (mA)", 3) 
        self.vol_label = row("Voltage (V)", 4)
        self.tmp_label = row("Temp (degC)", 5)
        self.goal_label = row("Goal (rev)", 6)

        layout.addLayout(grid)

        btn_layout = QHBoxLayout()
        self.btn_left = QPushButton("LEFT (CCW)")
        self.btn_right = QPushButton("RIGHT (CW)")
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
            "Torque Output (mA)": "current",
            "Voltage (V)": "voltage",
            "Temp (degC)": "temperature",
        }

        layout = QVBoxLayout()
        controls = QHBoxLayout()
        self.checkboxes = {}

        for label, key in self.field_options.items():
            cb = QCheckBox(label)
            # Default to checking current/torque
            cb.setChecked(key in ["current"])
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
            "current": "#e34c26",  # Reddish color for current/torque
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
    def __init__(self, port_name, baudrate, dxl_id):
        super().__init__()
        self.setWindowTitle("Keyboard Motor Control - Auto Detect")
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

        title = QLabel(f"AUTO DETECT DYNAMIXEL | ID {dxl_id} | {baudrate} bps | {port_name}")
        title.setStyleSheet("color: #00d4ff; font-family: Consolas; font-size: 13px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)

        instructions = QLabel("Use <b>Left Arrow</b> for CCW and <b>Right Arrow</b> for CW movement.")
        instructions.setAlignment(Qt.AlignCenter)
        instructions.setStyleSheet("color: #8b949e; font-size: 12px; margin-bottom: 10px;")
        main_layout.addWidget(instructions)

        self.panel = MotorPanel(f"MOTOR ID {dxl_id}", "#58a6ff")
        main_layout.addWidget(self.panel)

        graph_button = QPushButton("OPEN TORQUE / FORCE GRAPH")
        graph_button.setStyleSheet(
            "QPushButton { background:#21262d; color:#ffd33d; border:1px solid #ffd33d; border-radius:4px; padding:10px; font-family:Consolas; font-size:11px; } QPushButton:hover{background:#30363d;}"
        )
        graph_button.clicked.connect(self.open_graph_window)
        main_layout.addWidget(graph_button)

        self.status = QLabel("Running at 20Hz")
        self.status.setStyleSheet("color: #484f58; font-family: Consolas; font-size: 10px;")
        self.status.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.status)

        central.setLayout(main_layout)
        self.setCentralWidget(central)
        
        # Focus policy to receive key events on the main window
        self.setFocusPolicy(Qt.StrongFocus)

        self.panel.btn_left.clicked.connect(lambda: self.move_motor("LEFT"))
        self.panel.btn_right.clicked.connect(lambda: self.move_motor("RIGHT"))
        self.panel.btn_zero.clicked.connect(self.zero)
        self.panel.btn_ton.clicked.connect(lambda: packetHandler.write1ByteTxRx(portHandler, detected_id, ADDR_TORQUE_ENABLE, 1))
        self.panel.btn_toff.clicked.connect(lambda: packetHandler.write1ByteTxRx(portHandler, detected_id, ADDR_TORQUE_ENABLE, 0))

        self.graph_window = None

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_telemetry)
        self.timer.start(50)

    def keyPressEvent(self, event: QKeyEvent):
        if event.key() == Qt.Key_Right:
            self.move_motor("RIGHT")
        elif event.key() == Qt.Key_Left:
            self.move_motor("LEFT")
        else:
            super().keyPressEvent(event)

    def move_motor(self, direction):
        global goal
        step = STEP_REV if direction == "RIGHT" else -STEP_REV
        goal = max(MIN_REV, min(MAX_REV, goal + step))
        send_goal(goal)

    def zero(self):
        global zero, goal
        zero = read_raw(detected_id)
        goal = 0.0
        send_goal(goal)

    def open_graph_window(self):
        if self.graph_window is None:
            self.graph_window = GraphWindow(self)
        self.graph_window.show()
        self.graph_window.raise_()
        self.graph_window.activateWindow()
        self.update_graph()

    def update_telemetry(self):
        global goal
        pos_rev, vel_rpm, cur_ma, vol_v, tmp_c = read_telemetry(detected_id, zero)
        self.panel.update_telemetry(pos_rev, vel_rpm, cur_ma, vol_v, tmp_c, goal)

        self.history["position"].append(pos_rev)
        self.history["goal_position"].append(goal)
        self.history["velocity"].append(vel_rpm)
        self.history["current"].append(cur_ma)
        self.history["voltage"].append(vol_v)
        self.history["temperature"].append(tmp_c)

        self.status.setText(f"Position: {pos_rev:.3f} rev | Goal: {goal:.3f} rev | 20Hz")
        if self.graph_window and self.graph_window.isVisible():
            self.update_graph()

    def update_graph(self):
        if self.graph_window:
            self.graph_window.refresh_plot(self.history)


def main():
    global detected_baud, detected_id
    
    app = QApplication(sys.argv)
    app.setFont(QFont("Consolas", 10))

    # Auto-detect ID and Baudrate
    ok, err, baud, dxl_id = auto_detect_motor(PORT_NAME, PROTOCOL_VERSION)
    
    if not ok:
        print(f"AUTO-DETECT ERROR: {err}")
        QMessageBox.critical(None, "Auto-Detect Error", err)
        return 1
        
    detected_baud = baud
    detected_id = dxl_id

    # Initialize Hardware with detected values
    ok, err = init_hardware(PORT_NAME, detected_baud, PROTOCOL_VERSION, detected_id)
    if not ok:
        print(f"HARDWARE ERROR: {err}")
        QMessageBox.critical(None, "Hardware Error", err)
        return 1

    window = MainWindow(PORT_NAME, detected_baud, detected_id)
    window.show()

    try:
        return app.exec()
    finally:
        if packetHandler and portHandler:
            packetHandler.write1ByteTxRx(portHandler, detected_id, ADDR_TORQUE_ENABLE, 0)
            portHandler.closePort()


if __name__ == "__main__":
    sys.exit(main())
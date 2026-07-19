"""Dynamixel Control Center.

A general-purpose PySide6 GUI for Protocol 2.0 (X-series) Dynamixel motors:
connect to any port/baud, scan the bus for motors, switch operating mode,
and change ID/baud persistently -- the everyday tasks Dynamixel Wizard is
normally needed for -- plus live telemetry, jog/velocity/current control,
an auto step routine, a live graph, and CSV logging.
"""

import csv
import sys
from collections import deque
from datetime import datetime

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QProgressDialog,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from dxl_manager import (
    MODE_NAME_TO_VALUE,
    OPERATING_MODES,
    STANDARD_BAUDS,
    TICKS_PER_REV,
    DynamixelBus,
)

try:
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure

    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

STEP_REV = 0.02
MAX_REV = 10.0
MIN_REV = -10.0

DARK_BG = "#0d1117"
ACCENT = "#00d4ff"
PANEL_COLORS = ["#58a6ff", "#ffd33d", "#7ee787", "#f778ba", "#ff9d5c", "#a5a5ff"]

BUTTON_QSS = """
    QPushButton {
        background: #21262d; color: #c9d1d9;
        border: 1px solid #30363d; border-radius: 4px;
        padding: 6px; font-family: Consolas; font-size: 11px;
    }
    QPushButton:hover { background: #30363d; }
    QPushButton:pressed { background: #161b22; }
    QPushButton:disabled { color: #484f58; border-color: #21262d; }
"""
INPUT_QSS = """
    QComboBox, QSpinBox, QDoubleSpinBox {
        background: #161b22; color: #c9d1d9; border: 1px solid #30363d;
        border-radius: 4px; padding: 3px; font-family: Consolas; font-size: 11px;
    }
"""


def available_ports() -> list[str]:
    if list_ports is None:
        return []
    return [p.device for p in list_ports.comports()]


class ConnectionBar(QGroupBox):
    """Port / baud selection and connect/disconnect -- no config files, no Wizard."""

    def __init__(self, bus: DynamixelBus, on_change):
        super().__init__("CONNECTION")
        self.bus = bus
        self.on_change = on_change
        self.setStyleSheet(
            f"""
            QGroupBox {{ color: {ACCENT}; border: 1px solid {ACCENT}; border-radius: 6px;
                margin-top: 10px; font-weight: bold; font-size: 12px; }}
            QGroupBox::title {{ subcontrol-origin: margin; left: 10px; }}
            QLabel {{ color: #c9d1d9; font-family: Consolas; font-size: 11px; }}
            {INPUT_QSS}
        """
        )

        layout = QHBoxLayout()

        layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(120)
        layout.addWidget(self.port_combo)

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.setStyleSheet(BUTTON_QSS)
        self.refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(self.refresh_btn)

        layout.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems([str(b) for b in STANDARD_BAUDS])
        self.baud_combo.setCurrentText("57600")
        layout.addWidget(self.baud_combo)

        self.connect_btn = QPushButton("CONNECT")
        self.connect_btn.setStyleSheet(
            "QPushButton { background:#238636; color:white; border:none; border-radius:4px; padding:6px 14px; } QPushButton:hover{background:#2ea043;}"
        )
        self.connect_btn.clicked.connect(self.toggle_connect)
        layout.addWidget(self.connect_btn)

        self.status_label = QLabel("Not connected")
        self.status_label.setStyleSheet("color: #8b949e;")
        layout.addWidget(self.status_label)
        layout.addStretch()

        self.setLayout(layout)
        self.refresh_ports()

    def refresh_ports(self):
        current = self.port_combo.currentText()
        ports = available_ports()
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)

    def toggle_connect(self):
        if self.bus.is_connected:
            self.bus.disconnect()
            self.connect_btn.setText("CONNECT")
            self.connect_btn.setStyleSheet(
                "QPushButton { background:#238636; color:white; border:none; border-radius:4px; padding:6px 14px; } QPushButton:hover{background:#2ea043;}"
            )
            self.status_label.setText("Not connected")
        else:
            port = self.port_combo.currentText()
            if not port:
                QMessageBox.warning(self, "No port", "No serial port selected. Click Refresh and plug in the U2D2.")
                return
            baud = int(self.baud_combo.currentText())
            ok, err = self.bus.connect(port, baud)
            if not ok:
                QMessageBox.critical(self, "Connection failed", err)
                return
            self.connect_btn.setText("DISCONNECT")
            self.connect_btn.setStyleSheet(
                "QPushButton { background:#da3633; color:white; border:none; border-radius:4px; padding:6px 14px; } QPushButton:hover{background:#f85149;}"
            )
            self.status_label.setText(f"Connected: {port} @ {baud}")
        self.on_change()


class ScanDialog(QDialog):
    """Broadcast-pings the bus for live motors -- replaces 'open Wizard, scan bus'."""

    def __init__(self, bus: DynamixelBus, parent=None):
        super().__init__(parent)
        self.bus = bus
        self.found = []
        self.setWindowTitle("Scan Bus")
        self.setMinimumSize(420, 320)
        self.setStyleSheet(f"background: {DARK_BG}; color: #c9d1d9;")

        layout = QVBoxLayout()
        self.info = QLabel("Scanning IDs 0-252 at the current baud rate...")
        layout.addWidget(self.info)

        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["ID", "Model Number", "Baud"])
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setStyleSheet("QTableWidget { background:#161b22; color:#c9d1d9; gridline-color:#30363d; }")
        layout.addWidget(self.table)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        layout.addWidget(btns)

        self.setLayout(layout)
        QTimer.singleShot(50, self.run_scan)

    def run_scan(self):
        progress = QProgressDialog("Scanning bus...", "Cancel", 0, 252, self)
        progress.setWindowModality(Qt.WindowModal)
        progress.setMinimumDuration(0)

        def on_progress(i, n):
            progress.setValue(i)
            QApplication.processEvents()

        self.found = self.bus.scan(on_progress=on_progress)
        progress.setValue(252)

        self.table.setRowCount(len(self.found))
        for row, info in enumerate(self.found):
            self.table.setItem(row, 0, QTableWidgetItem(str(info.dxl_id)))
            self.table.setItem(row, 1, QTableWidgetItem(str(info.model_number)))
            self.table.setItem(row, 2, QTableWidgetItem(str(info.baudrate)))
        self.info.setText(
            f"Found {len(self.found)} motor(s)." if self.found else "No motors responded at this baud rate."
        )

    def selected_ids(self) -> list[int]:
        rows = {idx.row() for idx in self.table.selectedIndexes()}
        return [self.found[r].dxl_id for r in rows]


class MotorPanel(QGroupBox):
    """One motor: live telemetry, mode-aware jog controls, and identity tools
    (change ID / change baud) that would otherwise need Dynamixel Wizard."""

    def __init__(self, bus: DynamixelBus, dxl_id: int, color: str, on_remove, on_activate):
        super().__init__(f"MOTOR ID {dxl_id}")
        self.bus = bus
        self.dxl_id = dxl_id
        self.on_remove = on_remove
        self.on_activate = on_activate
        self.zero_offset = 0
        self.goal_rev = 0.0

        self.setStyleSheet(
            f"""
            QGroupBox {{ color: {color}; border: 1px solid {color}; border-radius: 6px;
                margin-top: 10px; font-weight: bold; font-size: 13px; }}
            QGroupBox::title {{ subcontrol-origin: margin; left: 10px; }}
            QLabel {{ color: #c9d1d9; font-family: Consolas; font-size: 11px; }}
            {INPUT_QSS}
        """
        )

        outer = QVBoxLayout()

        # --- telemetry ---
        grid = QGridLayout()

        def row(label, r):
            lbl = QLabel(label)
            lbl.setStyleSheet("color: #8b949e;")
            val = QLabel("--")
            val.setAlignment(Qt.AlignRight)
            grid.addWidget(lbl, r, 0)
            grid.addWidget(val, r, 1)
            return val

        self.pos_label = row("Position (rev)", 0)
        self.deg_label = row("Angle (deg)", 1)
        self.vel_label = row("Velocity (rpm)", 2)
        self.cur_label = row("Current (mA)", 3)
        self.vol_label = row("Voltage (V)", 4)
        self.tmp_label = row("Temp (degC)", 5)
        outer.addLayout(grid)

        # --- mode + torque ---
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Mode:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(list(OPERATING_MODES.values()))
        mode_row.addWidget(self.mode_combo)
        self.apply_mode_btn = QPushButton("Apply")
        self.apply_mode_btn.setStyleSheet(BUTTON_QSS)
        self.apply_mode_btn.clicked.connect(self.apply_mode)
        mode_row.addWidget(self.apply_mode_btn)
        outer.addLayout(mode_row)

        # --- jog controls (shown/hidden per mode) ---
        self.position_controls = QWidget()
        pc = QHBoxLayout(self.position_controls)
        pc.setContentsMargins(0, 0, 0, 0)
        self.btn_left = QPushButton("LEFT")
        self.btn_right = QPushButton("RIGHT")
        self.btn_zero = QPushButton("ZERO")
        for b in (self.btn_left, self.btn_right, self.btn_zero):
            b.setStyleSheet(BUTTON_QSS)
            pc.addWidget(b)
        self.btn_left.clicked.connect(lambda: self.jog(-1))
        self.btn_right.clicked.connect(lambda: self.jog(1))
        self.btn_zero.clicked.connect(self.zero)
        outer.addWidget(self.position_controls)

        self.value_controls = QWidget()
        vc = QHBoxLayout(self.value_controls)
        vc.setContentsMargins(0, 0, 0, 0)
        vc.addWidget(QLabel("Goal:"))
        self.value_spin = QSpinBox()
        self.value_spin.setRange(-32767, 32767)
        vc.addWidget(self.value_spin)
        self.send_value_btn = QPushButton("Set")
        self.send_value_btn.setStyleSheet(BUTTON_QSS)
        self.send_value_btn.clicked.connect(self.send_value)
        vc.addWidget(self.send_value_btn)
        outer.addWidget(self.value_controls)
        self.value_controls.setVisible(False)

        torque_row = QHBoxLayout()
        self.btn_ton = QPushButton("TORQUE ON")
        self.btn_toff = QPushButton("TORQUE OFF")
        self.btn_ton.setStyleSheet(
            "QPushButton { background:#238636; color:white; border:none; border-radius:4px; padding:5px; font-family:Consolas; font-size:10px; } QPushButton:hover{background:#2ea043;}"
        )
        self.btn_toff.setStyleSheet(
            "QPushButton { background:#da3633; color:white; border:none; border-radius:4px; padding:5px; font-family:Consolas; font-size:10px; } QPushButton:hover{background:#f85149;}"
        )
        self.btn_ton.clicked.connect(lambda: self.bus.set_torque(self.dxl_id, True))
        self.btn_toff.clicked.connect(lambda: self.bus.set_torque(self.dxl_id, False))
        torque_row.addWidget(self.btn_ton)
        torque_row.addWidget(self.btn_toff)
        outer.addLayout(torque_row)

        # --- identity tools: what Dynamixel Wizard is normally opened for ---
        identity_box = QGroupBox("Identity (persistent, EEPROM)")
        identity_box.setStyleSheet("QGroupBox { color:#8b949e; border:1px solid #30363d; border-radius:4px; margin-top:8px; font-size:10px; }")
        idf = QFormLayout()

        id_row = QHBoxLayout()
        self.new_id_spin = QSpinBox()
        self.new_id_spin.setRange(0, 252)
        self.new_id_spin.setValue(dxl_id)
        id_row.addWidget(self.new_id_spin)
        self.change_id_btn = QPushButton("Change ID")
        self.change_id_btn.setStyleSheet(BUTTON_QSS)
        self.change_id_btn.clicked.connect(self.change_id)
        id_row.addWidget(self.change_id_btn)
        idf.addRow("New ID:", id_row)

        baud_row = QHBoxLayout()
        self.new_baud_combo = QComboBox()
        self.new_baud_combo.addItems([str(b) for b in STANDARD_BAUDS])
        self.new_baud_combo.setCurrentText(str(bus.baudrate or 57600))
        baud_row.addWidget(self.new_baud_combo)
        self.change_baud_btn = QPushButton("Change Baud")
        self.change_baud_btn.setStyleSheet(BUTTON_QSS)
        self.change_baud_btn.clicked.connect(self.change_baud)
        baud_row.addWidget(self.change_baud_btn)
        idf.addRow("New baud:", baud_row)

        identity_box.setLayout(idf)
        outer.addWidget(identity_box)

        bottom_row = QHBoxLayout()
        self.activate_btn = QPushButton("USE FOR ROUTINE / GRAPH")
        self.activate_btn.setStyleSheet(BUTTON_QSS)
        self.activate_btn.clicked.connect(lambda: self.on_activate(self.dxl_id))
        bottom_row.addWidget(self.activate_btn)
        self.remove_btn = QPushButton("Remove")
        self.remove_btn.setStyleSheet(BUTTON_QSS)
        self.remove_btn.clicked.connect(lambda: self.on_remove(self.dxl_id))
        bottom_row.addWidget(self.remove_btn)
        outer.addLayout(bottom_row)

        self.setLayout(outer)
        self.zero_offset = self.bus.read_position_raw(self.dxl_id)
        self.sync_mode_from_hardware()

    def sync_mode_from_hardware(self):
        mode = self.bus.get_operating_mode(self.dxl_id)
        if mode in OPERATING_MODES:
            self.mode_combo.setCurrentText(OPERATING_MODES[mode])
        self.update_controls_for_mode()

    def update_controls_for_mode(self):
        mode_name = self.mode_combo.currentText()
        is_position = mode_name in ("Position Control", "Extended Position Control", "Current-based Position Control")
        self.position_controls.setVisible(is_position)
        self.value_controls.setVisible(not is_position)
        if mode_name == "Velocity Control":
            self.value_spin.setRange(-1023, 1023)
        elif mode_name == "Current Control":
            self.value_spin.setRange(-1000, 1000)
        elif mode_name == "PWM Control":
            self.value_spin.setRange(-885, 885)

    def apply_mode(self):
        mode_name = self.mode_combo.currentText()
        mode_val = MODE_NAME_TO_VALUE[mode_name]
        reply = QMessageBox.question(
            self,
            "Change operating mode",
            f"Set ID {self.dxl_id} to '{mode_name}'?\nTorque will be turned off to write this.",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        ok, err = self.bus.set_operating_mode(self.dxl_id, mode_val)
        if not ok:
            QMessageBox.critical(self, "Failed", err)
            return
        self.update_controls_for_mode()

    def jog(self, direction):
        step = STEP_REV * direction
        self.goal_rev = max(MIN_REV, min(MAX_REV, self.goal_rev + step))
        ticks = int(self.zero_offset + self.goal_rev * TICKS_PER_REV) & 0xFFFFFFFF
        self.bus.send_goal_position(self.dxl_id, ticks)

    def zero(self):
        self.zero_offset = self.bus.read_position_raw(self.dxl_id)
        self.goal_rev = 0.0
        self.bus.send_goal_position(self.dxl_id, self.zero_offset)

    def send_value(self):
        mode_name = self.mode_combo.currentText()
        value = self.value_spin.value()
        if mode_name == "Velocity Control":
            self.bus.send_goal_velocity(self.dxl_id, value)
        elif mode_name == "Current Control":
            self.bus.send_goal_current(self.dxl_id, value)
        elif mode_name == "PWM Control":
            self.bus.send_goal_pwm(self.dxl_id, value)

    def change_id(self):
        new_id = self.new_id_spin.value()
        reply = QMessageBox.question(
            self,
            "Change ID",
            f"Change motor ID {self.dxl_id} -> {new_id}?\nTorque will be turned off. "
            "You'll need to remove and re-add this panel under the new ID afterward.",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        ok, err = self.bus.change_id(self.dxl_id, new_id)
        if ok:
            QMessageBox.information(self, "Done", f"ID changed to {new_id}. Rescan the bus to find it.")
        else:
            QMessageBox.critical(self, "Failed", err)

    def change_baud(self):
        new_baud = int(self.new_baud_combo.currentText())
        reply = QMessageBox.question(
            self,
            "Change baud rate",
            f"Change motor ID {self.dxl_id}'s baud rate to {new_baud}?\n"
            "This motor will stop responding at the bus's current baud rate until "
            "you reconnect at the new one.",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        ok, err = self.bus.change_baudrate(self.dxl_id, new_baud)
        if ok:
            QMessageBox.information(self, "Done", f"Baud changed to {new_baud}. Reconnect at that baud to continue.")
        else:
            QMessageBox.critical(self, "Failed", err)

    def poll(self):
        pos_rev, vel_rpm, cur_ma, vol_v, tmp_c = self.bus.read_telemetry(self.dxl_id, self.zero_offset)
        self.pos_label.setText(f"{pos_rev:.3f}")
        self.deg_label.setText(f"{pos_rev * 360:.1f}")
        self.vel_label.setText(f"{vel_rpm:.2f}")
        self.cur_label.setText(f"{cur_ma:.1f}")
        self.vol_label.setText(f"{vol_v:.1f}")
        self.tmp_label.setText(f"{tmp_c}")
        return pos_rev, vel_rpm, cur_ma, vol_v, tmp_c


class GraphWindow(QWidget):
    def __init__(self, parent):
        super().__init__(parent, Qt.Window)
        self.setWindowTitle("Telemetry Graph")
        self.setMinimumSize(760, 520)
        self.setStyleSheet(f"background:{DARK_BG};")

        self.field_options = {
            "Position (rev)": "position",
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
            cb.setStyleSheet("color:#c9d1d9;")
            cb.setChecked(key == "position")
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
        colors = {"position": "#58a6ff", "velocity": "#a5d6ff", "current": "#c9d1d4", "voltage": "#7ee787", "temperature": "#f4d469"}
        for field in self.selected_fields():
            self.ax.plot(x, history[field], label=field, color=colors.get(field, "#c9d1d9"))
        self.ax.set_xlabel("Samples")
        self.ax.set_ylabel("Value")
        self.ax.grid(True, linestyle=":", alpha=0.5)
        self.ax.legend(loc="upper left", fontsize="small")
        self.canvas.draw()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.bus = DynamixelBus()
        self.panels: dict[int, MotorPanel] = {}
        self.active_id: int | None = None
        self.history = {k: deque(maxlen=200) for k in ("position", "velocity", "current", "voltage", "temperature")}
        self.csv_file = None
        self.log_writer = None
        self.routine_active = False
        self.routine_direction = 1

        self.setWindowTitle("Dynamixel Control Center")
        self.setStyleSheet(f"background-color: {DARK_BG}; color: #c9d1d9;")
        self.setMinimumSize(900, 700)

        central = QWidget()
        root = QVBoxLayout()

        title = QLabel("DYNAMIXEL CONTROL CENTER")
        title.setStyleSheet(f"color: {ACCENT}; font-family: Consolas; font-size: 15px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        self.conn_bar = ConnectionBar(self.bus, self.on_connection_changed)
        root.addWidget(self.conn_bar)

        add_row = QHBoxLayout()
        self.scan_btn = QPushButton("SCAN BUS")
        self.scan_btn.setStyleSheet(
            "QPushButton { background:#21262d; color:#ffd33d; border:1px solid #ffd33d; border-radius:4px; padding:8px; } QPushButton:hover{background:#30363d;}"
        )
        self.scan_btn.clicked.connect(self.scan_bus)
        add_row.addWidget(self.scan_btn)

        add_row.addWidget(QLabel("Add ID directly:"))
        self.manual_id_spin = QSpinBox()
        self.manual_id_spin.setRange(0, 252)
        self.manual_id_spin.setStyleSheet(INPUT_QSS)
        add_row.addWidget(self.manual_id_spin)
        self.add_id_btn = QPushButton("Add")
        self.add_id_btn.setStyleSheet(BUTTON_QSS)
        self.add_id_btn.clicked.connect(lambda: self.add_panel(self.manual_id_spin.value()))
        add_row.addWidget(self.add_id_btn)
        add_row.addStretch()

        self.graph_btn = QPushButton("OPEN GRAPH")
        self.graph_btn.setStyleSheet(BUTTON_QSS)
        self.graph_btn.clicked.connect(self.open_graph_window)
        add_row.addWidget(self.graph_btn)
        root.addLayout(add_row)

        self.active_label = QLabel("Active motor (drives routine/graph): none")
        self.active_label.setStyleSheet("color:#8b949e;")
        root.addWidget(self.active_label)

        self.panel_area = QScrollArea()
        self.panel_area.setWidgetResizable(True)
        self.panel_container = QWidget()
        self.panel_layout = QGridLayout()
        self.panel_container.setLayout(self.panel_layout)
        self.panel_area.setWidget(self.panel_container)
        root.addWidget(self.panel_area, stretch=1)

        routine_box = self._build_routine_box()
        root.addWidget(routine_box)

        self.status = QLabel("Not connected -- pick a port and hit CONNECT")
        self.status.setStyleSheet("color: #484f58; font-family: Consolas; font-size: 10px;")
        self.status.setAlignment(Qt.AlignCenter)
        root.addWidget(self.status)

        central.setLayout(root)
        self.setCentralWidget(central)

        self.graph_window = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.poll_all)
        self.timer.start(50)

    def _build_routine_box(self):
        box = QGroupBox("AUTO STEP ROUTINE (active motor)")
        box.setStyleSheet(
            f"""QGroupBox {{ color:#ffd33d; border:1px solid #ffd33d; border-radius:6px; margin-top:10px;
                font-weight:bold; font-size:13px; }}
            QLabel {{ color:#c9d1d9; font-family:Consolas; font-size:11px; }}
            {INPUT_QSS}"""
        )
        layout = QGridLayout()
        layout.addWidget(QLabel("Step size (rev):"), 0, 0)
        self.step_size_input = QDoubleSpinBox()
        self.step_size_input.setRange(0.001, 5.0)
        self.step_size_input.setSingleStep(0.01)
        self.step_size_input.setValue(STEP_REV)
        layout.addWidget(self.step_size_input, 0, 1)

        layout.addWidget(QLabel("Hold time (s):"), 1, 0)
        self.hold_time_input = QDoubleSpinBox()
        self.hold_time_input.setRange(0.1, 60.0)
        self.hold_time_input.setSingleStep(0.1)
        self.hold_time_input.setValue(1.0)
        layout.addWidget(self.hold_time_input, 1, 1)

        btn_row = QHBoxLayout()
        self.start_routine_btn = QPushButton("START ROUTINE")
        self.stop_routine_btn = QPushButton("STOP ROUTINE")
        self.start_routine_btn.setStyleSheet(
            "QPushButton { background:#238636; color:white; border:none; border-radius:4px; padding:8px; } QPushButton:hover{background:#2ea043;}"
        )
        self.stop_routine_btn.setStyleSheet(
            "QPushButton { background:#da3633; color:white; border:none; border-radius:4px; padding:8px; } QPushButton:hover{background:#f85149;}"
        )
        self.stop_routine_btn.setEnabled(False)
        self.start_routine_btn.clicked.connect(self.start_routine)
        self.stop_routine_btn.clicked.connect(self.stop_routine)
        btn_row.addWidget(self.start_routine_btn)
        btn_row.addWidget(self.stop_routine_btn)
        layout.addLayout(btn_row, 2, 0, 1, 2)

        self.routine_status = QLabel("Routine stopped")
        self.routine_status.setStyleSheet("color:#8b949e;")
        layout.addWidget(self.routine_status, 3, 0, 1, 2)

        box.setLayout(layout)
        return box

    def on_connection_changed(self):
        connected = self.bus.is_connected
        self.scan_btn.setEnabled(connected)
        self.add_id_btn.setEnabled(connected)
        if not connected:
            for dxl_id in list(self.panels):
                self.remove_panel(dxl_id)

    def scan_bus(self):
        if not self.bus.is_connected:
            return
        dlg = ScanDialog(self.bus, self)
        if dlg.exec() == QDialog.Accepted:
            for dxl_id in dlg.selected_ids():
                self.add_panel(dxl_id)

    def add_panel(self, dxl_id: int):
        if not self.bus.is_connected:
            QMessageBox.warning(self, "Not connected", "Connect to a port first.")
            return
        if dxl_id in self.panels:
            return
        info = self.bus.ping(dxl_id)
        if info is None:
            QMessageBox.warning(self, "No response", f"ID {dxl_id} did not respond at the current baud rate.")
            return
        color = PANEL_COLORS[len(self.panels) % len(PANEL_COLORS)]
        panel = MotorPanel(self.bus, dxl_id, color, self.remove_panel, self.set_active)
        self.panels[dxl_id] = panel
        n = len(self.panels) - 1
        self.panel_layout.addWidget(panel, n // 2, n % 2)
        if self.active_id is None:
            self.set_active(dxl_id)

    def remove_panel(self, dxl_id: int):
        panel = self.panels.pop(dxl_id, None)
        if panel:
            self.panel_layout.removeWidget(panel)
            panel.deleteLater()
        if self.active_id == dxl_id:
            self.active_id = next(iter(self.panels), None)
            self.active_label.setText(
                f"Active motor (drives routine/graph): {self.active_id if self.active_id is not None else 'none'}"
            )

    def set_active(self, dxl_id: int):
        self.active_id = dxl_id
        self.active_label.setText(f"Active motor (drives routine/graph): {dxl_id}")
        for k in self.history:
            self.history[k].clear()

    def open_graph_window(self):
        if not HAS_MATPLOTLIB:
            QMessageBox.warning(self, "matplotlib missing", "Install matplotlib to use the graph window.")
            return
        if self.graph_window is None:
            self.graph_window = GraphWindow(self)
        self.graph_window.show()
        self.graph_window.raise_()
        self.graph_window.activateWindow()
        self.update_graph()

    def update_graph(self):
        if self.graph_window:
            self.graph_window.refresh_plot(self.history)

    def create_log_file(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = open(f"motor_{self.active_id}_routine_{timestamp}.csv", "w", newline="", encoding="utf-8")
        self.log_writer = csv.writer(self.csv_file)
        self.log_writer.writerow(["timestamp", "position_rev", "velocity_rpm", "current_mA", "voltage_V", "temperature_C"])
        self.csv_file.flush()

    def close_log_file(self):
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
            self.log_writer = None

    def start_routine(self):
        if self.routine_active or self.active_id is None or self.active_id not in self.panels:
            QMessageBox.warning(self, "No active motor", "Add a motor panel and click 'USE FOR ROUTINE / GRAPH' first.")
            return
        self.routine_active = True
        self.routine_direction = 1
        self.create_log_file()
        self.start_routine_btn.setEnabled(False)
        self.stop_routine_btn.setEnabled(True)
        self.routine_status.setText(f"Routine running on ID {self.active_id}")
        self.perform_next_routine_step()

    def stop_routine(self):
        if not self.routine_active:
            return
        self.routine_active = False
        self.close_log_file()
        self.start_routine_btn.setEnabled(True)
        self.stop_routine_btn.setEnabled(False)
        self.routine_status.setText("Routine stopped")

    def perform_next_routine_step(self):
        if not self.routine_active:
            return
        panel = self.panels.get(self.active_id)
        if not panel:
            self.stop_routine()
            return
        panel.jog(self.routine_direction)
        self.routine_direction *= -1
        hold_ms = int(self.hold_time_input.value() * 1000)
        QTimer.singleShot(hold_ms, self.perform_next_routine_step)

    def poll_all(self):
        for dxl_id, panel in self.panels.items():
            pos_rev, vel_rpm, cur_ma, vol_v, tmp_c = panel.poll()
            if dxl_id == self.active_id:
                self.history["position"].append(pos_rev)
                self.history["velocity"].append(vel_rpm)
                self.history["current"].append(cur_ma)
                self.history["voltage"].append(vol_v)
                self.history["temperature"].append(tmp_c)
                if self.routine_active and self.log_writer:
                    self.log_writer.writerow(
                        [datetime.now().isoformat(timespec="seconds"), f"{pos_rev:.6f}", f"{vel_rpm:.2f}", f"{cur_ma:.2f}", f"{vol_v:.2f}", f"{tmp_c}"]
                    )
                    self.csv_file.flush()

        if self.bus.is_connected:
            self.status.setText(f"Connected: {self.bus.port_name} @ {self.bus.baudrate} | {len(self.panels)} motor(s) | 20Hz")
        else:
            self.status.setText("Not connected -- pick a port and hit CONNECT")

        if self.graph_window and self.graph_window.isVisible():
            self.update_graph()

    def closeEvent(self, event):
        self.stop_routine()
        for dxl_id in list(self.panels):
            self.bus.set_torque(dxl_id, False)
        self.bus.disconnect()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    app.setFont(QFont("Consolas", 10))
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())

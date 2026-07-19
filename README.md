# Dynamixel Control Center

A general-purpose PySide6 GUI for Protocol 2.0 (X-series) Dynamixel motors: connect to any port/baud, **scan the bus for connected motors, switch operating mode, and change ID or baud rate persistently** -- the everyday tasks Dynamixel Wizard is normally opened for -- plus live telemetry, jog/velocity/current control, an auto step routine, a live graph, and CSV logging.

Built for joint stiffness experimentation and actuator characterization, but not hardcoded to one motor, port, or mode.

---

## Features

- **No Dynamixel Wizard needed for basic setup:**
  - Port picker (auto-detected, refreshable) + baud rate selector, connect/disconnect on demand
  - **Scan Bus** -- broadcast-pings IDs 0-252 and lists every motor that responds
  - Per-motor **operating mode switch** (Current, Velocity, Position, Extended Position, Current-based Position, PWM) applied live
  - Per-motor **ID change** and **baud rate change**, written directly to EEPROM
- Multiple motors at once -- add as many panels as you have motors on the bus
- Mode-aware controls: jog buttons in position modes, a goal spinbox in velocity/current/PWM modes
- Torque enable / disable per motor
- Live telemetry: position, angle, velocity, current, voltage, temperature (~20 Hz)
- Real-time graph window, toggle any combination of telemetry streams
- Automated bidirectional step routine (stiffness/hysteresis characterization) with CSV logging

---

## Hardware

- Dynamixel Servo Motor(s), Protocol 2.0 / X-series control table
- USB2Dynamixel / U2D2 Interface
- Windows PC
- Python 3.11+

## Software Stack

Python, PySide6, Dynamixel SDK, pyserial, Matplotlib.

---

## Installation

```bash
git clone https://github.com/Namit-nair/dynamixel-control.git
cd dynamixel-control
python -m venv .venv
.venv\Scripts\activate        # Windows
pip install -r requirements.txt
```

## Running

```bash
python dynamixel_control_center.py
```

There is nothing to edit before running -- port, baud, ID, and operating mode are all chosen from the GUI.

---

## Typical workflow

1. Plug in the U2D2, hit **Refresh**, pick the port, pick a baud rate, **CONNECT**.
2. **SCAN BUS** -- select the motor(s) you want from the results, or type an ID directly if you already know it.
3. Each motor gets its own panel: check telemetry, pick an **operating mode** and hit Apply, jog it or set a goal value, toggle torque.
4. New motor out of the box, or need a different ID to avoid a bus conflict? Use the **Identity** section on its panel -- Change ID / Change Baud -- no separate tool required.
5. For characterization runs: set step size + hold time in **Auto Step Routine**, mark a panel active ("USE FOR ROUTINE / GRAPH"), hit Start. Telemetry streams to a timestamped CSV and the graph window.

---

## Module layout

- `dxl_manager.py` -- hardware-facing `DynamixelBus` class (connect, scan, mode/ID/baud writes, telemetry reads). No Qt dependency; reusable from a script or a different UI.
- `dynamixel_control_center.py` -- the PySide6 GUI described above. Run this.
- `Keyboard_Motor_gui.py` -- an older, simpler keyboard-teleop variant with the port/baud/ID still hardcoded at the top of the file. Kept for reference; not part of the dynamic workflow above.

---

## CSV Logging

Auto Step Routine runs log to `motor_<id>_routine_<timestamp>.csv`:

```text
timestamp, position_rev, velocity_rpm, current_mA, voltage_V, temperature_C
```

---

## Safety notes

- Changing **operating mode**, **ID**, or **baud rate** writes to EEPROM and requires torque to be off; the app disables torque automatically before writing.
- Changing a motor's **baud rate** takes effect on the motor immediately -- it will stop responding at the bus's current baud until you reconnect at the new one.
- Changing a motor's **ID** means the old ID stops responding; rescan the bus to find it under the new one.

# Dynamixel Servo Control


Single Dynamixel motor control and telemetry interface built using PySide6 and Dynamixel SDK for joint stiffness experimentation and actuator characterization.

---

## Features

- Real-time Dynamixel position control
- GUI-based motor interface
- Torque ON / OFF controls
- Incremental left-right stepping
- Zero position reset
- Live telemetry monitoring
- Real-time graph visualization
- Automated oscillatory step routine
- CSV telemetry logging
- 20 Hz telemetry update loop

---

## Hardware

- Dynamixel Servo Motor
- USB2Dynamixel / U2D2 Interface
- Windows PC
- Python 3.11+

---

## Software Stack

- Python
- PySide6
- Dynamixel SDK
- Matplotlib

---

## Telemetry Parameters

The software continuously reads:

- Position (revolutions)
- Angular position (degrees)
- Velocity (RPM)
- Current (mA)
- Voltage (V)
- Temperature (°C)

Telemetry is updated at approximately 20 Hz.

---

## GUI Overview

### Main Motor Panel

Provides:

- LEFT / RIGHT incremental motion
- Zero calibration
- Torque enable / disable
- Live telemetry display
- Goal position display

---

### Graph Window

Real-time plotting interface for:

- Position
- Goal Position
- Velocity
- Current
- Voltage
- Temperature

Multiple telemetry streams can be toggled on/off dynamically.

---

### Auto Step Routine

Automated bidirectional stepping routine used for:

- stiffness characterization
- hysteresis observation
- actuator response analysis
- repeated loading experiments

User configurable:

- Step size (rev)
- Hold duration (seconds)

The routine alternates direction automatically.

---

## CSV Logging

During automated routines, telemetry data is logged into timestamped CSV files.

Logged parameters:

```text
timestamp
position_rev
goal_rev
velocity_rpm
current_mA
voltage_V
temperature_C
```

Example filename:

```text
motor_routine_20260522_143015.csv
```

---

## Installation

Clone the repository:

```bash
git clone https://github.com/Namit-nair/Joint-Stiffness.git
cd Joint-Stiffness
```

Create virtual environment:

```bash
python -m venv .venv
```

Activate environment:

### Windows

```bash
.venv\Scripts\activate
```

Install dependencies:

```bash
pip install -r requirements.txt
```

---

## Running the Application

Run the GUI:

```bash
python Single_Motor_gui.py
```

---

## Configuration

The following parameters can be modified directly in the script:

```python
PORT_NAME = "COM4"
BAUDRATE = 57600
DXL_ID = 13
```

Motor limits:

```python
MAX_REV = 10.0
MIN_REV = -10.0
STEP_REV = 0.02
```

---

## Dynamixel Operating Mode

The motor is configured in:

```text
Extended Position Control Mode
```

Using:

```python
ADDR_OPERATING_MODE = 11
```

Mode value:

```python
4
```

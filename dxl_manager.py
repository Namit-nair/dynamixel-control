"""Reusable Dynamixel Protocol 2.0 (X-series control table) bus manager.

Wraps PortHandler/PacketHandler with the operations Dynamixel Wizard normally
handles by hand: port/baud connection, bus scanning, operating-mode switching,
and persistent ID/baud changes -- so a GUI can drive them directly.
"""

from __future__ import annotations

from dataclasses import dataclass

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

# --- X-series (Protocol 2.0) control table addresses ---
ADDR_MODEL_NUMBER = 0
ADDR_ID = 7
ADDR_BAUD_RATE = 8
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_PWM = 100
ADDR_GOAL_CURRENT = 102
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VOLTAGE = 144
ADDR_PRESENT_TEMP = 146

PROTOCOL_VERSION = 2.0
TICKS_PER_REV = 4096

# index written to ADDR_BAUD_RATE -> actual bps
BAUD_TABLE = {
    0: 9600,
    1: 57600,
    2: 115200,
    3: 1000000,
    4: 2000000,
    5: 3000000,
    6: 4000000,
    7: 4500000,
}
BAUD_TABLE_INV = {v: k for k, v in BAUD_TABLE.items()}
STANDARD_BAUDS = list(BAUD_TABLE.values())

OPERATING_MODES = {
    0: "Current Control",
    1: "Velocity Control",
    3: "Position Control",
    4: "Extended Position Control",
    5: "Current-based Position Control",
    16: "PWM Control",
}
MODE_NAME_TO_VALUE = {v: k for k, v in OPERATING_MODES.items()}


@dataclass
class MotorInfo:
    dxl_id: int
    model_number: int
    baudrate: int


class DynamixelBus:
    """One open serial connection to a Dynamixel chain at a fixed baud rate."""

    def __init__(self) -> None:
        self.port_handler: PortHandler | None = None
        self.packet_handler: PacketHandler | None = None
        self.port_name: str | None = None
        self.baudrate: int | None = None

    @property
    def is_connected(self) -> bool:
        return self.port_handler is not None

    def connect(self, port_name: str, baudrate: int) -> tuple[bool, str]:
        self.disconnect()
        ph = PortHandler(port_name)
        pkt = PacketHandler(PROTOCOL_VERSION)

        if not ph.openPort():
            return False, f"Failed to open port: {port_name}"
        if not ph.setBaudRate(baudrate):
            ph.closePort()
            return False, f"Failed to set baudrate: {baudrate}"

        self.port_handler = ph
        self.packet_handler = pkt
        self.port_name = port_name
        self.baudrate = baudrate
        return True, ""

    def disconnect(self) -> None:
        if self.port_handler is not None:
            self.port_handler.closePort()
        self.port_handler = None
        self.packet_handler = None
        self.port_name = None
        self.baudrate = None

    def set_baudrate(self, baudrate: int) -> bool:
        if self.port_handler is None:
            return False
        ok = self.port_handler.setBaudRate(baudrate)
        if ok:
            self.baudrate = baudrate
        return ok

    def ping(self, dxl_id: int) -> MotorInfo | None:
        if not self.packet_handler:
            return None
        model, result, _ = self.packet_handler.ping(self.port_handler, dxl_id)
        if result != COMM_SUCCESS:
            return None
        return MotorInfo(dxl_id=dxl_id, model_number=model, baudrate=self.baudrate or 0)

    def scan(self, id_range: range = range(0, 253), on_progress=None) -> list[MotorInfo]:
        found: list[MotorInfo] = []
        for dxl_id in id_range:
            info = self.ping(dxl_id)
            if info is not None:
                found.append(info)
            if on_progress:
                on_progress(dxl_id, id_range.stop - 1)
        return found

    def scan_all_bauds(self, id_range: range = range(0, 253), bauds=STANDARD_BAUDS, on_progress=None):
        """Scan every standard baud rate on the currently-open port. Leaves the
        bus connected at whichever baud last matched (or the original if none did)."""
        original_baud = self.baudrate
        results: dict[int, list[MotorInfo]] = {}
        for baud in bauds:
            if not self.set_baudrate(baud):
                continue
            found = self.scan(id_range, on_progress=lambda i, n, b=baud: on_progress(b, i, n) if on_progress else None)
            if found:
                results[baud] = found
        if original_baud:
            self.set_baudrate(original_baud)
        return results

    # --- torque / mode ---

    def set_torque(self, dxl_id: int, enable: bool) -> bool:
        if not self.packet_handler:
            return False
        result, _ = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, 1 if enable else 0
        )
        return result == COMM_SUCCESS

    def get_operating_mode(self, dxl_id: int) -> int | None:
        if not self.packet_handler:
            return None
        val, result, _ = self.packet_handler.read1ByteTxRx(self.port_handler, dxl_id, ADDR_OPERATING_MODE)
        return val if result == COMM_SUCCESS else None

    def set_operating_mode(self, dxl_id: int, mode: int) -> tuple[bool, str]:
        """Operating mode lives in EEPROM: torque must be off to write it."""
        if not self.packet_handler:
            return False, "Not connected"
        self.set_torque(dxl_id, False)
        result, _ = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_OPERATING_MODE, mode)
        if result != COMM_SUCCESS:
            return False, "Write failed (check ID / wiring)"
        return True, ""

    # --- persistent identity changes (what Dynamixel Wizard is normally for) ---

    def change_id(self, dxl_id: int, new_id: int) -> tuple[bool, str]:
        if not self.packet_handler:
            return False, "Not connected"
        if not (0 <= new_id <= 252):
            return False, "ID must be 0-252"
        self.set_torque(dxl_id, False)
        result, _ = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_ID, new_id)
        if result != COMM_SUCCESS:
            return False, "Write failed (check ID / wiring)"
        return True, ""

    def change_baudrate(self, dxl_id: int, new_baud: int) -> tuple[bool, str]:
        """Writes the motor's own EEPROM baud rate. The bus connection itself
        must be switched to new_baud afterward to keep talking to this motor."""
        if not self.packet_handler:
            return False, "Not connected"
        if new_baud not in BAUD_TABLE_INV:
            return False, f"Unsupported baud: {new_baud}"
        self.set_torque(dxl_id, False)
        result, _ = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_BAUD_RATE, BAUD_TABLE_INV[new_baud]
        )
        if result != COMM_SUCCESS:
            return False, "Write failed (check ID / wiring)"
        return True, ""

    # --- motion ---

    def read_position_raw(self, dxl_id: int) -> int:
        if not self.packet_handler:
            return 0
        val, result, _ = self.packet_handler.read4ByteTxRx(self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
        return val if result == COMM_SUCCESS else 0

    def send_goal_position(self, dxl_id: int, ticks: int) -> bool:
        if not self.packet_handler:
            return False
        result, _ = self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, ADDR_GOAL_POSITION, ticks & 0xFFFFFFFF
        )
        return result == COMM_SUCCESS

    def send_goal_velocity(self, dxl_id: int, value: int) -> bool:
        if not self.packet_handler:
            return False
        result, _ = self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, ADDR_GOAL_VELOCITY, value & 0xFFFFFFFF
        )
        return result == COMM_SUCCESS

    def send_goal_current(self, dxl_id: int, value: int) -> bool:
        if not self.packet_handler:
            return False
        result, _ = self.packet_handler.write2ByteTxRx(
            self.port_handler, dxl_id, ADDR_GOAL_CURRENT, value & 0xFFFF
        )
        return result == COMM_SUCCESS

    def send_goal_pwm(self, dxl_id: int, value: int) -> bool:
        if not self.packet_handler:
            return False
        result, _ = self.packet_handler.write2ByteTxRx(
            self.port_handler, dxl_id, ADDR_GOAL_PWM, value & 0xFFFF
        )
        return result == COMM_SUCCESS

    def read_telemetry(self, dxl_id: int, zero_offset: int = 0):
        pkt, ph = self.packet_handler, self.port_handler
        if not pkt:
            return 0.0, 0.0, 0.0, 0.0, 0

        pos, r, _ = pkt.read4ByteTxRx(ph, dxl_id, ADDR_PRESENT_POSITION)
        vel, r2, _ = pkt.read4ByteTxRx(ph, dxl_id, ADDR_PRESENT_VELOCITY)
        cur, r3, _ = pkt.read2ByteTxRx(ph, dxl_id, ADDR_PRESENT_CURRENT)
        vol, r4, _ = pkt.read2ByteTxRx(ph, dxl_id, ADDR_PRESENT_VOLTAGE)
        tmp, r5, _ = pkt.read1ByteTxRx(ph, dxl_id, ADDR_PRESENT_TEMP)

        pos_rev = (pos - zero_offset) / TICKS_PER_REV if r == COMM_SUCCESS else 0.0
        vel_rpm = vel * 0.229 if r2 == COMM_SUCCESS else 0.0
        if vel_rpm > 2147483647 * 0.229:
            vel_rpm -= 4294967296 * 0.229
        cur_ma = ((cur - 65536) * 2.69 if cur > 32767 else cur * 2.69) if r3 == COMM_SUCCESS else 0.0
        vol_v = vol * 0.1 if r4 == COMM_SUCCESS else 0.0
        tmp_c = tmp if r5 == COMM_SUCCESS else 0

        return pos_rev, vel_rpm, cur_ma, vol_v, tmp_c

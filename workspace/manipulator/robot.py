"""Robot I/O layer for the Automated Grocery Bagger.

The dev app should send absolute motor STEP targets to the Teensy firmware:
    Q <j1_steps> <j2_steps> <j3_steps> <j4_steps>

The kinematics/transmission class owns the conversion from physical joints to steps.
This file only owns serial connection, formatting, TX/RX, and dry-run behavior.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import degrees
from queue import Queue
from threading import Event, Thread
from time import sleep, time
from typing import Iterable, Optional
import json
import sys
from pathlib import Path

SERIAL_IMPORT_ERROR: str | None = None


def _import_pyserial():  # noqa: ANN202 - imported module objects are intentionally dynamic
    """Import the external pyserial package even when this folder has serial.py."""
    local_dir = Path(__file__).resolve().parent
    original_path = list(sys.path)
    old_serial = sys.modules.get("serial")
    old_serial_file = Path(getattr(old_serial, "__file__", "") or "").resolve() if old_serial else None

    if old_serial_file and old_serial_file == local_dir / "serial.py":
        sys.modules.pop("serial", None)

    try:
        sys.path = [p for p in sys.path if Path(p or ".").resolve() != local_dir]
        import serial as pyserial  # type: ignore
        from serial.tools import list_ports as pyserial_list_ports  # type: ignore

        if not hasattr(pyserial, "Serial"):
            raise ImportError(f"Imported serial module is not pyserial: {getattr(pyserial, '__file__', 'unknown')}")
        return pyserial, pyserial_list_ports, None
    except ImportError as exc:  # pyserial is optional until hardware testing
        return None, None, str(exc)
    finally:
        sys.path = original_path


serial, list_ports, SERIAL_IMPORT_ERROR = _import_pyserial()

JointVector = tuple[float, float, float, float]
MotorSteps = tuple[int, int, int, int]


class SerialProtocol(str, Enum):
    """Supported outgoing serial message formats."""

    TEENSY_Q_STEPS = "teensy_q_steps"      # Q s1 s2 s3 s4
    TEENSY_MOVE_STEPS = "teensy_move_steps"  # MOVE s1 s2 s3 s4
    CSV_RADIANS = "csv_radians"            # q1,q2,q3,q4
    CSV_DEGREES = "csv_degrees"            # q1_deg,q2_deg,q3_m,q4_deg
    JSON = "json"                          # {"q":[...], "timestamp":...}
    M_COMMANDS = "m_commands"              # M1 <val>\nM2 <val>... old Arduino style
    J_COMMANDS = "j_commands"              # J1 <val>\nJ2 <val>... current Teensy legacy style
    ONE_PER_LINE = "one_per_line"          # J1:<val> etc.
    TEMPLATE = "template"                  # arbitrary Python format template


@dataclass
class JointOutputConfig:
    """Optional scale/offset for legacy physical-joint protocols.

    The preferred Teensy path is motor steps from kinematics.py, so this is mostly
    kept for debugging and old protocol compatibility.
    """

    scale: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    offset: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    round_digits: Optional[int] = 5

    def apply(self, q: Iterable[float]) -> list[float]:
        vals = []
        for value, scale, offset in zip(tuple(float(v) for v in q), self.scale, self.offset):
            out = value * scale + offset
            if self.round_digits is not None:
                out = round(out, self.round_digits)
            vals.append(out)
        return vals


@dataclass
class SerialConfig:
    port: Optional[str] = None
    baudrate: int = 115200
    timeout_s: float = 0.05
    write_delay_s: float = 0.01
    startup_delay_s: float = 1.5
    newline: str = "\n"
    protocol: SerialProtocol = SerialProtocol.TEENSY_Q_STEPS
    template: str = "Q {s1} {s2} {s3} {s4}"
    output: JointOutputConfig = field(default_factory=JointOutputConfig)


class BaggerRobot:
    """Low-level robot connection, TX/RX, and dynamic serial formatting."""

    def __init__(self, config: Optional[SerialConfig] = None, dry_run: bool = True) -> None:
        self.config = config or SerialConfig()
        self.dry_run = dry_run
        self._ser = None
        self._rx_queue: Queue[str] = Queue()
        self._stop_reader = Event()
        self._reader_thread: Optional[Thread] = None
        self.command_log: list[str] = []
        self.last_q: Optional[JointVector] = None
        self.last_steps: Optional[MotorSteps] = None

    @staticmethod
    def available_ports() -> list[str]:
        if list_ports is None:
            return []
        return [p.device for p in list_ports.comports()]

    @property
    def connected(self) -> bool:
        return self._ser is not None and getattr(self._ser, "is_open", False)

    def connect(self, port: Optional[str] = None) -> None:
        if port is not None:
            self.config.port = port
        if self.dry_run:
            return
        if serial is None:
            detail = f" ({SERIAL_IMPORT_ERROR})" if SERIAL_IMPORT_ERROR else ""
            raise RuntimeError(f"pyserial is not available{detail}. Run: python -m pip install pyserial")
        if not self.config.port:
            raise ValueError("No serial port selected.")

        self._ser = serial.Serial(
            self.config.port,
            self.config.baudrate,
            timeout=self.config.timeout_s,
        )
        sleep(self.config.startup_delay_s)
        self._start_reader()

    def disconnect(self) -> None:
        self._stop_reader.set()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=0.5)
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def _start_reader(self) -> None:
        self._stop_reader.clear()
        self._reader_thread = Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def _read_loop(self) -> None:
        while not self._stop_reader.is_set() and self._ser is not None:
            try:
                if self._ser.in_waiting:
                    line = self._ser.readline().decode(errors="replace").strip()
                    if line:
                        self._rx_queue.put(line)
                else:
                    sleep(0.01)
            except Exception as exc:  # keep UI alive if serial gets unplugged
                self._rx_queue.put(f"RX_ERROR: {exc}")
                sleep(0.1)

    def read_all(self) -> list[str]:
        lines: list[str] = []
        while not self._rx_queue.empty():
            lines.append(self._rx_queue.get_nowait())
        return lines

    def write_line(self, line: str) -> None:
        self.command_log.append(line)
        if self.dry_run:
            return
        if not self.connected or self._ser is None:
            raise RuntimeError("Serial port is not connected.")
        self._ser.write((line + self.config.newline).encode())
        sleep(self.config.write_delay_s)

    def enable(self, enabled: bool = True) -> None:
        self.write_line(f"EN {1 if enabled else 0}")

    def stop(self) -> None:
        self.write_line("STOP")

    def ping(self) -> None:
        self.write_line("PING")

    def zero(self, joint: int | None = None) -> None:
        """Set the Teensy's current step counter(s) to zero without moving."""
        if joint is None:
            self.write_line("ZERO")
            return
        self._validate_joint_index(joint)
        self.write_line(f"ZERO J{joint}")

    def set_current_steps(self, steps: Iterable[int]) -> None:
        """Tell the Teensy: this physical pose equals these absolute motor steps.

        This does not move the robot. It only resets AccelStepper's internal
        currentPosition()/targetPosition() counters. Use after physically placing
        the arm at a known calibration pose.
        """
        s = tuple(int(round(v)) for v in steps)
        if len(s) != 4:
            raise ValueError("Expected four current motor step values: [s1, s2, s3, s4].")
        self.write_line(f"SETPOS {s[0]} {s[1]} {s[2]} {s[3]}")
        self.last_steps = s

    def set_current_joint_steps(self, joint: int, steps: int) -> None:
        """Set one Teensy current-position counter in controller steps."""
        self._validate_joint_index(joint)
        self.write_line(f"SETPOS J{joint} {int(round(steps))}")

    def set_motor_angle(self, joint: int, angle_deg: float) -> None:
        """Low-level direct motor-space angle command.

        The Teensy converts this motor angle to steps using its local
        MICROSTEPS/FULL_STEPS_PER_REV constants. This bypasses robot IK.
        For physical arm angles, prefer Python kinematics -> send_motor_steps().
        """
        self._validate_joint_index(joint)
        self.write_line(f"SETANGLE J{joint} {float(angle_deg):.6f}")

    def request_position(self) -> None:
        self.write_line("POS")

    def request_status(self) -> None:
        self.write_line("STAT")

    def set_servo(self, angle_deg: int) -> None:
        self.write_line(f"SERVO {int(angle_deg)}")

    @staticmethod
    def _validate_joint_index(joint: int) -> None:
        if joint < 1 or joint > 4:
            raise ValueError("joint must be 1, 2, 3, or 4")

    # -----------------------------
    # Preferred Teensy step protocol
    # -----------------------------

    def format_motor_steps_command(self, steps: Iterable[int]) -> list[str]:
        s = tuple(int(round(v)) for v in steps)
        if len(s) != 4:
            raise ValueError("Expected four motor step targets: [s1, s2, s3, s4].")

        protocol = self.config.protocol
        if protocol == SerialProtocol.TEENSY_Q_STEPS:
            return [f"Q {s[0]} {s[1]} {s[2]} {s[3]}"]

        if protocol == SerialProtocol.TEENSY_MOVE_STEPS:
            return [f"MOVE {s[0]} {s[1]} {s[2]} {s[3]}"]

        if protocol == SerialProtocol.J_COMMANDS:
            return [f"J{i + 1} {value}" for i, value in enumerate(s)]

        if protocol == SerialProtocol.M_COMMANDS:
            return [f"M{i + 1} {value}" for i, value in enumerate(s)]

        if protocol == SerialProtocol.JSON:
            return [json.dumps({"steps": list(s), "timestamp": time()})]

        if protocol == SerialProtocol.TEMPLATE:
            fields = {f"s{i + 1}": value for i, value in enumerate(s)}
            fields.update({"steps": s, "s_csv": ",".join(str(v) for v in s)})
            return [self.config.template.format(**fields)]

        raise ValueError(f"Protocol {protocol.value} is not a motor-step protocol.")

    def send_motor_steps(self, steps: Iterable[int]) -> list[str]:
        s_tuple = tuple(int(round(v)) for v in steps)
        if len(s_tuple) != 4:
            raise ValueError("Expected four motor step targets: [s1, s2, s3, s4].")
        lines = self.format_motor_steps_command(s_tuple)
        for line in lines:
            self.write_line(line)
        self.last_steps = s_tuple  # optimistic until real feedback exists
        return lines

    # -----------------------------
    # Legacy physical-joint protocol
    # -----------------------------

    def format_joint_command(self, q: Iterable[float]) -> list[str]:
        q_tuple = tuple(float(v) for v in q)
        q_scaled = self.config.output.apply(q_tuple)
        protocol = self.config.protocol

        if protocol == SerialProtocol.CSV_RADIANS:
            return [",".join(str(v) for v in q_scaled)]

        if protocol == SerialProtocol.CSV_DEGREES:
            vals = [degrees(q_tuple[0]), degrees(q_tuple[1]), q_tuple[2], degrees(q_tuple[3])]
            vals = self.config.output.apply(vals)
            return [",".join(str(v) for v in vals)]

        if protocol == SerialProtocol.JSON:
            return [json.dumps({"q": q_scaled, "timestamp": time()})]

        if protocol == SerialProtocol.M_COMMANDS:
            return [f"M{i + 1} {value}" for i, value in enumerate(q_scaled)]

        if protocol == SerialProtocol.J_COMMANDS:
            return [f"J{i + 1} {value}" for i, value in enumerate(q_scaled)]

        if protocol == SerialProtocol.ONE_PER_LINE:
            return [f"J{i + 1}:{value}" for i, value in enumerate(q_scaled)]

        if protocol == SerialProtocol.TEMPLATE:
            fields = {f"q{i + 1}": value for i, value in enumerate(q_scaled)}
            fields.update({"q": q_scaled, "q_csv": ",".join(str(v) for v in q_scaled)})
            return [self.config.template.format(**fields)]

        raise ValueError(f"Protocol {protocol.value} is a motor-step protocol. Use send_motor_steps().")

    def send_joint_positions(self, q: Iterable[float]) -> list[str]:
        q_tuple = tuple(float(v) for v in q)
        if len(q_tuple) != 4:
            raise ValueError("Expected four joint values: [q1, q2, q3, q4].")
        lines = self.format_joint_command(q_tuple)
        for line in lines:
            self.write_line(line)
        self.last_q = q_tuple
        return lines

    def send_queue_steps(self, queue: Iterable[Iterable[int]]) -> list[str]:
        sent: list[str] = []
        for steps in queue:
            sent.extend(self.send_motor_steps(steps))
        return sent

    # -----------------------------
    # Coordinated motion helpers
    # -----------------------------

    def wait_for_done(self, timeout_s: float = 30.0) -> bool:
        """Block until the Teensy emits a 'DONE' line. Returns True on DONE, False on timeout.

        Lines other than DONE are returned to the queue so the caller can still see them.
        """
        if self.dry_run:
            return True
        deadline = time() + timeout_s
        stash: list[str] = []
        try:
            while time() < deadline:
                got_done = False
                while not self._rx_queue.empty():
                    line = self._rx_queue.get_nowait()
                    if line.startswith("DONE"):
                        got_done = True
                        break
                    stash.append(line)
                if got_done:
                    return True
                sleep(0.005)
            return False
        finally:
            for line in stash:
                self._rx_queue.put(line)

    def send_motor_steps_waypoints(
        self,
        waypoints: Iterable[Iterable[int]],
        wait_each: bool = True,
        timeout_s: float = 10.0,
        interval_s: float = 0.0,
    ) -> list[str]:
        """Send a sequence of absolute step targets.

        wait_each=True  : block on Teensy 'DONE' between waypoints (precise, slower).
        wait_each=False : fire-and-forget at fixed rate (continuous re-targeting,
                          smoother but lower per-waypoint accuracy).
        """
        all_lines: list[str] = []
        for steps in waypoints:
            all_lines.extend(self.send_motor_steps(steps))
            if wait_each:
                self.wait_for_done(timeout_s)
            elif interval_s > 0:
                sleep(interval_s)
        return all_lines

    def move_single_motor(self, joint: int, steps: int) -> None:
        """Move ONE motor to an absolute step position using Teensy's J{n} command.

        This is the diagnostic primitive: it bypasses all transmission coupling logic
        and commands a raw motor step target. Use with kinematics.predict_single_motor_jog
        to verify whether the configured transmission matches the real hardware.
        """
        self._validate_joint_index(joint)
        self.write_line(f"J{joint} {int(round(steps))}")

    def __enter__(self) -> "BaggerRobot":
        self.connect()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.disconnect()
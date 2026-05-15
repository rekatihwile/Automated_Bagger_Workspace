# ============================================================
#  arm_controller.py
#  Windows PC controller for RRPR robotic manipulator
#
#  Usage:
#    python arm_controller.py
#    python arm_controller.py --port COM4 --file coordinates.csv
#
#  Coordinate file format (CSV with header):
#    X,Y,Z,Phi
#    100,50,200,0
#    150,75,150,45
# ============================================================

import serial
import time
import csv
import os
import argparse
import math

# ============================================================
#  CONFIGURATION  — edit these defaults as needed
# ============================================================

DEFAULT_PORT      = 'COM4'
DEFAULT_BAUD      = 115200
DEFAULT_COORD_FILE = 'coordinates.csv'
STEPS_PER_DEG_J1 = 142.857
STEPS_PER_DEG_J2 = 142.857
STEPS_PER_MM_J3 = 50.93
STEPS_PER_DEG_J4 = 4.444

# ---- Claw servo angles ----
CLAW_OPEN_ANGLE   = 70   # fully open
CLAW_CLOSED_ANGLE = 10   # fully closed / grasping

# ============================================================
#  HOME POSITIONS  (step counts)
#  These must match the HOME_J* constants in arm_firmware.ino.
# ============================================================

HOME_STEPS = {
    "J1": 0,   # TODO: set your home step value for J1
    "J2": 12857,   # TODO: set your home step value for J2
    "J3": 0,   # TODO: set your home step value for J3
    "J4": -400,   # TODO: set your home step value for J4
}

# ============================================================
#  INVERSE KINEMATICS
#
#  Convert a Cartesian target (X, Y, Z, Phi) into absolute
#  step counts for each joint.
#
#  Fill in your own equations below.  The return value must be
#  a tuple of four integers: (j1_steps, j2_steps, j3_steps, j4_steps).
#
#  Geometry reminders:
#    J1 — base revolute  (horizontal plane)
#    J2 — elbow revolute (horizontal plane)
#    J3 — prismatic lift (vertical)
#    J4 — wrist revolute (end-effector orientation, horizontal plane)
# ============================================================

def inverse_kinematics(x: float, y: float, z: float, phi: float):
    """
    Compute joint step targets from a Cartesian pose.

    Parameters
    ----------
    x, y : float  — horizontal-plane position (mm)
    z    : float  — vertical height (mm)
    phi  : float  — end-effector heading (degrees)

    Returns
    -------
    (j1_steps, j2_steps, j3_steps, j4_steps) : tuple[int, int, int, int]
    """

    L1 = 450         # link 1 length (mm)
    L2 = 450          # link 2 length (mm)

    cos_q2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = math.acos(cos_q2)
    theta1 = math.atan2(y, x) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    
    j1_steps = int(math.degrees(theta1) * -STEPS_PER_DEG_J1)             # J1 = n * q1
    j2_steps = int(math.degrees(theta1 + theta2) * STEPS_PER_DEG_J2)   # J2 = n * (q1 + q2)
    j3_steps = int(-z * STEPS_PER_MM_J3)
    j4_steps = int((phi - math.degrees(theta1 + theta2)) * STEPS_PER_DEG_J4)

    return j1_steps, j2_steps, j3_steps, j4_steps

# ============================================================
#  COORDINATE FILE LOADER
# ============================================================

def load_coordinates(filepath: str) -> list[tuple]:
    """
    Load target poses from a CSV file.

    Expected columns (case-sensitive header):  X, Y, Z, Phi
    Returns a list of (x, y, z, phi) tuples of floats.
    Raises FileNotFoundError or KeyError with a clear message on bad input.
    """

    coords = []

    with open(filepath, newline='') as f:
        reader = csv.DictReader(f)

        required = {'X', 'Y', 'Z', 'Phi'}
        if not required.issubset(reader.fieldnames or []):
            missing = required - set(reader.fieldnames or [])
            raise KeyError(f"Coordinate file is missing columns: {missing}")

        for i, row in enumerate(reader, start=2):   # start=2 because row 1 is the header
            try:
                coords.append((
                    float(row['X']),
                    float(row['Y']),
                    float(row['Z']),
                    float(row['Phi'])
                ))
            except ValueError as e:
                raise ValueError(f"Cannot parse row {i}: {e}")

    return coords

# ============================================================
#  ARM CONTROLLER  (serial communication)
# ============================================================

class ArmController:
    """Manages the serial connection to the Teensy 4.1."""

    def __init__(self, port: str, baud: int):
        print(f"Connecting to {port} at {baud} baud...")
        self.ser = serial.Serial(port, baud, timeout=2)
        time.sleep(2)                       # allow Teensy to finish booting
        self._flush_startup()
        self.claw_open = True               # firmware boots with servo at 40°, but
                                            # we'll command an explicit open on connect
        self.set_claw(open=True)
        print("Connected.\n")

    # ---- Internal helpers ----

    def _flush_startup(self):
        """Discard any boot messages (e.g. 'READY')."""
        time.sleep(0.1)
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors='replace').strip()
            if line:
                print(f"  [boot] {line}")

    def _send(self, cmd: str):
        self.ser.write((cmd + '\n').encode())

    def _read_line(self, timeout: float = 5.0) -> str:
        """Read one newline-terminated response, return stripped string."""
        self.ser.timeout = timeout
        return self.ser.readline().decode(errors='replace').strip()

    def _wait_for(self, token: str, timeout: float = 30.0) -> bool:
        """
        Block until a line containing `token` is received or timeout expires.
        Prints all intermediate Teensy output for visibility.
        Returns True on success, False on timeout.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            remaining = max(0.05, deadline - time.time())
            line = self._read_line(timeout=remaining)
            if line:
                print(f"  [Teensy] {line}")
            if token in line:
                return True
        return False

    # ---- Public interface ----

    def move(self, j1: int, j2: int, j3: int, j4: int) -> bool:
        """
        Send a MOVE command and block until all joints reach their targets.
        Returns True on success, False on timeout or communication error.
        """
        cmd = f"MOVE {j1} {j2} {j3} {j4}"
        print(f"  → {cmd}")
        self._send(cmd)

        if not self._wait_for("MOVING", timeout=5.0):
            print("  WARNING: no MOVING acknowledgement received.")
            return False

        print("  Waiting for motion to complete...")
        if not self._wait_for("DONE", timeout=60.0):
            print("  ERROR: motion did not complete within 60 s.")
            return False

        print("  Move complete.")
        return True

    def home(self) -> bool:
        """
        Send HOME command. The arm does not move — the Teensy simply
        reassigns its step counters to the HOME_J* preset values.
        Execute only when the arm is physically in the home pose.
        """
        print("  Setting home pose...")
        self._send("HOME")

        if not self._wait_for("HOMED", timeout=3.0):
            print("  WARNING: no HOMED acknowledgement received.")
            return False

        print("  Home pose set.")
        return True

    def zero(self):
        """Declare the current physical position as step zero."""
        self._send("ZERO")
        self._wait_for("ZEROED", timeout=3.0)

    def get_position(self) -> dict | None:
        """
        Query current step positions.
        Returns {'J1': int, 'J2': int, 'J3': int, 'J4': int} or None.
        """
        self._send("POS")
        line = self._read_line(timeout=3.0)
        if line.startswith("POS"):
            parts = line.split()
            if len(parts) == 5:
                return {
                    "J1": int(parts[1]),
                    "J2": int(parts[2]),
                    "J3": int(parts[3]),
                    "J4": int(parts[4]),
                }
        print(f"  WARNING: unexpected POS response: '{line}'")
        return None

    def enable(self, state: bool = True):
        self._send(f"EN {1 if state else 0}")
        self._wait_for("ENABLED" if state else "DISABLED", timeout=3.0)

    def set_claw(self, open: bool):
        """
        Drive the claw servo to either the open or closed angle.
        Does not block — the servo reaches its target in ~300 ms.
        """
        angle = CLAW_OPEN_ANGLE if open else CLAW_CLOSED_ANGLE
        self._send(f"SERVO {angle}")
        self._wait_for("SERVO OK", timeout=2.0)
        self.claw_open = open

    def toggle_claw(self):
        """Flip the claw between open and closed."""
        self.set_claw(open=not self.claw_open)

    def close(self):
        """Disable motors and close the serial port cleanly."""
        self.enable(False)
        self.ser.close()

# ============================================================
#  DISPLAY HELPERS
# ============================================================

def display_move_table(loaded_coords: list[tuple], computed_steps: list[tuple]):
    """Print a formatted table of all loaded moves."""
    print()
    print(f"  {'#':<5} {'X':>8} {'Y':>8} {'Z':>8} {'Phi':>8}    "
          f"{'J1':>8} {'J2':>8} {'J3':>8} {'J4':>8}")
    print("  " + "─" * 78)
    for i, ((x, y, z, phi), (j1, j2, j3, j4)) in enumerate(
            zip(loaded_coords, computed_steps)):
        print(f"  {i:<5} {x:>8.2f} {y:>8.2f} {z:>8.2f} {phi:>8.2f}    "
              f"{j1:>8} {j2:>8} {j3:>8} {j4:>8}")
    print()

def display_position(pos: dict):
    if pos:
        print(f"\n  Current step positions:")
        for k, v in pos.items():
            print(f"    {k}: {v:>10} steps")
    else:
        print("  Could not read position from Teensy.")

# ============================================================
#  MAIN MENU
# ============================================================

MENU = """
╔══════════════════════════════════╗
║      Robot Arm Controller        ║
╠══════════════════════════════════╣
║  1. Load coordinates from file   ║
║  2. Execute a move               ║
║  3. Run all moves in sequence    ║
║  4. Set home pose (no motion)    ║
║  5. Show current position        ║
║  6. Enable motors                ║
║  7. Disable motors               ║
║  8. Zero (set current pos = 0)   ║
║  9. Toggle claw (open / closed)  ║
║  0. Quit                         ║
╚══════════════════════════════════╝
"""

def main():

    parser = argparse.ArgumentParser(description="RRPR Robot Arm Controller")
    parser.add_argument('--port', default=DEFAULT_PORT,
                        help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument('--file', default=DEFAULT_COORD_FILE,
                        help=f"Coordinate CSV file (default: {DEFAULT_COORD_FILE})")
    args = parser.parse_args()

    arm = ArmController(args.port, DEFAULT_BAUD)

    loaded_coords: list[tuple] = []   # (x, y, z, phi) from file
    computed_steps: list[tuple] = []  # (j1, j2, j3, j4) after IK

    while True:

        print(MENU)
        choice = input("Select option: ").strip()

        # ---- 1. Load coordinates ----
        if choice == '1':
            path = input(f"  File path [{args.file}]: ").strip() or args.file

            try:
                loaded_coords  = load_coordinates(path)
                computed_steps = [inverse_kinematics(*c) for c in loaded_coords]
                print(f"\n  Loaded {len(loaded_coords)} pose(s) from '{path}':")
                display_move_table(loaded_coords, computed_steps)
            except (FileNotFoundError, KeyError, ValueError) as e:
                print(f"  ERROR: {e}")

        # ---- 2. Execute a single move ----
        elif choice == '2':
            if not computed_steps:
                print("  No coordinates loaded — use option 1 first.")
                continue

            display_move_table(loaded_coords, computed_steps)
            raw = input("  Enter move index: ").strip()
            try:
                i = int(raw)
                j1, j2, j3, j4 = computed_steps[i]
                arm.move(j1, j2, j3, j4)
            except (ValueError, IndexError):
                print("  Invalid index.")

        # ---- 3. Run all moves in sequence ----
        elif choice == '3':
            if not computed_steps:
                print("  No coordinates loaded — use option 1 first.")
                continue

            display_move_table(loaded_coords, computed_steps)
            confirm = input(f"  Run all {len(computed_steps)} move(s)? [y/N]: ").strip().lower()
            if confirm == 'y':
                for i, (j1, j2, j3, j4) in enumerate(computed_steps):
                    print(f"\n  ── Move {i + 1} / {len(computed_steps)} ──")
                    ok = arm.move(j1, j2, j3, j4)
                    if not ok:
                        print("  Aborting sequence due to error.")
                        break
                print("\n  Sequence complete.")

        # ---- 4. Set home pose ----
        elif choice == '4':
            print(f"  The step counters will be reassigned to: {HOME_STEPS}")
            print(f"  Ensure the arm is physically in the home pose before confirming.")
            confirm = input("  Confirm? [y/N]: ").strip().lower()
            if confirm == 'y':
                arm.home()

        # ---- 5. Position ----
        elif choice == '5':
            display_position(arm.get_position())

        # ---- 6. Enable ----
        elif choice == '6':
            arm.enable(True)
            print("  Motors enabled.")

        # ---- 7. Disable ----
        elif choice == '7':
            arm.enable(False)
            print("  Motors disabled.")

        # ---- 8. Zero ----
        elif choice == '8':
            confirm = input("  Set current position as step zero? [y/N]: ").strip().lower()
            if confirm == 'y':
                arm.zero()
                print("  All joint positions set to zero.")

        # ---- 9. Toggle claw ----
        elif choice == '9':
            arm.toggle_claw()
            state = "open" if arm.claw_open else "closed"
            print(f"  Claw now {state}.")

        # ---- 0. Quit ----
        elif choice == '0':
            arm.close()
            print("  Motors disabled. Goodbye.")
            break

        else:
            print("  Unknown option.")

# ============================================================

if __name__ == "__main__":
    main()
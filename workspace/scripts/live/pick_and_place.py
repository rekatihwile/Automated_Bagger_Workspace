# ============================================================
#  pick_and_place.py
#  High-level pick-and-place controller for the RRPR arm.
#
#  Architecture:
#    - Imports the hardware layer (ArmController + IK) from
#      arm_controller.py. This file owns only the task logic.
#    - A worker thread runs the state machine; the main thread
#      runs the menu. This keeps pause/stop responsive even
#      though individual moves are blocking.
#    - The CV program writes a JSON file (cv_target.json) containing
#      an array of detected objects. This script reads the first
#      entry, executes a pick-and-place cycle for it, then rewrites
#      the file with the remaining entries (deleting the file when
#      empty). This allows multiple objects in a scene to be grasped
#      sequentially.
#
#  Testing without the CV program:
#    Create cv_target.json by hand with the schema below to
#    simulate a detection (array of one or more targets).
#
#      [
#        {"x": 350.0, "y": 200.0, "z": 10.0, "phi": 0.0},
#        {"x": 380.0, "y": 220.0, "z": 10.0, "phi": 45.0}
#      ]
# ============================================================

import argparse
import json
import threading
import time
from enum import Enum
from pathlib import Path

from arm_controller import (
    ArmController,
    inverse_kinematics,
    display_position,
    DEFAULT_PORT,
    DEFAULT_BAUD,
)

# ============================================================
#  POSITION CONFIGURATION  — TODO: fill in for your arm
# ============================================================

# ---- Neutral pose: the arm waits here between picks ----
NEUTRAL_X   = 450   # mm
NEUTRAL_Y   = 450   # mm
NEUTRAL_Z   = 200   # mm
NEUTRAL_PHI = 0.0   # deg

# ---- Pre-grasp height (elevated Z above target) ----
# Used both as the approach height before descending onto the object
# and as the lift height after grasping. Should be high enough that
# the claw clears anything in the workspace while traveling.
PRE_GRASP_Z = 200   # mm

# ---- Placement location ----
PLACE_X      = 300   # mm
PLACE_Y      = -250   # mm
PLACE_PHI    = 0.0   # deg
PLACE_Z      = 50   # mm — release height (low)
POST_PLACE_Z = 200   # mm — retract height after releasing

# ============================================================
#  CV INTEGRATION
#
#  The CV program writes a single JSON file containing an array of
#  detected objects:
#      [
#        {"x": <mm>, "y": <mm>, "z": <mm>, "phi": <deg>},
#        ...
#      ]
#
#  Each pick cycle pops the first entry and rewrites the file with
#  the remaining entries. When the array is empty the file is
#  deleted. This means the file always represents the work that is
#  still outstanding.
# ============================================================

CV_TARGET_FILE   = Path(r"C:\Users\elipp\OneDrive\Documents\Grocery Bagger\workspace\scripts\live\cv_target.json")
CV_POLL_INTERVAL = 0.3   # seconds between checks while waiting for a target

# ============================================================
#  STATE MACHINE
# ============================================================

class State(Enum):
    IDLE                 = "idle (sequence not running)"
    WAITING_FOR_TARGET   = "at neutral, waiting for CV target"
    MOVING_TO_PREGRASP   = "moving to pre-grasp"
    DESCENDING_TO_GRASP  = "descending to grasp"
    GRASPING             = "closing claw"
    LIFTING_OBJECT       = "lifting object"
    MOVING_TO_PLACE      = "moving to placement"
    DESCENDING_TO_PLACE  = "descending to release"
    RELEASING            = "opening claw"
    LIFTING_AFTER_PLACE  = "retracting after release"
    RETURNING_TO_NEUTRAL = "returning to neutral"
    PAUSED               = "paused"


class PickAndPlace:
    """
    Pick-and-place state machine.

    Public methods (called from the UI thread):
        start()   — begin the sequence in a worker thread
        pause()   — request a pause after the current motion
        resume()  — resume a paused sequence
        stop()    — request a clean stop after the current motion

    Internal flow (running in the worker thread):
        1. Move to neutral, open claw.
        2. Poll for a CV target.
        3. When a target arrives, run one pick-and-place cycle.
        4. Return to neutral and repeat from step 2.
    """

    def __init__(self, arm: ArmController):
        self.arm = arm

        self.state           = State.IDLE
        self.current_target  = None   # last (x, y, z, phi) being processed

        # Threading primitives
        self._thread         = None
        self._pause_event    = threading.Event()
        self._pause_event.set()                # set → running; clear → paused
        self._stop_requested = False
        self._running        = False

    # ------------------------------------------------------------------
    # Public control API
    # ------------------------------------------------------------------

    def start(self):
        if self._running:
            print("  Sequence already running.")
            return
        self._running        = True
        self._stop_requested = False
        self._pause_event.set()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print("  Sequence started — moving to neutral, then watching for CV targets.")

    def pause(self):
        if not self._running:
            print("  Sequence is not running.")
            return
        if not self._pause_event.is_set():
            print("  Already paused.")
            return
        self._pause_event.clear()
        print("  Pause requested — will pause after the current motion completes.")

    def resume(self):
        if not self._running:
            print("  Sequence is not running.")
            return
        if self._pause_event.is_set():
            print("  Not currently paused.")
            return
        self._pause_event.set()
        print("  Resumed.")

    def stop(self):
        if not self._running:
            print("  Sequence is not running.")
            return
        self._stop_requested = True
        self._pause_event.set()              # release the worker if paused
        print("  Stop requested — will stop after the current motion completes.")
        if self._thread is not None:
            self._thread.join(timeout=90)
        self._running = False
        self.state    = State.IDLE
        print("  Sequence stopped.")

    @property
    def is_running(self) -> bool:
        return self._running

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _check(self) -> bool:
        """
        Called between motions inside the worker thread.
        If paused, blocks here until resumed (state shows PAUSED in the meantime).
        Returns True to continue, False if stop has been requested.
        """
        if not self._pause_event.is_set():
            previous_state = self.state
            self.state = State.PAUSED
            self._pause_event.wait()
            self.state = previous_state
        return not self._stop_requested

    def _move(self, x: float, y: float, z: float, phi: float):
        """Run IK and command a blocking move."""
        j1, j2, j3, j4 = inverse_kinematics(x, y, z, phi)
        self.arm.move(j1, j2, j3, j4)

    def _read_cv_target(self):
        """
        Read the next target from the CV file.
        Pops the first entry from the JSON array, then either:
          - rewrites the file with the remaining entries, or
          - deletes the file if no entries remain.
        Returns (x, y, z, phi) or None if no valid target is available.
        """
        if not CV_TARGET_FILE.exists():
            return None

        # ---- Load the array ----
        try:
            with open(CV_TARGET_FILE) as f:
                data = json.load(f)
            if not isinstance(data, list):
                raise ValueError("expected a JSON array at the top level")
        except (json.JSONDecodeError, ValueError, OSError) as e:
            print(f"  Bad CV target file ({e}); discarding.")
            self._remove_target_file()
            return None

        # ---- Empty array: nothing to do, clear the file ----
        if len(data) == 0:
            self._remove_target_file()
            return None

        # ---- Pop the first entry ----
        first     = data[0]
        remaining = data[1:]

        target = None
        try:
            target = (
                float(first["x"]),
                float(first["y"]),
                float(first["z"]),
                float(first["phi"]),
            )
        except (KeyError, ValueError, TypeError) as e:
            # Malformed entry — skip it and let the loop try the next one
            print(f"  Skipping malformed CV entry ({e}).")

        # ---- Write back the remainder (or delete if empty) ----
        if remaining:
            try:
                with open(CV_TARGET_FILE, "w") as f:
                    json.dump(remaining, f, indent=2)
            except OSError as e:
                print(f"  Could not update CV target file: {e}")
        else:
            self._remove_target_file()

        return target

    def _remove_target_file(self):
        """Quietly delete the CV target file if it exists."""
        try:
            CV_TARGET_FILE.unlink()
        except OSError:
            pass

    # ------------------------------------------------------------------
    # Worker thread entry point
    # ------------------------------------------------------------------

    def _run(self):
        """Main worker loop — runs in its own thread."""
        try:
            # ---- Initial setup: open claw, go to neutral ----
            self.arm.set_claw(open=True)
            self.state = State.RETURNING_TO_NEUTRAL
            self._move(NEUTRAL_X, NEUTRAL_Y, NEUTRAL_Z, NEUTRAL_PHI)

            # ---- Main loop: wait for target, then run one cycle ----
            while not self._stop_requested:
                if not self._check():
                    break

                target = self._wait_for_target()
                if target is None:    # _wait_for_target returns None on stop
                    break

                self.current_target = target
                print(f"  CV target received: "
                      f"X={target[0]:.2f}  Y={target[1]:.2f}  "
                      f"Z={target[2]:.2f}  Phi={target[3]:.2f}")
                self._cycle(target)
                self.current_target = None

        except Exception as e:
            print(f"  ERROR in state machine: {e}")
        finally:
            self.state    = State.IDLE
            self._running = False

    def _wait_for_target(self):
        """Poll for a CV target. Returns (x,y,z,phi) or None if stopped."""
        self.state = State.WAITING_FOR_TARGET
        while not self._stop_requested:
            if not self._check():
                return None
            target = self._read_cv_target()
            if target is not None:
                return target
            time.sleep(CV_POLL_INTERVAL)
        return None

    def _cycle(self, target):
        """Run one complete pick-and-place cycle for a single target."""
        x, y, z, phi = target

        # 1. Move above the target, claw open
        if not self._check(): return
        self.arm.set_claw(open=True)
        self.state = State.MOVING_TO_PREGRASP
        self._move(x, y, PRE_GRASP_Z, phi)

        # 2. Descend onto the target
        if not self._check(): return
        self.state = State.DESCENDING_TO_GRASP
        self._move(x, y, z, phi)

        # 3. Close the claw
        if not self._check(): return
        self.state = State.GRASPING
        self.arm.set_claw(open=False)

        # 4. Lift the object back to pre-grasp height
        if not self._check(): return
        self.state = State.LIFTING_OBJECT
        self._move(x, y, PRE_GRASP_Z, phi)

        # 5. Travel to the placement location at pre-grasp height
        if not self._check(): return
        self.state = State.MOVING_TO_PLACE
        self._move(PLACE_X, PLACE_Y, PRE_GRASP_Z, PLACE_PHI)

        # 6. Descend to the placement Z
        if not self._check(): return
        self.state = State.DESCENDING_TO_PLACE
        self._move(PLACE_X, PLACE_Y, PLACE_Z, PLACE_PHI)

        # 7. Open the claw to release
        if not self._check(): return
        self.state = State.RELEASING
        self.arm.set_claw(open=True)

        # 8. Retract upward
        if not self._check(): return
        self.state = State.LIFTING_AFTER_PLACE
        self._move(PLACE_X, PLACE_Y, POST_PLACE_Z, PLACE_PHI)

        # 9. Return to neutral
        if not self._check(): return
        self.state = State.RETURNING_TO_NEUTRAL
        self._move(NEUTRAL_X, NEUTRAL_Y, NEUTRAL_Z, NEUTRAL_PHI)


# ============================================================
#  USER INTERFACE
# ============================================================

MENU = """
╔══════════════════════════════════╗
║   Pick & Place Controller        ║
╠══════════════════════════════════╣
║  1. Start sequence               ║
║  2. Pause sequence               ║
║  3. Resume sequence              ║
║  4. Stop sequence                ║
║  5. Show sequence state          ║
║  6. Show arm position            ║
║  7. Set home pose (no motion)    ║
║  8. Enable motors                ║
║  9. Disable motors               ║
║  0. Quit                         ║
╚══════════════════════════════════╝
"""

def main():

    parser = argparse.ArgumentParser(description="RRPR Pick & Place Controller")
    parser.add_argument('--port', default=DEFAULT_PORT,
                        help=f"Serial port (default: {DEFAULT_PORT})")
    args = parser.parse_args()

    arm        = ArmController(args.port, DEFAULT_BAUD)
    controller = PickAndPlace(arm)

    try:
        while True:
            print(MENU)
            print(f"  Current state: {controller.state.value}")
            if controller.current_target is not None:
                t = controller.current_target
                print(f"  Working on: X={t[0]:.1f}  Y={t[1]:.1f}  "
                      f"Z={t[2]:.1f}  Phi={t[3]:.1f}")

            choice = input("Select option: ").strip()

            # ---- 1. Start ----
            if choice == '1':
                controller.start()

            # ---- 2. Pause ----
            elif choice == '2':
                controller.pause()

            # ---- 3. Resume ----
            elif choice == '3':
                controller.resume()

            # ---- 4. Stop ----
            elif choice == '4':
                controller.stop()

            # ---- 5. Show sequence state ----
            elif choice == '5':
                print(f"\n  State: {controller.state.value}")
                if controller.current_target is not None:
                    t = controller.current_target
                    print(f"  Target: X={t[0]:.2f}  Y={t[1]:.2f}  "
                          f"Z={t[2]:.2f}  Phi={t[3]:.2f}")

            # ---- 6. Show arm position ----
            elif choice == '6':
                display_position(arm.get_position())

            # ---- 7. Set home pose ----
            elif choice == '7':
                if controller.is_running:
                    print("  Cannot set home pose while sequence is running. Stop first.")
                else:
                    print("  The step counters will be reassigned to your HOME_STEPS values.")
                    print("  Ensure the arm is physically in the home pose before confirming.")
                    confirm = input("  Confirm? [y/N]: ").strip().lower()
                    if confirm == 'y':
                        arm.home()

            # ---- 8. Enable motors ----
            elif choice == '8':
                arm.enable(True)
                print("  Motors enabled.")

            # ---- 9. Disable motors ----
            elif choice == '9':
                if controller.is_running:
                    print("  Cannot disable motors while sequence is running. Stop first.")
                else:
                    arm.enable(False)
                    print("  Motors disabled.")

            # ---- 0. Quit ----
            elif choice == '0':
                if controller.is_running:
                    controller.stop()
                arm.close()
                print("  Motors disabled. Goodbye.")
                break

            else:
                print("  Unknown option.")

    except KeyboardInterrupt:
        print("\n  Interrupted.")
        if controller.is_running:
            controller.stop()
        arm.close()


# ============================================================

if __name__ == "__main__":
    main()
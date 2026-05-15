# Manipulator IK + Teensy Dev App

This package is the manipulator-side dev environment for the automated grocery bagger. It is intentionally separated from the stereo vision code so the camera pipeline can later output workspace coordinates without caring about Teensy serial formatting.

## Key architecture

```text
workspace xyz
  -> IK physical joints q = [q1, q2, q3, q4]
  -> transmission mapping, including base-mounted J2 coupling
  -> absolute motor step targets
  -> Teensy serial command
```

The DH/IK model uses the **physical elbow angle** `q2`. The motor command accounts for the J2 belt coupling afterward.

For the current expected base-mounted J2 layout:

```text
J2 motor target ∝ q1 + q2
```

That is implemented in `BaggerTransmission` as:

```python
j2_motor_rad_per_elbow_rad = 32.0
j2_motor_rad_per_j1_rad = 32.0
```

If the belt direction is reversed, set:

```python
j2_sign = -1.0
```

## Files

```text
workspace/manipulator/
  kinematics.py             # FK/IK + DH debug + transmission coupling + step conversion
  robot.py                  # serial connection + command formatting + zero/set-position helpers
  dev_app.py                # Tkinter queue/send UI
  teensy_serial_check.py    # non-motion Teensy serial sanity check

firmware/teensy_bagger_controller/
  teensy_bagger_controller.ino
```

## Run the dev app

From the repository root:

```powershell
python -m workspace.manipulator.dev_app
```

The app starts in dry-run mode.

## Check the Teensy serial connection

List ports:

```powershell
python -m workspace.manipulator.teensy_serial_check --list
```

Dry-run the check without opening serial:

```powershell
python -m workspace.manipulator.teensy_serial_check --port COM5 --dry-run
```

Actually check the board, without moving motors:

```powershell
python -m workspace.manipulator.teensy_serial_check --port COM5
```

The check sends only:

```text
PING
STAT
POS
```

It passes if it sees `READY` or `PONG`.

## Flash the Teensy

Open this sketch in Arduino IDE / Teensyduino:

```text
firmware/teensy_bagger_controller/teensy_bagger_controller.ino
```

Required libraries:

```text
AccelStepper
TMCStepper
Servo
```

## Preferred serial command

The Python app sends:

```text
Q <J1_steps> <J2_steps> <J3_steps> <J4_steps>
```

Example:

```text
Q 1200 -4500 3000 0
```

The Teensy replies:

```text
OK MOVE 1200 -4500 3000 0
DONE 1200 -4500 3000 0
```

## Useful manual commands

```text
PING
POS
STAT
EN 1
EN 0
ZERO
ZERO J2
SETPOS 0 0 0 0
SETPOS J2 12345
SETANGLE J1 45
STOP
FREEZE
SERVO 40
CLAW OPEN
CLAW CLOSE
J1 1000
Q 1000 2000 0 0
SETMAX 1 20000
SETACC 1 10000
LIMIT 1 -100000 100000
```

### Important command meanings

- `ZERO` / `ZERO Jn`: sets current step counter(s) to zero without moving.
- `SETPOS ...`: sets current step counter(s) to known absolute values without moving.
- `SETANGLE Jn deg`: low-level motor-space angle move. This is **not** physical IK angle and does not know about J2 coupling.
- `Q s1 s2 s3 s4`: preferred coordinated absolute step move from Python.

In the UI, **Set current = shown q** computes the coupled motor steps from the shown physical joint pose and sends `SETPOS s1 s2 s3 s4`. That is the safest calibration workflow because it includes the J2 belt compensation.

## Calibration notes

Start with dry-run on. Confirm the TX lines make sense before connecting hardware.

The live tuning panel exposes:

- `L1`, `L2`, `d_base`
- base offsets
- J3 limits
- microsteps
- J1 ratio
- J2 elbow ratio
- J2-from-J1 coupling ratio
- J3 steps/m
- signs for each joint

For your likely J2 belt coupling, start with:

```text
J2 elbow = 32
J2 from J1 = 32
J2 sign = 1
```

If J2 moves the wrong way, flip `J2 sign` to `-1`.

If J1 motion makes the elbow compensation worse instead of better, set `J2 from J1 = -32`.

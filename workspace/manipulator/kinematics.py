"""Kinematics + transmission utilities for the Automated Grocery Bagger RRPR arm.

Important separation:
- IK/FK use PHYSICAL JOINT COORDINATES of the links.
- The base-mounted J2 belt/gear coupling is handled only in BaggerTransmission.

Physical joint convention:
    q = [q1, q2, q3, q4]
    q1 = shoulder/base revolute angle [rad]
    q2 = actual elbow angle between L1 and L2 [rad]
    q3 = downward prismatic extension [m]
    q4 = wrist/end-effector rotation [rad]
    z  = base_z + d_base - q3

Motor/step convention:
    motor_steps = joint_to_motor_steps(q)

For the current base-mounted J2 belt concept, the expected default is:
    motor_2_angle ∝ q1 + q2

That compensates for the fact that the J2 motor does not rotate with link 1.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import acos, atan2, cos, hypot, pi, sin, sqrt, tau
from typing import Iterable, Literal, Optional

JointVector = tuple[float, float, float, float]
Pose = tuple[float, float, float]
MotorRadians = tuple[float, float, float, float]
MotorSteps = tuple[int, int, int, int]
ElbowMode = Literal["down", "up", "nearest"]


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return atan2(sin(angle), cos(angle))


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass(frozen=True)
class BaggerGeometry:
    """Physical DH/geometry parameters for the grocery bagger.

    These are the values you tune when the CAD/measured geometry changes.
    They describe the physical links, not the motor transmission.
    """

    # Link / base geometry
    l1: float = 0.45
    l2: float = 0.45
    d_base: float = 0.8
    base_x: float = 0.0
    base_y: float = 0.0
    base_z: float = 0.0

    # Joint limits in physical joint coordinates
    q3_min: float = 0.0
    q3_max: float = 0.7
    q1_min: float = -pi
    q1_max: float = pi
    q2_min: float = -pi
    q2_max: float = pi
    q4_min: float = -pi
    q4_max: float = pi

    # Optional physical zero offsets for measured link angles.
    # Keep these 0 unless your CAD/measured zero pose is not the math zero pose.
    q1_zero_offset: float = 0.0
    q2_zero_offset: float = 0.0
    q3_zero_offset: float = 0.0
    q4_zero_offset: float = 0.0

    # Workspace layout, matching the MATLAB sim defaults
    workspace_width: float = 1.0
    workspace_depth: float = 0.7
    workspace_center_x: float = 0.0
    workspace_center_y: float = 0.3

    staging_offset_x: float = 0.1
    staging_offset_y: float = 0.1
    staging_width: float = 0.5
    staging_depth: float = 0.5

    bag_offset_x: float = 0.75
    bag_offset_y: float = 0.2
    bag_width: float = 0.2
    bag_depth: float = 0.3
    bag_z: float = 0.4

    belt_z: float = 0.05
    nominal_item_height: float = 0.08

    @property
    def dx(self) -> float:
        return self.workspace_center_x - self.workspace_width / 2.0

    @property
    def dy(self) -> float:
        return self.workspace_center_y - self.workspace_depth / 2.0

    @property
    def bag_target(self) -> Pose:
        return (
            self.dx + self.bag_offset_x + self.bag_width / 2.0,
            self.dy + self.bag_offset_y + self.bag_depth / 2.0,
            self.bag_z,
        )

    @property
    def park_pose(self) -> Pose:
        return (self.base_x, self.base_y + self.workspace_center_y, self.base_z + self.d_base)

    @property
    def pick_z(self) -> float:
        return self.belt_z + self.nominal_item_height

    def in_staging_xy(self, x: float, y: float) -> bool:
        return (
            self.dx + self.staging_offset_x <= x <= self.dx + self.staging_offset_x + self.staging_width
            and self.dy + self.staging_offset_y <= y <= self.dy + self.staging_offset_y + self.staging_depth
        )

    def in_workspace_xy(self, x: float, y: float) -> bool:
        return self.dx <= x <= self.dx + self.workspace_width and self.dy <= y <= self.dy + self.workspace_depth


@dataclass(frozen=True)
class BaggerTransmission:
    """Mapping from physical joints to motor shaft rotation / controller steps.

    The useful default for the base-mounted J2 motor is:
        J2 motor rotation ∝ q1 + q2

    If the real belt direction is reversed, flip j2_sign to -1.
    If your pulley/gear ratio differs, tune j2_motor_rad_per_*.
    """

    # Stepper electronics
    full_steps_per_rev: float = 200.0
    microsteps: float = 8.0

    # Revolute motor shaft radians per physical joint radian.
    # If a 32:1 gearbox means the motor shaft turns 32 rad for 1 rad of joint output,
    # keep this at 32. If you command output-pulley-equivalent angle instead, set 1.
    j1_motor_rad_per_joint_rad: float = 32.0
    j2_motor_rad_per_elbow_rad: float = 32.0
    j2_motor_rad_per_j1_rad: float = 32.0  # coupling compensation: motor2 ∝ q2 + q1
    j4_motor_rad_per_joint_rad: float = 1.0

    # Motor direction signs. Flip these if a positive command moves the wrong way.
    j1_sign: float = 1.0
    j2_sign: float = 1.0
    j3_sign: float = 1.0
    j4_sign: float = 1.0

    # Motor-space zero offsets.
    j1_motor_zero_rad: float = 0.0
    j2_motor_zero_rad: float = 0.0
    j3_zero_m: float = 0.0
    j4_motor_zero_rad: float = 0.0

    # Linear axis conversion. Tune this experimentally once J3 hardware is known.
    # This maps physical q3 meters directly to steps.
    j3_steps_per_meter: float = 10000.0

    # Final controller step offsets after conversion.
    j1_home_steps: int = 0
    j2_home_steps: int = 0
    j3_home_steps: int = 0
    j4_home_steps: int = 0

    @property
    def steps_per_motor_rad(self) -> float:
        return (self.full_steps_per_rev * self.microsteps) / tau


class BaggerKinematics:
    """Analytic FK/IK plus motor transmission mapping for the RRPR manipulator."""

    def __init__(
        self,
        geometry: Optional[BaggerGeometry] = None,
        transmission: Optional[BaggerTransmission] = None,
    ) -> None:
        self.geometry = geometry or BaggerGeometry()
        self.transmission = transmission or BaggerTransmission()

    # -----------------------------
    # Live tuning helpers
    # -----------------------------

    def update_geometry(self, **kwargs: float) -> None:
        """Update geometry fields while the app is running.

        Example:
            kin.update_geometry(l1=0.47, l2=0.52, d_base=0.76)
        """
        self.geometry = replace(self.geometry, **kwargs)

    def update_transmission(self, **kwargs: float | int) -> None:
        """Update transmission fields while the app is running.

        Example:
            kin.update_transmission(j2_sign=-1, j2_motor_rad_per_j1_rad=32)
        """
        self.transmission = replace(self.transmission, **kwargs)

    # -----------------------------
    # Physical FK/IK
    # -----------------------------

    def _apply_physical_offsets(self, q: Iterable[float]) -> JointVector:
        q1, q2, q3, q4 = tuple(float(v) for v in q)
        g = self.geometry
        return (
            q1 + g.q1_zero_offset,
            q2 + g.q2_zero_offset,
            q3 + g.q3_zero_offset,
            q4 + g.q4_zero_offset,
        )

    def is_reachable_xy(self, x: float, y: float) -> bool:
        g = self.geometry
        x_local = x - g.base_x
        y_local = y - g.base_y
        r = hypot(x_local, y_local)
        return abs(g.l1 - g.l2) <= r <= (g.l1 + g.l2)

    def fk(self, q: Iterable[float]) -> Pose:
        """Forward kinematics in physical workspace coordinates."""
        q1, q2, q3, _q4 = self._apply_physical_offsets(q)
        g = self.geometry
        x = g.base_x + g.l1 * cos(q1) + g.l2 * cos(q1 + q2)
        y = g.base_y + g.l1 * sin(q1) + g.l2 * sin(q1 + q2)
        z = g.base_z + g.d_base - q3
        return (x, y, z)

    def ik(
        self,
        xyz: Iterable[float],
        elbow: ElbowMode = "down",
        q_prev: Optional[Iterable[float]] = None,
    ) -> JointVector:
        """Solve IK for physical joint coordinates, not motor commands."""
        x_world, y_world, z_world = tuple(float(v) for v in xyz)
        g = self.geometry
        x = x_world - g.base_x
        y = y_world - g.base_y
        z = z_world - g.base_z

        c2 = (x * x + y * y - g.l1 * g.l1 - g.l2 * g.l2) / (2.0 * g.l1 * g.l2)

        # Tolerate tiny floating point overshoot at the boundary.
        if c2 < -1.0 - 1e-9 or c2 > 1.0 + 1e-9:
            raise ValueError(f"Target ({x_world:.3f}, {y_world:.3f}, {z_world:.3f}) is outside planar reach.")
        c2 = _clamp(c2, -1.0, 1.0)

        # q3 is independent of the planar IK; let validate_joints flag it instead of
        # silently clamping (the old _clamp here hid out-of-range Z targets).
        q3_raw = g.d_base - z
        q4_raw = 0.0

        candidates: list[JointVector] = []
        errors: list[str] = []
        for q2_raw in (acos(c2), -acos(c2)):
            q1_raw = atan2(y, x) - atan2(g.l2 * sin(q2_raw), g.l1 + g.l2 * cos(q2_raw))

            # Remove physical zero offsets so returned q is the commanded physical coordinate.
            q = (
                wrap_to_pi(q1_raw - g.q1_zero_offset),
                wrap_to_pi(q2_raw - g.q2_zero_offset),
                q3_raw - g.q3_zero_offset,
                q4_raw - g.q4_zero_offset,
            )
            try:
                self.validate_joints(q)
            except ValueError as exc:
                # Keep iterating: one elbow branch can be valid while the other is out of limits.
                errors.append(str(exc))
                continue
            candidates.append(q)

        if not candidates:
            raise ValueError(
                f"No reachable IK solution for ({x_world:.3f}, {y_world:.3f}, {z_world:.3f}): "
                + "; ".join(errors)
            )

        if elbow == "up":
            return candidates[0]
        if elbow == "down" or q_prev is None:
            return candidates[-1]

        prev = tuple(float(v) for v in q_prev)
        return min(
            candidates,
            key=lambda q: sqrt(wrap_to_pi(q[0] - prev[0]) ** 2 + wrap_to_pi(q[1] - prev[1]) ** 2),
        )

    def validate_joints(self, q: Iterable[float]) -> None:
        q1, q2, q3, q4 = tuple(float(v) for v in q)
        g = self.geometry
        limits = (
            ("q1", q1, g.q1_min, g.q1_max),
            ("q2", q2, g.q2_min, g.q2_max),
            ("q3", q3, g.q3_min, g.q3_max),
            ("q4", q4, g.q4_min, g.q4_max),
        )
        for name, value, lo, hi in limits:
            if value < lo - 1e-9 or value > hi + 1e-9:
                raise ValueError(f"{name}={value:.4f} is outside [{lo:.4f}, {hi:.4f}].")

    def manipulability_xy(self, q: Iterable[float]) -> float:
        _q1, q2, _q3, _q4 = self._apply_physical_offsets(q)
        g = self.geometry
        return abs(g.l1 * g.l2 * sin(q2))

    def is_near_singularity(self, q: Iterable[float], sin_threshold: float = 0.1) -> bool:
        """True when the elbow is near a singularity (fully extended or fully folded).

        Near singularity the planar Jacobian is rank-deficient, so a small Cartesian
        request maps to a huge joint motion and any transmission error blows up.
        sin_threshold=0.1 corresponds to roughly within 6° of the singular configuration.
        """
        _q1, q2, _q3, _q4 = self._apply_physical_offsets(q)
        return abs(sin(q2)) < sin_threshold

    def cartesian_waypoints(
        self,
        xyz_start: Iterable[float],
        xyz_end: Iterable[float],
        n_segments: int = 20,
        elbow: ElbowMode = "nearest",
        q_prev: Optional[Iterable[float]] = None,
    ) -> list[JointVector]:
        """Sample n_segments+1 joint vectors along a Cartesian-straight line.

        Use this to drive straight-line moves on uncoordinated steppers. Each adjacent
        pair of waypoints needs only a tiny motor move, so the independent AccelStepper
        ramps on the Teensy can't curve the path noticeably between them.
        """
        p0 = tuple(float(v) for v in xyz_start)
        p1 = tuple(float(v) for v in xyz_end)
        n = max(1, int(n_segments))
        prev = tuple(float(v) for v in q_prev) if q_prev is not None else None
        waypoints: list[JointVector] = []
        for i in range(n + 1):
            t = i / n
            pose = (
                p0[0] + (p1[0] - p0[0]) * t,
                p0[1] + (p1[1] - p0[1]) * t,
                p0[2] + (p1[2] - p0[2]) * t,
            )
            q = self.ik(pose, elbow=elbow, q_prev=prev)
            waypoints.append(q)
            prev = q
        return waypoints

    def dh_table(self, q: Iterable[float]) -> list[dict[str, float | str]]:
        """Return a simple DH-style table for debug/reporting.

        This table uses physical joint angles. Motor angles do not belong here.
        """
        q1, q2, q3, q4 = self._apply_physical_offsets(q)
        g = self.geometry
        return [
            {"joint": "J1", "a": g.l1, "alpha": 0.0, "d": g.d_base, "theta": q1, "type": "R"},
            {"joint": "J2", "a": g.l2, "alpha": 0.0, "d": 0.0, "theta": q2, "type": "R"},
            {"joint": "J3", "a": 0.0, "alpha": 0.0, "d": -q3, "theta": 0.0, "type": "P"},
            {"joint": "J4", "a": 0.0, "alpha": 0.0, "d": 0.0, "theta": q4, "type": "R"},
        ]

    # -----------------------------
    # Transmission mapping
    # -----------------------------

    def joint_to_motor_radians(self, q: Iterable[float]) -> MotorRadians:
        """Convert physical joint coordinates to motor shaft coordinates.

        J2 coupling is handled here:
            m2 = sign * (k_elbow*q2 + k_j1*q1) + offset
        """
        q1, q2, q3, q4 = tuple(float(v) for v in q)
        t = self.transmission

        m1 = t.j1_sign * (t.j1_motor_rad_per_joint_rad * q1) + t.j1_motor_zero_rad
        m2 = t.j2_sign * (
            t.j2_motor_rad_per_elbow_rad * q2
            + t.j2_motor_rad_per_j1_rad * q1
        ) + t.j2_motor_zero_rad
        # J3 is linear, so the third slot is meters before step conversion.
        m3 = t.j3_sign * q3 + t.j3_zero_m
        m4 = t.j4_sign * (t.j4_motor_rad_per_joint_rad * q4) + t.j4_motor_zero_rad
        return (m1, m2, m3, m4)

    def motor_radians_to_joint(self, m: Iterable[float]) -> JointVector:
        """Approximate inverse of joint_to_motor_radians for feedback/debug."""
        m1, m2, m3, m4 = tuple(float(v) for v in m)
        t = self.transmission
        q1 = ((m1 - t.j1_motor_zero_rad) / t.j1_sign) / t.j1_motor_rad_per_joint_rad
        q2 = (((m2 - t.j2_motor_zero_rad) / t.j2_sign) - t.j2_motor_rad_per_j1_rad * q1) / t.j2_motor_rad_per_elbow_rad
        q3 = (m3 - t.j3_zero_m) / t.j3_sign
        q4 = ((m4 - t.j4_motor_zero_rad) / t.j4_sign) / t.j4_motor_rad_per_joint_rad
        return (q1, q2, q3, q4)

    def joint_to_motor_steps(self, q: Iterable[float]) -> MotorSteps:
        """Convert physical joint coordinates to absolute controller step targets."""
        m1, m2, m3, m4 = self.joint_to_motor_radians(q)
        t = self.transmission
        steps = (
            int(round(m1 * t.steps_per_motor_rad + t.j1_home_steps)),
            int(round(m2 * t.steps_per_motor_rad + t.j2_home_steps)),
            int(round(m3 * t.j3_steps_per_meter + t.j3_home_steps)),
            int(round(m4 * t.steps_per_motor_rad + t.j4_home_steps)),
        )
        return steps

    def ik_to_motor_steps(
        self,
        xyz: Iterable[float],
        elbow: ElbowMode = "down",
        q_prev: Optional[Iterable[float]] = None,
    ) -> tuple[JointVector, MotorSteps]:
        q = self.ik(xyz, elbow=elbow, q_prev=q_prev)
        return q, self.joint_to_motor_steps(q)

    # -----------------------------
    # Single-motor diagnostic
    # -----------------------------

    def predict_single_motor_jog(
        self,
        joint: int,
        delta_motor_rad: float,
        q_now: Iterable[float],
    ) -> dict[str, float | tuple[float, float, float] | str]:
        """Predict what the arm should do if ONE motor jogs by delta_motor_rad.

        This is the easiest way to verify the transmission model against reality:
            1. Note current EE pose (with a ruler).
            2. Command exactly this motor jog (no IK in the loop).
            3. Compare measured EE motion to the predicted delta returned here.
        If predicted and measured disagree, the row of the transmission for that motor
        is wrong (sign, gear ratio, or coupling structure).
        """
        if joint < 1 or joint > 4:
            raise ValueError("joint must be 1..4")
        q_now_t = tuple(float(v) for v in q_now)
        t = self.transmission
        notes = ""

        # Map motor-rad delta back to a joint-coordinate delta, isolating one motor.
        # This is NOT a full inverse: it answers "if I rotate ONLY this motor, what
        # does the geometry say should happen to physical joints?"
        if joint == 1:
            # Rotating only J1 motor: q1 changes; q2 stays as a joint coord, BUT
            # because m2 has a q1 coupling term, holding m2 fixed forces q2 to
            # change by -k_j1/k_elbow * dq1 to keep m2 = const.
            dq1 = delta_motor_rad / (t.j1_sign * t.j1_motor_rad_per_joint_rad)
            if t.j2_motor_rad_per_elbow_rad == 0:
                dq2 = 0.0
                notes = "WARNING: j2_motor_rad_per_elbow_rad is 0; q2 coupling ill-defined."
            else:
                dq2 = -(t.j2_motor_rad_per_j1_rad / t.j2_motor_rad_per_elbow_rad) * dq1
            dq3, dq4 = 0.0, 0.0
        elif joint == 2:
            # Rotating only J2 motor: m1 fixed → q1 fixed; m2 change → q2 change.
            dq1 = 0.0
            dq2 = delta_motor_rad / (t.j2_sign * t.j2_motor_rad_per_elbow_rad)
            dq3, dq4 = 0.0, 0.0
        elif joint == 3:
            dq1, dq2, dq4 = 0.0, 0.0, 0.0
            dq3 = delta_motor_rad / t.j3_sign
        else:  # joint == 4
            dq1, dq2, dq3 = 0.0, 0.0, 0.0
            dq4 = delta_motor_rad / (t.j4_sign * t.j4_motor_rad_per_joint_rad)

        q_after = (q_now_t[0] + dq1, q_now_t[1] + dq2, q_now_t[2] + dq3, q_now_t[3] + dq4)
        ee_before = self.fk(q_now_t)
        ee_after = self.fk(q_after)
        delta_ee = (
            ee_after[0] - ee_before[0],
            ee_after[1] - ee_before[1],
            ee_after[2] - ee_before[2],
        )
        return {
            "joint": joint,
            "delta_motor_rad": delta_motor_rad,
            "predicted_dq": (dq1, dq2, dq3, dq4),
            "predicted_ee_before": ee_before,
            "predicted_ee_after": ee_after,
            "predicted_delta_ee": delta_ee,
            "notes": notes,
        }

    # -----------------------------
    # Task queue helpers
    # -----------------------------

    def make_pick_place_queue(
        self,
        x: float,
        y: float,
        item_height: Optional[float] = None,
        q_prev: Optional[Iterable[float]] = None,
    ) -> list[JointVector]:
        """Return hover/pick/lift/transfer/place/retract/park physical joint targets."""
        g = self.geometry
        z_pick = g.belt_z + (g.nominal_item_height if item_height is None else float(item_height))
        bag_x, bag_y, bag_z = g.bag_target
        poses: list[Pose] = [
            (x, y, g.base_z + g.d_base),
            (x, y, z_pick),
            (x, y, g.base_z + g.d_base),
            (bag_x, bag_y, g.base_z + g.d_base),
            (bag_x, bag_y, bag_z),
            (bag_x, bag_y, g.base_z + g.d_base),
            g.park_pose,
        ]
        q_list: list[JointVector] = []
        prev = q_prev
        for pose in poses:
            q = self.ik(pose, elbow="nearest", q_prev=prev)
            q_list.append(q)
            prev = q
        return q_list
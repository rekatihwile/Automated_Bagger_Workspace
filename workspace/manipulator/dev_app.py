"""Tkinter dev app for solving IK, queueing joint targets, and sending Teensy commands.

Run from the repository root:
    python -m workspace.manipulator.dev_app

This works without hardware in dry-run mode. Turn dry-run off once the Teensy is ready.
"""

from __future__ import annotations

from pathlib import Path
import sys
import tkinter as tk
from tkinter import messagebox, ttk
from math import degrees, radians
from typing import Iterable

if __package__ in (None, ""):
    # Support direct execution: python workspace/manipulator/dev_app.py
    repo_root = Path(__file__).resolve().parents[2]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))
    from workspace.manipulator.kinematics import (
        BaggerGeometry,
        BaggerKinematics,
        BaggerTransmission,
        JointVector,
        MotorSteps,
        Pose,
    )
    from workspace.manipulator.robot import (
        BaggerRobot,
        JointOutputConfig,
        SerialConfig,
        SerialProtocol,
    )
else:
    from .kinematics import BaggerGeometry, BaggerKinematics, BaggerTransmission, JointVector, MotorSteps, Pose
    from .robot import BaggerRobot, JointOutputConfig, SerialConfig, SerialProtocol


class BaggerDevApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Grocery Bagger IK + Teensy Dev App")
        self.geometry("1400x900")
        self.minsize(900, 600)

        self.kin = BaggerKinematics(BaggerGeometry(), BaggerTransmission())
        self.robot = BaggerRobot(dry_run=True)
        self.queue: list[JointVector] = []
        self.selected_index: int | None = None
        self._initializing = True  # Prevent config callbacks during init

        self._build_vars()
        self._build_ui()
        self._refresh_ports()
        
        self._initializing = False  # Now allow config changes
        # Force dry-run mode ON at startup
        self.dry_run_var.set(True)
        self.robot.dry_run = True
        
        self._log("Dry-run mode is ON. IK/transmission/UI can be tested without the Teensy.")
        self._log("Default send path: physical q -> coupled motor steps -> Q s1 s2 s3 s4")

    def _build_vars(self) -> None:
        g = self.kin.geometry
        t = self.kin.transmission

        # Target / joint vars
        self.x_var = tk.DoubleVar(value=g.workspace_center_y)
        self.y_var = tk.DoubleVar(value=0.0)
        self.z_var = tk.DoubleVar(value=g.d_base)
        self.item_height_var = tk.DoubleVar(value=g.nominal_item_height)
        self.elbow_var = tk.StringVar(value="nearest")

        self.q_vars = [tk.DoubleVar(value=0.0) for _ in range(4)]
        self.q_unit_var = tk.StringVar(value="deg")

        # Geometry tuning vars
        self.l1_var = tk.DoubleVar(value=g.l1)
        self.l2_var = tk.DoubleVar(value=g.l2)
        self.d_base_var = tk.DoubleVar(value=g.d_base)
        self.base_x_var = tk.DoubleVar(value=g.base_x)
        self.base_y_var = tk.DoubleVar(value=g.base_y)
        self.base_z_var = tk.DoubleVar(value=g.base_z)
        self.q3_min_var = tk.DoubleVar(value=g.q3_min)
        self.q3_max_var = tk.DoubleVar(value=g.q3_max)

        # Transmission tuning vars
        self.microsteps_var = tk.DoubleVar(value=t.microsteps)
        self.j1_ratio_var = tk.DoubleVar(value=t.j1_motor_rad_per_joint_rad)
        self.j2_elbow_ratio_var = tk.DoubleVar(value=t.j2_motor_rad_per_elbow_rad)
        self.j2_j1_ratio_var = tk.DoubleVar(value=t.j2_motor_rad_per_j1_rad)
        self.j3_spm_var = tk.DoubleVar(value=t.j3_steps_per_meter)
        self.j4_ratio_var = tk.DoubleVar(value=t.j4_motor_rad_per_joint_rad)
        self.j1_sign_var = tk.DoubleVar(value=t.j1_sign)
        self.j2_sign_var = tk.DoubleVar(value=t.j2_sign)
        self.j3_sign_var = tk.DoubleVar(value=t.j3_sign)
        self.j4_sign_var = tk.DoubleVar(value=t.j4_sign)
        self.j1_steps_deg_var = tk.DoubleVar(value=0.0)
        self.j2_elbow_steps_deg_var = tk.DoubleVar(value=0.0)
        self.j2_j1_steps_deg_var = tk.DoubleVar(value=0.0)
        self.j3_steps_mm_var = tk.DoubleVar(value=t.j3_steps_per_meter / 1000.0)
        self.j4_steps_deg_var = tk.DoubleVar(value=0.0)
        self._refresh_step_conversion_vars()

        # Direction test vars
        self.test_axis_var = tk.StringVar(value="x")
        self.test_delta_mm_var = tk.DoubleVar(value=25.0)

        # Cartesian-straight motion vars
        self.waypoints_var = tk.IntVar(value=20)
        self.wait_each_var = tk.BooleanVar(value=True)
        self.use_waypoints_var = tk.BooleanVar(value=True)

        # Single-motor diagnostic vars
        self.diag_joint_var = tk.IntVar(value=1)
        self.diag_delta_deg_var = tk.DoubleVar(value=10.0)
        self.singularity_var = tk.StringVar(value="")

        # Serial vars
        self.port_var = tk.StringVar(value="")
        self.baud_var = tk.IntVar(value=115200)
        self.dry_run_var = tk.BooleanVar(value=True)
        self.protocol_var = tk.StringVar(value=SerialProtocol.TEENSY_Q_STEPS.value)
        self.template_var = tk.StringVar(value="Q {s1} {s2} {s3} {s4}")
        self.scale_var = tk.StringVar(value="1,1,1,1")
        self.offset_var = tk.StringVar(value="0,0,0,0")

        self.servo_var = tk.IntVar(value=40)
        self.status_var = tk.StringVar(value="Disconnected | dry-run")
        self.fk_var = tk.StringVar(value="FK: --")
        self.manip_var = tk.StringVar(value="Manipulability: --")
        self.motor_var = tk.StringVar(value="Motor steps: --")

    def _build_ui(self) -> None:
        # Three-column layout: scrollable left, scrollbar, right (expands).
        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=0)
        self.columnconfigure(2, weight=1)
        self.rowconfigure(0, weight=1)

        # Scrollable left column. Width is fixed so the controls don't get squashed.
        LEFT_W = 360
        left_canvas = tk.Canvas(self, borderwidth=0, highlightthickness=0, width=LEFT_W)
        left_scroll = ttk.Scrollbar(self, orient="vertical", command=left_canvas.yview)
        left_canvas.configure(yscrollcommand=left_scroll.set)
        left_canvas.grid(row=0, column=0, sticky="ns")
        left_scroll.grid(row=0, column=1, sticky="ns")

        left = ttk.Frame(left_canvas, padding=12)
        left_window = left_canvas.create_window((0, 0), window=left, anchor="nw")

        # Keep the inner frame the same width as the canvas, and keep scrollregion fresh.
        def _on_left_configure(_event: object) -> None:
            left_canvas.configure(scrollregion=left_canvas.bbox("all"))
        left.bind("<Configure>", _on_left_configure)

        def _on_canvas_configure(event: tk.Event) -> None:
            left_canvas.itemconfigure(left_window, width=event.width)
        left_canvas.bind("<Configure>", _on_canvas_configure)

        # Mouse wheel scrolling, only active while the cursor is over the left column.
        def _on_mousewheel(event: tk.Event) -> None:
            # Windows/macOS: event.delta in multiples of 120. Linux uses Button-4/5.
            if event.num == 4:
                left_canvas.yview_scroll(-3, "units")
            elif event.num == 5:
                left_canvas.yview_scroll(3, "units")
            else:
                left_canvas.yview_scroll(int(-event.delta / 40), "units")

        def _bind_wheel(_event: object) -> None:
            left_canvas.bind_all("<MouseWheel>", _on_mousewheel)
            left_canvas.bind_all("<Button-4>", _on_mousewheel)
            left_canvas.bind_all("<Button-5>", _on_mousewheel)

        def _unbind_wheel(_event: object) -> None:
            left_canvas.unbind_all("<MouseWheel>")
            left_canvas.unbind_all("<Button-4>")
            left_canvas.unbind_all("<Button-5>")

        left_canvas.bind("<Enter>", _bind_wheel)
        left_canvas.bind("<Leave>", _unbind_wheel)
        left.bind("<Enter>", _bind_wheel)
        left.bind("<Leave>", _unbind_wheel)

        right = ttk.Frame(self, padding=12)
        right.grid(row=0, column=2, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)

        # Let each LabelFrame stretch to the column width.
        left.columnconfigure(0, weight=1)

        self._build_target_panel(left)
        self._build_tuning_panel(left)
        self._build_direction_test_panel(left)
        self._build_diagnostic_panel(left)
        self._build_serial_panel(left)
        self._build_queue_panel(right)
        self._build_log_panel(right)

    def _build_target_panel(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="IK Target / Joint Target", padding=10)
        box.grid(row=0, column=0, sticky="ew")
        box.columnconfigure(1, weight=1)

        for i, (label, var) in enumerate((("x [m]", self.x_var), ("y [m]", self.y_var), ("z [m]", self.z_var))):
            ttk.Label(box, text=label).grid(row=i, column=0, sticky="w")
            ttk.Entry(box, textvariable=var, width=12).grid(row=i, column=1, sticky="ew", padx=6, pady=2)

        ttk.Label(box, text="elbow").grid(row=3, column=0, sticky="w")
        ttk.Combobox(box, textvariable=self.elbow_var, values=["nearest", "down", "up"], width=10, state="readonly").grid(row=3, column=1, sticky="ew", padx=6, pady=2)

        ttk.Button(box, text="Solve IK → physical joints", command=self.solve_ik).grid(row=4, column=0, columnspan=2, sticky="ew", pady=(8, 2))
        ttk.Button(box, text="Add IK target to queue", command=self.add_ik_to_queue).grid(row=5, column=0, columnspan=2, sticky="ew", pady=2)

        ttk.Separator(box).grid(row=6, column=0, columnspan=2, sticky="ew", pady=10)

        ttk.Label(box, text="joint input units").grid(row=7, column=0, sticky="w")
        ttk.Combobox(box, textvariable=self.q_unit_var, values=["rad", "deg"], width=10, state="readonly").grid(row=7, column=1, sticky="ew", padx=6, pady=2)

        labels = ["q1", "q2", "q3 [m]", "q4"]
        for i, (label, var) in enumerate(zip(labels, self.q_vars), start=8):
            ttk.Label(box, text=label).grid(row=i, column=0, sticky="w")
            ttk.Entry(box, textvariable=var, width=12).grid(row=i, column=1, sticky="ew", padx=6, pady=2)

        ttk.Button(box, text="Add joint target to queue", command=self.add_joint_to_queue).grid(row=12, column=0, columnspan=2, sticky="ew", pady=(8, 2))
        ttk.Button(box, text="Add pick/place sequence", command=self.add_pick_place_sequence).grid(row=13, column=0, columnspan=2, sticky="ew", pady=2)
        ttk.Label(box, textvariable=self.fk_var).grid(row=14, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Label(box, textvariable=self.manip_var).grid(row=15, column=0, columnspan=2, sticky="w")
        ttk.Label(box, textvariable=self.motor_var).grid(row=16, column=0, columnspan=2, sticky="w")

    def _build_tuning_panel(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Live Geometry / Transmission Tuning", padding=10)
        box.grid(row=1, column=0, sticky="ew", pady=(12, 0))
        for c in (1, 3):
            box.columnconfigure(c, weight=1)

        fields = [
            ("L1", self.l1_var), ("L2", self.l2_var), ("d_base", self.d_base_var),
            ("base_x", self.base_x_var), ("base_y", self.base_y_var), ("base_z", self.base_z_var),
            ("q3_min", self.q3_min_var), ("q3_max", self.q3_max_var),
            ("microsteps", self.microsteps_var), ("J1 ratio", self.j1_ratio_var),
            ("J2 elbow", self.j2_elbow_ratio_var), ("J2 from J1", self.j2_j1_ratio_var),
            ("J3 steps/m", self.j3_spm_var), ("J4 ratio", self.j4_ratio_var),
            ("J1 sign", self.j1_sign_var), ("J2 sign", self.j2_sign_var),
            ("J3 sign", self.j3_sign_var), ("J4 sign", self.j4_sign_var),
        ]
        for i, (label, var) in enumerate(fields):
            r = i // 2
            c = (i % 2) * 2
            ttk.Label(box, text=label).grid(row=r, column=c, sticky="w")
            ttk.Entry(box, textvariable=var, width=9).grid(row=r, column=c + 1, sticky="ew", padx=4, pady=1)

        ttk.Button(box, text="Apply geometry/transmission", command=self.apply_tuning).grid(row=9, column=0, columnspan=4, sticky="ew", pady=(8, 0))

        ttk.Separator(box).grid(row=10, column=0, columnspan=4, sticky="ew", pady=10)
        conversions = [
            ("J1 steps/deg", self.j1_steps_deg_var), ("J2 elbow steps/deg", self.j2_elbow_steps_deg_var),
            ("J2 from J1 steps/deg", self.j2_j1_steps_deg_var), ("J3 steps/mm", self.j3_steps_mm_var),
            ("J4 steps/deg", self.j4_steps_deg_var),
        ]
        for i, (label, var) in enumerate(conversions, start=11):
            r = i
            c = 0 if (i - 11) % 2 == 0 else 2
            if c == 0 and i > 11:
                r = 11 + (i - 11) // 2
            else:
                r = 11 + (i - 11) // 2
            ttk.Label(box, text=label).grid(row=r, column=c, sticky="w")
            ttk.Entry(box, textvariable=var, width=9).grid(row=r, column=c + 1, sticky="ew", padx=4, pady=1)
        ttk.Button(box, text="Use step conversions", command=self.apply_step_conversions).grid(row=14, column=0, columnspan=4, sticky="ew", pady=(8, 0))

    def _build_direction_test_panel(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Direction Test (Cartesian-straight)", padding=10)
        box.grid(row=2, column=0, sticky="ew", pady=(12, 0))
        box.columnconfigure(1, weight=1)

        ttk.Label(box, text="axis").grid(row=0, column=0, sticky="w")
        ttk.Combobox(box, textvariable=self.test_axis_var, values=["x", "y", "z"], width=8, state="readonly").grid(row=0, column=1, sticky="ew", padx=6, pady=2)
        ttk.Label(box, text="delta [mm]").grid(row=1, column=0, sticky="w")
        ttk.Entry(box, textvariable=self.test_delta_mm_var, width=12).grid(row=1, column=1, sticky="ew", padx=6, pady=2)

        ttk.Label(box, text="waypoints").grid(row=2, column=0, sticky="w")
        ttk.Entry(box, textvariable=self.waypoints_var, width=12).grid(row=2, column=1, sticky="ew", padx=6, pady=2)
        ttk.Checkbutton(box, text="Stream as Cartesian waypoints", variable=self.use_waypoints_var).grid(row=3, column=0, columnspan=2, sticky="w")
        ttk.Checkbutton(box, text="Wait for DONE between waypoints", variable=self.wait_each_var).grid(row=4, column=0, columnspan=2, sticky="w")

        ttk.Label(box, textvariable=self.singularity_var, foreground="#b06000").grid(row=5, column=0, columnspan=2, sticky="w", pady=(4, 0))

        buttons = ttk.Frame(box)
        buttons.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        buttons.columnconfigure(0, weight=1)
        buttons.columnconfigure(1, weight=1)
        ttk.Button(buttons, text="Move -", command=lambda: self.direction_test(-1)).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(buttons, text="Move +", command=lambda: self.direction_test(1)).grid(row=0, column=1, sticky="ew")

    def _build_diagnostic_panel(self, parent: ttk.Frame) -> None:
        """Single-motor jog: bypasses kinematics entirely so the user can verify
        each row of the transmission against physical reality with a ruler."""
        box = ttk.LabelFrame(parent, text="Single-Motor Diagnostic", padding=10)
        box.grid(row=3, column=0, sticky="ew", pady=(12, 0))
        box.columnconfigure(1, weight=1)

        ttk.Label(box, text="joint").grid(row=0, column=0, sticky="w")
        ttk.Combobox(box, textvariable=self.diag_joint_var, values=[1, 2, 3, 4], width=8, state="readonly").grid(row=0, column=1, sticky="ew", padx=6, pady=2)
        ttk.Label(box, text="motor delta [deg]").grid(row=1, column=0, sticky="w")
        ttk.Entry(box, textvariable=self.diag_delta_deg_var, width=12).grid(row=1, column=1, sticky="ew", padx=6, pady=2)

        note = ttk.Label(
            box,
            text=(
                "Moves ONE motor by the raw amount you specify (no IK, no coupling math).\n"
                "Predicted EE delta is logged before sending. Measure with a ruler:\n"
                "  - If predicted ≈ measured → that motor's transmission row is correct.\n"
                "  - If they disagree → that row of BaggerTransmission is wrong\n"
                "    (sign, gear ratio, or for J2 the q1-coupling structure)."
            ),
            justify="left",
            wraplength=320,
        )
        note.grid(row=2, column=0, columnspan=2, sticky="w", pady=(4, 4))

        buttons = ttk.Frame(box)
        buttons.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        buttons.columnconfigure(0, weight=1)
        buttons.columnconfigure(1, weight=1)
        ttk.Button(buttons, text="Jog -", command=lambda: self.diagnostic_jog(-1)).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(buttons, text="Jog +", command=lambda: self.diagnostic_jog(1)).grid(row=0, column=1, sticky="ew")

    def _build_serial_panel(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Serial / Teensy Sender", padding=10)
        box.grid(row=4, column=0, sticky="ew", pady=(12, 0))
        box.columnconfigure(1, weight=1)

        ttk.Checkbutton(box, text="Dry run", variable=self.dry_run_var, command=self.apply_serial_config).grid(row=0, column=0, columnspan=2, sticky="w")

        ttk.Label(box, text="port").grid(row=1, column=0, sticky="w")
        port_row = ttk.Frame(box)
        port_row.grid(row=1, column=1, sticky="ew")
        port_row.columnconfigure(0, weight=1)
        self.port_combo = ttk.Combobox(port_row, textvariable=self.port_var, values=[], width=14)
        self.port_combo.grid(row=0, column=0, sticky="ew", padx=(6, 2))
        ttk.Button(port_row, text="Refresh", command=self._refresh_ports).grid(row=0, column=1)

        ttk.Label(box, text="baud").grid(row=2, column=0, sticky="w")
        ttk.Entry(box, textvariable=self.baud_var, width=12).grid(row=2, column=1, sticky="ew", padx=6, pady=2)

        ttk.Label(box, text="protocol").grid(row=3, column=0, sticky="w")
        ttk.Combobox(box, textvariable=self.protocol_var, values=[p.value for p in SerialProtocol], state="readonly", width=18).grid(row=3, column=1, sticky="ew", padx=6, pady=2)

        ttk.Label(box, text="template").grid(row=4, column=0, sticky="w")
        ttk.Entry(box, textvariable=self.template_var, width=24).grid(row=4, column=1, sticky="ew", padx=6, pady=2)

        ttk.Label(box, text="servo").grid(row=5, column=0, sticky="w")
        servo_row = ttk.Frame(box)
        servo_row.grid(row=5, column=1, sticky="ew")
        ttk.Entry(servo_row, textvariable=self.servo_var, width=8).grid(row=0, column=0, padx=(6, 2))
        ttk.Button(servo_row, text="Send", command=self.send_servo).grid(row=0, column=1)

        ttk.Button(box, text="Apply config", command=self.apply_serial_config).grid(row=6, column=0, columnspan=2, sticky="ew", pady=(8, 2))
        ttk.Button(box, text="Connect", command=self.connect_robot).grid(row=7, column=0, sticky="ew", padx=(0, 4), pady=2)
        ttk.Button(box, text="Disconnect", command=self.disconnect_robot).grid(row=7, column=1, sticky="ew", pady=2)
        ttk.Button(box, text="Enable", command=lambda: self.safe_robot_call(lambda: self.robot.enable(True))).grid(row=8, column=0, sticky="ew", padx=(0, 4), pady=2)
        ttk.Button(box, text="Disable", command=lambda: self.safe_robot_call(lambda: self.robot.enable(False))).grid(row=8, column=1, sticky="ew", pady=2)
        ttk.Button(box, text="ZERO ALL", command=lambda: self.safe_robot_call(self.robot.zero)).grid(row=9, column=0, sticky="ew", padx=(0, 4), pady=2)
        ttk.Button(box, text="Set current = shown q", command=self.set_current_from_joint_entries).grid(row=9, column=1, sticky="ew", pady=2)
        ttk.Button(box, text="POS", command=lambda: self.safe_robot_call(self.robot.request_position)).grid(row=10, column=0, sticky="ew", padx=(0, 4), pady=2)
        ttk.Button(box, text="STAT", command=lambda: self.safe_robot_call(self.robot.request_status)).grid(row=10, column=1, sticky="ew", pady=2)
        ttk.Button(box, text="STOP", command=lambda: self.safe_robot_call(self.robot.stop)).grid(row=11, column=0, columnspan=2, sticky="ew", pady=(6, 2))
        ttk.Label(box, textvariable=self.status_var).grid(row=12, column=0, columnspan=2, sticky="w", pady=(8, 0))

    def _build_queue_panel(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Queued Physical Joint Positions + Coupled Motor Steps", padding=10)
        box.grid(row=0, column=0, sticky="ew")
        box.columnconfigure(0, weight=1)

        columns = ("q1", "q2", "q3", "q4", "s1", "s2", "s3", "s4", "x", "y", "z")
        self.queue_tree = ttk.Treeview(box, columns=columns, show="headings", height=10)
        headers = {
            "q1": "q1 [deg]", "q2": "q2 [deg]", "q3": "q3 [m]", "q4": "q4 [deg]",
            "s1": "J1 steps", "s2": "J2 steps", "s3": "J3 steps", "s4": "J4 steps",
            "x": "x", "y": "y", "z": "z",
        }
        for col, label in headers.items():
            self.queue_tree.heading(col, text=label)
            self.queue_tree.column(col, width=86, anchor="center")
        self.queue_tree.grid(row=0, column=0, sticky="ew")
        self.queue_tree.bind("<<TreeviewSelect>>", self._on_select_queue)

        buttons = ttk.Frame(box)
        buttons.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        for i in range(6):
            buttons.columnconfigure(i, weight=1)
        ttk.Button(buttons, text="Send selected", command=self.send_selected).grid(row=0, column=0, sticky="ew", padx=2)
        ttk.Button(buttons, text="Send all", command=self.send_all).grid(row=0, column=1, sticky="ew", padx=2)
        ttk.Button(buttons, text="Move up", command=lambda: self.move_selected(-1)).grid(row=0, column=2, sticky="ew", padx=2)
        ttk.Button(buttons, text="Move down", command=lambda: self.move_selected(1)).grid(row=0, column=3, sticky="ew", padx=2)
        ttk.Button(buttons, text="Delete", command=self.delete_selected).grid(row=0, column=4, sticky="ew", padx=2)
        ttk.Button(buttons, text="Clear", command=self.clear_queue).grid(row=0, column=5, sticky="ew", padx=2)

    def _build_log_panel(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="TX/RX Log", padding=10)
        box.grid(row=1, column=0, sticky="nsew", pady=(12, 0))
        box.rowconfigure(0, weight=1)
        box.columnconfigure(0, weight=1)
        self.log = tk.Text(box, height=18, wrap="word")
        self.log.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(box, command=self.log.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=scroll.set)

    def _parse_four(self, text: str) -> tuple[float, float, float, float]:
        parts = [p.strip() for p in text.split(",")]
        if len(parts) != 4:
            raise ValueError("Expected four comma-separated numbers.")
        return tuple(float(p) for p in parts)  # type: ignore[return-value]

    def _steps_per_motor_rad_from_vars(self) -> float:
        return (200.0 * float(self.microsteps_var.get())) / (2.0 * 3.141592653589793)

    def _ratio_to_steps_per_degree(self, ratio: float) -> float:
        return float(ratio) * self._steps_per_motor_rad_from_vars() * radians(1.0)

    def _steps_per_degree_to_ratio(self, steps_per_degree: float) -> float:
        return float(steps_per_degree) / (self._steps_per_motor_rad_from_vars() * radians(1.0))

    def _refresh_step_conversion_vars(self) -> None:
        self.j1_steps_deg_var.set(round(self._ratio_to_steps_per_degree(self.j1_ratio_var.get()), 6))
        self.j2_elbow_steps_deg_var.set(round(self._ratio_to_steps_per_degree(self.j2_elbow_ratio_var.get()), 6))
        self.j2_j1_steps_deg_var.set(round(self._ratio_to_steps_per_degree(self.j2_j1_ratio_var.get()), 6))
        self.j3_steps_mm_var.set(round(float(self.j3_spm_var.get()) / 1000.0, 6))
        self.j4_steps_deg_var.set(round(self._ratio_to_steps_per_degree(self.j4_ratio_var.get()), 6))

    def apply_step_conversions(self) -> None:
        try:
            self.j1_ratio_var.set(self._steps_per_degree_to_ratio(self.j1_steps_deg_var.get()))
            self.j2_elbow_ratio_var.set(self._steps_per_degree_to_ratio(self.j2_elbow_steps_deg_var.get()))
            self.j2_j1_ratio_var.set(self._steps_per_degree_to_ratio(self.j2_j1_steps_deg_var.get()))
            self.j3_spm_var.set(float(self.j3_steps_mm_var.get()) * 1000.0)
            self.j4_ratio_var.set(self._steps_per_degree_to_ratio(self.j4_steps_deg_var.get()))
            self.apply_tuning()
            self._log("Applied step conversion fields to transmission ratios.")
        except Exception as exc:
            messagebox.showerror("Conversion error", str(exc))

    def apply_tuning(self) -> None:
        if self._initializing:
            return
        try:
            self.kin.update_geometry(
                l1=float(self.l1_var.get()),
                l2=float(self.l2_var.get()),
                d_base=float(self.d_base_var.get()),
                base_x=float(self.base_x_var.get()),
                base_y=float(self.base_y_var.get()),
                base_z=float(self.base_z_var.get()),
                q3_min=float(self.q3_min_var.get()),
                q3_max=float(self.q3_max_var.get()),
            )
            self.kin.update_transmission(
                microsteps=float(self.microsteps_var.get()),
                j1_motor_rad_per_joint_rad=float(self.j1_ratio_var.get()),
                j2_motor_rad_per_elbow_rad=float(self.j2_elbow_ratio_var.get()),
                j2_motor_rad_per_j1_rad=float(self.j2_j1_ratio_var.get()),
                j3_steps_per_meter=float(self.j3_spm_var.get()),
                j4_motor_rad_per_joint_rad=float(self.j4_ratio_var.get()),
                j1_sign=float(self.j1_sign_var.get()),
                j2_sign=float(self.j2_sign_var.get()),
                j3_sign=float(self.j3_sign_var.get()),
                j4_sign=float(self.j4_sign_var.get()),
            )
            self._refresh_step_conversion_vars()
            self.refresh_queue()
            try:
                self._update_fk_labels(self._get_joint_entries())
            except Exception:
                pass
            self._log("Applied geometry/transmission tuning.")
        except Exception as exc:
            messagebox.showerror("Tuning error", str(exc))

    def apply_serial_config(self) -> None:
        if self._initializing:
            return
        try:
            scale = self._parse_four(self.scale_var.get())
            offset = self._parse_four(self.offset_var.get())
            self.robot.config = SerialConfig(
                port=self.port_var.get().strip() or None,
                baudrate=int(self.baud_var.get()),
                protocol=SerialProtocol(self.protocol_var.get()),
                template=self.template_var.get(),
                output=JointOutputConfig(scale=scale, offset=offset),
            )
            self.robot.dry_run = bool(self.dry_run_var.get())
            self._update_status()
            self._log(f"Applied serial config: protocol={self.robot.config.protocol.value}, dry_run={self.robot.dry_run}")
        except Exception as exc:
            messagebox.showerror("Config error", str(exc))

    def _refresh_ports(self) -> None:
        ports = BaggerRobot.available_ports()
        self.port_combo.configure(values=ports)
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        if hasattr(self, "log"):
            if ports:
                self._log(f"Serial ports found: {', '.join(ports)}")
            else:
                self._log("No serial ports found. Check USB connection or pyserial install.")

    def connect_robot(self) -> None:
        self.apply_serial_config()
        self.safe_robot_call(lambda: self.robot.connect(self.port_var.get().strip() or None))
        self._update_status()

    def disconnect_robot(self) -> None:
        self.safe_robot_call(self.robot.disconnect)
        self._update_status()

    def _update_status(self) -> None:
        mode = "dry-run" if self.robot.dry_run else "hardware"
        state = "Connected" if self.robot.connected else "Disconnected"
        self.status_var.set(f"{state} | {mode}")

    def _ui_xyz_to_internal(self) -> Pose:
        return (self.y_var.get(), self.x_var.get(), self.z_var.get())

    @staticmethod
    def _internal_xyz_to_ui(x: float, y: float, z: float) -> Pose:
        return (y, x, z)

    def solve_ik(self) -> JointVector | None:
        try:
            self.apply_tuning()
            q_prev = self.queue[-1] if self.queue else None
            q = self.kin.ik(self._ui_xyz_to_internal(), elbow=self.elbow_var.get(), q_prev=q_prev)  # type: ignore[arg-type]
            self._set_joint_entries(q)
            self._update_fk_labels(q)
            return q
        except Exception as exc:
            messagebox.showerror("IK error", str(exc))
            return None

    def direction_test(self, sign: int) -> None:
        try:
            self.apply_tuning()
            axis = self.test_axis_var.get()
            delta_m = float(self.test_delta_mm_var.get()) / 1000.0 * float(sign)
            x, y, z = self.x_var.get(), self.y_var.get(), self.z_var.get()
            if axis == "x":
                x_new, y_new, z_new = x + delta_m, y, z
            elif axis == "y":
                x_new, y_new, z_new = x, y + delta_m, z
            elif axis == "z":
                x_new, y_new, z_new = x, y, z + delta_m
            else:
                raise ValueError(f"Unknown axis: {axis}")

            # UI x/y is swapped into kinematics y/x throughout the app.
            kin_start = (y, x, z)
            kin_end = (y_new, x_new, z_new)

            q_prev = self._get_joint_entries()

            if self.use_waypoints_var.get():
                n = max(1, int(self.waypoints_var.get()))
                waypoints = self.kin.cartesian_waypoints(kin_start, kin_end, n_segments=n, elbow="nearest", q_prev=q_prev)
                # Warn if any waypoint sits at a singularity (Jacobian inversion blows up there).
                bad = [i for i, q in enumerate(waypoints) if self.kin.is_near_singularity(q)]
                if bad:
                    self._log(f"WARNING: {len(bad)} waypoint(s) near elbow singularity (indices {bad[:5]}{'...' if len(bad) > 5 else ''}). Small motor error will cause large EE error there.")
                q_final = waypoints[-1]
                self.x_var.set(round(x_new, 6))
                self.y_var.set(round(y_new, 6))
                self.z_var.set(round(z_new, 6))
                self._set_joint_entries(q_final)
                self._update_fk_labels(q_final)
                self._send_cartesian_waypoints(waypoints)
                self._log(f"Direction test: {axis} {delta_m * 1000.0:+.1f} mm via {n} Cartesian waypoints.")
            else:
                q = self.kin.ik(kin_end, elbow="nearest", q_prev=q_prev)
                self.x_var.set(round(x_new, 6))
                self.y_var.set(round(y_new, 6))
                self.z_var.set(round(z_new, 6))
                self._set_joint_entries(q)
                self._update_fk_labels(q)
                self._send_q(q)
                self._log(f"Direction test: {axis} {delta_m * 1000.0:+.1f} mm in one shot (uncoordinated motor moves).")
        except Exception as exc:
            messagebox.showerror("Direction test error", str(exc))

    def _get_joint_entries(self) -> JointVector:
        q = tuple(v.get() for v in self.q_vars)
        if self.q_unit_var.get() == "deg":
            return (radians(q[0]), radians(q[1]), q[2], radians(q[3]))
        return q  # type: ignore[return-value]

    def _set_joint_entries(self, q: Iterable[float]) -> None:
        q1, q2, q3, q4 = tuple(float(v) for v in q)
        if self.q_unit_var.get() == "deg":
            vals = (degrees(q1), degrees(q2), q3, degrees(q4))
        else:
            vals = (q1, q2, q3, q4)
        for var, value in zip(self.q_vars, vals):
            var.set(round(value, 6))

    def _update_fk_labels(self, q: JointVector) -> None:
        x, y, z = self._internal_xyz_to_ui(*self.kin.fk(q))
        mu = self.kin.manipulability_xy(q)
        steps = self.kin.joint_to_motor_steps(q)
        self.fk_var.set(f"FK: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        self.manip_var.set(f"Manipulability: {mu:.4f}")
        self.motor_var.set(f"Motor steps: J1={steps[0]}, J2={steps[1]}, J3={steps[2]}, J4={steps[3]}")
        if self.kin.is_near_singularity(q):
            self.singularity_var.set("⚠ Near elbow singularity — small motor errors cause large EE errors here.")
        else:
            self.singularity_var.set("")

    def _send_cartesian_waypoints(self, waypoints: list[JointVector]) -> None:
        """Stream a list of joint targets to the Teensy as a coordinated Cartesian path."""
        def op() -> None:
            self.apply_tuning()
            self.apply_serial_config()
            step_seq: list[MotorSteps] = [self.kin.joint_to_motor_steps(q) for q in waypoints]
            lines = self.robot.send_motor_steps_waypoints(
                step_seq,
                wait_each=self.wait_each_var.get(),
                interval_s=0.02,
            )
            for line in lines:
                self._log(f"TX: {line}")
            for line in self.robot.read_all():
                self._log(f"RX: {line}")
        self.safe_robot_call(op)

    def diagnostic_jog(self, sign: int) -> None:
        """Move one motor by a known delta and log the predicted EE motion.

        Compare predicted EE delta to what the arm actually does. Mismatch tells you
        exactly which row of BaggerTransmission is wrong. This bypasses IK and the
        J2 coupling math entirely — it's a raw single-motor command via J{n}.
        """
        def op() -> None:
            self.apply_tuning()
            self.apply_serial_config()
            joint = int(self.diag_joint_var.get())
            delta_deg = float(self.diag_delta_deg_var.get()) * float(sign)

            q_now = self._get_joint_entries()

            if joint == 3:
                # J3 is linear: deg field is reused as mm for this joint.
                delta_motor_rad = delta_deg / 1000.0  # treat the field as mm
                # joint_to_motor_steps uses j3_steps_per_meter, not steps_per_motor_rad,
                # so build absolute step target manually.
                cur_steps = self.kin.joint_to_motor_steps(q_now)
                t = self.kin.transmission
                steps_delta = int(round(delta_motor_rad * t.j3_sign * t.j3_steps_per_meter))
                target_steps = cur_steps[2] + steps_delta
                pred = self.kin.predict_single_motor_jog(joint, delta_motor_rad, q_now)
                self._log(
                    f"DIAG J3 jog: +{delta_motor_rad * 1000.0:.2f} mm -> "
                    f"predicted ΔEE = ({pred['predicted_delta_ee'][0]*1000:+.1f}, "
                    f"{pred['predicted_delta_ee'][1]*1000:+.1f}, "
                    f"{pred['predicted_delta_ee'][2]*1000:+.1f}) mm. Target step: {target_steps}"
                )
                self.robot.move_single_motor(joint, target_steps)
            else:
                delta_motor_rad = radians(delta_deg)
                cur_steps = self.kin.joint_to_motor_steps(q_now)
                steps_delta = int(round(delta_motor_rad * self.kin.transmission.steps_per_motor_rad))
                target_steps = cur_steps[joint - 1] + steps_delta
                pred = self.kin.predict_single_motor_jog(joint, delta_motor_rad, q_now)
                dq = pred["predicted_dq"]
                dee = pred["predicted_delta_ee"]
                self._log(
                    f"DIAG J{joint} jog: motor {delta_deg:+.2f}° ({delta_motor_rad:+.4f} rad) -> "
                    f"predicted Δq = ({dq[0]:+.4f}, {dq[1]:+.4f}, {dq[2]:+.4f}, {dq[3]:+.4f}) rad -> "
                    f"predicted ΔEE = ({dee[0]*1000:+.1f}, {dee[1]*1000:+.1f}, {dee[2]*1000:+.1f}) mm. "
                    f"Target step: {target_steps}"
                )
                if pred["notes"]:
                    self._log(f"  Note: {pred['notes']}")
                self.robot.move_single_motor(joint, target_steps)
            for line in self.robot.read_all():
                self._log(f"RX: {line}")
        self.safe_robot_call(op)

    def add_ik_to_queue(self) -> None:
        q = self.solve_ik()
        if q is not None:
            self.queue.append(q)
            self.refresh_queue()

    def add_joint_to_queue(self) -> None:
        try:
            self.apply_tuning()
            q = self._get_joint_entries()
            self.kin.validate_joints(q)
            self.queue.append(q)
            self._update_fk_labels(q)
            self.refresh_queue()
        except Exception as exc:
            messagebox.showerror("Joint target error", str(exc))

    def add_pick_place_sequence(self) -> None:
        try:
            self.apply_tuning()
            x, y, _z = self._ui_xyz_to_internal()
            if not self.kin.geometry.in_staging_xy(x, y):
                if not messagebox.askyesno("Outside staging", "This point is outside the staging rectangle. Add sequence anyway?"):
                    return
            sequence = self.kin.make_pick_place_queue(x, y, self.item_height_var.get(), q_prev=self.queue[-1] if self.queue else None)
            self.queue.extend(sequence)
            self.refresh_queue()
            self._log(f"Added pick/place sequence with {len(sequence)} joint targets.")
        except Exception as exc:
            messagebox.showerror("Pick/place error", str(exc))

    def refresh_queue(self) -> None:
        self.queue_tree.delete(*self.queue_tree.get_children())
        for i, q in enumerate(self.queue):
            x, y, z = self._internal_xyz_to_ui(*self.kin.fk(q))
            steps = self.kin.joint_to_motor_steps(q)
            values = (degrees(q[0]), degrees(q[1]), q[2], degrees(q[3]), *steps, x, y, z)
            self.queue_tree.insert("", "end", iid=str(i), values=[f"{v:.4f}" if isinstance(v, float) else str(v) for v in values])
        if self.selected_index is not None and 0 <= self.selected_index < len(self.queue):
            iid = str(self.selected_index)
            self.queue_tree.selection_set(iid)
            self.queue_tree.focus(iid)

    def _on_select_queue(self, _event: object) -> None:
        selected = self.queue_tree.selection()
        if selected:
            self.selected_index = int(selected[0])
            q = self.queue[self.selected_index]
            self._set_joint_entries(q)
            self._update_fk_labels(q)

    def delete_selected(self) -> None:
        if self.selected_index is not None and 0 <= self.selected_index < len(self.queue):
            del self.queue[self.selected_index]
            self.selected_index = None
            self.refresh_queue()

    def clear_queue(self) -> None:
        self.queue.clear()
        self.selected_index = None
        self.refresh_queue()

    def move_selected(self, direction: int) -> None:
        if self.selected_index is None:
            return
        new_i = self.selected_index + direction
        if 0 <= new_i < len(self.queue):
            self.queue[self.selected_index], self.queue[new_i] = self.queue[new_i], self.queue[self.selected_index]
            self.selected_index = new_i
            self.refresh_queue()
            self.queue_tree.selection_set(str(new_i))

    def send_selected(self) -> None:
        selected = self.queue_tree.selection()
        if selected:
            self.selected_index = int(selected[0])
        if self.selected_index is None or not (0 <= self.selected_index < len(self.queue)):
            messagebox.showinfo("Nothing selected", "Select a queued target first.")
            return
        self.queue_tree.focus(str(self.selected_index))
        idx = self.selected_index
        if self.use_waypoints_var.get() and idx > 0:
            self._send_q_via_waypoints(self.queue[idx - 1], self.queue[idx])
        else:
            self._send_q(self.queue[idx])

    def send_all(self) -> None:
        if not self.queue:
            messagebox.showinfo("Queue empty", "Add at least one joint target first.")
            return
        if self.use_waypoints_var.get():
            prev_q = self._get_joint_entries()
            for q in self.queue:
                self._send_q_via_waypoints(prev_q, q)
                prev_q = q
        else:
            for q in self.queue:
                self._send_q(q)

    def _send_q_via_waypoints(self, q_start: JointVector, q_end: JointVector) -> None:
        """Interpolate from q_start to q_end in Cartesian space and stream waypoints."""
        try:
            self.apply_tuning()
            xyz_start = self.kin.fk(q_start)
            xyz_end = self.kin.fk(q_end)
            n = max(1, int(self.waypoints_var.get()))
            waypoints = self.kin.cartesian_waypoints(xyz_start, xyz_end, n_segments=n, elbow="nearest", q_prev=q_start)
            self._send_cartesian_waypoints(waypoints)
        except Exception as exc:
            messagebox.showerror("Send error", str(exc))
            self._log(f"ERROR: {exc}")

    def _send_q(self, q: JointVector) -> None:
        def op() -> None:
            self.apply_tuning()
            self.apply_serial_config()
            steps: MotorSteps = self.kin.joint_to_motor_steps(q)
            lines = self.robot.send_motor_steps(steps)
            for line in lines:
                self._log(f"TX: {line}")
            for line in self.robot.read_all():
                self._log(f"RX: {line}")
        self.safe_robot_call(op)

    def send_servo(self) -> None:
        def op() -> None:
            angle = int(self.servo_var.get())
            self.robot.set_servo(angle)
            self._log(f"TX: SERVO {angle}")
        self.safe_robot_call(op)

    def set_current_from_joint_entries(self) -> None:
        """Calibrate Teensy current step counters from the shown physical joint pose.

        This does not move the robot. It computes coupled motor steps from the
        current q-entry fields, including the J2 belt compensation, then sends:
            SETPOS s1 s2 s3 s4
        """
        def op() -> None:
            self.apply_tuning()
            q = self._get_joint_entries()
            self.kin.validate_joints(q)
            steps: MotorSteps = self.kin.joint_to_motor_steps(q)
            self.robot.set_current_steps(steps)
            self._update_fk_labels(q)
            self._log(f"TX: SETPOS {steps[0]} {steps[1]} {steps[2]} {steps[3]}")
            self._log("Set current Teensy counters from shown physical q, including J2 coupling.")
        self.safe_robot_call(op)

    def safe_robot_call(self, func) -> None:  # noqa: ANN001 - Tkinter callback helper
        try:
            func()
        except Exception as exc:
            messagebox.showerror("Robot error", str(exc))
            self._log(f"ERROR: {exc}")
        finally:
            self._update_status()

    def _log(self, msg: str) -> None:
        self.log.insert("end", msg + "\n")
        self.log.see("end")


def main() -> None:
    app = BaggerDevApp()
    app.mainloop()


if __name__ == "__main__":
    main()
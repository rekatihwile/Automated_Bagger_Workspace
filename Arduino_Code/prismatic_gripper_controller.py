import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import time

BAUD_RATE = 115200

MIN_GRIPPER_ANGLE = 5
MAX_GRIPPER_ANGLE = 75
MID_GRIPPER_ANGLE = 40

DEFAULT_JOG_STEPS = 5000


class RobotController:
    def __init__(self, root):
        self.root = root
        self.root.title("Cooper Wiring Robot Controller")

        self.ser = None

        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")

        self.desired1 = 0
        self.desired2 = 0
        self.current1 = 0
        self.current2 = 0

        self.desired1_var = tk.StringVar(value="0")
        self.desired2_var = tk.StringVar(value="0")
        self.current1_var = tk.StringVar(value="0")
        self.current2_var = tk.StringVar(value="0")

        self.jog_steps_var = tk.StringVar(value=str(DEFAULT_JOG_STEPS))
        self.absolute1_var = tk.StringVar(value="0")
        self.absolute2_var = tk.StringVar(value="0")

        self.gripper_angle_var = tk.IntVar(value=MIN_GRIPPER_ANGLE)
        self.manual_command_var = tk.StringVar()

        self.enabled = True
        self.last_sent_angle = None

        self.build_ui()
        self.refresh_ports()

    def build_ui(self):
        main = ttk.Frame(self.root, padding=16)
        main.grid(row=0, column=0, sticky="nsew")

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(0, weight=1)

        # ---------------- CONNECTION ----------------
        conn = ttk.LabelFrame(main, text="Arduino Connection", padding=12)
        conn.grid(row=0, column=0, sticky="ew", pady=(0, 12))

        ttk.Label(conn, text="Serial Port").grid(row=0, column=0, sticky="w")

        self.port_menu = ttk.Combobox(conn, textvariable=self.port_var, width=32, state="readonly")
        self.port_menu.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(2, 8))

        ttk.Button(conn, text="Refresh Ports", command=self.refresh_ports).grid(row=1, column=2, padx=(8, 0))
        ttk.Button(conn, text="Connect", command=self.connect).grid(row=2, column=0, sticky="ew")
        ttk.Button(conn, text="Disconnect", command=self.disconnect).grid(row=2, column=1, sticky="ew", padx=8)

        conn.columnconfigure(0, weight=1)
        conn.columnconfigure(1, weight=1)
        conn.columnconfigure(2, weight=1)

        # ---------------- GRIPPER ----------------
        grip = ttk.LabelFrame(main, text="End Effector / Gripper Servo", padding=12)
        grip.grid(row=1, column=0, sticky="ew", pady=(0, 12))

        ttk.Label(grip, text="Gripper Angle").grid(row=0, column=0, sticky="w")

        self.gripper_label = ttk.Label(grip, text=f"{MIN_GRIPPER_ANGLE}°", font=("Arial", 18))
        self.gripper_label.grid(row=0, column=2, sticky="e")

        self.gripper_slider = ttk.Scale(
            grip,
            from_=MIN_GRIPPER_ANGLE,
            to=MAX_GRIPPER_ANGLE,
            orient="horizontal",
            command=self.on_gripper_slider
        )
        self.gripper_slider.set(MIN_GRIPPER_ANGLE)
        self.gripper_slider.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(4, 12))

        ttk.Button(grip, text=f"Open / {MIN_GRIPPER_ANGLE}°", command=lambda: self.set_gripper(MIN_GRIPPER_ANGLE)).grid(row=2, column=0, sticky="ew")
        ttk.Button(grip, text=f"Half / {MID_GRIPPER_ANGLE}°", command=lambda: self.set_gripper(MID_GRIPPER_ANGLE)).grid(row=2, column=1, sticky="ew", padx=8)
        ttk.Button(grip, text=f"Close / {MAX_GRIPPER_ANGLE}°", command=lambda: self.set_gripper(MAX_GRIPPER_ANGLE)).grid(row=2, column=2, sticky="ew")

        grip.columnconfigure(0, weight=1)
        grip.columnconfigure(1, weight=1)
        grip.columnconfigure(2, weight=1)

        # ---------------- MOTORS ----------------
        motors = ttk.LabelFrame(main, text="Prismatic Joint Motors", padding=12)
        motors.grid(row=2, column=0, sticky="ew", pady=(0, 12))

        ttk.Label(motors, text="Motor 1 Desired").grid(row=0, column=0)
        ttk.Label(motors, text="Motor 2 Desired").grid(row=0, column=2)

        ttk.Label(motors, textvariable=self.desired1_var, font=("Arial", 14)).grid(row=1, column=0)
        ttk.Label(motors, textvariable=self.desired2_var, font=("Arial", 14)).grid(row=1, column=2)

        ttk.Label(motors, text="Motor 1 Current").grid(row=2, column=0)
        ttk.Label(motors, text="Motor 2 Current").grid(row=2, column=2)

        ttk.Label(motors, textvariable=self.current1_var, font=("Arial", 14)).grid(row=3, column=0)
        ttk.Label(motors, textvariable=self.current2_var, font=("Arial", 14)).grid(row=3, column=2)

        ttk.Separator(motors).grid(row=4, column=0, columnspan=3, sticky="ew", pady=12)

        ttk.Label(motors, text="Jog Step Size").grid(row=5, column=0, sticky="w")
        ttk.Entry(motors, textvariable=self.jog_steps_var).grid(row=6, column=0, columnspan=3, sticky="ew", pady=(2, 8))

        ttk.Button(motors, text="M1 -", command=lambda: self.jog_m1(-self.get_jog())).grid(row=7, column=0, sticky="ew")
        ttk.Button(motors, text="Both -", command=lambda: self.jog_both(-self.get_jog())).grid(row=7, column=1, sticky="ew", padx=8)
        ttk.Button(motors, text="M2 -", command=lambda: self.jog_m2(-self.get_jog())).grid(row=7, column=2, sticky="ew")

        ttk.Button(motors, text="M1 +", command=lambda: self.jog_m1(self.get_jog())).grid(row=8, column=0, sticky="ew", pady=(8, 0))
        ttk.Button(motors, text="Both +", command=lambda: self.jog_both(self.get_jog())).grid(row=8, column=1, sticky="ew", padx=8, pady=(8, 0))
        ttk.Button(motors, text="M2 +", command=lambda: self.jog_m2(self.get_jog())).grid(row=8, column=2, sticky="ew", pady=(8, 0))

        ttk.Separator(motors).grid(row=9, column=0, columnspan=3, sticky="ew", pady=12)

        ttk.Label(motors, text="M1 Absolute").grid(row=10, column=0, sticky="w")
        ttk.Label(motors, text="M2 Absolute").grid(row=10, column=2, sticky="w")

        ttk.Entry(motors, textvariable=self.absolute1_var).grid(row=11, column=0, sticky="ew")
        ttk.Button(motors, text="Go M1", command=self.go_m1_absolute).grid(row=11, column=1, sticky="ew", padx=8)
        ttk.Entry(motors, textvariable=self.absolute2_var).grid(row=11, column=2, sticky="ew")

        ttk.Button(motors, text="Go M2", command=self.go_m2_absolute).grid(row=12, column=2, sticky="ew", pady=(8, 0))
        ttk.Button(motors, text="Zero Both", command=self.zero_both).grid(row=12, column=0, sticky="ew", pady=(8, 0))
        ttk.Button(motors, text="Force Hold", command=self.force_hold).grid(row=12, column=1, sticky="ew", padx=8, pady=(8, 0))

        self.enable_btn = ttk.Button(motors, text="Disable Motors", command=self.toggle_enable)
        self.enable_btn.grid(row=13, column=0, columnspan=3, sticky="ew", pady=(12, 0))

        motors.columnconfigure(0, weight=1)
        motors.columnconfigure(1, weight=1)
        motors.columnconfigure(2, weight=1)

        # ---------------- MANUAL ----------------
        manual = ttk.LabelFrame(main, text="Manual Serial Command", padding=12)
        manual.grid(row=3, column=0, sticky="ew", pady=(0, 12))

        entry = ttk.Entry(manual, textvariable=self.manual_command_var)
        entry.grid(row=0, column=0, sticky="ew")
        entry.bind("<Return>", lambda event: self.send_manual())

        ttk.Button(manual, text="Send", command=self.send_manual).grid(row=0, column=1, padx=(8, 0))

        manual.columnconfigure(0, weight=1)

        ttk.Label(main, textvariable=self.status_var).grid(row=4, column=0, sticky="w")

    # ---------------- SERIAL ----------------

    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        names = [p.device for p in ports]
        self.port_menu["values"] = names
        if names and not self.port_var.get():
            self.port_var.set(names[0])
        if not names:
            self.status_var.set("No serial ports found")

    def connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("No port selected", "Select an Arduino serial port first.")
            return

        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
            time.sleep(2)
            self.clear_serial()
            self.status_var.set(f"Connected to {port}")

            self.send("C")
            self.send("H")
            self.send(f"A{self.gripper_angle_var.get()}")

            self.update_position_loop()

        except Exception as e:
            self.ser = None
            messagebox.showerror("Connection failed", str(e))
            self.status_var.set("Disconnected")

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.status_var.set("Disconnected")

    def send(self, cmd):
        if not self.ser or not self.ser.is_open:
            self.status_var.set("Not connected")
            return

        self.ser.write((cmd.strip() + "\n").encode())
        self.ser.flush()
        self.status_var.set(f"Sent: {cmd}")

    def clear_serial(self):
        if self.ser and self.ser.is_open:
            while self.ser.in_waiting:
                self.ser.readline()

    # ---------------- GRIPPER ----------------

    def on_gripper_slider(self, value):
        angle = int(round(float(value)))
        self.gripper_angle_var.set(angle)
        self.gripper_label.config(text=f"{angle}°")

        if angle != self.last_sent_angle:
            self.send(f"A{angle}")
            self.last_sent_angle = angle

    def set_gripper(self, angle):
        angle = max(MIN_GRIPPER_ANGLE, min(MAX_GRIPPER_ANGLE, int(angle)))
        self.gripper_slider.set(angle)
        self.gripper_angle_var.set(angle)
        self.gripper_label.config(text=f"{angle}°")
        self.send(f"A{angle}")
        self.last_sent_angle = angle

    # ---------------- MOTORS ----------------

    def get_jog(self):
        try:
            return int(self.jog_steps_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid jog", "Jog step size must be an integer.")
            return 0

    def jog_m1(self, delta):
        if delta == 0:
            return
        self.desired1 += delta
        self.desired1_var.set(str(self.desired1))
        self.send(f"M1 {self.desired1}")

    def jog_m2(self, delta):
        if delta == 0:
            return
        self.desired2 += delta
        self.desired2_var.set(str(self.desired2))
        self.send(f"M2 {self.desired2}")

    def jog_both(self, delta):
        if delta == 0:
            return
        self.desired1 += delta
        self.desired2 += delta
        self.desired1_var.set(str(self.desired1))
        self.desired2_var.set(str(self.desired2))
        self.send(f"M1 {self.desired1}")
        self.send(f"M2 {self.desired2}")

    def go_m1_absolute(self):
        try:
            pos = int(self.absolute1_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid position", "M1 absolute position must be an integer.")
            return

        self.desired1 = pos
        self.desired1_var.set(str(pos))
        self.send(f"M1 {pos}")

    def go_m2_absolute(self):
        try:
            pos = int(self.absolute2_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid position", "M2 absolute position must be an integer.")
            return

        self.desired2 = pos
        self.desired2_var.set(str(pos))
        self.send(f"M2 {pos}")

    def zero_both(self):
        self.desired1 = 0
        self.desired2 = 0
        self.desired1_var.set("0")
        self.desired2_var.set("0")
        self.send("Z")

    def force_hold(self):
        self.send("H")

    def toggle_enable(self):
        self.enabled = not self.enabled

        if self.enabled:
            self.send("EN 1")
            self.enable_btn.config(text="Disable Motors")
        else:
            self.send("EN 0")
            self.enable_btn.config(text="Enable Motors")

    def update_position_loop(self):
        if not self.ser or not self.ser.is_open:
            return

        self.send("POS")
        time.sleep(0.02)

        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors="ignore").strip()

            if line.startswith("P1"):
                try:
                    self.current1 = int(line.split()[1])
                    self.current1_var.set(str(self.current1))
                except Exception:
                    pass

            elif line.startswith("P2"):
                try:
                    self.current2 = int(line.split()[1])
                    self.current2_var.set(str(self.current2))
                except Exception:
                    pass

        self.root.after(150, self.update_position_loop)

    # ---------------- MANUAL ----------------

    def send_manual(self):
        cmd = self.manual_command_var.get().strip()
        if not cmd:
            return
        self.send(cmd)
        self.manual_command_var.set("")

    def on_close(self):
        # I am intentionally NOT sending EN 0 here,
        # because your vertical/prismatic joint needs holding torque.
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotController(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
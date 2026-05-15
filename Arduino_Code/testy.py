import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import time

# -------------------- SERIAL SETTINGS --------------------
BAUD_RATE = 9600

# -------------------- SERVO SETTINGS --------------------
MIN_ANGLE = 5
MAX_ANGLE = 75
MID_ANGLE = 40

# -------------------- STEPPER SETTINGS --------------------
DEFAULT_RELATIVE_STEPS = 200
DEFAULT_ABSOLUTE_POSITION = 0


class StepperServoControllerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Stepper + Servo Controller")

        self.ser = None
        self.last_sent_angle = None

        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")

        self.angle_var = tk.IntVar(value=MIN_ANGLE)
        self.relative_steps_var = tk.StringVar(value=str(DEFAULT_RELATIVE_STEPS))
        self.absolute_position_var = tk.StringVar(value=str(DEFAULT_ABSOLUTE_POSITION))
        self.manual_command_var = tk.StringVar()

        self.build_ui()
        self.refresh_ports()

    # -------------------- UI --------------------

    def build_ui(self):
        main = ttk.Frame(self.root, padding=16)
        main.grid(row=0, column=0, sticky="nsew")

        # ---------- Serial connection ----------
        serial_frame = ttk.LabelFrame(main, text="Serial Connection", padding=12)
        serial_frame.grid(row=0, column=0, sticky="ew", pady=(0, 12))

        ttk.Label(serial_frame, text="Arduino Port").grid(row=0, column=0, sticky="w")

        self.port_menu = ttk.Combobox(
            serial_frame,
            textvariable=self.port_var,
            width=28,
            state="readonly"
        )
        self.port_menu.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(2, 8))

        ttk.Button(serial_frame, text="Refresh", command=self.refresh_ports).grid(
            row=1, column=2, padx=(8, 0)
        )

        ttk.Button(serial_frame, text="Connect", command=self.connect).grid(
            row=2, column=0, sticky="ew", pady=(0, 4)
        )

        ttk.Button(serial_frame, text="Disconnect", command=self.disconnect).grid(
            row=2, column=1, sticky="ew", padx=8, pady=(0, 4)
        )

        serial_frame.columnconfigure(0, weight=1)
        serial_frame.columnconfigure(1, weight=1)
        serial_frame.columnconfigure(2, weight=1)

        # ---------- Servo ----------
        servo_frame = ttk.LabelFrame(main, text="Servo Gripper", padding=12)
        servo_frame.grid(row=1, column=0, sticky="ew", pady=(0, 12))

        ttk.Label(servo_frame, text="Gripper Angle").grid(row=0, column=0, sticky="w")

        self.angle_label = ttk.Label(servo_frame, text=f"{MIN_ANGLE}°", font=("Arial", 18))
        self.angle_label.grid(row=0, column=2, sticky="e")

        self.slider = ttk.Scale(
            servo_frame,
            from_=MIN_ANGLE,
            to=MAX_ANGLE,
            orient="horizontal",
            command=self.on_slider_move,
        )
        self.slider.set(MIN_ANGLE)
        self.slider.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(4, 12))

        ttk.Button(
            servo_frame,
            text=f"Open / {MIN_ANGLE}°",
            command=lambda: self.set_angle(MIN_ANGLE)
        ).grid(row=2, column=0, sticky="ew")

        ttk.Button(
            servo_frame,
            text=f"Mid / {MID_ANGLE}°",
            command=lambda: self.set_angle(MID_ANGLE)
        ).grid(row=2, column=1, sticky="ew", padx=8)

        ttk.Button(
            servo_frame,
            text=f"Close / {MAX_ANGLE}°",
            command=lambda: self.set_angle(MAX_ANGLE)
        ).grid(row=2, column=2, sticky="ew")

        servo_frame.columnconfigure(0, weight=1)
        servo_frame.columnconfigure(1, weight=1)
        servo_frame.columnconfigure(2, weight=1)

        # ---------- Stepper ----------
        stepper_frame = ttk.LabelFrame(main, text="Stepper Motor", padding=12)
        stepper_frame.grid(row=2, column=0, sticky="ew", pady=(0, 12))

        ttk.Button(
            stepper_frame,
            text="Enable Stepper",
            command=self.enable_stepper
        ).grid(row=0, column=0, sticky="ew")

        ttk.Button(
            stepper_frame,
            text="Disable Stepper",
            command=self.disable_stepper
        ).grid(row=0, column=1, sticky="ew", padx=8)

        ttk.Button(
            stepper_frame,
            text="Zero Position",
            command=self.zero_stepper
        ).grid(row=0, column=2, sticky="ew")

        ttk.Separator(stepper_frame).grid(
            row=1, column=0, columnspan=3, sticky="ew", pady=12
        )

        ttk.Label(stepper_frame, text="Relative Steps").grid(row=2, column=0, sticky="w")

        relative_entry = ttk.Entry(stepper_frame, textvariable=self.relative_steps_var)
        relative_entry.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(2, 8))

        ttk.Button(
            stepper_frame,
            text="Move -Steps",
            command=self.move_relative_negative
        ).grid(row=4, column=0, sticky="ew")

        ttk.Button(
            stepper_frame,
            text="Move +Steps",
            command=self.move_relative_positive
        ).grid(row=4, column=1, sticky="ew", padx=8)

        ttk.Button(
            stepper_frame,
            text="Send Relative",
            command=self.move_relative_from_entry
        ).grid(row=4, column=2, sticky="ew")

        ttk.Separator(stepper_frame).grid(
            row=5, column=0, columnspan=3, sticky="ew", pady=12
        )

        ttk.Label(stepper_frame, text="Absolute Step Position").grid(row=6, column=0, sticky="w")

        absolute_entry = ttk.Entry(stepper_frame, textvariable=self.absolute_position_var)
        absolute_entry.grid(row=7, column=0, columnspan=2, sticky="ew", pady=(2, 8))

        ttk.Button(
            stepper_frame,
            text="Go To Position",
            command=self.move_absolute_from_entry
        ).grid(row=7, column=2, sticky="ew", padx=(8, 0), pady=(2, 8))

        stepper_frame.columnconfigure(0, weight=1)
        stepper_frame.columnconfigure(1, weight=1)
        stepper_frame.columnconfigure(2, weight=1)

        # ---------- Manual command ----------
        manual_frame = ttk.LabelFrame(main, text="Manual Serial Command", padding=12)
        manual_frame.grid(row=3, column=0, sticky="ew", pady=(0, 12))

        manual_entry = ttk.Entry(manual_frame, textvariable=self.manual_command_var)
        manual_entry.grid(row=0, column=0, sticky="ew")

        ttk.Button(
            manual_frame,
            text="Send",
            command=self.send_manual_command
        ).grid(row=0, column=1, sticky="ew", padx=(8, 0))

        manual_entry.bind("<Return>", lambda event: self.send_manual_command())

        manual_frame.columnconfigure(0, weight=1)

        # ---------- Status ----------
        status_frame = ttk.Frame(main)
        status_frame.grid(row=4, column=0, sticky="ew")

        ttk.Label(status_frame, textvariable=self.status_var).grid(
            row=0, column=0, sticky="w"
        )

        main.columnconfigure(0, weight=1)

    # -------------------- SERIAL --------------------

    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        port_names = [p.device for p in ports]

        self.port_menu["values"] = port_names

        if port_names and not self.port_var.get():
            self.port_var.set(port_names[0])

        if not port_names:
            self.status_var.set("No serial ports found")

    def connect(self):
        port = self.port_var.get()

        if not port:
            messagebox.showerror("No port selected", "Select an Arduino serial port first.")
            return

        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=0.2)
            time.sleep(2.0)  # Arduino usually resets when serial opens

            self.status_var.set(f"Connected to {port}")

            # Read any startup text from Arduino
            self.read_available_lines()

            # Send current servo angle after connection
            self.send_angle(self.angle_var.get())

        except Exception as e:
            self.ser = None
            messagebox.showerror("Connection failed", str(e))
            self.status_var.set("Disconnected")

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

        self.ser = None
        self.status_var.set("Disconnected")

    def send_command(self, command):
        if not self.ser or not self.ser.is_open:
            self.status_var.set("Not connected")
            return None

        try:
            full_command = command.strip() + "\n"
            self.ser.write(full_command.encode("utf-8"))
            self.ser.flush()

            time.sleep(0.05)

            response = self.read_available_lines()

            if response:
                self.status_var.set(f"Sent: {command} | Arduino: {response[-1]}")
                return response[-1]
            else:
                self.status_var.set(f"Sent: {command}")
                return None

        except Exception as e:
            self.status_var.set(f"Serial error: {e}")
            return None

    def read_available_lines(self):
        if not self.ser or not self.ser.is_open:
            return []

        lines = []

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    lines.append(line)
        except Exception:
            pass

        return lines

    # -------------------- SERVO --------------------

    def on_slider_move(self, value):
        angle = int(round(float(value)))
        self.angle_var.set(angle)
        self.angle_label.config(text=f"{angle}°")

        if angle != self.last_sent_angle:
            self.send_angle(angle)

    def set_angle(self, angle):
        angle = self.clamp_angle(angle)

        self.slider.set(angle)
        self.angle_var.set(angle)
        self.angle_label.config(text=f"{angle}°")

        self.send_angle(angle)

    def send_angle(self, angle):
        angle = self.clamp_angle(angle)
        command = f"A{angle}"

        response = self.send_command(command)

        if response is not None or self.ser:
            self.last_sent_angle = angle

    def clamp_angle(self, angle):
        return max(MIN_ANGLE, min(MAX_ANGLE, int(angle)))

    # -------------------- STEPPER --------------------

    def enable_stepper(self):
        self.send_command("E1")

    def disable_stepper(self):
        self.send_command("E0")

    def zero_stepper(self):
        self.send_command("Z")

    def move_relative_positive(self):
        steps = self.get_relative_steps()
        if steps is None:
            return

        self.send_command(f"S{abs(steps)}")

    def move_relative_negative(self):
        steps = self.get_relative_steps()
        if steps is None:
            return

        self.send_command(f"S{-abs(steps)}")

    def move_relative_from_entry(self):
        steps = self.get_relative_steps()
        if steps is None:
            return

        self.send_command(f"S{steps}")

    def move_absolute_from_entry(self):
        position = self.get_absolute_position()
        if position is None:
            return

        self.send_command(f"G{position}")

    def get_relative_steps(self):
        value = self.relative_steps_var.get().strip()

        try:
            steps = int(value)
            return steps
        except ValueError:
            messagebox.showerror("Invalid steps", "Relative steps must be an integer, like 200 or -200.")
            return None

    def get_absolute_position(self):
        value = self.absolute_position_var.get().strip()

        try:
            position = int(value)
            return position
        except ValueError:
            messagebox.showerror("Invalid position", "Absolute position must be an integer, like 0, 500, or -200.")
            return None

    # -------------------- MANUAL COMMAND --------------------

    def send_manual_command(self):
        command = self.manual_command_var.get().strip()

        if not command:
            return

        self.send_command(command)
        self.manual_command_var.set("")

    # -------------------- CLOSE --------------------

    def on_close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = StepperServoControllerApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
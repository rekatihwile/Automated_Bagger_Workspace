import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import time

BAUD_RATE = 9600
MIN_ANGLE = 0
MAX_ANGLE = 80


class ServoSliderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Servo Gripper Controller")

        self.ser = None
        self.last_sent_angle = None

        self.port_var = tk.StringVar()
        self.angle_var = tk.IntVar(value=MIN_ANGLE)
        self.status_var = tk.StringVar(value="Disconnected")

        self.build_ui()
        self.refresh_ports()

    def build_ui(self):
        frame = ttk.Frame(self.root, padding=16)
        frame.grid(row=0, column=0, sticky="nsew")

        ttk.Label(frame, text="Arduino Port").grid(row=0, column=0, sticky="w")

        self.port_menu = ttk.Combobox(frame, textvariable=self.port_var, width=28, state="readonly")
        self.port_menu.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(2, 8))

        ttk.Button(frame, text="Refresh", command=self.refresh_ports).grid(row=1, column=2, padx=(8, 0))
        ttk.Button(frame, text="Connect", command=self.connect).grid(row=2, column=0, pady=(0, 12), sticky="ew")
        ttk.Button(frame, text="Disconnect", command=self.disconnect).grid(row=2, column=1, pady=(0, 12), sticky="ew")

        ttk.Label(frame, text="Gripper Angle").grid(row=3, column=0, sticky="w")

        self.angle_label = ttk.Label(frame, text=f"{MIN_ANGLE}°", font=("Arial", 18))
        self.angle_label.grid(row=3, column=2, sticky="e")

        self.slider = ttk.Scale(
            frame,
            from_=MIN_ANGLE,
            to=MAX_ANGLE,
            orient="horizontal",
            command=self.on_slider_move,
        )
        self.slider.set(MIN_ANGLE)
        self.slider.grid(row=4, column=0, columnspan=3, sticky="ew", pady=(4, 12))

        ttk.Button(frame, text="Open / 5°", command=lambda: self.set_angle(MIN_ANGLE)).grid(row=5, column=0, sticky="ew")
        ttk.Button(frame, text="Mid / 40°", command=lambda: self.set_angle(40)).grid(row=5, column=1, sticky="ew", padx=8)
        ttk.Button(frame, text="Close / 75°", command=lambda: self.set_angle(MAX_ANGLE)).grid(row=5, column=2, sticky="ew")

        ttk.Label(frame, textvariable=self.status_var).grid(row=6, column=0, columnspan=3, sticky="w", pady=(12, 0))

        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=1)

    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        port_names = [p.device for p in ports]

        self.port_menu["values"] = port_names

        if port_names and not self.port_var.get():
            self.port_var.set(port_names[0])

    def connect(self):
        port = self.port_var.get()

        if not port:
            messagebox.showerror("No port selected", "Select an Arduino serial port first.")
            return

        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
            time.sleep(2.0)  # Arduino resets when serial opens
            self.status_var.set(f"Connected to {port}")
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

    def on_slider_move(self, value):
        angle = int(round(float(value)))
        self.angle_var.set(angle)
        self.angle_label.config(text=f"{angle}°")

        # Only send if angle actually changed
        if angle != self.last_sent_angle:
            self.send_angle(angle)

    def set_angle(self, angle):
        angle = max(MIN_ANGLE, min(MAX_ANGLE, int(angle)))
        self.slider.set(angle)
        self.angle_var.set(angle)
        self.angle_label.config(text=f"{angle}°")
        self.send_angle(angle)

    def send_angle(self, angle):
        if not self.ser or not self.ser.is_open:
            return

        angle = max(MIN_ANGLE, min(MAX_ANGLE, int(angle)))

        try:
            command = f"A{angle}\n"
            self.ser.write(command.encode("utf-8"))
            self.last_sent_angle = angle
            self.status_var.set(f"Sent angle: {angle}°")

            response = self.ser.readline().decode("utf-8", errors="ignore").strip()
            if response:
                self.status_var.set(f"Sent {angle}° | Arduino: {response}")

        except Exception as e:
            self.status_var.set(f"Serial error: {e}")

    def on_close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ServoSliderApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
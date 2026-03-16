import serial
import time
import tkinter as tk

# ---------- SERIAL SETUP ----------

arduino = serial.Serial('COM3',115200)
time.sleep(2)

def send(cmd):
    arduino.write((cmd + "\n").encode())

# ---------- STATE VARIABLES ----------

desired1 = 0
desired2 = 0

current1 = 0
current2 = 0

step_size = 5000
enabled = True

# ---------- GUI ----------

root = tk.Tk()
root.title("Robot Motor Controller")

# Desired position labels
desired1_var = tk.StringVar(value="0")
desired2_var = tk.StringVar(value="0")

# Current position labels
current1_var = tk.StringVar(value="0")
current2_var = tk.StringVar(value="0")

# ---------- FUNCTIONS ----------

def jog_m1(pos_delta):
    global desired1
    desired1 += pos_delta
    send(f"M1 {desired1}")
    desired1_var.set(str(desired1))

def jog_m2(pos_delta):
    global desired2
    desired2 += pos_delta
    send(f"M2 {desired2}")
    desired2_var.set(str(desired2))

def toggle_enable():
    global enabled
    enabled = not enabled
    send(f"EN {1 if enabled else 0}")
    enable_btn.config(text="Disable Motors" if enabled else "Enable Motors")

def quit_program():
    send("EN 0")
    root.destroy()

def update_position():
    global current1, current2

    send("POS")

    time.sleep(0.02)

    while arduino.in_waiting:
        line = arduino.readline().decode().strip()

        if line.startswith("P1"):
            current1 = int(line.split()[1])
            current1_var.set(str(current1))

        if line.startswith("P2"):
            current2 = int(line.split()[1])
            current2_var.set(str(current2))

    root.after(100, update_position)

# ---------- LAYOUT ----------

tk.Label(root,text="Motor 1").grid(row=0,column=0)
tk.Label(root,text="Motor 2").grid(row=0,column=2)

tk.Label(root,text="Desired").grid(row=1,column=0)
tk.Label(root,text="Desired").grid(row=1,column=2)

tk.Label(root,textvariable=desired1_var).grid(row=2,column=0)
tk.Label(root,textvariable=desired2_var).grid(row=2,column=2)

tk.Label(root,text="Current").grid(row=3,column=0)
tk.Label(root,text="Current").grid(row=3,column=2)

tk.Label(root,textvariable=current1_var).grid(row=4,column=0)
tk.Label(root,textvariable=current2_var).grid(row=4,column=2)

# Jog buttons
tk.Button(root,text="M1 +",width=10,command=lambda: jog_m1(step_size)).grid(row=5,column=0)
tk.Button(root,text="M1 -",width=10,command=lambda: jog_m1(-step_size)).grid(row=6,column=0)

tk.Button(root,text="M2 +",width=10,command=lambda: jog_m2(step_size)).grid(row=5,column=2)
tk.Button(root,text="M2 -",width=10,command=lambda: jog_m2(-step_size)).grid(row=6,column=2)

# Enable button
enable_btn = tk.Button(root,text="Disable Motors",width=15,command=toggle_enable)
enable_btn.grid(row=7,column=0,columnspan=3,pady=10)

# Quit button
quit_btn = tk.Button(root,text="Quit",width=15,command=quit_program)
quit_btn.grid(row=8,column=0,columnspan=3,pady=10)

# ---------- START LOOP ----------

update_position()

root.mainloop()
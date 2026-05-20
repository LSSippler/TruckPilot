"""
vJoy Axis Tester v3 - mit Auto-Wackel fuer ETS2 Achsen-Zuweisung

X-Achse fuer Steering (bipolar)
Slider fuer Throttle (unipolar)
Dial/Slider2 fuer Brake (unipolar)

Voraussetzung:
- vJoy Device 1 aktiv mit X, Slider, Dial/Slider2 Axes (in vJoyConf aktivieren)
- vJoyInterface.dll im PATH oder im Standard-Pfad

Auto-Wackel:
- Pro Achse ein "Wackeln"-Button
- Schickt fuer 8 Sekunden eine Sinus-Bewegung auf der Achse
- ETS2 erkennt waehrenddessen die Achse und kann zugewiesen werden
"""

import ctypes
import math
import os
import sys
import time
import tkinter as tk
from tkinter import ttk

# vJoy axis constants from vjoyinterface.h
HID_USAGE_X = 0x30
HID_USAGE_Y = 0x31
HID_USAGE_Z = 0x32
HID_USAGE_RX = 0x33
HID_USAGE_RY = 0x34
HID_USAGE_RZ = 0x35
HID_USAGE_SL0 = 0x36  # Slider
HID_USAGE_SL1 = 0x37  # Dial/Slider2

VJOY_DEVICE_ID = 1
AXIS_MIN = 0
AXIS_MAX = 32767
AXIS_CENTER = 16384

WIGGLE_DURATION_SEC = 8.0
WIGGLE_FREQ_HZ = 1.5  # 1.5 Schwingungen pro Sekunde
WIGGLE_TICK_MS = 20    # 50 Hz Update-Rate

DLL_CANDIDATES = [
    r"C:\Program Files\vJoy\x64\vJoyInterface.dll",
    r"C:\Program Files (x86)\vJoy\x64\vJoyInterface.dll",
    "vJoyInterface.dll",
]


def load_vjoy():
    for path in DLL_CANDIDATES:
        if os.path.exists(path):
            return ctypes.WinDLL(path)
    return ctypes.WinDLL("vJoyInterface.dll")


def main():
    print("Loading vJoy DLL...")
    try:
        vjoy = load_vjoy()
    except OSError as e:
        print(f"FAIL: Could not load vJoyInterface.dll: {e}")
        sys.exit(1)

    vjoy.vJoyEnabled.restype = ctypes.c_bool
    vjoy.AcquireVJD.argtypes = [ctypes.c_uint]
    vjoy.AcquireVJD.restype = ctypes.c_bool
    vjoy.RelinquishVJD.argtypes = [ctypes.c_uint]
    vjoy.RelinquishVJD.restype = None
    vjoy.SetAxis.argtypes = [ctypes.c_long, ctypes.c_uint, ctypes.c_uint]
    vjoy.SetAxis.restype = ctypes.c_bool
    vjoy.ResetVJD.argtypes = [ctypes.c_uint]
    vjoy.ResetVJD.restype = ctypes.c_bool
    vjoy.GetVJDAxisExist.argtypes = [ctypes.c_uint, ctypes.c_uint]
    vjoy.GetVJDAxisExist.restype = ctypes.c_bool

    if not vjoy.vJoyEnabled():
        print("FAIL: vJoy driver not enabled. Run vJoyConf and enable Device 1.")
        sys.exit(1)

    if not vjoy.AcquireVJD(VJOY_DEVICE_ID):
        print(f"FAIL: Could not acquire vJoy Device {VJOY_DEVICE_ID}. Already in use?")
        sys.exit(1)

    print(f"Acquired vJoy Device {VJOY_DEVICE_ID}")

    x_exists = vjoy.GetVJDAxisExist(VJOY_DEVICE_ID, HID_USAGE_X)
    sl0_exists = vjoy.GetVJDAxisExist(VJOY_DEVICE_ID, HID_USAGE_SL0)
    sl1_exists = vjoy.GetVJDAxisExist(VJOY_DEVICE_ID, HID_USAGE_SL1)

    print(f"X (Steering):       {'OK' if x_exists else 'MISSING'}")
    print(f"Slider (Throttle):  {'OK' if sl0_exists else 'MISSING - aktiviere in vJoyConf'}")
    print(f"Dial/Slider2 (Brake): {'OK' if sl1_exists else 'MISSING - aktiviere in vJoyConf'}")

    if not (x_exists and sl0_exists and sl1_exists):
        print("\nFAIL: Aktiviere die fehlenden Achsen in vJoyConf.exe, Apply, dann Tester neu starten.")
        vjoy.RelinquishVJD(VJOY_DEVICE_ID)
        sys.exit(1)

    vjoy.ResetVJD(VJOY_DEVICE_ID)
    vjoy.SetAxis(AXIS_CENTER, VJOY_DEVICE_ID, HID_USAGE_X)
    vjoy.SetAxis(AXIS_MIN, VJOY_DEVICE_ID, HID_USAGE_SL0)
    vjoy.SetAxis(AXIS_MIN, VJOY_DEVICE_ID, HID_USAGE_SL1)

    # GUI
    root = tk.Tk()
    root.title("vJoy Axis Tester v3 - TruckPilot")
    root.geometry("560x520")
    root.resizable(False, False)

    main_frame = ttk.Frame(root, padding=20)
    main_frame.pack(fill=tk.BOTH, expand=True)

    title = ttk.Label(main_frame, text=f"vJoy Device {VJOY_DEVICE_ID}", font=("Segoe UI", 14, "bold"))
    title.pack(pady=(0, 4))

    hint = ttk.Label(
        main_frame,
        text="Tipp: 'Wackeln' Button fuer 8s automatische Achsenbewegung — ETS2 kann waehrenddessen zuweisen.",
        font=("Segoe UI", 8),
        foreground="#666666",
        wraplength=520,
    )
    hint.pack(pady=(0, 12))

    # State fuer Wackel-Animationen
    wiggle_state = {"steer": None, "throttle": None, "brake": None}

    def stop_wiggle(key):
        if wiggle_state[key] is not None:
            try:
                root.after_cancel(wiggle_state[key])
            except Exception:
                pass
            wiggle_state[key] = None

    # ---- Steering (X-Axis, bipolar) ----
    steer_frame = ttk.LabelFrame(main_frame, text="Steering (X-Axis, bipolar)", padding=10)
    steer_frame.pack(fill=tk.X, pady=4)

    steer_value_var = tk.StringVar(value="16384 (Mitte)")
    steer_status_var = tk.StringVar(value="")

    ttk.Label(steer_frame, textvariable=steer_value_var, font=("Consolas", 10)).pack(anchor=tk.W)

    def set_steer_label(v):
        if v == AXIS_CENTER:
            steer_value_var.set(f"{v} (Mitte)")
        elif v < AXIS_CENTER:
            pct = int((AXIS_CENTER - v) / AXIS_CENTER * 100)
            steer_value_var.set(f"{v} (Links {pct}%)")
        else:
            pct = int((v - AXIS_CENTER) / AXIS_CENTER * 100)
            steer_value_var.set(f"{v} (Rechts {pct}%)")

    def on_steer(val):
        v = int(float(val))
        vjoy.SetAxis(v, VJOY_DEVICE_ID, HID_USAGE_X)
        set_steer_label(v)

    steer_scale = ttk.Scale(
        steer_frame, from_=AXIS_MIN, to=AXIS_MAX, orient=tk.HORIZONTAL,
        length=480, command=on_steer
    )
    steer_scale.set(AXIS_CENTER)
    steer_scale.pack()

    steer_btn_row = ttk.Frame(steer_frame)
    steer_btn_row.pack(pady=(6, 0), fill=tk.X)

    def reset_steer():
        stop_wiggle("steer")
        steer_status_var.set("")
        steer_scale.set(AXIS_CENTER)

    def start_wiggle_steer():
        stop_wiggle("steer")
        start_time = time.time()
        steer_status_var.set("Wackelt... ETS2 jetzt zuweisen")

        def tick():
            elapsed = time.time() - start_time
            if elapsed >= WIGGLE_DURATION_SEC:
                vjoy.SetAxis(AXIS_CENTER, VJOY_DEVICE_ID, HID_USAGE_X)
                steer_scale.set(AXIS_CENTER)
                steer_status_var.set("Fertig.")
                wiggle_state["steer"] = None
                root.after(1500, lambda: steer_status_var.set(""))
                return
            # Sinus zwischen AXIS_MIN und AXIS_MAX, gross genug damit ETS2 es sieht
            phase = math.sin(2 * math.pi * WIGGLE_FREQ_HZ * elapsed)
            v = int(AXIS_CENTER + phase * (AXIS_CENTER - 100))
            v = max(AXIS_MIN, min(AXIS_MAX, v))
            vjoy.SetAxis(v, VJOY_DEVICE_ID, HID_USAGE_X)
            steer_scale.set(v)
            wiggle_state["steer"] = root.after(WIGGLE_TICK_MS, tick)

        tick()

    ttk.Button(steer_btn_row, text="Zur Mitte", command=reset_steer).pack(side=tk.LEFT, padx=(0, 8))
    ttk.Button(steer_btn_row, text="Wackeln (8s)", command=start_wiggle_steer).pack(side=tk.LEFT)
    ttk.Label(steer_btn_row, textvariable=steer_status_var, foreground="#0066cc").pack(side=tk.LEFT, padx=(12, 0))

    # ---- Throttle (Slider, unipolar) ----
    throttle_frame = ttk.LabelFrame(main_frame, text="Throttle (Slider, unipolar)", padding=10)
    throttle_frame.pack(fill=tk.X, pady=4)

    throttle_value_var = tk.StringVar(value="0 (0%)")
    throttle_status_var = tk.StringVar(value="")

    ttk.Label(throttle_frame, textvariable=throttle_value_var, font=("Consolas", 10)).pack(anchor=tk.W)

    def set_throttle_label(v):
        pct = int(v / AXIS_MAX * 100)
        throttle_value_var.set(f"{v} ({pct}%)")

    def on_throttle(val):
        v = int(float(val))
        vjoy.SetAxis(v, VJOY_DEVICE_ID, HID_USAGE_SL0)
        set_throttle_label(v)

    throttle_scale = ttk.Scale(
        throttle_frame, from_=AXIS_MIN, to=AXIS_MAX, orient=tk.HORIZONTAL,
        length=480, command=on_throttle
    )
    throttle_scale.set(AXIS_MIN)
    throttle_scale.pack()

    throttle_btn_row = ttk.Frame(throttle_frame)
    throttle_btn_row.pack(pady=(6, 0), fill=tk.X)

    def reset_throttle():
        stop_wiggle("throttle")
        throttle_status_var.set("")
        throttle_scale.set(AXIS_MIN)

    def start_wiggle_throttle():
        stop_wiggle("throttle")
        start_time = time.time()
        throttle_status_var.set("Wackelt... ETS2 jetzt zuweisen")

        def tick():
            elapsed = time.time() - start_time
            if elapsed >= WIGGLE_DURATION_SEC:
                vjoy.SetAxis(AXIS_MIN, VJOY_DEVICE_ID, HID_USAGE_SL0)
                throttle_scale.set(AXIS_MIN)
                throttle_status_var.set("Fertig.")
                wiggle_state["throttle"] = None
                root.after(1500, lambda: throttle_status_var.set(""))
                return
            # Unipolar Sinus: zwischen 0 und MAX, beginnt bei 0
            phase = (math.sin(2 * math.pi * WIGGLE_FREQ_HZ * elapsed - math.pi / 2) + 1) / 2
            v = int(phase * AXIS_MAX)
            v = max(AXIS_MIN, min(AXIS_MAX, v))
            vjoy.SetAxis(v, VJOY_DEVICE_ID, HID_USAGE_SL0)
            throttle_scale.set(v)
            wiggle_state["throttle"] = root.after(WIGGLE_TICK_MS, tick)

        tick()

    ttk.Button(throttle_btn_row, text="Auf 0", command=reset_throttle).pack(side=tk.LEFT, padx=(0, 8))
    ttk.Button(throttle_btn_row, text="Wackeln (8s)", command=start_wiggle_throttle).pack(side=tk.LEFT)
    ttk.Label(throttle_btn_row, textvariable=throttle_status_var, foreground="#0066cc").pack(side=tk.LEFT, padx=(12, 0))

    # ---- Brake (Dial/Slider2, unipolar) ----
    brake_frame = ttk.LabelFrame(main_frame, text="Brake (Dial/Slider2, unipolar)", padding=10)
    brake_frame.pack(fill=tk.X, pady=4)

    brake_value_var = tk.StringVar(value="0 (0%)")
    brake_status_var = tk.StringVar(value="")

    ttk.Label(brake_frame, textvariable=brake_value_var, font=("Consolas", 10)).pack(anchor=tk.W)

    def set_brake_label(v):
        pct = int(v / AXIS_MAX * 100)
        brake_value_var.set(f"{v} ({pct}%)")

    def on_brake(val):
        v = int(float(val))
        vjoy.SetAxis(v, VJOY_DEVICE_ID, HID_USAGE_SL1)
        set_brake_label(v)

    brake_scale = ttk.Scale(
        brake_frame, from_=AXIS_MIN, to=AXIS_MAX, orient=tk.HORIZONTAL,
        length=480, command=on_brake
    )
    brake_scale.set(AXIS_MIN)
    brake_scale.pack()

    brake_btn_row = ttk.Frame(brake_frame)
    brake_btn_row.pack(pady=(6, 0), fill=tk.X)

    def reset_brake():
        stop_wiggle("brake")
        brake_status_var.set("")
        brake_scale.set(AXIS_MIN)

    def start_wiggle_brake():
        stop_wiggle("brake")
        start_time = time.time()
        brake_status_var.set("Wackelt... ETS2 jetzt zuweisen")

        def tick():
            elapsed = time.time() - start_time
            if elapsed >= WIGGLE_DURATION_SEC:
                vjoy.SetAxis(AXIS_MIN, VJOY_DEVICE_ID, HID_USAGE_SL1)
                brake_scale.set(AXIS_MIN)
                brake_status_var.set("Fertig.")
                wiggle_state["brake"] = None
                root.after(1500, lambda: brake_status_var.set(""))
                return
            phase = (math.sin(2 * math.pi * WIGGLE_FREQ_HZ * elapsed - math.pi / 2) + 1) / 2
            v = int(phase * AXIS_MAX)
            v = max(AXIS_MIN, min(AXIS_MAX, v))
            vjoy.SetAxis(v, VJOY_DEVICE_ID, HID_USAGE_SL1)
            brake_scale.set(v)
            wiggle_state["brake"] = root.after(WIGGLE_TICK_MS, tick)

        tick()

    ttk.Button(brake_btn_row, text="Auf 0", command=reset_brake).pack(side=tk.LEFT, padx=(0, 8))
    ttk.Button(brake_btn_row, text="Wackeln (8s)", command=start_wiggle_brake).pack(side=tk.LEFT)
    ttk.Label(brake_btn_row, textvariable=brake_status_var, foreground="#0066cc").pack(side=tk.LEFT, padx=(12, 0))

    # ---- Reset Alles ----
    def reset_all():
        stop_wiggle("steer")
        stop_wiggle("throttle")
        stop_wiggle("brake")
        steer_status_var.set("")
        throttle_status_var.set("")
        brake_status_var.set("")
        steer_scale.set(AXIS_CENTER)
        throttle_scale.set(AXIS_MIN)
        brake_scale.set(AXIS_MIN)

    ttk.Button(main_frame, text="Reset Alles", command=reset_all).pack(pady=(12, 0))

    # Cleanup on close
    def on_close():
        stop_wiggle("steer")
        stop_wiggle("throttle")
        stop_wiggle("brake")
        vjoy.ResetVJD(VJOY_DEVICE_ID)
        vjoy.RelinquishVJD(VJOY_DEVICE_ID)
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    print("\nTester laeuft. Schliesse das Fenster zum Beenden.")
    print("Tipp: Klick 'Wackeln (8s)' pro Achse, dann in ETS2 die Achse zuweisen.")
    root.mainloop()


if __name__ == "__main__":
    main()
"""
vJoy Axis Tester v4 - mit Delayed-Test Funktion

X-Achse fuer Steering (bipolar)
Slider fuer Throttle (unipolar)
Dial/Slider2 fuer Brake (unipolar)

Pro Achse:
- Slider zum manuellen Setzen
- "Wackeln (8s)" Button fuer ETS2 Achsen-Zuweisung
- TARGET-Input + Delay + "Test starten" Button: nach Delay-Sekunden faehrt
  die Achse auf den Target-Wert an
"""

import ctypes
import math
import os
import sys
import time
import tkinter as tk
from tkinter import ttk

HID_USAGE_X = 0x30
HID_USAGE_SL0 = 0x36
HID_USAGE_SL1 = 0x37

VJOY_DEVICE_ID = 1
AXIS_MIN = 0
AXIS_MAX = 32767
AXIS_CENTER = 16384

WIGGLE_DURATION_SEC = 8.0
WIGGLE_FREQ_HZ = 1.5
WIGGLE_TICK_MS = 20

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

    print(f"X (Steering):         {'OK' if x_exists else 'MISSING'}")
    print(f"Slider (Throttle):    {'OK' if sl0_exists else 'MISSING'}")
    print(f"Dial/Slider2 (Brake): {'OK' if sl1_exists else 'MISSING'}")

    if not (x_exists and sl0_exists and sl1_exists):
        print("\nFAIL: Aktiviere die fehlenden Achsen in vJoyConf.exe, Apply, dann Tester neu starten.")
        vjoy.RelinquishVJD(VJOY_DEVICE_ID)
        sys.exit(1)

    vjoy.ResetVJD(VJOY_DEVICE_ID)
    vjoy.SetAxis(AXIS_CENTER, VJOY_DEVICE_ID, HID_USAGE_X)
    vjoy.SetAxis(AXIS_MIN, VJOY_DEVICE_ID, HID_USAGE_SL0)
    vjoy.SetAxis(AXIS_MIN, VJOY_DEVICE_ID, HID_USAGE_SL1)

    root = tk.Tk()
    root.title("vJoy Axis Tester v4 - TruckPilot")
    root.geometry("620x780")
    root.resizable(False, False)

    main_frame = ttk.Frame(root, padding=16)
    main_frame.pack(fill=tk.BOTH, expand=True)

    title = ttk.Label(main_frame, text=f"vJoy Device {VJOY_DEVICE_ID}", font=("Segoe UI", 14, "bold"))
    title.pack(pady=(0, 4))

    hint = ttk.Label(
        main_frame,
        text="Target + Delay: Zahl eingeben, Delay-Sekunden setzen, 'Test starten' druecken. Achse faehrt nach Delay den Target-Wert an.",
        font=("Segoe UI", 8),
        foreground="#666666",
        wraplength=580,
    )
    hint.pack(pady=(0, 12))

    # State fuer Animationen
    wiggle_state = {"steer": None, "throttle": None, "brake": None}
    test_state = {"steer": None, "throttle": None, "brake": None}

    def stop_wiggle(key):
        if wiggle_state[key] is not None:
            try:
                root.after_cancel(wiggle_state[key])
            except Exception:
                pass
            wiggle_state[key] = None

    def stop_test(key):
        if test_state[key] is not None:
            try:
                root.after_cancel(test_state[key])
            except Exception:
                pass
            test_state[key] = None

    def build_axis_panel(parent, label, hid_usage, is_bipolar, default_value, target_default):
        """
        Gibt zurueck: (set_func, status_var, scale, target_var, delay_var, test_button)
        set_func(value) setzt vJoy und Slider+Label
        """
        frame = ttk.LabelFrame(parent, text=label, padding=10)
        frame.pack(fill=tk.X, pady=4)

        value_var = tk.StringVar()
        status_var = tk.StringVar(value="")

        def format_label(v):
            if is_bipolar:
                if v == AXIS_CENTER:
                    return f"{v} (Mitte)"
                elif v < AXIS_CENTER:
                    pct = int((AXIS_CENTER - v) / AXIS_CENTER * 100)
                    return f"{v} (Links {pct}%)"
                else:
                    pct = int((v - AXIS_CENTER) / AXIS_CENTER * 100)
                    return f"{v} (Rechts {pct}%)"
            else:
                pct = int(v / AXIS_MAX * 100)
                return f"{v} ({pct}%)"

        value_var.set(format_label(default_value))

        ttk.Label(frame, textvariable=value_var, font=("Consolas", 10)).pack(anchor=tk.W)

        def on_scale(val):
            v = int(float(val))
            vjoy.SetAxis(v, VJOY_DEVICE_ID, hid_usage)
            value_var.set(format_label(v))

        scale = ttk.Scale(
            frame, from_=AXIS_MIN, to=AXIS_MAX, orient=tk.HORIZONTAL,
            length=560, command=on_scale
        )
        scale.set(default_value)
        scale.pack()

        # Buttons-Reihe 1: Reset + Wackeln
        btn_row1 = ttk.Frame(frame)
        btn_row1.pack(pady=(6, 4), fill=tk.X)

        return frame, value_var, status_var, scale, format_label, on_scale, btn_row1

    # ---- Steering ----
    steer_frame, steer_value_var, steer_status_var, steer_scale, steer_format, steer_set, steer_btn_row1 = \
        build_axis_panel(main_frame, "Steering (X-Axis, bipolar)", HID_USAGE_X, True, AXIS_CENTER, 24000)

    def reset_steer():
        stop_wiggle("steer")
        stop_test("steer")
        steer_status_var.set("")
        steer_scale.set(AXIS_CENTER)

    def start_wiggle_steer():
        stop_wiggle("steer")
        stop_test("steer")
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
            phase = math.sin(2 * math.pi * WIGGLE_FREQ_HZ * elapsed)
            v = int(AXIS_CENTER + phase * (AXIS_CENTER - 100))
            v = max(AXIS_MIN, min(AXIS_MAX, v))
            vjoy.SetAxis(v, VJOY_DEVICE_ID, HID_USAGE_X)
            steer_scale.set(v)
            wiggle_state["steer"] = root.after(WIGGLE_TICK_MS, tick)

        tick()

    ttk.Button(steer_btn_row1, text="Zur Mitte", command=reset_steer).pack(side=tk.LEFT, padx=(0, 8))
    ttk.Button(steer_btn_row1, text="Wackeln (8s)", command=start_wiggle_steer).pack(side=tk.LEFT)
    ttk.Label(steer_btn_row1, textvariable=steer_status_var, foreground="#0066cc").pack(side=tk.LEFT, padx=(12, 0))

    # Test-Bereich Steering
    steer_test_row = ttk.Frame(steer_frame)
    steer_test_row.pack(fill=tk.X, pady=(8, 0))

    ttk.Label(steer_test_row, text="Target:").pack(side=tk.LEFT)
    steer_target_var = tk.StringVar(value="24000")
    steer_target_entry = ttk.Entry(steer_test_row, textvariable=steer_target_var, width=8)
    steer_target_entry.pack(side=tk.LEFT, padx=(4, 8))

    ttk.Label(steer_test_row, text="Delay:").pack(side=tk.LEFT)
    steer_delay_var = tk.IntVar(value=5)
    steer_delay_spin = ttk.Spinbox(steer_test_row, from_=0, to=30, textvariable=steer_delay_var, width=4)
    steer_delay_spin.pack(side=tk.LEFT, padx=(4, 4))
    ttk.Label(steer_test_row, text="s").pack(side=tk.LEFT, padx=(0, 8))

    steer_countdown_var = tk.StringVar(value="")
    ttk.Label(steer_test_row, textvariable=steer_countdown_var, foreground="#cc6600", font=("Consolas", 10, "bold")).pack(side=tk.LEFT, padx=(8, 0))

    def start_test_steer():
        stop_wiggle("steer")
        stop_test("steer")
        try:
            target = int(steer_target_var.get())
        except ValueError:
            steer_countdown_var.set("Target ungueltig")
            return
        target = max(AXIS_MIN, min(AXIS_MAX, target))
        delay = max(0, steer_delay_var.get())
        start_time = time.time()

        def tick():
            elapsed = time.time() - start_time
            remaining = delay - elapsed
            if remaining > 0:
                steer_countdown_var.set(f"in {remaining:.1f}s -> {target}")
                test_state["steer"] = root.after(100, tick)
            else:
                vjoy.SetAxis(target, VJOY_DEVICE_ID, HID_USAGE_X)
                steer_scale.set(target)
                steer_countdown_var.set(f"Gesetzt: {target}")
                test_state["steer"] = None
                root.after(2000, lambda: steer_countdown_var.set(""))

        tick()

    ttk.Button(steer_test_row, text="Test starten", command=start_test_steer).pack(side=tk.LEFT)

    # ---- Throttle ----
    throttle_frame, throttle_value_var, throttle_status_var, throttle_scale, throttle_format, throttle_set, throttle_btn_row1 = \
        build_axis_panel(main_frame, "Throttle (Slider, unipolar)", HID_USAGE_SL0, False, AXIS_MIN, 16000)

    def reset_throttle():
        stop_wiggle("throttle")
        stop_test("throttle")
        throttle_status_var.set("")
        throttle_scale.set(AXIS_MIN)

    def start_wiggle_throttle():
        stop_wiggle("throttle")
        stop_test("throttle")
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
            phase = (math.sin(2 * math.pi * WIGGLE_FREQ_HZ * elapsed - math.pi / 2) + 1) / 2
            v = int(phase * AXIS_MAX)
            v = max(AXIS_MIN, min(AXIS_MAX, v))
            vjoy.SetAxis(v, VJOY_DEVICE_ID, HID_USAGE_SL0)
            throttle_scale.set(v)
            wiggle_state["throttle"] = root.after(WIGGLE_TICK_MS, tick)

        tick()

    ttk.Button(throttle_btn_row1, text="Auf 0", command=reset_throttle).pack(side=tk.LEFT, padx=(0, 8))
    ttk.Button(throttle_btn_row1, text="Wackeln (8s)", command=start_wiggle_throttle).pack(side=tk.LEFT)
    ttk.Label(throttle_btn_row1, textvariable=throttle_status_var, foreground="#0066cc").pack(side=tk.LEFT, padx=(12, 0))

    throttle_test_row = ttk.Frame(throttle_frame)
    throttle_test_row.pack(fill=tk.X, pady=(8, 0))

    ttk.Label(throttle_test_row, text="Target:").pack(side=tk.LEFT)
    throttle_target_var = tk.StringVar(value="16000")
    throttle_target_entry = ttk.Entry(throttle_test_row, textvariable=throttle_target_var, width=8)
    throttle_target_entry.pack(side=tk.LEFT, padx=(4, 8))

    ttk.Label(throttle_test_row, text="Delay:").pack(side=tk.LEFT)
    throttle_delay_var = tk.IntVar(value=5)
    throttle_delay_spin = ttk.Spinbox(throttle_test_row, from_=0, to=30, textvariable=throttle_delay_var, width=4)
    throttle_delay_spin.pack(side=tk.LEFT, padx=(4, 4))
    ttk.Label(throttle_test_row, text="s").pack(side=tk.LEFT, padx=(0, 8))

    throttle_countdown_var = tk.StringVar(value="")
    ttk.Label(throttle_test_row, textvariable=throttle_countdown_var, foreground="#cc6600", font=("Consolas", 10, "bold")).pack(side=tk.LEFT, padx=(8, 0))

    def start_test_throttle():
        stop_wiggle("throttle")
        stop_test("throttle")
        try:
            target = int(throttle_target_var.get())
        except ValueError:
            throttle_countdown_var.set("Target ungueltig")
            return
        target = max(AXIS_MIN, min(AXIS_MAX, target))
        delay = max(0, throttle_delay_var.get())
        start_time = time.time()

        def tick():
            elapsed = time.time() - start_time
            remaining = delay - elapsed
            if remaining > 0:
                throttle_countdown_var.set(f"in {remaining:.1f}s -> {target}")
                test_state["throttle"] = root.after(100, tick)
            else:
                vjoy.SetAxis(target, VJOY_DEVICE_ID, HID_USAGE_SL0)
                throttle_scale.set(target)
                throttle_countdown_var.set(f"Gesetzt: {target}")
                test_state["throttle"] = None
                root.after(2000, lambda: throttle_countdown_var.set(""))

        tick()

    ttk.Button(throttle_test_row, text="Test starten", command=start_test_throttle).pack(side=tk.LEFT)

    # ---- Brake ----
    brake_frame, brake_value_var, brake_status_var, brake_scale, brake_format, brake_set, brake_btn_row1 = \
        build_axis_panel(main_frame, "Brake (Dial/Slider2, unipolar)", HID_USAGE_SL1, False, AXIS_MIN, 16000)

    def reset_brake():
        stop_wiggle("brake")
        stop_test("brake")
        brake_status_var.set("")
        brake_scale.set(AXIS_MIN)

    def start_wiggle_brake():
        stop_wiggle("brake")
        stop_test("brake")
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

    ttk.Button(brake_btn_row1, text="Auf 0", command=reset_brake).pack(side=tk.LEFT, padx=(0, 8))
    ttk.Button(brake_btn_row1, text="Wackeln (8s)", command=start_wiggle_brake).pack(side=tk.LEFT)
    ttk.Label(brake_btn_row1, textvariable=brake_status_var, foreground="#0066cc").pack(side=tk.LEFT, padx=(12, 0))

    brake_test_row = ttk.Frame(brake_frame)
    brake_test_row.pack(fill=tk.X, pady=(8, 0))

    ttk.Label(brake_test_row, text="Target:").pack(side=tk.LEFT)
    brake_target_var = tk.StringVar(value="16000")
    brake_target_entry = ttk.Entry(brake_test_row, textvariable=brake_target_var, width=8)
    brake_target_entry.pack(side=tk.LEFT, padx=(4, 8))

    ttk.Label(brake_test_row, text="Delay:").pack(side=tk.LEFT)
    brake_delay_var = tk.IntVar(value=5)
    brake_delay_spin = ttk.Spinbox(brake_test_row, from_=0, to=30, textvariable=brake_delay_var, width=4)
    brake_delay_spin.pack(side=tk.LEFT, padx=(4, 4))
    ttk.Label(brake_test_row, text="s").pack(side=tk.LEFT, padx=(0, 8))

    brake_countdown_var = tk.StringVar(value="")
    ttk.Label(brake_test_row, textvariable=brake_countdown_var, foreground="#cc6600", font=("Consolas", 10, "bold")).pack(side=tk.LEFT, padx=(8, 0))

    def start_test_brake():
        stop_wiggle("brake")
        stop_test("brake")
        try:
            target = int(brake_target_var.get())
        except ValueError:
            brake_countdown_var.set("Target ungueltig")
            return
        target = max(AXIS_MIN, min(AXIS_MAX, target))
        delay = max(0, brake_delay_var.get())
        start_time = time.time()

        def tick():
            elapsed = time.time() - start_time
            remaining = delay - elapsed
            if remaining > 0:
                brake_countdown_var.set(f"in {remaining:.1f}s -> {target}")
                test_state["brake"] = root.after(100, tick)
            else:
                vjoy.SetAxis(target, VJOY_DEVICE_ID, HID_USAGE_SL1)
                brake_scale.set(target)
                brake_countdown_var.set(f"Gesetzt: {target}")
                test_state["brake"] = None
                root.after(2000, lambda: brake_countdown_var.set(""))

        tick()

    ttk.Button(brake_test_row, text="Test starten", command=start_test_brake).pack(side=tk.LEFT)

    # ---- Reset Alles ----
    def reset_all():
        for k in ["steer", "throttle", "brake"]:
            stop_wiggle(k)
            stop_test(k)
        steer_status_var.set("")
        throttle_status_var.set("")
        brake_status_var.set("")
        steer_countdown_var.set("")
        throttle_countdown_var.set("")
        brake_countdown_var.set("")
        steer_scale.set(AXIS_CENTER)
        throttle_scale.set(AXIS_MIN)
        brake_scale.set(AXIS_MIN)

    ttk.Button(main_frame, text="Reset Alles", command=reset_all).pack(pady=(12, 0))

    def on_close():
        for k in ["steer", "throttle", "brake"]:
            stop_wiggle(k)
            stop_test(k)
        vjoy.ResetVJD(VJOY_DEVICE_ID)
        vjoy.RelinquishVJD(VJOY_DEVICE_ID)
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    print("\nTester laeuft. Schliesse das Fenster zum Beenden.")
    print("Tipp: Target eingeben, Delay einstellen, 'Test starten' druecken.")
    root.mainloop()


if __name__ == "__main__":
    main()
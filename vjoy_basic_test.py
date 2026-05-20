import pyvjoy
import time

j = pyvjoy.VJoyDevice(1)
print("Connected")

for i in range(10):
    j.set_axis(pyvjoy.HID_USAGE_X, 32768)
    print(f"Tick {i}: X=MAX (32768)")
    time.sleep(0.5)
    j.set_axis(pyvjoy.HID_USAGE_X, 1)
    print(f"Tick {i}: X=MIN (1)")
    time.sleep(0.5)

print("Done, X back to mid")
j.set_axis(pyvjoy.HID_USAGE_X, 16384)

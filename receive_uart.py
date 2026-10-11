# receive_uart.py
import sys
import numpy as np
import serial
import matplotlib.pyplot as plt

PORT = "COM5"        # Device Manager -> Ports. The Basys 3 shows two; try the other if this stays silent.
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=15)
ser.reset_input_buffer()

# find the A5 5A header
prev = b""
while True:
    b = ser.read(1)
    if not b:
        sys.exit("Timeout: no data. Check the COM port, the board programming, and that the render finished (led0 on).")
    if prev == b"\xa5" and b == b"\x5a":
        break
    prev = b

W, H = ser.read(2)
n = W * H * 3
print(f"Header OK: {W}x{H}, reading {n} bytes...")
data = ser.read(n)
if len(data) != n:
    sys.exit(f"Short read: got {len(data)} of {n} bytes")

img = np.frombuffer(data, dtype=np.uint8).reshape(H, W, 3)
plt.imsave("uart_rt.png", img)

# compare with the golden model
try:
    gold = [l.strip() for l in open("golden_rt.txt") if l.strip()]
    ref = np.array([[int(g[0:2], 16), int(g[2:4], 16), int(g[4:6], 16)] for g in gold],
                   dtype=np.uint8).reshape(H, W, 3)
    bad = np.any(img != ref, axis=2).sum()
    print(f"Mismatching pixels vs golden: {bad}")
    print("PASS" if bad == 0 else "FAIL")
except FileNotFoundError:
    print("golden_rt.txt not found, skipping comparison")

plt.imshow(img); plt.title("Image received over UART"); plt.axis("off"); plt.show()
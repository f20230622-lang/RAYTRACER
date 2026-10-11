# verify_rt.py
import numpy as np
import matplotlib.pyplot as plt
from scene import CAMERA

W, H = CAMERA["width"], CAMERA["height"]
gold = [l.strip() for l in open("golden_rt.txt") if l.strip()]
hw   = [l.strip() for l in open("hw_rt.txt")   if l.strip()]

print(f"golden: {len(gold)}   hardware: {len(hw)}   expected: {W*H}")
bad = [i for i, (g, h) in enumerate(zip(gold, hw)) if g != h]
print(f"Mismatching pixels: {len(bad)}")
for i in bad[:10]:
    print(f"  pixel {i} (x={i % W}, y={i // W}): golden={gold[i]}  hw={hw[i]}")
print("PASS" if not bad and len(gold) == len(hw) == W * H else "FAIL")

img = np.array([[int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)] for h in hw],
               dtype=np.uint8).reshape(H, W, 3)
plt.imsave("hw_rt.png", img)
plt.imshow(img); plt.title("Hardware ray tracer"); plt.axis("off"); plt.show()
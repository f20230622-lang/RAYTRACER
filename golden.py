# golden.py
import numpy as np
import matplotlib.pyplot as plt
from scene import CAMERA, FRAC_BITS, to_fixed
import sys, time

def progress(i, n, label="", width=30):
    """Text progress bar; call with i = items done so far."""
    frac = i / n
    filled = int(width * frac)
    bar = "#" * filled + "-" * (width - filled)
    sys.stdout.write(f"\r{label:<14}[{bar}] {frac*100:5.1f}%")
    if i == n:
        sys.stdout.write("\n")
    sys.stdout.flush()

F = FRAC_BITS
MISS_ID = 0xFFFFFFFF
DET_EPS = 16            # |det| below this (~0.00024) counts as parallel -> miss

# ---- 32-bit signed fixed-point primitives (mirror Verilog) ------------
def s32(x):
    """Wrap to signed 32-bit, like a 32-bit Verilog register."""
    x &= 0xFFFFFFFF
    return x - (1 << 32) if x & 0x80000000 else x

def mul(a, b):
    """Q16.16 multiply: 64-bit product, arithmetic shift right, truncate to 32."""
    return s32((a * b) >> F)

def sub(a, b):
    return [s32(a[i] - b[i]) for i in range(3)]

def dot(a, b):
    acc = 0
    for i in range(3):
        acc = s32(acc + mul(a[i], b[i]))
    return acc

def cross(a, b):
    return [s32(mul(a[1], b[2]) - mul(a[2], b[1])),
            s32(mul(a[2], b[0]) - mul(a[0], b[2])),
            s32(mul(a[0], b[1]) - mul(a[1], b[0]))]

# ---- Load the exact files Verilog will read ---------------------------
def read_mem(path):
    with open(path) as f:
        return [int(line, 16) for line in f if line.strip()]

def load_scene():
    vw = [s32(w) for w in read_mem("verts.mem")]
    verts = [vw[i:i+3] for i in range(0, len(vw), 3)]
    tw = read_mem("tris.mem")
    tris = [tw[i:i+3] for i in range(0, len(tw), 3)]
    return verts, tris

# ---- Ray generation (pixel centers, origin at 0, direction z = 1) -----
def make_rays_fixed():
    W, H = CAMERA["width"], CAMERA["height"]
    hw = to_fixed(CAMERA["fov_half_width"])
    step = (2 * hw) // W                 # pixel pitch on image plane
    hh = (step * H) // 2                 # half-height (square pixels)
    rays = []
    for py in range(H):
        for px in range(W):
            u = -hw + step // 2 + px * step
            v = hh - step // 2 - py * step     # +y is up, row 0 is top
            rays.append((u, v, 1 << F))
    return rays

# ---- Fixed-point Moller-Trumbore (division only after a confirmed hit) -
def intersect_parts(o, d, v0, v1, v2):
    """Returns (hit, a, tn). a and tn are the normalized (a > 0) values."""
    e1 = sub(v1, v0)
    e2 = sub(v2, v0)
    h = cross(d, e2)
    a = dot(e1, h)
    s = sub(o, v0)
    un = dot(s, h)
    q = cross(s, e1)
    vn = dot(d, q)
    tn = dot(e2, q)

    if abs(a) < DET_EPS:
        return False, 0, 0
    if a < 0:
        a, un, vn, tn = -a, -un, -vn, -tn
    if un < 0 or un > a:
        return False, 0, 0
    if vn < 0 or un + vn > a:
        return False, 0, 0
    if tn <= 0:
        return False, 0, 0
    return True, a, tn

def intersect_fixed(o, d, v0, v1, v2):
    hit, a, tn = intersect_parts(o, d, v0, v1, v2)
    if not hit:
        return False, 0
    return True, s32((tn << F) // a)

def render_fixed():
    verts, tris = load_scene()
    rays = make_rays_fixed()
    W, H = CAMERA["width"], CAMERA["height"]
    origin = [0, 0, 0]
    ids, ts = [], []
    for n, d in enumerate(rays):
        best_id, best_t = MISS_ID, 0
        for ti, (i0, i1, i2) in enumerate(tris):
            hit, t = intersect_fixed(origin, d, verts[i0], verts[i1], verts[i2])
            if hit and (best_id == MISS_ID or t < best_t):
                best_id, best_t = ti, t
        ids.append(best_id)
        ts.append(best_t)
        if (n + 1) % W == 0:
            progress(n + 1, W * H, "Fixed-point")
    return ids, ts

# ---- Float reference (sanity check on the fixed-point model) ----------
def render_float():
    import json
    cfg = json.load(open("config.json"))
    V = np.array(cfg["vertices"]); T = cfg["triangles"]
    W, H = CAMERA["width"], CAMERA["height"]
    hw = CAMERA["fov_half_width"]; step = 2 * hw / W; hh = step * H / 2
    ids, ts = [], []
    for py in range(H):
        for px in range(W):
            d = np.array([-hw + step * (px + 0.5), hh - step * (py + 0.5), 1.0])
            best_id, best_t = MISS_ID, 0.0
            for ti, tri in enumerate(T):
                v0, v1, v2 = V[tri[0]], V[tri[1]], V[tri[2]]
                e1, e2 = v1 - v0, v2 - v0
                h = np.cross(d, e2); a = e1 @ h
                if abs(a) < 1e-9: continue
                s = -v0
                u = (s @ h) / a
                q = np.cross(s, e1)
                v = (d @ q) / a
                t = (e2 @ q) / a
                if u >= 0 and v >= 0 and u + v <= 1 and t > 0:
                    if best_id == MISS_ID or t < best_t:
                        best_id, best_t = ti, t
            ids.append(best_id); ts.append(best_t)
        progress(py + 1, H, "Float ref")
    return ids, ts

# ---- Output -----------------------------------------------------------
def write_golden(ids, ts, path="golden.txt"):
    with open(path, "w") as f:
        for i, t in zip(ids, ts):
            f.write(f"{i & 0xFFFFFFFF:08x} {t & 0xFFFFFFFF:08x}\n")

if __name__ == "__main__":
    W, H = CAMERA["width"], CAMERA["height"]
    ids_fx, ts_fx = render_fixed()
    ids_fl, ts_fl = render_float()
    write_golden(ids_fx, ts_fx)

    id_mismatch = sum(a != b for a, b in zip(ids_fx, ids_fl))
    both = [(a, b) for a, b, i, j in zip(ts_fx, ts_fl, ids_fx, ids_fl)
            if i == j and i != MISS_ID]
    max_err = max((abs(a / (1 << F) - b) for a, b in both), default=0)
    print(f"Hit pixels (fixed): {sum(i != MISS_ID for i in ids_fx)} / {W*H}")
    print(f"Fixed vs float ID mismatches: {id_mismatch}")
    print(f"Max |t| error (fixed vs float): {max_err:.6f}")
    print("Wrote golden.txt")

    # Visualize
    idimg = np.array([-1 if i == MISS_ID else i for i in ids_fx]).reshape(H, W)
    timg = np.array([t / (1 << F) if i != MISS_ID else np.nan
                     for i, t in zip(ids_fx, ts_fx)]).reshape(H, W)
    fig, ax = plt.subplots(1, 2, figsize=(10, 5))
    ax[0].imshow(idimg, cmap="viridis"); ax[0].set_title("Triangle ID (-1 = miss)")
    im = ax[1].imshow(timg, cmap="magma"); ax[1].set_title("Depth t")
    fig.colorbar(im, ax=ax[1])
    plt.savefig("golden.png"); plt.show()
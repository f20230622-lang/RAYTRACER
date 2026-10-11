# golden_rt.py -- bit-accurate fixed-point ray tracer (spec for the Verilog)
import re
import numpy as np
import matplotlib.pyplot as plt
from golden import load_scene, make_rays_fixed, intersect_parts, mul, s32, dot, F, progress
from scene import CAMERA

ONE = 1 << F

# ---- Load the exact files the hardware will read ---------------------
def read_words(path):
    return [s32(int(l, 16)) for l in open(path) if l.strip()]

def read_params(path="rt_params.vh"):
    p = {}
    for line in open(path):
        m = re.match(r"localparam\s+signed\s+\[31:0\]\s+(\w+)\s*=\s*32'h([0-9a-fA-F]+);", line)
        if m:
            p[m.group(1)] = s32(int(m.group(2), 16))
    return p

P = read_params()
verts, tris = load_scene()
nw = read_words("normals.mem")
NORM = [nw[i:i + 3] for i in range(0, len(nw), 3)]
mw = read_words("mat.mem")
MAT = [mw[i:i + 16] for i in range(0, len(mw), 16)]
assert len(NORM) == len(tris) == len(MAT)

L  = [P["RT_LIGHT_X"], P["RT_LIGHT_Y"], P["RT_LIGHT_Z"]]
L1 = [P["RT_L1_X"], P["RT_L1_Y"], P["RT_L1_Z"]]
AMB, BIAS, MAXB = P["RT_AMBIENT"], P["RT_BIAS"], P["RT_MAX_BOUNCES"]
SKY_LO = [P["RT_SKY_LO_R"], P["RT_SKY_LO_G"], P["RT_SKY_LO_B"]]
SKY_HI = [P["RT_SKY_HI_R"], P["RT_SKY_HI_G"], P["RT_SKY_HI_B"]]
CAM = [P["RT_CAM_X"], P["RT_CAM_Y"], P["RT_CAM_Z"]]
D00 = [P["RT_D00_X"], P["RT_D00_Y"], P["RT_D00_Z"]]
DXV = [P["RT_DX_X"], P["RT_DX_Y"], P["RT_DX_Z"]]
DYV = [P["RT_DY_X"], P["RT_DY_Y"], P["RT_DY_Z"]]
SO = [[P[f"RT_SO{k}_{a}"] for a in "XYZ"] for k in range(4)]

def make_rays_cam():
    W, H = CAMERA["width"], CAMERA["height"]
    rays = []
    for py in range(H):
        row = [s32(D00[c] + py * DYV[c]) for c in range(3)]
        for px in range(W):
            rays.append([s32(row[c] + px * DXV[c]) for c in range(3)])
    return rays

stats = {"closest": 0, "shadow": 0, "hits": 0}

# ---- Ray queries ------------------------------------------------------
def closest(o, d):
    """Closest hit; strict '<' so the lower triangle index wins ties."""
    stats["closest"] += 1
    best, best_t = -1, 0
    for ti, (i0, i1, i2) in enumerate(tris):
        hit, a, tn = intersect_parts(o, d, verts[i0], verts[i1], verts[i2])
        if hit:
            stats["hits"] += 1
            t = s32((tn << F) // a)
            if best < 0 or t < best_t:
                best, best_t = ti, t
    return best, best_t

def any_hit(o, d):
    """Shadow query: first hit wins, no division needed."""
    stats["shadow"] += 1
    for (i0, i1, i2) in tris:
        if intersect_parts(o, d, verts[i0], verts[i1], verts[i2])[0]:
            return True
    return False

# ---- Shading helpers --------------------------------------------------
def sky_fx(d):
    k = min(ONE, max(0, (d[1] + ONE) >> 1))
    return [s32(SKY_LO[c] + mul(SKY_HI[c] - SKY_LO[c], k)) for c in range(3)]

def to8(x):
    return min(255, max(0, x) >> 8)

def trace_pixel(d):
    o = list(CAM)
    acc = [0, 0, 0]
    w = ONE
    for depth in range(MAXB + 1):
        ti, t = closest(o, d)
        if ti < 0:
            s = sky_fx(d)
            acc = [s32(acc[c] + mul(w, s[c])) for c in range(3)]
            break

        p = [s32(o[c] + mul(d[c], t)) for c in range(3)]
        n = NORM[ti] if dot(d, NORM[ti]) < 0 else [s32(-x) for x in NORM[ti]]
        so = [s32(p[c] + mul(n[c], BIAS)) for c in range(3)]

        bl0 = any_hit(so, L)
        bl1 = any_hit(so, L1)
        d0 = 0 if bl0 else max(0, dot(n, L))
        d1 = 0 if bl1 else max(0, dot(n, L1))
        dsum = min(ONE, s32(d0 + d1))
        light = s32(AMB + mul(ONE - AMB, dsum))

        r, g, b, refl, flag, r2, g2, b2, spec = MAT[ti][:9]
        if flag != 0 and (((p[0] >> F) ^ (p[2] >> F)) & 1):
            r, g, b = r2, g2, b2

        dn2 = s32(2 * dot(d, n))
        R = [s32(d[c] - mul(dn2, n[c])) for c in range(3)]
        s0 = 0 if bl0 else max(0, dot(R, L))
        s1 = 0 if bl1 else max(0, dot(R, L1))
        for _ in range(3):                       # ^8
            s0 = mul(s0, s0)
            s1 = mul(s1, s1)
        sp = mul(spec, s32(s0 + s1))

        local = [s32(mul(r, light) + sp), s32(mul(g, light) + sp), s32(mul(b, light) + sp)]

        reflect = refl > 0 and depth < MAXB
        wl = mul(w, ONE - refl) if reflect else w
        acc = [s32(acc[c] + mul(wl, local[c])) for c in range(3)]
        if not reflect:
            break
        d = R
        o = so
        w = mul(w, refl)
    return acc

def pixel_aa(d):
    sums = [0, 0, 0]
    for k in range(4):
        dk = [s32(d[c] + SO[k][c]) for c in range(3)]
        a = trace_pixel(dk)
        sums = [sums[c] + a[c] for c in range(3)]
    return (to8(sums[0] >> 2) << 16) | (to8(sums[1] >> 2) << 8) | to8(sums[2] >> 2)

# ---- Main -------------------------------------------------------------
if __name__ == "__main__":
    W, H = CAMERA["width"], CAMERA["height"]
    pix = []
    for n, d in enumerate(make_rays_cam()):
        pix.append(pixel_aa(list(d)))
        if (n + 1) % W == 0:
            progress(n + 1, W * H, "Fixed RT")

    with open("golden_rt.txt", "w") as f:
        for p in pix:
            f.write(f"{p:06x}\n")

    print(f"Closest-hit rays: {stats['closest']}  (primary {W*H}, "
          f"reflection {stats['closest'] - W*H})")
    print(f"Shadow rays: {stats['shadow']}")
    print(f"Triangle hits needing a divide: {stats['hits']}")
    print(f"Triangle tests (closest, full scan): {stats['closest'] * len(tris)}")
    print("Wrote golden_rt.txt")

    img_fx = np.array([[(p >> 16) & 255, (p >> 8) & 255, p & 255] for p in pix],
                      dtype=np.int32).reshape(H, W, 3)
    try:
        flt = np.load("rt_float.npy")
    except FileNotFoundError:
        print("rt_float.npy not found: run rt_float.py after the patch to compare.")
        flt = None

    if flt is not None:
        img_fl = np.minimum(255, np.floor(np.clip(flt, 0, 1) * 256)).astype(np.int32)
        diff = np.abs(img_fx - img_fl).max(axis=2)
        mse = np.mean((img_fx - img_fl) ** 2)
        psnr = 99.0 if mse == 0 else 10 * np.log10(255 ** 2 / mse)
        print(f"Fixed vs float: mean |diff| = {np.abs(img_fx - img_fl).mean():.3f}/255, "
              f"max = {diff.max()}, PSNR = {psnr:.1f} dB")
        print(f"Pixels differing by more than 8/255: {(diff > 8).sum()} "
              f"({100 * (diff > 8).mean():.2f}%)")

        fig, ax = plt.subplots(1, 3, figsize=(15, 5))
        ax[0].imshow(img_fl.astype(np.uint8)); ax[0].set_title("Float")
        ax[1].imshow(img_fx.astype(np.uint8)); ax[1].set_title("Fixed-point golden")
        ax[2].imshow(diff, cmap="gray_r", vmax=64); ax[2].set_title("Max channel diff")
        for a in ax: a.axis("off")
        plt.tight_layout(); plt.savefig("golden_rt.png"); plt.show()
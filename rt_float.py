# rt_float.py -- floating-point ray tracer: the target image
import numpy as np
import matplotlib.pyplot as plt
from scene import (VERTICES as V, TRIANGLES as T, CAMERA, LIGHT_DIR, AMBIENT,
                   MAX_BOUNCES, TRI_COLOR, TRI_REFL, BIAS, SKY_LO, SKY_HI,
                   CAM_POS, camera_basis, TRI_CHECK, TRI_COLOR2, AA_OFFSETS, TRI_SPEC, LIGHT1_DIR, LIGHT1_I,
                   LIGHT0_I)
from golden import progress

T_MIN = 1e-4

E1 = V[T[:, 1]] - V[T[:, 0]]
E2 = V[T[:, 2]] - V[T[:, 0]]
N = np.cross(E1, E2)
N /= np.linalg.norm(N, axis=1, keepdims=True)
L = LIGHT0_I * LIGHT_DIR / np.linalg.norm(LIGHT_DIR)
L1 = LIGHT1_I * LIGHT1_DIR / np.linalg.norm(LIGHT1_DIR)

stats = {"primary": 0, "shadow": 0, "reflect": 0}

def intersect(o, d, any_hit=False):
    best, best_t = -1, np.inf
    for ti in range(len(T)):
        e1, e2 = E1[ti], E2[ti]
        h = np.cross(d, e2)
        a = e1 @ h
        if abs(a) < 1e-9:
            continue
        s = o - V[T[ti, 0]]
        u = (s @ h) / a
        if u < 0 or u > 1:
            continue
        q = np.cross(s, e1)
        v = (d @ q) / a
        if v < 0 or u + v > 1:
            continue
        t = (e2 @ q) / a
        if t > T_MIN and t < best_t:
            best, best_t = ti, t
            if any_hit:
                return best, best_t
    return best, best_t

def sky(d):
    k = min(1.0, max(0.0, 0.5 * (d[1] + 1.0)))
    return (1 - k) * np.array(SKY_LO) + k * np.array(SKY_HI)

def trace(o, d, depth):
    ti, t = intersect(o, d)
    if ti < 0:
        return sky(d)
    p = o + d * t

    base = TRI_COLOR[ti]
    if TRI_CHECK[ti] and (int(np.floor(p[0])) + int(np.floor(p[2]))) % 2 != 0:
        base = TRI_COLOR2[ti]

    n = N[ti] if N[ti] @ d < 0 else -N[ti]
    so = p + n * BIAS
    stats["shadow"] += 2
    sh0 = intersect(so, L,  any_hit=True)[0] >= 0
    sh1 = intersect(so, L1, any_hit=True)[0] >= 0
    d0 = 0.0 if sh0 else max(0.0, n @ L)
    d1 = 0.0 if sh1 else max(0.0, n @ L1)
    diffuse = min(1.0, d0 + d1)
    col = base * (AMBIENT + (1 - AMBIENT) * diffuse)

    R = d - 2 * (d @ n) * n
    s0 = 0.0 if sh0 else max(0.0, R @ L)
    s1 = 0.0 if sh1 else max(0.0, R @ L1)
    col = col + TRI_SPEC[ti] * (s0 ** 8 + s1 ** 8)

    r = TRI_REFL[ti]
    if r > 0 and depth < MAX_BOUNCES:
        stats["reflect"] += 1
        col = (1 - r) * col + r * trace(so, R, depth + 1)
    return col

if __name__ == "__main__":
    W, H = CAMERA["width"], CAMERA["height"]
    hw = CAMERA["fov_half_width"]
    step = 2 * hw / W
    hh = step * H / 2
    o = CAM_POS.copy()
    r_, up_, f_ = camera_basis()
    img = np.zeros((H, W, 3))
    for py in range(H):
        for px in range(W):
            acc = np.zeros(3)
            for ox, oy in AA_OFFSETS:
                d = ((-hw + step * (px + 0.5 + ox)) * r_
                     + (hh - step * (py + 0.5 + oy)) * up_ + f_)
                stats["primary"] += 1
                acc += trace(o, d, 0)
            img[py, px] = acc / len(AA_OFFSETS)
        progress(py + 1, H, "Ray tracing")
    img = np.clip(img, 0, 1)
    np.save("rt_float.npy", img)
    print(f"Rays: {stats['primary']} primary, {stats['shadow']} shadow, "
          f"{stats['reflect']} reflection")
    plt.imsave("rt_float.png", img)
    plt.imshow(img); plt.title("Float ray tracer"); plt.axis("off"); plt.show()
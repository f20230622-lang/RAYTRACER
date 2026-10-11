# scene.py
import json
import numpy as np
from meshes import load_obj, icosphere, fit_mesh, clean_faces

FRAC_BITS = 16          # Q16.16
WORD_BITS = 32

# ---- Settings -----------------------------------------------------------
RES = 64                # image is RES x RES (use 32 for quick sims)
MESH = "icosphere"      # None, "icosphere", or a path like "C:/Users/khnay/Desktop/Ray/bunny.obj"
MESH_SUBDIV = 1         # icosphere: 0 = 20 tris, 1 = 80, 2 = 320
MESH_COLOR = (0.95, 0.65, 0.15)
MESH_REFL = 0.25
FLOOR_REFL = 0.2
FLOOR_Y = -1.8

# ---- Geometry helpers ---------------------------------------------------
def rot_x(deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def rot_y(deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def make_cube(center, size, ry_deg, rx_deg):
    """8 vertices; index = (x>0) + 2*(y>0) + 4*(z>0)."""
    h = size / 2
    local = np.array([[sx * h, sy * h, sz * h]
                      for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)])
    R = rot_x(rx_deg) @ rot_y(ry_deg)
    return local @ R.T + np.array(center)

CUBE_QUADS = [
    (0, 2, 3, 1),  # -z
    (4, 5, 7, 6),  # +z
    (0, 4, 6, 2),  # -x
    (1, 3, 7, 5),  # +x
    (0, 1, 5, 4),  # -y
    (2, 6, 7, 3),  # +y
]

# ---- Parts: (vertices, triangles, per-triangle color, per-triangle reflectivity) ----
parts = []

# Cube, resting on the floor, left of center
cube_v = make_cube((0.0, 0.0, 0.0), 1.6, 30, 20)
cube_v = cube_v + np.array([-1.3, FLOOR_Y - cube_v[:, 1].min(), 6.0])
cube_t = []
for a, b, c, d in CUBE_QUADS:
    cube_t += [[a, b, c], [a, c, d]]
_face = [(0.9, 0.2, 0.2), (0.2, 0.8, 0.2), (0.2, 0.3, 0.9),
         (0.9, 0.8, 0.2), (0.8, 0.3, 0.8), (0.2, 0.8, 0.8)]
parts.append((cube_v, np.array(cube_t, dtype=np.int64),
              np.array([c for c in _face for _ in range(2)]), np.full(12, 0.10)))

# Mesh, right of center
if MESH is not None:
    if MESH == "icosphere":
        mv, mf = icosphere(MESH_SUBDIV)
    else:
        mv, mf = load_obj(MESH)
    mv = fit_mesh(mv, size=2.2, cx=1.4, cz=5.5, floor_y=FLOOR_Y)
    mf = clean_faces(mv, mf)
    parts.append((mv, mf, np.tile(MESH_COLOR, (len(mf), 1)), np.full(len(mf), MESH_REFL)))

# Floor (two triangles), last so it keeps the highest indices
floor_v = np.array([
    [-8.0, FLOOR_Y,  1.5],
    [ 8.0, FLOOR_Y,  1.5],
    [ 8.0, FLOOR_Y, 15.0],
    [-8.0, FLOOR_Y, 15.0],
])
parts.append((floor_v, np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int64),
              np.tile((0.75, 0.75, 0.75), (2, 1)), np.full(2, FLOOR_REFL)))

# ---- Assemble ----------------------------------------------------------
_v, _t, _c, _r, _off = [], [], [], [], 0
for V_, F_, C_, R_ in parts:
    _v.append(V_); _t.append(F_ + _off); _c.append(C_); _r.append(R_)
    _off += len(V_)
VERTICES = np.vstack(_v)
TRIANGLES = np.vstack(_t).astype(np.int64)
TRI_COLOR = np.vstack(_c)
TRI_REFL = np.concatenate(_r)

# Checker floor: the last 2 triangles are the floor
TRI_CHECK = np.zeros(len(TRIANGLES))
TRI_CHECK[-2:] = 1.0
TRI_COLOR2 = TRI_COLOR.copy()
TRI_COLOR2[-2:] = (0.20, 0.22, 0.30)

# Specular strength: cube moderate, mesh shiny, floor matte
TRI_SPEC = np.zeros(len(TRIANGLES))
TRI_SPEC[:12] = 0.3
TRI_SPEC[12:-2] = 0.8        # empty slice if MESH is None

# ---- Camera / image ---------------------------------------------------
CAMERA = {
    "origin": [0.0, 0.0, 0.0],
    "image_plane_z": 1.0,
    "fov_half_width": 0.5,
    "width": RES,
    "height": RES,
}

# Camera pose (look-at). Keep y > FLOOR_Y, outside the objects, don't aim straight up/down.
CAM_POS    = np.array([2.5, 0.6, 1.0])
CAM_TARGET = np.array([0.0, -0.8, 6.0])

def camera_basis():
    f = CAM_TARGET - CAM_POS
    f = f / np.linalg.norm(f)
    r = np.cross([0.0, 1.0, 0.0], f)
    r = r / np.linalg.norm(r)
    up = np.cross(f, r)
    return r, up, f

# ---- Lighting and materials ------------------------------------------
LIGHT_DIR  = np.array([-1.0, 1.0, -0.4])    # direction TOWARD light 1
LIGHT0_I   = 0.5
LIGHT1_DIR = np.array([ 1.0, 1.0, -0.4])    # direction TOWARD light 2
LIGHT1_I   = 0.5
AMBIENT = 0.15
MAX_BOUNCES = 2
BIAS = 0.004
SKY_LO = (0.9, 0.9, 1.0)
SKY_HI = (0.3, 0.5, 0.9)

# Anti-aliasing: 4 sub-pixel offsets (in pixels), rotated grid
AA_OFFSETS = [(-0.375, -0.125), (0.125, -0.375), (0.375, 0.125), (-0.125, 0.375)]

# ---- Fixed-point helpers ---------------------------------------------
def to_fixed(x: float) -> int:
    v = int(round(x * (1 << FRAC_BITS)))
    lo, hi = -(1 << (WORD_BITS - 1)), (1 << (WORD_BITS - 1)) - 1
    if not (lo <= v <= hi):
        raise OverflowError(f"{x} does not fit in Q{WORD_BITS-FRAC_BITS}.{FRAC_BITS}")
    return v

def to_hex(v: int) -> str:
    return f"{v & ((1 << WORD_BITS) - 1):08x}"

# ---- Export ----------------------------------------------------------
def export(outdir="."):
    with open(f"{outdir}/verts.mem", "w") as f:
        for v in VERTICES:
            for c in v:
                f.write(to_hex(to_fixed(c)) + "\n")
    with open(f"{outdir}/tris.mem", "w") as f:
        for t in TRIANGLES:
            for idx in t:
                f.write(f"{int(idx):08x}\n")
    cam = CAMERA
    with open(f"{outdir}/params.vh", "w") as f:
        f.write(f"localparam FRAC_BITS = {FRAC_BITS};\n")
        f.write(f"localparam NUM_VERTS = {len(VERTICES)};\n")
        f.write(f"localparam NUM_TRIS  = {len(TRIANGLES)};\n")
        f.write(f"localparam IMG_W     = {cam['width']};\n")
        f.write(f"localparam IMG_H     = {cam['height']};\n")
        f.write(f"localparam FOV_HW   = {to_fixed(cam['fov_half_width'])};\n")
    with open(f"{outdir}/config.json", "w") as f:
        json.dump({
            "frac_bits": FRAC_BITS,
            "camera": cam,
            "vertices": VERTICES.tolist(),
            "triangles": TRIANGLES.tolist(),
        }, f, indent=2)

def export_rt(outdir="."):
    E1 = VERTICES[TRIANGLES[:, 1]] - VERTICES[TRIANGLES[:, 0]]
    E2 = VERTICES[TRIANGLES[:, 2]] - VERTICES[TRIANGLES[:, 0]]
    N = np.cross(E1, E2)
    N /= np.linalg.norm(N, axis=1, keepdims=True)

    with open(f"{outdir}/normals.mem", "w") as f:
        for n in N:
            for c in n:
                f.write(to_hex(to_fixed(c)) + "\n")

    # mat.mem: 16 words per triangle:
    # r, g, b, refl, checker_flag, r2, g2, b2, spec, 7 x padding
    with open(f"{outdir}/mat.mem", "w") as f:
        for col, r, fl, c2, sp in zip(TRI_COLOR, TRI_REFL, TRI_CHECK, TRI_COLOR2, TRI_SPEC):
            for c in (*col, r, fl, *c2, sp, 0, 0, 0, 0, 0, 0, 0):
                f.write(to_hex(to_fixed(c)) + "\n")

    # Light vectors carry their intensity
    L0 = LIGHT0_I * LIGHT_DIR / np.linalg.norm(LIGHT_DIR)
    L1 = LIGHT1_I * LIGHT1_DIR / np.linalg.norm(LIGHT1_DIR)
    consts = {
        "RT_LIGHT_X": L0[0], "RT_LIGHT_Y": L0[1], "RT_LIGHT_Z": L0[2],
        "RT_L1_X": L1[0], "RT_L1_Y": L1[1], "RT_L1_Z": L1[2],
        "RT_AMBIENT": AMBIENT, "RT_BIAS": BIAS,
        "RT_SKY_LO_R": SKY_LO[0], "RT_SKY_LO_G": SKY_LO[1], "RT_SKY_LO_B": SKY_LO[2],
        "RT_SKY_HI_R": SKY_HI[0], "RT_SKY_HI_G": SKY_HI[1], "RT_SKY_HI_B": SKY_HI[2],
    }

    # Camera constants: pixel (0,0) direction, per-pixel and per-row steps
    r_, up_, f_ = camera_basis()
    W_, H_ = CAMERA["width"], CAMERA["height"]
    hw_ = CAMERA["fov_half_width"]
    st_ = 2 * hw_ / W_
    hh_ = st_ * H_ / 2
    DX_ = st_ * r_
    DY_ = -st_ * up_
    D00_ = (-hw_ + st_ / 2) * r_ + (hh_ - st_ / 2) * up_ + f_
    for nm, vec in (("CAM", CAM_POS), ("D00", D00_), ("DX", DX_), ("DY", DY_)):
        for ax, c in zip("XYZ", vec):
            consts[f"RT_{nm}_{ax}"] = c

    # Anti-aliasing sample offsets as direction vectors
    for k, (ox, oy) in enumerate(AA_OFFSETS):
        vec = ox * DX_ + oy * DY_
        for ax, c in zip("XYZ", vec):
            consts[f"RT_SO{k}_{ax}"] = c

    with open(f"{outdir}/rt_params.vh", "w") as f:
        for name, val in consts.items():
            f.write(f"localparam signed [31:0] {name} = 32'h{to_hex(to_fixed(val))};\n")
        f.write(f"localparam signed [31:0] RT_MAX_BOUNCES = 32'h{MAX_BOUNCES:08x};\n")

def report():
    e1 = VERTICES[TRIANGLES[:, 1]] - VERTICES[TRIANGLES[:, 0]]
    e2 = VERTICES[TRIANGLES[:, 2]] - VERTICES[TRIANGLES[:, 0]]
    area = 0.5 * np.linalg.norm(np.cross(e1, e2), axis=1)
    print(f"Scene: {len(VERTICES)} vertices, {len(TRIANGLES)} triangles, "
          f"min triangle area {area.min():.5f}")
    if area.min() < 0.004:
        print("WARNING: some triangles are very small for Q16.16 (hit test gets noisy). "
              "Scale the mesh up or use fewer triangles.")
    if len(TRIANGLES) > 4000:
        print("WARNING: more than ~4000 triangles overflows the 16-bit indices in rt_trace_s.")

if __name__ == "__main__":
    export()
    export_rt()
    report()
    n = len(TRIANGLES)
    print(f"Wrote verts.mem ({len(VERTICES)*3} words), tris.mem ({n*3} words), "
          f"normals.mem ({n*3} words), mat.mem ({n*16} words), "
          f"params.vh, rt_params.vh, config.json")
# meshes.py
import numpy as np

def load_obj(path):
    verts, faces = [], []
    for line in open(path, errors="ignore"):
        p = line.split()
        if not p:
            continue
        if p[0] == "v":
            verts.append([float(x) for x in p[1:4]])
        elif p[0] == "f":
            idx = []
            for tok in p[1:]:
                i = int(tok.split("/")[0])
                idx.append(i - 1 if i > 0 else len(verts) + i)
            for k in range(1, len(idx) - 1):          # fan triangulation
                faces.append([idx[0], idx[k], idx[k + 1]])
    return np.array(verts, dtype=np.float64), np.array(faces, dtype=np.int64)

def icosphere(subdiv=1):
    t = (1 + 5 ** 0.5) / 2
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t),
         (0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    f = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
         (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
         (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
         (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    V = [np.array(x, dtype=np.float64) / np.linalg.norm(x) for x in v]
    for _ in range(subdiv):
        cache = {}
        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in cache:
                m = V[a] + V[b]
                V.append(m / np.linalg.norm(m))
                cache[key] = len(V) - 1
            return cache[key]
        nf = []
        for a, b, c in f:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            nf += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        f = nf
    return np.array(V), np.array(f, dtype=np.int64)

def fit_mesh(V, size, cx, cz, floor_y, ry_deg=0.0):
    """Rotate about Y, scale so the largest extent == size, rest on floor_y, move to (cx, cz)."""
    a = np.radians(ry_deg); c, s = np.cos(a), np.sin(a)
    V = V @ np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]).T
    V = V - (V.max(0) + V.min(0)) / 2
    V = V * (size / (V.max(0) - V.min(0)).max())
    V[:, 1] += floor_y - V[:, 1].min()
    V[:, 0] += cx
    V[:, 2] += cz
    return V

def clean_faces(V, F, min_area=1e-6):
    e1 = V[F[:, 1]] - V[F[:, 0]]
    e2 = V[F[:, 2]] - V[F[:, 0]]
    area = 0.5 * np.linalg.norm(np.cross(e1, e2), axis=1)
    return F[area > min_area]
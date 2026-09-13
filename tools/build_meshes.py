#!/usr/bin/env python3
"""
Reconstruct per-link meshes for the Assemforchouk AS/RS cell.

The SolidWorks URDF export wrote the *entire assembly* into every link's STL
(5 x ~60 MB, 1.2 M triangles each, all containing the same 266 static parts).
This script:
  1. segments each STL into connected components (= physical parts),
  2. de-duplicates them in the base_link frame,
  3. assigns every part to the link it actually belongs to,
  4. decimates each part to a triangle budget proportional to its size,
  5. writes small per-link / per-material binary STLs.

Usage:  python3 build_meshes.py [--src DIR] [--out DIR] [--cache FILE]
"""
import argparse, os, pickle, struct, sys
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kin import link_pose

SRC_STL = {'base_link': 'base_link.STL', 'convoyeur': 'convoyeur.STL',
           'guidage': 'guidage.STL', 'axe x': 'axe x.STL', 'axe y': 'axe y.STL'}

# ---------------------------------------------------------------- STL binary I/O
def read_stl(path):
    with open(path, 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        d = np.fromfile(f, dtype=np.uint8, count=n * 50)
    if d.size != n * 50:
        raise IOError(f'{path}: truncated ({d.size} of {n*50} bytes)')
    d = d.reshape(n, 50)
    return np.frombuffer(d[:, 12:48].tobytes(), dtype='<f4').reshape(n, 3, 3).astype(np.float64)


def write_stl(path, tri, header=b'assemforchouk'):
    tri = np.asarray(tri, dtype=np.float64)
    n = len(tri)
    nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.divide(nrm, ln, out=np.zeros_like(nrm), where=ln > 1e-20)
    rec = np.zeros((n, 50), dtype=np.uint8)
    rec[:, 0:12] = np.ascontiguousarray(nrm.astype('<f4')).view(np.uint8).reshape(n, 12)
    rec[:, 12:48] = np.ascontiguousarray(tri.astype('<f4')).view(np.uint8).reshape(n, 36)
    with open(path, 'wb') as f:
        f.write(header.ljust(80, b'\0')[:80])
        f.write(struct.pack('<I', n))
        rec.tofile(f)
    return n

# ---------------------------------------------------------------- segmentation
def segment(tri):
    """Label triangles by connected component (shared-vertex connectivity)."""
    t = len(tri)
    q = np.round(tri.reshape(-1, 3), 5)
    _, inv = np.unique(q, axis=0, return_inverse=True)
    inv = np.asarray(inv).ravel().reshape(t, 3)
    nv = inv.max() + 1
    i = np.concatenate([inv[:, 0], inv[:, 1], inv[:, 2]])
    j = np.concatenate([inv[:, 1], inv[:, 2], inv[:, 0]])
    g = coo_matrix((np.ones(len(i), np.int8), (i, j)), shape=(nv, nv))
    _, lab = connected_components(g, directed=False)
    return lab[inv[:, 0]]

# ---------------------------------------------------------------- decimation
def cluster_decimate(tri, grid):
    """Vertex-clustering decimation: snap to `grid`, collapse, drop degenerates."""
    v = tri.reshape(-1, 3)
    cell = np.floor(v / grid).astype(np.int64)
    _, inv = np.unique(cell, axis=0, return_inverse=True)
    inv = np.asarray(inv).ravel()
    nrep = int(inv.max()) + 1
    rep = np.zeros((nrep, 3))
    cnt = np.zeros(nrep)
    np.add.at(rep, inv, v)
    np.add.at(cnt, inv, 1.0)
    rep /= cnt[:, None]
    f = inv.reshape(-1, 3)
    keep = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 2] != f[:, 0])
    f = f[keep]
    if len(f) == 0:
        return np.zeros((0, 3, 3))
    key = np.sort(f, axis=1)
    _, u = np.unique(key, axis=0, return_index=True)
    return rep[f[np.sort(u)]]


def decimate_to(tri, target):
    """Binary-search the clustering grid so the result lands near `target` triangles."""
    if len(tri) <= target:
        return tri
    p = tri.reshape(-1, 3)
    diag = float(np.linalg.norm(p.max(0) - p.min(0)))
    lo, hi, best = diag / 3000.0, diag / 2.0, None
    for _ in range(22):
        g = np.sqrt(lo * hi)
        out = cluster_decimate(tri, g)
        if len(out) > target:
            lo = g
        else:
            best, hi = out, g
        if hi / lo < 1.02:
            break
    return best if best is not None and len(best) >= 4 else tri

# ---------------------------------------------------------------- part classification
def classify(ntri, ext):
    """Map a static base_link part to a visual material group."""
    e = np.sort(ext)[::-1]
    long_axis = int(np.argmax(ext))            # 0=X 1=Y 2=Z
    if e[0] > 7.0:                              return 'floor'
    if ntri in (5822, 5072) and e[0] < 0.35:    return 'chain'
    if ntri == 2198 and 0.9 < e[0] < 1.2:       return 'drum'
    if ntri == 200 and abs(e[0] - 1.0) < 0.05:  return 'roller'
    if e[0] > 5.0:
        # lift columns run along Z, the roller-table frame runs along X
        return 'tower_frame' if long_axis == 2 else 'roller_frame'
    return 'drive'


# Parts present in the CAD that are dropped on purpose.  This one is a loose
# 0.32 x 0.70 x 0.10 plate lying on the roller table; it belongs to no link, so
# it just sat there statically in the middle of the carriage's path.
EXCLUDE = [((-2.172, -5.710, -1.417), (0.320, 0.700, 0.100), 'stray plate on the roller table')]


def excluded(ctr, ext):
    for c, e, _why in EXCLUDE:
        if np.allclose(ctr, c, atol=0.02) and np.allclose(ext, e, atol=0.02):
            return True
    return False


def sig(ctr, ext):
    """Spatial signature used to de-duplicate a part seen in several source meshes."""
    q = np.floor(np.concatenate([ctr, ext]) * 200.0 + 0.5) / 200.0   # 5 mm bins
    return tuple(q)


# moving parts, identified by (centre, extent) in the base_link frame.
# The values come from SolidWorks mass properties cross-checked against the mesh
# bounding boxes, which agree to about 1 cm -- hence the tolerance match below.
MOVING_PARTS = [
    ((-2.163, -5.723, -1.437), (0.500, 0.900, 0.200), 'convoyeur', 'platform + pushers'),
    ((-2.303, -6.065, -1.402), (0.220, 0.010, 0.070), 'guidage',   'pusher blade A'),
    ((-2.023, -6.065, -1.402), (0.220, 0.010, 0.070), 'guidage',   'pusher blade B'),
    ((-1.483, -3.818, -1.458), (4.100, 0.800, 0.210), 'axe y',     'shelf / comptoir'),
    ((-2.713, -3.810, -1.503), (1.280, 0.700, 0.100), 'axe y',     'tray on shelf'),
]

# triangle budget: the moving parts are what the user watches, so they get more
DETAIL = {'convoyeur': 6000, 'guidage': 2000, 'axe y': 2000}

# The CAD only fits the compartment tray to 1.28 m of the 4.1 m comptoir. Repeat
# it along +X so the whole shelf is divided into package slots.
TRAY = ((-2.713, -3.810, -1.503), (1.280, 0.700, 0.100))
TRAY_PITCH = 1.280
TRAY_COPIES = 3          # 1.28 -> 3.84 m of the 4.10 m deck


def owner_of(ctr, ext):
    for c, e, link, _lbl in MOVING_PARTS:
        if np.allclose(ctr, c, atol=0.02) and np.allclose(ext, e, atol=0.02):
            return link
    return None


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument('--src', default=os.path.abspath(os.path.join(here, '..', '..', 'meshes')))
    ap.add_argument('--out', default=os.path.abspath(os.path.join(here, '..', 'src',
                                                    'assemforchouk_sim', 'meshes')))
    ap.add_argument('--cache', default=os.path.join(here, '.seg_cache.pkl'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    # ---- 1. segment every source mesh (cached: the pass costs ~1 min) ----------
    if os.path.exists(a.cache):
        store = pickle.load(open(a.cache, 'rb'))
        print(f'[cache] {a.cache}')
    else:
        store = {}
        for link, fn in SRC_STL.items():
            tri = read_stl(os.path.join(a.src, fn))
            lab = segment(tri)
            R, t = (np.eye(3), np.zeros(3)) if link == 'base_link' else link_pose(link)
            store[link] = dict(lab=lab, R=R, t=t, ntri=len(tri))
            print(f'  segmented {link:<10} {len(tri):>8} tris -> {lab.max()+1:>3} parts', flush=True)
        pickle.dump(store, open(a.cache, 'wb'))

    # ---- 2. gather every distinct part, in base_link frame --------------------
    parts, seen = [], {}
    for link, fn in SRC_STL.items():
        tri = read_stl(os.path.join(a.src, fn))
        st = store[link]
        triw = (tri.reshape(-1, 3) @ st['R'].T + st['t']).reshape(-1, 3, 3)
        for k in range(st['lab'].max() + 1):
            m = st['lab'] == k
            nt = int(m.sum())
            if nt < 4:
                continue
            p = triw[m].reshape(-1, 3)
            lo, hi = p.min(0), p.max(0)
            s = sig((lo + hi) / 2, hi - lo)
            if s in seen:
                continue
            seen[s] = True
            parts.append(dict(sig=s, tri=triw[m], ntri=nt, ctr=(lo + hi) / 2, ext=hi - lo))
    # bin edges can still split two views of the same part: merge within 5 mm
    kept = []
    n_drop = 0
    for p in parts:
        if any(np.allclose(p['ctr'], k['ctr'], atol=5e-3) and
               np.allclose(p['ext'], k['ext'], atol=5e-3) for k in kept):
            continue
        if excluded(p['ctr'], p['ext']):
            n_drop += 1
            continue
        kept.append(p)
    parts = kept
    if n_drop:
        print(f'dropped {n_drop} part(s) listed in EXCLUDE')
    print(f'\n{len(parts)} distinct physical parts recovered '
          f'({sum(p["ntri"] for p in parts)} triangles)')

    # ---- 3. assign parts to links / material groups ---------------------------
    groups = {}
    for p in parts:
        owner = owner_of(p['ctr'], p['ext'])
        key = (owner, 'part') if owner else ('base_link', classify(p['ntri'], p['ext']))
        groups.setdefault(key, []).append(p)

    # ---- 4. decimate + write ---------------------------------------------------
    OUT = {('base_link', 'floor'): 'base_floor', ('base_link', 'chain'): 'base_chain',
           ('base_link', 'drum'): 'base_drum', ('base_link', 'tower_frame'): 'base_tower',
           ('base_link', 'roller'): 'base_roller', ('base_link', 'roller_frame'): 'base_rollerframe',
           ('base_link', 'drive'): 'base_drive', ('convoyeur', 'part'): 'carriage',
           ('guidage', 'part'): 'pusher', ('axe y', 'part'): 'shelf'}
    LINK_OF = {'base_link': 'base_link', 'convoyeur': 'convoyeur',
               'guidage': 'guidage', 'axe y': 'axe y'}

    print(f'\n{"output":<20}{"parts":>7}{"src tris":>10}{"out tris":>10}{"KB":>8}')
    print('-' * 55)
    total_in = total_out = 0
    manifest = {}
    for key, plist in sorted(groups.items(), key=lambda kv: OUT.get(kv[0], 'zz')):
        name = OUT.get(key)
        if name is None:
            continue
        link = LINK_OF[key[0]]
        R, t = (np.eye(3), np.zeros(3)) if link == 'base_link' else link_pose(link)
        Rinv = R.T                                     # base frame -> link frame
        chunks = []
        for p in plist:
            diag = float(np.linalg.norm(p['ext']))
            target = DETAIL.get(key[0], int(np.clip(120.0 * diag / 0.30, 60, 4000)))
            d = decimate_to(p['tri'], target)
            # tile the compartment tray across the full length of the comptoir
            is_tray = (np.allclose(p['ctr'], TRAY[0], atol=0.02) and
                       np.allclose(p['ext'], TRAY[1], atol=0.02))
            shifts = [i * TRAY_PITCH for i in range(TRAY_COPIES)] if is_tray else [0.0]
            for dx in shifts:
                v = d.reshape(-1, 3).copy()
                v[:, 0] += dx
                chunks.append((v - t) @ Rinv.T)
        tri = np.concatenate(chunks).reshape(-1, 3, 3)
        fp = os.path.join(a.out, name + '.stl')
        n = write_stl(fp, tri, header=f'assemforchouk:{name}'.encode())
        src_n = sum(p['ntri'] for p in plist)
        total_in += src_n
        total_out += n
        kb = os.path.getsize(fp) / 1024
        manifest[name] = dict(link=link, parts=len(plist), tris=n)
        print(f'{name+".stl":<20}{len(plist):>7}{src_n:>10}{n:>10}{kb:>8.0f}')
    print('-' * 55)
    print(f'{"TOTAL":<20}{"":>7}{total_in:>10}{total_out:>10}'
          f'{sum(os.path.getsize(os.path.join(a.out,f)) for f in os.listdir(a.out))/1024:>8.0f}')
    print(f'\nreduction: {total_in/max(total_out,1):.0f}x fewer triangles')
    pickle.dump(manifest, open(os.path.join(a.out, '.manifest.pkl'), 'wb'))


if __name__ == '__main__':
    main()

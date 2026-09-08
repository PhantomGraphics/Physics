"""Generate the hand-written background stage + synthetic cube map used by
PhysicsView's glTF rendering (docs/todo/PLAN_physicsview_gltf_rendering.md,
Phase 1).

No Blender, no external deps -- a hand-written binary glTF plus a minimal PNG
writer, mirroring CGApp/FluidStudio/scenarios/models/gen_baseline_assets.py and
Phantom/CGLib/GltfViewer/scenarios/models/gen_test_models.py.

Outputs (committed alongside this script):
  stage.glb          a floor + 2 walls + one obstacle block, flat-shaded, one
                     PBR material (~120 verts). PhysicsView's FluidRenderer
                     orbits (20,20,20); the floor sits at y=0 loosely under the
                     default fluid domain.
  env/{right,left,top,bottom,front,back}.png   a synthetic studio cube map for
                     SetEnvironment (SSFR reflections + the SSFR-mode skybox).

Run:  python gen_render_assets.py
"""
import json
import os
import struct
import zlib

OUT = os.path.dirname(os.path.abspath(__file__))


def box(cx, cy, cz, sx, sy, sz):
    """Axis-aligned box centred at (cx,cy,cz) with full sizes (sx,sy,sz);
    per-face flat normals. Returns (positions, normals, indices)."""
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5
    faces = [
        ([(-hx, -hy,  hz), ( hx, -hy,  hz), ( hx,  hy,  hz), (-hx,  hy,  hz)], ( 0,  0,  1)),
        ([( hx, -hy, -hz), (-hx, -hy, -hz), (-hx,  hy, -hz), ( hx,  hy, -hz)], ( 0,  0, -1)),
        ([( hx, -hy,  hz), ( hx, -hy, -hz), ( hx,  hy, -hz), ( hx,  hy,  hz)], ( 1,  0,  0)),
        ([(-hx, -hy, -hz), (-hx, -hy,  hz), (-hx,  hy,  hz), (-hx,  hy, -hz)], (-1,  0,  0)),
        ([(-hx,  hy,  hz), ( hx,  hy,  hz), ( hx,  hy, -hz), (-hx,  hy, -hz)], ( 0,  1,  0)),
        ([(-hx, -hy, -hz), ( hx, -hy, -hz), ( hx, -hy,  hz), (-hx, -hy,  hz)], ( 0, -1,  0)),
    ]
    pos, nrm, idx = [], [], []
    for corners, n in faces:
        base = len(pos)
        for c in corners:
            pos.append((cx + c[0], cy + c[1], cz + c[2]))
            nrm.append(n)
        idx += [base, base + 1, base + 2, base, base + 2, base + 3]
    return pos, nrm, idx


PARTS = [
    box(20,  -1,  20,  90,   2, 90),   # floor (top at y=0)
    box(20,  22, -25,  90,  48,  2),   # back wall
    box(-25, 22,  20,   2,  48, 90),   # left wall
    box(34,   6,  34,  12,  12, 12),   # obstacle block
]

positions, normals, indices = [], [], []
for p, n, i in PARTS:
    off = len(positions)
    positions += p
    normals += n
    indices += [x + off for x in i]

pos_bytes = b"".join(struct.pack("<3f", *v) for v in positions)
nrm_bytes = b"".join(struct.pack("<3f", *v) for v in normals)
idx_bytes = b"".join(struct.pack("<I", v) for v in indices)


def pad4(b):
    return b + b"\x00" * ((4 - len(b) % 4) % 4)


pos_bytes, nrm_bytes, idx_bytes = pad4(pos_bytes), pad4(nrm_bytes), pad4(idx_bytes)
bin_blob = pos_bytes + nrm_bytes + idx_bytes

pmin = [min(v[i] for v in positions) for i in range(3)]
pmax = [max(v[i] for v in positions) for i in range(3)]

gltf = {
    "asset": {"version": "2.0", "generator": "gen_render_assets.py"},
    "scene": 0,
    "scenes": [{"nodes": [0]}],
    "nodes": [{"mesh": 0, "name": "stage"}],
    "materials": [{
        "name": "stage",
        "pbrMetallicRoughness": {
            "baseColorFactor": [0.70, 0.70, 0.73, 1.0],
            "metallicFactor": 0.0,
            "roughnessFactor": 0.85,
        },
    }],
    "meshes": [{
        "name": "stage",
        "primitives": [{
            "attributes": {"POSITION": 0, "NORMAL": 1},
            "indices": 2,
            "material": 0,
        }],
    }],
    "accessors": [
        {"bufferView": 0, "componentType": 5126, "count": len(positions),
         "type": "VEC3", "min": pmin, "max": pmax},
        {"bufferView": 1, "componentType": 5126, "count": len(normals), "type": "VEC3"},
        {"bufferView": 2, "componentType": 5125, "count": len(indices), "type": "SCALAR"},
    ],
    "bufferViews": [
        {"buffer": 0, "byteOffset": 0, "byteLength": len(pos_bytes), "target": 34962},
        {"buffer": 0, "byteOffset": len(pos_bytes), "byteLength": len(nrm_bytes), "target": 34962},
        {"buffer": 0, "byteOffset": len(pos_bytes) + len(nrm_bytes),
         "byteLength": len(idx_bytes), "target": 34963},
    ],
    "buffers": [{"byteLength": len(bin_blob)}],
}

_json_raw = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
json_blob = _json_raw + b" " * ((4 - len(_json_raw) % 4) % 4)

glb = struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(json_blob) + 8 + len(bin_blob))
glb += struct.pack("<II", len(json_blob), 0x4E4F534A) + json_blob
glb += struct.pack("<II", len(bin_blob), 0x004E4942) + bin_blob

with open(os.path.join(OUT, "stage.glb"), "wb") as f:
    f.write(glb)
print(f"stage.glb  verts={len(positions)} tris={len(indices)//3} bytes={len(glb)}")

# ---------------------------------------------------------------------------
# env/{right,left,top,bottom,front,back}.png -- a synthetic studio cube map.
# Not a real HDRI; a soft vertical gradient per face so the SSFR skybox +
# reflections have something plausible to sample without shipping a photo.
# ---------------------------------------------------------------------------
_SZ = 128
_FACES = {
    "top":    ((150, 178, 214), (120, 150, 190)),
    "bottom": ((70,  70,  74),  (52,  52,  56)),
    "right":  ((150, 178, 214), (168, 158, 140)),
    "left":   ((150, 178, 214), (168, 158, 140)),
    "front":  ((150, 178, 214), (168, 158, 140)),
    "back":   ((150, 178, 214), (168, 158, 140)),
}


def _write_png(fp, w, h, rows):
    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data +
                struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))
    raw = b"".join(b"\x00" + bytes(r) for r in rows)
    fp.write(b"\x89PNG\r\n\x1a\n")
    fp.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
    fp.write(chunk(b"IDAT", zlib.compress(raw, 9)))
    fp.write(chunk(b"IEND", b""))


env_dir = os.path.join(OUT, "env")
os.makedirs(env_dir, exist_ok=True)
for name, (top, bot) in _FACES.items():
    rows = []
    for y in range(_SZ):
        t = y / (_SZ - 1)
        px = bytes(int(round(top[c] * (1 - t) + bot[c] * t)) for c in range(3)) * _SZ
        rows.append(px)
    with open(os.path.join(env_dir, name + ".png"), "wb") as f:
        _write_png(f, _SZ, _SZ, rows)
print(f"env/  6 x {_SZ}x{_SZ} synthetic cube faces")

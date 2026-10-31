"""Engrave the Tinker robot-face logo into the left finger body.

    pip install manifold3d numpy
    python engrave_tinker_logo.py

Reads cad/left_body.stl and writes cad/left_body_tinker_logo.stl. The plain
body and its STEP file are unchanged; the engraving is cosmetic only.

Where: the outer face (printed on the bed, Z = 0) behind the 50 mm wide-zone
rail. That face is outside when mounted, away from the pads and the bolts.
The antenna points toward the mount, so the face reads upright when the
fingers point down. The shapes come from tinker.robot_face(): the head,
ears and antenna are cut in, and the eyes and mouth are left standing.
"""
from pathlib import Path
import struct

import numpy as np
from manifold3d import CrossSection, Manifold, Mesh

HERE = Path(__file__).resolve().parent
SRC = HERE / 'cad' / 'left_body.stl'
OUT = HERE / 'cad' / 'left_body_tinker_logo.stl'

LOGO_SIZE = 30.0            # mm across the 32-unit logo grid
CENTRE = (68.0, 55.0)       # print-frame XY: middle of the wide rail's outer face
DEPTH = 0.6                 # 3 layers at 0.2 mm; the rail is 8.45 mm thick
CIRCLE_SEGMENTS = 64

# tinker.robot_face(32): screen units, y down. The antenna line is drawn with
# width 2 (size / 16), so it becomes a 2-unit-wide rectangle here.
CUT = [('rect', (15, 2, 17, 7)),        # antenna
       ('oval', (13.5, 0.5, 18.5, 5.5)),  # antenna light
       ('rect', (2, 13, 4, 21)),        # ears
       ('rect', (28, 13, 30, 21)),
       ('rect', (4, 7, 28, 29))]        # head
KEEP = [('oval', (9, 13, 14, 18)),      # eyes
        ('oval', (18, 13, 23, 18)),
        ('rect', (11, 22, 21, 24))]     # mouth


def to_print(h, v):
    """Logo units -> print XY. Seen from below (the outer face), so X is mirrored;
    screen-up (antenna) maps to -Y, toward the mounting plate."""
    u = LOGO_SIZE / 32
    return CENTRE[0] - (h - 16) * u, CENTRE[1] + (v - 16) * u


def shape(kind, box):
    (xa, ya), (xb, yb) = to_print(box[0], box[1]), to_print(box[2], box[3])
    x0, x1, y0, y1 = min(xa, xb), max(xa, xb), min(ya, yb), max(ya, yb)
    if kind == 'rect':
        return CrossSection.square((x1 - x0, y1 - y0)).translate((x0, y0))
    r = (x1 - x0) / 2
    return CrossSection.circle(r, CIRCLE_SEGMENTS).translate(((x0 + x1) / 2, (y0 + y1) / 2))


def read_stl(path):
    b = path.read_bytes()
    n = struct.unpack('<I', b[80:84])[0]
    tri = np.frombuffer(b[84:], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]), count=n)
    verts = tri['v'].reshape(-1, 3).astype(np.float64)
    uniq, inv = np.unique(np.round(verts, 5), axis=0, return_inverse=True)
    return Manifold(Mesh(vert_properties=uniq.astype(np.float32),
                         tri_verts=inv.reshape(-1, 3).astype(np.uint32)))


def write_stl(path, m, name):
    mesh = m.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3]
    t = np.asarray(mesh.tri_verts)
    with open(path, 'wb') as f:
        f.write(name.encode()[:80].ljust(80, b' '))
        f.write(struct.pack('<I', len(t)))
        for a, b, c in v[t]:
            n = np.cross(b - a, c - a)
            n = n / (np.linalg.norm(n) or 1)
            f.write(struct.pack('<12fH', *n, *a, *b, *c, 0))
    return len(t)


def main():
    body = read_stl(SRC)
    assert body.status().name == 'NoError', body.status()
    logo = CrossSection.batch_boolean([shape(*s) for s in CUT], _op('Add'))
    logo = logo - CrossSection.batch_boolean([shape(*s) for s in KEEP], _op('Add'))
    cutter = Manifold.extrude(logo, DEPTH + 1.0).translate((0, 0, -1.0))
    out = body - cutter
    assert out.status().name == 'NoError', out.status()
    removed = body.volume() - out.volume()
    n = write_stl(OUT, out, 'CSI dual-zone V2.1 left body + Tinker logo (mm)')
    b = out.bounding_box()
    print(f'{OUT.name}: {n} triangles, genus {out.genus()}, bbox {np.round(b, 2).tolist()}, '
          f'logo {logo.area():.0f} mm^2, removed {removed:.0f} mm^3 '
          f'(expected {logo.area() * DEPTH:.0f})')


def _op(name):
    from manifold3d import OpType
    return getattr(OpType, name)


if __name__ == '__main__':
    main()

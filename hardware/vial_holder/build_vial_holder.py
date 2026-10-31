"""Printable vial holder matching the simulator's vial destination (dual_v2).

Pure Python, no CAD library needed. Writes binary STLs, an OpenSCAD source
and a top-view SVG into this folder:

    python build_vial_holder.py

Geometry (mm). Origin = holder centre on the table, vial axis along +X.
It copies scan_sort_bins.vial_stops() for FINGER_SET=dual_v2:
  - two slots 80 mm apart (sim slots y = 0.35 / 0.43 m)
  - four stops per slot, 8 (X) x 4 (Y) x 10 (Z) mm, centres at
    x = +/-20 mm and y = slot +/- (vial radius + 4.5 mm)
    -> 2.5 mm side clearance to the 27.18 mm vial
A thin frame ties the stops together. It has an open window under each vial,
so the vial rests on the table exactly as in the simulator.
"""
from pathlib import Path
import struct

OUT = Path(__file__).resolve().parent

VIAL_D, VIAL_L = 27.18, 203.0
R = VIAL_D / 2
SLOTS_Y = (-40.0, 40.0)
STOP_X, STOP_SIZE = 20.0, (8.0, 4.0, 10.0)   # height measured from the table
STOP_Y = R + 4.5
PLATE_T = 1.6          # 3.4 mm under the 5 mm fingertip pick height
WIN_X = 105.5          # window half-length: 4 mm past each vial end
WIN_Y = 16.0           # window half-width about each slot
END_BAR = 4.0
OUT_X = WIN_X + END_BAR
OUT_Y = 62.0
NOTCH_W, NOTCH_D = 1.5, 2.0   # alignment notches (slot lines, centre line)


def boxes():
    """(kind, x0, x1, y0, y1): solid plate regions, holes and stops."""
    yield 'plate', -OUT_X, OUT_X, -OUT_Y, OUT_Y
    for y in SLOTS_Y:
        yield 'hole', -WIN_X, WIN_X, y - WIN_Y, y + WIN_Y
        # slot-line notches in both end bars
        for s in (-1, 1):
            x_out = s * OUT_X
            yield 'hole', min(x_out, x_out - s * NOTCH_D), max(x_out, x_out - s * NOTCH_D), \
                y - NOTCH_W / 2, y + NOTCH_W / 2
    # centre-line notches (x = 0) in both side rails
    for s in (-1, 1):
        y_out = s * OUT_Y
        yield 'hole', -NOTCH_W / 2, NOTCH_W / 2, min(y_out, y_out - s * NOTCH_D), \
            max(y_out, y_out - s * NOTCH_D)
    sx, sy, _ = STOP_SIZE
    for y in SLOTS_Y:
        for ex in (-1, 1):
            for ey in (-1, 1):
                cx, cy = ex * STOP_X, y + ey * STOP_Y
                yield 'stop', cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2


def height_field(x_min=-OUT_X):
    """Rectilinear grid; each cell gets 0, PLATE_T or the stop height."""
    bs = list(boxes())
    xs = sorted({v for b in bs for v in b[1:3]} | {x_min})
    xs = [x for x in xs if x >= x_min]
    ys = sorted({v for b in bs for v in b[3:5]})

    def h(cx, cy):
        inside = lambda b: b[1] < cx < b[2] and b[3] < cy < b[4]
        if any(b[0] == 'stop' and inside(b) for b in bs):
            return STOP_SIZE[2]
        if any(b[0] == 'hole' and inside(b) for b in bs):
            return 0.0
        return PLATE_T if any(b[0] == 'plate' and inside(b) for b in bs) else 0.0

    H = [[h((xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2) for j in range(len(ys) - 1)]
         for i in range(len(xs) - 1)]
    return xs, ys, H


def mesh(xs, ys, H):
    """Closed, consistently wound triangle mesh of the height field."""
    levels = sorted({0.0} | {v for row in H for v in row})
    nx, ny = len(H), len(H[0])
    get = lambda i, j: H[i][j] if 0 <= i < nx and 0 <= j < ny else 0.0
    tris = []

    def quad(a, b, c, d):          # counter-clockwise seen from outside
        tris.extend([(a, b, c), (a, c, d)])

    for i in range(nx):
        for j in range(ny):
            hz = H[i][j]
            if hz <= 0:
                continue
            x0, x1, y0, y1 = xs[i], xs[i + 1], ys[j], ys[j + 1]
            quad((x0, y0, hz), (x1, y0, hz), (x1, y1, hz), (x0, y1, hz))      # top
            quad((x0, y0, 0), (x0, y1, 0), (x1, y1, 0), (x1, y0, 0))          # bottom
    # vertical walls, split at every level so no T-junctions remain
    def bands(lo, hi):
        ls = [v for v in levels if lo <= v <= hi]
        return zip(ls, ls[1:])
    for i in range(nx + 1):            # walls on x = xs[i]
        x = xs[i]
        for j in range(ny):
            a, b = get(i - 1, j), get(i, j)       # left, right cell
            y0, y1 = ys[j], ys[j + 1]
            for z0, z1 in bands(min(a, b), max(a, b)):
                if a > b:   # solid on the left: face +X
                    quad((x, y0, z0), (x, y1, z0), (x, y1, z1), (x, y0, z1))
                else:       # face -X
                    quad((x, y0, z0), (x, y0, z1), (x, y1, z1), (x, y1, z0))
    for j in range(ny + 1):            # walls on y = ys[j]
        y = ys[j]
        for i in range(nx):
            a, b = get(i, j - 1), get(i, j)
            x0, x1 = xs[i], xs[i + 1]
            for z0, z1 in bands(min(a, b), max(a, b)):
                if a > b:   # face +Y
                    quad((x0, y, z0), (x0, y, z1), (x1, y, z1), (x1, y, z0))
                else:       # face -Y
                    quad((x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1))
    return tris


def check(tris):
    """Every directed edge must be matched by its reverse exactly once."""
    from collections import Counter
    e = Counter()
    for t in tris:
        for k in range(3):
            e[(t[k], t[(k + 1) % 3])] += 1
    bad = [k for k, n in e.items() if n != 1 or e.get((k[1], k[0]), 0) != 1]
    vol = sum((a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
               + a[2] * (b[0] * c[1] - b[1] * c[0])) for a, b, c in tris) / 6
    return len(bad), vol


def write_stl(path, tris, name):
    with open(path, 'wb') as f:
        f.write(name.encode()[:80].ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        for a, b, c in tris:
            u = [b[k] - a[k] for k in range(3)]
            v = [c[k] - a[k] for k in range(3)]
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            m = sum(q * q for q in n) ** .5 or 1
            f.write(struct.pack('<12fH', *(q / m for q in n), *a, *b, *c, 0))


def write_scad(path):
    rows = []
    for kind, x0, x1, y0, y1 in boxes():
        z = {'plate': (-0, PLATE_T), 'hole': (-1, PLATE_T + 2), 'stop': (0, STOP_SIZE[2])}[kind]
        rows.append((kind, f'translate([{x0:.3f},{y0:.3f},{z[0]}]) cube([{x1-x0:.3f},{y1-y0:.3f},{z[1]-z[0]}]);'))
    body = '\n'.join(f'      {r}' for k, r in rows if k == 'plate')
    holes = '\n'.join(f'      {r}' for k, r in rows if k == 'hole')
    stops = '\n'.join(f'    {r}' for k, r in rows if k == 'stop')
    path.write_text(f"""// Generated by build_vial_holder.py -- edit the Python parameters, then rerun.
// Units mm. Origin = holder centre, vial axis along +X.
// half = true gives one of two identical halves (cut at x = 0).
half = false;
intersection() {{
  union() {{
    difference() {{
      union() {{
{body}
      }}
{holes}
    }}
{stops}
  }}
  if (half) translate([0,-200,-1]) cube([400,400,50]);
  else translate([-200,-200,-1]) cube([400,400,50]);
}}
""")


def write_svg(path, xs, ys, H):
    s, pad = 3.0, 20
    W, Hh = (2 * OUT_X) * s + 2 * pad, (2 * OUT_Y) * s + 2 * pad
    X = lambda x: pad + (x + OUT_X) * s
    Y = lambda y: pad + (OUT_Y - y) * s
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{Hh+40:.0f}" '
           f'viewBox="0 0 {W:.0f} {Hh+40:.0f}" font-family="sans-serif" font-size="12">',
           f'<rect width="100%" height="100%" fill="white"/>']
    for i in range(len(H)):
        for j in range(len(H[0])):
            hz = H[i][j]
            if hz <= 0:
                continue
            col = '#2a7f8a' if hz > PLATE_T else '#9fc7cc'
            out.append(f'<rect x="{X(xs[i]):.2f}" y="{Y(ys[j+1]):.2f}" width="{(xs[i+1]-xs[i])*s:.2f}" '
                       f'height="{(ys[j+1]-ys[j])*s:.2f}" fill="{col}"/>')
    for y in SLOTS_Y:   # vial outlines
        out.append(f'<rect x="{X(-VIAL_L/2):.1f}" y="{Y(y+R):.1f}" width="{VIAL_L*s:.1f}" height="{VIAL_D*s:.1f}" '
                   f'rx="6" fill="none" stroke="#c0392b" stroke-dasharray="6 4" stroke-width="1.5"/>')
    out.append(f'<text x="{pad}" y="{Hh+20:.0f}">Top view. Dark = 10 mm stops, light = {PLATE_T} mm frame, '
               f'dashed = Ø{VIAL_D} × {VIAL_L:.0f} mm vials. Overall {2*OUT_X:.0f} × {2*OUT_Y:.0f} mm.</text>')
    out.append('</svg>')
    path.write_text('\n'.join(out), encoding='utf-8')


def main():
    report = {}
    for name, x_min in (('vial_holder_full', -OUT_X), ('vial_holder_half_quantity_2', 0.0)):
        xs, ys, H = height_field(x_min)
        tris = mesh(xs, ys, H)
        bad, vol = check(tris)
        assert bad == 0, f'{name}: {bad} unmatched edges'
        write_stl(OUT / 'cad' / f'{name}.stl', tris, f'CSI {name} (mm)')
        size = (xs[-1] - xs[0], ys[-1] - ys[0], max(max(r) for r in H))
        report[name] = (len(tris), size, vol)
        if x_min < 0:
            write_svg(OUT / 'top_view.svg', xs, ys, H)
    write_scad(OUT / 'cad' / 'vial_holder.scad')
    for k, (n, size, vol) in report.items():
        print(f'{k}: {n} triangles, {size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} mm, '
              f'{vol/1000:.1f} cm^3, watertight')


if __name__ == '__main__':
    (OUT / 'cad').mkdir(exist_ok=True)
    main()

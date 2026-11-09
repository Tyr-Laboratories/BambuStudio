"""stretch.py GCODE Z X Y [REF_GCODE]: the Bridge path of the layer at Z that ends at (X, Y); its free stretch from that end (centre line
farther than w/2 + 0.01 mm from every bead of the layer below), sampled every mm: the nearest bead of the layer below and of the layer
above at each sample, and in REF_GCODE (Cubic) what the layer at Z and the layer below print there."""
import os as _os
BAC_DATA = _os.environ.get('BAC_DATA', '/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data')
BAC_WORK = _os.environ.get('BAC_WORK', BAC_DATA + '/work')
_TOOLING = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
import sys
sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import gcr as G, fp
from shapely.geometry import LineString, Point
from shapely.ops import unary_union, substring
L = G.parse(sys.argv[1]); z = float(sys.argv[2]); x, y = float(sys.argv[3]), float(sys.argv[4])
R = G.parse(sys.argv[5]) if len(sys.argv) > 5 else None
zs = sorted(L); i = min(range(len(zs)), key=lambda k: abs(zs[k] - z)); zz, zb, za = zs[i], zs[i - 1], zs[i + 1]
end = Point(x, y)
cands = [p for p in L[zz] if p[0] == 'Bridge' and min(Point(p[3][0]).distance(end), Point(p[3][-1]).distance(end)) < 0.05]
if not cands:
    print('no bridge path ends there'); sys.exit()
feat, w, o, pts, e = cands[0]
ls = LineString(pts)
if Point(pts[-1]).distance(end) < Point(pts[0]).distance(end):
    ls = LineString(list(reversed(pts)))
below = [p for p in L[zb] if len(p[3]) > 1]
F = unary_union([g for g in (fp.path_geom(p[3], p[1]) for p in below) if g is not None]).buffer(w / 2 + 0.01)
# The free stretch from the end: walk along until the line is held.
step = 0.1; d = 0.0
while d < ls.length and not F.contains(ls.interpolate(d)):
    d += step
print(f'path of {ls.length:.1f} mm, width {w}; free from the end for {d:.2f} mm; end ({x}, {y}); stretch to {tuple(round(v, 2) for v in ls.interpolate(d).coords[0])}')
def nearest(layer_paths, pt):
    best = None
    for f2, w2, o2, p2, e2 in layer_paths:
        if len(p2) < 2: continue
        dd = LineString(p2).distance(pt)
        if best is None or dd < best[0]: best = (dd, f2)
    return best
above = L[za]
for s in [k * 1.0 for k in range(0, int(d) + 1)]:
    pt = ls.interpolate(s)
    nb = nearest(below, pt); na = nearest(above, pt)
    line = f'  {s:4.1f} mm at ({pt.x:.2f}, {pt.y:.2f}): below {nb[1]} {nb[0]:.2f}; above {na[1]} {na[0]:.2f}'
    if R is not None:
        rz = sorted(R); ri = min(range(len(rz)), key=lambda k: abs(rz[k] - z))
        rc = nearest([p for p in R[rz[ri]] if len(p[3]) > 1], pt); rb = nearest([p for p in R[rz[ri - 1]] if len(p[3]) > 1], pt)
        line += f'; Cubic here {rc[1]} {rc[0]:.2f}, below {rb[1]} {rb[0]:.2f}'
    print(line)

"""runs.py GCODE Z [N]: the N longest free stretches of the Bridge paths of the layer at Z over the beads of the layer below (centre line
farther than w/2 + 0.01 mm from them), with their ends, and what the layer below prints within 1 mm of their middle."""
import os as _os
BAC_DATA = _os.environ.get('BAC_DATA', '/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data')
BAC_WORK = _os.environ.get('BAC_WORK', BAC_DATA + '/work')
_TOOLING = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
import sys
sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import gcr as G, fp
from shapely.geometry import LineString, Point
from shapely.ops import unary_union
L = G.parse(sys.argv[1]); z = float(sys.argv[2]); n = int(sys.argv[3]) if len(sys.argv) > 3 else 3
zs = sorted(L); i = min(range(len(zs)), key=lambda k: abs(zs[k] - z)); zz, zb = zs[i], zs[i - 1]
mine = [p for p in L[zz] if p[0] == 'Bridge']
below = L[zb]
F = unary_union([g for g in (fp.path_geom(p[3], p[1]) for p in below) if g is not None])
out = []
for feat, w, o, pts, e in mine:
    ls = LineString(pts); free = ls.difference(F.buffer(w / 2 + 0.01))
    for g in getattr(free, 'geoms', [free]):
        if not g.is_empty and g.length > 5: out.append((g.length, g))
out.sort(key=lambda t: -t[0])
print(f'layer {zz} over {zb}: {len(out)} free stretches over 5 mm')
for length, g in out[:n]:
    c = list(g.coords); mid = g.interpolate(0.5, normalized=True)
    near = sorted({(round(LineString(p[3]).distance(mid), 2), p[0], p[1]) for p in below if len(p[3]) > 1 and LineString(p[3]).distance(mid) < 3})[:6]
    print(f'  {length:.2f} mm from ({c[0][0]:.2f}, {c[0][1]:.2f}) to ({c[-1][0]:.2f}, {c[-1][1]:.2f}); below within 3 mm of its middle: {near}')

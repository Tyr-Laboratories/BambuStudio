"""near4.py GCODE Z X Y R: the extrusions within R mm of (X, Y) on the layer above Z, the layer at Z, and the two below: feature, width,
distance, path length."""
import os as _os
BAC_DATA = _os.environ.get('BAC_DATA', '/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data')
BAC_WORK = _os.environ.get('BAC_WORK', BAC_DATA + '/work')
_TOOLING = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
import sys
sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import gcr as G
from shapely.geometry import LineString, Point
L = G.parse(sys.argv[1]); z = float(sys.argv[2]); x, y, r = float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
zs = sorted(L); i = min(range(len(zs)), key=lambda k: abs(zs[k] - z))
pt = Point(x, y)
for k in (i + 1, i, i - 1, i - 2):
    if k < 0 or k >= len(zs): continue
    out = []
    for feat, w, o, pts, e in L[zs[k]]:
        if len(pts) < 2: continue
        ls = LineString(pts); d = ls.distance(pt)
        if d <= r: out.append((round(d, 2), feat, round(w, 3), round(ls.length, 1)))
    feats = {}
    for d, f, w, l in sorted(out):
        feats.setdefault(f, []).append((d, l))
    print(f'  layer {zs[k]}: ' + '; '.join(f'{f} x{len(v)} nearest {v[0][0]} mm (path {v[0][1]} mm)' for f, v in sorted(feats.items(), key=lambda t: t[1][0][0])))

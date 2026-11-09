"""near.py GCODE Z X Y R: the extrusions of the layer at Z (and the layer below) within R mm of (X, Y): feature, width, distance."""
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
for zz in (zs[i], zs[i - 1]):
    print(f'layer {zz}:')
    pt = Point(x, y)
    out = []
    for feat, w, o, pts, e in L[zz]:
        if len(pts) < 2: continue
        ls = LineString(pts); d = ls.distance(pt)
        if d <= r: out.append((round(d, 3), feat, w, round(ls.length, 2)))
    for item in sorted(out)[:14]: print('  ', item)

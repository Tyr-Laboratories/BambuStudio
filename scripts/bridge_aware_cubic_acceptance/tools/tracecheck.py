"""tracecheck.py DUMP_DIR GCODE Z X Y [mode]: for the Bridge path of the layer at Z that ends at (X, Y) (mode 'bridge'), or for the
point and a short neighbourhood (mode 'point'), which areas of the measurement dumps hold its free stretch: on its own layer and the
layer below, the sparse fill that the bridges took (taken_bridge, taken_solid), the predicted ensuring strips left sparse, and the solid
laid that the plan did not predict."""
import os as _os
BAC_DATA = _os.environ.get('BAC_DATA', '/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data')
BAC_WORK = _os.environ.get('BAC_WORK', BAC_DATA + '/work')
_TOOLING = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
import sys, os, glob, collections
sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import gcr as G, fp
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
dump_dir, gcode, z, x, y = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
mode = sys.argv[6] if len(sys.argv) > 6 else 'bridge'
areas = collections.defaultdict(list)
for f in glob.glob(os.path.join(dump_dir, '*.txt')):
    for line in open(f):
        parts = line.split()
        if len(parts) < 5: continue
        kind, zz = parts[0], round(float(parts[1]), 2)
        pts = [tuple(map(float, p.split(','))) for p in parts[2:]]
        try:
            poly = Polygon(pts).buffer(0)
        except Exception:
            continue
        areas[(kind, zz)].append(poly)
areas = {k: unary_union(v) for k, v in areas.items()}
L = G.parse(gcode)
zs = sorted(L); i = min(range(len(zs)), key=lambda k: abs(zs[k] - z)); zz, zb = round(zs[i], 2), round(zs[i - 1], 2)
end = Point(x, y)
if mode == 'bridge':
    cands = [p for p in L[zs[i]] if p[0] == 'Bridge' and min(Point(p[3][0]).distance(end), Point(p[3][-1]).distance(end)) < 0.05]
    feat, w, o, pts, e = cands[0]
    ls = LineString(pts if Point(pts[0]).distance(end) < Point(pts[-1]).distance(end) else list(reversed(pts)))
    below = [p for p in L[zs[i - 1]] if len(p[3]) > 1]
    F = unary_union([g for g in (fp.path_geom(p[3], p[1]) for p in below) if g is not None]).buffer(w / 2 + 0.01)
    d = 0.0
    while d < ls.length and not F.contains(ls.interpolate(d)):
        d += 0.1
    samples = [ls.interpolate(s) for s in [k * 0.5 for k in range(int(d / 0.5) + 1)]]
else:
    samples = [end]
    d = 0.0
kinds = ['taken_bridge', 'taken_solid', 'predicted_left_sparse', 'solid_not_predicted', 'demand']
for layer, name in ((zz, 'own layer'), (zb, 'layer below')):
    counts = collections.Counter()
    near = collections.Counter()
    for pt in samples:
        for k in kinds:
            a = areas.get((k, layer))
            if a is None: continue
            if a.contains(pt): counts[k] += 1
            elif a.distance(pt) < 0.5: near[k] += 1
    print(f'  {name} ({layer}): of {len(samples)} samples along {d:.1f} mm, inside ' + ', '.join(f'{k} {counts[k]}' for k in kinds)
          + '; within 0.5 mm ' + ', '.join(f'{k} {near[k]}' for k in kinds))

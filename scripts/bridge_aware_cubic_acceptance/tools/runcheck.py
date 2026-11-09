"""runcheck.py DUMP_DIR GCODE Z X Y: the sparse infill path of the layer at Z through (X, Y), its unsupported piece there (centre line
farther than w/2 + 0.01 mm from every bead of the layer below), sampled every 0.5 mm, against the measurement dumps on its own layer and
the layer below, and the nearest bead of the layer below at each sample."""
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
areas = collections.defaultdict(list)
for f in glob.glob(os.path.join(dump_dir, '*.txt')):
    for line in open(f):
        parts = line.split()
        if len(parts) < 5: continue
        pts = [tuple(map(float, p.split(','))) for p in parts[2:]]
        try: areas[(parts[0], round(float(parts[1]), 2))].append(Polygon(pts).buffer(0))
        except Exception: pass
areas = {k: unary_union(v) for k, v in areas.items()}
L = G.parse(gcode)
zs = sorted(L); i = min(range(len(zs)), key=lambda k: abs(zs[k] - z)); zz, zb = round(zs[i], 2), round(zs[i - 1], 2)
pt0 = Point(x, y)
paths = [p for p in L[zs[i]] if p[0] == 'Sparse infill' and len(p[3]) > 1 and LineString(p[3]).distance(pt0) < 0.05]
if not paths:
    print('no sparse path there'); sys.exit()
feat, w, o, pts, e = paths[0]
ls = LineString(pts)
below = [p for p in L[zs[i - 1]] if len(p[3]) > 1]
F = unary_union([g for g in (fp.path_geom(p[3], p[1]) for p in below) if g is not None]).buffer(w / 2 + 0.01)
free = ls.difference(F)
pieces = [g for g in getattr(free, 'geoms', [free]) if not g.is_empty]
piece = min(pieces, key=lambda g: g.distance(pt0))
print(f'path of {ls.length:.2f} mm; unsupported piece of {piece.length:.2f} mm from {tuple(round(v,2) for v in piece.coords[0])} to {tuple(round(v,2) for v in piece.coords[-1])}')
kinds = ['taken_bridge', 'taken_solid', 'predicted_left_sparse', 'solid_not_predicted', 'demand']
samples = [piece.interpolate(s) for s in [k * 0.5 for k in range(int(piece.length / 0.5) + 1)]]
for layer, name in ((zz, 'own layer'), (zb, 'layer below')):
    inside = collections.Counter(); near = collections.Counter()
    for pt in samples:
        for k in kinds:
            a = areas.get((k, layer))
            if a is None: continue
            if a.contains(pt): inside[k] += 1
            elif a.distance(pt) < 0.5: near[k] += 1
    print(f'  {name} ({layer}): of {len(samples)} samples, inside ' + ', '.join(f'{k} {inside[k]}' for k in kinds) + '; within 0.5 mm ' +
          ', '.join(f'{k} {near[k]}' for k in kinds))
def nearest(paths, pt):
    best = None
    for f2, w2, o2, p2, e2 in paths:
        dd = LineString(p2).distance(pt)
        if best is None or dd < best[0]: best = (dd, f2)
    return best
for s, pt in zip([k * 0.5 for k in range(len(samples))], samples):
    nb = nearest(below, pt)
    print(f'    {s:4.1f} mm at ({pt.x:.2f}, {pt.y:.2f}): nearest below {nb[1]} {nb[0]:.2f}')

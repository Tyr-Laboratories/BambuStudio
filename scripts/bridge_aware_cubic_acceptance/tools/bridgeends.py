"""K1 on the internal bridges of a plate: on every layer and object where bridge_over_infill made a bridge (FINAL records of the Cubic
dump), the free stretches of each G-code's Bridge paths over the beads below (centre line farther than w/2 + 0.01 mm from them) and its
hanging ends. Usage: bridgecheck.py plate OUT.json label=GCODE [label=GCODE ...] (Batch BX-fix copy of bridgecheck2.py)"""
import os as _os
BAC_DATA = _os.environ.get('BAC_DATA', '/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data')
BAC_WORK = _os.environ.get('BAC_WORK', BAC_DATA + '/work')
_TOOLING = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
import sys, os, collections, json
sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import gcr as G, fp, dump as D
from shapely.geometry import LineString, Point
from shapely.ops import unary_union
R = BAC_WORK + '/bx/runs'
plate = sys.argv[1]
objs = D.load_run('base', 'cubic', plate)
single = len(objs) == 1
want = set()
for label, o in objs.items():
    for a, g in o.recs['FINAL']:
        want.add((round(float(a['z']), 3), -1 if single else label))
res = {}
for spec in sys.argv[3:]:
    label, path = spec.split('=', 1)
    Lg = G.parse(path)
    zs = sorted(Lg); prev = {zs[i]: zs[i - 1] for i in range(1, len(zs))}
    over5 = 0; runs = []; hang = []; npaths = 0
    for z, obj in sorted(want):
        if z not in prev: continue
        mine = [p for p in Lg.get(z, []) if p[0] == 'Bridge' and p[2] == obj]
        if not mine: continue
        below = [p for p in Lg.get(prev[z], []) if p[2] == obj]
        box = unary_union([LineString(p[3]).buffer(1.0) for p in mine]).envelope
        F = unary_union([g for g in (fp.path_geom(p[3], p[1]) for p in below) if g is not None and g.intersects(box)])
        for feat, w, o, pts, e in mine:
            npaths += 1
            ls = LineString(pts); held = F.buffer(w / 2 + 0.01)
            free = ls.difference(held)
            parts = [g for g in getattr(free, 'geoms', [free]) if not g.is_empty and g.length > 0]
            for g in parts:
                runs.append((round(g.length, 2), z))
                if g.length > 5.0: over5 += 1
            for end in (Point(pts[0]), Point(pts[-1])):
                if not held.contains(end):
                    hang.append((round(max([g.length for g in parts if g.distance(end) < 1e-6] or [0.0]), 2), z))
                    print(f'  {label} hanging end z {z} at ({end.x:.2f}, {end.y:.2f}), free {hang[-1][0]} mm, path of {round(ls.length, 2)} mm', flush=True)
    runs.sort(reverse=True); hang.sort(reverse=True)
    res[label] = dict(paths=npaths, over5=over5, longest=runs[:5], hanging=len(hang), hang_longest=hang[:5])
    print(f'plate {plate} {label}: {len(want)} bridge layers; {npaths} bridge paths; free stretches over 5 mm: {over5}; longest {runs[:5]}; hanging ends {len(hang)}, longest {hang[:5]}')


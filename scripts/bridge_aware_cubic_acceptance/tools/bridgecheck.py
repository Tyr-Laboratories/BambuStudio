"""K1 on the internal bridges of a plate, as the export observer measures it (count_bridge_ends). This fork prints internal and
external bridges with one role ("Bridge"), so a bridge path is internal when its midpoint lies inside the outline of the layer below
(the area its outer walls enclose, holes excluded). The sparse void below is that outline less every bead of the layer below, the
parts of it beside sparse fill. For each G-code: the free stretches of every internal bridge path (centre line farther than
w/2 + 0.01 mm from the beads below) over that void, and its hanging ends there that no wall of the bridge's own layer anchors.
Usage: bridgecheck.py plate OUT.json label=GCODE [label=GCODE ...]"""
import os as _os
import sys, json
from functools import reduce
sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import gcr as G, fp
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

WALLS = ('Outer wall', 'Overhang wall')

def outline(paths, obj):
    """The area the object's outer-wall loops enclose on a layer, even-odd: consecutive wall pieces that meet end to start are one
    loop; a loop whose ends lie within 2 mm closes."""
    loops, cur = [], None
    for feat, w, o, pts, e in paths:
        if o != obj or feat not in WALLS:
            if cur is not None: loops.append(cur); cur = None
            continue
        if cur is not None and abs(cur[-1][0] - pts[0][0]) < 1e-3 and abs(cur[-1][1] - pts[0][1]) < 1e-3:
            cur.extend(pts[1:])
        else:
            if cur is not None: loops.append(cur)
            cur = list(pts)
    if cur is not None: loops.append(cur)
    rings = []
    for pts in loops:
        if len(pts) >= 4 and ((pts[0][0] - pts[-1][0]) ** 2 + (pts[0][1] - pts[-1][1]) ** 2) ** 0.5 <= 2.0:
            poly = shapely.make_valid(Polygon(pts))
            if not poly.is_empty and poly.area > 0:
                rings.append(poly)
    if not rings:
        return None
    return reduce(lambda a, b: a.symmetric_difference(b), rings)

res = {}
plate = sys.argv[1]
for spec in sys.argv[3:]:
    label, path = spec.split('=', 1)
    Lg = G.parse(path)
    zs = sorted(Lg); prev = {zs[i]: zs[i - 1] for i in range(1, len(zs))}
    layers = 0; over5 = 0; runs = []; hang = []; npaths = 0
    for z in zs:
        if z not in prev: continue
        bridges = [p for p in Lg[z] if p[0] == 'Bridge']
        if not bridges: continue
        below_all = Lg.get(prev[z], [])
        for obj in sorted(set(p[2] for p in bridges)):
            area = outline(below_all, obj)
            if area is None: continue
            mine = []
            for p in bridges:
                if p[2] != obj or len(p[3]) < 2: continue
                ls = LineString(p[3])
                if ls.length > 0 and area.contains(ls.interpolate(0.5, normalized=True)):
                    mine.append(p)
            if not mine: continue
            layers += 1
            below = [p for p in below_all if p[2] == obj]
            box = unary_union([LineString(p[3]).buffer(1.0) for p in mine]).envelope
            geoms = [(p, fp.path_geom(p[3], p[1])) for p in below]
            geoms = [(p, g) for p, g in geoms if g is not None and g.intersects(box)]
            F = unary_union([g for p, g in geoms])
            sparse = unary_union([g for p, g in geoms if p[0] == 'Sparse infill'])
            void = area.intersection(box).difference(F)
            sparse_void = unary_union([g for g in getattr(void, 'geoms', [void])
                                       if g.area > 0 and not sparse.is_empty and g.distance(sparse) < 0.01])
            walls = unary_union([g for g in (fp.path_geom(p[3], p[1]) for p in Lg[z] if p[2] == obj and p[0] in WALLS + ('Inner wall',))
                                 if g is not None and g.intersects(box)])
            for feat, w, o, pts, e in mine:
                npaths += 1
                ls = LineString(pts); held = F.buffer(w / 2 + 0.01)
                anchored = walls.buffer(w / 2 + 0.01) if not walls.is_empty else walls
                free = ls.difference(held)
                parts = [g for g in getattr(free, 'geoms', [free])
                         if not g.is_empty and g.length > 0 and sparse_void.contains(g.interpolate(0.5, normalized=True))]
                for g in parts:
                    mid = g.interpolate(0.5, normalized=True)
                    runs.append((round(g.length, 2), z, round(mid.x, 2), round(mid.y, 2)))
                    if g.length > 5.0: over5 += 1
                for end in (Point(pts[0]), Point(pts[-1])):
                    if not held.contains(end) and sparse_void.contains(end) and not anchored.contains(end):
                        hang.append((round(max([g.length for g in parts if g.distance(end) < 1e-6] or [0.0]), 2), z, round(end.x, 2), round(end.y, 2)))
    runs.sort(reverse=True); hang.sort(reverse=True)
    res[label] = dict(layers=layers, paths=npaths, over5=over5, longest=runs[:10], hanging=len(hang), hang_longest=hang[:10])
    print(f'plate {plate} {label}: {layers} internal bridge layers; {npaths} internal bridge paths; free stretches over 5 mm: {over5}; '
          f'longest {runs[:5]}; hanging ends {len(hang)}, longest {hang[:5]}')
json.dump(res, open(sys.argv[2], 'w'))

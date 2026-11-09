"""Footprints: union of each feature's beads at a layer, from gc.parse output."""
import shapely
from shapely.geometry import LineString, Polygon, MultiPolygon
from shapely.ops import unary_union
import collections

def path_geom(pts, w):
    if len(pts) < 2:
        return None
    ls = LineString(pts)
    if ls.length <= 0:
        return None
    return ls.buffer(w / 2, cap_style='flat', join_style='mitre', mitre_limit=2.0)

def footprints(paths, objs=None, feats=None):
    """paths: list of (feature, width, obj, pts, e). Returns feature -> geometry (union)."""
    groups = collections.defaultdict(list)
    for feat, w, obj, pts, e in paths:
        if objs is not None and obj not in objs: continue
        if feats is not None and feat not in feats: continue
        g = path_geom(pts, w)
        if g is not None and not g.is_empty:
            groups[feat].append(g)
    return {k: unary_union(v) for k, v in groups.items()}

def lengths(paths, objs=None):
    out = collections.defaultdict(lambda: [0.0, 0.0, 0])
    for feat, w, obj, pts, e in paths:
        if objs is not None and obj not in objs: continue
        L = sum(((pts[i+1][0]-pts[i][0])**2 + (pts[i+1][1]-pts[i][1])**2) ** 0.5 for i in range(len(pts)-1))
        r = out[feat]; r[0] += L; r[1] += e; r[2] += 1
    return dict(out)

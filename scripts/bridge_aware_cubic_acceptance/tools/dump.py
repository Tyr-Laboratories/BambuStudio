"""Reader for the BX probe dumps (runs/<tag>/<kind>/p<N>/dump/obj_<label>.txt)."""
import os as _os
BAC_DATA = _os.environ.get('BAC_DATA', '/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data')
BAC_WORK = _os.environ.get('BAC_WORK', BAC_DATA + '/work')
_TOOLING = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
import os, re, glob, collections, pickle
import shapely
from shapely import wkt as W
from shapely.ops import unary_union
from shapely.geometry import MultiPolygon, Polygon, GeometryCollection

ST = {0: 'top', 1: 'bottom', 2: 'bottombridge', 3: 'internal', 4: 'fvs', 5: 'isolid', 6: 'subtop', 7: 'ibridge', 8: 'ivoid', 9: 'perimeter'}
ROLE = {4: 'sparse', 5: 'isolid', 6: 'fvs', 7: 'top', 8: 'bottom', 10: 'bridge', 11: 'gap'}
KV = re.compile(r'(\w+)=("[^"]*"|\S+)')

def _geom(text):
    g = W.loads(text)
    if g.geom_type in ('MultiPolygon', 'Polygon'):
        if not g.is_valid:
            g = shapely.make_valid(g)
            g = unary_union([p for p in getattr(g, 'geoms', [g]) if p.geom_type in ('Polygon', 'MultiPolygon')])
    return g

class Obj:
    def __init__(self, path):
        self.path = path
        self.name = ''; self.inst = []
        self.recs = collections.defaultdict(list)   # tag -> list of (attrs, geom-text)
        with open(path) as f:
            for line in f:
                line = line.rstrip('\n')
                if line.startswith('OBJ '):
                    m = re.search(r'name="([^"]*)"', line); self.name = m.group(1) if m else ''
                    self.inst = [(int(a), float(b), float(c)) for a, b, c in re.findall(r'inst=(\d+):([-0-9.e]+),([-0-9.e]+)', line)]
                    continue
                tag, rest = line.split(' ', 1)
                i = rest.find('MULTI')
                head, g = (rest[:i], rest[i:]) if i >= 0 else (rest, None)
                attrs = {k: v for k, v in KV.findall(head)}
                self.recs[tag].append((attrs, g))
        self._cache = {}
    def label(self):
        return self.inst[0][0] if self.inst else None
    def shift(self):
        return (self.inst[0][1], self.inst[0][2]) if self.inst else (0.0, 0.0)
    def layers(self, tag):
        return sorted(set(int(a['L']) for a, g in self.recs[tag]))
    def z_of(self, L):
        for tag in ('PRE', 'FIN', 'GF', 'POST'):
            for a, g in self.recs[tag]:
                if int(a['L']) == L: return round(float(a['z']), 3)
        return None
    def get(self, tag, L=None, **flt):
        """Union of the geometries of records with tag (and L and attribute filters)."""
        key = (tag, L, tuple(sorted(flt.items())))
        if key in self._cache: return self._cache[key]
        gs = []
        for a, g in self.recs[tag]:
            if L is not None and int(a['L']) != L: continue
            if any(str(a.get(k)) != str(v) for k, v in flt.items()): continue
            if g is None or g.endswith('EMPTY'): continue
            gs.append(_geom(g))
        out = unary_union(gs) if gs else GeometryCollection()
        self._cache[key] = out
        return out
    def attrs(self, tag, L=None):
        return [a for a, g in self.recs[tag] if L is None or int(a['L']) == L]
    def surf(self, stage, L, stype):
        """Union over regions of a stage's surfaces of one type name (ST values) at layer L."""
        code = [k for k, v in ST.items() if v == stype][0]
        return self.get(stage, L, type=code)
    def fill(self, L, role):
        code = [k for k, v in ROLE.items() if v == role][0]
        return self.get('GF', L, role=code)

def load_run(tag, kind, plate):
    base = BAC_WORK + f'/bx/runs/{tag}/{kind}/p{plate}/dump'
    objs = {}
    for p in sorted(glob.glob(base + '/obj_*.txt')):
        o = Obj(p)
        objs[o.label()] = o
    return objs

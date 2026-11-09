"""causes.py GCODE TSV OUT: for every free end that k2check.py found (TSV: z kind length x y path_length), what lies near it: the
nearest extrusion of each feature on its own layer and on the layer below (distance from the free end, mm), so that each can be
classed by place. Writes OUT (TSV) and prints a summary by kind and by the nearest non-sparse feature of the layer below."""
import collections, math, re, sys
import numpy as np
import shapely
from shapely import STRtree

AX = re.compile(r'([XYE])(-?[0-9]*\.?[0-9]+)')

def parse(path):
    layers = collections.defaultdict(list)   # z -> [(x0, y0, x1, y1, w, feature)]
    z = None; feat = ''; w = 0.4; x = y = 0.0
    with open(path, errors='replace') as f:
        for line in f:
            if line[0] == ';':
                if line.startswith('; Z_HEIGHT:'):
                    z = round(float(line.split(':')[1]), 3)
                elif line.startswith('; FEATURE:'):
                    feat = line[10:].strip()
                elif line.startswith('; LINE_WIDTH:'):
                    w = float(line.split(':')[1])
                continue
            if not line.startswith(('G0 ', 'G1 ')):
                continue
            d = dict((k, float(v)) for k, v in AX.findall(line.split(';')[0]))
            nx, ny = d.get('X', x), d.get('Y', y)
            if d.get('E', 0) > 0 and z is not None and (nx != x or ny != y):
                layers[z].append((x, y, nx, ny, w, feat))
            x, y = nx, ny
    return layers

def main():
    gcode, tsv, out = sys.argv[1:4]
    layers = parse(gcode)
    zs = sorted(layers)
    below = {zs[i]: zs[i - 1] for i in range(1, len(zs))}
    trees = {}
    def tree(z):
        if z not in trees:
            segs = layers.get(z, [])
            geoms = [shapely.LineString([(s[0], s[1]), (s[2], s[3])]) for s in segs]
            trees[z] = (STRtree(geoms) if geoms else None, geoms, segs)
        return trees[z]
    def nearest(z, p, skip_sparse):
        t, geoms, segs = tree(z)
        best = {}
        if t is None:
            return best
        for i in t.query(p.buffer(3.0)):
            f = segs[i][5]
            if skip_sparse and f == 'Sparse infill':
                continue
            d = geoms[i].distance(p) - segs[i][4] / 2
            if f not in best or d < best[f]:
                best[f] = d
        return best
    rows = []
    with open(tsv) as f:
        next(f)
        for line in f:
            z, kind, length, x, y, plen = line.split('\t')
            if not kind.startswith('free') and not kind.startswith('long'):
                continue
            z = round(float(z), 3); p = shapely.Point(float(x), float(y))
            zb = below.get(z)
            here = nearest(z, p, True)
            under = nearest(zb, p, False) if zb is not None else {}
            near_here = min(here.items(), key=lambda kv: kv[1]) if here else ('-', 99)
            under_solid = {k: v for k, v in under.items() if k != 'Sparse infill'}
            near_under = min(under_solid.items(), key=lambda kv: kv[1]) if under_solid else ('-', 99)
            rows.append((z, kind, float(length), float(x), float(y), float(plen), near_here[0], near_here[1], near_under[0], near_under[1],
                         under.get('Sparse infill', 99)))
    with open(out, 'w') as o:
        o.write('z\tkind\tlength\tx\ty\tpath_length\tnearest_here\td_here\tnearest_below_non_sparse\td_below\td_below_sparse\n')
        for r in rows:
            o.write('%.3f\t%s\t%.3f\t%.3f\t%.3f\t%.3f\t%s\t%.3f\t%s\t%.3f\t%.3f\n' % r)
    summary = collections.Counter()
    for r in rows:
        place = r[6] if r[7] < 1.0 else 'nothing within 1 mm'
        summary[(r[1], place)] += 1
    for (kind, place), n in sorted(summary.items()):
        print(f'{kind}\t{place}\t{n}')

main()

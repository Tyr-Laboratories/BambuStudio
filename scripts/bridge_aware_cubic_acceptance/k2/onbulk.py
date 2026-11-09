"""onbulk.py BAC_GCODE CUBIC_GCODE CAUSES_TSV OUT: for each free end in CAUSES_TSV (from causes.py), the sparse infill segment of the BAC
G-code that ends there, and whether its line (infinitely extended) is one of Cubic's sparse infill lines on the same layer (collinear
within 0.5 degrees and 0.02 mm): a bulk line of the lattice, which Cubic prints elsewhere on that layer, or not (a plane that only Bridge
aware cubic prints: a needed strand, an extension or a column)."""
import collections, math, re, sys
AX = re.compile(r'([XYE])(-?[0-9]*\.?[0-9]+)')
def parse(path):
    layers = collections.defaultdict(list)
    z = None; feat = ''; x = y = 0.0
    with open(path, errors='replace') as f:
        for line in f:
            if line[0] == ';':
                if line.startswith('; Z_HEIGHT:'): z = round(float(line.split(':')[1]), 3)
                elif line.startswith('; FEATURE:'): feat = line[10:].strip()
                continue
            if not line.startswith(('G0 ', 'G1 ')): continue
            d = dict((k, float(v)) for k, v in AX.findall(line.split(';')[0]))
            nx, ny = d.get('X', x), d.get('Y', y)
            if d.get('E', 0) > 0 and z is not None and feat == 'Sparse infill' and (nx != x or ny != y):
                layers[z].append((x, y, nx, ny))
            x, y = nx, ny
    return layers
bac = parse(sys.argv[1]); cubic = parse(sys.argv[2])
def lines_of(segs):
    out = collections.defaultdict(list)   # angle bucket (deg, rounded) -> offsets
    for x0, y0, x1, y1 in segs:
        a = math.atan2(y1 - y0, x1 - x0) % math.pi
        n = (-math.sin(a), math.cos(a))
        out[round(math.degrees(a)) % 180].append((math.degrees(a), n[0] * x0 + n[1] * y0))
    return out
cache = {}
rows = []
with open(sys.argv[3]) as f:
    header = next(f).rstrip('\n')
    for line in f:
        r = line.rstrip('\n').split('\t')
        z, x, y = float(r[0]), float(r[3]), float(r[4])
        z = round(z, 3)
        best = None
        for s in bac.get(z, []):
            for (ex, ey) in ((s[0], s[1]), (s[2], s[3])):
                d = math.hypot(ex - x, ey - y)
                if d < 0.01 and (best is None or d < best[0]):
                    best = (d, s)
        verdict = 'no segment'
        if best is not None:
            x0, y0, x1, y1 = best[1]
            a = math.atan2(y1 - y0, x1 - x0) % math.pi
            n = (-math.sin(a), math.cos(a)); off = n[0] * x0 + n[1] * y0
            if z not in cache: cache[z] = lines_of(cubic.get(z, []))
            on = False
            for bucket in (round(math.degrees(a)) % 180, (round(math.degrees(a)) + 1) % 180, (round(math.degrees(a)) - 1) % 180):
                for ang, o in cache[z].get(bucket, []):
                    da = abs(ang - math.degrees(a)); da = min(da, 180 - da)
                    if da < 0.5:
                        # Cubic's line with the same direction: compare offsets along the same normal
                        aa = math.radians(ang); nn = (-math.sin(aa), math.cos(aa))
                        if abs(o - (nn[0] * x0 + nn[1] * y0)) < 0.02: on = True; break
                if on: break
            verdict = 'on a Cubic line' if on else 'BAC plane'
        rows.append(r + [verdict])
with open(sys.argv[4], 'w') as o:
    o.write(header + '\tline\n')
    for r in rows: o.write('\t'.join(r) + '\n')
c = collections.Counter((r[1], r[-1]) for r in rows)
for k, v in sorted(c.items()): print(k[0], k[1], v, sep='\t')

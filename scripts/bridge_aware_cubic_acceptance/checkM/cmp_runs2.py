import csv, sys, math, collections
def load(path):
    rows = []
    try:
        for r in csv.DictReader(open(path), delimiter='\t'):
            rows.append((float(r['z']), r['role'], float(r['length']), float(r['x']), float(r['y'])))
    except FileNotFoundError:
        return None
    return rows
for p in sys.argv[1:]:
    o, b, c = load(f'runs2/open-p{p}.tsv'), load(f'runs2/base-p{p}.tsv'), load(f'runs2/cubic-p{p}.tsv')
    if o is None or b is None:
        print(p, 'missing'); continue
    def index(rows):
        d = collections.defaultdict(list)
        for z, role, L, x, y in rows: d[round(z, 2)].append((role, L, x, y))
        return d
    bi = index(b); ci = index(c) if c else None
    new = []
    for z, role, L, x, y in o:
        # same run in base: same z, same role, midpoint within 1.5 mm, length within 20%
        hit = any(r == role and math.hypot(xx - x, yy - y) < 1.5 and abs(LL - L) < 0.2 * L + 0.3 for r, LL, xx, yy in bi[round(z, 2)])
        if not hit:
            inc = None
            if ci is not None:
                inc = any(math.hypot(xx - x, yy - y) < 3.0 for r, LL, xx, yy in ci[round(z, 2)])
            new.append((z, role, L, x, y, inc))
    cnt = collections.Counter(r[1] for r in new)
    print(f'plate {p}: open runs>G {len(o)}, base {len(b)}, cubic {len(c) if c else "-"}; open runs not in base: {len(new)} {dict(cnt)}')
    for r in sorted(new, key=lambda r: -r[2])[:12]:
        print(f'   z {r[0]:.2f} {r[1]} {r[2]:.2f} mm at ({r[3]:.2f},{r[4]:.2f}) cubic has a run within 3 mm: {r[5]}')

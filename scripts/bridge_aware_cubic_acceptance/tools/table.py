"""table.py BX_SUMMARY: per-plate and per-role tables of Batch BX-fix's Heimdall runs against Cubic (investigation BX's base Cubic runs,
bx/runs/metrics.txt) and M-2a's fix round 2 (83868c540, fx2/final/summary.txt), in M-2a's format."""
import os as _os
BAC_DATA = _os.environ.get('BAC_DATA', '/Users/jforbes/Documents/Development.nosync/BambuStudio-bac-data')
BAC_WORK = _os.environ.get('BAC_WORK', BAC_DATA + '/work')
_TOOLING = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
import sys, re, collections
def read(path, prefix):
    out = {}
    for line in open(path):
        if not line.startswith(prefix):
            continue
        m = re.match(prefix + r' (\d+) (.*)', line.strip())
        if not m:
            continue
        kv = dict(t.split('=', 1) for t in m.group(2).split() if '=' in t)
        out[int(m.group(1))] = kv
    return out
# The Cubic baseline: 16 plates sliced with the same build from the Cubic project (README). The optional second column is any
# earlier run's summary.txt, given as argv[2]; without it the column reads 0.
cubic = read(BAC_DATA + '/baselines/cubic/metrics.txt', 'base cubic plate')
fx2 = read(sys.argv[2], 'bac plate') if len(sys.argv) > 2 else {}
bx = read(sys.argv[1], 'bac plate')
plates = sorted(bx)
f = lambda d, k: float(d.get(k, 0) or 0)
print('Per plate (Cubic / M-2a fix round 2, 83868c540 / BX-fix):')
print('| Plate | rc | Grams | Est. h | Sparse paths | <= 0.6 mm fix 2 / BX | Retractions | Instructions G fix 2 / BX | RSS MB fix 2 / BX |')
print('| --- | --- | --- | --- | --- | --- | --- | --- | --- |')
tot = collections.Counter()
for p in plates:
    c, a, b = cubic.get(p, {}), fx2.get(p, {}), bx[p]
    print(f"| {p} | {b.get('rc','')} | {f(c,'grams'):.2f} / {f(a,'grams'):.2f} / {f(b,'grams'):.2f} | {f(c,'total_time_s')/3600:.2f} / "
          f"{f(a,'total_time_s')/3600:.2f} / {f(b,'total_time_s')/3600:.2f} | {int(f(c,'sparse_paths')):,} / {int(f(a,'sparse_paths')):,} / "
          f"{int(f(b,'sparse_paths')):,} | {int(f(a,'short_sparse_paths')):,} / {int(f(b,'short_sparse_paths')):,} | {int(f(c,'retractions')):,} / "
          f"{int(f(a,'retractions')):,} / {int(f(b,'retractions')):,} | {f(a,'instructions')/1e9:.0f} / {f(b,'instructions')/1e9:.0f} | "
          f"{int(f(a,'rss_mb'))} / {int(f(b,'rss_mb'))} |")
    for k, d in (('c', c), ('a', a), ('b', b)):
        for key in ('grams', 'total_time_s', 'sparse_paths', 'short_sparse_paths', 'retractions', 'instructions'):
            tot[(k, key)] += f(d, key)
print()
print('| Total | Cubic | M-2a fix round 2 | BX-fix |')
print('| --- | --- | --- | --- |')
print(f"| Grams | {tot[('c','grams')]:,.2f} | {tot[('a','grams')]:,.2f} | {tot[('b','grams')]:,.2f} ({(tot[('b','grams')]/tot[('c','grams')]-1)*100:+.1f}% vs C) |")
print(f"| Estimated time | {tot[('c','total_time_s')]/3600:.1f} h | {tot[('a','total_time_s')]/3600:.1f} h | {tot[('b','total_time_s')]/3600:.1f} h |")
print(f"| Sparse paths | {int(tot[('c','sparse_paths')]):,} | {int(tot[('a','sparse_paths')]):,} | {int(tot[('b','sparse_paths')]):,} |")
print(f"| of them 0.6 mm or shorter | | {int(tot[('a','short_sparse_paths')]):,} | {int(tot[('b','short_sparse_paths')]):,} |")
print(f"| Retractions | {int(tot[('c','retractions')]):,} | {int(tot[('a','retractions')]):,} | {int(tot[('b','retractions')]):,} |")
print(f"| Instructions retired | | {tot[('a','instructions')]/1e9:,.0f} G | {tot[('b','instructions')]/1e9:,.0f} G (x{tot[('b','instructions')]/tot[('a','instructions')]:.3f} vs fix 2) |")
roles = sorted({k for d in list(bx.values()) + list(cubic.values()) for k in d if k[0].isupper()})
print()
print('Per role, all plates, grams (Cubic / fix 2 / BX-fix, and BX-fix - Cubic):')
print('| Role | Cubic | fix 2 | BX-fix | BX - Cubic | BX - fix 2 |')
print('| --- | --- | --- | --- | --- | --- |')
for r in roles:
    sc = sum(f(cubic.get(p, {}), r) for p in plates); sa = sum(f(fx2.get(p, {}), r) for p in plates); sb = sum(f(bx[p], r) for p in plates)
    print(f'| {r.replace("_", " ")} | {sc:.2f} | {sa:.2f} | {sb:.2f} | {sb-sc:+.2f} | {sb-sa:+.2f} |')
print()
print('Per plate and role, BX-fix - Cubic, grams (roles that differ by 0.1 g or more somewhere):')
big = [r for r in roles if r not in ('Outer_wall', 'Inner_wall', 'Brim') and any(abs(f(bx[p], r) - f(cubic.get(p, {}), r)) >= .1 for p in plates)]
print('| Plate | ' + ' | '.join(r.replace('_', ' ') for r in big) + ' |')
print('| --- |' + ' --- |' * len(big))
for p in plates:
    print(f'| {p} | ' + ' | '.join(f'{f(bx[p], r) - f(cubic.get(p, {}), r):+.2f}' for r in big) + ' |')

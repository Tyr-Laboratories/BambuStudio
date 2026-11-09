import csv, collections, re, sys
rows = list(csv.DictReader(open('grams-all.tsv'), delimiter='\t'))
g = collections.defaultdict(dict); xy = collections.defaultdict(dict); np_ = collections.defaultdict(dict); rt = collections.defaultdict(dict); hdr = {}; est = {}
for r in rows:
    l = r['label']; k = r['feature']
    g[l][k] = float(r['grams']); xy[l][k] = float(r['xy_mm']); np_[l][k] = int(r['paths']); rt[l][k] = int(r['retracts']); hdr[l] = r['header_total_g']; est[l] = r['est']
def tot(l): return sum(g[l].values())
def secs(s):
    m = re.search(r'total estimated time: (.*)$', s or '')
    if not m: return float('nan')
    t = 0
    for v, u in re.findall(r'(\d+)([dhms])', m.group(1)):
        t += int(v) * {'d': 86400, 'h': 3600, 'm': 60, 's': 1}[u]
    return t
print('plate | totals c/b/o (header c/b/o) | sparse c/b/o | open-cubic sparse | sparse paths c/b/o | sparse retractions c/b/o | est. time h c/b/o')
S = collections.Counter()
for p in range(1, 17):
    c, b, o = f'cubic-p{p}', f'base-p{p}', f'open-p{p}'
    if c not in g: print(p, 'no G-code'); continue
    cs, bs, os_ = (g[x].get('Sparse infill', 0) for x in (c, b, o))
    print(f"{p} | {tot(c):.2f}/{tot(b):.2f}/{tot(o):.2f} ({hdr[c]}/{hdr[b]}/{hdr[o]}) | {cs:.2f}/{bs:.2f}/{os_:.2f} | {os_-cs:+.2f} ({os_/cs:.3f}x) | "
          f"{np_[c].get('Sparse infill',0)}/{np_[b].get('Sparse infill',0)}/{np_[o].get('Sparse infill',0)} | "
          f"{rt[c].get('Sparse infill',0)}/{rt[b].get('Sparse infill',0)}/{rt[o].get('Sparse infill',0)} | "
          f"{secs(est[c])/3600:.2f}/{secs(est[b])/3600:.2f}/{secs(est[o])/3600:.2f}")
    for k, l in (('c', c), ('b', b), ('o', o)):
        for pre, cond in (('all', True), ('p1-5', p <= 5)):
            if not cond: continue
            S[(pre, k, 'tot')] += tot(l); S[(pre, k, 'sp')] += g[l].get('Sparse infill', 0); S[(pre, k, 'br')] += g[l].get('Bridge', 0)
            S[(pre, k, 'paths')] += np_[l].get('Sparse infill', 0); S[(pre, k, 'retr')] += sum(rt[l].values()); S[(pre, k, 'h')] += secs(est[l]) / 3600
for pre in ('p1-5', 'all'):
    print(pre, ' '.join(f"{k}:{w}={S[(pre,k,w)]:.2f}" for w in ('tot', 'sp', 'br', 'paths', 'retr', 'h') for k in 'cbo'))
if len(sys.argv) > 1:
    roles = collections.Counter()
    for v in ('cubic', 'base', 'open', 'open4dev'):
        for p in range(1, 6):
            for k, x in g.get(f'{v}-p{p}', {}).items(): roles[(v, k)] += x
    keys = sorted({k for (_, k) in roles}, key=lambda k: -roles[('open', k)])
    print('role (plates 1-5) | cubic | base | open | open4dev | open-cubic')
    for k in keys:
        print(f"{k} | {roles[('cubic',k)]:.2f} | {roles[('base',k)]:.2f} | {roles[('open',k)]:.2f} | {roles[('open4dev',k)]:.2f} | {roles[('open',k)]-roles[('cubic',k)]:+.2f}")

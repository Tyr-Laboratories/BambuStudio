import re, sys, os
# Batch M-2a Heimdall tables: this run's summary.txt (BAC all plates, Cubic plates re-sliced), M-1's summary.txt (Cubic and M-1 BAC), and
# instructions retired from the /usr/bin/time -l logs (Cubic baselines and M-1 BAC from M-1's runs, as the controller asked).
HERE = os.path.dirname(os.path.abspath(__file__))
M1 = os.path.join(HERE, '..', '..', 'm1', 'final')
W = {1: 5057, 2: 6813, 3: 5285, 4: 3986, 5: 5208, 6: 3984, 7: 4394, 8: 4026, 9: 4223, 10: 5279, 11: 5330, 12: 5217, 13: 4082, 14: 4022,
     15: 6371, 16: 3950}
def load(path):
    rows = {}
    for line in open(path):
        m = re.match(r'(cubic|bac) plate (\d+) (.*)', line)
        if m:
            rows.setdefault(int(m.group(2)), {})[m.group(1)] = dict(re.findall(r'(\w+)=(\S*)', m.group(3)))
    return rows
def instructions(path):
    try:
        for line in open(path):
            if 'instructions retired' in line:
                return int(line.split()[0])
    except OSError:
        pass
    return None
now = load(os.path.join(HERE, 'summary.txt'))
m1 = load(os.path.join(M1, 'summary.txt'))
plates = sorted(now)
num = lambda d, k, t=float: t(d[k]) if d.get(k) not in (None, '', 'None') else None
print('| Plate | rc | Total g C / M-1 / M-2a | Sparse g C / M-1 / M-2a | Est. h C / M-1 / M-2a | Sparse paths C / M-1 / M-2a | <= 0.6 mm M-2a | '
      'Retractions C / M-1 / M-2a | Instr. G C / M-1 / M-2a | M-2a / C | RSS MB C / W / M-2a |')
print('| --- ' * 11 + '|')
T = {}
def add(key, v):
    if v is not None:
        T[key] = T.get(key, 0) + v
for p in plates:
    b = now[p].get('bac', {})
    c = now[p].get('cubic') or m1.get(p, {}).get('cubic', {})
    o = m1.get(p, {}).get('bac', {})
    ic = instructions(os.path.join(M1, 'cubic', f'p{p}', 'time.log'))
    io = instructions(os.path.join(M1, 'bac', f'p{p}', 'time.log'))
    ib = num(b, 'instructions', int)
    vals = {}
    for tag, d in (('c', c), ('o', o), ('b', b)):
        vals[tag] = dict(g=num(d, 'grams'), s=num(d, 'sparse'), t=num(d, 'total_time_s', int), paths=num(d, 'sparse_paths', int),
                         r=num(d, 'retractions', int), short=num(d, 'short_sparse_paths', int), rss=num(d, 'rss_mb', int))
        for k, v in vals[tag].items():
            add(tag + k, v)
    add('ic', ic); add('io', io); add('ib', ib)
    h = lambda s: f'{s / 3600:.2f}' if s is not None else '?'
    g = lambda v: f'{v / 1e9:.0f}' if v is not None else '?'
    ratio = f'{ib / ic:.3f}' if ib and ic else '?'
    rss_gate = W[p] if p != 6 else f"{W[p]} (C+10%: {int(vals['c']['rss'] * 1.1) if vals['c']['rss'] else '?'})"
    print(f"| {p} | {b.get('rc', '?')} | {vals['c']['g']} / {vals['o']['g']} / {vals['b']['g']} | {vals['c']['s']} / {vals['o']['s']} / {vals['b']['s']} | "
          f"{h(vals['c']['t'])} / {h(vals['o']['t'])} / {h(vals['b']['t'])} | {vals['c']['paths']} / {vals['o']['paths']} / {vals['b']['paths']} | "
          f"{vals['b']['short']} | {vals['c']['r']} / {vals['o']['r']} / {vals['b']['r']} | {g(ic)} / {g(io)} / {g(ib)} | {ratio} | "
          f"{vals['c']['rss']} / {rss_gate} / {vals['b']['rss']} |")
pct = lambda a, b: f'{100 * (a / b - 1):+.1f}%'
print()
print(f"Totals ({len(plates)} plates): grams C {T['cg']:.2f}, M-1 {T['og']:.2f}, M-2a {T['bg']:.2f} ({pct(T['bg'], T['cg'])} vs C, {pct(T['bg'], T['og'])} vs M-1); "
      f"sparse g C {T['cs']:.2f}, M-1 {T['os']:.2f}, M-2a {T['bs']:.2f}; est. h C {T['ct'] / 3600:.1f}, M-1 {T['ot'] / 3600:.1f}, M-2a {T['bt'] / 3600:.1f} "
      f"({pct(T['bt'], T['ct'])} vs C, {pct(T['bt'], T['ot'])} vs M-1); sparse paths C {T['cpaths']}, M-1 {T['opaths']}, M-2a {T['bpaths']} "
      f"(<= 0.6 mm: {T.get('bshort', 0)}); retractions C {T['cr']}, M-1 {T['or']}, M-2a {T['br']}; instructions G C {T['ic'] / 1e9:.0f}, "
      f"M-1 {T['io'] / 1e9:.0f}, M-2a {T['ib'] / 1e9:.0f} (M-2a / C {T['ib'] / T['ic']:.3f}, M-1 / C {T['io'] / T['ic']:.3f})")
sub = [p for p in plates if p <= 5]
if sub:
    def s(tag, k): return sum((now[p].get('bac') if tag == 'b' else (now[p].get('cubic') or m1[p]['cubic']) if tag == 'c' else m1[p]['bac']).get(k) and float((now[p].get('bac') if tag == 'b' else (now[p].get('cubic') or m1[p]['cubic']) if tag == 'c' else m1[p]['bac'])[k]) or 0 for p in sub)
    print(f"Plates 1-5: grams C {s('c', 'grams'):.2f}, M-2a {s('b', 'grams'):.2f} ({s('b', 'grams') - s('c', 'grams'):+.2f} g); est. h C {s('c', 'total_time_s') / 3600:.2f}, "
          f"M-2a {s('b', 'total_time_s') / 3600:.2f} ({(s('b', 'total_time_s') - s('c', 'total_time_s')) / 3600:+.2f} h, x{s('b', 'total_time_s') / s('c', 'total_time_s'):.2f})")

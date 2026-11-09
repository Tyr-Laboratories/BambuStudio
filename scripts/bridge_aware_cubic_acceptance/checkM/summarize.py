import re, sys, glob
rows = []
cur = None
for path in sorted(glob.glob(sys.argv[1] + '/part*.txt')):
    for line in open(path):
        m = re.match(r'== (\S+) d(\S+) g(\S+)', line)
        if m:
            cur = dict(case=m.group(1), d=float(m.group(2)), g=float(m.group(3)))
            rows.append(cur); continue
        if cur is None: continue
        if line.strip().startswith('mass:'):
            m = re.search(r'g cubic ([\d.]+) bac ([\d.]+) \(\+([-\d.]+)\)', line)
            cur['cubic_g'], cur['bac_g'], cur['dg'] = map(float, m.groups())
            for k in ('needed', 'post', 'bulk', 'strand', 'extension', 'loop'):
                mm = re.search(k + r' ([\d.]+) mm/([\d.]+) g', line)
                cur[k + '_g'] = float(mm.group(2)) if mm else 0.0
        elif line.strip().startswith('paths:'):
            m = re.search(r'sparse cubic (\d+) bac (\d+) \(<=0.6 mm: (\d+)\)', line)
            cur['cpaths'], cur['bpaths'], cur['short'] = map(int, m.groups())
        elif line.strip().startswith('K1 verifier'):
            m = re.search(r'openings>G (\d+) \(max ([\d.]+)\) runs>G (\d+) \(max ([\d.]+) ([^)]*)\)', line)
            cur['v_open_over'], cur['v_open'], cur['v_run_over'], cur['v_run'], cur['v_role'] = int(m.group(1)), float(m.group(2)), int(m.group(3)), float(m.group(4)), m.group(5)
        elif line.strip().startswith('K1 independent'):
            m = re.search(r'W8 openings>G (\d+) \(max ([\d.]+).*runs>G (\d+) \(max ([\d.]+)', line)
            cur['w8_over'], cur['w8'], cur['i_run_over'], cur['i_run'] = int(m.group(1)), float(m.group(2)), int(m.group(3)), float(m.group(4))
        elif line.strip().startswith('K2'):
            fe = dict()
            for k, n, l, mx in re.findall(r'(bulk\(cubic too\)|bulk\(new\)|post|needed|extension|strand|loop) (\d+) \(([\d.]+) mm, max ([\d.]+)\)', line):
                fe[k] = (int(n), float(l), float(mx))
            cur['fe'] = fe
            m = re.search(r'runs>G (\d+)', line); cur['k2_runs'] = int(m.group(1))
        elif line.strip().startswith('survival'):
            m = re.search(r'short (\d+) missing ([\d.]+)', line)
            cur['surv_short'], cur['surv_mm'] = int(m.group(1)), float(m.group(2))
hdr = 'case d g | cubic g | +g (x) | post g (% of +) | needed g | sparse paths c/b (<=0.6) | K1 verifier open>G/run>G (max open, max run role) | W8 open>G (max) | runs>G indep (max) | K2 free ends on strands [post,needed,ext,strand] (max mm) | K2 bulk new (max) | K2 runs>G | survival short'
print(hdr)
for r in rows:
    fe = r.get('fe', {})
    on = sum(fe.get(k, (0,))[0] for k in ('post', 'needed', 'extension', 'strand', 'loop'))
    onmax = max([fe.get(k, (0, 0, 0))[2] for k in ('post', 'needed', 'extension', 'strand', 'loop')] + [0])
    det = ','.join(str(fe.get(k, (0,))[0]) for k in ('post', 'needed', 'extension', 'strand'))
    bn = fe.get('bulk(new)', (0, 0, 0))
    print(f"{r['case']} {r['d']:g} {r['g']:g} | {r['cubic_g']:.2f} | +{r['dg']:.2f} (x{r['bac_g']/r['cubic_g']:.2f}) | {r['post_g']:.2f} ({100*r['post_g']/max(r['dg'],1e-9):.0f}%) | {r['needed_g']:.2f} | {r['cpaths']}/{r['bpaths']} ({r['short']}) | "
          f"{r['v_open_over']}/{r['v_run_over']} ({r['v_open']:.2f}, {r['v_run']:.2f} {r['v_role']}) | {r['w8_over']} ({r['w8']:.2f}) | {r['i_run_over']} ({r['i_run']:.2f}) | {on} [{det}] ({onmax:.3f}) | {bn[0]} ({bn[2]:.3f}) | {r['k2_runs']} | {r['surv_short']}")

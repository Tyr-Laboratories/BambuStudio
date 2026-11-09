"""take.py TAKE_DIR: per plate, the strand length that the plan made where the bridges then took the sparse fill (the BX_TAKE lines of the
instrumented runs), summed over the plate's objects."""
import re, sys, os
pat = re.compile(r'BX_TAKE object "(.*)": planned strands ([0-9.]+) mm; of them in sparse area that became internal bridge ([0-9.]+) mm, '
                 r'internal solid ([0-9.]+) mm, other ([0-9.]+) mm; bulk lines there ([0-9.]+) mm; area taken ([0-9.]+) mm2 on (\d+) layers')
root = sys.argv[1]
tot = [0.0] * 6
print('| Plate | Objects | Planned strands (m) | In area that became internal bridge (mm) | became internal solid (mm) | other (mm) | Share | Bulk lines there (mm) | Area taken (mm2) |')
print('| --- | --- | --- | --- | --- | --- | --- | --- | --- |')
for p in range(1, 17):
    f = os.path.join(root, f'p{p}', 'stdout.log')
    if not os.path.exists(f):
        continue
    rows = [pat.search(l) for l in open(f, errors='replace')]
    rows = [r for r in rows if r]
    s = [sum(float(r.group(i)) for r in rows) for i in (2, 3, 4, 5, 6, 7)]
    for i in range(6): tot[i] += s[i]
    share = (s[1] + s[2] + s[3]) / s[0] * 100 if s[0] else 0
    print(f'| {p} | {len(rows)} | {s[0]/1000:.1f} | {s[1]:.1f} | {s[2]:.1f} | {s[3]:.1f} | {share:.2f}% | {s[4]:.1f} | {s[5]:.0f} |')
share = (tot[1] + tot[2] + tot[3]) / tot[0] * 100 if tot[0] else 0
print(f'| all | | {tot[0]/1000:.1f} | {tot[1]:.1f} | {tot[2]:.1f} | {tot[3]:.1f} | {share:.2f}% | {tot[4]:.1f} | {tot[5]:.0f} |')

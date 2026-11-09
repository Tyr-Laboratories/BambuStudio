#!/usr/bin/env python3
"""Independent grams-per-role recount (Batch M check).

For each G-code file: per ';FEATURE:' role, the filament length extruded (sum of positive E deltas in relative mode, or of
positive increments in absolute mode, honouring G92 E resets), the XY path length of extruding moves, and the number of
extrusion paths (maximal runs of consecutive extruding moves of one role without a travel or retraction between them).
Grams = E * pi * (d/2)^2 * density / 1000, d and density from the G-code's own header.

Usage: grams2.py OUT.tsv LABEL=PATH [LABEL=PATH ...]
"""
import math
import re
import sys
from concurrent.futures import ProcessPoolExecutor

AX = re.compile(r'([XYZEIJ])(-?[0-9]*\.?[0-9]+)')


def scan(arg):
    label, path = arg
    rel = True
    e_abs = 0.0
    x = y = 0.0
    feat = 'none'
    dens = diam = None
    e_tot = {}
    xy = {}
    npaths = {}
    in_path = False
    total_w = None
    retracts = {}
    prev_wipe = False
    est = None
    with open(path, errors='replace') as f:
        for line in f:
            c = line[0]
            if c == ';':
                if line.startswith('; FEATURE:'):
                    feat = line[10:].strip()
                    in_path = False
                elif dens is None and line.startswith('; filament_density'):
                    dens = float(re.split('[:=]', line)[1].split(',')[0])
                elif diam is None and line.startswith('; filament_diameter'):
                    diam = float(re.split('[:=]', line)[1].split(',')[0])
                elif est is None and 'total estimated time' in line:
                    est = line.strip()
                elif total_w is None and line.startswith('; total filament weight'):
                    total_w = float(line.split(':')[1].split(',')[0])
                continue
            if c != 'G' and c != 'M':
                continue
            if line.startswith('M82'):
                rel = False
                continue
            if line.startswith('M83'):
                rel = True
                continue
            cmd = line.split(' ', 1)[0].split(';')[0].strip()
            if cmd == 'G92':
                for a, v in AX.findall(line.split(';')[0]):
                    if a == 'E':
                        e_abs = float(v)
                continue
            if cmd not in ('G0', 'G1', 'G2', 'G3'):
                continue
            nx, ny, e = x, y, None
            for a, v in AX.findall(line.split(';')[0]):
                if a == 'X':
                    nx = float(v)
                elif a == 'Y':
                    ny = float(v)
                elif a == 'E':
                    e = float(v)
            de = 0.0
            if e is not None:
                if rel:
                    de = e
                else:
                    de = e - e_abs
                    e_abs = e
            moved = (nx != x) or (ny != y)
            if de > 0 and moved:
                e_tot[feat] = e_tot.get(feat, 0.0) + de
                xy[feat] = xy.get(feat, 0.0) + math.hypot(nx - x, ny - y)
                if not in_path:
                    npaths[feat] = npaths.get(feat, 0) + 1
                    in_path = True
            else:
                # an unretract, a prime or a wipe-retract: not material laid down, and the end of a path
                if de > 0 and not moved:
                    # one unretract per retraction: count retractions by their unretract
                    retracts[feat] = retracts.get(feat, 0) + 1
                if moved or de != 0:
                    in_path = False
            prev_wipe = (de < 0 and moved)
            x, y = nx, ny
    area = math.pi * (diam / 2) ** 2
    grams = {k: v * area * dens / 1000.0 for k, v in e_tot.items()}
    return label, path, grams, xy, npaths, total_w, retracts, est


def main():
    out = sys.argv[1]
    args = [a.split('=', 1) for a in sys.argv[2:]]
    with ProcessPoolExecutor(max_workers=8) as ex:
        res = list(ex.map(scan, args))
    with open(out, 'w') as f:
        f.write('label\tfeature\tgrams\txy_mm\tpaths\tretracts\theader_total_g\test\n')
        for label, path, grams, xy, npaths, total_w, retracts, est in res:
            for k in sorted(set(grams) | set(retracts)):
                f.write(f'{label}\t{k}\t{grams.get(k, 0.0):.4f}\t{xy.get(k, 0.0):.1f}\t{npaths.get(k, 0)}\t{retracts.get(k, 0)}\t{total_w}\t{est}\n')
    print('wrote', out)


if __name__ == '__main__':
    main()

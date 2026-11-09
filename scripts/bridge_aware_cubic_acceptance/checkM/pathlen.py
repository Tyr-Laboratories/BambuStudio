#!/usr/bin/env python3
"""Histogram of sparse-infill path lengths (maximal extruding runs between travels/retractions) and their grams.
Usage: pathlen.py LABEL=GCODE ..."""
import math, re, sys
from concurrent.futures import ProcessPoolExecutor
AX = re.compile(r'([XYE])(-?[0-9]*\.?[0-9]+)')
def scan(arg):
    label, path = arg
    feat = ''; x = y = 0.0; dens = diam = None
    cur_len = 0.0; cur_e = 0.0
    paths = []
    def close():
        nonlocal cur_len, cur_e
        if cur_len > 0:
            paths.append((cur_len, cur_e))
        cur_len = 0.0; cur_e = 0.0
    with open(path, errors='replace') as f:
        for line in f:
            if line[0] == ';':
                if line.startswith('; FEATURE:'):
                    if feat == 'Sparse infill': close()
                    feat = line[10:].strip()
                elif dens is None and line.startswith('; filament_density'):
                    dens = float(re.split('[:=]', line)[1].split(',')[0])
                elif diam is None and line.startswith('; filament_diameter'):
                    diam = float(re.split('[:=]', line)[1].split(',')[0])
                continue
            if not line.startswith(('G0 ', 'G1 ', 'G2 ', 'G3 ')):
                continue
            nx, ny, e = x, y, None
            for a, v in AX.findall(line.split(';')[0]):
                if a == 'X': nx = float(v)
                elif a == 'Y': ny = float(v)
                elif a == 'E': e = float(v)
            moved = nx != x or ny != y
            if feat == 'Sparse infill':
                if moved and e is not None and e > 0:
                    cur_len += math.hypot(nx - x, ny - y); cur_e += e
                elif moved or (e is not None and e != 0):
                    close()
            x, y = nx, ny
    close()
    area = math.pi * (diam / 2) ** 2
    g = lambda e: e * area * dens / 1000.0
    bins = [0.1, 0.25, 0.4, 0.55, 0.75, 1.0, 1.6, 3, 5, 10, 1e9]
    cnt = [0] * len(bins); mm = [0.0] * len(bins); gr = [0.0] * len(bins)
    for L, e in paths:
        i = next(i for i, b in enumerate(bins) if L <= b)
        cnt[i] += 1; mm[i] += L; gr[i] += g(e)
    return label, bins, cnt, mm, gr
if __name__ == '__main__':
    with ProcessPoolExecutor(8) as ex:
        for label, bins, cnt, mm, gr in ex.map(scan, [a.split('=', 1) for a in sys.argv[1:]]):
            print(label, 'paths', sum(cnt), 'grams', round(sum(gr), 2))
            lo = 0
            for b, c, m, g in zip(bins, cnt, mm, gr):
                print(f'   ({lo:g}, {b:g}] mm: {c} paths, {m:.0f} mm, {g:.2f} g')
                lo = b

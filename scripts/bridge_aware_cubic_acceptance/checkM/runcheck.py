#!/usr/bin/env python3
"""Independent K1 run check on exported G-code (Batch M check).

For every non-sparse extrusion path (maximal run of extruding moves of one role between travels) of the roles a roof prints
(Bridge, Top surface, Internal solid infill, Floating vertical shell, Bottom surface, Inner/Outer/Overhang wall, Gap infill) on
layer n, the parts whose centre line lies farther than w/2 + tol from every bead footprint of layer n - 1 (butt-ended rectangles of
';LINE_WIDTH', every role) are unsupported. An unsupported stretch between two supported points longer than GAP is a long run.
Free ends are not runs. Writes one row per long run.

Usage: runcheck.py GCODE GAP OUT.tsv [--tol 0.01]
"""
import argparse
import re

import numpy as np
import shapely

AX = re.compile(r'([XYE])(-?[0-9]*\.?[0-9]+)')
ONLY = None
ROLES = {'Bridge', 'Top surface', 'Internal solid infill', 'Floating vertical shell', 'Bottom surface', 'Inner wall', 'Outer wall',
         'Overhang wall', 'Gap infill'}


def parse(path):
    layers = {}
    z = None
    feat = ''
    width = 0.4
    x = y = 0.0
    cur = None
    cur_w = []
    cur_f = ''

    def close():
        nonlocal cur
        if cur is not None and len(cur) >= 2 and z is not None:
            layers[z][1].append((cur, list(cur_w), cur_f))
        cur = None

    with open(path, errors='replace') as f:
        for line in f:
            c = line[0]
            if c == ';':
                if line.startswith('; Z_HEIGHT:'):
                    close()
                    z = round(float(line.split(':')[1]), 3)
                    layers.setdefault(z, ([], []))
                elif line.startswith('; FEATURE:'):
                    close()
                    feat = line[10:].strip()
                elif line.startswith('; LINE_WIDTH:'):
                    width = float(line.split(':')[1])
                continue
            if not line.startswith(('G0 ', 'G1 ', 'G2 ', 'G3 ')):
                continue
            nx, ny, e = x, y, None
            for a, v in AX.findall(line.split(';')[0]):
                if a == 'X':
                    nx = float(v)
                elif a == 'Y':
                    ny = float(v)
                elif a == 'E':
                    e = float(v)
            moved = nx != x or ny != y
            if moved and e is not None and e > 0 and z is not None:
                layers[z][0].append((x, y, nx, ny, width))
                if feat in ROLES and (ONLY is None or feat in ONLY):
                    if cur is None:
                        cur = [(x, y)]
                        cur_w = []
                        cur_f = feat
                    cur.append((nx, ny))
                    cur_w.append(width)
                else:
                    close()
            elif moved or (e is not None and e != 0):
                close()
            x, y = nx, ny
    close()
    return layers


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('gcode')
    ap.add_argument('gap', type=float)
    ap.add_argument('out')
    ap.add_argument('--tol', type=float, default=0.01)
    ap.add_argument('--only', default='')
    args = ap.parse_args()
    global ONLY
    if args.only:
        ONLY = set(args.only.split(','))
    L = parse(args.gcode)
    zs = sorted(L)
    n_runs = 0
    by_role = {}
    longest = 0.0
    with open(args.out, 'w') as out:
        out.write('z\trole\tlength\tx\ty\tpath_length\n')
        for k in range(1, len(zs)):
            z, zb = zs[k], zs[k - 1]
            segs, paths = L[z]
            if not paths:
                continue
            lower = L[zb][0]
            if not lower:
                continue
            low = np.array(lower, dtype=float)
            rects = shapely.buffer(shapely.linestrings(np.stack([low[:, 0:2], low[:, 2:4]], axis=1)), low[:, 4] / 2, cap_style='flat')
            rects = rects[~shapely.is_empty(rects)]
            footprint = shapely.union_all(rects)
            supports = {}

            def support(w):
                key = round(w, 2)
                if key not in supports:
                    supports[key] = shapely.buffer(footprint, key / 2 + args.tol)
                return supports[key]

            for pts, ws, role in paths:
                line = shapely.LineString(pts)
                plen = line.length
                if len(set(round(w, 3) for w in ws)) == 1:
                    un = shapely.difference(line, support(ws[0]))
                else:
                    segs = [shapely.difference(shapely.LineString([pts[i], pts[i + 1]]), support(ws[i])) for i in range(len(ws))]
                    segs = [g for g in segs if not g.is_empty]
                    un = shapely.union_all(segs) if segs else shapely.LineString()
                if un.is_empty:
                    continue
                if hasattr(un, 'geoms'):
                    un = shapely.line_merge(un)
                pieces = list(un.geoms) if hasattr(un, 'geoms') else [un]
                p0 = shapely.Point(pts[0])
                p1 = shapely.Point(pts[-1])
                # A loop printed with a seam gap is a ring: the unsupported pieces at its two ends are one run.
                ring = p0.distance(p1) < 2.0 and plen > 4.0
                end_pieces = []
                for piece in pieces:
                    if piece.geom_type != 'LineString':
                        continue
                    a = shapely.Point(piece.coords[0])
                    b = shapely.Point(piece.coords[-1])
                    if min(a.distance(p0), a.distance(p1), b.distance(p0), b.distance(p1)) < 1e-6:
                        end_pieces.append(piece)
                if ring and len(end_pieces) == 2:
                    joined = end_pieces[0].length + end_pieces[1].length
                    if joined > args.gap:
                        m = end_pieces[0].interpolate(0.5, normalized=True)
                        out.write(f'{z:.3f}\t{role} (ring)\t{joined:.3f}\t{m.x:.3f}\t{m.y:.3f}\t{plen:.3f}\n')
                        n_runs += 1
                        by_role[role + ' (ring)'] = by_role.get(role + ' (ring)', 0) + 1
                        longest = max(longest, joined)
                for piece in pieces:
                    if piece.geom_type != 'LineString' or piece.length <= args.gap:
                        continue
                    a = shapely.Point(piece.coords[0])
                    b = shapely.Point(piece.coords[-1])
                    if min(a.distance(p0), a.distance(p1), b.distance(p0), b.distance(p1)) < 1e-6:
                        continue
                    m = piece.interpolate(0.5, normalized=True)
                    out.write(f'{z:.3f}\t{role}\t{piece.length:.3f}\t{m.x:.3f}\t{m.y:.3f}\t{plen:.3f}\n')
                    n_runs += 1
                    by_role[role] = by_role.get(role, 0) + 1
                    longest = max(longest, piece.length)
    print(args.gcode, 'runs>G', n_runs, by_role, 'longest', round(longest, 3))


if __name__ == '__main__':
    main()

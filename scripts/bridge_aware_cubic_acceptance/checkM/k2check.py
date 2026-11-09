#!/usr/bin/env python3
"""Independent K2 check on exported G-code (Batch M check).

For every sparse-infill path (maximal run of extruding moves of ';FEATURE: Sparse infill' between travels) on layer n, the parts
whose centre line lies farther than w_up/2 + tol from every bead footprint of layer n - 1 (each bead a butt-ended rectangle of its
';LINE_WIDTH', every role) are unsupported. An unsupported piece that reaches an end of its path is a free end; one between two
supported points longer than GAP is a long run.

Each free end is classified against a reference G-code (Cubic of the same plate): 'bulk' if a reference sparse segment on the same
layer is collinear with the free piece (angle < 0.5 deg, offset < 0.02 mm) and overlaps it; otherwise 'added' (a BAC strand), with
'post' when the whole path is no longer than 0.6 mm.

Usage: k2check.py GCODE REFERENCE_GCODE GAP OUT.tsv [--tol 0.01]
"""
import argparse
import math
import re
import sys

import numpy as np
import shapely
from shapely import STRtree

AX = re.compile(r'([XYE])(-?[0-9]*\.?[0-9]+)')
# The lattice line directions of the project (infill_direction 0): 0, 60 and 120 degrees.
DIRS = (0.0, 60.0, 120.0)


def parse(path):
    """{z: (segments list [(x0,y0,x1,y1,w,feature)], sparse paths [list of (x,y) points with width])}"""
    layers = {}
    z = None
    feat = ''
    width = 0.4
    x = y = 0.0
    cur = None  # current sparse path points
    cur_w = 0.0

    def close():
        nonlocal cur
        if cur is not None and len(cur) >= 2 and z is not None:
            layers[z][1].append((cur, cur_w))
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
                layers[z][0].append((x, y, nx, ny, width, feat))
                if feat == 'Sparse infill':
                    if cur is None:
                        cur = [(x, y)]
                        cur_w = width
                    cur.append((nx, ny))
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
    ap.add_argument('reference')
    ap.add_argument('gap', type=float)
    ap.add_argument('out')
    ap.add_argument('--tol', type=float, default=0.01)
    args = ap.parse_args()
    L = parse(args.gcode)
    R = parse(args.reference)
    zs = sorted(L)
    counts = {'paths': 0, 'free_bulk': 0, 'free_added': 0, 'free_connection': 0, 'free_post': 0, 'long_bulk': 0, 'long_added': 0}
    mm = {'free_bulk': 0.0, 'free_added': 0.0, 'free_connection': 0.0, 'free_post': 0.0}
    mx = {'free_bulk': 0.0, 'free_added': 0.0, 'free_connection': 0.0, 'free_post': 0.0, 'long_bulk': 0.0, 'long_added': 0.0}
    over = {'free_added>0.25': 0, 'free_post>0.25': 0, 'free_added>0.5': 0, 'free_post>0.5': 0, 'free_connection>0.25': 0, 'free_connection>0.5': 0}
    out = open(args.out, 'w')
    out.write('z\tkind\tlength\tx\ty\tpath_length\n')
    for k in range(1, len(zs)):
        z, zb = zs[k], zs[k - 1]
        segs, paths = L[z]
        if not paths:
            continue
        lower = L[zb][0]
        if not lower:
            continue
        low = np.array([s[:5] for s in lower], dtype=float)
        rects = shapely.buffer(shapely.linestrings(np.stack([low[:, 0:2], low[:, 2:4]], axis=1)), low[:, 4] / 2, cap_style='flat')
        rects = rects[~shapely.is_empty(rects)]
        footprint = shapely.union_all(rects)
        # reference sparse segments on this layer
        ref_segs = [s for s in R.get(z, ([], []))[0] if s[5] == 'Sparse infill']
        ref = np.array([s[:4] for s in ref_segs], dtype=float) if ref_segs else np.zeros((0, 4))
        ref_tree = STRtree(shapely.linestrings(np.stack([ref[:, 0:2], ref[:, 2:4]], axis=1))) if len(ref) else None
        widths = sorted({round(p[1], 4) for p in paths})
        supports = {w: shapely.buffer(footprint, w / 2 + args.tol) for w in widths}
        for pts, w in paths:
            counts['paths'] += 1
            line = shapely.LineString(pts)
            plen = line.length
            if plen <= 0:
                continue
            un = shapely.difference(line, supports[round(w, 4)])
            if un.is_empty:
                continue
            if hasattr(un, 'geoms'):
                un = shapely.line_merge(un)
            pieces = list(un.geoms) if hasattr(un, 'geoms') else [un]
            p0 = shapely.Point(pts[0])
            p1 = shapely.Point(pts[-1])
            for piece in pieces:
                if piece.geom_type != 'LineString' or piece.length < 1e-4:
                    continue
                a = shapely.Point(piece.coords[0])
                b = shapely.Point(piece.coords[-1])
                at_end = min(a.distance(p0), a.distance(p1), b.distance(p0), b.distance(p1)) < 1e-6
                # classification against the reference
                bulk = False
                if ref_tree is not None:
                    cand = ref_tree.query(piece, predicate='dwithin', distance=0.02)
                    (ax, ay), (bx, by) = piece.coords[0], piece.coords[-1]
                    dx, dy = bx - ax, by - ay
                    lp = math.hypot(dx, dy)
                    for i in cand:
                        r = ref[i]
                        rx, ry = r[2] - r[0], r[3] - r[1]
                        lr = math.hypot(rx, ry)
                        if lr < 1e-6 or lp < 1e-6:
                            continue
                        cross = abs(dx * ry - dy * rx) / (lp * lr)
                        if cross > math.sin(math.radians(0.5)):
                            continue
                        # offset of piece midpoint from the reference line
                        mx_, my_ = (ax + bx) / 2, (ay + by) / 2
                        off = abs((mx_ - r[0]) * ry - (my_ - r[1]) * rx) / lr
                        if off > 0.02:
                            continue
                        bulk = True
                        break
                (ax_, ay_), (bx_, by_) = piece.coords[0], piece.coords[-1]
                ang = math.degrees(math.atan2(by_ - ay_, bx_ - ax_)) % 180.0
                lattice = any(min(abs(ang - d), 180.0 - abs(ang - d)) < 1.0 for d in DIRS)
                if at_end:
                    if bulk:
                        kind = 'free_bulk'
                    elif plen <= 0.6:
                        kind = 'free_post'
                    elif lattice:
                        kind = 'free_added'
                    else:
                        kind = 'free_connection'
                    counts[kind] += 1
                    mm[kind] += piece.length
                    mx[kind] = max(mx[kind], piece.length)
                    if kind != 'free_bulk':
                        over[kind + '>0.25'] += piece.length > 0.25
                        over[kind + '>0.5'] += piece.length > 0.5
                    end = a if min(a.distance(p0), a.distance(p1)) < 1e-6 else b
                    out.write(f'{z:.3f}\t{kind}\t{piece.length:.3f}\t{end.x:.3f}\t{end.y:.3f}\t{plen:.3f}\n')
                elif piece.length > args.gap:
                    kind = 'long_bulk' if bulk else 'long_added'
                    counts[kind] += 1
                    mx[kind] = max(mx[kind], piece.length)
                    m = piece.interpolate(0.5, normalized=True)
                    out.write(f'{z:.3f}\t{kind}\t{piece.length:.3f}\t{m.x:.3f}\t{m.y:.3f}\t{plen:.3f}\n')
    out.close()
    print(args.gcode.split('/')[-3], counts, {k: round(v, 2) for k, v in mm.items()}, {k: round(v, 3) for k, v in mx.items()}, over)


if __name__ == '__main__':
    main()

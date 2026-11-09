#!/usr/bin/env python3
"""Independent K2 check on exported G-code, aligned to the hold rule R of the rewrite architecture's 3.7 (stage 5a).

For every sparse-infill path (maximal run of extruding moves of ';FEATURE: Sparse infill' between travels) on layer n with bead
width w, the parts of its centre line off the hold set of layer n - 1 are unsupported. The hold set reads rule R on the G-code:
  (i, ii) a sparse bead below whose centre line passes within w/2 of the point (round ends: a disc about every point of the lower
          centre line; a parallel bead merely touching, centre lines w apart, does not count), or
  (iii)   a non-sparse bead below (walls, solid fill, bridges: a butt-ended rectangle of its ';LINE_WIDTH') whose footprint lies
          within w/2 + tol of the point (tol = 0.01 mm, the touching measure).
A path is measured in the parts between its vertices that lie on another bead of the layer (a junction: the pattern's line ended
on existing material there and its anchor nub begins, or the connector joined two lines). An unsupported piece that reaches an
end of a part is a free end, unless that end is fused alongside another bead of its own layer (centre lines within half the two
widths and tol: an anchor along a perimeter, a nub along the line it meets, a junction). A piece between two held points longer
than G_b = GAP - 0.1 mm is a long run (--exact-gap compares to GAP itself).

Each piece is classified against a reference G-code (Cubic of the same plate): 'bulk' if every segment of it is collinear with a
reference sparse segment on the same layer (angle < 0.5 deg, offset < 0.02 mm, overlapping it in projection); 'connection' if it
runs outside the lattice directions (DIRS, infill_direction 0: 0, 60, 120 degrees) and every segment lies on the line of a
reference segment (the connector's join along the island boundary, which Cubic's own lines get too: accepted boundary behaviour,
reported but not a line of the pattern); otherwise 'added' (a line of the pattern), with 'post' when the whole path is no longer
than 0.6 mm.

Usage: k2check.py GCODE REFERENCE_GCODE GAP OUT.tsv [--tol 0.01] [--exact-gap]
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


def on_reference_line(ax, ay, bx, by, ref, ref_tree, overlap=True):
    """Whether the segment a-b is collinear with a reference segment (angle < 0.5 deg, midpoint offset < 0.02 mm), and, with
    `overlap`, overlapping it in projection."""
    dx, dy = bx - ax, by - ay
    lp = math.hypot(dx, dy)
    if lp < 1e-6:
        return True
    if ref_tree is None:
        return False
    candidates = ref_tree.query(shapely.LineString([(ax, ay), (bx, by)]), predicate='dwithin', distance=0.02) if overlap else range(len(ref))
    for i in candidates:
        r = ref[i]
        rx, ry = r[2] - r[0], r[3] - r[1]
        lr = math.hypot(rx, ry)
        if lr < 1e-6:
            continue
        if abs(dx * ry - dy * rx) / (lp * lr) > math.sin(math.radians(0.5)):
            continue
        mx, my = (ax + bx) / 2, (ay + by) / 2
        if abs((mx - r[0]) * ry - (my - r[1]) * rx) / lr > 0.02:
            continue
        if not overlap:
            return True
        t = ((mx - r[0]) * rx + (my - r[1]) * ry) / (lr * lr)
        if -0.05 <= t <= 1.05:
            return True
    return False


def along_lattice(ax, ay, bx, by):
    ang = math.degrees(math.atan2(by - ay, bx - ax)) % 180.0
    return any(min(abs(ang - d), 180.0 - abs(ang - d)) < 1.5 for d in DIRS)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('gcode')
    ap.add_argument('reference')
    ap.add_argument('gap', type=float)
    ap.add_argument('out')
    ap.add_argument('--tol', type=float, default=0.01)
    ap.add_argument('--exact-gap', action='store_true', help='compare long runs to GAP itself rather than to G_b = GAP - 0.1')
    args = ap.parse_args()
    gap_b = args.gap if args.exact_gap else args.gap - 0.1
    L = parse(args.gcode)
    R = parse(args.reference)
    zs = sorted(L)
    counts = {'paths': 0, 'free_bulk': 0, 'free_added': 0, 'free_connection': 0, 'free_post': 0, 'long_bulk': 0, 'long_added': 0, 'long_connection': 0, 'fused_ends': 0}
    mm = {'free_bulk': 0.0, 'free_added': 0.0, 'free_connection': 0.0, 'free_post': 0.0}
    mx = {'free_bulk': 0.0, 'free_added': 0.0, 'free_connection': 0.0, 'free_post': 0.0, 'long_bulk': 0.0, 'long_added': 0.0, 'long_connection': 0.0}
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
        sparse_below = np.array([s[5] == 'Sparse infill' for s in lower])
        lower_lines = shapely.linestrings(np.stack([low[:, 0:2], low[:, 2:4]], axis=1))
        # (iii): the footprints of the non-sparse beads below, butt-ended rectangles of their own widths.
        other_rects = shapely.buffer(lower_lines[~sparse_below], low[~sparse_below, 4] / 2, cap_style='flat')
        other_rects = other_rects[~shapely.is_empty(other_rects)]
        other_footprint = shapely.union_all(other_rects) if len(other_rects) else None
        sparse_lines = lower_lines[sparse_below]
        # reference sparse segments on this layer
        ref_segs = [s for s in R.get(z, ([], []))[0] if s[5] == 'Sparse infill']
        ref = np.array([s[:4] for s in ref_segs], dtype=float) if ref_segs else np.zeros((0, 4))
        ref_tree = STRtree(shapely.linestrings(np.stack([ref[:, 0:2], ref[:, 2:4]], axis=1))) if len(ref) else None
        # the layer's own beads, for the fused-end test
        own = np.array([s[:5] for s in segs], dtype=float) if segs else np.zeros((0, 5))
        own_lines = shapely.linestrings(np.stack([own[:, 0:2], own[:, 2:4]], axis=1)) if len(own) else None
        own_tree = STRtree(own_lines) if own_lines is not None and len(own_lines) else None
        max_w = float(own[:, 4].max()) if len(own) else 0.0
        widths = sorted({round(p[1], 4) for p in paths})
        holds = {}
        for w in widths:
            parts = []
            if len(sparse_lines):
                parts.append(shapely.union_all(shapely.buffer(sparse_lines, w / 2, cap_style='round')))
            if other_footprint is not None:
                parts.append(shapely.buffer(other_footprint, w / 2 + args.tol, cap_style='round'))
            holds[w] = shapely.union_all(parts) if parts else shapely.Polygon()
        for pts, w in paths:
            counts['paths'] += 1
            plen = shapely.LineString(pts).length
            if plen <= 0:
                continue
            own_vertices = set(pts)

            def fused(end):
                """The end's bead overlaps another bead of its own layer: centre lines within half the two widths and tol."""
                if own_tree is None:
                    return False
                ex, ey = end.x, end.y
                for i in own_tree.query(end, predicate='dwithin', distance=(w + max_w) / 2 + args.tol):
                    s = own[i]
                    if (s[0], s[1]) in own_vertices and (s[2], s[3]) in own_vertices and ((s[0] == ex and s[1] == ey) or (s[2] == ex and s[3] == ey)):
                        continue    # the end's own segment (another part of the same chain counts)
                    if own_lines[i].distance(end) <= (w + s[4]) / 2 + args.tol:
                        return True
                return False

            # The parts of the path between its junction vertices.
            parts, part = [], [pts[0]]
            for i in range(1, len(pts)):
                part.append(pts[i])
                if i + 1 < len(pts) and fused(shapely.Point(pts[i])):
                    parts.append(part)
                    part = [pts[i]]
            if len(part) >= 2:
                parts.append(part)
            pieces = []
            for part in parts:
                line = shapely.LineString(part)
                un = shapely.difference(line, holds[round(w, 4)])
                if un.is_empty:
                    continue
                if hasattr(un, 'geoms'):
                    un = shapely.line_merge(un)
                for piece in (list(un.geoms) if hasattr(un, 'geoms') else [un]):
                    pieces.append((piece, shapely.Point(part[0]), shapely.Point(part[-1])))

            for piece, p0, p1 in pieces:
                if piece.geom_type != 'LineString' or piece.length < 1e-4:
                    continue
                a = shapely.Point(piece.coords[0])
                b = shapely.Point(piece.coords[-1])
                ends = [e for e in (p0, p1) if min(a.distance(e), b.distance(e)) < 1e-6]
                at_end = any(not fused(e) for e in ends)
                if ends and not at_end:
                    counts['fused_ends'] += 1
                coords = list(piece.coords)
                segs_of_piece = [(coords[i][0], coords[i][1], coords[i + 1][0], coords[i + 1][1]) for i in range(len(coords) - 1)]
                bulk = all(on_reference_line(*sg, ref, ref_tree) for sg in segs_of_piece)
                # The connector's join along the island boundary: off the lattice directions, on the line of a reference segment.
                connection = not bulk and all(not along_lattice(*sg) and on_reference_line(*sg, ref, ref_tree, overlap=False) for sg in segs_of_piece)
                (ax_, ay_), (bx_, by_) = coords[0], coords[-1]
                lattice = along_lattice(ax_, ay_, bx_, by_)
                if at_end:
                    if bulk:
                        kind = 'free_bulk'
                    elif connection:
                        kind = 'free_connection'
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
                elif piece.length > gap_b:
                    kind = 'long_bulk' if bulk else ('long_connection' if connection else 'long_added')
                    counts[kind] += 1
                    mx[kind] = max(mx[kind], piece.length)
                    m = piece.interpolate(0.5, normalized=True)
                    out.write(f'{z:.3f}\t{kind}\t{piece.length:.3f}\t{m.x:.3f}\t{m.y:.3f}\t{plen:.3f}\n')
    out.close()
    print(args.gcode.split('/')[-3] if args.gcode.count('/') >= 2 else args.gcode, counts, {k: round(v, 2) for k, v in mm.items()}, {k: round(v, 3) for k, v in mx.items()}, over, 'gap_b', gap_b)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Independent K2 check on exported G-code, aligned to the hold rule R of the rewrite architecture's 3.7 (stage 5a).

For every sparse-infill path (maximal run of extruding moves of ';FEATURE: Sparse infill' between travels, split where the line
width changes) on layer n with bead width w, the parts of its centre line off the hold set of layer n - 1 are unsupported. The hold
set reads rule R on the G-code:
  (i, ii) a sparse bead below whose centre line passes within w/2 of the point (round ends: a disc about every point of the lower
          centre line; a parallel bead merely touching, centre lines w apart, does not count), or
  (iii)   a non-sparse model bead below (walls, solid fill, bridges, gap fill: its path grown by half its ';LINE_WIDTH', flat ends,
          round joins) whose footprint lies within w/2 + tol of the point (tol = 0.01 mm, the touching measure).
Rule (iv), the anchors: an end of a path, or an interior vertex of it, that lies ON another bead of its own layer (its centre within
w_o/2 + tol of that bead's centre line, in the interior of that bead's path, at least w_o/2 + tol from the bead's own ends, and
not on the path's own geometry within one bead's arc length) is fused to it. A path is measured in the parts between its fused
vertices (junctions: the pattern's line ended on existing material and its nub begins). An unsupported piece that reaches an end
of a part is a free end unless that end is fused; a piece between two held points longer than G_b = GAP - 0.1 mm is a long run
(--exact-gap compares to GAP itself).

Each piece is classified against a reference G-code (Cubic of the same plate): 'bulk' if every segment of it lies on the line of
a reference sparse segment on the same layer (angle < 0.5 deg, offset < 0.02 mm; the two slices' islands differ where the bridge
generator took sparse area, so the extents are not compared: a piece on a bulk line's line lies on that printed line);
'connection' if it runs outside the three lattice directions (the G-code's '; infill_direction' + 0, 60, 120 degrees) and every
segment lies on the line of a reference segment (the connector's join along the island boundary, which Cubic's own lines get too:
accepted boundary behaviour, reported but not a line of the pattern); otherwise 'added' (a line of the pattern), with 'post' when
the whole path is no longer than 0.6 mm.

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
MODEL_FEATURES = {'Outer wall', 'Inner wall', 'Overhang wall', 'Sparse infill', 'Internal solid infill', 'Top surface',
                  'Bottom surface', 'Bridge', 'Internal Bridge', 'Gap infill', 'Floating vertical shell'}


def parse(path):
    """{z: paths [(points, width, feature)]} for the model features, plus the infill direction (degrees) from the config block."""
    layers = {}
    direction = 0.0
    z = None
    feat = ''
    width = 0.4
    x = y = 0.0
    relative_e = True
    cur = None
    cur_w = 0.0
    cur_f = ''

    def close():
        nonlocal cur
        if cur is not None and len(cur) >= 2 and z is not None:
            layers[z].append((cur, cur_w, cur_f))
        cur = None

    with open(path, errors='replace') as f:
        for line in f:
            c = line[0]
            if c == ';':
                if line.startswith('; Z_HEIGHT:'):
                    close()
                    z = round(float(line.split(':')[1]), 3)
                    layers.setdefault(z, [])
                elif line.startswith('; FEATURE:'):
                    close()
                    feat = line[10:].strip()
                elif line.startswith('; LINE_WIDTH:'):
                    w = float(line.split(':')[1])
                    if w != width:
                        close()
                    width = w
                elif line.startswith('; infill_direction = '):
                    direction = float(line.split('=')[1])
                continue
            if line.startswith('M82'):
                relative_e = False
                continue
            if line.startswith('M83'):
                relative_e = True
                continue
            if not line.startswith(('G0 ', 'G1 ', 'G2 ', 'G3 ', 'G92 ')):
                continue
            nx, ny, e = x, y, None
            for a, v in AX.findall(line.split(';')[0]):
                if a == 'X':
                    nx = float(v)
                elif a == 'Y':
                    ny = float(v)
                elif a == 'E':
                    e = float(v)
            if line.startswith('G92'):
                close()
                continue
            if not relative_e:
                sys.exit('k2check: absolute E distances are not supported (the acceptance exports with relative E)')
            moved = nx != x or ny != y
            if moved and e is not None and e > 0 and z is not None:
                if line.startswith(('G2 ', 'G3 ')):
                    sys.exit('k2check: an extruding arc move; export with arc fitting off')
                if feat in MODEL_FEATURES:
                    if cur is None:
                        cur = [(x, y)]
                        cur_w = width
                        cur_f = feat
                    cur.append((nx, ny))
                else:
                    close()
            elif moved or (e is not None and e != 0):
                close()
            x, y = nx, ny
    close()
    return layers, direction


def on_reference_line(ax, ay, bx, by, ref, ref_tree, overlap=True):
    """Whether the segment a-b is collinear with a reference segment (angle < 0.5 deg, midpoint offset < 0.02 mm), and, with
    `overlap`, overlapping it in projection within 0.2 mm."""
    dx, dy = bx - ax, by - ay
    lp = math.hypot(dx, dy)
    if lp < 1e-6:
        return True
    if ref_tree is None:
        return False
    candidates = ref_tree.query(shapely.LineString([(ax, ay), (bx, by)]), predicate='dwithin', distance=0.25) if overlap else range(len(ref))
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
        t = ((mx - r[0]) * rx + (my - r[1]) * ry) / lr
        if -0.2 <= t <= lr + 0.2:
            return True
    return False


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
    L, direction = parse(args.gcode)
    R, _ = parse(args.reference)
    dirs = [(direction + k * 60.0) % 180.0 for k in range(3)]

    def along_lattice(ax, ay, bx, by):
        ang = math.degrees(math.atan2(by - ay, bx - ax)) % 180.0
        return any(min(abs(ang - d), 180.0 - abs(ang - d)) < 1.5 for d in dirs)

    zs = sorted(L)
    counts = {'paths': 0, 'free_bulk': 0, 'free_added': 0, 'free_connection': 0, 'free_post': 0, 'long_bulk': 0, 'long_added': 0, 'long_connection': 0, 'fused_ends': 0}
    mm = {'free_bulk': 0.0, 'free_added': 0.0, 'free_connection': 0.0, 'free_post': 0.0}
    mx = {'free_bulk': 0.0, 'free_added': 0.0, 'free_connection': 0.0, 'free_post': 0.0, 'long_bulk': 0.0, 'long_added': 0.0, 'long_connection': 0.0}
    over = {'free_added>0.25': 0, 'free_post>0.25': 0, 'free_added>0.5': 0, 'free_post>0.5': 0, 'free_connection>0.25': 0, 'free_connection>0.5': 0}
    out = open(args.out, 'w')
    out.write('z\tkind\tlength\tx\ty\tpath_length\n')
    for k in range(1, len(zs)):
        z, zb = zs[k], zs[k - 1]
        paths = L[z]
        sparse_paths = [(pts, w) for pts, w, f in paths if f == 'Sparse infill']
        if not sparse_paths:
            continue
        lower = L[zb]
        if not lower:
            continue
        # The hold set: (i, ii) the lower sparse centre lines grown by w/2 with round ends; (iii) the lower non-sparse footprints
        # (flat ends, round joins) grown by w/2 + tol.
        # The lower sparse paths: their segments along the lattice directions hold by their centre lines (rules i and ii); a
        # segment off the lattice directions is a bead the pattern prints whole (a seam wall, a rail) or a connector's hook, and
        # holds as material does, by its footprint (rule iii).
        # The reference's sparse segments on the lower layer, for the hooks along the island boundary.
        ref_lower_rows = []
        for pts, w, f in R.get(zb, []):
            if f != 'Sparse infill':
                continue
            for i in range(1, len(pts)):
                ref_lower_rows.append((pts[i - 1][0], pts[i - 1][1], pts[i][0], pts[i][1]))
        ref_lower = np.array(ref_lower_rows, dtype=float) if ref_lower_rows else np.zeros((0, 4))
        ref_lower_tree = STRtree(shapely.linestrings(np.stack([ref_lower[:, 0:2], ref_lower[:, 2:4]], axis=1))) if len(ref_lower) else None
        lower_sparse, lower_beads = [], []
        for pts, w, f in lower:
            if f != 'Sparse infill':
                continue
            for i in range(1, len(pts)):
                seg = shapely.LineString([pts[i - 1], pts[i]])
                if seg.length < 1e-6:
                    continue
                sg = (pts[i - 1][0], pts[i - 1][1], pts[i][0], pts[i][1])
                # Off the lattice directions and at least a bead long, or on a reference line: a course or a hook, held by its
                # footprint; a short off-lattice hop (a nub's step, a join link) holds by its centre line.
                if not along_lattice(*sg) and (seg.length >= w - 1e-4 or on_reference_line(*sg, ref_lower, ref_lower_tree, overlap=False)):
                    lower_beads.append(seg.buffer(w / 2, cap_style='round'))
                else:
                    lower_sparse.append(seg)
        other_footprints = [shapely.LineString(pts).buffer(w / 2, cap_style='flat', join_style='round') for pts, w, f in lower if f != 'Sparse infill'] + lower_beads
        other_footprint = shapely.union_all(other_footprints) if other_footprints else None
        # reference sparse segments on this layer
        ref_rows = []
        for pts, w, f in R.get(z, []):
            if f != 'Sparse infill':
                continue
            for i in range(1, len(pts)):
                ref_rows.append((pts[i - 1][0], pts[i - 1][1], pts[i][0], pts[i][1]))
        ref = np.array(ref_rows, dtype=float) if ref_rows else np.zeros((0, 4))
        ref_tree = STRtree(shapely.linestrings(np.stack([ref[:, 0:2], ref[:, 2:4]], axis=1))) if len(ref) else None
        # the layer's own beads, as paths, for the fused test
        own_paths = [(shapely.LineString(pts), pts, w) for pts, w, f in paths]
        own_tree = STRtree([p[0] for p in own_paths]) if own_paths else None
        max_w = max((w for pts, w, f in paths), default=0.0)
        widths = sorted({round(p[1], 4) for p in sparse_paths})
        holds = {}
        for w in widths:
            parts = []
            if lower_sparse:
                parts.append(shapely.union_all([ls.buffer(w / 2, cap_style='round') for ls in lower_sparse]))
            if other_footprint is not None:
                parts.append(other_footprint.buffer(w / 2 + args.tol, cap_style='round'))
            holds[w] = shapely.union_all(parts) if parts else shapely.Polygon()
        for pts, w in sparse_paths:
            counts['paths'] += 1
            plen = shapely.LineString(pts).length
            if plen <= 0:
                continue
            orig_pts = pts   # the path as emitted: the own entry of own_paths
            # The arc length from each vertex to the path's start, to exclude the path's own geometry near a vertex.
            arc = [0.0]
            for i in range(1, len(pts)):
                arc.append(arc[-1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))

            closed = len(pts) > 2 and pts[0] == pts[-1]   # a course laid as a loop has no ends: arc distances go round

            def own_distance(a, b):
                d = abs(a - b)
                return min(d, arc[-1] - d) if closed else d

            def fused(vi):
                """Vertex vi lies on another bead of the layer: within w_o/2 + tol of its centre line, in the interior of that
                bead's path (w_o/2 + tol from its ends; a loop has none); the path's own geometry within one bead's arc length does
                not count."""
                if own_tree is None:
                    return False
                v = shapely.Point(pts[vi])
                for j in own_tree.query(v, predicate='dwithin', distance=max_w / 2 + args.tol):
                    line, opts, ow = own_paths[j]
                    if opts is orig_pts:
                        # Own path: only parts farther than one bead along the path, and in the interior of that part.
                        near = False
                        for i in range(1, len(pts)):
                            if min(own_distance(arc[i - 1], arc[vi]), own_distance(arc[i], arc[vi])) <= w + args.tol:
                                continue
                            seg = shapely.LineString([pts[i - 1], pts[i]])
                            if seg.distance(v) <= ow / 2 + args.tol:
                                foot = arc[i - 1] + seg.project(v)
                                if closed or (foot >= ow / 2 + args.tol and arc[-1] - foot >= ow / 2 + args.tol):
                                    near = True
                                    break
                        if near:
                            return True
                        continue
                    if line.distance(v) > ow / 2 + args.tol:
                        continue
                    # In the interior of the met path: not within w_o/2 + tol of its ends (a loop has none).
                    foot = line.project(v)
                    other_closed = len(opts) > 2 and opts[0] == opts[-1]
                    if not other_closed and (foot < ow / 2 + args.tol or line.length - foot < ow / 2 + args.tol):
                        continue
                    return True
                return False

            def is_hook(i):
                """The segment ending at vertex i is a connector hook along the island boundary: off the lattice directions, on
                the line of a reference segment (Cubic's lines hook along the same boundary)."""
                sg = (pts[i - 1][0], pts[i - 1][1], pts[i][0], pts[i][1])
                return not along_lattice(*sg) and on_reference_line(*sg, ref, ref_tree, overlap=False)

            inserted = None   # a pivot inside a segment: the measure's first and last point, not a vertex of the path
            if closed:
                # Measured from the first point where the loop is held (where it enters the hold set: a vertex or a point inside a
                # segment) or a fused vertex, round to itself, so that a stretch through the start point is one stretch; that point
                # counts as fused at both ends. A loop with no held and no fused point is held nowhere: one free piece.
                hold_w = holds[round(w, 4)]
                ring = len(pts) - 1
                entry = None
                for i in range(ring):
                    if hold_w.covers(shapely.Point(pts[i])) or fused(i):
                        entry = (i, tuple(pts[i]))
                        break
                    inside = shapely.intersection(shapely.LineString([pts[i], pts[i + 1]]), hold_w)
                    if not inside.is_empty:
                        q = min((c for g in (getattr(inside, 'geoms', None) or [inside]) for c in g.coords),
                                key=lambda c: math.hypot(c[0] - pts[i][0], c[1] - pts[i][1]), default=None)
                        if q is not None:
                            entry = (i, (q[0], q[1]))
                            break
                if entry is None:
                    coords = [(x, y) for x, y in pts]
                    segs_of_loop = [(coords[i][0], coords[i][1], coords[i + 1][0], coords[i + 1][1]) for i in range(len(coords) - 1)]
                    if not all(on_reference_line(*sg, ref, ref_tree, overlap=False) for sg in segs_of_loop):
                        kind = 'free_post' if plen <= 0.6 else 'free_added'
                        counts[kind] += 1
                        mm[kind] += plen
                        mx[kind] = max(mx[kind], plen)
                        over[kind + '>0.25'] += plen > 0.25
                        over[kind + '>0.5'] += plen > 0.5
                        out.write(f'{z:.3f}\t{kind}\t{plen:.3f}\t{pts[0][0]:.3f}\t{pts[0][1]:.3f}\t{plen:.3f}\n')
                    continue
                i, q = entry
                rotated = [q] + [tuple(pts[(i + j) % ring]) for j in range(1, ring + 1)]
                if q != tuple(pts[i]):
                    rotated.append(q)
                    inserted = q
                pts = rotated
                arc = [0.0]
                for k in range(1, len(pts)):
                    arc.append(arc[-1] + math.hypot(pts[k][0] - pts[k - 1][0], pts[k][1] - pts[k - 1][1]))
            hooks = [False] + [is_hook(i) for i in range(1, len(pts))]
            if closed:
                hooks[0] = hooks[-1]   # the segment into the start point is the loop's last
            # A junction: a vertex fused to another bead, or the vertex where a lattice line turns into a connector hook along the
            # boundary (Cubic's boundary behaviour, accepted by the user): the hook is measured as a connection piece of its own
            # and the line's end at the junction is not free.
            fused_vertex = [(closed and (i == 0 or i == len(pts) - 1)) or fused(i) or (0 < i < len(pts) - 1 and hooks[i] != hooks[i + 1]) for i in range(len(pts))]
            # The parts of the path between its junction vertices.
            parts, part, part_start = [], [pts[0]], 0
            for i in range(1, len(pts)):
                part.append(pts[i])
                if i + 1 < len(pts) and fused_vertex[i]:
                    parts.append((part, part_start, i))
                    part, part_start = [pts[i]], i
            if len(part) >= 2:
                parts.append((part, part_start, len(pts) - 1))
            pieces = []
            for part, i0, i1 in parts:
                line = shapely.LineString(part)
                un = shapely.difference(line, holds[round(w, 4)])
                if un.is_empty:
                    continue
                if hasattr(un, 'geoms'):
                    un = shapely.line_merge(un)
                for piece in (list(un.geoms) if hasattr(un, 'geoms') else [un]):
                    pieces.append((piece, shapely.Point(part[0]), fused_vertex[i0], shapely.Point(part[-1]), fused_vertex[i1]))

            for piece, p0, fused0, p1, fused1 in pieces:
                if piece.geom_type != 'LineString' or piece.length < 1e-4:
                    continue
                a = shapely.Point(piece.coords[0])
                b = shapely.Point(piece.coords[-1])
                free_at = None
                for end, is_fused in ((p0, fused0), (p1, fused1)):
                    if min(a.distance(end), b.distance(end)) < 1e-6:
                        if is_fused:
                            counts['fused_ends'] += 1
                        elif free_at is None:
                            free_at = end
                coords = list(piece.coords)
                segs_of_piece = [(coords[i][0], coords[i][1], coords[i + 1][0], coords[i + 1][1]) for i in range(len(coords) - 1)]
                bulk = all(on_reference_line(*sg, ref, ref_tree, overlap=False) for sg in segs_of_piece)
                # The connector's join along the island boundary: off the lattice directions, on the line of a reference segment.
                connection = not bulk and all(not along_lattice(*sg) and on_reference_line(*sg, ref, ref_tree, overlap=False) for sg in segs_of_piece)
                if free_at is not None:
                    if bulk:
                        kind = 'free_bulk'
                    elif connection:
                        kind = 'free_connection'
                    elif plen <= 0.6:
                        kind = 'free_post'
                    else:
                        kind = 'free_added'
                    counts[kind] += 1
                    mm[kind] += piece.length
                    mx[kind] = max(mx[kind], piece.length)
                    if kind != 'free_bulk':
                        over[kind + '>0.25'] += piece.length > 0.25
                        over[kind + '>0.5'] += piece.length > 0.5
                    out.write(f'{z:.3f}\t{kind}\t{piece.length:.3f}\t{free_at.x:.3f}\t{free_at.y:.3f}\t{plen:.3f}\n')
                elif piece.length > gap_b:
                    kind = 'long_bulk' if bulk else ('long_connection' if connection else 'long_added')
                    counts[kind] += 1
                    mx[kind] = max(mx[kind], piece.length)
                    m = piece.interpolate(0.5, normalized=True)
                    out.write(f'{z:.3f}\t{kind}\t{piece.length:.3f}\t{m.x:.3f}\t{m.y:.3f}\t{plen:.3f}\n')
    out.close()
    print(args.gcode.split('/')[-3] if args.gcode.count('/') >= 2 else args.gcode, counts, {k: round(v, 2) for k, v in mm.items()}, {k: round(v, 3) for k, v in mx.items()}, over, 'gap_b', gap_b, 'infill_direction', direction)


if __name__ == '__main__':
    main()

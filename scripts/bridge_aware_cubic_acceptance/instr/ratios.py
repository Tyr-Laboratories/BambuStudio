"""ratios.py MATRIX.tsv: per fixture, slice instructions (--slice 1 minus --slice 99 of the same app, fixture and iteration),
the BAC/official ratio per iteration (median and worst), fork Cubic/official parity, and BAC's added slice instructions as a
fraction of official Cubic's slice ([BACTiming]'s gate is 0.55)."""
import sys, statistics as st, collections
rows = [l.rstrip('\n').split('\t') for l in open(sys.argv[1]) if l.strip()]
d = collections.defaultdict(dict)
for it, size, app, pat, mode, rc, ins, cyc in rows:
    d[(size, app, pat, mode)][int(it)] = int(ins)
G = 1e9
for size in ('small', 'large', 'rib', 'modifier'):
    sl = {}
    for key in (('official', 'cubic'), ('fork', 'cubic'), ('fork', 'bridgeawarecubic')):
        s, l = d[(size,) + key + ('slice',)], d[(size,) + key + ('load',)]
        sl[key] = {i: s[i] - l[i] for i in s if i in l}
    off, fc, fb = sl[('official', 'cubic')], sl[('fork', 'cubic')], sl[('fork', 'bridgeawarecubic')]
    pairs = [fb[i] / off[i] for i in off if i in fb]
    parity = [fc[i] / off[i] for i in off if i in fc]
    added = (st.median(fb.values()) - st.median(fc.values())) / st.median(off.values())
    print(f'{size:9s} official slice {st.median(off.values())/G:6.3f} G  fork Cubic {st.median(fc.values())/G:6.3f} G  BAC {st.median(fb.values())/G:6.3f} G  '
          f'BAC/official median {st.median(pairs):.3f} worst {max(pairs):.3f} (gate 2.0)  parity {st.median(parity):.3f}  added/official {added:.3f} (gate 0.55)')

import sys, collections
# The free ends of a plate (bac-pN-lines.tsv) by class: column pieces (free_post) and lattice pieces (free_added) by what is nearest within
# 1.5 mm on their own layer or the layer below (walls, solid surfaces, nothing); connections and bulk as they are.
walls = {'Outer wall', 'Inner wall', 'Overhang wall'}
total = collections.Counter()
for path in sys.argv[1:]:
    for i, line in enumerate(open(path)):
        if i == 0: continue
        r = line.rstrip('\n').split('\t')
        kind = r[1]; here, dh, under, du, line_kind = r[6], float(r[7]), r[8], float(r[9]), r[11]
        near = []
        if dh < 1.5: near.append((dh, here))
        if du < 1.5: near.append((du, under))
        cls = 'nothing within 1.5 mm'
        if near:
            f = min(near)[1]
            cls = 'wall' if f in walls else 'solid (' + f + ')'
        if kind in ('free_post', 'free_added'):
            total[(kind, line_kind, cls if cls in ('wall', 'nothing within 1.5 mm') else 'solid surface')] += 1
        else:
            total[(kind, line_kind, '')] += 1
for k, v in sorted(total.items()): print('\t'.join(k), v, sep='\t')

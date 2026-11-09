import re, sys, math
# Metrics of one exported G-code file (relative E): the header's total grams and estimated times, grams per ; FEATURE: (positive E on
# XY moves, with the header's filament density and diameter: feature_mm3.py's method), the number of sparse-infill paths (runs of
# extruding XY moves inside the Sparse infill feature, broken by any move that does not extrude, a retraction or a feature change), the
# sparse paths of 0.6 mm or less, and the retractions (counted by their unretracts: G1 moves with a positive E and no X, Y or Z).
def seconds(text):
    total = 0
    for value, unit in re.findall(r'(\d+)([dhms])', text):
        total += int(value) * {'d': 86400, 'h': 3600, 'm': 60, 's': 1}[unit]
    return total
path = sys.argv[1]
dens = diam = None; feat = 'none'; tot = {}
grams = None; model_time = total_time = None
sparse_paths = short_paths = unretracts = 0
in_path = False; path_len = 0.0; x = y = None
def close_path():
    global in_path, path_len, short_paths
    if in_path and feat_of_path == 'Sparse infill' and path_len <= 0.6:
        short_paths += 1
    in_path = False; path_len = 0.0
feat_of_path = None
with open(path, errors='replace') as f:
    for line in f:
        if line.startswith(';'):
            if line.startswith('; filament_density'): dens = float(re.split('[:=]', line)[1].split(',')[0])
            elif line.startswith('; filament_diameter'): diam = float(re.split('[:=]', line)[1].split(',')[0])
            elif line.startswith('; total filament weight') and grams is None: grams = float(line.split(':')[1].split(',')[0])
            elif line.startswith('; model printing time') and model_time is None:
                parts = line.split(';')
                model_time = seconds(parts[1].split(':', 1)[1])
                total_time = seconds(parts[2].split(':', 1)[1])
            elif line.startswith('; FEATURE:'):
                close_path(); feat = line[10:].strip()
            continue
        if not line.startswith(('G1', 'G0', 'G2', 'G3')):
            continue
        mx = re.search(r' X(-?[0-9.]+)', line); my = re.search(r' Y(-?[0-9.]+)', line); me = re.search(r' E(-?[0-9.]+)', line)
        nx = float(mx.group(1)) if mx else x; ny = float(my.group(1)) if my else y
        e = float(me.group(1)) if me else 0.0
        if mx or my:
            if e > 0:
                tot[feat] = tot.get(feat, 0.0) + e
                if not in_path:
                    in_path = True; feat_of_path = feat; path_len = 0.0
                    if feat == 'Sparse infill': sparse_paths += 1
                if x is not None and y is not None: path_len += math.hypot(nx - x, ny - y)
            else:
                close_path()
            x, y = nx, ny
        else:
            if e > 0 and ' Z' not in line: unretracts += 1
            if e != 0: close_path()
close_path()
area = math.pi * (diam / 2) ** 2
g = {k: v * area * dens / 1000.0 for k, v in tot.items()}
print(f"grams={grams} sparse={g.get('Sparse infill', 0.0):.2f} model_time_s={model_time} total_time_s={total_time} "
      f"sparse_paths={sparse_paths} short_sparse_paths={short_paths} retractions={unretracts} " +
      ' '.join(f"{k.replace(' ', '_')}={v:.2f}" for k, v in sorted(g.items())))

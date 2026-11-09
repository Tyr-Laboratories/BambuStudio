"""G-code reader for BX: per layer (Z_HEIGHT), per feature, the extruding XY moves as polylines with their width and object id.
Cached as a pickle beside the G-code's copy in scratchpad/bx/cache. Relative E (the Heimdall exports use M83)."""
import re, os, sys, pickle, collections, math
AX = re.compile(r'([XYZE])(-?[0-9]*\.?[0-9]+)')
CACHE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'cache')

def parse(path):
    path = os.path.abspath(path)
    key = path.replace('/', '_')
    cp = os.path.join(CACHE, key + '.pkl')
    if os.path.exists(cp) and os.path.getmtime(cp) > os.path.getmtime(path):
        with open(cp, 'rb') as f:
            return pickle.load(f)
    # layers: z -> list of paths; path = (feature, width, object_id, [(x,y),...], e_total)
    layers = collections.defaultdict(list)
    z = None; feat = ''; w = 0.4; obj = -1; x = y = 0.0
    cur = None  # current path being built
    def close():
        nonlocal cur
        if cur is not None and len(cur[3]) >= 2:
            layers[cur[5]].append(tuple(cur[:5]))
        cur = None
    with open(path, 'r', errors='replace') as f:
        for line in f:
            if line[0] == ';':
                if line.startswith('; FEATURE:'):
                    close(); feat = line[10:].strip()
                elif line.startswith('; Z_HEIGHT:'):
                    close(); z = round(float(line[11:]), 3)
                elif line.startswith('; LINE_WIDTH:'):
                    close(); w = float(line[13:])
                elif line.startswith('; start printing object, unique label id:'):
                    close(); obj = int(line.rsplit(':', 1)[1])
                continue
            if not (line.startswith('G1 ') or line.startswith('G0 ') or line.startswith('G2 ') or line.startswith('G3 ')):
                if line.startswith('M625'):
                    close()
                continue
            d = dict((k, float(v)) for k, v in AX.findall(line.split(';', 1)[0]))
            nx, ny = d.get('X', x), d.get('Y', y); e = d.get('E', 0.0)
            moved = (nx != x or ny != y)
            if moved and e > 0 and z is not None:
                if cur is None:
                    cur = [feat, w, obj, [(x, y)], 0.0, z]
                cur[3].append((nx, ny)); cur[4] += e
            elif moved or e < 0:
                close()
            x, y = nx, ny
    close()
    layers = dict(layers)
    os.makedirs(CACHE, exist_ok=True)
    with open(cp, 'wb') as f:
        pickle.dump(layers, f, protocol=pickle.HIGHEST_PROTOCOL)
    return layers

def header_density(path):
    dens = diam = None
    with open(path, errors='replace') as f:
        for line in f:
            if line.startswith('; filament_density'): dens = float(re.split('[:=]', line)[1].split(',')[0])
            elif line.startswith('; filament_diameter'): diam = float(re.split('[:=]', line)[1].split(',')[0])
            if dens and diam: break
    return dens, diam

def grams_per_mm_e(path):
    dens, diam = header_density(path)
    return math.pi * (diam / 2) ** 2 * dens / 1000.0

if __name__ == '__main__':
    for p in sys.argv[1:]:
        L = parse(p)
        print(p, len(L), 'layers')

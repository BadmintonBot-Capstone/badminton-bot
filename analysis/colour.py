# colour of the labelled shuttle points vs the labelled false points, to pick a colour check
#   colour.py <dir> [<dir> ...]
# runs the same running background as the detector plus a colour one, and at every labelled point
# measures colour in a small window weighted by how much each pixel changed
import sys, csv, pathlib, numpy as np, cv2
from rec import load

ALPHA, BLUR, THRESH, WIN = 0.05, 1.5, 12, 5   # WIN is the half width of the window in half res px

def rgb2(f):
    # half res colour straight from the BayerRG mosaic: R top left, G averaged, B bottom right
    f = f.astype(np.float32)
    return np.dstack([f[0::2, 0::2], 0.5 * (f[0::2, 1::2] + f[1::2, 0::2]), f[1::2, 1::2]])

def grey(c):
    # same weights as the detector's 2x2 average: (R + 2G + B) / 4
    return 0.25 * (c[..., 0] + 2 * c[..., 1] + c[..., 2])

def read_pts(p):
    out = {}
    if p.exists():
        for r in csv.DictReader(open(p)):
            out.setdefault(int(r['frame']), []).append((float(r['x']) / 2, float(r['y']) / 2))
    return out

def features(c, cb, w):
    s = w.sum()
    if s <= 0: return None
    cur = (c * w[..., None]).sum((0, 1)) / s
    d = ((c - cb) * w[..., None]).sum((0, 1)) / s
    R, G, B = cur
    hsv = cv2.cvtColor(np.uint8([[np.clip(cur[::-1], 0, 255)]]), cv2.COLOR_BGR2HSV)[0, 0]
    dR, dG, dB = d
    return dict(cur_yel=((R + G) / 2 - B) / ((R + G) / 2 + B + 1), hue=float(hsv[0]), sat=float(hsv[1]), val=float(hsv[2]),
                chg_yel=((dR + dG) / 2 - dB) / (abs(dR) + abs(dG) + abs(dB) + 1), chg_bright=(dR + dG + dB) / 3,
                dB=dB, dRG=(dR + dG) / 2)

rows = []
for d in map(pathlib.Path, sys.argv[1:]):
    f, m = load(d)
    truth, false = read_pts(d / 'truth.csv'), read_pts(d / 'false.csv')
    want = set(truth) | set(false)
    c = rgb2(f[0]); gb = grey(c); cb = c.copy()
    for i in range(len(f)):
        c = rgb2(f[i]); gr = grey(c)
        if i in want:
            diff = cv2.GaussianBlur(np.abs(gr - gb), (0, 0), BLUR)
            for lab, pts in (('shuttle', truth.get(i, [])), ('false', false.get(i, []))):
                for x, y in pts:
                    x0, y0 = int(round(x)), int(round(y))
                    ys, xs = slice(max(y0 - WIN, 0), y0 + WIN + 1), slice(max(x0 - WIN, 0), x0 + WIN + 1)
                    w = np.clip(diff[ys, xs] - THRESH, 0, None)
                    ft = features(c[ys, xs], cb[ys, xs], w)
                    if ft: rows.append(dict(rec=d.name, frame=i, label=lab, **ft))
        gb += ALPHA * (gr - gb)
        cb += ALPHA * (c - cb)

keys = [k for k in rows[0] if k not in ('rec', 'frame', 'label')]
with open('colour_points.csv', 'w') as fo:
    wr = csv.DictWriter(fo, fieldnames=list(rows[0])); wr.writeheader(); wr.writerows(rows)
S = [r for r in rows if r['label'] == 'shuttle']; F = [r for r in rows if r['label'] == 'false']
print(f'{len(S)} shuttle points, {len(F)} false points')
for k in keys:
    s = np.array([r[k] for r in S]); fa = np.array([r[k] for r in F])
    # best single threshold, either direction, scored by how many false points go while keeping 98% of the shuttle
    lo = np.percentile(s, 2); hi = np.percentile(s, 98)
    kill_lo = (fa < lo).mean(); kill_hi = (fa > hi).mean()
    print(f'{k:>10}  shuttle p2/p50/p98 {lo:7.2f} {np.median(s):7.2f} {hi:7.2f}   false p50 {np.median(fa):7.2f}   '
          f'keeping 98% of shuttle drops {100 * max(kill_lo, kill_hi):.0f}% of false ({"keep above p2" if kill_lo >= kill_hi else "keep below p98"})')

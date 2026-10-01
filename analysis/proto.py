# prototype: running background, blurred |diff|, blobs, 2d constant-velocity tracker
import sys, json, numpy as np, cv2
from rec import load, gray2, bgr

P = dict(alpha=0.05, blur=1.5, thresh=12, min_area=4, max_area=1500, big_area=3000,
         gate=25, gate_v=0.6, init_tol=12, min_speed=3, init_speed=8, coast=4, still_frames=8)

def blobs(g, B, P):
    d = cv2.GaussianBlur(np.abs(g - B), (0, 0), P['blur'])
    mask = (d > P['thresh']).astype(np.uint8)
    n, lab, st, cen = cv2.connectedComponentsWithStats(mask, connectivity=8)
    cands, bigmask = [], np.zeros_like(mask)
    for k in range(1, n):
        a = st[k, cv2.CC_STAT_AREA]
        if a > P['big_area']:
            bigmask[lab == k] = 1
        elif P['min_area'] <= a <= P['max_area']:
            cands.append((cen[k][0], cen[k][1], a, float(d[lab == k].max())))
    return cands, mask, d

class Tracker:
    def __init__(s, P): s.P, s.hist, s.state, s.miss, s.p, s.v, s.still = P, [], 'search', 0, None, None, 0
    def step(s, cands):
        P = s.P
        if s.state == 'track':
            pred = s.p + s.v
            r = P['gate'] + P['gate_v'] * np.linalg.norm(s.v)
            best = min(cands, key=lambda c: np.hypot(c[0] - pred[0], c[1] - pred[1]), default=None)
            if best is not None and np.hypot(best[0] - pred[0], best[1] - pred[1]) < r:
                q = np.array(best[:2]); s.v = 0.5 * s.v + 0.5 * (q - s.p); s.p = q; s.miss = 0
                s.still = s.still + 1 if np.linalg.norm(s.v) < P['min_speed'] else 0
                if s.still > P['still_frames']: s.state, s.hist = 'search', []; return ('lost', None)
                return ('hit', q)
            s.miss += 1; s.p = pred
            if not (-20 < pred[0] < 980 and -20 < pred[1] < 620): s.miss = P['coast'] + 1
            if s.miss > P['coast']: s.state, s.hist = 'search', []
            return ('coast', pred) if s.state == 'track' else ('lost', None)
        # search: keep candidate history of the last 3 frames, look for a straight fast triple
        s.hist = (s.hist + [cands])[-3:]
        if len(s.hist) == 3:
            for c3 in s.hist[2]:
                for c2 in s.hist[1]:
                    v = np.array(c3[:2]) - np.array(c2[:2])
                    if np.linalg.norm(v) < P['init_speed'] or np.linalg.norm(v) > 150: continue
                    for c1 in s.hist[0]:
                        if np.linalg.norm(np.array(c2[:2]) - np.array(c1[:2]) - v) < P['init_tol']:
                            s.state, s.p, s.v, s.miss = 'track', np.array(c3[:2]), v, 0
                            return ('init', s.p)
        return ('search', None)

def run(d, P=P, start=0, stop=None):
    f, m = load(d)
    stop = stop or len(f)
    B = gray2(f[start]).copy()
    trk, out = Tracker(P), []
    for i in range(start, stop):
        g = gray2(f[i])
        cands, mask, dd = blobs(g, B, P)
        st, p = trk.step(cands)
        fg = mask.astype(bool)
        B = (1 - P['alpha']) * B + P['alpha'] * g
        out.append(dict(i=i, state=st, p=None if p is None else [float(p[0]), float(p[1])], n=len(cands)))
    return out

if __name__ == '__main__':
    d = sys.argv[1]
    out = run(d)
    json.dump(out, open(f'{d}/proto.json', 'w'))
    states = [o['state'] for o in out]
    print({k: states.count(k) for k in set(states)})
    segs, cur = [], None
    for o in out:
        if o['state'] in ('init', 'hit', 'coast'):
            if cur is None: cur = [o['i'], o['i']]
            cur[1] = o['i']
        elif cur: segs.append(cur); cur = None
    if cur: segs.append(cur)
    print('track segments (start,end,len):', [(a, b, b - a + 1) for a, b in segs])

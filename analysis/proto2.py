# prototype v2: same detector, multi-track manager, a track only reports once confirmed
import sys, json, numpy as np, cv2
from rec import load, gray2

def blobs2(g, B, P):
    d = cv2.GaussianBlur(np.abs(g - B), (0, 0), P['blur'])
    mask = (d > P['thresh']).astype(np.uint8)
    n, lab, st, cen = cv2.connectedComponentsWithStats(mask, connectivity=8)
    big = np.isin(lab, [k for k in range(1, n) if st[k, cv2.CC_STAT_AREA] > P['big_area']]).astype(np.uint8)
    if big.any():
        r = P['near_big']; big = cv2.dilate(big, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1)))
    cands, near = [], []
    for k in range(1, n):
        a = st[k, cv2.CC_STAT_AREA]
        if P['min_area'] <= a <= P['max_area']:
            cands.append((cen[k][0], cen[k][1], a)); near.append(bool(big[int(cen[k][1]), int(cen[k][0])]))
    return cands, near

P = dict(alpha=0.05, blur=1.5, thresh=12, min_area=4, max_area=900, big_area=900, near_big=40,
         gate=25, gate_v=0.6, spawn_r=150, confirm_hits=4, confirm_speed=8, still_speed=3,
         still_frames=8, coast=4, W=960, H=600, window=8, max_jerk=0.5, min_travel=60)

class Track:
    def __init__(s, c, i):
        s.p, s.v, s.hits, s.miss, s.speeds, s.still, s.born, s.confirmed, s.steps = np.array(c[:2]), None, 1, 0, [], 0, i, False, []
        s.origin = s.p.copy()
    def pred(s): return s.p if s.v is None else s.p + s.v

class Manager:
    def __init__(s, P): s.P, s.tracks = P, []
    def step(s, cands, i, near):
        P = s.P
        pairs = []
        for ti, t in enumerate(s.tracks):
            pr = t.pred()
            r = P['spawn_r'] if t.v is None else P['gate'] + P['gate_v'] * np.linalg.norm(t.v)
            for ci, c in enumerate(cands):
                dd = np.hypot(c[0] - pr[0], c[1] - pr[1])
                if dd < r: pairs.append((dd, ti, ci))
        # confirmed tracks get first pick, then nearest first
        pairs.sort(key=lambda x: (not s.tracks[x[1]].confirmed, x[0]))
        used_t, used_c = set(), set()
        for dd, ti, ci in pairs:
            if ti in used_t or ci in used_c: continue
            used_t.add(ti); used_c.add(ci)
            t, q = s.tracks[ti], np.array(cands[ci][:2])
            step = q - t.p
            t.v = step if t.v is None else 0.5 * t.v + 0.5 * step
            t.p, t.hits, t.miss = q, t.hits + 1, 0
            t.speeds.append(float(np.linalg.norm(step))); t.steps.append(step)
            if not t.confirmed and t.hits >= P['confirm_hits'] and np.median(t.speeds) >= P['confirm_speed']:
                t.confirmed = True
            t.still = t.still + 1 if np.linalg.norm(t.v) < P['still_speed'] else 0
        keep = []
        for ti, t in enumerate(s.tracks):
            if ti not in used_t:
                t.miss += 1
                if t.v is not None: t.p = t.p + t.v
            off = not (-20 < t.p[0] < P['W'] + 20 and -20 < t.p[1] < P['H'] + 20)
            limit = P['coast'] if t.confirmed else 1
            if t.miss > limit or off or t.still > P['still_frames']: continue
            if not t.confirmed and t.hits >= 8: continue      # had its chance
            keep.append(t)
        s.tracks = keep
        for ci, c in enumerate(cands):
            if ci in used_c or near[ci]: continue
            s.tracks.append(Track(c, i))
        def good(t):
            sp = np.array(t.speeds[-P['window']:])
            if t.hits < P['confirm_hits'] or np.median(sp) < P['confirm_speed']: return False
            if np.linalg.norm(t.p - t.origin) < P['min_travel']: return False
            steps = np.array(t.steps[-P['window']:])
            jerk = np.median(np.linalg.norm(np.diff(steps, axis=0), axis=1))
            return jerk <= P['max_jerk'] * np.median(sp)
        conf = [t for t in s.tracks if good(t)]
        if not conf: return ('search', None)
        best = max(conf, key=lambda t: t.hits)
        return ('hit' if best.miss == 0 else 'coast', best.p)

def run(d, P=P, start=0, stop=None):
    f, m = load(d); stop = stop or len(f)
    B = gray2(f[start]).copy(); mgr, out = Manager(P), []
    for i in range(start, stop):
        g = gray2(f[i])
        cands, near = blobs2(g, B, P)
        st, p = mgr.step(cands, i, near)
        B = (1 - P['alpha']) * B + P['alpha'] * g
        out.append(dict(i=i, state=st, p=None if p is None else [float(p[0]), float(p[1])], n=len(cands)))
    return out

if __name__ == '__main__':
    d = sys.argv[1]
    out = run(d)
    json.dump(out, open(f'{d}/proto2.json', 'w'))
    segs, cur = [], None
    for o in out:
        if o['state'] in ('hit', 'coast'):
            if cur is None: cur = [o['i'], o['i']]
            cur[1] = o['i']
        elif cur: segs.append(cur); cur = None
    if cur: segs.append(cur)
    print(len(segs), 'segments:', [(a, b, b - a + 1) for a, b in segs])

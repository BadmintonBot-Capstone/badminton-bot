# physics gated 3D tracker, run offline on the cands.py blobs of both cameras and scored against the left labels
#   track3d.py <clip> [<clip> ...] [--model tri] [--acq 15] [--gate-thresh 8] [--tag name]
#
# every frame, left blobs are paired with right blobs on the same rectified rows (gap at most 4 px, areas
# within 3x, disparity inside the room) and each pair is a 3D point. two points in consecutive frames
# moving between 0.5 and 80 m/s start a track: an EKF with the drag plus gravity model (gravity direction
# and alpha from flightfit.py for the clip's range), measuring left x, left y and disparity in px. after
# that a track only takes the point whose distance from the prediction is within the gate (chi square
# --chi2 on 3 values), at the lower --gate-thresh, so the shuttle can fade without being dropped. a track
# is reported once it holds --confirm points and moves faster than --v-confirm m/s, and from then on in
# every frame it gets a point; coasted frames are never reported. reports have the frame they became
# known, so a track's first points are counted twice: live (known at that frame) and with the
# confirmation delay. writes <clip>/sweep/track3d_<tag>.csv and prints scores with the sweep's rule.
import csv, pathlib, argparse, numpy as np, yaml
from multiprocessing import Pool
import flightfit as ff
import sweep

REC = pathlib.Path(__file__).resolve().parent.parent.parent / 'recordings' / 'E04_settings_sweep'


def frame_pairs(t, r, cal, amin, amax, ymin, smin=0, smax=np.inf):
    # every stereo pair per frame: rectified left x, y, disparity, and the left blob's raw box and index.
    # smin and smax bound the blob's real size in cm^2, from its area and the pair's depth
    t = t[(t[:, 2] >= amin) & (t[:, 2] <= amax) & (t[:, 10] >= ymin)]
    r = r[r[:, 2] >= 2]
    out = {}
    if not len(t) or not len(r): return out
    def span(b, i):
        top = ff.rect(np.column_stack([b[:, 3], b[:, 6]]), cal, i)[:, 1]
        bot = ff.rect(np.column_stack([b[:, 3], b[:, 6] + b[:, 8]]), cal, i)[:, 1]
        return ff.rect(b[:, 3:5], cal, i), top, bot
    cl, tl, bl = span(t, 1)
    cr, tr, br = span(r, 2)
    fl, fr = t[:, 0].astype(int), r[:, 0].astype(int)
    order = np.argsort(fr, kind='stable')
    fr, cr, tr, br, ra = fr[order], cr[order], tr[order], br[order], r[order, 2]
    n = max(fl.max(), fr.max()) + 1
    cnt = np.bincount(fr, minlength=n)
    start = np.cumsum(cnt) - cnt
    rep = cnt[fl]
    ii = np.repeat(np.arange(len(t)), rep)
    jj = start[fl][ii] + np.arange(rep.sum()) - np.repeat(np.cumsum(rep) - rep, rep)
    gap = np.maximum(tl[ii], tr[jj]) - np.minimum(bl[ii], br[jj])
    d = cl[ii, 0] - cr[jj, 0]
    ar = t[ii, 2] / np.maximum(ra[jj], 1)
    ok = (gap <= 4) & (d >= 60) & (d <= 1300) & (ar <= 3) & (ar >= 1 / 3)
    ii, d = ii[ok], d[ok]
    Z = cal['f'] * cal['B'] / (d - cal['dcx'])
    cm2 = t[ii, 2] * 4 * (Z / cal['f']) ** 2 * 1e4
    keep = (cm2 >= smin) & (cm2 <= smax)
    ii, d = ii[keep], d[keep]
    z = np.column_stack([cl[ii, 0], cl[ii, 1], d])
    box = t[ii][:, 5:9]
    feat = np.column_stack([t[ii][:, 10], t[ii][:, 2]])  # yellowness, area in half resolution px
    for k in np.unique(fl[ii]):
        m = fl[ii] == k
        out[k] = dict(z=z[m], box=box[m], blob=ii[m], feat=feat[m])
    return out


class Ekf:
    def __init__(s, cal, g, alpha, sig, sig_a):
        s.c, s.g, s.alpha, s.R, s.sig_a = cal, g, alpha, np.diag(np.square(sig)), sig_a

    def step(s, S, dt):
        # one rk4 step of the drag plus gravity model for rows of (p, v)
        def d(S):
            v = S[:, 3:]
            return np.hstack([v, s.g - s.alpha * np.linalg.norm(v, axis=1, keepdims=True) * v])
        k1 = d(S); k2 = d(S + 0.5 * dt * k1); k3 = d(S + 0.5 * dt * k2); k4 = d(S + dt * k3)
        return S + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    def predict(s, x, P, dt):
        eps = 1e-6 * np.maximum(1, np.abs(x))
        X = s.step(np.vstack([x, x + np.diag(eps)]), dt)
        F = ((X[1:] - X[0]) / eps[:, None]).T
        q = s.sig_a ** 2
        I = np.eye(3)
        Q = q * np.block([[dt ** 3 / 3 * I, dt ** 2 / 2 * I], [dt ** 2 / 2 * I, dt * I]])
        return X[0], F @ P @ F.T + Q

    def h(s, x):
        c = s.c
        X, Y, Z = x[:3]
        z = np.array([c['f'] * X / Z + c['cx'], c['f'] * Y / Z + c['cy'], c['f'] * c['B'] / Z + c['dcx']])
        H = np.zeros((3, 6))
        H[0, 0], H[0, 2] = c['f'] / Z, -c['f'] * X / Z ** 2
        H[1, 1], H[1, 2] = c['f'] / Z, -c['f'] * Y / Z ** 2
        H[2, 2] = -c['f'] * c['B'] / Z ** 2
        return z, H

    def point(s, z):
        # 3D point and its covariance from one measurement
        c = s.c
        p = ff.tri(z[None], c)[0]
        Z = p[2]
        dZ = -Z ** 2 / (c['f'] * c['B'])
        J = np.array([[Z / c['f'], 0, (z[0] - c['cx']) / c['f'] * dZ],
                      [0, Z / c['f'], (z[1] - c['cy']) / c['f'] * dZ],
                      [0, 0, dZ]])
        return p, J @ s.R @ J.T


def reportable(h, a):
    # a confirmed track's point is sent only while it moves like a flying shuttle: fast enough, and its mean
    # prediction error over the last frames within --nis-max. h is box, yellowness, area, depth, speed, error
    return h[7] >= a.v_report and (np.isnan(h[8]) or h[8] <= a.nis_max)


def run_clip(job):
    clip, a = job
    clip = pathlib.Path(clip)
    cal = ff.load_cal()
    meta = yaml.safe_load(open(clip / 'meta.yaml'))
    fit = np.load(REC / 'flightfit.npy', allow_pickle=True).item()[meta['notes']['range_ft']]
    gx, gz, alpha = fit['theta']
    ekf = Ekf(cal, ff.gdir(gx, gz), alpha, a.sig, a.sig_a)
    ts = np.genfromtxt(clip / 'frames.csv', delimiter=',', names=True)['left_ts_ns'] / 1e9
    L, Rt = np.load(clip / 'sweep' / 'left.npz')[a.model], np.load(clip / 'sweep' / 'right.npz')[a.model]
    acq = frame_pairs(L[L[:, 1] == a.acq], Rt[Rt[:, 1] == a.acq], cal, a.amin, a.amax, a.yel_acq, a.smin, a.smax)
    gate = frame_pairs(L[L[:, 1] == a.gate_thresh], Rt[Rt[:, 1] == a.gate_thresh], cal, a.amin, a.amax, a.yel_gate,
                       a.smin, a.smax)

    tracks, singles, reports, next_id = [], [], [], 0
    for f in range(len(ts)):
        dt = ts[f] - ts[f - 1] if f else 0.01
        for tr in tracks:
            tr['x'], tr['P'] = ekf.predict(tr['x'], tr['P'], dt)
        # gate: each track takes its closest point in the gate, older confirmed tracks first, one track per blob
        g = gate.get(f)
        used, used_boxes = set(), []
        cand = []
        if g is not None:
            for k, tr in enumerate(tracks):
                zp, H = ekf.h(tr['x'])
                S = H @ tr['P'] @ H.T + ekf.R
                Si = np.linalg.inv(S)
                y = g['z'] - zp
                nis = np.einsum('ij,jk,ik->i', y, Si, y)
                for j in np.flatnonzero(nis < a.chi2):
                    cand.append((not tr['confirmed'], tr['born'], nis[j], k, j, H, S))
        cand.sort(key=lambda q: q[:3])
        took = set()
        for _, _, nis, k, j, H, S in cand:
            if k in took or g['blob'][j] in used: continue
            tr = tracks[k]
            tr['nis'].append(nis)
            K = tr['P'] @ H.T @ np.linalg.inv(S)
            zp, _ = ekf.h(tr['x'])
            tr['x'] = tr['x'] + K @ (g['z'][j] - zp)
            IKH = np.eye(6) - K @ H
            tr['P'] = IKH @ tr['P'] @ IKH.T + K @ ekf.R @ K.T
            tr['hits'].append((f, np.concatenate([g['box'][j], g['feat'][j], [tr['x'][2], np.linalg.norm(tr['x'][3:]),
                                                                         np.mean(tr['nis'][-a.nis_window:])]])))
            tr['miss'] = 0
            took.add(k); used.add(g['blob'][j]); used_boxes.append(g['box'][j])
            if not tr['confirmed'] and len(tr['hits']) >= a.confirm and np.linalg.norm(tr['x'][3:]) >= a.v_confirm:
                tr['confirmed'] = True
                for hf, hb in tr['hits']:
                    if reportable(hb, a): reports.append((hf, f, tr['id'], *hb))
            elif tr['confirmed'] and reportable(tr['hits'][-1][1], a):
                reports.append((f, f, tr['id'], *tr['hits'][-1][1]))
        for k, tr in enumerate(tracks):
            if k not in took: tr['miss'] += 1
        tracks = [tr for tr in tracks if tr['miss'] <= (a.miss_conf if tr['confirmed'] else a.miss_tent)]
        # new tracks from acquisition points left over, paired with last frame's leftovers
        A = acq.get(f)
        new_singles = []
        if A is not None:
            for j in range(len(A['z'])):
                # an acquisition blob inside a blob a track just took is that track's, not a new start
                bx, by = A['box'][j][0] + A['box'][j][2] / 2, A['box'][j][1] + A['box'][j][3] / 2
                if any(b[0] <= bx <= b[0] + b[2] and b[1] <= by <= b[1] + b[3] for b in used_boxes): continue
                p, Pp = ekf.point(A['z'][j])
                if not (0.3 < p[2] < 40): continue
                for sf, sp, sPp, sbox in singles:
                    if sf != f - 1: continue
                    v = (p - sp) / dt
                    if not (a.v_min <= np.linalg.norm(v) <= a.v_max): continue
                    P0 = np.block([[Pp, Pp / dt], [Pp / dt, (Pp + sPp) / dt ** 2]])
                    sp_ = np.linalg.norm(v)
                    tracks.append(dict(x=np.concatenate([p, v]), P=P0, hits=[(sf, np.concatenate([sbox, [sp[2], sp_, np.nan]])), (f, np.concatenate([A['box'][j], A['feat'][j], [p[2], sp_, np.nan]]))], nis=[],
                                       miss=0, confirmed=False, born=f, id=next_id))
                    next_id += 1
                new_singles.append((f, p, Pp, np.concatenate([A['box'][j], A['feat'][j]])[:6]))
        singles = new_singles
        if len(tracks) > a.max_tracks:
            tracks.sort(key=lambda tr: (not tr['confirmed'], -len(tr['hits'])))
            tracks = tracks[:a.max_tracks]

    out = clip / 'sweep' / f'track3d_{a.tag}.csv'
    with open(out, 'w') as fh:
        w = csv.writer(fh)
        w.writerow(['frame', 'known_at', 'track', 'x0', 'y0', 'w', 'h', 'yellow', 'area', 'z', 'speed', 'nis'])
        for r in sorted(reports, key=lambda r: r[:3]): w.writerow([r[0], r[1], r[2]] + [f'{v:.0f}' for v in r[3:7]] + [f'{v:.3f}' for v in r[7:]])
    return str(out)


def score(clip, reports, info, margin=10, live_only=False):
    # per flight: flying found, flying frames; plus false reports and counted frames for the whole clip
    st, xy, ep, n = info['st'], info['xy'], info['ep'], info['n']
    hit = np.zeros(n, bool)
    false = np.zeros(info['nflights'])
    for f, known, x0, y0, w, h in reports:
        if live_only and known != f: continue
        s = st[f]
        if s < 0 or s == 3: continue
        lx, ly = xy[f]
        on = s in (1, 2) and x0 - margin <= lx <= x0 + w + margin and y0 - margin <= ly <= y0 + h + margin
        if on and s == 1: hit[f] = True
        if not on: false[ep[f]] += 1
    fly = st == 1
    E = info['nflights']
    found = np.bincount(ep[fly], hit[fly], E)
    total = np.bincount(ep[fly], None, E)
    # delay: frames from each flight's first flying frame to its first found frame
    delays = []
    for k in range(E):
        fr = np.flatnonzero(fly & (ep == k))
        hk = fr[hit[fr]]
        if len(hk): delays.append(hk[0] - fr[0])
    cnt = (st >= 0) & (st <= 2)
    counted = np.bincount(ep[cnt], None, E)
    return found, total, false, counted, delays


def current_reports(clip, info):
    # the current tracker's reports as boxes: its blob found by centroid, or a 60 px box around the report
    t = np.load(clip / 'sweep' / 'left.npz')['ra']
    t = t[(t[:, 1] == 12) & (t[:, 2] >= 4) & (t[:, 2] <= 900) & (t[:, 10] >= 0.3)]
    out = []
    for r in csv.DictReader(open(clip / 'track.csv')):
        if r['state'] != 'hit': continue
        f, x, y = int(r['frame']), float(r['x']), float(r['y'])
        b = t[t[:, 0] == f]
        k = np.argmin(np.hypot(b[:, 3] - x, b[:, 4] - y)) if len(b) else None
        if k is not None and np.hypot(b[k, 3] - x, b[k, 4] - y) < 6:
            out.append((f, f, b[k, 5], b[k, 6], b[k, 7], b[k, 8]))
        else:
            out.append((f, f, x - 30, y - 30, 60, 60))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clips', nargs='+')
    ap.add_argument('--model', default='tri')
    ap.add_argument('--acq', type=int, default=15)
    ap.add_argument('--gate-thresh', type=int, default=8)
    ap.add_argument('--amin', type=int, default=4)
    ap.add_argument('--amax', type=int, default=3000)
    ap.add_argument('--yel-acq', type=float, default=0.2)
    ap.add_argument('--yel-gate', type=float, default=0.1)
    ap.add_argument('--smin', type=float, default=0, help='smallest blob in cm^2 at its depth')
    ap.add_argument('--smax', type=float, default=np.inf, help='largest blob in cm^2 at its depth')
    ap.add_argument('--sig', type=float, nargs=3, default=[5, 5, 3], help='measurement noise px: x, y, disparity')
    ap.add_argument('--sig-a', type=float, default=10, help='model error, m/s^2')
    ap.add_argument('--chi2', type=float, default=16)
    ap.add_argument('--nis-window', type=int, default=10, help='frames the mean prediction error is taken over')
    ap.add_argument('--nis-max', type=float, default=np.inf, help='largest mean prediction error a reported point may have')
    ap.add_argument('--v-report', type=float, default=0, help='slowest speed in m/s a reported point may have')
    ap.add_argument('--confirm', type=int, default=5)
    ap.add_argument('--v-confirm', type=float, default=1.5)
    ap.add_argument('--v-min', type=float, default=0.5)
    ap.add_argument('--v-max', type=float, default=80)
    ap.add_argument('--miss-conf', type=int, default=4)
    ap.add_argument('--miss-tent', type=int, default=1)
    ap.add_argument('--max-tracks', type=int, default=200)
    ap.add_argument('--tag', default='default')
    a = ap.parse_args()
    clips = [pathlib.Path(c) for c in a.clips]
    with Pool() as pool:
        outs = pool.map(run_clip, [(str(c), a) for c in clips])

    rng = np.random.default_rng(1)
    rows = {}
    for c, o in zip(clips, outs):
        info = sweep.clip_info(c, 'left')
        rep = [(int(r['frame']), int(r['known_at']), float(r['x0']), float(r['y0']), float(r['w']), float(r['h']))
               for r in csv.DictReader(open(o))]
        key = (info['setting'], info['range'])
        for name, reps, live in (('3D live', rep, True), ('3D after confirming', rep, False),
                                 ('current tracker', current_reports(c, info), False)):
            fd, tot, fa, cnt, dl = score(c, reps, info, live_only=live)
            r = rows.setdefault((name,) + key, dict(found=[], total=[], false=0, frames=0, delays=[], fps=info['fps']))
            r['found'] += list(fd); r['total'] += list(tot); r['false'] += fa.sum(); r['frames'] += cnt.sum(); r['delays'] += dl

    print(f'tag {a.tag}: ' + ' '.join(f'{k}={v}' for k, v in vars(a).items() if k not in ('clips', 'tag')))
    print('  tracker              range setting             flying found         false/min  flights found  delay median, p90')
    for name in ('current tracker', '3D live', '3D after confirming'):
        for rg in ('11', '35'):
            for s in sweep.SETTINGS:
                r = rows.get((name, s, rg))
                if not r: continue
                fd, tot = np.array(r['found']), np.array(r['total'])
                lo, hi = sweep.ci(fd, tot, rng)
                fpm = r['false'] / (r['frames'] / r['fps'] / 60)
                dl = r['delays']
                print(f'  {name:20s} {rg:>3s} ft {s:18s} {100 * fd.sum() / tot.sum():5.1f}% [{100 * lo:2.0f}-{100 * hi:3.0f}]  '
                      f'{fpm:8.1f}  {len(dl):5d}/{len(tot):<5d}  '
                      + (f'{np.median(dl):.0f}, {np.percentile(dl, 90):.0f}' if dl else '-'))


if __name__ == '__main__':
    main()

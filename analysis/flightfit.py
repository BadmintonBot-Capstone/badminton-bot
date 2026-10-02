# fits drag plus gravity to the labelled flights of stereo clips: alpha, the direction of gravity in the
# camera frame, and how far each setting's 3D points scatter about their fitted flight
#   flightfit.py <clip> [<clip> ...] [--out flights.npy] [--skip 3] [--min 10]
#
# needs cands.py on both cameras. in each frame labelled flying, the left blob holding the label (tri model,
# threshold 10) is paired with the right blob on the same rectified rows (gap at most 4 px, areas within 3x,
# disparity inside the room) and triangulated. the model is the EKF's, a = g - alpha |v| v, with g of
# 9.81 m/s^2 in a direction fitted per batch of clips (one batch per range, since the rig moved between
# them), alpha shared by the batch, and a start position and velocity per flight. residuals are in left
# image px and disparity px, so near and far points weigh by their real pixel noise, with a soft_l1 loss
# (3 px) so a wrong pairing cannot drag a flight. the first --skip frames of each flight are left out while
# the shuttle turns over. given alpha and gravity every flight fits on its own, so the flights run in
# parallel and an outer search moves only the three shared values. limits on every value are wide of
# anything physical, and a value that ends on one is reported.
import csv, pathlib, argparse, numpy as np, yaml, cv2
from multiprocessing import Pool
from scipy.optimize import least_squares, minimize

CAL = pathlib.Path(__file__).resolve().parent.parent / 'config' / 'calibration.yaml'
G = 9.81
H = 1 / 200  # integration step, s (rk4 plus linear interpolation, under 0.3 mm off)
LO = np.array([-30, -30, 0.3, -80, -80, -80, 0.01])  # start position m, start velocity m/s, alpha per m
HI = np.array([30, 30, 40, 80, 80, 80, 2])
SETTINGS = ['baseline', 'short_exposure', 'shorter_exposure', 'higher_frame_rate']


def load_cal():
    fs = cv2.FileStorage(str(CAL), cv2.FILE_STORAGE_READ)
    c = {k: fs.getNode(k).mat() for k in ('M1', 'D1', 'R1', 'P1', 'M2', 'D2', 'R2', 'P2')}
    c['f'], c['cx'], c['cy'] = c['P1'][0, 0], c['P1'][0, 2], c['P1'][1, 2]
    c['B'] = -c['P2'][0, 3] / c['f']  # m, the calibration is in metres
    c['dcx'] = c['P1'][0, 2] - c['P2'][0, 2]
    return c


def rect(xy, c, i):
    return cv2.undistortPoints(xy.reshape(-1, 1, 2).astype(np.float64), c[f'M{i}'], c[f'D{i}'],
                               R=c[f'R{i}'], P=c[f'P{i}']).reshape(-1, 2)


def tri(obs, c):
    Z = c['f'] * c['B'] / (obs[..., 2] - c['dcx'])
    return np.stack([(obs[..., 0] - c['cx']) * Z / c['f'], (obs[..., 1] - c['cy']) * Z / c['f'], Z], -1)


def project(X, c):
    return np.stack([c['f'] * X[..., 0] / X[..., 2] + c['cx'], c['f'] * X[..., 1] / X[..., 2] + c['cy'],
                     c['f'] * c['B'] / X[..., 2] + c['dcx']], -1)


def points(clip, c, skip):
    # (flight number in the clip, time s, x, y, disparity) for every paired flying frame
    meta = yaml.safe_load(open(clip / 'meta.yaml'))
    ts = np.genfromtxt(clip / 'frames.csv', delimiter=',', names=True)['left_ts_ns'] / 1e9
    lab = list(csv.DictReader(open(clip / 'labels' / 'left.csv')))
    L, Rt = np.load(clip / 'sweep' / 'left.npz')['tri'], np.load(clip / 'sweep' / 'right.npz')['tri']
    L, Rt = L[L[:, 1] == 10], Rt[Rt[:, 1] == 10]
    span = lambda b, i: (rect(b[:, 3:5], c, i), rect(np.column_stack([b[:, 3], b[:, 6]]), c, i)[:, 1],
                         rect(np.column_stack([b[:, 3], b[:, 6] + b[:, 8]]), c, i)[:, 1])
    (cl, tl, bl), (cr, tr, br) = span(L, 1), span(Rt, 2)
    byl, byr = {}, {}
    for k, f in enumerate(L[:, 0].astype(int)): byl.setdefault(f, []).append(k)
    for k, f in enumerate(Rt[:, 0].astype(int)): byr.setdefault(f, []).append(k)
    out, flight, n_in, prev_flying = [], -1, 0, False
    for r in lab:
        f, flying = int(r['frame']), r['state'] == 'flying'
        if flying and (r['new_flight'] == '1' or not prev_flying):
            flight += 1
            n_in = 0
        prev_flying = flying
        if not flying: continue
        n_in += 1
        if n_in <= skip: continue
        lx, ly = float(r['x']), float(r['y'])
        ks = [k for k in byl.get(f, []) if L[k, 5] - 10 <= lx <= L[k, 5] + L[k, 7] + 10
              and L[k, 6] - 10 <= ly <= L[k, 6] + L[k, 8] + 10]
        if not ks: continue
        k = min(ks, key=lambda k: np.hypot(L[k, 3] - lx, L[k, 4] - ly))
        js = [j for j in byr.get(f, [])
              if max(tl[k], tr[j]) - min(bl[k], br[j]) <= 4 and 60 <= cl[k, 0] - cr[j, 0] <= 1300
              and 1 / 3 <= L[k, 2] / max(Rt[j, 2], 1) <= 3]
        if not js: continue
        j = min(js, key=lambda j: abs(cr[j, 1] - cl[k, 1]))
        out.append((flight, ts[f], cl[k, 0], cl[k, 1], cl[k, 0] - cr[j, 0]))
    return np.array(out), meta


def simulate(p0, v0, g, alpha, steps):
    # rk4 of a = g - alpha |v| v for many starts at once (alpha a number or one per start); positions per step
    acc = lambda v: g - alpha * np.linalg.norm(v, axis=-1, keepdims=True) * v
    p, v = p0.copy(), v0.copy()
    out = np.empty((steps + 1,) + p.shape)
    out[0] = p
    for s in range(steps):
        k1v = acc(v); k1p = v
        k2p = v + 0.5 * H * k1v; k2v = acc(k2p)
        k3p = v + 0.5 * H * k2v; k3v = acc(k3p)
        k4p = v + H * k3v; k4v = acc(k4p)
        p = p + H / 6 * (k1p + 2 * k2p + 2 * k3p + k4p)
        v = v + H / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
        out[s + 1] = p
    return out


def gdir(gx, gz):
    u = np.array([gx, 1.0, gz])
    return G * u / np.linalg.norm(u)


def fit_flight(job):
    # one flight's start (and alpha when asked) for fixed gravity; the nudges for the jacobian run as one batch
    t, obs, c, g, alpha, q0 = job
    free = len(q0)  # 6, or 7 with alpha
    steps = int(np.ceil(t.max() / H)) + 1
    i0 = np.floor(t / H).astype(int)
    i1 = np.minimum(i0 + 1, steps)
    w = (t / H - i0)[:, None, None]

    def resid(Q):
        a = Q[:, 6:7] if free == 7 else alpha
        tr = simulate(Q[:, :3], Q[:, 3:6], g, a, steps)
        X = tr[i0] + w * (tr[i1] - tr[i0])
        return project(X, c) - obs[:, None, :]  # (points, starts, 3)

    def jac(q):
        h = 1e-6 * np.maximum(1, np.abs(q))
        r = resid(np.vstack([q, q + np.diag(h)]))
        return ((r[:, 1:] - r[:, :1]) / h[None, :, None]).transpose(0, 2, 1).reshape(-1, free)

    lo, hi = LO[:free], HI[:free]
    q0 = np.clip(q0, lo + 1e-6, hi - 1e-6)
    r = least_squares(lambda q: resid(q[None])[:, 0].ravel(), q0, jac=jac, bounds=(lo, hi), loss='soft_l1',
                      f_scale=3, x_scale='jac', ftol=1e-10, xtol=1e-10, gtol=1e-10)
    return r.x, r.fun, r.cost


def first_guess(t, obs, c):
    X = tri(obs, c)
    n = min(len(t), 8)
    A = np.column_stack([np.ones(n), t[:n], t[:n] ** 2])
    coef = np.linalg.lstsq(A, X[:n], rcond=None)[0]
    return np.concatenate([coef[0], coef[1]])


def fit_batch(flights, c, pool):
    # flights: list of (t, obs). returns (gx, gz, alpha), per flight starts, per flight residuals
    starts = [first_guess(t, o, c) for t, o in flights]

    def solve(th):
        g = gdir(th[0], th[1])
        out = pool.map(fit_flight, [(t, o, c, g, th[2], q) for (t, o), q in zip(flights, starts)])
        for k, o in enumerate(out): starts[k] = o[0]
        return out

    def cost(th):
        if not (-1 < th[0] < 1 and -1 < th[1] < 1 and 0.01 < th[2] < 2): return 1e12
        return sum(o[2] for o in solve(th))

    solve(np.array([0.0, 0.0, 0.2]))
    r = minimize(cost, [0.0, 0.0, 0.2], method='Nelder-Mead',
                 options=dict(xatol=1e-4, fatol=1e-3, initial_simplex=[[0, 0, 0.2], [0.05, 0, 0.2], [0, 0.05, 0.2], [0, 0, 0.25]]))
    out = solve(r.x)
    return r.x, [o[0] for o in out], [o[1].reshape(-1, 3) for o in out], r.nfev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clips', nargs='+')
    ap.add_argument('--skip', type=int, default=3)
    ap.add_argument('--min', type=int, default=10, help='fewest paired points for a flight to be fitted')
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    c = load_cal()
    batches = {}
    for clip in map(pathlib.Path, a.clips):
        p, meta = points(clip, c, a.skip)
        if not len(p): continue
        b = batches.setdefault(meta['notes']['range_ft'], [])
        for k in np.unique(p[:, 0]).astype(int):
            q = p[p[:, 0] == k]
            if len(q) >= a.min:
                b.append(dict(clip=clip.name, flight=k, setting=meta['notes']['setting'],
                              t=q[:, 1] - q[0, 1], obs=q[:, 2:]))

    save = {}
    with Pool() as pool:
        for rng, fl in sorted(batches.items(), key=lambda kv: float(kv[0])):
            th, qs, res, nfev = fit_batch([(f['t'], f['obs']) for f in fl], c, pool)
            gx, gz, alpha = th
            g = gdir(gx, gz) / G
            print(f'\n{rng} ft: {len(fl)} flights, {sum(len(f["t"]) for f in fl)} points, {nfev} outer steps')
            print(f'  alpha {alpha:.3f} per m (terminal speed {np.sqrt(G / alpha):.2f} m/s)')
            print(f'  gravity in the camera frame ({g[0]:+.3f}, {g[1]:+.3f}, {g[2]:+.3f}), '
                  f'{np.degrees(np.arccos(g[1])):.1f} degrees off the image down axis')
            on_limit = sum(np.any((q - LO[:6] < 0.01 * (HI[:6] - LO[:6])) | (HI[:6] - q < 0.01 * (HI[:6] - LO[:6]))) for q in qs)
            if on_limit: print(f'  WARNING: {on_limit} flights end with a start value on a limit')

            print('  setting            flights points  px rms (x, y, disparity)  over 15 px  3D mm median (across, depth)')
            for s in SETTINGS:
                ids = [i for i, f in enumerate(fl) if f['setting'] == s]
                if not ids: continue
                q = np.concatenate([res[i] for i in ids])
                core = np.all(np.abs(q) < 15, 1)
                rms = np.sqrt(np.mean(q[core] ** 2, 0))
                # 3D scatter: measured point against the fitted flight at the same moment
                err = np.concatenate([(tri(fl[i]['obs'], c) - tri(fl[i]['obs'] + res[i], c)) for i in ids]) * 1000
                err = err[core]
                print(f'  {s:18s} {len(ids):7d} {len(q):6d}  {rms[0]:5.2f} {rms[1]:5.2f} {rms[2]:5.2f}  '
                      f'{100 * (1 - core.mean()):9.1f}%   {np.median(np.hypot(err[:, 0], err[:, 1])):6.1f} '
                      f'{np.median(np.abs(err[:, 2])):6.1f}')

            # alpha per flight, refitting it with the flight's own start while gravity stays fixed
            jobs = [(f['t'], f['obs'], c, gdir(gx, gz), alpha, np.append(q, alpha)) for f, q in zip(fl, qs)]
            alphas = np.array([o[0][6] for o in pool.map(fit_flight, jobs)])
            lim = np.sum((alphas < 0.02) | (alphas > 1.98))
            print(f'  alpha per flight: median {np.median(alphas):.3f}, middle half {np.percentile(alphas, 25):.3f} '
                  f'to {np.percentile(alphas, 75):.3f}' + (f'  (WARNING: {lim} on a limit)' if lim else ''))
            save[rng] = dict(theta=th, flights=[{k: v for k, v in f.items()} for f in fl], starts=qs, res=res,
                             alphas=alphas)
    if a.out:
        np.save(a.out, save, allow_pickle=True)


if __name__ == '__main__':
    main()

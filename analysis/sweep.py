# detector sweep over labelled stereo clips: for each camera setting, which background model, threshold,
# blob size and colour rule finds the most flying frames for a given number of false candidates
#   sweep.py <clip> [<clip> ...] [--camera left] [--margin 10] [--budget 0.3]
#
# needs cands.py run on every clip first. a frame is found when a kept blob's box, grown by --margin
# full resolution px, holds the label; every other kept blob on a labelled frame is a false candidate
# (unsure frames are left out). tuning and testing use 5-fold cross validation by flight: each flight
# goes into a fold with the frames before it (hand, wind up, pick up), each setting is tuned on four
# folds (most flying frames found with at most --budget false candidates per frame) and tested on the
# fifth, so every flight is scored once by a detector that never saw it. the ranges in brackets are 95%
# intervals from resampling flights.
import csv, pathlib, argparse, itertools, numpy as np, yaml, cv2

SETTINGS = ['baseline', 'short_exposure', 'shorter_exposure', 'higher_frame_rate']
MIN_AREA = [2, 4, 8, 16]
MAX_AREA = [300, 900, 3000, 10 ** 9]
MIN_YELLOW = [-1, 0.1, 0.2, 0.3, 0.4]
STEREO = [0, 2, 4, 8]  # rectified row gap in px allowed between the left and right boxes, 0 = left camera only
DISPARITY = (60, 1300)  # px, about 0.9 to 20 m
SIZE_RATIO = 3  # left and right blob areas within this factor
BUDGETS = [0.03, 0.1, 0.3, 1, 3]
CURRENT = ('ra', 12, 4, 900, 0.3, 0)  # the tracker's detector today
FOLDS = 5
CALIBRATION = pathlib.Path(__file__).resolve().parent.parent / 'config' / 'calibration.yaml'


def clip_info(c, cam):
    meta = yaml.safe_load(open(c / 'meta.yaml'))
    lab = list(csv.DictReader(open(c / 'labels' / f'{cam}.csv')))
    n = meta['frames']
    st = np.full(n, -1, np.int8)
    xy = np.full((n, 2), np.nan)
    new = np.zeros(n, bool)
    code = {'none': 0, 'flying': 1, 'still': 2, 'unsure': 3}
    for r in lab:
        f = int(r['frame'])
        st[f] = code[r['state']]
        if r['x']: xy[f] = float(r['x']), float(r['y'])
        new[f] = r['new_flight'] == '1'
    # flights as score_frames.py has them: a new_flight label starts one, it runs while frames stay flying
    flights, cur = [], None
    for f in range(n):
        if st[f] == 1 and (new[f] or cur is None):
            if cur: flights.append(cur)
            cur = [f, f]
        elif st[f] == 1 and cur: cur[1] = f
        elif cur: flights.append(cur); cur = None
    if cur: flights.append(cur)
    # each frame belongs to the first flight ending at or after it, the tail to the last flight
    ends = np.array([e for _, e in flights])
    ep = np.minimum(np.searchsorted(ends, np.arange(n)), len(flights) - 1)
    rng = np.random.default_rng(0)
    fold = rng.permutation(len(flights)) % FOLDS
    return dict(name=c.name, setting=meta['notes']['setting'], range=meta['notes']['range_ft'],
                fps=meta['fps'], n=n, st=st, xy=xy, ep=ep, fold=fold, nflights=len(flights))


def rectify(xy, cal, i):
    return cv2.undistortPoints(xy.reshape(-1, 1, 2).astype(np.float64), cal[f'M{i}'], cal[f'D{i}'],
                               R=cal[f'R{i}'], P=cal[f'P{i}']).reshape(-1, 2)


def stereo_ok(t, r, cal):
    # per left blob and row tolerance: is there a right blob whose rectified rows overlap it (gap at most the
    # tolerance) at a disparity inside the room. boxes rather than centres, since a big smeared shuttle splits
    # differently in the two cameras. right blobs keep every colour, and their size only has to match the left's
    out = {tol: np.zeros(len(t), bool) for tol in STEREO if tol}
    if not len(t) or not len(r): return out
    def span(b, i):
        top = rectify(np.column_stack([b[:, 3], b[:, 6]]), cal, i)
        bot = rectify(np.column_stack([b[:, 3], b[:, 6] + b[:, 8]]), cal, i)
        return rectify(b[:, 3:5], cal, i)[:, 0], top[:, 1], bot[:, 1]
    xl, tl, bl = span(t, 1)
    xr, tr, br = span(r, 2)
    fl, fr = t[:, 0].astype(int), r[:, 0].astype(int)
    order = np.argsort(fr, kind='stable')
    fr, xr, tr, br = fr[order], xr[order], tr[order], br[order]
    n = max(fl.max(), fr.max()) + 1
    cnt = np.bincount(fr, minlength=n)
    start = np.cumsum(cnt) - cnt
    rep = cnt[fl]
    ii = np.repeat(np.arange(len(t)), rep)
    jj = start[fl][ii] + np.arange(rep.sum()) - np.repeat(np.cumsum(rep) - rep, rep)
    gap = np.maximum(tl[ii], tr[jj]) - np.minimum(bl[ii], br[jj])
    d = xl[ii] - xr[jj]
    ar = t[ii, 2] / np.maximum(r[order][jj, 2], 1)
    geom = (d >= DISPARITY[0]) & (d <= DISPARITY[1]) & (ar <= SIZE_RATIO) & (ar >= 1 / SIZE_RATIO)
    for tol in out: out[tol][ii[geom & (gap <= tol)]] = True
    return out


def score_clip(c, info, cam, margin, configs, cal):
    # per config, per flight: flying found, flying, still found, still, false candidates, counted frames
    z = np.load(c / 'sweep' / f'{cam}.npz')
    zr = np.load(c / 'sweep' / 'right.npz') if cam == 'left' and (c / 'sweep' / 'right.npz').exists() else None
    n, st, xy, ep, E = info['n'], info['st'], info['xy'], info['ep'], info['nflights']
    out = np.zeros((len(configs), E, 6))
    fly, still, counted = st == 1, st == 2, (st >= 0) & (st <= 2)
    base = np.stack([np.bincount(ep[fly], minlength=E), np.bincount(ep[still], minlength=E),
                     np.bincount(ep[counted], minlength=E)], 1)
    out[:, :, 1], out[:, :, 3], out[:, :, 5] = base[:, 0], base[:, 1], base[:, 2]
    index = {}
    for k, cfg in enumerate(configs): index.setdefault(cfg[:2], []).append(k)
    for (m, T), ks in index.items():
        t = z[m]
        t = t[t[:, 1] == T]
        f = t[:, 0].astype(int)
        s = st[f]
        lx, ly = xy[f, 0], xy[f, 1]
        x0, y0, w, h = t[:, 5], t[:, 6], t[:, 7], t[:, 8]
        inside = ((s == 1) | (s == 2)) & (lx >= x0 - margin) & (lx <= x0 + w + margin) \
                 & (ly >= y0 - margin) & (ly <= y0 + h + margin)
        cnt = (s >= 0) & (s <= 2)
        st_ok = stereo_ok(t, zr[m][zr[m][:, 1] == T], cal) if zr is not None else {}
        for k in ks:
            _, _, amin, amax, ymin, tol = configs[k]
            if tol and tol not in st_ok: continue
            keep = (t[:, 2] >= amin) & (t[:, 2] <= amax) & (t[:, 10] >= ymin)
            if tol: keep &= st_ok[tol]
            hit = np.zeros(n, bool)
            hit[f[keep & inside]] = True
            false = np.bincount(f[keep & ~inside & cnt], minlength=n)
            out[k, :, 0] = np.bincount(ep[fly], hit[fly], E)
            out[k, :, 2] = np.bincount(ep[still], hit[still], E)
            out[k, :, 4] = np.bincount(ep, false * counted, E)
    return out


def ci(found, total, rng, reps=2000):
    # 95% interval of found/total over flights resampled with replacement
    i = rng.integers(0, len(found), (reps, len(found)))
    r = found[i].sum(1) / np.maximum(total[i].sum(1), 1)
    return np.percentile(r, [2.5, 97.5])


def name(cfg):
    m, T, a, x, y, tol = cfg
    return f'{m} T{T} area {a}-{"any" if x > 1e8 else x}' + ('' if y < 0 else f' yellow {y}') \
        + (f' stereo {tol}' if tol else '')


def cross_validate(R, eps, fold, allowed, budget):
    # test counts per flight, each from the config tuned on the other folds, and the configs picked
    test = np.zeros((len(eps), 6))
    picks = []
    for k in range(FOLDS):
        tr, te = eps[fold[eps] != k], eps[fold[eps] == k]
        if not len(te): continue
        s = R[np.ix_(allowed, tr)].sum(1)
        rec, fpf = s[:, 0] / s[:, 1], s[:, 4] / s[:, 5]
        ok = fpf <= budget
        if not ok.any(): ok = fpf == fpf.min()
        best = allowed[np.argmax(np.where(ok, rec, -1))]
        picks.append(best)
        test[np.searchsorted(eps, te)] = R[best, te]
    return test, picks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clips', nargs='+')
    ap.add_argument('--camera', default='left')
    ap.add_argument('--margin', type=float, default=10)
    ap.add_argument('--budget', type=float, default=0.3)
    a = ap.parse_args()
    clips = [pathlib.Path(c) for c in a.clips if (pathlib.Path(c) / 'sweep' / f'{a.camera}.npz').exists()]
    z = np.load(clips[0] / 'sweep' / f'{a.camera}.npz')
    models, thresh = list(z['models']), [int(t) for t in z['thresh']]
    configs = list(itertools.product(models, thresh, MIN_AREA, MAX_AREA, MIN_YELLOW, STEREO))
    fs = cv2.FileStorage(str(CALIBRATION), cv2.FILE_STORAGE_READ)
    cal = {k: fs.getNode(k).mat() for k in ('M1', 'D1', 'R1', 'P1', 'M2', 'D2', 'R2', 'P2')}
    infos = [clip_info(c, a.camera) for c in clips]
    R = np.concatenate([score_clip(c, i, a.camera, a.margin, configs, cal) for c, i in zip(clips, infos)], 1)
    setting = np.concatenate([[i['setting']] * i['nflights'] for i in infos])
    rng_ft = np.concatenate([[i['range']] * i['nflights'] for i in infos])
    fold = np.concatenate([i['fold'] for i in infos])
    fps = np.concatenate([[i['fps']] * i['nflights'] for i in infos])
    ranges = sorted(set(rng_ft), key=float)
    rng = np.random.default_rng(1)
    allc = np.arange(len(configs))
    cur = configs.index(CURRENT)

    print(f'{len(clips)} clips, {len(setting)} flights, {len(configs)} detector variants, '
          f'margin {a.margin:.0f} px, budget {a.budget} false candidates per frame\n')

    print('current detector (no tuning): flying found, still found, false candidates per frame')
    for s in SETTINGS:
        for r in ranges:
            e = np.flatnonzero((setting == s) & (rng_ft == r))
            if not len(e): continue
            t = R[cur, e].sum(0)
            lo, hi = ci(R[cur, e, 0], R[cur, e, 1], rng)
            print(f'  {s:18s} {r:>3s} ft  {100 * t[0] / t[1]:5.1f}% [{100 * lo:.0f}-{100 * hi:.0f}]  '
                  f'{100 * t[2] / max(t[3], 1):5.1f}%  {t[4] / t[5]:.2f}')

    for label, allowed in (('left camera only', np.array([k for k, c in enumerate(configs) if not c[5]])),
                           ('left camera with the right camera check allowed', allc)):
        print(f'\ntuned per setting, tested on held out flights, {label} (budget {a.budget}):')
        print('  setting            range  flying found        still found  false/frame  flights')
        for s in SETTINGS:
            eps = np.flatnonzero(setting == s)
            if not len(eps): continue
            test, picks = cross_validate(R, eps, fold, allowed, a.budget)
            for r in ranges:
                sel = rng_ft[eps] == r
                if not sel.any(): continue
                t = test[sel].sum(0)
                lo, hi = ci(test[sel, 0], test[sel, 1], rng)
                print(f'  {s:18s} {r:>3s} ft  {100 * t[0] / t[1]:5.1f}% [{100 * lo:2.0f}-{100 * hi:3.0f}]  '
                      f'{100 * t[2] / max(t[3], 1):9.1f}%  {t[4] / t[5]:11.2f}  {sel.sum():7d}')
            print(f'  {"":18s} picked: ' + '; '.join(sorted(set(name(configs[p]) for p in picks))))

    print('\nflying found on held out flights by background model, right camera check and budget '
          '(false candidates per frame), all settings and ranges:')
    print('  model stereo' + ''.join(f'{b:>8}' for b in BUDGETS))
    for m, tol in itertools.product(models, STEREO):
        allowed = np.array([k for k, c in enumerate(configs) if c[0] == m and c[5] == tol])
        row = []
        for b in BUDGETS:
            t = np.zeros(6)
            for s in SETTINGS:
                eps = np.flatnonzero(setting == s)
                if len(eps): t += cross_validate(R, eps, fold, allowed, b)[0].sum(0)
            row.append(f'{100 * t[0] / t[1]:7.1f}%')
        print(f'  {m:5s} {tol if tol else "off":>5}  ' + ''.join(row))

    np.savez_compressed(clips[0].parent / f'sweep_{a.camera}.npz', R=R, configs=np.array(configs, object),
                        setting=setting, range=rng_ft, fold=fold, fps=fps, clips=[i['name'] for i in infos],
                        nflights=[i['nflights'] for i in infos])


if __name__ == '__main__':
    main()

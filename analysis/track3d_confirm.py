# E10: how few points a track can confirm on, and whether dropping quiet confirmed tracks helps, scored by how
# early and how close the tracker predicts where each flight ends
#   track3d_confirm.py <clip> [<clip> ...] [--budget 15] [--workers 14]
#
# truth: each labelled flight's drag plus gravity fit (flightfit.npy) at its last fitted point, where the flight
# ended in view; the floor height is not known, so this stands in for the landing point. the last raw stereo point
# of the flight is kept as a second truth (median 2 to 3 cm from the fitted one). at each time in TAUS
# after the hit (the flight's first flying frame) the tracker's latest state sent with a report, on a track
# that reported this flight's shuttle, is pushed forward with the same physics to the end time and compared
# with the fitted end point. a flight with no such state yet counts as missed; flights that end before a time
# are left out of it. every other setting is picked by 5-fold cross validation by flight: most flights within
# PICK_CM at PICK_TAU with at most --budget false reports per minute, picked on four folds, scored on the fifth.
import argparse, itertools, pathlib, numpy as np
from multiprocessing import Pool
import track3d, track3d_sweep, sweep
import flightfit as ff

REC = track3d.REC
TAUS = [0.05, 0.1, 0.2, 0.3, 0.5]  # s after the hit
CMS = [5, 10, 20]
PICK_TAU, PICK_CM = 0.3, 20
GRID = dict(yel=[(0.3, 0.25), (0.35, 0.3)], sig=[(5, 5, 3), (3, 3, 2)], nis_max=[np.inf, 2.5, 1.5],
            v_report=[0, 1.5, 2.5], confirm=[2, 3, 4, 5, 6, 8], drop_after=[0, 3, 6, 12])
BASE = dict(model='tri', acq=15, gate_thresh=8, amin=4, amax=3000, smin=4, smax=80, sig_a=2, chi2=16, v_confirm=2.5,
            v_min=0.5, v_max=80, miss_conf=4, miss_tent=1, max_tracks=200, nis_window=10, tag='')
E05 = ((0.35, 0.3), (5, 5, 3), 2.5, 2.5, 8, 0)  # the setting E05 picked most often at 15 false reports per minute


def args(cfg):
    a = argparse.Namespace(**BASE)
    (a.yel_acq, a.yel_gate), a.sig, a.nis_max, a.v_report, a.confirm, a.drop_after = cfg
    return a


def propagate(p, v, g, alpha, T):
    # position after T s of drag plus gravity from p, v
    n = int(np.ceil(T / ff.H)) + 1
    tr = ff.simulate(p[None], v[None], g, alpha, n)[:, 0]
    i = min(int(T / ff.H), n - 1)
    return tr[i] + (T / ff.H - i) * (tr[i + 1] - tr[i])


def truth(clip, info, ts, cal, fit):
    # per fitted flight in this clip: hit time, end time, fitted end point, last raw stereo point, physics
    rng = info['range']
    d = fit[rng]
    g, alpha = ff.gdir(*d['theta'][:2]), d['theta'][2]
    pts, _ = ff.points(clip, cal, 3)
    fly = info['st'] == 1
    out = {}
    for fl, q in zip(d['flights'], d['starts']):
        if fl['clip'] != clip.name: continue
        k = fl['flight']
        fr = np.flatnonzero(fly & (info['ep'] == k))
        t0 = pts[pts[:, 0] == k][0, 1]
        assert len(fr) and ts[fr[0]] <= t0 <= ts[fr[-1]], (clip.name, k)
        T = fl['t'].max()
        out[k] = (ts[fr[0]], t0 + T, propagate(q[:3], q[3:6], g, alpha, T),
                  ff.tri(fl['obs'][np.argmax(fl['t'])][None], cal)[0], g, alpha)
    return out


def run(job):
    # one setting on one clip: per flight E05 scores (found live, flying, found late, false, minutes) and the
    # end point error in m at each time against both truths (nan: no truth or flight already over, inf: no
    # prediction yet)
    clip, cfg, info, ts, tru = job
    reps, _, states = track3d.track_clip(pathlib.Path(clip), args(cfg))
    per = track3d_sweep.per_flight(clip, info, [r[:2] + r[3:7] for r in reps])
    st, xy, ep = info['st'], info['xy'], info['ep']
    on = {}
    for f, _, tid, x0, y0, w, h, *_ in reps:
        if st[f] == 1 and x0 - 10 <= xy[f, 0] <= x0 + w + 10 and y0 - 10 <= xy[f, 1] <= y0 + h + 10:
            on.setdefault(ep[f], set()).add(tid)
    S = np.array(states) if states else np.zeros((0, 8))
    err = np.full((2, info['nflights'], len(TAUS)), np.nan)
    for k, (t_hit, t_end, p_fit, p_raw, g, alpha) in tru.items():
        rows = S[np.isin(S[:, 1], list(on.get(k, ())))]
        tk = ts[rows[:, 0].astype(int)]
        for j, tau in enumerate(TAUS):
            t = t_hit + tau
            if t > t_end: continue
            m = np.flatnonzero((tk >= t_hit) & (tk <= t))
            if not len(m):
                err[:, k, j] = np.inf
                continue
            i = m[np.argmax(tk[m])]
            p = propagate(rows[i, 2:5], rows[i, 5:8], g, alpha, t_end - tk[i])
            err[:, k, j] = np.linalg.norm(p - p_fit), np.linalg.norm(p - p_raw)
    return per, err


def within(E, cm, j):
    # flights within cm at TAUS[j] over flights with truth that were still in the air
    e = E[:, j]
    ok = ~np.isnan(e)
    return (e[ok] <= cm / 100).sum(), ok.sum()


def cross_validate(R, E, Eraw, fold, allowed, budget):
    # per flight test rows from the setting picked on the other four folds, and the picks
    j = TAUS.index(PICK_TAU)
    tR, tE, tEr, picks = np.zeros(R.shape[1:]), np.full(E.shape[1:], np.nan), np.full(E.shape[1:], np.nan), []
    for k in range(sweep.FOLDS):
        tr, te = fold != k, fold == k
        s = R[allowed][:, tr].sum(1)
        e = E[allowed][:, tr, j]
        hit = (e <= PICK_CM / 100).sum(1) / np.maximum((~np.isnan(e)).sum(1), 1)
        ok = s[:, 3] / s[:, 4] <= budget
        best = allowed[np.argmax(np.where(ok, hit, -1))] if ok.any() else allowed[np.argmin(s[:, 3] / s[:, 4])]
        picks.append(best)
        tR[te], tE[te], tEr[te] = R[best, te], E[best, te], Eraw[best, te]
    return tR, tE, tEr, picks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clips', nargs='+')
    ap.add_argument('--budget', type=float, default=15)
    ap.add_argument('--workers', type=int, default=14)
    a = ap.parse_args()
    clips = [pathlib.Path(c) for c in a.clips]
    cal = ff.load_cal()
    fit = np.load(REC / 'flightfit.npy', allow_pickle=True).item()
    infos = [sweep.clip_info(c, 'left') for c in clips]
    tss = [np.genfromtxt(c / 'frames.csv', delimiter=',', names=True)['left_ts_ns'] / 1e9 for c in clips]
    trus = [truth(c, i, t, cal, fit) for c, i, t in zip(clips, infos, tss)]
    configs = list(itertools.product(*GRID.values()))
    jobs = [(str(c), cfg, i, t, u) for cfg in configs for c, i, t, u in zip(clips, infos, tss, trus)]
    with Pool(a.workers) as pool:
        outs = pool.map(run, jobs, chunksize=4)
    n = len(clips)
    R = np.stack([np.concatenate([outs[k * n + m][0] for m in range(n)]) for k in range(len(configs))])
    E2 = np.stack([np.concatenate([outs[k * n + m][1] for m in range(n)], 1) for k in range(len(configs))])
    E, Eraw = E2[:, 0], E2[:, 1]
    rng_ft = np.concatenate([[i['range']] * i['nflights'] for i in infos])
    fold = np.concatenate([i['fold'] for i in infos])
    np.savez_compressed(REC / 'track3d_confirm.npz', R=R, E=E, Eraw=Eraw, configs=np.array(configs, object), range=rng_ft,
                        fold=fold, taus=TAUS, clips=[i['name'] for i in infos], nflights=[i['nflights'] for i in infos])

    def line(name, tR, tE, tEr, e):
        t = tR[e].sum(0)
        cells = []
        for cm in CMS:
            cells += [f'{100 * h / max(m, 1):5.1f}' for h, m in (within(tE[e], cm, j) for j in range(len(TAUS)))]
        j = TAUS.index(PICK_TAU)
        raw = [f'{100 * h / max(m, 1):5.1f}' for h, m in (within(tEr[e], cm, j) for cm in CMS)]
        med = tE[e, j]
        med = med[~np.isnan(med)]
        print(f'  {name:24s} {100 * t[0] / t[1]:5.1f}% {t[3] / t[4]:6.1f}  ' + ' '.join(cells)
              + f'   {100 * np.median(med):6.1f}   raw ' + ' '.join(raw))

    taus = ' '.join(f'{t:5g}' for t in TAUS)
    print(f'{n} clips, {R.shape[1]} flights ({int((~np.isnan(E[0, :, 0])).sum())} with a fitted end point), '
          f'{len(configs)} settings, picked for most flights within {PICK_CM} cm at {PICK_TAU} s with at most '
          f'{a.budget:g} false reports per minute')
    print(f'  columns: found live, false/min, then % of flights within 5, 10, 20 cm at {taus} s, then median error '
          f'at {PICK_TAU} s in cm, then % within 5, 10, 20 cm at {PICK_TAU} s against the last raw stereo point')
    for rg in ('all', '11', '35'):
        e = np.ones(len(rng_ft), bool) if rg == 'all' else rng_ft == rg
        print(f'\n{rg} ft' if rg != 'all' else '\nall ranges')
        k = configs.index(E05)
        line('E05 setting (fixed)', R[k], E[k], Eraw[k], e)
        for c in GRID['confirm']:
            tR, tE, tEr, _ = cross_validate(R, E, Eraw, fold, np.array([i for i, x in enumerate(configs) if x[4] == c]), a.budget)
            line(f'confirm {c}', tR, tE, tEr, e)
        for d in GRID['drop_after']:
            tR, tE, tEr, _ = cross_validate(R, E, Eraw, fold, np.array([i for i, x in enumerate(configs) if x[5] == d]), a.budget)
            line(f'drop after {d}' if d else 'never drop', tR, tE, tEr, e)
        tR, tE, tEr, picks = cross_validate(R, E, Eraw, fold, np.arange(len(configs)), a.budget)
        line('best of all', tR, tE, tEr, e)
    print('\npicks of "best of all": ' + '; '.join(f'{picks.count(p)}x {configs[p]}' for p in sorted(set(picks))))


if __name__ == '__main__':
    main()

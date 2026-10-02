# runs track3d.py over a grid of settings on every clip and picks one by 5-fold cross validation by flight
#   track3d_sweep.py <clip> [<clip> ...] [--budget 5 15 30]
#
# one tracker setting has to serve every camera setting and range, so the pick is made on four folds of
# all flights (most flying frames found live with at most --budget false reports per minute) and scored
# on the fifth. found counts live reports only; false counts every report off the shuttle, the ones sent
# late after a track confirmed included. the current tracker is scored with the same rule.
import argparse, itertools, pathlib, csv, numpy as np
from multiprocessing import Pool
import track3d, sweep

# model tri, sizes 4 to 80 cm^2 and confirming at 2.5 m/s were the unanimous picks of a first, wider grid
# (background models tri and prev, yellowness 0.2 to 0.35, no size limit, confirming at 1.5 m/s)
GRID = dict(yel=[(0.3, 0.25), (0.35, 0.3)], confirm=[5, 8], sig=[(5, 5, 3), (3, 3, 2)],
            nis_max=[np.inf, 2.5, 1.5], v_report=[0, 1.5, 2.5])


def config_args(base, cfg):
    a = argparse.Namespace(**vars(base))
    (a.yel_acq, a.yel_gate), a.confirm, a.sig, a.nis_max, a.v_report = cfg
    a.tag = 'grid2_' + '_'.join(str(v) for v in [cfg[0][0], cfg[1], cfg[2][0], cfg[3], cfg[4]])
    return a


def per_flight(clip, info, reps):
    # columns: found live, flying, found after confirming, false (all), minutes counted
    lf, tot, _, _, _ = track3d.score(clip, reps, info, live_only=True)
    bf, _, fa, cnt, _ = track3d.score(clip, reps, info)
    return np.column_stack([lf, tot, bf, fa, cnt / info['fps'] / 60])


def delays(clip, info, reps):
    return track3d.score(clip, reps, info, live_only=True)[4]


def cross_validate(R, fold, budget):
    # per flight test rows, each from the grid setting picked on the other four folds, and the picks
    test, picks = np.zeros(R.shape[1:]), []
    for k in range(sweep.FOLDS):
        tr, te = fold != k, fold == k
        s = R[:, tr].sum(1)
        ok = s[:, 3] / s[:, 4] <= budget
        best = np.argmax(np.where(ok, s[:, 0] / s[:, 1], -1)) if ok.any() else np.argmin(s[:, 3] / s[:, 4])
        picks.append(best)
        test[te] = R[best, te]
    return test, picks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clips', nargs='+')
    ap.add_argument('--budget', type=float, nargs='+', default=[5, 15, 30])
    a = ap.parse_args()
    clips = [pathlib.Path(c) for c in a.clips]
    base = argparse.Namespace(model='tri', acq=15, gate_thresh=8, amin=4, amax=3000, yel_acq=0.35, yel_gate=0.3,
                              smin=4, smax=80, sig=[5, 5, 3], sig_a=2, chi2=16, confirm=8, v_confirm=2.5,
                              v_min=0.5, v_max=80, miss_conf=4, miss_tent=1, max_tracks=200, nis_window=10,
                              nis_max=np.inf, v_report=0, tag='')
    configs = list(itertools.product(*GRID.values()))
    jobs = [(str(c), config_args(base, cfg)) for cfg in configs for c in clips]
    with Pool() as pool:
        outs = pool.map(track3d.run_clip, jobs, chunksize=1)
    infos = [sweep.clip_info(c, 'left') for c in clips]
    read = lambda o: [(int(r['frame']), int(r['known_at']), float(r['x0']), float(r['y0']), float(r['w']), float(r['h']))
                      for r in csv.DictReader(open(o))]
    R = np.stack([np.concatenate([per_flight(c, i, read(outs[k * len(clips) + n]))
                                  for n, (c, i) in enumerate(zip(clips, infos))]) for k in range(len(configs))])
    cur = np.concatenate([per_flight(c, i, track3d.current_reports(c, i)) for c, i in zip(clips, infos)])
    setting = np.concatenate([[i['setting']] * i['nflights'] for i in infos])
    rng_ft = np.concatenate([[i['range']] * i['nflights'] for i in infos])
    fold = np.concatenate([i['fold'] for i in infos])
    rng = np.random.default_rng(1)
    np.savez_compressed(clips[0].parent / 'track3d_grid.npz', R=R, cur=cur, configs=np.array(configs, object),
                        setting=setting, range=rng_ft, fold=fold)

    def table(name, T):
        for r in ['11', '35']:
            for s in sweep.SETTINGS:
                e = (setting == s) & (rng_ft == r)
                t = T[e].sum(0)
                lo, hi = sweep.ci(T[e, 0], T[e, 1], rng)
                print(f'  {name:26s} {r:>3s} ft {s:18s} {100 * t[0] / t[1]:5.1f}% [{100 * lo:2.0f}-{100 * hi:3.0f}]  '
                      f'{100 * t[2] / t[1]:5.1f}%  {t[3] / t[4]:7.1f}  {np.sum(T[e, 0] > 0):3d}/{e.sum():<3d}')
        t = T.sum(0)
        print(f'  {name:26s} all                    {100 * t[0] / t[1]:5.1f}%          {100 * t[2] / t[1]:5.1f}%  {t[3] / t[4]:7.1f}  '
              f'{np.sum(T[:, 0] > 0):3d}/{len(T)}')

    print(f'{len(clips)} clips, {len(setting)} flights, {len(configs)} tracker settings')
    print('  tracker                    range setting            found live          late    false/min  flights')
    table('current tracker', cur)
    for b in a.budget:
        test, picks = cross_validate(R, fold, b)
        table(f'3D, picked at {b:g} false/min', test)
        for p in sorted(set(picks)):
            print(f'      picked {picks.count(p)}x: yellow {configs[p][0]}, confirm after {configs[p][1]} points, noise px {configs[p][2]}, '
                  f'error at most {configs[p][3]}, speed at least {configs[p][4]} m/s')


if __name__ == '__main__':
    main()

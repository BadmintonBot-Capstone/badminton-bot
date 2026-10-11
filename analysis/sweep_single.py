# detector sweep over labelled single camera clips: which background model, threshold, blob size and
# colour rule finds the most flying frames for a given number of false candidates
#   sweep_single.py <clip> [<clip> ...] [--camera primary|left] [--margin 10] [--budget 0.3] [--start 500]
#                   [--yellow plain|weighted]
#
# sweep.py without the camera settings, ranges and right camera check, for one camera, reading what
# cands_mog2.py writes. each model is tried at the thresholds it was run with (grey levels for ra, prev
# and tri, MOG2's varThreshold for the mog2 models).
# a frame is found when a kept blob's box, grown by --margin full resolution px, holds the label; every
# other kept blob on a labelled frame is a false candidate (unsure frames are left out, and so are the
# first --start frames, while the background models settle). tuning and testing use 5-fold cross
# validation by flight: each flight goes into a fold with the frames before it (hand, wind up, pick up),
# each choice is tuned on four folds (most flying frames found with at most --budget false candidates
# per frame) and tested on the fifth, so every flight is scored once by a detector that never saw it.
# the ranges in brackets are 95% intervals from resampling flights.
# --yellow picks the yellowness the colour rule uses: plain (the blob's mean colour, the same for every
# model) or weighted (cands.py's, weighted by change). the current detector is always scored with weighted,
# since that is what the tracker measures. results are given with the colour rule allowed and with it off.
# every variant's counts are saved, whatever the budget, in <clip>/sweep/sweep_single_<camera>_s<start>_<yellow>.npz
import csv, pathlib, argparse, itertools, numpy as np
from rec_any import default_camera

MIN_AREA = [2, 4, 8, 16, 32, 64]
MAX_AREA = [50, 100, 150, 300, 900, 3000, 10 ** 9]
MIN_YELLOW = [-1, 0, 0.1, 0.2, 0.3, 0.35, 0.4, 0.45, 0.5, 0.6]
BUDGETS = [0.03, 0.1, 0.3, 1, 3]
YCOL = {'weighted': 10, 'plain': 11}
CURRENT = ('ra', 12, 4, 900, 0.3, 'weighted')  # the tracker's detector today
FOLDS = 5


def load_cands(c, cam):
    # every model's blob table from the group files cands_mog2.py wrote, plus the run's settings
    p = c / 'sweep' / f'{cam}_mog2.npz'
    if not p.exists(): return None
    meta = np.load(p)
    tables, models = {}, []
    for g in meta['groups']:
        z = np.load(c / 'sweep' / f'{cam}_mog2_{g}.npz')
        for m in z['models']:
            models.append(str(m))
            tables[str(m)] = z[m]
    bench = dict(zip(meta['bench_inputs'].tolist(), meta['bench_ms'].tolist()))
    return dict(models=models, tables=tables, frames=int(meta['frames']), fps=float(meta['fps']), bench=bench)


def model_ms(m, bench):
    # MOG2 apply time for a model's input, from the bench run (the change models are far cheaper, not timed)
    if not m.startswith('mog2'): return None
    return bench.get(m.split('_')[1])


def clip_info(c, cam, n, start):
    lab = list(csv.DictReader(open(c / 'labels' / f'{cam}.csv')))
    st = np.full(n, -1, np.int8)
    xy = np.full((n, 2), np.nan)
    new = np.zeros(n, bool)
    code = {'none': 0, 'flying': 1, 'still': 2, 'unsure': 3}
    for r in lab:
        f = int(r['frame'])
        if f >= n: continue
        st[f] = code[r['state']]
        if r['x']: xy[f] = float(r['x']), float(r['y'])
        new[f] = r['new_flight'] == '1'
    st[:start] = -1
    # flights as score_frames.py has them: a new_flight label starts one, it runs while frames stay flying
    flights, cur = [], None
    for f in range(n):
        if st[f] == 1 and (new[f] or cur is None):
            if cur: flights.append(cur)
            cur = [f, f]
        elif st[f] == 1 and cur: cur[1] = f
        elif cur: flights.append(cur); cur = None
    if cur: flights.append(cur)
    if not flights:
        raise SystemExit(f'{c.name}: no flying frames labelled after frame {start}')
    # each frame belongs to the first flight ending at or after it, the tail to the last flight
    ends = np.array([e for _, e in flights])
    ep = np.minimum(np.searchsorted(ends, np.arange(n)), len(flights) - 1)
    rng = np.random.default_rng(0)
    fold = rng.permutation(len(flights)) % FOLDS
    return dict(name=c.name, n=n, st=st, xy=xy, ep=ep, fold=fold, nflights=len(flights))


def score_clip(tables, info, margin, configs):
    # per config, per flight: flying found, flying, still found, still, false candidates, counted frames
    n, st, xy, ep, E = info['n'], info['st'], info['xy'], info['ep'], info['nflights']
    out = np.zeros((len(configs), E, 6))
    fly, still, counted = st == 1, st == 2, (st >= 0) & (st <= 2)
    base = np.stack([np.bincount(ep[fly], minlength=E), np.bincount(ep[still], minlength=E),
                     np.bincount(ep[counted], minlength=E)], 1)
    out[:, :, 1], out[:, :, 3], out[:, :, 5] = base[:, 0], base[:, 1], base[:, 2]
    index = {}
    for k, cfg in enumerate(configs): index.setdefault(cfg[:2], []).append(k)
    for (m, T), ks in index.items():
        t = tables[m]
        t = t[(t[:, 1] == T) & (t[:, 0] < n)]
        f = t[:, 0].astype(int)
        s = st[f]
        lx, ly = xy[f, 0], xy[f, 1]
        x0, y0, w, h = t[:, 5], t[:, 6], t[:, 7], t[:, 8]
        inside = ((s == 1) | (s == 2)) & (lx >= x0 - margin) & (lx <= x0 + w + margin) \
                 & (ly >= y0 - margin) & (ly <= y0 + h + margin)
        cnt = (s >= 0) & (s <= 2)
        for k in ks:
            _, _, amin, amax, ymin, ycol = configs[k]
            keep = (t[:, 2] >= amin) & (t[:, 2] <= amax) & (t[:, YCOL[ycol]] >= ymin)
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
    m, T, a, x, y, yc = cfg
    return f'{m} T{T:g} area {a}-{"any" if x > 1e8 else x}' + ('' if y < 0 else f' yellow {y}' + ('w' if yc == 'weighted' else ''))


def cross_validate(R, eps, fold, allowed, budget):
    # test counts per flight, each from the config tuned on the other folds, and the configs picked
    test = np.zeros((len(eps), 6))
    picks = []
    for k in range(FOLDS):
        tr, te = eps[fold[eps] != k], eps[fold[eps] == k]
        if not len(te): continue
        s = R[np.ix_(allowed, tr)].sum(1)
        rec, fpf = s[:, 0] / np.maximum(s[:, 1], 1), s[:, 4] / np.maximum(s[:, 5], 1)
        ok = fpf <= budget
        if not ok.any(): ok = fpf == fpf.min()
        best = allowed[np.argmax(np.where(ok, rec, -1))]
        picks.append(best)
        test[np.searchsorted(eps, te)] = R[best, te]
    return test, picks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clips', nargs='+')
    ap.add_argument('--camera')
    ap.add_argument('--margin', type=float, default=10)
    ap.add_argument('--budget', type=float, default=0.3)
    ap.add_argument('--start', type=int, default=500, help='frames left out at the start while the models settle')
    ap.add_argument('--yellow', choices=list(YCOL), default='plain')
    a = ap.parse_args()
    clips, cands, infos = [], [], []
    for c in map(pathlib.Path, a.clips):
        cam = a.camera or default_camera(c)
        z = load_cands(c, cam)
        if z is None:
            print(f'skipping {c.name}: run cands_mog2.py on it first')
            continue
        clips.append(c); cands.append(z); infos.append(clip_info(c, cam, z['frames'], a.start))
    if not clips:
        raise SystemExit('nothing to score')
    models, bench = cands[0]['models'], cands[0]['bench']
    thresh = {m: sorted(set(np.unique(cands[0]['tables'][m][:, 1]).tolist())) for m in models}
    configs = [(m, T, amin, amax, ymin, a.yellow) for m in models
               for T, amin, amax, ymin in itertools.product(thresh[m], MIN_AREA, MAX_AREA, MIN_YELLOW)]
    # the current detector is in the grid when --yellow weighted; otherwise it is scored on its own
    extra = CURRENT not in configs
    if extra: configs.append(CURRENT)
    R = np.concatenate([score_clip(z['tables'], i, a.margin, configs) for z, i in zip(cands, infos)], 1)
    fold = np.concatenate([i['fold'] for i in infos])
    eps = np.arange(R.shape[1])
    rng = np.random.default_rng(1)

    print(f'{len(clips)} clip(s), {len(eps)} flights, {len(configs)} detector variants, margin {a.margin:.0f} px, '
          f'first {a.start} frames left out, {a.yellow} yellowness, budget {a.budget} false candidates per frame\n')

    cur = configs.index(CURRENT)
    t = R[cur].sum(0)
    lo, hi = ci(R[cur, :, 0], R[cur, :, 1], rng)
    print(f'current detector ({name(CURRENT)}), no tuning:')
    print(f'  flying found {100 * t[0] / t[1]:.1f}% [{100 * lo:.0f}-{100 * hi:.0f}], '
          f'still found {100 * t[2] / max(t[3], 1):.1f}%, {t[4] / t[5]:.2f} false candidates per frame\n')

    colour = {'colour rule allowed': lambda c: True, 'colour rule off': lambda c: c[4] < 0}
    for label, use in colour.items():
        print(f'each model tuned on four folds and tested on the fifth, {label} (budget {a.budget}):')
        print('  model                 MOG2 ms   flying found        still found  false/frame  picked')
        for m in models:
            allowed = np.array([k for k, c in enumerate(configs) if c[0] == m and use(c) and not (extra and k == cur)])
            test, picks = cross_validate(R, eps, fold, allowed, a.budget)
            t = test.sum(0)
            lo, hi = ci(test[:, 0], test[:, 1], rng)
            ms = model_ms(m, bench)
            print(f'  {m:20s} {f"{ms:6.1f}" if ms else "      "}  {100 * t[0] / t[1]:5.1f}% [{100 * lo:2.0f}-{100 * hi:3.0f}]  '
                  f'{100 * t[2] / max(t[3], 1):9.1f}%  {t[4] / t[5]:11.2f}  '
                  + '; '.join(sorted(set(name(configs[p]) for p in picks))))
        print()

    for label, use in colour.items():
        print(f'flying found on held out flights by model and budget (false candidates per frame), {label}:')
        print('  model               ' + ''.join(f'{b:>8}' for b in BUDGETS))
        for m in models:
            allowed = np.array([k for k, c in enumerate(configs) if c[0] == m and use(c) and not (extra and k == cur)])
            row = []
            for b in BUDGETS:
                t = cross_validate(R, eps, fold, allowed, b)[0].sum(0)
                row.append(f'{100 * t[0] / t[1]:7.1f}%')
            print(f'  {m:20s}' + ''.join(row))
        print()

    cam = a.camera or default_camera(clips[0])
    out = clips[0] / 'sweep' / f'sweep_single_{cam}_s{a.start}_{a.yellow}.npz'
    np.savez_compressed(out, R=R, configs=np.array(configs, object), fold=fold, start=a.start, yellow=a.yellow,
                        clips=[i['name'] for i in infos], nflights=[i['nflights'] for i in infos])
    print(f'saved {out}')


if __name__ == '__main__':
    main()

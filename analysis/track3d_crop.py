# E09: the 3D tracker searching only a crop around each confirmed track, every crop shape and size on every clip
#   track3d_crop.py <clip> [<clip> ...] [--workers 6] [--only none circle ...]
#
# the tracker setting is the one track3d_sweep.py picked most often at 15 false reports per minute (E05, 3 of
# 5 folds). while a track is confirmed only its crops are searched, in both cameras, and no new track starts;
# full frames come back when the last confirmed track dies. scored with the E05 rules, plus how often the
# labelled shuttle sat outside every crop, and the share of both full frames searched, each crop grown by PAD
# for the blur and the blob's own size. writes track3d_crop.npz next to the clips and prints a table.
import argparse, csv, pathlib, numpy as np
from multiprocessing import Pool
import track3d, track3d_sweep, sweep
import flightfit as ff

W, H = 1920, 1200
PAD = 20  # px around each crop for the blur and the blob's own extent
VARIANTS = [('none', 0), ('rule', 0)] + [('circle', r) for r in (40, 80, 160, 320)] + [('box', r) for r in (40, 80, 160, 320)] \
    + [('ellipse', k) for k in (3, 4, 5, 6)] + [('metres', m) for m in (0.1, 0.2, 0.4)]
BASE = dict(model='tri', acq=15, gate_thresh=8, amin=4, amax=3000, yel_acq=0.35, yel_gate=0.3, smin=4, smax=80,
            sig=[5, 5, 3], sig_a=2, chi2=16, confirm=8, v_confirm=2.5, v_min=0.5, v_max=80, miss_conf=4,
            miss_tent=1, max_tracks=200, nis_window=10, nis_max=2.5, v_report=2.5)


def name(shape, size):
    return shape if shape in ('none', 'rule') else f'{shape} {size:g}'


def tag(shape, size):
    return 'e09_' + name(shape, size).replace(' ', '_')


def area(m00, m01, m11, box):
    # pixels one crop covers once grown by PAD, at most a whole frame
    if box:
        a = (2 / np.sqrt(m00) + 2 * PAD) * (2 / np.sqrt(m11) + 2 * PAD)
    else:
        e = np.linalg.eigvalsh(np.array([[m00, m01], [m01, m11]]))
        a = np.pi * (1 / np.sqrt(e[0]) + PAD) * (1 / np.sqrt(e[1]) + PAD)
    return min(a, W * H)


def crop_stats(clip, info, shape, size, cal):
    # per frame the share of both full frames searched, and per flight the flying frames searched in a crop
    # and those whose label was outside every crop (left camera, where the labels are)
    n, st, xy, ep, E = info['n'], info['st'], info['xy'], info['ep'], info['nflights']
    share = np.ones(n)
    in_mode, outside = np.zeros(E), np.zeros(E)
    if shape == 'none': return share, in_mode, outside
    rows = list(csv.DictReader(open(clip / 'sweep' / f'track3d_{tag(shape, size)}_crops.csv')))
    by_frame = {}
    for r in rows: by_frame.setdefault(int(r['frame']), []).append({k: float(v) for k, v in r.items()})
    ok = ~np.isnan(xy[:, 0])
    lab = np.full((n, 2), np.nan)
    lab[ok] = ff.rect(xy[ok], cal, 1)
    for f, cs in by_frame.items():
        if f >= n: continue
        px = sum(area(c['ml00'], c['ml01'], c['ml11'], shape == 'box') + area(c['mr00'], c['mr01'], c['mr11'], shape == 'box')
                 for c in cs)
        share[f] = min(1, px / (2 * W * H))
        if st[f] != 1 or np.isnan(lab[f, 0]): continue
        in_mode[ep[f]] += 1
        inside = False
        for c in cs:
            M = np.array([[c['ml00'], c['ml01']], [c['ml01'], c['ml11']]])
            p = lab[f][None]
            inside |= bool(track3d.in_crop(p, np.array([c['lx'], c['ly']]), M, shape == 'box')[0])
        if not inside: outside[ep[f]] += 1
    return share, in_mode, outside


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clips', nargs='+')
    ap.add_argument('--workers', type=int, default=6)
    ap.add_argument('--only', nargs='+', help='shapes to run, all by default')
    a = ap.parse_args()
    clips = [pathlib.Path(c) for c in a.clips]
    variants = [v for v in VARIANTS if not a.only or v[0] in a.only]
    jobs = [(str(c), argparse.Namespace(**BASE, crop=s, crop_size=z, tag=tag(s, z))) for s, z in variants for c in clips]
    with Pool(a.workers) as pool:
        outs = pool.map(track3d.run_clip, jobs, chunksize=1)
    cal = ff.load_cal()
    infos = [sweep.clip_info(c, 'left') for c in clips]
    read = lambda o: [(int(r['frame']), int(r['known_at']), float(r['x0']), float(r['y0']), float(r['w']), float(r['h']))
                      for r in csv.DictReader(open(o))]
    rng_ft = np.concatenate([[i['range']] * i['nflights'] for i in infos])
    R, D, C, S = [], [], [], []
    for k, (s, z) in enumerate(variants):
        per, dl, cm, sh = [], [], [], []
        for n, (c, i) in enumerate(zip(clips, infos)):
            reps = read(outs[k * len(clips) + n])
            per.append(track3d_sweep.per_flight(c, i, reps))
            dl += [(i['range'], d) for d in track3d_sweep.delays(c, i, reps)]
            share, in_mode, outside = crop_stats(c, i, s, z, cal)
            cm.append(np.column_stack([in_mode, outside]))
            sh.append((i['range'], share))
        R.append(np.concatenate(per)); D.append(dl); C.append(np.concatenate(cm)); S.append(sh)
    R, C = np.stack(R), np.stack(C)
    np.savez_compressed(clips[0].parent / 'track3d_crop.npz', R=R, C=C, range=rng_ft,
                        variants=np.array([name(s, z) for s, z in variants]),
                        share=np.array([np.mean(np.concatenate([x for _, x in sh])) for sh in S]),
                        clips=[i['name'] for i in infos])

    print(f'{len(clips)} clips, {R.shape[1]} flights; found counts live reports, outside is flying frames searched '
          f'in a crop with the label outside every crop, searched is the mean share of both full frames')
    print('  crop            range  found live  false/min  flights  delay med, p90  outside  searched  in crop mode')
    for k, (s, z) in enumerate(variants):
        for rg in ('all', '11', '35'):
            e = np.ones(len(rng_ft), bool) if rg == 'all' else rng_ft == rg
            t, c = R[k, e].sum(0), C[k, e].sum(0)
            dl = [d for r, d in D[k] if rg == 'all' or r == rg]
            sh = np.concatenate([x for r, x in S[k] if rg == 'all' or r == rg])
            print(f'  {name(s, z):15s} {rg:>4s}  {100 * t[0] / t[1]:9.1f}%  {t[3] / t[4]:9.1f}  '
                  f'{np.sum(R[k, e, 0] > 0):3d}/{e.sum():<3d}  {np.median(dl):6.0f}, {np.percentile(dl, 90):4.0f}  '
                  f'{100 * c[1] / max(c[0], 1):6.1f}%  {100 * sh.mean():7.1f}%  {100 * np.mean(sh < 1):9.1f}%')


if __name__ == '__main__':
    main()

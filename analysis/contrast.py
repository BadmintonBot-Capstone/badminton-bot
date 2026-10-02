# how far the flying shuttle stands out per camera setting, before any detector choice
#   contrast.py <clip> [<clip> ...] [--camera left]
#
# uses what cands.py saved: at every frame labelled flying, the largest change within 24 px of the label,
# and the typical (median) change over the whole frame. a detector threshold can only find the frames
# where the shuttle's change clears it, so the share of frames above each threshold is a ceiling on what
# any threshold detector gets with that background model. split by the shuttle's image speed from the
# labels, in full resolution px per ms.
import csv, pathlib, argparse, numpy as np, yaml

SETTINGS = ['baseline', 'short_exposure', 'shorter_exposure', 'higher_frame_rate']
SPEEDS = [0, 1, 2, 4, 100]
LEVELS = [6, 12, 20]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clips', nargs='+')
    ap.add_argument('--camera', default='left')
    a = ap.parse_args()
    rows = []
    for c in map(pathlib.Path, a.clips):
        p = c / 'sweep' / f'{a.camera}.npz'
        if not p.exists(): continue
        z = np.load(p)
        meta = yaml.safe_load(open(c / 'meta.yaml'))
        lab = {int(r['frame']): r for r in csv.DictReader(open(c / 'labels' / f'{a.camera}.csv'))}
        for f, r in lab.items():
            if r['state'] != 'flying' or not r['x']: continue
            q = lab.get(f - 1)
            if not q or q['state'] != 'flying' or not q['x'] or r['new_flight'] == '1': continue
            v = np.hypot(float(r['x']) - float(q['x']), float(r['y']) - float(q['y'])) * meta['fps'] / 1000
            rows.append((meta['notes']['setting'], meta['notes']['range_ft'], v, z['peak'][f], z['noise'][f]))
        models = list(z['models'])
    if not rows: return
    st = np.array([r[0] for r in rows])
    rg = np.array([r[1] for r in rows])
    v = np.array([r[2] for r in rows])
    peak = np.stack([r[3] for r in rows])
    noise = np.stack([r[4] for r in rows])

    for mi, m in enumerate(models):
        print(f'\nmodel {m}: median shuttle change / median frame change, and share of flying frames above '
              + ', '.join(f'{t}' for t in LEVELS))
        for r in sorted(set(rg), key=float):
            for s in SETTINGS:
                k = (st == s) & (rg == r)
                if not k.any(): continue
                p, nz = peak[k, mi], noise[k, mi]
                print(f'  {r:>3s} ft {s:18s} {k.sum():5d} frames  {np.nanmedian(p):5.1f} / {np.median(nz):4.2f}  '
                      + '  '.join(f'{100 * np.mean(p > t):3.0f}%' for t in LEVELS))

    print('\nshare of flying frames with shuttle change above 12, by image speed (px per ms), model ra:')
    print('  range setting            ' + ''.join(f'{lo}-{hi if hi < 100 else "":<6}' for lo, hi in zip(SPEEDS, SPEEDS[1:])))
    mi = models.index('ra')
    for r in sorted(set(rg), key=float):
        for s in SETTINGS:
            cells = []
            for lo, hi in zip(SPEEDS, SPEEDS[1:]):
                k = (st == s) & (rg == r) & (v >= lo) & (v < hi)
                cells.append(f'{100 * np.mean(peak[k, mi] > 12):3.0f}% ({k.sum():4d})' if k.sum() >= 20 else f'{"-":>11}')
            print(f'  {r:>3s} ft {s:18s} ' + ' '.join(cells))


if __name__ == '__main__':
    main()

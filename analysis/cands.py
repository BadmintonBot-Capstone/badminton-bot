# motion blobs in every frame of a clip, for several background models and thresholds, saved for sweep.py
#   cands.py <clip> [--camera left] [--frames N]
#
# one read of the clip gives the blobs every detector variant would see, so the sweep runs on tables.
# models, all on the tracker's half resolution grey with its gaussian blur of the change:
#   ra    change from a running average background (alpha 0.05, the current tracker)
#   prev  change from the previous frame
#   tri   smaller of the changes from the previous and the next frame (reports one frame late)
# writes <clip>/sweep/<camera>.npz: one row per blob per model and threshold (area, centroid and box in
# full resolution pixels, mean change, yellowness), and per frame per model the typical background
# change and the largest change within 24 px of the label, for the signal check.
import sys, pathlib, argparse, csv, time, numpy as np, cv2
from rec import load

cv2.setNumThreads(1)

MODELS = ['ra', 'prev', 'tri']
THRESH = [6, 8, 10, 12, 15, 20, 25, 30]
BLUR = 1.5
NEAR = 12  # half resolution px around the label


def labels(clip, cam, n):
    st = np.full(n, '', object)
    xy = np.full((n, 2), np.nan, np.float32)
    p = clip / 'labels' / f'{cam}.csv'
    if p.exists():
        for r in csv.DictReader(open(p)):
            f = int(r['frame'])
            if f < n:
                st[f] = r['state']
                if r['x']: xy[f] = float(r['x']), float(r['y'])
    return st, xy


def blobs(d, T, R, G, B):
    mask = (d > T).astype(np.uint8)
    n, lbl, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8, ltype=cv2.CV_32S)
    if n <= 1: return None
    ys, xs = np.nonzero(mask)
    k = lbl[ys, xs]
    w = d[ys, xs] - T
    sw = np.bincount(k, w, n)
    rg = 0.5 * (np.bincount(k, w * R[ys, xs], n) + np.bincount(k, w * G[ys, xs], n)) / np.maximum(sw, 1e-6)
    b = np.bincount(k, w * B[ys, xs], n) / np.maximum(sw, 1e-6)
    mean = np.bincount(k, d[ys, xs], n) / np.maximum(stats[:, 4], 1)
    keep = np.arange(1, n)[stats[1:, 4] >= 2]
    s = stats[keep]
    return np.column_stack([np.full(len(keep), T), s[:, 4], 2 * cents[keep, 0] + 0.5, 2 * cents[keep, 1] + 0.5,
                            2 * s[:, 0], 2 * s[:, 1], 2 * s[:, 2], 2 * s[:, 3], mean[keep],
                            ((rg - b) / (rg + b + 1))[keep]]).astype(np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clip')
    ap.add_argument('--camera', default='left')
    ap.add_argument('--frames', type=int, default=0)
    a = ap.parse_args()
    clip = pathlib.Path(a.clip)
    fr, meta = load(clip, a.camera)
    n = a.frames or len(fr)
    st, xy = labels(clip, a.camera, n)
    half = lambda f: cv2.resize(f, (f.shape[1] // 2, f.shape[0] // 2), interpolation=cv2.INTER_AREA).astype(np.float32)

    rows = {m: [] for m in MODELS}
    noise = np.zeros((n, len(MODELS)), np.float32)
    peak = np.full((n, len(MODELS)), np.nan, np.float32)
    bg = g_prev = None
    g_next = half(fr[0])
    t0 = time.time()
    for i in range(n):
        g, raw = g_next, np.asarray(fr[i])
        g_next = half(fr[i + 1]) if i + 1 < n else None
        R = raw[0::2, 0::2].astype(np.float32)
        G = 0.5 * (raw[0::2, 1::2].astype(np.float32) + raw[1::2, 0::2])
        B = raw[1::2, 1::2].astype(np.float32)
        if bg is None: bg = g.copy()
        zero = np.zeros_like(g)
        ds = {'ra': cv2.absdiff(g, bg),
              'prev': cv2.absdiff(g, g_prev) if g_prev is not None else zero,
              'tri': np.minimum(cv2.absdiff(g, g_prev), cv2.absdiff(g, g_next))
                     if g_prev is not None and g_next is not None else zero}
        cv2.accumulateWeighted(g, bg, 0.05)
        g_prev = g
        for mi, m in enumerate(MODELS):
            d = cv2.GaussianBlur(ds[m], (0, 0), BLUR)
            noise[i, mi] = np.median(d[::4, ::4])
            if st[i] in ('flying', 'still') and not np.isnan(xy[i, 0]):
                hx, hy = int((xy[i, 0] - 0.5) / 2), int((xy[i, 1] - 0.5) / 2)
                win = d[max(0, hy - NEAR):hy + NEAR + 1, max(0, hx - NEAR):hx + NEAR + 1]
                if win.size: peak[i, mi] = win.max()
            for T in THRESH:
                b = blobs(d, T, R, G, B)
                if b is not None:
                    rows[m].append(np.column_stack([np.full(len(b), i, np.float32), b]))
        if i % 500 == 0:
            print(f'{clip.name} {i}/{n} {time.time() - t0:.0f}s', flush=True)

    out = clip / 'sweep'
    out.mkdir(exist_ok=True)
    cols = 'frame,thresh,area,cx,cy,x0,y0,w,h,mean,yellow'
    np.savez_compressed(out / f'{a.camera}.npz', models=np.array(MODELS), thresh=np.array(THRESH), cols=cols,
                        noise=noise, peak=peak, fps=meta['fps'],
                        **{m: np.concatenate(rows[m]) if rows[m] else np.zeros((0, 11), np.float32) for m in MODELS})
    print(f'{clip.name} done {n} frames {time.time() - t0:.0f}s')


if __name__ == '__main__':
    main()

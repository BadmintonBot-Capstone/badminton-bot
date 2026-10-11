# motion blobs in every frame of a clip, for several background models and thresholds, saved for sweep_single.py
#   cands_mog2.py <clip> [--camera primary|left|right] [--frames N] [--history 100 500] [--var 16 32 64 128 256]
#                 [--tri-t 6 12] [--inputs g gr c cr] [--workers 4] [--bench 200] [--add]
#
# cands.py with opencv's MOG2 background subtractor added, and reading preview recordings as well as
# record_stereo and record_single clips (rec_any.py).
# one read of the clip gives the blobs every detector variant would see, so the sweep runs on tables.
# models, all at half resolution (each 2x2 bayer cell is one pixel):
#   ra    change from a running average background (alpha 0.05, the current tracker)
#   prev  change from the previous frame
#   tri   smaller of the changes from the previous and the next frame (reports one frame late)
#         these three are on grey with a gaussian blur of the change, exactly as in cands.py
#   mog2_<in>_h<H>         MOG2 foreground, history H frames, shadows off. its threshold column is
#                          MOG2's varThreshold (squared distance in standard deviations) instead of a
#                          grey level; one subtractor runs per varThreshold
#   mog2tri<T>_<in>_h<H>   MOG2 foreground where tri also changed by more than T grey levels, to drop
#                          the ghost a still object leaves when it starts moving (reports one frame late)
#   <in> is MOG2's input: g grey blurred, gr grey not blurred, c colour blurred, cr colour not blurred
#   (colour is the cell's red, mean green and blue; blurred is a gaussian of sigma BLUR on the image)
#   g8, c8 are g and c made in 8 bits all the way (no float round trip; rounding differs slightly)
# MOG2 gives a yes/no mask, so its blobs' mean change comes from the change against MOG2's own background
# image (mean over the channels for colour). MOG2 learns its first frame as all foreground, so no MOG2
# blobs are kept on frame 0; it needs more frames to settle, which sweep_single.py skips with --start.
# every blob has two yellowness values:
#   yellow        cands.py's: colour weighted by how far each pixel changed (past the threshold for the
#                 change models, against MOG2's background for MOG2), so not equally strict across models
#   yellow_plain  the plain mean colour of the blob's pixels, the same for every model
# the work is split over --workers processes by model group; each group writes
#   <clip>/sweep/<camera>_mog2_<group>.npz  (group: classic, or one MOG2 input)
# with one row per blob per model and threshold (area, centroid and box in full resolution pixels, mean
# change, both yellownesses), and per frame per model the typical background change and the largest change
# within 24 px of the label, for the signal check. afterwards each MOG2 input is timed alone on --bench
# frames (one subtractor, apply only), so the parallel run does not inflate the timing.
import sys, pathlib, argparse, csv, time, numpy as np, cv2
from multiprocessing import Pool
from rec_any import load, default_camera

cv2.setNumThreads(1)

MODELS = ['ra', 'prev', 'tri']
THRESH = [6, 8, 10, 12, 15, 20, 25, 30, 40, 50]
MOG2_HISTORY = [100, 500]
MOG2_VAR = [16, 32, 64, 128, 256, 512]  # opencv's default is 16
TRI_T = [6, 12]  # grey levels tri must change by for mog2tri
INPUTS = {'g': 'grey, blurred', 'gr': 'grey, not blurred', 'c': 'colour, blurred', 'cr': 'colour, not blurred',
          'g8': 'grey, blurred, 8 bit', 'c8': 'colour, blurred, 8 bit'}
BLUR = 1.5
NEAR = 12  # half resolution px around the label
COLS = 'frame,thresh,area,cx,cy,x0,y0,w,h,mean,yellow,yellow_plain,red,green,blue'  # tables before g8/c8 stop at yellow_plain


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


def blobs(mask, wgt, d, T, R, G, B):
    # mask picks the pixels, wgt weighs each pixel's colour for yellow, d gives the blob's mean change,
    # T is stored as the threshold
    n, lbl, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8, ltype=cv2.CV_32S)
    if n <= 1: return None
    ys, xs = np.nonzero(mask)
    k = lbl[ys, xs]
    Rp, Gp, Bp = R[ys, xs], G[ys, xs], B[ys, xs]
    w = wgt[ys, xs]
    sw = np.bincount(k, w, n)
    rg = 0.5 * (np.bincount(k, w * Rp, n) + np.bincount(k, w * Gp, n)) / np.maximum(sw, 1e-6)
    b = np.bincount(k, w * Bp, n) / np.maximum(sw, 1e-6)
    area = np.maximum(stats[:, 4], 1)
    r_p, g_p = np.bincount(k, Rp, n) / area, np.bincount(k, Gp, n) / area
    rg_p = 0.5 * (r_p + g_p)
    b_p = np.bincount(k, Bp, n) / area
    mean = np.bincount(k, d[ys, xs], n) / area
    keep = np.arange(1, n)[stats[1:, 4] >= 2]
    s = stats[keep]
    return np.column_stack([np.full(len(keep), T), s[:, 4], 2 * cents[keep, 0] + 0.5, 2 * cents[keep, 1] + 0.5,
                            2 * s[:, 0], 2 * s[:, 1], 2 * s[:, 2], 2 * s[:, 3], mean[keep],
                            ((rg - b) / (rg + b + 1))[keep], ((rg_p - b_p) / (rg_p + b_p + 1))[keep],
                            r_p[keep], g_p[keep], b_p[keep]]).astype(np.float32)


def half(f):
    # 2x2 area average of the bayer mosaic: grey at half resolution
    return cv2.resize(f, (f.shape[1] // 2, f.shape[0] // 2), interpolation=cv2.INTER_AREA).astype(np.float32)


def channels(raw):
    # red, mean green and blue of each 2x2 BayerRG cell
    R = raw[0::2, 0::2].astype(np.float32)
    G = 0.5 * (raw[0::2, 1::2].astype(np.float32) + raw[1::2, 0::2])
    B = raw[1::2, 1::2].astype(np.float32)
    return R, G, B


def mog_input(g, R, G, B, inp, raw=None):
    # the 8 bit image MOG2 is given for input inp
    if inp == 'g8':
        return cv2.GaussianBlur(cv2.resize(raw, (raw.shape[1] // 2, raw.shape[0] // 2), interpolation=cv2.INTER_AREA),
                                (0, 0), BLUR)
    if inp == 'c8':
        c = lambda a: np.ascontiguousarray(a)
        g8 = cv2.addWeighted(c(raw[0::2, 1::2]), 0.5, c(raw[1::2, 0::2]), 0.5, 0)
        return cv2.GaussianBlur(cv2.merge([c(raw[1::2, 1::2]), g8, c(raw[0::2, 0::2])]), (0, 0), BLUR)
    x = g if inp in ('g', 'gr') else cv2.merge([B, G, R])
    if inp in ('g', 'c'): x = cv2.GaussianBlur(x, (0, 0), BLUR)
    return np.clip(x + 0.5, 0, 255).astype(np.uint8)


def mog_change(u8, bgimg):
    # change against MOG2's background image, in grey levels (mean over the channels for colour)
    d = np.abs(u8.astype(np.float32) - bgimg.astype(np.float32))
    return d.mean(2) if d.ndim == 3 else d


def mog_models(inp, history, tri_t):
    return [f'mog2_{inp}_h{h}' for h in history] + [f'mog2tri{t}_{inp}_h{h}' for h in history for t in tri_t]


def run_group(job):
    # one model group over the whole clip: 'classic' (ra, prev, tri) or one MOG2 input
    clip, cam, n, group, history, var, tri_t = job
    fr, _ = load(clip, cam)
    st, xy = labels(clip, cam, n)
    models = MODELS if group == 'classic' else mog_models(group, history, tri_t)
    subs = {} if group == 'classic' else \
        {h: [cv2.createBackgroundSubtractorMOG2(history=h, varThreshold=v, detectShadows=False) for v in var]
         for h in history}
    rows = {m: [] for m in models}
    noise = np.zeros((n, len(models)), np.float32)
    peak = np.full((n, len(models)), np.nan, np.float32)
    bg = g_prev = None
    g_next = half(fr[0])
    t0 = time.time()
    for i in range(n):
        g, raw = g_next, np.asarray(fr[i])
        g_next = half(fr[i + 1]) if i + 1 < n else None
        R, G, B = channels(raw)
        has_label = st[i] in ('flying', 'still') and not np.isnan(xy[i, 0])
        hx, hy = (int((xy[i, 0] - 0.5) / 2), int((xy[i, 1] - 0.5) / 2)) if has_label else (0, 0)

        def signal(mi, d):
            noise[i, mi] = np.median(d[::4, ::4])
            if has_label:
                win = d[max(0, hy - NEAR):hy + NEAR + 1, max(0, hx - NEAR):hx + NEAR + 1]
                if win.size: peak[i, mi] = win.max()

        def add(m, b):
            if b is not None:
                rows[m].append(np.column_stack([np.full(len(b), i, np.float32), b]))

        zero = np.zeros_like(g)
        tri = np.minimum(cv2.absdiff(g, g_prev), cv2.absdiff(g, g_next)) \
            if g_prev is not None and g_next is not None else zero
        if group == 'classic':
            if bg is None: bg = g.copy()
            ds = {'ra': cv2.absdiff(g, bg),
                  'prev': cv2.absdiff(g, g_prev) if g_prev is not None else zero,
                  'tri': tri}
            cv2.accumulateWeighted(g, bg, 0.05)
            for mi, m in enumerate(MODELS):
                d = cv2.GaussianBlur(ds[m], (0, 0), BLUR)
                signal(mi, d)
                for T in THRESH:
                    add(m, blobs(d > T, d - T, d, T, R, G, B))
        else:
            u8 = mog_input(g, R, G, B, group, raw)
            moved = {t: cv2.GaussianBlur(tri, (0, 0), BLUR) > t for t in tri_t}
            for h in history:
                fgs = [s.apply(u8) for s in subs[h]]
                # the background image does not depend on varThreshold, so the first subtractor's serves all
                dm = mog_change(u8, subs[h][0].getBackgroundImage())
                names = [f'mog2_{group}_h{h}'] + [f'mog2tri{t}_{group}_h{h}' for t in tri_t]
                for m in names:
                    signal(models.index(m), dm)
                if i == 0: continue  # MOG2 marks the whole first frame as foreground
                for v, fg in zip(var, fgs):
                    fg = fg > 0
                    add(names[0], blobs(fg, dm + 1, dm, v, R, G, B))
                    for t, m in zip(tri_t, names[1:]):
                        add(m, blobs(fg & moved[t], dm + 1, dm, v, R, G, B))
        g_prev = g
        if i % 500 == 0:
            print(f'{clip.name} {group} {i}/{n} {time.time() - t0:.0f}s', flush=True)

    out = clip / 'sweep' / f'{cam}_mog2_{group}.npz'
    np.savez_compressed(out, models=np.array(models), cols=COLS, noise=noise, peak=peak, frames=n,
                        **{m: np.concatenate(rows[m]) if rows[m] else np.zeros((0, 15), np.float32) for m in models})
    return group, time.time() - t0


def bench(fr, n, inputs, history):
    # ms per frame for one MOG2 apply on each input, run alone (frames are read and prepared outside the timing)
    out = {}
    for inp in inputs:
        s = cv2.createBackgroundSubtractorMOG2(history=max(history), varThreshold=16, detectShadows=False)
        ms = []
        for i in range(n):
            raw = np.asarray(fr[i])
            u8 = mog_input(half(raw), *channels(raw), inp, raw)
            t1 = time.perf_counter()
            s.apply(u8)
            ms.append((time.perf_counter() - t1) * 1000)
        out[inp] = float(np.median(ms[10:]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clip')
    ap.add_argument('--camera')
    ap.add_argument('--frames', type=int, default=0)
    ap.add_argument('--history', type=int, nargs='+', default=MOG2_HISTORY)
    ap.add_argument('--var', type=float, nargs='+', default=MOG2_VAR)
    ap.add_argument('--tri-t', type=float, nargs='*', default=TRI_T, help='none for no mog2tri models')
    ap.add_argument('--inputs', nargs='+', default=list(INPUTS), choices=list(INPUTS))
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--add', action='store_true', help='run only --inputs and add them to the earlier run')
    ap.add_argument('--bench', type=int, default=200, help='frames to time each MOG2 input on, 0 to skip')
    a = ap.parse_args()
    clip = pathlib.Path(a.clip)
    cam = a.camera or default_camera(clip)
    fr, meta = load(clip, cam)
    n = min(a.frames or len(fr), len(fr))
    (clip / 'sweep').mkdir(exist_ok=True)
    t0 = time.time()
    groups = a.inputs if a.add else ['classic'] + a.inputs
    jobs = [(clip, cam, n, g, a.history, a.var, a.tri_t) for g in groups]
    with Pool(min(a.workers, len(jobs))) as pool:
        for group, s in pool.imap_unordered(run_group, jobs):
            print(f'{clip.name} {group} done in {s:.0f}s', flush=True)
    ms = bench(fr, min(a.bench, n), a.inputs, a.history) if a.bench else {}
    if a.add:
        # keep the groups and timings of the earlier run (same frames and settings)
        old = np.load(clip / 'sweep' / f'{cam}_mog2.npz')
        if int(old['frames']) != n: raise SystemExit('--add needs the same number of frames as the earlier run')
        groups = [g for g in old['groups'].tolist() if g not in groups] + groups
        ms = {**dict(zip(old['bench_inputs'].tolist(), old['bench_ms'].tolist())), **ms}
    np.savez_compressed(clip / 'sweep' / f'{cam}_mog2.npz', groups=np.array(groups),
                        fps=meta['fps'], frames=n, mog2_history=np.array(a.history), mog2_var=np.array(a.var),
                        tri_t=np.array(a.tri_t), blur=BLUR, bench_inputs=np.array(list(ms)),
                        bench_ms=np.array(list(ms.values())))
    print(f'{clip.name} done {n} frames in {time.time() - t0:.0f}s')
    for inp, v in ms.items():
        print(f'  MOG2 apply, {INPUTS[inp]:20s} {v:6.2f} ms per frame (median, one subtractor, one thread)')


if __name__ == '__main__':
    main()

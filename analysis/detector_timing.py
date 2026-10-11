# timing, centroid accuracy and colour check for the detector variants sweep_single.py picked
#   detector_timing.py <clip> [--camera primary|left] [--start 100] [--budgets 0.1 0.3 1] [--threads-frames 1000]
#
# the variants are the ones cross validation picked (colour rule allowed, mog2tri models left out) at each
# of --budgets in <clip>/sweep/sweep_single_<camera>_s<start>_plain.npz, plus the tracker's current detector.
# each background model and threshold is run on its own over the whole clip as a detector would run it:
#   prep    raw frame to half resolution grey and red, green, blue (the copy out of the file is not timed)
#   input   MOG2 only: its input image (grey or colour, blurred or not, to 8 bits)
#   model   ra, prev, tri: change image, blur, threshold; MOG2: apply
#   blobs   connected components, area rule, yellowness, colour rule (timed separately per variant)
# one OpenCV thread over the whole clip, then OpenCV's default threads over --threads-frames frames.
# times are medians and 95th percentiles per frame after the first --start frames, in python, so the blob
# step is an upper bound on what C++ would take. the blobs are checked against cands_mog2.py's tables.
# accuracy: on each labelled frame the detector found (a kept blob's box, grown by --margin, holds the
# label) the nearest such blob is the shuttle; its centroid's distance to the label is reported, with the
# share inside --circle px (the ring label.py draws). colour: for each model, threshold and area rule, the
# yellowness of the shuttle blobs against every blob that is not the shuttle, before the colour rule, with
# a white balanced yellowness and a hue and saturation rule for comparison.
# results go to <clip>/sweep/detector_timing_<camera>.txt and .npz
import sys, csv, time, pathlib, argparse, numpy as np, cv2
DEFAULT_THREADS = cv2.getNumThreads()  # before cands_mog2, which sets one thread on import
from rec_any import load, default_camera
from cands_mog2 import half, channels, mog_input, BLUR
from sweep_single import cross_validate, name, CURRENT

SPEED_BINS = [0, 5, 15, 30, 1e9]  # label speed, full resolution px per frame


class Frames:
    # frames read with plain file reads rather than through the memory map: touching every page of a long
    # clip through the map keeps it charged to the process on windows and runs the machine out of memory
    def __init__(self, mm):
        self.path, self.offset, self.shape = mm.filename, mm.offset, mm.shape
        self.size = int(np.prod(self.shape[1:]))
        self.f = open(self.path, 'rb', buffering=0)

    def __len__(self):
        return self.shape[0]

    def __getitem__(self, i):
        out = np.empty(self.shape[1:], np.uint8)
        self.f.seek(self.offset + int(i) * self.size)
        self.f.readinto(memoryview(out).cast('B'))
        return out


def variants(clip, cam, start, budgets):
    z = np.load(clip / 'sweep' / f'sweep_single_{cam}_s{start}_plain.npz', allow_pickle=True)
    R, configs, fold = z['R'], [tuple(c) for c in z['configs']], z['fold']
    eps = np.arange(R.shape[1])
    cur = configs.index(CURRENT)
    models = sorted(set(c[0] for c in configs if not c[0].startswith('mog2tri')),
                    key=lambda m: (m.startswith('mog2'), m))
    out = []
    for m in models:
        allowed = np.array([k for k, c in enumerate(configs) if c[0] == m and k != cur])
        for b in budgets:
            for p in cross_validate(R, eps, fold, allowed, b)[1]:
                if configs[p] not in out: out.append(configs[p])
    out.append(CURRENT)
    return [(str(m), float(T), int(a), int(x), float(y), str(yc)) for m, T, a, x, y, yc in out]


def find(mask, R, G, B, amin, amax, ymin, wgt=None):
    # kept blobs: centroid, box (full resolution), area and plain mean R, G, B; yellow is plain or weighted
    n, lbl, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8, ltype=cv2.CV_32S)
    area = stats[:, 4]
    ok = (area >= amin) & (area <= amax)
    ok[0] = False
    if not ok.any(): return None
    ys, xs = np.nonzero(mask)
    k = lbl[ys, xs]
    sel = ok[k]
    ys, xs, k = ys[sel], xs[sel], k[sel]
    Rp, Gp, Bp = R[ys, xs], G[ys, xs], B[ys, xs]
    a = np.maximum(area, 1)
    r, g, b = np.bincount(k, Rp, n) / a, np.bincount(k, Gp, n) / a, np.bincount(k, Bp, n) / a
    if wgt is None:
        rg = 0.5 * (r + g)
        yel = (rg - b) / (rg + b + 1)
    else:
        w = wgt[ys, xs]
        sw = np.maximum(np.bincount(k, w, n), 1e-6)
        rg = 0.5 * (np.bincount(k, w * Rp, n) + np.bincount(k, w * Gp, n)) / sw
        bw = np.bincount(k, w * Bp, n) / sw
        yel = (rg - bw) / (rg + bw + 1)
    keep = np.nonzero(ok & (yel >= ymin))[0]
    return keep, ok, stats, cents, r, g, b, yel


def run(fr, n, model, T, cfgs, frames, threads, collect):
    # one background model and threshold over frames [0, frames): per frame stage times in ms, and the blobs
    cv2.setNumThreads(threads)
    nc = len(cfgs)
    tm = np.full((frames, 3 + nc), np.nan)
    kept = [[] for _ in cfgs]   # frame, cx, cy, x0, y0, w, h, area, R, G, B, yellow, passed colour rule
    mog = model.startswith('mog2')
    if mog:
        inp, h = model.split('_')[1], int(model.split('_h')[1])
        sub = cv2.createBackgroundSubtractorMOG2(history=h, varThreshold=T, detectShadows=False)
    bg = g_prev = d_prev = None
    raw_next = np.array(fr[0])
    t = time.perf_counter()
    g_next, ch_next = half(raw_next), channels(raw_next)
    prep0 = time.perf_counter() - t
    for i in range(frames):
        g, (R, G, B), raw = g_next, ch_next, raw_next
        # prep of the next frame counts towards this one (tri needs it; the cost per frame is the same)
        if i + 1 < n:
            raw_next = np.array(fr[i + 1])
            t = time.perf_counter()
            g_next, ch_next = half(raw_next), channels(raw_next)
            tm[i, 0] = (time.perf_counter() - t) * 1000
        else:
            g_next, tm[i, 0] = None, prep0 * 1000
        tm[i, 1] = 0
        t = time.perf_counter()
        d = None
        if mog:
            u8 = mog_input(g, R, G, B, inp, raw)
            tm[i, 1] = (time.perf_counter() - t) * 1000
            t = time.perf_counter()
            fg = sub.apply(u8)
            mask = fg if i > 0 else None   # MOG2 marks the whole first frame as foreground
        else:
            if model == 'ra':
                if bg is None: bg = g.copy()
                d = cv2.absdiff(g, bg)
                cv2.accumulateWeighted(g, bg, 0.05)
            elif model == 'prev':
                d = cv2.absdiff(g, g_prev) if g_prev is not None else None
            else:  # tri: the difference to the previous frame is last frame's difference to the next
                d_next = cv2.absdiff(g, g_next) if g_next is not None else None
                d = cv2.min(d_prev, d_next) if d_prev is not None and d_next is not None else None
                d_prev = d_next
            if d is not None:
                d = cv2.GaussianBlur(d, (0, 0), BLUR)
                mask = cv2.compare(d, T, cv2.CMP_GT)
            else:
                mask = None
        tm[i, 2] = (time.perf_counter() - t) * 1000
        g_prev = g
        for j, (_, _, amin, amax, ymin, yc) in enumerate(cfgs):
            t = time.perf_counter()
            res = None
            if mask is not None:
                res = find(mask, R, G, B, amin, amax, ymin, d - T if yc == 'weighted' else None)
            tm[i, 3 + j] = (time.perf_counter() - t) * 1000
            if collect and res is not None:
                keep, ok, st, ce, r, gg, b, yel = res
                idx = np.nonzero(ok)[0]
                kept[j].append(np.column_stack([np.full(len(idx), i), 2 * ce[idx, 0] + 0.5, 2 * ce[idx, 1] + 0.5,
                                                2 * st[idx, 0], 2 * st[idx, 1], 2 * st[idx, 2], 2 * st[idx, 3],
                                                st[idx, 4], r[idx], gg[idx], b[idx], yel[idx],
                                                np.isin(idx, keep)]))
        if collect and i % 1000 == 0:
            print(f'  {model} T{T:g} {i}/{frames}', flush=True)
    kept = [np.concatenate(k).astype(np.float32) if k else np.zeros((0, 13), np.float32) for k in kept]
    return tm, kept


def check(clip, cam, model, T, cfg, rows, n):
    # the blobs against cands_mog2.py's table for this model and threshold, area rule applied
    group = 'classic' if not model.startswith('mog2') else model.split('_')[1]
    t = np.load(clip / 'sweep' / f'{cam}_mog2_{group}.npz')[model]
    t = t[(t[:, 1] == T) & (t[:, 2] >= cfg[2]) & (t[:, 2] <= cfg[3]) & (t[:, 0] < n)]
    t = t[np.lexsort((t[:, 4], t[:, 3], t[:, 0]))]
    r = rows[np.lexsort((rows[:, 2], rows[:, 1], rows[:, 0]))]
    if len(t) != len(r): return f'{len(r)} blobs here, {len(t)} in the table'
    diff = max(np.abs(t[:, [3, 4]] - r[:, [1, 2]]).max(initial=0), np.abs(t[:, 11] - r[:, 11]).max(initial=0)) \
        if cfg[5] == 'plain' else np.abs(t[:, [3, 4]] - r[:, [1, 2]]).max(initial=0)
    return f'{len(r)} blobs match, largest difference {diff:.2g}' if diff < 1e-3 else \
        f'{len(r)} blobs, largest difference {diff:.3g}'


def labels(clip, cam, n, start):
    st = np.full(n, -1, np.int8)
    xy = np.full((n, 2), np.nan)
    code = {'none': 0, 'flying': 1, 'still': 2, 'unsure': 3}
    for r in csv.DictReader(open(clip / 'labels' / f'{cam}.csv')):
        f = int(r['frame'])
        if f >= n: continue
        st[f] = code[r['state']]
        if r['x']: xy[f] = float(r['x']), float(r['y'])
    st[:start] = -1
    # label speed from the neighbouring labelled frames
    sp = np.full(n, np.nan)
    d = np.hypot(*(xy[1:] - xy[:-1]).T)
    sp[1:] = d
    sp[:-1] = np.where(np.isnan(sp[:-1]), d, sp[:-1])
    return st, xy, sp


def shuttle(rows, st, xy, margin, passed_only):
    # per blob: 1 the shuttle (nearest blob holding the label), 0 not the shuttle, -1 left out (another
    # blob holding the label, or a frame not counted); and per found frame the centroid error
    f = rows[:, 0].astype(int)
    s = st[f]
    lx, ly = xy[f, 0], xy[f, 1]
    inside = ((s == 1) | (s == 2)) & (lx >= rows[:, 3] - margin) & (lx <= rows[:, 3] + rows[:, 5] + margin) \
             & (ly >= rows[:, 4] - margin) & (ly <= rows[:, 4] + rows[:, 6] + margin)
    use = rows[:, 12] > 0 if passed_only else np.ones(len(rows), bool)
    dist = np.hypot(rows[:, 1] - lx, rows[:, 2] - ly)
    role = np.where((s >= 0) & (s <= 2) & use, 0, -1)
    role[inside & use] = -1
    cand = np.nonzero(inside & use)[0]
    best = {}
    for k in cand[np.argsort(dist[cand])]:
        best.setdefault(f[k], k)
    idx = np.array(sorted(best.values()), int)
    role[idx] = 1
    return role, f[idx], dist[idx]


def hue_sat(r, g, b):
    mx, mn = np.maximum(np.maximum(r, g), b), np.minimum(np.minimum(r, g), b)
    c = np.maximum(mx - mn, 1e-6)
    h = np.where(mx == r, ((g - b) / c) % 6, np.where(mx == g, (b - r) / c + 2, (r - g) / c + 4)) * 60
    return h, (mx - mn) / np.maximum(mx, 1e-6)


def false_at(pos, neg, keep):
    # smallest share of non shuttle blobs kept by a threshold keeping at least `keep` of the shuttle blobs
    t = np.quantile(pos, 1 - keep)
    return (neg >= t).mean(), t


def colour_report(pos, neg, wb):
    # pos, neg: rows of R, G, B, yellow for shuttle and other blobs
    out = []
    y_raw = lambda x: x[:, 3]
    def y_wb(x):
        r, g, b = x[:, 0] * wb[0], x[:, 1] * wb[1], x[:, 2] * wb[2]
        rg = 0.5 * (r + g)
        return (rg - b) / (rg + b + 1)
    p, q = y_raw(pos), y_raw(neg)
    out.append(f'    yellowness, shuttle median {np.median(p):.2f} [10%: {np.quantile(p, .1):.2f}], others median '
               f'{np.median(q):.2f} [90%: {np.quantile(q, .9):.2f}]; at 0.4 keeps {100 * (p >= .4).mean():.1f}% of '
               f'shuttle blobs and {100 * (q >= .4).mean():.1f}% of the others')
    for lab, fn in (('yellowness', y_raw), ('white balanced yellowness', y_wb)):
        p, q = fn(pos), fn(neg)
        parts = []
        for keep in (.9, .95, .99):
            fa, t = false_at(p, q, keep)
            parts.append(f'{100 * keep:.0f}% of shuttle: {100 * fa:.1f}% of others (>= {t:.2f})')
        out.append(f'    {lab:26s} ' + '; '.join(parts))
    # hue within a window and saturation above a floor, white balanced, best rule per shuttle share
    hp, sp_ = hue_sat(*(pos[:, :3] * wb).T)
    hq, sq = hue_sat(*(neg[:, :3] * wb).T)
    parts = []
    for keep in (.9, .95, .99):
        best = (1.1, None)
        for lo in range(0, 90, 5):
            for hi in range(lo + 10, 120, 5):
                inp, inq = (hp >= lo) & (hp <= hi), (hq >= lo) & (hq <= hi)
                if inp.mean() < keep: continue
                # the highest saturation floor still keeping `keep` of the shuttle blobs
                s = np.sort(sp_[inp])[::-1]
                need = int(np.ceil(keep * len(sp_)))
                if need > len(s): continue
                smin = s[need - 1]
                fa = (inq & (sq >= smin)).mean()
                if fa < best[0]: best = (fa, (lo, hi, smin))
        if best[1]:
            lo, hi, smin = best[1]
            parts.append(f'{100 * keep:.0f}% of shuttle: {100 * best[0]:.1f}% of others (hue {lo}-{hi}, sat >= {smin:.2f})')
    out.append(f'    {"hue and saturation":26s} ' + '; '.join(parts))
    return out


def stats(x):
    return f'mean {x.mean():5.1f}  median {np.median(x):5.1f}  90% {np.quantile(x, .9):5.1f}' if len(x) else 'none'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clip')
    ap.add_argument('--camera')
    ap.add_argument('--start', type=int, default=100)
    ap.add_argument('--budgets', type=float, nargs='+', default=[0.1, 0.3, 1])
    ap.add_argument('--margin', type=float, default=10)
    ap.add_argument('--circle', type=float, default=28)
    ap.add_argument('--frames', type=int, default=0)
    ap.add_argument('--threads-frames', type=int, default=1000)
    ap.add_argument('--models', nargs='+', help='only these models (for a quick check)')
    a = ap.parse_args()
    clip = pathlib.Path(a.clip)
    cam = a.camera or default_camera(clip)
    fr, meta = load(clip, cam)
    fr = Frames(fr)
    n = min(a.frames or len(fr), len(fr))
    if sys.platform == 'win32':
        # keep the machine and screen awake until this process ends (Modern Standby sleeps with the screen)
        import ctypes
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000003)
    cfgs = variants(clip, cam, a.start, a.budgets)
    if a.models: cfgs = [c for c in cfgs if c[0] in a.models]
    st, xy, speed = labels(clip, cam, n, a.start)
    groups = {}
    for c in cfgs: groups.setdefault(c[:2], []).append(c)
    lines = [f'{clip.name}: {len(cfgs)} variants ({len(groups)} model and threshold passes), {n} frames, '
             f'times after frame {a.start}, OpenCV default threads {DEFAULT_THREADS}', '']
    print(lines[0], flush=True)

    # grey world white balance from 50 frames spread over the clip
    m = np.mean([[c.mean() for c in channels(np.array(fr[i]))] for i in np.linspace(0, n - 1, 50).astype(int)], 0)
    wb = m[1] / m
    lines.append(f'white balance gains (grey world) R {wb[0]:.2f} G {wb[1]:.2f} B {wb[2]:.2f}\n')

    # each pass is saved as it finishes and reused if the run is restarted with the same settings
    parts = clip / 'sweep' / f'detector_timing_{cam}_parts'
    parts.mkdir(exist_ok=True)
    res = {}
    t0 = time.time()
    for (model, T), cs in groups.items():
        p = parts / f'{model}_T{T:g}_n{n}_s{a.start}_t{a.threads_frames}.npz'
        z = np.load(p, allow_pickle=True) if p.exists() else None
        if z is not None and [tuple(c) for c in z['cfgs']] == [tuple(c) for c in cs]:
            tm1, tmd, kept = z['tm1'], z['tmd'], [z[f'rows_{j}'] for j in range(len(cs))]
            print(f'  {model} T{T:g} loaded from {p.name}', flush=True)
        else:
            tm1, kept = run(fr, n, model, T, cs, n, 1, True)
            tmd, _ = run(fr, n, model, T, cs, min(n, a.start + a.threads_frames), DEFAULT_THREADS, False)
            np.savez(p, cfgs=np.array(cs, object), tm1=tm1, tmd=tmd, **{f'rows_{j}': k for j, k in enumerate(kept)})
        for j, c in enumerate(cs):
            res[c] = dict(tm1=tm1[:, [0, 1, 2, 3 + j]], tmd=tmd[:, [0, 1, 2, 3 + j]], rows=kept[j],
                          check=check(clip, cam, model, T, c, kept[j], n))
        print(f'  {model} T{T:g} done, {time.time() - t0:.0f}s', flush=True)

    # timing
    lines.append('timing, ms per frame (prep + MOG2 input + model + blobs; stage medians, total median / 95th percentile)')
    lines.append(f'  {"variant":52s} {"prep":>5s} {"input":>6s} {"model":>6s} {"blobs":>6s}   {"total 1 thread":>16s}   '
                 f'{"total " + str(DEFAULT_THREADS) + " threads":>17s}')
    for c in cfgs:
        t1, td = res[c]['tm1'][a.start:], res[c]['tmd'][a.start:]
        tot1, totd = t1.sum(1), td.sum(1)
        lines.append(f'  {name(c):52s} {np.median(t1[:, 0]):5.2f} {np.median(t1[:, 1]):6.2f} {np.median(t1[:, 2]):6.2f} {np.median(t1[:, 3]):6.2f}'
                     f'   {np.median(tot1):6.2f} / {np.quantile(tot1, .95):6.2f}   '
                     f'{np.median(totd):6.2f} / {np.quantile(totd, .95):6.2f}')
    lines.append('')

    # centroid accuracy, with the colour rule applied as the variant has it
    lines.append(f'centroid to label, full resolution px, on frames the variant found (inside {a.circle:g} px = '
                 f'share within the ring label.py draws)')
    bins = list(zip(SPEED_BINS[:-1], SPEED_BINS[1:]))
    for c in cfgs:
        role, ff, dist = shuttle(res[c]['rows'], st, xy, a.margin, True)
        lines.append(f'  {name(c)}')
        for lab, code in (('flying', 1), ('still', 2)):
            sel = st[ff] == code
            dd = dist[sel]
            lines.append(f'    {lab:7s} {len(dd):5d} frames  {stats(dd)}  inside {a.circle:g} px '
                         f'{100 * (dd <= a.circle).mean() if len(dd) else 0:5.1f}%')
        fly = st[ff] == 1
        parts = []
        for lo, hi in bins:
            sel = fly & (speed[ff] >= lo) & (speed[ff] < hi)
            dd = dist[sel]
            if len(dd):
                parts.append(f'{lo:g}-{"" if hi > 1e8 else f"{hi:g}"} px/frame: {len(dd)} frames, median '
                             f'{np.median(dd):.1f}, inside {100 * (dd <= a.circle).mean():.0f}%')
        lines.append('    flying by label speed: ' + '; '.join(parts))
        lines.append(f'    check against cands_mog2.py: {res[c]["check"]}')
    lines.append('')

    # colour, before the colour rule, per model, threshold and area rule
    lines.append('colour of the shuttle blobs against every other blob on counted frames, area rule applied, '
                 'colour rule not (the white balance and hue rules are tuned on this clip, not cross validated)')
    seen = set()
    for c in cfgs:
        key = c[:4]
        if key in seen: continue
        seen.add(key)
        rows = res[c]['rows']
        role, _, _ = shuttle(rows, st, xy, a.margin, False)
        pos, neg = rows[role == 1][:, 8:12], rows[role == 0][:, 8:12]
        if c[5] == 'weighted': continue  # its rows hold the weighted yellowness; the plain variants cover ra
        lines.append(f'  {c[0]} T{c[1]:g} area {c[2]}-{"any" if c[3] > 1e8 else c[3]}: {len(pos)} shuttle blobs, '
                     f'{len(neg)} others')
        lines += colour_report(pos, neg, wb) if len(pos) and len(neg) else ['    too few blobs']
    text = '\n'.join(lines)
    print('\n' + text)
    out = clip / 'sweep' / f'detector_timing_{cam}'
    open(f'{out}.txt', 'w').write(text + '\n')
    np.savez_compressed(f'{out}.npz', configs=np.array(cfgs, object), wb=wb,
                        **{f'tm1_{k}': res[c]['tm1'] for k, c in enumerate(cfgs)},
                        **{f'tmd_{k}': res[c]['tmd'] for k, c in enumerate(cfgs)},
                        **{f'rows_{k}': res[c]['rows'] for k, c in enumerate(cfgs)})
    print(f'saved {out}.txt and .npz')


if __name__ == '__main__':
    main()

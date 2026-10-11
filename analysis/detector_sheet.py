# contact sheet of what each detector sees on a random sample of labelled frames, to check the sweep by eye
#   detector_sheet.py <clip> [--models ra prev tri ...] [--flying 24] [--still 4] [--none 4] [--seed 0]
#                     [--budget 0.3] [--start 500] [--yellow plain|weighted]
#
# needs cands_mog2.py run on the clip first. each model is shown with the variant that finds the most
# flying frames over the whole clip within --budget false candidates per frame (the same scoring as
# sweep_single.py, picked on all flights, so this is for looking, not a held out score). without --models
# it shows ra, prev, tri and the best MOG2 model for each MOG2 input.
# frames are drawn at random with --seed from the labelled frames after --start, by state, at least
# --gap frames apart so the sample spreads over the clip.
# one row per frame: the whole frame with the crop marked, then per model a 256 px crop around the label,
# or on frames labelled none around a false candidate one of the shown models kept (the middle if none did):
#   magenta  the model's foreground mask, recomputed the way cands_mog2.py computes it
#   green    a kept blob that holds the label (the frame counts as found)
#   red      a kept blob that does not (a false candidate)
#   grey     a blob the area or colour filter dropped
#   cyan +   the label
# the text gives found or miss and the false candidates kept over the whole frame, not only the crop.
# writes <clip>/sweep/sheet_<n>.jpg, --rows rows per page.
import argparse, pathlib, itertools, numpy as np, cv2
from rec_any import load, default_camera
import sweep_single as ss
from cands_mog2 import BLUR, half, channels, mog_input

TILE, CROP, HEAD, MARGIN = 200, 256, 34, 10


def parse(m):
    # 'mog2_<in>_h<H>' or 'mog2tri<T>_<in>_h<H>' -> input, history, tri threshold (None for plain MOG2)
    kind, inp, h = m.split('_')
    return inp, int(h[1:]), (float(kind[7:]) if kind.startswith('mog2tri') else None)


def pick_variants(tables, models, info, budget, ycol):
    thresh = {m: sorted(set(np.unique(tables[m][:, 1]).tolist())) for m in models}
    configs = [(m, T, amin, amax, ymin, ycol) for m in models
               for T, amin, amax, ymin in itertools.product(thresh[m], ss.MIN_AREA, ss.MAX_AREA, ss.MIN_YELLOW)]
    s = ss.score_clip(tables, info, MARGIN, configs).sum(1)
    rec, fpf = s[:, 0] / np.maximum(s[:, 1], 1), s[:, 4] / np.maximum(s[:, 5], 1)
    out = {}
    for m in models:
        k = np.array([i for i, c in enumerate(configs) if c[0] == m])
        ok = fpf[k] <= budget
        if not ok.any(): ok = fpf[k] == fpf[k].min()
        best = k[np.argmax(np.where(ok, rec[k], -1))]
        out[m] = (configs[best], rec[best], fpf[best])
    return out


def sample(st, n, code, k, gap, rng):
    # k random frames of a state, each at least gap frames from the others
    pool = rng.permutation(np.flatnonzero(st[:n - 1] == code))
    chosen = []
    for f in pool:
        if len(chosen) == k: break
        if all(abs(f - c) >= gap for c in chosen): chosen.append(int(f))
    return chosen


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clip')
    ap.add_argument('--camera')
    ap.add_argument('--models', nargs='+')
    ap.add_argument('--flying', type=int, default=24)
    ap.add_argument('--still', type=int, default=4)
    ap.add_argument('--none', type=int, default=4)
    ap.add_argument('--gap', type=int, default=10)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--budget', type=float, default=0.3)
    ap.add_argument('--start', type=int, default=500)
    ap.add_argument('--yellow', choices=list(ss.YCOL), default='plain')
    ap.add_argument('--rows', type=int, default=12)
    a = ap.parse_args()
    clip = pathlib.Path(a.clip)
    cam = a.camera or default_camera(clip)
    cz = ss.load_cands(clip, cam)
    n, tables = cz['frames'], cz['tables']
    fr, meta = load(clip, cam)
    W, H = meta['width'], meta['height']
    info = ss.clip_info(clip, cam, n, a.start)
    st, xy = info['st'], info['xy']

    if a.models:
        models = a.models
        picks = pick_variants(tables, models, info, a.budget, a.yellow)
    else:
        # ra, prev, tri and the MOG2 model that finds the most flying frames within the budget, per input
        allp = pick_variants(tables, cz['models'], info, a.budget, a.yellow)
        models = ['ra', 'prev', 'tri']
        for inp in sorted({parse(m)[0] for m in cz['models'] if m.startswith('mog2')}):
            cand = [m for m in cz['models'] if m.startswith('mog2') and parse(m)[0] == inp]
            models.append(max(cand, key=lambda m: allp[m][1]))
        picks = {m: allp[m] for m in models}
    print(f'variants shown (best over the whole clip at {a.budget} false candidates per frame, {a.yellow} yellowness):')
    for m, (cfg, r, f) in picks.items():
        print(f'  {ss.name(cfg):48s} flying found {100 * r:.1f}%, {f:.2f} false per frame')

    rng = np.random.default_rng(a.seed)
    frames = sorted(sample(st, n, 1, a.flying, a.gap, rng) + sample(st, n, 2, a.still, a.gap, rng)
                    + sample(st, n, 0, a.none, a.gap, rng))
    print(f'sampled frames (seed {a.seed}, at least {a.gap} apart within a state): {frames}')

    # replay every model up to the last sampled frame, keeping each shown variant's mask on sampled frames
    subs = {m: cv2.createBackgroundSubtractorMOG2(history=parse(m)[1], varThreshold=picks[m][0][1], detectShadows=False)
            for m in models if m.startswith('mog2')}
    masks = {}
    want = set(frames)
    bg = g_prev = None
    g_next = half(fr[0])
    for i in range(max(frames) + 1):
        g, raw = g_next, np.asarray(fr[i])
        g_next = half(fr[i + 1]) if i + 1 < n else None
        if bg is None: bg = g.copy()
        zero = np.zeros_like(g)
        tri = np.minimum(cv2.absdiff(g, g_prev), cv2.absdiff(g, g_next)) \
            if g_prev is not None and g_next is not None else zero
        ds = {'ra': cv2.absdiff(g, bg), 'prev': cv2.absdiff(g, g_prev) if g_prev is not None else zero, 'tri': tri}
        cv2.accumulateWeighted(g, bg, 0.05)
        g_prev = g
        R, G, B = channels(raw)
        fg = {m: s.apply(mog_input(g, R, G, B, parse(m)[0])) > 0 for m, s in subs.items()}
        if i not in want: continue
        d = {k: cv2.GaussianBlur(v, (0, 0), BLUR) for k, v in ds.items()}
        masks[i] = {}
        for m in models:
            T = picks[m][0][1]
            if m in ds: masks[i][m] = d[m] > T
            else:
                t = parse(m)[2]
                masks[i][m] = fg[m] if t is None else fg[m] & (d['tri'] > t)

    names = {0: 'none', 1: 'flying', 2: 'still'}
    thumb_w = int(TILE * W / H)
    found_count = {m: 0 for m in models}
    ycol = ss.YCOL[a.yellow]

    def put(img, text, y, scale=0.45, col=(255, 255, 255)):
        cv2.putText(img, text, (4, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3)
        cv2.putText(img, text, (4, y), cv2.FONT_HERSHEY_SIMPLEX, scale, col, 1)

    def kept(m, i):
        (_, T, amin, amax, ymin, _), _, _ = picks[m]
        t = tables[m]
        t = t[(t[:, 0] == i) & (t[:, 1] == T)]
        return t, (t[:, 2] >= amin) & (t[:, 2] <= amax) & (t[:, ycol] >= ymin)

    rows = []
    for i in frames:
        img = cv2.convertScaleAbs(cv2.cvtColor(np.ascontiguousarray(fr[i]), cv2.COLOR_BayerBG2BGR), alpha=1.8)
        labelled = st[i] in (1, 2) and not np.isnan(xy[i, 0])
        if labelled:
            cx, cy = xy[i]
        else:
            # centre on a false candidate some shown model kept, picked at random
            fb = [(r[3], r[4]) for m in models for r, k in zip(*kept(m, i)) if k]
            cx, cy = fb[rng.integers(len(fb))] if fb else (W / 2, H / 2)
        x0, y0 = int(np.clip(cx - CROP / 2, 0, W - CROP)), int(np.clip(cy - CROP / 2, 0, H - CROP))
        thumb = cv2.resize(img, (thumb_w, TILE), interpolation=cv2.INTER_AREA)
        sx = thumb_w / W
        cv2.rectangle(thumb, (int(x0 * sx), int(y0 * sx)), (int((x0 + CROP) * sx), int((y0 + CROP) * sx)), (255, 255, 0), 1)
        if labelled: cv2.circle(thumb, (int(cx * sx), int(cy * sx)), 8, (255, 255, 0), 1)
        put(thumb, f'frame {i}  {names[int(st[i])]}', 16)
        tiles = [thumb]
        for m in models:
            crop = img[y0:y0 + CROP, x0:x0 + CROP].copy()
            mk = masks[i][m][y0 // 2:(y0 + CROP) // 2, x0 // 2:(x0 + CROP) // 2].astype(np.uint8)
            mk = cv2.resize(mk, (CROP, CROP), interpolation=cv2.INTER_NEAREST).astype(bool)
            crop[mk] = (0.45 * crop[mk] + 0.55 * np.array([255, 0, 255])).astype(np.uint8)
            tile = cv2.resize(crop, (TILE, TILE), interpolation=cv2.INTER_NEAREST)
            k = TILE / CROP
            t, keep = kept(m, i)
            bx, by, bw, bh = t[:, 5], t[:, 6], t[:, 7], t[:, 8]
            inside = labelled & (xy[i, 0] >= bx - MARGIN) & (xy[i, 0] <= bx + bw + MARGIN) \
                     & (xy[i, 1] >= by - MARGIN) & (xy[i, 1] <= by + bh + MARGIN)
            for j in np.argsort(keep):  # dropped blobs first, so kept ones draw on top
                col = (0, 255, 0) if keep[j] and inside[j] else (0, 0, 255) if keep[j] else (150, 150, 150)
                p0 = (int((bx[j] - x0) * k), int((by[j] - y0) * k))
                p1 = (int((bx[j] + bw[j] - x0) * k), int((by[j] + bh[j] - y0) * k))
                cv2.rectangle(tile, p0, p1, col, 2 if keep[j] else 1)
            if labelled:
                cv2.drawMarker(tile, (int((cx - x0) * k), int((cy - y0) * k)), (255, 255, 0), cv2.MARKER_CROSS, 14, 1)
            found = bool((keep & inside).any())
            if found: found_count[m] += 1
            nfalse = int((keep & ~inside).sum())
            if labelled:
                put(tile, 'FOUND' if found else 'miss', 16, col=(0, 255, 0) if found else (0, 0, 255))
            put(tile, f'{nfalse} false', TILE - 8, col=(0, 0, 255) if nfalse else (255, 255, 255))
            tiles.append(tile)
        rows.append(cv2.hconcat([cv2.copyMakeBorder(t, 2, 2, 2, 2, cv2.BORDER_CONSTANT) for t in tiles]))

    # header with each model and the variant shown
    width = rows[0].shape[1]
    head = np.zeros((HEAD, width, 3), np.uint8)
    xpos = thumb_w + 4
    for m in models:
        (_, T, amin, amax, ymin, _), _, _ = picks[m]
        cv2.putText(head, m, (xpos + 4, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        sub = f'T{T:g} area {amin}-{"any" if amax > 1e8 else amax}' + ('' if ymin < 0 else f' y{ymin}')
        cv2.putText(head, sub, (xpos + 4, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)
        xpos += TILE + 4
    cv2.putText(head, f'{clip.name}  seed {a.seed}', (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    out = clip / 'sweep'
    for old in out.glob('sheet_*.jpg'): old.unlink()
    for p in range(0, len(rows), a.rows):
        page = cv2.vconcat([head] + rows[p:p + a.rows])
        path = out / f'sheet_{p // a.rows + 1}.jpg'
        cv2.imwrite(str(path), page, [cv2.IMWRITE_JPEG_QUALITY, 90])
        print(f'wrote {path}')
    nl = sum(1 for i in frames if st[i] in (1, 2))
    print('found on the sampled flying and still frames: ' + ', '.join(f'{m} {found_count[m]}/{nl}' for m in models))


if __name__ == '__main__':
    main()

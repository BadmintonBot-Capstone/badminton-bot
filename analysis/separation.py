# 3D distance between the two shuttles on the E03 stick, every frame, the way src/core/stereo.cpp does it
#   separation.py <clip> [--pose across 500] [--pose along 1500] [--pick across 0,1] [--calib config/calibration.yaml]
#
# each frame is debayered and rectified whole with the calibration's maps (as StereoRig::rectify), the
# shuttles are found in the rectified images by colour, and each matching pair is triangulated with P1
# and P2 (as StereoRig::triangulate). each --pose names a frame inside one hold of the stick; at that
# frame the shuttle coloured blobs are paired across the cameras by row, and if there are more than two
# pairs --pick chooses them. from there each shuttle is followed forwards and backwards until it is lost.
# writes <clip>/separation.csv and <clip>/separation_<pose>.jpg to check what was found.
import argparse, pathlib, csv, numpy as np, cv2, yaml
from rec import load, bgr

# opencv hsv: hue 0 to 180. shuttle hue ~27, stick 15, wall 21, yellow bins 18, plant leaves 30 but dark
H_LO, H_HI, S_MIN, V_MIN = 22, 40, 120, 60
# a shuttle covers about 1300 px at 11 ft and 170 px at 30 ft, specks that pass the colour rule stay under 40
MIN_AREA, WIN, MAX_DY, MAX_SPEED = 60, 40, 4, 3
# per frame: clean clips reach 2.5 px of row mismatch at worst, wrong locks give 5 px or more
MAX_ROW, MAX_RATIO = 3.0, 1.5


def shuttle_mask(img):
    h, s, v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
    return ((h >= H_LO) & (h <= H_HI) & (s > S_MIN) & (v > V_MIN)).astype(np.uint8)


class Rig:
    # mirrors StereoRig: same maps, same interpolation, same triangulation
    def __init__(self, path, w, h):
        fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        c = {k: fs.getNode(k).mat() for k in ('M1', 'D1', 'M2', 'D2', 'R1', 'R2', 'P1', 'P2')}
        self.P1, self.P2 = c['P1'], c['P2']
        self.maps = [cv2.initUndistortRectifyMap(c[f'M{i}'], c[f'D{i}'], c[f'R{i}'], c[f'P{i}'], (w, h), cv2.CV_16SC2)
                     for i in (1, 2)]

    def rectify(self, raw, i):
        return cv2.remap(bgr(raw), *self.maps[i], cv2.INTER_LINEAR)

    def triangulate(self, l, r):
        X = cv2.triangulatePoints(self.P1, self.P2, np.array(l, float).reshape(2, 1), np.array(r, float).reshape(2, 1))
        return (X[:3] / X[3]).ravel()


def blobs(mask):
    n, _, st, cen = cv2.connectedComponentsWithStats(mask, connectivity=8)
    return [(cen[i], st[i, cv2.CC_STAT_AREA]) for i in range(1, n) if st[i, cv2.CC_STAT_AREA] >= MIN_AREA]


def pairs(bl, br):
    # left and right blobs on the same rectified row, right one shifted left (positive disparity)
    out = []
    for cl, al in bl:
        for cr, ar in br:
            if abs(cl[1] - cr[1]) < MAX_DY and 0 < cl[0] - cr[0] < 600 and 0.4 < al / ar < 2.5:
                out.append((cl, cr, al))
    return out


def follow(rect, f0, start, area, n):
    # follows one blob from frame f0 in both directions; rect(f) gives the rectified image
    pos = {f0: (start, area)}
    for step in (1, -1):
        p, f = np.array(start, float), f0 + step
        while 0 <= f < n:
            img = rect(f)
            x0, y0 = int(p[0]) - WIN, int(p[1]) - WIN
            if x0 < 0 or y0 < 0 or x0 + 2 * WIN >= img.shape[1] or y0 + 2 * WIN >= img.shape[0]:
                break
            cand = [(c + (x0, y0), a) for c, a in blobs(shuttle_mask(img[y0:y0 + 2 * WIN, x0:x0 + 2 * WIN]))
                    if a >= 0.3 * area]
            if not cand:
                break
            c, a = min(cand, key=lambda ca: np.hypot(*(ca[0] - p)))
            if np.hypot(*(c - p)) > WIN / 2:
                break
            pos[f], p, f = (c, a), c, f + step
    return pos


def sheet(path, clip, picked, found, rig, fl, fr, half=50):
    # one row per frame: left view with both shuttles circled, then each shuttle zoomed in each camera
    out = []
    for x in picked:
        f, pts = x['frame'], [s[0] for s in found[x['frame']]]
        L, R = rig.rectify(fl[f], 0), rig.rectify(fr[f], 1)
        view = L.copy()
        for p in pts:
            cv2.circle(view, (int(p[0][0]), int(p[0][1])), 40, (0, 0, 255), 4)
        view = cv2.resize(view, (480, 300))
        crops = []
        for p in pts:
            for img, c in ((L, p[0]), (R, p[1])):
                cx, cy = int(round(c[0])), int(round(c[1]))
                crop = cv2.copyMakeBorder(img, half, half, half, half, cv2.BORDER_CONSTANT)[cy:cy + 2 * half, cx:cx + 2 * half]
                crop = cv2.resize(crop, (300, 300), interpolation=cv2.INTER_NEAREST)
                k = 300 / (2 * half)
                mx, my = int(150 + (c[0] - cx) * k), int(150 + (c[1] - cy) * k)
                cv2.line(crop, (mx - 20, my), (mx + 20, my), (255, 0, 255), 1)
                cv2.line(crop, (mx, my - 20), (mx, my + 20), (255, 0, 255), 1)
                crops.append(cv2.copyMakeBorder(crop, 0, 0, 0, 4, cv2.BORDER_CONSTANT))
        row = cv2.hconcat([cv2.copyMakeBorder(view, 0, 0, 0, 4, cv2.BORDER_CONSTANT), cv2.hconcat(crops)])
        row = cv2.convertScaleAbs(row, alpha=2.0)
        label = f'{clip.name} frame {f} {x["pose"]} {x["sep_mm"]:.1f} mm'
        if x['bad']:
            label += f' DROPPED {x["bad"]}: rows {x["dy1"]:+.1f} {x["dy2"]:+.1f} px, size ratio {x["ratio"]:.2f}'
        cv2.putText(row, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        out.append(cv2.copyMakeBorder(row, 0, 4, 0, 0, cv2.BORDER_CONSTANT))
    if out:
        cv2.imwrite(str(path), cv2.vconcat(out))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clip')
    ap.add_argument('--pose', nargs=2, action='append', metavar=('NAME', 'FRAME'))
    ap.add_argument('--pick', nargs=2, action='append', metavar=('NAME', 'I,J'), default=[])
    ap.add_argument('--calib', default=pathlib.Path(__file__).parent.parent / 'config/calibration.yaml')
    a = ap.parse_args()
    poses = a.pose or [('across', '500'), ('along', '1500')]
    picks = {k: [int(i) for i in v.split(',')] for k, v in a.pick}

    clip = pathlib.Path(a.clip)
    meta = yaml.safe_load(open(clip / 'meta.yaml'))
    true_mm = float(meta.get('notes', {}).get('separation_mm', 914.4))
    fl, _ = load(clip, 'left')
    fr, _ = load(clip, 'right')
    n = min(len(fl), len(fr))
    rig = Rig(a.calib, meta['width'], meta['height'])
    cache = {}

    def rect(f, i):
        if (f, i) not in cache:
            if len(cache) > 400: cache.clear()
            cache[(f, i)] = rig.rectify((fl, fr)[i][f], i)
        return cache[(f, i)]

    found = {}
    for name, f0 in poses:
        f0 = int(f0)
        L, R = rect(f0, 0), rect(f0, 1)
        pp = sorted(pairs(blobs(shuttle_mask(L)), blobs(shuttle_mask(R))), key=lambda p: p[0][0])
        if name in picks:
            pp = [pp[i] for i in picks[name]]
        if len(pp) != 2:
            print(f'{clip.name} {name} frame {f0}: {len(pp)} candidate pairs, choose two with --pick {name} i,j')
            for i, (cl, cr, ar) in enumerate(pp):
                print(f'  {i}: left ({cl[0]:.0f}, {cl[1]:.0f})  right ({cr[0]:.0f}, {cr[1]:.0f})  area {ar}')
            continue
        tracks = [[follow(lambda f, i=i: rect(f, i), f0, p[i], p[2], n) for i in (0, 1)] for p in pp]
        frames = sorted(set.intersection(*[set(t) for s in tracks for t in s]))

        shot = cv2.hconcat([L, R])
        for (cl, cr, _) in pp:
            for c, dx in ((cl, 0), (cr, L.shape[1])):
                cv2.circle(shot, (int(c[0]) + dx, int(c[1])), 30, (0, 0, 255), 3)
        cv2.imwrite(str(clip / f'separation_{name}.jpg'), cv2.resize(shot, None, fx=0.4, fy=0.4))

        # a track can run through the turn into the other hold, so each frame is kept once.
        # per shuttle: ((left centre, right centre), (left area, right area))
        for f in frames:
            found.setdefault(f, [((s[0][f][0], s[1][f][0]), (s[0][f][1], s[1][f][1])) for s in tracks])

    # each frame is labelled by the stick's angle out of the image plane and how fast the shuttles move, and
    # dropped when a shuttle's two views disagree on the row or on the size, which a wrong lock in one camera causes
    rows, prev = [], None
    for f in sorted(found):
        pts, areas = [s[0] for s in found[f]], [s[1] for s in found[f]]
        X = [rig.triangulate(l, r) for l, r in pts]
        sep = np.linalg.norm(X[0] - X[1])
        angle = np.degrees(np.arcsin(min(1, abs(X[0][2] - X[1][2]) / sep)))
        speed = max(np.hypot(*(pts[k][0] - prev[1][k][0])) for k in (0, 1)) if prev and prev[0] == f - 1 else np.nan
        prev = (f, pts)
        pose = 'across' if angle < 25 else 'along' if angle > 35 else 'turning'
        dy = [p[0][1] - p[1][1] for p in pts]
        ratio = max(max(al / ar, ar / al) for al, ar in areas)
        bad = 'rows' if max(abs(d) for d in dy) > MAX_ROW else 'size' if ratio > MAX_RATIO else ''
        rows.append(dict(frame=f, pose=pose, moving=int(speed > MAX_SPEED), bad=bad, sep_mm=1000 * sep, angle=angle,
                         speed=speed, z1=X[0][2], z2=X[1][2], dy1=dy[0], dy2=dy[1], ratio=ratio,
                         l1x=pts[0][0][0], l1y=pts[0][0][1], l2x=pts[1][0][0], l2y=pts[1][0][1]))

    if not rows:
        return
    with open(clip / 'separation.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    used = lambda name: [x for x in rows if x['pose'] == name and not x['moving'] and not x['bad']]
    picked = [r[i] for name in ('across', 'along') if (r := used(name)) for i in np.linspace(0, len(r) - 1, 3).astype(int)]
    sheet(clip / 'separation_check.jpg', clip, picked, found, rig, fl, fr)
    worst = sorted([x for x in rows if x['bad']], key=lambda x: -max(abs(x['dy1']), abs(x['dy2']), 4 * (x['ratio'] - 1)))
    sheet(clip / 'separation_dropped.jpg', clip, worst[:6], found, rig, fl, fr)

    print(f'{clip.name}: true separation {true_mm:.1f} mm, frames moving over {MAX_SPEED} px per frame left out')
    for name in ('across', 'along'):
        r = used(name)
        if not r: continue
        s = np.array([x['sep_mm'] for x in r])
        z = np.median([(x['z1'] + x['z2']) / 2 for x in r])
        ang = np.median([x['angle'] for x in r])
        dy = np.median([abs(x['dy1']) for x in r] + [abs(x['dy2']) for x in r])
        print(f'  {name:7s} {len(r):4d} frames  range {z:5.2f} m  stick {ang:3.0f} deg out of plane  '
              f'separation {s.mean():6.1f} mm (error {s.mean() - true_mm:+6.1f}, std {s.std():4.1f})  '
              f'row mismatch {dy:.2f} px')
    print(f'  {sum(x["moving"] for x in rows)} moving and {sum(x["pose"] == "turning" for x in rows)} turning frames, '
          f'{n - len(rows)} frames without all four shuttle views, dropped {sum(x["bad"] == "rows" for x in rows)} '
          f'for rows over {MAX_ROW} px and {sum(x["bad"] == "size" for x in rows)} for sizes over {MAX_RATIO}x apart')


if __name__ == '__main__':
    main()

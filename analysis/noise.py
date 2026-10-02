# brightness, noise and false motion on still clips (E02)
#   noise.py <clip> [<clip> ...] [--camera left|right] [--baseline <clip>]
#
# brightness is the mean of the half resolution grey image the tracker uses, and saturated is the
# share of raw pixels at 250 or more. noise is each pixel's standard deviation over time, median over
# pixels, after dividing out the frame-wide flicker. false motion runs the tracker's own detection
# (running background, blur, threshold, from config/motion.yaml defaults) on the still clip and counts
# the pixels and blobs it flags. matched gain is where brightness at each exposure crosses the baseline
# clip's, interpolated in log brightness between the two gains around it.
import sys, argparse, pathlib, numpy as np, cv2
from rec import load, gray2

ALPHA, BLUR, THRESH, MIN_AREA, WARMUP = 0.05, 1.5, 12, 4, 40


def measure(d, camera):
    f, m = load(d, camera)
    g = gray2(f)
    med = np.median(g, axis=0)
    ok = (med > 10) & (med < 240)
    scale = (g[:, ok].astype(np.float64) / med[ok]).mean(axis=1)
    noise = np.median((g / scale[:, None, None].astype(np.float32)).std(axis=0))

    bg = g[0].copy()
    px, blobs = [], []
    for i, frame in enumerate(g):
        diff = cv2.GaussianBlur(cv2.absdiff(frame, bg), (0, 0), BLUR)
        mask = (diff > THRESH).astype(np.uint8)
        if i >= WARMUP:
            n, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            px.append(mask.mean())
            blobs.append(int((stats[1:, cv2.CC_STAT_AREA] >= MIN_AREA).sum()))
        cv2.accumulateWeighted(frame, bg, ALPHA)

    return dict(exposure=m['exposure_us'] / 1000, gain=m['gain_db'], mean=float(g.mean()), noise=float(noise),
                sat=100 * float((f[::10] >= 250).mean()), px=100 * float(np.mean(px)), blobs=float(np.mean(blobs)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clips', nargs='+')
    ap.add_argument('--camera', default='left')
    ap.add_argument('--baseline', help='clip whose brightness the other exposures are matched to')
    a = ap.parse_args()

    rows = {pathlib.Path(d).name: measure(d, a.camera) for d in a.clips}
    print(f'{a.camera} camera')
    print(f'{"clip":12s} {"exp ms":>6s} {"gain":>5s} {"bright":>7s} {"noise":>6s} {"sat %":>6s} {"false px %":>10s} {"false blobs":>11s}')
    for k, r in sorted(rows.items(), key=lambda kv: (-kv[1]['exposure'], kv[1]['gain'])):
        print(f'{k:12s} {r["exposure"]:6.2f} {r["gain"]:5.1f} {r["mean"]:7.1f} {r["noise"]:6.2f} {r["sat"]:6.2f} {r["px"]:10.4f} {r["blobs"]:11.2f}')

    if a.baseline:
        target = rows[pathlib.Path(a.baseline).name]['mean']
        print(f'\nmatched gain for brightness {target:.1f} (baseline {pathlib.Path(a.baseline).name}):')
        for e in sorted({r['exposure'] for r in rows.values()}, reverse=True):
            pts = sorted((r['gain'], np.log(r['mean'])) for r in rows.values() if r['exposure'] == e)
            for (g0, l0), (g1, l1) in zip(pts, pts[1:]):
                if l0 <= np.log(target) <= l1:
                    print(f'  {e:.2f} ms: {g0 + (np.log(target) - l0) / (l1 - l0) * (g1 - g0):.1f} dB')
                    break


if __name__ == '__main__':
    main()

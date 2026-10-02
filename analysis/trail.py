# where yellow things moved over a stereo clip, both cameras side by side, coloured by time
#   trail.py <clip> [out.jpg]   (default <clip>/<clip name>_trail.jpg)
#
# a quick check after each recording that the flights stayed in both views. the background is the
# median of every 25th frame; a pixel is drawn when it passes a loose yellow rule and differs from
# the background by more than 15 grey levels, coloured by the frame where it differed most
# (blue early, red late). skin and clothes also pass the loose rule, so the thrower shows up too.
import sys, pathlib, numpy as np, cv2
from rec import load


def half(f):
    # 2x2 bayer (RG) to half resolution bgr
    f = f.astype(np.uint16)
    return np.dstack([f[1::2, 1::2], (f[0::2, 1::2] + f[1::2, 0::2]) // 2, f[0::2, 0::2]]).astype(np.uint8)


def trail(clip, cam):
    fr, _ = load(clip, cam)
    n = len(fr)
    bg = np.median(np.stack([half(fr[i]) for i in range(0, n, 25)]), axis=0).astype(np.int16)
    best = np.zeros(bg.shape[:2], np.int16)
    when = np.zeros(bg.shape[:2], np.int32)
    for i in range(n):
        h = half(fr[i])
        hsv = cv2.cvtColor(h, cv2.COLOR_BGR2HSV)
        yel = (hsv[..., 0] >= 22) & (hsv[..., 0] <= 40) & (hsv[..., 1] > 120) & (hsv[..., 2] > 40)
        d = np.abs(h.astype(np.int16) - bg).max(axis=2) * yel
        m = d > best
        best[m] = d[m]
        when[m] = i
    img = (bg * 0.6).astype(np.uint8)
    hit = best > 15
    col = cv2.applyColorMap((when * 255 // max(n - 1, 1)).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[hit] = col[hit]
    img = cv2.dilate(img * hit[..., None], np.ones((3, 3), np.uint8)) + img * ~hit[..., None]
    cv2.putText(img, f'{cam}  (blue = start of clip, red = end)', (10, 30), 0, 0.8, (255, 255, 255), 2)
    return img


def main():
    clip = pathlib.Path(sys.argv[1])
    out = sys.argv[2] if len(sys.argv) > 2 else str(clip / f'{clip.name}_trail.jpg')
    cv2.imwrite(out, cv2.hconcat([trail(clip, 'left'), trail(clip, 'right')]))
    print(out)


if __name__ == '__main__':
    main()

# finds the led spots in a window of one single camera clip: raw max and pixels at 255 per bayer site, and saves a 4x crop
#   led_spots.py <clip> <x0> <x1> <y0> <y1> <out.png>   (E11)
import sys, numpy as np, cv2
clip, x0, x1, y0, y1, out = sys.argv[1], *map(int, sys.argv[2:6]), sys.argv[6]
W, H = 1920, 1200
raw = np.memmap(clip + '/frames.raw', np.uint8, 'r').reshape(-1, H, W)
f = np.asarray(raw[150])
sub = f[y0:y1, x0:x1]
bgr = cv2.cvtColor(np.ascontiguousarray(f), cv2.COLOR_BayerBG2BGR)[y0:y1, x0:x1]
cv2.imwrite(out, cv2.resize(bgr, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
ys, xs = np.mgrid[y0:y1, x0:x1]
site = np.where((ys % 2 == 0) & (xs % 2 == 0), 'R', np.where((ys % 2 == 1) & (xs % 2 == 1), 'B', 'G'))
m = cv2.GaussianBlur(sub, (0, 0), 1.5) > np.median(sub) + 25
n, lab, st, cen = cv2.connectedComponentsWithStats(m.astype(np.uint8))
for i in np.argsort(-st[:, 4])[1:6]:
    k = lab == i
    hue = cv2.cvtColor(bgr[k].reshape(-1, 1, 3), cv2.COLOR_BGR2HSV)[:, 0, :]
    print(f'spot ({cen[i][0]+x0:.0f}, {cen[i][1]+y0:.0f}) {st[i][4]:4d} px | ' + '  '.join(
        f'{c} max {sub[k & (site == c)].max():3d} at255 {int(((sub == 255) & k & (site == c)).sum()):3d}' for c in 'RGB'))

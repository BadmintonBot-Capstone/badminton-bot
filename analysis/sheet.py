# contact sheet of every k-th frame, plus per-frame motion energy
import sys, numpy as np, cv2
from rec import load, gray2, bgr
d, k, start, stop = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
f, m = load(d)
tiles = []
for i in range(start, min(stop, len(f)), k):
    t = cv2.resize(bgr(f[i]), (480, 300), interpolation=cv2.INTER_AREA)
    t = cv2.convertScaleAbs(t, alpha=2.0)
    cv2.putText(t, str(i), (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    tiles.append(t)
while len(tiles) % 4: tiles.append(np.zeros_like(tiles[0]))
rows = [np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]
cv2.imwrite(f"{d}/sheet_{start}_{stop}_{k}.png", np.vstack(rows))

# one tile per track segment: middle frame with the tracked path drawn on it
import sys, json, numpy as np, cv2
from rec import load, bgr
d = sys.argv[1]; name = sys.argv[2] if len(sys.argv) > 2 else 'proto'
f, m = load(d)
if name.endswith('.csv'):
    # track_single output is full resolution, the json prototypes are half
    import csv
    out = [dict(i=int(r['frame']), state=r['state'], p=[float(r['x']) / 2, float(r['y']) / 2] if r['x'] else None)
           for r in csv.DictReader(open(f'{d}/{name}'))]
    name = name[:-4]
else:
    out = json.load(open(f'{d}/{name}.json'))
segs, cur = [], []
for o in out:
    if o['state'] in ('init', 'hit', 'coast'): cur.append(o)
    elif cur: segs.append(cur); cur = []
if cur: segs.append(cur)
tiles = []
for s in segs:
    mid = s[len(s) // 2]['i']
    im = cv2.convertScaleAbs(bgr(f[mid]), alpha=1.8)
    pts = [(int(2 * o['p'][0]), int(2 * o['p'][1])) for o in s]
    for a, b in zip(pts, pts[1:]): cv2.line(im, a, b, (0, 0, 255), 6)
    for o in s:
        if o['state'] == 'hit': cv2.circle(im, (int(2 * o['p'][0]), int(2 * o['p'][1])), 9, (0, 255, 0), 3)
    t = cv2.resize(im, (480, 300), interpolation=cv2.INTER_AREA)
    cv2.putText(t, f"{s[0]['i']}-{s[-1]['i']}", (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    tiles.append(t)
while len(tiles) % 4: tiles.append(np.zeros_like(tiles[0]))
rows = [np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]
for k in range(0, len(rows), 5):
    cv2.imwrite(f'{d}/{name}_segs_{k // 5}.png', np.vstack(rows[k:k + 5]))
print(len(segs), 'segments,', (len(rows) + 4) // 5, 'sheets')

# scores a track_single csv against hand checked shuttle points, and grows those labels
#   score.py arcs   <dir> <track.csv>          sheet of the reported arcs not labelled yet
#   score.py accept <dir> <track.csv> <ids..>  those arcs are the shuttle, add to truth.csv (id 7:1512 = one frame)
#   score.py reject <dir> <track.csv> <ids..>  those arcs are not, add to false.csv
#   score.py skip   <dir> <track.csv> <ids..>  can't tell by eye, add to skip.csv
#   score.py frames <dir> <track.csv> accept|reject|skip <frames..>  same, picked by frame (1368-1370 is a range)
#   score.py score  <dir> <track.csv>          numbers
import sys, csv, json, pathlib, numpy as np, cv2

R = 30          # full res px, a report this close to a labelled point is the same object
JUMP = 60       # full res px of change in step between frames that splits an arc
FLIGHT_GAP = 10 # frames without a truth point that split two flights

def read_track(path):
    return [(int(r['frame']), r['state'], float(r['x']) if r['x'] else None, float(r['y']) if r['y'] else None)
            for r in csv.DictReader(open(path))]

def read_pts(path):
    pts = {}
    if path.exists():
        for r in csv.DictReader(open(path)):
            pts.setdefault(int(r['frame']), []).append((float(r['x']), float(r['y'])))
    return pts

def write_pts(path, pts):
    with open(path, 'w') as f:
        f.write('frame,x,y\n')
        for fr in sorted(pts):
            for x, y in pts[fr]: f.write(f'{fr},{x:.1f},{y:.1f}\n')

def near(pts, fr, x, y):
    return any(np.hypot(x - a, y - b) < R for a, b in pts.get(fr, []))

def hits(track):
    return [(fr, x, y) for fr, st, x, y in track if st == 'hit']

def arcs(track):
    # consecutive hits that move smoothly, split on a gap or a jump in the step
    out, cur = [], []
    for fr, x, y in hits(track):
        if cur:
            pf, px, py = cur[-1]
            step = np.array([x - px, y - py])
            prev = np.array([px - cur[-2][1], py - cur[-2][2]]) if len(cur) > 1 else step
            if fr - pf > 5 or np.linalg.norm(step - prev * (fr - pf)) > JUMP:
                out.append(cur); cur = []
        cur.append((fr, x, y))
    if cur: out.append(cur)
    return out

def labels(d):
    return read_pts(d / 'truth.csv'), read_pts(d / 'false.csv'), read_pts(d / 'skip.csv')

def unlabelled(d, track):
    lab = labels(d)
    return [a for a in arcs(track) if not all(any(near(p, fr, x, y) for p in lab) for fr, x, y in a)]

def sheet(d, name, todo):
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    from rec import load, bgr
    f, m = load(d)
    for old in d.glob(f'arcs_{name}_*.png'): old.unlink()
    tiles = []
    for k, a in enumerate(todo):
        mid = a[len(a) // 2]
        im = cv2.convertScaleAbs(bgr(f[mid[0]]), alpha=1.8)
        for fr, x, y in a: cv2.circle(im, (int(x), int(y)), 9, (0, 255, 0), 3)
        t = [cv2.resize(im, (256, 160), interpolation=cv2.INTER_AREA)]
        cv2.putText(t[0], f'{k}: {a[0][0]}-{a[-1][0]}', (4, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        # zoom on the reported point at the start, middle and end of the arc
        for fr, x, y in (a[0], mid, a[-1]):
            c = cv2.convertScaleAbs(bgr(f[fr]), alpha=1.8)
            x0, y0 = int(np.clip(x - 60, 0, m['width'] - 120)), int(np.clip(y - 60, 0, m['height'] - 120))
            c = c[y0:y0 + 120, x0:x0 + 120].copy()
            cv2.circle(c, (int(x - x0), int(y - y0)), 14, (0, 0, 255), 1)
            c = cv2.resize(c, (160, 160), interpolation=cv2.INTER_NEAREST)
            cv2.putText(c, str(fr), (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            t.append(c)
        tiles.append(np.hstack(t))
    if not tiles: return
    while len(tiles) % 2: tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack([tiles[i], np.zeros((160, 8, 3), np.uint8), tiles[i + 1]]) for i in range(0, len(tiles), 2)]
    for k in range(0, len(rows), 8):
        cv2.imwrite(str(d / f'arcs_{name}_{k // 8}.png'), np.vstack(rows[k:k + 8]))

def main():
    cmd, d, tname = sys.argv[1], pathlib.Path(sys.argv[2]), sys.argv[3]
    track = read_track(d / tname)
    name = tname.rsplit('.', 1)[0]
    todo = unlabelled(d, track)
    if cmd == 'arcs':
        json.dump(todo, open(d / f'arcs_{name}.json', 'w'))
        for k, a in enumerate(todo): print(k, a[0][0], a[-1][0], len(a))
        sheet(d, name, todo)
    elif cmd in ('accept', 'reject', 'skip'):
        # ids refer to the last arcs listing, so the labels match what was looked at
        todo = [[tuple(p) for p in a] for a in json.load(open(d / f'arcs_{name}.json'))]
        path = d / {'accept': 'truth.csv', 'reject': 'false.csv', 'skip': 'skip.csv'}[cmd]
        # only points with no label yet, so an arc that runs into already judged frames can't relabel them
        lab = labels(d)
        todo = [[p for p in a if not any(near(l, *p) for l in lab)] for a in todo]
        pts = read_pts(path)
        # an id is a whole arc (7), one frame of it (7:1512) or a frame range (7:1512-1520)
        for tok in sys.argv[4:]:
            k, _, rng = tok.partition(':')
            lo, _, hi = rng.partition('-')
            lo = int(lo) if lo else -1; hi = int(hi) if hi else (lo if rng else 10 ** 9)
            for fr, x, y in todo[int(k)]:
                if lo <= fr <= hi: pts.setdefault(fr, []).append((x, y))
        write_pts(path, pts)
    elif cmd == 'frames':
        # label the unlabelled hits of this track by frame: frames <dir> <track.csv> accept|reject|skip 1015 1368-1370
        what = sys.argv[4]
        path = d / {'accept': 'truth.csv', 'reject': 'false.csv', 'skip': 'skip.csv'}[what]
        lab = labels(d)
        want = set()
        for tok in sys.argv[5:]:
            lo, _, hi = tok.partition('-')
            want.update(range(int(lo), int(hi or lo) + 1))
        pts = read_pts(path)
        for fr, x, y in hits(track):
            if fr in want and not any(near(l, fr, x, y) for l in lab): pts.setdefault(fr, []).append((x, y))
        write_pts(path, pts)
    elif cmd == 'score':
        truth, false, skip = labels(d)
        on = off = unk = amb = 0; on_frames, bursts, last_off = set(), 0, -99
        for fr, x, y in hits(track):
            if near(truth, fr, x, y): on += 1; on_frames.add(fr)
            elif near(skip, fr, x, y): amb += 1
            elif near(false, fr, x, y) or fr in truth:
                # a burst is a run of off frames with no more than a few frames between them
                off += 1; bursts += fr - last_off > 5; last_off = fr
            else: unk += 1
        frames = sorted(truth)
        flights, cur = [], []
        for fr in frames:
            if cur and fr - cur[-1] > FLIGHT_GAP: flights.append(cur); cur = []
            cur.append(fr)
        if cur: flights.append(cur)
        got = sum(1 for fl in flights if len(on_frames & set(fl)) >= 5)
        tot = on + off + unk + amb
        print(f'{d.name} {tname}: flights {got}/{len(flights)}, hit frames {tot}: on {on} ({100 * on / max(tot, 1):.0f}%), '
              f'off {off} in {bursts} bursts, unclear {amb}, unlabelled {unk}, truth covered {len(on_frames)}/{len(frames)}')

if __name__ == '__main__':
    main()

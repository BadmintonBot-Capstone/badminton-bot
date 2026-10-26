# red and blue LED finder on still clips of the robot's marker LEDs, ring first
#   leds.py <dir> [<dir> ...] [--red r_min,b_max] [--blue b_min,r_max] [--size min,max]
# The LED cores clip to white, so colour is looked for in the ring around them: half res cells
# straight from the mosaic (no debayer), above a brightness floor and far enough from white in r-b
# chromaticity. Red is held stricter than blue: the 10 m blue LED's ring only reached b 0.48 against
# the lit breadboard behind it. Each colour's mask is closed 3x3 and its outer contours filled, which
# takes in the white core without guessing an erosion size. Filled regions outside the size limits
# are dropped: in the lab clips the LEDs were 4-5 cells at 10 m and 18-31 at 3 m, most clutter 1-2
# cells; the maximum has to grow for closer range (area goes as 1/distance^2). Per region:
# chromaticity of the ring cells, the unweighted centroid of the filled region and the brightness
# weighted one, and whether it holds a saturated core. Saves <dir>/leds.csv, <dir>/leds.jpg (frame 0,
# regions marked: circle at the unweighted centroid, cross at the weighted one) and
# <dir>/leds_chroma.png (r-b of every bright cell in frame 0 with the thresholds and regions), and
# prints one line per region position with each centroid's jitter over the clip.
import sys, pathlib, numpy as np, cv2
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from rec import load

FLOOR, SAT = 50, 250           # cell brightness (peak channel) floor, saturated level
RED = dict(r=0.70, b=0.10)     # red ring cell: r at least, b at most
BLUE = dict(b=0.42, r=0.20)    # blue ring cell: b at least, r at most
SIZE = [3, 40]                 # filled region area in half res cells, min and max
K = np.ones((3, 3), np.uint8)

def rgb2(f):
    # half res colour straight from the BayerRG mosaic: R top left, G averaged, B bottom right
    f = f.astype(np.float32)
    return np.dstack([f[0::2, 0::2], 0.5 * (f[0::2, 1::2] + f[1::2, 0::2]), f[1::2, 1::2]])

def cells(frame):
    c = rgb2(frame)
    ch = c / (c.sum(2, keepdims=True) + 1e-6)   # r, g, b chromaticity
    bright = c.max(2) > FLOOR                   # brightness is the cell's peak channel
    red = bright & (ch[..., 0] >= RED['r']) & (ch[..., 2] <= RED['b'])
    blue = bright & (ch[..., 2] >= BLUE['b']) & (ch[..., 0] <= BLUE['r'])
    return c, ch, bright, red, blue

def regions(c, mask, colour):
    closed = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, K)
    cs, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    out = []
    for ct in cs:
        x, y, w, h = cv2.boundingRect(ct)
        fill = np.zeros((h, w), np.uint8)
        cv2.drawContours(fill, [ct - (x, y)], -1, 1, -1)
        if not SIZE[0] <= fill.sum() <= SIZE[1]: continue
        mo = cv2.moments(fill, True)
        fill = fill.astype(bool)
        win = c[y:y + h, x:x + w]
        ring = mask[y:y + h, x:x + w] & fill
        R, G, B = win[ring].sum(0)
        S = R + G + B + 1e-6
        wgt = win.sum(2) * fill
        yy, xx = np.mgrid[0:h, 0:w]
        # cell i covers full res pixels 2i and 2i + 1
        out.append(dict(colour=colour, x_unw=2 * (x + mo['m10'] / mo['m00']) + 0.5,
                        y_unw=2 * (y + mo['m01'] / mo['m00']) + 0.5,
                        x_wtd=2 * (x + (wgt * xx).sum() / wgt.sum()) + 0.5,
                        y_wtd=2 * (y + (wgt * yy).sum() / wgt.sum()) + 0.5,
                        area=int(fill.sum()), ring=int(ring.sum()), r=R / S, g=G / S, b=B / S,
                        white=float((win[fill].min(1) >= SAT).mean()), core=int(win[fill].max() >= SAT)))
    return out

def group(rows, dist=8):
    # still clips: gather each frame's regions by colour and position, first come first served
    groups = []
    for r in rows:
        for g in groups:
            if g[0]['colour'] == r['colour'] and abs(g[0]['x_unw'] - r['x_unw']) < dist \
                    and abs(g[0]['y_unw'] - r['y_unw']) < dist:
                g.append(r); break
        else:
            groups.append([r])
    return groups

args = sys.argv[1:]
for opt, th, k in (('--red', RED, ('r', 'b')), ('--blue', BLUE, ('b', 'r'))):
    if opt in args:
        i = args.index(opt); th[k[0]], th[k[1]] = map(float, args[i + 1].split(',')); del args[i:i + 2]
if '--size' in args:
    i = args.index('--size'); SIZE = list(map(int, args[i + 1].split(','))); del args[i:i + 2]

keys = ['frame', 'colour', 'x_unw', 'y_unw', 'x_wtd', 'y_wtd', 'area', 'ring', 'r', 'g', 'b', 'white', 'core']
for d in map(pathlib.Path, args):
    f, m = load(d)
    rows = []
    for i in range(len(f)):
        c, ch, bright, red, blue = cells(f[i])
        if i == 0: ch0, bright0 = ch, bright
        for b in regions(c, red, 'red') + regions(c, blue, 'blue'):
            rows.append(dict(frame=i, **b))
    with open(d / 'leds.csv', 'w') as fh:
        fh.write(','.join(keys) + '\n')
        for r in rows:
            fh.write(','.join(f"{r[k]:.6g}" if isinstance(r[k], float) else str(r[k]) for k in keys) + '\n')

    print(f"\n{d.name}: {len(f)} frames, exposure {m['exposure_us']} us, gain {m['gain_db']:.1f} dB, "
          f"red r>={RED['r']} b<={RED['b']}, blue b>={BLUE['b']} r<={BLUE['r']}, size {SIZE[0]}-{SIZE[1]} cells")
    print("colour      x      y  frames  area  ring     r     g     b  white  core  jitter unweighted/weighted px")
    img = cv2.cvtColor(np.ascontiguousarray(f[0]), cv2.COLOR_BayerBG2BGR)
    groups = sorted(group(rows), key=lambda g: -len(g))
    for g in groups:
        med = {k: float(np.median([r[k] for r in g])) for k in keys[2:]}
        jit = lambda a, b: float(np.hypot(np.std([r[a] for r in g]), np.std([r[b] for r in g])))
        print(f"{g[0]['colour']:5s} {med['x_unw']:6.0f} {med['y_unw']:6.0f} {len(g):7d} {med['area']:5.0f} {med['ring']:5.0f}"
              f" {med['r']:5.2f} {med['g']:5.2f} {med['b']:5.2f} {med['white']:6.2f} {med['core']:5.0f}"
              f"  {jit('x_unw', 'y_unw'):5.2f} / {jit('x_wtd', 'y_wtd'):5.2f}")
        if len(g) < len(f) / 2: continue   # only mark regions seen in most frames
        col = (0, 0, 255) if g[0]['colour'] == 'red' else (255, 128, 0)
        cv2.circle(img, (int(med['x_unw']), int(med['y_unw'])), 20, col, 2)
        cv2.drawMarker(img, (int(med['x_wtd']), int(med['y_wtd'])), (255, 255, 255), cv2.MARKER_CROSS, 12, 1)
        cv2.putText(img, f"r{med['r']:.2f} b{med['b']:.2f}", (int(med['x_unw']) + 24, int(med['y_unw']) + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
    cv2.imwrite(str(d / 'leds.jpg'), img)

    fig, ax = plt.subplots(figsize=(7, 6))
    ax.hist2d(ch0[..., 0][bright0], ch0[..., 2][bright0], bins=120, range=[[0, 1], [0, 1]], cmap='Greys', cmin=1,
              norm=matplotlib.colors.LogNorm())
    ax.add_patch(plt.Rectangle((RED['r'], 0), 1 - RED['r'], RED['b'], fill=False, ec='red', ls='--'))
    ax.add_patch(plt.Rectangle((0, BLUE['b']), BLUE['r'], 1 - BLUE['b'], fill=False, ec='blue', ls='--'))
    ax.plot(1 / 3, 1 / 3, 'k+', ms=12)
    for g in groups:
        if len(g) < len(f) / 2: continue
        ax.plot(np.median([r['r'] for r in g]), np.median([r['b'] for r in g]), 'o', mfc='none',
                mec=g[0]['colour'], ms=4 + 8 * min(1, np.median([r['ring'] for r in g]) / 20))
    ax.plot([0, 1], [1, 0], 'k:', lw=0.8)
    ax.set(xlim=(0, 1), ylim=(0, 1), xlabel='r = R/(R+G+B)', ylabel='b = B/(R+G+B)',
           title=f"{d.name}\nbright cells in frame 0 (grey, log), regions seen in half the frames (circles)")
    fig.tight_layout(); fig.savefig(d / 'leds_chroma.png', dpi=110); plt.close(fig)

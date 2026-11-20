# E12: how high the E04 flights went and how much frame was left above them, then a side view of which part
# of a clear and a lift a rig sees for each height, tilt and side of the court
#   rig_placement.py
# part 1 reads the E04 flight fits (flightfit.npy). part 2 flies two shots with the E04 drag fit, a = g - alpha
# |v| v with alpha 0.208 per m, down the court's centre line, and checks each point against the rectified
# view's vertical field (P1 in config/calibration.yaml). only the vertical field is checked in part 2.
import pathlib, numpy as np
import flightfit as ff

REC = pathlib.Path(__file__).resolve().parents[2] / 'recordings' / 'E04_settings_sweep'
c = ff.load_cal()

print('part 1: E04 flights, apex and the frame left above it')
d = np.load(REC / 'flightfit.npy', allow_pickle=True).item()
for rng in ('11', '35'):
    v = d[rng]
    gx, gz, alpha = v['theta']
    g = ff.gdir(gx, gz)
    gu = g / np.linalg.norm(g)
    R = []
    for f, q in zip(v['flights'], v['starts']):
        steps = int(np.ceil(f['t'].max() / ff.H)) + 1
        tr = ff.simulate(q[None, :3], q[None, 3:6], g, alpha, steps)[:, 0]
        h = -tr @ gu  # height above the left camera
        k = np.argmax(h)
        img = ff.project(tr, c)
        R.append((h[k], tr[k, 2], img[k, 1], img[k, 1] * tr[k, 2] / c['f'], (img[:, 1] < 0).any()))
    R = np.array(R)
    print(f'{rng} ft, {len(R)} flights (median, 10th, 90th percentile)')
    for name, col in [('apex above the cameras m', 0), ('apex distance m', 1), ('apex image row px, 0 is the top', 2),
                      ('frame left above the apex m', 3)]:
        x = R[:, col]
        print(f'  {name:32s} {np.median(x):7.2f} {np.percentile(x, 10):7.2f} {np.percentile(x, 90):7.2f}')
    print(f'  flights whose fitted path leaves the top of the frame: {int(R[:, 4].sum())}')

G, A, DT = 9.81, 0.208, 0.001
F, CY, HPX, B = c['f'], c['cy'], 1200, c['B']
UP, DN = np.degrees(np.arctan(CY / F)), np.degrees(np.arctan((HPX - CY) / F))


def fly(v0, ang, h0, x0):
    # one shot toward the robot (decreasing x) until it reaches the floor: rows of time, x, height
    p, v, t, out = np.array([x0, h0]), v0 * np.array([-np.cos(ang), np.sin(ang)]), 0.0, []
    while p[1] > 0:
        out.append((t, *p))
        v = v + (np.array([0, -G]) - A * np.linalg.norm(v) * v) * DT
        p, t = p + v * DT, t + DT
    return np.array(out)


def seen(xs, hs, rx, H, tilt):
    # inside the vertical field of a rig at x rx, height H, tilted down by tilt degrees
    el = np.degrees(np.arctan2(hs - H, np.abs(xs - rx))) + tilt
    return (el <= UP) & (el >= -DN)


# court x in m: robot's baseline 0, net 6.7, opponent's baseline 13.4
clear = fly(43.1, np.radians(40), 2.6, 12.4)  # from 1 m inside the opponent's baseline, lands 11.4 m away
lift = fly(17.1, np.radians(50), 0.8, 7.7)  # from 1 m behind the net on their side, lands 6.5 m away
print(f'\npart 2: view {UP:.2f} deg above the axis and {DN:.2f} below')
for name, s in (('clear', clear), ('lift', lift)):
    k = np.argmax(s[:, 2])
    print(f'{name}: flight {s[-1, 0]:.2f} s, apex {s[k, 2]:.2f} m at {s[k, 0]:.2f} s, {s[k, 1]:.2f} m from the robot baseline')
print('side      height tilt  floor seen from | clear seen, first seen, its distance, depth noise | lift seen | LED at 0.4 m seen over')
xs = np.linspace(0.5, 4, 351)  # the robot's working zone, x from its baseline
for side, rx in (('robot', -1.0), ('opponent', 14.4)):
    for H in (1, 2, 3, 4):
        for tilt in (-10, 0, 10, 20):
            fl = H / np.tan(np.radians(tilt + DN)) if tilt + DN > 0 else np.inf
            cs, ls = seen(clear[:, 1], clear[:, 2], rx, H, tilt), seen(lift[:, 1], lift[:, 2], rx, H, tilt)
            first = clear[cs, 0].min() if cs.any() else np.nan
            z = abs(clear[cs, 1][0] - rx) if cs.any() else np.nan
            ok = xs[seen(xs, 0.4, rx, H, tilt)]
            led = f'{ok.min():.1f} to {ok.max():.1f} m' if ok.size else 'never'
            print(f'{side:8s} {H:4d} m {tilt:4d} {fl:9.1f} m     | {cs.sum() * DT:5.2f} s  {first:5.2f} s  {z:5.1f} m '
                  f'{z * z * 0.2 / (F * B) * 100:5.1f} cm | {ls.sum() * DT:5.2f} s | {led}')


def spans(t, m):
    # the time intervals where m is true, as text
    out, start, prev = [], None, None
    for ti, mi in zip(t, m):
        if mi and start is None: start = ti
        if not mi and start is not None: out.append((start, prev)); start = None
        prev = ti
    if start is not None: out.append((start, prev))
    return ' + '.join(f'{a:.2f}-{b:.2f}' for a, b in out) or 'none'


# part 3, his question 2026-10-15: keep the rig low and tilt it up
print('\npart 3: low rigs tilted up, the times each shot is in view and where LEDs at 0.4 and 1.5 m are seen')
for side, rx in (('robot', -1.0), ('opponent', 14.4)):
    for H in (0.3, 0.5, 1.0):
        for tilt in (0, -10, -20, -30, -40):
            leds = []
            for lh in (0.4, 1.5):
                ok = xs[seen(xs, lh, rx, H, tilt)]
                leds.append(f'{ok.min():.1f}-{ok.max():.1f} m' if ok.size else 'never')
            print(f'{side:8s} {H:3.1f} m {tilt:4d} | clear {spans(clear[:, 0], seen(clear[:, 1], clear[:, 2], rx, H, tilt)):24s} '
                  f'| lift {spans(lift[:, 0], seen(lift[:, 1], lift[:, 2], rx, H, tilt)):22s} | LED 0.4 m {leds[0]:11s} LED 1.5 m {leds[1]}')


# part 4: the Dec 1 gate is tosses from about 4 m, so check those from the robot's side. the robot stands 2 m in
# from its baseline and the tosser 4 m further out, tossing underhand from 1 m to land at the robot
print('\npart 4: tosses from 4 m, rig on the robot side')
for ang in (40, 55, 70):
    lo, hi = 1, 60
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if fly(mid, np.radians(ang), 1.0, 6.0)[-1, 1] > 2.0 else (lo, mid)
    s = fly(mid, np.radians(ang), 1.0, 6.0)
    k = np.argmax(s[:, 2])
    print(f'toss {ang} deg: {mid:.1f} m/s, apex {s[k, 2]:.2f} m, flight {s[-1, 0]:.2f} s')
    for back in (1, 2, 3):
        for H, tilt in ((1, 0), (1, -5), (1, -10), (1.5, 0), (1.5, -5)):
            m = seen(s[:, 1], s[:, 2], -back, H, tilt)
            ok = xs[seen(xs, 0.4, -back, H, tilt)]
            led = f'{ok.min():.1f}-{ok.max():.1f} m' if ok.size else 'never'
            print(f'   rig {back} m back, {H} m high, tilt {tilt:3d}: toss in view {m.mean() * 100:5.1f}%  LED 0.4 m seen {led}')

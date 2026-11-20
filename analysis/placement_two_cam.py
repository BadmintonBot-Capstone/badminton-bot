# stage 2 of the camera placement: a stereo pair centred near the best one camera placement
#   placement_two_cam.py [--every 20]
#
# both cameras portrait, sharing pitch, yaw mirrored by toe-in gamma (camera 1 turns +gamma, camera 2 -gamma).
# camera 2 sits b from camera 1 along (cos e cos l, sin e, cos e sin l), the midpoint at x = 1.525. a placement
# passes when b is at most 1 m (cable reach), both cameras are at least 0.5 m up, inside the 2.5 m behind the baseline, and both see the whole
# home zone. it scores the average, over the home-zone shots, of the fraction of each shot's frames both cameras
# see. the tie breaker is depth error: for each frame both see, the triangulation error from 1 px of noise in each
# camera, d / (f sin a) with d the mean distance and a the angle between the two rays; its median over those frames.
import argparse, itertools, pathlib, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

TH, TV = np.tan(np.radians(44.9 / 2)), np.tan(np.radians(29.0 / 2))
F_PX = 960 / TH
HOME, R_HOME = np.array([1.525, 0.0, 3.35]), 1.0
X0, Z0, MIN_H = 1.525, 12.40, 0.5
HEIGHT = np.round(np.arange(0.5, 1.501, 0.1), 2)
BACK = (1.5, 1.75, 2.0, 2.25, 2.5)
PITCH = np.arange(4, 20.001, 2.0)
B = np.round(np.arange(0.2, 1.001, 0.1), 2)
LAM = (0, 45, 90, -45, -90)
EPS = (0, 45, 90, -45)
GAM = (0, 2, 4, 6, 8, 10, 15)
REC = pathlib.Path(__file__).resolve().parents[2] / 'recordings' / 'placement'

ap = argparse.ArgumentParser()
ap.add_argument('--every', type=int, default=20)
every = ap.parse_args().every

d = np.load(REC / 'trajectory_set.npz')
psi = np.radians(d['psi'])
shots = []
for name in ('P1', 'P2'):
    ok, sf, kept, path = d[f'{name}_ok'], d[f'{name}_sf'], d[f'{name}_kept'], d[f'{name}_path']
    row = {f: k for k, f in enumerate(kept)}
    for f, j in zip(*np.nonzero(ok)):
        lx, lz = X0 + sf[f] * np.sin(psi[j]), Z0 - sf[f] * np.cos(psi[j])
        if np.hypot(lx - HOME[0], lz - HOME[2]) <= R_HOME:
            sh = path[row[f]]
            sh = sh[np.isfinite(sh[:, 0])]
            shots.append(np.stack([X0 + sh[:, 0] * np.sin(psi[j]), sh[:, 1], Z0 - sh[:, 0] * np.cos(psi[j])], 1))
shots = shots[::every]
pts = np.concatenate(shots)
owner = np.repeat(np.arange(len(shots)), [len(s) for s in shots])
frames = np.bincount(owner)
g = np.arange(-R_HOME, R_HOME + 1e-9, 0.05)
gx, gz = np.meshgrid(g, g)
inside = np.hypot(gx, gz) <= R_HOME
home = np.stack([HOME[0] + gx[inside], np.zeros(inside.sum()), HOME[2] + gz[inside]], 1)


def axes(yaw, pitch):
    a, b = np.radians(yaw), np.radians(pitch)
    f = np.array([np.cos(b) * np.sin(a), np.sin(b), np.cos(b) * np.cos(a)])
    r = np.array([-np.cos(a), 0, np.sin(a)])
    return f, r, np.cross(r, f)


def in_view(p, c, ax):
    f, r, u = ax
    q = p - c
    x, y, z = q @ r, q @ u, q @ f
    return (z > 0.05) & (np.abs(x) <= TV * z) & (np.abs(y) <= TH * z)


rows = []
for y, back, pitch, b, lam, eps, gam in itertools.product(HEIGHT, BACK, PITCH, B, LAM, EPS, GAM):
    if eps in (90, -90) and lam != 0:
        continue
    l, e = np.radians(lam), np.radians(eps)
    dvec = np.array([np.cos(e) * np.cos(l), np.sin(e), np.cos(e) * np.sin(l)])
    m = np.array([X0, y, -back])
    c1, c2 = m - dvec * b / 2, m + dvec * b / 2
    if min(c1[1], c2[1]) < MIN_H or max(c1[2], c2[2]) > 0 or min(c1[2], c2[2]) < -2.5:
        continue
    a1, a2 = axes(gam, pitch), axes(-gam, pitch)
    if not (in_view(home, c1, a1).all() and in_view(home, c2, a2).all()):
        continue
    both = in_view(pts, c1, a1) & in_view(pts, c2, a2)
    score = (np.bincount(owner, both, len(shots)) / frames).mean()
    v1, v2 = pts[both] - c1, pts[both] - c2
    n1, n2 = np.linalg.norm(v1, axis=1), np.linalg.norm(v2, axis=1)
    sin_a = np.linalg.norm(np.cross(v1, v2), axis=1) / (n1 * n2)
    err = np.median((n1 + n2) / 2 / (F_PX * sin_a)) if both.any() else np.inf
    rows.append((score, err, y, back, pitch, b, lam, eps, gam))
rows = np.array(rows)
print(f'{len(shots)} shots, {len(pts)} points; {len(rows)} placements pass')
np.save(REC / 'placement_two_cam.npy', rows)

hdr = 'score  depth err  height  back  pitch     B   lambda  eps  gamma'
fmt = lambda r: f'{r[0]:.3f}  {1000 * r[1]:6.1f} mm  {r[2]:.2f}  {r[3]:.2f}  {r[4]:+3.0f}  {r[5]:.2f}  {r[6]:+4.0f}  {r[7]:+4.0f}  {r[8]:3.0f}'
print('best by score\n' + hdr)
for r in rows[np.argsort(-rows[:, 0])][:8]:
    print(fmt(r))
near = rows[rows[:, 0] >= rows[:, 0].max() - 0.01]
print(f'within 0.01 of the best score ({len(near)}), sharpest depth first\n' + hdr)
for r in near[np.argsort(near[:, 1])][:8]:
    print(fmt(r))

print('best at each midpoint height (highest score, then sharpest depth within 0.001 of it)\n' + hdr)
for y in HEIGHT:
    r = rows[np.isclose(rows[:, 2], y)]
    if len(r):
        r = r[r[:, 0] >= r[:, 0].max() - 0.001]
        print(fmt(r[np.argmin(r[:, 1])]))

fig, ax = plt.subplots(figsize=(7.5, 5.5), constrained_layout=True)
sc = ax.scatter(1000 * rows[:, 1], rows[:, 0], c=rows[:, 5], s=8, cmap='viridis')
ax.set_xlabel('median depth error from 1 px of noise (mm)', fontsize=12)
ax.set_ylabel('average fraction of each shot both cameras see', fontsize=12)
fig.colorbar(sc, ax=ax).set_label('baseline B (m)', fontsize=12)
fig.savefig(REC / 'placement_two_cam.png', dpi=150)
print('saved', REC / 'placement_two_cam.png')

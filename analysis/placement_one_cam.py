# stage 1 of the camera placement: one camera at x = 1.525 behind the robot's baseline, facing straight down the court
#   placement_one_cam.py [--every 5]
#
# reads trajectory_set.npz. heights start at 0.5 m so the leds on the robot stay apart in the picture. a placement
# passes when the whole home zone (the floor within 1 m of p_home) is in view, and scores the average, over the shots
# landing in the home zone, of the fraction of each shot's frames in view. pinhole camera, 44.9 by 29.0 degrees, no distortion. --every keeps one shot in n.
import argparse, pathlib, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

TH, TV = np.tan(np.radians(44.9 / 2)), np.tan(np.radians(29.0 / 2))
HOME, R_HOME = np.array([1.525, 0.0, 3.35]), 1.0
X0, Z0 = 1.525, 12.40
F_PX = 960 / TH
HEIGHT = np.round(np.arange(0.5, 5.001, 0.1), 2)
BACK = np.arange(0, 2.501, 0.25)
PITCH = np.arange(-20, 70.001, 2.0)
ROLL = ('landscape', 'portrait')
REC = pathlib.Path(__file__).resolve().parents[2] / 'recordings' / 'placement'

ap = argparse.ArgumentParser()
ap.add_argument('--every', type=int, default=5)
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


def cam(p, c, pitch):
    b = np.radians(pitch)
    f, r, u = np.array([0, np.sin(b), np.cos(b)]), np.array([-1.0, 0, 0]), np.array([0, np.cos(b), -np.sin(b)])
    q = p - c
    return q @ r, q @ u, q @ f


def in_view(p, c, pitch, portrait):
    x, y, z = cam(p, c, pitch)
    th, tv = (TV, TH) if portrait else (TH, TV)
    return (z > 0.05) & (np.abs(x) <= th * z) & (np.abs(y) <= tv * z)


score = np.full((len(ROLL), HEIGHT.size, BACK.size, PITCH.size), np.nan)
for a, portrait in enumerate((False, True)):
    for i, y in enumerate(HEIGHT):
        for k, back in enumerate(BACK):
            c = np.array([X0, y, -back])
            for m, pitch in enumerate(PITCH):
                if in_view(home, c, pitch, portrait).all():
                    seen = np.bincount(owner, in_view(pts, c, pitch, portrait), len(shots))
                    score[a, i, k, m] = (seen / frames).mean()
print(f'{len(shots)} shots, {len(pts)} points; {np.isfinite(score).sum()} of {score.size} placements pass')
for a in range(len(ROLL)):
    ok = np.isfinite(score[a]).any(2)
    low = [f'{b:.2f} m back: {HEIGHT[ok[:, k]].min():.1f} m' if ok[:, k].any() else f'{b:.2f} m back: none' for k, b in enumerate(BACK)]
    print(f'{ROLL[a]} lowest passing height, ' + '; '.join(low))

best = np.argsort(np.nan_to_num(score, nan=-1).ravel())[::-1][:10]
for n in best:
    a, i, k, m = np.unravel_index(n, score.shape)
    print(f'{ROLL[a]:9s} height {HEIGHT[i]:.2f} m, {BACK[k]:.2f} m behind the baseline, pitch {PITCH[m]:+.0f}: {score[a, i, k, m]:.3f}')
np.savez_compressed(REC / 'placement_one_cam.npz', score=score, height=HEIGHT, back=BACK, pitch=PITCH, roll=np.array(ROLL))

# heatmap for portrait: the best score over pitch for each height and distance, the best cell starred
s = score[ROLL.index('portrait')]
bs = np.where(np.isfinite(s).any(2), np.nanmax(np.where(np.isfinite(s), s, -1), 2), np.nan)
fig, ax = plt.subplots(figsize=(7, 5.5), constrained_layout=True)
im = ax.imshow(bs, origin='lower', aspect='auto', vmin=0, vmax=1, cmap='viridis',
               extent=[BACK[0] - 0.125, BACK[-1] + 0.125, HEIGHT[0] - 0.05, HEIGHT[-1] + 0.05])
i, k = np.unravel_index(np.nanargmax(bs), bs.shape)
ax.plot(BACK[k], HEIGHT[i], marker='*', ms=16, mec='k', mfc='w')
ax.set_ylim(0.45, 4.5)
ax.set_xlabel("distance behind the robot's baseline (m)", fontsize=12)
ax.set_ylabel('camera height (m)', fontsize=12)
ax.set_title('portrait', fontsize=13)
fig.colorbar(im, ax=ax).set_label('average fraction of each shot in view', fontsize=12)
fig.savefig(REC / 'placement_one_cam.png', dpi=150)
print('saved', REC / 'placement_one_cam.png')

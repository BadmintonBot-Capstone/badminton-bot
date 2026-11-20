# the set of shots the camera placement is scored against, as in the 2026-11-17 placement logbook entry
#   trajectory_set.py
#
# every (v0, theta) is flown once per start height in the shot's own plane (s along the floor, h up) with
# a = g - alpha |v| v, rk4 at 1 ms. each flight keeps s_f, h_p, its heights from s = 5.70 to 5.90 m and its
# path at 100 fps (saved for flights valid at some heading). each heading psi is then a check on those numbers,
# with no new flights.
# court frame: x across, y up, z toward the net; start at x 1.525, z 12.40.
import pathlib, numpy as np

ALPHA, G, DT = 0.215, 9.81, 0.001
X0, Z0, NET_Z, NET_H, STRIP = 1.525, 12.40, 6.70, 1.55, 3.05
H_MAX = 7.0
STARTS = {'P1': 2.6, 'P2': 1.1}
V0 = np.arange(5, 60.001, 0.5)
THETA = np.arange(-30, 85.001, 1.0)
PSI = np.arange(-15, 15.001, 1.0)
S_NET = np.round(np.arange(5.70, 5.9001, 0.01), 2)
OUT = pathlib.Path(__file__).resolve().parents[2] / 'recordings' / 'placement' / 'trajectory_set.npz'


def fly(v0, th, h0):
    n = v0.size
    s, h = np.zeros(n), np.full(n, h0)
    u, w = v0 * np.cos(th), v0 * np.sin(th)
    sf, hp = np.full(n, np.nan), np.full(n, h0)
    hnet = np.full((n, S_NET.size), np.nan)
    path = [np.stack([s, h], 1)]
    alive = np.ones(n, bool)

    def d(u, w):
        sp = np.hypot(u, w)
        return -ALPHA * sp * u, -G - ALPHA * sp * w

    step = 0
    while alive.any() and step < 8000:
        a1, b1 = d(u, w)
        a2, b2 = d(u + DT / 2 * a1, w + DT / 2 * b1)
        a3, b3 = d(u + DT / 2 * a2, w + DT / 2 * b2)
        a4, b4 = d(u + DT * a3, w + DT * b3)
        s2 = s + DT / 6 * (u + 2 * (u + DT / 2 * a1) + 2 * (u + DT / 2 * a2) + (u + DT * a3))
        h2 = h + DT / 6 * (w + 2 * (w + DT / 2 * b1) + 2 * (w + DT / 2 * b2) + (w + DT * b3))
        u, w = u + DT / 6 * (a1 + 2 * a2 + 2 * a3 + a4), w + DT / 6 * (b1 + 2 * b2 + 2 * b3 + b4)
        for k, g in enumerate(S_NET):
            m = alive & (s < g) & (s2 >= g)
            hnet[m, k] = h[m] + (g - s[m]) / (s2[m] - s[m]) * (h2[m] - h[m])
        land = alive & (h2 <= 0)
        sf[land] = s[land] + h[land] / (h[land] - h2[land]) * (s2[land] - s[land])
        hp = np.where(alive, np.maximum(hp, h2), hp)
        alive &= ~land
        s, h = np.where(alive, s2, s), np.where(alive, h2, h)
        step += 1
        if step % 10 == 0:
            path.append(np.where(alive[:, None], np.stack([s, h], 1), np.nan))
    return sf, hp, hnet, np.stack(path, 1)


def headings(sf, hp, hnet):
    # one row per flight, one column per psi: true when that shot is valid
    p = np.radians(PSI)[None, :]
    k = np.round((Z0 - NET_Z) / np.cos(p) - 5.70, 2) / 0.01
    clears = hnet[:, np.round(k[0]).astype(int)] >= NET_H
    x = X0 + sf[:, None] * np.sin(p)
    z = Z0 - sf[:, None] * np.cos(p)
    lands = (x >= 0) & (x <= STRIP) & (z >= 0) & (z <= NET_Z)
    return (hp[:, None] <= H_MAX) & clears & lands


v, t = np.meshgrid(V0, np.radians(THETA), indexing='ij')
v, t = v.ravel(), t.ravel()
out = {'v0': v, 'theta': np.degrees(t), 'psi': PSI, 's_net': S_NET}
for name, h0 in STARTS.items():
    sf, hp, hnet, path = fly(v, t, h0)
    ok = headings(sf, hp, hnet)
    keep = ok.any(1)
    path = path[keep][:, :np.isfinite(path[keep, :, 0]).any(0).sum()]
    out |= {f'{name}_sf': sf, f'{name}_hp': hp, f'{name}_hnet': hnet, f'{name}_ok': ok,
            f'{name}_kept': np.flatnonzero(keep), f'{name}_path': path.astype(np.float32)}
    print(f'{name} from {h0} m: {v.size} flights, {keep.sum()} valid at some heading, {ok.sum()} valid (flight, heading) pairs')
OUT.parent.mkdir(parents=True, exist_ok=True)
np.savez_compressed(OUT, **out)
print('saved', OUT)

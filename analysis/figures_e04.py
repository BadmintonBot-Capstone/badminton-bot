# draws the E04 result figures from what cands.py, sweep.py and flightfit.py saved
#   figures_e04.py [recordings/E04_settings_sweep] [experiments/figures/E04]
#
# contrast.png              how far the flying shuttle stands out from the background, per setting
# detector_vs_tracker.png   flying frames the detector finds against those each tracker reports
# tracker_tradeoff.png      3D tracker: flying frames found live against false reports per minute
# found_vs_junk.png         flying frames found on held out flights against false candidates per frame
# flights_35ft.png          one 35 ft clip's flights in 3D with the fitted drag plus gravity curves
# alpha_per_flight.png      alpha fitted flight by flight, both ranges
# scatter_3d.png            how far each setting's 3D points sit from their fitted flight
import sys, csv, pathlib, numpy as np, yaml, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sweep, flightfit, track3d_sweep

HERE = pathlib.Path(__file__).resolve().parent
REC = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else HERE.parent.parent / 'recordings' / 'E04_settings_sweep'
OUT = pathlib.Path(sys.argv[2]) if len(sys.argv) > 2 else HERE.parent / 'experiments' / 'figures' / 'E04'
SETTINGS = ['baseline', 'short_exposure', 'shorter_exposure', 'higher_frame_rate']
NAMES = {'baseline': '8.33 ms, 6 dB\n(baseline)', 'short_exposure': '4 ms, 12.3 dB',
         'shorter_exposure': '2 ms, 18.3 dB', 'higher_frame_rate': '4 ms, 12.3 dB\n150 fps'}
COL = {'baseline': '#4c72b0', 'short_exposure': '#dd8452', 'shorter_exposure': '#55a868',
       'higher_frame_rate': '#c44e52'}
plt.rcParams.update({'font.size': 11, 'axes.spines.top': False, 'axes.spines.right': False,
                     'figure.dpi': 150, 'savefig.bbox': 'tight'})


def clips():
    return sorted(c for c in REC.glob('ft*') if (c / 'sweep' / 'left.npz').exists())


def setting_of(c):
    return yaml.safe_load(open(c / 'meta.yaml'))['notes']


def contrast():
    rows = []
    for c in clips():
        z = np.load(c / 'sweep' / 'left.npz')
        n = setting_of(c)
        for r in csv.DictReader(open(c / 'labels' / 'left.csv')):
            f = int(r['frame'])
            if r['state'] == 'flying' and not np.isnan(z['peak'][f, 0]):
                rows.append((n['setting'], n['range_ft'], z['peak'][f, 0], z['noise'][f, 0]))
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.5), sharey=True)
    for ax, rng in zip(axs, ['11', '35']):
        data = [[p for s_, r_, p, _ in rows if s_ == s and r_ == rng] for s in SETTINGS]
        noise = [np.median([q for s_, r_, _, q in rows if s_ == s and r_ == rng]) for s in SETTINGS]
        b = ax.boxplot(data, widths=0.5, showfliers=False, patch_artist=True)
        for patch, s in zip(b['boxes'], SETTINGS): patch.set_facecolor(COL[s]); patch.set_alpha(0.6)
        for m in b['medians']: m.set_color('black')
        ax.scatter(range(1, 5), noise, marker='_', s=600, color='black', zorder=3,
                   label='typical background change (median pixel)' if rng == '11' else None)
        ax.axhline(12, ls='--', color='grey', label="tracker's threshold (12 grey levels)" if rng == '11' else None)
        ax.set_yscale('log')
        ax.set_xticks(range(1, 5), [NAMES[s] for s in SETTINGS], fontsize=9)
        ax.set_title(f'{rng} ft')
    axs[0].set_ylabel('largest change at the shuttle (grey levels)')
    fig.legend(loc='lower center', ncol=2, fontsize=9, bbox_to_anchor=(0.5, -0.08))
    fig.suptitle('Flying shuttle against the background, every labelled flying frame\n'
                 'boxes: middle half of frames, whiskers: 1.5 box lengths', fontsize=12, y=1.04)
    fig.savefig(OUT / 'contrast.png'); plt.close(fig)


def tracker_found(c, info):
    # flying frames the tracker reported on, by the sweep's rule: the label inside the reported blob's box
    # grown by 10 px (the tracker's blob found by its centroid), or within 30 px of the report
    t = np.load(c / 'sweep' / 'left.npz')['ra']
    t = t[(t[:, 1] == 12) & (t[:, 2] >= 4) & (t[:, 2] <= 900) & (t[:, 10] >= 0.3)]
    hit = np.zeros(info['n'], bool)
    for r in csv.DictReader(open(c / 'track.csv')):
        f = int(r['frame'])
        if r['state'] != 'hit' or info['st'][f] != 1: continue
        x, y = float(r['x']), float(r['y'])
        lx, ly = info['xy'][f]
        if np.hypot(x - lx, y - ly) <= 30: hit[f] = True; continue
        b = t[t[:, 0] == f]
        if not len(b): continue
        k = np.argmin(np.hypot(b[:, 3] - x, b[:, 4] - y))
        if np.hypot(b[k, 3] - x, b[k, 4] - y) < 6 and b[k, 5] - 10 <= lx <= b[k, 5] + b[k, 7] + 10 \
                and b[k, 6] - 10 <= ly <= b[k, 6] + b[k, 8] + 10:
            hit[f] = True
    fly = info['st'] == 1
    return np.bincount(info['ep'][fly], hit[fly], info['nflights']), np.bincount(info['ep'][fly], None, info['nflights'])


def detector_vs_tracker(z, g, budget=15):
    # detector from sweep.py, both trackers from track3d_sweep.py (3D tracker cross validated at the budget)
    R, configs = z['R'], [tuple(c) for c in z['configs']]
    cur = configs.index(sweep.CURRENT)
    test, _ = track3d_sweep.cross_validate(g['R'], g['fold'], budget)
    rng = np.random.default_rng(1)
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.8), sharey=True)
    for ax, r in zip(axs, ['11', '35']):
        for i, s in enumerate(SETTINGS):
            e = np.flatnonzero((z['setting'] == s) & (z['range'] == r))
            eg = (g['setting'] == s) & (g['range'] == r)
            bars = [(R[cur, e, 0], R[cur, e, 1], dict(alpha=1.0), 'detector: shuttle among its blobs'),
                    (g['cur'][eg, 0], g['cur'][eg, 1], dict(alpha=0.3, hatch='//', edgecolor='black'),
                     'current tracker: reported live'),
                    (test[eg, 0], test[eg, 1], dict(alpha=0.65, hatch='..', edgecolor='black'),
                     f'3D physics tracker: reported live (picked at {budget} false reports/min)')]
            for k, (fd, tot, style, lab) in enumerate(bars):
                v = 100 * fd.sum() / tot.sum()
                lo, hi = sweep.ci(fd, tot, rng)
                x = i + (k - 1) * 0.27
                ax.bar(x, v, 0.27, color=COL[s], yerr=[[v - 100 * lo], [100 * hi - v]], capsize=2,
                       label=lab if i == 0 and r == '11' else None, **style)
                ax.text(x, 2, f'{v:.0f}', ha='center', fontsize=8, color='white' if k == 0 else 'black')
        ax.set_xticks(range(4), [NAMES[s] for s in SETTINGS], fontsize=9)
        ax.set_ylim(0, 100)
        ax.set_title(f'{r} ft')
    axs[0].set_ylabel('flying frames (%)')
    fig.legend(loc='lower center', ncol=3, fontsize=9, bbox_to_anchor=(0.5, -0.08))
    fig.suptitle('Detector and trackers on every labelled flying frame\n'
                 'error bars: 95% range from resampling flights; 3D tracker scored on flights it was not tuned on',
                 fontsize=12, y=1.04)
    fig.savefig(OUT / 'detector_vs_tracker.png'); plt.close(fig)


def tracker_tradeoff(g):
    R, cur = g['R'], g['cur']
    t = R.sum(1)
    fig, ax = plt.subplots(figsize=(7.5, 5))
    ax.scatter(t[:, 3] / t[:, 4], 100 * t[:, 0] / t[:, 1], s=14, color='grey', alpha=0.6,
               label='every 3D tracker setting tried (scored on all flights)')
    for b, col in [(5, '#55a868'), (15, '#4c72b0'), (30, '#dd8452')]:
        test, _ = track3d_sweep.cross_validate(R, g['fold'], b)
        q = test.sum(0)
        ax.scatter(q[3] / q[4], 100 * q[0] / q[1], s=70, color=col, zorder=3,
                   label=f'picked for at most {b} false/min, scored on held out flights')
    c = cur.sum(0)
    ax.scatter(c[3] / c[4], 100 * c[0] / c[1], marker='*', s=220, color='black', zorder=4, label='current tracker')
    ax.set_xscale('log')
    ax.set_xlabel('false reports per minute')
    ax.set_ylabel('flying frames reported live (%)')
    ax.set_ylim(0, 70)
    ax.grid(alpha=0.3, which='both')
    ax.legend(fontsize=8.5, loc='lower right')
    ax.set_title('3D physics tracker against the current tracker, all clips', fontsize=12)
    fig.savefig(OUT / 'tracker_tradeoff.png'); plt.close(fig)


def found_vs_junk(z):
    R, configs = z['R'], [tuple(c) for c in z['configs']]
    setting, fold = z['setting'], z['fold']
    names = {'ra': 'running average', 'prev': 'last frame', 'tri': 'both neighbouring frames'}
    cols = {'ra': '#4c72b0', 'prev': '#dd8452', 'tri': '#55a868'}
    budgets = [0.01, 0.03, 0.1, 0.3, 1, 3]
    fig, ax = plt.subplots(figsize=(7.5, 5))
    for m in ['ra', 'prev', 'tri']:
        for tol, ls in [(0, '-'), (4, '--')]:
            allowed = np.array([k for k, c in enumerate(configs) if c[0] == m and c[5] == tol])
            xs, ys = [], []
            for b in budgets:
                t = np.zeros(6)
                for s in SETTINGS:
                    eps = np.flatnonzero(setting == s)
                    t += sweep.cross_validate(R, eps, fold, allowed, b)[0].sum(0)
                xs.append(t[4] / t[5]); ys.append(100 * t[0] / t[1])
            ax.plot(xs, ys, ls, marker='o', ms=4, color=cols[m],
                    label=f'{names[m]}' + (', right camera check' if tol else ''))
    cur = configs.index(sweep.CURRENT)
    t = R[cur].sum(0)
    ax.scatter(t[4] / t[5], 100 * t[0] / t[1], marker='*', s=200, color='black', zorder=4, label='current detector')
    ax.set_xscale('log')
    ax.set_xlabel('false candidates per frame (blobs that are not the shuttle)')
    ax.set_ylabel('flying frames with the shuttle among the blobs (%)')
    ax.set_title('Shuttle found against junk let through, every setting and range\n'
                 'each point: detector tuned on 4/5 of the flights, scored on the other 1/5', fontsize=12)
    ax.legend(fontsize=8.5, loc='lower right')
    ax.grid(alpha=0.3, which='both')
    fig.savefig(OUT / 'found_vs_junk.png'); plt.close(fig)


def frame_axes(g):
    # up against gravity, forward is the camera axis laid flat, side completes the right handed set
    up = -g / np.linalg.norm(g)
    fwd = np.array([0, 0, 1.0]) - up[2] * up
    fwd /= np.linalg.norm(fwd)
    return up, fwd, np.cross(up, fwd)


def flights_35ft(fit, clip='ft35_c5_e8333_g6'):
    c = flightfit.load_cal()
    b = fit['35']
    gx, gz, alpha = b['theta']
    g = flightfit.gdir(gx, gz)
    up, fwd, side = frame_axes(g)
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.8), gridspec_kw={'width_ratios': [2.2, 1]})
    cmap = plt.get_cmap('viridis')
    ids = [i for i, f in enumerate(b['flights']) if f['clip'] == clip]
    for n, i in enumerate(ids):
        f, q = b['flights'][i], b['starts'][i]
        col = cmap(n / max(len(ids) - 1, 1))
        X = flightfit.tri(f['obs'], c)
        steps = int(np.ceil(f['t'].max() / flightfit.H)) + 1
        curve = flightfit.simulate(q[None, :3], q[None, 3:], g, alpha, steps)[:, 0]
        axs[0].plot(X @ fwd, X @ up, '.', ms=3, color=col)
        axs[0].plot(curve @ fwd, curve @ up, '-', lw=1.2, color=col)
        axs[1].plot(X @ side, X @ fwd, '.', ms=3, color=col)
        axs[1].plot(curve @ side, curve @ fwd, '-', lw=1.2, color=col)
        if n == 0:
            free = flightfit.simulate(q[None, :3], q[None, 3:], g, 0.0, steps)[:, 0]
            axs[0].plot(free @ fwd, free @ up, ':', color='grey', lw=1.2, label='same launch with no drag')
    axs[0].plot([], [], 'k.', label='measured (stereo, one dot per frame)')
    axs[0].plot([], [], 'k-', label=f'fitted drag plus gravity (alpha {alpha:.3f} per m)')
    axs[0].scatter([0], [0], marker='s', color='black')
    axs[0].annotate('cameras', (0, 0), textcoords='offset points', xytext=(6, 6))
    axs[0].set_xlabel('distance from the cameras along the floor (m)')
    axs[0].set_ylabel('height relative to the cameras (m)')
    axs[0].set_title('side view')
    axs[0].legend(fontsize=9, loc='lower left')
    axs[0].set_aspect('equal', adjustable='datalim')
    axs[0].set_ylim(top=max(axs[0].get_ylim()[1], 1.2))
    axs[1].set_xlabel('sideways (m)')
    axs[1].set_ylabel('distance from the cameras (m)')
    axs[1].set_title('top view')
    axs[1].set_aspect('equal', adjustable='datalim')
    fig.suptitle(f'Racket hits toward the rig from about 35 ft ({clip}), up taken from the fitted gravity', fontsize=12)
    fig.savefig(OUT / 'flights_35ft.png'); plt.close(fig)


def alpha_per_flight(fit):
    fig, ax = plt.subplots(figsize=(8, 3.8))
    for y, (rng, lab) in enumerate([('35', '35 ft racket hits'), ('11', '11 ft tosses and hits')]):
        a = fit[rng]['alphas']
        shown = np.clip(a, 0, 0.6)
        ax.scatter(shown, y + np.random.default_rng(0).uniform(-0.15, 0.15, len(a)), s=18, alpha=0.7,
                   color='#4c72b0' if rng == '35' else '#dd8452')
        over = np.sum(a > 0.6)
        if over: ax.annotate(f'{over} above 0.6', (0.6, y), textcoords='offset points', xytext=(-60, 12), fontsize=9)
        ax.plot([fit[rng]['theta'][2]] * 2, [y - 0.3, y + 0.3], color='black', lw=2)
    ax.axvline(0.215, ls='--', color='grey', label="EKF's alpha today (0.215)")
    ax.plot([], [], color='black', lw=2, label='fitted to the whole range at once')
    ax.set_yticks([0, 1], ['35 ft racket hits', '11 ft tosses and hits'])
    ax.set_xlim(0, 0.62)
    ax.set_xlabel('alpha (per m), one dot per flight')
    ax.set_title('Drag coefficient flight by flight (terminal speed = sqrt(9.81 / alpha))', fontsize=12)
    ax.legend(fontsize=9, loc='lower right')
    fig.savefig(OUT / 'alpha_per_flight.png'); plt.close(fig)


def scatter_3d(fit):
    c = flightfit.load_cal()
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2), sharey=False)
    for ax, (k, lab) in zip(axs, [(0, 'across the view'), (1, 'along the view (depth)')]):
        for j, rng in enumerate(['11', '35']):
            b = fit[rng]
            for i, s in enumerate(SETTINGS):
                ids = [n for n, f in enumerate(b['flights']) if f['setting'] == s]
                res = np.concatenate([b['res'][n] for n in ids])
                obs = np.concatenate([b['flights'][n]['obs'] for n in ids])
                core = np.all(np.abs(res) < 15, 1)
                err = (flightfit.tri(obs, c) - flightfit.tri(obs + res, c))[core] * 1000
                v = np.median(np.hypot(err[:, 0], err[:, 1])) if k == 0 else np.median(np.abs(err[:, 2]))
                ax.bar(j * 5 + i, v, color=COL[s], label=NAMES[s].replace('\n', ' ') if j == 0 and k == 0 else None)
                ax.text(j * 5 + i, v + 0.3, f'{v:.0f}', ha='center', fontsize=9)
        ax.set_xticks([1.5, 6.5], ['11 ft', '35 ft'])
        ax.set_title(lab)
        ax.set_ylabel('median distance from the fitted flight (mm)')
    fig.legend(loc='lower center', ncol=4, fontsize=9, bbox_to_anchor=(0.5, -0.07))
    fig.suptitle('3D scatter of flying points about their fitted flight (points over 15 px off left out)', fontsize=12, y=1.02)
    fig.savefig(OUT / 'scatter_3d.png'); plt.close(fig)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    z = np.load(REC / 'sweep_left.npz', allow_pickle=True)
    fit = np.load(REC / 'flightfit.npy', allow_pickle=True).item()
    g = np.load(REC / 'track3d_grid.npz', allow_pickle=True)
    for f in (contrast, lambda: detector_vs_tracker(z, g), lambda: tracker_tradeoff(g), lambda: found_vs_junk(z), lambda: flights_35ft(fit),
              lambda: alpha_per_flight(fit), lambda: scatter_3d(fit)):
        f()
    print('\n'.join(str(p) for p in sorted(OUT.glob('*.png'))))


if __name__ == '__main__':
    main()

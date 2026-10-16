# draws the E08, E09 and E10 result figures from what track3d_crop.py and track3d_confirm.py saved
#   figures_e08_e10.py [recordings/E04_settings_sweep] [experiments/figures]
#
# E08/detectors_by_budget.png   flying frames found on held out flights against false blobs per frame, per detector
# E09/crop_tradeoff.png         every crop variant: flying frames found live against false reports per minute
# E10/within_20cm_over_time.png share of flights predicted within 20 cm, against time after the hit
# E10/error_sources.png         end point error at 0.3 s, and what it would be with the true velocity or position
# E10/example_flight_error.png  one 35 ft flight: prediction error as the flight goes on
import sys, json, pathlib, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import track3d_confirm as t3

HERE = pathlib.Path(__file__).resolve().parent
REC = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else HERE.parent.parent / 'recordings' / 'E04_settings_sweep'
OUT = pathlib.Path(sys.argv[2]) if len(sys.argv) > 2 else HERE.parent / 'experiments' / 'figures'
INK, INK2, GRID, SURF = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']  # categorical slots 1 to 4 of the dataviz reference palette
plt.rcParams.update({'font.size': 11, 'axes.spines.top': False, 'axes.spines.right': False, 'figure.dpi': 150,
                     'savefig.bbox': 'tight', 'axes.edgecolor': INK2, 'axes.labelcolor': INK, 'xtick.color': INK2,
                     'ytick.color': INK2, 'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8,
                     'figure.facecolor': SURF, 'axes.facecolor': SURF, 'lines.linewidth': 2})


def save(fig, name):
    p = OUT / name
    p.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(p)
    plt.close(fig)
    print(p)


def detectors_by_budget():
    # the E08 report's held out table (sweep_single.py on the 13 clips, colour rule allowed)
    budgets = [0.03, 0.1, 0.3, 1, 3]
    rows = {'tri': [65.5, 79.2, 87.5, 97.2, 98.8], 'prev': [65.2, 83.6, 89.5, 97.6, 99.4],
            'ra': [51.8, 75.0, 84.3, 92.2, 97.3], 'MOG2 colour': [20.6, 53.9, 87.2, 91.9, 98.5]}
    fig, ax = plt.subplots(figsize=(7, 4.2))
    for (name, ys), col in zip(rows.items(), SERIES):
        ax.plot(budgets, ys, color=col, marker='o', markersize=5, label=name)
    ax.set_xscale('log')
    ax.set_xticks(budgets, [f'{b:g}' for b in budgets])
    ax.set_xlim(0.02, 4)
    ax.legend(frameon=False, loc='lower right')
    ax.set_xlabel('false blobs allowed per frame')
    ax.set_ylabel('flying frames found (%)')
    ax.set_title('E08: detectors on the E04 clips, held out flights', loc='left', color=INK)
    save(fig, 'E08/detectors_by_budget.png')


def crop_tradeoff():
    z = np.load(REC / 'track3d_crop.npz', allow_pickle=True)
    R, names = z['R'], list(z['variants'])
    found = 100 * R[:, :, 0].sum(1) / R[:, :, 1].sum(1)
    fpm = R[:, :, 3].sum(1) / R[:, :, 4].sum(1)
    fig, ax = plt.subplots(figsize=(7, 4.2))
    ax.scatter(fpm, found, s=60, color=SERIES[0], edgecolor=SURF, linewidth=1.5, zorder=3)
    groups = {}
    for n, x, y in zip(names, fpm, found): groups.setdefault((round(x, 1), round(y, 1)), []).append(n)
    for (x, y), ns in groups.items():
        label = ', '.join(ns) if len(ns) <= 4 else f'{len(ns)} small crops: {", ".join(ns)}'
        ax.annotate(label, (x, y), xytext=(8, -12 if len(ns) > 4 else 4), textcoords='offset points', fontsize=8.5, color=INK,
                    wrap=True)
    ax.set_xlabel('false reports per minute')
    ax.set_ylabel('flying frames found live (%)')
    ax.set_xlim(8, 16)
    ax.set_ylim(43.2, 46)
    ax.set_title('E09: crop variants, all 85 flights (top left is better)', loc='left', color=INK)
    save(fig, 'E09/crop_tradeoff.png')


def within_over_time():
    z = np.load(REC / 'track3d_confirm.npz', allow_pickle=True)
    R, E, Er, fold = z['R'], z['E'], z['Eraw'], z['fold']
    configs = [tuple(c) for c in z['configs']]
    taus = list(z['taus'])
    fig, ax = plt.subplots(figsize=(7, 4.2))
    for c, col in zip((4, 6, 8), SERIES):
        tR, tE, _, _ = t3.cross_validate(R, E, Er, fold, np.array([i for i, x in enumerate(configs) if x[4] == c]), 15)
        ys = [100 * h / max(m, 1) for h, m in (t3.within(tE, 20, j) for j in range(len(taus)))]
        ax.plot(taus, ys, color=col, marker='o', markersize=5, label=f'confirm after {c} points')
    ax.set_xticks(taus, [f'{t:g}' for t in taus])
    ax.set_xlim(0, 0.55)
    ax.legend(frameon=False, loc='upper left')
    ax.set_ylim(0, 100)
    ax.set_xlabel('time after the hit (s)')
    ax.set_ylabel('flights predicted within 20 cm (%)')
    ax.set_title('E10: how soon the end point is known, held out flights', loc='left', color=INK)
    save(fig, 'E10/within_20cm_over_time.png')


def error_sources():
    # the E10 report's breakdown, E05 setting, latest state at 0.3 s after the hit, medians
    rows = [('11 ft', 15.2, 2.7, 12.5), ('35 ft', 25.1, 6.6, 18.8)]
    labels = ['as tracked', 'with the true velocity', 'with the true position']
    fig, ax = plt.subplots(figsize=(7, 3.4))
    y = np.arange(len(rows))
    for k, (lab, col) in enumerate(zip(labels, SERIES)):
        vals = [r[1 + k] for r in rows]
        ax.barh(y + (k - 1) * 0.26, vals, height=0.24, color=col, label=lab)
        for yy, v in zip(y, vals): ax.text(v + 0.4, yy + (k - 1) * 0.26, f'{v:.1f}', va='center', fontsize=9, color=INK)
    ax.set_yticks(y, [r[0] for r in rows])
    ax.invert_yaxis()
    ax.set_xlabel('median end point error at 0.3 s after the hit (cm)')
    ax.legend(frameon=False, loc='upper left', bbox_to_anchor=(1.0, 1.0), fontsize=9)
    ax.grid(axis='y', visible=False)
    ax.set_title('E10: the velocity estimate causes most of the error', loc='left', color=INK)
    save(fig, 'E10/error_sources.png')


def example_flight():
    p = REC / 'pipeline_walkthrough' / 'data.js'
    D = json.loads(p.read_text().split('=', 1)[1].rstrip().rstrip(';'))
    t = [q['t'] for q in D['predictions']]
    e = [q['error_cm'] for q in D['predictions']]
    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.plot(t, e, color=SERIES[0], marker='o', markersize=4)
    for cm in (5, 10, 20):
        ax.axhline(cm, color=INK2, linewidth=1, linestyle=(0, (4, 3)))
        ax.text(t[-1], cm, f' {cm} cm', va='center', fontsize=9, color=INK2)
    ax.set_xlabel('time after the hit (s)')
    ax.set_ylabel('predicted end point error (cm)')
    ax.set_title(f"one 35 ft flight ({D['clip']}): the guess closes in as points arrive", loc='left', color=INK)
    save(fig, 'E10/example_flight_error.png')


if __name__ == '__main__':
    detectors_by_budget()
    crop_tradeoff()
    within_over_time()
    error_sources()
    example_flight()

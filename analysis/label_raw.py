# label.py for recordings made by preview (primary.raw, timestamps.csv, metadata.txt)
#   label_raw.py <recording> [--proposals <track_single csv>] [--camera primary|secondary]
#
# same keys, screen and labels as label.py (see the top of that file); only the loading differs.
# labels autosave to <recording>/labels/primary.csv (or secondary.csv).
#
# frame numbers are positions in primary.raw, the same as the index column of timestamps.csv.
# if the recorder dropped frames, the camera's frame counter (id_primary) jumps there; those places
# are printed at startup. treat a flight across one with care: interpolation (i) assumes even steps.
import sys, csv, pathlib, numpy as np
import label


def load(d, camera='primary'):
    # memory mapped like rec.load, so a 14 GB recording opens instantly
    d = pathlib.Path(d)
    m = {}
    for line in open(d / 'metadata.txt'):
        k, eq, v = line.strip().partition('=')
        if eq:
            m[k] = int(v) if v.lstrip('-').isdigit() else v
    if m.get('pixel_format') != 'BayerRG8':
        sys.exit(f"only BayerRG8 recordings are supported, this one is {m.get('pixel_format')}")
    p = d / f'{camera}.raw'
    if not p.exists():
        sys.exit(f'{p} not found')
    f = np.memmap(p, np.uint8, mode='r').reshape(-1, m['height'], m['width'])
    report_dropped(d, camera, len(f))
    return f, m


def report_dropped(d, camera, n):
    col = 'id_primary' if camera == 'primary' else 'id_secondary'
    rows = list(csv.DictReader(open(d / 'timestamps.csv')))
    if len(rows) != n:
        print(f'warning: timestamps.csv has {len(rows)} rows but the raw file has {n} frames')
    ids = [int(r[col]) for r in rows if r[col]]
    gaps = [(i, ids[i] - ids[i - 1] - 1) for i in range(1, len(ids)) if ids[i] - ids[i - 1] != 1]
    if not gaps:
        print(f'{n} frames, none dropped while recording')
        return
    print(f'{n} frames, {sum(g for _, g in gaps)} dropped while recording, just before these frames:')
    for i, g in gaps[:20]:
        print(f'  frame {i}: {g} missing')
    if len(gaps) > 20:
        print(f'  ... and {len(gaps) - 20} more, the last before frame {gaps[-1][0]}')


if __name__ == '__main__':
    # label.py defaults to the stereo camera name 'left'; preview recordings call it 'primary'
    if '--camera' not in sys.argv:
        sys.argv += ['--camera', 'primary']
    label.load = load
    label.main()

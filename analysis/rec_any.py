# loader for all three clip layouts: preview (primary.raw, metadata.txt, timestamps.csv) as well as the
# record_stereo and record_single layouts rec.py reads. meta always has width, height, fps and frames.
import csv, pathlib, numpy as np
import rec


def is_preview(d):
    return (pathlib.Path(d) / 'metadata.txt').exists()


def default_camera(d):
    # preview recordings call their one camera primary; stereo clips default to the left camera
    return 'primary' if is_preview(d) else 'left'


def load(d, camera=None):
    d = pathlib.Path(d)
    camera = camera or default_camera(d)
    if not is_preview(d):
        return rec.load(d, camera)
    m = {}
    for line in open(d / 'metadata.txt'):
        k, eq, v = line.strip().partition('=')
        if eq:
            m[k] = int(v) if v.lstrip('-').isdigit() else v
    if m.get('pixel_format') != 'BayerRG8':
        raise SystemExit(f"only BayerRG8 recordings are supported, this one is {m.get('pixel_format')}")
    f = np.memmap(d / f'{camera}.raw', np.uint8, mode='r').reshape(-1, m['height'], m['width'])
    # preview does not write the frame rate; take it from the camera timestamps
    col = 'ts_primary' if camera == 'primary' else 'ts_secondary'
    ts = np.array([int(r[col]) for r in csv.DictReader(open(d / 'timestamps.csv')) if r[col]], np.int64)
    m['fps'] = float(round(1e9 / np.median(np.diff(ts)), 2)) if len(ts) > 1 else 0.0
    m['frames'] = len(f)
    return f, m

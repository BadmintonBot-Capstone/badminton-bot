# shared loader for record_single clips (frames.raw) and record_stereo clips (left.raw, right.raw)
import numpy as np, cv2, yaml, pathlib

def load(d, camera='left'):
    # memory mapped, so a 14 GB stereo clip opens instantly and only the frames used get read
    d = pathlib.Path(d)
    m = yaml.safe_load(open(d / "meta.yaml"))
    p = d / f"{camera}.raw"
    if not p.exists(): p = d / "frames.raw"
    f = np.memmap(p, np.uint8, mode='r').reshape(-1, m["height"], m["width"])
    return f, m

def gray2(f):
    # 2x2 bayer superpixel average, half resolution
    f = f.astype(np.float32)
    return 0.25 * (f[..., 0::2, 0::2] + f[..., 0::2, 1::2] + f[..., 1::2, 0::2] + f[..., 1::2, 1::2])

def bgr(frame):
    return cv2.cvtColor(np.ascontiguousarray(frame), cv2.COLOR_BayerBG2BGR)

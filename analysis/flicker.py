import sys, numpy as np, cv2
from rec import load, gray2, bgr
for d in sys.argv[1:]:
    f, m = load(d)
    g = gray2(f)
    mean = g.mean(axis=(1, 2))
    top = g[:, :100, :].mean(axis=(1, 2))           # ceiling lights band
    sat = (f >= 250).mean() * 100
    spec = np.abs(np.fft.rfft(mean - mean.mean())); fr = np.fft.rfftfreq(len(mean), 1 / m["fps"])
    pk = fr[1:][np.argmax(spec[1:])]
    tstd = np.median(g.std(axis=0))
    print(f"{d}: exp={m['exposure_us']:.0f}us gain={m['gain_db']}dB  mean={mean.mean():.1f}  "
          f"frame-to-frame mean flicker={100*mean.std()/mean.mean():.2f}%  top band={100*top.std()/top.mean():.2f}%  "
          f"peak freq={pk:.1f}Hz  median per-pixel temporal std={tstd:.2f}  saturated={sat:.2f}%")
    cv2.imwrite(f"{d}/frame0.png", bgr(f[0]))

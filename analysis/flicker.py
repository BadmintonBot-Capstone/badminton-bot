# finds the light flicker in still clips (E01)
#   flicker.py <clip> [<clip> ...]
#
# per-frame brightness is the mean over pixels of each pixel divided by its own median over time,
# leaving out the top and bottom 10% each frame so a person walking through barely moves it (a plain
# median sticks at exactly 1 on whole-number pixels). the strongest frequency in that series is the
# beat, and the table lists where flicker at each multiple of 120 Hz would land at the clip's fps.
import sys, numpy as np, cv2
from scipy.stats import trim_mean
from rec import load, gray2, bgr


def alias(f, r):
    # where flicker at f Hz shows up when sampled at r fps
    return abs(f - r * round(f / r))


for d in sys.argv[1:]:
    f, m = load(d)
    fps = m['fps']
    g = gray2(f)
    med = np.median(g, axis=0)
    ok = (med > 10) & (med < 240)
    # float64, since float32 sums over half a million pixels round at the 0.1% level
    b = trim_mean(g[:, ok].astype(np.float64) / med[ok], 0.1, axis=1)
    b = b - b.mean()
    spec = np.abs(np.fft.rfft(b * np.hanning(len(b))))
    fr = np.fft.rfftfreq(len(b), 1 / fps)
    i = 1 + np.argmax(spec[1:])
    snr = spec[i] / np.median(spec[1:])
    print(f'{d}: exposure {m["exposure_us"]:.0f} us, {fps:.0f} fps, {len(b)} frames, {100 * ok.mean():.0f}% of pixels used')
    print(f'  brightness swing {100 * (b.max() - b.min()):.2f}% peak to peak, {100 * b.std():.2f}% std')
    print(f'  strongest frequency {fr[i]:.1f} Hz (every {fps / fr[i]:.2f} frames), {snr:.0f}x the median spectrum')
    print('  flicker at ' + ', '.join(f'{120 * k} Hz shows at {alias(120 * k, fps):.0f}' for k in range(1, 7)))
    cv2.imwrite(f'{d}/frame0.png', bgr(f[0]))

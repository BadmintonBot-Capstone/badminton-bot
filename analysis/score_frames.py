# scores a track_single csv against per-frame labels from label.py
#   score_frames.py <clip> <track.csv> [--labels labels/left.csv] [--radius 30]
#
# detection: in every labelled visible frame (flying and still counted apart), is there a report
# within --radius full resolution pixels of the label. a report on a frame labelled not visible is
# false, a report far from a visible label is misplaced, unsure frames are left out.
# flight start: every flight (from a new_flight label to the end of its flying run) is found or not,
# and the delay is the frames from its first flying frame to the first report on it.
import sys, csv, pathlib, argparse, numpy as np, yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clip')
    ap.add_argument('track')
    ap.add_argument('--labels', default='labels/left.csv')
    ap.add_argument('--radius', type=float, default=30)
    a = ap.parse_args()

    clip = pathlib.Path(a.clip)
    fps = yaml.safe_load(open(clip / 'meta.yaml'))['fps']
    if not (clip / a.labels).exists():
        sys.exit(f'no labels at {clip / a.labels}, label the clip first: python label.py {clip}')
    lab = {int(r['frame']): r for r in csv.DictReader(open(clip / a.labels))}
    rows = list(csv.DictReader(open(clip / a.track if not pathlib.Path(a.track).exists() else a.track)))
    hits = {int(r['frame']): (float(r['x']), float(r['y'])) for r in rows if r['state'] == 'hit'}
    ms = [float(r['detect_ms']) for r in rows if r.get('detect_ms')]

    def near(f):
        l, h = lab[f], hits.get(f)
        return h is not None and np.hypot(h[0] - float(l['x']), h[1] - float(l['y'])) <= a.radius

    found = {'flying': [0, 0], 'still': [0, 0]}
    false_none = misplaced = unlabelled = 0
    for f, l in lab.items():
        if l['state'] in found:
            found[l['state']][1] += 1
            if near(f): found[l['state']][0] += 1
            elif f in hits: misplaced += 1
    for f in hits:
        if f not in lab: unlabelled += 1
        elif lab[f]['state'] == 'none': false_none += 1

    # flights: a new_flight label starts one, it runs while the following labels stay flying
    flights, cur = [], None
    for f in sorted(lab):
        l = lab[f]
        if l['state'] == 'flying' and (l['new_flight'] == '1' or cur is None):
            if cur: flights.append(cur)
            cur = [f]
        elif l['state'] == 'flying' and cur:
            cur.append(f)
        elif cur:
            flights.append(cur); cur = None
    if cur: flights.append(cur)
    delays = []
    for fl in flights:
        first = next((f for f in fl if near(f)), None)
        if first is not None: delays.append(first - fl[0])

    labelled_minutes = len(lab) / fps / 60
    print(f'{clip.name} {pathlib.Path(a.track).name}  ({len(lab)} labelled frames, radius {a.radius:.0f} px)')
    for k, (got, tot) in found.items():
        print(f'  {k:6s} found {got}/{tot}' + (f' ({100 * got / tot:.0f}%)' if tot else ''))
    print(f'  false reports {false_none} on not-visible frames, {misplaced} misplaced on visible frames, '
          f'{(false_none + misplaced) / max(labelled_minutes, 1e-9):.1f} per minute')
    if delays:
        print(f'  flights found {len(delays)}/{len(flights)}, frames from flight start to first report: '
              f'median {np.median(delays):.0f}, p90 {np.percentile(delays, 90):.0f}')
    else:
        print(f'  flights found 0/{len(flights)}')
    if ms:
        print(f'  detect {np.mean(ms):.2f} ms mean, {np.percentile(ms, 95):.2f} ms p95')
    if unlabelled:
        print(f'  {unlabelled} reports on unlabelled frames were not scored')


if __name__ == '__main__':
    main()

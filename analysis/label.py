# per-frame shuttle labelling, keyboard first
#   label.py <clip> [--camera left|right] [--proposals <track_single csv>]
#
# each frame gets a state and, when the shuttle is visible, where it is:
#   f  flying             s  visible but still (hand, racket, floor)
#   n  not visible        u  unsure
# a state key labels the current frame and moves on. the position is the last click on this frame,
# otherwise the tracker's report (--proposals), otherwise the previous label carried forward
# (a flying shuttle keeps its last step).
#
#   click        set the position on this frame (full view or zoom)
#   space        play: every unlabelled frame gets the state you were on until you press space again
#   1 2 3 4      play speed 10, 25, 50, 100 frames a second
#   , .          back / forward one frame      [ ]  back / forward 10 frames
#   g            go to the next unlabelled frame
#   h            toggle "new flight starts here" (set on its own after a non-flying frame)
#   i            fill unlabelled gaps of up to 15 frames inside a flight by interpolation
#   x            clear this frame's label       + -  zoom in / out
#   q            save and quit
# labels autosave to <clip>/labels/<camera>.csv as frame,state,x,y,new_flight,source
import sys, csv, time, pathlib, argparse, numpy as np, cv2
from rec import load, bgr

STATES = {ord('f'): 'flying', ord('s'): 'still', ord('n'): 'none', ord('u'): 'unsure'}
COLOUR = {'flying': (0, 255, 0), 'still': (255, 160, 0), 'none': (128, 128, 128), 'unsure': (0, 200, 255)}
VIEW_W, VIEW_H, ZOOM = 960, 600, 600
SPEEDS = {ord('1'): 10, ord('2'): 25, ord('3'): 50, ord('4'): 100}
LEFT_KEYS, RIGHT_KEYS = {ord(','), 65361, 81}, {ord('.'), 65363, 83}


def load_labels(path, n):
    lab = [None] * n
    if path.exists():
        for r in csv.DictReader(open(path)):
            i = int(r['frame'])
            if i < n:
                lab[i] = dict(state=r['state'], x=float(r['x']) if r['x'] else None, y=float(r['y']) if r['y'] else None,
                              new_flight=r['new_flight'] == '1', source=r['source'])
    return lab


def save_labels(path, lab):
    path.parent.mkdir(exist_ok=True)
    tmp = path.with_suffix('.tmp')
    with open(tmp, 'w') as f:
        f.write('frame,state,x,y,new_flight,source\n')
        for i, l in enumerate(lab):
            if l:
                xy = f"{l['x']:.1f},{l['y']:.1f}" if l['x'] is not None else ','
                f.write(f"{i},{l['state']},{xy},{int(l['new_flight'])},{l['source']}\n")
    tmp.replace(path)


def load_proposals(path, n):
    prop = [None] * n
    if path:
        for r in csv.DictReader(open(path)):
            i = int(r['frame'])
            if i < n and r['state'] == 'hit':
                prop[i] = (float(r['x']), float(r['y']))
    return prop


def fill_gap(lab, keys, j):
    # x and y as quadratics in time through up to three labelled flying frames on each side of the gap
    a_i, b_i = keys[j], keys[j + 1]
    lo, hi = j, j + 1
    while lo > max(0, j - 2) and not lab[keys[lo]]['new_flight']:
        lo -= 1
    while hi < j + 3 and hi + 1 < len(keys) and not lab[keys[hi + 1]]['new_flight']:
        hi += 1
    near = [k for k in keys[lo:hi + 1] if abs(k - a_i) <= 20 or abs(k - b_i) <= 20]
    t = np.array(near, float)
    deg = min(2, len(near) - 1)
    cx = np.polyfit(t, [lab[k]['x'] for k in near], deg)
    cy = np.polyfit(t, [lab[k]['y'] for k in near], deg)
    return {k: (float(np.polyval(cx, k)), float(np.polyval(cy, k))) for k in range(a_i + 1, b_i)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('clip')
    ap.add_argument('--camera', default='left')
    ap.add_argument('--proposals')
    a = ap.parse_args()

    clip = pathlib.Path(a.clip)
    frames, meta = load(clip, a.camera)
    n, W, H = len(frames), meta['width'], meta['height']
    out = clip / 'labels' / f'{a.camera}.csv'
    lab = load_labels(out, n)
    prop = load_proposals(a.proposals, n)

    s = dict(i=next((k for k, l in enumerate(lab) if l is None), 0), click=None, playing=False, play_state=None,
             speed=25, crop=160, dirty=0, last_save=time.time())

    def guess(i):
        # where the shuttle probably is on frame i, before the user says anything
        if s['click'] is not None and s['click'][0] == i:
            return s['click'][1], 'click'
        if prop[i] is not None:
            return prop[i], 'proposal'
        prev = lab[i - 1] if i > 0 else None
        if prev and prev['x'] is not None:
            p = np.array([prev['x'], prev['y']])
            back = lab[i - 2] if i > 1 else None
            if prev['state'] == 'flying' and back and back['state'] == 'flying' and back['x'] is not None:
                p = p + (p - np.array([back['x'], back['y']]))
            return (float(p[0]), float(p[1])), 'carried'
        return None, None

    def put(i, state):
        pos, src = guess(i) if state in ('flying', 'still') else (None, None)
        if state in ('flying', 'still') and pos is None:
            print(f'  frame {i}: click the shuttle first')
            return False
        # a new flight unless the nearest earlier label, within 15 frames, is flying
        prev = next((lab[k] for k in range(i - 1, max(-1, i - 16), -1) if lab[k]), None)
        new_flight = state == 'flying' and (prev is None or prev['state'] != 'flying')
        lab[i] = dict(state=state, x=pos[0] if pos else None, y=pos[1] if pos else None, new_flight=new_flight, source=src or 'key')
        s['dirty'] += 1
        return True

    def maybe_save(force=False):
        if s['dirty'] and (force or s['dirty'] >= 25 or time.time() - s['last_save'] > 10):
            save_labels(out, lab)
            s['dirty'], s['last_save'] = 0, time.time()

    def interpolate():
        filled = 0
        keys = [k for k, l in enumerate(lab) if l and l['state'] == 'flying']
        for j in range(len(keys) - 1):
            a_i, b_i = keys[j], keys[j + 1]
            gap = b_i - a_i - 1
            if gap < 1 or gap > 15 or lab[b_i]['new_flight'] or any(lab[k] for k in range(a_i + 1, b_i)):
                continue
            for k, (x, y) in fill_gap(lab, keys, j).items():
                lab[k] = dict(state='flying', x=x, y=y, new_flight=False, source='interpolated')
                filled += 1
        s['dirty'] += filled
        print(f'  interpolated {filled} frames')

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_MOUSEMOVE:
            # the zoom follows the cursor over the full view and holds still while the cursor is
            # in the zoom, so a click there lands where it was aimed; below the views it goes back
            # to following the shuttle
            s['cursor_in_view'] = x < VIEW_W and y < VIEW_H
            if s['cursor_in_view']:
                s['hover'] = (x * W / VIEW_W, y * H / VIEW_H)
            elif y >= VIEW_H:
                s['hover'] = None
            return
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if x < VIEW_W and y < VIEW_H:
            pos = (x * W / VIEW_W, y * H / VIEW_H)
        elif VIEW_W <= x < VIEW_W + ZOOM and y < ZOOM and s.get('zoom_origin'):
            ox, oy = s['zoom_origin']
            pos = (ox + (x - VIEW_W) * s['crop'] / ZOOM, oy + y * s['crop'] / ZOOM)
        else:
            return
        s['click'] = (s['i'], pos)

    win = f'label {clip.name} {a.camera}'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, VIEW_W + ZOOM, VIEW_H + 70)
    cv2.setMouseCallback(win, on_mouse)

    cache = {}
    while True:
        i = s['i']
        if cache.get('i') != i:
            cache.update(i=i, img=cv2.convertScaleAbs(bgr(frames[i]), alpha=1.8))
        img = cache['img']
        canvas = np.zeros((VIEW_H + 70, VIEW_W + ZOOM, 3), np.uint8)
        view = cv2.resize(img, (VIEW_W, VIEW_H), interpolation=cv2.INTER_AREA)
        sx, sy = VIEW_W / W, VIEW_H / H

        # trail of the last labelled positions, the tracker's report, and this frame's label or guess
        for k in range(max(0, i - 15), i):
            if lab[k] and lab[k]['x'] is not None:
                cv2.circle(view, (int(lab[k]['x'] * sx), int(lab[k]['y'] * sy)), 2, COLOUR[lab[k]['state']], -1)
        if prop[i] is not None:
            cv2.circle(view, (int(prop[i][0] * sx), int(prop[i][1] * sy)), 10, (255, 255, 0), 1)
        cur = lab[i]
        if s['click'] is not None and s['click'][0] == i:
            pos, src = s['click'][1], 'click'
        elif cur and cur['x'] is not None:
            pos, src = (cur['x'], cur['y']), cur['source']
        else:
            pos, src = guess(i)
        if pos is not None:
            col = COLOUR[cur['state']] if cur else (255, 255, 255)
            cv2.circle(view, (int(pos[0] * sx), int(pos[1] * sy)), 14, col, 2)
        # zoom around the cursor, else the position, else the middle of the frame
        if s.get('hover') is not None:
            cx, cy = s['hover']
        else:
            cx, cy = pos if pos is not None else (W / 2, H / 2)
        c = s['crop']
        ox, oy = int(np.clip(cx - c / 2, 0, W - c)), int(np.clip(cy - c / 2, 0, H - c))
        s['zoom_origin'] = (ox, oy)
        # outline the zoomed area on the full view
        cv2.rectangle(view, (int(ox * sx), int(oy * sy)), (int((ox + c) * sx), int((oy + c) * sy)), (0, 0, 255), 1)
        canvas[:VIEW_H, :VIEW_W] = view
        zoom = cv2.resize(img[oy:oy + c, ox:ox + c], (ZOOM, ZOOM), interpolation=cv2.INTER_NEAREST)
        if pos is not None:
            zx, zy = int((pos[0] - ox) * ZOOM / c), int((pos[1] - oy) * ZOOM / c)
            cv2.drawMarker(zoom, (zx, zy), (0, 0, 255), cv2.MARKER_CROSS, 30, 1)
        if s.get('cursor_in_view') and s.get('hover') is not None:
            # where a click on the full view would land, so it can be checked before clicking
            hx, hy = int((s['hover'][0] - ox) * ZOOM / c), int((s['hover'][1] - oy) * ZOOM / c)
            cv2.drawMarker(zoom, (hx, hy), (255, 255, 0), cv2.MARKER_TILTED_CROSS, 20, 1)
            cv2.circle(zoom, (hx, hy), 12, (255, 255, 0), 1)
        canvas[:ZOOM, VIEW_W:] = zoom

        done = sum(l is not None for l in lab)
        state_txt = (cur['state'] + (' NEW FLIGHT' if cur['new_flight'] else '') + f' ({cur["source"]})') if cur else f'unlabelled, guess from {src}'
        line1 = f'frame {i}/{n - 1}   {state_txt}   labelled {done}/{n}'
        line2 = (f'PLAYING {s["play_state"]} at {s["speed"]} fps' if s['playing'] else
                 'f flying  s still  n none  u unsure  space play  , . [ ] move  g next gap  h new flight  i interp  x clear  q quit')
        cv2.putText(canvas, line1, (10, VIEW_H + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(canvas, line2, (10, VIEW_H + 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255) if s['playing'] else (200, 200, 200), 1)
        cv2.imshow(win, canvas)

        # short wait while idle too, so a click shows up without a key press
        key = cv2.waitKeyEx(max(1, int(1000 / s['speed'])) if s['playing'] else 30)
        k8 = key & 0xFF if key != -1 else -1

        if s['playing']:
            if key != -1 and (k8 == ord(' ') or k8 in STATES or k8 == ord('q')):
                s['playing'] = False
                maybe_save(force=True)
                if k8 == ord('q'):
                    break
                continue
            if lab[i] is None and not put(i, s['play_state']):
                s['playing'] = False
                continue
            if i + 1 >= n:
                s['playing'] = False
            else:
                s['i'] = i + 1
            maybe_save()
            continue

        if key == -1:
            continue
        if k8 == ord('q'):
            break
        if k8 in STATES:
            if put(i, STATES[k8]) and i + 1 < n:
                s['i'] = i + 1
        elif k8 == ord(' '):
            last = lab[i] or (lab[i - 1] if i > 0 else None)
            if last:
                s['playing'], s['play_state'] = True, last['state']
            else:
                print('  label a frame first, play repeats its state')
        elif k8 in SPEEDS:
            s['speed'] = SPEEDS[k8]
        elif key in LEFT_KEYS:
            s['i'] = max(0, i - 1)
        elif key in RIGHT_KEYS:
            s['i'] = min(n - 1, i + 1)
        elif k8 == ord('['):
            s['i'] = max(0, i - 10)
        elif k8 == ord(']'):
            s['i'] = min(n - 1, i + 10)
        elif k8 == ord('g'):
            s['i'] = next((k for k in range(i + 1, n) if lab[k] is None), i)
        elif k8 == ord('h') and lab[i] and lab[i]['state'] == 'flying':
            lab[i]['new_flight'] = not lab[i]['new_flight']
            s['dirty'] += 1
        elif k8 == ord('i'):
            interpolate()
        elif k8 == ord('x'):
            lab[i] = None
            s['dirty'] += 1
        elif k8 in (ord('+'), ord('=')):
            s['crop'] = max(40, s['crop'] // 2)
        elif k8 == ord('-'):
            s['crop'] = min(640, s['crop'] * 2)
        maybe_save()

    maybe_save(force=True)
    cv2.destroyAllWindows()
    print(f'saved {sum(l is not None for l in lab)} labels to {out}')


if __name__ == '__main__':
    main()

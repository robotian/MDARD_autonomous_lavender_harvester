#!/usr/bin/env python3
"""metrics.py <recording.jsonl> [label]: per-segment step metrics of step.py's recording, against the ground truth:
mean and its error (last half of the hold), sd (% of the command), time to 90 %, overshoot, stop time, max tilt.
Writes <recording>.metrics.json."""
import json
import math
import sys

import numpy as np

rows = [json.loads(l) for l in open(sys.argv[1])]
label = sys.argv[2] if len(sys.argv) > 2 else ''
cmd = [r for r in rows if r[0] == 'cmd']
ref = np.array([r[1:6] for r in rows if r[0] == 'ref'])  # t, x, y, yaw, tilt
k = 2
tr = ref[:, 0]
dt = tr[2 * k:] - tr[:-2 * k]
yaw_mid = ref[k:-k, 3]
dx, dy = ref[2 * k:, 1] - ref[:-2 * k, 1], ref[2 * k:, 2] - ref[:-2 * k, 2]
dyaw = np.arctan2(np.sin(ref[2 * k:, 3] - ref[:-2 * k, 3]), np.cos(ref[2 * k:, 3] - ref[:-2 * k, 3]))
T = tr[k:-k]
V = (dx * np.cos(yaw_mid) + dy * np.sin(yaw_mid)) / dt
W = dyaw / dt
TILT = ref[k:-k, 4]

segs = []
labels = [c[4] for c in cmd]
for i, lab in enumerate(labels):
    if lab != 'stop' and (i == 0 or labels[i - 1] != lab):
        start = cmd[i][1]
    if lab != 'stop' and (i == len(labels) - 1 or labels[i + 1] != lab):
        segs.append((lab, start, cmd[i][1] + 0.05, cmd[i][2], cmd[i][3]))

out = []
for lab, a, b, v, w in segs:
    seg = {'segment': lab, 'v': v, 'w': w}
    on = (T >= a) & (T < b)
    late = (T >= a + 0.5 * (b - a)) & (T < b)
    after = (T >= b) & (T < b + 2.5)
    seg['tilt_max'] = float(np.max(TILT[on | after]))
    seg['tilt_sd'] = float(np.std(TILT[on]))
    for axis, target, Y in (('v', v, V), ('w', w, W)):
        if abs(target) < 1e-9:
            seg[f'{axis}_stray'] = float(np.mean(np.abs(Y[late])))  # motion on the axis that should be still
            continue
        s, A = np.sign(target), abs(target)
        y_on = Y[on] * s
        t_on = T[on] - a
        r90 = next((t for t, y in zip(t_on, y_on) if y >= 0.9 * A), float('nan'))
        mean = float(np.mean(Y[late]))
        y_after = Y[after] * s
        stop = next((t - b for t, y in zip(T[after], y_after) if y <= 0.1 * A), float('nan'))
        seg[axis] = dict(mean=mean, err=(abs(mean) - A) / A * 100, sd=float(np.std(Y[late])) / A * 100,
                         rise90=r90, overshoot=(float(np.max(y_on)) - A) / A * 100, stop=stop)
    out.append(seg)

print(f"== {label}")
print(f"{'segment':<20}{'axis':<5}{'mean':>8}{'err%':>7}{'sd%':>7}{'t90 s':>7}{'over%':>7}{'stop s':>7}{'tilt°':>7}")
worst = dict(err=0, sd=0, over=0, tilt=0, rise=0)
for s in out:
    for axis in ('v', 'w'):
        m = s.get(axis)
        if not m:
            continue
        print(f"{s['segment']:<20}{axis:<5}{m['mean']:8.3f}{m['err']:7.1f}{m['sd']:7.1f}{m['rise90']:7.2f}"
              f"{m['overshoot']:7.0f}{m['stop']:7.2f}{s['tilt_max']:7.1f}")
        worst['err'] = max(worst['err'], abs(m['err']))
        worst['sd'] = max(worst['sd'], m['sd'])
        worst['over'] = max(worst['over'], m['overshoot'])
        worst['rise'] = max(worst['rise'], m['rise90'] if not math.isnan(m['rise90']) else 99)
    worst['tilt'] = max(worst['tilt'], s['tilt_max'])
print(f"worst: |err| {worst['err']:.1f} %, sd {worst['sd']:.1f} %, overshoot {worst['over']:.0f} %, "
      f"t90 {worst['rise']:.2f} s, tilt {worst['tilt']:.1f}°")
json.dump({'label': label, 'segments': out, 'worst': worst}, open(sys.argv[1].replace('.jsonl', '.metrics.json'), 'w'), indent=1)

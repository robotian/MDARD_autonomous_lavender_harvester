#!/usr/bin/env python3
"""plot.py <recording.jsonl> <out.png> [robot]: commanded vs actual (sim ground truth) vs platform/odom twist,
with per-segment numbers (also written to <out>.json)."""
import json
import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

IN, PNG = sys.argv[1], sys.argv[2]
rows = [json.loads(l) for l in open(IN)]
cmd = [r for r in rows if r[0] == 'cmd']
ref = np.array([r[1:5] for r in rows if r[0] == 'ref'])          # t, x, y, yaw
odom = np.array([[r[1], r[6], r[7]] for r in rows if r[0] == 'odom'])  # t, v, w
t0 = cmd[0][1]

# ground-truth body-frame speed and yaw rate: central differences over +-2 samples (~0.13 s)
k = 2
tr = ref[:, 0]
dt = tr[2 * k:] - tr[:-2 * k]
dx, dy = ref[2 * k:, 1] - ref[:-2 * k, 1], ref[2 * k:, 2] - ref[:-2 * k, 2]
yaw_mid = ref[k:-k, 3]
dyaw = np.arctan2(np.sin(ref[2 * k:, 3] - ref[:-2 * k, 3]), np.cos(ref[2 * k:, 3] - ref[:-2 * k, 3]))
truth_t = tr[k:-k] - t0
truth_v = (dx * np.cos(yaw_mid) + dy * np.sin(yaw_mid)) / dt
truth_w = dyaw / dt
od_t, od_v, od_w = odom[:, 0] - t0, odom[:, 1], odom[:, 2]

# command as a step function
ct = np.array([c[1] for c in cmd]) - t0
cv = np.array([c[2] for c in cmd])
cw = np.array([c[3] for c in cmd])
labels = [c[4] for c in cmd]

# segments: [start, end) of each non-stop command run
segs = []
for i, lab in enumerate(labels):
    if lab != 'stop' and (i == 0 or labels[i - 1] != lab):
        start = ct[i]
    if lab != 'stop' and (i == len(labels) - 1 or labels[i + 1] != lab):
        segs.append((lab, start, ct[i] + 0.05, cv[i], cw[i]))


def interp(t, tt, yy):
    return np.interp(t, tt, yy)


def step_metrics(t_on, t_off, target, tt, yy):
    """delay to 10 %, time to 90 %, steady-state mean (last 60 % of the hold), overshoot, stop time + coast."""
    if abs(target) < 1e-9:
        return None
    s = np.sign(target)
    on = (tt >= t_on) & (tt < t_off)
    t_on_arr, y_on = tt[on], yy[on] * s
    a = abs(target)
    d10 = next((t - t_on for t, y in zip(t_on_arr, y_on) if y >= 0.1 * a), float('nan'))
    r90 = next((t - t_on for t, y in zip(t_on_arr, y_on) if y >= 0.9 * a), float('nan'))
    ss_mask = (tt >= t_on + 0.4 * (t_off - t_on)) & (tt < t_off)
    ss = float(np.mean(yy[ss_mask])) if ss_mask.any() else float('nan')
    sd = float(np.std(yy[ss_mask])) if ss_mask.any() else float('nan')
    over = (float(np.max(y_on)) - a) / a * 100
    off = (tt >= t_off) & (tt < t_off + 3.0)
    t_off_arr, y_off = tt[off], yy[off] * s
    stop = next((t - t_off for t, y in zip(t_off_arr, y_off) if y <= 0.1 * a), float('nan'))
    # coast after the zero command: integrate the speed until it stays below 2 %
    coast = float(np.trapz(np.clip(y_off, 0, None), t_off_arr)) if len(t_off_arr) > 1 else float('nan')
    return dict(delay10=d10, rise90=r90, ss=ss, sd=sd, err=(abs(ss) - a) / a * 100, overshoot=over, stop=stop,
                coast=coast)


report = []
for lab, a, b, v, w in segs:
    for name, target, (tt, yy), (ot, oy), unit in (
            ('linear', v, (truth_t, truth_v), (od_t, od_v), 'm/s'), ('angular', w, (truth_t, truth_w), (od_t, od_w), 'rad/s')):
        if abs(target) < 1e-9:
            continue
        m = step_metrics(a, b, target, tt, yy)
        mo = step_metrics(a, b, target, ot, oy)
        report.append((lab, name, target, unit, m, mo))
        print(f"{lab:<26} {name:<8} cmd {target:.2f} {unit:<5} | truth ss {m['ss']:.3f} ({m['err']:+.1f} %, sd {m['sd']:.3f}) "
              f"delay10 {m['delay10']:.2f} s rise90 {m['rise90']:.2f} s overshoot {m['overshoot']:+.0f} % "
              f"stop {m['stop']:.2f} s coast {m['coast']:.3f} {'m' if name == 'linear' else 'rad'} | "
              f"odom ss {mo['ss']:.3f} ({mo['err']:+.1f} %) delay10 {mo['delay10']:.2f} s")
json.dump([dict(segment=s, axis=n, cmd=t, unit=u, truth=m, odom=mo) for s, n, t, u, m, mo in report],
          open(PNG.replace('.png', '.json'), 'w'), indent=1)

# ---- plot
INK, INK2, MUTED, GRID, SURF = '#0b0b0b', '#52514e', '#898781', '#e1e0d9', '#fcfcfb'
TRUTH, ODOM = '#2a78d6', '#eb6834'
plt.rcParams.update({'font.size': 10, 'axes.edgecolor': '#c3c2b7', 'axes.labelcolor': INK2, 'xtick.color': MUTED,
                     'ytick.color': MUTED, 'axes.titlecolor': INK, 'figure.facecolor': SURF, 'axes.facecolor': SURF})
fig, axes = plt.subplots(2, 1, figsize=(12, 7.2), sharex=True, constrained_layout=True)
step_t = np.repeat(ct, 2)[1:]
for ax, (cy, ty, oy, title, unit) in zip(axes, ((cv, truth_v, od_v, 'Linear velocity (forward)', 'm/s'),
                                                 (cw, truth_w, od_w, 'Angular velocity (yaw rate)', 'rad/s'))):
    for lab, a, b, v, w in segs:
        ax.axvspan(a, b, color='#f0efec', zorder=0)
    ax.plot(step_t, np.repeat(cy, 2)[:-1], color=INK, lw=1.6, ls=(0, (5, 3)), label='commanded (cmd_vel)', zorder=3)
    ax.plot(truth_t, ty, color=TRUTH, lw=2, label='actual (sim ground truth)', zorder=4)
    ax.plot(od_t, oy, color=ODOM, lw=1.2, alpha=0.9, label='platform/odom twist', zorder=2)
    ax.set_title(title, loc='left', fontsize=11.5, fontweight='semibold')
    ax.set_ylabel(unit)
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
handles, names = axes[0].get_legend_handles_labels()
fig.legend(handles, names, loc='upper right', bbox_to_anchor=(0.995, 0.962), frameon=False, ncol=3)
axes[1].set_xlabel('sim time since the first command (s)')
axes[0].set_xlim(-1.0, segs[-1][2] + 3.5)
axes[0].set_ylim(-0.05, 0.6)
axes[1].set_ylim(-0.05, 0.9)
for lab, a, b, v, w in segs:
    axes[0].text((a + b) / 2, 0.585, lab, ha='center', va='top', fontsize=9.5, color=INK, fontweight='semibold')
# per-segment numbers in the free space above the data
for lab, name, target, unit, m, mo in report:
    a, b = next((s[1], s[2]) for s in segs if s[0] == lab)
    ax, y = (axes[0], 0.52) if name == 'linear' else (axes[1], 0.84)
    ax.text((a + b) / 2, y, f"mean {m['ss']:.3f} ({m['err']:+.1f} %)  ±{m['sd']:.3f}\n"
            f"90 % in {m['rise90']:.2f} s · overshoot {m['overshoot']:+.0f} %", ha='center', va='top', fontsize=8.5,
            color=TRUTH, linespacing=1.4)
fig.suptitle(f"{sys.argv[3] if len(sys.argv) > 3 else 'robot'} (sim): cmd_vel steps vs actual velocity", x=0.01, ha='left', fontsize=13, fontweight='semibold',
             color=INK)
fig.savefig(PNG, dpi=130)
print('wrote', PNG)

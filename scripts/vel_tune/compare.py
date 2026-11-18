#!/usr/bin/env python3
"""compare.py before.jsonl after.jsonl out.png [robot] [before title] [after title]: commanded vs actual (ground
truth) side by side, e.g. before and after a vel_ctl / wheel_friction change."""
import json, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

def load(path):
    rows = [json.loads(l) for l in open(path)]
    cmd = [r for r in rows if r[0] == 'cmd']
    ref = np.array([r[1:5] for r in rows if r[0] == 'ref'])
    t0 = next(c[1] for c in cmd if c[4] != 'stop') - 2.0
    k = 2; tr = ref[:, 0]; dt = tr[2*k:] - tr[:-2*k]; yaw = ref[k:-k, 3]
    dx, dy = ref[2*k:, 1] - ref[:-2*k, 1], ref[2*k:, 2] - ref[:-2*k, 2]
    dyaw = np.arctan2(np.sin(ref[2*k:, 3] - ref[:-2*k, 3]), np.cos(ref[2*k:, 3] - ref[:-2*k, 3]))
    ct = np.array([c[1] for c in cmd]) - t0
    segs, labels = [], [c[4] for c in cmd]
    for i, lab in enumerate(labels):
        if lab != 'stop' and (i == 0 or labels[i-1] != lab): start = ct[i]
        if lab != 'stop' and (i == len(labels)-1 or labels[i+1] != lab): segs.append((lab, start, ct[i]))
    return dict(t=tr[k:-k] - t0, v=(dx*np.cos(yaw) + dy*np.sin(yaw))/dt, w=dyaw/dt, ct=ct,
                cv=np.array([c[2] for c in cmd]), cw=np.array([c[3] for c in cmd]), segs=segs)

B, A = load(sys.argv[1]), load(sys.argv[2])
INK, INK2, MUTED, GRID, SURF, BLUE = '#0b0b0b', '#52514e', '#898781', '#e1e0d9', '#fcfcfb', '#2a78d6'
plt.rcParams.update({'font.size': 10, 'axes.edgecolor': '#c3c2b7', 'axes.labelcolor': INK2, 'xtick.color': MUTED,
                     'ytick.color': MUTED, 'figure.facecolor': SURF, 'axes.facecolor': SURF})
fig, ax = plt.subplots(2, 2, figsize=(14, 7.4), sharex=True, constrained_layout=True)
names = {'linear 0.3 m/s': 'linear 0.3', 'angular 0.3 rad/s': 'angular 0.3', 'linear 0.3 + angular 0.3': 'both'}
titles = (sys.argv[5] if len(sys.argv) > 5 else 'Before', sys.argv[6] if len(sys.argv) > 6 else 'After')
for col, (D, title) in enumerate(((B, titles[0]), (A, titles[1]))):
    for row, (key, ckey, unit, ylim) in enumerate((('v', 'cv', 'm/s', (-0.05, 0.5)), ('w', 'cw', 'rad/s', (-0.05, 0.75)))):
        a = ax[row, col]
        for lab, s, e in D['segs']:
            a.axvspan(s, e, color='#f0efec', zorder=0)
            if row == 0:
                a.text((s + e) / 2, ylim[1] * 0.96, names.get(lab, lab), ha='center', va='top', fontsize=9, color=INK2)
        a.plot(np.repeat(D['ct'], 2)[1:], np.repeat(D[ckey], 2)[:-1], color=INK, lw=1.5, ls=(0, (5, 3)),
               label='commanded', zorder=3)
        a.plot(D['t'], D[key], color=BLUE, lw=1.9, label='actual (ground truth)', zorder=4)
        a.set_ylim(*ylim); a.grid(True, color=GRID, lw=0.8); a.set_axisbelow(True)
        for sp in ('top', 'right'): a.spines[sp].set_visible(False)
        if col == 0: a.set_ylabel(('Linear  ' if row == 0 else 'Angular  ') + unit)
        if row == 0: a.set_title(title, loc='left', fontsize=11.5, fontweight='semibold', color=INK)
        if row == 1: a.set_xlabel('sim time (s)')
h, l = ax[0, 0].get_legend_handles_labels()
fig.legend(h, l, loc='upper right', bbox_to_anchor=(0.995, 1.0), frameon=False, ncol=2)
fig.suptitle(f"{sys.argv[4] if len(sys.argv) > 4 else 'robot'} (sim): cmd_vel steps, before vs after", x=0.01, ha='left',
             fontsize=13, fontweight='semibold', color=INK)
fig.savefig(sys.argv[3], dpi=125)
print('wrote', sys.argv[3])

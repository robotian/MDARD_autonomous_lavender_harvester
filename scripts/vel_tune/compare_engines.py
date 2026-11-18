#!/usr/bin/env python3
"""compare_engines.py <out_dir> [sweep_log_dir]: plots of j100_0921's drive under Newton (2026-10-20 tuning): the step
test's key segments (command vs ground-truth speed) and the calibrate_velocity.py sweeps (achieved / commanded), before
and after the ground-plane / wheel-friction fixes and the retuning. Inputs: sim/generated/vel_check/j100_0921_newton_*.jsonl and the
sweep logs named in SWEEPS."""
import json
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

OUT = sys.argv[1]
ROOT = os.path.join(os.path.dirname(__file__), '../..')
VEL = os.path.join(ROOT, 'sim/generated/vel_check')
# the skill's reference categorical palette, slots 1-3 in fixed order; text in ink tokens, never the series colour
INK, INK2, GRID, SURFACE = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
RUNS = [  # (label, step-test recording, sweep log or None, colour)
    ('Newton now: plane ground, friction fixed, tuned', 'j100_0921_newton_E.jsonl', 'newton_E_sweep.log', '#2a78d6'),
    ('Newton before: box ground, previous gains', 'j100_0921_newton_C.jsonl', 'newton_C_sweep.log', '#eb6834'),
    ('Newton now, controller off', 'j100_0921_newton_ol_plane.jsonl', None, '#1baf7a'),
]
SWEEP_DIR = sys.argv[2] if len(sys.argv) > 2 else OUT
PANELS = [  # (title, segment label, channel, y-limits)
    ('Linear step, 0.5 m/s', 'lin 0.5', 'v', 'm/s'),
    ('In-place turn, 0.3 rad/s', 'ang 0.3', 'w', 'rad/s'),
    ('In-place turn, 0.1 rad/s', 'ang 0.1', 'w', 'rad/s'),
    ('Arc 0.3 m/s + 0.3 rad/s: yaw rate', 'arc 0.3/0.3', 'w', 'rad/s'),
]
plt.rcParams.update({'font.size': 10, 'axes.edgecolor': INK2, 'axes.labelcolor': INK2, 'xtick.color': INK2,
                     'ytick.color': INK2, 'axes.titlecolor': INK, 'axes.facecolor': SURFACE,
                     'figure.facecolor': SURFACE, 'axes.spines.top': False, 'axes.spines.right': False})


def load(name):
    """(command segments {label: (t0, t1, v, w)}, truth t, v, w) as plot.py computes them."""
    rows = [json.loads(line) for line in open(os.path.join(VEL, name))]
    cmd = [r for r in rows if r[0] == 'cmd']
    ref = np.array([r[1:5] for r in rows if r[0] == 'ref'])
    t0 = cmd[0][1]
    k = 2
    tr = ref[:, 0]
    dt = tr[2 * k:] - tr[:-2 * k]
    dx, dy = ref[2 * k:, 1] - ref[:-2 * k, 1], ref[2 * k:, 2] - ref[:-2 * k, 2]
    yaw = ref[k:-k, 3]
    dyaw = np.arctan2(np.sin(ref[2 * k:, 3] - ref[:-2 * k, 3]), np.cos(ref[2 * k:, 3] - ref[:-2 * k, 3]))
    t, v, w = tr[k:-k] - t0, (dx * np.cos(yaw) + dy * np.sin(yaw)) / dt, dyaw / dt
    segs = {}
    for i, c in enumerate(cmd):
        lab = c[4]
        if lab == 'stop':
            continue
        s = segs.setdefault(lab, [c[1] - t0, c[1] - t0, c[2], c[3]])
        s[1] = c[1] - t0 + 0.05
    return segs, t, v, w


# ---------------------------------------------------------------- figure 1: step-test segments
data = {label: load(f) for label, f, _, _ in RUNS}
fig, axes = plt.subplots(2, 2, figsize=(12, 7.2))
for ax, (title, seg, ch, unit) in zip(axes.flat, PANELS):
    t_cmd = None
    for label, _, _, color in RUNS:
        segs, t, v, w = data[label]
        if seg not in segs:
            continue
        a, b, cv, cw = segs[seg]
        m = (t >= a - 0.5) & (t <= b + 2.0)
        ax.plot(t[m] - a, (v if ch == 'v' else w)[m], color=color, lw=2, label=label, solid_capstyle='round')
        if t_cmd is None:
            t_cmd = (b - a, cv if ch == 'v' else cw)
    if t_cmd:
        dur, target = t_cmd
        ax.plot([-0.5, 0, 0, dur, dur, dur + 2.0], [0, 0, target, target, 0, 0], color=INK, lw=1.2,
                ls=(0, (4, 3)), label='command')
    ax.set_title(title, loc='left', fontsize=11)
    ax.set_xlabel('time since the command (s, sim time)')
    ax.set_ylabel(f'speed ({unit})' if ch == 'v' else f'yaw rate ({unit})')
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
handles, labels = axes.flat[0].get_legend_handles_labels()
fig.legend(handles, labels, loc='upper center', ncol=4, frameon=False, bbox_to_anchor=(0.5, 1.0))
fig.suptitle('j100_0921 under Newton: commanded vs. actual (sim ground truth), step test', y=1.045,
             fontsize=13, color=INK)
fig.tight_layout()
fig.savefig(os.path.join(OUT, 'newton_step_test.png'), dpi=130, bbox_inches='tight')

# ---------------------------------------------------------------- figure 2: sweeps
fig, axes = plt.subplots(1, 2, figsize=(12, 4.4), sharey=True)
for ax, mode, unit in zip(axes, ('lin', 'ang'), ('m/s', 'rad/s')):
    ax.axhspan(0.9, 1.1, color=GRID, alpha=0.6, lw=0, label='±10 % tolerance')
    ax.axhline(1.0, color=INK2, lw=0.8)
    for label, _, log, color in RUNS:
        if log is None or not os.path.exists(os.path.join(SWEEP_DIR, log)):
            continue
        pts = [(abs(float(c)), float(r)) for c, r in
               re.findall(rf'^{mode}\s+([+-][\d.]+)\s+[+-][\d.]+\s+([\d.]+)', open(os.path.join(SWEEP_DIR, log)).read(), re.M)]
        if pts:
            x, y = zip(*sorted(pts))
            ax.plot(x, y, color=color, lw=2, marker='o', ms=8, mec=SURFACE, mew=2, label=label)
    ax.set_title('Linear speed' if mode == 'lin' else 'Yaw rate (turn in place)', loc='left', fontsize=11)
    ax.set_xlabel(f'commanded ({unit})')
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    ax.set_ylim(0, 1.25)
axes[0].set_ylabel('achieved / commanded')
handles, labels = axes[1].get_legend_handles_labels()
fig.legend(handles, labels, loc='upper center', ncol=4, frameon=False, bbox_to_anchor=(0.5, 1.02))
fig.suptitle('j100_0921 under Newton: steady-state speed sweep (calibrate_velocity.py)', y=1.1, fontsize=13,
             color=INK)
fig.tight_layout()
fig.savefig(os.path.join(OUT, 'newton_sweep.png'), dpi=130, bbox_inches='tight')
print('wrote', os.path.join(OUT, 'newton_step_test.png'), os.path.join(OUT, 'newton_sweep.png'))

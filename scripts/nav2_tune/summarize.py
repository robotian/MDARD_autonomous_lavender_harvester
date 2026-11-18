#!/usr/bin/env python3
"""Summarize nav2_bench.py results: one table per run, then one line per run.

    scripts/nav2_tune/summarize.py <label | results.json> ...   (a label = sim/generated/nav2_tune/<label>/results.json)

Legs whose name starts with `warmup` are left out (they bring the robot to the course start), and so are legs marked
`synthetic` in the course (not something the coordinator sends) unless --synthetic. A leg counts as succeeded only if
Nav2 said so and it took at most 300 % of its estimated time (nav2_bench.py's est_s; it cancels slower legs itself).
Errors are the ground-truth distance / heading to the goal 2 s after the result (nav2_bench.py --settle)."""
import argparse
import json
import os
import statistics as st

FACTOR = 3.0


def load(arg):
    path = arg if arg.endswith('.json') else os.path.join('sim/generated/nav2_tune', arg, 'results.json')
    with open(path) as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--synthetic', action='store_true', help='count synthetic legs too')
    ap.add_argument('--brief', action='store_true', help='only the one-line summaries')
    a = ap.parse_args()
    lines = []
    for run in a.runs:
        d = load(run)
        legs = [leg for leg in d['legs'] if not leg['name'].startswith('warmup')
                and (a.synthetic or not leg.get('synthetic'))]
        for leg in legs:
            if leg['status'] == 'SUCCEEDED' and leg.get('est_s') and leg['time_s'] > FACTOR * leg['est_s']:
                leg['status'] = 'SLOW>300%'
        ok = [leg for leg in legs if leg['status'] == 'SUCCEEDED']
        if not a.brief:
            print(f"\n== {d.get('label', run)}")
            print(f"{'leg':<17}{'status':<10}{'time':>7}{'est':>6}{'pos m':>7}{'yaw°':>6}{'rev m':>7}{'revmax':>7}"
                  f"{'spin':>6}{'rec':>4}{'path':>6}")
            for leg in legs:
                print(f"{leg['name']:<17}{leg['status']:<10}{leg.get('time_s', 0):7.1f}{leg.get('est_s', 0):6.1f}"
                      f"{leg.get('pos_err_m', 0):7.3f}{leg.get('yaw_err_deg', 0):6.1f}{leg.get('reverse_m', 0):7.2f}"
                      f"{leg.get('reverse_max_m', 0):7.2f}{leg.get('spin_rad', 0):6.2f}{leg.get('recoveries', 0):4d}"
                      f"{leg.get('path_m', 0):6.1f}")
        line = f"{d.get('label', run)}: {len(ok)}/{len(legs)} legs succeeded"
        if ok:
            line += (f", time {sum(leg['time_s'] for leg in legs):.1f} s (succeeded {sum(leg['time_s'] for leg in ok):.1f} s)"
                     f", xy err mean {st.mean(leg['pos_err_m'] for leg in ok):.3f} max {max(leg['pos_err_m'] for leg in ok):.3f} m"
                     f", yaw err mean {st.mean(leg['yaw_err_deg'] for leg in ok):.1f} max {max(leg['yaw_err_deg'] for leg in ok):.1f} deg"
                     f", reverse {sum(leg['reverse_m'] for leg in legs):.2f} m (longest {max(leg['reverse_max_m'] for leg in legs):.2f})"
                     f", in-place rotation {sum(leg['spin_rad'] for leg in legs):.2f} rad"
                     f", recoveries {sum(leg['recoveries'] for leg in legs)}")
        lines.append(line)
    print()
    print('\n'.join(lines))


if __name__ == '__main__':
    main()

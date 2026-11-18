#!/usr/bin/env python3
"""Build a nav2_bench.py course from the coordinator's own routes: each leg is the waypoint list status_server
publishes between two graph nodes (status_server/tools/path_check.py `path A B --waypoints`, run in the base station
container against the farm database), minus the first waypoint (the executor replaces it with the robot's pose).

    scripts/nav2_tune/course.py > scripts/nav2_tune/course_lavender_farm.json
    scripts/nav2_tune/course.py --leg lane_run:97:117 --leg back:220:909 > my_course.json

Default legs (the lavender farm graph): Staging 909 -> a harvest pickup in lane 1 (94), 0.45 m and 0.9 m harvest
steps, the rest of the lane, the U-turn at the east end into the same lane's south side (41), the ring to lane 3
(220), and lane 3 back to Staging. A `warmup_to_staging` leg first brings the robot to the start, and a synthetic
`back_0.5` leg last asks for a goal 0.5 m straight behind (the graph never does: its edges are one-way)."""
import argparse
import json
import math
import re
import subprocess
import sys

LEGS = [('staging_to_lane', 909, 94), ('step_0.45', 94, 95), ('step_0.9', 95, 97), ('lane_run', 97, 117),
        ('uturn_same_lane', 117, 41), ('ring_to_lane3', 41, 220), ('lane_to_staging', 220, 909)]
POSE = r'x=\s*(-?[\d.]+) y=\s*(-?[\d.]+) theta=\s*(-?[\d.]+)'


def waypoints(container, a, b, raw=False):
    """The published waypoints from node a to node b (raw: the nodes with their stored headings)."""
    out = subprocess.run(['docker', 'exec', container, 'bash', '-c',
                          f'cd /home/robot/colcon_ws/src/status_server && python3 tools/path_check.py path {a} {b} --waypoints'],
                         capture_output=True, text=True, check=True).stdout
    if raw or 'Published waypoints' not in out:
        section = out.split('Raw node headings', 1)[1].split('Published waypoints', 1)[0]
    else:
        section = out.split('Published waypoints', 1)[1]
    return [(float(x), float(y), float(t)) for x, y, t in re.findall(POSE, section)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--leg', action='append', default=[], help='name:FROM_NODE:TO_NODE (replaces the default legs)')
    ap.add_argument('--container', default='basestation')
    ap.add_argument('--no-extras', action='store_true', help='no warmup / synthetic reverse leg')
    a = ap.parse_args()
    legs = [(n, int(f), int(t)) for n, f, t in (s.split(':') for s in a.leg)] if a.leg else LEGS
    out = []
    for name, f, t in legs:
        pts = waypoints(a.container, f, t)
        out.append({'name': name, 'from': f, 'to': t,
                    'poses': [[round(x, 4), round(y, 4), round(math.degrees(th), 2)] for x, y, th in pts[1:]]})
        print(f'{name}: {f} -> {t}, {len(pts) - 1} poses', file=sys.stderr)
    if not a.no_extras:
        start = waypoints(a.container, legs[0][1], legs[0][2], raw=True)[0]  # the node's own heading
        out.insert(0, {'name': 'warmup_to_staging', 'poses': [[start[0], start[1], round(math.degrees(start[2]), 2)]]})
        # back where the course ends (Staging for the default legs), 0.5 m straight behind it
        last = out[-1]['poses'][-1]
        yaw = math.radians(last[2])
        out.append({'name': 'back_0.5', 'synthetic': True,
                    'poses': [[round(last[0] - 0.5 * math.cos(yaw), 4), round(last[1] - 0.5 * math.sin(yaw), 4), last[2]]]})
    json.dump({'legs': out}, sys.stdout)
    print()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Nav2 benchmark for one robot: drive a list of NavigateThroughPoses legs and measure each one.

Runs inside a robot container (`docker exec <ns> bash -c 'python3 /scripts/nav2_tune/nav2_bench.py course.json --label X'`;
scripts/nav2_tune/trial.sh does the whole trial),
on the node clock (sim time in the sim). Ground truth = `ref_pose` (nav_msgs/Odometry in the map frame: the sim's
true pose; on a real robot, mocap/GPS). Per leg it records:

  time_s          goal accepted -> result (node clock)
  pos_err_m       |final xy - goal xy| after `settle` seconds, yaw_err_deg likewise (wrapped)
  reverse_m       distance driven backwards (ground-truth motion along the heading), reverse_max_m the longest
                  continuous backward stretch
  spin_rad        heading change while (nearly) standing: |v| < 0.03 m/s (in-place rotation)
  path_m          distance travelled, recoveries = Nav2's number_of_recoveries
  est_s           estimated time = route length (robot -> each pose) / --est-speed + --est-overhead; a leg still
                  running after --time-factor (3) x est_s is cancelled and counts as TIMEOUT (a failure)

Routes file (JSON): {"legs": [{"name": "...", "poses": [[x, y, yaw_deg], ...]}, ...]}; the last pose is the goal.
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

TIPPED_DEG = 30.0
STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Bench(Node):
    def __init__(self, ns):
        super().__init__('nav2_bench', namespace=ns,
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pose = None  # (t, x, y, yaw)
        self.max_tilt = 0.0
        self.tilt = 0.0  # deg between the body's z axis and the world's: a tipped robot ends the course
        self.track = None  # list of samples while a leg runs
        self.create_subscription(Odometry, 'ref_pose', self.on_pose, 20)
        self.client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.recoveries = 0

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg):
        p = msg.pose.pose
        q = p.orientation
        self.tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))  # body z vs world z
        self.max_tilt = max(self.max_tilt, self.tilt)
        self.pose = (self.now(), p.position.x, p.position.y, yaw_of(p.orientation))
        if self.track is not None:
            self.track.append(self.pose)

    def spin_until(self, ex, cond, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and not cond() and time.monotonic() < end:
            ex.spin_once(timeout_sec=0.05)
        return cond()

    def sleep(self, ex, seconds):
        """Node-clock sleep (sim time)."""
        t0 = self.now()
        self.spin_until(ex, lambda: self.now() - t0 >= seconds, 10 * seconds + 30)

    def pose_msg(self, x, y, yaw_deg):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
        m.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
        return m

    def estimate(self, leg, speed, overhead):
        """Estimated time: route length (robot -> every pose) at `speed`, plus `overhead` for start/stop/turns."""
        pts = [self.pose[1:3]] + [p[:2] for p in leg['poses']]
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        return length / speed + overhead, length

    def run_leg(self, ex, leg, timeout, settle):
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self.pose_msg(*p) for p in leg['poses']]
        self.recoveries = 0
        self.max_tilt = 0.0

        def feedback(fb):
            self.recoveries = max(self.recoveries, fb.feedback.number_of_recoveries)

        start_pose = self.pose
        self.track = []
        # Nav2 can report its lifecycle nodes active a few seconds before bt_navigator accepts goals: retry
        for attempt in range(4):
            fut = self.client.send_goal_async(goal, feedback_callback=feedback)
            if self.spin_until(ex, fut.done, 30) and fut.result().accepted:
                break
            self.sleep(ex, 5.0)
        else:
            self.track = None
            return {'name': leg['name'], 'status': 'REJECTED'}
        t0 = self.now()
        handle = fut.result()
        res_fut = handle.get_result_async()
        wall_end = time.monotonic() + timeout * 4 + 60
        while rclpy.ok() and not res_fut.done():
            ex.spin_once(timeout_sec=0.05)
            if self.now() - t0 > timeout or time.monotonic() > wall_end or self.tilt > TIPPED_DEG:
                handle.cancel_goal_async()
                self.spin_until(ex, res_fut.done, 15)
                break
        t_end = self.now()
        status = STATUS.get(res_fut.result().status, str(res_fut.result().status)) if res_fut.done() else 'TIMEOUT'
        if self.now() - t0 > timeout:
            status = 'TIMEOUT'
        if self.tilt > TIPPED_DEG:
            status = 'TIPPED'
        err_code = getattr(res_fut.result().result, 'error_code', 0) if res_fut.done() else -1
        at_result = self.pose
        self.sleep(ex, settle)
        track, self.track = self.track, None
        gx, gy, gyaw = leg['poses'][-1]

        def errors(p):
            return math.hypot(p[1] - gx, p[2] - gy), math.degrees(abs(wrap(p[3] - math.radians(gyaw))))

        # motion metrics from ground truth: forward/backward along the heading, rotation while standing
        path = rev = rev_run = rev_max = spin = 0.0
        for a, b in zip(track, track[1:]):
            dt = b[0] - a[0]
            if dt <= 0:
                continue
            dx, dy = b[1] - a[1], b[2] - a[2]
            ds = math.hypot(dx, dy)
            along = dx * math.cos(a[3]) + dy * math.sin(a[3])
            path += ds
            if along < -1e-4:
                rev += -along
                rev_run += -along
                rev_max = max(rev_max, rev_run)
            elif along > 1e-4:
                rev_run = 0.0
            if ds / dt < 0.03:
                spin += abs(wrap(b[3] - a[3]))
        # peak acceleration / deceleration along the heading, speeds over ~0.2 s windows
        spd = []
        i = 0
        for j in range(len(track)):
            while track[j][0] - track[i][0] > 0.2:
                i += 1
            if j > i and track[j][0] - track[i][0] > 0.1:
                a_, b_ = track[i], track[j]
                spd.append(((a_[0] + b_[0]) / 2, ((b_[1] - a_[1]) * math.cos(a_[3]) + (b_[2] - a_[2]) * math.sin(a_[3]))
                            / (b_[0] - a_[0])))
        acc = [(b_[1] - a_[1]) / (b_[0] - a_[0]) for a_, b_ in zip(spd[::4], spd[4::4]) if b_[0] > a_[0]]
        pe, ye = errors(self.pose)
        pe0, ye0 = errors(at_result)
        return {
            'name': leg['name'], 'status': status, 'error_code': err_code, 'timeout_s': round(timeout, 1),
            'time_s': round(t_end - t0, 2),
            'pos_err_m': round(pe, 4), 'yaw_err_deg': round(ye, 2),
            'pos_err_at_result_m': round(pe0, 4), 'yaw_err_at_result_deg': round(ye0, 2),
            'path_m': round(path, 2), 'reverse_m': round(rev, 3), 'reverse_max_m': round(rev_max, 3),
            'spin_rad': round(spin, 2), 'recoveries': self.recoveries,
            'peak_accel_mps2': round(max(acc, default=0.0), 2), 'peak_decel_mps2': round(-min(acc, default=0.0), 2),
            'max_tilt_deg': round(self.max_tilt, 1),
            'start': [round(v, 3) for v in start_pose[1:3]] + [round(math.degrees(start_pose[3]), 1)],
            'end': [round(v, 3) for v in self.pose[1:3]] + [round(math.degrees(self.pose[3]), 1)],
            'track': [[round(s[0] - t0, 2), round(s[1], 3), round(s[2], 3), round(s[3], 3)]
                      for s in track[::4]],
        }


def upright(ns):
    rclpy.init()
    node = Bench(ns)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    got = []
    node.create_subscription(Odometry, 'ref_pose', got.append, 5)
    node.spin_until(ex, lambda: bool(got), 20)
    ex.shutdown()
    rclpy.try_shutdown()
    if not got:
        print('no ref_pose')
        return 2
    p = got[-1].pose.pose
    q = p.orientation
    roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
    ok = abs(roll) < 10 and abs(pitch) < 10
    print(f'pose ({p.position.x:.2f}, {p.position.y:.2f}, {p.position.z:.2f}) roll {roll:.1f} pitch {pitch:.1f} deg: '
          f'{"upright" if ok else "NOT upright"}')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('routes', nargs='?')
    ap.add_argument('--upright', action='store_true',
                    help='only check that the robot stands upright (|roll|, |pitch| < 10 deg): exit 0 / 1')
    ap.add_argument('--ns', default=None, help='robot namespace (default $ROBOT_NAMESPACE)')
    ap.add_argument('--label', default='run')
    ap.add_argument('--out', default=None, help='results JSON (default /tmp/nav2_bench_<label>.json)')
    ap.add_argument('--timeout', type=float, default=600.0, help='per leg, node-clock seconds (upper bound)')
    ap.add_argument('--settle', type=float, default=2.0)
    ap.add_argument('--time-factor', type=float, default=3.0,
                    help='a leg fails (TIMEOUT) after this x its estimated time (0: only --timeout)')
    ap.add_argument('--est-speed', type=float, default=0.5, help='m/s for the estimate (the speed limit)')
    ap.add_argument('--est-overhead', type=float, default=5.0, help='s added to the estimate per leg')
    ap.add_argument('--legs', default=None, help='comma-separated leg names to run (default all)')
    a = ap.parse_args()
    ns = a.ns or os.environ.get('ROBOT_NAMESPACE', '')
    if a.upright:
        return upright(ns)
    with open(a.routes) as f:
        legs = json.load(f)['legs']
    if a.legs:
        keep = a.legs.split(',')
        legs = [leg for leg in legs if leg['name'] in keep]
    rclpy.init()
    node = Bench(ns)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    results = []
    out = a.out or f'/tmp/nav2_bench_{a.label}.json'
    try:
        if not node.spin_until(ex, lambda: node.pose is not None, 30):
            sys.exit('no ref_pose')
        if not node.client.wait_for_server(timeout_sec=60):
            sys.exit('no navigate_through_poses action server')
        for leg in legs:
            est, length = node.estimate(leg, a.est_speed, a.est_overhead)
            timeout = min(a.timeout, a.time_factor * est) if a.time_factor > 0 else a.timeout
            r = node.run_leg(ex, leg, timeout, a.settle)
            r['est_s'], r['route_m'] = round(est, 1), round(length, 2)
            if leg.get('synthetic'):
                r['synthetic'] = True
            results.append(r)
            print(f"{a.label} {r['name']:<14} {r['status']:<9} t={r.get('time_s', 0):6.1f}s "
                  f"err={r.get('pos_err_m', 0):.3f}m/{r.get('yaw_err_deg', 0):5.1f}deg "
                  f"rev={r.get('reverse_m', 0):.2f}m(max {r.get('reverse_max_m', 0):.2f}) "
                  f"spin={r.get('spin_rad', 0):.2f}rad rec={r.get('recoveries', 0)} path={r.get('path_m', 0):.1f}m",
                  flush=True)
            with open(out, 'w') as f:
                json.dump({'label': a.label, 'legs': results}, f)
            if r['status'] == 'TIPPED':
                print(f'{a.label}: the robot tipped over (tilt {node.tilt:.0f} deg): course ended', flush=True)
                break
            node.sleep(ex, 1.0)
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())

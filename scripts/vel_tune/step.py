#!/usr/bin/env python3
"""Step test of the sim drive: publish cmd_vel steps, record commanded vs actual. Run inside the robot container
(scripts/vel_tune/step_test.sh does it): python3 step.py out.jsonl, VEL_SEGMENTS=full|user|screen.

full (default): straight 0.5, 0.3 and -0.2 m/s, in place 0.3 and 0.1 rad/s, arcs 0.3/0.3 and 0.5/0.5, each held a few
seconds with STOP s of zero between; user: 0.3 m/s, 0.3 rad/s, both (8 s each); screen: a short open-loop check.
Records the commanded twist, the sim's ground truth (ref_pose, map frame) and platform/odom (pose + twist) to a
JSON-lines file. The arm is put in its SRDF 'stow' state first (as scripts/calibrate_velocity.py does). Sim only.
"""
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

NS = os.environ['ROBOT_NAMESPACE']
OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/vel_step.jsonl'
STOP, RATE = 2.5, 20.0
# (label, v, w, hold s). Straight segments first, while the robot still faces open ground east of its spawn pose.
SEGMENTS = [('lin 0.5', 0.5, 0.0, 4.0), ('lin 0.3', 0.3, 0.0, 6.0), ('lin -0.2 (reverse)', -0.2, 0.0, 5.0),
            ('ang 0.3', 0.0, 0.3, 6.0), ('ang 0.1', 0.0, 0.1, 6.0), ('arc 0.3/0.3', 0.3, 0.3, 8.0),
            ('arc 0.5/0.5', 0.5, 0.5, 6.0)]
if os.environ.get('VEL_SEGMENTS') == 'screen':
    SEGMENTS = [('lin 0.3', 0.3, 0.0, 4.0), ('ang 0.3', 0.0, 0.3, 5.0), ('ang 0.6', 0.0, 0.6, 4.0),
                ('arc 0.3/0.3', 0.3, 0.3, 5.0)]
if os.environ.get('VEL_SEGMENTS') == 'user':
    SEGMENTS = [('linear 0.3 m/s', 0.3, 0.0, 8.0), ('angular 0.3 rad/s', 0.0, 0.3, 8.0),
                ('linear 0.3 + angular 0.3', 0.3, 0.3, 8.0)]

rclpy.init()
node = rclpy.create_node('vel_step', parameter_overrides=[Parameter('use_sim_time', value=True)])
pub = node.create_publisher(TwistStamped, f'/{NS}/cmd_vel', 10)
out = open(OUT, 'w')
joints = {}


def now():
    return node.get_clock().now().nanoseconds * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def tilt(q):
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))


def on_ref(m):
    p = m.pose.pose
    out.write(json.dumps(['ref', now(), p.position.x, p.position.y, yaw(p.orientation), tilt(p.orientation)]) + '\n')


def on_odom(m):
    p, t = m.pose.pose, m.twist.twist
    stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    out.write(json.dumps(['odom', now(), stamp, p.position.x, p.position.y, yaw(p.orientation),
                          t.linear.x, t.angular.z]) + '\n')


def on_joints(m):
    joints.update(zip(m.name, m.position))


node.create_subscription(Odometry, f'/{NS}/ref_pose', on_ref, 50)
node.create_subscription(Odometry, f'/{NS}/platform/odom', on_odom, 50)
node.create_subscription(JointState, f'/{NS}/platform/joint_states', on_joints, 10)


def spin(seconds):
    end = now() + seconds
    while now() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def send(v, w, label):
    m = TwistStamped()
    m.header.stamp = node.get_clock().now().to_msg()
    m.header.frame_id = 'base_link'
    m.twist.linear.x, m.twist.angular.z = float(v), float(w)
    pub.publish(m)
    out.write(json.dumps(['cmd', now(), v, w, label]) + '\n')


def hold(v, w, seconds, label):
    end = now() + seconds
    while now() < end:
        send(v, w, label)
        spin(1.0 / RATE)


def stow():
    try:
        state = next(g for g in ET.parse('/etc/clearpath/robot.srdf').getroot().findall('group_state')
                     if g.get('name') == 'stow' and g.get('group') == 'arm_0')
    except (OSError, ET.ParseError, StopIteration):
        print('no arm stow state: skipped')
        return
    target = {j.get('name'): float(j.get('value')) for j in state.findall('joint')}
    spin(1.0)
    if not all(n in joints for n in target):
        print('arm joints not seen: stow skipped')
        return
    cmd = node.create_publisher(JointState, f'/{NS}/arm_0/joint_command', 10)
    while cmd.get_subscription_count() == 0:
        spin(0.2)
    start = {n: joints[n] for n in target}
    t0 = now()
    while now() - t0 < 8.0:
        f = 0.5 - 0.5 * math.cos(math.pi * min(1.0, (now() - t0) / 6.0))
        m = JointState()
        m.name = list(target)
        m.position = [start[n] + (target[n] - start[n]) * f for n in target]
        cmd.publish(m)
        spin(0.05)
    print(f'arm stowed (max joint error {max(abs(joints[n] - target[n]) for n in target):.3f} rad)')


while now() == 0.0:
    spin(0.1)
stow()
while pub.get_subscription_count() == 0:
    spin(0.2)
hold(0.0, 0.0, 2.0, 'stop')
for label, v, w, hold_s in SEGMENTS:
    print(f'{now():.1f} {label}', flush=True)
    hold(v, w, hold_s, label)
    hold(0.0, 0.0, STOP, 'stop')
out.close()
rclpy.try_shutdown()
print('done', OUT)

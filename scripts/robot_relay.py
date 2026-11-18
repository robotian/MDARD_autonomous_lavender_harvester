#!/usr/bin/env python3
"""Relay the operation coordinator's topics between one real robot and the base station, without linking their
zenoh routers. Runs in the base station container (scripts/ is mounted there as /scripts), one per robot, started by
scripts/basestation_relay.sh from the setting BASESTATION_RELAYS.

  python3 /scripts/robot_relay.py a300_00036 tcp/192.168.50.65:7447

Why: with the base station's router linked to every robot's (BASESTATION_ZENOH_CONNECT), a robot's restart floods
the links with its whole graph (~200 topics, their services, actions and liveliness tokens) and they clog: "Unable
to push non droppable network message ... Closing transport!", the robot's router passing nothing off the robot for
minutes, and the other robots' data held too (2026-10-19). The coordinator needs three topics per robot. Here each
crosses as an ordinary client session of the robot's own router (like the web UI's link monitor and RViz), so a
robot's restart stays on that robot.

  robot -> base station   status/robot         status_interfaces/msg/RobotStatus   reliable, depth 10
                          platform/bms/state   sensor_msgs/msg/BatteryState        best effort
  base station -> robot   status/task          status_interfaces/msg/Task          reliable, depth 10

One process can't hold two zenoh sessions with different configs, so this one runs two children of itself: --side
robot (a client of the robot's router) and --side base (a client of the base station's own router, the container's
default). They pass serialized messages through pipes (frame: 4-byte length, 1-byte topic index, the bytes), so
nothing is deserialized here. Each topic crosses in one direction only: no loops.

The pair is restarted when a child exits (the robot's router down, a session that failed to open) and when nothing
came from the robot for STALE_S (a client session can stay open across a router restart and receive nothing),
waiting RETRY_S, doubling up to RETRY_MAX_S while runs deliver nothing. Log lines on stdout: /tmp/relay_<robot>.log.
"""
import argparse
import os
import signal
import struct
import subprocess
import sys
import threading
import time

TOPICS = (  # (name in the robot's namespace, type, reliable, from the robot)
    ("status/robot", "status_interfaces/msg/RobotStatus", True, True),
    ("platform/bms/state", "sensor_msgs/msg/BatteryState", False, True),
    ("status/task", "status_interfaces/msg/Task", True, False),
)
FRAME = struct.Struct(">IB")
STALE_S = 60  # the battery state comes at 1-3 Hz on every robot
RETRY_S, RETRY_MAX_S = 3, 300


def log(msg):
    print(f"[relay] {time.strftime('%H:%M:%S')} {msg}", flush=True)


# ---------------------------------------------------------------- a child: one ROS session

def side_main(robot, side):
    import importlib

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

    def msg_type(name):
        pkg, _, cls = name.split("/")
        return getattr(importlib.import_module(f"{pkg}.msg"), cls)

    # Frames go out on a private copy of stdout; fd 1 becomes stderr, since zenoh logs to stdout ("Unable to connect")
    # and those bytes, read as a frame, made a failed run look like it delivered: no backoff, a restart every ~12 s
    out, write_lock = os.fdopen(os.dup(1), "wb"), threading.Lock()
    os.dup2(2, 1)
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=false"])
    node = rclpy.create_node(f"relay_{side}", namespace=f"/{robot}")
    publishers = {}
    for i, (name, type_name, reliable, from_robot) in enumerate(TOPICS):
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE) if reliable else qos_profile_sensor_data
        if from_robot == (side == "robot"):  # this side receives it and sends it over
            def forward(data, i=i):
                with write_lock:
                    out.write(FRAME.pack(len(data), i) + data)
                    out.flush()
            node.create_subscription(msg_type(type_name), name, forward, qos, raw=True)
        else:
            publishers[i] = node.create_publisher(msg_type(type_name), name, qos)

    def read_frames():
        src = sys.stdin.buffer
        while True:
            head = src.read(FRAME.size)
            if len(head) < FRAME.size:
                break
            size, i = FRAME.unpack(head)
            data = src.read(size)
            if len(data) < size:
                break
            publishers[i].publish(data)
        os.kill(os.getpid(), signal.SIGTERM)  # the other side is gone: this pair is done

    threading.Thread(target=read_frames, daemon=True).start()
    signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        rclpy.try_shutdown()


# ---------------------------------------------------------------- the parent: a pair of children, restarted

def run_pair(robot, endpoint):
    """One pair until a child exits or the robot side goes quiet. Returns the number of frames from the robot."""
    me = [sys.executable, "-u", os.path.abspath(__file__), robot, endpoint, "--side"]
    robot_env = dict(os.environ, ZENOH_CONFIG_OVERRIDE=f'mode="client";connect/endpoints=["{endpoint}"]')
    robot_side = subprocess.Popen(me + ["robot"], env=robot_env, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    base_side = subprocess.Popen(me + ["base"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    counts, last = {"up": 0, "down": 0}, {"t": time.time()}

    def pump(src, dst, key):
        while True:
            head = src.read(FRAME.size)
            if len(head) < FRAME.size:
                break
            size = FRAME.unpack(head)[0]
            data = src.read(size)
            if len(data) < size:
                break
            try:
                dst.write(head + data)
                dst.flush()
            except (BrokenPipeError, ValueError):
                break
            counts[key] += 1
            if key == "up":
                last["t"] = time.time()

    for src, dst, key in ((robot_side.stdout, base_side.stdin, "up"), (base_side.stdout, robot_side.stdin, "down")):
        threading.Thread(target=pump, args=(src, dst, key), daemon=True).start()
    reported, why = 0, ""
    while not why:
        time.sleep(1)
        if robot_side.poll() is not None:
            why = f"the robot side exited ({robot_side.returncode}): its router unreachable?"
        elif base_side.poll() is not None:
            why = f"the base station side exited ({base_side.returncode})"
        elif time.time() - last["t"] > STALE_S:
            why = f"nothing from {robot} for {STALE_S} s"
        if counts["up"] and not reported:
            log(f"relaying: first message from {robot}")
            reported = 1
    for p in (robot_side, base_side):
        if p.poll() is None:
            p.terminate()
    for p in (robot_side, base_side):
        try:
            p.wait(5)
        except subprocess.TimeoutExpired:
            p.kill()
    log(f"stopped: {why} ({counts['up']} messages from the robot, {counts['down']} to it)")
    return counts["up"]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("robot", help="the robot's namespace, e.g. a300_00036")
    ap.add_argument("endpoint", help="its zenoh router, e.g. tcp/192.168.50.65:7447")
    ap.add_argument("--side", choices=("robot", "base"), help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.side:
        return side_main(args.robot, args.side)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    log(f"{args.robot} via {args.endpoint}: " + ", ".join(f"{n} ({'from' if up else 'to'} the robot)"
                                                          for n, _, _, up in TOPICS))
    retry = RETRY_S
    while True:
        delivered = run_pair(args.robot, args.endpoint)
        retry = RETRY_S if delivered else min(retry * 2, RETRY_MAX_S)
        log(f"restarting in {retry} s")
        time.sleep(retry)


if __name__ == "__main__":
    main()

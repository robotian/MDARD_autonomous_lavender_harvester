#!/usr/bin/env python3
"""Local web UI for the fleet: the simulated robots, the real ones, or both (the page's mode).

Sim: start/stop/reset the sim (the scene only), spawn robots at chosen poses into the running scene (and start
their containers), start/stop mtu32_bringup's sim_robot_upstart.launch.py per robot.
Real (the real_robot table of the settings database, reached over SSH like scripts/deploy_robot.sh): Clearpath's services and
restarting them, deploying colcon_ws/src, linking the robot to the base station's zenoh router.
Either: move the arm to named SRDF states (optionally recording commanded vs. observed joint positions while it
moves), cut_stem, stop motion, RViz, each robot's ref_localizer (map -> odom source / anchor), OptiTrack Motive's
rigid bodies and which robot follows each.
Configuration page (/config): the settings, robot slots and profiles in this checkout's fleet_config.sqlite
(scripts/fleetcfg.py), which .env is generated from; their change log and runs; what the running containers differ in.

  python3 tools/sim_ui/server.py                 # http://127.0.0.1:8090
  python3 tools/sim_ui/server.py --mode real     # the page's mode until the browser picks its own
  python3 tools/sim_ui/server.py --host 0.0.0.0  # reachable from the LAN -- it runs docker commands, so only on a trusted network
  FLEET_ROOT=<checkout> python3 server.py        # drive another checkout's fleet (e.g. this UI from a worktree)

Stdlib only (the settings database is SQLite; if its file can't be opened, the page runs on .env as last written
and refuses setting changes). Long operations run as background jobs whose output the page polls.
"""
import argparse
import base64
import collections
import csv
import datetime
import hashlib
import importlib
import json
import math
import os
import re
import select
import shlex
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(os.environ.get("FLEET_ROOT") or Path(__file__).resolve().parents[2]).resolve()
sys.path.insert(0, str(ROOT / "scripts"))
import fleet_ctl  # noqa: E402  (the sim's spawn protocol: request/state files)
import fleetcfg  # noqa: E402  (the settings: this checkout's fleet_config.sqlite, .env generated from it)
sys.path.insert(0, str(ROOT / "colcon_ws/src/mocap_fake_localizer/scripts"))
import natnet  # noqa: E402  (OptiTrack Motive's NatNet protocol, shared with the robots' natnet_ref_pose.py)

PAGE = Path(__file__).with_name("index.html")
PROJECT = "clearpath-fleet"
SIM = "a300-isaac-sim"
BASESTATION = "basestation"  # basestation.compose.yml's container (host network); real robots' RViz runs there
MAX_SLOTS = 8
MODES = ("sim", "real", "both")
DEFAULT_MODE = "sim"  # --mode
NAME_RE = re.compile(r"^[A-Za-z0-9_]+$")
# Commands run as `bash -c '<command>'` (docker exec, or ssh's remote shell), whose own command line then holds every
# pgrep/pkill pattern in it: each pattern's first letter is bracketed ("[r]os2") so it can't match that shell.
# The cut_stem client (`ros2 action send_goal ... /<ns>/cut_stem`) is exec'd directly (no wrapper shell), so this
# matches exactly one process per run; SIGINT to it makes ros2cli cancel the goal.
CUT_MATCH = "[r]os2 action send_goal.*cut_stem"
CUT_CMD = ('exec ros2 action send_goal --feedback /$ROBOT_NAMESPACE/cut_stem '
           'plant_cutter_msgs/action/CutStem "{start_cutting: true}"')
# mocap_fake_localizer's calibrate_ref_offset.py drives the robot (forward/back, one turn); SIGINT makes it send a
# zero cmd_vel and exit. Matches both `ros2 run` and its python child. Unbuffered: the job log follows each phase.
CAL_MATCH = "[c]alibrate_ref_offset.py"
CAL_CMD = ("PYTHONUNBUFFERED=1 exec ros2 run mocap_fake_localizer calibrate_ref_offset.py --yes --apply "
           "--ros-args -r __ns:=/$ROBOT_NAMESPACE")
LAUNCH_LOG = "/tmp/sim_robot_upstart.log"
LAUNCH_MATCH = "[s]im_robot_upstart.launch.py"
SRDF = "/etc/clearpath/robot.srdf"  # written at container boot; sim_robot_upstart needs it
SRDF_WAIT_S = 120
LAUNCH_CHECK_S = 20  # a launch that fails at start says so (and exits) within this
LAUNCH_CMD = ("source /home/robot/colcon_ws/install/setup.bash && "
              f"exec ros2 launch mtu32_bringup sim_robot_upstart.launch.py > {LAUNCH_LOG} 2>&1")
# RViz from clearpath_viz (colcon_ws/src/clearpath_desktop), opened on the host display through the container's X11
# mount; one window per view and robot. The launch exits when its rviz2 window is closed. A sim robot's runs in its
# own container; a real robot's in the base station container (host network, the same colcon_ws), on wall time.
RVIZ_VIEWS = ("navigation", "moveit", "robot")
RVIZ_MATCH = r"[c]learpath_viz view_(navigation|moveit|robot)\.launch\.py"
RVIZ_CMD = ("source /home/robot/colcon_ws/install/setup.bash && "
            "exec ros2 launch clearpath_viz view_{view}.launch.py namespace:={ns} "
            "use_sim_time:={sim_time} > /tmp/rviz_{log}.log 2>&1")
# A real robot: Clearpath's systemd services (bringup_main runs from clearpath-platform-extras).
REAL_SERVICES = ("clearpath-robot", "clearpath-platform-extras", "clearpath-manipulators")
SUDOERS_HINT = ("once, on the robot: echo 'robot ALL=(root) NOPASSWD: /usr/bin/systemctl restart clearpath-robot' "
                "| sudo tee /etc/sudoers.d/fleet-ui && sudo chmod 440 /etc/sudoers.d/fleet-ui")
REAL_MAX_VELOCITY = 0.3  # arm velocity/acceleration scale cap on a real robot
# Every action goal a robot may be running from here: cut_stem, arm_goto (move_action / the trajectory and gripper
# controllers), RViz's MoveIt panel (execute_trajectory). A CancelGoal with a zero goal id and stamp cancels all.
STOP_ACTIONS = ("cut_stem", "move_action", "execute_trajectory",
                "manipulators/arm_0_joint_trajectory_controller/follow_joint_trajectory",
                "manipulators/arm_0_gripper_controller/gripper_cmd")
# ros2 service call prints "making request" once it found the server and sent it. On a300_00036 (2026-10-06) the
# move_action / trajectory controller cancels arrived (PREEMPTED 1.9 s after the button) but their replies never
# came back within the timeout: that is "sent, no reply", not "no action server".
# "waiting for service" = its ROS session is up and the server is missing; no output at all = no session (a busy or
# wedged zenoh router): not known to be stopped.
STOP_CMD = (f"pkill -INT -f '{CUT_MATCH}' && echo 'cut_stem client: Ctrl+C'; "
            "pkill -INT -f '[/ ]arm_goto( |$)' && echo 'arm_goto: Ctrl+C'; "
            f"pkill -INT -f '{CAL_MATCH}' && echo 'calibrate_ref_offset: Ctrl+C'; "
            "for a in " + " ".join(STOP_ACTIONS) + "; do ( "
            "out=$(timeout -k 2 10 ros2 service call /$ROBOT_NAMESPACE/$a/_action/cancel_goal action_msgs/srv/CancelGoal "
            "'{}' 2>&1); r=$(grep -o 'return_code=[0-9]*, goals_canceling=\\[[^]]*\\]' <<< \"$out\"); "
            "if [ -z \"$r\" ]; then if grep -q 'making request' <<< \"$out\"; then r='cancel sent, no reply in 10 s'; "
            "elif grep -q 'waiting for service' <<< \"$out\"; then r='no action server'; "
            "else r='no ROS session in 10 s'; fi; fi; echo \"$a: $r\" ) & done; wait")


def sh(argv, timeout=30, input=None):
    # stdin: `input` (e.g. a sudo password, never put on a command line), else /dev/null: ssh would otherwise read
    # the server's terminal
    stdin = {"input": input} if input is not None else {"stdin": subprocess.DEVNULL}
    p = subprocess.run(argv, cwd=ROOT, capture_output=True, text=True, timeout=timeout, **stdin)
    return p.returncode, p.stdout + p.stderr


def arm_tool(tool, args=""):
    """moveit_sim_bridge's arm_goto / arm_joints from ~/colcon_ws (the sim's shared workspace, a real robot's
    deployed one); a sim robot image from before they moved there has its own copy in /usr/local/bin."""
    exe = f"$HOME/colcon_ws/install/moveit_sim_bridge/lib/moveit_sim_bridge/{tool}"
    return (f"[ -f ~/colcon_ws/install/setup.bash ] && source ~/colcon_ws/install/setup.bash >/dev/null 2>&1; "
            f"if [ -x {exe} ]; then exec ros2 run moveit_sim_bridge {tool} {args}; fi; "
            f"command -v {tool} >/dev/null || {{ echo 'no {tool} here: build/deploy colcon_ws (moveit_sim_bridge)'; "
            f"exit 127; }}; exec {tool} {args}")


# ---------------------------------------------------------------- robots: sim containers and real robots

class SimTarget:
    """A sim robot: its container, `docker exec` (bash -c, so BASH_ENV sources ROS and ROBOT_NAMESPACE)."""
    kind = "sim"
    cutter = True  # every sim robot runs the cutter stack (pruner_stub); cut_stem is gated on its action server

    def __init__(self, name):
        self.name = name

    def argv(self, command):
        return ["docker", "exec", self.name, "bash", "-c", command]

    def run(self, command, timeout=30):
        return sh(self.argv(command), timeout=timeout)


SSH_DIR = Path(f"/tmp/fleet-ui-ssh-{os.getuid()}")  # ControlMaster sockets: one connection per robot, reused
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=5",
            "-o", "ServerAliveCountMax=3", "-o", "ControlMaster=auto", "-o", f"ControlPath={SSH_DIR}/%C",
            "-o", "ControlPersist=120"]


class RealTarget:
    """A real robot over SSH (key login, as scripts/deploy_robot.sh): its login shell runs `bash -c` with Clearpath's
    setup.bash (ROS + its workspaces from robot.yaml) and ROBOT_NAMESPACE, which a robot's own shell doesn't set."""
    kind = "real"

    def __init__(self, name, host, user, cutter):
        self.name, self.host, self.user, self.cutter = name, host, user, cutter

    def argv(self, command, ros_env=True):
        SSH_DIR.mkdir(mode=0o700, exist_ok=True)
        # ros_env=False: the bare command (sourcing setup.bash takes ~0.5 s, too slow for timing a round trip)
        script = (f"source /etc/clearpath/setup.bash >/dev/null 2>&1; export ROBOT_NAMESPACE={self.name}; {command}"
                  if ros_env else command)
        return ["ssh", *SSH_OPTS, f"{self.user}@{self.host}", "bash -c " + shlex.quote(script)]

    def run(self, command, timeout=30, input=None, ros_env=True):
        return sh(self.argv(command, ros_env), timeout=timeout, input=input)


# Which real robots the UI shows: {"<id>": {"host": ..., "user": ..., "cutter": true}}, the settings database's
# real_robot table (fleetcfg.real_robots(); while it can't be reached, the copy it last wrote to real_robots.json).
# Host defaults to the robot's mDNS name (cpr-<id with - for _>.local, as deploy_robot.sh), user to robot, cutter
# (the stem cutter is fitted: cut_stem is offered; bringup_main advertises the action on every robot) to false.
ROBOT_ID_RE = re.compile(r"^[a-z0-9]+_[0-9]+$")  # deploy_robot.sh's
HOST_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9.-]*$")
USER_RE = re.compile(r"^[a-z_][a-z0-9_-]*$")
REAL_LOCK = threading.Lock()
REAL_CACHE_S = 5  # read by every status poll and the link monitors: one database round trip per 5 s at most
_real_cache = {"t": 0.0, "data": {}}
ACTOR = "web UI"  # who the settings database's change log names for changes made here


def default_host(robot_id):
    return f"cpr-{robot_id.replace('_', '-')}.local"


def real_config():
    with REAL_LOCK:
        if time.time() - _real_cache["t"] > REAL_CACHE_S:
            data = fleetcfg.real_robots(ACTOR)
            _real_cache.update(t=time.time(), data={k: v for k, v in data.items() if ROBOT_ID_RE.match(k)})
        return dict(_real_cache["data"])


def real_target(name, config=None):
    c = (real_config() if config is None else config).get(name)
    if c is None:
        raise ValueError(f"{name} is not in the real robots list")
    return RealTarget(name, c.get("host") or default_host(name), c.get("user") or "robot", bool(c.get("cutter")))


def known_real_ids():
    """MTU's real robots this checkout knows (robot_data/<id>/robot.yaml), offered when adding one."""
    d = ROOT / "robot_data"
    return sorted(p.parent.name for p in d.glob("*/robot.yaml") if ROBOT_ID_RE.match(p.parent.name)) if d.is_dir() else []


def act_real_save(body):
    robot = body.get("robot")
    if not (isinstance(robot, str) and ROBOT_ID_RE.match(robot)):
        raise ValueError("robot id like j100_0921")
    host = (body.get("host") or "").strip()
    user = (body.get("user") or "").strip()
    if host and not HOST_RE.match(host):
        raise ValueError("bad host")
    if user and not USER_RE.match(user):
        raise ValueError("bad user")
    fleetcfg.real_save(robot, host or default_host(robot), user or "robot", bool(body.get("cutter")), ACTOR)
    _real_cache["t"] = 0.0
    PROBES.pop(("real", robot), None)
    return {"real_robots": real_config()}


def act_real_remove(body):
    fleetcfg.real_remove(body.get("robot"), ACTOR)
    _real_cache["t"] = 0.0
    return {"real_robots": real_config()}


# One command per robot per status poll (the old per-field docker execs were 5 per robot, one after another; over
# WiFi with SSH that adds up): `key=value` lines. ros2 action list doubles as the move_group check (move_action).
# `timeout -k 2` on every ros2 CLI call here: one hung in zenoh ignores the SIGTERM (seen 2026-10-09: a `ros2 param
# get` sat for 10 min) and would stay behind in the robot, holding its session.
ACTIONS_PROBE = ('timeout -k 2 6 ros2 action list 2>/dev/null | '
                 'sed -n "s#^/$ROBOT_NAMESPACE/\\(move_action\\|cut_stem\\)\\$#action=\\1#p"')
SIM_PROBE_FMT = (f"if pgrep -f '{LAUNCH_MATCH}' >/dev/null; then echo launch=1; {{actions}}fi; "
                 f"pgrep -f '{CUT_MATCH}' >/dev/null && echo cutting=1; "
                 f"pgrep -af '{RVIZ_MATCH}' | grep -o 'view_[a-z]*' | sed 's/^view_/rviz=/'; true")
SIM_PROBE = SIM_PROBE_FMT.format(actions=f"{ACTIONS_PROBE}; ")
SIM_PROBE_NO_ACTIONS = SIM_PROBE_FMT.format(actions="")
REAL_PROBE = ("for s in " + " ".join(REAL_SERVICES) + "; do echo \"service=$s:$(systemctl is-active $s)\"; done; "
              f"pgrep -f '{CUT_MATCH}' >/dev/null && echo cutting=1; echo \"rmw=$RMW_IMPLEMENTATION\"; "
              "sed -n '1s/^multirobot_sim \\([0-9a-f]*\\).*/deployed=\\1/p' ~/colcon_ws/DEPLOYED 2>/dev/null; true")
# A ros2 CLI call costs a robot's CPU 1-5 s (5 s on a300_00036 while its zenoh router hung): on a real robot the
# actions (move_group, cut_stem) are checked this often, systemd and processes every poll.
REAL_ACTIONS_EVERY_S = 30
# Sim robots share one zenoh router, and every new session pauses the fleet's data (measured 2026-10-09: one ros2
# CLI call ~1-2 s, eight at once ~10 s, long enough for MoveIt to time out a move): their actions are checked this
# often while the launch runs and one is still missing, then every REAL_ACTIONS_EVERY_S; processes every poll.
SIM_ACTIONS_EVERY_S = 10
PROBES = {}  # (kind, name) -> {"t": time, "actions_t": time, "data": {...}}
OFFLINE_RETRY_S = 15
RETRYING = set()  # offline robots being probed again in the background


PROBE_LOCKS = {}  # (kind, name) -> Lock: overlapping polls (several tabs) wait for one probe and reuse it


def probe(target, max_age=0.0):
    key = (target.kind, target.name)
    with PROBE_LOCKS.setdefault(key, threading.Lock()):
        last = PROBES.get(key)
        if last and time.time() - last["t"] < max_age:
            return last["data"]
        if last and not last["data"]["online"]:
            # An unreachable robot costs ssh's ConnectTimeout (or a few seconds of mDNS) per try: retry it now and
            # then, in the background, so it doesn't hold up every status poll.
            if time.time() - last["t"] > OFFLINE_RETRY_S and key not in RETRYING:
                RETRYING.add(key)
                threading.Thread(target=lambda: (_probe(target, key), RETRYING.discard(key)), daemon=True).start()
            return last["data"]
        return _probe(target, key)


def _probe(target, key):
    last = PROBES.get(key) or {}
    command, actions_t = SIM_PROBE, time.time()
    if target.kind == "real":
        command = REAL_PROBE
        if time.time() - last.get("actions_t", 0) > REAL_ACTIONS_EVERY_S or not last["data"]["online"]:
            command += f"; {ACTIONS_PROBE}; true"
        else:
            actions_t = last["actions_t"]
    elif last:
        found = last["data"]["move_group"] and last["data"]["cut_stem_action"]
        if time.time() - last.get("actions_t", 0) <= (REAL_ACTIONS_EVERY_S if found else SIM_ACTIONS_EVERY_S):
            command, actions_t = SIM_PROBE_NO_ACTIONS, last["actions_t"]
    try:
        code, out = target.run(command, timeout=20)
    except subprocess.TimeoutExpired:
        code, out = -1, "no answer in 20 s"
    data = {"online": code == 0, "launch": False, "cutting": False, "move_group": False, "cut_stem_action": False,
            "rviz": [], "services": {}, "rmw": "", "deployed": "", "error": ""}
    if code != 0:
        data["error"] = (out.strip().splitlines() or [f"exit {code}"])[-1]
    for line in out.splitlines() if code == 0 else []:
        k, _, v = line.partition("=")
        if k in ("launch", "cutting"):
            data[k] = True
        elif k == "action":
            data["move_group" if v == "move_action" else "cut_stem_action"] = True
        elif k == "rviz":
            data["rviz"].append(v[len("view_"):] if v.startswith("view_") else v)
        elif k == "service":
            name, _, state = v.partition(":")
            data["services"][name] = state
        elif k in ("rmw", "deployed"):
            data[k] = v
    if target.kind == "real":
        data["launch"] = data["services"].get("clearpath-platform-extras") == "active"
    # not checked this time: the last check's (a sim robot's only while its launch runs; a real robot's move_group
    # can also come from clearpath-manipulators)
    if code == 0 and actions_t == last.get("actions_t") and (data["launch"] or target.kind == "real"):
        data["move_group"], data["cut_stem_action"] = last["data"]["move_group"], last["data"]["cut_stem_action"]
    if target.kind == "sim" and not data["launch"]:
        actions_t = 0.0  # no launch: check its actions at the first poll that sees one
    PROBES[key] = {"t": time.time(), "actions_t": actions_t, "data": data}
    return data


def running_sims(rows=None):
    return {c["name"] for c in (containers() if rows is None else rows)
            if c["state"] == "running" and re.fullmatch(r"robot\d+", c["service"])}


CONFLICT = ("{name} is both a running sim robot and a real robot that is online or linked to the base station's "
            "router: they share every ROS name and the base station bridges their networks, so a command could "
            "reach the other one. Spawn the sim with another model, or unlink the real robot (Base station card)")


def linked_reals(env=None):
    """Real robots the base station's zenoh router dials (BASESTATION_ZENOH_CONNECT has tcp/<host or its last known
    IP>:7447) or relays (BASESTATION_RELAYS): sharing one graph with the sim even while offline -- the router keeps
    dialling, the graphs join when the robot comes up; a relay carries its operation topics, under its name. Like
    fleetcfg.linked_real_robots, which refuses to start such a sim slot."""
    env = read_env() if env is None else env
    if (env.get("BASESTATION_RMW") or env.get("FLEET_RMW", "rmw_zenoh_cpp")) != "rmw_zenoh_cpp":
        return set()
    endpoints, relayed = set(zenoh_endpoints(env)), relays(env)
    return {name for name, c in real_config().items()
            if name in relayed or {f"tcp/{c['host']}:7447", f"tcp/{known_ip(c['host'])}:7447"} & endpoints}


def resolve(body, kinds=("sim", "real"), check_conflict=True):
    """The robot an action is for: body {"robot": <name>, "kind": "sim" (default) | "real"}."""
    kind, name = body.get("kind") or "sim", body.get("robot")
    if not isinstance(name, str) or not NAME_RE.match(name):
        raise ValueError("bad robot name")
    if kind not in kinds:
        raise ValueError(f"not for a {kind} robot")
    if kind == "sim":
        if name not in running_sims():
            raise ValueError(f"robot {name} is not running")
        target = SimTarget(name)
        # linked first: no ssh round trip, and an offline robot's cached probe can be minutes old
        if check_conflict and name in real_config() and (
                name in linked_reals() or probe(real_target(name), max_age=30)["online"]):
            raise ValueError(CONFLICT.format(name=name))
        return target
    target = real_target(name)
    if check_conflict and name in running_sims():
        raise ValueError(CONFLICT.format(name=name))
    return target


# ---------------------------------------------------------------- jobs

class Job:
    def __init__(self, name, lock=None):
        self.id = uuid.uuid4().hex[:8]
        self.name = name
        self.lock = lock  # see start_job
        self.status = "running"
        self.lines = []
        self.result = None
        self.started = time.time()

    def log(self, line):
        self.lines.append(line.rstrip("\n"))
        del self.lines[:-2000]

    def run(self, argv, **kw):
        # an ssh argv ends in the whole remote script: log what it runs, not the ssh options
        self.log("$ " + (f"ssh {argv[-2]} {argv[-1]}" if argv[0] == "ssh" else " ".join(argv)))
        p = subprocess.Popen(argv, cwd=ROOT, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, **kw)
        for line in p.stdout:
            self.log(line)
        return p.wait()

    def to_json(self, full=False):
        # lock: the page disables the buttons of an action whose lock a running job holds (index.html applyGates)
        d = {"id": self.id, "name": self.name, "status": self.status, "started": self.started, "lock": self.lock}
        if full:
            d["log"] = self.lines
            d["result"] = self.result
        return d


JOBS = {}
JOBS_LOCK = threading.Lock()


# Job locks: one running job per lock, so two clicks (or two tabs) can't run the same kind of job on the same thing
# at once -- two launches double a robot's nodes, a Start during a Spawn recreates the sim under it. Stop motion
# and read-only jobs take none: stopping must always get through.
FLEET_LOCK = "fleet"  # sim start/stop/reset, spawn, applying sim/robot settings: they recreate or restart containers
BASESTATION_LOCK = "basestation"


def stack_lock(t):  # a robot's launch / services / deploy
    return f"stack:{t.kind}:{t.name}"


def motion_lock(t):  # a robot's arm moves and cut_stem runs
    return f"motion:{t.kind}:{t.name}"


def check_free(*locks):
    """Refuse (400) while a job holding one of `locks` runs: called before an action changes settings."""
    with JOBS_LOCK:
        for j in JOBS.values():
            if j.status == "running" and j.lock in locks:
                raise ValueError(f"{j.name} is still running (job {j.id}): wait for it, or stop it first")


def start_job(name, fn, lock=None):
    """Run fn(job) in a thread. lock: refused while another job with the same lock runs (see FLEET_LOCK)."""
    job = Job(name, lock)
    with JOBS_LOCK:
        busy = lock and next((j for j in JOBS.values() if j.status == "running" and j.lock == lock), None)
        if busy:
            raise ValueError(f"{busy.name} is still running (job {busy.id}): wait for it, or stop it first")
        JOBS[job.id] = job
        done = sorted((j for j in JOBS.values() if j.status != "running"), key=lambda j: j.started)
        for old in done[:max(0, len(JOBS) - 50)]:  # finished ones only: a running job's log stays reachable
            JOBS.pop(old.id)

    def target():
        try:
            ok = fn(job)
            # not `ok in (None, True, 0)`: False == 0, so every failed `return rc == 0` read as done
            job.status = "done" if ok is None or ok is True or (type(ok) is int and ok == 0) else "failed"
        except Exception as e:  # surfaced in the job log rather than killing the server
            job.log(f"error: {e!r}")
            job.status = "failed"

    threading.Thread(target=target, daemon=True).start()
    return job


def label(t):
    return t.name if t.kind == "sim" else f"{t.name} (real)"


# ---------------------------------------------------------------- state

def read_env():
    """The variables compose sees: .env, rendered again from the settings database first when it is reachable (so a
    change made elsewhere, e.g. scripts/fleetcfg.py set, shows here)."""
    return fleetcfg.env(ACTOR)


def write_env(updates):
    """Settings into the database's active profile, then .env rendered from it. Refused (ValueError) while the
    database can't be reached, unless the values are already in effect."""
    try:
        return fleetcfg.set_settings(updates, ACTOR)
    except fleetcfg.Unavailable as e:
        raise ValueError(f"can't change settings: {e}") from None


def available_models():
    # what the running sim imported and can spawn; without a sim, every generated URDF (scripts/gen_urdf.sh)
    state = fleet_ctl.read_state()
    if state and state.get("models"):
        return sorted(state["models"])
    return sorted(d.name for d in (ROOT / "sim/assets").iterdir() if (d / f"{d.name}.urdf").exists())


SCENE_DIR = ROOT / "sim/scene"  # mounted in the sim as /sim/scene
SCENE_EXTS = (".usd", ".usda", ".usdc", ".usdz")
SCENE_MAX_BYTES = 512 * 1024 * 1024
# "lavender" is only reachable through .env (SIM_SCENE=lavender); the page offers a file picker + Default
BUILTIN_SCENES = {"": "Default (ground plane + lights)", "lavender": "Lavender farm (built-in)"}


def scene_files():
    """USD files under sim/scene/ (paths relative to it), e.g. scenes saved from Isaac Sim with File > Save As."""
    if not SCENE_DIR.is_dir():
        return []
    return sorted(str(p.relative_to(SCENE_DIR)) for p in SCENE_DIR.rglob("*")
                  if p.is_file() and p.suffix.lower() in SCENE_EXTS)


def scene_label(scene):
    return BUILTIN_SCENES.get(scene, f"sim/scene/{scene}")


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def act_scene_upload(body):
    """The page's file picker can only hand over the file's contents, not its path. A scene's assets are usually
    referenced relative to it (Isaac's Save As writes ../assets/...), so it has to be loaded from where it lives:
    a file under sim/scene/ with the same contents is used in place; anything else is copied into sim/scene/
    (never over a different file of the same name) and its relative references then resolve from there."""
    name = Path(str(body.get("name") or "")).name
    if not name.lower().endswith(SCENE_EXTS):
        raise ValueError(f"pick a USD file ({', '.join(SCENE_EXTS)})")
    try:
        data = base64.b64decode(body.get("data") or "", validate=True)
    except ValueError:
        raise ValueError("bad file data")
    if not data or len(data) > SCENE_MAX_BYTES:
        raise ValueError(f"the file must be 1 byte to {SCENE_MAX_BYTES >> 20} MB")
    digest = hashlib.sha256(data).hexdigest()
    same_size = [f for f in scene_files() if (SCENE_DIR / f).stat().st_size == len(data)]
    # prefer a file with the picked name (the usual case: picking a scene saved into sim/scene/)
    for f in sorted(same_size, key=lambda f: Path(f).name != name):
        if _sha256(SCENE_DIR / f) == digest:
            return {"scene": f, "label": scene_label(f), "copied": False}
    SCENE_DIR.mkdir(parents=True, exist_ok=True)
    try:
        SCENE_DIR.chmod(0o777)  # the sim (uid 1234) saves scenes here too
    except OSError:
        pass
    dest = SCENE_DIR / name
    if dest.exists():
        dest = SCENE_DIR / f"{Path(name).stem}_{digest[:8]}{Path(name).suffix}"
    dest.write_bytes(data)
    f = str(dest.relative_to(SCENE_DIR))
    return {"scene": f, "label": scene_label(f), "copied": True}


def containers():
    _, out = sh(["docker", "ps", "-a", "--filter", f"label=com.docker.compose.project={PROJECT}",
                 "--format", '{{.Names}}\t{{.Label "com.docker.compose.service"}}\t{{.State}}\t{{.Status}}'])
    rows = []
    for line in out.splitlines():
        parts = line.split("\t")
        if len(parts) == 4:
            rows.append(dict(zip(("name", "service", "state", "status"), parts)))
    return rows


def container_state(name):
    code, out = sh(["docker", "inspect", "-f", "{{.State.Status}}", name], timeout=10)
    return out.strip() if code == 0 else None


def zenoh_endpoints(env):
    return env.get("BASESTATION_ZENOH_CONNECT", "tcp/127.0.0.1:7448").strip("\"'").split()


def relays(env):
    """BASESTATION_RELAYS: {robot: endpoint} the base station relays (scripts/robot_relay.py) instead of linking."""
    return dict(e.split("=", 1) for e in env.get("BASESTATION_RELAYS", "").strip("\"'").split() if "=" in e)


IP_RE = re.compile(r"^\d{1,3}(\.\d{1,3}){3}$")
HOST_IPS = {}  # host -> (time, ip or None); an mDNS name that doesn't resolve can take seconds


def host_ip(host):
    if IP_RE.match(host):
        return host
    last = HOST_IPS.get(host)
    if last and time.time() - last[0] < 60:
        return last[1]
    try:
        ip = socket.gethostbyname(host)
    except OSError:
        ip = None
    HOST_IPS[host] = (time.time(), ip)
    return ip


def known_ip(host):
    """host_ip() without a lookup: a literal IP, or the last one resolved (None if never)."""
    if IP_RE.match(host):
        return host
    return (HOST_IPS.get(host) or (0, None))[1]


DEPLOY_CURRENT = {}  # (deployed commit, HEAD) -> bool or None


def deploy_current(deployed, head):
    """Whether a robot's deployed commit has this checkout's colcon_ws/src (submodule pointers included): a newer
    commit that changed nothing there needs no deploy. None: that commit isn't in this repository."""
    if not deployed or not head:
        return None
    if (deployed, head) not in DEPLOY_CURRENT:
        code, _ = sh(["git", "-C", str(ROOT), "diff", "--quiet", deployed, head, "--", "colcon_ws/src"], timeout=10)
        DEPLOY_CURRENT[(deployed, head)] = {0: True, 1: False}.get(code)
    return DEPLOY_CURRENT[(deployed, head)]


# What the base station has of each real robot: its RViz windows, and whether the robot's graph reaches it (a robot
# publishing platform/joint_states, or platform/bms/state: all of a relayed robot's graph that reaches it).
# Configured isn't enough: a300_00036's router once took the base station's link but logged "Could not find
# corresponding link in routers network" and routed nothing.
BASESTATION_PROBE = (f"pgrep -af '{RVIZ_MATCH}'; timeout -k 2 6 ros2 topic list -v 2>/dev/null | "
                     "sed -n 's#^ \\* /\\([A-Za-z0-9_]*\\)/platform/\\(joint_states\\|bms/state\\) .* publishers\\?$#seen=\\1#p'")


BS_VIEW = {"t": 0.0, "v": ({}, None), "lock": threading.Lock()}


def basestation_view(max_age=10.0):
    """basestation_view_now(), at most every max_age s: its `ros2 topic list -v` is a new session on the router."""
    with BS_VIEW["lock"]:
        if time.time() - BS_VIEW["t"] >= max_age:
            BS_VIEW.update(v=basestation_view_now(), t=time.time())
        return BS_VIEW["v"]


def basestation_view_now():
    """({robot: [RViz views open]}, {robots whose topics the base station sees}); ({}, None) without it."""
    try:
        code, out = sh(["docker", "exec", BASESTATION, "bash", "-c", BASESTATION_PROBE], timeout=15)
    except subprocess.TimeoutExpired:
        return {}, None
    if code != 0:
        return {}, None
    views = collections.defaultdict(list)
    for view, ns in re.findall(r"view_(\w+)\.launch\.py namespace:=(\w+)", out):
        views[ns].append(view)
    return views, set(re.findall(r"^seen=(\w+)$", out, re.M))


def robots(mode, rows, env, head):
    sims = [c for c in rows if re.fullmatch(r"robot\d+", c["service"])] if mode != "real" else []
    config = real_config() if mode != "sim" else {}
    reals = [real_target(name, config) for name in sorted(config)]
    targets = [SimTarget(c["name"]) for c in sims if c["state"] == "running"] + reals
    with ThreadPoolExecutor(max_workers=len(targets) + 1) as pool:
        bs = pool.submit(basestation_view) if reals else None
        probed = dict(zip(((t.kind, t.name) for t in targets), pool.map(lambda t: probe(t, max_age=3), targets)))
        views, seen = bs.result() if bs else ({}, None)
    running = running_sims(rows)
    linked = linked_reals(env)
    out = []
    for c in sims:
        r = dict(c, kind="sim", slot=int(c["service"][5:]), cutter=True, **probed.get(("sim", c["name"]), {}))
        r["queued"] = queue_info(c["name"])  # 0, or its place in the launch queue (1 = next)
        real = PROBES.get(("real", c["name"]))
        r["conflict"] = c["name"] in running and c["name"] in real_config() and (
            c["name"] in linked or bool(real and real["data"]["online"]))
        out.append(r)
    endpoints, relayed = zenoh_endpoints(env), relays(env)
    zenoh = (env.get("BASESTATION_RMW") or env.get("FLEET_RMW", "rmw_zenoh_cpp")) == "rmw_zenoh_cpp"
    for t in reals:
        p = probed[("real", t.name)]
        ip = host_ip(t.host)
        out.append(dict(p, kind="real", name=t.name, host=t.host, user=t.user, cutter=t.cutter, ip=ip,
                        state="online" if p["online"] else "offline", rviz=views.get(t.name, []),
                        restart_queued=restart_queue_info(t.name), settling=restart_settling(t.name, check=False),
                        conflict=t.name in running and (p["online"] or t.name in linked),
                        deploy_current=deploy_current(p["deployed"], head),
                        # the base station's router dials the robot's (basestation.compose.yml); None: not zenoh
                        zenoh_linked=(t.name in relayed or ip is not None and f"tcp/{ip}:7447" in endpoints)
                        if zenoh else None, relayed=t.name in relayed,
                        # and the robot's topics actually reach it; None: no base station, or can't tell (offline,
                        # or a sim robot of the same name publishes the same topics)
                        basestation_sees=None if seen is None or not p["online"] or t.name in running
                        else t.name in seen))
    return sorted(out, key=lambda r: (r["kind"] != "sim", r.get("slot", 0), r["name"]))


def status(q):
    mode = q.get("mode") if q.get("mode") in MODES else DEFAULT_MODE
    env = read_env()
    rows = containers()
    sim = next((c for c in rows if c["service"] == "isaac-sim"), None)
    n = int(env.get("NUM_ROBOTS", "0") or 0)
    code, head = sh(["git", "-C", str(ROOT), "rev-parse", "HEAD"], timeout=10)
    head = head.strip() if code == 0 else ""
    return {
        "mode": mode,
        "default_mode": DEFAULT_MODE,
        "sim": sim,
        # the running sim's own report (scene loading/ready, robots in it, last spawn); None if not running
        "scene": fleet_ctl.read_state() if sim and sim["state"] == "running" else None,
        # last spawn request that worked, else the last one written (prefills the spawn form)
        "request": fleet_ctl.read_request(applied=True) or fleet_ctl.read_request(),
        "num_robots": n,
        "slots": [env.get(f"ROBOT_MODEL_{i}", "a300") for i in range(MAX_SLOTS)],
        "models": available_models(),
        "robots": robots(mode, rows, env, head),
        "max_slots": MAX_SLOTS,
        "sim_mode": env.get("SIM_MODE", "stream"),
        "robot_looks": env.get("ROBOT_LOOKS", "full"),
        "sim_physics": env.get("SIM_PHYSICS", "physx"),  # the setting; the running engine is scene.physics
        "sim_scene": env.get("SIM_SCENE", ""),
        "sim_scene_label": scene_label(env.get("SIM_SCENE", "")),
        "fleet_rmw": env.get("FLEET_RMW", "rmw_zenoh_cpp"),
        "use_sim_time": env.get("USE_SIM_TIME", "true"),
        "basestation": container_state(BASESTATION) if mode != "sim" else None,
        # the harvesting operation (the /operation page), for the header's pill; None without a base station
        "operation": OPERATION_WATCH.summary(),
        "known_real_ids": known_real_ids() if mode != "sim" else [],
        "head": head,
        # where .env came from: the settings database (its active profile), or "offline" (.env as last written)
        "settings": {k: fleetcfg.LAST_RENDER.get(k) for k in ("source", "profile", "why", "errors")},
    }


# ---------------------------------------------------------------- actions

SIM_MODES = ("stream", "headed")
ROBOT_LOOKS = ("full", "basic", "off")  # sim/scripts/robot_looks.py MODE
SIM_PHYSICS = ("physx", "newton")  # sim/scripts/setup_scene.py SIM_PHYSICS


def host_display():
    """The X display for SIM_MODE=headed: this server's $DISPLAY, else the first local X socket (:N)."""
    if os.environ.get("DISPLAY"):
        return os.environ["DISPLAY"]
    sockets = sorted(Path("/tmp/.X11-unix").glob("X*"))
    return f":{sockets[0].name[1:]}" if sockets else None


def act_sim_start(body):
    # mode: "stream" (headless, WebRTC client) or "headed" (Isaac Sim's own window on this machine's display).
    # Saved as the setting SIM_MODE; fleet.sh's `docker compose up -d` recreates the sim when it changes.
    current = read_env()
    mode = (body or {}).get("mode") or current.get("SIM_MODE", "stream")
    if mode not in SIM_MODES:
        raise ValueError(f"mode must be one of {SIM_MODES}")
    # looks: robot materials (ROBOT_LOOKS), "full" (textured), "basic" (plain colours) or "off" (importer's own);
    # saved like SIM_MODE, so a change recreates the sim too
    looks = (body or {}).get("looks") or current.get("ROBOT_LOOKS", "full")
    if looks not in ROBOT_LOOKS:
        raise ValueError(f"looks must be one of {ROBOT_LOOKS}")
    # scene: SIM_SCENE, "" (ground plane + lights), "lavender" or a file in sim/scene/; saved like SIM_MODE, so a
    # change recreates the sim
    scene = (body or {}).get("scene")
    scene = current.get("SIM_SCENE", "") if scene is None else scene
    if scene not in BUILTIN_SCENES and scene not in scene_files():
        raise ValueError(f"no scene {scene!r}: pick a USD file again")
    # physics engine (SIM_PHYSICS): physx or newton; saved like SIM_MODE, so a change recreates the sim
    physics = (body or {}).get("physics") or current.get("SIM_PHYSICS", "physx")
    if physics not in SIM_PHYSICS:
        raise ValueError(f"physics must be one of {SIM_PHYSICS}")
    # before the job, so a refusal (database down and a value changed) shows on the page
    check_free(FLEET_LOCK)
    write_env({"SIM_MODE": mode, "ROBOT_LOOKS": looks, "SIM_SCENE": scene, "SIM_PHYSICS": physics})

    def fn(j):
        env = dict(os.environ)
        if mode == "headed":
            display = host_display()
            if not display:
                j.log("headed mode needs an X display: no $DISPLAY and no /tmp/.X11-unix socket")
                return False
            env["DISPLAY"] = display
            j.log(f"headed: DISPLAY={display}")
            if j.run(["scripts/x11_auth.sh"], env=env) != 0:
                return False
        j.log(f"settings: SIM_MODE={mode} ROBOT_LOOKS={looks} SIM_SCENE={scene} SIM_PHYSICS={physics}")
        # scene only: the robots are spawned afterwards (act_spawn); waits until the scene is ready
        return j.run(["scripts/fleet.sh", "scene"], env=env) == 0

    return start_job(f"start sim ({mode}, looks {looks}, scene {scene_label(scene)}, {physics})", fn, lock=FLEET_LOCK)


def act_sim_stop(_):
    return start_job("stop sim", lambda j: j.run(["scripts/fleet.sh", "down"]) == 0)


def act_sim_reset(_):
    # The running sim stops and plays its timeline (Isaac's Stop and Play buttons): every robot back at its spawn
    # state in seconds, instead of the ~1 min of restarting the sim container. Robot containers keep running.
    # reloaded so an edited scripts/fleet_ctl.py takes effect without restarting this server (a stale copy once
    # reset the sim before stopping the robots)
    def fn(j):
        importlib.reload(fleet_ctl)
        return fleet_ctl.reset(log=j.log)

    return start_job("reset scene", fn, lock=FLEET_LOCK)


def act_spawn(body):
    # body: {robots: [{model, x, y, yaw}, ...]}, one per slot; x/y in m (world frame), yaw in degrees.
    # The sim checks the rest (ground limits, spacing, duplicate real robots) and the job log shows its answer.
    entries = body.get("robots")
    allowed = set(available_models())
    if not isinstance(entries, list) or len(entries) > MAX_SLOTS:
        raise ValueError(f"robots must be a list of up to {MAX_SLOTS} entries")
    poses = []
    for i, e in enumerate(entries):
        if not isinstance(e, dict) or e.get("model") not in allowed:
            raise ValueError(f"slot {i}: model must be one of {sorted(allowed)}")
        pose = {}
        for k in ("x", "y", "yaw"):
            try:
                pose[k] = float(e.get(k))
            except (TypeError, ValueError):
                raise ValueError(f"slot {i}: {k} must be a number")
            if not math.isfinite(pose[k]):
                raise ValueError(f"slot {i}: {k} must be finite")
        poses.append(pose)
    state = fleet_ctl.read_state()
    if not state or state.get("scene") != "ready":
        raise ValueError("the scene is not ready -- press Start and wait for it")
    if (state.get("spawn") or {}).get("state") == "spawning":
        raise ValueError("a spawn is already in progress")
    models = [e["model"] for e in entries]
    check_free(FLEET_LOCK)
    # slots 0..N-1 (model and pose) and NUM_ROBOTS into the settings database, before the job so a refusal shows on
    # the page; the poses are kept, so a later `scripts/fleet.sh spawn` places the robots here again
    try:
        fleetcfg.set_slots([dict(model=m, **p) for m, p in zip(models, poses)], ACTOR, num_robots=len(models))
    except fleetcfg.Unavailable as e:
        raise ValueError(f"can't save the robots: {e}") from None

    def fn(j):
        j.log(f"settings: NUM_ROBOTS={len(models)}, slots {', '.join(models) or '(none)'}")
        # the sim replaces its robots (the scene keeps running), then the robot containers are (re)created
        return j.run(["scripts/fleet.sh", "spawn", "--poses", json.dumps(poses)]) == 0

    return start_job(f"spawn {len(models)} robot(s)", fn, lock=FLEET_LOCK)


# Launch queue (sim robots on the shared zenoh router): a stack's start floods the router with declarations, and
# with others running it pegs a core for 30-90 s, stalling every robot's data (3 at once, 2026-10-12: up to 7 min,
# 208 s without data, new sessions refused). Started 120 s apart each one settled before the next. So a launch
# waits its turn: LAUNCH_SPACING_S after the previous launch started here, then until the router is idle
# (LAUNCH_IDLE_CPU, two samples in a row: also covers launches started elsewhere), at most LAUNCH_IDLE_MAX_S
# for that. Stop on a queued robot takes it out of the queue.
LAUNCH_SPACING_S = 120
LAUNCH_IDLE_CPU = 50.0  # % of one core
LAUNCH_IDLE_MAX_S = 300
ROUTER = "zenoh-router"
LAUNCH_QUEUE = []  # robots waiting, first = next to start
LAUNCH_STOPPED = {}  # robot -> time of its last launch Stop
LAUNCH_CV = threading.Condition()
LAST_LAUNCH = {"t": 0.0, "robot": ""}


def router_cpu():
    # docker stats needs ~2 s for its sample; None: no router (not zenoh, or not running)
    code, out = sh(["docker", "stats", "--no-stream", "--format", "{{.CPUPerc}}", ROUTER], timeout=15)
    try:
        return float(out.strip().rstrip("%")) if code == 0 else None
    except ValueError:
        return None


def queue_info(robot):
    with LAUNCH_CV:
        return LAUNCH_QUEUE.index(robot) + 1 if robot in LAUNCH_QUEUE else 0


def launch_running(robot):
    return sh(["docker", "exec", robot, "pgrep", "-f", "sim_robot_upstart.launch.py"])[0] == 0


def wait_launch_turn(j, robot):
    """Queue `robot`'s launch until its turn; it stays first in LAUNCH_QUEUE until launch_done. False: Stop took
    it out meanwhile."""
    def stopped():
        if robot not in LAUNCH_QUEUE:
            j.log("taken out of the queue (Stop): not starting")
            return True
        return False

    with LAUNCH_CV:
        LAUNCH_QUEUE.append(robot)
        if len(LAUNCH_QUEUE) > 1:
            j.log(f"queued behind {', '.join(LAUNCH_QUEUE[:-1])}: stacks start one at a time on the shared "
                  f"zenoh router")
        while robot in LAUNCH_QUEUE and LAUNCH_QUEUE[0] != robot:
            LAUNCH_CV.wait(5)
        if stopped():
            return False
    prev = LAST_LAUNCH["robot"]
    wait = LAST_LAUNCH["t"] + LAUNCH_SPACING_S - time.time()
    if wait > 0 and prev in running_sims() and launch_running(prev):  # a stack stopped since costs nothing
        j.log(f"waiting {wait:.0f} s: {prev}'s stack started {LAUNCH_SPACING_S - wait:.0f} s ago "
              f"(launches {LAUNCH_SPACING_S} s apart)")
        while time.time() < LAST_LAUNCH["t"] + LAUNCH_SPACING_S:
            if stopped():
                return False
            time.sleep(2)
    deadline, idle, said = time.time() + LAUNCH_IDLE_MAX_S, 0, False
    while idle < 2:
        if stopped():
            return False
        cpu = router_cpu()  # ~2 s per sample
        if cpu is None:
            break
        idle = idle + 1 if cpu < LAUNCH_IDLE_CPU else 0
        if idle == 0 and not said:
            j.log(f"waiting for {ROUTER} to settle ({cpu:.0f} % CPU, starting below {LAUNCH_IDLE_CPU:.0f} %)")
            said = True
        if idle == 0 and time.time() > deadline:
            j.log(f"{ROUTER} still at {cpu:.0f} % after {LAUNCH_IDLE_MAX_S} s: starting anyway")
            break
    return not stopped()


def launch_done(robot):
    with LAUNCH_CV:
        if robot in LAUNCH_QUEUE:
            LAUNCH_QUEUE.remove(robot)
        LAUNCH_CV.notify_all()


def act_launch_start(body):
    robot = resolve(body, kinds=("sim",)).name
    check_free(FLEET_LOCK)

    def fn(j):
        if launch_running(robot):
            j.log("already running")
            return True
        # The launch reads /etc/clearpath/robot.srdf, which the container's boot (robot/bin/generate_srdf) writes
        # ~5-30 s after the container starts; launched before that it dies at once. Waited for before queueing,
        # so a booting robot doesn't hold up the others.
        deadline = time.time() + SRDF_WAIT_S
        if sh(["docker", "exec", robot, "test", "-s", SRDF])[0] != 0:
            j.log(f"waiting for {SRDF} (the robot is still booting)...")
            while sh(["docker", "exec", robot, "test", "-s", SRDF])[0] != 0:
                if time.time() > deadline:
                    j.log(f"no {SRDF} after {SRDF_WAIT_S} s; see `docker logs {robot}` ([generate_srdf] lines)")
                    return False
                time.sleep(2)
        if LAUNCH_STOPPED.get(robot, 0) > j.started:  # Stop pressed while it waited, before it could be queued
            j.log("stopped before it started")
            return True
        if read_env().get("FLEET_RMW", "rmw_zenoh_cpp") != "rmw_zenoh_cpp":  # no shared router: no queue
            return start_stack(j)
        container = sh(["docker", "inspect", "-f", "{{.Id}}", robot])[1].strip()
        try:
            if not wait_launch_turn(j, robot):
                return True
            # a Spawn or Reset while it waited replaced the container: not what was asked for any more
            if sh(["docker", "inspect", "-f", "{{.Id}}", robot])[1].strip() != container:
                j.log(f"{robot}'s container was recreated while queued: not starting (start it again)")
                return False
            return start_stack(j)
        finally:
            launch_done(robot)

    def start_stack(j):
        rc = j.run(["docker", "exec", "-d", robot, "bash", "-c", LAUNCH_CMD])
        if rc != 0:
            return False
        LAST_LAUNCH.update(t=time.time(), robot=robot)
        # A launch that can't start (missing file, bad package, stale install) logs "Caught exception in launch" in a
        # second or two, but it exits only once it has stopped every node it started (SIGINT, SIGTERM after 5 s,
        # SIGKILL after 5 more): a check at 5 s still saw it running and reported a dead launch as started (j100_0921,
        # 2026-10-22, mtu32_bringup's scripts not built yet). Either sign fails the job.
        deadline = time.time() + LAUNCH_CHECK_S
        while time.time() < deadline:
            time.sleep(1)
            failed = SimTarget(robot).run(f"grep -q 'Caught exception in launch' {LAUNCH_LOG}")[0] == 0
            if failed or not launch_running(robot):
                break
        else:
            failed = False
        if failed or not launch_running(robot):
            j.log("the launch failed at start; its error and the end of its log:")
            j.log(SimTarget(robot).run(f"grep -m 3 -E '^\\[ERROR\\] \\[launch\\]' {LAUNCH_LOG}; echo ...; "
                                       f"tail -n 15 {LAUNCH_LOG}")[1].rstrip())
            # nodes it started before failing (e.g. moveit_sim_bridge) outlive it and would double up with the
            # next launch's; restart_ros (= the Stop button) removes them
            j.run(["docker", "exec", robot, "restart_ros"])
            return False
        j.log(f"started; output in {robot}:{LAUNCH_LOG} (move_group comes up ~20 s later)")
        return True

    return start_job(f"{robot}: start sim_robot_upstart", fn, lock=stack_lock(SimTarget(robot)))


def act_launch_stop(body):
    robot = resolve(body, kinds=("sim",), check_conflict=False).name
    LAUNCH_STOPPED[robot] = time.time()  # a launch job still waiting for the SRDF doesn't start
    launch_done(robot)  # a queued launch: out of the queue, its job ends without starting
    return start_job(f"{robot}: restart_ros", lambda j: j.run(["docker", "exec", robot, "restart_ros"]) == 0)


class NeedPassword(ValueError):
    """The robot's sudo wants a password: the page asks for it and sends the request again with it."""


# Restart services, one robot at a time. A restart drops every session of the robot's zenoh router and its ~30
# nodes declare everything again at once; the base station's router relays that to every robot and sim it dials,
# and the router-to-router links clog: "Unable to push non droppable network message ... Closing transport!" every
# 5 s, during which the robot's router passes nothing off the robot (2026-10-19: a300_00036's link jammed for
# 3.5 min, and a200_0284 restarted on top of it made it worse). A restart waits until every robot restarted from
# here in the last RESTART_SETTLE_MAX_S has settled: RESTART_MIN_S since its restart, no such error in its router's
# journal nor the base station's router log for RESTART_QUIET_S, and, while the Communication card runs its link
# monitor, its odom arriving. The base station's link to a restarted robot can loop (connect, both sides blocked
# 5 s, closed, again) with quiet gaps of 35 s (a200_0284, 2026-10-19 16:48-16:51): 30 s let the next one go.
RESTART_MIN_S = 45  # its stack is up by then (move_group ~12 s after the services)
RESTART_QUIET_S = 60
RESTART_SETTLE_MAX_S = 300  # then the next one goes anyway
RESTART_QUEUE = []  # robots waiting, first = next
RESTART_CV = threading.Condition()
RESTARTED = {}  # robot -> time its services were restarted from here; removed once it settled
SETTLE_CACHE = {}  # robot -> (time checked, why it hasn't settled: "" = settled)
SETTLE_CHECK_S = 10
ROUTER_JAM_RE = "Unable to push non droppable|Closing transport|Error syncing transport"
BS_ROUTER_LOG = "/tmp/zenoh_router.log"  # in the base station container (basestation/entrypoint.sh)


def router_jam_age(t=None):
    """Seconds since the last router-link error in a real robot's router journal (t) or the base station's router
    log (no t); None: none in the last RESTART_QUIET_S, or nothing to read."""
    if t is not None:
        code, out = t.run(f"journalctl -u clearpath-zenoh-router --since=-{RESTART_QUIET_S}s -q --no-pager "
                          f"-o short-unix 2>/dev/null | grep -E '{ROUTER_JAM_RE}' | tail -n 1 | cut -d' ' -f1; "
                          "date +%s.%N", timeout=20, ros_env=False)
        stamps = out.split()
        return float(stamps[1]) - float(stamps[0]) if code == 0 and len(stamps) == 2 else None
    if container_state(BASESTATION) != "running":
        return None
    code, out = sh(["docker", "exec", BASESTATION, "bash", "-c",
                    f"tail -n 400 {BS_ROUTER_LOG} 2>/dev/null | grep -E '{ROUTER_JAM_RE}' | tail -n 1 | "
                    "grep -o '20[0-9][0-9]-[0-9-]*T[0-9:]*' | head -n 1"], timeout=15)
    if code != 0 or not out.strip():
        return None
    when = datetime.datetime.strptime(out.strip(), "%Y-%m-%dT%H:%M:%S").replace(tzinfo=datetime.timezone.utc)
    age = time.time() - when.timestamp()
    return age if age < RESTART_QUIET_S else None


def restart_settling(name, check=True):
    """Why `name`, restarted from here, hasn't settled yet: "" = settled (or not restarted lately). check=False:
    the last check's answer only (status polls must not ssh)."""
    started = RESTARTED.get(name)
    if started is None:
        return ""
    age = time.time() - started
    if age > RESTART_SETTLE_MAX_S:
        RESTARTED.pop(name, None)
        return ""
    if age < RESTART_MIN_S:
        return f"restarted {age:.0f} s ago (settling for at least {RESTART_MIN_S} s)"
    last = SETTLE_CACHE.get(name)
    if not check or (last and time.time() - last[0] < SETTLE_CHECK_S):
        return last[1] if last else f"restarted {age:.0f} s ago"
    why = ""
    if name in real_config():
        jam = router_jam_age(real_target(name))
        bs_jam = router_jam_age()
        received, report = LINK.ros.get(name, (0.0, None))
        odom = ((report or {}).get("topics") or {}).get("odom") if time.time() - received < 10 else None
        if jam is not None:
            why = f"its zenoh router's links clogged {jam:.0f} s ago (restarted {age:.0f} s ago)"
        elif bs_jam is not None:
            why = f"the base station's zenoh router links clogged {bs_jam:.0f} s ago (restarted {age:.0f} s ago)"
        elif odom is not None and (odom.get("silent") is None or odom["silent"] > LINK_LIMITS["silent_s"][1]):
            why = f"its odom doesn't reach the base station yet (restarted {age:.0f} s ago)"
    SETTLE_CACHE[name] = (time.time(), why)
    if not why:
        RESTARTED.pop(name, None)
    return why


def restart_queue_info(robot):
    with RESTART_CV:
        return RESTART_QUEUE.index(robot) + 1 if robot in RESTART_QUEUE else 0


def restart_done(robot):
    with RESTART_CV:
        if robot in RESTART_QUEUE:
            RESTART_QUEUE.remove(robot)
        RESTART_CV.notify_all()


def wait_restart_turn(j, robot):
    """Queue `robot`'s restart until its turn and until the robots restarted before it settled. False: cancelled
    (it was taken out of RESTART_QUEUE meanwhile)."""
    with RESTART_CV:
        RESTART_QUEUE.append(robot)
        if len(RESTART_QUEUE) > 1:
            j.log(f"queued behind {', '.join(RESTART_QUEUE[:-1])}: robots restart one at a time")
        while robot in RESTART_QUEUE and RESTART_QUEUE[0] != robot:
            RESTART_CV.wait(5)
    deadline, said, said_t = time.time() + RESTART_SETTLE_MAX_S, "", 0.0
    while robot in RESTART_QUEUE:
        waiting = {n: w for n in list(RESTARTED) if n != robot and (w := restart_settling(n))}
        if not waiting:
            return True
        msg = "; ".join(f"{n}: {w}" for n, w in waiting.items())
        if re.sub(r"\d+", "", msg) != said or time.time() - said_t > 30:  # a new reason, else every 30 s
            j.log(f"waiting for {msg}")
            said, said_t = re.sub(r"\d+", "", msg), time.time()
        if time.time() > deadline:
            j.log(f"still not settled after {RESTART_SETTLE_MAX_S} s: restarting anyway")
            return True
        time.sleep(5)
    j.log("taken out of the queue (Cancel): not restarting")
    return False


def act_real_restart_cancel(body):
    t = resolve(body, kinds=("real",), check_conflict=False)
    if not restart_queue_info(t.name):
        raise ValueError(f"no restart of {t.name} is waiting")
    restart_done(t.name)
    return {"cancelled": t.name}


def act_real_restart(body):
    # The real robot's counterpart of start/stop: Clearpath's services (clearpath-platform-extras runs bringup_main,
    # clearpath-manipulators the arm driver and move_group). Without a NOPASSWD sudoers entry for this, the page asks
    # for the robot user's sudo password: it goes to `sudo -S` on ssh's stdin, never on a command line or in a log,
    # and isn't kept (Handler.do_POST only takes it from this machine).
    t = resolve(body, kinds=("real",))
    password = body.get("password")
    if password is None:
        code, out = t.run("sudo -n true 2>&1", timeout=30)
        if code != 0:
            if "password" not in out:
                raise ValueError(f"sudo on {t.name}: {out.strip() or f'exit {code}'}")
            raise NeedPassword(f"sudo on {t.name} needs robot's password (or, to skip this: {SUDOERS_HINT})")
    elif not isinstance(password, str) or not password or len(password) > 1024 or "\n" in password:
        raise ValueError("bad password")

    def fn(j):
        # the password (if any) stays in this closure, in memory only, while the restart waits its turn
        try:
            if not wait_restart_turn(j, t.name):
                return True
            RESTARTED[t.name] = time.time()  # before the command: others wait for it even if this job fails
            SETTLE_CACHE.pop(t.name, None)
            return restart(j)
        finally:
            restart_done(t.name)

    def restart(j):
        if password is None:
            code, out = t.run("sudo -n systemctl restart clearpath-robot 2>&1", timeout=120)
        else:
            # -p '': no prompt in the output; -k: always read the password (a cached sudo timestamp would leave it
            # unread on stdin)
            code, out = t.run("sudo -k -S -p '' systemctl restart clearpath-robot 2>&1", timeout=120,
                              input=password + "\n")
        j.log(out.rstrip() or f"exit {code}")
        if code != 0:
            if re.search(r"incorrect password|Sorry, try again|no password was provided|a password is required", out):
                RESTARTED.pop(t.name, None)  # sudo refused: nothing was restarted, nothing to wait for
                if password is not None:
                    j.log(f"wrong sudo password for {t.user}@{t.host}")
            return False
        j.log("restarted; waiting for clearpath-platform-extras ...")
        for _ in range(30):
            code, out = t.run("systemctl is-active " + " ".join(REAL_SERVICES), timeout=20)
            if out.split()[1:2] == ["active"]:
                j.log("services: " + " ".join(f"{s}={v}" for s, v in zip(REAL_SERVICES, out.split())))
                return True
            time.sleep(2)
        j.log("clearpath-platform-extras isn't active after 60 s: see the robot's Service log")
        return False

    return start_job(f"{label(t)}: restart clearpath-robot", fn, lock=stack_lock(t))


def act_rviz_start(body):
    t = resolve(body)
    view = body.get("view")
    if view not in RVIZ_VIEWS:
        raise ValueError(f"view must be one of {RVIZ_VIEWS}")
    # also in compose's environment; here too for containers created before that (NVIDIA GL thread crashed rviz2)
    exec_env = ["-e", "__GL_THREADED_OPTIMIZATIONS=0"]
    if t.kind == "sim":
        where, ns, sim_time, log = t.name, "$ROBOT_NAMESPACE", "${USE_SIM_TIME:-false}", view
    else:
        if container_state(BASESTATION) != "running":
            raise ValueError("a real robot's RViz runs in the base station container: "
                             "docker compose -f basestation.compose.yml up -d")
        where, ns, sim_time, log = BASESTATION, t.name, "false", f"{t.name}_{view}"
        # a zenoh client of that robot's own router, like its link monitor: not through the base station's router,
        # where opening / closing RViz once held up another robot's data (router-to-router deadlock)
        exec_env += ["-e", f'ZENOH_CONFIG_OVERRIDE=mode="client";connect/endpoints=["tcp/{host_ip(t.host) or t.host}:7447"]']
    match = f"[c]learpath_viz view_{view}.launch.py namespace:={t.name if t.kind == 'real' else ''}"

    def fn(j):
        if sh(["docker", "exec", where, "bash", "-c", f"pgrep -f '{match}'"])[0] == 0:
            j.log(f"view_{view} is already open")
            return True
        if sh(["docker", "exec", where, "bash", "-c",
               "source /home/robot/colcon_ws/install/setup.bash && ros2 pkg prefix clearpath_viz"])[0] != 0:
            j.log("clearpath_viz is not built: scripts/colcon_build.sh --packages-select clearpath_viz")
            return False
        env = dict(os.environ)
        display = host_display()
        if display:
            env["DISPLAY"] = display
            j.run(["scripts/x11_auth.sh"], env=env)  # .x11/xauth, mounted into the robot containers
        args = [ns, sim_time]
        if t.kind == "sim":  # expand in the container, so the job log shows the values the launch gets
            code, out = sh(["docker", "exec", where, "bash", "-c", f"echo {ns} {sim_time}"])
            if code == 0 and len(out.split()) == 2:
                args = out.split()
        cmd = RVIZ_CMD.format(view=view, ns=args[0], sim_time=args[1], log=log)
        if j.run(["docker", "exec", "-d", *exec_env, where, "bash", "-c", cmd]) != 0:
            return False
        time.sleep(3)  # no display / bad package: the launch exits within a second or two
        if sh(["docker", "exec", where, "bash", "-c", f"pgrep -f '{match}'"])[0] != 0:
            j.log("the launch exited right away; end of its log:")
            j.log(sh(["docker", "exec", where, "tail", "-n", "15", f"/tmp/rviz_{log}.log"])[1].rstrip())
            return False
        j.log(f"started; output in {where}:/tmp/rviz_{log}.log")
        return True

    return start_job(f"{label(t)}: rviz {view}", fn)


def act_cutstem_start(body):
    t = resolve(body)
    if not t.cutter:
        raise ValueError(f"{t.name} has no stem cutter (tick 'cutter' in Real robots if it has one now)")

    def fn(j):
        if t.run(f"pgrep -f '{CUT_MATCH}'")[0] == 0:
            j.log("a cut_stem goal is already running")
            return False
        code, out = t.run("ros2 action list 2>/dev/null | grep -x /$ROBOT_NAMESPACE/cut_stem", timeout=20)
        if code != 0:
            j.log("no cut_stem action server -- " + ("start sim_robot_upstart first and wait ~20 s for it to come up"
                                                    if t.kind == "sim" else "is clearpath-platform-extras running?"))
            return False
        return j.run(t.argv(CUT_CMD)) == 0

    return start_job(f"{label(t)}: cut_stem", fn, lock=motion_lock(t))


def act_cutstem_stop(body):
    t = resolve(body, check_conflict=False)  # stopping is always safe

    def fn(j):
        # SIGINT, not SIGKILL: ros2 action send_goal cancels the goal on Ctrl+C, so the server stops the task.
        code, out = t.run(f"pkill -INT -f '{CUT_MATCH}'")
        if code in (0, 1):  # pkill: 0 = signalled, 1 = no such process
            j.log("sent Ctrl+C to the cut_stem client (goal is cancelled)" if code == 0 else "no cut_stem goal running")
            return True
        j.log(f"could not reach {label(t)} (exit {code}): {out.strip()[-200:]} -- use Stop motion, check the robot")
        return False

    return start_job(f"{label(t)}: stop cut_stem", fn)


def stop_robot(j, t):
    """Ctrl+C to this UI's goal clients, then cancel every goal of every action that moves the robot's arm, so it
    stops even if a client is gone (e.g. this server restarted mid-goal) or another program sent the goal.
    Returns "ok", "unreachable" (no answer, ssh failed: nothing was sent) or "unconfirmed" (a cancel went out but
    its reply didn't come back, or the CLI got no ROS session): the last two must not read as stopped."""
    try:
        code, out = t.run(STOP_CMD, timeout=40)
    except subprocess.TimeoutExpired:
        j.log(f"{label(t)}: no answer in 40 s -- not known to be stopped")
        return "unreachable"
    lines = out.strip().splitlines() or [f"exit {code}"]
    for line in lines:
        j.log(f"{label(t)}: {line}")
    known = r"\S+: (return_code=|no action server|cancel sent|no ROS session)"
    if code != 0 or not any(re.match(known, l) for l in lines):
        j.log(f"{label(t)}: could not run the stop there (exit {code}) -- not known to be stopped")
        return "unreachable"
    if any(": cancel sent, no reply" in l or ": no ROS session" in l for l in lines):
        j.log(f"{label(t)}: a cancel was not confirmed (no reply, or no ROS session) -- check that it stopped")
        return "unconfirmed"
    return "ok"


def act_stop(body):
    t = resolve(body, check_conflict=False)
    return start_job(f"{label(t)}: stop motion", lambda j: stop_robot(j, t) == "ok")


def act_stop_all(body):
    """Every robot of the page's mode: the running sim robots and every real robot in the list, all at once. Not
    only those last probed online: an offline robot's probe can be minutes old, and probing first would delay the
    stop -- one that is really off comes back unreachable after ssh's ConnectTimeout."""
    mode = body.get("mode") if body.get("mode") in MODES else DEFAULT_MODE
    targets = [SimTarget(n) for n in sorted(running_sims())] if mode != "real" else []
    if mode != "sim":
        config = real_config()
        targets += [real_target(n, config) for n in sorted(config)]
    if not targets:
        raise ValueError("no robots to stop")

    def fn(j):
        with ThreadPoolExecutor(max_workers=len(targets)) as pool:
            results = list(pool.map(lambda t: stop_robot(j, t), targets))
        bad = []
        for t, r in zip(targets, results):
            seen_online = t.kind == "sim" or (PROBES.get((t.kind, t.name)) or {}).get("data", {}).get("online")
            if r == "unconfirmed" or (r == "unreachable" and seen_online):
                bad.append(f"{label(t)} ({r})")
            elif r == "unreachable":
                j.log(f"{label(t)}: unreachable, and offline at the last check")
        if bad:
            j.log("not known to be stopped: " + ", ".join(bad))
        return not bad

    return start_job("stop all motion: " + ", ".join(label(t) for t in targets), fn)


def act_arm_goto(body):
    t = resolve(body)
    state, group = body.get("state"), body.get("group", "arm_0")
    if not (isinstance(state, str) and NAME_RE.match(state) and isinstance(group, str) and NAME_RE.match(group)):
        raise ValueError("bad state/group")
    vel, record = float(body.get("velocity_scale", 0.3)), float(body.get("record", 0) or 0)
    if not (math.isfinite(vel) and math.isfinite(record)):  # NaN passes every min/max: the real-robot cap too
        raise ValueError("velocity_scale and record must be numbers")
    vel = min(max(vel, 0.01), 1.0)
    if t.kind == "real":
        if body.get("direct"):
            raise ValueError("direct (unplanned) moves are sim only: they go to moveit_sim_bridge")
        vel = min(vel, REAL_MAX_VELOCITY)
    args = f"{state} --group {group} --velocity-scale {vel}" + (" --direct" if body.get("direct") else "")

    def fn(j):
        if record <= 0:
            return j.run(t.argv(arm_tool("arm_goto", args))) == 0
        samples = []
        rec = subprocess.Popen(t.argv(arm_tool("arm_joints", f"--record {min(record, 120)}")),
                               stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        reader = threading.Thread(target=lambda: samples.extend(json.loads(l) for l in rec.stdout if l.startswith("{")))
        reader.start()
        time.sleep(1.5)  # let the recorder's own rclpy startup finish before the arm starts moving
        rc = j.run(t.argv(arm_tool("arm_goto", args)))
        j.log(f"recording joints for {record:.0f}s total ...")
        rec.wait()
        reader.join()
        j.result = {"samples": samples}
        j.log(f"recorded {len(samples)} samples")
        return rc == 0

    return start_job(f"{label(t)}: {group} -> {state}" + (" (direct)" if body.get("direct") else ""), fn,
                     lock=motion_lock(t))


def last_json(code, out, what):
    # the last JSON object line: zenoh can log after it (a300_00036: "close operation timed out!" at shutdown)
    lines = [l for l in out.strip().splitlines() if l.startswith("{")]
    if code != 0 or not lines:
        raise ValueError(out.strip() or f"{what} failed")
    return json.loads(lines[-1])


def get_arm_states(q):
    t = resolve(q, check_conflict=False)  # read only
    return last_json(*t.run(arm_tool("arm_goto", "--list")), "arm_goto --list")


def get_joints(q):
    t = resolve(q, check_conflict=False)
    return last_json(*t.run(arm_tool("arm_joints")), "arm_joints")


def get_launch_log(q):
    t = resolve(q, check_conflict=False)
    if t.kind == "sim":
        return {"log": t.run(f"tail -n 300 {LAUNCH_LOG}")[1]}
    return {"log": t.run("journalctl -u clearpath-platform-extras -n 300 --no-pager")[1]}


def act_deploy(body):
    """scripts/deploy_robot.sh: dry-run (what would change, and files edited on the robot), deploy (after the page
    showed a dry run; --yes, since the script's own delete prompt needs a terminal) or pull (the robot's edits)."""
    t = resolve(body, kinds=("real",), check_conflict=False)
    what = body.get("action")
    flags = {"dry-run": ["--dry-run"], "deploy": ["--yes"], "pull": ["--pull"]}
    if what not in flags:
        raise ValueError(f"action must be one of {sorted(flags)}")
    argv = ["scripts/deploy_robot.sh", t.name, "--host", t.host, "--user", t.user, *flags[what]]
    return start_job(f"{label(t)}: deploy {what}", lambda j: j.run(argv) == 0, lock=stack_lock(t))


def act_basestation_link(body):
    """Relay a real robot's operation topics to the base station (BASESTATION_RELAYS, scripts/robot_relay.py) and
    recreate the base station with it. Not a router link: a linked robot's restart clogged every link of the base
    station's router (2026-10-19). A router link to it (BASESTATION_ZENOH_CONNECT) is replaced."""
    t = resolve(body, kinds=("real",), check_conflict=False)
    # fleetcfg's clash check (linked_real_robots) matches tcp/<host>:7447, so only a literal IP can be relayed
    if not IP_RE.match(t.host):
        raise ValueError(f"{t.host} is not an IP address: give the robot an IP address in Real robots")
    env = read_env()
    endpoint, current = f"tcp/{t.host}:7447", relays(env)
    if current.get(t.name) == endpoint:
        raise ValueError(f"the base station already relays {t.name} ({endpoint})")
    current[t.name] = endpoint
    updates = {"BASESTATION_RELAYS": " ".join(f"{n}={e}" for n, e in sorted(current.items()))}
    if endpoint in zenoh_endpoints(env):
        updates["BASESTATION_ZENOH_CONNECT"] = " ".join(e for e in zenoh_endpoints(env) if e != endpoint)
    check_free(BASESTATION_LOCK)
    write_env(updates)

    def fn(j):
        j.log("settings: " + " ".join(f"{k}={v}" for k, v in updates.items()))
        return j.run(["docker", "compose", "-f", "basestation.compose.yml", "up", "-d"]) == 0

    return start_job(f"base station: relay {t.name} ({endpoint})", fn, lock=BASESTATION_LOCK)


# ---------------------------------------------------------------- base station card

BASESTATION_COMPOSE = ["docker", "compose", "-f", "basestation.compose.yml"]
# Compose 2.38+ builds through buildx bake, which refuses the build's network: host without this grant, and
# `docker compose up --build` can't pass it
BASESTATION_BUILD = ["docker", "buildx", "bake", "-f", "basestation.compose.yml", "--allow=network.host", "--load"]
BASESTATION_IMAGE = "basestation:jazzy"
BASESTATION_EVERY_S = 10
ENDPOINT_RE = re.compile(r"^tcp/[A-Za-z0-9.-]+:\d{1,5}$")


class BasestationWatch:
    """The Base station card's slow half: scripts/basestation_probe.py in the container (zenoh router and its links,
    PostgreSQL, ROS programs, the graph) and `docker stats`, refreshed in the background every 10 s while the card
    polls, so a poll never waits on them (~1 s each)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.data, self.t, self.busy = {}, 0.0, False

    def poll(self, running):
        with self.lock:
            if not running:
                self.data, self.t = {}, 0.0
            elif not self.busy and time.time() - self.t > BASESTATION_EVERY_S:
                self.busy = True
                threading.Thread(target=self.refresh, daemon=True).start()
            return dict(self.data)

    def refresh(self):
        data = {}
        try:
            # stats first: sampled right after the probe, they'd measure the probe's own ros2 CLI
            code, out = sh(["docker", "stats", "--no-stream", "--format", "{{json .}}", BASESTATION], timeout=15)
            data["stats"] = json.loads(out) if code == 0 else None
            code, out = sh(["docker", "exec", BASESTATION, "bash", "-c", "python3 /scripts/basestation_probe.py"],
                           timeout=25)
            try:
                data["probe"] = json.loads(out.strip().splitlines()[-1]) if code == 0 else None
            except (ValueError, IndexError):
                data["probe"] = None
            if data["probe"] is None:
                data["probe_error"] = out.strip()[-400:] or f"exit {code}"
        except (subprocess.TimeoutExpired, ValueError) as e:
            data.setdefault("probe_error", repr(e))
        finally:
            with self.lock:
                self.data, self.t, self.busy = dict(data, probed=time.time()), time.time(), False


BASESTATION_WATCH = BasestationWatch()


def unquote(value):
    return (value or "").strip().strip("\"'")


def get_basestation(q):
    """The container (state, health, image, settings it was created with vs. .env's) on every poll; the probe's
    report as of its last background run."""
    env = read_env()
    code, out = sh(["docker", "inspect", BASESTATION], timeout=10)
    info = json.loads(out)[0] if code == 0 else None
    # what basestation.compose.yml would give the container now (compose treats an empty value as unset)
    want = {"RMW_IMPLEMENTATION": unquote(env.get("BASESTATION_RMW")) or unquote(env.get("FLEET_RMW")) or "rmw_zenoh_cpp",
            "ROS_DOMAIN_ID": unquote(env.get("ROS_DOMAIN_ID")) or "0",
            "USE_SIM_TIME": unquote(env.get("USE_SIM_TIME")) or "true",
            "BASESTATION_ZENOH_CONNECT": " ".join(zenoh_endpoints(env)),
            "BASESTATION_RELAYS": " ".join(f"{n}={e}" for n, e in sorted(relays(env).items()))}
    r = {"exists": info is not None, "want": want, "endpoints_env": zenoh_endpoints(env)}
    if info is None:
        return r
    st = info["State"]
    have = dict(e.split("=", 1) for e in info["Config"]["Env"] if "=" in e)
    code, image = sh(["docker", "image", "inspect", "-f", "{{.Id}}\t{{.Created}}", BASESTATION_IMAGE], timeout=10)
    image_id, image_created = (image.strip().split("\t") + [""])[:2] if code == 0 else ("", "")
    r.update(state=st["Status"], health=(st.get("Health") or {}).get("Status"), started_at=st["StartedAt"],
             finished_at=st["FinishedAt"], exit_code=st["ExitCode"], restart_count=info["RestartCount"],
             have={k: have.get(k) for k in want},
             drift=[k for k in want if have.get(k, "") != want[k]],
             # the image was rebuilt since the container was created: Recreate runs the new one
             image_outdated=bool(image_id) and image_id != info["Image"], image_created=image_created)
    r.update(BASESTATION_WATCH.poll(st["Status"] == "running"))
    # name the router's endpoints (the container's own list, not .env's) and whether each has a live link
    # labels only: IPs already known (literal, or resolved earlier by a status poll), never a lookup here -- an
    # mDNS name that doesn't resolve (cpr-j100-0921.local) takes 5 s
    names = {"127.0.0.1": "sim (zenoh-router)"}
    config = real_config()
    for name in config:
        ip = known_ip(real_target(name, config).host)
        if ip:
            names[ip] = name
    probe = r.get("probe") or {}
    links = (probe.get("router") or {}).get("links") or []
    endpoints = []
    for e in (have.get("BASESTATION_ZENOH_CONNECT") or "").split():
        host, _, port = e.removeprefix("tcp/").rpartition(":")
        ip = known_ip(host) or host
        endpoints.append({"endpoint": e, "name": names.get(ip, ""), "in_env": e in r["endpoints_env"],
                          "linked": any(l["dir"] == "out" and l["remote"] == f"{ip}:{port}" for l in links)
                          if probe.get("router") else None})
    r["endpoints"] = endpoints
    # relayed robots (the container's own list): whether each relay runs, and its last log line
    have_relays = relays({"BASESTATION_RELAYS": have.get("BASESTATION_RELAYS") or ""})
    r["relays"] = [{"robot": n, "endpoint": e, "in_env": n in relays(env)} for n, e in sorted(have_relays.items())]
    if r["relays"] and st["Status"] == "running":
        script = "; ".join(f"echo \"{x['robot']} $(pgrep -f '[r]obot_relay.py {x['robot']} ' | head -1)|"
                           f"$(tail -n 1 /tmp/relay_{x['robot']}.log 2>/dev/null)\"" for x in r["relays"])
        code, out = sh(["docker", "exec", BASESTATION, "bash", "-c", script], timeout=10)
        lines = dict(l.split(" ", 1) for l in out.splitlines() if " " in l) if code == 0 else {}
        for x in r["relays"]:
            pid, _, last = lines.get(x["robot"], "|").partition("|")
            x.update(running=bool(pid.strip()), last=last.strip())
    # incoming links: robots' routers dialling this one; local ones are this container's own sessions
    r["sessions_local"] = sum(1 for l in links if l["dir"] == "in" and l["remote"].startswith("127."))
    r["incoming"] = [dict(l, name=names.get(l["remote"].rsplit(":", 1)[0], "")) for l in links
                     if l["dir"] == "in" and not l["remote"].startswith("127.")]
    return r


def get_basestation_log(q):
    which = q.get("which", "container")
    if which == "container":
        code, out = sh(["docker", "logs", "--tail", "300", BASESTATION], timeout=15)
    elif which == "router":
        code, out = sh(["docker", "exec", BASESTATION, "tail", "-n", "300", "/tmp/zenoh_router.log"], timeout=15)
    else:
        raise ValueError("which must be container or router")
    return {"log": re.sub(r"\x1b\[[0-9;]*m", "", out)}


def act_basestation(body):
    """Start / stop / restart / recreate (.env applied) / rebuild the base station container, restart its zenoh
    router, or add / remove one of the endpoints its router dials (BASESTATION_ZENOH_CONNECT, then recreate)."""
    action = body.get("action")
    if action == "unrelay":
        robot, current = body.get("robot"), relays(read_env())
        if robot not in current:
            raise ValueError(f"the base station doesn't relay {robot}")
        del current[robot]
        value = " ".join(f"{n}={e}" for n, e in sorted(current.items()))
        check_free(BASESTATION_LOCK)
        write_env({"BASESTATION_RELAYS": value})

        def unrelay(j):
            j.log(f"settings: BASESTATION_RELAYS={value}")
            return j.run(BASESTATION_COMPOSE + ["up", "-d"]) == 0

        return start_job(f"base station: stop relaying {robot}", unrelay, lock=BASESTATION_LOCK)
    if action in ("link", "unlink"):
        endpoint = (body.get("endpoint") or "").strip()
        if not ENDPOINT_RE.match(endpoint):
            raise ValueError("endpoint must look like tcp/<host or ip>:<port>")
        endpoints = zenoh_endpoints(read_env())
        if (endpoint in endpoints) == (action == "link"):
            raise ValueError(f"{endpoint} is {'already' if action == 'link' else 'not'} in BASESTATION_ZENOH_CONNECT")
        new = endpoints + [endpoint] if action == "link" else [e for e in endpoints if e != endpoint]
        value = " ".join(new)
        check_free(BASESTATION_LOCK)
        write_env({"BASESTATION_ZENOH_CONNECT": value})

        def fn(j):
            j.log(f"settings: BASESTATION_ZENOH_CONNECT={value}")
            return j.run(BASESTATION_COMPOSE + ["up", "-d"]) == 0

        return start_job(f"base station: {action} {endpoint}", fn, lock=BASESTATION_LOCK)
    commands = {  # each runs in order, stopping at the first that fails
        "start": [BASESTATION_COMPOSE + ["up", "-d"]],
        "stop": [BASESTATION_COMPOSE + ["stop"]],
        "restart": [["docker", "restart", BASESTATION]],
        "recreate": [BASESTATION_COMPOSE + ["up", "-d", "--force-recreate"]],
        "rebuild": [BASESTATION_BUILD, BASESTATION_COMPOSE + ["up", "-d"]],
        # entrypoint.sh runs the router in a loop: it is back 2 s after it ends; sessions reconnect on their own
        "router": [["docker", "exec", BASESTATION, "pkill", "-f", "[r]mw_zenohd"]],
    }
    if action not in commands:
        raise ValueError(f"action must be one of {sorted(commands) + ['link', 'unlink', 'unrelay']}")
    if action != "start" and container_state(BASESTATION) is None:
        raise ValueError("there is no base station container: Start creates it")
    with BASESTATION_WATCH.lock:
        BASESTATION_WATCH.t = 0.0  # probe again at the next poll
    return start_job(f"base station: {action}", lambda j: all(j.run(c) == 0 for c in commands[action]),
                     lock=BASESTATION_LOCK)


# ---------------------------------------------------------------- motion capture / localization

# Which Motive rigid body each robot follows: a ROS params file keyed by node (/<ns>/natnet_ref_pose), loaded by
# mtu32_bringup's bringup_main.launch.py between config/ref_localization.yaml and the hand-written
# config/ref_localization/<ns>.yaml. Written as JSON (valid YAML: the server stays stdlib-only) under a comment
# header. It is part of colcon_ws/src, so a real robot gets it with the workspace.
ASSIGNMENTS = ROOT / "colcon_ws/src/mtu32_husky/mtu32_bringup/config/ref_localization/assignments.yaml"
ASSIGNMENTS_HEADER = """# Motive rigid body per robot (natnet_ref_pose's rigid_body), written by multirobot_sim's web UI (tools/sim_ui).
# A robot without an entry follows the rigid body named after its namespace. Loaded by bringup_main.launch.py
# after ../ref_localization.yaml and before <namespace>.yaml.
"""
MOCAP_IDLE_S = 30  # the listener stops this long after the page's last poll


def read_assignments():
    try:
        text = "\n".join(l for l in ASSIGNMENTS.read_text().splitlines() if not l.lstrip().startswith("#"))
        data = json.loads(text or "{}")
    except (OSError, json.JSONDecodeError):
        return {}
    return {k.strip("/").split("/")[0]: v.get("ros__parameters", {}).get("rigid_body", "")
            for k, v in data.items() if k.endswith("/natnet_ref_pose")}


def write_assignments(mapping):
    data = {f"/{ns}/natnet_ref_pose": {"ros__parameters": {"rigid_body": rb}} for ns, rb in sorted(mapping.items())}
    tmp = ASSIGNMENTS.with_suffix(".tmp")
    tmp.write_text(ASSIGNMENTS_HEADER + json.dumps(data, indent=2) + "\n")
    tmp.replace(ASSIGNMENTS)


class MocapMonitor:
    """Listens to Motive's NatNet stream on this host while the page polls it: rigid bodies, tracking, rate.

    Registers for unicast (NAT_CONNECT from the data socket, as natnet_ref_pose.py does) and joins the default
    multicast group, so either of Motive's transmission types works."""

    def __init__(self):
        self.lock = threading.Lock()
        self.server = None
        self.thread = None
        self.last_poll = 0.0
        self.reset()

    def reset(self):
        self.version = None
        self.app = None
        self.names = {}
        self.bodies = {}
        self.frames = collections.deque()
        self.last_frame = None
        self.error = None

    def poll(self, server):
        with self.lock:
            self.last_poll = time.time()
            if server != self.server or not (self.thread and self.thread.is_alive()):
                self.server = server
                self.reset()
                self.thread = threading.Thread(target=self.run, args=(server,), daemon=True)
                self.thread.start()
            now = time.time()
            while self.frames and now - self.frames[0] > 2.0:
                self.frames.popleft()
            return {
                "server": server,
                "app": self.app, "natnet": ".".join(map(str, self.version[:2])) if self.version else None,
                "rate": round(len(self.frames) / 2.0, 1),
                # seconds since the last frame (None: none yet); bodies keep the last frame's values
                "age": round(now - self.last_frame, 1) if self.last_frame else None,
                "error": self.error,
                "bodies": [dict(id=i, name=self.names.get(i, ""), **b) for i, b in sorted(self.bodies.items())],
            }

    def run(self, server):
        try:
            local = self.local_ip(server)
            data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            data.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            data.bind(("", natnet.DATA_PORT))
            try:
                data.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                socket.inet_aton(natnet.DEFAULT_MULTICAST) + socket.inet_aton(local))
            except OSError:
                pass  # e.g. Motive on this machine (loopback): unicast only
        except OSError as e:
            self.error = f"cannot listen for Motive: {e}"
            return
        target = (server, natnet.COMMAND_PORT)

        def send(message_id):
            try:
                data.sendto(struct.pack("<HH", message_id, 0), target)
            except OSError as e:
                self.error = f"cannot reach {server}: {e}"

        last_connect = last_modeldef = 0.0
        last_number = None
        try:
            while self.server == server and time.time() - self.last_poll < MOCAP_IDLE_S:
                now = time.time()
                if now - last_connect > 2.0 and (not self.frames or now - self.frames[-1] > 1.0):
                    send(natnet.NAT_CONNECT)  # (re)register; also answers with the NatNet version
                    last_connect = now
                if self.version and now - last_modeldef > 5.0:
                    send(natnet.NAT_REQUEST_MODELDEF)  # rigid bodies renamed / added in Motive
                    last_modeldef = now
                if not select.select([data], [], [], 0.2)[0]:
                    continue
                packet, addr = data.recvfrom(65535)
                if addr[0] != server or len(packet) < 4:
                    continue
                message_id = struct.unpack_from("<H", packet)[0]
                try:
                    if message_id == natnet.NAT_SERVERINFO:
                        name, app_version, self.version = natnet.parse_server_info(packet)
                        self.app = f"{name} {'.'.join(map(str, app_version[:2]))}"
                        self.error = None
                    elif self.version is None:
                        continue
                    elif message_id == natnet.NAT_MODELDEF:
                        self.names = natnet.parse_model_def(packet, self.version)
                    elif message_id == natnet.NAT_FRAMEOFDATA:
                        number, bodies = natnet.parse_frame(packet, self.version)
                        if last_number is not None and 0 <= last_number - number < 1000:
                            continue  # the same frame twice (unicast + multicast)
                        last_number = number
                        with self.lock:
                            self.last_frame = time.time()
                            self.frames.append(self.last_frame)
                            self.bodies = {i: self.body(pos, quat, err, tracked)
                                           for i, (pos, quat, err, tracked) in bodies.items()}
                except (struct.error, ValueError, IndexError) as e:
                    self.error = f"cannot parse NatNet message {message_id}: {e}"
        finally:
            data.close()

    @staticmethod
    def local_ip(server):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((server, natnet.COMMAND_PORT))
            return s.getsockname()[0]
        finally:
            s.close()

    @staticmethod
    def body(pos, quat, err, tracked):
        x, y, z, w = quat
        yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))  # Motive Z-up
        return {"tracked": tracked, "pos": [round(v, 3) for v in pos], "yaw": round(yaw, 1),
                "err_mm": round(err * 1000, 2)}


MOCAP = MocapMonitor()


def get_mocap(q):
    server = q.get("server") or "192.168.50.80"
    if not IP_RE.match(server):
        raise ValueError("server must be an IPv4 address")
    mode = q.get("mode") if q.get("mode") in MODES else DEFAULT_MODE
    snap = MOCAP.poll(server)
    snap["assignments"] = read_assignments()
    # robots to offer, by the page's mode: MTU's real robots (ids with "_" the sim knows, and the real robots list)
    # and the running sim robots
    real = {m for m in available_models() if "_" in m} | set(real_config()) if mode != "sim" else set()
    sim = {r["name"] for r in containers() if r["service"].startswith("robot")} if mode != "real" else set()
    snap["robots"] = sorted(real | sim)
    return snap


def act_mocap_assign(body):
    robot, rigid_body = body.get("robot"), body.get("rigid_body", "")
    if not (isinstance(robot, str) and NAME_RE.match(robot)):
        raise ValueError("bad robot name")
    if not isinstance(rigid_body, str) or (rigid_body and not re.fullmatch(r"[\w.\- ]+", rigid_body)):
        raise ValueError("bad rigid body name")
    mapping = read_assignments()
    for ns in [ns for ns, rb in mapping.items() if rb == rigid_body and ns != robot]:
        del mapping[ns]  # a rigid body follows one robot
    if rigid_body and rigid_body != robot:
        mapping[robot] = rigid_body
    else:
        mapping.pop(robot, None)  # unassigned, or the default (rigid body named after the robot)
    write_assignments(mapping)
    return {"assignments": mapping}


OFFSET_RE = re.compile(r"^base_link_offset: \[([-\d.eE, ]+)\]$", re.M)
CAL_RESULT_RE = {"radius_cm": r"circle radius ([\d.]+) cm", "rms_mm": r"fit RMS ([\d.]+) mm",
                 "tilt_deg": r"reported z axis is ([\d.]+) deg", "yaw_deg": r"yaw correction ([-\d.]+) deg",
                 "correction": r"^base_link_offset correction: (.*)$"}


def act_mocap_calibrate(body):
    """A real robot's base_link_offset (calibrate_ref_offset.py: drives forward/back and turns once): follows the
    card's rigid body first (at runtime: assignments.yaml reaches the robot only at its next launch), then measures
    and applies the corrected offset to the running natnet_ref_pose. The result (job.result.calibration) is saved
    to this checkout's <robot>.yaml only on /api/mocap/calibration/save."""
    t = resolve(body, kinds=("real",))
    rigid_body = body.get("rigid_body")
    if not (isinstance(rigid_body, str) and re.fullmatch(r"[\w.\- ]+", rigid_body)):
        raise ValueError("bad rigid body name")

    def fn(j):
        if t.run(f"pgrep -f '{CAL_MATCH}'")[0] == 0:
            j.log("a calibration is already running on the robot")
            return False
        code, out = t.run("timeout -k 2 30 ros2 param set /$ROBOT_NAMESPACE/natnet_ref_pose rigid_body "
                          + shlex.quote(rigid_body), timeout=40)
        j.log(out.strip())
        if code != 0 or "successful" not in out:
            j.log("could not set natnet_ref_pose's rigid_body: is clearpath-platform-extras running?")
            return False
        lines_before = len(j.lines)
        if j.run(t.argv(CAL_CMD)) != 0:
            return False
        out = "\n".join(j.lines[lines_before:])
        m = OFFSET_RE.search(out)
        if not m:
            j.log("no base_link_offset in the output")
            return False
        cal = {"robot": t.name, "rigid_body": rigid_body, "offset": [float(v) for v in m.group(1).split(",")],
               "applied": "applied to the running client" in out}
        for k, pattern in CAL_RESULT_RE.items():
            found = re.search(pattern, out, re.M)
            cal[k] = (found.group(1) if k == "correction" else float(found.group(1))) if found else None
        j.result = {"calibration": cal}
        return cal["applied"]

    return start_job(f"{label(t)}: calibrate mocap offset ({rigid_body})", fn, lock=motion_lock(t))


def act_mocap_calibration_save(body):
    """Write a finished calibration job's base_link_offset to ref_localization/<robot>.yaml in this checkout (the
    robot gets it on deploy). Only its base_link_offset line changes; a new file gets just that parameter."""
    job = JOBS.get(body.get("job") or "")
    cal = job and job.status == "done" and (job.result or {}).get("calibration")
    if not cal:
        raise ValueError("no finished calibration with that job id")
    path = ASSIGNMENTS.with_name(f"{cal['robot']}.yaml")
    line = f"    base_link_offset: [{', '.join(f'{v:g}' for v in cal['offset'])}]"
    if path.exists():
        text = path.read_text()
        lines = [i for i, l in enumerate(text.splitlines()) if l.lstrip().startswith("base_link_offset:")]
        if len(lines) != 1:
            raise ValueError(f"{path.relative_to(ROOT)} has {len(lines)} base_link_offset lines: edit it by hand")
        rows = text.splitlines()
        rows[lines[0]] = line
        text = "\n".join(rows) + "\n"
    else:
        text = f"/**/natnet_ref_pose:\n  ros__parameters:\n{line}\n"
    path.write_text(text)
    return {"path": str(path.relative_to(ROOT)), "offset": cal["offset"]}


# ---------------------------------------------------------------- communication quality (real robots)

# Four layers, each of which has looked fine while another was broken: radio (the robot's WiFi), network (ping from
# here, the base station's spot on the LAN), middleware (the robot's zenoh router: on 2026-10-06 a300_00036's
# stopped accepting sessions, 85 queued, while ping was perfect) and ROS data (scripts/link_monitor.py in the base
# station: odom, GPS, the TF chains). Everything runs only while the page polls /api/link and stops LINK_IDLE_S
# later. Thresholds: (degraded, bad) per measure.
LINK_IDLE_S = 30
LINK_SSH_EVERY_S = 10
CLOCK_TRUSTED_MS = 25  # a robot's clock offset is used only if measured to within this (half the SSH round trip)
PING_WINDOW_S = 60
LINK_LIMITS = {
    "rtt_ms": (20, 100), "loss_pct": (1, 5), "signal_dbm": (-67, -75),  # signal: below these
    "router_queue": (1, 10), "clock_ms": (50, 500), "ros_rtt_ms": (50, 250), "ros_timeouts_pct": (1, 5),
    "age_s": (0.2, 1.0), "gap_s": (0.5, 2.0), "silent_s": (1.0, 2.0), "rate_ratio": (0.8, 0.5),  # rate: below
}
# One SSH round per robot every LINK_SSH_EVERY_S: the interface toward this machine and, if it is WiFi, its link;
# the zenoh router; load; NTP. ~10 ms of shell on the robot.
LINK_PROBE = (
    "iface=$(ip route get {local} 2>/dev/null | sed -n 's/.* dev \\([^ ]*\\).*/\\1/p'); echo \"iface=$iface\"; "
    "if [ -n \"$iface\" ] && [ -d /sys/class/net/$iface/wireless ]; then "
    "iw dev $iface link 2>/dev/null | sed -n 's/^[[:space:]]*\\(SSID\\|freq\\|signal\\|tx bitrate\\|rx bitrate\\): /\\1=/p'; "
    "awk -v i=\"$iface:\" '$1 == i {{print \"retries=\" $9; print \"missed_beacons=\" $11}}' /proc/net/wireless; fi; "
    "echo \"router=$(systemctl is-active clearpath-zenoh-router)\"; "
    "echo \"router_queue=$(ss -ltnH 'sport = :7447' | awk '{{print $2; exit}}')\"; "
    "echo \"router_errors=$(journalctl -u clearpath-zenoh-router --since=-{every}s -q --no-pager 2>/dev/null | grep -c ERROR)\"; "
    "echo \"load=$(cut -d' ' -f1 /proc/loadavg)\"; echo \"cores=$(nproc)\"; "
    "echo \"ntp=$(timedatectl show -p NTPSynchronized --value 2>/dev/null)\"")


def level_of(value, limits, lower_is_worse=False):
    if value is None:
        return None
    degraded, bad = limits
    if lower_is_worse:
        return "bad" if value < bad else "warn" if value < degraded else "ok"
    return "bad" if value > bad else "warn" if value > degraded else "ok"


class Pinger:
    """`ping -O -i 1` to one robot, from this machine: RTT, jitter and loss over the last PING_WINDOW_S."""

    def __init__(self, host, error=None):
        self.host = host
        self.started = time.time()
        self.samples = collections.deque()  # (time, rtt ms or None for no answer)
        # a restart keeps the last one's error until a reply: else the robot flipped between "unreachable" and
        # "starting" every retry (j100_0921, unresolvable host)
        self.error = error
        self.proc = subprocess.Popen(["ping", "-n", "-O", "-i", "1", "-W", "1", host], stdin=subprocess.DEVNULL,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        for line in self.proc.stdout:
            m = re.search(r"icmp_seq=\d+ .*time=([\d.]+) ms", line)
            if m:
                self.samples.append((time.time(), float(m.group(1))))
                self.error = None
            elif "no answer yet" in line:
                self.samples.append((time.time(), None))
            elif "PING" not in line and line.strip():
                self.error = line.strip()  # e.g. unknown host
        self.error = self.error or "ping stopped"

    def stop(self):
        self.proc.kill()

    def summary(self):
        now = time.time()
        while self.samples and now - self.samples[0][0] > PING_WINDOW_S:
            self.samples.popleft()
        rtts = [r for _, r in self.samples if r is not None]
        if not self.samples:
            return {"error": self.error} if self.error else {}
        return {"rtt_ms": round(sum(rtts) / len(rtts), 2) if rtts else None, "rtt_max_ms": max(rtts, default=None),
                # mean change between consecutive replies (RFC 3550 style, unsmoothed)
                "jitter_ms": round(sum(abs(a - b) for a, b in zip(rtts, rtts[1:])) / (len(rtts) - 1), 2)
                if len(rtts) > 1 else None,
                "loss_pct": round(100 * (len(self.samples) - len(rtts)) / len(self.samples), 1),
                "last_reply_s": round(now - max((t for t, r in self.samples if r is not None), default=0), 1)
                if rtts else None,
                "window_s": round(now - self.samples[0][0])}


class LinkWatch:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_poll = 0.0
        self.pingers = {}       # robot -> Pinger
        self.ssh = {}           # robot -> {"t": time, ...probe values}
        self.ssh_busy = set()
        # One scripts/link_monitor.py per robot, a zenoh client connected straight to that robot's router -- not
        # through the base station's router, whose router-to-router links to two robots deadlocked (2026-10-06):
        # one robot's trouble then held up the other's data. robot -> {"proc", "started", "endpoint", "error"}
        self.monitors = {}
        self.ros = {}           # robot -> (receive time, its last report)
        threading.Thread(target=self.idle_stop, daemon=True).start()

    # -- lifecycle: everything stops LINK_IDLE_S after the page's last poll
    def idle_stop(self):
        while True:
            time.sleep(5)
            with self.lock:
                if self.last_poll and time.time() - self.last_poll > LINK_IDLE_S:
                    self.stop_all()

    def stop_all(self):
        for p in self.pingers.values():
            p.stop()
        self.pingers = {}
        for name in list(self.monitors):
            self.stop_monitor(name)
        self.last_poll = 0.0

    @staticmethod
    def kill_monitor_process(name):
        # killing `docker exec` leaves its process running in the container: stop that one
        sh(["docker", "exec", BASESTATION, "pkill", "-INT", "-f", f"/scripts/[l]ink_monitor.py {name}$"], timeout=10)

    def stop_monitor(self, name):
        m = self.monitors.pop(name, None)
        if m and m["proc"]:
            self.kill_monitor_process(name)
            m["proc"].kill()
        self.ros.pop(name, None)

    def start_monitor(self, name, endpoint):
        self.stop_monitor(name)
        m = self.monitors[name] = {"proc": None, "started": time.time(), "endpoint": endpoint, "error": None}
        if container_state(BASESTATION) != "running":
            m["error"] = "the base station is not running"
            return
        self.kill_monitor_process(name)
        override = f'mode="client";connect/endpoints=["{endpoint}"]'  # replaces the container's (its own router)
        m["proc"] = subprocess.Popen(
            ["docker", "exec", "-e", f"ZENOH_CONFIG_OVERRIDE={override}", BASESTATION, "bash", "-c",
             f"exec python3 -u /scripts/link_monitor.py {name}"],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        threading.Thread(target=self.read_monitor, args=(name, m), daemon=True).start()

    def read_monitor(self, name, m):
        tail = collections.deque(maxlen=5)
        for line in m["proc"].stdout:
            if line.startswith("{"):
                try:
                    report = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if self.monitors.get(name) is m:
                    self.ros[name] = (time.time(), report["robots"].get(name))
            else:
                tail.append(line.rstrip())
        if self.monitors.get(name) is m:
            m["error"] = "link_monitor.py stopped: " + (" | ".join(tail) or f"exit {m['proc'].wait()}")

    # -- SSH round per robot (background, so a slow robot never holds up a poll)
    def ssh_probe(self, t):
        try:
            local = MocapMonitor.local_ip(host_ip(t.host) or t.host)
            code, out = t.run(LINK_PROBE.format(local=local, every=LINK_SSH_EVERY_S), timeout=20, ros_env=False)
            data = dict(line.split("=", 1) for line in out.splitlines() if "=" in line) if code == 0 else {}
            # clock: the robot's time against the middle of a bare round trip (error: half of it), best of 3
            best = None
            for _ in range(3 if code == 0 else 0):
                t0 = time.time()
                code2, remote = t.run("date +%s.%N", timeout=10, ros_env=False)
                t1 = time.time()
                if code2 == 0 and (best is None or t1 - t0 < best[1]):
                    best = (float(remote.strip().splitlines()[-1]) - (t0 + t1) / 2, t1 - t0)
            if best:
                data["clock_offset_ms"] = round(best[0] * 1000, 1)
                data["clock_error_ms"] = round(best[1] / 2 * 1000, 1)
            prev = self.ssh.get(t.name, {})
            for key in ("retries", "missed_beacons"):  # counters since boot: per-interval change
                if key in data and key in prev:
                    data[key + "_delta"] = int(data[key]) - int(prev[key])
            data["error"] = None if code == 0 else (out.strip().splitlines() or [f"exit {code}"])[-1]
            data["t"] = time.time()
            self.ssh[t.name] = data
        except (subprocess.TimeoutExpired, OSError, ValueError) as e:
            self.ssh[t.name] = {"t": time.time(), "error": str(e)}
        finally:
            self.ssh_busy.discard(t.name)

    def poll(self, targets):
        with self.lock:
            self.last_poll = time.time()
            names = tuple(sorted(t.name for t in targets))
            for name in set(self.pingers) - set(names):
                self.pingers.pop(name).stop()
            for t in targets:
                p = self.pingers.get(t.name)
                # a new robot, a changed host, or a ping process that ended (killed, or the host didn't resolve):
                # start it (again); a dead one is retried at most every 10 s
                if p is None or p.host != t.host or (p.proc.poll() is not None and time.time() - p.started > 10):
                    if p is not None:
                        p.stop()
                    self.pingers[t.name] = Pinger(t.host, p.error if p is not None and p.host == t.host else None)
                if (time.time() - self.ssh.get(t.name, {}).get("t", 0) > LINK_SSH_EVERY_S
                        and t.name not in self.ssh_busy):
                    self.ssh_busy.add(t.name)
                    threading.Thread(target=self.ssh_probe, args=(t,), daemon=True).start()
            for name in set(self.monitors) - set(names):
                self.stop_monitor(name)
            for t in targets:
                # a robot's monitor starts once it answers ping (an unreachable one would only churn), and is
                # restarted for a new address, or 30 s after it last started if it has died since
                endpoint = f"tcp/{host_ip(t.host) or t.host}:7447"
                m = self.monitors.get(t.name)
                reachable = self.pingers[t.name].summary().get("rtt_ms") is not None
                if (m is None and reachable) or (m and m["endpoint"] != endpoint) or (
                        m and (m["proc"] is None or m["proc"].poll() is not None)
                        and time.time() - m["started"] > 30 and reachable):
                    self.start_monitor(t.name, endpoint)
            return {t.name: self.robot_report(t) for t in targets}

    # -- one robot: values + a level per value + the overall verdict and its reasons
    def robot_report(self, t):
        reasons = []
        sensor_faults = []  # robot-side: shown, not counted against the link

        def judge(label, value, limits, unit="", lower_is_worse=False, fmt="{:g}"):
            lv = level_of(value, limits, lower_is_worse)
            if lv in ("warn", "bad"):
                reasons.append((lv, f"{label} {fmt.format(value)}{unit}"))
            return lv

        ping = self.pingers[t.name].summary() if t.name in self.pingers else {}
        net = dict(ping, levels={})
        if ping.get("error"):
            reasons.append(("bad", f"ping: {ping['error']}"))
        elif ping.get("rtt_ms") is None and ping.get("window_s", 0) >= 3:
            reasons.append(("bad", "no ping replies"))
            net["levels"]["rtt_ms"] = "bad"
        else:
            net["levels"]["rtt_ms"] = judge("RTT", ping.get("rtt_ms"), LINK_LIMITS["rtt_ms"], " ms")
            net["levels"]["loss_pct"] = judge("ping loss", ping.get("loss_pct"), LINK_LIMITS["loss_pct"], " %")

        s = self.ssh.get(t.name, {})
        wifi = {k: s.get(k) for k in ("iface", "SSID", "freq", "signal", "tx bitrate", "rx bitrate",
                                      "retries_delta", "missed_beacons_delta") if s.get(k) not in (None, "")}
        signal = float(s["signal"].split()[0]) if s.get("signal") else None
        wifi["signal_dbm"] = signal
        wifi["levels"] = {"signal": judge("WiFi signal", signal, LINK_LIMITS["signal_dbm"], " dBm", True)}
        queue = int(s["router_queue"]) if s.get("router_queue", "").isdigit() else None
        errors = int(s["router_errors"]) if s.get("router_errors", "").isdigit() else None
        zenoh = {"router": s.get("router"), "queue": queue, "errors": errors, "levels": {}}
        if s.get("router") and s["router"] != "active":
            reasons.append(("bad", f"zenoh router {s['router']}"))
            zenoh["levels"]["router"] = "bad"
        zenoh["levels"]["queue"] = judge("zenoh router queue", queue, LINK_LIMITS["router_queue"])
        if errors:
            reasons.append(("warn", f"{errors} zenoh router errors in {LINK_SSH_EVERY_S} s"))
            zenoh["levels"]["errors"] = "warn"
        offset, err = s.get("clock_offset_ms"), s.get("clock_error_ms")
        # only an offset measured to within CLOCK_TRUSTED_MS is judged and used to correct the ROS ages
        if offset is not None and (err is None or err > CLOCK_TRUSTED_MS):
            offset = None
        clock = {"offset_ms": s.get("clock_offset_ms"), "error_ms": err, "trusted": offset is not None,
                 "ntp": s.get("ntp"), "load": s.get("load"), "cores": s.get("cores"),
                 "levels": {"offset_ms": judge("clock offset", max(0.0, abs(offset) - err) if offset is not None
                                               else None, LINK_LIMITS["clock_ms"], " ms", fmt="{:.0f}")}}
        if s.get("error"):
            reasons.append(("warn", f"SSH: {s['error']}"))

        m = self.monitors.get(t.name)
        ros = {"error": m["error"] if m else None}
        received, report = self.ros.get(t.name, (0.0, None))
        if report and time.time() - received < 5:
            correct = (offset or 0) / 1000  # age measured with this machine's clock against the robot's stamps
            # ref_localizer says it publishes no map -> odom (no Motive rigid body / GPS fix yet): that is the
            # robot's localization, not its link -- shown as information. Only when its status is fresh: a status
            # that stopped arriving is itself a link (or bringup) problem.
            loc = report.get("localizer")
            no_source = None
            if loc and loc.get("age", 99) < 5 and loc.get("publishing") is False:
                ages = ", ".join(f"{k} {'none' if loc.get(f'{k}_age') is None else str(loc[f'{k}_age']) + ' s old'}"
                                 for k in ("ref", "gps"))
                no_source = f"not published: ref_localizer has no localization source (source {loc.get('source')}; {ages})"
            ros["localizer"] = loc
            # Streams of this robot that do arrive: if some do, the link works, and a topic that delivers nothing
            # is the robot's sensor / driver (a200_0284's GPS, 2026-10-06), not the link -- a sensor fault, shown
            # apart. If nothing arrives at all, it stays a link problem.
            alive = {n for g in ("topics", "tf") for n, s in report[g].items()
                     if not s.get("broken") and s.get("silent") is not None
                     and s["silent"] <= LINK_LIMITS["silent_s"][1]}
            for group, items in (("topics", report["topics"]), ("tf", report["tf"])):
                ros[group] = {}
                for name, v in items.items():
                    v = dict(v, levels={})
                    if group == "tf" and name == "map->odom" and no_source and (v.get("broken") or (
                            v.get("silent") is None or v["silent"] > LINK_LIMITS["silent_s"][1])):
                        ros[group][name] = {"info": no_source, "levels": {}}
                        continue
                    if v.get("broken"):
                        reasons.append(("bad", f"TF {name}: {v['broken']}"))
                        v["levels"]["broken"] = "bad"
                    elif not v.get("static"):
                        if v.get("age") is not None:
                            v["age"] = round(v["age"] + correct, 4)
                        silent = v.get("silent")
                        if silent is None or silent > LINK_LIMITS["silent_s"][1]:
                            since = "since the monitor started" if silent is None else f"for {silent:.0f} s"
                            if group == "topics" and alive - {name}:
                                v["sensor_fault"] = (f"no data {since} while the robot's other data arrive: "
                                                     "its sensor or driver on the robot, not the link")
                                v["levels"]["rate"] = "sensor"
                                sensor_faults.append(f"{name}: no data {since}")
                            else:
                                reasons.append(("bad", f"{name}: no data {since}"))
                                v["levels"]["rate"] = "bad"
                        else:
                            ratio = v["rate"] / v["usual_rate"] if v.get("usual_rate") else None
                            v["levels"]["rate"] = judge(f"{name} rate", ratio, LINK_LIMITS["rate_ratio"], "x usual",
                                                        True, "{:.2f}")
                            v["levels"]["max_gap"] = judge(f"{name} gap", v.get("max_gap"), LINK_LIMITS["gap_s"], " s")
                            v["levels"]["age"] = judge(f"{name} age", v.get("age"), LINK_LIMITS["age_s"], " s",
                                                       fmt="{:.3f}")
                    ros[group][name] = v
            # request + reply through zenoh both ways (link_monitor.py: robot_state_publisher's parameter service)
            rt = dict(report.get("round_trip") or {}, levels={})
            if rt.get("error"):
                reasons.append(("warn", rt["error"]))
            else:
                rt["levels"]["rtt_ms"] = judge("ROS round trip", rt.get("rtt_ms"), LINK_LIMITS["ros_rtt_ms"], " ms")
                rt["levels"]["timeouts_pct"] = judge("ROS round trips unanswered", rt.get("timeouts_pct"),
                                                     LINK_LIMITS["ros_timeouts_pct"], " %")
            ros["round_trip"] = rt
        elif not ros["error"]:
            ros["error"] = "waiting for this robot's link monitor (a client of its own router)..."

        if ping.get("error") or (ping.get("rtt_ms") is None and ping.get("window_s", 0) >= 3):
            # unreachable: everything else follows from that
            reasons = [("bad", "unreachable: " + (ping.get("error") or "no ping replies"))]
        worst = "bad" if any(l == "bad" for l, _ in reasons) else "warn" if reasons else "ok"
        if reasons and reasons[0][1].startswith("unreachable"):
            sensor_faults = []  # can't tell while the robot is unreachable
        reasons = [r for _, r in sorted(reasons, key=lambda r: r[0] != "bad")]
        # restarted from here and not settled yet (restart_settling; ssh at most every SETTLE_CHECK_S): an outage
        # that a restart causes, not a link fault
        settling = restart_settling(t.name) if worst != "ok" else ""
        if settling:
            worst, reasons = "settling", [f"settling after Restart services: {settling}"] + reasons
        return {"host": t.host, "level": worst, "reasons": reasons,
                "sensor_faults": sensor_faults,
                "network": net, "wifi": wifi, "zenoh": zenoh, "clock": clock, "ros": ros}


LINK = LinkWatch()


def get_link(q):
    """Communication quality of the real robots in the list (the page polls this while its card is shown)."""
    config = real_config()
    targets = [real_target(name, config) for name in sorted(config)]
    return {"robots": LINK.poll(targets), "limits": LINK_LIMITS}


LOC_SOURCES = ("auto", "ref", "gps", "external")
LOC_ANCHORS = ("fixed", "start", "external")
LOC_NODE = "/$ROBOT_NAMESPACE/ref_localizer"


def get_localization(q):
    t = resolve(q, check_conflict=False)
    code, out = t.run(f"timeout -k 2 6 ros2 topic echo --once --no-daemon --full-length {LOC_NODE}/status "
                      "std_msgs/msg/String", timeout=15)
    m = re.search(r"^data: '(.*)'$", out, re.M)
    if code != 0 or not m:
        return {"running": False}  # no ref_localizer (sim_robot_upstart / clearpath-platform-extras not running)
    return {"running": True, **json.loads(m.group(1).replace("''", "'"))}


def act_localization_set(body):
    t = resolve(body)
    cmds = []
    if body.get("source"):
        if body["source"] not in LOC_SOURCES:
            raise ValueError(f"source must be one of {LOC_SOURCES}")
        cmds.append(f"ros2 param set {LOC_NODE} source {body['source']}")
    if body.get("anchor"):
        if body["anchor"] not in LOC_ANCHORS:
            raise ValueError(f"anchor must be one of {LOC_ANCHORS}")
        cmds.append(f"ros2 param set {LOC_NODE} anchor {body['anchor']}")
    if body.get("service"):
        if body["service"] not in ("save_anchor", "reset_anchor"):
            raise ValueError("service must be save_anchor or reset_anchor")
        # ros2 service call exits 0 whatever the response: fail the job on success=False
        cmds.append(f'out=$(ros2 service call {LOC_NODE}/{body["service"]} std_srvs/srv/Trigger); echo "$out"; '
                    f'grep -q "success=True" <<< "$out"')
    if not cmds:
        raise ValueError("nothing to do")
    what = ", ".join(f"{k} {body[k]}" for k in ("source", "anchor", "service") if body.get(k))
    return start_job(f"{label(t)}: localization {what}", lambda j: all(j.run(t.argv(c)) == 0 for c in cmds))


# ---------------------------------------------------------------- configuration page (the settings database)

CONFIG_PAGE = Path(__file__).with_name("config.html")
# Not compared between running containers and the settings: the shell's own (the server's DISPLAY isn't the one a
# container was made with), and secrets from db.env, never sent to the page.
NOT_COMPARED = {"DISPLAY", "XAUTHORITY"}
SECRET_RE = re.compile(r"PASSWORD|SECRET|TOKEN|KEY", re.I)
DRIFT_EVERY_S = 4
_drift = {"t": 0.0, "data": None, "lock": threading.Lock()}


def jsonable(rows):
    """datetimes (timestamptz) as ISO strings: the handler's json.dumps knows no datetime."""
    def conv(v):
        return v.isoformat() if hasattr(v, "isoformat") else v
    return [{k: conv(v) for k, v in r.items()} for r in rows]


def compose_services(args):
    """What `docker compose config` resolves now (from .env): {service: {...}}, or None."""
    try:
        p = subprocess.run(["docker", "compose", *args, "config", "--format", "json"], cwd=ROOT, capture_output=True,
                           text=True, timeout=30, stdin=subprocess.DEVNULL)
        return json.loads(p.stdout)["services"] if p.returncode == 0 else None
    except (subprocess.TimeoutExpired, ValueError, KeyError):
        return None


def env_diff(want, info):
    have = dict(e.split("=", 1) for e in info["Config"]["Env"] if "=" in e)
    out = []
    for k, v in sorted((want.get("environment") or {}).items()):
        if k in NOT_COMPARED or SECRET_RE.search(k):
            continue
        v = "" if v is None else str(v)
        if have.get(k, "") != v:
            out.append({"key": k, "running": have.get(k), "configured": v})
    if want.get("hostname") and info["Config"].get("Hostname") != want["hostname"]:
        out.append({"key": "hostname", "running": info["Config"].get("Hostname"), "configured": want["hostname"]})
    return out


def inspect_all(names):
    if not names:
        return {}
    code, out = sh(["docker", "inspect", *names], timeout=15)
    try:
        return {i["Name"].lstrip("/"): i for i in json.loads(out)} if code == 0 else {}
    except ValueError:
        return {}


def config_drift():
    """What the running containers differ in from the settings (they read them at creation): per target, the
    variables (running vs. configured) and robot slots to start, rename or stop. Cached for a few seconds."""
    with _drift["lock"]:
        if _drift["data"] is not None and time.time() - _drift["t"] < DRIFT_EVERY_S:
            return _drift["data"]
        want = compose_services([]) or {}
        rows = [c for c in containers() if c["state"] == "running"]
        bs_want = (compose_services(["-f", "basestation.compose.yml"]) or {}).get("basestation")
        infos = inspect_all([c["name"] for c in rows] + ([BASESTATION] if container_state(BASESTATION) == "running" else []))
        by_service = {c["service"]: c for c in rows}
        data = {"sim": None, "robots": [], "basestation": None}
        sim = by_service.get("isaac-sim")
        if sim and "isaac-sim" in want and sim["name"] in infos:
            data["sim"] = {"running": True, "diff": env_diff(want["isaac-sim"], infos[sim["name"]])}
        else:
            data["sim"] = {"running": False, "diff": []}
        for i in range(MAX_SLOTS):
            svc, c = f"robot{i}", by_service.get(f"robot{i}")
            w = want.get(svc)
            if w is None and c is None:
                continue
            row = {"slot": i, "configured": w.get("container_name") if w else None, "running": c["name"] if c else None}
            if w is None:
                row["problem"] = "running, but NUM_ROBOTS doesn't start this slot"
            elif c is None:
                row["problem"] = "not running" if data["sim"]["running"] else "not running (the sim isn't either)"
            elif c["name"] != w.get("container_name"):
                row["problem"] = f"running as {c['name']}, configured as {w.get('container_name')}"
            else:
                row["diff"] = env_diff(w, infos.get(c["name"], {"Config": {"Env": []}}))
            data["robots"].append(row)
        if bs_want and BASESTATION in infos:
            data["basestation"] = {"running": True, "diff": env_diff(bs_want, infos[BASESTATION])}
        else:
            data["basestation"] = {"running": False, "diff": []}
        _drift.update(t=time.time(), data=data)
        return data


def get_config(q):
    v = fleetcfg.view(ACTOR)
    v["models"] = sorted(set(available_models()) | set(fleetcfg.cat.GENERIC_MODELS))
    v["scenes"] = [""] + (["lavender"]) + scene_files()
    v["real_robots"] = [dict(id=k, **c) for k, c in sorted(real_config().items())]
    v["known_real_ids"] = known_real_ids()
    v["profiles"] = jsonable(v["profiles"])
    v["drift"] = config_drift()
    return v


def get_config_history(q):
    return jsonable(fleetcfg.history(int(q.get("n", 100)), ACTOR))


def get_config_runs(q):
    return jsonable(fleetcfg.runs(int(q.get("n", 50)), ACTOR))


def get_config_export(q):
    return fleetcfg.profile_export(q.get("profile") or None, ACTOR)


def config_change(fn, *args, **kw):
    """A fleetcfg change, with the database being down as a plain refusal; the drift cache is stale after it."""
    # a running spawn / sim start / base station job reads .env again partway (compose up): not under its feet
    check_free(FLEET_LOCK, BASESTATION_LOCK)
    try:
        r = fn(*args, actor=ACTOR, **kw)
    except fleetcfg.Unavailable as e:
        raise ValueError(f"can't change settings: {e}") from None
    _drift["t"] = 0.0
    _real_cache["t"] = 0.0
    return r if isinstance(r, dict) else {"ok": True}


def act_config_set(body):
    # {updates: {KEY: "value" | null (= back to the default)}}
    updates = body.get("updates")
    if not isinstance(updates, dict) or not updates:
        raise ValueError("updates: {KEY: value or null}")
    return config_change(fleetcfg.set_settings, {k: (None if v is None else str(v)) for k, v in updates.items()})


def act_config_slots(body):
    # {slots: [{model, x, y, yaw} | null, ...] for slots 0.., num_robots: N}
    slots = body.get("slots")
    if not isinstance(slots, list):
        raise ValueError("slots: a list")
    return config_change(fleetcfg.set_slots, slots, num_robots=body.get("num_robots"))


def act_config_profile(body):
    op, name = body.get("op"), (body.get("name") or "").strip()
    ops = {
        "use": lambda: config_change(fleetcfg.profile_use, name),
        "new": lambda: config_change(fleetcfg.profile_new, name, body.get("from") or None, body.get("notes") or ""),
        "rename": lambda: config_change(fleetcfg.profile_rename, name, (body.get("new_name") or "").strip()),
        "delete": lambda: config_change(fleetcfg.profile_delete, name),
        "notes": lambda: config_change(fleetcfg.profile_notes, name, body.get("notes") or ""),
        "import": lambda: config_change(fleetcfg.profile_import, body.get("data") or {}, name or None),
    }
    if op not in ops:
        raise ValueError(f"op must be one of {sorted(ops)}")
    if not name and op != "import":
        raise ValueError("name: which profile")
    return ops[op]()


def act_config_revert(body):
    return config_change(fleetcfg.revert, int(body.get("id")))


def act_config_apply(body):
    """Recreate what runs with old settings: the sim (fleet.sh scene: compose recreates it when its settings changed;
    its robots respawn by themselves), the robots (fleet.sh spawn: the slots' models and poses), the base station."""
    what = body.get("what")
    commands = {"sim": ["scripts/fleet.sh", "scene"], "robots": ["scripts/fleet.sh", "spawn"],
                "basestation": BASESTATION_COMPOSE + ["up", "-d"]}
    if what not in commands:
        raise ValueError(f"what must be one of {sorted(commands)}")
    if what == "robots":
        state = fleet_ctl.read_state()
        if not state or state.get("scene") != "ready":
            raise ValueError("the scene is not ready: start the sim first")
    _drift["t"] = 0.0
    return start_job(f"apply settings: {what}", lambda j: j.run(commands[what]) == 0,
                     lock=BASESTATION_LOCK if what == "basestation" else FLEET_LOCK)


# ---------------------------------------------------------------- harvesting operation (/operation)
# The coordinator (colcon_ws/src/status_server: task_manager + status_server) runs in the base station container
# under scripts/basestation_coordinator.sh while public.operation.coordinator_enabled is true; its state, the
# plants (harvest_state), the robots (robot_live) and the event log are in the farm database (migration 002). This
# page reads them with the container's psql (no ROS session from here), writes the operation row for its buttons,
# and keeps the sim's plants in step: a harvested plant is hidden (scripts/fleet_ctl.py plants), replayed after a
# sim restart.

OPERATION_PAGE = Path(__file__).with_name("operation.html")
OPERATION_EVERY_S = 2.0  # refresh period while the page polls
STATUS_SERVER_LOG = "/tmp/status_server.log"  # in the base station container (basestation_coordinator.sh)
COORDINATOR_MATCH = "[s]tatus_server.launch.py"

OPERATION_SQL = """
SELECT json_build_object(
  'operation', (SELECT row_to_json(o) FROM public.operation o WHERE id = 1),
  'rows', (SELECT json_agg(r ORDER BY r) FROM (SELECT DISTINCT row_id r FROM public.object_data) t),
  'counters', (SELECT COALESCE(json_object_agg(status, n), '{}'::json) FROM (
      SELECT h.status, count(*) n FROM public.harvest_state h
      WHERE (SELECT target_rows FROM public.operation WHERE id = 1) IS NULL
         OR h.row_id = ANY ((SELECT target_rows FROM public.operation WHERE id = 1)::int[]) GROUP BY h.status) c),
  'rate15', (SELECT count(*) FROM public.harvest_state WHERE harvested_at > now() - interval '15 minutes'),
  'plants_stamp', (SELECT max(updated_at) FROM public.harvest_state),
  'plants', (SELECT COALESCE(json_agg(json_build_array(h.object_id, h.row_id, round(o.x_coord::numeric, 2),
      round(o.y_coord::numeric, 2), h.status, h.assigned_robot_id, h.attempts, h.last_error) ORDER BY h.object_id), '[]'::json)
      FROM public.harvest_state h JOIN public.object_data o USING (object_id)),
  'robots', (SELECT COALESCE(json_agg(json_build_object('id', i.id, 'name', i.robot_name, 'serial', i.serial_number,
      'type', i.robot_type, 'online_flag', l.online_flag, 'status', l.status, 'task', l.task,
      'battery', l.battery_level, 'load', l.load_status, 'node', l.current_node_id, 'target_node', l.target_node_id,
      'x', (l.topo_map_pos).x, 'y', (l.topo_map_pos).y, 'yaw', (l.topo_map_pos).yaw, 'updated_at', l.updated_at,
      'enrolled', COALESCE(ro.enrolled, true), 'op_state', COALESCE(ro.state, 'active'), 'request', ro.request,
      'job', (SELECT json_build_object('id', j.id, 'type', j.task_type, 'schedule', j.job_schedule,
              'progress', j.progress_status) FROM public.farm_harvesting_job j
              WHERE j.assigned_robot_id = i.id AND j.progress_status IN (0, 1, 2) ORDER BY j.id DESC LIMIT 1))
      ORDER BY i.id), '[]'::json) FROM public.robot_info i LEFT JOIN public.robot_live l ON l.id = i.id
      LEFT JOIN public.robot_operation ro ON ro.robot_id = i.id),
  'docks', (SELECT COALESCE(json_agg(row_to_json(d) ORDER BY d.id), '[]'::json) FROM public.dock_station d),
  'chargers', (SELECT COALESCE(json_agg(row_to_json(c) ORDER BY c.id), '[]'::json) FROM public.charging_stations c),
  'events', (SELECT COALESCE(json_agg(e), '[]'::json) FROM (SELECT id, ts, source, robot, kind, object_id, job_id, message
      FROM public.operation_event ORDER BY id DESC LIMIT 80) e))
"""
OPERATION_SUMMARY_SQL = """
SELECT json_build_object('state', (SELECT state FROM public.operation WHERE id = 1),
  'harvested', (SELECT count(*) FROM public.harvest_state WHERE status = 'harvested'),
  'total', (SELECT count(*) FROM public.harvest_state))
"""


def bs_psql(sql, timeout=15):
    """SQL in the farm database through the base station container's psql (its trusted local socket): (code, out);
    one statement string, results tab-separated without headers."""
    return sh(["docker", "exec", BASESTATION, "psql", "-X", "-At", "-q", "-v", "ON_ERROR_STOP=1", "-c", sql],
              timeout=timeout)


def sql_str(value):
    return "'" + str(value).replace("'", "''") + "'"


class OperationWatch:
    """The operation page's data, refreshed in the background every OPERATION_EVERY_S while the page polls: one
    psql query (OPERATION_SQL), the coordinator's processes and log tail, and the plant bridge to the sim."""

    def __init__(self):
        self.lock = threading.Lock()
        self.data, self.t, self.busy = {}, 0.0, False
        self.summary_data, self.summary_t = None, 0.0
        self.bridge = {}

    def poll(self):
        with self.lock:
            if not self.busy and time.time() - self.t > OPERATION_EVERY_S:
                self.busy = True
                threading.Thread(target=self.refresh, daemon=True).start()
            # the first poll after an idle spell answers with what was cached: say so, the next poll is fresh
            return dict(self.data, stale=time.time() - self.data.get("probed", 0) > 3 * OPERATION_EVERY_S)

    def summary(self):
        """For /api/status (the main page's pill): the operation state and progress, at most every 10 s."""
        with self.lock:
            data = self.data
        db = data.get("db")
        if db and time.time() - data.get("probed", 0) < 10:
            c = db.get("counters") or {}
            return {"state": (db.get("operation") or {}).get("state"), "harvested": c.get("harvested", 0),
                    "total": sum(c.values()) if c else 0}
        if time.time() - self.summary_t < 10:
            return self.summary_data
        self.summary_t = time.time()
        self.summary_data = None
        if container_state(BASESTATION) == "running":
            code, out = bs_psql(OPERATION_SUMMARY_SQL, timeout=8)
            if code == 0:
                try:
                    self.summary_data = json.loads(out.strip())
                except ValueError:
                    pass
        return self.summary_data

    def refresh(self):
        data = {"basestation": container_state(BASESTATION)}
        try:
            if data["basestation"] == "running":
                code, out = bs_psql(OPERATION_SQL, timeout=15)
                if code == 0:
                    try:
                        data["db"] = json.loads(out.strip())
                    except ValueError as e:
                        data["db_error"] = f"bad JSON from psql: {e}"
                else:
                    data["db_error"] = (out.strip()[-400:] or f"psql exit {code}"
                                        ) + ("" if "operation" not in out else " (migration 002 missing?)")
                data["coordinator"] = self.coordinator()
                if "db" in data:
                    self.bridge = self.sync_plants(data["db"])
            data["bridge"] = dict(self.bridge)
        except (subprocess.TimeoutExpired, ValueError, OSError) as e:
            data["db_error"] = repr(e)
        finally:
            with self.lock:
                self.data, self.t, self.busy = dict(data, probed=time.time()), time.time(), False

    def coordinator(self):
        """status_server's processes in the base station container and the tail of its log."""
        script = (f"pgrep -f '{COORDINATOR_MATCH}' | head -1; echo ---; "
                  "pgrep -af 'lib/status_server/(status_server|task_manager)' | wc -l; echo ---; "
                  f"tail -n 30 {STATUS_SERVER_LOG} 2>/dev/null; echo ---; tail -n 6 /tmp/coordinator.log 2>/dev/null; "
                  "echo ---; test -d /home/robot/colcon_ws/install/status_server && echo built; echo ---; "
                  "test -f /scripts/basestation_coordinator.sh && pgrep -f '[b]asestation_coordinator.sh' >/dev/null && echo supervised")
        code, out = sh(["docker", "exec", BASESTATION, "bash", "-c", script], timeout=15)
        if code != 0:
            return {"error": out.strip()[-300:] or f"exit {code}"}
        parts = (out.split("---\n") + [""] * 6)[:6]
        pid, nodes, log_tail, sup_tail, built, supervised = [p.strip() for p in parts]
        return {"running": bool(pid), "pid": pid, "nodes": int(nodes or 0), "log_tail": log_tail[-4000:],
                "supervisor_tail": sup_tail[-1200:], "built": built == "built", "supervised": supervised == "supervised"}

    def sync_plants(self, db):
        """The sim's plants follow the database: hide the harvested ones it still shows, show the ones it hides
        that aren't harvested any more (Reset harvest state). Covers new harvests, a sim restart (plants_hidden
        empty again) and Reset scene. One request per refresh."""
        state = fleet_ctl.read_state()
        if not state or state.get("scene") != "ready":
            return {"sim": "not running"}
        if "plants" not in (state.get("actions") or []):
            return {"sim": "this sim predates the plants action: restart it (docker restart a300-isaac-sim)"}
        harvested = {int(p[0]) for p in db.get("plants") or [] if p[4] == "harvested"}
        hidden = {int(i) for i in state.get("plants_hidden") or []}
        hide, show = harvested - hidden, hidden - harvested
        r = dict(self.bridge, sim="ok", hidden=len(hidden), harvested=len(harvested))
        if hide or show:
            answer = fleet_ctl.plants(hide, show, timeout=20, log=lambda m: None)
            r.update(last_sync=time.time(), last_hide=len(hide), last_show=len(show),
                     error=None if answer and answer.get("state") == "done" else (answer or {}).get("message", "no answer"))
            if answer and answer.get("state") == "done":
                r["hidden"] = len(hidden | hide) - len(show & hidden)
        return r


OPERATION_WATCH = OperationWatch()


def get_operation(q):
    return OPERATION_WATCH.poll()


def get_operation_log(q):
    n = max(10, min(int(q.get("n", 300)), 2000))
    code, out = sh(["docker", "exec", BASESTATION, "bash", "-c", f"tail -n {n} {STATUS_SERVER_LOG} 2>/dev/null; "
                    "echo; echo '--- supervisor (/tmp/coordinator.log)'; tail -n 20 /tmp/coordinator.log 2>/dev/null"],
                   timeout=15)
    return {"log": out if code == 0 else f"exit {code}: {out}"}


GRAPH_CACHE = {"t": 0.0, "data": None}


def get_operation_graph(q):
    """The route graph (graph_node / graph_edge) for the field map's overlay; cached a minute."""
    if time.time() - GRAPH_CACHE["t"] < 60 and GRAPH_CACHE["data"]:
        return GRAPH_CACHE["data"]
    code, out = bs_psql("SELECT json_build_object('nodes', (SELECT json_agg(json_build_array(id, node_type, round(x::numeric, 3), "
                        "round(y::numeric, 3), round(theta::numeric, 3), obj_id) ORDER BY id) FROM public.graph_node), "
                        "'edges', (SELECT json_agg(json_build_array(source_id, target_id)) FROM public.graph_edge))", timeout=20)
    if code != 0:
        raise ValueError(out.strip()[-300:] or f"psql exit {code}")
    GRAPH_CACHE.update(t=time.time(), data=json.loads(out.strip()))
    return GRAPH_CACHE["data"]


def operation_state():
    code, out = bs_psql("SELECT state FROM public.operation WHERE id = 1", timeout=8)
    if code != 0:
        raise ValueError("the farm database is not reachable: " + (out.strip()[-200:] or f"psql exit {code}"))
    return out.strip()


def act_operation_control(body):
    """Start / pause / resume / stop the operation, set its target rows and max attempts: the operation row (the
    coordinator reads it every tick), plus an event. Synchronous (<1 s): no job."""
    if container_state(BASESTATION) != "running":
        raise ValueError("the base station container is not running")
    action = body.get("action")
    event = lambda kind, msg: ("INSERT INTO public.operation_event (source, kind, message) VALUES "
                               f"('web_ui', {sql_str(kind)}, {sql_str(msg)});")
    if action == "start":
        sql = ("UPDATE public.operation SET state = 'running', coordinator_enabled = true, "
               "started_at = COALESCE(started_at, now()), updated_at = now() WHERE id = 1 "
               "AND state IN ('stopped', 'complete', 'paused');" + event("start", "operation started"))
    elif action == "pause":
        sql = ("UPDATE public.operation SET state = 'paused', updated_at = now() WHERE id = 1 AND state = 'running';"
               + event("pause", "operation paused: no new harvest tasks, robots finish their current one"))
    elif action == "resume":
        sql = ("UPDATE public.operation SET state = 'running', updated_at = now() WHERE id = 1 AND state = 'paused';"
               + event("resume", "operation resumed"))
    elif action == "stop":
        sql = ("UPDATE public.operation SET state = 'stopped', updated_at = now() WHERE id = 1 "
               "AND state IN ('running', 'paused');" + event("stop", "operation stopped: harvest tasks cancelled, robots park at their chargers"))
    elif action == "target":
        rows = body.get("rows")
        if rows is not None and not (isinstance(rows, list) and all(isinstance(r, int) and 0 <= r < 10000 for r in rows)):
            raise ValueError("rows must be a list of row numbers, or null for every row")
        rows_sql = "NULL" if not rows else "ARRAY[" + ",".join(str(r) for r in sorted(set(rows))) + "]::int[]"
        sql = (f"UPDATE public.operation SET target_rows = {rows_sql}, updated_at = now() WHERE id = 1;"
               + event("target", "target rows: " + ("all" if not rows else ", ".join(str(r) for r in sorted(set(rows))))))
    elif action == "max_attempts":
        n = body.get("max_attempts")
        if not isinstance(n, int) or not 1 <= n <= 20:
            raise ValueError("max_attempts must be 1-20")
        sql = f"UPDATE public.operation SET max_attempts = {n}, updated_at = now() WHERE id = 1;" + event("target", f"max attempts {n}")
    else:
        raise ValueError(f"unknown action {action!r}")
    code, out = bs_psql(sql, timeout=10)
    if code != 0:
        raise ValueError(out.strip()[-300:] or f"psql exit {code}")
    with OPERATION_WATCH.lock:
        OPERATION_WATCH.t = 0.0  # refresh at the next poll
    return {"ok": True, "state": operation_state()}


ROBOT_ACTIONS = {
    # action -> (SQL on robot_operation for robot %(id)s, event kind, event message)
    "add": ("enrolled = true", "add", "included in the operation by the operator"),
    "remove": ("enrolled = false", "remove", "excluded from the operation by the operator (its current task finishes)"),
    "start": ("state = 'active', request = NULL", "start", "started by the operator"),
    "resume": ("state = 'active', request = NULL", "resume", "resumed by the operator"),
    "pause": ("state = 'paused'", "pause", "paused by the operator (its current task finishes, then nothing new)"),
    "stop": ("state = 'stopped', request = 'stop'", "stop", "stop requested by the operator: its task is cancelled"),
    "charge": ("request = 'charge'", "charge", "charging requested by the operator"),
}


def act_operation_robot(body):
    """One robot's operator controls (the Robots card): robot_operation row + an event; the coordinator acts on
    its next tick (enrolled / state gate the next assignment; a request is one-shot: charge now, stop now)."""
    if container_state(BASESTATION) != "running":
        raise ValueError("the base station container is not running")
    serial, action = body.get("serial"), body.get("action")
    if not isinstance(serial, str) or not NAME_RE.match(serial):
        raise ValueError("bad robot serial")
    if action not in ROBOT_ACTIONS:
        raise ValueError(f"unknown action {action!r}")
    set_sql, kind, message = ROBOT_ACTIONS[action]
    code, out = bs_psql(f"SELECT id FROM public.robot_info WHERE serial_number = {sql_str(serial)}", timeout=8)
    if code != 0:
        raise ValueError(out.strip()[-300:] or f"psql exit {code}")
    if not out.strip():
        raise ValueError(f"{serial} is not in robot_info")
    sql = (f"INSERT INTO public.robot_operation (robot_id) SELECT id FROM public.robot_info WHERE serial_number = {sql_str(serial)} "
           "ON CONFLICT (robot_id) DO NOTHING;"
           f"UPDATE public.robot_operation SET {set_sql}, updated_at = now() WHERE robot_id = "
           f"(SELECT id FROM public.robot_info WHERE serial_number = {sql_str(serial)});"
           f"INSERT INTO public.operation_event (source, robot, kind, message) VALUES ('web_ui', {sql_str(serial)}, "
           f"{sql_str(kind)}, {sql_str(message)});"
           f"SELECT enrolled, state, request FROM public.robot_operation WHERE robot_id = "
           f"(SELECT id FROM public.robot_info WHERE serial_number = {sql_str(serial)})")
    code, out = bs_psql(sql, timeout=10)
    if code != 0:
        raise ValueError(out.strip()[-300:] or f"psql exit {code}")
    if not out.strip():
        raise ValueError(f"{serial} is not in robot_info")
    with OPERATION_WATCH.lock:
        OPERATION_WATCH.t = 0.0
    enrolled, state, request = (out.strip().split("|") + ["", "", ""])[:3]
    return {"ok": True, "serial": serial, "enrolled": enrolled == "t", "state": state, "request": request or None}


def act_operation_coordinator(body):
    """Run or stop the coordinator (status_server) in the base station: the supervisor there follows the flag."""
    if container_state(BASESTATION) != "running":
        raise ValueError("the base station container is not running")
    action = body.get("action")
    if action not in ("start", "stop"):
        raise ValueError("action must be start or stop")
    check_free(BASESTATION_LOCK)

    def fn(j):
        want = "true" if action == "start" else "false"
        code, out = bs_psql(f"UPDATE public.operation SET coordinator_enabled = {want}, updated_at = now() WHERE id = 1;"
                            f"INSERT INTO public.operation_event (source, kind, message) VALUES ('web_ui', 'coordinator', "
                            f"'coordinator {action} requested')", timeout=10)
        if code != 0:
            j.log(out.strip()[-300:])
            return False
        j.log(f"operation.coordinator_enabled = {want}; waiting for the supervisor in {BASESTATION}...")
        for _ in range(30):
            time.sleep(1)
            c = OPERATION_WATCH.coordinator()
            if c.get("error"):
                j.log(c["error"])
                return False
            if c.get("running") == (action == "start"):
                j.log(f"status_server {'running' if c['running'] else 'stopped'} ({c.get('nodes', 0)} node processes)")
                if c.get("log_tail"):
                    j.log(c["log_tail"][-800:])
                return True
            if action == "start" and not c.get("built"):
                j.log("colcon_ws/install/status_server is missing: build it with scripts/colcon_build.sh (from a robot container)")
                return False
            if action == "start" and not c.get("supervised"):
                j.log("no supervisor in the container: recreate the base station (its entrypoint starts "
                      "scripts/basestation_coordinator.sh), or run it by hand: docker exec -d basestation bash /scripts/basestation_coordinator.sh")
                return False
        j.log("no change after 30 s; see the coordinator log")
        return False

    return start_job(f"coordinator {action}", fn, lock=BASESTATION_LOCK)


def act_operation_reset(body):
    """Every plant pending again (attempts 0), open harvest jobs released, the sim's plants shown again. Only while
    the operation is stopped or complete and no harvest task is open."""
    if container_state(BASESTATION) != "running":
        raise ValueError("the base station container is not running")
    check_free(BASESTATION_LOCK)
    state = operation_state()
    if state not in ("stopped", "complete"):
        raise ValueError(f"the operation is {state}: stop it first")

    def fn(j):
        sql = ("UPDATE public.farm_harvesting_job SET progress_status = 5 WHERE task_type = 1 AND progress_status IN (0, 1, 2);"
               "UPDATE public.harvest_state SET status = 'pending', attempts = 0, last_error = NULL, assigned_robot_id = NULL, "
               "job_id = NULL, lane_id = NULL, pickup_node_id = NULL, assigned_at = NULL, harvested_at = NULL, updated_at = now();"
               "UPDATE public.operation SET started_at = NULL, updated_at = now() WHERE id = 1;"
               "INSERT INTO public.operation_event (source, kind, message) VALUES ('web_ui', 'reset', 'harvest state reset: every plant pending');"
               "SELECT count(*) FROM public.harvest_state")
        code, out = bs_psql(sql, timeout=15)
        if code != 0:
            j.log(out.strip()[-300:])
            return False
        j.log(f"{out.strip()} plants pending again")
        state = fleet_ctl.read_state()
        if state and state.get("scene") == "ready" and "plants" in (state.get("actions") or []):
            answer = fleet_ctl.plants((), "all", timeout=20, log=j.log)
            j.log(f"sim: {answer}")
        with OPERATION_WATCH.lock:
            OPERATION_WATCH.t = 0.0
        return True

    return start_job("reset harvest state", fn, lock=BASESTATION_LOCK)


POST = {
    "/api/sim/start": act_sim_start,
    "/api/sim/stop": act_sim_stop,
    "/api/sim/reset": act_sim_reset,
    "/api/scene/upload": act_scene_upload,
    "/api/spawn": act_spawn,
    "/api/launch/start": act_launch_start,
    "/api/launch/stop": act_launch_stop,
    "/api/real/restart": act_real_restart,
    "/api/real/restart/cancel": act_real_restart_cancel,
    "/api/real/save": act_real_save,
    "/api/real/remove": act_real_remove,
    "/api/deploy": act_deploy,
    "/api/basestation/link": act_basestation_link,
    "/api/basestation/action": act_basestation,
    "/api/arm/goto": act_arm_goto,
    "/api/cutstem/start": act_cutstem_start,
    "/api/cutstem/stop": act_cutstem_stop,
    "/api/stop": act_stop,
    "/api/stop_all": act_stop_all,
    "/api/rviz/start": act_rviz_start,
    "/api/mocap/assign": act_mocap_assign,
    "/api/mocap/calibrate": act_mocap_calibrate,
    "/api/mocap/calibration/save": act_mocap_calibration_save,
    "/api/localization/set": act_localization_set,
    "/api/config/set": act_config_set,
    "/api/config/slots": act_config_slots,
    "/api/config/profile": act_config_profile,
    "/api/config/revert": act_config_revert,
    "/api/config/apply": act_config_apply,
    "/api/operation/control": act_operation_control,
    "/api/operation/coordinator": act_operation_coordinator,
    "/api/operation/robot": act_operation_robot,
    "/api/operation/reset": act_operation_reset,
}
GET = {
    "/api/status": status,
    "/api/arm/states": get_arm_states,
    "/api/joints": get_joints,
    "/api/launch/log": get_launch_log,
    "/api/mocap": get_mocap,
    "/api/localization": get_localization,
    "/api/link": get_link,
    "/api/basestation": get_basestation,
    "/api/basestation/log": get_basestation_log,
    "/api/jobs": lambda q: [j.to_json() for j in sorted(JOBS.values(), key=lambda j: -j.started)],
    "/api/job": lambda q: JOBS[q["id"]].to_json(full=True),
    "/api/config": get_config,
    "/api/config/history": get_config_history,
    "/api/config/runs": get_config_runs,
    "/api/config/export": get_config_export,
    "/api/operation": get_operation,
    "/api/operation/log": get_operation_log,
    "/api/operation/graph": get_operation_graph,
}


class Handler(BaseHTTPRequestHandler):
    def reply(self, code, payload, ctype="application/json"):
        data = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass  # the page went away mid-request (reload/close during a ~1.5 s status poll); nothing to answer

    def reply_file(self, path, ctype):
        """A file, with byte ranges (a <video> seeks with them)."""
        size = path.stat().st_size
        m = re.fullmatch(r"bytes=(\d*)-(\d*)", self.headers.get("Range") or "")
        start, end = 0, size - 1
        if m and (m[1] or m[2]):
            start = int(m[1]) if m[1] else max(size - int(m[2]), 0)
            end = min(int(m[2]), size - 1) if m[1] and m[2] else size - 1
            if start > end or start >= size:
                self.send_response(416)
                self.send_header("Content-Range", f"bytes */{size}")
                self.end_headers()
                return
        self.send_response(206 if m and (m[1] or m[2]) else 200)
        self.send_header("Content-Type", ctype)
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Length", str(end - start + 1))
        if m and (m[1] or m[2]):
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.end_headers()
        try:
            with open(path, "rb") as f:
                f.seek(start)
                left = end - start + 1
                while left > 0:
                    chunk = f.read(min(1 << 16, left))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    left -= len(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def handle_api(self, table, arg):
        path = urlparse(self.path).path
        if path not in table:
            return self.reply(404, {"error": "not found"})
        try:
            out = table[path](arg)
            self.reply(200, out.to_json() if isinstance(out, Job) else out)
        except NeedPassword as e:
            self.reply(400, {"error": str(e), "need_password": True})
        except (ValueError, KeyError) as e:
            self.reply(400, {"error": str(e)})
        except fleetcfg.Unavailable as e:
            self.reply(503, {"error": str(e)})
        except subprocess.TimeoutExpired:
            self.reply(504, {"error": "command timed out"})

    def do_GET(self):
        url = urlparse(self.path)
        if url.path in ("/", "/index.html"):
            return self.reply(200, PAGE.read_bytes(), "text/html; charset=utf-8")
        if url.path in ("/config", "/config.html"):
            return self.reply(200, CONFIG_PAGE.read_bytes(), "text/html; charset=utf-8")
        if url.path in ("/operation", "/operation.html"):
            return self.reply(200, OPERATION_PAGE.read_bytes(), "text/html; charset=utf-8")
        self.handle_api(GET, {k: v[0] for k, v in parse_qs(url.query).items()})

    def do_POST(self):
        # A cross-site page can only send a JSON content type after a CORS preflight, which this server never
        # answers -- so requiring it keeps other websites from driving docker through this port.
        if not (self.headers.get("Content-Type") or "").startswith("application/json"):
            return self.reply(415, {"error": "expected application/json"})
        length = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except json.JSONDecodeError:
            return self.reply(400, {"error": "bad json"})
        # A password (a robot's sudo) only from this machine: with --host 0.0.0.0 it would cross the LAN as plain HTTP.
        if isinstance(body, dict) and "password" in body and self.client_address[0] not in ("127.0.0.1", "::1"):
            return self.reply(403, {"error": "passwords are only accepted from this machine (127.0.0.1)"})
        self.handle_api(POST, body)

    def log_message(self, *args):
        pass


def main():
    global DEFAULT_MODE
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--mode", choices=MODES, default=DEFAULT_MODE,
                    help="what the page shows until the browser picks its own: sim, real robots, or both")
    args = ap.parse_args()
    DEFAULT_MODE = args.mode
    print(f"fleet UI on http://{args.host}:{args.port} (fleet: {ROOT}, mode {args.mode})")
    try:
        ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        LINK.stop_all()  # its pings and the base station's link_monitor.py would outlive the server


if __name__ == "__main__":
    main()

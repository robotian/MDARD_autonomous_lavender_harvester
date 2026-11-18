#!/usr/bin/env python3
"""Host side of the sim's spawn protocol (sim/scripts/setup_scene.py: FLEET_REQUEST / FLEET_STATE).

The sim builds the scene with no robots and then watches sim/generated/fleet/spawn_request.json; each request with
a new id replaces the robots in the scene. It reports progress in sim/generated/fleet/state.json.

  scripts/fleet_ctl.py wait-scene            wait until the running sim's scene is ready (and a pending spawn done)
  scripts/fleet_ctl.py spawn [--poses JSON]  spawn NUM_ROBOTS robots (models ROBOT_MODEL_<i> from .env), wait
  scripts/fleet_ctl.py reset                 stop + play the sim's timeline (robots back at their spawn
                                             state), then restart the robot containers
  scripts/fleet_ctl.py snapshot [--poses J]  viewport PNGs of the robots now (front/rear/close/wide) into
                                             sim/generated/snapshots/
  scripts/fleet_ctl.py plants --hide 1,2,3   hide harvested plants (object ids) in the running sim, or
  scripts/fleet_ctl.py plants --show all     show them again (the web UI's operation page does this from the DB)
  scripts/fleet_ctl.py camera X,Y,Z X,Y,Z  aim the sim's viewport: eye, then target (world frame, m); kept
                                             across spawns until the sim restarts (camera default: the usual view)
  scripts/fleet_ctl.py camera --follow cutter X,Y,Z X,Y,Z  the viewport rides with the VBD plant's cutter: eye and
                                             target in its CAD frame (mm; vbd_plant.py to_cad), until another camera
  scripts/fleet_ctl.py camera --follow cutter --view close X,Y,Z X,Y,Z  the same for the recorder's second view
                                             (a close-up camera of its own, saved as close_%06d.jpg beside the frames)
  scripts/fleet_ctl.py record DIR [--fps N] [--res WxH]  save the viewport as JPEG frames into sim/generated/
                                             recordings/DIR (works on a locked screen); record --stop ends it
  scripts/fleet_ctl.py sun HOUR | --from H --to H --duration S  the sun at a local time of day (Houghton, MI,
                                             June 21), or moving between two over S wall-clock seconds
  scripts/fleet_ctl.py clouds [--cover C] [--speed M/S] [--heading DEG] [--altitude M] [--seed N] | off
                                             a drifting cloud layer (its shadows cross the field); off removes it
  scripts/fleet_ctl.py clear                 delete the request (the next sim start has no robots)
  scripts/fleet_ctl.py state                 print the sim's state (null if the sim isn't running / stale)

Poses: --poses '[{"x":0,"y":-1.6,"yaw":0}, ...]' (one per slot, null or missing keys = default), else
ROBOT_POSE_<i>="x,y,yaw" in .env (a slot's pose in the settings), else the sim's default layout (state.json's default_poses). x/y in metres in the
world frame, yaw in degrees. Normally run through scripts/fleet.sh (which renders .env from the settings database
first and also starts the robot containers).
Stdlib only; tools/sim_ui/server.py imports it.
"""
import argparse
import datetime
import json
import os
import subprocess
import sys
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FLEET_DIR = ROOT / "sim/generated/fleet"
REQUEST = FLEET_DIR / "spawn_request.json"
STATE = FLEET_DIR / "state.json"
APPLIED = FLEET_DIR / "applied_request.json"  # written by the sim: the last request it spawned successfully
CONTROL = FLEET_DIR / "control.json"  # {id, action: "reset"|"snapshot"|"plants"}: timeline stop+play / PNGs / plants
SIM = "a300-isaac-sim"
PROJECT = "clearpath-fleet"  # docker-compose.yml name:
MAX_SLOTS = 8


def read_env():
    env = {}
    path = ROOT / ".env"
    if path.exists():
        for line in path.read_text().splitlines():
            if "=" in line and not line.lstrip().startswith("#"):
                k, v = line.split("=", 1)
                env[k.strip()] = v.strip()
    return env


def sim_started_at():
    """Start time (epoch s) of the running sim container, or None if it isn't running."""
    p = subprocess.run(["docker", "inspect", "-f", "{{.State.Running}} {{.State.StartedAt}}", SIM],
                       capture_output=True, text=True)
    if p.returncode != 0 or not p.stdout.startswith("true"):
        return None
    ts = p.stdout.split()[1]  # 2026-10-01T15:04:05.123456789Z
    base, _, frac = ts.rstrip("Z").partition(".")
    t = datetime.datetime.fromisoformat(base).replace(tzinfo=datetime.timezone.utc).timestamp()
    return t + float(f"0.{frac or 0}")


def read_state():
    """The sim's state.json if it belongs to the sim container running now, else None."""
    started = sim_started_at()
    if started is None:
        return None
    try:
        state = json.loads(STATE.read_text())
    except (OSError, ValueError):
        return None
    # written by this container's run of setup_scene.py (not a previous one)? Same host clock, 5 s of slack.
    return state if state.get("started", 0) >= started - 5 else None


def read_request(applied=False):
    """The last spawn request written (applied=True: the last one the sim spawned successfully)."""
    try:
        return json.loads((APPLIED if applied else REQUEST).read_text())
    except (OSError, ValueError):
        return None


def write_request(robots, at_start=False):
    """Write a spawn request ({model, x?, y?, yaw?} per slot); returns its id. at_start: only for the sim's next start
    (the running sim ignores it), for replacing robots by restarting the sim."""
    FLEET_DIR.mkdir(parents=True, exist_ok=True)
    try:
        os.chmod(FLEET_DIR, 0o777)  # the sim (uid 1234) writes state.json here
    except OSError:
        pass
    req = {"id": uuid.uuid4().hex[:12], "written": time.time(), "robots": robots}
    if at_start:
        req["at_start"] = True
    tmp = FLEET_DIR / f".spawn_request.{os.getpid()}.tmp"
    tmp.write_text(json.dumps(req, indent=1))
    os.chmod(tmp, 0o666)
    os.replace(tmp, REQUEST)  # atomic: the sim never reads half a file
    return req["id"]


def clear_request():
    REQUEST.unlink(missing_ok=True)
    APPLIED.unlink(missing_ok=True)


def slot_robots(poses=None, env=None):
    """The requested robots from .env (NUM_ROBOTS, ROBOT_MODEL_<i>, ROBOT_POSE_<i>) and optional per-slot poses."""
    env = env if env is not None else read_env()
    n = int(env.get("NUM_ROBOTS") or 0)
    if not 0 <= n <= MAX_SLOTS:
        raise ValueError(f"NUM_ROBOTS must be 0-{MAX_SLOTS}")
    poses = poses or []
    robots = []
    for i in range(n):
        r = {"model": env.get(f"ROBOT_MODEL_{i}") or "a300"}
        pose = poses[i] if i < len(poses) else None
        if pose is None and env.get(f"ROBOT_POSE_{i}"):
            vals = [v.strip() for v in env[f"ROBOT_POSE_{i}"].split(",")]
            pose = dict(zip(("x", "y", "yaw"), (float(v) for v in vals if v)))
        for k in ("x", "y", "yaw"):
            if pose and pose.get(k) not in (None, ""):
                r[k] = float(pose[k])
        robots.append(r)
    return robots


def wait(pred, timeout, what, log=print):
    """Poll read_state() until pred(state) returns a non-None result (True = ok, a string = error)."""
    t0, last = time.time(), None
    while time.time() - t0 < timeout:
        state = read_state()
        if state is None and sim_started_at() is None:
            log("the sim container is not running")
            return False
        res = pred(state) if state else None
        if res is True:
            return True
        if isinstance(res, str):
            log(res)
            return False
        msg = f"waiting for {what}: " + ("sim starting" if state is None else f"scene {state.get('scene')}")
        if msg != last:
            log(msg)
            last = msg
        time.sleep(1)
    log(f"timed out after {timeout:.0f} s waiting for {what}")
    return False


def scene_ready(state):
    if state["scene"] == "error":
        return "scene failed to build:\n" + state.get("error", "")
    return True if state["scene"] == "ready" else None


def log_robots(state, log=print):
    for r in state["robots"]:
        log(f"  slot {r['slot']}: {r['ns']:<12} x={r['x']:g} y={r['y']:g} yaw={r['yaw']:g} deg")


def wait_scene(timeout=900, log=print):
    """Wait for the (re)started sim's scene and for the robots it spawns at start (a pending request it never
    answered, else the last applied one -- see setup_scene.spawn_loop)."""
    t0 = time.time()
    if not wait(scene_ready, timeout, "the scene", log):
        return False
    state = read_state()
    log(f"scene ready ({state.get('scene_source', 'lavender')}) after {time.time() - t0:.0f} s; models: {', '.join(state['models'])}")
    if not wait(lambda s: True if s.get("boot") == "done" else None, timeout, "the start-up spawn", log):
        return False
    state = read_state()
    if state.get("spawn"):
        log(f"spawned: {state['spawn']['message']}")
        log_robots(state, log)
    return True


def wait_spawn(req_id, timeout=600, log=print):
    def done(state):
        sp = state.get("spawn") or {}
        if sp.get("id") != req_id or sp.get("state") == "spawning":
            return None
        return True if sp["state"] == "done" else f"spawn failed: {sp.get('message')}"

    if not wait(done, timeout, "the robots to spawn", log):
        return False
    state = read_state()
    log(f"spawned: {state['spawn']['message']}")
    log_robots(state, log)
    return True


def robot_containers():
    """Names of the running robot containers (compose services robot0..robot7)."""
    p = subprocess.run(["docker", "ps", "--filter", f"label=com.docker.compose.project={PROJECT}",
                        "--format", '{{.Names}}\t{{.Label "com.docker.compose.service"}}'],
                       capture_output=True, text=True)
    rows = [line.split("\t") for line in p.stdout.splitlines()]
    return [name for name, service in rows if service.startswith("robot") and service[5:].isdigit()]


def reset(timeout=60, restart_robots=True, log=print):
    """Put the robots back at their spawn state: the running sim stops and plays its timeline (Isaac's Stop and Play
    buttons) -- seconds, not the ~1 min of restarting the sim container. With restart_robots the robot containers
    are stopped *first* (a cut_stem run, MoveIt, the EKF can't keep acting on the restarted scene), then the sim is
    reset, then they are started again and boot fresh; whatever was launched by hand must be started again."""
    state = read_state()
    if not state or state.get("scene") != "ready":
        log("the sim is not running with a ready scene -- press Start first")
        return False
    t0 = time.time()
    names = robot_containers() if restart_robots else []
    if names:
        log(f"killing robot containers: {', '.join(names)}")
        # SIGKILL, all at once: they boot fresh afterwards, so a clean shutdown buys nothing, and `docker stop`'s
        # grace period let a running cut_stem keep moving the arm for ~4 s after the click
        p = subprocess.run(["docker", "kill", *names], capture_output=True, text=True)
        if p.returncode != 0:
            log(p.stderr.strip())
            return False
        log(f"killed in {time.time() - t0:.1f} s")
    FLEET_DIR.mkdir(parents=True, exist_ok=True)
    ctl = {"id": uuid.uuid4().hex[:12], "written": time.time(), "action": "reset"}
    tmp = FLEET_DIR / f".control.{os.getpid()}.tmp"
    tmp.write_text(json.dumps(ctl))
    os.chmod(tmp, 0o666)
    os.replace(tmp, CONTROL)
    log(f"reset {ctl['id']}: stop + play the timeline")
    t1, done = time.time(), False
    while time.time() - t1 < timeout:
        r = (read_state() or {}).get("reset") or {}
        if r.get("id") == ctl["id"] and r.get("state") == "done":
            done = True
            log(f"sim reset in {time.time() - t1:.1f} s")
            break
        time.sleep(0.3)
    if not done:
        log(f"no answer from the sim after {timeout:.0f} s")
    if names:  # started again even if the sim didn't answer, so the fleet isn't left down
        p = subprocess.run(["docker", "start", *names], capture_output=True, text=True)
        if p.returncode != 0:
            log(p.stderr.strip())
            return False
        log(f"robot containers started; done in {time.time() - t0:.1f} s (their boot -- robot.srdf etc. -- takes "
            f"~10-30 s more)")
    return done


def snapshot(poses=None, views=None, tag="now", timeout=120, log=print):
    """Viewport PNGs of the robots as they are now, from the sim's SNAPSHOT_VIEWS (front, rear, close, wide) into
    sim/generated/snapshots/<ns>_<view>_<tag>.png. poses {ns: [x, y, yaw_deg]}: where each robot is (world frame,
    e.g. its ref_pose); without it the sim uses the chassis prim's transform, which can lag a robot that drove."""
    state = read_state()
    if not state or state.get("scene") != "ready" or not state.get("robots"):
        log("the sim is not running with robots")
        return False
    FLEET_DIR.mkdir(parents=True, exist_ok=True)
    ctl = {"id": uuid.uuid4().hex[:12], "written": time.time(), "action": "snapshot", "tag": tag,
           "dir": "/sim/generated/snapshots", "poses": poses, "views": views}
    tmp = FLEET_DIR / f".control.{os.getpid()}.tmp"
    tmp.write_text(json.dumps(ctl))
    os.chmod(tmp, 0o666)
    os.replace(tmp, CONTROL)
    log(f"snapshot {ctl['id']}")
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = (read_state() or {}).get("snapshot") or {}
        if r.get("id") == ctl["id"] and r.get("state") in ("done", "error"):
            if r["state"] == "error":
                log(f"snapshot failed: {r.get('message')}")
                return False
            for f in r.get("files", []):
                log(str(ROOT / f.removeprefix("/")))
            return True
        time.sleep(0.5)
    log(f"no answer from the sim after {timeout:.0f} s")
    return False


def write_control(ctl):
    """Write a control request for the running sim (one at a time: a newer one replaces an unanswered one)."""
    FLEET_DIR.mkdir(parents=True, exist_ok=True)
    ctl = dict(ctl, id=uuid.uuid4().hex[:12], written=time.time())
    tmp = FLEET_DIR / f".control.{os.getpid()}.tmp"
    tmp.write_text(json.dumps(ctl))
    os.chmod(tmp, 0o666)
    os.replace(tmp, CONTROL)
    return ctl["id"]


def plants(hide=(), show=(), timeout=30, log=print):
    """Hide (harvested) plants by object_id in the running sim, or show them again (show="all" for every hidden one).
    Returns the sim's answer {state, hidden, shown, unknown} or None. Refused unless the running sim advertises the
    action (state.json actions): an older sim resets its timeline on an unknown action."""
    state = read_state()
    if not state or state.get("scene") != "ready":
        log("the sim is not running with a ready scene")
        return None
    if "plants" not in (state.get("actions") or []):
        log("this sim doesn't support the plants action: restart it (docker restart a300-isaac-sim)")
        return None
    hide, show = sorted({int(i) for i in hide}), "all" if show == "all" else sorted({int(i) for i in show})
    if not hide and not show:
        return {"state": "done", "hidden": 0, "shown": 0, "unknown": []}
    cid = write_control({"action": "plants", "hide": hide, "show": show})
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = (read_state() or {}).get("plants") or {}
        if r.get("id") == cid:
            if r.get("state") == "error":
                log(f"plants failed: {r.get('message')}")
            return r
        time.sleep(0.3)
    log(f"no answer from the sim after {timeout:.0f} s")
    return None


def camera(eye, target, timeout=30, log=print, follow=None, view=None):
    """Aim the running sim's viewport from eye at target ([x, y, z] each, world frame); eye None: the default view.
    The sim keeps the view across spawns and snapshots until it restarts. follow "cutter": eye and target in the VBD
    plant's cutter's CAD frame (mm), the camera riding with it; view "close" aims the recorder's second view (a
    close-up camera of its own) that way instead of the viewport. Returns True once the sim answered."""
    state = read_state()
    if not state or state.get("scene") != "ready":
        log("the sim is not running with a ready scene")
        return False
    if "camera" not in (state.get("actions") or []):
        log("this sim doesn't support the camera action: restart it (docker restart a300-isaac-sim)")
        return False
    cid = write_control({"action": "camera", "eye": eye, "target": target, **({"follow": follow} if follow else {}),
                         **({"view": view} if view else {})})
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = (read_state() or {}).get("camera") or {}
        if r.get("id") == cid:
            if r.get("state") == "error":
                log(f"camera failed: {r.get('message')}")
            return r.get("state") == "done"
        time.sleep(0.3)
    log(f"no answer from the sim after {timeout:.0f} s")
    return False


def record(name=None, fps=10, res=None, log=print):
    """Start saving the sim's viewport as JPEG frames into sim/generated/recordings/<name>/ (frame_%06d.jpg and
    frames.csv with each frame's wall and sim time), or stop it (name None). Encode afterwards, e.g.
    ffmpeg -framerate <fps> -i frame_%06d.jpg -c:v libx264 -pix_fmt yuv420p out.mp4."""
    state = read_state()
    if not state or state.get("scene") != "ready":
        log("the sim is not running with a ready scene")
        return False
    if "record" not in (state.get("actions") or []):
        log("this sim doesn't support the record action: restart it (docker restart a300-isaac-sim)")
        return False
    if name is None:
        write_control({"action": "record", "stop": True})
    else:
        (FLEET_DIR.parent / "recordings").mkdir(parents=True, exist_ok=True)
        os.chmod(FLEET_DIR.parent / "recordings", 0o777)  # the sim runs as uid 1234
        write_control({"action": "record", "dir": f"/sim/generated/recordings/{name}", "fps": fps, "res": res})
    time.sleep(1.5)
    log(json.dumps((read_state() or {}).get("record")))
    return True


def sun(hour=None, start=None, end=None, duration=None, timeout=30, log=print):
    """Set the running sim's sun to a local time of day (`hour`), or move it from `start` to `end` (hours) over
    `duration` wall-clock seconds: the solar position at Houghton, MI on June 21 (EDT; the sim's SUN_SITE). Lost
    when the sim restarts. Returns True once the sim answered."""
    state = read_state()
    if not state or state.get("scene") != "ready":
        log("the sim is not running with a ready scene")
        return False
    if "sun" not in (state.get("actions") or []):
        log("this sim doesn't support the sun action: restart it (docker restart a300-isaac-sim)")
        return False
    req = {"action": "sun", "hour": hour} if hour is not None else {
        "action": "sun", "start": start, "end": end, "duration": duration}
    cid = write_control(req)
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = (read_state() or {}).get("sun") or {}
        if r.get("id") == cid:
            if r.get("state") == "error":
                log(f"sun failed: {r.get('message')}")
            else:
                log(json.dumps(r))
            return r.get("state") != "error"
        time.sleep(0.3)
    log(f"no answer from the sim after {timeout:.0f} s")
    return False


def clouds(params, timeout=60, log=print):
    """Put a drifting cloud layer over the running sim's scene (`params`: cover 0-1, speed m/s, heading = compass
    direction it drifts toward, altitude m, seed; missing = the sim's CLOUDS defaults), or remove it (cover 0). Lost
    when the sim restarts. Returns True once the sim answered."""
    state = read_state()
    if not state or state.get("scene") != "ready":
        log("the sim is not running with a ready scene")
        return False
    if "clouds" not in (state.get("actions") or []):
        log("this sim doesn't support the clouds action: restart it (docker restart a300-isaac-sim)")
        return False
    cid = write_control({"action": "clouds", **{k: v for k, v in params.items() if v is not None}})
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = (read_state() or {}).get("clouds") or {}
        if r.get("id") == cid:
            log(json.dumps(r) if r.get("state") != "error" else f"clouds failed: {r.get('message')}")
            return r.get("state") != "error"
        time.sleep(0.3)
    log(f"no answer from the sim after {timeout:.0f} s")
    return False


def spawn(robots, timeout=900, log=print):
    """Spawn `robots`. Into an empty scene the running sim adds them in place; robots already in the scene are
    replaced by restarting the sim, which spawns the new request at start: removing robots from a running sim (their
    graphs, cameras and render products) made the RTX renderer abort a few seconds later (headed mode, a 3-robot
    respawn). If the sim rejects the request, it falls back to the previous robots."""
    if not wait(scene_ready, 900, "the scene", log):
        return False
    state = read_state()
    replace = bool(state["robots"])
    req_id = write_request(robots, at_start=replace)
    log(f"spawn request {req_id}: " + (", ".join(r["model"] for r in robots) or "no robots"))
    if not replace:
        return wait_spawn(req_id, timeout, log)
    log(f"replacing {', '.join(r['ns'] for r in state['robots'])}: restarting the sim (~1 min)")
    subprocess.run(["docker", "restart", SIM], check=True, capture_output=True)
    if not wait_scene(timeout, log):
        return False
    state = read_state()
    sp = state.get("spawn") or {}
    if sp.get("id") == req_id and sp.get("state") == "done":
        return True
    err = state.get("last_error") or {}
    msg = err.get("message") if err.get("id") == req_id else "not spawned"
    log(f"spawn failed: {msg}" + ("; the previous robots are back" if state["robots"] else ""))
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("wait-scene").add_argument("--timeout", type=float, default=900)
    sp = sub.add_parser("spawn")
    sp.add_argument("--poses", help="JSON list, one {x, y, yaw} (or null) per slot")
    sp.add_argument("--timeout", type=float, default=600)
    sub.add_parser("clear")
    pl = sub.add_parser("plants", help="hide / show plants in the running sim by object id")
    pl.add_argument("--hide", default="", help="comma-separated object ids to hide")
    pl.add_argument("--show", default="", help="comma-separated object ids to show, or all")
    sub.add_parser("reset").add_argument("--keep-robots", action="store_true", help="don't restart the robot containers")
    sub.add_parser("state")
    rc = sub.add_parser("record", help="save the sim's viewport as JPEG frames (sim/generated/recordings/NAME)")
    rc.add_argument("name", nargs="?")
    rc.add_argument("--fps", type=int, default=10)
    rc.add_argument("--res", help="WxH to render the viewport at while recording, e.g. 1920x1080")
    rc.add_argument("--stop", action="store_true")
    cm = sub.add_parser("camera", help="aim the sim's viewport (kept until the sim restarts)")
    cm.add_argument("eye", help="X,Y,Z of the camera, or default (the usual view)")
    cm.add_argument("target", nargs="?", help="X,Y,Z it looks at")
    cm.add_argument("--follow", choices=["cutter"], help="eye and target in the VBD plant cutter's CAD frame (mm)")
    cm.add_argument("--view", choices=["close"], help="with --follow cutter: aim the recorder's close-up view instead "
                    "(recorded as close_%%06d.jpg beside the viewport's frames)")
    su = sub.add_parser("sun", help="sun position by local time of day (Houghton, MI, June 21), fixed or moving")
    su.add_argument("hour", nargs="?", type=float, help="a fixed local time, e.g. 13.5")
    su.add_argument("--from", dest="start", type=float, help="move from this local time ...")
    su.add_argument("--to", dest="end", type=float, help="... to this one ...")
    su.add_argument("--duration", type=float, help="... over this many wall-clock seconds")
    cl = sub.add_parser("clouds", help="a drifting cloud layer over the scene (kept until the sim restarts)")
    cl.add_argument("off", nargs="?", choices=["off"], help="remove the clouds")
    cl.add_argument("--cover", type=float, help="fraction of the sky covered, 0-1 (default 0.4)")
    cl.add_argument("--speed", type=float, help="drift speed, m/s (default 2)")
    cl.add_argument("--heading", type=float, help="compass direction the clouds drift toward, deg (default 70)")
    cl.add_argument("--altitude", type=float, help="height of the layer, m (default 300)")
    cl.add_argument("--seed", type=int, help="cloud pattern (default 1)")
    sn = sub.add_parser("snapshot", help="viewport PNGs of the robots now, into sim/generated/snapshots/")
    sn.add_argument("--poses", help='JSON {"<ns>": [x, y, yaw_deg]}: where each robot is now (world frame)')
    sn.add_argument("--views", help="comma-separated subset of front,rear,close,wide")
    sn.add_argument("--tag", default="now", help="file name suffix (default now)")
    args = ap.parse_args()
    if args.cmd == "reset":
        return 0 if reset(restart_robots=not args.keep_robots) else 1
    if args.cmd == "snapshot":
        return 0 if snapshot(json.loads(args.poses) if args.poses else None,
                             args.views.split(",") if args.views else None, args.tag) else 1
    if args.cmd == "wait-scene":
        return 0 if wait_scene(args.timeout) else 1
    if args.cmd == "spawn":
        poses = json.loads(args.poses) if args.poses else None
        try:
            robots = slot_robots(poses)
        except ValueError as e:
            print(e, file=sys.stderr)
            return 2
        return 0 if spawn(robots, args.timeout) else 1
    if args.cmd == "clear":
        clear_request()
        return 0
    if args.cmd == "record":
        if not args.stop and not args.name:
            print("record wants a NAME (or --stop)", file=sys.stderr)
            return 2
        res = [int(v) for v in args.res.lower().split("x")] if args.res else None
        return 0 if record(None if args.stop else args.name, args.fps, res) else 1
    if args.cmd == "camera":
        if args.eye == "default":
            return 0 if camera(None, None) else 1
        xyz = lambda s: [float(v) for v in s.split(",")]
        try:
            eye, target = xyz(args.eye), xyz(args.target or "")
            assert len(eye) == len(target) == 3
        except (ValueError, AssertionError):
            print("camera wants eye and target as X,Y,Z each", file=sys.stderr)
            return 2
        if args.view and args.follow != "cutter":
            print("--view close needs --follow cutter", file=sys.stderr)
            return 2
        return 0 if camera(eye, target, follow=args.follow, view=args.view) else 1
    if args.cmd == "sun":
        if args.hour is None and None in (args.start, args.end, args.duration):
            print("sun wants HOUR, or --from H --to H --duration S", file=sys.stderr)
            return 2
        return 0 if sun(args.hour, args.start, args.end, args.duration) else 1
    if args.cmd == "clouds":
        params = {"cover": 0.0} if args.off else {k: getattr(args, k) for k in
                                                    ("cover", "speed", "heading", "altitude", "seed")}
        return 0 if clouds(params) else 1
    if args.cmd == "plants":
        ids = lambda s: [i for i in s.split(",") if i.strip()]
        r = plants(ids(args.hide), "all" if args.show == "all" else ids(args.show))
        print(json.dumps(r))
        return 0 if r and r.get("state") == "done" else 1
    print(json.dumps(read_state(), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())

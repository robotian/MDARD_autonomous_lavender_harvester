#!/usr/bin/env python3
"""Calibrate every robot model's drive (wheel_friction + vel_ctl) for one or both physics engines, one model at a
time on the empty scene, writing the results into sim/config/model_params.yaml. sim/CLAUDE.md "Velocity control"
has the procedure this automates:

  1. friction: the established 0.3/0.3 `min` of every hand-tuned robot (set for the model if it has none);
     Newton: the wheel torque cap (wheel_max_force) from the robot's mass, x1.5 when the sweep saturates;
  2. open loop (VELCTL=0): step test -> the speed ratios the robot achieves on its own;
  3. gains: Newton (smooth contact, turns near the command on its own) -> feed-forward 1/ratio, small integrals, no P;
     PhysX (rigid Coulomb skid-steer, ~2 % open-loop yaw) -> the yaw loop of the nearest hand-tuned sibling,
     feed-forward 1/ratio for linear;
  4. closed loop (VELCTL=1): step test + calibrate_velocity.py sweep; up to 2 corrections (feed-forward scaled by
     the sweep's ratios when it fails, the yaw integral halved when turns wobble).

  scripts/vel_tune/calibrate_all.py --jobs a200_0284:newton a200:physx ... [--state FILE]
  scripts/vel_tune/calibrate_all.py --all

Progress is kept in --state (JSON), so a rerun skips finished jobs. Leaves the fleet on the last job's settings;
restore them afterwards (SIM_PHYSICS, SIM_SCENE, VELCTL, slots)."""
import argparse
import json
import os
import re
import statistics
import subprocess
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
# FLEET_ROOT=<checkout>: the checkout whose fleet runs the sim (its scripts, settings and vel_check results), e.g. the
# main checkout while this one is a worktree whose model_params.yaml the sim mounts (COMPOSE_FILE override)
FLEET = os.path.abspath(os.environ.get("FLEET_ROOT", ROOT))
sys.path.insert(0, os.path.dirname(__file__))
from params_edit import set_params  # noqa: E402

PARAMS = os.path.join(ROOT, "sim/config/model_params.yaml")
VEL = os.path.join(FLEET, "sim/generated/vel_check")
FRICTION = {"static": 0.3, "dynamic": 0.3, "combine": "min"}
HZ = {"physx": 60, "newton": 240}
SIM_RATE = 20
NO_CONTAINER = {"a200_0333"}  # no robot_data/<id>/robot.yaml: its container exits; run the tests in the base station
PHYSX_TUNED = {"j100_0921", "a200_0284", "a300_00036"}  # hand-tuned on PhysX already (keep)
# PhysX yaw loops of the hand-tuned siblings (kp_w, ki_w, lim_w, ff_w) and their linear (kp_v, ki_v, lim_v)
PHYSX_FAMILY = {"j100": ((0.3, 1.0, 0.2), (1.0, 5.0, 2.0, 2.0)), "a200": ((0.3, 1.0, 0.1), (1.0, 3.0, 1.5, 1.5)),
                "a300": ((0.3, 1.0, 0.2), (1.0, 3.0, 1.5, 1.0)), "r100": ((0.3, 3.0, 1.0), (0.5, 10.0, 3.0, 1.0))}
ALL = (["a200_0284:newton", "a300_00036:newton"]
       + [f"{m}:{e}" for e in ("physx", "newton")
          for m in ("j100", "j100_0922", "j100_0936", "a200", "a200_0333", "a300", "a300_00037", "r100")])


def log(msg):
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


def sh(cmd, timeout=1800):
    p = subprocess.run(cmd, shell=True, cwd=FLEET, capture_output=True, text=True, timeout=timeout)
    return p.returncode, p.stdout + p.stderr


def container(model):
    return model if "_" in model else f"{model}_0000"


def ns(model):
    return container(model)


def runner(model):
    return "basestation" if model in NO_CONTAINER else container(model)


def ros(c, cmd, timeout=60, env=""):
    return sh(f"docker exec {env} {c} bash -c {json.dumps('source /opt/ros/jazzy/setup.bash; ' + cmd)}", timeout)


def odom_ok(model):
    code, out = ros(runner(model), f"timeout -k 2 8 ros2 topic hz /{ns(model)}/platform/odom -w 5 2>&1 | head -1", 40)
    return "average" in out


def wait_odom(model, since, limit=600):
    t0 = time.time()
    while time.time() - t0 < limit:
        _, out = sh(f"docker logs --since {since} a300-isaac-sim 2>&1 | grep -a -c 'simulation running'", 30)
        if out.strip().isdigit() and int(out.strip()) > 0:
            if model in NO_CONTAINER and time.time() - t0 > 180 and not odom_ok(model):
                sh("docker restart basestation", 180)  # its router link to the sim's goes stale on a fleet recreate
                time.sleep(30)
            if odom_ok(model):
                time.sleep(20)  # settle
                return True
        time.sleep(10)
    return False


def restart_sim(model):
    """model_params.yaml is read at sim start: restart the sim and wait for the robot's odom (3 tries: ~1 in 10
    sim starts fails, CLAUDE.md)."""
    for attempt in range(3):
        since = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        sh("docker restart a300-isaac-sim", 180)
        if wait_odom(model, since):
            return True
        log(f"  {model}: no odom after a sim restart (attempt {attempt + 1})")
    return False


def start(model, engine, velctl):
    """Fleet with only `model` (slot 0, pose 0,0), on `engine`, empty scene, VELCTL, the current model_params."""
    sh(f"python3 scripts/fleetcfg.py set SIM_PHYSICS={engine} 'SIM_SCENE=' VELCTL={velctl} NUM_ROBOTS=1 FLEET_DEBUG=0")
    sh(f"python3 scripts/fleetcfg.py slot 0 {model} --pose=0,0,0")
    code, out = sh("scripts/fleet.sh 1", 900)
    if code:
        log(f"  fleet.sh failed: {out.strip()[-300:]}")
    return restart_sim(model)


def step_test(model, label):
    extra = f" --in {runner(model)}" if model in NO_CONTAINER else ""
    code, out = sh(f"scripts/vel_tune/step_test.sh {ns(model)} {label} --segments full{extra}", 900)
    path = os.path.join(VEL, f"{ns(model)}_{label}.metrics.json")
    if not os.path.exists(path):
        log(f"  step test {label} failed: {out.strip()[-300:]}")
        return None
    return json.load(open(path))


def sweep(model, engine):
    modes = "lin lat ang" if model.startswith("r100") else "lin ang"
    code, out = ros(runner(model), f"timeout -k 5 1500 python3 /scripts/calibrate_velocity.py --ns {ns(model)} "
                    f"--modes {modes} --physics-hz {HZ[engine]} --sim-rate-hz {SIM_RATE} 2>&1", 1600,
                    env=f"-e ROBOT_NAMESPACE={ns(model)}")
    rows = re.findall(r"^(lin|lat|ang)\s+([+-][\d.]+)\s+([+-][\d.]+)\s+([\d.]+)", out, re.M)
    ratios = {}
    for mode, cmd, ach, ratio in rows:
        ratios.setdefault(mode, []).append(float(ratio))
    return {"pass": "PASS" in out, "ratios": ratios, "tail": out.strip().splitlines()[-16:]}


def seg(metrics, name, axis):
    s = next((x for x in metrics["segments"] if x["segment"] == name), None)
    v = s.get(axis) if s else None
    return v if isinstance(v, dict) else None


def summarize(m):
    """Open/closed-loop figures from a step test: speed ratios and the yaw wobble/overshoot."""
    lin = [seg(m, n, "v") for n in ("lin 0.5", "lin 0.3", "lin -0.2 (reverse)")]
    lin = [1 + x["err"] / 100 for x in lin if x]
    yaw = [seg(m, n, "w") for n in ("ang 0.3", "arc 0.3/0.3", "arc 0.5/0.5")]
    yaw = [x for x in yaw if x]
    return {"ratio_v": statistics.mean(lin) if lin else 1.0,
            "ratio_w": statistics.mean(1 + x["err"] / 100 for x in yaw) if yaw else 1.0,
            "wobble_w": max((x["sd"] for x in yaw), default=0.0),
            "overshoot_w": max((x["overshoot"] for x in yaw), default=0.0),
            "worst_err": max((abs(x[a]["err"]) for x in m["segments"] for a in ("v", "w")
                              if isinstance(x.get(a), dict)), default=0.0)}


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def initial_gains(model, engine, ol):
    ff_v = round(clamp(1 / ol["ratio_v"], 0.8, 1.25), 3)
    if engine == "newton":
        ff_w = round(clamp(1 / max(ol["ratio_w"], 0.05), 0.7, 2.5), 3)
        ki_w, lim_w = (0.5, 0.15) if ol["ratio_w"] >= 0.8 else (1.0, 0.4)
        return [0.0, 0.3, 0.05, 0.0, ki_w, lim_w, ff_v, ff_w]
    (kp_v, ki_v, lim_v), (kp_w, ki_w, lim_w, ff_w) = PHYSX_FAMILY[model.split("_")[0]]
    return [kp_v, ki_v, lim_v, kp_w, ki_w, lim_w, ff_v, ff_w]


def wheel_cap(model):
    """Newton's wheel torque cap for `model`: 6 x mass x wheel radius (N*m), at least 20 (Jackal). j100_0921
    (33.7 kg, r 0.098) is fine at 20; a200_0284 (65.8 kg, r 0.165) couldn't turn at 20 and is within 4 % at 60."""
    import glob
    urdf = glob.glob(os.path.join(ROOT, f"sim/assets/{model}/*.urdf"))[0]
    mass = sum(float(x) for x in re.findall(r'<mass value="([\d.eE+-]+)"', open(urdf).read()))
    radius = {"j100": 0.098, "a200": 0.1651, "a300": 0.1625, "r100": 0.0759}[model.split("_")[0]]
    return max(20, int(round(6 * mass * radius / 10.0)) * 10)


def saturated(sw):
    """The sweep falls off at the top while lower levels reach the command: the wheels hit their torque cap."""
    for mode, r in sw["ratios"].items():
        if len(r) >= 3 and r[-1] < 0.92 and max(r[:-2]) > 0.97:
            return True
    return False


def score(t):
    """Smaller is better: a passing sweep first, then the sweep's worst error and the step test's yaw wobble."""
    worst = max((abs(x - 1) for r in t["sweep"].values() for x in r), default=9.9)
    return (not t["sweep_pass"], worst + (t["step"]["wobble_w"] / 100 if t["step"] else 1.0))


def calibrate(model, engine, state):
    key = f"{model}:{engine}"
    res = state.setdefault(key, {})
    if res.get("done"):
        log(f"{key}: done already, skipped")
        return
    log(f"=== {key}")
    if engine == "physx" and model in PHYSX_TUNED:
        res.update(done=True, note="hand-tuned on PhysX already")
        return
    set_params(PARAMS, model, "physx", wheel_friction=FRICTION)  # friction first (inherited by Newton)
    cap = wheel_cap(model) if engine == "newton" else None
    tries = []
    for round_ in range(2):  # Newton: a second round with a higher torque cap when the first saturates
        if cap:
            set_params(PARAMS, model, "newton", wheel_max_force=cap)
            res["wheel_max_force"] = cap
        if not start(model, engine, 0):
            res.update(done=True, error="sim/robot did not come up (open loop)")
            return
        ol = step_test(model, f"{engine}_ol" + (f"_cap{cap}" if cap else ""))
        if not ol:
            res.update(done=True, error="open-loop step test failed")
            return
        tilt = max(s.get("tilt_max", 0) for s in ol["segments"])
        if tilt > 20:  # tipped over: nothing to tune until its masses are fixed (massless_density, frame_mass)
            res.update(done=True, error=f"tipped over open loop (tilt {tilt:.0f} deg)")
            log(f"  {res['error']}")
            return
        res["open_loop"] = summarize(ol)
        log(f"  open loop{f' (cap {cap} N*m)' if cap else ''}: {res['open_loop']}")
        gains = initial_gains(model, engine, res["open_loop"])
        sat = False
        for attempt in range(3):
            set_params(PARAMS, model, engine, vel_ctl=gains)
            if not (start(model, engine, 1) if attempt == 0 else restart_sim(model)):
                res.update(done=True, error="sim/robot did not come up (closed loop)")
                return
            label = f"{engine}_cl{len(tries) + 1}"
            cl = step_test(model, label)
            sw = sweep(model, engine)
            t = {"label": label, "vel_ctl": list(gains), "wheel_max_force": cap, "step": summarize(cl) if cl else None,
                 "sweep_pass": sw["pass"], "sweep": sw["ratios"]}
            tries.append(t)
            res["tries"] = tries
            log(f"  {label}: vel_ctl={t['vel_ctl']} step={t['step']} sweep pass={sw['pass']} ratios={sw['ratios']}")
            if cap and saturated(sw):
                sat = True
                break
            wobbly = t["step"] and t["step"]["wobble_w"] > (15.0 if engine == "physx" else 8.0)
            if sw["pass"] and not wobbly:
                break
            gains = list(gains)
            if not sw["pass"] and sw["ratios"]:
                rv = max(statistics.mean(sw["ratios"].get("lin", [1.0])), 0.05)
                rw = max(statistics.mean(sw["ratios"].get("ang", [1.0])), 0.05)
                gains[6] = round(clamp(gains[6] / rv, 0.7, 1.5), 3)
                gains[7] = round(clamp(gains[7] / rw, 0.5, 3.0), 3)
            if wobbly:
                gains[4] = round(gains[4] / 2, 3)
        if not sat or round_:
            break
        cap = int(cap * 1.5 / 10) * 10
        log(f"  saturated at the top of the sweep: torque cap -> {cap} N*m")
    best = min(tries, key=score)
    res.update(done=True, best=best["label"], vel_ctl=best["vel_ctl"])
    set_params(PARAMS, model, engine, vel_ctl=best["vel_ctl"], wheel_max_force=best["wheel_max_force"],
               note=f"calibrate_all {engine} {time.strftime('%Y-%m-%d')}: sim/generated/vel_check/{ns(model)}_{engine}_* "
                    f"(best {best['label']}, sweep {'pass' if best['sweep_pass'] else 'FAIL'})")
    log(f"  best {best['label']}: {best['vel_ctl']} cap={best['wheel_max_force']}")


RELAYED = ("a200_0284", "a300_00036")  # real robots the base station relays: same names as their sim slot


def relays(drop):
    """Drop / restore the base station's relays of the real robots being calibrated (a sim slot with a relayed
    robot's name would share its ROS names)."""
    code, out = sh("python3 scripts/fleetcfg.py get BASESTATION_RELAYS")
    cur = out.strip()
    keep = " ".join(r for r in cur.split() if r.split("=")[0] not in drop) if drop else ORIG_RELAYS
    if keep != cur:
        sh(f"python3 scripts/fleetcfg.py set 'BASESTATION_RELAYS={keep}'")
        sh("docker compose -f basestation.compose.yml up -d", 300)
        log(f"  base station relays: {keep or '(none)'}")


ORIG_RELAYS = ""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--jobs", nargs="*", default=[])
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--state", default=os.path.join(ROOT, "sim/generated/vel_check/calibrate_all.json"))
    args = ap.parse_args()
    jobs = ALL if args.all else args.jobs
    state = json.load(open(args.state)) if os.path.exists(args.state) else {}
    global ORIG_RELAYS
    ORIG_RELAYS = state.setdefault("_orig", {}).setdefault(
        "relays", sh("python3 scripts/fleetcfg.py get BASESTATION_RELAYS")[1].strip())
    restore = {"SIM_PHYSICS": "physx", "SIM_SCENE": "lavender_farm_chargers.usda", "VELCTL": "1"}
    try:
        for i, job in enumerate(jobs):
            model, engine = job.split(":")
            if model in RELAYED and not state.get(job, {}).get("done"):
                relays(RELAYED)
            try:
                calibrate(model, engine, state)
            except Exception as e:  # keep going with the next model
                state.setdefault(job, {}).update(done=True, error=repr(e))
                log(f"{job}: {e!r}")
            json.dump(state, open(args.state, "w"), indent=1)
            if model in RELAYED and not any(j.split(":")[0] in RELAYED for j in jobs[i + 1:]):
                sh("scripts/fleet.sh down", 300)  # the relayed robot's sim slot first, or both share its names
                relays([])
    finally:
        relays([])
        sh("scripts/fleet.sh down", 300)
        sh("python3 scripts/fleetcfg.py set " + " ".join(f"{k}={v}" for k, v in restore.items()))
        sh("python3 scripts/fleetcfg.py slot 0 j100_0921")
    log("all jobs done; fleet down, settings restored (physx, farm scene, VELCTL=1, slot 0 j100_0921)")


if __name__ == "__main__":
    main()

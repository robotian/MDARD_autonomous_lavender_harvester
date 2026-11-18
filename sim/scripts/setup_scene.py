"""Isaac Sim fleet scene: a world (SIM_SCENE), plus Clearpath robots spawned into it on request, bridged to ROS 2.

Runs inside the streaming Kit app (isaac-sim.streaming.sh --exec /sim/scripts/setup_scene.py), in two parts:
  1. scene: import (or reuse the cached import of) every robot model, build the world and start the timeline --
     no robots. Reported in FLEET_STATE as scene "ready".
  2. spawn: FLEET_REQUEST (written by scripts/fleet_ctl.py, i.e. `scripts/fleet.sh spawn` or the web UI) lists one
     {model, x, y, yaw} per slot. The sim stops the timeline, removes the robots it has, spawns the requested ones
     at their poses and plays again (spawn_fleet). Slot i is namespaced "<its model>_%04d" % i, e.g. a Jackal
     (j100) in slot 1 is j100_0001 -- matches docker-compose.yml's container naming -- except a real robot id,
     used directly as its own namespace with no slot suffix (j100_0921, not j100_0921_0000): it's one specific
     physical robot, not a generic model needing a slot index to stay unique. The request file stays until
     `scripts/fleet.sh down`/a fresh `fleet.sh scene`, so a restarted sim (web UI "Reset scene") spawns the same
     robots again as soon as its scene is up.
Configuration comes from environment variables (see docker-compose.yml):
  SCENE_LANES        robot lanes between the two lavender rows (default 3); the default spawn poses fill them
  SIM_SCENE          "" = ground plane + lights (default), "lavender" = the lavender farm, else a USD file
                     in sim/scene/ (e.g. one saved from Isaac Sim), see build_world
  LAVENDER_SOFT      1 (default): robots pass through lavender foliage, only a rigid core per plant stops them
                     (lidars still see the foliage); 0: the whole plant is solid, see soften_lavender
  CAMERA_WIDTH/HEIGHT, CAMERA_FRAME_SKIP, FORCE_REIMPORT, ...
"""
import asyncio
import json
import math
import os
import random
import re
import sys
import time
import traceback

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

sys.path.insert(0, "/sim/scripts")
import fleet_nodes  # noqa: E402  (ScriptNodes' rclpy nodes, destroyed on respawn)
import robot_looks  # noqa: E402  (ROBOT_LOOKS: photo-realistic robot materials)
import farm_workers  # noqa: E402  (FARM_WORKERS=1: two people sitting at a lavender row)
import fleet_pose  # noqa: E402  (world poses under either physics engine; the script nodes import it too)
import ee_cutter  # noqa: E402  (end_effector: ee_cutter, the real cutter + gripper on the 2F Lite)

# ROBOTS: one (namespace, model) pair per robot currently in the scene, in slot order (empty until a spawn
# request arrives); ROBOT_POSES: their spawn poses as (x, y, yaw in degrees), same order.
ROBOTS = []
ROBOT_POSES = []
# Host <-> sim spawn protocol (./sim is mounted at /sim; scripts/fleet_ctl.py is the host side). The directory is
# shared by the host user and the sim's uid 1234, hence world-writable.
FLEET_DIR = "/sim/generated/fleet"
FLEET_REQUEST = f"{FLEET_DIR}/spawn_request.json"
FLEET_STATE = f"{FLEET_DIR}/state.json"
# the last request that was spawned successfully: what a restarted sim spawns again (a rejected request is not)
FLEET_APPLIED = f"{FLEET_DIR}/applied_request.json"
# {id, action: "reset"} (scripts/fleet_ctl.py reset, the web UI's "Reset scene"): stop and play the timeline, like
# Isaac's Stop and Play buttons, which puts every robot back in its spawn state; answered in state.json "reset".
# {id, action: "snapshot", ...} (scripts/fleet_ctl.py snapshot): viewport PNGs of the robots now (snapshot_request)
# {id, action: "camera", eye, target} (scripts/fleet_ctl.py camera): aim the viewport there and keep it (camera_request)
# {id, action: "record", dir, fps, res | stop} (scripts/fleet_ctl.py record): viewport frames to JPEGs (record_request)
FLEET_CONTROL = f"{FLEET_DIR}/control.json"


def robot_namespace(slot, model):
    """Must match docker-compose.yml's container/ROBOT_NAMESPACE naming (and its own real-robot special case, see
    robot/entrypoint.sh's own copy of this same logic): a real robot's own id (contains "_", e.g. j100_0921)
    already *is* its correct namespace -- one specific physical robot with one fixed real identity, not a generic
    model that needs a slot index to stay unique -- so it's used directly; only generic catalog models (a300, ...)
    get the slot-indexed "<model>_%04d"."""
    return model if "_" in model else f"{model}_{slot:04d}"

CAM_W = int(os.environ.get("CAMERA_WIDTH", "640"))
CAM_H = int(os.environ.get("CAMERA_HEIGHT", "360"))
# Robot cameras' far clipping plane. Was 30 m: the farm scene's tree line, pushed back near its ground's edge
# (sim/scripts/build_farm_scene.py BORDER, 21-37 m past the field), was cut off and the horizon showed sky. Colour and
# depth render from the same camera, so depth is limited separately to each sensor's real range (DEPTH_RANGE_M).
CAM_FAR_CLIP = 120.0
# Published depth range per sensor (m): outside it a pixel is 0, as the RealSense/ZED drivers publish "no depth".
# D435i: min-Z ~0.28 m, Intel's stated maximum ~10 m (accurate within ~3 m). D405: rated 7-50 cm, kept to 1 m
# (the wrist camera looks at plants within reach). ZED 2i: Stereolabs' 0.3-20 m.
KINOVA_MOUNT_USD = "/sim/assets/kinova_base_mount/kinova_base_mount.usd"  # 48 mm tall, origin at its foot
DEPTH_RANGE_M = {"d435i": (0.28, 10.0), "d405": (0.07, 1.0), "zed2i": (0.3, 20.0)}
_depth_ranges = {}  # (namespace, topic) -> (min, max), filled when a robot's camera graph is built
CAM_FRAME_SKIP = int(os.environ.get("CAMERA_FRAME_SKIP", "0"))  # 0 = publish every simulation frame
# Each camera-equipped robot costs ~12 ms of frame time, so 3 robots cannot render at the app's default 60 Hz.
# The stage runs at SIM_RATE_HZ instead and PhysX substeps at PHYSICS_HZ inside each frame, which keeps
# simulated time in step with wall-clock time.
SIM_RATE_HZ = float(os.environ.get("SIM_RATE_HZ", "20"))
PHYSICS_HZ = int(os.environ.get("PHYSICS_HZ", "60"))
# Newton steps at NEWTON_PHYSICS_HZ instead (select_physics_engine swaps it in): MuJoCo's soft contacts (time constant
# 0.02 s) and the arms' stiff position drives are unstable at PhysX's 60 Hz (robots catapulted on the first wheel
# turn, 2026-10-20); 240 Hz was stable. PHYSICS_FRAME_DT follows (physics steps per rendered frame).
NEWTON_PHYSICS_HZ = int(os.environ.get("NEWTON_PHYSICS_HZ", "240"))
# Newton's solver: mujoco (MuJoCo-Warp, both Isaac versions) or vbd (Vertex Block Descent: Isaac Sim 6.2 / Newton 1.6
# only; the study's lavender cables and the G1 on one solver, docs/lavender_plant_model/study/vbd_cable.md)
NEWTON_SOLVER = os.environ.get("NEWTON_SOLVER", "mujoco").strip().lower()
# The VBD lavender plant and its cut_stem scenes (vbd_plant.py, vbd_scene.py) are not part of this release: these stay
# empty, so the code below that uses them never runs.
VBD_PLANT = ""
VBD_PLANT_ASSET = ""
_vbd_plant = None
VBD_SCENE = ""
VBD_SCENE_GAINS = {}  # the scene's finger_gains / blade_gains (they replace model_params' vbd_*_gains)
NEWTON_PARSE_CALLBACKS = []  # f(builder) after each Newton parse of the stage, before the model is finalized
# Prim paths (space-separated) Newton's importer skips. Newton makes a shape of every mesh, instance proxies included,
# and the lavender farm's foliage is 1900 instanced plant meshes of ~190k points each (362 M points, they are the
# plants' colliders too), plus 2300 tree and shrub meshes: parsing them pinned the sim for 10+ minutes and never
# finished (2026-10-20). Plain prim paths: the importer passes them to UsdPhysics.LoadUsdPhysicsFromRange as
# excludePaths (the subtree's colliders go too) and matches them as regexes against visual meshes, bodies and
# joints; a pattern with ".*" only does the latter, so the colliders would still be parsed. The plants' cores
# (/World/lavender_cores: another prim), rocks, arch, chargers and ground stay, so driving is unchanged; Newton's
# lidar raytracer doesn't see the foliage (PhysX's scene queries do).
NEWTON_IGNORE_PATHS = os.environ.get(
    "NEWTON_IGNORE_PATHS", "/World/lavender /World/shrubs /World/trees").split() + ["/World/Clouds"]  # clouds_request
# SIM_PHYSICS: physx (default) or newton (isaacsim.physics.newton, see "Physics engine" in sim/CLAUDE.md).
# PHYSICS_ENGINE is the one that runs, set by select_physics_engine() in main(): a failed switch to Newton falls back
# to PhysX, and state.json says so (physics vs physics_requested).
PHYSICS_REQUESTED = os.environ.get("SIM_PHYSICS", "physx").strip().lower()
PHYSICS_ENGINE = "physx"
# true: the sim publishes /clock (simulation time, monotonic across Stop/Play) and stamps every message with it, so ROS
# nodes with use_sim_time (the robot containers get the same USE_SIM_TIME) keep step with a sim slower than real time.
# false: messages are stamped with wall-clock time and there is no /clock.
USE_SIM_TIME = os.environ.get("USE_SIM_TIME", "true").lower() == "true"
FORCE_REIMPORT = os.environ.get("FORCE_REIMPORT", "0") == "1"
# 1: every robot publishes its exact world pose as ref_pose + TF ref_frame -> base_link_ref (REF_POSE_SCRIPT), the
# sim's stand-in for the lab's motion capture (mocap_fake_localizer's natnet_ref_pose on a real robot).
REF_POSE = os.environ.get("SIM_REF_POSE", "1") == "1"
# Which D435i streams to publish. Each one costs main-thread time in the sim, so trim if the frame rate suffers.
# "none" turns the cameras off (no render products at all), which is what gives the streamed viewport its full frame rate.
CAM_STREAMS = set(filter(None, os.environ.get("CAMERA_STREAMS", "color,depth").split(","))) - {"none", "off"}
# 1: a robot camera renders only while something subscribes to one of its topics (camera_on_demand_loop); an idle
# render product still costs frame time, and CAMERA_FRAME_SKIP only skips the publish, not the render.
# 0: every camera renders every frame.
CAM_ON_DEMAND = os.environ.get("CAMERA_ON_DEMAND", "1") == "1"
CAM_IDLE_OFF_S = 2.0  # a camera stays on this long after its last subscriber left
CAM_WARMUP_S = 5.0  # and renders this long after play/spawn/reset, so its ROS publishers exist (topic type known)

# Per-model URDF/USD paths, from scripts/gen_urdf.sh's output (sim/assets/<model>/) and the importer's cache
# (sim/generated/<model>/, see import_urdf_if_needed).
# Every model scripts/gen_urdf.sh has produced (sim/assets/<m>/<m>.urdf) is available -- gen_urdf.sh itself takes
# the real robots from robot_data/<id>/, so a new robot needs no list edited here (nor a MODEL_PARAMS entry).
# The fixed names are kept as a floor so a listing problem can never make a known model disappear.
_ASSET_ROOT = "/sim/assets"
_GENERATED = [d for d in (os.listdir(_ASSET_ROOT) if os.path.isdir(_ASSET_ROOT) else [])
              if os.path.isfile(f"{_ASSET_ROOT}/{d}/{d}.urdf")]
MODEL_ASSETS = {
    m: {
        "urdf": f"/sim/assets/{m}/{m}.urdf",
        "usd_dir": f"/sim/generated/{m}",
        "usd_path": f"/sim/generated/{m}/{m}/{m}.usda",
    }
    for m in sorted({"a300", "a200", "j100", "r100", "j100_0921", "j100_0936", "a200_0333", "a300_00036", "j100_0922"}
                    | set(_GENERATED))
}

# Decorative lavender plants (SM_Lavender_Nanite_01.usd, default prim /Root). The asset's own layer is
# centimetres (metersPerUnit 0.01) but this stage is metres, and USD does not rescale geometry across that
# boundary by itself, so references to it need an explicit 0.01 scale. LAVENDER_BASE_Z lifts each plant so its
# lowest point (bbox min z, measured once in the source asset) sits on the ground instead of poking through it.
SKY_HDR = "/sim/assets/sky/farm_field_puresky_2k.hdr"
SKY_INTENSITY = 400
TREE_USDS = [f"/sim/assets/trees/{n}.usd" for n in ("Douglas_Fir", "Black_Oak", "Douglas_Fir")]
TREE_COUNT = 36
TREE_DIST = 22.0  # chosen when the cameras clipped at 30 m (now CAM_FAR_CLIP)
TREE_HEIGHT = (7.0, 11.0)  # m
SHRUB_USDS = [f"/sim/assets/shrubs/{n}.usd" for n in ("Rhododendron", "Lilac", "Goldflame_Spirea", "Barberry")]
SHRUB_COUNT = 70
SHRUB_DIST = 21.0
SHRUB_HEIGHT = (1.0, 2.2)  # m
ROCK_USDS = [f"/sim/assets/rocks/rock_small_{i:02d}.usda" for i in range(1, 7)]
ROCK_COUNT = 30
ROCK_DIST = 19.0
ROCK_HEIGHT = (0.4, 1.0)  # m
GROUND_SOIL_COLOR = (0.16, 0.10, 0.06)  # dark brown soil under the grass
# Scene source (SIM_SCENE, see build_world) and the default scene's look
SIM_SCENE = os.environ.get("SIM_SCENE", "").strip()
SCENE_DIR = "/sim/scene"  # ./sim/scene on the host
SCENE_FILE_EXTS = (".usd", ".usda", ".usdc", ".usdz")
GROUND_COLOR = (0.25, 0.26, 0.25)  # default scene: mid grey
DEFAULT_SKY_COLOR = (0.75, 0.85, 1.0)
DEFAULT_SKY_INTENSITY = 300
DEFAULT_SUN_INTENSITY = 4000
GROUND_COVER_USD = "/sim/assets/Ground_cover/ground_cover.usd"
GROUND_COVER_SCALE_XY = 1.0
GROUND_COVER_SCALE_Z = 0.6
LAVENDER_USD = "/sim/assets/lavender/SM_Lavender_Nanite_01.usd"
LAVENDER_SCALE = 0.006
LAVENDER_BASE_Z = 0.05
LAVENDER_ROW_OFFSET = 1.6   # row centre distance beyond the outermost robot lane, m
LAVENDER_ROW_X0 = 2.0
LAVENDER_PLANT_PITCH = 1.0  # plants are ~1.2 m wide, so they overlap into a hedge
LAVENDER_ROW_PLANTS = 10
# Soft lavender (see soften_lavender): the foliage stops blocking robots but stays visible to the lidars, and a
# small rigid cylinder at each plant's centre stands in for the woody crown, which does stop a robot.
LAVENDER_SOFT = os.environ.get("LAVENDER_SOFT", "1") != "0"
LAVENDER_CORE_RADIUS = 0.1  # m
LAVENDER_CORE_HEIGHT = 0.3  # m, from the ground
SOFT_PLANTS_GROUP = "/World/collisionGroups/soft_plants"
ROBOTS_GROUP = "/World/collisionGroups/robots"

# Per-model sim parameters (drive constants, sensors, arm) are derived, not listed: model_params() builds them
# from sim/config/model_params.yaml's `platforms` (the generic drive constants), the model's own robot.yaml and
# flattened URDF in sim/assets/<m>/, and that file's `robots` overrides -- see the file's header. A new robot or a
# changed robot.yaml therefore needs only scripts/gen_urdf.sh and a sim restart, no edit here.
# chassis_link (the prim the drive/odometry OmniGraph targets) is not derived from the URDF at all: it must be the
# prim the URDF importer gave ArticulationRootAPI, which no URDF rule predicted reliably (a200's and j100's
# importer root is base_link, a300's/r100's chassis_link, and an unrelated change such as merging j100's fenders
# into base_link or a300_00036's IMU box once flipped it, failing only at runtime with "Articulation controller
# failed"), so chassis_prim() reads it from the spawned robot. A robot with more than one articulation root (cf.
# flatten_urdf.py's weld_empty_root_children) is refused at spawn with that reason.
MODEL_CONFIG = "/sim/config/model_params.yaml"
MODEL_PARAMS = {}  # model -> params, filled by load_model_params() at sim start


def _pvc_value(pvc, key):
    """platform_velocity_controller value by its dotted ROS parameter name, flat ("linear.x.max_velocity") or
    nested ({linear: {x: {max_velocity}}})."""
    if key in pvc:
        return pvc[key]
    node = pvc
    for part in key.split("."):
        if not isinstance(node, dict) or part not in node:
            return None
        node = node[part]
    return node


def derive_model_params(model, platforms):
    """Parameters of `model` from sim/assets/<model>/robot.yaml + <model>.urdf (+ merged_links.json)."""
    import xml.etree.ElementTree as ET

    import yaml

    d = f"{_ASSET_ROOT}/{model}"
    with open(f"{d}/robot.yaml") as f:
        cfg = yaml.safe_load(f) or {}
    platform = str(cfg.get("serial_number") or model).split("-")[0].split("_")[0]
    if platform not in platforms:
        raise ValueError(f"platform {platform!r} (robot.yaml serial_number) has no entry in {MODEL_CONFIG} platforms")
    params = dict(platforms[platform], platform=platform)

    # The robot's own drive calibration (as the real robot runs it), on top of the platform's generic constants.
    plat = cfg.get("platform") if isinstance(cfg.get("platform"), dict) else {}
    pvc = (((plat.get("extras") or {}).get("ros_parameters") or {}).get("platform_velocity_controller")) or {}
    if pvc:
        radius = _pvc_value(pvc, "wheel_radius") or params["wheel_radius"]
        left = _pvc_value(pvc, "left_wheel_radius_multiplier") or 1.0
        right = _pvc_value(pvc, "right_wheel_radius_multiplier") or 1.0
        params["wheel_radius"] = radius * (left + right) / 2  # one radius: a left/right asymmetry can't be modelled
        for key, name in (("wheel_separation", "wheel_separation"),
                          ("separation_multiplier", "wheel_separation_multiplier"),
                          ("max_linear", "linear.x.max_velocity"), ("max_angular", "angular.z.max_velocity")):
            value = _pvc_value(pvc, name)
            if value is not None:
                params[key] = value

    urdf = ET.parse(f"{d}/{model}.urdf").getroot()
    links = {link.get("name"): link for link in urdf.findall("link")}
    parent_of = {j.find("child").get("link"): j.find("parent").get("link") for j in urdf.findall("joint")}
    merged_path = f"{d}/merged_links.json"
    merged = json.load(open(merged_path)) if os.path.isfile(merged_path) else None
    sensors = cfg.get("sensors") or {}

    def frames(kind, suffix):
        """URDF frames of robot.yaml's sensors.<kind> entries. Clearpath numbers a platform's built-in sensors
        first (j100's own imu_0_link and gps slot), so the yaml's k sensors are the k highest-numbered frames."""
        k = len(sensors.get(kind) or [])
        names = set(links) | set(merged or {})
        idx = sorted(int(m.group(1)) for n in names if (m := re.fullmatch(rf"{kind}_(\d+)_{suffix}", n)))
        return [f"{kind}_{i}_{suffix}" for i in idx[-k:]] if k else []

    def body(name):
        """The URDF link that carries frame `name` as a body: the link it was merged into (merged_links.json),
        then up the tree to the first link with a collision or inertial."""
        name = (merged or {}).get(name, name)
        while name in links and links[name].find("collision") is None and links[name].find("inertial") is None:
            if name not in parent_of:
                break
            name = parent_of[name]
        return name if name in links else None

    params["has_camera"] = bool(sensors.get("camera")) and any(n.startswith("camera_0_") for n in links)
    if "camera_0_left_camera_frame_optical" in links:  # Stereolabs ZED: the xacro has a ROS optical frame
        params["camera_optical_link"] = "camera_0_left_camera_frame_optical"
    if "camera_1_link" in links and len(sensors.get("camera") or []) < 2:
        params["wrist_camera"] = True  # mtu32_description's D405 on the arm, not a robot.yaml sensor
    imus = frames("imu", "link")
    if imus:
        imu_link = body(imus[0])
        if imu_link:
            params["imu_link"] = imu_link
            params["imu_index"] = int(imus[0].split("_")[1])
        else:
            log(f"{model}: {imus[0]} was merged away and {merged_path} is missing (run scripts/gen_urdf.sh): no IMU")
    params["gps_links"] = [n for n in frames("gps", "link") if n in links]
    lidar2d = [n for n in frames("lidar2d", "laser") if n in links]
    lidar3d = [n for n in frames("lidar3d", "laser") if n in links]
    if lidar2d:
        params["lidar2d_link"] = lidar2d[0]
    if lidar3d:
        params["lidar3d_link"] = lidar3d[0]
    params["has_arm"] = any(j.get("name").startswith("arm_0_") and j.get("type") != "fixed" for j in urdf.findall("joint"))
    return params


def load_model_params(models):
    """MODEL_PARAMS for every model in `models` that has a robot.yaml; a model that fails is logged and left out
    (it can't be spawned)."""
    import yaml

    with open(MODEL_CONFIG) as f:
        config = yaml.safe_load(f) or {}
    platforms = config.get("platforms") or {}
    overrides = config.get("robots") or {}
    for model in models:
        try:
            params = derive_model_params(model, platforms)
        except Exception as e:
            log(f"params of {model} could not be derived, it can't be spawned: {e}")
            continue
        params.update(overrides.get(model) or {})
        # robots[<m>].newton: overrides of their own for SIM_PHYSICS=newton (vel_ctl, wheel_friction, ...): the
        # drive tuning above was measured on PhysX
        newton_overrides = params.pop("newton", None) or {}
        if PHYSICS_ENGINE == "newton":
            params.update(newton_overrides)
        MODEL_PARAMS[model] = params
        log(f"params {model}: " + ", ".join(f"{k}={v}" for k, v in params.items()))


def chassis_prim(stage, root, params):
    """Path of the robot's articulation root (or of an explicit chassis_link override) under `root`."""
    if params.get("chassis_link"):
        return find_prim(stage, root, params["chassis_link"])
    roots = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies())
             if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise RuntimeError(f"{root} has {len(roots)} articulation roots, needs exactly 1: {roots}")
    return roots[0]

# Velocity calibration. The open-loop wheel kinematics above (MODEL_PARAMS) cannot hold the commanded speed on a
# skid-steer robot: a 0.2 rad/s in-place turn did not move at all (static-friction breakaway), 0.5 turned 0.2-0.6x,
# a200's forward speed was +5%, j100_0921's reverse 0.6-0.9x. VEL_CTL_SCRIPT therefore closes a small PI loop per
# robot around the DifferentialController (feed-forward = the commanded value, so the wheel params above still
# matter), on the speed the chassis actually achieves; scripts/calibrate_velocity.py measures the result
# (every model within ~5-8% over 0.1-1.0 m/s and 0.1-1.0 rad/s at 0.25 m/s^2 / 0.5 rad/s^2).
# (kp_v, ki_v, int_limit_v [m/s], kp_w, ki_w, int_limit_w [rad/s][, ff_v, ff_w]); per-model override via
# MODEL_PARAMS["vel_ctl"]. ff_* (default 1) scale the feed-forward: each channel outputs ff * cmd + kp * e + integral.
# ki_w 10 is what removed the low-rate dead zone (5 left 0.1 rad/s at 0.75-0.85); the linear gains are kept low
# because the combination kp 0.3 + ki 3 is about the most a light Jackal tolerates before its pitch oscillates.
VELCTL_DEFAULT = (0.3, 3.0, 1.0, 0.5, 10.0, 3.0)
VELCTL_LATERAL = (0.3, 6.0, 1.5)  # kp, ki, integrator limit of the sideways channel (Ridgeback's BodyDrive)
VELCTL_ENABLED = os.environ.get("VELCTL", "1") == "1"  # VELCTL=0: open loop, for A/B comparisons
# The feedback may command more than the robot's own limit (max_linear/max_angular clamp the *command* only).
VELCTL_HEADROOM = 4.0
# Wheel parking brake (WHEEL_BRAKE_SCRIPT). The wheel drives are pure velocity dampers (stiffness 0, damping 1000),
# which resist speed but never hold a position, so at rest the robots rocked forward/back by ~2 cm (a300_00036: 69
# direction reversals and 0.4 deg of yaw in 60 s with no cmd_vel at all; Diff's wheel targets were exactly 0): the
# solver's small alternating contact torques integrate into wheel rotation. Real motor controllers hold position at
# zero speed (their integral term acts as a spring). With no command, once every wheel is slower than
# WHEEL_BRAKE_LATCH_SPEED (or after WHEEL_BRAKE_LATCH_DELAY s), the brake latches the wheels' current angles as
# position targets with WHEEL_BRAKE_STIFFNESS; any non-zero command releases it (stiffness back to 0), so driving
# and the velocity calibration are unchanged.
WHEEL_BRAKE = True
WHEEL_BRAKE_STIFFNESS = 1.0e4  # N*m/rad per wheel: a 5 N*m disturbance holds within 0.5 mrad
WHEEL_BRAKE_LATCH_SPEED = 0.05  # rad/s
WHEEL_BRAKE_LATCH_DELAY = 1.0  # s
# Physics time advanced per rendered frame: Kit runs floor(PHYSICS_HZ / SIM_RATE_HZ) fixed PhysX steps per frame
# (22 Hz frames, 60 Hz physics -> 2 steps = 1/30 s, not the 1/22 s the timeline counts). Measured: the position
# change per published odom message is 1/30 s of commanded velocity at SIM_RATE_HZ=22, for every model.
# Consequence: real-time factor = render fps x this, e.g. 14 fps -> 0.47; SIM_RATE_HZ near 60/k (20, 15) would
# make frames whole multiples of the step.
PHYSICS_FRAME_DT = max(1, int(PHYSICS_HZ // SIM_RATE_HZ)) / PHYSICS_HZ

# Ridgeback's real sideways motion (and, see below, its yaw rate). Forward/back is left entirely to the same real
# DifferentialController + IsaacArticulationController wheel driving every model uses (genuine wheel-ground
# rolling -- reliable even from a standstill, exactly like the other three models). Only linear.y is patched in
# here, since real wheel rolling structurally cannot produce it (see sim/config/model_params.yaml's r100 comment). Every tick,
# this reads the chassis' CURRENT actual world velocity, decomposes it into the chassis' own body frame,
# replaces just the lateral component with the commanded vy (leaving the forward component -- whatever the real
# diff-drive wheels produced -- untouched), and recomposes back to world frame. Since the velocity-calibration
# work it also sets the yaw rate (VelCtl's feedback-corrected command): rolling the cylinder wheels alone turned
# the Ridgeback erratically (stick-slip), 0.74-1.0 of the command on repeat runs, now within ~1%.
#
# An earlier version set the FULL (vx, vy, wz) velocity directly every tick, bypassing wheel physics for
# everything, not just Vy, and hit the same limitation described there: a *pure*, small in-place rotation
# command (0.5 rad/s alone, verified live) produced ~0 measured rotation, while a larger one (2.0 rad/s) or one
# combined with any translation came through mostly intact. Switching rotation to the real diff-drive wheels
# (this version) did NOT fix that -- it's the same underlying effect either way: from a standstill, turning in
# place needs each wheel's contact patch to break static friction and scrub sideways (skid-steer's normal
# mechanism), and PhysX's contact solver resists that breakaway far more than it resists continuing an already-
# sliding motion, for both a direct velocity override *and* real wheel torque. Real wheel rolling was kept
# anyway since it's at least as effective, and keeps rotation on the same, already-proven code path as the other
# 3 models rather than adding a second special case to BodyDrive.
#
# Articulation.set_velocities()/.get_velocities() (isaacsim.core.experimental.prims) act on a floating-base
# articulation's *root* velocity, which is what a URDF import with fix_base=False gives every robot here --
# confirmed chassis == that root via ArticulationRootAPI, same check chassis_prim() already relies
# on. Constructing Articulation() needs the physics tensor view, which only exists once the timeline is playing,
# so it's created lazily on first compute rather than in setup(), and re-tried on failure instead of latching a
# permanent error.
BODY_DRIVE_SCRIPT = """
import math

import omni.usd
from pxr import UsdGeom, Gf

from fleet_pose import world_matrix  # noqa: E402


def setup(db):
    db.per_instance_state.articulation = None


def compute(db):
    state = db.per_instance_state
    if state.articulation is None:
        from isaacsim.core.experimental.prims import Articulation
        try:
            state.articulation = Articulation(str(db.inputs.chassisPath))
        except Exception as e:
            db.log_warning(f"BodyDrive: Articulation not ready yet ({e}), retrying next tick")
            return

    rot = world_matrix(str(db.inputs.chassisPath)).ExtractRotation()
    fwd = rot.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))  # chassis' local +X axis, in world frame
    yaw = math.atan2(fwd[1], fwd[0])

    lin, ang = state.articulation.get_velocities()
    cur_wx, cur_wy, _cur_wz = ang.numpy()[0]
    cur_vx_w, cur_vy_w, cur_vz_w = lin.numpy()[0]
    cur_fwd = cur_vx_w * math.cos(yaw) + cur_vy_w * math.sin(yaw)  # decompose actual velocity into body frame

    vy = db.inputs.vy  # commanded lateral speed, body frame -- the only component this overrides
    vx_w = cur_fwd * math.cos(yaw) - vy * math.sin(yaw)
    vy_w = cur_fwd * math.sin(yaw) + vy * math.cos(yaw)
    # yaw rate too (VelCtl's feedback-corrected value): rolling the cylinder wheels alone turns the Ridgeback
    # erratically (stick-slip), measured 0.74-1.0 of the command on repeat runs
    state.articulation.set_velocities(
        linear_velocities=[[vx_w, vy_w, cur_vz_w]], angular_velocities=[[cur_wx, cur_wy, float(db.inputs.wz)]]
    )
"""

WHEEL_BRAKE_SCRIPT = """
import os
import time

import numpy as np

_DEBUG = os.environ.get("FLEET_DEBUG", "0") == "1"


def setup(db):
    st = db.per_instance_state
    st.art = None
    st.latched = False
    st.idle_since = None
    st.last_dbg = 0.0


def _log(msg):
    print(f"[fleet] WheelBrake {msg}", flush=True)


def compute(db):
    st = db.per_instance_state
    if st.art is None:
        from isaacsim.core.experimental.prims import Articulation
        try:
            st.art = Articulation(str(db.inputs.chassisPath))
            names = list(st.art.dof_names)
            st.dofs = [names.index(n) for n in str(db.inputs.wheelNames).split(",") if n in names]
            _log(f"{db.inputs.chassisPath}: wheel dofs {st.dofs}")
        except Exception as e:
            db.log_warning(f"WheelBrake: articulation not ready yet ({e}), retrying next tick")
            st.art = None
            return
    commanded = max(abs(float(db.inputs.cmd_v)), abs(float(db.inputs.cmd_w)), abs(float(db.inputs.cmd_y))) > 1e-4
    now = time.monotonic()
    if _DEBUG and now - st.last_dbg > 5.0:
        st.last_dbg = now
        stiff, damp = (g.numpy()[0] for g in st.art.get_dof_gains(dof_indices=st.dofs))
        pos = st.art.get_dof_positions(dof_indices=st.dofs).numpy()[0]
        tgt = st.art.get_dof_position_targets(dof_indices=st.dofs).numpy()[0]
        vel = st.art.get_dof_velocities(dof_indices=st.dofs).numpy()[0]
        _log(f"latched={st.latched} commanded={commanded} stiffness={np.round(stiff, 1)} damping={np.round(damp, 2)} "
             f"pos-target={np.round(pos - tgt, 4)} vel={np.round(vel, 3)}")
    if commanded:
        st.idle_since = None
        if st.latched:
            st.art.set_dof_gains(stiffnesses=np.zeros(len(st.dofs)), dof_indices=st.dofs)
            st.latched = False
            if _DEBUG:
                _log("released")
        return
    if st.latched:
        return
    if st.idle_since is None:
        st.idle_since = now
    speed = np.abs(st.art.get_dof_velocities(dof_indices=st.dofs).numpy()[0]).max()
    if speed < float(db.inputs.latchSpeed) or now - st.idle_since > float(db.inputs.latchDelay):
        pos = st.art.get_dof_positions(dof_indices=st.dofs).numpy()
        st.art.set_dof_position_targets(pos, dof_indices=st.dofs)
        st.art.set_dof_gains(stiffnesses=np.full(len(st.dofs), float(db.inputs.stiffness)), dof_indices=st.dofs)
        st.latched = True
        if _DEBUG:
            _log(f"latched at wheel speed {speed:.3f} rad/s")
"""

# Velocity feedback between cmd_vel and the DifferentialController (see VELCTL_* below): PI on the chassis'
# measured body-frame forward speed and yaw rate, plus feed-forward (the commanded value itself).
VEL_CTL_SCRIPT = """
import math

import omni.usd
from pxr import UsdGeom, Gf

from fleet_pose import world_matrix  # noqa: E402


def setup(db):
    st = db.per_instance_state
    st.prev = None
    st.prev_t = None
    st.int_v = 0.0
    st.int_w = 0.0
    st.int_y = 0.0
    st.mv = 0.0
    st.mw = 0.0
    st.my = 0.0


def _chan(cmd, meas, integ, kp, ki, dt, lim, ff=1.0):
    if abs(cmd) < 1e-4:
        return 0.0, 0.0
    e = cmd - meas
    integ = max(-lim, min(lim, integ + ki * e * dt))
    return ff * cmd + kp * e + integ, integ


def compute(db):
    st = db.per_instance_state
    cmd_v = max(-db.inputs.max_v, min(db.inputs.max_v, float(db.inputs.cmd_v)))
    cmd_w = max(-db.inputs.max_w, min(db.inputs.max_w, float(db.inputs.cmd_w)))
    cmd_y = max(-db.inputs.max_v, min(db.inputs.max_v, float(db.inputs.cmd_y)))
    # Measured from the chassis' pose change per physics frame, NOT from PhysX's reported velocity: the reported
    # angular velocity read ~0.02-0.03 rad/s above the actual yaw change (a 0.1 rad/s command "measured" 0.1 while
    # the robot turned 0.078), so a loop closed on it settled 20% low at low rates.
    xf = world_matrix(str(db.inputs.chassisPath))
    pos = xf.ExtractTranslation()
    fwd = xf.ExtractRotation().TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
    yaw = math.atan2(fwd[1], fwd[0])
    dt = float(db.inputs.dt)
    t = float(db.inputs.simTime)  # connected under Newton only: its frames aren't PHYSICS_FRAME_DT long
    if st.prev_t is not None and 0.0 < t - st.prev_t < 1.0:
        dt = t - st.prev_t
    st.prev_t = t if t > 0.0 else None
    if st.prev is not None:
        px, py, pyaw = st.prev
        a = float(db.inputs.alpha)
        v = ((pos[0] - px) * math.cos(yaw) + (pos[1] - py) * math.sin(yaw)) / dt
        w = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw)) / dt
        y = (-(pos[0] - px) * math.sin(yaw) + (pos[1] - py) * math.cos(yaw)) / dt
        st.mv += a * (v - st.mv)
        st.mw += a * (w - st.mw)
        st.my += a * (y - st.my)
    st.prev = (pos[0], pos[1], yaw)
    db.outputs.odom_lin = (st.mv, st.my, 0.0)
    db.outputs.odom_ang = (0.0, 0.0, st.mw)
    if not db.inputs.enabled:
        db.outputs.out_v = cmd_v
        db.outputs.out_w = cmd_w
        db.outputs.out_y = cmd_y
        return
    db.outputs.out_y, st.int_y = _chan(cmd_y, st.my, st.int_y, db.inputs.kp_y, db.inputs.ki_y, dt, db.inputs.lim_y)
    db.outputs.out_v, st.int_v = _chan(cmd_v, st.mv, st.int_v, db.inputs.kp_v, db.inputs.ki_v, dt, db.inputs.lim_v,
                                       db.inputs.ff_v)
    db.outputs.out_w, st.int_w = _chan(cmd_w, st.mw, st.int_w, db.inputs.kp_w, db.inputs.ki_w, dt, db.inputs.lim_w,
                                       db.inputs.ff_w)
"""

# Real MTU robots' Microstrain IMU (see derive_model_params' body() for why it's physically attached to
# chassis_link, not the real imu_1_link name). isaacsim.sensors.experimental.physics has no OGN "read" node, so
# this authors the actual IsaacImuSensor prim on first tick (lazily, same reasoning as BodyDrive's Articulation:
# needs the physics tensor view, which only exists once playing) and reads it every tick after.
# IMUSensor.get_data()'s orientation is [w, x, y, z]; ROS2PublishImu's quatd[4] input wants IJKR (x, y, z, w).
IMU_READ_SCRIPT = """
def setup(db):
    db.per_instance_state.sensor = None


def compute(db):
    state = db.per_instance_state
    if state.sensor is None:
        from isaacsim.sensors.experimental.physics import IMU, IMUSensor
        path = str(db.inputs.imuPath)
        try:
            IMU.create(path)
            state.sensor = IMUSensor(path)
        except Exception as e:
            db.log_warning(f"ImuRead: sensor not ready yet ({e}), retrying next tick")
            return

    frame = state.sensor.get_data()
    o = frame["orientation"]
    db.outputs.orientation = [float(o[1]), float(o[2]), float(o[3]), float(o[0])]
    db.outputs.linearAcceleration = [float(v) for v in frame["linear_acceleration"]]
    db.outputs.angularVelocity = [float(v) for v in frame["angular_velocity"]]
"""

# The same IMU under Newton (SIM_PHYSICS=newton): isaacsim.sensors.experimental.physics is a PhysX plugin (driven by
# PhysX step events, reading omni.physx.tensors), so the reading is computed here from the articulation root's
# velocity (Articulation.get_velocities: engine-neutral tensor API, as BodyDrive uses it) once per frame: the linear
# acceleration is the velocity's change over the frame (dt = PHYSICS_FRAME_DT) plus gravity (specific force: +g up
# at rest), both rotated into the IMU frame (world_matrix of the IMU body, from Fabric). Assumes the IMU body is
# the articulation root's body (true for the real MTU robots: derive_model_params folds the IMU into chassis_link).
IMU_NEWTON_SCRIPT = """
import numpy as np
from pxr import Gf

from fleet_pose import world_matrix  # noqa: E402


def setup(db):
    st = db.per_instance_state
    st.articulation = None
    st.prev_v = None
    st.prev_t = None


def compute(db):
    st = db.per_instance_state
    if st.articulation is None:
        from isaacsim.core.experimental.prims import Articulation
        try:
            st.articulation = Articulation(str(db.inputs.chassisPath))
        except Exception as e:
            db.log_warning(f"ImuRead: Articulation not ready yet ({e}), retrying next tick")
            return
    m = world_matrix(str(db.inputs.imuPath))
    inv = m.ExtractRotation().GetInverse()
    q = m.ExtractRotationQuat().GetNormalized()
    lin, ang = st.articulation.get_velocities()
    v = np.array(lin.numpy()[0], dtype=float)
    w = np.array(ang.numpy()[0], dtype=float)
    dt, t = float(db.inputs.dt), float(db.inputs.simTime)  # simTime: the frame's real sim time (see VelCtl)
    if st.prev_t is not None and 0.0 < t - st.prev_t < 1.0:
        dt = t - st.prev_t
    st.prev_t = t if t > 0.0 else None
    a_world = (v - st.prev_v) / dt if st.prev_v is not None else np.zeros(3)
    st.prev_v = v
    a_world[2] += 9.81  # specific force: an IMU at rest reads +g along the world's up
    a = inv.TransformDir(Gf.Vec3d(*a_world))
    g = inv.TransformDir(Gf.Vec3d(*w))
    qi = q.GetImaginary()
    db.outputs.orientation = [float(qi[0]), float(qi[1]), float(qi[2]), float(q.GetReal())]
    db.outputs.linearAcceleration = [float(a[0]), float(a[1]), float(a[2])]
    db.outputs.angularVelocity = [float(g[0]), float(g[1]), float(g[2])]
"""

# Real MTU robots' dual SwiftNav Duro GPS: no real satellite geometry (this is a simulation, not an RF model --
# same simplification Gazebo's own GPS plugins make), just a flat-earth/equirectangular projection of the
# chassis' actual simulated world XY around a fixed reference origin, computed fresh every tick from each GPS
# antenna's own real link position (not just the chassis) so the two antennas' real ~0.56m separation still
# shows up as a small, physically meaningful difference between the two NavSatFix readings -- SwiftNav Duro
# pairs are commonly used for exactly that, deriving heading from dual-antenna GPS. Origin: Michigan Tech's
# Houghton, MI campus (~47.1211, -88.5455, ~326m elevation) -- ties the fake origin to the real institution
# these robots belong to (mtu32_description, *.sabu.mtu.edu hostnames in the real robot.yaml) rather than an
# arbitrary placeholder. Sim world X/Y map to East/North (ENU), matching the real robot.yaml's own
# microstrain_imu use_enu_frame: true.
#
# Publishes sensor_msgs/NavSatFix via a plain rclpy publisher created directly in this script, not via
# isaacsim.ros2.bridge.ROS2Publisher (the generic any-message OGN node, tried first). That node's own literal
# SET_VALUES on its dynamically-created attributes work fine (confirmed live: a literal test altitude came
# through correctly), but a *connection* from another node's dynamically-created output into one of its
# dynamically-created inputs silently never propagates (confirmed live: latitude/longitude stayed exactly 0.0,
# not merely wrong -- ruling out a units/sign bug -- while status/covariance, set as literals, worked) -- since
# GPS position is inherently a per-tick computed value, not a literal, a real connection is unavoidable here,
# so this bypasses that node entirely. Isaac Sim's ROS2 bridge already loads an internal rclpy into this same
# process (confirmed live in the sim's own boot log) and every container in this fleet shares one
# RMW_IMPLEMENTATION/ROS_DOMAIN_ID, so a plain rclpy.init()/Node() here joins the same ROS graph the OmniGraph
# ROS2 bridge nodes do, no special context wiring needed. As a bonus this also gives a real, populated
# header.frame_id/stamp, which the generic publisher route left empty/zero with no way found to set them.
GPS_READ_SCRIPT = """
import math

import omni.usd
import rclpy
from pxr import UsdGeom
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

_ORIGIN_LAT = 47.1211
_ORIGIN_LON = -88.5455
_ORIGIN_ALT = 326.0
_M_PER_DEG_LAT = 111320.0


from fleet_nodes import stamp as _stamp  # noqa: E402
from fleet_pose import world_matrix  # noqa: E402


def setup(db):
    db.per_instance_state.node = None
    db.per_instance_state.pub = None


def compute(db):
    state = db.per_instance_state
    if state.node is None:
        if not rclpy.ok():
            rclpy.init()
        safe_name = str(db.inputs.topicName).strip("/").replace("/", "_")
        state.node = Node(f"gps_read_{safe_name}", namespace=str(db.inputs.namespace))
        state.pub = state.node.create_publisher(NavSatFix, str(db.inputs.topicName), 10)
        import fleet_nodes  # destroyed by a respawn (cleanup() doesn't run when the graph is deleted)
        fleet_nodes.track(db.inputs.namespace, state)

    pos = world_matrix(str(db.inputs.gpsPath)).ExtractTranslation()
    m_per_deg_lon = _M_PER_DEG_LAT * math.cos(math.radians(_ORIGIN_LAT))

    msg = NavSatFix()
    msg.header.stamp = _stamp(db.inputs.stamp)
    msg.header.frame_id = str(db.inputs.frameId)
    msg.status.status = 0  # STATUS_FIX
    msg.status.service = 1  # SERVICE_GPS
    msg.latitude = _ORIGIN_LAT + pos[1] / _M_PER_DEG_LAT
    msg.longitude = _ORIGIN_LON + pos[0] / m_per_deg_lon
    msg.altitude = _ORIGIN_ALT + pos[2]
    msg.position_covariance_type = 0  # COVARIANCE_TYPE_UNKNOWN
    state.pub.publish(msg)


def cleanup(db):
    state = db.per_instance_state
    if state.node is not None:
        state.node.destroy_node()
        state.node = None
        state.pub = None
"""

# cmd_vel as geometry_msgs/TwistStamped, like the real Clearpath (Jazzy) platform: isaacsim.ros2.bridge's
# Reference pose (SIM_REF_POSE): base_link's exact pose in the Isaac world frame, published like the lab's motion
# capture client (mocap_fake_localizer's natnet_ref_pose.py) does on a real robot: nav_msgs/Odometry on ref_pose,
# frame ref_frame -> base_link_ref, plus the same pose on the robot's own tf. ref_frame is the world frame (the GPS
# datum's origin), unlike the ground_truth TF branch, which starts at zero at the spawn pose. Same in-process rclpy
# mechanism as GPS_READ_SCRIPT. base_link sits at localPos/localQuat on the chassis (articulation root) body, read
# once at spawn: the chassis is what PhysX moves, while base_link can be a plain parent Xform that stays put.
REF_POSE_SCRIPT = """
import omni.usd
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from pxr import Gf, UsdGeom
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


from fleet_nodes import stamp as _stamp  # noqa: E402
from fleet_pose import world_matrix  # noqa: E402


def setup(db):
    db.per_instance_state.node = None


def compute(db):
    state = db.per_instance_state
    if state.node is None:
        if not rclpy.ok():
            rclpy.init()
        ns = str(db.inputs.namespace)
        state.node = Node("ref_pose_sim", namespace=ns)
        state.pub = state.node.create_publisher(Odometry, f"/{ns}/ref_pose", 10)
        state.tf = state.node.create_publisher(TFMessage, f"/{ns}/tf", 100)
        p, q = [float(v) for v in db.inputs.localPos], [float(v) for v in db.inputs.localQuat]
        state.local = Gf.Matrix4d().SetTransform(Gf.Rotation(Gf.Quatd(q[0], q[1], q[2], q[3])), Gf.Vec3d(*p))
        import fleet_nodes  # destroyed by a respawn (cleanup() doesn't run when the graph is deleted)
        fleet_nodes.track(ns, state)

    m = state.local * world_matrix(str(db.inputs.chassisPath))  # base_link -> world (row vectors)
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat().GetNormalized()
    qi = q.GetImaginary()

    msg = Odometry()
    msg.header.stamp = _stamp(db.inputs.stamp)
    msg.header.frame_id = str(db.inputs.refFrame)
    msg.child_frame_id = str(db.inputs.childFrame)
    pos, rot = msg.pose.pose.position, msg.pose.pose.orientation
    pos.x, pos.y, pos.z = float(t[0]), float(t[1]), float(t[2])
    rot.x, rot.y, rot.z, rot.w = float(qi[0]), float(qi[1]), float(qi[2]), float(q.GetReal())
    cov = [0.0] * 36
    for i in range(6):
        cov[i * 7] = 1e-6  # exact
    msg.pose.covariance = cov
    msg.twist.covariance[0] = -1.0  # no twist
    state.pub.publish(msg)

    tf = TransformStamped()
    tf.header = msg.header
    tf.child_frame_id = msg.child_frame_id
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = pos.x, pos.y, pos.z
    tf.transform.rotation = rot
    state.tf.publish(TFMessage(transforms=[tf]))


def cleanup(db):
    state = db.per_instance_state
    if state.node is not None:
        state.node.destroy_node()
        state.node = None
"""

# ROS2SubscribeTwist only takes plain Twist and has no stamped option, so this ScriptNode subscribes with an
# in-process rclpy node (same mechanism and reasoning as GPS_READ_SCRIPT) and exposes the same outputs
# (linearVelocity/angularVelocity, holding the last message up to CMD_VEL_TIMEOUT_S), so the drive graph is
# unchanged. Pending messages are drained every tick without blocking.
# A command older than this (sim time) stops the robot, like the real platform (twist_mux / diff_drive_controller,
# 0.5 s): without it a robot whose sender died (teleop, Nav2, a killed container at Reset) drove on forever.
CMD_VEL_TIMEOUT_S = 0.5
CMD_VEL_SCRIPT = """
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


def setup(db):
    db.per_instance_state.node = None


def compute(db):
    state = db.per_instance_state
    now = db.inputs.stamp  # sim time (wall time without USE_SIM_TIME), like every message's stamp
    if state.node is None:
        if not rclpy.ok():
            rclpy.init()
        ns = str(db.inputs.namespace)
        state.node = Node(f"cmd_vel_sub_{ns}", namespace=ns)
        state.last = None
        state.last_t = None

        def on_cmd(msg):
            state.last = msg

        state.sub = state.node.create_subscription(TwistStamped, str(db.inputs.topicName), on_cmd, 10)
        state.executor = SingleThreadedExecutor()
        state.executor.add_node(state.node)
        import fleet_nodes  # destroyed by a respawn (cleanup() doesn't run when the graph is deleted)
        fleet_nodes.track(ns, state)
    for _ in range(20):  # drain everything queued since the last tick
        before = state.last
        state.executor.spin_once(timeout_sec=0.0)
        if state.last is before:
            break
        state.last_t = now
    if state.last is not None and now - state.last_t > db.inputs.timeout:
        state.last = None  # the sender stopped (or died): stop, as the real base does
    t = state.last.twist if state.last is not None else None
    db.outputs.linearVelocity = [t.linear.x, t.linear.y, t.linear.z] if t else [0.0, 0.0, 0.0]
    db.outputs.angularVelocity = [t.angular.x, t.angular.y, t.angular.z] if t else [0.0, 0.0, 0.0]


def cleanup(db):
    state = db.per_instance_state
    if state.node is not None:
        state.executor.shutdown()
        state.node.destroy_node()
        state.node = None
"""

# Real MTU robots' 2D lidar (a200_0333's Hokuyo UST, j100_0936's SICK LMS1xx): both declare (or, for the SICK,
# really have -- clearpath_config's own lms1xx schema just doesn't expose it as a robot.yaml field the way
# urg_node's does) the same ~270deg FOV -- hokuyo's is the real robot.yaml's own urg_node.angle_min/max
# (-2.356/2.356 rad); SICK's isn't in its robot.yaml at all, so this uses the same value, matching the real
# LMS1xx hardware's own published FOV. NUM_RAYS/RANGE_* are approximate (0.5deg resolution, not either sensor's
# exact real spec) -- shared module-level constants rather than a MODEL_PARAMS field per robot, since both real
# robots that need this happen to agree; revisit if a future robot's 2D lidar genuinely differs.
LIDAR2D_ANGLE_MIN = -2.356
LIDAR2D_ANGLE_MAX = 2.356
LIDAR2D_NUM_RAYS = 541  # ~0.5deg resolution over the 270deg FOV
LIDAR2D_RANGE_MIN = 0.1
LIDAR2D_RANGE_MAX = 10.0

# The 2D/3D lidars cast their rays themselves, from their script nodes, with PhysX's scene-query API
# (omni.physx get_physx_scene_query_interface().raycast_closest), once per frame (the 3D one spread over
# LIDAR3D_TICKS_PER_SCAN frames). They used to be isaacsim.sensors.experimental.physics Raycast/RaycastSensor prims,
# but that plugin segfaults the whole sim (exit 139) inside its own per-physics-step update, a second or two after
# play: a200_0333 (its only sensors are the two lidars) as the first spawn at (0.4, -0.5) crashed 6/6 with the scene/
# spawn split and 2/4 with the older one-shot startup, 0/5 with both lidars left out and 0/5 with these raycasts;
# the user's 3-robot fleet crashed the same way at the default poses in headed mode. The minidump's faulting frame is
# in libisaacsim.sensors.experimental.physics.plugin.so under omni.physx's step, dereferencing a garbage pointer --
# not fixable from here. (That plugin also reported every ray's 'depths' as min_range; ranges came from
# 'hit_positions'.) Rays start at range_min along their direction, like the plugin's min_range, and a miss is +Inf
# (REP-117). Like the plugin, they also hit the robot's own colliders (Nav2's laser_filters box removes those).
# Cost, a200_0333 alone (FLEET_DEBUG=1): render_fps ~29 without lidars, ~21 with both (~13 ms per frame for 541 +
# 1440 rays), about what the plugin's 3D lidar alone cost (12.1 -> 10.4 fps in a 4-robot fleet).
LIDAR2D_READ_SCRIPT = """
import math

import numpy as np
import omni.usd
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


from fleet_nodes import stamp as _stamp  # noqa: E402
from fleet_pose import ray_caster, world_matrix  # noqa: E402


def setup(db):
    db.per_instance_state.sensor = None
    db.per_instance_state.node = None
    db.per_instance_state.pub = None


def compute(db):
    state = db.per_instance_state
    angle_min = float(db.inputs.angleMin)
    angle_max = float(db.inputs.angleMax)
    num_rays = int(db.inputs.numRays)
    range_min = float(db.inputs.rangeMin)
    range_max = float(db.inputs.rangeMax)

    if state.sensor is None:
        state.cast = ray_caster()  # PhysX scene queries, or Newton's raytracer (fleet_pose / fleet_newton)
        if state.cast is None:
            return  # Newton hasn't parsed the scene yet (it does at the first physics step)
        state.sensor = str(db.inputs.sensorPath)  # the lidar link prim; rays fan out in its x-y plane
        angles = np.linspace(angle_min, angle_max, num_rays)
        state.dirs = np.stack([np.cos(angles), np.sin(angles), np.zeros(num_rays)], axis=1)

    if state.node is None:
        if not rclpy.ok():
            rclpy.init()
        safe_name = str(db.inputs.topicName).strip("/").replace("/", "_")
        state.node = Node(f"lidar2d_read_{safe_name}", namespace=str(db.inputs.namespace))
        state.pub = state.node.create_publisher(LaserScan, str(db.inputs.topicName), 10)
        import fleet_nodes  # destroyed by a respawn (cleanup() doesn't run when the graph is deleted)
        fleet_nodes.track(db.inputs.namespace, state)

    if not omni.usd.get_context().get_stage().GetPrimAtPath(state.sensor).IsValid():
        return
    m = np.array(world_matrix(state.sensor))  # lidar link -> world (row vectors)
    w = state.dirs @ m[:3, :3]
    w /= np.linalg.norm(w, axis=1, keepdims=True)
    s = m[3, :3] + w * range_min  # each ray starts at range_min, like the plugin's min_range
    d = state.cast(s, w, range_max - range_min)  # distance from each ray's start, +Inf for a miss
    ranges = [float(v) + range_min if math.isfinite(v) else math.inf for v in d]  # a miss is +Inf (REP-117)
    n = len(ranges)

    msg = LaserScan()
    msg.header.stamp = _stamp(db.inputs.stamp)
    msg.header.frame_id = str(db.inputs.frameId)
    msg.angle_min = angle_min
    msg.angle_max = angle_max
    msg.angle_increment = (angle_max - angle_min) / max(n - 1, 1)
    msg.range_min = range_min
    msg.range_max = range_max
    msg.ranges = ranges
    state.pub.publish(msg)


def cleanup(db):
    state = db.per_instance_state
    if state.node is not None:
        state.node.destroy_node()
        state.node = None
        state.pub = None
"""

# a200_0333's real Velodyne VLP16: 16 channels over a real ±15deg vertical FOV (VLP16's actual, evenly-2deg-
# spaced channel angles -- real hardware fires them in an interleaved, non-sequential order for timing reasons,
# irrelevant here since this is a per-tick snapshot, not a simulated scan sweep), full 360deg horizontal.
# H_COUNT: 1deg horizontal resolution, 5760 rays per scan.
# RANGE_MIN/MAX are real-hardware-representative (VLP16's real minimum is close to this; its real 100m max is
# cut down to something sane for this scene's own scale). Z_OFFSET nudges every ray's own origin up by 4cm in
# the sensor's local frame before casting -- found live that the flattened URDF's own lidar3d_0_link collision
# (a cylinder representing the VLP16's base housing) has its top surface only ~3.4cm above lidar3d_0_laser's
# own frame origin, so an unmoved horizontal ray would immediately self-intersect that housing; this offset
# clears it (re-verified live: horizontal rays now correctly reach real scene geometry, not an immediate ~5cm
# self-hit).
LIDAR3D_V_ANGLE_MIN = -0.2618  # -15deg
LIDAR3D_V_ANGLE_MAX = 0.2618  # +15deg
LIDAR3D_V_COUNT = 16
LIDAR3D_H_COUNT = 360  # 1deg horizontal resolution, 5760 rays total -- see comment above
LIDAR3D_RANGE_MIN = 0.4
LIDAR3D_RANGE_MAX = 30.0
LIDAR3D_Z_OFFSET = 0.04
LIDAR3D_TICKS_PER_SCAN = 4  # a full cloud every 4 frames (~5 Hz at SIM_RATE_HZ 22); 4 of the 16 channels per frame

# Same scene-query raycasts as LIDAR2D_READ_SCRIPT (see the comment above it). Publishes sensor_msgs/PointCloud2
# (unorganized: height=1, width=point count) in the lidar's own frame; a ray starts at (0, 0, Z_OFFSET) + range_min
# along its direction and misses are dropped, as a real point cloud only carries returns. A full cloud is published
# every ticksPerScan frames (v_count/ticksPerScan channels per frame), each channel cast from the pose of its frame,
# so a moving robot's cloud is slightly skewed, like a real spinning lidar's.
LIDAR3D_READ_SCRIPT = """
import math

import numpy as np
import omni.usd
import rclpy
from pxr import Gf
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


from fleet_nodes import stamp as _stamp  # noqa: E402
from fleet_pose import ray_caster, world_matrix  # noqa: E402


def setup(db):
    db.per_instance_state.sensor = None
    db.per_instance_state.node = None
    db.per_instance_state.pub = None


def compute(db):
    state = db.per_instance_state
    v_min = float(db.inputs.vAngleMin)
    v_max = float(db.inputs.vAngleMax)
    v_count = int(db.inputs.vCount)
    h_count = int(db.inputs.hCount)
    range_min = float(db.inputs.rangeMin)
    range_max = float(db.inputs.rangeMax)
    z_offset = float(db.inputs.zOffset)

    if state.sensor is None:
        state.cast = ray_caster()  # PhysX scene queries, or Newton's raytracer (fleet_pose / fleet_newton)
        if state.cast is None:
            return  # Newton hasn't parsed the scene yet (it does at the first physics step)
        state.sensor = str(db.inputs.sensorPath)  # the chassis body; the lidar sits at localPos/localQuat on it
        p, q = [float(v) for v in db.inputs.localPos], [float(v) for v in db.inputs.localQuat]
        state.local = np.array(Gf.Matrix4d().SetTransform(Gf.Rotation(Gf.Quatd(q[0], q[1], q[2], q[3])), Gf.Vec3d(*p)))
        h = 2.0 * math.pi * np.arange(h_count) / h_count
        state.rows = []  # one (h_count, 3) array of unit directions (lidar frame) per vertical channel
        for v in np.linspace(v_min, v_max, v_count):
            state.rows.append(np.stack([math.cos(v) * np.cos(h), math.cos(v) * np.sin(h), np.full(h_count, math.sin(v))], axis=1))
        state.next_row = 0
        state.buf = bytearray()
        state.n = 0

    if state.node is None:
        if not rclpy.ok():
            rclpy.init()
        safe_name = str(db.inputs.topicName).strip("/").replace("/", "_")
        state.node = Node(f"lidar3d_read_{safe_name}", namespace=str(db.inputs.namespace))
        state.pub = state.node.create_publisher(PointCloud2, str(db.inputs.topicName), 10)
        import fleet_nodes  # destroyed by a respawn (cleanup() doesn't run when the graph is deleted)
        fleet_nodes.track(db.inputs.namespace, state)

    # A scan is spread over ticksPerScan ticks (a few channels per tick): 5760 PhysX raycasts in one tick would cost
    # tens of ms. Points are in the lidar frame, measured from each ray's origin (0, 0, zOffset); misses are dropped.
    if not omni.usd.get_context().get_stage().GetPrimAtPath(state.sensor).IsValid():
        return
    m = state.local @ np.array(world_matrix(state.sensor))  # lidar -> world (row vectors)
    o = np.array([0.0, 0.0, z_offset, 1.0]) @ m  # ray origin, world
    per_tick = max(1, -(-len(state.rows) // max(1, int(db.inputs.ticksPerScan))))
    rows = np.concatenate(state.rows[state.next_row:state.next_row + per_tick])  # this tick's channels, one cast
    w = rows @ m[:3, :3]
    w /= np.linalg.norm(w, axis=1, keepdims=True)
    d = state.cast(o[:3] + w * range_min, w, range_max - range_min)
    hit = np.isfinite(d)
    pts = rows[hit] * (d[hit] + range_min)[:, None]
    pts[:, 2] += z_offset
    state.buf += pts.astype("<f4").tobytes()
    state.n += int(hit.sum())
    state.next_row += per_tick
    if state.next_row < len(state.rows):
        return
    buf, n = state.buf, state.n
    state.next_row, state.buf, state.n = 0, bytearray(), 0

    msg = PointCloud2()
    msg.header.stamp = _stamp(db.inputs.stamp)
    msg.header.frame_id = str(db.inputs.frameId)
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.data = bytes(buf)
    msg.is_dense = True
    state.pub.publish(msg)


def cleanup(db):
    state = db.per_instance_state
    if state.node is not None:
        state.node.destroy_node()
        state.node = None
        state.pub = None
"""

ROBOT_SPACING = 1.6  # m between robot lanes along Y
SCENE_LANES = max(1, int(os.environ.get("SCENE_LANES", "3")))  # lanes between the lavender rows (scene layout)
RANK_SPACING = 2.5  # m between ranks along X, when there are more robots than lanes
SPAWN_Z = 0.15  # base_link height: wheel bottoms end up ~1.4 cm above the ground, then it settles
SPAWN_LIMIT = 38.0  # |x|, |y| of a spawn pose: inside the 80 x 80 m ground box
SPAWN_MIN_DIST = 1.0  # m between two robots' spawn points (they would start inside each other)


def default_poses(n):
    """Default spawn poses (x, y, yaw deg) for n robots: SCENE_LANES robots side by side at x=0, facing +X (the
    lanes between the lavender rows), the next ones in further ranks behind them; each rank centred on y=0.
    Up to SCENE_LANES robots this is the layout the sim always used."""
    poses = []
    for i in range(n):
        rank, lane = divmod(i, SCENE_LANES)
        in_rank = min(SCENE_LANES, n - rank * SCENE_LANES)
        poses.append((-rank * RANK_SPACING, (lane - (in_rank - 1) / 2) * ROBOT_SPACING, 0.0))
    return poses


# D435i RGB sensor: 69.4 deg horizontal FOV
HFOV_DEG = 69.4
# ZED2i (real MTU robots, HD720 general.grab_resolution): Stereolabs' published horizontal FOV at 16:9.
ZED_HFOV_DEG = 87.0
# RealSense D405 (j100_0921's wrist camera_1): published depth/colour horizontal FOV ~87 deg.
D405_HFOV_DEG = 87.0


def find_prim(stage, root, name):
    """First prim called `name` under `root` (the importer's nesting depends on its merge settings)."""
    from pxr import Usd

    for prim in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()):
        if prim.GetName() == name:
            return str(prim.GetPath())
    raise RuntimeError(f"no prim named {name!r} under {root}")


def log(msg):
    print(f"[fleet] {msg}", flush=True)


def apply_kit_settings():
    """FLEET_SETTINGS="/path/a=1;/path/b=text" -> carb settings, applied before the scene is built."""
    import carb.settings

    st = carb.settings.get_settings()
    for item in filter(None, os.environ.get("FLEET_SETTINGS", "").split(";")):
        key, _, raw = item.partition("=")
        val = {"true": True, "false": False}.get(raw.lower())
        if val is None:
            for cast in (int, float, str):
                try:
                    val = cast(raw)
                    break
                except ValueError:
                    continue
        st.set(key.strip(), val)
        log(f"setting {key.strip()} = {val!r}")
    # The sim's own graphs are full of ScriptNodes (CmdVel, VelCtl, WheelBrake, GPS/IMU/lidar readers, RefPose). When a
    # stage containing them is opened -- e.g. the file Isaac reopens after File > Save As -- omni.graph.scriptnode
    # asks "enable the scriptnode functionality for this session?", and "No" disables them all (robots stop driving,
    # sensors stop). The sim authored them, so opt in for the session up front: the dialog is then skipped.
    st.set("/app/omni.graph.scriptnode/opt_in", True)
    # Harmless but per-frame: IsaacReadSystemTime falls back to "now" when it has no sim-time history.
    for channel in ("isaacsim.core.simulation_manager.plugin",):
        st.set(f"/log/channels/{channel}", "error")
    log(f"rtx rendermode = {st.get('/rtx/rendermode')!r}, dlss = {st.get('/rtx/post/dlss/execMode')!r}, "
        f"aa = {st.get('/rtx/post/aa/op')!r}, timeCodesPerSecond-fixed = {st.get('/app/player/useFixedTimeStepping')!r}")


# NVIDIA's general Omniverse libraries, on the same public S3 bucket as the Isaac assets but outside
# Assets/Isaac/, so the Content tab's "Isaac Sim" collection (isaacsim.gui.content_browser) doesn't list them.
_OV_CONTENT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
OMNIVERSE_FOLDERS = [f"{_OV_CONTENT}/{p}" for p in (
    "Assets/ArchVis", "Assets/Characters", "Assets/DigitalTwin", "Assets/Scenes", "Assets/Skies",
    "Assets/Terrain", "Assets/Vegetation", "Assets/Particles", "Assets/simready_content",
    "Environments", "Materials", "Samples", "Demos",
)]


def add_omniverse_collection():
    """An "Omniverse" collection in the Content tab next to "Isaac Sim", listing OMNIVERSE_FOLDERS. Reuses
    isaacsim.gui.content_browser's read-only collection; its folder list is read at extension startup, so it
    can't be extended from FLEET_SETTINGS (applied later). No content window (no UI) -> nothing to do."""
    try:
        import omni.client
        from isaacsim.gui.content_browser.impl.isaac_collection import ICON_PATH, IsaacCollection
        from omni.kit.window.content_browser import get_content_window
        from omni.kit.window.filepicker import CollectionItem
    except ImportError:
        return
    window = get_content_window()
    if not window:
        return

    class OmniverseCollection(IsaacCollection):
        def __init__(self):  # IsaacCollection's own __init__ fixes the title and reads the Isaac folder list
            CollectionItem.__init__(self, identifier="Omniverse", title="Omniverse", protocol="https",
                                    icon=f"{ICON_PATH}/cloud.svg", access=omni.client.AccessFlags.READ,
                                    populated=False, order=6)  # right after "Isaac Sim" (5)
            self._asset_root = _OV_CONTENT
            self._folders = OMNIVERSE_FOLDERS

    window.api.register_collection_item(OmniverseCollection())
    log(f"content browser: Omniverse collection with {len(OMNIVERSE_FOLDERS)} folders")


def install_depth_range_limit():
    """Limit every robot depth image to its sensor's real range (_depth_ranges, DEPTH_RANGE_M): pixels outside it
    become 0. ROS2CameraHelper(type=depth) has no range input. The node writer it takes from rep.writers.get
    (DistanceToImagePlaneSDROS2[SystemTime]PublishImage) feeds ROS2PublishImage a GPU pointer
    (...IsaacPassthroughImagePtr), which can't be augmented; once the writer is initialized for a registered topic,
    that annotator is swapped for `distance_to_image_plane` with a Warp augmentation, feeding the node's `data`
    input (the route of NVIDIA's camera-noise tutorial). The timestamp connection is kept. A depth topic with no
    registered range is published unchanged; any failure leaves the writer as it was (logged)."""
    import omni.replicator.core as rep
    import warp as wp

    @wp.kernel
    def depth_window(data_in: wp.array2d(dtype=wp.float32), data_out: wp.array2d(dtype=wp.float32),
                     min_m: float, max_m: float):
        i, j = wp.tid()
        d = data_in[i, j]
        data_out[i, j] = wp.where(d >= min_m and d <= max_m, d, 0.0)

    orig_get = rep.writers.get
    registered = set()

    def get(name, *args, **kwargs):
        writer = orig_get(name, *args, **kwargs)
        if not (name.startswith("DistanceToImagePlaneSDROS2") and name.endswith("PublishImage")):
            return writer
        orig_init = writer.initialize

        def initialize(**init):
            orig_init(**init)
            key = (str(init.get("nodeNamespace", "")).strip("/"), str(init.get("topicName", "")).strip("/"))
            rng = _depth_ranges.get(key)
            if rng is None:
                return
            try:
                annos = list(writer.annotators)
                idx = [i for i, a in enumerate(annos) if isinstance(a, str) and "IsaacPassthroughImagePtr" in a]
                if not idx:
                    log(f"depth range: no depth pointer annotator on /{key[0]}/{key[1]} ({annos}), unlimited")
                    return
                # registered by name: the helper deep-copies the writer, an Annotator object doesn't copy
                anno_name = f"fleet_depth_{rng[0]}_{rng[1]}"
                if anno_name not in registered:
                    aug = rep.annotators.Augmentation.from_function(depth_window, min_m=float(rng[0]),
                                                                    max_m=float(rng[1]))
                    rep.annotators.register(name=anno_name, annotator=rep.annotators.augment_compose(
                        source_annotator="distance_to_image_plane", augmentations=[aug]))
                    registered.add(anno_name)
                annos[idx[0]] = anno_name
                writer._annotators = annos
                log(f"depth range: /{key[0]}/{key[1]} limited to {rng[0]}-{rng[1]} m")
            except Exception:
                log(f"depth range: /{key[0]}/{key[1]} left unlimited:\n" + traceback.format_exc())

        writer.initialize = initialize
        return writer

    rep.writers.get = get


def enable_extensions(names):
    em = omni.kit.app.get_app().get_extension_manager()
    for n in names:
        em.set_extension_enabled_immediate(n, True)


def install_newton_parse_hook(newton):
    """Wrap newton.ModelBuilder.add_usd (NewtonStage parses the stage with it at the first physics step and after
    any USD change): add NEWTON_IGNORE_PATHS to its ignore_paths and log what the parse took."""
    original = newton.ModelBuilder.add_usd
    if getattr(original, "_fleet_hook", False):
        return

    def add_usd(self, *args, **kwargs):
        kwargs["ignore_paths"] = list(kwargs.get("ignore_paths") or []) + NEWTON_IGNORE_PATHS
        t0 = time.time()
        result = original(self, *args, **kwargs)
        for cb in NEWTON_PARSE_CALLBACKS:  # shapes the stage can't express (vbd_plant's flower hulls)
            try:
                cb(self)
            except Exception:
                log("newton parse callback failed:\n" + traceback.format_exc())
        log(f"newton parsed the stage in {time.time() - t0:.1f} s: {self.body_count} bodies, {self.shape_count} "
            f"shapes, {self.joint_count} joints (ignoring {' '.join(NEWTON_IGNORE_PATHS) or 'nothing'})")
        return result

    add_usd._fleet_hook = True
    newton.ModelBuilder.add_usd = add_usd


def select_physics_engine():
    """SIM_PHYSICS=newton: enable isaacsim.physics.newton (its auto_switch_on_startup makes Newton the active engine
    of isaacsim.core.simulation_manager's unified physics interface) and check that the switch took. Anything else,
    or a failed switch (logged in full), runs PhysX. Returns the engine that runs. Before the stage exists: switching
    engines detaches and re-attaches the stage."""
    if PHYSICS_REQUESTED != "newton":
        if PHYSICS_REQUESTED != "physx":
            log(f"SIM_PHYSICS={PHYSICS_REQUESTED!r} is neither physx nor newton: running PhysX")
        return "physx"
    global PHYSICS_HZ, PHYSICS_FRAME_DT
    try:
        # .tensors: the C++ omni.physics.tensors backend named "Newton" (registers itself on load). Without it the
        # engine steps but no simulation view can be created: IsaacArticulationController, IsaacComputeOdometry,
        # the joint-state publisher and Articulation.get_velocities all fail ("Failed to find simulation backend").
        # omni.usd.schema.newton (6.2): the solver scene APIs switch_newton_solver applies (VBD's alias, setup_vbd)
        enable_extensions((["omni.usd.schema.newton"] if NEWTON_SOLVER == "vbd" else [])
                          + ["isaacsim.physics.newton", "isaacsim.physics.newton.tensors"])
        from isaacsim.core.simulation_manager import SimulationManager
        if SimulationManager.get_active_physics_engine() != "newton" and not SimulationManager.switch_physics_engine("newton"):
            raise RuntimeError(f"switch failed; engines: {SimulationManager.get_available_physics_engines()}")
        import newton
        from isaacsim.physics.newton import acquire_stage
        cfg = acquire_stage().cfg
        # Contacts: MuJoCo's own collision pipeline, not Newton's (the extension's default since 0.7.8). With
        # Newton's, a j100 at rest sat right but the first wheel rotation launched it metres into the air (the wheel
        # colliders are the importer's convex hulls): 2026-10-19, see sim/CLAUDE.md "Physics engine".
        if NEWTON_SOLVER == "vbd":
            setup_vbd(cfg)
        elif getattr(cfg.solver_cfg, "solver_type", "") == "mujoco":
            cfg.solver_cfg.use_mujoco_contacts = True
        PHYSICS_HZ = NEWTON_PHYSICS_HZ
        PHYSICS_FRAME_DT = max(1, int(PHYSICS_HZ // SIM_RATE_HZ)) / PHYSICS_HZ
        install_newton_parse_hook(newton)
        log(f"physics: Newton {newton.__version__} ({getattr(cfg.solver_cfg, 'solver_type', NEWTON_SOLVER)} solver"
            f"{', integrator ' + cfg.solver_cfg.integrator + ', mujoco contacts' if getattr(cfg.solver_cfg, 'solver_type', '') == 'mujoco' else ''}, CUDA graph "
            f"{'on' if cfg.use_cuda_graph else 'off'}, {PHYSICS_HZ} Hz (NEWTON_PHYSICS_HZ) from the physics scene). PhysX-only tuning is "
            f"skipped (wheel CCD, articulation iterations, PhysX material combine modes); the IMU is computed from "
            f"the root body's velocity (IMU_NEWTON_SCRIPT); lidars cast with Newton's raytracer (fleet_newton)")
        return "newton"
    except Exception:
        log("SIM_PHYSICS=newton: Newton could not be activated, running PhysX instead:\n" + traceback.format_exc())
        return "physx"


PHYSICS_SOLVER = os.environ.get("PHYSICS_SOLVER", "PGS")


# Newton VBD: contact stiffness / damping of each robot's wheel colliders, {wheel body path: (ke, kd)}, from
# model_params `vbd_wheel_contact` at spawn (setup_vbd applies it after every parse).
VBD_WHEEL_CONTACT = {}


# Newton VBD ignores joint effort limits (SolverVBD docs: joint_effort_limit "not supported"), so an arm joint's
# position drive pushed with stiffness x its lag, without bound: held back by stems hooked in the closed gripper, joint
# 2 (1500 N*m/rad) could push 150 N*m at 0.1 rad behind, against its 42 N*m maxForce, and the reaction rocked the
# robot onto its front wheels (vbd_cut_stem_v3h, 2026-11-10). On: each arm joint's drive target is held within
# effort / stiffness of the joint's angle after every step (a motor that stalls at its effort limit, like the cutter's
# finger and blade), and every 0.25 s a capped joint is logged with the torque it asked for.
VBD_ARM_EFFORT_CAP = True
# False: only the drive's stiffness part is held to the effort, as until vbd_cut_stem_v3l: the damping part alone
# then braked joint 2 with 123 N*m (limit 42) when two torn stems let the arm spring free (a VBD scene's arm_cap)
VBD_ARM_CAP_DAMPING = True
VBD_WHEEL_HOLD = True  # setup_vbd's wheel_hold; False: the wheels are pure dampers, as until vbd_cut_stem_v3o
VBD_CONTACT_MAX = 500_000  # rigid contacts the collision pipeline can hold (the plant's first steps need ~454 k)


VBD_CFG = None  # the stage config setup_vbd configured


def setup_vbd(cfg):
    """NEWTON_SOLVER=vbd (Isaac Sim 6.2): the extension steps SolverVBD, as the study's newton_rod_kit_tests.py does.
    6.2's newton_usd_schemas has no NewtonVbdSceneAPI although the extension's table names it: "vbd" is aliased onto
    NewtonXpbdSceneAPI (xpbd parked on another existing API, not removed: the simulation manager indexes the table by
    every solver name), and the simulation manager's VBD scene object is kept from applying the missing schema (a
    traceback at every play otherwise). One solver per process. setup_physics applies the alias to the scene prim.
    Contacts as the study's grasp: soft (penalty) rigid contacts, a 256-contact list per body (the G1's 41 hulls
    overflowed 64), VBD's friction regularisation 0.01 m/s."""
    from isaacsim.core.simulation_manager.impl import vbd_scene
    from isaacsim.physics.newton.impl import utils as newton_utils
    from isaacsim.physics.newton.impl.solver_config import VBDSolverConfig

    newton_utils.newton_solver_to_api_schema["xpbd"] = "NewtonKaminoSceneAPI"
    newton_utils.newton_solver_to_api_schema["vbd"] = "NewtonXpbdSceneAPI"
    vbd_scene.NewtonVbdScene.__init__ = lambda self, prim: vbd_scene.PhysicsScene.__init__(self, prim)
    global VBD_CFG
    VBD_CFG = cfg  # load_vbd_scene sets the scene's iterations on it (the solver is built at the first play)
    cfg.solver_cfg = VBDSolverConfig(iterations=10)
    cfg.solver_cfg.rigid_contact_hard = False  # not config fields: _get_solver forwards the config's __dict__
    # Contact capacity: the plant's first steps (before vbd_plant's 2 mm gap and lower-stem skip apply) had 453 k
    # candidate contacts against the extension's 84 k, and one body 1106 against VBD's 256 per body (both dropped the
    # rest, 2026-11-09); sized for that so nothing is dropped whatever the robot does in the plant
    cfg.solver_cfg.rigid_body_contact_buffer_size = 2048
    cc = getattr(cfg, "collision_cfg", None)
    if cc is not None:
        cc.rigid_contact_max = VBD_CONTACT_MAX
    log(f"VBD: stage config {type(cfg).__name__} id {id(cfg)}, collision_cfg {type(cc).__name__} "
        f"rigid_contact_max {getattr(cc, 'rigid_contact_max', None)}")
    # m/s: below this slip speed VBD's friction ramps linearly to Coulomb's. At 0.01 a cut piece held in the
    # foam pads at 4-5 N crept out at mm/s and fell within 1 s (vbd_cut_stem11, piece 36): a squeeze that pushes
    # the stem sideways at all lets it slide at ~eps x that push / (mu N)
    # Not lower: at 1e-4 the uncut stems the closed gripper holds could no longer slip through it, and pulling out
    # of the plant at the far patch the arm tipped the robot over (vbd_cut_stem18)
    cfg.solver_cfg.friction_epsilon = 0.001
    # VBD moves bodies (maximal coordinates) and never writes the state's joint coordinates: the joint-state
    # publisher read 0.0 for every joint while the arm moved (2026-11-08). They are recovered from the body poses
    # after every step (one kernel), so joint_states, MoveIt and moveit_sim_bridge see the real angles.
    import newton
    from isaacsim.physics.newton import acquire_stage

    ns = acquire_stage()

    def joint_coords_from_bodies(dt):
        if ns.model is not None and ns.state_0 is not None and ns.model.joint_count:
            newton.eval_ik(ns.model, ns.state_0, ns.state_0.joint_q, ns.state_0.joint_qd)

    ns.physics_callbacks.append(joint_coords_from_bodies)

    # A VBD contact's stiffness is the mean of its two shapes' ke (Newton's default 2.5e3 N/m) and its damping kd
    # acts only while it compresses: on four such tyres j100_0921 (22 kg) rocked at 2-3 Hz, +-10 px in the video,
    # whenever the arm pushed into the plant or pulled out of it (2026-11-08, runs vbd_cut_stem14-15)
    done = {"model": None}

    def wheel_contacts(dt):
        m = ns.model
        if m is None or m is done["model"] or not VBD_WHEEL_CONTACT:
            return
        done["model"] = m
        labels = [str(v) for v in (getattr(m, "shape_label", None) or m.shape_key)]
        ke, kd = m.shape_material_ke.numpy(), m.shape_material_kd.numpy()
        n = 0
        for sh, lab in enumerate(labels):
            hit = next((v for body, v in VBD_WHEEL_CONTACT.items() if lab.startswith(body + "/")), None)
            if hit:
                ke[sh], kd[sh] = hit
                n += 1
        if n:
            m.shape_material_ke.assign(ke)
            m.shape_material_kd.assign(kd)
            ns.solver.notify_model_changed(newton.ModelFlags.SHAPE_PROPERTIES)
        log(f"VBD: wheel contact ke/kd on {n} wheel collider shape(s) of {len(VBD_WHEEL_CONTACT)} wheel(s)")

    ns.physics_callbacks.append(wheel_contacts)

    import numpy as np

    cap = {"model": None, "t": 0.0, "logged": -1e9, "peak": {}}

    def arm_effort_cap(dt):
        """VBD_ARM_EFFORT_CAP: hold each arm joint's drive target within effort / stiffness of its angle."""
        m = ns.model
        if m is None or ns.control is None or getattr(ns.control, "joint_target_q", None) is None:
            return
        cap["t"] += dt
        if m is not cap["model"]:
            cap["model"] = m
            labels = [str(v) for v in m.joint_label]
            qs, qds = m.joint_q_start.numpy(), m.joint_qd_start.numpy()
            ke, kd, eff = m.joint_target_ke.numpy(), m.joint_target_kd.numpy(), m.joint_effort_limit.numpy()
            rows = []
            for j, lab in enumerate(labels):
                hit = re.search(r"/(arm_0_joint_(\d+))$", lab)
                if hit and ke[int(qds[j])] > 0 and 0 < eff[int(qds[j])] < 1e6:
                    d = int(qds[j])
                    rows.append((hit.group(2), int(qs[j]), float(ke[d]), float(eff[d]), d, float(kd[d])))
            cap["rows"] = rows
            cap["q"] = np.array([r[1] for r in rows], dtype=np.int64)
            cap["dof"] = np.array([r[4] for r in rows], dtype=np.int64)
            cap["ke"] = np.array([r[2] for r in rows])
            cap["kd"] = np.array([r[5] for r in rows])
            cap["eff"] = np.array([r[3] for r in rows])
            cap["cmd"] = cap["cmd_qd"] = None
            cap["written"] = cap["written_qd"] = None
            log("VBD: arm effort cap " + ("on" if VBD_ARM_EFFORT_CAP else "OFF (logging only)") + (
                " (stiffness and damping together): " if VBD_ARM_CAP_DAMPING else " (the stiffness part only): ") + ", ".join(f"j{r[0]} {r[3]:.0f} N*m (stiffness {r[2]:.0f} N*m/rad, damping "
                                                  f"{r[5]:.0f} N*m*s/rad)" for r in rows))
        if not len(cap.get("q", ())):
            return
        tq = ns.control.joint_target_q.numpy()
        tqd_all = ns.control.joint_target_qd.numpy() if getattr(ns.control, "joint_target_qd", None) is not None else None
        tgt = tq[cap["q"]]
        tgt_qd = tqd_all[cap["dof"]] if tqd_all is not None else np.zeros(len(tgt))
        if cap["cmd"] is None or cap["written"] is None or np.any(np.abs(tgt - cap["written"]) > 1e-7):
            cap["cmd"] = tgt.copy()  # the controller wrote a new command since the last step
        if cap["cmd_qd"] is None or cap["written_qd"] is None or np.any(np.abs(tgt_qd - cap["written_qd"]) > 1e-7):
            cap["cmd_qd"] = tgt_qd.copy()
        q = ns.state_0.joint_q.numpy()[cap["q"]]
        qd = ns.state_0.joint_qd.numpy()[cap["dof"]]
        # VBD's drive: ke * (target - q) + kd * (target_qd - qd) (rigid_vbd_kernels._evaluate_drive_limit_axis); the
        # damping part alone braked j2 with 123 N*m (limit 42) when two torn stems let the arm spring free, and the
        # reaction made the chassis hop (vbd_cut_stem_v3l, 2026-11-11): the whole torque is capped, not just the spring
        want = cap["cmd"] - q
        tau = cap["ke"] * want + (cap["kd"] * (cap["cmd_qd"] - qd) if VBD_ARM_CAP_DAMPING else 0.0)
        over = np.abs(tau) > cap["eff"] * 1.001
        for i in np.nonzero(over)[0]:
            cap["peak"][cap["rows"][i][0]] = max(cap["peak"].get(cap["rows"][i][0], 0.0), float(abs(tau[i])))
        if over.any() and cap["t"] - cap["logged"] >= 0.25:
            cap["logged"] = cap["t"]
            log(f"VBD: arm drive over its effort at sim t {cap['t']:.2f} s: " + ", ".join(
                f"j{cap['rows'][i][0]} asks {abs(tau[i]):.0f} N*m (limit {cap['rows'][i][3]:.0f}, {want[i]:+.3f} rad "
                f"behind, {qd[i]:+.2f} rad/s)" for i in np.nonzero(over)[0]) + ("; capped" if VBD_ARM_EFFORT_CAP else "; NOT capped"))
        if not VBD_ARM_EFFORT_CAP:
            return
        # a joint over its effort: the drive gives exactly its effort toward the command, as a stalled motor (target
        # velocity = the joint's own, so the damper adds nothing at this speed); the others get the command as written
        new, new_qd = cap["cmd"].copy(), cap["cmd_qd"].copy()
        if over.any():
            sat = np.clip(tau, -cap["eff"], cap["eff"])
            new[over] = q[over] + sat[over] / cap["ke"][over]
            if VBD_ARM_CAP_DAMPING:
                new_qd[over] = qd[over]
        if np.any(np.abs(new - tgt) > 1e-7):
            tq[cap["q"]] = new
            ns.control.joint_target_q.assign(tq)
        if tqd_all is not None and np.any(np.abs(new_qd - tgt_qd) > 1e-7):
            tqd_all[cap["dof"]] = new_qd
            ns.control.joint_target_qd.assign(tqd_all)
        cap["written"], cap["written_qd"] = new, new_qd

    ns.physics_callbacks.append(arm_effort_cap)

    # Wheel hold (VBD): the wheels' drives are pure velocity dampers (target 0, wheel_damping N*m*s/rad) and
    # WHEEL_BRAKE is PhysX-only, so at zero command nothing held a wheel's angle: the stems in the gripper and the
    # basket pulled j100_0921 toward the plant with 35-55 N and it rolled back at 11-15 mm/s, into the plant
    # (vbd_cut_stem_v3n, 2026-11-11). A real base's motor loop holds zero speed. While every wheel's commanded
    # velocity is 0 the angles are latched and held through the velocity target (the damper then acts as
    # VBD_WHEEL_HOLD_KP x the angle error, minus kd x speed), the torque within the wheel's effort limit; a non-zero
    # command releases it at once.
    VBD_WHEEL_HOLD_KP = 2000.0  # N*m/rad per wheel
    hold = {"model": None}

    def wheel_hold(dt):
        m = ns.model
        if not VBD_WHEEL_HOLD:
            if hold["model"] is None:
                hold["model"] = False
                log("VBD: wheel hold OFF (the scene's wheel_hold): the wheels are pure velocity dampers")
            return
        if m is None or ns.control is None or getattr(ns.control, "joint_target_qd", None) is None:
            return
        if m is not hold["model"]:
            hold.update(model=m, cmd=None, written=None, q0=None)
            labels = [str(v) for v in m.joint_label]
            qs, qds = m.joint_q_start.numpy(), m.joint_qd_start.numpy()
            kd, eff = m.joint_target_kd.numpy(), m.joint_effort_limit.numpy()
            rows = [(int(qs[j]), int(qds[j])) for j, lab in enumerate(labels) if lab.endswith("_wheel_joint")
                    and kd[int(qds[j])] > 0]
            hold["q"] = np.array([r[0] for r in rows], dtype=np.int64)
            hold["dof"] = np.array([r[1] for r in rows], dtype=np.int64)
            hold["kd"] = np.array([kd[d] for _, d in rows])
            hold["eff"] = np.array([eff[d] if 0 < eff[d] < 1e6 else 20.0 for _, d in rows])
            log(f"VBD: wheel hold at zero command on {len(rows)} wheel(s): {VBD_WHEEL_HOLD_KP:.0f} N*m/rad, damping "
                f"{hold['kd'].tolist()} N*m*s/rad, within {hold['eff'].tolist()} N*m")
        if not len(hold["dof"]):
            return
        tqd = ns.control.joint_target_qd.numpy()
        tgt = tqd[hold["dof"]]
        if hold["cmd"] is None or hold["written"] is None or np.any(np.abs(tgt - hold["written"]) > 1e-9):
            hold["cmd"] = tgt.copy()  # the drive graph wrote a new command
        q = ns.state_0.joint_q.numpy()[hold["q"]]
        qd = ns.state_0.joint_qd.numpy()[hold["dof"]]
        if np.any(np.abs(hold["cmd"]) > 1e-6):
            hold["q0"] = None
            new = hold["cmd"]
        else:
            if hold["q0"] is None:
                hold["q0"] = q.copy()
            want = qd - VBD_WHEEL_HOLD_KP * (q - hold["q0"]) / hold["kd"]  # kd * (want - qd) = Kp * (q0 - q)
            new = np.clip(want, qd - hold["eff"] / hold["kd"], qd + hold["eff"] / hold["kd"])
        if np.any(np.abs(new - tgt) > 1e-9):
            tqd[hold["dof"]] = new
            ns.control.joint_target_qd.assign(tqd)
        hold["written"] = new.copy()

    ns.physics_callbacks.append(wheel_hold)

    # The parse rebuilds the stage's collision config (rigid_contact_max back to None, the extension's 84 k
    # estimate); every pipeline build goes through this method, so the capacity is set right before each
    stage_cls = type(ns)
    build_pipeline = stage_cls._build_collision_pipeline

    def build_with_capacity(self, model):
        cc = getattr(self.cfg, "collision_cfg", None)
        if cc is not None and (cc.rigid_contact_max or 0) < VBD_CONTACT_MAX:
            cc.rigid_contact_max = VBD_CONTACT_MAX
        return build_pipeline(self, model)

    stage_cls._build_collision_pipeline = build_with_capacity


def vbd_mass_floor(stage, root, frame_mass):
    """Newton VBD: a rigid body with no colliders of its own (child bodies' colliders don't count) and no positive
    mass gets frame_mass and a matching small inertia. fix_massless_bodies gives such a body a density when a
    collider lies anywhere below it; PhysX copes, Newton parses it as zero mass, and VBD makes a massless body
    static: j100_0921's base_link, arm_0_end_effector_link and ee_cutter_link (2026-11-08), the last two pinning the
    gripper while the arm moved. Returns how many bodies it set."""
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        mapi = UsdPhysics.MassAPI(prim) if prim.HasAPI(UsdPhysics.MassAPI) else None
        if mapi is not None and (mapi.GetMassAttr().Get() or 0) > 0:
            continue
        it = iter(Usd.PrimRange(prim))
        own = False
        for q in it:
            if q != prim and q.HasAPI(UsdPhysics.RigidBodyAPI):
                it.PruneChildren()
                continue
            if q.HasAPI(UsdPhysics.CollisionAPI):
                own = True
                break
        if own:
            continue
        mapi = UsdPhysics.MassAPI.Apply(prim)
        mapi.CreateMassAttr(float(frame_mass))
        mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(*(3 * [float(frame_mass) * 0.02 ** 2])))  # a 4 cm solid's
        n += 1
    return n


def fix_massless_bodies(stage, root, density, frame_mass):
    """Give every rigid body that has no mass authored by the URDF importer (a URDF link without <inertial>) a
    realistic one. PhysX otherwise computes it from the collider volume at 1000 kg/m^3 (solid water) and gives a
    collider-less frame link a default 1 kg: j100_0922 weighs 18.4 kg in its URDF and 75 kg in the sim (its
    top_mount_link mesh 29.8 kg, each GPS sphere 5.9 kg, 13 frame links 1 kg each), all of it high above a 26 cm
    wheelbase, and it tips over backwards at 0.2 m/s. Returns (bodies with density, frame bodies)."""
    n_dens = n_frame = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        mapi = UsdPhysics.MassAPI(prim) if prim.HasAPI(UsdPhysics.MassAPI) else None
        if mapi is not None and (mapi.GetMassAttr().Get() or 0) > 0:
            continue
        has_collider = any(
            q.HasAPI(UsdPhysics.CollisionAPI)
            for q in Usd.PrimRange(prim)
            if q == prim or not q.HasAPI(UsdPhysics.RigidBodyAPI)
        )
        mapi = UsdPhysics.MassAPI.Apply(prim)
        if has_collider:
            mapi.CreateDensityAttr(float(density))
            n_dens += 1
        else:
            mapi.CreateMassAttr(float(frame_mass))
            n_frame += 1
    return n_dens, n_frame


# Newton (MuJoCo solver): a wheel's velocity drive (damping 1000 N*m*s/rad, see IMPORT_SETTINGS) is an actuator whose
# torque is bounded only by the drive's maxForce; unbounded, a tyre momentarily held by a soft contact gets hundreds
# of N*m and the chassis is catapulted (j100 at 0.2 m/s: 2 m/s and tipped, 2026-10-20). PhysX bounds the same drive
# through its rigid contacts and joint solver. A real Jackal/Husky motor delivers a few tens of N*m. 20 suits the
# Jackal; a heavier robot needs more or it can't turn (a200_0284 at 20: 0.83 of 1 rad/s in place, a 0.5/0.5 arc at
# 0.3 of its speed; at 60 within 4 %): model_params.yaml's `wheel_max_force` (per model, under `newton:`) overrides it.
NEWTON_WHEEL_MAX_FORCE = float(os.environ.get("NEWTON_WHEEL_MAX_FORCE", "20.0"))


def cap_wheel_drives(stage, root, max_force, damping=None):
    """DriveAPI maxForce on every wheel joint's angular drive, and its damping (N*m*s/rad) if given: USD angular drive
    gains are per degree and Newton scales them by 57.3, like the arm's (NEWTON_ARM_GAINS). Returns the number of
    joints."""
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if "wheel" in prim.GetName().lower() and prim.IsA(UsdPhysics.Joint):
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drive:
                drive.CreateMaxForceAttr().Set(float(max_force))
                if damping is not None:
                    drive.GetDampingAttr().Set(float(damping) * math.pi / 180.0)
                n += 1
    return n


def enable_wheel_ccd(stage, root):
    """CCD on every wheel rigid body (see the solver comment in build_world)."""
    from pxr import PhysxSchema

    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if "wheel" in prim.GetName().lower() and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateEnableCCDAttr(True)


def apply_wheel_friction(stage, root, ns, spec):
    """A physics material on every wheel rigid body (its colliders inherit it), from model_params `wheel_friction`:
    {static, dynamic, combine (average|min|multiply|max, default average), improve_patch (bool)}. Without it the
    wheels have no material: PhysX's default (0.5/0.5), averaged with the ground's (1.0/0.9). PhysX combines two
    materials with the mode of higher priority (average < min < multiply < max), so a wheel `min` wins over the
    ground's default `average`. Returns the number of wheels."""
    from pxr import PhysxSchema

    mat = UsdShade.Material.Define(stage, f"/World/Materials/{ns}_wheel_physics")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(float(spec["static"]))
    pm.CreateDynamicFrictionAttr(float(spec.get("dynamic", spec["static"])))
    pm.CreateRestitutionAttr(0.0)
    px = PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim())
    px.CreateFrictionCombineModeAttr(spec.get("combine", "average"))
    if "improve_patch" in spec:
        px.CreateImprovePatchFrictionAttr(bool(spec["improve_patch"]))
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if "wheel" in prim.GetName().lower() and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.strongerThanDescendants, "physics")
            n += 1
            if PHYSICS_ENGINE == "newton":
                # Newton reads only a collider's own physics binding (the body's above left the wheels at its
                # default mu 1.0), and MuJoCo combines two geoms' friction by taking the larger one unless a geom
                # has a higher priority: bind on each collider and give it priority 1, so the wheel's value is
                # the contact's, as PhysX's `min` combine makes it (2026-10-20: wheels 1.0 against the ground's 0.9
                # made skid-steer turning stick-slip).
                for col in Usd.PrimRange(prim):
                    if col.HasAPI(UsdPhysics.CollisionAPI) and not col.IsInstanceProxy():
                        UsdShade.MaterialBindingAPI.Apply(col).Bind(mat, UsdShade.Tokens.strongerThanDescendants,
                                                                    "physics")
                        col.CreateAttribute("mjc:priority", Sdf.ValueTypeNames.Int).Set(1)
    return n


# Articulation solver iterations (PhysxArticulationAPI on each robot's articulation root; 0 = PhysX's default,
# 32 position / 1 velocity). With 1 velocity iteration the resting robots jittered (a300_00036, wheels braked: IMU z
# std 0.44 m/s^2, gripper fingertips at 0.2 rad/s RMS, 0.38 deg/min of yaw creep from contact micro-slip); 8 velocity
# iterations: 0.31 m/s^2, 0.01 rad/s, 0.06 deg/min, at the same frame rate. 64 position iterations added nothing
# measurable and cost ~15% fps. Override with the env vars (pass them to the isaac-sim service).
ARTIC_POS_ITERS = int(os.environ.get("ARTIC_POS_ITERS", "0"))
ARTIC_VEL_ITERS = int(os.environ.get("ARTIC_VEL_ITERS", "8"))


def set_articulation_iterations(stage, root):
    from pxr import PhysxSchema

    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            if ARTIC_POS_ITERS:
                api.CreateSolverPositionIterationCountAttr(ARTIC_POS_ITERS)
            if ARTIC_VEL_ITERS:
                api.CreateSolverVelocityIterationCountAttr(ARTIC_VEL_ITERS)
            n += 1
    return n


IMPORT_SETTINGS = {
    "merge_fixed_joints": os.environ.get("FLEET_MERGE_FIXED", "0") == "1",
    "merge_mesh": False,
    "collision_from_visuals": False,
    "joint_target_type": "velocity",
    "joint_drive_type": "force",
    "override_joint_stiffness": 0.0,
    "override_joint_damping": 1000.0,
    # The resting creep once noted here (wheels slowly rolling with Diff's targets at exactly 0; damping 10x/100x
    # made no difference) is handled by the wheel parking brake (WHEEL_BRAKE_SCRIPT: a velocity damper never holds a
    # position) plus 8 articulation velocity iterations (ARTIC_VEL_ITERS) against the contact jitter.

    # The URDF root (base_link) has no inertia, so the generated chassis joint would pin the chassis to the
    # world. Floating base = the robot is free to drive.
    "fix_base": False,
}


def import_stamp(model):
    """(stamp the cached import should have, stamp it has or None): import settings + URDF mtime/size."""
    assets = MODEL_ASSETS[model]
    st = os.stat(assets["urdf"])
    stamp = json.dumps({"settings": IMPORT_SETTINGS, "urdf": [st.st_mtime_ns, st.st_size]}, sort_keys=True)
    try:
        cached = open(f"{assets['usd_dir']}/.import_stamp").read()
    except OSError:
        cached = None
    return stamp, cached


def import_urdf_if_needed(model):
    """Convert the model's URDF to USD unless the cached import is current. Only before the scene is built: the
    importer opens its result in the USD context (stage_utils.open_stage), which would replace an open scene."""
    assets = MODEL_ASSETS[model]
    urdf_path, usd_dir, usd_path = assets["urdf"], assets["usd_dir"], assets["usd_path"]
    stamp, cached = import_stamp(model)
    stamp_path = f"{usd_dir}/.import_stamp"
    if os.path.exists(usd_path) and cached == stamp and not FORCE_REIMPORT:
        log(f"using cached USD {usd_path}")
        return
    log(f"converting {model} URDF -> USD (cached afterwards)")
    import shutil

    shutil.rmtree(usd_dir, ignore_errors=True)  # the importer would otherwise write to <model>_1/, <model>_2/, ...
    os.makedirs(usd_dir, exist_ok=True)
    enable_extensions(["omni.scene.optimizer.core", "isaacsim.robot.schema", "isaacsim.asset.importer.urdf"])
    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig

    cfg = URDFImporterConfig()
    cfg.urdf_path = urdf_path
    cfg.usd_path = usd_dir
    for key, value in IMPORT_SETTINGS.items():
        setattr(cfg, key, value)
    out = URDFImporter(cfg).import_urdf()
    if out != usd_path:
        log(f"WARNING: importer wrote {out}, expected {usd_path}")
    with open(stamp_path, "w") as f:
        f.write(stamp)


# Vegetation colliders. The lidars are PhysX scene-query raycasts, which only hit prims with a collider,
# and the plant/tree/rock assets come without any, so the lidars saw straight through them. collider_asset() writes a
# small wrapper layer per asset into /sim/generated/colliders/ that references the asset and gives each of its meshes
# a static, exact triangle-mesh collider (approximation "none"; static colliders don't need convex shapes, and a hull
# around e.g. the oak's limb mesh would be a 20 m wall). The scene references the wrapper instead of the asset, so the
# visuals and instancing (lavender) are unchanged and PhysX cooks each distinct mesh once. Meshes that are
# PointInstancer prototypes (the trees' and shrubs' leaves/twigs) can't be colliders and stay invisible to the lidar.
# The lavender's colliders don't block the robots (soften_lavender); everything else is solid for them too.
VEGETATION_COLLIDERS = True
_COLLIDER_DIR = "/sim/generated/colliders"


def collider_asset(asset):
    if not VEGETATION_COLLIDERS:
        return asset
    name = os.path.splitext(os.path.basename(asset))[0]
    out = f"{_COLLIDER_DIR}/{name}.usda"
    stamp = f"{asset} {os.path.getmtime(asset)} v1"
    try:
        if Sdf.Layer.FindOrOpen(out).customLayerData.get("collider_stamp") == stamp:
            return out
    except Exception:
        pass
    os.makedirs(_COLLIDER_DIR, exist_ok=True)
    src = Usd.Stage.Open(asset)
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(st, UsdGeom.GetStageUpAxis(src))
    UsdGeom.SetStageMetersPerUnit(st, UsdGeom.GetStageMetersPerUnit(src))
    root = st.DefinePrim("/Root", "Xform")
    root.GetReferences().AddReference(asset)
    st.SetDefaultPrim(root)
    # Opinions below an instanceable prim are ignored, so un-instance any inside the asset (the rocks have one).
    while True:
        inst = [q for q in st.Traverse() if q.IsInstanceable()]
        if not inst:
            break
        for q in inst:
            q.SetInstanceable(False)
    n = 0
    it = iter(Usd.PrimRange(root))
    for q in it:
        if q.IsA(UsdGeom.PointInstancer):
            it.PruneChildren()
        elif q.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(q)
            UsdPhysics.MeshCollisionAPI.Apply(q).CreateApproximationAttr(UsdPhysics.Tokens.none)
            n += 1
    st.GetRootLayer().customLayerData = {"collider_stamp": stamp}
    st.GetRootLayer().Export(out)
    log(f"collider wrapper {out}: {n} mesh colliders")
    return out


def add_lavender(stage, path, pos, rot_z=0.0):
    """One lavender clump (~1.2 m wide, ~0.75 m tall at LAVENDER_SCALE) from LAVENDER_USD, with colliders (see
    collider_asset). instanceable=True shares the (heavy) mesh data, BVH and cooked collider between the copies."""
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(collider_asset(LAVENDER_USD))
    prim.SetInstanceable(True)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(pos[0], pos[1], pos[2] + LAVENDER_BASE_Z))
    xf.AddRotateZOp().Set(rot_z)
    xf.AddScaleOp().Set(Gf.Vec3d(LAVENDER_SCALE, LAVENDER_SCALE, LAVENDER_SCALE))
    return prim


def soften_lavender(stage):
    """Lavender stems are thin and flexible: a robot brushing a plant pushes through it, only the woody crown at
    its centre stops it. Here every collider under /World/lavender (any scene source) goes into SOFT_PLANTS_GROUP,
    filtered against ROBOTS_GROUP (spawn_robot adds each robot), so robots pass through the foliage while the lidars'
    scene-query raycasts still hit it -- collision groups only filter contacts. Each plant (an instanceable prim
    under /World/lavender) gets an invisible rigid cylinder at its bbox centre, /World/lavender_cores/..., outside
    the group. Measured in a headless spike (Isaac 6.0): an inverted group with no filteredGroups ("collide with
    nothing") is ignored, the plants still blocked; the explicit plants/robots pair lets a box through at full speed,
    instanced plants included, and raycasts hit the same foliage as before."""
    if not LAVENDER_SOFT or not stage.GetPrimAtPath("/World/lavender"):
        return
    plants_g = UsdPhysics.CollisionGroup.Define(stage, SOFT_PLANTS_GROUP)
    robots_g = UsdPhysics.CollisionGroup.Define(stage, ROBOTS_GROUP)
    plants_g.CreateFilteredGroupsRel().SetTargets([robots_g.GetPath()])
    robots_g.CreateFilteredGroupsRel().SetTargets([plants_g.GetPath()])
    plants_g.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets([Sdf.Path("/World/lavender")])
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    xfc = UsdGeom.XformCache()
    UsdGeom.Scope.Define(stage, "/World/lavender_cores")
    n = 0
    it = iter(Usd.PrimRange(stage.GetPrimAtPath("/World/lavender")))
    for prim in it:
        # a plant: an instance, or a farm plant's Xform (object_id; its invisible harvested model adds no bounds)
        if not prim.IsInstance() and prim.GetCustomDataByKey("object_id") is None:
            continue
        it.PruneChildren()
        mid = xfc.GetLocalToWorldTransform(prim).Transform(bbox.ComputeUntransformedBound(prim).ComputeCentroid())
        rel = prim.GetPath().MakeRelativePath("/World/lavender")
        core = UsdGeom.Cylinder.Define(stage, Sdf.Path("/World/lavender_cores").AppendPath(rel))
        core.CreateAxisAttr(UsdGeom.Tokens.z)
        core.CreateRadiusAttr(LAVENDER_CORE_RADIUS)
        core.CreateHeightAttr(LAVENDER_CORE_HEIGHT)
        core.CreatePurposeAttr(UsdGeom.Tokens.guide)  # not rendered (cameras), still a collider (robots, lidars)
        core.ClearXformOpOrder()  # a saved scene's sublayer can already have this core (AddTranslateOp would raise)
        core.AddTranslateOp().Set(Gf.Vec3d(mid[0], mid[1], LAVENDER_CORE_HEIGHT / 2))
        UsdPhysics.CollisionAPI.Apply(core.GetPrim())
        n += 1
    log(f"soft lavender: robots pass through the foliage, {n} rigid {LAVENDER_CORE_RADIUS * 200:.0f} cm crown cores")


def add_ground_cover(stage, path="/World/GroundCover"):
    """One GROUND_COVER_USD patch (a 100 x 100 m grass field, ~73k PointInstancer blades of ~9-11 cm) over the
    80 x 80 m ground plane. The layer says metersPerUnit=0.01, but its geometry is really authored in metres
    (blade meshes measure 0.09-0.11 units), so it is referenced at GROUND_COVER_SCALE_XY/_Z (1.0 = unscaled).
    Visual only: no collider, so wheel traction still comes from /World/ground."""
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(GROUND_COVER_USD)
    UsdGeom.Xformable(prim).AddScaleOp().Set(
        Gf.Vec3d(GROUND_COVER_SCALE_XY, GROUND_COVER_SCALE_XY, GROUND_COVER_SCALE_Z)
    )
    return prim


def add_scatter(stage, root, usds, count, dist, height, seed, arc_deg=80.0):
    """Scatter `count` copies of the given USDs on an arc ahead of the robots at a random radius in `dist` and a
    random target height in `height` (m). Assets are Z-up and authored in cm, but referenced unscaled and sized
    from their own measured bbox, so the scale factor is just target_height / bbox_height."""
    rng = random.Random(seed)
    UsdGeom.Xform.Define(stage, root)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    sizes = {}
    for u in set(usds):
        st = Usd.Stage.Open(u)
        r = cache.ComputeWorldBound(st.GetDefaultPrim() or st.GetPseudoRoot()).ComputeAlignedRange()
        sizes[u] = r.GetSize()[2]
        log(f"scatter asset {u}: size={r.GetSize()} up={UsdGeom.GetStageUpAxis(st)}")
    for t in range(count):
        ang = math.radians(-arc_deg + 2 * arc_deg * t / (count - 1)) + rng.uniform(-0.03, 0.03)
        d = rng.uniform(*dist)
        u = usds[rng.randrange(len(usds))]
        k = rng.uniform(*height) / max(sizes[u], 1e-6)
        # The asset's own root may carry xform ops, so it gets its own child prim under the placement Xform.
        prim = stage.DefinePrim(f"{root}/item_{t}", "Xform")
        stage.DefinePrim(f"{root}/item_{t}/asset", "Xform").GetReferences().AddReference(collider_asset(u))
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(d * math.cos(ang), d * math.sin(ang), 0.0))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, rng.uniform(0, 360)))
        xf.AddScaleOp().Set(Gf.Vec3d(k, k, k))


def add_horizon_vegetation(stage):
    """Tree line (NVIDIA Omniverse Assets/Vegetation/Trees) with shrubs and boulders (Shrub, Rocks) in front of
    and between the trunks, so the horizon has no bare gap. All within the cameras' 30 m clipping range."""
    add_scatter(stage, "/World/trees", TREE_USDS, TREE_COUNT, (TREE_DIST, TREE_DIST + 5), TREE_HEIGHT, 7, 80.0)
    add_scatter(stage, "/World/shrubs", SHRUB_USDS, SHRUB_COUNT, (SHRUB_DIST, SHRUB_DIST + 8), SHRUB_HEIGHT, 11, 85.0)
    add_scatter(stage, "/World/rocks", ROCK_USDS, ROCK_COUNT, (ROCK_DIST, ROCK_DIST + 8), ROCK_HEIGHT, 13, 85.0)


def add_box(stage, path, size, pos, color):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3d(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def setup_physics(stage):
    """Physics scene and stage timing, the same for every scene source: the robots' drive and velocity loop were
    calibrated with these settings. A loaded scene file's own physics scene is reused (the settings are authored
    over it on the stage's root layer, so the file itself never changes); without one, /World/physicsScene."""
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    if not stage.GetPrimAtPath("/World"):
        UsdGeom.Xform.Define(stage, "/World")
    found = next((p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)), None)
    scene = UsdPhysics.Scene(found) if found else UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    from pxr import PhysxSchema

    scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    scene_api.CreateTimeStepsPerSecondAttr().Set(PHYSICS_HZ)
    # Skid-steer wheels need the PGS solver (TGS, the default, barely turns them in place: angular velocity far
    # below the wheel speeds; NVIDIA forum "Skid-Steered behavior for robots") plus CCD on the wheels (below).
    # Measured here: PGS at the same 60 Hz costs no frame rate, 360 Hz halves it.
    scene_api.CreateSolverTypeAttr().Set(PHYSICS_SOLVER)
    scene_api.CreateEnableCCDAttr().Set(True)
    # GPU broadphase buffers for pairs involving aggregates (each articulation is one): the default 1024 overflowed
    # once an arm reached into the dynamic lavender plant (stem model 3: one 25-45 cm body per stem), PhysX logged
    # "increase totalAggregatePairsCapacity" every step and stopped the simulation after 1000 errors (2026-10-31)
    scene_api.CreateGpuTotalAggregatePairsCapacityAttr().Set(16384)
    scene_api.CreateGpuFoundLostAggregatePairsCapacityAttr().Set(16384)
    if PHYSICS_ENGINE == "newton" and NEWTON_SOLVER == "vbd":  # the extension picks its solver by the scene's API
        from isaacsim.physics.newton.impl import utils as newton_utils
        newton_utils.switch_newton_solver("vbd", scene.GetPrim())
    stage.SetTimeCodesPerSecond(SIM_RATE_HZ)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(10_000_000)
    omni.timeline.get_timeline_interface().set_time_codes_per_second(SIM_RATE_HZ)
    return scene


def add_ground(stage, color=GROUND_COLOR, look="ground_grey"):
    """80 x 80 m ground box, top face at z=0 (SPAWN_Z and SPAWN_LIMIT assume that), with a grippy physics
    material so the skid-steer wheels get traction and a matte surface of `color` (RTX ignores displayColor: an
    unbound box renders near-white)."""
    mat = UsdShade.Material.Define(stage, "/World/Materials/ground_physics")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(1.0)
    pm.CreateDynamicFrictionAttr(0.9)
    pm.CreateRestitutionAttr(0.0)
    ground = add_box(stage, "/World/ground", (80, 80, 1), (0, 0, -0.5), color)
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    surface = UsdShade.Material.Define(stage, f"/World/Materials/{look}")
    shader = UsdShade.Shader.Define(stage, f"/World/Materials/{look}/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    surface.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(ground.GetPrim()).Bind(surface)
    return ground


def newton_ground_plane(stage):
    """SIM_PHYSICS=newton: the ground box's collider (/World/ground, top at z=0 in every scene) replaced by an infinite
    plane with the same physics material. MuJoCo has no exact cylinder-box contact (a general convex routine gives a
    few jumping contact points), but an exact cylinder-plane one: on the box the wheels' contacts jittered and the
    skid-steer yaw wobbled (2026-10-20). The box still renders. Returns the plane's height, or None."""
    ground = stage.GetPrimAtPath("/World/ground")
    if not ground or not ground.HasAPI(UsdPhysics.CollisionAPI):
        return None
    top = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.guide]).ComputeWorldBound(ground) \
        .ComputeAlignedRange().GetMax()[2]
    UsdPhysics.CollisionAPI(ground).CreateCollisionEnabledAttr(False)
    plane = UsdGeom.Plane.Define(stage, "/World/ground_plane")
    plane.CreateAxisAttr("Z")
    plane.CreateWidthAttr(200.0)
    plane.CreateLengthAttr(200.0)
    plane.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdGeom.Xformable(plane).AddTranslateOp().Set(Gf.Vec3d(0, 0, top))
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
    material, _ = UsdShade.MaterialBindingAPI(ground).ComputeBoundMaterial("physics")
    if material:
        UsdShade.MaterialBindingAPI.Apply(plane.GetPrim()).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
    return top


def add_lights(stage, sky_hdr=None, sky_intensity=DEFAULT_SKY_INTENSITY, sun_intensity=DEFAULT_SUN_INTENSITY):
    """Dome light (a plain sky colour, or an HDR panorama the cameras also see as background) and a sun."""
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/dome")
    dome.CreateIntensityAttr(sky_intensity)
    if sky_hdr:
        dome.CreateTextureFileAttr(sky_hdr)
        dome.CreateTextureFormatAttr("latlong")
    else:
        dome.CreateColorAttr(Gf.Vec3f(*DEFAULT_SKY_COLOR))
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/sun")
    sun.CreateIntensityAttr(sun_intensity)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-60, 33, -30))


def build_default_world(stage):
    """SIM_SCENE empty: ground plane and lights, nothing else."""
    setup_physics(stage)
    add_ground(stage)
    add_lights(stage)
    return {"lavender_rows": []}


def build_lavender_world(stage):
    """SIM_SCENE=lavender: the lavender farm, modelled on real photos."""
    setup_physics(stage)
    # Dirt-coloured surface, so what shows through the grass is soil, not a bright grey box.
    add_ground(stage, GROUND_SOIL_COLOR, "ground_soil")

    add_ground_cover(stage)
    add_horizon_vegetation(stage)
    add_lights(stage, SKY_HDR, SKY_INTENSITY, 10000)  # cloudy sky panorama

    # Lavender farm: a hedge row of overlapping plants on each side of the fleet's driving lanes (robots drive
    # along +X), like the real field's rows. Each plant is ~1.26M triangles, so the row length is what costs fps.
    # The scene is built before any robot exists, so the rows are placed for SCENE_LANES lanes, not a robot count.
    half = ((SCENE_LANES - 1) / 2) * ROBOT_SPACING
    k = 0
    rows = []
    for side in (-1, 1):
        y = side * (half + LAVENDER_ROW_OFFSET)
        xs = [LAVENDER_ROW_X0 + LAVENDER_PLANT_PITCH * j for j in range(LAVENDER_ROW_PLANTS)]
        for x in xs:
            add_lavender(stage, f"/World/lavender/plant_{k}", (x, y, 0.0), rot_z=(k * 47.0) % 360)
            k += 1
        rows.append([xs[0] - 0.6, xs[-1] + 0.6, y])  # plants are ~1.2 m wide
    return {"lavender_rows": rows}  # for the web UI's spawn map


_scene_layers = []  # the loaded scene file's layer stack, held so the in-memory removals below stick


def scene_file_path(name):
    """SIM_SCENE -> the scene file, which must be inside SCENE_DIR (sim/scene/ on the host)."""
    path = os.path.realpath(os.path.join(SCENE_DIR, name))
    if not path.startswith(os.path.realpath(SCENE_DIR) + os.sep):
        raise ValueError(f"SIM_SCENE={name!r}: the scene file must be inside sim/scene/")
    if not path.lower().endswith(SCENE_FILE_EXTS) or not os.path.isfile(path):
        raise ValueError(f"SIM_SCENE={name!r}: no USD file {path} (expected one of {', '.join(SCENE_FILE_EXTS)})")
    return path


def build_file_world(stage, name):
    """SIM_SCENE=<file in sim/scene/>: a scene saved from Isaac Sim (File > Save As), used as a sublayer of the
    stage, so its relative asset paths resolve and nothing the sim does is ever written into it. What the fleet
    needs is added on the root layer: the physics settings (setup_physics), and the default ground / lights when
    the file has no collider / no light at all. Its ground must be at z=0 (robots spawn at SPAWN_Z) within
    +-SPAWN_LIMIT m. What the sim adds itself and was saved along with the scene -- robots, their graphs,
    soften_lavender's cores and collision groups, the farm workers -- is dropped (from the in-memory copy of the
    file only), since the sim adds its own; in every layer of the file, as a file saved from a sim that ran a saved
    file has the older one, with all of that, as its sublayer."""
    global _scene_layers
    path = scene_file_path(name)
    layer = Sdf.Layer.FindOrOpen(path)
    if layer is None:
        raise ValueError(f"could not open scene file {path}")
    generic = [m for m in MODEL_ASSETS if "_" not in m]
    robot_re = re.compile(r"(%s)_\d{4}" % "|".join(map(re.escape, generic))) if generic else None
    sim_owned = {"lavender_cores", "collisionGroups", Sdf.Path(farm_workers.ROOT).name}  # rebuilt from the settings
    _scene_layers = scene_layer_stack(layer)
    for lyr in _scene_layers:
        stale = []
        world = lyr.GetPrimAtPath("/World")
        for child in (world.nameChildren if world else []):
            if child.name in MODEL_ASSETS or (robot_re and robot_re.fullmatch(child.name)) or child.name in sim_owned:
                stale.append(child.path)
        if lyr.GetPrimAtPath("/Graphs"):
            stale.append(Sdf.Path("/Graphs"))
        if not stale:
            continue
        edit = Sdf.BatchNamespaceEdit()
        for p in stale:
            edit.Add(p, Sdf.Path.emptyPath)
        where = os.path.relpath(lyr.realPath, SCENE_DIR) if lyr.realPath else lyr.identifier
        if lyr.Apply(edit):
            log(f"scene {name}: left out what was saved from a previous fleet ({where}): {', '.join(map(str, stale))}")
        else:
            log(f"scene {name}: could not leave out {', '.join(map(str, stale))} ({where})")
    for key, want in (("upAxis", "Z"), ("metersPerUnit", 1.0)):
        info = layer.pseudoRoot
        if info.HasInfo(key) and info.GetInfo(key) != want:
            log(f"WARNING scene {name}: {key} is {info.GetInfo(key)}, the fleet's stage uses {want}; "
                "its contents will look rotated/scaled")
    stage.GetRootLayer().subLayerPaths.append(path)

    setup_physics(stage)
    prims = list(stage.Traverse(Usd.TraverseInstanceProxies()))
    if not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in prims):
        log(f"scene {name}: no collider in it, adding the default ground")
        add_ground(stage)
    if not any(p.HasAPI(UsdLux.LightAPI) for p in prims):
        log(f"scene {name}: no light in it, adding the default lights")
        add_lights(stage)
    # rows recorded by scripts/make_farm_scene.py ([x_min, x_max, y, width]), for the web UI's spawn map: from the
    # topmost layer that has them (a Save As writes only render settings into its own customLayerData)
    rows = next((lyr.customLayerData["lavender_rows"] for lyr in _scene_layers
                 if lyr.customLayerData.get("lavender_rows")), [])
    # charging stations recorded by scripts/make_charger_scene.py (world frame): robot/bin/battery_sim charges a
    # robot that stands at one
    stations = next((lyr.customLayerData["charging_stations"] for lyr in _scene_layers
                     if lyr.customLayerData.get("charging_stations")), {})
    chargers = [dict({"id": k, "x": float(c["x"]), "y": float(c["y"]), "yaw_deg": float(c.get("yaw_deg", 0.0))},
                     **({"dock_id": str(c["dock_id"])} if c.get("dock_id") else {}))
                for k, c in sorted(stations.items())]  # + their tags (charger_tag) once the scene has loaded
    return {"lavender_rows": [[float(v) for v in r] for r in rows], "chargers": chargers}


def charger_tag(stage, key):
    """The AprilTag of /World/charging_stations/<key> (scripts/make_charger_scene.py): its centre in the world frame
    and id, for the robots' dock poses (mtu32_bringup's charger docks); {} if the scene has no such prim."""
    root = stage.GetPrimAtPath(f"/World/charging_stations/{key}")
    # (its material is named AprilTag too)
    tag = next((p for p in Usd.PrimRange(root) if p.GetName() == "AprilTag" and p.IsA(UsdGeom.Xformable)),
               None) if root else None
    if tag is None:
        log(f"WARNING charger {key}: no AprilTag prim under /World/charging_stations/{key}")
        return {}
    # its mesh's points, centred, in the world frame (Kit's BBoxCache gives an empty box for it)
    mesh = next((p for p in Usd.PrimRange(tag) if p.IsA(UsdGeom.Mesh)), None)
    points = UsdGeom.Mesh(mesh).GetPointsAttr().Get() if mesh else None
    if not points:
        log(f"WARNING charger {key}: its AprilTag {tag.GetPath()} has no mesh points")
        return {}
    local = sum((Gf.Vec3d(p) for p in points), Gf.Vec3d(0, 0, 0)) / len(points)
    mid = UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).Transform(local)
    tag_id = root.GetCustomDataByKey("apriltag_id")
    dock_id = root.GetCustomDataByKey("dock_id")  # the dock's name in Nav2 (dock_station.dock_id), if the DB has one
    return {"tag": [round(float(v), 4) for v in mid], **({"apriltag_id": int(tag_id)} if tag_id is not None else {}),
            **({"dock_id": str(dock_id)} if dock_id else {})}


def scene_layer_stack(layer):
    """`layer` and every layer below it (its sublayers, theirs, ...), top first, each once."""
    stack, todo = [], [layer]
    while todo:
        lyr = todo.pop(0)
        if any(lyr is s or lyr.identifier == s.identifier for s in stack):
            continue
        stack.append(lyr)
        for sub in lyr.subLayerPaths:
            found = Sdf.Layer.FindOrOpen(lyr.ComputeAbsolutePath(sub))
            if found is None:
                log(f"WARNING scene: sublayer {sub} of {lyr.identifier} not found")
            else:
                todo.append(found)
    return stack


def load_vbd_scene():
    """VBD_SCENE: the scene's parameters onto vbd_plant, ee_cutter and this module's VBD switches, before the world
    and any robot are built. A scene that can't be read leaves no plant (a run on other values than asked for would
    pass for the scene's); state.json's vbd_scene says which scene runs, with every parameter."""
    global VBD_PLANT, VBD_PLANT_ASSET, VBD_ARM_EFFORT_CAP, VBD_ARM_CAP_DAMPING, VBD_WHEEL_HOLD
    if not VBD_SCENE:
        return
    if not (PHYSICS_ENGINE == "newton" and NEWTON_SOLVER == "vbd"):
        log(f"VBD_SCENE {VBD_SCENE}: ignored (it needs SIM_PHYSICS newton and NEWTON_SOLVER vbd)")
        return
    try:
        import vbd_plant
        import vbd_scene
        doc = vbd_scene.read(VBD_SCENE)
        mine = vbd_scene.apply(doc["params"], vbd_plant, ee_cutter)
    except Exception as e:
        VBD_PLANT = ""
        log(f"VBD_SCENE {VBD_SCENE}: NOT loaded, no plant: {e}")
        write_state(vbd_scene={"name": VBD_SCENE, "error": str(e)})
        return
    VBD_PLANT, VBD_PLANT_ASSET = mine["plant"], mine["asset"]
    VBD_ARM_EFFORT_CAP, VBD_ARM_CAP_DAMPING = mine["arm_cap"] != "off", mine["arm_cap"] == "full"
    VBD_WHEEL_HOLD = mine["wheel_hold"]
    if VBD_CFG is not None:
        VBD_CFG.solver_cfg.iterations = mine["iterations"]
    VBD_SCENE_GAINS.update(finger=mine["finger_gains"], blade=mine["blade_gains"])
    log(f"vbd scene {VBD_SCENE}: {vbd_scene.line(doc['params'])}")
    write_state(vbd_scene={"name": VBD_SCENE, "params": doc["params"]})


def build_world(stage):
    """The scene chosen by SIM_SCENE: "" -> ground plane + lights, "lavender" -> the lavender farm, else a USD
    file in sim/scene/. Returns layout info for FLEET_STATE (the web UI's spawn map)."""
    if not SIM_SCENE:
        layout = build_default_world(stage)
    elif SIM_SCENE == "lavender":
        layout = build_lavender_world(stage)
    else:
        layout = build_file_world(stage, SIM_SCENE)
    soften_lavender(stage)
    if PHYSICS_ENGINE == "newton" and NEWTON_SOLVER == "vbd" and VBD_PLANT.strip():
        global _vbd_plant
        import vbd_plant
        try:
            _vbd_plant = vbd_plant.build(stage, VBD_PLANT, log, VBD_PLANT_ASSET)
            if _vbd_plant is not None and vbd_plant.FLOWER_HULLS:
                NEWTON_PARSE_CALLBACKS.append(_vbd_plant.add_flower_hulls)
            NEWTON_IGNORE_PATHS.extend(p for p in vbd_plant.RENDER_ONLY if p not in NEWTON_IGNORE_PATHS)
        except Exception:
            log("VBD_PLANT: the plant could not be built:\n" + traceback.format_exc())
    return dict(layout, scene_source=SIM_SCENE or "default")


def spawn_robot(stage, ns, model, pose):
    """Reference the model's USD at /World/<ns>, at pose = (x, y, yaw in degrees) on the ground."""
    x, y, yaw = pose
    root = f"/World/{ns}"
    prim = stage.DefinePrim(root, "Xform")
    prim.GetReferences().AddReference(MODEL_ASSETS[model]["usd_path"])
    vset = prim.GetVariantSets().GetVariantSet("Physics")
    if vset.GetVariantNames():  # Isaac Sim 6.2's URDF converter: physics per engine in a variant set, none selected
        # (under Newton VBD nothing got picked: no articulation root, 2026-11-08). physx and mujoco both sublayer
        # "physics" (plain UsdPhysics) and add their engine's attributes; VBD reads the plain one. Not "mujoco" for
        # Newton's MuJoCo solver: it deletes every joint's DriveAPI for the converter's MjcActuators (arm joint_2: 17
        # N*m/rad, 14 N*m), so configure_arm_drives / cap_wheel_drives found nothing and the arm fell (2026-11-11);
        # "physx" keeps the drives, as on 6.0 before this selection existed.
        vset.SetVariantSelection("physics" if PHYSICS_ENGINE == "newton" and NEWTON_SOLVER == "vbd" else "physx")
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, SPAWN_Z))
    xf.AddRotateZOp().Set(float(yaw))
    robots_g = UsdPhysics.CollisionGroup.Get(stage, ROBOTS_GROUP)
    if robots_g:  # soften_lavender: the lavender foliage doesn't block robots
        robots_g.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(root)
    return root


def add_camera(stage, robot_root, optical_link=None, hfov_deg=HFOV_DEG, index=0):
    """Colour camera. Frame chain: camera_0_link -> optical frame (z fwd, y down) -> USD camera.

    optical_link (MODEL_PARAMS' camera_optical_link): most models' D435i xacro doesn't produce a ROS-optical-
    convention frame on its own, so the default path builds one by hand under camera_0_link. The real MTU
    robots' ZED2i xacro already emits one (verified: its joint rpy is exactly the standard link->optical
    rotation), so for them this is instead the existing link name to mount the camera under directly, with no
    extra rotation needed beyond the USD-camera/ROS-optical "flip" every model needs.
    """
    if optical_link:
        optical = find_prim(stage, robot_root, optical_link)
    else:
        link = find_prim(stage, robot_root, f"camera_{index}_link")
        optical = f"{link}/camera_{index}_color_optical_frame"
        xf = UsdGeom.Xform.Define(stage, optical)
        # rows = optical x/y/z axes expressed in the ROS link frame (x fwd, y left, z up)
        m = Gf.Matrix4d(1.0)
        m.SetRow3(0, Gf.Vec3d(0, -1, 0))
        m.SetRow3(1, Gf.Vec3d(0, 0, -1))
        m.SetRow3(2, Gf.Vec3d(1, 0, 0))
        if index == 0:
            m.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in os.environ.get("CAMERA_OFFSET", "0.0,0.0,0.0").split(",")]))
        xf.AddTransformOp().Set(m)

    cam_path = f"{optical}/camera"
    cam = UsdGeom.Camera.Define(stage, cam_path)
    flip = Gf.Matrix4d(1.0)  # USD camera looks down -Z with +Y up; optical frame looks down +Z with +Y down
    flip.SetRow3(1, Gf.Vec3d(0, -1, 0))
    flip.SetRow3(2, Gf.Vec3d(0, 0, -1))
    UsdGeom.Xformable(cam).AddTransformOp().Set(flip)
    h_aperture = 20.955
    cam.CreateHorizontalApertureAttr(h_aperture)
    cam.CreateVerticalApertureAttr(h_aperture * CAM_H / CAM_W)
    cam.CreateFocalLengthAttr(h_aperture / 2 / math.tan(math.radians(hfov_deg) / 2))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, CAM_FAR_CLIP))
    return cam_path


# Real position-servo gains for the real MTU robots' Kinova arm+gripper joints, overriding IMPORT_SETTINGS'
# global stiffness=0/damping=1000 there (see configure_arm_drives). Round, conventional Isaac Sim
# position-control values (in the same ballpark commonly used for imported robot-arm URDFs, e.g. Franka Panda
# samples), not the Kinova's own real servo gains -- this is a simulation of the joints' *response*, not a
# torque-accurate model, and disclosed as such; adjust if the arm moves too slowly/oscillates in practice.
ARM_DRIVE_STIFFNESS = 1.0e5
ARM_DRIVE_DAMPING = 1.0e4
# arm_0_joint_2 (the "shoulder" joint -- carries by far the most gravitational load/inertia of the 6, matching
# its own URDF effort limit of 14 N*m vs 7-10 for the others) is a real outlier under the uniform gains above:
# confirmed live, comparing commanded vs. observed joint_states during an actual zero->cut_init trajectory
# (control_msgs/FollowJointTrajectory, not a raw instantaneous step) -- joints 1/3 show smooth, bounded
# following error throughout, but joint_2 swings *through* zero error and overshoots by over 0.6 rad before
# slowly settling, a genuine underdamped oscillation, not simple lag. Tried uniformly scaling stiffness+damping
# together (preserves the same relative damping ratio, so joint_2 stayed just as underdamped, only smaller in
# absolute terms) and damping alone (didn't fix joint_2's overshoot and added much worse lag to joints 1/3,
# which were fine before) -- neither worked, because one uniform gain pair can't properly serve joints with
# this different a load. Only arm_0_joint_2 gets its own much stronger, non-uniform gain; every other arm/
# gripper joint keeps the values above.
ARM_JOINT_2_STIFFNESS = 1.0e7
ARM_JOINT_2_DAMPING = 1.0e6
# The actual root cause of joint_2 "falling" (the gain above only helped with overshoot): the importer copies the
# URDF effort limit (14 N*m) into the drive's maxForce, and in this sim joint_2 needs more than that -- measured
# via platform/joint_states effort: ~11-12 N*m just holding cut_init, 19-26 N*m while moving at ~0.14 rad/s.
# Once the drive saturates, gain is irrelevant and gravity wins: joint_2 dropped from -0.65 to -2.1 rad mid
# zero->cut_init, couldn't climb back from -2.35 on cut_init->zero, and this was also grid_cutter's patch-2
# servo stall (blocked toward 0, free toward -2.5). With 40 N*m every move tracks within ~0.06 rad and a full
# cut_stem runs patch after patch (peak 24 N*m, >14 N*m for 27% of the run). Gains from 1e4/1e3 to 1e7/1e6 all
# need the same torque, so the extra demand isn't the drive fighting itself. OPEN: static gravity from the URDF
# masses (which match the imported USD) predicts only ~7.5 N*m at cut_init and ~9-10 N*m peak; the real arm
# works within 14 N*m. Self-collision and joint friction ruled out; chassis pitch/rocking not yet checked. This
# is therefore a disclosed workaround, not a realistic torque model.
#
# Made generic (was a fixed 40 N*m on arm_0_joint_2, i.e. 14 x ~3) when a200_0284's Kinova Gen3 7-DOF (joint effort
# limits 39/39/39/39/9/9/9) went through the same failure at a bigger size: joint_2 sat at exactly the 40 N*m cap
# for two seconds during the cutter's first plan move, then the arm collapsed and the physics went chaotic (joint 5
# swung 7 rad in a second, joint 6 wound up to 14 rad, efforts of hundreds of N*m from the resulting collisions).
# So every arm_0_joint_N drive now gets ARM_EFFORT_SCALE x its own URDF effort limit: 14 -> 42 for the Gen3
# Lite's joint_2 (~ the old 40), 39 -> 117 for the 7-DOF's. The gripper joints keep their URDF limits.
ARM_EFFORT_SCALE = 3.0
# Newton (MuJoCo) arm gains, per radian: (stiffness N*m/rad, damping N*m*s/rad) for arm_0_joint_N. The PhysX values
# above can't be used: Newton reads USD angular drive gains as per degree and scales them by 57.3, so joint_2's
# damping became 5.7e7 N*m*s/rad against its 42 N*m cap, and the joint could only creep (stow: joint_2 0.03 rad of
# 0.79 after 20 s, joints 4-6 0.17-0.57 rad off, 2026-10-20). Shoulder/elbow stiff enough to hold the arm's weight
# within ~1 deg at the effort cap; damping near critical for the arm's inertia.
NEWTON_ARM_GAINS = {1: (600.0, 40.0), 2: (1500.0, 120.0), 3: (1000.0, 60.0), 4: (300.0, 15.0), 5: (300.0, 15.0),
                    6: (200.0, 10.0), 7: (200.0, 10.0)}
# Newton gains (per radian) for every other arm_0 joint, i.e. the gripper's. With the PhysX values (57.3x per radian:
# 5.7e6 N*m/rad on fingers of ~1e-5 kg*m^2) the 2F Lite's bottom joints banged against their 10 N*m cap and flailed
# (-1.24 rad held for "open", limits -0.1..0.96; tips at 1000 N*m held), so cut_stem's gripper never followed (2026-10-20).
NEWTON_GRIPPER_GAINS = (20.0, 1.0)


def configure_arm_drives(stage, root, drop_mimic_constraints=True):
    """Real MTU robots' Kinova arm+gripper: give every arm_0_*/gripper joint a real position-servo drive.

    IMPORT_SETTINGS' global override_joint_stiffness=0.0 is correct for the wheels (a pure velocity drive with
    no position-holding spring, so they can spin continuously) but leaves EVERY joint, arm included, unable to
    hold a commanded position at all -- confirmed live: a JointState position command barely moved the arm
    (a few hundredths of a radian over 2s, not the ~0.5 rad commanded). USD's DriveAPI needs nonzero stiffness
    to act as a position servo at all (with stiffness=0 it's pure velocity damping, so a targetPosition write
    has no effect regardless of what IsaacArticulationController sends) -- this reconfigures just the arm/
    gripper joints' DriveAPI directly on the imported USD, leaving every other joint (wheels, rockers, ...)
    on the global velocity-drive settings untouched. Matched by "arm_0" appearing anywhere in the joint prim's
    own name, covering both arm_0_joint_N and arm_0_gripper_*_joint uniformly without hardcoding either list
    (except arm_0_joint_2's own stronger gain, see above).
    No-op for every model without an arm (nothing named "arm_0" exists in their USD).
    """
    # Mimic constraints (drop_mimic_constraints, default on for every arm): the URDF importer turns each gripper
    # <mimic> joint into a physics constraint (NewtonMimicAPI) on top of the position drive every arm_0 joint gets
    # here, and moveit_sim_bridge also commands every gripper joint (from the URDF's own mimic multiplier/offset).
    # Constraint and drives then fight: on the Kinova 2F Lite, commanding only the driven joint barely moved it (0.04
    # rad for any target) and "open" left it at -0.204, outside its -0.1 limit (bridge pulls the tip to its +0.149
    # offset, constraint + drive settle at bottom = -0.149/0.676); on a200_0284's Robotiq 2F-85 the constraints had
    # wrong coefficients and closing threw joints >1 rad past their limits and spun the wrist. Removed (the whole API
    # schema; clearing only the target relationship did nothing), each gripper joint follows its own command.
    from pxr import Usd, UsdPhysics

    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if "arm_0" not in prim.GetName():
            continue
        stiffness = ARM_JOINT_2_STIFFNESS if prim.GetName() == "arm_0_joint_2" else ARM_DRIVE_STIFFNESS
        damping = ARM_JOINT_2_DAMPING if prim.GetName() == "arm_0_joint_2" else ARM_DRIVE_DAMPING
        idx = re.fullmatch(r"arm_0_joint_(\d+)", prim.GetName())
        if PHYSICS_ENGINE == "newton":
            # USD angular drive gains are per degree; Newton converts them back to the per-radian values above
            kp, kd = NEWTON_ARM_GAINS.get(int(idx.group(1)), NEWTON_GRIPPER_GAINS) if idx else NEWTON_GRIPPER_GAINS
            stiffness, damping = kp * math.pi / 180.0, kd * math.pi / 180.0
        mimic = prim.GetRelationship("newton:mimicJoint")  # set by the URDF importer for <mimic> follower joints
        if drop_mimic_constraints and mimic and mimic.GetTargets():
            # the joint keeps its position drive; only the constraint goes. Clearing the target alone wasn't enough
            # (the constraint kept coupling the joints), so the API schema itself is removed as well.
            mimic.ClearTargets(True)
            prim.RemoveAppliedSchema("NewtonMimicAPI")
            log(f"mimic follower {prim.GetName()}: mimic constraint removed (driven directly)")
        for dof in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, dof)
            if drive:
                drive.GetStiffnessAttr().Set(stiffness)
                drive.GetDampingAttr().Set(damping)
                if re.fullmatch(r"arm_0_joint_\d+", prim.GetName()):
                    max_force = drive.GetMaxForceAttr()
                    effort = max_force.Get()  # the importer's copy of the URDF <limit effort>
                    if effort and effort > 0:
                        max_force.Set(float(effort) * ARM_EFFORT_SCALE)


def build_ros_graph(og, usdrt_sdf, stage, root, chassis, ns, cam_path, params, cam1_path=None):
    keys = og.Controller.Keys
    articulation_controller = "isaacsim.core.nodes.IsaacArticulationController"

    nodes = [
        ("Tick", "omni.graph.action.OnPlaybackTick"),
        # message timestamps: simulation time (USE_SIM_TIME, the same clock as /clock) or wall-clock time
        ("Stamp", "isaacsim.core.nodes.IsaacReadSimulationTime" if USE_SIM_TIME else "isaacsim.core.nodes.IsaacReadSystemTime"),
        # --- drive: cmd_vel -> wheel velocities (diff drive for every model; BodyDrive adds Ridgeback's sideways motion below)
        ("CmdVel", "omni.graph.scriptnode.ScriptNode"),  # TwistStamped, see CMD_VEL_SCRIPT
        ("BreakLin", "omni.graph.nodes.BreakVector3"),
        ("BreakAng", "omni.graph.nodes.BreakVector3"),
        # --- state: odometry, tf, joint states
        ("Odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("PubTfOdom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ("PubJoints", "isaacsim.ros2.bridge.ROS2PublishJointState"),
    ]
    stamp = "Stamp.outputs:simulationTime" if USE_SIM_TIME else "Stamp.outputs:systemTime"
    values = ([("Stamp.inputs:resetOnStop", False)] if USE_SIM_TIME else []) + [  # monotonic across Stop/Play
        ("CmdVel.inputs:namespace", ns),
        ("CmdVel.inputs:topicName", "cmd_vel"),
        ("CmdVel.inputs:timeout", CMD_VEL_TIMEOUT_S),
        ("CmdVel.inputs:script", CMD_VEL_SCRIPT),
        ("Odom.inputs:chassisPrim", [usdrt_sdf.Path(chassis)]),
        ("PubOdom.inputs:nodeNamespace", ns),
        ("PubOdom.inputs:topicName", "platform/odom"),
        ("PubOdom.inputs:chassisFrameId", "base_link"),
        ("PubOdom.inputs:odomFrameId", "odom"),
        ("PubTfOdom.inputs:nodeNamespace", ns),
        # Exact pose as its own TF branch, ground_truth -> base_link_ground_truth: odom -> base_link belongs to the
        # robot's platform EKF (robot/bin/ekf), and a frame can have only one parent. platform/odom (above) stays in
        # the odom frame: it is the EKF's wheel-odometry input, like the real platform/odom.
        ("PubTfOdom.inputs:parentFrameId", "ground_truth"),
        ("PubTfOdom.inputs:childFrameId", "base_link_ground_truth"),
        ("PubJoints.inputs:nodeNamespace", ns),
        ("PubJoints.inputs:topicName", "platform/joint_states"),
        ("PubJoints.inputs:targetPrim", [usdrt_sdf.Path(chassis)]),
    ]
    connections = [
        ("Tick.outputs:tick", "CmdVel.inputs:execIn"),
        (stamp, "CmdVel.inputs:stamp"),
        ("CmdVel.outputs:linearVelocity", "BreakLin.inputs:tuple"),
        ("CmdVel.outputs:angularVelocity", "BreakAng.inputs:tuple"),
    ]
    create_attributes = [
        ("CmdVel.inputs:namespace", "token"), ("CmdVel.inputs:topicName", "token"),
        ("CmdVel.inputs:stamp", "double"), ("CmdVel.inputs:timeout", "double"),
        ("CmdVel.outputs:linearVelocity", "double[3]"), ("CmdVel.outputs:angularVelocity", "double[3]"),
    ]

    # Every model, including Ridgeback, drives forward/back and rotation via real diff_4wd.yaml-style wheel
    # rolling -- see sim/config/model_params.yaml's r100 comment for why Ridgeback's sideways motion needs a different mechanism
    # (BodyDrive, added below) instead of extending this same approach to linear.y.
    front = ["front_left_wheel_joint", "front_right_wheel_joint"]
    rear = ["rear_left_wheel_joint", "rear_right_wheel_joint"]
    nodes += [
        ("Diff", "isaacsim.robot.wheeled_robots.DifferentialController"),
        ("VelCtl", "omni.graph.scriptnode.ScriptNode"),
        ("DriveFront", articulation_controller),
        ("DriveRear", articulation_controller),
    ]
    create_attributes += [
        ("VelCtl.inputs:cmd_v", "double"), ("VelCtl.inputs:cmd_w", "double"),
        ("VelCtl.inputs:chassisPath", "token"), ("VelCtl.inputs:enabled", "bool"),
        ("VelCtl.inputs:kp_v", "double"), ("VelCtl.inputs:ki_v", "double"), ("VelCtl.inputs:lim_v", "double"),
        ("VelCtl.inputs:kp_w", "double"), ("VelCtl.inputs:ki_w", "double"), ("VelCtl.inputs:lim_w", "double"),
        ("VelCtl.inputs:ff_v", "double"), ("VelCtl.inputs:ff_w", "double"),
        ("VelCtl.inputs:alpha", "double"), ("VelCtl.inputs:dt", "double"), ("VelCtl.inputs:simTime", "double"),
        ("VelCtl.inputs:max_v", "double"), ("VelCtl.inputs:max_w", "double"),
        ("VelCtl.inputs:cmd_y", "double"), ("VelCtl.outputs:out_y", "double"),
        ("VelCtl.outputs:odom_lin", "double[3]"), ("VelCtl.outputs:odom_ang", "double[3]"),
        ("VelCtl.inputs:kp_y", "double"), ("VelCtl.inputs:ki_y", "double"), ("VelCtl.inputs:lim_y", "double"),
        ("VelCtl.outputs:out_v", "double"), ("VelCtl.outputs:out_w", "double"),
    ]
    kp_v, ki_v, lim_v, kp_w, ki_w, lim_w, *ff = params.get("vel_ctl", VELCTL_DEFAULT)
    ff_v, ff_w = (list(ff) + [1.0, 1.0])[:2]
    values += [
        ("VelCtl.inputs:script", VEL_CTL_SCRIPT),
        ("VelCtl.inputs:chassisPath", chassis),
        ("VelCtl.inputs:enabled", VELCTL_ENABLED),
        ("VelCtl.inputs:kp_v", kp_v), ("VelCtl.inputs:ki_v", ki_v), ("VelCtl.inputs:lim_v", lim_v),
        ("VelCtl.inputs:kp_w", kp_w), ("VelCtl.inputs:ki_w", ki_w), ("VelCtl.inputs:lim_w", lim_w),
        ("VelCtl.inputs:ff_v", float(ff_v)), ("VelCtl.inputs:ff_w", float(ff_w)),
        ("VelCtl.inputs:alpha", 0.4), ("VelCtl.inputs:dt", PHYSICS_FRAME_DT),
        ("VelCtl.inputs:max_v", params["max_linear"]), ("VelCtl.inputs:max_w", params["max_angular"]),
        ("VelCtl.inputs:kp_y", VELCTL_LATERAL[0]), ("VelCtl.inputs:ki_y", VELCTL_LATERAL[1]),
        ("VelCtl.inputs:lim_y", VELCTL_LATERAL[2]),
        ("Diff.inputs:wheelRadius", params["wheel_radius"]),
        ("Diff.inputs:wheelDistance", params["wheel_separation"] * params["separation_multiplier"]),
        # headroom: VelCtl clamps the *command* to the robot's real limits, its feedback may exceed them
        ("Diff.inputs:maxLinearSpeed", params["max_linear"] * VELCTL_HEADROOM),
        ("Diff.inputs:maxAngularSpeed", params["max_angular"] * VELCTL_HEADROOM),
        ("DriveFront.inputs:targetPrim", [usdrt_sdf.Path(chassis)]),
        ("DriveFront.inputs:jointNames", front),
        ("DriveRear.inputs:targetPrim", [usdrt_sdf.Path(chassis)]),
        ("DriveRear.inputs:jointNames", rear),
    ]
    connections += [
        # VelCtl (feedback on the commanded twist) feeds Diff every tick, the drives apply its wheel speeds
        ("Tick.outputs:tick", "VelCtl.inputs:execIn"),
        ("VelCtl.outputs:execOut", "Diff.inputs:execIn"),
    ]
    if PHYSICS_ENGINE == "newton":
        # Newton advances 11 steps of 1/240 s per 1/20 s frame (0.0458 s, /clock), not PHYSICS_FRAME_DT's 12: a
        # speed measured over the assumed 0.05 s read 9 % high, so the closed loop held the robot at 0.90 of every
        # command (2026-10-20). The real per-frame sim time from IsaacReadSimulationTime instead.
        nodes.append(("PhysTime", "isaacsim.core.nodes.IsaacReadSimulationTime"))
        values.append(("PhysTime.inputs:resetOnStop", False))
        connections.append(("PhysTime.outputs:simulationTime", "VelCtl.inputs:simTime"))
    connections += [
        ("Tick.outputs:tick", "DriveFront.inputs:execIn"),
        ("Tick.outputs:tick", "DriveRear.inputs:execIn"),
        ("BreakLin.outputs:x", "VelCtl.inputs:cmd_v"),
        ("BreakAng.outputs:z", "VelCtl.inputs:cmd_w"),
        ("BreakLin.outputs:y", "VelCtl.inputs:cmd_y"),
        ("VelCtl.outputs:out_v", "Diff.inputs:linearVelocity"),
        ("VelCtl.outputs:out_w", "Diff.inputs:angularVelocity"),
        ("Diff.outputs:velocityCommand", "DriveFront.inputs:velocityCommand"),
        ("Diff.outputs:velocityCommand", "DriveRear.inputs:velocityCommand"),
    ]
    # Not under Newton: the brake rewrites the wheel drives' stiffness at runtime (1e4 parked, 0 driving), which
    # Newton's tensor backend syncs into MuJoCo actuator gains in position-actuator form (gainprm[0] = kp,
    # biasprm = [0, -kp, -kd]): a velocity drive (kp 0) loses its velocity gain once the brake releases. MuJoCo's
    # implicit integrator holds a damped wheel still on its own (the brake works around PhysX's resting jitter).
    if WHEEL_BRAKE and PHYSICS_ENGINE == "physx":  # see WHEEL_BRAKE_SCRIPT
        nodes += [("WheelBrake", "omni.graph.scriptnode.ScriptNode")]
        create_attributes += [
            ("WheelBrake.inputs:chassisPath", "token"), ("WheelBrake.inputs:wheelNames", "token"),
            ("WheelBrake.inputs:cmd_v", "double"), ("WheelBrake.inputs:cmd_w", "double"),
            ("WheelBrake.inputs:cmd_y", "double"), ("WheelBrake.inputs:stiffness", "double"),
            ("WheelBrake.inputs:latchSpeed", "double"), ("WheelBrake.inputs:latchDelay", "double"),
        ]
        values += [
            ("WheelBrake.inputs:script", WHEEL_BRAKE_SCRIPT),
            ("WheelBrake.inputs:chassisPath", chassis),
            ("WheelBrake.inputs:wheelNames", ",".join(front + rear)),
            ("WheelBrake.inputs:stiffness", WHEEL_BRAKE_STIFFNESS),
            ("WheelBrake.inputs:latchSpeed", WHEEL_BRAKE_LATCH_SPEED),
            ("WheelBrake.inputs:latchDelay", WHEEL_BRAKE_LATCH_DELAY),
        ]
        connections += [
            ("Tick.outputs:tick", "WheelBrake.inputs:execIn"),
            ("BreakLin.outputs:x", "WheelBrake.inputs:cmd_v"),
            ("BreakAng.outputs:z", "WheelBrake.inputs:cmd_w"),
            ("BreakLin.outputs:y", "WheelBrake.inputs:cmd_y"),
        ]

    if params["drive"] == "omni":
        # Ridgeback: BodyDrive patches in the one motion component real wheel rolling structurally cannot
        # produce (sideways/linear.y) -- see its comment (BODY_DRIVE_SCRIPT) for the full reasoning and the
        # comment on r100 in sim/config/model_params.yaml for the underlying wheel-collision-geometry finding.
        nodes += [("BodyDrive", "omni.graph.scriptnode.ScriptNode")]
        create_attributes += [
            ("BodyDrive.inputs:vy", "double"),
            ("BodyDrive.inputs:wz", "double"),
            ("BodyDrive.inputs:chassisPath", "token"),
        ]
        values += [
            ("BodyDrive.inputs:chassisPath", chassis),
            ("BodyDrive.inputs:script", BODY_DRIVE_SCRIPT),
        ]
        connections += [
            ("Tick.outputs:tick", "BodyDrive.inputs:execIn"),
            ("VelCtl.outputs:out_y", "BodyDrive.inputs:vy"),
            ("VelCtl.outputs:out_w", "BodyDrive.inputs:wz"),
        ]

    connections += [
        # state
        ("Tick.outputs:tick", "Odom.inputs:execIn"),
        ("Odom.outputs:execOut", "PubOdom.inputs:execIn"),
        ("Odom.outputs:execOut", "PubTfOdom.inputs:execIn"),
        ("Odom.outputs:execOut", "PubJoints.inputs:execIn"),
        ("Odom.outputs:position", "PubOdom.inputs:position"),
        ("Odom.outputs:orientation", "PubOdom.inputs:orientation"),
        # published twist = VelCtl's pose-derived body velocity (PhysX's reported angular velocity read up to 25%
        # above the real yaw change at low rates, and does not include BodyDrive's overrides)
        ("VelCtl.outputs:odom_lin", "PubOdom.inputs:linearVelocity"),
        ("VelCtl.outputs:odom_ang", "PubOdom.inputs:angularVelocity"),
        ("Odom.outputs:position", "PubTfOdom.inputs:translation"),
        ("Odom.outputs:orientation", "PubTfOdom.inputs:rotation"),
        (stamp, "PubOdom.inputs:timeStamp"),
        (stamp, "PubTfOdom.inputs:timeStamp"),
        (stamp, "PubJoints.inputs:timeStamp"),
    ]

    cameras = []  # (topic index, render product node, frameId, depth range): publishers added below
    # --- camera: one render product feeds every enabled stream
    if cam_path and CAM_STREAMS:
        optical_link = params.get("camera_optical_link")
        nodes += [("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct")]
        values += [
            ("RenderProduct.inputs:cameraPrim", [usdrt_sdf.Path(cam_path)]),
            ("RenderProduct.inputs:width", CAM_W),
            ("RenderProduct.inputs:height", CAM_H),
        ]
        connections += [("Tick.outputs:tick", "RenderProduct.inputs:execIn")]
        if optical_link:
            # A real URDF link (the ZED2i's own xacro already emits a correctly-oriented optical frame, see
            # add_camera/MODEL_PARAMS) -- its fixed joint to its parent is part of the real URDF, so the
            # robot's own robot_state_publisher (robot/bin/robot_state, running in that robot's container)
            # already publishes this TF relationship. No manual PubTfCamera needed, unlike the D435i case below.
            optical = optical_link
        else:
            optical = "camera_0_color_optical_frame"
            nodes += [("PubTfCamera", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree")]
            values += [
                # camera_0_link -> optical frame (fixed): quaternion (x, y, z, w)
                ("PubTfCamera.inputs:nodeNamespace", ns),
                ("PubTfCamera.inputs:parentFrameId", "camera_0_link"),
                ("PubTfCamera.inputs:childFrameId", optical),
                ("PubTfCamera.inputs:rotation", [-0.5, 0.5, -0.5, 0.5]),
            ]
            connections += [
                ("Odom.outputs:execOut", "PubTfCamera.inputs:execIn"),
                (stamp, "PubTfCamera.inputs:timeStamp"),
            ]
        cameras.append((0, "RenderProduct", optical, params["depth_range"]))

    # --- wrist camera (camera_1, D405 on the arm's end effector): its own render product, same helpers as
    # camera_0. frameId is camera_1_depth_optical_frame, which the robot's own robot_state_publisher already
    # publishes from the real URDF (same standard link->optical rotation the D435i path builds by hand).
    if cam1_path and CAM_STREAMS:
        nodes += [("RenderProduct1", "isaacsim.core.nodes.IsaacCreateRenderProduct")]
        values += [
            ("RenderProduct1.inputs:cameraPrim", [usdrt_sdf.Path(cam1_path)]),
            ("RenderProduct1.inputs:width", CAM_W),
            ("RenderProduct1.inputs:height", CAM_H),
        ]
        connections += [("Tick.outputs:tick", "RenderProduct1.inputs:execIn")]
        cameras.append((1, "RenderProduct1", "camera_1_depth_optical_frame", DEPTH_RANGE_M["d405"]))

    for index, rp, frame, depth_range in cameras:
        for stream, kind in (("color", "rgb"), ("depth", "depth")):
            if stream not in CAM_STREAMS:
                continue
            base = f"Pub{index or ''}{stream.title()}"  # PubColor / Pub1Color
            pub, info = base, f"{base}Info"
            nodes += [(pub, "isaacsim.ros2.bridge.ROS2CameraHelper"), (info, "isaacsim.ros2.bridge.ROS2CameraInfoHelper")]
            if stream == "depth":  # install_depth_range_limit
                _depth_ranges[(ns.strip("/"), f"sensors/camera_{index}/{stream}/image")] = depth_range
            for name, topic, extra in (
                (pub, f"sensors/camera_{index}/{stream}/image", [(f"{pub}.inputs:type", kind)]),
                (info, f"sensors/camera_{index}/{stream}/camera_info", []),
            ):
                values += [
                    (f"{name}.inputs:nodeNamespace", ns),
                    (f"{name}.inputs:topicName", topic),
                    (f"{name}.inputs:frameId", frame),
                    (f"{name}.inputs:frameSkipCount", CAM_FRAME_SKIP),
                    (f"{name}.inputs:useSystemTime", not USE_SIM_TIME),
                    # its sim time otherwise restarts at 0 on Stop/Play (Reset scene), behind /clock and the rest
                    (f"{name}.inputs:resetSimulationTimeOnStop", False),
                ] + extra
                connections += [
                    (f"{rp}.outputs:execOut", f"{name}.inputs:execIn"),
                    (f"{rp}.outputs:renderProductPath", f"{name}.inputs:renderProductPath"),
                ]

    # --- IMU (real MTU robots only): a ScriptNode reads isaacsim.sensors.experimental.physics' IMU/IMUSensor
    # (no ready-made OGN node exists to read an IMU, only isaacsim.ros2.bridge.ROS2PublishImu to publish one
    # already-read) and feeds it into ROS2PublishImu. frameId/topicName use imu_index (Clearpath's own real
    # per-robot numbering, confirmed from each robot's raw, un-flattened generate_description+xacro output, not
    # guessed): Jackal (j100) always has a separate platform-default IMU occupying slot 0, so the explicit
    # sensor these robots configure is imu_1; A300 has no such default, so its own explicit sensor is imu_0 --
    # publishing it as "imu_1" (this code's old hardcoded literal) would be wrong for a300_00036 specifically.
    # frameId is the real robot's own imu_<n>_link name (what that robot's robot_state_publisher, running from
    # its own un-flattened URDF regeneration in robot_state, actually publishes in its TF tree), not
    # params["imu_link"] -- that field names where the sensor is physically attached in *this sim's* flattened/
    # merged USD (see derive_model_params' body() on why those differ), which has no bearing on the real TF frame name
    # ROS clients expect.
    if params.get("imu_link"):
        imu_idx = params.get("imu_index", 0)
        imu_frame = f"imu_{imu_idx}_link"
        imu_body = find_prim(stage, root, params["imu_link"])
        imu_sensor_path = f"{imu_body}/{imu_frame}_sensor"
        nodes += [("ImuRead", "omni.graph.scriptnode.ScriptNode"), ("PubImu", "isaacsim.ros2.bridge.ROS2PublishImu")]
        create_attributes += [
            ("ImuRead.inputs:imuPath", "token"),
            ("ImuRead.outputs:orientation", "quatd[4]"),
            ("ImuRead.outputs:linearAcceleration", "vectord[3]"),
            ("ImuRead.outputs:angularVelocity", "vectord[3]"),
        ]
        if PHYSICS_ENGINE == "newton":  # IMU_NEWTON_SCRIPT: no sensor prim, the IMU body's pose + the root's velocity
            create_attributes += [("ImuRead.inputs:chassisPath", "token"), ("ImuRead.inputs:dt", "double"),
                                  ("ImuRead.inputs:simTime", "double")]
            connections += [("PhysTime.outputs:simulationTime", "ImuRead.inputs:simTime")]
            values += [("ImuRead.inputs:imuPath", imu_body), ("ImuRead.inputs:script", IMU_NEWTON_SCRIPT),
                       ("ImuRead.inputs:chassisPath", chassis), ("ImuRead.inputs:dt", PHYSICS_FRAME_DT)]
        else:
            values += [("ImuRead.inputs:imuPath", imu_sensor_path), ("ImuRead.inputs:script", IMU_READ_SCRIPT)]
        values += [
            ("PubImu.inputs:nodeNamespace", ns),
            ("PubImu.inputs:topicName", f"sensors/imu_{imu_idx}/data"),
            ("PubImu.inputs:frameId", imu_frame),
        ]
        connections += [
            ("Tick.outputs:tick", "ImuRead.inputs:execIn"),
            ("ImuRead.outputs:execOut", "PubImu.inputs:execIn"),
            ("ImuRead.outputs:orientation", "PubImu.inputs:orientation"),
            ("ImuRead.outputs:linearAcceleration", "PubImu.inputs:linearAcceleration"),
            ("ImuRead.outputs:angularVelocity", "PubImu.inputs:angularVelocity"),
            (stamp, "PubImu.inputs:timeStamp"),
        ]

    # --- 2D lidar (real robots with lidar2d_link only): see LIDAR2D_READ_SCRIPT's own comment for the sensor
    # API and why this publishes directly via rclpy. lidar2d_link (e.g. "lidar2d_0_laser") survives flattening
    # as its own real link (unlike imu_link, it's never merged away -- confirmed in the flattened URDF), so it
    # doubles correctly as both the mount-search name and the real TF frame name, no divergence to handle.
    if params.get("lidar2d_link"):
        lidar_link = params["lidar2d_link"]
        lidar_body = find_prim(stage, root, lidar_link)
        lidar_sensor_path = lidar_body  # Lidar2dRead casts from this link's pose; no sensor prim is authored
        # "lidar2d_0" from the link's own "lidar2d_0_laser" name -- Clearpath's own sensor-index convention,
        # same reasoning as GPS's gps_<n> derivation.
        lidar_topic_stem = "_".join(lidar_link.split("_")[:2])
        nodes += [("Lidar2dRead", "omni.graph.scriptnode.ScriptNode")]
        create_attributes += [
            ("Lidar2dRead.inputs:sensorPath", "token"),
            ("Lidar2dRead.inputs:topicName", "token"),
            ("Lidar2dRead.inputs:frameId", "token"),
            ("Lidar2dRead.inputs:namespace", "token"),
            ("Lidar2dRead.inputs:angleMin", "double"),
            ("Lidar2dRead.inputs:angleMax", "double"),
            ("Lidar2dRead.inputs:numRays", "int"),
            ("Lidar2dRead.inputs:rangeMin", "double"),
            ("Lidar2dRead.inputs:rangeMax", "double"),
        ]
        values += [
            ("Lidar2dRead.inputs:sensorPath", lidar_sensor_path),
            ("Lidar2dRead.inputs:topicName", f"/{ns}/sensors/{lidar_topic_stem}/scan"),
            ("Lidar2dRead.inputs:frameId", lidar_link),
            ("Lidar2dRead.inputs:namespace", ns),
            ("Lidar2dRead.inputs:angleMin", LIDAR2D_ANGLE_MIN),
            ("Lidar2dRead.inputs:angleMax", LIDAR2D_ANGLE_MAX),
            ("Lidar2dRead.inputs:numRays", LIDAR2D_NUM_RAYS),
            ("Lidar2dRead.inputs:rangeMin", LIDAR2D_RANGE_MIN),
            ("Lidar2dRead.inputs:rangeMax", LIDAR2D_RANGE_MAX),
            ("Lidar2dRead.inputs:script", LIDAR2D_READ_SCRIPT),
        ]
        create_attributes += [("Lidar2dRead.inputs:stamp", "double")]
        connections += [("Tick.outputs:tick", "Lidar2dRead.inputs:execIn"), (stamp, "Lidar2dRead.inputs:stamp")]

    # --- 3D lidar (real robots with lidar3d_link only): see LIDAR3D_READ_SCRIPT. Its rays are cast from the chassis
    # body's pose composed with lidar3d_link's pose relative to it (computed once here; both are rigid parts of one
    # body), not from lidar3d_link itself: a200_0333's sensor_arch subtree is USD-instanceable, which once made
    # authoring a sensor prim under it fail ("authoring to an instance proxy is not allowed"); the chassis is not.
    if params.get("lidar3d_link"):
        lidar3d_link = params["lidar3d_link"]
        laser_prim = stage.GetPrimAtPath(find_prim(stage, root, lidar3d_link))
        chassis_prim = stage.GetPrimAtPath(chassis)
        laser_world = UsdGeom.Xformable(laser_prim).ComputeLocalToWorldTransform(0)
        chassis_world = UsdGeom.Xformable(chassis_prim).ComputeLocalToWorldTransform(0)
        rel = laser_world * chassis_world.GetInverse()
        rel_t = rel.ExtractTranslation()
        rel_q = rel.ExtractRotationQuat()
        rel_im = rel_q.GetImaginary()
        lidar3d_sensor_path = chassis  # Lidar3dRead casts from chassis * (rel_t, rel_q); no sensor prim is authored
        lidar3d_topic_stem = "_".join(lidar3d_link.split("_")[:2])
        nodes += [("Lidar3dRead", "omni.graph.scriptnode.ScriptNode")]
        create_attributes += [
            ("Lidar3dRead.inputs:sensorPath", "token"),
            ("Lidar3dRead.inputs:topicName", "token"),
            ("Lidar3dRead.inputs:frameId", "token"),
            ("Lidar3dRead.inputs:namespace", "token"),
            ("Lidar3dRead.inputs:localPos", "double[3]"),
            ("Lidar3dRead.inputs:localQuat", "double[4]"),
            ("Lidar3dRead.inputs:vAngleMin", "double"),
            ("Lidar3dRead.inputs:vAngleMax", "double"),
            ("Lidar3dRead.inputs:vCount", "int"),
            ("Lidar3dRead.inputs:hCount", "int"),
            ("Lidar3dRead.inputs:rangeMin", "double"),
            ("Lidar3dRead.inputs:rangeMax", "double"),
            ("Lidar3dRead.inputs:zOffset", "double"),
            ("Lidar3dRead.inputs:ticksPerScan", "int"),
        ]
        values += [
            ("Lidar3dRead.inputs:sensorPath", lidar3d_sensor_path),
            ("Lidar3dRead.inputs:topicName", f"/{ns}/sensors/{lidar3d_topic_stem}/points"),
            ("Lidar3dRead.inputs:frameId", lidar3d_link),
            ("Lidar3dRead.inputs:namespace", ns),
            ("Lidar3dRead.inputs:localPos", [rel_t[0], rel_t[1], rel_t[2]]),
            ("Lidar3dRead.inputs:localQuat", [rel_q.GetReal(), rel_im[0], rel_im[1], rel_im[2]]),
            ("Lidar3dRead.inputs:vAngleMin", LIDAR3D_V_ANGLE_MIN),
            ("Lidar3dRead.inputs:vAngleMax", LIDAR3D_V_ANGLE_MAX),
            ("Lidar3dRead.inputs:vCount", LIDAR3D_V_COUNT),
            ("Lidar3dRead.inputs:hCount", LIDAR3D_H_COUNT),
            ("Lidar3dRead.inputs:rangeMin", LIDAR3D_RANGE_MIN),
            ("Lidar3dRead.inputs:rangeMax", LIDAR3D_RANGE_MAX),
            ("Lidar3dRead.inputs:zOffset", LIDAR3D_Z_OFFSET),
            ("Lidar3dRead.inputs:ticksPerScan", LIDAR3D_TICKS_PER_SCAN),
            ("Lidar3dRead.inputs:script", LIDAR3D_READ_SCRIPT),
        ]
        create_attributes += [("Lidar3dRead.inputs:stamp", "double")]
        connections += [("Tick.outputs:tick", "Lidar3dRead.inputs:execIn"), (stamp, "Lidar3dRead.inputs:stamp")]

    # --- GPS x2 (real MTU robots only): see GPS_READ_SCRIPT's own comment for why this publishes directly via
    # a plain rclpy publisher inside the script, not isaacsim.ros2.bridge.ROS2Publisher (the generic any-
    # message OGN node, tried first -- its literal SET_VALUES work but a connection into one of its
    # dynamically-created inputs silently never propagates a value, confirmed live).
    for gps_link in params.get("gps_links", []):
        # Index from the link's own name (Clearpath's real numbering, e.g. gps_1_link/gps_2_link), not list
        # position -- currently the same thing since gps_links is always listed in that order, but deriving it
        # from the actual name is what's actually correct and doesn't depend on staying that way.
        i = int(re.search(r"gps_(\d+)_link", gps_link).group(1))
        node = f"Gps{i}"
        gps_path = find_prim(stage, root, gps_link)
        nodes += [(node, "omni.graph.scriptnode.ScriptNode")]
        create_attributes += [
            (f"{node}.inputs:gpsPath", "token"),
            (f"{node}.inputs:topicName", "token"),
            (f"{node}.inputs:frameId", "token"),
            (f"{node}.inputs:namespace", "token"),
        ]
        values += [
            (f"{node}.inputs:gpsPath", gps_path),
            (f"{node}.inputs:topicName", f"/{ns}/sensors/gps_{i}/fix"),
            (f"{node}.inputs:frameId", gps_link),
            # topicName above is already an absolute path, so the rclpy Node's own namespace doesn't affect
            # which topic it publishes to -- this is purely so the *node itself* (ros2 node list) shows up
            # under the robot's namespace instead of at the top level (confirmed live: without this, GPS nodes
            # appeared as bare /gps_read_j100_0921_sensors_gps_1_fix instead of /j100_0921/gps_read_...).
            (f"{node}.inputs:namespace", ns),
            (f"{node}.inputs:script", GPS_READ_SCRIPT),
        ]
        create_attributes += [(f"{node}.inputs:stamp", "double")]
        connections += [("Tick.outputs:tick", f"{node}.inputs:execIn"), (stamp, f"{node}.inputs:stamp")]

    # --- Reference pose (SIM_REF_POSE): see REF_POSE_SCRIPT. base_link's pose on the chassis body, read now (spawn).
    if REF_POSE:
        chassis_world = UsdGeom.Xformable(stage.GetPrimAtPath(chassis)).ComputeLocalToWorldTransform(0)
        base_world = UsdGeom.Xformable(stage.GetPrimAtPath(find_prim(stage, root, "base_link"))) \
            .ComputeLocalToWorldTransform(0)
        rel = base_world * chassis_world.GetInverse()
        rel_t, rel_q = rel.ExtractTranslation(), rel.ExtractRotationQuat()
        rel_im = rel_q.GetImaginary()
        nodes += [("RefPose", "omni.graph.scriptnode.ScriptNode")]
        create_attributes += [
            ("RefPose.inputs:chassisPath", "token"),
            ("RefPose.inputs:namespace", "token"),
            ("RefPose.inputs:refFrame", "token"),
            ("RefPose.inputs:childFrame", "token"),
            ("RefPose.inputs:localPos", "double[3]"),
            ("RefPose.inputs:localQuat", "double[4]"),
            ("RefPose.inputs:stamp", "double"),
        ]
        values += [
            ("RefPose.inputs:chassisPath", chassis),
            ("RefPose.inputs:namespace", ns),
            ("RefPose.inputs:refFrame", "ref_frame"),
            ("RefPose.inputs:childFrame", "base_link_ref"),
            ("RefPose.inputs:localPos", [rel_t[0], rel_t[1], rel_t[2]]),
            ("RefPose.inputs:localQuat", [rel_q.GetReal(), rel_im[0], rel_im[1], rel_im[2]]),
            ("RefPose.inputs:script", REF_POSE_SCRIPT),
        ]
        connections += [("Tick.outputs:tick", "RefPose.inputs:execIn"), (stamp, "RefPose.inputs:stamp")]

    # --- Arm + gripper (real MTU robots only): a second IsaacArticulationController, position-mode, targeting
    # the same chassis articulation root as the wheel drive above. jointNames/positionCommand are wired
    # straight through from ROS2SubscribeJointState's own outputs rather than a hardcoded joint list, so
    # whatever names/order a published JointState message actually uses is exactly what gets commanded --
    # hardcoding a fixed jointNames list here and trusting positionCommand's array to line up with it
    # positionally (this project's existing DriveFront/DriveRear pattern) would silently command the wrong
    # joints if a client ever published a different subset/order than guessed. Every joint IMPORT_SETTINGS
    # configured for velocity-mode (this whole project's global import setting) still accepts a positionCommand
    # write here -- IsaacArticulationController takes position *or* velocity *or* effort per call regardless of
    # the prim's own configured drive type -- so no special per-joint drive reconfiguration was needed.
    if params.get("has_arm"):
        nodes += [
            ("ArmCmd", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("DriveArm", articulation_controller),
        ]
        values += [
            ("ArmCmd.inputs:nodeNamespace", ns),
            ("ArmCmd.inputs:topicName", "arm_0/joint_command"),
            ("DriveArm.inputs:targetPrim", [usdrt_sdf.Path(chassis)]),
        ]
        connections += [
            # ArmCmd's outputs hold their last-received message's values between messages (same as CmdVel's
            # linearVelocity/angularVelocity do for the wheel drive above), so DriveArm re-applies them every
            # tick rather than only on ArmCmd's own execOut -- matches DriveFront/DriveRear/Drive's pattern.
            ("Tick.outputs:tick", "ArmCmd.inputs:execIn"),
            ("Tick.outputs:tick", "DriveArm.inputs:execIn"),
            ("ArmCmd.outputs:jointNames", "DriveArm.inputs:jointNames"),
            ("ArmCmd.outputs:positionCommand", "DriveArm.inputs:positionCommand"),
        ]
        if params.get("end_effector") == "ee_cutter":  # the linkage follows the 2F Lite joint, see ee_cutter.py
            nodes += [("EeCutter", "omni.graph.scriptnode.ScriptNode")]
            create_attributes += [("EeCutter.inputs:chassisPath", "token"), ("EeCutter.inputs:namespace", "token"),
                                  ("EeCutter.inputs:stamp", "double")]
            values += [("EeCutter.inputs:script", ee_cutter.EE_SCRIPT), ("EeCutter.inputs:chassisPath", chassis),
                       ("EeCutter.inputs:namespace", ns)]
            connections += [("Tick.outputs:tick", "EeCutter.inputs:execIn"), (stamp, "EeCutter.inputs:stamp")]

    og.Controller.edit(
        {"graph_path": f"/Graphs/{ns}", "evaluator_name": "execution"},
        {keys.CREATE_NODES: nodes, keys.CREATE_ATTRIBUTES: create_attributes, keys.SET_VALUES: values, keys.CONNECT: connections},
    )


def build_clock_graph(og):
    """USE_SIM_TIME: one /clock for the whole fleet (rclcpp/rclpy subscribe to the absolute /clock whatever the node's
    namespace). Published every played frame from the same simulation time the robots' messages are stamped with;
    nothing is published while the timeline is stopped (spawning, Reset scene), so ROS time pauses with the sim.
    resetOnStop=False keeps it monotonic across Stop/Play: a jump back would make every ROS node drop its TF buffer."""
    if not USE_SIM_TIME:
        return
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/Graphs/fleet_clock", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [("SimTime.inputs:resetOnStop", False), ("PubClock.inputs:topicName", "/clock")],
            keys.CONNECT: [
                ("Tick.outputs:tick", "PubClock.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
            ],
        },
    )


# (eye, target) set by a camera request: replaces aim_viewport's own view until the sim restarts (or eye=None)
VIEW_OVERRIDE = None


def aim_viewport(points=None):
    """Look over the robots at `points` ((x, y, ...) each) from behind and to the right; without points, over
    the scene's empty lanes. For robots in one rank at x=0 this is the view the sim always had. A camera
    request's view (VIEW_OVERRIDE) wins."""
    try:
        from omni.kit.viewport.utility.camera_state import ViewportCameraState

        if VIEW_OVERRIDE:
            state = ViewportCameraState("/OmniverseKit_Persp")
            state.set_position_world(Gf.Vec3d(*VIEW_OVERRIDE[0]), True)
            state.set_target_world(Gf.Vec3d(*VIEW_OVERRIDE[1]), True)
            return
        points = points or [(0.0, y, 0.0) for _, y, _ in default_poses(SCENE_LANES)]
        xs, ys = [p[0] for p in points], [p[1] for p in points]
        cy = (min(ys) + max(ys)) / 2
        n = (max(ys) - min(ys)) / ROBOT_SPACING + 1  # lanes covered
        state = ViewportCameraState("/OmniverseKit_Persp")
        state.set_position_world(Gf.Vec3d(min(xs) - 5.0, cy - ROBOT_SPACING * n * 0.9, 3.2), True)
        state.set_target_world(Gf.Vec3d(max(xs) + 3.0, cy, 0.3), True)
    except Exception as e:  # cosmetic only
        log(f"could not aim viewport camera: {e}")


def set_viewport_resolution():
    """FLEET_VIEWPORT_RES="1280x720": render the main viewport (the one that is streamed) at a fixed size.

    By default the streamed framebuffer follows the size of the WebRTC client window, and the viewport is path
    traced at that size, which is expensive on a large window.
    """
    res = os.environ.get("FLEET_VIEWPORT_RES", "").lower()
    if not res:
        return
    try:
        from omni.kit.viewport.utility import get_active_viewport

        w, h = (int(v) for v in res.split("x"))
        vp = get_active_viewport()
        vp.fill_frame = False
        vp.resolution = (w, h)
        log(f"viewport resolution set to {vp.resolution}, fill_frame={vp.fill_frame}")
    except Exception as e:
        log(f"could not set viewport resolution: {e}")


_looks = {"lib": None, "applied": set()}


def apply_robot_looks(models):
    """ROBOT_LOOKS=full|basic|0: rebind the robots' visuals to realistic materials (sim/scripts/robot_looks.py).
    Visual only, and best effort: on any error the robots keep the importer's materials. Runs at spawn time, once
    per model (the edits stay in the held in-memory layers, so a respawn reuses them); it only opens the model's
    own layers, never the scene's stage."""
    if not robot_looks.enabled():
        return
    try:
        if _looks["lib"] is None:
            _looks["lib"] = robot_looks.build_library()
        for model in dict.fromkeys(models):
            if model not in _looks["applied"]:
                robot_looks.apply(model, MODEL_ASSETS[model]["usd_path"], _looks["lib"])
                _looks["applied"].add(model)
    except Exception:
        log("robot looks failed, keeping the importer's materials:\n" + traceback.format_exc())


SNAPSHOT_VIEWS = {  # name: (camera offset from the robot, in m, robot frame; look-at height)
    "front": ((2.0, -1.3, 1.1), 0.35),
    "rear": ((-1.8, 1.4, 1.2), 0.35),
    "close": ((0.9, -0.7, 0.55), 0.25),
    "wide": ((-4.5, 3.0, 3.2), 0.3),  # high, behind and to the left: the robot in its surroundings
}


async def snapshot_loop(out=None, settle=True, poses=None, views=None, tag=None):
    """Aim the viewport at each robot from SNAPSHOT_VIEWS and save PNGs to `out`, then put the viewport back.
    FLEET_SNAPSHOT=<dir>: once after each spawn (settle: wait 8 s first), for comparing looks with photos. A
    FLEET_CONTROL {action: "snapshot"} request (scripts/fleet_ctl.py snapshot) runs it on demand: `poses`
    {ns: [x, y, yaw_deg]} is where each robot is now (from its ref_pose), since the chassis prim's USD transform can
    lag the simulation; offsets are in the robot's frame (rotated by yaw). Returns the files written."""
    out = out or os.environ.get("FLEET_SNAPSHOT", "")
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
    from omni.kit.viewport.utility.camera_state import ViewportCameraState

    app = omni.kit.app.get_app()
    os.makedirs(out, exist_ok=True)
    stage = omni.usd.get_context().get_stage()
    if settle:
        for _ in range(int(SIM_RATE_HZ * 8)):  # let the robots settle
            await app.next_update_async()
    vp = get_active_viewport()
    state = ViewportCameraState("/OmniverseKit_Persp")
    tag = tag or robot_looks.MODE
    files = []
    for ns, model in ROBOTS:
        if poses and ns not in poses:
            continue
        if poses:
            x, y, yaw = poses[ns]
        else:
            chassis = stage.GetPrimAtPath(chassis_prim(stage, f"/World/{ns}", MODEL_PARAMS[model]))
            p = fleet_pose.world_matrix(chassis.GetPath(), stage).ExtractTranslation()
            x, y, yaw = p[0], p[1], 0.0
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        for view, ((dx, dy, dz), tz) in SNAPSHOT_VIEWS.items():
            if views and view not in views:
                continue
            state.set_position_world(Gf.Vec3d(x + c * dx - s * dy, y + s * dx + c * dy, dz), True)
            state.set_target_world(Gf.Vec3d(x, y, tz), True)
            for _ in range(40):  # path-tracing accumulation
                await app.next_update_async()
            path = f"{out}/{ns}_{view}_{tag}.png"
            capture_viewport_to_file(vp, path)
            for _ in range(5):
                await app.next_update_async()
            log(f"snapshot {path}")
            files.append(path)
    aim_viewport(ROBOT_POSES)
    return files


async def snapshot_request(ctl):
    """FLEET_CONTROL {id, action: "snapshot", dir, poses, views, tag}: snapshot_loop now, answered in state.json
    "snapshot" ({id, state, files} / message on an error)."""
    write_state(snapshot={"id": ctl["id"], "state": "running", "time": time.time()})
    try:
        files = await snapshot_loop(out=ctl.get("dir") or "/sim/generated/snapshots", settle=False,
                                    poses=ctl.get("poses"), views=ctl.get("views"), tag=ctl.get("tag") or "now")
        write_state(snapshot={"id": ctl["id"], "state": "done", "files": files, "time": time.time()})
    except Exception as e:
        log("snapshot failed:\n" + traceback.format_exc())
        write_state(snapshot={"id": ctl["id"], "state": "error", "message": f"{type(e).__name__}: {e}"})


async def camera_request(ctl):
    """FLEET_CONTROL {id, action: "camera", eye: [x, y, z], target: [x, y, z]} (world frame, m): aim the viewport
    and keep that view across spawns and snapshots; eye null goes back to the default view. Answered in state.json
    "camera" {id, state, eye, target}."""
    global VIEW_OVERRIDE
    try:
        eye, target = ctl.get("eye"), ctl.get("target")
        if ctl.get("view") == "close":  # the recorder's second view: a camera riding with the cutter, its own viewport
            if _vbd_plant is None or eye is None:
                raise ValueError("the close-up view needs the VBD plant (VBD_PLANT) and an eye and target")
            op = close_view(ctl.get("res") or (960, 540))
            _vbd_plant.close = (tuple(float(v) for v in eye), tuple(float(v) for v in target), op)
            await omni.kit.app.get_app().next_update_async()
            log(f"camera {ctl.get('id')}: close-up view riding with the cutter from {eye} at {target} (CAD frame, mm)")
            write_state(camera={"id": ctl.get("id"), "state": "done", "eye": eye, "target": target, "view": "close"})
            return
        if _vbd_plant is not None:  # any camera request ends a follow; follow "cutter" starts one (vbd_plant.aim_follow)
            _vbd_plant.follow = None
        if ctl.get("follow") == "cutter":
            if _vbd_plant is None or eye is None:
                raise ValueError("follow cutter needs the VBD plant (VBD_PLANT) and an eye and target")
            _vbd_plant.follow = (tuple(float(v) for v in eye), tuple(float(v) for v in target))
            await omni.kit.app.get_app().next_update_async()
            log(f"camera {ctl.get('id')}: following the cutter from {eye} at {target} (CAD frame, mm)")
            write_state(camera={"id": ctl.get("id"), "state": "done", "eye": eye, "target": target, "follow": "cutter"})
            return
        VIEW_OVERRIDE = None if eye is None else (tuple(float(v) for v in eye), tuple(float(v) for v in target))
        aim_viewport(ROBOT_POSES)
        await omni.kit.app.get_app().next_update_async()
        log(f"camera {ctl.get('id')}: " + (f"eye {eye}, target {target}" if VIEW_OVERRIDE else "default view"))
        write_state(camera={"id": ctl.get("id"), "state": "done", "eye": eye, "target": target})
    except Exception as e:  # a bad request must not take the sim down
        log(f"camera {ctl.get('id')} failed: {e!r}")
        write_state(camera={"id": ctl.get("id"), "state": "error", "message": repr(e)})


_record = {"task": None, "stop": False}
_close = {"vp": None, "op": None}  # the close-up view (camera_request view "close"): its viewport, its camera's transform


def close_view(res):
    """A second viewport window on the VBD plant's close-up camera (vbd_plant.CLOSE_CAM, made with the plant before
    the first step so no prim appears mid-run), made once; record_loop saves it beside the main viewport. Returns the
    camera's transform op (vbd_plant.aim_follow aims it)."""
    if _close["vp"] is None:
        import vbd_plant
        from omni.kit.viewport.utility import create_viewport_window, get_active_viewport, get_active_viewport_window

        main, main_win = get_active_viewport(), get_active_viewport_window()
        win = create_viewport_window("Close-up", width=int(res[0]), height=int(res[1]), camera_path=vbd_plant.CLOSE_CAM)
        vp = win.viewport_api
        vp.fill_frame = False
        vp.resolution = (int(res[0]), int(res[1]))
        _close.update(vp=vp, win=win, main=main)
        if main_win is not None:  # the new window took the focus, and with it "the active viewport" (record_loop's)
            main_win.focus()
    return _vbd_plant.close_op


async def record_loop(out, fps, res):
    """Save the viewport every 1/fps s (wall clock) as out/frame_%06d.jpg, with each frame's wall and sim time in
    out/frames.csv, until a stop request. It captures the render itself, so it works headless or on a locked screen."""
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    app = omni.kit.app.get_app()
    tl = omni.timeline.get_timeline_interface()
    vp = _close.get("main") or get_active_viewport()  # not the close-up's, which is saved beside it
    old = (vp.fill_frame, vp.resolution)
    if res:
        vp.fill_frame = False
        vp.resolution = tuple(res)
    os.makedirs(out, exist_ok=True)
    n, t0 = 0, time.time()
    with open(f"{out}/frames.csv", "a") as times:
        times.write("frame,wall,sim\n")
        try:
            while not _record["stop"]:
                if time.time() >= t0 + n / fps:
                    capture_viewport_to_file(vp, f"{out}/frame_{n:06d}.jpg")
                    if _close["vp"] is not None:  # the close-up view, same frame number
                        capture_viewport_to_file(_close["vp"], f"{out}/close_{n:06d}.jpg")
                    times.write(f"{n},{time.time():.3f},{tl.get_current_time():.3f}\n")
                    n += 1
                    if n % (fps * 60) == 0:
                        times.flush()
                        write_state(record={"state": "recording", "dir": out, "frames": n})
                await app.next_update_async()
        finally:
            for _ in range(5):  # the last capture is written on a later frame
                await app.next_update_async()
            if res:
                vp.fill_frame, vp.resolution = old
            write_state(record={"state": "stopped", "dir": out, "frames": n, "seconds": round(time.time() - t0)})
            log(f"record: stopped, {n} frame(s) in {out}")


async def record_request(ctl):
    """FLEET_CONTROL {id, action: "record", dir, fps, res: [w, h]} starts record_loop (a running one is stopped
    first); {id, action: "record", stop: true} stops it. Answered in state.json "record"."""
    task = _record["task"]
    if task and not task.done():
        _record["stop"] = True
        await task
    _record.update(task=None, stop=False)
    if ctl.get("stop"):
        return
    out, fps = ctl.get("dir") or "/sim/generated/recordings/now", int(ctl.get("fps") or 10)
    log(f"record {ctl.get('id')}: {fps} fps into {out}" + (f" at {ctl['res']}" if ctl.get("res") else ""))
    write_state(record={"id": ctl.get("id"), "state": "recording", "dir": out, "frames": 0})
    _record["task"] = asyncio.ensure_future(record_loop(out, fps, ctl.get("res")))


SUN_SITE = {"lat": 47.12, "lon": -88.55, "utc_offset": -4.0, "day": 172}  # Houghton, MI (MTU), EDT, June 21
_sun = {"task": None, "op": None}


def sun_position(hour, lat, lon, utc_offset, day):
    """Solar azimuth (from north, clockwise) and elevation, radians, at local clock time `hour` on day-of-year `day`
    (NOAA's approximation, within ~1 deg)."""
    g = 2 * math.pi / 365 * (day - 1 + (hour - utc_offset - 12) / 24)
    eqtime = 229.18 * (0.000075 + 0.001868 * math.cos(g) - 0.032077 * math.sin(g)
                       - 0.014615 * math.cos(2 * g) - 0.040849 * math.sin(2 * g))
    decl = (0.006918 - 0.399912 * math.cos(g) + 0.070257 * math.sin(g) - 0.006758 * math.cos(2 * g)
            + 0.000907 * math.sin(2 * g) - 0.002697 * math.cos(3 * g) + 0.00148 * math.sin(3 * g))
    ha = math.radians((hour * 60 + eqtime + 4 * lon - 60 * utc_offset) / 4 - 180)
    phi = math.radians(lat)
    el = math.asin(math.sin(phi) * math.sin(decl) + math.cos(phi) * math.cos(decl) * math.cos(ha))
    az = math.atan2(math.sin(ha), math.cos(ha) * math.sin(phi) - math.tan(decl) * math.cos(phi)) + math.pi
    return az % (2 * math.pi), el


def set_sun(hour, site):
    """Point the scene's sun (/World/Lights/sun, else its first DistantLight) at the solar position of `hour`. World
    +x is east, +y north (the GPS datum's). The light's xform becomes one orient op (its -Z is the light's
    direction). Returns (azimuth, elevation) in degrees."""
    op = _sun["op"]
    if op is None or not op.GetAttr().GetPrim().IsValid():
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath("/World/Lights/sun")
        if not prim.IsValid():
            prim = next((p for p in stage.Traverse() if p.IsA(UsdLux.DistantLight)), None)
            if prim is None:
                raise RuntimeError("the scene has no DistantLight")
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        op = _sun["op"] = xf.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
    az, el = sun_position(hour, **site)
    toward_sun = Gf.Vec3d(math.cos(el) * math.sin(az), math.cos(el) * math.cos(az), math.sin(el))
    op.Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), toward_sun).GetQuat()))
    return round(math.degrees(az), 1), round(math.degrees(el), 1)


async def sun_loop(start, end, duration, site, cid=None):
    """Move the sun from local time `start` to `end` (hours) over `duration` wall-clock seconds, once a second; it
    then stays at `end`."""
    app = omni.kit.app.get_app()
    t0, last, logged = time.time(), 0.0, 0.0
    while True:
        now = time.time()
        if now - last >= 1.0:
            last = now
            f = min(1.0, (now - t0) / duration)
            hour = start + (end - start) * f
            az, el = set_sun(hour, site)
            if now - logged >= 60 or f >= 1.0:
                logged = now
                write_state(sun={"id": cid, "state": "moving" if f < 1.0 else "done", "hour": round(hour, 3),
                                 "azimuth": az, "elevation": el, "start": start, "end": end, "duration": duration})
            if f >= 1.0:
                log(f"sun: reached {end:.2f} h (azimuth {az}, elevation {el} deg)")
                return
        await app.next_update_async()


async def sun_request(ctl):
    """FLEET_CONTROL {id, action: "sun", hour} sets the sun to a local time of day; {id, action: "sun", start, end,
    duration} moves it from start to end over duration wall-clock seconds (a running move is replaced). Optional
    lat, lon, utc_offset, day (day of year) override SUN_SITE. Answered in state.json "sun"."""
    task = _sun["task"]
    if task and not task.done():
        task.cancel()
    _sun["task"] = None
    site = {k: float(ctl[k]) if ctl.get(k) is not None else v for k, v in SUN_SITE.items()}
    try:
        if ctl.get("hour") is not None:
            az, el = set_sun(float(ctl["hour"]), site)
            write_state(sun={"id": ctl.get("id"), "state": "done", "hour": float(ctl["hour"]), "azimuth": az,
                             "elevation": el})
            log(f"sun {ctl.get('id')}: {float(ctl['hour']):.2f} h, azimuth {az}, elevation {el} deg")
        else:
            start, end, duration = float(ctl["start"]), float(ctl["end"]), max(1.0, float(ctl["duration"]))
            log(f"sun {ctl.get('id')}: {start:.2f} h -> {end:.2f} h over {duration:.0f} s")
            write_state(sun={"id": ctl.get("id"), "state": "moving", "hour": start, "start": start, "end": end,
                             "duration": duration})
            _sun["task"] = asyncio.ensure_future(sun_loop(start, end, duration, site, ctl.get("id")))
    except Exception as e:  # a bad request must not take the sim down
        log(f"sun {ctl.get('id')} failed: {e!r}")
        write_state(sun={"id": ctl.get("id"), "state": "error", "message": repr(e)})


CLOUDS_PATH = "/World/Clouds"
CLOUDS = {"cover": 0.4, "speed": 2.0, "heading": 70.0, "altitude": 300.0, "seed": 1, "tile": 600.0, "opacity": 0.85}
CLOUDS_SIZE = 6000.0  # m: wide enough that a low sun's ray to the field still crosses the layer
CLOUDS_DIR = "/sim/generated/clouds"
_clouds = {"task": None}


def cloud_texture(cover, seed, opacity, n=1024):
    """A tileable cloud layer: RGBA PNG, white, alpha = cloud density (fractal noise from a 1/f spectrum, thresholded
    so `cover` of the area is cloud, soft edges). Cached by its parameters in CLOUDS_DIR; returns the path."""
    import numpy as np
    from PIL import Image
    path = f"{CLOUDS_DIR}/clouds_s{seed}_c{cover:.2f}_o{opacity:.2f}.png"
    if os.path.exists(path):
        return path
    rng = np.random.default_rng(seed)
    f = np.fft.fftfreq(n) * n
    k = np.hypot(*np.meshgrid(f, f))
    amp = np.where(k < 2, 0.0, 1.0 / np.maximum(k, 1.0) ** 1.6)  # features from a quarter tile down, none bigger
    noise = np.fft.ifft2(amp * np.exp(2j * np.pi * rng.random((n, n)))).real  # periodic, so the tile repeats seamlessly
    noise = (noise - noise.mean()) / noise.std()
    thr = np.quantile(noise, 1.0 - cover)
    a = np.clip((noise - thr) / 0.6, 0.0, 1.0)
    a = a * a * (3 - 2 * a) * opacity  # smoothstep edges; a cloud's core lets 1 - opacity of the sun through
    rgba = np.empty((n, n, 4), np.uint8)
    rgba[..., :3] = 255
    rgba[..., 3] = (a * 255).astype(np.uint8)
    os.makedirs(CLOUDS_DIR, exist_ok=True)
    Image.fromarray(rgba, "RGBA").save(path)
    return path


def make_clouds(stage, p):
    """(Re)build CLOUDS_PATH: one horizontal quad CLOUDS_SIZE wide at `altitude`, the cloud texture repeated every
    `tile` m (OmniPBR with fractional cutout opacity). Its shadows are the clouds' shadows on the field; the robot
    cameras' far clip (CAM_FAR_CLIP) leaves it out of their image. Returns the translate op that drifts it."""
    tex = cloud_texture(p["cover"], int(p["seed"]), p["opacity"])
    carb.settings.get_settings().set("/rtx/raytracing/fractionalCutoutOpacity", True)
    if stage.GetPrimAtPath(CLOUDS_PATH).IsValid():
        stage.RemovePrim(CLOUDS_PATH)
    root = UsdGeom.Xform.Define(stage, CLOUDS_PATH)
    op = root.AddTranslateOp()
    h, s, t = p["altitude"], CLOUDS_SIZE / 2, p["tile"]
    mesh = UsdGeom.Mesh.Define(stage, f"{CLOUDS_PATH}/layer")
    mesh.CreatePointsAttr([Gf.Vec3f(-s, -s, h), Gf.Vec3f(s, -s, h), Gf.Vec3f(s, s, h), Gf.Vec3f(-s, s, h)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateDoubleSidedAttr(True)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([Gf.Vec2f(-s / t, -s / t), Gf.Vec2f(s / t, -s / t), Gf.Vec2f(s / t, s / t), Gf.Vec2f(-s / t, s / t)])
    mat = UsdShade.Material.Define(stage, f"{CLOUDS_PATH}/material")
    mdl = UsdShade.Shader.Define(stage, f"{CLOUDS_PATH}/material/omnipbr")
    mdl.SetSourceAsset("OmniPBR.mdl", "mdl")
    mdl.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    mdl.CreateIdAttr("")
    mdl.GetPrim().GetAttribute("info:implementationSource").Set("sourceAsset")
    mdl.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.9, 0.9, 0.92))
    mdl.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
    mdl.CreateInput("enable_opacity_texture", Sdf.ValueTypeNames.Bool).Set(True)
    mdl.CreateInput("opacity_texture", Sdf.ValueTypeNames.Asset).Set(tex)
    mdl.GetInput("opacity_texture").GetAttr().SetColorSpace("raw")
    mdl.CreateInput("opacity_mode", Sdf.ValueTypeNames.Int).Set(0)  # mono_alpha
    mdl.CreateInput("opacity_threshold", Sdf.ValueTypeNames.Float).Set(0.0)
    mdl.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(mdl.ConnectableAPI(), "out")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return op


async def clouds_loop(op, p):
    """Drift the cloud layer at `speed` m/s toward compass `heading` (wall clock), 10 times a second; the offset wraps
    at one texture tile, which looks the same."""
    app = omni.kit.app.get_app()
    a = math.radians(p["heading"])
    vx, vy, t = p["speed"] * math.sin(a), p["speed"] * math.cos(a), p["tile"]
    t0, last = time.time(), 0.0
    while True:
        now = time.time()
        if now - last >= 0.1 and op.GetAttr().GetPrim().IsValid():
            last = now
            op.Set(Gf.Vec3d((vx * (now - t0)) % t, (vy * (now - t0)) % t, 0.0))
        await app.next_update_async()


async def clouds_request(ctl):
    """FLEET_CONTROL {id, action: "clouds", cover, speed, heading, altitude, seed, tile, opacity} (each optional,
    defaults CLOUDS) puts a drifting cloud layer over the scene, replacing one already there; cover 0 removes it.
    Answered in state.json "clouds"."""
    task = _clouds["task"]
    if task and not task.done():
        task.cancel()
    _clouds["task"] = None
    p = {k: float(ctl[k]) if ctl.get(k) is not None else v for k, v in CLOUDS.items()}
    try:
        stage = omni.usd.get_context().get_stage()
        if p["cover"] <= 0:
            if stage.GetPrimAtPath(CLOUDS_PATH).IsValid():
                stage.RemovePrim(CLOUDS_PATH)
            write_state(clouds={"id": ctl.get("id"), "state": "off"})
            log(f"clouds {ctl.get('id')}: off")
            return
        op = make_clouds(stage, p)
        _clouds["task"] = asyncio.ensure_future(clouds_loop(op, p))
        write_state(clouds={"id": ctl.get("id"), "state": "on", **p})
        log(f"clouds {ctl.get('id')}: cover {p['cover']:.2f}, {p['speed']:.1f} m/s toward {p['heading']:.0f} deg, "
            f"{p['altitude']:.0f} m up")
    except Exception as e:  # a bad request must not take the sim down
        log(f"clouds {ctl.get('id')} failed: {e!r}")
        write_state(clouds={"id": ctl.get("id"), "state": "error", "message": repr(e)})


async def debug_loop(og):
    """FLEET_DEBUG=1: log the command chain and chassis pose of the first robot every 2 s."""
    from pxr import UsdGeom as _G
    ns, model = ROBOTS[0]
    stage = omni.usd.get_context().get_stage()
    chassis = stage.GetPrimAtPath(chassis_prim(stage, f"/World/{ns}", MODEL_PARAMS[model]))
    app = omni.kit.app.get_app()
    bbox = _G.BBoxCache(0, [_G.Tokens.default_, _G.Tokens.guide, _G.Tokens.proxy, _G.Tokens.render])
    lows = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(f"/World/{ns}")):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            r = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
            if not r.IsEmpty():
                lows.append((round(r.GetMin()[2], 3), round(r.GetMax()[2], 3), prim.GetTypeName(), str(prim.GetPath()).split(ns + "/")[-1][-90:]))
    for row in sorted(lows)[:12]:
        log(f"debug lowest collider z: {row}")
    tl = omni.timeline.get_timeline_interface()
    w0, s0 = time.time(), tl.get_current_time()
    newton_dumped = False
    if PHYSICS_ENGINE == "newton":  # an early look (the first steps) before the 2 s cadence below
        import fleet_newton
        for _ in range(12):
            await app.next_update_async()
        log("debug newton dump (early):\n" + fleet_newton.debug_report(limit=70))
    while True:
        n_frames = 120
        for _ in range(n_frames):
            await app.next_update_async()
        w1, s1 = time.time(), tl.get_current_time()
        log(f"debug rtf={(s1 - s0) / (w1 - w0):.2f} render_fps={n_frames / (w1 - w0):.1f}")
        if PHYSICS_ENGINE == "newton" and not newton_dumped:
            import fleet_newton
            report = fleet_newton.debug_report(limit=70)
            newton_dumped = not report.startswith("newton: no model")
            log("debug newton dump:\n" + report)
        w0, s0 = w1, s1
        g = f"/Graphs/{ns}"
        try:
            vals = {k: og.Controller.attribute(f"{g}/{k}").get() for k in (
                "CmdVel.outputs:linearVelocity", "CmdVel.outputs:angularVelocity", "Diff.outputs:velocityCommand")}
        except Exception as e:
            vals = f"attr read failed: {e}"
        if not chassis.IsValid():  # robots were respawned
            log("debug: first robot gone (respawned), debug pose logging stopped")
            return
        m = fleet_pose.world_matrix(chassis.GetPath())
        pos, yaw = m.ExtractTranslation(), math.degrees(math.atan2(m[0][1], m[0][0]))
        extra = ""
        if PHYSICS_ENGINE == "newton":  # Newton's own pose of the chassis body, to check world_matrix against
            import fleet_newton
            nst = fleet_newton.newton_stage()
            if nst is not None:
                try:
                    i = nst.model.body_label.index(str(chassis.GetPath()))
                    q = nst.state_0.body_q.numpy()[i]
                    extra = (f" newton_body=({q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}) yaw="
                             f"{math.degrees(math.atan2(2 * (q[6] * q[5] + q[3] * q[4]), 1 - 2 * (q[4] * q[4] + q[5] * q[5]))):.1f}")
                except (ValueError, IndexError) as e:
                    extra = f" newton_body=? ({e})"
        log(f"debug {ns}: chassis_pos={tuple(round(v, 3) for v in pos)} yaw={yaw:.1f}{extra} {vals}")


def _robot_camera_textures():
    """{render product path: (HydraTexture, [topics])} of the robot cameras (camera prim under /World/<ns>/, not the
    viewport's). IsaacCreateRenderProduct makes them with rep.create.render_product, which registers them in
    replicator's viewport manager; looked up in every context of it (attach_hydra_texture's lookup uses another
    context name). Topics as build_ros_graph names them: /<ns>/sensors/camera_<i>/<stream>/{image,camera_info}."""
    import omni.replicator.core as rep

    out = {}
    for textures in rep.vp_manager.ViewportManager()._hydra_textures._hydra_textures.values():
        for t in textures:
            ht = t.hydra_texture
            cam = str(ht.get_camera_path()) if ht is not None else ""
            idx = re.findall(r"camera_(\d+)_", cam)
            if not cam.startswith("/World/") or not idx:
                continue
            base = f"/{cam.split('/')[2]}/sensors/camera_{idx[-1]}"
            out[str(ht.get_render_product_path())] = (
                ht, [f"{base}/{s}/{k}" for s in sorted(CAM_STREAMS) for k in ("image", "camera_info")])
    return out


async def camera_on_demand_loop():
    """CAMERA_ON_DEMAND=1: a robot camera's render product updates (hydra texture updates on) only while one of its
    topics has a subscriber, checked every 0.5 s and kept on for CAM_IDLE_OFF_S after the last one leaves.
    Measured with j100_0921 alone (2 cameras, FLEET_DEBUG render_fps): both rendering 19.4, camera_1 off 24.2.
    Switching only when a subscriber comes or goes is the point: toggling the updates often costs more than it
    saves (alternating the cameras every frame: 9.5 fps and no images at all; in blocks of 5 frames: 18.6 fps).
    All cameras are switched back on while the timeline is stopped (spawn/reset); the list is refreshed every 2 s."""
    import rclpy

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("fleet_camera_on_demand")
    app = omni.kit.app.get_app()
    tl = omni.timeline.get_timeline_interface()
    textures, on, last_used, next_check, refresh = {}, {}, {}, 0.0, 0.0
    while True:
        await app.next_update_async()
        if not tl.is_playing():
            for ht, _ in textures.values():
                ht.set_updates_enabled(True)
            textures, on, last_used, refresh = {}, {}, {}, 0.0
            continue
        now = time.time()
        if now < next_check:
            continue
        next_check = now + 0.5
        if now >= refresh:
            refresh = now + 2.0
            try:
                found = _robot_camera_textures()
            except Exception as e:
                log(f"camera on demand: render products not readable ({type(e).__name__}: {e}), all cameras stay on")
                for ht, _ in textures.values():
                    ht.set_updates_enabled(True)
                node.destroy_node()
                return
            if found.keys() != textures.keys():
                log(f"camera on demand: {len(found)} camera(s), each rendered only while its topics have a subscriber")
            textures = found
            for rp in found:  # warm-up: a camera that never rendered has no ROS publisher, so `ros2 topic
                last_used.setdefault(rp, now + CAM_WARMUP_S - CAM_IDLE_OFF_S)  # hz`/Foxglove can't subscribe
        for rp, (ht, topics) in textures.items():
            try:
                if any(node.count_subscribers(t) for t in topics):
                    last_used[rp] = now
            except Exception:
                last_used[rp] = now  # can't tell: keep it on
            want = now - last_used.get(rp, -1e9) < CAM_IDLE_OFF_S
            if on.get(rp) != want:
                ht.set_updates_enabled(want)
                on[rp] = want
                log(f"camera on demand: {topics[0].rsplit('/', 2)[0]} {'on' if want else 'off'}")


# ---------------------------------------------------------------- spawn protocol (see FLEET_REQUEST/FLEET_STATE)

MAX_ROBOTS = 8  # docker-compose.yml's robot0..robot7 slots
_state = {
    "started": time.time(),  # the host compares this with the container's start time to ignore a stale file
    "scene": "loading",  # loading | ready | error
    "error": "",
    "lanes": SCENE_LANES,
    "scene_source": SIM_SCENE or "default",  # see build_world
    "physics": PHYSICS_ENGINE,  # the engine running (select_physics_engine); "physics_requested" = SIM_PHYSICS
    "physics_requested": PHYSICS_REQUESTED,
    "default_poses": {str(n): default_poses(n) for n in range(MAX_ROBOTS + 1)},
    "models": [],  # models that can be spawned (imported)
    "robots": [],  # robots in the scene: {slot, ns, model, x, y, yaw}
    "spawn": None,  # last request handled: {id, state: spawning | done | error, message}
    "boot": "pending",  # "done" once the requests found at start (see spawn_loop) are spawned or rejected
}


def _write_json(path, data):
    """Atomic write into FLEET_DIR, readable/deletable by the host user."""
    try:
        os.makedirs(FLEET_DIR, exist_ok=True)
        try:
            os.chmod(FLEET_DIR, 0o777)  # the host user writes requests here
        except OSError:
            pass
        tmp = f"{path}.{os.getpid()}.tmp"
        with open(tmp, "w") as f:
            json.dump(data, f, indent=1)
        os.chmod(tmp, 0o666)
        os.replace(tmp, path)
    except OSError as e:
        log(f"could not write {path}: {e}")


def write_state(**updates):
    """Rewrite FLEET_STATE (read by scripts/fleet_ctl.py and the web UI)."""
    _state.update(updates, updated=time.time())
    _write_json(FLEET_STATE, _state)


def parse_request(req):
    """FLEET_REQUEST -> [(ns, model, (x, y, yaw_deg))]. Raises ValueError with a message for the user."""
    entries = req.get("robots")
    if not isinstance(entries, list) or len(entries) > MAX_ROBOTS:
        raise ValueError(f"'robots' must be a list of at most {MAX_ROBOTS} entries")
    defaults = default_poses(len(entries))
    robots = []
    for slot, e in enumerate(entries):
        model = e.get("model") if isinstance(e, dict) else None
        if model not in _state["models"]:
            raise ValueError(f"slot {slot}: model {model!r} is not available (have: {', '.join(_state['models'])})")
        stamp, cached = import_stamp(model)
        if stamp != cached:
            raise ValueError(f"slot {slot}: {model}'s URDF changed after the sim started (scripts/gen_urdf.sh?); "
                             "the importer can't run with the scene open -- restart the sim (Reset scene) to re-import it")
        pose = []
        for k, d in zip(("x", "y", "yaw"), defaults[slot]):
            v = e.get(k)
            try:
                v = d if v is None or v == "" else float(v)
            except (TypeError, ValueError):
                raise ValueError(f"slot {slot}: {k} must be a number, got {v!r}")
            if not math.isfinite(v):
                raise ValueError(f"slot {slot}: {k} must be finite")
            pose.append(v)
        if abs(pose[0]) > SPAWN_LIMIT or abs(pose[1]) > SPAWN_LIMIT:
            raise ValueError(f"slot {slot}: x and y must be within +-{SPAWN_LIMIT:g} m (the ground)")
        robots.append((robot_namespace(slot, model), model, tuple(pose)))
    names = [ns for ns, _, _ in robots]
    for ns in set(names):
        if names.count(ns) > 1:
            raise ValueError(f"{ns} is requested twice (a real robot id can only be in one slot)")
    for i in range(len(robots)):
        for j in range(i):
            (a, _, pa), (b, _, pb) = robots[i], robots[j]
            if math.hypot(pa[0] - pb[0], pa[1] - pb[1]) < SPAWN_MIN_DIST:
                raise ValueError(f"{a} and {b} are less than {SPAWN_MIN_DIST:g} m apart")
    return robots


async def spawn_one(app, stage, og, usdrt_sdf, ns, model, pose):
    params = MODEL_PARAMS[model]
    root = spawn_robot(stage, ns, model, pose)
    cam_path = None
    if params.get("has_camera", True):
        optical_link = params.get("camera_optical_link")
        hfov = ZED_HFOV_DEG if optical_link else HFOV_DEG
        if not optical_link:
            # RealSense with nominal extrinsics (robot/Dockerfile): the URDF has its own optical frame, which the
            # robot's robot_state_publisher already publishes, so mount the camera there like the ZED path does.
            try:
                find_prim(stage, root, "camera_0_color_optical_frame")
                optical_link = "camera_0_color_optical_frame"
            except RuntimeError:
                pass  # older asset without the nominal frames: add_camera builds one under camera_0_link
        # build_ros_graph: no PubTfCamera for a URDF frame; the depth range from the sensor, before that override
        params = dict(params, camera_optical_link=optical_link,
                      depth_range=DEPTH_RANGE_M["zed2i" if params.get("camera_optical_link") else "d435i"])
        cam_path = add_camera(stage, root, optical_link=optical_link, hfov_deg=hfov)

    log(f"spawned {ns} ({model}) at {root}, x={pose[0]:g} y={pose[1]:g} yaw={pose[2]:g} deg")
    for _ in range(3):
        await app.next_update_async()
    chassis = chassis_prim(stage, root, params)
    if PHYSICS_ENGINE == "physx":  # PhysxSchema settings; Newton reads only the UsdPhysics ones (masses, materials)
        enable_wheel_ccd(stage, root)
    else:
        # Newton's importer enables an articulation's self-collisions unless the stage says otherwise (its
        # SchemaResolverNewton reads newton:selfCollisionEnabled, default True; PhysX articulations never
        # self-collide unless asked). The URDF importer already authors it false on the root; set again as a guard
        # for assets imported by an older importer.
        stage.GetPrimAtPath(chassis).CreateAttribute("newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool).Set(False)
        cap = float(params.get("wheel_max_force", NEWTON_WHEEL_MAX_FORCE))
        damping = params.get("wheel_damping")
        log(f"{ns}: newton:selfCollisionEnabled = false on {chassis}; wheel drives capped at "
            f"{cap} N*m on {cap_wheel_drives(stage, root, cap, damping)} joint(s)"
            + (f", damping {damping} N*m*s/rad" if damping is not None else ""))
    if PHYSICS_ENGINE == "newton" and NEWTON_SOLVER == "vbd" and params.get("vbd_wheel_contact"):
        ke_kd = tuple(float(v) for v in params["vbd_wheel_contact"])
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
            if "wheel" in prim.GetName().lower() and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                VBD_WHEEL_CONTACT[str(prim.GetPath())] = ke_kd
        log(f"{ns}: VBD wheel contact ke {ke_kd[0]:g} N/m, kd {ke_kd[1]:g} N*s/m")
    if params.get("wheel_friction"):
        log(f"{ns}: wheel friction {params['wheel_friction']} on "
            f"{apply_wheel_friction(stage, root, ns, params['wheel_friction'])} wheel(s)")
    if PHYSICS_ENGINE == "physx" and (ARTIC_POS_ITERS or ARTIC_VEL_ITERS):
        log(f"{ns}: solver iterations pos={ARTIC_POS_ITERS} vel={ARTIC_VEL_ITERS} on "
            f"{set_articulation_iterations(stage, root)} articulation root(s)")
    if params.get("massless_density"):
        nd, nf = fix_massless_bodies(stage, root, params["massless_density"], params.get("frame_mass", 0.02))
        log(f"{ns}: massless bodies: {nd} at {params['massless_density']} kg/m^3, {nf} frames")
    cam1_path = None
    if params.get("wrist_camera"):
        cam1_path = add_camera(stage, root, hfov_deg=D405_HFOV_DEG, index=1)
    build_ros_graph(og, usdrt_sdf, stage, root, chassis, ns, cam_path, params, cam1_path)
    if params.get("has_arm"):
        configure_arm_drives(stage, root, params.get("drop_mimic_constraints", True))
    if params.get("end_effector") == "ee_cutter":
        vbd = PHYSICS_ENGINE == "newton" and NEWTON_SOLVER == "vbd"
        n, (ee, walls, pairs) = ee_cutter.attach(stage, find_prim(stage, root, "arm_0_gripper_gripper_base_link"),
                                                  PHYSICS_ENGINE, root,
                                                  finger_gains=(VBD_SCENE_GAINS.get("finger") or
                                                                params.get("vbd_finger_gains")) if vbd else None,
                                                  blade_gains=(VBD_SCENE_GAINS.get("blade") or
                                                               params.get("vbd_blade_gains")) if vbd else None)
        log(f"{ns}: end effector ee_cutter, {n} joints (2F Lite fingers hidden); self-collisions on, only its {ee} "
            f"colliders vs the basket's {walls} ({pairs} pairs filtered)")
        cutting = ee_cutter.cut_stems(stage, find_prim(stage, root, "arm_0_gripper_gripper_base_link"), root,
                                      PHYSICS_ENGINE, log)
        if cutting:
            log(f"{ns}: {cutting}")
    if params.get("arm_base_mount"):  # Kinova's base mount kit (make_kinova_mount.py), visual only, standing on the plate
        mount = UsdGeom.Xform.Define(stage, find_prim(stage, root, "arm_0_base_link") + "/kinova_base_mount")
        mount.GetPrim().GetReferences().AddReference(KINOVA_MOUNT_USD)
        mount.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -params["arm_base_mount"]))
        log(f"{ns}: Kinova base mount under arm_0_base_link, {params['arm_base_mount'] * 1000:g} mm to the plate")
    # last: ee_cutter.attach deactivates ee_cutter_link's URDF geometry, leaving it collider-less
    if PHYSICS_ENGINE == "newton" and NEWTON_SOLVER == "vbd":
        log(f"{ns}: VBD mass floor: {vbd_mass_floor(stage, root, params.get('frame_mass', 0.02))} collider-less "
            f"bodies set to {params.get('frame_mass', 0.02)} kg")


async def spawn_fleet(app, stage, og, usdrt_sdf, robots):
    """Replace the robots in the scene with `robots` ([(ns, model, pose)]). The timeline is stopped meanwhile:
    PhysX only takes masses (fix_massless_bodies) and new articulations cleanly when the simulation (re)starts."""
    import omni.kit.commands

    tl = omni.timeline.get_timeline_interface()
    tl.stop()
    for _ in range(3):
        await app.next_update_async()
    try:
        old = [p for ns, _ in ROBOTS for p in (f"/Graphs/{ns}", f"/World/{ns}") if stage.GetPrimAtPath(p)]
        if old:
            # DeletePrims also tears down the OmniGraphs and their C++ ROS publishers, but not the rclpy nodes the
            # ScriptNodes made (their cleanup() isn't called): those are destroyed through fleet_nodes.
            omni.kit.commands.execute("DeletePrims", paths=old)
            n_nodes = sum(fleet_nodes.destroy(ns) for ns, _ in ROBOTS)
            log(f"removed {', '.join(ns for ns, _ in ROBOTS)} ({n_nodes} script rclpy node(s) destroyed)")
        ROBOTS.clear()
        ROBOT_POSES.clear()
        fleet_pose.reset()  # cached body anchors of the removed robots (the same paths come back)
        for _ in range(3):
            await app.next_update_async()
        apply_robot_looks(model for _, model, _ in robots)
        for ns, model, pose in robots:
            # tracked first: a spawn that fails partway leaves its prims, which the next spawn must remove
            ROBOTS.append((ns, model))
            ROBOT_POSES.append(pose)
            await spawn_one(app, stage, og, usdrt_sdf, ns, model, pose)
        aim_viewport(ROBOT_POSES)
        for _ in range(10):
            await app.next_update_async()
    finally:
        tl.play()


def _read_json(path):
    try:
        with open(path) as f:
            data = json.load(f)
        return data if isinstance(data, dict) else None
    except (OSError, ValueError):  # none (or being replaced)
        return None


# Harvested plants: FLEET_CONTROL {id, action: "plants", hide: [object_id...], show: [object_id...] | "all"} (scripts/
# fleet_ctl.py plants, sent by the web UI's operation page from the farm database's harvest_state) marks those plants
# harvested / not harvested (set_harvested). Visibility only, authored on the root layer (the scene file is never
# written), so colliders stay: the crown core, and the foliage's too (PhysX ignores visibility). Lost on a sim restart
# (state.json's plants_hidden starts empty again): the host replays what the database says.
PLANT_PRIMS = {}  # object_id -> Sdf.Path, indexed once the scene is built (index_plants)
HIDDEN_PLANTS = set()


def set_harvested(stage, path, harvested):
    """A farm plant with `ready`/`harvested` children (build_farm_scene.py) shows the one that applies; any other
    plant is hidden or shown. Not a swap of the instance's prototype: that turned every RTX frame black."""
    prim = stage.GetPrimAtPath(path)
    cut = prim.GetChild("harvested")
    shown = [(prim.GetChild("ready"), not harvested), (cut, harvested)] if cut else [(prim, not harvested)]
    for p, visible in shown:
        img = UsdGeom.Imageable(p)
        img.MakeVisible() if visible else img.MakeInvisible()


def index_plants(stage):
    """object_id -> prim path of every lavender plant under /World/lavender (instances, customData object_id from
    build_farm_scene.py; the built-in lavender scene's plant_<k> prims by their number)."""
    found = {}
    root = stage.GetPrimAtPath("/World/lavender")
    if not root:
        return found
    it = iter(Usd.PrimRange(root))
    for prim in it:
        if prim == root:
            continue
        oid = prim.GetCustomDataByKey("object_id")
        if oid is None and prim.IsInstance() and prim.GetName().startswith("plant_"):
            oid = prim.GetName()[6:]
        if oid is not None:
            it.PruneChildren()
            try:
                found[int(oid)] = prim.GetPath()
            except (TypeError, ValueError):
                pass
        elif prim.IsInstance():
            it.PruneChildren()
    return found


async def plants_request(stage, ctl):
    """FLEET_CONTROL {id, action: "plants", hide, show}: answered in state.json as `plants` {id, state, hidden, shown,
    unknown}, plus `plants_hidden` (every object_id hidden now). Idempotent."""
    answer = {"id": ctl.get("id"), "state": "done", "hidden": 0, "shown": 0, "unknown": []}
    try:
        show = ctl.get("show") or []
        ids_show = sorted(HIDDEN_PLANTS) if show == "all" else [int(i) for i in show]
        ids_hide = [int(i) for i in (ctl.get("hide") or [])]
        for oid in ids_show:
            path = PLANT_PRIMS.get(oid)
            if path is None:
                answer["unknown"].append(oid)
            elif oid in HIDDEN_PLANTS:
                set_harvested(stage, path, False)
                HIDDEN_PLANTS.discard(oid)
                answer["shown"] += 1
        for oid in ids_hide:
            path = PLANT_PRIMS.get(oid)
            if path is None:
                answer["unknown"].append(oid)
            elif oid not in HIDDEN_PLANTS:
                set_harvested(stage, path, True)
                HIDDEN_PLANTS.add(oid)
                answer["hidden"] += 1
        if answer["hidden"] or answer["shown"]:
            await omni.kit.app.get_app().next_update_async()
        log(f"plants {ctl.get('id')}: hid {answer['hidden']}, showed {answer['shown']}, "
            f"{len(HIDDEN_PLANTS)} hidden now" + (f", unknown {answer['unknown'][:10]}" if answer["unknown"] else ""))
    except Exception as e:  # a bad request must not take the sim down
        log(f"plants {ctl.get('id')} failed: {e!r}")
        answer.update(state="error", message=repr(e))
    write_state(plants=answer, plants_hidden=sorted(HIDDEN_PLANTS))


async def reset_timeline(app, ctl_id):
    """Stop and play the timeline (Isaac's Stop/Play buttons): physics and the stage go back to their authored
    state, i.e. every robot at its spawn pose with its joints at their initial positions. Seconds, where restarting
    the sim container takes ~1 min."""
    write_state(reset={"id": ctl_id, "state": "running", "time": time.time()})
    tl = omni.timeline.get_timeline_interface()
    tl.stop()
    for _ in range(5):
        await app.next_update_async()
    tl.play()
    for _ in range(5):
        await app.next_update_async()
    log(f"reset: timeline stopped and played again, {len(ROBOTS)} robot(s) back at their spawn state")
    write_state(reset={"id": ctl_id, "state": "done", "time": time.time()})


def current_stage(stage):
    """The stage open in the USD context and whether it differs from `stage` (logged). Isaac's File > Save As closes
    the stage it saved and reopens the saved file, so the stage main() built is gone: spawning into it failed with
    "Stage.DefinePrim(Stage, str, str) did not match C++ signature", and the reopened stage is stopped. The reopened
    file has what the scene build and the spawns added (physics settings, collision groups, lavender cores, the
    robots and their graphs), so spawning into it works and pressing play resumes the robots (spawn_loop does)."""
    cur = omni.usd.get_context().get_stage()
    if cur is None:
        raise RuntimeError("no stage is open in Isaac Sim; restart the sim")
    try:
        same = stage is not None and stage.GetRootLayer() == cur.GetRootLayer()
    except Exception:  # an expired stage raises on any call
        same = False
    if not same:
        log(f"the open stage changed (Save As?), now {cur.GetRootLayer().identifier}")
    return cur, not same


async def spawn_loop(app, stage, og, usdrt_sdf, prev_state=None):
    """Spawn the requests found at start, then poll FLEET_REQUEST; a request with a new id replaces the robots.

    At start: FLEET_REQUEST if the previous run never answered it (scripts/fleet_ctl.py replaces robots that way: it
    writes the request and restarts the sim, because removing robots from a running sim crashed the renderer), falling
    back to FLEET_APPLIED, the last request that worked, if that one is rejected (and plain restarts, "Reset scene",
    bring back the same robots). Ids answered before this start are not replayed."""
    applied, pending = _read_json(FLEET_APPLIED), _read_json(FLEET_REQUEST)
    answered = ((prev_state or {}).get("spawn") or {}).get("id")
    boot = []
    if pending and pending.get("id") not in (answered, (applied or {}).get("id")):
        boot.append(pending)
    if applied:
        boot.append(applied)
    if not boot:
        write_state(boot="done")
    handled = {r.get("id") for r in (applied, pending) if r}
    control_done = (_read_json(FLEET_CONTROL) or {}).get("id")  # one left from a previous run isn't replayed
    next_check = 0.0
    while True:
        await app.next_update_async()
        booting = bool(boot)
        if booting:
            req = boot.pop(0)
        else:
            if time.time() < next_check:
                continue
            next_check = time.time() + 0.5
            stage, changed = current_stage(stage)
            if changed and ROBOTS:
                # measured: after a Save As with j100_0921 running, play brought /clock, odom and the GPS script node
                # back at full rate, same ROS nodes (no duplicates), drive_test 0.5 m/s -> 0.495
                omni.timeline.get_timeline_interface().play()
                log(f"resumed the simulation with {len(ROBOTS)} robot(s) in the reopened stage")
            ctl = _read_json(FLEET_CONTROL)
            if ctl and ctl.get("id") != control_done:
                control_done = ctl.get("id")
                if ctl.get("action") == "snapshot":
                    await snapshot_request(ctl)
                elif ctl.get("action") == "plants":
                    await plants_request(stage, ctl)
                elif ctl.get("action") == "camera":
                    await camera_request(ctl)
                elif ctl.get("action") == "record":
                    await record_request(ctl)
                elif ctl.get("action") == "sun":
                    await sun_request(ctl)
                elif ctl.get("action") == "clouds":
                    await clouds_request(ctl)
                else:
                    await reset_timeline(app, control_done)
                continue
            req = _read_json(FLEET_REQUEST)
            # at_start: written for the restart that fleet_ctl is about to do -- the next start spawns it
            if req is None or req.get("id") in handled or req.get("at_start"):
                continue
        last_id = req.get("id")
        handled.add(last_id)
        write_state(spawn={"id": last_id, "state": "spawning", "message": ""})
        t0 = time.time()
        try:
            robots = parse_request(req)
            log(f"spawn request {last_id}: {len(robots)} robot(s)")
            stage, _ = current_stage(stage)
            await spawn_fleet(app, stage, og, usdrt_sdf, robots)
            msg = (f"{len(ROBOTS)} robot(s) in {time.time() - t0:.0f} s: {', '.join(ns for ns, _ in ROBOTS)}"
                   if ROBOTS else "no robots")
            state = "done"
            _write_json(FLEET_APPLIED, req)
            log(f"simulation running with {msg}")
            if os.environ.get("FLEET_DEBUG") == "1" and ROBOTS:
                asyncio.ensure_future(debug_loop(og))
            if os.environ.get("FLEET_SNAPSHOT") and ROBOTS:
                asyncio.ensure_future(snapshot_loop())
        except ValueError as e:
            state, msg = "error", str(e)
            log(f"spawn request {last_id} rejected: {msg}")
        except Exception as e:
            state, msg = "error", f"{type(e).__name__}: {e}"
            log(f"spawn request {last_id} failed:\n" + traceback.format_exc())
        if state == "error":  # kept: a start-up fallback to the applied request overwrites "spawn" right after
            write_state(last_error={"id": last_id, "message": msg})
        write_state(
            robots=[{"slot": i, "ns": ns, "model": model, "x": p[0], "y": p[1], "yaw": p[2]}
                    for i, ((ns, model), p) in enumerate(zip(ROBOTS, ROBOT_POSES))],
            spawn={"id": last_id, "state": state, "message": msg},
        )
        # settled, or nothing left to try; a partial spawn (ROBOTS) isn't replaced in place: DeletePrims aborts the
        # headed renderer (sim/CLAUDE.md)
        if booting and (state == "done" or not boot or ROBOTS):
            boot.clear()  # the start-up spawn is settled (a rejected new request fell back to the applied one)
            write_state(boot="done")


async def main():
    prev_state = _read_json(FLEET_STATE)  # the previous run's report: which request it answered last
    try:
        write_state()
        app = omni.kit.app.get_app()
        for _ in range(5):
            await app.next_update_async()
        apply_kit_settings()
        global PHYSICS_ENGINE
        PHYSICS_ENGINE = select_physics_engine()
        fleet_pose.reset()  # it caches the engine on first use
        write_state(physics=PHYSICS_ENGINE)
        load_vbd_scene()
        try:
            add_omniverse_collection()
        except Exception:
            log("content browser: Omniverse collection failed (the sim is unaffected):\n" + traceback.format_exc())
        enable_extensions([
            "isaacsim.ros2.bridge", "isaacsim.robot.wheeled_robots.nodes", "omni.graph.action", "omni.graph.nodes", "omni.graph.scriptnode",
            # isaacsim.sensors.experimental.rtx (j100_0936's 2D lidar) deliberately NOT enabled -- broken in
            # this Isaac Sim 6.0 install, see add_lidar2d's docstring.
        ] + (["isaacsim.sensors.experimental.physics"] if PHYSICS_ENGINE == "physx" else []))  # PhysX IMU plugin
        try:
            install_depth_range_limit()
        except Exception:
            log("depth range: limit not installed, depth published unlimited:\n" + traceback.format_exc())
        # Every model that could be spawned later is imported now: the importer can't run once the scene is open.
        # A cached, current import only costs a stat.
        load_model_params([m for m in MODEL_ASSETS if os.path.isfile(f"{_ASSET_ROOT}/{m}/robot.yaml")])
        models = []
        for model in MODEL_ASSETS:
            if model not in MODEL_PARAMS or not os.path.isfile(MODEL_ASSETS[model]["urdf"]):
                continue
            try:
                import_urdf_if_needed(model)
                models.append(model)
            except Exception:
                log(f"import of {model} failed, it can't be spawned:\n" + traceback.format_exc())
        write_state(models=models)

        import omni.graph.core as og
        import usdrt.Sdf as usdrt_sdf

        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        layout = build_world(stage)
        if PHYSICS_ENGINE == "newton":
            top = newton_ground_plane(stage)
            log(f"newton: ground collider = a plane at z={top:.3f}" if top is not None else
                "newton: no /World/ground collider to replace with a plane")
        if farm_workers.ENABLED:  # FARM_WORKERS=1: two people sitting at a lavender row (costs fps)
            farm_workers.add(stage, layout.get("lavender_rows"))
        build_clock_graph(og)
        aim_viewport()
        set_viewport_resolution()
        for _ in range(10):
            await app.next_update_async()
        layout["chargers"] = [dict(c, **charger_tag(stage, c["id"])) for c in layout.get("chargers", [])]
        PLANT_PRIMS.update(index_plants(stage))
        layout.update(plant_count=len(PLANT_PRIMS), plants_hidden=[], actions=["reset", "snapshot", "plants", "camera", "record", "sun", "clouds"])
        # The timeline stays stopped: spawn_fleet plays it once every robot of a request is in the scene.
        log(f"scene ready ({layout['scene_source']}, stopped, {'sim time on /clock' if USE_SIM_TIME else 'wall-clock stamps'}), "
            f"waiting for a spawn request in {FLEET_REQUEST}; "
            f"models: {', '.join(models)}")
        write_state(scene="ready", ground=SPAWN_LIMIT, **layout)
        if CAM_ON_DEMAND and CAM_STREAMS:
            asyncio.ensure_future(camera_on_demand_loop())
    except Exception:
        log("FATAL error while building the scene:\n" + traceback.format_exc())
        write_state(scene="error", error=traceback.format_exc(limit=3))
        return
    await spawn_loop(app, stage, og, usdrt_sdf, prev_state)


asyncio.ensure_future(main())

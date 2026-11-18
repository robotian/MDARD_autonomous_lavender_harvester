"""The real cutter + gripper end effector (sim/assets/ee_cutter, make_ee_cutter_looks.py) on a robot's Kinova 2F Lite.

The real EE clamps onto the 2F Lite body and replaces its fingers: the 2F Lite's internal carriage slides a T-shaped
crossbar (CAD part `finger`) away from the arm to close, and its wing pin drives link_pusher -> link_1 (pivot on
gripper_base) -> link_2 -> finger_tip (pivot on gripper_base), which swings shut against gripper_base's curved jaw.
A Dynamixel turns the pruner blade through a spur gear and the blade gear's sector.

In the sim the 2F Lite's own finger joints stay (hidden, no colliders): arm_0_gripper_right_finger_bottom_joint is
what MoveIt, moveit_sim_bridge and joint_states know as the gripper. EE_SCRIPT reads its position every tick and
drives the linkage joints to the exact closed-loop solution (two four-bar loops; an articulation is a tree, so the
loops are closed by the targets, not by constraints, which also keeps it engine-neutral). The blade follows the
`pruner/blade` topic (Float64, 0 open .. 1 closed), which robot/bin/pruner_stub cycles on a pruner command.

Geometry from the STEP (sim/assets/ee_cutter/src/cutter_and_gripper_and_camera_bracket.stp, pin holes measured on
the tessellation): CAD frame x across the jaws, y along the tool axis (mount face y = 0), z the pivot axis.
"""
import math
import re

USD = "/sim/assets/ee_cutter/cutter_gripper.usd"
PARTS = "/cutter_gripper/cutter_fixture_design"  # every part of the assembly, CAD frame
# CAD frame -> arm_0_gripper_gripper_base_link: x -> y, y -> z (tool axis), z -> x (the D405 side, camera_1 at -x);
# gripper_base's holes for the 2F Lite finger axes (x = +-30.5 mm, y 67.8) land on the URDF's (y +-30.5, z 70.0)
EE_ORIENT = (0.5, 0.5, 0.5, 0.5)  # w, x, y, z
EE_OFFSET = (0.0, 0.0, 0.0022)

DRIVER = "arm_0_gripper_right_finger_bottom_joint"  # the 2F Lite's finger motor, as MoveIt commands it
STOCK_FINGERS = ("arm_0_gripper_right_finger_prox_link", "arm_0_gripper_right_finger_dist_link",
                 "arm_0_gripper_left_finger_prox_link", "arm_0_gripper_left_finger_dist_link")
BLADE_TOPIC = "pruner/blade"

# pin centres (CAD, m)
W0 = (-0.05103, 0.05555)  # crossbar wing -> link_pusher
E0 = (-0.05892, 0.09792)  # link_pusher -> link_1
C = (-0.04443, 0.10780)   # link_1 on gripper_base
D0 = (-0.07743, 0.08521)  # link_1 -> link_2
B0 = (-0.06135, 0.14158)  # link_2 -> finger_tip
A = (-0.04448, 0.14780)   # finger_tip on gripper_base
BLADE = (0.00593, 0.10604)
SPUR = (0.00085, 0.05580)

# Kinova 0 = the CAD pose (open), its 0.96 limit = closed: 14 mm of crossbar travel swings finger_tip 88.7 deg,
# where its tip meets the jaw's far horn (1 mm gap). grid_cutter's close (0.8) stops short of that, like on a stem.
KINOVA_CLOSED = 0.96
SLIDE_CLOSED = 0.014
# blade: +80 deg (about +z) lays its edge along the counter-blade (prunner_rcv); the sector's teeth allow 95 deg.
# 88 since 2026-11-08 (user): the edge closes 8 deg past the counter-blade's
BLADE_CLOSED = math.radians(88.0)
BLADE_SPEED = 3.0  # fraction of the stroke per second: closes in ~0.33 s like an electric pruner
SPUR_RATIO = -5.05  # spur / blade: pitch radii 8.35 / 42.15 mm from the outer radii and the 50.5 mm centre distance
SPUR_LIMIT = 6.2  # rad, just inside PhysX's 2 pi limit on revolute drive targets (the gear stops turning there)

# body: (parts under PARTS, parent body ("base" = gripper_base_link), joint, pivot, mass kg (estimates: printed
# parts at their size, the steel blade), colliders)
BODIES = {
    "slide": (["finger_1", "pin_W0_1"], "base", "prismatic", (0.0, 0.0), 0.015, False),
    "pusher": (["link_pusher_1"], "slide", "revolute", W0, 0.004, False),
    "link_1": (["link_1_1", "pin_E0_1", "pin_D0_1"], "base", "revolute", C, 0.006, False),
    "link_2": (["link_2_1"], "link_1", "revolute", D0, 0.004, False),
    "finger": (["finger_tip_1", "pin_B0_1"], "base", "revolute", A, 0.012, True),
    "blade": (["new_one_1/prunner_blade_1", "new_one_1/blade_gear_1"], "base", "revolute", BLADE, 0.065, True),
    "spur": (["new_one_1/Spur_Gear1_1"], "base", "revolute", SPUR, 0.010, False),
}
JOINTS = [f"ee_cutter_{b}_joint" for b in BODIES]
STATIC_COLLIDERS = ("gripper_base_1", "new_one_1/prunner_rcv_1")  # the jaw and the counter-blade
# moving parts split into convex pieces too: their single hulls were 1.5-2.1x the part (the blade's curve, the gear's
# teeth, the finger's inner arc filled in) and held light cut pieces after the release (2026-11-01)
DECOMPOSED = ("new_one_1/prunner_blade_1", "new_one_1/blade_gear_1", "finger_tip_1")
# the housing, so the EE meets the robot's own basket (none of them reaches past y 134 mm, where stems sit in the jaw)
HOUSING_COLLIDERS = ("new_one_1/cutter_base_1",
                     "new_one_1/XM_H_430_idler_1", "cutter_fixture_cutterside_1", "cutter_fixture_cutterside_2",
                     "D405_bracket_abs_1")
# the blade's two covers: thin open shells whose single hulls were 7.0x / 6.4x their volume, solid out to y 121 mm
# at the pocket's mouth: uncut stems the real covers let through hooked on them at 9-21 N and tore at the root (3 in
# patch 1 of vbd_cut_stem_v3p, 2026-11-11), split into convex pieces like the jaw
HOUSING_DECOMPOSED = ("new_one_1/cutter_cover_top_1", "new_one_1/cutter_cover_bottome_1")
HOUSING_SPLIT = True  # False: one convex hull each, as until vbd_cut_stem_v3q (a VBD scene's `covers: hull`)
# The robot's links the EE collides with (its self-collisions are otherwise off): the basket walls. URDF_ENVELOPE is
# mtu32_description's link for this EE (a mesh for RViz, boxes for MoveIt); the sim shows and collides its parts.
CONTACT_LINKS = re.compile(r"unloader_wall_\d+_link$")
URDF_ENVELOPE = "ee_cutter_link"
BASKET_FLOOR = "top_plate_rail_link"  # the basket's floor (cut_stems: cut stems land on it)
# position drives: PhysX (as the arm's gripper joints), Newton per radian / per metre (NEWTON_GRIPPER_GAINS' values)
GAINS = {"physx": {"revolute": (1.0e5, 1.0e4), "prismatic": (1.0e5, 1.0e4)},
         "newton": {"revolute": (20.0, 5.0), "prismatic": (2000.0, 100.0)}}
MAX_FORCE = {"revolute": 10.0, "prismatic": 100.0}  # N*m (the 2F Lite fingers' effort), N
BLADE_LEVER = 0.03  # m, blade pivot to where a stem sits on its edge (cut_stems' force estimate)
# Black foam pads in the pocket between the jaw's curve and finger_tip's inner arc (user, 2026-10-30: more contact and
# friction on the stems): CAD mm at x = PAD_X, from the parts' sections at z = 10 mm, the finger at KINOVA_CLOSED.
# Each pad fills half the pocket less PAD_GAP / 2, so closed they leave PAD_GAP and squeeze a stem: 1 mm since the 2 mm
# stems of stem models 2 and 3 (user, 2026-10-31; 3 mm for the 5 mm stems before).
PAD_X = range(-36, 17, 4)
PAD_JAW = (126.5, 123.5, 120.6, 117.6, 114.7, 113.3, 113.2, 113.3, 114.5, 116.2, 118.4, 121.0, 124.4, 128.6)
PAD_FINGER = (146.6, 148.0, 149.9, 151.4, 152.3, 153.0, 153.2, 153.0, 152.5, 151.5, 150.2, 148.4, 146.0, 144.1)
PAD_GAP = 1.0
# m, the jaw plates' / finger arm's thickness. Taller pads (to 28 mm, next to the blade's plane) caught more of the
# leaning stems' cut pieces, but also clamped and dragged more uncut ones: on the VBD plant the arm got stuck in it or
# the robot tipped over at the far patches in 4 of 5 runs (vbd_cut_stem21-25), so they stay the parts' thickness.
PAD_Z = {"jaw": (-0.0122, 0.0119), "finger": (-0.0071, 0.0220)}
# foam: rubbery friction (max: it wins over the stems' 0.5), soft through PhysX compliant contacts (N/m, N*s/m)
# 25000 N/m since 2026-11-01 (user): at 5000 a 2 mm stem sank its whole 1 mm radius into a pad at ~5 N and popped
# through, or stayed embedded so cut pieces rode back out of the basket; 25000 (0.2 mm at 5 N) caught every stem in the
# study (docs/lavender_plant_model/study/RESULTS.md S2, T2)
FOAM = {"friction": (1.0, 0.9), "stiffness": 25000.0, "damping": 10.0}
# Housing seal (user, 2026-11-12: "a design flaw of the end effector ... seal those gaps with colliders", sim only, a
# stand-in for a design change): every stem torn at the root in the snag captures (vbd_cut_stem_snag1, _flowers1) had
# lain in a gap of the blade's housing. Four (29, 33, 73, 95) were in the 3.3 mm slot between the jaw plate (gripper_base,
# top at CAD z 10.3 mm) and the cover top's floor (z 13.6), open all round the housing behind the pocket. One (44) was
# in a side opening of the covers at the blade gear's height (z 21-34), where the gear's sector swings out of the
# housing. The seal is one invisible convex prism: the covers' outline (x, y), from SEAL_UNDER_M below the jaw plate's
# top to the cover bottom's outer face. It belongs to the static parts' body, and contact_with_robot filters it against
# every other robot body like any EE collider, so the blade, gear and finger pass through it; stems meet it. It
# reaches CAD y 123 mm at the outline's apex, over the jaw pad and short of the pads' gap (y ~133): the pocket and
# the blade's cut in front of the housing stay as they were.
SEAL_HOUSING = True
SEAL_UNDER_M = 0.0005


def _ang(v):
    return math.atan2(v[1], v[0])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _circles(c0, r0, c1, r1, near):
    """The intersection of two circles nearest `near` (the assembly's own branch)."""
    dx, dy = c1[0] - c0[0], c1[1] - c0[1]
    d = math.hypot(dx, dy)
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    mx, my = c0[0] + a * dx / d, c0[1] + a * dy / d
    p, q = (mx - h * dy / d, my + h * dx / d), (mx + h * dy / d, my - h * dx / d)
    return min(p, q, key=lambda r: math.hypot(r[0] - near[0], r[1] - near[1]))


def _dist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def targets(q, blade):
    """Joint targets in JOINTS order for the 2F Lite joint at q (rad) and the blade at `blade` (0..1)."""
    s = SLIDE_CLOSED * min(max(q / KINOVA_CLOSED, 0.0), 1.0)
    w = (W0[0], W0[1] + s)
    e = _circles(w, _dist(E0, W0), C, _dist(E0, C), E0)
    link_1 = _wrap(_ang((e[0] - C[0], e[1] - C[1])) - _ang((E0[0] - C[0], E0[1] - C[1])))
    cs, sn = math.cos(link_1), math.sin(link_1)
    d = (C[0] + cs * (D0[0] - C[0]) - sn * (D0[1] - C[1]), C[1] + sn * (D0[0] - C[0]) + cs * (D0[1] - C[1]))
    b = _circles(d, _dist(B0, D0), A, _dist(B0, A), B0)
    pusher = _wrap(_ang((e[0] - w[0], e[1] - w[1])) - _ang((E0[0] - W0[0], E0[1] - W0[1])))
    link_2 = _wrap(_ang((b[0] - d[0], b[1] - d[1])) - _ang((B0[0] - D0[0], B0[1] - D0[1])) - link_1)
    finger = _wrap(_ang((b[0] - A[0], b[1] - A[1])) - _ang((B0[0] - A[0], B0[1] - A[1])))
    theta = BLADE_CLOSED * min(max(blade, 0.0), 1.0)
    # the (decorative) spur gear's target stays within PhysX's +-2 pi for a revolute articulation drive: at -5.05 x 80 deg
    # it was -7.05 rad, and the error it logged every step counted toward the 1000 that stop the simulation
    return [s, pusher, link_1, link_2, finger, theta, max(SPUR_RATIO * theta, -SPUR_LIMIT)]


# rad (set by attach from model_params `vbd_finger_gains`' third value): the finger's drive target goes this far past
# closed, and never more than this ahead of the finger toward closed: the closing torque is then at most stiffness x
# this however far the stems push the finger open (a motor that stalls), and a joint limit at closed stops it there
# with the pocket empty -- a force-limited gripper on a solver without effort limits (Newton VBD). Aimed only past
# closed, the target stayed put while a dragged stem pushed the finger 0.17 rad open, and the squeeze rose 13 -> 55 N
# (vbd_cut_stem42, stem 68)
FINGER_OVERCLOSE = 0.0
# rad the blade's drive target may run ahead of the measured blade angle (set by attach from model_params
# `vbd_blade_gains`' third value): the blade's torque is then at most its stiffness x this, a motor that stalls,
# instead of a spring wound up by a target that races to the end of the stroke (20 N*m/rad x 1 rad: a 700 N hammer
# at the edge, whatever it met: Newton VBD caps no joint effort)
BLADE_LAG = 0.0
# diagnostic: with a BLADE_LAG the EeCutter node writes <dir>/drive.csv (sim time, blade command, drive target, blade
# angle) every frame; vbd_plant writes the stems' side there (blade.csv). "" = off.
BLADE_LOG_DIR = ""  # e.g. "/sim/generated/blade_torque"


def attach(stage, gripper_base, physics, robot_root, finger_gains=None, blade_gains=None):
    """Add the EE under the robot's arm_0_gripper_gripper_base_link prim `gripper_base` (before the timeline plays),
    colliding with the CONTACT_LINKS of the robot at `robot_root`; returns (joints added, contact_with_robot()'s).
    physics: "physx" | "newton" (drive gain units differ). finger_gains: (stiffness N*m/rad, damping N*m*s/rad[,
    overclose rad]) of the finger joint instead of GAINS' (model_params `vbd_finger_gains`, Newton VBD); with an
    overclose the finger joint gets a limit at closed (FINGER_OVERCLOSE). blade_gains: (stiffness, damping[, lag rad])
    of the blade joint (model_params `vbd_blade_gains`); a lag makes the EeCutter node stall the target (BLADE_LAG)."""
    global FINGER_OVERCLOSE, BLADE_LAG
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    src = Usd.Stage.Open(USD)
    parts = [p.GetPath().pathString[len(PARTS) + 1:] for p in Usd.PrimRange(src.GetPrimAtPath(PARTS))
             if p.GetParent().GetPath().pathString in (PARTS, PARTS + "/new_one_1") and p.GetName() != "new_one_1"]
    cad = UsdGeom.XformCache()
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    moving = {p for b in BODIES.values() for p in b[0]}
    base = stage.GetPrimAtPath(gripper_base)
    ee_xf = Gf.Matrix4d(Gf.Rotation(Gf.Quatd(*EE_ORIENT)), Gf.Vec3d(*EE_OFFSET))
    q_ee = Gf.Quatf(*EE_ORIENT)

    # stock 2F Lite fingers: their instanced visuals and colliders go, the links and joints (DRIVER) stay
    for link in STOCK_FINGERS:
        for prim in Usd.PrimRange(base):
            if prim.GetName() == link:
                for child in prim.GetChildren():
                    if child.IsInstanceable():
                        child.SetActive(False)

    def add_parts(path, keep):
        """The whole asset (its materials live outside the parts) with every part but `keep` switched off."""
        prim = stage.DefinePrim(path, "Xform")
        prim.GetReferences().AddReference(USD)
        for p in parts:
            if p not in keep:
                stage.GetPrimAtPath(f"{path}{PARTS[len('/cutter_gripper'):]}/{p}").SetActive(False)
        return prim

    def collide(path, part, approximation):
        mesh = next(p for p in Usd.PrimRange(stage.GetPrimAtPath(f"{path}{PARTS[len('/cutter_gripper'):]}/{part}"))
                    if p.IsA(UsdGeom.Mesh))
        UsdPhysics.CollisionAPI.Apply(mesh)
        UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr(approximation)

    static = add_parts(f"{gripper_base}/ee_cutter", [p for p in parts if p not in moving])
    UsdGeom.Xformable(static).AddTransformOp().Set(ee_xf)
    for part in STATIC_COLLIDERS:
        decompose(stage, static.GetPath(), part, collide)
    for part in HOUSING_COLLIDERS:
        collide(static.GetPath(), part, "convexHull")
    for part in HOUSING_DECOMPOSED:  # CoACD at 0.5 left each cover one hull; 0.1: 30 / 38 pieces, ~2x the shell's volume
        if HOUSING_SPLIT:
            decompose(stage, static.GetPath(), part, collide, threshold=0.1)
        else:
            collide(static.GetPath(), part, "convexHull")

    kp_kd = GAINS[physics]
    for name, (keep, parent, kind, pivot, mass, colliders) in BODIES.items():
        path = f"{gripper_base}/ee_cutter_{name}"
        body = add_parts(path, keep)
        UsdGeom.Xformable(body).AddTransformOp().Set(ee_xf)  # every body at rest = the CAD frame
        UsdPhysics.RigidBodyAPI.Apply(body)
        # explicit mass and box inertia from the parts' CAD bounds (the colliders' density would weigh solid plastic)
        box = Gf.Range3d()
        for p in keep:
            box.UnionWith(bbox.ComputeWorldBound(src.GetPrimAtPath(f"{PARTS}/{p}")).ComputeAlignedRange())
        sx, sy, sz = box.GetSize()
        m_api = UsdPhysics.MassAPI.Apply(body)
        m_api.CreateMassAttr(mass)
        m_api.CreateCenterOfMassAttr(Gf.Vec3f(*box.GetMidpoint()))
        m_api.CreateDiagonalInertiaAttr(Gf.Vec3f(mass * (sy * sy + sz * sz) / 12, mass * (sx * sx + sz * sz) / 12,
                                                 mass * (sx * sx + sy * sy) / 12))
        if colliders:
            for p in keep:
                if p in DECOMPOSED:
                    decompose(stage, path, p, collide)
                else:
                    collide(path, p, "convexHull")

        joint_path = f"{gripper_base}/ee_cutter_{name}_joint"
        joint = (UsdPhysics.PrismaticJoint if kind == "prismatic" else UsdPhysics.RevoluteJoint).Define(stage, joint_path)
        joint.CreateAxisAttr("Y" if kind == "prismatic" else "Z")
        joint.CreateBody0Rel().SetTargets([gripper_base if parent == "base" else f"{gripper_base}/ee_cutter_{parent}"])
        joint.CreateBody1Rel().SetTargets([path])
        pos = Gf.Vec3d(pivot[0], pivot[1], 0.0)
        if parent == "base":  # joint frame = the CAD frame at the pivot, in both bodies
            joint.CreateLocalPos0Attr(Gf.Vec3f(ee_xf.Transform(pos)))
            joint.CreateLocalRot0Attr(q_ee)
        else:
            joint.CreateLocalPos0Attr(Gf.Vec3f(pos))
            joint.CreateLocalRot0Attr(Gf.Quatf(1.0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(pos))
        joint.CreateLocalRot1Attr(Gf.Quatf(1.0))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear" if kind == "prismatic" else "angular")
        kp, kd = kp_kd[kind]
        if name == "blade" and blade_gains:
            kp, kd = float(blade_gains[0]), float(blade_gains[1])
            if len(blade_gains) > 2 and float(blade_gains[2]) > 0:
                BLADE_LAG = float(blade_gains[2])
        if name == "finger" and finger_gains:
            kp, kd = float(finger_gains[0]), float(finger_gains[1])
            if len(finger_gains) > 2 and float(finger_gains[2]) > 0:
                FINGER_OVERCLOSE = float(finger_gains[2])
                closed = math.degrees(targets(KINOVA_CLOSED, 0.0)[4])  # USD revolute limits are in degrees
                joint.CreateLowerLimitAttr(min(closed, 0.0))
                joint.CreateUpperLimitAttr(max(closed, 0.0) + 5.0)
        if physics == "newton" and kind == "revolute":  # USD angular gains are per degree; Newton scales them back
            kp, kd = kp * math.pi / 180.0, kd * math.pi / 180.0
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(kp)
        drive.CreateDampingAttr(kd)
        drive.CreateMaxForceAttr(MAX_FORCE[kind])
        drive.CreateTargetPositionAttr(0.0)
    add_pads(stage, gripper_base)
    if SEAL_HOUSING:
        add_seal(stage, static.GetPath())
    return len(BODIES), contact_with_robot(stage, robot_root, gripper_base)


def pad_quads():
    """{"jaw" | "finger": [4 CAD (x, y) corners in m, counter-clockwise] per piece}, the finger's in its open (CAD) pose."""
    a = -targets(KINOVA_CLOSED, 0.0)[4]  # closed -> open, about A
    cs, sn = math.cos(a), math.sin(a)

    def unclose(x, y):
        return A[0] + cs * (x - A[0]) - sn * (y - A[1]), A[1] + sn * (x - A[0]) + cs * (y - A[1])

    mid = [(j + f) / 2 for j, f in zip(PAD_JAW, PAD_FINGER)]
    rows = {"jaw": (PAD_JAW, [m - PAD_GAP / 2 for m in mid]), "finger": ([m + PAD_GAP / 2 for m in mid], PAD_FINGER)}
    quads = {}
    for name, (lo, hi) in rows.items():
        quads[name] = []
        for i in range(len(PAD_X) - 1):
            q = [(PAD_X[i], lo[i]), (PAD_X[i + 1], lo[i + 1]), (PAD_X[i + 1], hi[i + 1]), (PAD_X[i], hi[i])]
            q = [(x / 1000, y / 1000) for x, y in q]
            quads[name].append([unclose(*p) for p in q] if name == "finger" else q)
    return quads


def add_pads(stage, gripper_base):
    """The foam pads (PAD_*): convex-hull pieces, black, FOAM's physics material; the jaw's on the static parts, the
    finger's on its body (both "ee" colliders for contact_with_robot, so they don't collide with each other)."""
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

    look = UsdShade.Material.Define(stage, f"{gripper_base}/ee_cutter_foam")
    sh = UsdShade.Shader.Define(stage, look.GetPath().AppendChild("shader"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.015, 0.015, 0.015))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    sh.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)  # no sky reflection at grazing angles
    sh.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
    look.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    mat = UsdPhysics.MaterialAPI.Apply(look.GetPrim())
    mat.CreateStaticFrictionAttr(FOAM["friction"][0])
    mat.CreateDynamicFrictionAttr(FOAM["friction"][1])
    mat.CreateRestitutionAttr(0.0)
    foam = PhysxSchema.PhysxMaterialAPI.Apply(look.GetPrim())
    foam.CreateFrictionCombineModeAttr("max")
    foam.CreateCompliantContactStiffnessAttr(FOAM["stiffness"])
    foam.CreateCompliantContactDampingAttr(FOAM["damping"])

    faces = [0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7]  # outward, counter-clockwise quads
    for name, quads in pad_quads().items():
        parent = f"{gripper_base}/ee_cutter" if name == "jaw" else f"{gripper_base}/ee_cutter_finger"
        z0, z1 = PAD_Z[name]
        for i, q in enumerate(quads):
            mesh = UsdGeom.Mesh.Define(stage, f"{parent}/foam_pad/piece_{i}")
            mesh.CreatePointsAttr([Gf.Vec3f(x, y, z) for z in (z0, z1) for x, y in q])
            mesh.CreateFaceVertexCountsAttr([4] * 6)
            mesh.CreateFaceVertexIndicesAttr(faces)
            mesh.CreateSubdivisionSchemeAttr("none")
            binding = UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
            binding.Bind(look)
            binding.Bind(look, materialPurpose="physics")
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")


def hull_2d(pts):
    """Convex hull of (x, y) points, counter-clockwise (monotone chain)."""
    pts = sorted(set(pts))

    def half(seq):
        h = []
        for p in seq:
            while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (p[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(p)
        return h[:-1]
    return half(pts) + half(reversed(pts))


def add_seal(stage, static):
    """SEAL_HOUSING's prism under the static parts' prim `static` (whose frame is the CAD frame, m): the covers'
    outline in x, y, from the jaw plate's top less SEAL_UNDER_M to the covers' outer face. Returns its (z0, z1) in m."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    xf = UsdGeom.XformCache()
    root = stage.GetPrimAtPath(static)

    def points(part):  # the part's visual mesh points in the CAD frame (not its _hulls)
        out = []
        for p in Usd.PrimRange(stage.GetPrimAtPath(f"{static}{PARTS[len('/cutter_gripper'):]}/{part}")):
            if p.IsA(UsdGeom.Mesh) and not p.GetParent().GetName().endswith("_hulls"):
                m, _ = xf.ComputeRelativeTransform(p, root)
                out += [m.Transform(Gf.Vec3d(v)) for v in UsdGeom.Mesh(p).GetPointsAttr().Get()]
        return out
    covers = points("new_one_1/cutter_cover_top_1") + points("new_one_1/cutter_cover_bottome_1")
    z0 = max(v[2] for v in points("gripper_base_1")) - SEAL_UNDER_M
    z1 = max(v[2] for v in covers)
    ring = hull_2d([(round(v[0], 5), round(v[1], 5)) for v in covers])
    n = len(ring)
    mesh = UsdGeom.Mesh.Define(stage, root.GetPath().AppendPath("seal/piece_0"))
    mesh.CreatePointsAttr([Gf.Vec3f(x, y, z) for z in (z0, z1) for x, y in ring])
    mesh.CreateFaceVertexCountsAttr([n, n] + [4] * n)
    mesh.CreateFaceVertexIndicesAttr(list(reversed(range(n))) + [n + i for i in range(n)]
                                     + [k for i in range(n) for k in (i, (i + 1) % n, n + (i + 1) % n, n + i)])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    UsdPhysics.MassAPI.Apply(mesh.GetPrim()).CreateMassAttr(1e-4)  # no weight (the body's mass is authored anyway)
    return z0, z1


def decompose(stage, path, part, collide, threshold=0.5):
    """Convex pieces of a concave part (the U jaw, the counter-blade, DECOMPOSED) as invisible convex-hull colliders beside its
    mesh, with CoACD and Newton's own settings: Newton's convexDecomposition makes the pieces at parse time, as shapes
    no USD prim stands for, so physics:filteredPairs (contact_with_robot) can't reach them. Without CoACD (PhysX:
    Newton's pip bundle isn't loaded) the mesh gets PhysX's convexDecomposition."""
    from pxr import Usd, UsdGeom, UsdPhysics, Vt
    try:
        import coacd
        import numpy as np
    except ImportError:
        return collide(path, part, "convexDecomposition")
    prim = stage.GetPrimAtPath(f"{path}{PARTS[len('/cutter_gripper'):]}/{part}")
    mesh = next(UsdGeom.Mesh(p) for p in Usd.PrimRange(prim) if p.IsA(UsdGeom.Mesh))
    pts = np.array(mesh.GetPointsAttr().Get(), np.float64)
    counts, idx = mesh.GetFaceVertexCountsAttr().Get(), np.array(mesh.GetFaceVertexIndicesAttr().Get())
    tris, k = [], 0
    for c in counts:  # fan-triangulate
        tris += [(idx[k], idx[k + i], idx[k + i + 1]) for i in range(1, c - 1)]
        k += c
    hulls = coacd.run_coacd(coacd.Mesh(pts, np.array(tris)), threshold=threshold, mcts_nodes=20, mcts_iterations=5,
                            mcts_max_depth=1, merge=False)
    group = UsdGeom.Xform.Define(stage, mesh.GetPath().GetParentPath().AppendChild(mesh.GetPrim().GetName() + "_hulls"))
    group.AddTransformOp().Set(mesh.GetLocalTransformation())
    for i, (v, f) in enumerate(hulls):
        hull = UsdGeom.Mesh.Define(stage, group.GetPath().AppendChild(f"hull_{i}"))
        hull.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(v, np.float32)))
        hull.CreateFaceVertexCountsAttr([3] * len(f))
        hull.CreateFaceVertexIndicesAttr(np.asarray(f).ravel().tolist())
        hull.MakeInvisible()
        UsdPhysics.CollisionAPI.Apply(hull.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(hull.GetPrim()).CreateApproximationAttr("convexHull")
    return len(hulls)


def contact_with_robot(stage, robot_root, gripper_base):
    """Turn the robot's self-collisions on, then filter every pair of its colliders on different bodies except the
    EE's against CONTACT_LINKS (Newton and PhysX both read physics:filteredPairs on colliders; Newton has no
    per-pair "collide" with self-collisions off). The robot's instanced prims are de-instanced first: a filter
    can't be authored on an instance proxy. Returns (EE colliders, contact colliders, filtered pairs)."""
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    root = stage.GetPrimAtPath(robot_root)
    while True:
        inst = [p for p in Usd.PrimRange(root) if p.IsInstance()]
        if not inst:
            break
        for p in inst:
            p.SetInstanceable(False)
    for p in Usd.PrimRange(root):  # the URDF's EE (RViz's mesh, MoveIt's boxes): the sim has the parts instead
        if p.GetName() == URDF_ENVELOPE:
            for c in Usd.PrimRange(p):
                if c.IsA(UsdGeom.Gprim):
                    c.SetActive(False)
    shapes = []  # (path, body path, kind) kind: "ee" | "contact" | ""
    for p in Usd.PrimRange(root):
        if not (p.IsA(UsdGeom.Gprim) and p.HasAPI(UsdPhysics.CollisionAPI)):
            continue
        if UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is False:
            continue
        body = p
        while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):
            body = body.GetParent()
        if not body:
            continue
        path = p.GetPath().pathString
        kind = ("ee" if path.startswith(gripper_base + "/ee_cutter")
                else "contact" if CONTACT_LINKS.search(body.GetName()) else "")
        shapes.append((p, body.GetPath(), kind))
    # MuJoCo (Newton's solver) treats bodies on fixed joints as one welded body and filters body pairs by weld, so
    # the walls would share the chassis' exclusion from the EE, and the gripper base (the housing's body) that of
    # its collider-less parent arm_0_end_effector_link: a locked hinge keeps each its own body.
    own = {b for _, b, k in shapes if k == "contact"} | {Sdf.Path(gripper_base)}
    for p in Usd.PrimRange(root):
        if p.IsA(UsdPhysics.FixedJoint) and set(UsdPhysics.Joint(p).GetBody1Rel().GetTargets()) & own:
            p.SetTypeName("PhysicsRevoluteJoint")
            hinge = UsdPhysics.RevoluteJoint(p)
            hinge.CreateAxisAttr("Z")
            hinge.CreateLowerLimitAttr(-0.01)  # degrees; MuJoCo refuses an empty range
            hinge.CreateUpperLimitAttr(0.01)
    pairs = 0
    for i, (p, body, kind) in enumerate(shapes):
        others = [q.GetPath() for q, b, k in shapes[i + 1:] if b != body and {kind, k} != {"ee", "contact"}]
        if others:
            UsdPhysics.FilteredPairsAPI.Apply(p).CreateFilteredPairsRel().SetTargets(others)
            pairs += len(others)
    for p in Usd.PrimRange(root):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            p.CreateAttribute("newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool).Set(True)
            p.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(True)
    counts = tuple(sum(k == kind for _, _, k in shapes) for kind in ("ee", "contact"))
    return counts + (pairs,)


_step_subs = []  # physics step subscriptions of the stem cutters (kept alive here)


def cut_stems(stage, gripper_base, robot_root, physics, log):
    """With a dynamic lavender plant in the scene (sim/assets/lavender_dyn, make_lavender_plant.py: /stems/stem_k/seg_i
    bodies): the plant touches only this EE (filtered against the robot's other bodies, which pass through it like
    soft lavender), and a StemCutter cuts a stem the blade pinches against the counter-blade, every physics step.
    PhysX only (Newton re-parses the stage when a cut switches a joint off). Returns a log line, or None."""
    from omni.physx import get_physx_interface
    from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics
    from stem_cutter import StemCutter

    # every dynamic body of the plant: seg_<i> (stem models 1, 2) or whole / upper_<i> / lower_<i> (model 3, the
    # inactive cut pieces included: once a cut activates them they must be filtered against the robot like the rest)
    scopes = [p for p in stage.Traverse() if p.GetName() == "stems" and any(c.GetName().startswith("stem_") for c in p.GetChildren())]
    segs = [q.GetPath() for sc in scopes for q in Usd.PrimRange(sc, Usd.PrimAllPrimsPredicate)
            if q.HasAPI(UsdPhysics.RigidBodyAPI) and q.GetParent().GetName().startswith("stem_")]
    if not segs:
        return None
    if physics != "physx":
        return f"dynamic lavender ({len(segs)} segments) not cuttable under {physics}: run SIM_PHYSICS=physx"
    crowns = sorted({s.GetParentPath().GetParentPath().GetParentPath().AppendChild("crown") for s in segs})
    plant = crowns + segs
    # filtered per collider, not per body: on the articulation root's body a filter covers the whole articulation,
    # the EE with it (no contact at all, 2026-10-30). contact_with_robot de-instanced the robot, so they're editable.
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_root)):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        owner = prim
        while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
            owner = owner.GetParent()
        path = owner.GetPath().pathString if owner else ""
        if path == gripper_base or path.startswith(gripper_base + "/"):
            continue
        # the basket (walls + floor plate) catches the cut stems the gripper lets go of: only the crown is kept out
        basket = owner and (CONTACT_LINKS.search(owner.GetName()) or owner.GetName() == BASKET_FLOOR)
        rel = UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel()
        for p in (crowns if basket else plant):
            rel.AddTarget(p)
        n += 1
    # a 5 g stem squeezed between the driven blade and the counter-blade: the blade sank through it with PhysX's
    # default iterations (plant_test.py needed 64 on its blade); the blade is an articulation link, so the robot's
    roots = [p for p in Usd.PrimRange(stage.GetPrimAtPath(robot_root)) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    for root in roots:  # articulation gets them
        PhysxSchema.PhysxArticulationAPI.Apply(root).CreateSolverPositionIterationCountAttr(64)
    cfg = stage.GetPrimAtPath(segs[0].GetParentPath().GetParentPath().GetParentPath()).GetCustomDataByKey(
        "lavender_plant") or {}
    base_prim, blade_prim = stage.GetPrimAtPath(gripper_base), stage.GetPrimAtPath(f"{gripper_base}/ee_cutter_blade")
    now = Usd.TimeCode.Default()

    def blade_in_base():
        return (UsdGeom.Xformable(blade_prim).ComputeLocalToWorldTransform(now)
                * UsdGeom.Xformable(base_prim).ComputeLocalToWorldTransform(now).GetInverse())

    open_inv = blade_in_base().GetInverse()  # the blade's rest pose in the base (the CAD frame's 120 deg turn in it)

    def blade_force():
        """The blade motor's force on a stem during its stroke (blade > 3 deg off open: the torque cap over a 3 cm
        lever, ~330 N, far over a stem's), else 0. PhysX's contact impulses read 1-12 N on stems the blade swept
        aside (2026-10-30), so a stem touching blade and counter-blade together in the stroke counts as pinched."""
        turned = (blade_in_base() * open_inv).ExtractRotation().GetAngle()
        return MAX_FORCE["revolute"] / BLADE_LEVER if turned > 3.0 else 0.0

    cutter = StemCutter(stage, [f"{gripper_base}/ee_cutter_blade"], float(cfg.get("cut_force", 20.0)),
                        int(cfg.get("cut_hold_steps", 2)), lambda m: log(f"stem cutter: {m}"),
                        anvil_paths=[f"{gripper_base}/ee_cutter{PARTS[len('/cutter_gripper'):]}/new_one_1/prunner_rcv_1"],
                        blade_force=blade_force)
    finger = stage.GetPrimAtPath(f"{gripper_base}/ee_cutter_finger")  # its pad's contacts too (the jaw's are on gripper_base)
    while finger and not finger.HasAPI(UsdPhysics.RigidBodyAPI):
        finger = finger.GetParent()
    if finger:
        PhysxSchema.PhysxContactReportAPI.Apply(finger).CreateThresholdAttr(0.0)
    clock, logged = [0.0], [0.0]

    base_link = next(p for p in Usd.PrimRange(stage.GetPrimAtPath(robot_root)) if p.GetName() == "base_link")
    where = {}  # cut segment -> its last place (gripper / basket / ground / air), logged when it changes
    moved = [0.0, {}]  # time of the last rolling check, the pieces' positions then

    def piece_bodies(seg):
        """The cut piece's bodies: seg_<i> and the stem's segments above it."""
        i = int(Sdf.Path(seg).name[4:])
        return [c for c in stage.GetPrimAtPath(Sdf.Path(seg).GetParentPath()).GetChildren()
                if c.GetName().startswith("seg_") and int(c.GetName()[4:]) >= i]

    def place(seg):
        """Where a cut piece is now, in the robot's frame: any of its segments within 10 cm of the gripper base (a
        piece stuck in the pads: until 2026-11-01 only its cut end counted, and stuck pieces read "basket"), inside
        the basket (base_link x -0.18..0.07, y -0.07..0.27, above its floor), on the ground, else in the air."""
        p = UsdGeom.Xformable(stage.GetPrimAtPath(seg)).ComputeLocalToWorldTransform(now).ExtractTranslation()
        g = UsdGeom.Xformable(base_prim).ComputeLocalToWorldTransform(now).ExtractTranslation()
        b = UsdGeom.Xformable(base_link).ComputeLocalToWorldTransform(now).GetInverse().Transform(p)
        if any((UsdGeom.Xformable(c).ComputeLocalToWorldTransform(now).ExtractTranslation() - g).GetLength() < 0.1
               for c in piece_bodies(seg)):
            return "gripper", b
        if -0.18 < b[0] < 0.07 and -0.07 < b[1] < 0.27 and b[2] > 0.2:
            return "basket", b
        return ("ground" if p[2] < 0.1 else "air"), b

    def on_step(dt):
        clock[0] += dt
        cutter.step(dt, clock[0])
        if clock[0] - moved[0] >= 5.0 and cutter.cuts:  # rolling: pieces that moved > 2 cm in the last 5 s
            moved[0] = clock[0]
            now_at = {seg: UsdGeom.Xformable(stage.GetPrimAtPath(seg)).ComputeLocalToWorldTransform(now)
                      .ExtractTranslation() for _, seg, _ in cutter.cuts}
            d = {seg: (p - moved[1][seg]).GetLength() for seg, p in now_at.items() if seg in moved[1]}
            going = {StemCutter.stem_of(Sdf.Path(seg)).name: v for seg, v in d.items() if v > 0.02}
            if d:
                log(f"stem cutter: t={clock[0]:.0f} {len(going)}/{len(d)} cut pieces moved > 2 cm in 5 s"
                    + (f", most {max(going.values()) * 100:.0f} cm ({max(going, key=going.get)})" if going else ""))
            moved[1] = now_at
            # where every cut piece neither in the basket nor on the ground is now (base_link), to find where they end up
            loose = [(StemCutter.stem_of(Sdf.Path(seg)), place(seg)) for seg in now_at]
            loose = [(stem, at, b) for stem, (at, b) in loose if at not in ("basket", "ground")]

            def touching(stem):  # the EE colliders this piece touches now (pads, blade, counter-blade, jaw)
                hit = {"/".join(str(c).split("/")[-2:]) for _, c, sg in getattr(cutter, "contacts", [])
                       if StemCutter.stem_of(sg) == stem and str(c).startswith(gripper_base)}
                return f" touching {', '.join(sorted(hit))}" if hit else ""
            if loose:
                log(f"stem cutter: t={clock[0]:.0f} not in the basket or on the ground: "
                    + ", ".join(f"{stem.name} {at} ({b[0]:+.2f} {b[1]:+.2f} {b[2]:+.2f}){touching(stem)}"
                                for stem, at, b in loose))
        if clock[0] - logged[0] >= 0.1:  # cut pieces: log each change of place
            logged[0] = clock[0]
            for _, seg, _ in cutter.cuts:
                at, b = place(seg)
                if where.get(seg) != at:
                    log(f"stem cutter: t={clock[0]:.1f} {StemCutter.stem_of(Sdf.Path(seg)).name} {where.get(seg, 'cut')} -> {at} "
                        f"(base_link {b[0]:+.2f} {b[1]:+.2f} {b[2]:+.2f})")
                    where[seg] = at

    # after each step (the contact report is filled then; subscribe_physics_step_events runs before it: no contacts),
    # subscribed again on every play: a timeline stop (the fleet's reset) ends the subscription
    # shortcut: kept for the sim's lifetime; a respawn in the same sim adds another cutter (a sim restart clears)
    import omni.timeline

    sub = [get_physx_interface().subscribe_physics_on_step_events(on_step, False, 0)]

    def on_play(_):
        sub[0] = get_physx_interface().subscribe_physics_on_step_events(on_step, False, 0)

    _step_subs.extend([sub, omni.timeline.get_timeline_interface().get_timeline_event_stream()
                       .create_subscription_to_pop_by_type(int(omni.timeline.TimelineEventType.PLAY), on_play)])
    return (f"dynamic lavender: {len(segs)} segments touch only the EE and the basket ({n} robot colliders filtered), cut at "
            f"{cutter.cut_force:g} N blade vs counter-blade, {len(roots)} articulation(s) at 64 position iterations")


EE_SCRIPT = """
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64

import ee_cutter


def setup(db):
    db.per_instance_state.art = None
    db.per_instance_state.node = None


def compute(db):
    st = db.per_instance_state
    if st.art is None:
        from isaacsim.core.experimental.prims import Articulation
        try:
            st.art = Articulation(str(db.inputs.chassisPath))
            names = list(st.art.dof_names)
            st.driver = [names.index(ee_cutter.DRIVER)]
            st.dofs = [names.index(j) for j in ee_cutter.JOINTS]
            st.blade_dof = [names.index("ee_cutter_blade_joint")]
            st.finger_dof = [names.index("ee_cutter_finger_joint")]
        except Exception as e:
            db.log_warning(f"EeCutter: articulation not ready yet ({e}), retrying next tick")
            st.art = None
            return
    if st.node is None:
        if not rclpy.ok():
            rclpy.init()
        ns = str(db.inputs.namespace)
        st.node = Node(f"ee_cutter_{ns}", namespace=ns)
        st.cmd, st.blade, st.t = 0.0, 0.0, None

        def on_blade(msg):
            st.cmd = min(max(float(msg.data), 0.0), 1.0)

        st.sub = st.node.create_subscription(Float64, ee_cutter.BLADE_TOPIC, on_blade, 10)
        st.executor = SingleThreadedExecutor()
        st.executor.add_node(st.node)
        import fleet_nodes  # destroyed by a respawn (cleanup() doesn't run when the graph is deleted)
        fleet_nodes.track(ns, st)
    for _ in range(20):
        st.executor.spin_once(timeout_sec=0.0)
    now = float(db.inputs.stamp)
    dt = 0.0 if st.t is None else min(max(now - st.t, 0.0), 0.1)
    st.t = now
    step = ee_cutter.BLADE_SPEED * dt
    st.blade += min(max(st.cmd - st.blade, -step), step)
    q = float(st.art.get_dof_positions(dof_indices=st.driver).numpy()[0][0])
    t = ee_cutter.targets(q, st.blade)
    if ee_cutter.FINGER_OVERCLOSE:  # past closed in proportion to the 2F Lite's closing, and stalled: a bounded squeeze
        t[4] -= ee_cutter.FINGER_OVERCLOSE * min(max(q / ee_cutter.KINOVA_CLOSED, 0.0), 1.0)
        fi = float(st.art.get_dof_positions(dof_indices=st.finger_dof).numpy()[0][0])
        t[4] = max(t[4], fi - ee_cutter.FINGER_OVERCLOSE)  # closing is negative: at most OVERCLOSE ahead of the finger
    if ee_cutter.BLADE_LAG:  # the blade's target at most BLADE_LAG ahead of the blade: a motor that stalls
        th = float(st.art.get_dof_positions(dof_indices=st.blade_dof).numpy()[0][0])
        t[5] = min(max(t[5], th - ee_cutter.BLADE_LAG), th + ee_cutter.BLADE_LAG)
        t[6] = max(ee_cutter.SPUR_RATIO * t[5], -ee_cutter.SPUR_LIMIT)
        if ee_cutter.BLADE_LOG_DIR:  # diagnostic: the drive's side of the blade (its torque = stiffness x (target - angle))
            if getattr(st, "blade_log", None) is None:
                import os
                os.makedirs(ee_cutter.BLADE_LOG_DIR, exist_ok=True)
                st.blade_log = open(os.path.join(ee_cutter.BLADE_LOG_DIR, "drive.csv"), "w")
                st.blade_log.write("t,command,target_rad,angle_rad\\n")
            st.blade_log.write(f"{now:.3f},{st.blade:.3f},{t[5]:.4f},{th:.4f}\\n")
            st.blade_log.flush()
    st.art.set_dof_position_targets(np.array([t]), dof_indices=st.dofs)


def cleanup(db):
    st = db.per_instance_state
    if st.node is not None:
        st.executor.shutdown()
        st.node.destroy_node()
        st.node = None
"""


if __name__ == "__main__":  # self-check of the linkage solution against the CAD measurements
    assert all(abs(v) < 1e-9 for v in targets(0.0, 0.0)), targets(0.0, 0.0)
    closed = targets(KINOVA_CLOSED, 1.0)
    assert abs(math.degrees(closed[4]) + 88.7) < 0.5, math.degrees(closed[4])  # finger_tip at the jaw's horn
    assert abs(closed[5] - BLADE_CLOSED) < 1e-9 and abs(closed[6] + SPUR_LIMIT) < 1e-12
    prev = targets(0.0, 0.0)
    for i in range(1, 97):  # continuous over the stroke (the solver stays on the assembly's branch)
        cur = targets(i * 0.01, 0.0)
        assert all(abs(a - b) < math.radians(3) for a, b in zip(cur[1:5], prev[1:5])), (i, cur, prev)
        prev = cur
    thick = [(f - j - PAD_GAP) / 2 for j, f in zip(PAD_JAW, PAD_FINGER)]
    assert min(thick) > 2 and PAD_GAP < 2, thick  # both pads there, closed they squeeze a 2 mm stem
    assert all(len(q) == len(PAD_X) - 1 for q in pad_quads().values())
    print("pads %.1f-%.1f mm thick, %.1f mm apart closed" % (min(thick), max(thick), PAD_GAP))
    print("ee_cutter ok: closed slide %.1f mm, pusher %.1f, link_1 %.1f, link_2 %.1f, finger %.1f deg" % (
        closed[0] * 1000, *(math.degrees(a) for a in closed[1:5])))

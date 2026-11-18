"""Cut lavender stems (sim/assets/lavender_dyn, make_lavender_plant.py) with a blade: PhysX only.

StemCutter watches the contact reports of the blade (and anvil) prims; when a segment is pushed with more than
`cut_force` N for `hold_steps` consecutive physics steps, the joint below that segment (`joint_<i>` next to
`seg_<i>`) is disabled, so the segment and everything above it become a free body: held by whatever grips it,
or falling. Call step(dt) once per physics step, after the step (the report covers the last step).

The force is the blade's contact impulse / dt, or, with `blade_force` (a function returning the force the blade's
drive applies now, N), that drive force while the segment touches a blade and, with `anvil_paths`, an anvil at
the same time: a stem trapped between blade and counter-blade takes the motor's force (several pinched stems share
the drive's force with every segment on the blade, in proportion to the contact impulses). A stem is cut once. PhysX's per-step contact
impulses on a 5 mm stem squeezed between plates jump by an order of magnitude from step to step.
A blade or anvil path is a rigid body or a collider prim under one (e.g. a counter-blade on a gripper's base link): a
contact matches it by its body or by its collider's path; the contact report goes on the body.
A segment is a seg_<i> body (stem models 1, 2: cut = its joint_<i> disabled) or a seg_<i> collider group of a stem's
single body (stem model 3, the stem prim's customData cut_mode "swap": the body is replaced by pre-built ones, see
make_lavender_plant.single_body_stem and swap()).
"""
import math

from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics


class StemCutter:
    def __init__(self, stage, blade_paths, cut_force, hold_steps=2, log=print, anvil_paths=(), blade_force=None,
                 touch=0.0005):
        self.stage, self.cut_force, self.hold_steps, self.log = stage, cut_force, hold_steps, log
        self.blades = [Sdf.Path(p) for p in blade_paths]
        self.anvils = [Sdf.Path(p) for p in anvil_paths]
        self.blade_force = blade_force
        self.touch = touch  # m: a contact point counts within this separation (PhysX also reports ones ~1 cm off)
        for p in self.blades + self.anvils:
            body = stage.GetPrimAtPath(p)
            while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):
                body = body.GetParent()
            PhysxSchema.PhysxContactReportAPI.Apply(body).CreateThresholdAttr(0.0)
        self._above = {}  # segment path -> consecutive steps over cut_force
        self.cut_stems = set()
        self.cuts = []  # (sim_time, segment path, force)
        import omni.physx
        self._sim = omni.physx.get_physx_simulation_interface()

    @staticmethod
    def segment_of(actor, collider):
        """The seg_<i> path a contact belongs to: the body itself (models 1, 2) or the collider's seg_<i> ancestor."""
        if actor.name.startswith("seg_"):
            return actor
        p = collider
        while p.pathElementCount > 1 and not p.name.startswith("seg_"):
            p = p.GetParentPath()
        return p if p.name.startswith("seg_") else None

    @staticmethod
    def stem_of(seg_path):
        """The stem_<k> ancestor, else the segment's parent (plant_test's probes)."""
        p = seg_path
        while p.pathElementCount > 1 and not p.name.startswith("stem_"):
            p = p.GetParentPath()
        return p if p.name.startswith("stem_") else seg_path.GetParentPath()

    @staticmethod
    def joint_of(seg_path):
        """The joint below a segment: /.../stem_<k>/seg_<i> -> /.../stem_<k>/joint_<i>."""
        return seg_path.GetParentPath().AppendChild(seg_path.name.replace("seg_", "joint_"))

    def step(self, dt, sim_time=0.0):
        headers, data = self._sim.get_contact_report()
        self.n_headers = len(headers)
        force, on_anvil = {}, set()  # segment -> total blade contact force this step; segments on an anvil
        self.contacts = []  # (body, collider, segment) of every touching segment contact this step, for diagnostics
        for h in headers:
            a0 = PhysicsSchemaTools.intToSdfPath(h.actor0)
            a1 = PhysicsSchemaTools.intToSdfPath(h.actor1)
            c0 = PhysicsSchemaTools.intToSdfPath(h.collider0)
            c1 = PhysicsSchemaTools.intToSdfPath(h.collider1)
            points = data[h.contact_data_offset:h.contact_data_offset + h.num_contact_data]
            if not any(d.separation <= self.touch for d in points):
                continue
            for tool, collider, other, other_collider in ((a0, c0, a1, c1), (a1, c1, a0, c0)):
                seg = self.segment_of(other, other_collider)
                if seg is None:
                    continue
                self.contacts.append((tool, collider, seg))
                if any(tool == t or collider.HasPrefix(t) for t in self.anvils):
                    on_anvil.add(seg)
                elif any(tool == t or collider.HasPrefix(t) for t in self.blades):
                    imp = sum(math.sqrt(d.impulse[0] ** 2 + d.impulse[1] ** 2 + d.impulse[2] ** 2) for d in points)
                    force[seg] = force.get(seg, 0.0) + imp / dt
        if self.blade_force is not None:  # the drive's force, shared by every segment on the blade by its impulse
            f_drive = self.blade_force()  # (one being pushed over takes its share too); only pinched ones count
            total = sum(force.values())
            force = {seg: (f_drive * (f / total if total > 0 else 1 / len(force))
                           if not self.anvils or seg in on_anvil else 0.0) for seg, f in force.items()}
        for seg, f in force.items():
            if self.stem_of(seg) in self.cut_stems:  # a blade edge cuts a stem once
                continue
            n = self._above.get(seg, 0) + 1 if f >= self.cut_force else 0
            self._above[seg] = n
            if n >= self.hold_steps:
                self.cut(seg, f, sim_time)
        for seg in list(self._above):
            if seg not in force:
                del self._above[seg]
        return force

    def cut(self, seg, f=0.0, sim_time=0.0):
        stem = self.stem_of(seg)
        if self.stage.GetPrimAtPath(stem).GetCustomDataByKey("cut_mode") == "swap":
            piece = self.swap(stem, seg)
            if piece is None:
                return False
        else:
            joint = self.stage.GetPrimAtPath(self.joint_of(seg))
            if not joint or not joint.GetAttribute("physics:jointEnabled").Get(Usd.TimeCode.Default()) in (None, True):
                return False
            joint.GetAttribute("physics:jointEnabled").Set(False)
            piece = seg
        self.cut_stems.add(stem)
        self.cuts.append((sim_time, str(piece), f))
        self._above.pop(seg, None)
        self.log(f"cut {seg} at {stem.name} z={self.stage.GetPrimAtPath(piece).GetCustomDataByKey('cut_z')}"  # seg may be inactive now
                 f" ({f:.1f} N, t={sim_time:.2f} s)")
        return True

    def swap(self, stem, seg):
        """Stem model 3: cut the single body `whole` between segments i-1 and i (i = 0: at its spring joint) by
        activating the pre-built upper_<i> (free) and lower_<i> + joint_lower_<i> (the stump on the spring) at the
        whole body's current pose and deactivating whole + joint_0. Returns the cut piece's seg path, or None.
        shortcut: the new bodies start at rest (the whole body's velocity is not carried over); fine for a stem
        pinched between blade and anvil, add it if cut pieces are thrown."""
        if seg.GetParentPath().name != "whole":  # already cut: a piece is cut once
            return None
        i = int(seg.name[4:])
        from omni.physx import get_physx_interface
        tr = get_physx_interface().get_rigidbody_transformation(str(seg.GetParentPath()))
        p, q = tr["position"], tr["rotation"]  # world, quaternion x y z w
        world = Gf.Matrix4d(Gf.Rotation(Gf.Quatd(q[3], q[0], q[1], q[2])), Gf.Vec3d(p[0], p[1], p[2]))
        parent = UsdGeom.XformCache().GetLocalToWorldTransform(self.stage.GetPrimAtPath(stem))
        local = world * parent.GetInverse()
        new = [stem.AppendChild(f"upper_{i}")] + ([stem.AppendChild(f"lower_{i}"), stem.AppendChild(f"joint_lower_{i}")] if i else [])
        with Sdf.ChangeBlock():
            for path in new[:2]:
                prim = self.stage.GetPrimAtPath(path)
                prim.GetAttribute("xformOp:translate").Set(local.ExtractTranslation())
                prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(local.ExtractRotationQuat()))
        for path in (stem.AppendChild("whole"), stem.AppendChild("joint_0")):
            self.stage.GetPrimAtPath(path).SetActive(False)
        for path in new:
            self.stage.GetPrimAtPath(path).SetActive(True)
        return stem.AppendChild(f"upper_{i}").AppendChild(seg.name)

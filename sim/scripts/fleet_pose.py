"""World poses of prims under either physics engine (SIM_PHYSICS, setup_scene.py).

PhysX writes the simulated poses back to USD, so `UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)` is the
pose of a moving body. Newton (isaacsim.physics.newton) writes them only to Fabric (`omni:fabric:worldMatrix` on
each rigid body prim, FabricManager.update_fabric): USD keeps the spawn pose, so every script node that reads a
pose per frame (VelCtl, BodyDrive, GPS, RefPose, the lidars, the IMU) goes through world_matrix() here.

Under Newton the pose of a prim is its nearest rigid-body ancestor's Fabric world matrix times the prim's offset
from that body, read once from USD (fixed: sensor frames never move relative to their link). Bodies themselves are
read straight from Fabric (IFabricHierarchy.get_world_xform, what isaacsim.core.experimental.prims' XformPrim does
with its usdrt backend). Before the first physics step Fabric still holds the USD pose, so the result is the same
as USD's.
"""
import math
import os

import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

_engine = None
_hierarchy = None
_hierarchy_stage_id = None
_anchors = {}  # prim path -> (body path, offset body -> prim as Gf.Matrix4d, or None for a body / no body)


def engine():
    """'physx' or 'newton': the engine actually active (SimulationManager), else the one SIM_PHYSICS asks for."""
    global _engine
    if _engine is None:
        try:
            from isaacsim.core.simulation_manager import SimulationManager
            _engine = SimulationManager.get_active_physics_engine()
        except Exception:
            _engine = os.environ.get("SIM_PHYSICS", "physx")
    return _engine


def reset():
    """Forget cached anchors and the engine (a new stage, a respawn)."""
    global _engine, _hierarchy, _hierarchy_stage_id
    _engine = None
    _hierarchy = None
    _hierarchy_stage_id = None
    _anchors.clear()


def world_matrix(path, stage=None):
    """The prim's local-to-world transform (Gf.Matrix4d, row vectors) as simulated now."""
    stage = stage or omni.usd.get_context().get_stage()
    if engine() != "newton":
        return UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)
    body, offset = _anchor(path, stage)
    world = _fabric_world(body)
    return offset * world if offset is not None else world


def ray_caster():
    """A cast(origins, directions, max_distance) -> distances (numpy, +Inf for a miss) for the lidar scripts:
    PhysX's scene-query interface (one raycast_closest per ray), or Newton's raytracer (fleet_newton, one kernel
    launch per call); None while Newton hasn't parsed the scene yet. origins/directions: (n, 3), world space."""
    if engine() == "newton":
        import fleet_newton
        rc = fleet_newton.raycaster()
        return rc.cast if rc is not None else None
    import carb
    import numpy as np
    from omni.physx import get_physx_scene_query_interface

    sqi = get_physx_scene_query_interface()

    def cast(origins, directions, max_distance):
        out = np.full(len(origins), math.inf)
        for i, ((sx, sy, sz), (wx, wy, wz)) in enumerate(zip(origins.tolist(), directions.tolist())):
            hit = sqi.raycast_closest(carb.Float3(sx, sy, sz), carb.Float3(wx, wy, wz), max_distance)
            if hit["hit"]:
                out[i] = hit["distance"]
        return out

    return cast


def _anchor(path, stage):
    path = str(path)
    if path not in _anchors:
        prim = stage.GetPrimAtPath(path)
        body = prim
        while body.IsValid() and not body.HasAPI(UsdPhysics.RigidBodyAPI):
            body = body.GetParent()
        if not body.IsValid() or body == prim:
            _anchors[path] = (path, None)
        else:
            prim_world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            body_world = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(0)
            _anchors[path] = (str(body.GetPath()), prim_world * body_world.GetInverse())
    return _anchors[path]


def _fabric_world(path):
    global _hierarchy, _hierarchy_stage_id
    import usdrt
    import usdrt.hierarchy

    stage_id = omni.usd.get_context().get_stage_id()
    if _hierarchy is None or _hierarchy_stage_id != stage_id:
        fabric_stage = usdrt.Usd.Stage.Attach(stage_id)
        _hierarchy = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(
            fabric_stage.GetFabricId(), fabric_stage.GetStageIdAsStageId())
        _hierarchy_stage_id = stage_id
    m = _hierarchy.get_world_xform(usdrt.Sdf.Path(str(path)))
    return Gf.Matrix4d(*[float(m[i][j]) for i in range(4) for j in range(4)])

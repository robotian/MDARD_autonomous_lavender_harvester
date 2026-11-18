"""Newton-engine helpers for the robot script nodes (SIM_PHYSICS=newton, setup_scene.py).

Lidar raycasts: under PhysX the lidar scripts cast with omni.physx's scene-query interface; under Newton PhysX
simulates nothing, so the rays go to Newton's own GPU raytracer (newton.sensors.SensorTiledCamera over the
Newton model's shapes, one "camera" whose pixels are the lidar's rays, given in world space). The sensor is built
once per Newton model (it loads every shape's geometry) and its shape BVH refitted once per physics step; a scan
then costs one kernel launch instead of one Python call per ray. Shapes without Newton's VISIBLE flag (collision-
only geometry such as the lavender cores) aren't hit; visual meshes are, so foliage and the robot's own body show
up as under PhysX.
"""
import math

import numpy as np

_raycasters = {}  # id(model) -> Raycaster


def newton_stage():
    """The extension's NewtonStage once it has parsed the scene (first physics step), else None."""
    try:
        from isaacsim.physics.newton import acquire_stage
    except ImportError:
        return None
    ns = acquire_stage()
    return ns if ns is not None and ns.initialized and ns.model is not None else None


def raycaster():
    """The Raycaster for the current Newton model, or None until the model exists."""
    ns = newton_stage()
    if ns is None:
        return None
    key = id(ns.model)
    if key not in _raycasters:
        _raycasters.clear()  # a re-parsed model (respawn): the old sensor's geometry is stale
        _raycasters[key] = Raycaster(ns)
    return _raycasters[key]


class Raycaster:
    def __init__(self, ns):
        import newton
        import warp as wp
        from newton.sensors import SensorTiledCamera

        self.ns = ns
        self.wp = wp
        self.sensor = SensorTiledCamera(ns.model, load_textures=False)
        newton.geometry.build_bvh_shape(ns.model, ns.state_0)
        self.refit_step = ns.simulation_step_count
        identity = wp.transformf(wp.vec3f(0.0, 0.0, 0.0), wp.quatf(0.0, 0.0, 0.0, 1.0))
        self.camera = wp.array([[identity]], dtype=wp.transformf)  # (world, camera): rays already in world space
        self.buffers = {}  # ray count -> (rays (1, 1, n, 2) vec3f, depth (1, 1, 1, n) float32)

    def cast(self, origins, directions, max_distance):
        """Distance along each ray (numpy, +Inf for a miss or beyond max_distance). origins/directions: (n, 3)
        in world space, directions unit length."""
        wp, ns = self.wp, self.ns
        n = len(origins)
        if n not in self.buffers:
            self.buffers[n] = (wp.empty((1, 1, n, 2), dtype=wp.vec3f), wp.empty((1, 1, 1, n), dtype=wp.float32))
        rays, depth = self.buffers[n]
        packed = np.empty((1, 1, n, 2, 3), dtype=np.float32)
        packed[0, 0, :, 0, :] = origins
        packed[0, 0, :, 1, :] = directions
        wp.copy(rays, wp.array(packed, dtype=wp.vec3f, shape=(1, 1, n, 2)))
        # refit the BVH to the bodies' current poses once per physics step, however many lidars cast in it
        refit = ns.simulation_step_count != self.refit_step
        self.refit_step = ns.simulation_step_count
        self.sensor.update(ns.state_0, self.camera, rays, depth_image=depth, refit_bvh=refit)
        d = depth.numpy()[0, 0, 0].astype(np.float64)
        d[(d <= 0.0) | (d > max_distance)] = math.inf
        return d


def debug_report(limit=48):
    """FLEET_DEBUG=1 (setup_scene.debug_loop): what Newton made of the stage, as text. Bodies with their world z,
    shapes (type, body, world z, flags), and the contact pipeline's state: MuJoCo's geoms (type, size, world z, AABB
    extent in z) and its active contacts, or Newton's rigid contacts. Every section is best effort."""
    ns = newton_stage()
    if ns is None:
        return "newton: no model yet (first physics step pending)"
    import warp as wp

    out = []
    try:
        m, st = ns.model, ns.state_0
        cfg = ns.cfg.solver_cfg
        out.append(f"newton model: {m.body_count} bodies, {m.shape_count} shapes, {m.joint_count} joints; solver "
                   f"{cfg.solver_type}, mujoco contacts {getattr(cfg, 'use_mujoco_contacts', None)}, "
                   f"dt {ns.sim_dt:.4f}")
        body_q = st.body_q.numpy()
        for b in range(min(m.body_count, limit)):
            qx, qy, qz, qw = (float(v) for v in body_q[b][3:7])
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))))
            roll = math.degrees(math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)))
            out.append(f" body {b:2d} z={body_q[b][2]:8.3f} roll={roll:6.1f} pitch={pitch:6.1f} {m.body_label[b][-70:]}")
        stype, sbody = m.shape_type.numpy(), m.shape_body.numpy()
        sxf, sscale, sflags = m.shape_transform.numpy(), m.shape_scale.numpy(), m.shape_flags.numpy()
        for s in range(min(m.shape_count, limit)):
            b = int(sbody[s])
            local = wp.transform(*[float(v) for v in sxf[s]])
            world = wp.transform_multiply(wp.transform(*[float(v) for v in body_q[b]]), local) if b >= 0 else local
            out.append(f" shape {s:2d} type={int(stype[s])} body={b:2d} z={world[2]:8.3f} scale={tuple(round(float(v), 3) for v in sscale[s])} "
                       f"flags={int(sflags[s])} {m.shape_label[s] if s < len(m.shape_label) else ''}")
    except Exception as e:
        out.append(f" model read failed: {e!r}")
    solver = getattr(ns, "solver", None)
    try:
        mjm, mjd = solver.mj_model, solver.mjw_data
        gxpos = mjd.geom_xpos.numpy()[0]
        aabb = getattr(mjm, "geom_aabb", None)
        ncon_arr = next((getattr(mjd, k) for k in ("ncon", "nacon", "ncollision") if hasattr(mjd, k)), None)
        ncon = int(ncon_arr.numpy().reshape(-1)[0]) if ncon_arr is not None else -1
        out.append(f"mujoco: {mjm.nbody} bodies, {mjm.ngeom} geoms, {mjm.nmesh} meshes, ncon={ncon}")
        for g in range(min(mjm.ngeom, limit)):
            ext = ""
            if aabb is not None:
                c, h = aabb[g][:3], aabb[g][3:]
                ext = f" aabb_z=[{c[2] - h[2]:.3f},{c[2] + h[2]:.3f}]"
            out.append(f" geom {g:2d} type={int(mjm.geom_type[g])} body={int(mjm.geom_bodyid[g])} xz={gxpos[g][2]:8.3f} "
                       f"size={tuple(round(float(v), 3) for v in mjm.geom_size[g])} pos={tuple(round(float(v), 3) for v in mjm.geom_pos[g])}{ext} "
                       f"contype={int(mjm.geom_contype[g])} conaff={int(mjm.geom_conaffinity[g])} "
                       f"friction={tuple(round(float(v), 3) for v in mjm.geom_friction[g])} condim={int(mjm.geom_condim[g])} "
                       f"priority={int(mjm.geom_priority[g])} solref={tuple(round(float(v), 4) for v in mjm.geom_solref[g])} "
                       f"solmix={float(mjm.geom_solmix[g]):.2f}")
        out.append(f" mujoco opt: timestep={float(mjm.opt.timestep):.5f} cone={int(mjm.opt.cone)} impratio={float(mjm.opt.impratio):.2f} "
                   f"noslip_iterations={int(mjm.opt.noslip_iterations)}")
        geom = mjd.contact.geom.numpy()[0] if hasattr(mjd.contact, "geom") else None
        dist = mjd.contact.dist.numpy()[0] if hasattr(mjd.contact, "dist") else None
        pos = mjd.contact.pos.numpy()[0] if hasattr(mjd.contact, "pos") else None
        for c in range(min(ncon, limit)):
            out.append(f" contact {c:2d} geoms={tuple(int(v) for v in geom.reshape(-1, 2)[c])} dist={float(dist.reshape(-1)[c]):.4f} "
                       f"pos={tuple(round(float(v), 3) for v in pos.reshape(-1, 3)[c])}")
    except Exception as e:
        out.append(f" mujoco read failed: {e!r}")
    try:
        ct = ns.contacts
        n = int(ct.rigid_contact_count.numpy()[0])
        s0, s1 = ct.rigid_contact_shape0.numpy(), ct.rigid_contact_shape1.numpy()
        p0 = ct.rigid_contact_point0.numpy()
        out.append(f"newton contacts: {n}")
        for c in range(min(n, limit)):
            out.append(f" contact {c:2d} shapes=({int(s0[c])},{int(s1[c])}) p0={tuple(round(float(v), 3) for v in p0[c])}")
    except Exception as e:
        out.append(f" newton contacts read failed: {e!r}")
    return "\n".join(out)

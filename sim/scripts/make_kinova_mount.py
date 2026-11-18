"""Kinova Gen3 lite Quick-Connect & Base Mount Kit (KR13132-01), modelled from photos -> sim/assets/kinova_base_mount/.

Visual only. Sizes from the photos scaled by the arm base (79 mm across): a black anodised bell-shaped base (106 mm
across, four arched windows with the bolts that hold it down), the quick-connect collar (72 mm) with its cam lever,
and the grey top plate the arm sits on. 48 mm tall: the gap between j100_0921's top plate and arm_0_base_link (its
URDF arm pose), so the mount's top is the arm base's bottom face. Origin: centre of the bottom face, z up, metres.
  blender -b --python sim/scripts/make_kinova_mount.py
"""
import math
import os

import bmesh
import bpy

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "kinova_base_mount",
                   "kinova_base_mount.usd")
HEIGHT = 0.048
MM = 0.001
WINDOWS = [math.radians(a) for a in (45, 135, 225, 315)]
LEVER = (math.radians(-100), math.radians(10))  # arc of the cam lever around the collar; hinge at the far end
# name: (base colour (linear), metallic, roughness)
MATERIALS = {"anodized_black": ((0.022, 0.022, 0.024), 1.0, 0.42), "anodized_grey": ((0.10, 0.10, 0.105), 1.0, 0.4),
             "plastic_black": ((0.015, 0.015, 0.016), 0.0, 0.35), "steel": ((0.62, 0.62, 0.64), 1.0, 0.28)}


def material(name):
    m = bpy.data.materials.get(name)
    if m is None:
        color, metal, rough = MATERIALS[name]
        m = bpy.data.materials.new(name)
        bsdf = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
        bsdf.inputs["Base Color"].default_value = (*color, 1)
        bsdf.inputs["Metallic"].default_value = metal
        bsdf.inputs["Roughness"].default_value = rough
    return m


def obj(name, bm, mat):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    me.materials.append(material(mat))
    return o


def revolve(profile, steps=96):
    """Closed (r, z) profile in mm spun about z."""
    bm = bmesh.new()
    vs = [bm.verts.new((r * MM, 0, z * MM)) for r, z in profile]
    edges = [bm.edges.new((vs[i], vs[(i + 1) % len(vs)])) for i in range(len(vs))]
    bmesh.ops.spin(bm, geom=vs + edges, axis=(0, 0, 1), cent=(0, 0, 0), angle=2 * math.pi, steps=steps)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-7)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


def cylinder(r, z0, z1, segments=48, at=(0, 0), axis=None):
    """Cylinder (mm) along z at `at`, or along the radial direction `axis` (angle) when given."""
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=segments, radius1=r * MM, radius2=r * MM, depth=(z1 - z0) * MM)
    bmesh.ops.translate(bm, verts=bm.verts, vec=(0, 0, (z0 + z1) / 2 * MM))
    if axis is not None:  # lay it along x, then turn it to the angle
        from mathutils import Matrix
        bmesh.ops.rotate(bm, verts=bm.verts, matrix=Matrix.Rotation(math.pi / 2, 3, "Y"))
        bmesh.ops.rotate(bm, verts=bm.verts, matrix=Matrix.Rotation(axis, 3, "Z"))
    bmesh.ops.translate(bm, verts=bm.verts, vec=(at[0] * MM, at[1] * MM, 0))
    return bm


def boolean(target, cutters, op="DIFFERENCE"):
    for c in cutters:
        m = target.modifiers.new("b", "BOOLEAN")
        m.operation, m.solver, m.object = op, "EXACT", c
        bpy.context.view_layer.objects.active = target
        bpy.ops.object.modifier_apply(modifier=m.name)
        bpy.data.objects.remove(c)


def window(a):
    """Arched window through the bell's wall at angle a: a 15 mm slot from z 7, round top at z 19 (r 7.5); two
    cutters (a box and a radial cylinder), since one mesh of two overlapping shells breaks the exact boolean."""
    from mathutils import Matrix
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=(0.034, 0.015, 0.012), verts=bm.verts)
    bmesh.ops.translate(bm, vec=(0.046, 0, 0.013), verts=bm.verts)
    bmesh.ops.rotate(bm, verts=bm.verts, matrix=Matrix.Rotation(a, 3, "Z"))
    top = cylinder(7.5, 29, 63, axis=a)
    bmesh.ops.translate(top, verts=top.verts, vec=(0, 0, 0.019))
    return [obj("w", bm, "steel"), obj("w", top, "steel")]


def sharp_edges(o, deg=35):
    for p in o.data.polygons:
        p.use_smooth = True
    try:
        o.data.set_sharp_from_angle(angle=math.radians(deg))  # Blender >= 4.1
    except AttributeError:
        o.data.use_auto_smooth, o.data.auto_smooth_angle = True, math.radians(deg)


bpy.ops.wm.read_factory_settings(use_empty=True)
root = bpy.data.objects.new("kinova_base_mount", None)
bpy.context.scene.collection.objects.link(root)

# bell: quarter-ellipse shoulder from the foot (r 53, z 5) up to the collar (r 36.5, z 28), 4 mm wall, bolt floor
outer = [(36.5 + 16.5 * math.cos(t), 5 + 23 * math.sin(t)) for t in (math.pi / 2 * k / 16 for k in range(17))]
inner = [(32.5 + 15.5 * math.cos(t), 4 + 20 * math.sin(t)) for t in (math.pi / 2 * k / 16 for k in range(17))]
bell = obj("base", revolve([(30, 0), (53, 0)] + outer + [(32.5, 28)] + inner[::-1][:-1] + [(47.9, 4), (30, 4)]),
           "anodized_black")
boolean(bell, [c for a in WINDOWS for c in window(a)])
parts = [bell]
for a in WINDOWS:  # button-head bolts on the floor inside each window
    x, y = 41 * math.cos(a), 41 * math.sin(a)
    parts.append(obj("bolt", revolve([(0, 4), (4.75, 4), (4.75, 4.8), (3.8, 6.2), (0, 6.6)], 32), "steel"))
    bmesh_t = parts[-1]
    bmesh_t.location = (x * MM, y * MM, 0)
parts.append(obj("collar", revolve([(0, 28), (36, 28), (36, 44), (0, 44)]), "anodized_black"))
parts.append(obj("top_plate", revolve([(0, 44), (35.5, 44), (35.5, 47.6), (35.1, 48), (0, 48)]), "anodized_grey"))
# cam lever: a 5 mm band around the collar, its hinge boss and pin at the far end
n = 24
lever = [(r * math.cos(LEVER[0] + (LEVER[1] - LEVER[0]) * k / n), r * math.sin(LEVER[0] + (LEVER[1] - LEVER[0]) * k / n))
         for r, ks in ((41.0, range(n + 1)), (36.2, range(n, -1, -1))) for k in ks]
bm = bmesh.new()
bot = [bm.verts.new((x * MM, y * MM, 31 * MM)) for x, y in lever]
top = [bm.verts.new((x * MM, y * MM, 41 * MM)) for x, y in lever]
bm.faces.new(bot[::-1])
bm.faces.new(top)
for i in range(len(lever)):
    j = (i + 1) % len(lever)
    bm.faces.new((bot[i], bot[j], top[j], top[i]))
bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
parts.append(obj("lever", bm, "plastic_black"))
hinge = cylinder(5.5, 35, 45, axis=LEVER[1])
bmesh.ops.translate(hinge, verts=hinge.verts, vec=(0, 0, 0.036))
parts.append(obj("lever_hinge", hinge, "plastic_black"))
pin = cylinder(3.0, 45, 46.2, axis=LEVER[1])
bmesh.ops.translate(pin, verts=pin.verts, vec=(0, 0, 0.036))
parts.append(obj("lever_pin", pin, "steel"))
for o in parts:
    sharp_edges(o)
    o.parent = root

top_z = max((o.matrix_world @ v.co).z for o in parts for v in o.data.vertices)
assert abs(top_z - HEIGHT) < 1e-6, top_z  # self-check: the arm base sits on the top plate
os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.usd_export(filepath=OUT, export_materials=True, generate_preview_surface=True, root_prim_path="/root",
                      export_lights=False, export_cameras=False)
print("wrote", OUT, sum(len(o.data.polygons) for o in parts), "faces")

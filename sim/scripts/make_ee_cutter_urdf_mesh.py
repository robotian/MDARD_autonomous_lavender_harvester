"""The cutter EE for the robot description (mtu32_description: RViz, Foxglove, MoveIt's display) as Wavefront OBJ +
.mtl (Blender 5 has no Collada exporter): ee_cutter.obj (the static parts) and one ee_cutter_<body>.obj per moving body
of sim/scripts/ee_cutter.py's BODIES, each in its joint's frame (the CAD pivot, axes of
arm_0_gripper_gripper_base_link: CAD x -> y, y -> z, z -> x, +2.2 mm in z). Flat colours per part (the baked
textures are the sim's), open pose, decimated. Prints the joint origins for robot_description_j100.urdf.xacro.
  blender -b sim/assets/ee_cutter/cutter_gripper.blend --python sim/scripts/make_ee_cutter_urdf_mesh.py
"""
import os
import sys

import bpy
from mathutils import Matrix, Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ee_cutter  # noqa: E402  (BODIES: the moving parts, their parents and pivots)

MESHES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "colcon_ws", "src", "mtu32_husky",
                      "mtu32_description", "meshes")
TRIANGLES = 60000  # all files together
# material name fragment (the baked materials are <prefix>_<part>, fasteners "screw"/"pin") -> sRGB colour; first wins
COLOURS = [("prunner_blade", (0.86, 0.68, 0.36)), ("prunner_rcv", (0.86, 0.68, 0.36)),
           ("Spur_Gear1", (0.70, 0.70, 0.72)), ("blade_shaft", (0.70, 0.70, 0.72)),
           ("screw", (0.72, 0.72, 0.74)), ("pin", (0.78, 0.78, 0.80)),
           ("XMH-430", (0.10, 0.10, 0.11)), ("D405_bracket_abs", (0.07, 0.07, 0.075)),
           ("cutter_fixture", (0.45, 0.45, 0.47)), ("hall_sensor_board", (0.05, 0.35, 0.12)), ("", (0.22, 0.22, 0.23))]
TO_GRIPPER = Matrix(((0, 0, 1, 0), (1, 0, 0, 0), (0, 1, 0, 0.0022), (0, 0, 0, 1)))


def part_key(o):
    """The part's prim name in the USD (BODIES' names): STEP product, ',' and ' ' -> '_', ':n' -> '_n'."""
    while o and o.name.startswith("=>"):  # unnamed sub-shapes sit under their part's empty
        o = o.parent
    name, _, n = o.name.rpartition(":")
    return f"{name.replace(',', '_').replace(' ', '_')}_{n}"


def pivot_world(body):
    """A BODIES entry's pivot in the gripper frame."""
    _, _, _, (x, y), *_ = ee_cutter.BODIES[body]
    return TO_GRIPPER @ Vector((x, y, 0))


def pivot(body):
    """A BODIES entry's joint origin in its parent's frame (the gripper frame for "base")."""
    parent = ee_cutter.BODIES[body][1]
    return pivot_world(body) - (pivot_world(parent) if parent != "base" else Vector())


flat = {}
for name, srgb in COLOURS:
    m = bpy.data.materials.new(f"ee_{name or 'plastic'}")
    bsdf = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = (*srgb, 1)  # the .mtl's Kd as is: RViz / Foxglove show it as sRGB
    bsdf.inputs["Roughness"].default_value = 0.5
    flat[name] = m
body_of = {p.split("/")[-1]: b for b, (parts, *_) in ee_cutter.BODIES.items() for p in parts}
groups = {}
for o in [o for o in bpy.context.scene.objects if o.type == "MESH" and o.data.polygons]:
    o.data = o.data.copy()  # instanced parts share a mesh
    for i, slot in enumerate(o.material_slots):
        name = slot.material.name if slot.material else ""
        o.data.materials[i] = flat[next(k for k, _ in COLOURS if k in name)]
    groups.setdefault(body_of.get(part_key(o), ""), []).append(o)
assert set(groups) == set(ee_cutter.BODIES) | {""}, sorted(groups)  # self-check: every moving part found
total = sum(len(p.vertices) - 2 for objs in groups.values() for o in objs for p in o.data.polygons)
for body, objs in groups.items():
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.object.convert(target="MESH")  # bakes parenting into each object's world transform
    bpy.ops.object.join()
    ee = bpy.context.view_layer.objects.active
    ee.parent = None
    origin = pivot_world(body) if body else Vector()
    ee.matrix_world = Matrix.Translation(-origin) @ TO_GRIPPER @ ee.matrix_world
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    tris = sum(len(p.vertices) - 2 for p in ee.data.polygons)
    dec = ee.modifiers.new("dec", "DECIMATE")
    dec.ratio = min(1.0, TRIANGLES / total)
    bpy.ops.object.modifier_apply(modifier=dec.name)
    bpy.ops.object.select_all(action="DESELECT")
    ee.select_set(True)
    out = os.path.abspath(os.path.join(MESHES, f"ee_cutter_{body}.obj" if body else "ee_cutter.obj"))
    bpy.ops.wm.obj_export(filepath=out, export_selected_objects=True, apply_modifiers=True,
                          export_triangulated_mesh=True, export_uv=False, export_materials=True,
                          forward_axis="Y", up_axis="Z")  # keep the frame as is (OBJ's default turns Z-up into Y-up)
    joint = f" joint origin {' '.join(f'{v:.5f}' for v in pivot(body))}" if body else ""
    print(f"wrote {out}: {tris} -> {sum(len(p.vertices) - 2 for p in ee.data.polygons)} triangles{joint}")

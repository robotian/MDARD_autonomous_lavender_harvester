"""EE cutter looks: glTF from step_to_glb.py -> Blender materials baked to textures -> <out>.usd + <out>.blend.

Procedural materials (gold-coated pruner blades, steel, dark printed plastic with layer lines, black anodized Dynamixel, PCB) baked per part into
base colour / roughness / normal maps, because UsdPreviewSurface (what Isaac reads) has no procedural nodes.
  blender -b --python sim/scripts/make_ee_cutter_looks.py -- sim/generated/ee_cutter/cutter.glb sim/assets/ee_cutter/cutter
An optional third glb (make_d405_bracket.py's) replaces the assembly's old D405_bracket:
  blender -b --python sim/scripts/make_ee_cutter_looks.py -- sim/generated/ee_cutter/cutter_and_gripper_and_camera_bracket.glb \
      sim/assets/ee_cutter/cutter_gripper sim/generated/ee_cutter/d405_bracket.glb
"""
import math
import os
import sys

import bpy

src, out, *bracket = sys.argv[sys.argv.index("--") + 1:]
out = os.path.abspath(out)
tex_dir = os.path.join(os.path.dirname(out), "textures")
os.makedirs(tex_dir, exist_ok=True)
prefix = os.path.basename(out)

# part name (STEP product) -> look; everything else is 3D-printed plastic
PART_LOOK = {"prunner_blade": "gold", "prunner_rcv": "gold", "Spur Gear1": "steel", "blade_shaft": "steel",
             "XM,H-430_idler": "dynamixel", "cutter_fixture_cutterside": "fixture",
             "D405_bracket_abs": "abs_black"}
# look: (colour A, colour B (linear), metallic, roughness, roughness spread, grain, bump strength)
# grain: brushed = metal streaks, layers = FDM layer lines along the part's own Z, else isotropic noise
LOOKS = {
    "gold": ((0.83, 0.60, 0.25), (0.68, 0.46, 0.16), 1.0, 0.24, 0.14, "brushed", 0.08),
    "steel": ((0.58, 0.58, 0.60), (0.46, 0.46, 0.48), 1.0, 0.30, 0.18, "brushed", 0.08),
    "plastic": ((0.050, 0.050, 0.053), (0.038, 0.038, 0.041), 0.0, 0.58, 0.16, "layers", 0.35),
    "abs_black": ((0.020, 0.020, 0.021), (0.014, 0.014, 0.015), 0.0, 0.50, 0.16, "layers", 0.35),
    "fixture": ((0.20, 0.20, 0.21), (0.15, 0.15, 0.16), 0.0, 0.62, 0.10, "noise", 0.25),
    "dynamixel": ((0.030, 0.030, 0.033), (0.022, 0.022, 0.025), 1.0, 0.38, 0.10, "noise", 0.06),  # black anodized Al
    "pcb": ((0.012, 0.13, 0.035), (0.020, 0.17, 0.05), 0.0, 0.35, 0.08, "noise", 0.15),
    "pcb_part": ((0.04, 0.04, 0.045), (0.06, 0.06, 0.065), 0.0, 0.40, 0.08, "noise", 0.15),
    "connector": ((0.02, 0.05, 0.45), (0.015, 0.04, 0.38), 0.0, 0.40, 0.08, "noise", 0.15),
}


def part_of(o):
    while o and o.name.startswith("=>"):  # unnamed sub-shapes sit under their part's empty
        o = o.parent
    return o.name.rsplit(":", 1)[0] if o else ""


def look_of(o, m):
    if part_of(o) == "hall_sensor_board":  # CAD colours: green board, blue connector, gray parts
        r, g, b = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED").inputs["Base Color"].default_value[:3]
        return "pcb" if g > r + 0.2 else "connector" if b > r + 0.2 else "pcb_part"
    return PART_LOOK.get(part_of(o), "plastic")


def procedural(look):
    """Principled BSDF fed by object-space noise; nodes COL / ROUGH are what gets baked."""
    a, b, metal, rough, spread, grain, bump = LOOKS[look]
    m = bpy.data.materials.new(look)
    nt = m.node_tree
    N, L = nt.nodes, nt.links
    bsdf = next(n for n in N if n.type == "BSDF_PRINCIPLED")
    tc = N.new("ShaderNodeTexCoord")
    mp = N.new("ShaderNodeMapping")
    L.new(tc.outputs["Object"], mp.inputs["Vector"])
    # brushed metal: noise varying 100x faster across x than along y/z = fine streaks; plastic: isotropic grain
    mp.inputs["Scale"].default_value = (4000, 40, 40) if grain == "brushed" else (2500, 2500, 2500)
    fine = N.new("ShaderNodeTexNoise")
    fine.inputs["Scale"].default_value = 1.0
    fine.inputs["Detail"].default_value = 8
    L.new(mp.outputs["Vector"], fine.inputs["Vector"])
    if grain == "layers":  # 0.3 mm layers (wave period 2*pi/20 per unit / scale), slightly wobbly, plus the grain
        wave = N.new("ShaderNodeTexWave")
        wave.bands_direction = "Z"
        wave.inputs["Scale"].default_value = 2 * math.pi / 20 / 0.0003
        wave.inputs["Distortion"].default_value = 0.6
        wave.inputs["Detail Scale"].default_value = 0.002
        L.new(tc.outputs["Object"], wave.inputs["Vector"])
        mix = N.new("ShaderNodeMix")
        mix.inputs["Factor"].default_value = 0.3
        L.new(wave.outputs["Fac"], mix.inputs["A"])
        L.new(fine.outputs["Fac"], mix.inputs["B"])
        fine = mix
    blotch = N.new("ShaderNodeTexNoise")  # cm-scale tone variation
    blotch.inputs["Scale"].default_value = 60
    blotch.inputs["Detail"].default_value = 4
    L.new(tc.outputs["Object"], blotch.inputs["Vector"])
    col = N.new("ShaderNodeMix")
    col.name = "COL"
    col.data_type = "RGBA"
    L.new(blotch.outputs["Fac"], col.inputs["Factor"])
    col.inputs["A"].default_value = (*a, 1)
    col.inputs["B"].default_value = (*b, 1)
    ro = N.new("ShaderNodeMapRange")
    ro.name = "ROUGH"
    ro.inputs["To Min"].default_value = rough - spread / 2
    ro.inputs["To Max"].default_value = rough + spread / 2
    L.new(fine.outputs[0], ro.inputs["Value"])
    bp = N.new("ShaderNodeBump")
    bp.inputs["Strength"].default_value = bump
    bp.inputs["Distance"].default_value = 0.0002
    L.new(fine.outputs[0], bp.inputs["Height"])
    L.new(col.outputs["Result"], bsdf.inputs["Base Color"])
    L.new(ro.outputs["Result"], bsdf.inputs["Roughness"])
    L.new(bp.outputs["Normal"], bsdf.inputs["Normal"])
    bsdf.inputs["Metallic"].default_value = metal
    return m


def bake(o, size, name):
    """Bake every material slot of o into three shared images; returns them."""
    imgs = {k: bpy.data.images.new(f"{name}_{k}", size, size) for k in ("color", "rough", "normal")}
    imgs["rough"].colorspace_settings.name = "Non-Color"
    imgs["normal"].colorspace_settings.name = "Non-Color"
    sc = bpy.context.scene
    for k, img in imgs.items():
        for m in o.data.materials:
            nt = m.node_tree
            out_n = next(n for n in nt.nodes if n.type == "OUTPUT_MATERIAL")
            bsdf = next(n for n in nt.nodes if n.type == "BSDF_PRINCIPLED")
            node = nt.nodes.new("ShaderNodeTexImage")
            node.image = img
            nt.nodes.active = node
            if k == "normal":
                nt.links.new(bsdf.outputs[0], out_n.inputs["Surface"])
            else:
                em = nt.nodes.get("EMIT") or nt.nodes.new("ShaderNodeEmission")
                em.name = "EMIT"
                nt.links.new(nt.nodes["COL" if k == "color" else "ROUGH"].outputs["Result"], em.inputs["Color"])
                nt.links.new(em.outputs[0], out_n.inputs["Surface"])
        sc.cycles.samples = 8  # >1 also antialiases the layer lines
        bpy.ops.object.bake(type="NORMAL" if k == "normal" else "EMIT", margin=4)
        img.filepath_raw = os.path.join(tex_dir, f"{img.name}.png")
        img.file_format = "PNG"
        img.save()
    return imgs


def baked_material(name, imgs, metal):
    m = bpy.data.materials.new(name)
    N, L = m.node_tree.nodes, m.node_tree.links
    bsdf = next(n for n in N if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Metallic"].default_value = metal
    t = {}
    for k, img in imgs.items():
        t[k] = N.new("ShaderNodeTexImage")
        t[k].image = img
    L.new(t["color"].outputs["Color"], bsdf.inputs["Base Color"])
    L.new(t["rough"].outputs["Color"], bsdf.inputs["Roughness"])
    nm = N.new("ShaderNodeNormalMap")
    L.new(t["normal"].outputs["Color"], nm.inputs["Color"])
    L.new(nm.outputs["Normal"], bsdf.inputs["Normal"])
    return m


# M3 socket head screws: bearing point of the head (CAD mm), axis from the head into the parts, length (mm)
SCREWS = [
    *[((x, y, 28.0), "+z", 10) for x, y in ((-19.5, 7), (-19.5, 26), (19.6, 7), (19.6, 26))],  # fixture bores -> base
    *[((x, y, -28.0), "-z", 12) for x, y in ((19.5, 7), (19.5, 26), (-19.6, 7), (-19.6, 26))],  # -> camera bracket
    ((-37.3, 6, 4.0), "-z", 6), ((-37.3, 22, 4.0), "-z", 6),  # the fixture halves' flanges
    ((37.3, 6, -4.0), "+z", 6), ((37.3, 22, -4.0), "+z", 6),
    *[((x, 38.5, z), "-y", 6) for x, z in ((-9.2, -22.2), (-9.1, 22.2), (9.1, -22.2), (9.2, 22.1))],  # housing spacers
    *[((x, y, 36.0), "+z", 6) for x, y in ((-24.8, 78), (-5.7, 89), (-4.8, 43.4), (14.3, 54.4))],  # Dynamixel nuts
    *[((x, y, 31.5), "+z", 10) for x, y in ((-44.2, 58), (33.6, 111.5), (45.8, 53.1))],  # cover top -> bottom
    ((-24.2, 46, 51.5), "-z", 16), ((25.8, 71, 51.5), "-z", 16),  # cover bottom -> cutter base
    ((0.0, -5.2, -39.5), "-z", 12),  # camera bracket -> D405
]
# linkage pins along z (CAD mm): name -> (x, y, z0, z1, diameter); the joints' holes are 6.2-6.4 mm, W0's 4.3 mm
PINS = {"pin_A": (-44.45, 147.8, -12.5, 12.2, 6), "pin_C": (-44.43, 107.8, -12.5, 12.2, 6),
        "pin_E0": (-58.87, 97.92, -10.5, 10.2, 6), "pin_D0": (-77.47, 85.22, -7.5, 7.2, 6),
        "pin_B0": (-61.33, 141.57, -7.5, 7.1, 6), "pin_W0": (-51.03, 55.55, -10.5, 10.1, 4)}
FASTENER_LOOK = {"screw": ((0.55, 0.55, 0.57), 0.35), "pin": ((0.62, 0.62, 0.64), 0.22)}  # stainless, ground steel


def tube(bm, rings, n=24):
    """Surface of revolution about +z from [(radius, z, shape)] rings (shape "o" circle, "h" hexagon, r = flat),
    both ends capped."""
    loops = []
    for r, z, shape in rings:
        pts = []
        for i in range(n):
            a = 2 * math.pi * i / n
            k = r / math.cos((a + math.pi / 6) % (math.pi / 3) - math.pi / 6) if shape == "h" else r  # r = flat
            pts.append(bm.verts.new((k * math.cos(a), k * math.sin(a), z)))
        loops.append(pts)
    for j in range(len(loops) - 1):
        (r0, z0, h0), (r1, z1, h1) = rings[j], rings[j + 1]
        for i in range(n):
            f = bm.faces.new((loops[j][i], loops[j][(i + 1) % n], loops[j + 1][(i + 1) % n], loops[j + 1][i]))
            f.smooth = h0 == h1 == "o" and z0 != z1  # round sides smooth, flat rings and hex walls flat
    bm.faces.new(loops[0][::-1])
    bm.faces.new(loops[-1])
    for e in bm.edges:  # the shading break between flat and smooth faces
        e.smooth = all(f.smooth for f in e.link_faces)


def fasteners():
    """{object name: mesh}: all screws in one mesh, each pin its own (they ride on the linkage bodies)."""
    import bmesh
    from mathutils import Matrix, Vector

    def mesh(name, build, look):
        bm = bmesh.new()
        build(bm)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        me = bpy.data.meshes.new(name)
        bm.to_mesh(me)
        m = bpy.data.materials.new(look)
        bsdf = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
        bsdf.inputs["Base Color"].default_value = (*FASTENER_LOOK[look][0], 1)
        bsdf.inputs["Metallic"].default_value = 1.0
        bsdf.inputs["Roughness"].default_value = FASTENER_LOOK[look][1]
        me.materials.append(m)
        return me

    def screws(bm):
        axes = {"+z": Vector((0, 0, 1)), "-z": Vector((0, 0, -1)), "-y": Vector((0, -1, 0))}
        for (x, y, z), ax, length in SCREWS:
            b = bmesh.new()  # ISO 4762 M3: head 5.5 x 3, 2.5 mm hex socket 1.6 deep; head at z < 0, shank along +z
            tube(b, [(1.5, length, "o"), (1.5, 0.0, "o"), (2.75, 0.0, "o"), (2.75, -2.7, "o"), (2.45, -3.0, "o"),
                     (1.25, -3.0, "h"), (1.25, -1.4, "h")])
            rot = Vector((0, 0, 1)).rotation_difference(axes[ax]).to_matrix().to_4x4()
            b.transform(Matrix.Translation(Vector((x, y, z)) * 0.001) @ rot @ Matrix.Scale(0.001, 4))
            tmp = bpy.data.meshes.new("tmp")
            b.to_mesh(tmp)
            bm.from_mesh(tmp)
            bpy.data.meshes.remove(tmp)
            b.free()

    def pin(x, y, z0, z1, d):
        def build(bm):  # 0.3 mm end chamfers
            r = d / 2
            tube(bm, [(r - 0.3, z0, "o"), (r, z0 + 0.3, "o"), (r, z1 - 0.3, "o"), (r - 0.3, z1, "o")])
            bm.transform(Matrix.Translation(Vector((x, y, 0)) * 0.001) @ Matrix.Scale(0.001, 4))
        return build

    out = {"m3_screws:1": mesh("m3_screws", screws, "screw")}
    for name, p in PINS.items():
        out[name + ":1"] = mesh(name, pin(*p), "pin")
    return out


bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=os.path.abspath(src))
sc = bpy.context.scene
sc.render.engine = "CYCLES"
try:
    bpy.context.preferences.addons["cycles"].preferences.compute_device_type = "OPTIX"
    bpy.context.preferences.addons["cycles"].preferences.get_devices()
    for d in bpy.context.preferences.addons["cycles"].preferences.devices:
        d.use = True
    sc.cycles.device = "GPU"
except TypeError:
    pass
sc.render.bake.use_selected_to_active = False
if bracket:  # the camera moved (2026-10-30): the CAD's bracket goes, the new one sits in the same CAD frame
    design = bpy.data.objects["cutter_fixture_design"]
    bpy.data.objects.remove(bpy.data.objects["D405_bracket:1"])
    bpy.ops.import_scene.gltf(filepath=os.path.abspath(bracket[0]))
    new = bpy.data.objects["D405_bracket_abs"]
    new.name = "D405_bracket_abs:1"
    new.parent = design
    new.matrix_parent_inverse = design.matrix_world.inverted()

meshes = [o for o in sc.objects if o.type == "MESH"]
for o in meshes:
    if not o.data.polygons:  # STEP sub-shapes without faces (curves, points)
        bpy.data.objects.remove(o)

done = {}
for o in sorted((o for o in sc.objects if o.type == "MESH"), key=lambda o: o.name):
    if o.data.name in done:  # instanced part (two fixtures, two spacers) shares the mesh and its materials
        continue
    looks = [look_of(o, m) for m in o.data.materials] or [PART_LOOK.get(part_of(o), "plastic")]
    o.data.materials.clear()
    for lk in looks:
        o.data.materials.append(procedural(lk))
    bpy.ops.object.select_all(action="DESELECT")
    o.select_set(True)
    bpy.context.view_layer.objects.active = o
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.uv.smart_project(angle_limit=math.radians(66), island_margin=0.004)
    bpy.ops.object.mode_set(mode="OBJECT")
    # ~10 texels per mm over the part's surface (UV packing fills ~half the square), 16 for the layer lines
    area_mm2 = sum(p.area for p in o.data.polygons) * max(o.matrix_world.to_scale()) ** 2 * 1e6
    density = 16 if any(LOOKS[lk][5] == "layers" for lk in looks) else 10
    size = min(2048, max(256, 2 ** math.ceil(math.log2(math.sqrt(area_mm2 * 2) * density))))
    name = f"{prefix}_{part_of(o)}".replace(",", "").replace(" ", "_").replace("+", "_")
    while any(n == name for n in done.values()):
        name += "_b"
    imgs = bake(o, size, name)
    metal = max(LOOKS[lk][2] for lk in looks)  # every multi-material part (the PCB) is non-metal
    olds = list(o.data.materials)
    o.data.materials.clear()
    o.data.materials.append(baked_material(name, imgs, metal))
    for m in olds:
        bpy.data.materials.remove(m)
    done[o.data.name] = name
    print(f"baked {name} {size}px looks={looks}", flush=True)

if bracket:  # the gripper assembly's fasteners (CAD has holes only); plain steel, not baked
    design = bpy.data.objects["cutter_fixture_design"]
    for name, mesh in fasteners().items():
        o = bpy.data.objects.new(name, mesh)
        sc.collection.objects.link(o)
        o.parent = design
        o.matrix_parent_inverse = design.matrix_world.inverted()
        print("added", name, len(mesh.polygons), "faces", flush=True)

bpy.ops.wm.save_as_mainfile(filepath=out + ".blend", relative_remap=True)
bpy.ops.wm.usd_export(filepath=out + ".usd", export_materials=True, generate_preview_surface=True,
                      export_textures_mode="KEEP", relative_paths=True, root_prim_path="/" + prefix,
                      export_lights=False, export_cameras=False)
print("wrote", out + ".usd", flush=True)

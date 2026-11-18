"""Write a scene USD of the real lavender field: one plant per object_data row at its map x/y.

Run by scripts/make_farm_scene.py inside the Isaac Sim image with only Isaac's USD libraries on the path (no Kit),
with ./sim mounted at /sim:  build_farm_scene.py <plants.json> <out.usd>
plants.json: {"source": ..., "plants": [{"object_id", "row_id", "x", "y"}, ...]} in the map frame, which is the
sim's world frame (the sim's GPS datum is its world origin, X/Y = East/North; sim_swift_nav_dual.launch.py).

The file is meant for SIM_SCENE=<file> (build_file_world in setup_scene.py: used as a sublayer, z=0 ground, the
fleet's physics settings authored over its physics scene), and also opens on its own in Isaac Sim. It only refers
to ../assets/... (relative to sim/scene/), nothing in sim/generated/: the colliders the lidars need are authored
in the file itself, on one `class` prototype per asset that the placed copies reference (the same meshes
setup_scene.collider_asset() would give colliders). The look follows the built-in `lavender` scene: soil-coloured
ground box, the ground-cover grass patch, HDR sky + sun, and a tree/shrub/rock border, here all around the field.
"""

import json
import math
import os
import random
import sys

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

ASSETS = "/sim/assets"
REL_ASSETS = "../assets"  # as seen from sim/scene/<file>

# Plant models (lavender_v2/: the Sim_LavenderFarm pack's Nanite lavenders, in cm): every plant starts as one of the
# harvest-ready models, in exactly these shares (which plants seeded by object_id); a harvested plant shows
# LAVENDER_HARVESTED instead (add_plants, setup_scene.set_harvested).
# Was lavender/SM_Lavender_Nanite_01.usd for all plants (until 2026-10-21; still the built-in lavender scene's).
# Flowering 01-04, flowerless 05/06 (the cut-back look of a harvested plant).
LAVENDER_READY = (("lavender_01", "lavender_v2/Props/SM_Lavender_Nanite_01.usd", 0.7),
                  ("lavender_02", "lavender_v2/Props/SM_Lavender_Nanite_02.usd", 0.3))
LAVENDER_HARVESTED = ("lavender_harvested", "lavender_v2/Props/SM_Lavender_Nanite_05.usd")
LAVENDER_HARVESTED_SCALE = 0.5  # of the plant's width: at full width the cut plant looked too big (2026-10-21)
# Plant size. Each model is scaled uniformly (its own proportions; a taller-than-wide scale looked squeezed) to this
# width: 01 (2.0 m wide, 1.34 m tall) ends up ~0.66 m tall, 02 ~0.75 m, the harvested 05 (at LAVENDER_HARVESTED_SCALE) ~0.27 m. Rows are ~1.85 m apart and
# plants ~0.44 m apart in a row, so the clumps overlap into a hedge and leave a ~1.03 m lane (the map,
# status_server, treats a row as 0.45 m thick; 0.5 m wide plants were only ~0.33 m tall; 0.75 m was made 10%
# bigger at the user's request, then 0.825 m 20% bigger on 2026-10-09: ~0.66 m tall, the lane ~0.86 m clear).
PLANT_DIAMETER = 0.99
# Each plant's width is a fixed random fraction of PLANT_DIAMETER (uniform, seeded by its object_id like its yaw, so
# every rebuild looks the same); identical clumps looked artificial to the user (2026-10-09).
PLANT_SIZE_RANGE = (0.9, 1.1)
# Margin around the plant centres for the field box the border (trees, shrubs, rocks) keeps its distance from. Kept
# at the 0.825 m plants' half width when they grew to 0.99 m, so the random border layout stayed the same.
FIELD_BOX_MARGIN = 0.4125
PLANT_SINK = 0.02  # lowest point below z=0, so no plant floats
# Stem/leaf look, set on each prototype's shaders by name prefix: (inputs:SubsurfaceOpacity, inputs:SubsurfaceColor).
# The pack's MDLs hardcoded opacity 1.0 and a black subsurface colour; lavender_v2/Materials exposes both like
# lavender/Materials does (MI_Leaf_high_01 has neither). Opacity only splits the subsurface half of the diffuse
# response between reflection and transmission (OmniUe4Subsurface), it isn't see-through: the colour is what shows.
# A quarter of lavender/Materials' defaults (stem 0.55, 0.80, 0.35; leaf 0.45, 0.60, 0.42), which looked washed out.
LAVENDER_SUBSURFACE = {"MI_Stem_": (1.0, (0.14, 0.2, 0.09)), "MI_Leaf_0": (1.0, (0.11, 0.15, 0.1))}

# Weed barrier: the black woven landscape fabric laid under each real row (farm photo: a dark strip just past the
# foliage, no grass on it). One flat strip per row along the row's fitted line, a common 3 ft (0.9 m) roll (~4 cm
# past the 0.825 m plants each side; 1.1 m looked too wide to the user), running BARRIER_END_M past the end plants'
# centres. Visual only (no collider: driving and the lidars are unchanged); the grass blades rooted on it are hidden
# (hide_grass_under). Textured like a photo of the fabric, sparsely dusted with sand/dirt (more towards its edges):
# sim/assets/weed_barrier/, written by make_farm_textures.py; one tile spans the strip's width (BARRIER_TILE_M =
# BARRIER_WIDTH) with green guide lines on its centre line and 0.3 m either side; each row starts the tile at its own
# random offset along the row, so neighbouring rows don't show the same dirt.
BARRIER_WIDTH = 0.9
BARRIER_END_M = 0.5
BARRIER_Z = 0.0015  # above the ground (z=0); 4 mm showed as a thick edge
# Mounds: the asset's stems converge ~3.5 cm above its lowest (drooping) leaves, so on flat ground the crown floated
# over the fabric. Each plant gets a soil mound, MOUND_HEIGHT at its centre falling to 0 at MOUND_RADIUS (cos^2),
# that buries the stem bases; neighbours (0.44 m apart) merge into a wavy ridge (the higher of the two, ~2 cm
# between plants). The fabric is laid over them: each strip is a grid (MOUND_GRID_M cells) following the mounds,
# flat on the ground at its edges (MOUND_RADIUS < BARRIER_WIDTH / 2). Visual only, like the flat strip.
MOUND_HEIGHT = 0.06
MOUND_RADIUS = 0.35
MOUND_GRID_M = 0.03
BARRIER_TEX = "weed_barrier"  # folder in sim/assets: albedo.jpg, normal.png, roughness.jpg
BARRIER_TILE_M = 0.9  # make_farm_textures.BARRIER_TILE_M
BARRIER_COLOR = (0.03, 0.03, 0.032)  # displayColor only (the textures give the look)
# RTX renders the OmniPBR (MDL) version; UsdPreviewSurface stays as the fallback for other renderers. Seen from a
# Jackal's camera (~25 cm up, grazing) a UsdPreviewSurface strip mirrored the sky white even at specular 0.01 /
# roughness 0.95 (its Fresnel always reaches 1 at grazing); OmniPBR's specular_level weights the whole reflection.
BARRIER_SPECULAR_LEVEL = 0.1  # OmniPBR, default 0.5

GROUND_COVER = "Ground_cover/ground_cover.usd"  # 100 x 100 m of grass blades, geometry really in metres
GROUND_COVER_SCALE_Z = 0.6
GROUND_SIZE = 100.0
GROUND_SOIL_COLOR = (0.07, 0.05, 0.035)  # displayColor only: the soil textures give the look
# The visible ground: a textured quad at z=0 (dark grainy sandy soil, sim/assets/soil/ from make_farm_textures.py,
# one tile per SOIL_TILE_M); the ground box under it is guide purpose (not rendered, still the collider and the
# physics material), since a Cube has no UVs. The flat brown box looked artificial.
SOIL_TEX = "soil"
SOIL_TILE_M = 3.0  # make_farm_textures.SOIL_TILE_M
SOIL_SPECULAR_LEVEL = 0.15
SKY_HDR = "sky/farm_field_puresky_2k.hdr"
SKY_INTENSITY = 400
SUN_INTENSITY = 5000
SUN_ROTATE_XYZ = (60, 2.0, -50)  # degrees, the distant light's rotateXYZ (user's choice; was (-60, 33, -30) at 10000)

# Border vegetation (setup_scene.py's add_horizon_vegetation, here a band around the field instead of an arc ahead
# of the robots): (assets, count, min distance from the field's plants m, margin to the ground's edge m, height m,
# footprint radius m, seed). Each group is scattered at random (no band, user's request) over the ground outside
# that distance, corners included (scatter_points), no two items closer than the sum of their footprint radii (so
# trunks, shrubs and rocks don't overlap). The clearing around the field stays open (the fleet's default spawn poses
# at x=0, y up to +1.6, and the charger ~9 m west of the field). The robot cameras see the trees from the whole
# field (setup_scene.CAM_FAR_CLIP); enough of them stand between the field and the edge to cover the horizon.
# Trunks stay >= 4 m inside the edge; the widest oak crowns (1.3x their height) may reach past it, which hides the
# edge. History: a band around the field (trees 7-11 m tall every 3 m 13-17 m out; later 10-16 m every 2 m 28-37 m
# out, shrubs 24-30 m, rocks 21-25 m).
BORDER = {
    "trees": ([f"trees/{n}.usd" for n in ("Douglas_Fir", "Black_Oak", "Douglas_Fir")], 200, 23.0, 4.0, (10.0, 16.0), 1.5, 7),
    "shrubs": ([f"shrubs/{n}.usd" for n in ("Holly", "Lilac", "Privet", "Yew")], 220, 20.0, 2.0, (2.0, 4.0), 0.8, 11),
    "rocks": ([f"rocks/rock_small_{i:02d}.usda" for i in range(1, 7)], 80, 18.0, 1.0, (0.6, 1.4), 0.6, 13),
}
EDGE_JITTER_M = 3.0

# Wooden entrance arch (gen_3d_model/scripts/wood_arch, from a photo of the farm's arch): 2.4 m tall, posts 2.28 m
# apart, beam along the asset's X, origin at its floor centre, posts and beam with convex-hull colliders. It stands
# across the west end of lane ARCH_LANE (0 = the lane between the two northernmost rows), ARCH_SETBACK_M before the
# end of the shorter row, so a robot entering that lane drives through it (2.14 m clear between the posts).
ARCH = "wood_arch/wood_arch.usd"
ARCH_LANE = 4
ARCH_SETBACK_M = 3.0

_bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])


def asset_bounds(asset):
    st = Usd.Stage.Open(f"{ASSETS}/{asset}")
    return _bbox.ComputeWorldBound(st.GetDefaultPrim() or st.GetPseudoRoot()).ComputeAlignedRange()


def add_prototype(stage, name, asset, colliders=True):
    """class /World/Prototypes/<name> referencing the asset, with a static exact triangle-mesh collider on each of
    its meshes (setup_scene.collider_asset, in this file) unless `colliders` is false. PointInstancer prototypes
    (leaves) can't be colliders."""
    path = f"/World/Prototypes/{name}"
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(f"{REL_ASSETS}/{asset}")
    # opinions below an instanceable prim are ignored (the rocks have one)
    while True:
        inst = [q for q in Usd.PrimRange(prim, Usd.PrimAllPrimsPredicate) if q.IsInstanceable()]
        if not inst:
            break
        for q in inst:
            q.SetInstanceable(False)
    it = iter(Usd.PrimRange(prim, Usd.PrimAllPrimsPredicate) if colliders else ())
    for q in it:
        if q.IsA(UsdGeom.PointInstancer):
            it.PruneChildren()
        elif q.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(q)
            UsdPhysics.MeshCollisionAPI.Apply(q).CreateApproximationAttr(UsdPhysics.Tokens.none)
    return path


def textured_material(stage, path, folder, specular_level):
    """Material at `path` with sim/assets/<folder>/{albedo.jpg, normal.png, roughness.jpg} on the mesh's `st`:
    OmniPBR (MDL, what RTX renders) plus a UsdPreviewSurface fallback for other renderers."""
    files = {"albedo": "albedo.jpg", "normal": "normal.png", "roughness": "roughness.jpg"}
    surface = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
    shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.03))
    surface.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    st = UsdShade.Shader.Define(stage, f"{path}/st")
    st.CreateIdAttr("UsdPrimvarReader_float2")
    st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st_out = st.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    def texture(name, color_space, output, out_type, **inputs):
        tex = UsdShade.Shader.Define(stage, f"{path}/{name}")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"{REL_ASSETS}/{folder}/{files[name]}")
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(color_space)
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_out)
        for wrap in ("wrapS", "wrapT"):
            tex.CreateInput(wrap, Sdf.ValueTypeNames.Token).Set("repeat")
        for k, v in inputs.items():
            tex.CreateInput(k, Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*v))
        return tex.CreateOutput(output, out_type)

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture("albedo", "sRGB", "rgb", Sdf.ValueTypeNames.Float3))
    shader.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(
        texture("normal", "raw", "rgb", Sdf.ValueTypeNames.Float3, scale=(2, 2, 2, 1), bias=(-1, -1, -1, 0)))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).ConnectToSource(
        texture("roughness", "raw", "r", Sdf.ValueTypeNames.Float))

    mdl = UsdShade.Shader.Define(stage, f"{path}/omnipbr")
    mdl.SetSourceAsset("OmniPBR.mdl", "mdl")
    mdl.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    mdl.CreateIdAttr("")
    mdl.GetPrim().GetAttribute("info:implementationSource").Set("sourceAsset")
    for name, key, space in (("albedo", "diffuse_texture", "sRGB"), ("roughness", "reflectionroughness_texture", "raw"),
                             ("normal", "normalmap_texture", "raw")):
        mdl.CreateInput(key, Sdf.ValueTypeNames.Asset).Set(f"{REL_ASSETS}/{folder}/{files[name]}")
        mdl.GetInput(key).GetAttr().SetColorSpace(space)
    mdl.CreateInput("reflection_roughness_texture_influence", Sdf.ValueTypeNames.Float).Set(1.0)
    mdl.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(specular_level)
    mdl.CreateInput("uv_space_index", Sdf.ValueTypeNames.Int).Set(0)  # the mesh's st
    surface.CreateSurfaceOutput("mdl").ConnectToSource(mdl.ConnectableAPI(), "out")
    return surface


def add_ground(stage, center):
    """GROUND_SIZE box centred on the field, top face at z=0, grippy physics material, not rendered (guide); the
    visible ground is a soil-textured quad on its top face."""
    mat = UsdShade.Material.Define(stage, "/World/Materials/ground_physics")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(1.0)
    pm.CreateDynamicFrictionAttr(0.9)
    pm.CreateRestitutionAttr(0.0)
    cube = UsdGeom.Cube.Define(stage, "/World/ground")
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], -0.5))
    xf.AddScaleOp().Set(Gf.Vec3d(GROUND_SIZE, GROUND_SIZE, 1.0))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*GROUND_SOIL_COLOR)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    UsdGeom.Imageable(cube).CreatePurposeAttr(UsdGeom.Tokens.guide)  # still collides (cf. setup_scene's lavender cores)
    surf = UsdGeom.Mesh.Define(stage, "/World/ground_surface")
    h = GROUND_SIZE / 2
    corners = [(-h, -h), (h, -h), (h, h), (-h, h)]
    surf.CreatePointsAttr([Gf.Vec3f(center[0] + x, center[1] + y, 0.0) for x, y in corners])
    surf.CreateFaceVertexCountsAttr([4])
    surf.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    surf.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    surf.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    surf.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    surf.CreateExtentAttr([Gf.Vec3f(center[0] - h, center[1] - h, 0), Gf.Vec3f(center[0] + h, center[1] + h, 0)])
    UsdGeom.PrimvarsAPI(surf).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
        [Gf.Vec2f((center[0] + x) / SOIL_TILE_M, (center[1] + y) / SOIL_TILE_M) for x, y in corners])
    surf.CreateDisplayColorAttr([Gf.Vec3f(*GROUND_SOIL_COLOR)])
    UsdShade.MaterialBindingAPI.Apply(surf.GetPrim()).Bind(
        textured_material(stage, "/World/Materials/ground_soil", SOIL_TEX, SOIL_SPECULAR_LEVEL))

    # the grass patch (visual only, traction comes from the box), centred on the field like the box
    gc = stage.DefinePrim("/World/GroundCover", "Xform")
    gc.GetReferences().AddReference(f"{REL_ASSETS}/{GROUND_COVER}")
    gxf = UsdGeom.Xformable(gc)
    gxf.AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], 0.0))
    gxf.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, GROUND_COVER_SCALE_Z))


def add_lights(stage):
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/dome")
    dome.CreateIntensityAttr(SKY_INTENSITY)
    dome.CreateTextureFileAttr(f"{REL_ASSETS}/{SKY_HDR}")
    dome.CreateTextureFormatAttr("latlong")
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/sun")
    sun.CreateIntensityAttr(SUN_INTENSITY)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(*SUN_ROTATE_XYZ))


def add_plant_prototype(stage, name, asset, colliders=True, scale=1.0):
    """add_prototype under /World/Prototypes/<name>/model, scaled to `scale` m wide with its bbox centred on the
    origin and its lowest point at z=0, so every model takes the same per-plant transform (the harvested model then
    stands on the ready one's spot, `scale` times its width). Applies LAVENDER_SUBSURFACE to its shaders."""
    path = f"/World/Prototypes/{name}"
    UsdGeom.Xform.Define(stage, path)
    model = add_prototype(stage, f"{name}/model", asset, colliders)
    r = asset_bounds(asset)
    size, mid = r.GetSize(), r.GetMidpoint()
    k = scale / max(size[0], size[1])
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(model))
    xf.AddScaleOp().Set(Gf.Vec3d(k, k, k))
    xf.AddTranslateOp(opSuffix="pivot").Set(Gf.Vec3d(-mid[0], -mid[1], -r.GetMin()[2]))
    n = 0
    for q in Usd.PrimRange(stage.GetPrimAtPath(path), Usd.PrimAllPrimsPredicate):  # the prototype is abstract
        look = next((v for pre, v in LAVENDER_SUBSURFACE.items() if q.GetName().startswith(pre)), None)
        if look and q.IsA(UsdShade.Shader):
            sh = UsdShade.Shader(q)
            sh.CreateInput("SubsurfaceOpacity", Sdf.ValueTypeNames.Float).Set(look[0])
            sh.CreateInput("SubsurfaceColor", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(*look[1]))
            n += 1
    print(f"{name}: {asset}, {size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} (asset units), subsurface on {n} shader(s)")
    return path


def add_plants(stage, plants):
    """One Xform per plant, /World/lavender/row_<row_id>/plant_<object_id> (customData object_id, diameter_m), scaled
    to its own width (PLANT_DIAMETER x PLANT_SIZE_RANGE) at its x/y, with two instanceable children: `ready`, a
    LAVENDER_READY model, and `harvested`, LAVENDER_HARVESTED, invisible until setup_scene.set_harvested swaps their
    visibility (swapping the instance's prototype at runtime turned every RTX frame black, 2026-10-21).
    shortcut: the harvested model has no colliders (PhysX ignores visibility, the lidars would see it from the
    start), so a harvested plant keeps the ready model's colliders; switch them with it if a lidar must see the cut
    plant. Returns the rows' extents [x_min, x_max, y_mean, width] for the web UI's spawn map."""
    ready = [(add_plant_prototype(stage, name, asset), w) for name, asset, w in LAVENDER_READY]
    harvested = add_plant_prototype(stage, *LAVENDER_HARVESTED, colliders=False, scale=LAVENDER_HARVESTED_SCALE)
    # exactly the weights' shares (a per-plant random pick gave 65/35 for 70/30), which plants seeded like the rest
    shuffled = sorted(plants, key=lambda p: random.Random(f"model {p['object_id']}").random())
    total, model, i = sum(w for _, w in ready), {}, 0
    for k, (proto, w) in enumerate(ready):
        n = len(plants) - i if k == len(ready) - 1 else round(w / total * len(plants))
        model.update((p["object_id"], proto) for p in shuffled[i:i + n])
        i += n
    UsdGeom.Xform.Define(stage, "/World/lavender")
    rows, counts = {}, {proto: 0 for proto, _ in ready}
    for p in sorted(plants, key=lambda p: (p["row_id"], p["object_id"])):
        row = f"/World/lavender/row_{p['row_id']}"
        if p["row_id"] not in rows:
            UsdGeom.Xform.Define(stage, row)
            rows[p["row_id"]] = []
        rng = random.Random(p["object_id"])  # same look on every rebuild
        yaw = rng.uniform(0.0, 360.0)
        d = PLANT_DIAMETER * rng.uniform(*PLANT_SIZE_RANGE)
        proto = model[p["object_id"]]
        counts[proto] += 1
        rows[p["row_id"]].append((p, d))
        prim = stage.DefinePrim(f"{row}/plant_{p['object_id']}", "Xform")
        for child, src in (("ready", proto), ("harvested", harvested)):
            c = stage.DefinePrim(prim.GetPath().AppendChild(child), "Xform")
            c.GetReferences().AddInternalReference(src)
            c.SetInstanceable(True)
        UsdGeom.Imageable(prim.GetChild("harvested")).MakeInvisible()
        prim.SetCustomDataByKey("object_id", int(p["object_id"]))
        prim.SetCustomDataByKey("diameter_m", round(d, 4))
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(p["x"], p["y"], -PLANT_SINK))
        xf.AddRotateZOp().Set(yaw)
        xf.AddScaleOp().Set(Gf.Vec3d(d, d, d))
    print("lavender: " + ", ".join(f"{n} x {q.split('/')[-1]}" for q, n in counts.items()))
    return [[min(q["x"] - d / 2 for q, d in ps), max(q["x"] + d / 2 for q, d in ps),
             sum(q["y"] for q, _ in ps) / len(ps), max(d for _, d in ps)]
            for _, ps in sorted(rows.items())]


def row_strips(plants):
    """Per row: (row_id, start xy, end xy, unit direction xy, plant xys) of its weed barrier, on the least-squares
    line through the row's plants (the DB rows are straight to < 1 mm), BARRIER_END_M past the outermost plants."""
    rows = {}
    for p in plants:
        rows.setdefault(p["row_id"], []).append((p["x"], p["y"]))
    strips = []
    for row_id, pts in sorted(rows.items()):
        n = len(pts)
        mx, my = sum(x for x, _ in pts) / n, sum(y for _, y in pts) / n
        sxx = sum((x - mx) ** 2 for x, _ in pts)
        syy = sum((y - my) ** 2 for _, y in pts)
        sxy = sum((x - mx) * (y - my) for x, y in pts)
        a = 0.5 * math.atan2(2 * sxy, sxx - syy) if n > 1 else 0.0  # principal axis
        d = (math.cos(a), math.sin(a))
        ts = [(x - mx) * d[0] + (y - my) * d[1] for x, y in pts]
        t0, t1 = min(ts) - BARRIER_END_M, max(ts) + BARRIER_END_M
        strips.append((row_id, (mx + t0 * d[0], my + t0 * d[1]), (mx + t1 * d[0], my + t1 * d[1]), d, pts))
    return strips


def add_weed_barrier(stage, strips):
    """One mesh per row under /World/weed_barrier/row_<row_id>, laid over the plants' mounds, the woven-fabric
    textures, no collider."""
    surface = textured_material(stage, "/World/Materials/weed_barrier", BARRIER_TEX, BARRIER_SPECULAR_LEVEL)
    UsdGeom.Xform.Define(stage, "/World/weed_barrier")
    h = BARRIER_WIDTH / 2
    for row_id, (x0, y0), (x1, y1), (dx, dy), plant_xy in strips:
        length = math.hypot(x1 - x0, y1 - y0)
        # strip-local grid: t along the row from its start, s across (+s = left of the direction)
        t = np.linspace(0.0, length, max(2, round(length / MOUND_GRID_M) + 1))
        s_ = np.linspace(-h, h, max(2, round(BARRIER_WIDTH / MOUND_GRID_M) + 1))
        T, S = np.meshgrid(t, s_, indexing="ij")
        pxy = np.array(plant_xy) - (x0, y0)
        pt, ps = pxy @ (dx, dy), pxy @ (-dy, dx)
        r = np.hypot(T[..., None] - pt, S[..., None] - ps)
        Z = BARRIER_Z + MOUND_HEIGHT * (np.cos(np.minimum(r / MOUND_RADIUS, 1.0) * np.pi / 2) ** 2).max(-1)
        # smooth normals from the height field
        gz_t, gz_s = np.gradient(Z, t, s_)
        nt, ns = -gz_t, -gz_s
        nrm = np.stack([nt * dx - ns * dy, nt * dy + ns * dx, np.ones_like(Z)], -1)
        nrm /= np.linalg.norm(nrm, axis=-1, keepdims=True)
        X, Y = x0 + T * dx - S * dy, y0 + T * dy + S * dx
        nt_, ns_ = len(t), len(s_)
        idx = np.arange(nt_ * ns_).reshape(nt_, ns_)
        # quads counter-clockwise seen from above: (t, s) -> (t+1, s) -> (t+1, s+1) -> (t, s+1)
        quads = np.stack([idx[:-1, :-1], idx[1:, :-1], idx[1:, 1:], idx[:-1, 1:]], -1).reshape(-1)
        mesh = UsdGeom.Mesh.Define(stage, f"/World/weed_barrier/row_{row_id}")
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.stack([X, Y, Z], -1).reshape(-1, 3).astype(np.float32)))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(quads) // 4, 4, np.int32)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(quads.astype(np.int32)))
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(nrm.reshape(-1, 3).astype(np.float32)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        # u: one tile per BARRIER_TILE_M along the row from a per-row offset; v: the tile across the strip's width
        u0 = random.Random(f"barrier{row_id}").random()
        uv = np.stack([u0 + T / BARRIER_TILE_M, 0.5 + S / BARRIER_TILE_M], -1).reshape(-1, 2).astype(np.float32)
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                UsdGeom.Tokens.vertex).Set(Vt.Vec2fArray.FromNumpy(uv))
        mesh.CreateExtentAttr([Gf.Vec3f(float(X.min()), float(Y.min()), float(Z.min())),
                               Gf.Vec3f(float(X.max()), float(Y.max()), float(Z.max()))])
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*BARRIER_COLOR)])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(surface)


def hide_grass_under(stage, strips, center):
    """invisibleIds on the ground cover's PointInstancers for every blade rooted on a weed barrier. The patch is
    at /World/GroundCover translated to `center`, unscaled in x/y, its positions in metres. Returns the count."""
    hidden = 0
    h = BARRIER_WIDTH / 2
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/GroundCover")):
        if not prim.IsA(UsdGeom.PointInstancer):
            continue
        pi = UsdGeom.PointInstancer(prim)
        ids = []
        for i, p in enumerate(pi.GetPositionsAttr().Get()):
            x, y = p[0] + center[0], p[1] + center[1]
            for _, (x0, y0), (x1, y1), (dx, dy), _ in strips:
                t = (x - x0) * dx + (y - y0) * dy
                if 0.0 <= t <= math.hypot(x1 - x0, y1 - y0) and abs(-(x - x0) * dy + (y - y0) * dx) <= h:
                    ids.append(i)
                    break
        pi.CreateInvisibleIdsAttr().Set(Vt.Int64Array(ids))
        hidden += len(ids)
    return hidden


def scatter_points(box, ground, count, min_dist, margin, radius, rng, placed):
    """Up to `count` random points on the ground square `ground` (x0, y0, x1, y1), at least `margin` inside its
    edge and at least `min_dist` from the field box `box` (distance to the rectangle, so the clearing has rounded
    corners), none closer than radius + r to a point already in `placed` [(x, y, r)], which it extends."""
    bx0, by0, bx1, by1 = box
    gx0, gy0, gx1, gy1 = (ground[0] + margin, ground[1] + margin, ground[2] - margin, ground[3] - margin)
    out = []
    for _ in range(count * 200):
        if len(out) == count:
            break
        x, y = rng.uniform(gx0, gx1), rng.uniform(gy0, gy1)
        # +-EDGE_JITTER_M per candidate: a ragged forest edge instead of a sharp rounded rectangle
        if (math.hypot(max(bx0 - x, 0.0, x - bx1), max(by0 - y, 0.0, y - by1))
                < min_dist + rng.uniform(-EDGE_JITTER_M, EDGE_JITTER_M)):
            continue
        if any((x - px) ** 2 + (y - py) ** 2 < (radius + pr) ** 2 for px, py, pr in placed):
            continue
        placed.append((x, y, radius))
        out.append((x, y))
    if len(out) < count:
        print(f"scatter: placed {len(out)} of {count} (no room left)")
    return out


def add_border(stage, box, center):
    h = GROUND_SIZE / 2
    ground = (center[0] - h, center[1] - h, center[0] + h, center[1] + h)
    placed = []  # shared, so no group lands on another's items
    for group, (assets, count, min_dist, margin, height, radius, seed) in BORDER.items():
        rng = random.Random(seed)
        protos = {}
        for a in dict.fromkeys(assets):
            name = f"{group}_{os.path.splitext(os.path.basename(a))[0]}"
            protos[a] = (add_prototype(stage, name, a), asset_bounds(a).GetSize()[2])
        UsdGeom.Xform.Define(stage, f"/World/{group}")
        for t, (x, y) in enumerate(scatter_points(box, ground, count, min_dist, margin, radius, rng, placed)):
            proto, z_size = protos[assets[rng.randrange(len(assets))]]
            k = rng.uniform(*height) / max(z_size, 1e-6)
            # the asset's root carries its own xform ops, so it goes on a child of the placement Xform
            prim = stage.DefinePrim(f"/World/{group}/item_{t}", "Xform")
            stage.DefinePrim(f"/World/{group}/item_{t}/asset", "Xform").GetReferences().AddInternalReference(proto)
            xf = UsdGeom.Xformable(prim)
            xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
            xf.AddRotateZOp().Set(rng.uniform(0, 360))
            xf.AddScaleOp().Set(Gf.Vec3d(k, k, k))


def add_arch(stage, rows):
    """The entrance arch across the west end of lane ARCH_LANE (rows: add_plants' [x_min, x_max, y, width])."""
    rows = sorted(rows, key=lambda r: r[2], reverse=True)
    if len(rows) < 2:
        return
    a, b = rows[min(ARCH_LANE, len(rows) - 2)], rows[min(ARCH_LANE, len(rows) - 2) + 1]
    x, y = min(a[0], b[0]) - ARCH_SETBACK_M, (a[2] + b[2]) / 2
    prim = stage.DefinePrim("/World/arch", "Xform")
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    xf.AddRotateZOp().Set(90.0)  # beam across the lane (lanes run along X)
    stage.DefinePrim("/World/arch/asset", "Xform").GetReferences().AddReference(f"{REL_ASSETS}/{ARCH}")
    return x, y


def main(plants_json, out):
    with open(plants_json) as f:
        data = json.load(f)
    plants = data["plants"]
    if not plants:
        sys.exit("no plants")
    half = FIELD_BOX_MARGIN
    box = (min(p["x"] for p in plants) - half, min(p["y"] for p in plants) - half,
           max(p["x"] for p in plants) + half, max(p["y"] for p in plants) + half)
    center = ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)

    layer = Sdf.Layer.CreateNew(out)
    stage = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    stage.CreateClassPrim("/World/Prototypes")
    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")  # setup_scene.py sets the fleet's settings on it
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    add_ground(stage, center)
    add_lights(stage)
    rows = add_plants(stage, plants)
    strips = row_strips(plants)
    add_weed_barrier(stage, strips)
    hidden = hide_grass_under(stage, strips, center)
    add_border(stage, box, center)
    arch = add_arch(stage, rows)

    # read by setup_scene.build_file_world: the web UI's spawn map draws these rows
    layer.customLayerData = {
        "lavender_rows": Vt.Vec4dArray([Gf.Vec4d(*r) for r in rows]),  # [x_min, x_max, y, width] per row
        "farm_source": data.get("source", ""),
        "plant_count": len(plants),
    }
    layer.Save()
    os.chmod(out, 0o666)  # written as the image's user; the host user may replace it
    print(f"wrote {out}: {len(plants)} plants in {len(rows)} rows, weed barrier under each "
          f"({hidden} grass blades hidden), field box "
          f"x {box[0]:.2f}..{box[2]:.2f} y {box[1]:.2f}..{box[3]:.2f}"
          + (f", arch at ({arch[0]:.2f}, {arch[1]:.2f})" if arch else ""))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])

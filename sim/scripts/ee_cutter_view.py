"""Look at the EE cutter USDs (make_ee_cutter_looks.py) in Isaac Sim: RTX snapshots under the farm's sky, headless.

  docker run --rm --gpus all --entrypoint bash -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNICLIENT_HUB_MODE=disabled \
      -v $PWD/sim:/sim a300-isaac-sim:6.0.0 -c "/isaac-sim/python.sh /sim/scripts/ee_cutter_view.py"
(scripts/plant_test.sh's cache mounts make the shader compile a one-off) -> sim/generated/ee_cutter/view_<model>_<angle>.png
--headed: a Kit window on the host display instead (X11 mounts as in scripts/plant_test.sh), open until closed.
"""
import sys

from isaacsim import SimulationApp

headed = "--headed" in sys.argv
app = SimulationApp({"headless": not headed, "renderer": "RayTracedLighting", "width": 1600, "height": 1000})

import os  # noqa: E402

import omni.usd  # noqa: E402
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport  # noqa: E402
from omni.kit.viewport.utility.camera_state import ViewportCameraState  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade  # noqa: E402

OUT = "/sim/generated/ee_cutter"
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
for i, (name, x) in enumerate((("cutter_gripper", 0.0), ("cutter", 0.3))):
    p = stage.DefinePrim(f"/World/{name}", "Xform")
    p.GetReferences().AddReference(f"/sim/assets/ee_cutter/{name}.usd")
    UsdGeom.Xformable(p).AddTranslateOp().Set(Gf.Vec3d(x, 0, 0.08))
ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.AddScaleOp().Set(Gf.Vec3f(2, 2, 0.01))
ground.AddTranslateOp(opSuffix="t").Set(Gf.Vec3d(0, 0, -1))  # scaled frame: top at z 0
mat = UsdShade.Material.Define(stage, "/World/ground_mat")
sh = UsdShade.Shader.Define(stage, "/World/ground_mat/s")
sh.CreateIdAttr("UsdPreviewSurface")
sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.35, 0.3, 0.25))
sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(mat)
sun = UsdLux.DistantLight.Define(stage, "/World/sun")  # same sky and sun as plant_test.py
sun.CreateIntensityAttr(5000)
sun.AddRotateXYZOp().Set(Gf.Vec3f(60, 2.0, -50))
sky = UsdLux.DomeLight.Define(stage, "/World/sky")
sky.CreateIntensityAttr(400)
sky.CreateTextureFileAttr("/sim/assets/sky/farm_field_puresky_2k.hdr")
sky.CreateTextureFormatAttr(UsdLux.Tokens.latlong)

vp = get_active_viewport()
cs = ViewportCameraState(vp.camera_path, vp)
for _ in range(60):  # textures load, RTX converges
    app.update()
# CAD frame: tool axis +y (blades, fingers), arm mount at y 0; eye directions from the model's centre
ANGLES = {"tip": (0, 1, 0.35), "mount": (0, -1, 0.35), "left": (-1, 0, 0.3), "right": (1, 0, 0.3),
          "top": (0.05, 0.15, 1), "three_quarter": (0.8, 0.9, 0.6)}
MODELS = ("cutter_gripper", "cutter")
bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
if headed:
    cs.set_position_world(Gf.Vec3d(-0.25, -0.15, 0.3), True)
    cs.set_target_world(Gf.Vec3d(0.0, 0.1, 0.1), True)
    while app.is_running():
        app.update()
    MODELS = ()
done = []
for name in MODELS:
    for other in MODELS:  # only this model in the frame
        UsdGeom.Imageable(stage.GetPrimAtPath(f"/World/{other}")).GetVisibilityAttr().Set(
            "inherited" if other == name else "invisible")
    box = bbox.ComputeWorldBound(stage.GetPrimAtPath(f"/World/{name}")).ComputeAlignedRange()
    ctr, r = box.GetMidpoint(), box.GetSize().GetLength() / 2
    for angle, d in ANGLES.items():
        cs.set_position_world(ctr + Gf.Vec3d(*d).GetNormalized() * r * 2.6, True)  # fits the ~40 deg vertical fov
        cs.set_target_world(ctr, True)
        for _ in range(40):
            app.update()
        capture_viewport_to_file(vp, f"{OUT}/view_{name}_{angle}.png")
        for _ in range(5):
            app.update()
        done.append(f"view_{name}_{angle}.png")
print("[ee_cutter_view] wrote", done, flush=True)
app.close()

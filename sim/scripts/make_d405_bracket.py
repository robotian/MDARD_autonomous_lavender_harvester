"""D405 bracket (black ABS print): holds the RealSense D405 where the robot's URDF has camera_1 (mtu32_description,
on arm_0_end_effector_link), bolted to the cutter fixture's camera-side face; replaces the assembly's old D405_bracket.

CAD frame of cutter_and_gripper_and_camera_bracket.stp (mm; y = tool axis, mount face y = 0, camera side -z). camera_1's
42 x 42 x 23 mm box sits at x -21..21, y -13.7..9.4 (lens +y), z -90..-48; the fixture face is z = -36 with M3 holes at
x +-19.5, y 7 / 26; the Kinova wrist reaches |z| 32.7 behind the mount face, so the plate (z -40..-36) clears it.
M3 x 12 screws from the fixture's inner bores go into heat-set inserts in the plate.
Writes sim/assets/ee_cutter/src/d405_bracket.stp (+ .stl for the printer), then step_to_glb.py makes the glb:
  v/bin/python sim/scripts/make_d405_bracket.py && v/bin/python sim/scripts/step_to_glb.py \
      sim/assets/ee_cutter/src/d405_bracket.stp sim/generated/ee_cutter/d405_bracket.glb
"""
import os

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.GProp import GProp_GProps
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.XCAFDoc import XCAFDoc_DocumentTool

NAME = "D405_bracket_abs"
OUT = os.path.join(os.path.dirname(__file__), "..", "assets", "ee_cutter", "src", "d405_bracket")
W = 24.5        # half width (x), as the old bracket
PLATE = (-40.0, -36.0)   # z: on the fixture face
SEAT_Z = -48.0  # camera_1's bottom face
BACK = -17.0    # y: 3 mm lip behind the camera's back (-13.7, 0.3 mm play)
FRONT = 33.0    # y: plate end over the fixture (its face runs to 36)
SEAT_FRONT = 9.4  # y: flush with the camera's front
FIXTURE_HOLES = ((-19.5, 7.0), (19.5, 7.0), (-19.5, 26.0), (19.5, 26.0))
CAMERA_SCREW = (0.0, -5.2)  # camera_1_bottom_screw_frame
M3, M3_HEAD = 1.7, 3.25     # clearance / counterbore radius
INSERT = 2.0  # M3 x 4 heat-set insert hole radius


def box(x0, x1, y0, y1, z0, z1):
    return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), gp_Pnt(x1, y1, z1)).Shape()


def cyl(x, y, z0, z1, r):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, z0), gp_Dir(0, 0, 1)), r, z1 - z0).Shape()


def fuse(a, b):
    return BRepAlgoAPI_Fuse(a, b).Shape()


def cut(a, b):
    return BRepAlgoAPI_Cut(a, b).Shape()


s = box(-W, W, BACK, FRONT, *PLATE)
s = fuse(s, box(-W, W, BACK, SEAT_FRONT, SEAT_Z, PLATE[0]))
s = fuse(s, box(-W, W, BACK, BACK + 3.0, SEAT_Z - 10.0, SEAT_Z))
# round the outer vertical corners (edges along z on the sides)
f = BRepFilletAPI_MakeFillet(s)
e = TopExp_Explorer(s, TopAbs_EDGE)
while e.More():
    edge = TopoDS.Edge_s(e.Current())
    a, b = (BRep_Tool.Pnt_s(v) for v in (TopExp.FirstVertex_s(edge), TopExp.LastVertex_s(edge)))
    if abs(a.X() - b.X()) < 1e-6 and abs(a.Y() - b.Y()) < 1e-6 and abs(abs(a.X()) - W) < 1e-6 \
            and min(abs(a.Y() - y) for y in (BACK, FRONT, SEAT_FRONT)) < 1e-6:
        f.Add(2.0, edge)
    e.Next()
s = f.Shape()
for x, y in FIXTURE_HOLES:  # M3 heat-set inserts: the screws come from the fixture's inner bores (heads at z -28)
    s = cut(s, cyl(x, y, PLATE[1] - 4.5, PLATE[1] + 1, INSERT))
x, y = CAMERA_SCREW  # M3 down into the camera; head sunk 3.5 mm under the plate's top (the wrist is 3.3 mm above it)
s = cut(s, cyl(x, y, SEAT_Z - 1, PLATE[1] + 1, M3))
s = cut(s, cyl(x, y, PLATE[1] - 3.5, PLATE[1] + 1, M3_HEAD))

props = GProp_GProps()
BRepGProp.VolumeProperties_s(s, props)
vol = props.Mass()  # mm^3
assert 12000 < vol < 30000, vol  # self-check: about 49 x 50 x 4 plate + seat + lip, less holes
print(f"{NAME}: {vol / 1000:.1f} cm^3, {vol / 1000 * 1.04:.0f} g solid ABS")

doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
TDataStd_Name.Set_s(st.AddShape(s, False), TCollection_ExtendedString(NAME))
w = STEPCAFControl_Writer()
w.SetNameMode(True)
assert w.Transfer(doc, STEPControl_AsIs) and w.Write(OUT + ".stp") == 1
BRepMesh_IncrementalMesh(s, 0.05, False, 0.1, True)
assert StlAPI_Writer().Write(s, OUT + ".stl")
print("wrote", os.path.abspath(OUT + ".stp"), os.path.abspath(OUT + ".stl"))

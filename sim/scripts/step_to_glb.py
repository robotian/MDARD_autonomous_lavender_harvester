"""STEP assembly -> glTF binary (metres, Y-up), part names and CAD colours kept; Blender imports the result.

Needs OpenCascade's Python bindings on the host (not the Isaac image): python3 -m venv v && v/bin/pip install cadquery-ocp
  v/bin/python sim/scripts/step_to_glb.py sim/assets/ee_cutter/src/cutter.stp out.glb [--deflection-mm 0.1]
"""
import argparse

from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.Message import Message_ProgressRange
from OCP.RWGltf import RWGltf_CafWriter
from OCP.RWMesh import RWMesh_CoordinateSystem
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TColStd import TColStd_IndexedDataMapOfStringString
from OCP.TCollection import TCollection_AsciiString, TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool

ap = argparse.ArgumentParser()
ap.add_argument("step")
ap.add_argument("glb")
ap.add_argument("--deflection-mm", type=float, default=0.1)
a = ap.parse_args()

doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
r = STEPCAFControl_Reader()
r.SetNameMode(True)
r.SetColorMode(True)
assert r.ReadFile(a.step) == IFSelect_RetDone, a.step
assert r.Transfer(doc)

st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
roots = TDF_LabelSequence()
st.GetFreeShapes(roots)
for i in range(1, roots.Length() + 1):
    # the reader keeps STEP's mm; 8 deg angular deflection still looks round on 3 mm bolts
    BRepMesh_IncrementalMesh(st.GetShape_s(roots.Value(i)), a.deflection_mm, False, 8 * 3.14159 / 180, True)

w = RWGltf_CafWriter(TCollection_AsciiString(a.glb), True)
w.ChangeCoordinateSystemConverter().SetInputLengthUnit(0.001)
w.ChangeCoordinateSystemConverter().SetInputCoordinateSystem(RWMesh_CoordinateSystem.RWMesh_CoordinateSystem_Zup)  # CAD Z-up -> glTF Y-up
assert w.Perform(doc, TColStd_IndexedDataMapOfStringString(), Message_ProgressRange()), a.glb
print("wrote", a.glb)

import json, sys, numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.BRep import BRep_Tool
from OCP.TopoDS import TopoDS
from OCP.IFSelect import IFSelect_RetDone
import pickle

doc = TDocStd_Document(TCollection_ExtendedString("doc"))
r = STEPCAFControl_Reader(); r.SetNameMode(True); r.SetColorMode(True)
assert r.ReadFile(sys.argv[1]) == IFSelect_RetDone
r.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

def name(lab):
    a = TDataStd_Name()
    if lab.FindAttribute(TDataStd_Name.GetID_s(), a):
        return a.Get().ToExtString()
    return "?"

def trsf_mat(loc):
    t = loc.Transformation()
    m = np.eye(4)
    for i in range(3):
        for j in range(4):
            m[i,j] = t.Value(i+1, j+1)
    return m

leaves = []; path_labels=[]; comp_stack=[]
def walk(lab, loc, path):
    if st.IsAssembly_s(lab):
        comps = TDF_LabelSequence(); st.GetComponents_s(lab, comps)
        for i in range(1, comps.Length()+1):
            c = comps.Value(i)
            ref = TDF_Label(); st.GetReferredShape_s(c, ref)
            cloc = st.GetLocation_s(c)
            comp_stack.append(c); walk(ref, loc.Multiplied(cloc), path + [name(c)]); comp_stack.pop()
    else:
        shape = st.GetShape_s(lab).Moved(loc)
        leaves.append((path, name(lab), shape, loc)); path_labels.append((lab, comp_stack[-1] if comp_stack else None))

roots = TDF_LabelSequence(); st.GetFreeShapes(roots)
for i in range(1, roots.Length()+1):
    walk(roots.Value(i), TopLoc_Location(), [name(roots.Value(i))])

from OCP.XCAFDoc import XCAFDoc_ColorType
from OCP.Quantity import Quantity_Color
ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
def get_color(labs, shape):
    c = Quantity_Color()
    for lab in labs:
        if lab is None: continue
        for t in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
            if ct.GetColor_s(lab, t, c): return [c.Red(), c.Green(), c.Blue()]
    # try faces of the part label
    from OCP.TopExp import TopExp_Explorer as E
    lab=labs[0]; sh=st.GetShape_s(lab); e=E(sh, TopAbs_FACE)
    while e.More():
        if ct.GetColor_s(e.Current(), XCAFDoc_ColorType.XCAFDoc_ColorSurf, c): return [c.Red(), c.Green(), c.Blue()]
        e.Next()
    return None
out = []
meshes = {}
for k,(path, pname, shape, loc) in enumerate(leaves):
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(shape, g)
    vol = g.Mass(); c = g.CentreOfMass()
    Im = g.MatrixOfInertia(); I=[[Im.Value(a,b) for b in (1,2,3)] for a in (1,2,3)]
    col = get_color(path_labels[k], shape)
    BRepMesh_IncrementalMesh(shape, 0.2, False, 0.3, True)
    V=[];F=[]
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        f = TopoDS.Face(exp.Current()); L = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, L)
        if tri is not None:
            tr = L.Transformation(); off=len(V)
            for i in range(1, tri.NbNodes()+1):
                p = tri.Node(i).Transformed(tr); V.append((p.X(),p.Y(),p.Z()))
            rev = f.Orientation()==1
            for i in range(1, tri.NbTriangles()+1):
                a,b_,c_ = tri.Triangle(i).Get()
                F.append((off+a-1, off+c_-1, off+b_-1) if rev else (off+a-1, off+b_-1, off+c_-1))
        exp.Next()
    meshes[k] = (np.array(V), np.array(F))
    out.append(dict(i=k, path=path, part=pname, vol_mm3=vol, com=[c.X(),c.Y(),c.Z()], bbox=(meshes[k][0].min(0).tolist()+meshes[k][0].max(0).tolist()) if len(meshes[k][0]) else [], T=trsf_mat(loc).tolist(), I=I, color=col))
json.dump(out, open("leaves.json","w"), indent=1)
pickle.dump(meshes, open("meshes.pkl","wb"))
print(len(out))

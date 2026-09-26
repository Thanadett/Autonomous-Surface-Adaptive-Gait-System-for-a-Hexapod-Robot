import vtk, numpy as np
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray
def polydata(V,F):
    pts=vtk.vtkPoints(); pts.SetData(numpy_to_vtk(np.ascontiguousarray(V,dtype=float),deep=True))
    cells=np.hstack([np.full((len(F),1),3),F]).astype(np.int64).ravel()
    ca=vtk.vtkCellArray(); ca.SetCells(len(F), numpy_to_vtkIdTypeArray(cells,deep=True))
    pd=vtk.vtkPolyData(); pd.SetPoints(pts); pd.SetPolys(ca); return pd
def render(items, out, pos, focal, up=(0,1,0), size=(1000,800), lines=(), labels=(), parallel=None):
    ren=vtk.vtkRenderer(); ren.SetBackground(1,1,1)
    for V,F,col,*op in items:
        m=vtk.vtkPolyDataMapper(); m.SetInputData(polydata(V,F))
        a=vtk.vtkActor(); a.SetMapper(m); a.GetProperty().SetColor(*col)
        if op: a.GetProperty().SetOpacity(op[0])
        ren.AddActor(a)
    for p0,p1,col in lines:
        l=vtk.vtkLineSource(); l.SetPoint1(*p0); l.SetPoint2(*p1)
        m=vtk.vtkPolyDataMapper(); m.SetInputConnection(l.GetOutputPort())
        a=vtk.vtkActor(); a.SetMapper(m); a.GetProperty().SetColor(*col); a.GetProperty().SetLineWidth(4); ren.AddActor(a)
    for p,txt in labels:
        t=vtk.vtkBillboardTextActor3D(); t.SetInput(txt); t.SetPosition(*p)
        t.GetTextProperty().SetFontSize(18); t.GetTextProperty().SetColor(0,0,0); ren.AddActor(t)
    cam=ren.GetActiveCamera(); cam.SetPosition(*pos); cam.SetFocalPoint(*focal); cam.SetViewUp(*up)
    if parallel: cam.ParallelProjectionOn(); cam.SetParallelScale(parallel)
    ren.ResetCameraClippingRange()
    rw=vtk.vtkRenderWindow(); rw.SetOffScreenRendering(1); rw.AddRenderer(ren); rw.SetSize(*size); rw.Render()
    w=vtk.vtkWindowToImageFilter(); w.SetInput(rw); w.Update()
    p=vtk.vtkPNGWriter(); p.SetFileName(out); p.SetInputConnection(w.GetOutputPort()); p.Write()

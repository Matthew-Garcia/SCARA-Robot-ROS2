#!/usr/bin/env python3
"""Export rigid links from the original STEP. Never writes to source CAD.
Requires cadquery==2.8.0, trimesh. Run from any directory.
"""
from pathlib import Path
import hashlib, json, math
import cadquery as cq
import numpy as np
import trimesh
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name

WS=Path(__file__).resolve().parents[1]/'ros2_ws'
SOURCE=WS.parent/'cad/step/SCARA Robot 3D Model.STEP'
OUT=WS/'src/scara_description'
DOC=TDocStd_Document(TCollection_ExtendedString('scara'))
reader=STEPCAFControl_Reader(); reader.SetNameMode(True)
assert int(reader.ReadFile(str(SOURCE))) == 1
assert reader.Transfer(DOC)
tool=XCAFDoc_DocumentTool.ShapeTool_s(DOC.Main())
roots=TDF_LabelSequence(); tool.GetFreeShapes(roots)
parts=[]
def walk(label, location=cq.Location()):
    if tool.IsReference_s(label):
        referred=TDF_Label(); tool.GetReferredShape_s(label,referred)
        walk(referred,location*cq.Location(tool.GetLocation_s(label))); return
    children=TDF_LabelSequence(); tool.GetComponents_s(label,children)
    if children.Length():
        for i in range(1,children.Length()+1): walk(children.Value(i),location)
    else:
        attr=TDataStd_Name(); assert label.FindAttribute(TDataStd_Name.GetID_s(),attr)
        parts.append((attr.Get().ToExtString(),cq.Shape.cast(tool.GetShape_s(label)).located(location)))
for i in range(1,roots.Length()+1): walk(roots.Value(i))
assert len(parts)==111, 'Assembly changed; review the rigid-body mapping before export.'
# STEP instance indices, explicitly assigned by mechanical connection.
groups={
 'base_link':[0,1,2,3,4,5,7,8,9,55,81,82,83,84,88,89,90],
 'shoulder_link':[6,*range(10,28),78,79,80,*range(97,101),*range(105,109)],
 'arm1_link':[28,29,30,31,33,34,36,37,38,39,40,41,42,43,44,47,57,77,85,86,91,92],
 'arm2_link':[32,35,45,46,48,50,51,52,54,56,87,*range(93,97),*range(101,105)],
 'tool_link':[49,53,58,59,60],
}
assigned=[i for ids in groups.values() for i in ids]
assert len(assigned)==len(set(assigned)) and set(assigned)==(set(range(109))-set(range(61,77)))
def center(i): return np.array(parts[i][1].BoundingBox().center.toTuple())
p1=np.array([42.5,0.,31.]);p2=center(32);p3=center(49)
p2[2]=parts[32][1].BoundingBox().zmin
p3[2]=parts[49][1].BoundingBox().zmin
zslide=parts[28][1].BoundingBox().zmin
alpha=math.atan2(p2[1],p2[0]-p1[0]); beta=math.atan2(p3[1]-p2[1],p3[0]-p2[0])
# Tool x is chosen parallel to forearm at zero; finger orientation is preserved in its mesh.
frames={'base_link':([42.5,0.,-35.],0.), 'shoulder_link':(p1,alpha),
 'arm1_link':([42.5,0.,zslide],alpha),'arm2_link':(p2,beta),'tool_link':(p3,beta)}
L1=float(np.linalg.norm(p2[:2]-p1[:2])); L2=float(np.linalg.norm(p3[:2]-p2[:2]))
assert abs(L1-228)<0.001 and abs(L2-136.5)<0.001
manifest={'source':'cad/step/SCARA Robot 3D Model.STEP','source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
 'units':'metres','tessellation_linear_tolerance_mm':0.25,'tessellation_angular_tolerance_rad':0.2,
 'reference_joints':[alpha,0.,beta-alpha,0.],
 'geometry':{'l1':L1/1000,'l2':L2/1000,'shoulder_z':.066,'slide_z':(zslide-31)/1000,
 'elbow_z':(p2[2]-zslide)/1000,'wrist_z':(p3[2]-p2[2])/1000,
 'tcp_z':(min(parts[i][1].BoundingBox().zmin for i in groups['tool_link'])-p3[2])/1000},
 'excluded':[{'index':i,'name':parts[i][0],'reason':'Loose laser accessory beside base, not attached to manipulator'} for i in [109,110]] + [{'index':i,'name':parts[i][0],'reason':'Original static gripper geometry omitted from robot visual; functional two-finger URDF gripper is used instead'} for i in range(61,77)],'links':{}}
(OUT/'meshes').mkdir(parents=True,exist_ok=True);(OUT/'config').mkdir(exist_ok=True)
# Collision hulls per component preserve the gaps between rods and the arm profile.
# Small fasteners, belts, and bearings need no additional collision hulls.
collision_ids={0,1,10,11,12,13,14,15,20,27,28,29,32,38,45,46,48,49,58,59,77,79,80,81,82}
for link,ids in groups.items():
    origin,yaw=frames[link];origin=np.asarray(origin)
    c,s=math.cos(yaw),math.sin(yaw);R=np.array([[c,s,0],[-s,c,0],[0,0,1]])
    meshes=[]; entries=[]
    for i in ids:
        vertices,faces=parts[i][1].tessellate(.25,.2)
        v=np.array([p.toTuple() for p in vertices]);v=((v-origin)@R.T)/1000
        mesh=trimesh.Trimesh(vertices=v,faces=faces,process=False);meshes.append(mesh)
        entry={'index':i,'name':parts[i][0],'triangles':len(faces)}
        if i in collision_ids:
            filename=f'{link}_collision_{i:03}.stl';mesh.convex_hull.export(OUT/'meshes'/filename);entry['collision']=filename
        entries.append(entry)
    merged=trimesh.util.concatenate(meshes);merged.export(OUT/'meshes'/f'{link}.stl')
    manifest['links'][link]={'cad_frame_mm':origin.tolist(),'cad_yaw_rad':yaw,'parts':entries,'bounds_m':merged.bounds.tolist()}
    print(link,len(ids),'parts',len(merged.faces),'triangles',flush=True)
(OUT/'config/cad_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(manifest['geometry'],indent=2));print('Reference joints:',manifest['reference_joints'])

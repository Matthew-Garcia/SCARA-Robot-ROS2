#!/usr/bin/env python3
"""Render the real exported meshes; this is a CAD preview, not a Gazebo screenshot."""
from pathlib import Path
import json
import math
import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
W=Path(__file__).resolve().parents[1]/'ros2_ws';D=W/'src/scara_description'
M=json.loads((D/'config/cad_manifest.json').read_text());g=M['geometry']
q=M['reference_joints'];a,z,e,w=q
frames={'base_link':(0,0,0,0),'shoulder_link':(0,0,g['shoulder_z'],a),'arm1_link':(0,0,g['shoulder_z']+g['slide_z']+z,a),
 'arm2_link':(g['l1']*math.cos(a),g['l1']*math.sin(a),g['shoulder_z']+g['slide_z']+z+g['elbow_z'],a+e),
 'tool_link':(g['l1']*math.cos(a)+g['l2']*math.cos(a+e),g['l1']*math.sin(a)+g['l2']*math.sin(a+e),g['shoulder_z']+g['slide_z']+z+g['elbow_z']+g['wrist_z'],a+e+w)}
colors={'base_link':(.35,.39,.46),'shoulder_link':(.73,.76,.80),'arm1_link':(.10,.61,.76),'arm2_link':(.12,.64,.78),'tool_link':(.52,.56,.63)}
fig=plt.figure(figsize=(15,10.5),facecolor='#0e131c')
ax=fig.add_axes([0,.09,1,.78],projection='3d',facecolor='#0e131c',computed_zorder=False)
light=np.array([-.4,-.6,1]);light/=np.linalg.norm(light)
for index,(name,(x,y,z,yaw)) in enumerate(frames.items()):
    mesh=trimesh.load_mesh(D/'meshes'/f'{name}.stl')
    # Preview-only decimation; checked-in ROS meshes remain untouched.
    mesh=mesh.simplify_quadric_decimation(face_count=min(22000,len(mesh.faces)))
    c,s=math.cos(yaw),math.sin(yaw);R=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    vertices=mesh.vertices@R.T+np.array([x,y,z]);normals=mesh.face_normals@R.T
    shade=.48+.52*np.maximum(normals@light,0)
    rgb=np.clip(np.array(colors[name])[None,:]*shade[:,None],0,1)
    poly=Poly3DCollection(vertices[mesh.faces],facecolors=rgb,edgecolors='none',linewidths=0,zorder=3+index)
    ax.add_collection3d(poly)
for i in range(-4,7):
    ax.plot([i*.1,i*.1],[-.4,.6],[0,0],color='#293443',lw=.5,zorder=1)
    ax.plot([-.5,.65],[i*.1,i*.1],[0,0],color='#293443',lw=.5,zorder=1)
ax.set_xlim(-.27,.43);ax.set_ylim(-.30,.30);ax.set_zlim(0,.55);ax.set_box_aspect((.7,.6,.55));ax.view_init(elev=20,azim=-65);ax.set_proj_type('ortho');ax.set_axis_off()
fig.text(.055,.94,'SCARA / ORIGINAL CAD',color='#eff7ff',size=25,weight='bold')
fig.text(.055,.902,'ROS 2 Humble  |  Ubuntu 22.04  |  MoveIt 2  |  ros2_control',color='#80cce5',size=14)
fig.text(.055,.07,'228 mm + 136.5 mm arms  /  4 controlled axes  /  109 CAD part instances',color='#ccd9e8',size=13)
fig.text(.055,.038,'CAD kinematic preview. Original STEP pose. Not a Gazebo screenshot.',color='#8599b0',size=11)
output=W.parent/'docs/images/scara_cad_preview.png';output.parent.mkdir(exist_ok=True)
fig.savefig(output,dpi=130,facecolor=fig.get_facecolor());print(output)

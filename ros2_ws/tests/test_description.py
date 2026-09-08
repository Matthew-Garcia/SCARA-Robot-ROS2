from pathlib import Path
import hashlib
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
import xacro
import yaml

WS=Path(__file__).resolve().parents[1]
DESC=WS/'src/scara_description'
M=json.loads((DESC/'config/cad_manifest.json').read_text())

def urdf(mode):
    return ET.fromstring(xacro.process_file(str(DESC/'urdf/scara.urdf.xacro'),mappings={
      'mode':mode,'description_share':str(DESC),'controllers_file':str(WS/'src/scara_bringup/config/controllers.yaml')}).toxml())

def test_original_cad_and_manifest():
    assert hashlib.sha256((WS.parent/M['source']).read_bytes()).hexdigest()==M['source_sha256']
    ids=[p['index'] for link in M['links'].values() for p in link['parts']]
    assert sorted(ids)==list(range(109))
    assert abs(M['geometry']['l1']-.228)<1e-9
    assert abs(M['geometry']['l2']-.1365)<1e-9

def test_control_and_mesh_paths():
    names=['shoulder_joint','z_joint','elbow_joint','wrist_joint']
    for mode,plugin in [('mock','mock_components/GenericSystem'),('gazebo','gazebo_ros2_control/GazeboSystem')]:
        root=urdf(mode)
        assert root.find('ros2_control/hardware/plugin').text==plugin
        assert [j.attrib['name'] for j in root.findall('joint') if j.attrib['type']!='fixed']==names
        assert [j.attrib['name'] for j in root.findall('ros2_control/joint')]==names
        for mesh in root.findall('.//mesh'):assert Path(mesh.attrib['filename'].removeprefix('file://')).is_file()
        for inertia in root.findall('.//inertia'):
            assert all(float(inertia.attrib[key])>0 for key in ['ixx','iyy','izz'])
    config=yaml.safe_load((WS/'src/scara_bringup/config/controllers.yaml').read_text())
    assert config['arm_controller']['ros__parameters']['joints']==names

def transform(xyz,yaw=0):
    c,s=math.cos(yaw),math.sin(yaw);T=np.eye(4);T[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]];T[:3,3]=xyz;return T

def test_urdf_reconstructs_original_assembly_frames():
    root=urdf('mock');q=dict(zip(['shoulder_joint','z_joint','elbow_joint','wrist_joint'],M['reference_joints']))
    frames={'world':np.eye(4)}
    for j in root.findall('joint'):
        xyz=[float(x) for x in j.find('origin').attrib['xyz'].split()] if j.find('origin') is not None else [0,0,0]
        T=transform(xyz);val=q.get(j.attrib['name'],0)
        if j.attrib['type']=='revolute':T=T@transform([0,0,0],val)
        if j.attrib['type']=='prismatic':T=T@transform([0,0,val])
        frames[j.find('child').attrib['link']]=frames[j.find('parent').attrib['link']]@T
    for link,data in M['links'].items():
        expected=transform((np.array(data['cad_frame_mm'])-np.array([42.5,0,-35]))/1000,data['cad_yaw_rad'])
        assert np.allclose(frames[link],expected,atol=1e-9),link
    g=M['geometry'];z0=sum(g[k] for k in ['shoulder_z','slide_z','elbow_z','wrist_z','tcp_z'])
    assert abs(frames['tcp_link'][2,3]-z0)<1e-10

def test_all_original_assets_preserved_after_reorganization():
    root=WS.parent
    source=json.loads((root/'docs/source_manifest.json').read_text())
    assert len(source['files'])==77
    for record in source['files']:
        assert hashlib.sha256((root/record['path']).read_bytes()).hexdigest()==record['sha256'],record['path']

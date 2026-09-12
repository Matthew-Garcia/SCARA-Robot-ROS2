"""Apply native SDF colors after Gazebo Classic has reduced fixed joints."""
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

COLORS = {
    'blue': '0.02 0.28 0.82 1',
    'grey': '0.75 0.78 0.82 1',
    'black': '0.02 0.02 0.02 1',
}


def apply_materials(sdf, urdf):
    # Match mesh URIs, not link names: fixed-joint reduction renames links/visuals.
    source = ET.fromstring(urdf)
    palette = {}
    for visual in source.findall('./link/visual'):
        mesh = visual.find('geometry/mesh')
        color = visual.find('material/color')
        if mesh is not None and color is not None:
            palette[mesh.attrib['filename']] = color.attrib['rgba']
    root = ET.fromstring(sdf)
    if root.tag != 'sdf' or root.find('model') is None:
        raise RuntimeError('Gazebo conversion did not produce an SDF model')
    seen = set()
    for visual in root.findall('.//link/visual'):
        uri = visual.findtext('geometry/mesh/uri')
        if uri not in palette:
            continue
        seen.add(uri)
        for old in visual.findall('material'):
            visual.remove(old)
        material = ET.SubElement(visual, 'material')
        # Native colors avoid OGRE script lookup and URDF extension loss.
        ET.SubElement(material, 'ambient').text = palette[uri]
        ET.SubElement(material, 'diffuse').text = palette[uri]
        ET.SubElement(material, 'specular').text = '0.1 0.1 0.1 1'
    missing = set(palette) - seen
    if missing:
        raise RuntimeError('Gazebo conversion lost colored meshes: ' + ', '.join(sorted(missing)))
    if not seen:
        raise RuntimeError('No robot mesh colors were applied')
    return ET.tostring(root, encoding='unicode')


def build_gazebo_sdf(urdf):
    with tempfile.TemporaryDirectory(prefix='scara-convert-') as directory:
        path = Path(directory) / 'robot.urdf'
        path.write_text(urdf)
        result = subprocess.run(['gz', 'sdf', '-p', str(path)],
                                capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise RuntimeError('Gazebo URDF conversion failed: ' + result.stderr)
        return apply_materials(result.stdout, urdf)

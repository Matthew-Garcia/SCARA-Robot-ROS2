"""Split original STEP-export triangles by material, without remeshing."""
import json
import struct
import sys
from pathlib import Path

def generate(source, output):
    source, output = Path(source), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((source/'config/cad_manifest.json').read_text())
    xml = ['<robot xmlns:xacro="http://www.ros.org/wiki/xacro">']
    counts = {'motors': 0, 'rods': 0}
    for link in ['base_link', 'shoulder_link', 'arm1_link', 'arm2_link']:
        raw = (source/'meshes'/f'{link}.stl').read_bytes()
        parts = manifest['links'][link]['parts']
        assert struct.unpack_from('<I', raw, 80)[0] == sum(p['triangles'] for p in parts)
        buckets = {}
        offset = 84
        for part in parts:
            name = part['name'].lower()
            color = 'grey' if link == 'shoulder_link' else 'blue'
            if 'nema 17' in name:
                color = 'black'; counts['motors'] += 1
            elif 'smooth rod d10mm l400mm' in name:
                color = 'grey'; counts['rods'] += 1
            elif link == 'shoulder_link' and any(s in name for s in ['z-axis top plate', 'z-axis bottom plate', 'top cover', 'base cover']):
                color = 'blue'
            length = part['triangles']*50
            buckets.setdefault(color, bytearray()).extend(raw[offset:offset+length])
            offset += length
        assert offset == len(raw)
        for color, data in buckets.items():
            visual = f'{link}_{color}_visual'
            filename = visual+'.stl'
            (output/filename).write_bytes(bytes(80)+struct.pack('<I', len(data)//50)+data)
            rgba, material = {'blue':('0.02 0.28 0.82 1','Gazebo/Blue'), 'grey':('0.75 0.78 0.82 1','Gazebo/Grey'), 'black':('0.02 0.02 0.02 1','Gazebo/Black')}[color]
            # Fixed visual-only children are reduced into their physical parent by
            # Gazebo. Materials transfer per visual; no dynamics or joints change.
            xml.append(f'<link name="{visual}"><visual><geometry><mesh filename="file://$(arg description_share)/meshes/{filename}"/></geometry><material name="{visual}_material"><color rgba="{rgba}"/></material></visual></link><joint name="{visual}_fixed" type="fixed"><parent link="{link}"/><child link="{visual}"/></joint>')
            xml.append(f'<gazebo reference="{visual}"><material>{material}</material></gazebo>')
    assert counts == {'motors':4, 'rods':4}, counts
    xml.append('</robot>')
    (output/'colored_visuals.xacro').write_text('\n'.join(xml))
    print('Preserved CAD triangles; colored four motors and four rods.')

if __name__ == '__main__':
    generate(*sys.argv[1:])

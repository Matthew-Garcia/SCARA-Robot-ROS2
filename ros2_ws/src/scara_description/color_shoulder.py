"""Color connected plate components without changing any CAD triangles."""
import struct
import sys
from pathlib import Path

def generate(source, target):
    data = Path(source).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    triangles = [[struct.unpack_from('<3f', data, 84+i*50+12+j*12)
                  for j in range(3)] for i in range(count)]
    parents = list(range(count))
    def root(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i
    seen = {}
    for i, triangle in enumerate(triangles):
        for vertex in triangle:
            key = tuple(round(v, 7) for v in vertex)
            if key in seen:
                parents[root(i)] = root(seen[key])
            else:
                seen[key] = i
    groups = {}
    for i in range(count):
        groups.setdefault(root(i), []).append(i)
    plates = set()
    for ids in groups.values():
        vertices = [v for i in ids for v in triangles[i]]
        lo = [min(v[a] for v in vertices) for a in range(3)]
        hi = [max(v[a] for v in vertices) for a in range(3)]
        if hi[0]-lo[0] > .119 and hi[2]-lo[2] < .07:
            plates.update(ids)
    assert len(plates) == 42216, 'Unexpected CAD plate geometry'
    positions = ' '.join(str(v) for t in triangles for p in t for v in p)
    effects = ''.join(f'<effect id="{name}-fx"><profile_COMMON><technique sid="common"><phong><diffuse><color>{color}</color></diffuse></phong></technique></profile_COMMON></effect>' for name,color in [('blue','0.02 0.28 0.82 1'),('grey','0.75 0.78 0.82 1')])
    materials = ''.join(f'<material id="{n}"><instance_effect url="#{n}-fx"/></material>' for n in ['blue','grey'])
    batches = ''
    for name, ids in [('blue',sorted(plates)),('grey',[i for i in range(count) if i not in plates])]:
        indices = ' '.join(str(3*i+j) for i in ids for j in range(3))
        batches += f'<triangles material="{name}" count="{len(ids)}"><input semantic="VERTEX" source="#verts" offset="0"/><p>{indices}</p></triangles>'
    bindings = ''.join(f'<instance_material symbol="{n}" target="#{n}"/>' for n in ['blue','grey'])
    Path(target).write_text(f'''<?xml version="1.0"?><COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1"><asset><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset><library_effects>{effects}</library_effects><library_materials>{materials}</library_materials><library_geometries><geometry id="shoulder"><mesh><source id="pos"><float_array id="pos-array" count="{count*9}">{positions}</float_array><technique_common><accessor source="#pos-array" count="{count*3}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source><vertices id="verts"><input semantic="POSITION" source="#pos"/></vertices>{batches}</mesh></geometry></library_geometries><library_visual_scenes><visual_scene id="scene"><node><instance_geometry url="#shoulder"><bind_material><technique_common>{bindings}</technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#scene"/></scene></COLLADA>''')

if __name__ == '__main__':
    generate(sys.argv[1], sys.argv[2])

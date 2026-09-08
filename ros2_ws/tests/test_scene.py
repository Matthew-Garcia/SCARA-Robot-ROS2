from pathlib import Path
import xml.etree.ElementTree as ET
import math
import yaml
import pytest

B=Path(__file__).resolve().parents[1]/'src/scara_bringup'
CONFIG=yaml.safe_load((B/'config/scene_objects.yaml').read_text())
WORLD=ET.parse(B/'worlds/scara.world').getroot().find('world')

def numbers(text):return [float(v) for v in text.split()]

def test_exactly_three_movable_hanoi_rings():
    movable=[o for o in CONFIG['objects'] if not o['static']]
    assert {o['name'] for o in movable}=={'pickup_cube','pickup_sphere','hanoi_ring_large','hanoi_ring_medium','hanoi_ring_small'}
    rings=[o for o in movable if o['name'].startswith('hanoi_ring')]
    assert len(rings)==3
    for ring in rings:
        assert len(ring['shapes'])==24
        for segment in ring['shapes']:
            x,y,*_=segment['pose']
            assert math.hypot(x,y)-segment['size'][0]/2==pytest.approx(.005)
        path=B/'models'/ring['name']/'meshes/ring.stl'
        assert path.is_file() and path.stat().st_size>84
        assert (path.parent.parent/'model.config').is_file()
    # All rings start stacked on the left peg, above the raised board.
    assert [o['pose'][2] for o in rings]==pytest.approx([.057,.065,.073])
    assert all(o['pose'][:2]==[.2,-.115] for o in rings)

def test_moveit_and_gazebo_collision_geometry_match():
    models={m.attrib['name']:m for m in WORLD.findall('model')}
    for item in CONFIG['objects']:
        model=models[item['name']]
        assert numbers(model.findtext('pose'))==pytest.approx(item['pose'])
        assert model.findtext('static')==str(item['static']).lower()
        collisions=model.findall('link/collision')
        assert len(collisions)==len(item['shapes'])
        for shape,collision in zip(item['shapes'],collisions):
            assert numbers(collision.findtext('pose'))==pytest.approx(shape['pose'])
            geom=collision.find('geometry/'+shape['type']);assert geom is not None
            if shape['type']=='box':assert numbers(geom.findtext('size'))==pytest.approx(shape['size'])
            else:
                assert float(geom.findtext('radius'))==pytest.approx(shape['radius'])
                if shape['type']=='cylinder':assert float(geom.findtext('length'))==pytest.approx(shape['length'])
        if not item['static']:
            assert float(model.findtext('link/inertial/mass'))>0
            assert all(float(model.findtext('link/inertial/inertia/'+k))>0 for k in ['ixx','iyy','izz'])
    assert WORLD.find("plugin[@filename='libgazebo_ros_state.so']") is not None

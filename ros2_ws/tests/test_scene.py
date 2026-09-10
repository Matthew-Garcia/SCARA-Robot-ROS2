from pathlib import Path
import xml.etree.ElementTree as ET
import math
import yaml
import pytest

B=Path(__file__).resolve().parents[1]/'src/scara_bringup'
CONFIG=yaml.safe_load((B/'config/scene_objects.yaml').read_text())
WORLD=ET.parse(B/'worlds/scara.world').getroot().find('world')
CONVEYOR=ET.parse(B/'worlds/conveyor_sorting.world').getroot().find('world')

def numbers(text):return [float(v) for v in text.split()]

def test_exactly_three_movable_hanoi_rings():
    movable=[o for o in CONFIG['objects'] if not o['static']]
    assert {o['name'] for o in movable}=={'pickup_cube','pickup_sphere','pickup_cylinder','pickup_hex_prism','hanoi_ring_large','hanoi_ring_medium','hanoi_ring_small'}
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

def test_colored_pickup_set_and_printable_tray():
    by_name={o['name']:o for o in CONFIG['objects']}
    for name in ['pickup_cube','pickup_sphere','pickup_cylinder','pickup_hex_prism']:
        assert len(by_name[name]['color'])==4
        assert by_name[name]['color'][3]==1
    tray=by_name['placement_tray']
    assert tray['static'] and tray['shapes'][0]['size']==[.24,.070,.004]
    assert len(tray['shapes'])==5
    cad=B.parents[2]/'cad/development'
    assert (cad/'pickup_placement_tray.scad').is_file()
    assert (cad/'pickup_placement_tray.stl').stat().st_size>84

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
            assert model.findtext('link/kinematic') == 'true'
            assert float(model.findtext('link/inertial/mass'))>0
            assert all(float(model.findtext('link/inertial/inertia/'+k))>0 for k in ['ixx','iyy','izz'])
    assert WORLD.find("plugin[@filename='libgazebo_ros_state.so']") is not None

def test_separate_opencv_conveyor_world_and_launch():
    models={m.attrib['name']:m for m in CONVEYOR.findall('model')}
    assert {'conveyor','sorting_bins','overhead_camera'} <= set(models)
    assert {'conveyor_cube_red','conveyor_cube_green','conveyor_cube_blue'} <= set(models)
    assert all(models[name].findtext('link/kinematic') == 'true'
               for name in ['conveyor_cube_red','conveyor_cube_green','conveyor_cube_blue'])
    camera=models['overhead_camera'].find("link/sensor[@type='camera']")
    assert camera is not None
    assert camera.find("plugin[@filename='libgazebo_ros_camera.so']") is not None
    assert camera.findtext('plugin/camera_name') == 'camera'
    assert CONVEYOR.find("plugin[@filename='libgazebo_ros_state.so']") is not None
    assert (B/'launch/conveyor_demo.launch.py').is_file()
    vision=(B/'scripts/conveyor_vision.py').read_text()
    assert 'cv2.cvtColor' in vision and all(color in vision for color in ['red','green','blue'])
    sorter=(B/'scripts/conveyor_sort_demo.py').read_text()
    assert '/conveyor/vision/detection' in sorter and "default=0" in sorter

def test_simulation_grasp_coupling_is_launched_and_installed():
    launch=(B/'launch/sim.launch.py').read_text()
    cmake=(B/'CMakeLists.txt').read_text()
    source=(B/'scripts/simulation_grasp.py').read_text()
    assert "executable='simulation_grasp.py'" in launch
    assert "'-s','libgazebo_ros_factory.so'" in launch
    assert 'simulation_grasp.py' in cmake
    assert '/gazebo/set_model_state' in source
    assert 'left_finger_joint' in source and 'right_finger_joint' in source

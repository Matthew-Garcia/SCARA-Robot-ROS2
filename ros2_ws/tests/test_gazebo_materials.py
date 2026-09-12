"""Check materials after fixed-joint reduction, and preserve simulation structure."""
from pathlib import Path
import runpy
import shutil
import xml.etree.ElementTree as ET
import unittest

WS = Path(__file__).resolve().parents[1]
API = runpy.run_path(str(WS/'src/scara_bringup/launch/gazebo_materials.py'))


class GazeboMaterialTests(unittest.TestCase):
    def test_reduced_visual_colors_preserve_physics(self):
        urdf = '<robot><link name="part"><visual><geometry><mesh filename="file:///blue.stl"/></geometry><material><color rgba="0.02 0.28 0.82 1"/></material></visual></link></robot>'
        sdf = '''<sdf version="1.6"><model name="scara"><link name="base_link"><inertial><mass>1</mass></inertial><visual name="fixed_joint_lump__part_visual"><geometry><mesh><uri>file:///blue.stl</uri></mesh></geometry><material><script><name>Gazebo/White</name></script></material></visual><collision name="collision"><geometry><box><size>1 1 1</size></box></geometry></collision></link><joint name="wrist_joint" type="revolute"/><plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so"/></model></sdf>'''
        result = ET.fromstring(API['apply_materials'](sdf, urdf))
        material = result.find('.//visual/material')
        assert material.findtext('ambient') == '0.02 0.28 0.82 1'
        assert material.findtext('diffuse') == '0.02 0.28 0.82 1'
        assert material.find('script') is None
        original = ET.fromstring(sdf)
        for root in (original, result):
            for visual in root.findall('.//visual'):
                for item in visual.findall('material'):
                    visual.remove(item)
        assert ET.tostring(original) == ET.tostring(result)
        with self.assertRaisesRegex(RuntimeError, 'lost colored meshes'):
            API['apply_materials'](sdf.replace('blue.stl', 'missing.stl'), urdf)


    @unittest.skipIf(shutil.which('gz') is None, 'Gazebo Classic CLI required')
    def test_actual_gazebo_conversion(self):
        from test_description import urdf
        source = ET.tostring(urdf('gazebo'), encoding='unicode')
        root = ET.fromstring(API['build_gazebo_sdf'](source))
        assert root.find('.//plugin[@filename="libgazebo_ros2_control.so"]') is not None
        joints = {j.attrib['name'] for j in root.findall('.//joint')}
        assert {'shoulder_joint', 'z_joint', 'elbow_joint', 'wrist_joint', 'left_finger_joint', 'right_finger_joint'} <= joints
        colors = {v.findtext('material/diffuse') for v in root.findall('.//link/visual')}
        assert {'0.02 0.28 0.82 1', '0.75 0.78 0.82 1', '0.02 0.02 0.02 1', '1 1 1 1'} <= colors

if __name__ == "__main__":
    unittest.main()

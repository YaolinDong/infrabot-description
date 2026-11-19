# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 YaolinDong
import copy
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from model import ROOT, load, forward_kinematics, physical_model, resolve_mesh
from validate_model import validate, inspect_structure, check_inertia, inspect_stl
from export_urdf import export


class ModelTests(unittest.TestCase):
    def test_complete_published_model(self):
        result = validate()
        self.assertEqual((result['links'], result['joints'], result['revolute_joints'], result['mesh_files']), (9, 8, 7, 8))
        self.assertAlmostEqual(result['total_mass_kg'], 4.627173355887553)

    def test_forward_kinematics_order_and_right_hand_rotation(self):
        root = ET.fromstring('''<robot name="test">
          <link name="base"/><link name="arm"/><link name="tip"/>
          <joint name="q" type="revolute"><parent link="base"/><child link="arm"/>
            <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
            <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/></joint>
          <joint name="tool" type="fixed"><parent link="arm"/><child link="tip"/>
            <origin xyz="1 0 0"/></joint></robot>''')
        pose = forward_kinematics(root, {'q': math.pi/2})['tip']
        for actual, expected in zip([pose[i][3] for i in range(3)], [1., 1., 0.]):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(pose[0][1], -1.)
        self.assertAlmostEqual(pose[1][0], 1.)

    def test_model_and_sample_poses_share_the_same_joint_order(self):
        config = json.loads((ROOT/'config/model.json').read_text())
        actual = [j.get('name') for j in load().findall('joint') if j.get('type') == 'revolute']
        self.assertEqual(config['joint_order'], actual)
        for positions in config['display_poses'].values():
            self.assertEqual(len(positions), len(actual))
            self.assertIn('Link_flange', forward_kinematics(load(), dict(zip(actual, positions))))

    def test_invalid_joint_reference_and_duplicate_name_are_rejected(self):
        root = load()
        root.find('joint/child').set('link', 'absent_link')
        with self.assertRaises(ValueError):
            inspect_structure(root)
        root = load()
        root.append(copy.deepcopy(root.find('link')))
        with self.assertRaises(ValueError):
            inspect_structure(root)

    def test_unphysical_inertia_is_rejected_even_when_positive_definite(self):
        inertia = {'mass': 1., 'inertia': {'ixx': 1., 'iyy': 1., 'izz': 3., 'ixy': 0., 'ixz': 0., 'iyz': 0.}}
        with self.assertRaisesRegex(ValueError, 'triangle'):
            check_inertia(inertia, 'test_link')

    def test_unknown_and_out_of_range_joint_positions_are_rejected(self):
        for positions in ({'Joint_2': -1.}, {'misspelled_joint': 0.}, {'Joint_1': float('nan')}):
            with self.assertRaises(ValueError):
                forward_kinematics(load(), positions)

    def test_mesh_resolution_rejects_external_and_missing_resources(self):
        for uri in ('file:///etc/passwd', 'package://another_robot/mesh.stl',
                    'package://infrabot_description/../outside.stl',
                    'package://infrabot_description/meshes/missing.STL'):
            with self.assertRaises(ValueError):
                resolve_mesh(uri)

    def test_filesystem_exports_preserve_physics_and_resolve_every_mesh(self):
        with tempfile.TemporaryDirectory() as tmp:
            for mode in ('absolute', 'relative'):
                out = export(Path(tmp)/(mode+'.urdf'), mode)
                root = ET.parse(out).getroot()
                self.assertEqual(physical_model(root), physical_model(load()))
                for mesh in root.findall('.//mesh'):
                    path = Path(mesh.get('filename'))
                    self.assertEqual(path.is_absolute(), mode == 'absolute')
                    self.assertTrue((out.parent/path).is_file())

    def test_truncated_stl_cannot_pass_as_a_model_asset(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'broken.STL'
            path.write_bytes(bytes(80)+struct.pack('<I', 2)+bytes(50))
            with self.assertRaises(ValueError):
                inspect_stl(path)


if __name__ == '__main__':
    unittest.main()

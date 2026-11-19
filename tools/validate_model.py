#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 YaolinDong
"""Validate source parity, URDF structure, inertia and complete mesh resources."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET
import yaml
from urdf_parser_py.urdf import URDF as ParsedURDF
from model import ROOT, URDF, forward_kinematics, numbers, physical_model, resolve_mesh


def determinant(a):
    return (a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1])
            - a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])
            + a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]))


def check_inertia(inertial, name):
    if not math.isfinite(inertial['mass']) or inertial['mass'] <= 0:
        raise ValueError(f'Non-positive mass: {name}')
    d = inertial['inertia']
    a = [[d['ixx'], d['ixy'], d['ixz']], [d['ixy'], d['iyy'], d['iyz']], [d['ixz'], d['iyz'], d['izz']]]
    if not all(math.isfinite(x) for row in a for x in row):
        raise ValueError(f'Non-finite inertia: {name}')
    if a[0][0] <= 0 or a[0][0]*a[1][1]-a[0][1]**2 <= 0 or determinant(a) <= 0:
        raise ValueError(f'Inertia is not positive definite: {name}')
    # Physical principal moments satisfy triangle inequalities. Equivalent to
    # positive semidefiniteness of the second-moment matrix, checked by minors.
    tr = sum(a[i][i] for i in range(3))/2
    b = [[(tr if i == j else 0)-a[i][j] for j in range(3)] for i in range(3)]
    scale = max(abs(x) for row in a for x in row)
    minors = [(b[i][i], scale) for i in range(3)]
    minors += [(b[i][i]*b[j][j]-b[i][j]**2, scale**2) for i, j in [(0, 1), (0, 2), (1, 2)]]
    minors.append((determinant(b), scale**3))
    if any(value < -1e-9*unit for value, unit in minors):
        raise ValueError(f'Inertia violates physical triangle inequalities: {name}')


def inspect_structure(root):
    for kind in ('link', 'joint'):
        names = [n.get('name') for n in root.findall(kind)]
        if any(not n for n in names) or len(names) != len(set(names)):
            raise ValueError(f'Missing or duplicate {kind} names')
    model = physical_model(root)
    for name, link in model['links'].items():
        if 'inertial' in link:
            check_inertia(link['inertial'], name)
        elif root.find(f"link[@name='{name}']/visual") is not None:
            raise ValueError(f'Geometry without inertial data: {name}')
    for name, joint in model['joints'].items():
        if joint['parent'] not in model['links'] or joint['child'] not in model['links']:
            raise ValueError(f'Joint references an absent link: {name}')
        if joint['type'] == 'fixed':
            continue
        if joint['type'] != 'revolute':
            raise ValueError(f'Unexpected H7C joint type: {name}')
        if not math.isclose(sum(x*x for x in joint['axis']), 1., abs_tol=1e-8):
            raise ValueError(f'Joint axis is not normalized: {name}')
        limits = joint['limit']
        if not all(math.isfinite(x) for x in limits.values()) or limits['lower'] >= limits['upper'] or limits['effort'] <= 0 or limits['velocity'] <= 0:
            raise ValueError(f'Invalid declared limits: {name}')
        if any(not math.isfinite(x) or x < 0 for x in joint.get('dynamics', {}).values()):
            raise ValueError(f'Invalid damping/friction: {name}')
    forward_kinematics(root)
    return model


def inspect_stl(path):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f'Truncated binary STL: {path.name}')
    triangles, = struct.unpack_from('<I', data, 80)
    if triangles == 0 or len(data) != 84+50*triangles:
        raise ValueError(f'Invalid binary STL length/count: {path.name}')
    lo, hi = [math.inf]*3, [-math.inf]*3
    for row in struct.iter_unpack('<12fH', memoryview(data)[84:]):
        if not all(math.isfinite(x) for x in row[:12]):
            raise ValueError(f'Non-finite mesh data: {path.name}')
        for start in (3, 6, 9):
            for axis in range(3):
                lo[axis] = min(lo[axis], row[start+axis])
                hi[axis] = max(hi[axis], row[start+axis])
    if any(a == b for a, b in zip(lo, hi)):
        raise ValueError(f'Degenerate mesh bounds: {path.name}')
    return {'triangles': triangles, 'bounds_m': [lo, hi], 'bytes': len(data)}


def validate(repo=ROOT, scan_meshes=True):
    repo = Path(repo).resolve()
    root = ET.parse(repo/'urdf/infrabot_h7c.urdf').getroot()
    model = inspect_structure(root)
    manifest = json.loads((repo/'model_manifest.json').read_text())
    if model != manifest['source_physics']:
        raise ValueError('Kinematics/dynamics differ from the supplied source model')
    for rel, wanted in manifest['published_sha256'].items():
        path = (repo/rel).resolve()
        if not path.is_relative_to(repo) or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != wanted:
            raise ValueError('Missing/changed model asset: '+rel)
    meshes = sorted({resolve_mesh(m.get('filename'), repo) for m in root.findall('.//mesh')})
    for m in root.findall('.//mesh'):
        if numbers(m.get('scale', '1 1 1'), 3) != (1., 1., 1.):
            raise ValueError('Unexpected mesh scaling')
    actual_meshes = set((repo/'meshes').rglob('*.STL'))
    if set(meshes) != actual_meshes:
        raise ValueError('Unused or missing mesh asset')
    friction = yaml.safe_load((repo/'config/h7c_friction.yaml').read_text())
    if friction != manifest['source_friction']:
        raise ValueError('Friction coefficients differ from the source')
    movable = {n: j for n, j in model['joints'].items() if j['type'] == 'revolute'}
    if set(friction['joints']) != set(movable):
        raise ValueError('Friction joint names do not match the model')
    for name, joint in movable.items():
        f = friction['joints'][name]
        if f['viscous'] != joint['dynamics']['damping'] or f['coulomb'] != joint['dynamics']['friction']:
            raise ValueError('URDF/YAML damping or friction mismatch: '+name)
    parser = ParsedURDF.from_xml_string(ET.tostring(root, encoding='unicode'))
    if parser.get_root() != 'base_link' or parser.get_chain('base_link', 'Link_flange', joints=True, links=False) != list(model['joints']):
        raise ValueError('Independent URDF parser disagrees with the declared chain')
    mesh_stats = {p.relative_to(repo).as_posix(): inspect_stl(p) for p in meshes} if scan_meshes else {}
    total_mass = sum(l['inertial']['mass'] for l in model['links'].values() if 'inertial' in l)
    return {'passed': True, 'model': 'H7C', 'links': len(model['links']), 'joints': len(model['joints']),
            'revolute_joints': len(movable), 'mesh_files': len(meshes), 'total_mass_kg': total_mass,
            'source_physics_preserved': True, 'mesh_stats': mesh_stats,
            'limitations': ['effort=100 and velocity=100 are inherited source values, not validated actuator ratings',
                            'collision geometry uses the full visual meshes; collision behavior is not validated',
                            'identified inertias and friction are supplied data, not independently re-identified here']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report', type=Path)
    args = p.parse_args()
    try:
        result = validate()
    except (ValueError, KeyError, OSError, ET.ParseError) as exc:
        result = {'passed': False, 'error': str(exc)}
    text = json.dumps(result, indent=2)+'\n'
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(text)
    print(text, end='')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())

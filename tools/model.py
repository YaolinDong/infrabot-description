# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 YaolinDong
"""Small URDF reading and kinematics helpers; no ROS runtime required."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'urdf/infrabot_h7c.urdf'
PACKAGE = 'infrabot_description'


def numbers(value, length):
    result = tuple(float(v) for v in value.split())
    if len(result) != length or not all(math.isfinite(v) for v in result):
        raise ValueError(f'Expected {length} finite numbers: {value!r}')
    return result


def origin(node):
    value = node.find('origin')
    if value is None:
        return {'xyz': [0., 0., 0.], 'rpy': [0., 0., 0.]}
    return {k: list(numbers(value.get(k, '0 0 0'), 3)) for k in ('xyz', 'rpy')}


def physical_model(root):
    """Exclude only mesh URIs, materials and comments from source comparison."""
    links, joints = {}, {}
    for link in root.findall('link'):
        inertial = link.find('inertial')
        item = {'visual_origins': [origin(v) for v in link.findall('visual')],
                'collision_origins': [origin(v) for v in link.findall('collision')]}
        if inertial is not None:
            item['inertial'] = {
                **origin(inertial),
                'mass': float(inertial.find('mass').get('value')),
                'inertia': {k: float(v) for k, v in inertial.find('inertia').attrib.items()},
            }
        links[link.get('name')] = item
    for joint in root.findall('joint'):
        item = {'type': joint.get('type'), **origin(joint),
                'parent': joint.find('parent').get('link'),
                'child': joint.find('child').get('link')}
        for key in ('limit', 'dynamics'):
            node = joint.find(key)
            if node is not None:
                item[key] = {k: float(v) for k, v in node.attrib.items()}
        if item['type'] != 'fixed':
            item['axis'] = list(numbers(joint.find('axis').get('xyz'), 3))
        joints[joint.get('name')] = item
    return {'robot_name': root.get('name'), 'links': links, 'joints': joints}


def resolve_mesh(uri, repo=ROOT):
    prefix = f'package://{PACKAGE}/'
    if not uri.startswith(prefix):
        raise ValueError(f'Expected a {PACKAGE} package URI: {uri}')
    base = Path(repo).resolve()
    path = (base / uri[len(prefix):]).resolve()
    if not path.is_relative_to(base) or not path.is_file():
        raise ValueError(f'Mesh is missing or escapes the package: {uri}')
    return path


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def transform(xyz=(0., 0., 0.), rpy=(0., 0., 0.)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, xyz[0]],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, xyz[1]],
            [-sp, cp*sr, cp*cr, xyz[2]], [0., 0., 0., 1.]]


def axis_rotation(axis, angle):
    norm = math.sqrt(sum(v*v for v in axis))
    if norm == 0:
        raise ValueError('A moving joint must have a nonzero axis')
    x, y, z = (v/norm for v in axis)
    c, s, v = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    return [[x*x*v+c, x*y*v-z*s, x*z*v+y*s, 0.],
            [y*x*v+z*s, y*y*v+c, y*z*v-x*s, 0.],
            [z*x*v-y*s, z*y*v+x*s, z*z*v+c, 0.], [0., 0., 0., 1.]]


def forward_kinematics(root, positions=None):
    model = physical_model(root)
    positions = positions or {}
    movable = {n for n, j in model['joints'].items() if j['type'] != 'fixed'}
    if set(positions) - movable:
        raise ValueError('Unknown movable joint: ' + ', '.join(sorted(set(positions)-movable)))
    roots = set(model['links']) - {j['child'] for j in model['joints'].values()}
    if len(roots) != 1:
        raise ValueError('Expected exactly one root link')
    result = {next(iter(roots)): identity()}
    pending = dict(model['joints'])
    while pending:
        progressed = False
        for name, joint in list(pending.items()):
            if joint['parent'] not in result:
                continue
            pose = transform(joint['xyz'], joint['rpy'])
            if joint['type'] == 'revolute':
                q = float(positions.get(name, 0.))
                if not math.isfinite(q) or not joint['limit']['lower'] <= q <= joint['limit']['upper']:
                    raise ValueError(f'Joint position outside declared limits: {name}={q}')
                pose = multiply(pose, axis_rotation(joint['axis'], q))
            elif joint['type'] != 'fixed':
                raise ValueError('This H7C helper supports fixed/revolute joints only')
            if joint['child'] in result:
                raise ValueError('Cycle or multiple parents in model')
            result[joint['child']] = multiply(result[joint['parent']], pose)
            del pending[name]
            progressed = True
        if not progressed:
            raise ValueError('Disconnected or cyclic joint graph')
    if set(result) != set(model['links']):
        raise ValueError('Disconnected links')
    return result


def load(path=URDF):
    return ET.parse(path).getroot()

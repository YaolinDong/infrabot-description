#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 YaolinDong
"""Render the actual URDF mesh assembly; requires an EGL-capable OpenGL driver."""
import argparse
import json
import math
import os
from pathlib import Path
os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
import numpy as np
from PIL import Image
import pyrender
import trimesh
from model import ROOT, load, origin, transform, forward_kinematics, resolve_mesh


def camera_pose(eye, target):
    z = eye-target
    z /= np.linalg.norm(z)
    x = np.cross([0., 0., 1.], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack([x, y, z])
    pose[:3, 3] = eye
    return pose


def render(pose_name, output, width=1500, height=1200):
    config = json.loads((ROOT/'config/model.json').read_text())
    root = load()
    positions = dict(zip(config['joint_order'], config['display_poses'][pose_name]))
    frames = forward_kinematics(root, positions)
    scene = pyrender.Scene(bg_color=[1., 1., 1., 1.], ambient_light=[0.22, 0.22, 0.22])
    material = pyrender.MetallicRoughnessMaterial(baseColorFactor=[0.45, 0.49, 0.53, 1.], metallicFactor=0.12, roughnessFactor=0.65)
    bounds = []
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            source = resolve_mesh(visual.find('geometry/mesh').get('filename'))
            mesh = trimesh.load_mesh(source, process=False)
            v = origin(visual)
            pose = np.asarray(frames[link.get('name')]) @ np.asarray(transform(v['xyz'], v['rpy']))
            world = trimesh.transform_points(mesh.vertices, pose)
            bounds.extend([world.min(axis=0), world.max(axis=0)])
            scene.add(pyrender.Mesh.from_trimesh(mesh, material=material, smooth=False), pose=pose)
    bounds = np.asarray(bounds)
    low, high = bounds.min(axis=0), bounds.max(axis=0)
    target = (low+high)/2
    radius = np.linalg.norm(high-low)/2
    distance = radius/math.sin(math.pi/8)*1.12
    direction = np.array([1.2, -1.6, 0.85]);direction /= np.linalg.norm(direction)
    cam = camera_pose(target+direction*distance, target)
    scene.add(pyrender.PerspectiveCamera(yfov=math.pi/4, znear=0.001, zfar=10.), pose=cam)
    for direction, intensity in [([1., -1., 2.], 1.8), ([-1., -0.4, 0.8], 0.9), ([0., 1., 2.], 1.1)]:
        light_pose = camera_pose(target+np.asarray(direction), target)
        scene.add(pyrender.DirectionalLight(color=np.ones(3), intensity=intensity), pose=light_pose)
    renderer = pyrender.OffscreenRenderer(width, height)
    try:
        color, depth = renderer.render(scene)
        if not np.any(depth > 0):
            raise RuntimeError('Renderer returned an empty view')
        output = Path(output);output.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(color).save(output)
    finally:
        renderer.delete()
    print(json.dumps({'image': str(output), 'pose': pose_name, 'joint_positions_rad': positions,
                      'world_bounds_m': [low.tolist(), high.tolist()]}, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pose', choices=['zero', 'bent'], default='zero')
    p.add_argument('--output', type=Path, default=ROOT/'.build/h7c.png')
    args = p.parse_args()
    render(args.pose, args.output)

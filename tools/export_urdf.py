#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 YaolinDong
"""Resolve package mesh URIs for tools that load ordinary filesystem URDFs."""
import argparse
import os
from pathlib import Path
import xml.etree.ElementTree as ET
from model import ROOT, URDF, load, resolve_mesh


def export(output, mode='absolute'):
    output = Path(output).resolve()
    if output == URDF.resolve():
        raise ValueError('Export to a separate file, not the canonical URDF')
    if mode not in ('absolute', 'relative'):
        raise ValueError('Unknown mesh path mode')
    root = load()
    for mesh in root.findall('.//mesh'):
        path = resolve_mesh(mesh.get('filename'))
        mesh.set('filename', str(path) if mode == 'absolute' else os.path.relpath(path, output.parent))
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(output, encoding='utf-8', xml_declaration=True)
    return output


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, default=ROOT/'.build/infrabot_h7c.urdf')
    p.add_argument('--mesh-paths', choices=['absolute', 'relative'], default='absolute')
    args = p.parse_args()
    print(export(args.output, args.mesh_paths))

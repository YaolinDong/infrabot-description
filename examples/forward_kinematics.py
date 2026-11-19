#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 YaolinDong
"""Inspect a model pose in metres/radians. No robot connection or commands."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from model import load, forward_kinematics

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--positions', type=float, nargs=7, default=[0.]*7,
                   metavar=('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7'))
    args = p.parse_args()
    poses = forward_kinematics(load(), {f'Joint_{i+1}': q for i, q in enumerate(args.positions)})
    print(json.dumps({'reference_frame': 'base_link', 'length_unit': 'm',
                      'angle_unit': 'rad', 'transforms': poses}, indent=2))

# Copyright 2026 Lucy Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""CLI entry point for lucy_config_generator."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from lucy_config_generator.generate import generate


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Generate firmware C, ros2_control xacro, and controllers YAML from hardware YAML.'
        ),
    )
    parser.add_argument(
        '--input',
        type=Path,
        required=True,
        help='Path to hardware mapping YAML (e.g. urdf/config/hardware/active.yaml).',
    )
    parser.add_argument(
        '--urdf',
        type=Path,
        required=True,
        help='Top-level robot xacro (e.g. urdf/description/urdf/robot.urdf.xacro).',
    )
    parser.add_argument(
        '--base-path',
        type=Path,
        required=True,
        help='xacro base_path (installed description/ or source description/).',
    )
    parser.add_argument(
        '--controller-config',
        type=Path,
        required=True,
        help='controllers.yaml passed into xacro (for extra_joints / URDF processing).',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        required=True,
        help='Directory to write generated files.',
    )
    parser.add_argument(
        '--targets',
        default='all',
        choices=('all', 'firmware', 'ros2_control', 'controllers'),
        help='Which outputs to generate (default: all).',
    )
    parser.add_argument(
        '--boards',
        default='',
        help='Comma-separated board ids to include (default: all known boards in YAML).',
    )
    args = parser.parse_args(argv)

    if args.targets == 'all':
        targets = {'firmware', 'ros2_control', 'controllers', 'gazebo'}
    else:
        targets = {args.targets}
    boards_filter: set[str] | None = None
    if args.boards.strip():
        boards_filter = {b.strip() for b in args.boards.split(',') if b.strip()}

    try:
        generate(
            input_yaml=args.input,
            urdf_xacro=args.urdf,
            base_path=args.base_path,
            controller_config=args.controller_config,
            output_dir=args.output_dir,
            targets=targets,
            boards_filter=boards_filter,
        )
    except (OSError, ValueError, RuntimeError, FileNotFoundError) as e:
        print(f'error: {e}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

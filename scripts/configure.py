#!/usr/bin/env python3
"""Resolve local asset paths; never change registration or fusion algorithms."""
import argparse
import json
import shlex
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--isaac-root', type=Path, required=True,
                        help='Directory containing isaac-sim.sh and python.sh')
    parser.add_argument('--scene', type=Path, required=True,
                        help='Original collected scene USD; retain its asset directory')
    parser.add_argument('--map', type=Path, required=True, dest='pointcloud')
    parser.add_argument('--occupancy', type=Path, default=ROOT/'assets/maps/room.yaml')
    parser.add_argument('--ros-overlay', type=Path, default=ROOT/'ros_ws/install/setup.bash')
    args = parser.parse_args()
    isaac, scene, cloud, occupancy, overlay = [p.expanduser().resolve() for p in
        (args.isaac_root, args.scene, args.pointcloud, args.occupancy, args.ros_overlay)]
    for path in (isaac/'isaac-sim.sh', isaac/'python.sh', scene, cloud, occupancy):
        if not path.is_file():
            parser.error(f'Missing required file: {path}')
    if any(c in str(scene) for c in ('@', '\n', '\r')):
        parser.error('Scene path cannot contain @ or a newline (USD asset syntax)')
    grid = yaml.safe_load(occupancy.read_text())
    image = Path(grid['image']).expanduser()
    if not image.is_absolute():
        image = occupancy.parent/image
    if not image.is_file():
        parser.error(f'Occupancy image missing: {image}')
    (ROOT/'logs').mkdir(exist_ok=True)
    replacements = {'map_path': str(cloud), 'occupancy_yaml': str(occupancy),
                    'event_log_csv': str(ROOT/'logs/recovery-events.csv')}
    for filename in ('localization', 'global-search', 'global-startup', 'global-recovery'):
        document = yaml.safe_load((ROOT/'config'/f'{filename}.yaml').read_text())
        parameters = document['/**']['ros__parameters']
        if filename in ('localization', 'global-search', 'global-startup'):
            parameters['map_path'] = replacements['map_path']
        if filename == 'global-search':
            parameters['occupancy_yaml'] = replacements['occupancy_yaml']
        if filename == 'global-recovery':
            parameters['event_log_csv'] = replacements['event_log_csv']
        (ROOT/'runtime'/f'{filename}.local.yaml').write_text(
            yaml.safe_dump(document, sort_keys=False, allow_unicode=True))
    template = (ROOT/'config/localization_scene.template.usda').read_text()
    assert template.count('__BASE_SCENE__') == 1
    scene_file = ROOT/'runtime/localization_scene.local.usda'
    scene_file.write_text(template.replace('__BASE_SCENE__', str(scene)))
    variables = {'ISAAC_SIM_ROOT': str(isaac), 'PROJECT_SCENE_FILE': str(scene_file),
                 'ROS_OVERLAY_SETUP': str(overlay)}
    (ROOT/'.env.local').write_text('# Generated locally; do not commit.\n' + ''.join(
        f'export {key}={shlex.quote(value)}\n' for key, value in variables.items()))
    print(json.dumps({'configured': True, 'scene': str(scene), 'map': str(cloud),
                      'occupancy': str(occupancy), 'ros_overlay_exists': overlay.is_file()},
                     ensure_ascii=False, indent=2))
    if not overlay.is_file():
        print('ROS overlay not built yet. Follow the README build steps before localization.')


if __name__ == '__main__':
    main()

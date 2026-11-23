#!/usr/bin/env python3
"""Compare separately distributed resources against the recorded content hashes."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-dir', type=Path,
                        default=ROOT/'assets/scene/Collected_final_wheeltec_with_scene')
    parser.add_argument('--map', type=Path, default=ROOT/'assets/maps/room.pcd')
    args = parser.parse_args()
    manifest = json.loads((ROOT/'assets/manifest.json').read_text())
    failures = []
    for entry in manifest['files']:
        relative = entry['path']
        if relative == 'assets/maps/room.pcd':
            path = args.map.expanduser()
        elif relative.startswith('assets/scene/Collected_final_wheeltec_with_scene/'):
            path = args.scene_dir.expanduser()/relative.split('/', 3)[3]
        else:
            path = ROOT/relative
        if not path.is_file():
            failures.append(f'Missing: {relative}')
            continue
        if path.stat().st_size != entry['bytes']:
            failures.append(f'Size mismatch: {relative}')
            continue
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(4*1024*1024), b''):
                digest.update(block)
        if digest.hexdigest() != entry['sha256']:
            failures.append(f'SHA-256 mismatch: {relative}')
    if failures:
        parser.exit(1, '\n'.join(failures)+'\n')
    print(f"Verified {len(manifest['files'])} asset files.")


if __name__ == '__main__':
    main()

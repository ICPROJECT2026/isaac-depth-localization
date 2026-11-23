#!/usr/bin/env python3
"""Static release checks only; no ROS process, GPU access, or robot motion."""
import ast
import json
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import unquote

import yaml

ROOT = Path(__file__).resolve().parents[1]


def source_files():
    if (ROOT/'.git').exists():
        result = subprocess.run(['git', 'ls-files', '-z', '--cached', '--others',
                                 '--exclude-standard'], cwd=ROOT, check=True,
                                stdout=subprocess.PIPE)
        return sorted({ROOT/name for name in result.stdout.decode().split('\0') if name})
    result = []
    for p in ROOT.rglob('*'):
        rel = p.relative_to(ROOT)
        if not p.is_file() or set(rel.parts) & {'.git', 'ros_ws', '__pycache__', '.venv'}:
            continue
        if rel.parts[0] in {'data', 'logs'} and p.name != '.gitkeep':
            continue
        if rel.parts[:2] == ('assets', 'scene') and p.name != 'README.md':
            continue
        if p.name == '.env.local' or '.local.' in p.name:
            continue
        if p.suffix in {'.pcd', '.ply', '.log', '.db3', '.mcap', '.zip', '.gz'}:
            continue
        result.append(p)
    return sorted(result)


def main():
    errors = []
    counts = {'python': 0, 'shell': 0, 'yaml': 0, 'markdown': 0}
    files = source_files()
    for p in files:
        rel = p.relative_to(ROOT)
        try:
            if p.stat().st_size >= 50*1024*1024:
                raise ValueError('File exceeds repository preparation limit (50 MiB)')
            if p.suffix == '.py':
                ast.parse(p.read_text(), filename=str(rel)); counts['python'] += 1
            if p.suffix == '.sh':
                subprocess.run(['bash', '-n', str(p)], check=True, capture_output=True)
                counts['shell'] += 1
            if p.suffix in {'.yaml', '.yml', '.repos'}:
                yaml.safe_load(p.read_text()); counts['yaml'] += 1
            if p.suffix == '.json':
                json.loads(p.read_text())
            if p.suffix == '.urdf':
                ET.parse(p)
            if p.suffix in {'.py', '.sh', '.yaml', '.yml', '.repos', '.json', '.md', '.usda', '.rviz', '.urdf'}:
                text = p.read_text()
                # Avoid committing paths copied from a developer workstation.
                if re.search(r'/home/[A-Za-z0-9_.-]+/', text):
                    raise ValueError('Developer-specific /home path')
                if re.search(r'gh[pousr]_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----', text):
                    raise ValueError('Possible credential/private key')
            if p.suffix == '.md':
                counts['markdown'] += 1
                for link in re.findall(r'\]\(([^\s)]+)\)', p.read_text()):
                    if link.startswith(('#', 'https://', 'http://', 'mailto:')):
                        continue
                    target = p.parent/unquote(link.split('#')[0])
                    if not target.exists():
                        raise ValueError(f'Broken local link: {link}')
        except Exception as error:
            errors.append(f'{rel}: {error}')
    dependencies = yaml.safe_load((ROOT/'dependencies.repos').read_text())['repositories']
    versions = json.loads((ROOT/'upstream-versions.json').read_text())
    for name, spec in dependencies.items():
        if spec['version'] != versions[name]['commit'] or spec['url'] != versions[name]['remote']:
            errors.append(f'Dependency lock mismatch: {name}')
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 1
    print(json.dumps({'passed': True, 'files_checked': len(files), **counts}, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())

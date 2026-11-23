#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
command -v vcs >/dev/null || { echo 'Install python3-vcstool first.' >&2; exit 1; }
mkdir -p "$project_root/ros_ws/src"
vcs import --skip-existing "$project_root/ros_ws/src" < "$project_root/dependencies.repos"
echo 'Pinned sources imported. Existing checkouts were not reset; check their revisions if reusing a workspace.'

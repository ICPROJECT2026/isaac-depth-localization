#!/usr/bin/env bash
# Use Isaac's own USD libraries for scene export/cropping tools.
set -e
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$project_root/.env.local"
usd_libraries=("$ISAAC_SIM_ROOT"/extscache/omni.usd.libs-*)
if [[ ! -d "${usd_libraries[0]}" ]]; then
  echo 'Isaac USD library directory not found; this helper targets the recorded source build.' >&2
  exit 1
fi
unset PYTHONHOME AMENT_PREFIX_PATH
export PYTHONPATH="${usd_libraries[0]}"
export LD_LIBRARY_PATH="${usd_libraries[0]}/bin"
exec "$ISAAC_SIM_ROOT/python.sh" --no-ros-env "$@"

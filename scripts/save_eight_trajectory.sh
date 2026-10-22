#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
setup_file="${workspace_dir}/devel/setup.bash"
if [[ ! -f "${setup_file}" ]]; then
  echo "ROS workspace is not built: ${setup_file}" >&2
  exit 1
fi

source "${setup_file}"
exec python3 "${workspace_dir}/scripts/save_eight_trajectory.py" "$@"

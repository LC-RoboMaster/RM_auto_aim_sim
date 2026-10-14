#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$project_dir/build/standalone/rm_auto_aim_sim" \
  --config "$project_dir/src/rm_auto_aim_sim/config/sim.yaml" "$@"

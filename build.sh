#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mujoco_python="${CONDA_PREFIX:+${CONDA_PREFIX}/bin/python}"
mujoco_python="${mujoco_python:-$(command -v python3)}"
cd "$project_dir"
if [[ "${1:-}" == "--standalone" ]]; then
  cmake -S src/rm_auto_aim_sim -B build/standalone -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_ROS2=OFF -DCONDA_PYTHON="$mujoco_python"
  cmake --build build/standalone -j "${BUILD_JOBS:-6}"
  ctest --test-dir build/standalone --output-on-failure
else
  if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
    echo 'Source /opt/ros/humble/setup.bash before building the ROS2 version.' >&2
    exit 1
  fi
  /usr/bin/python3 -m colcon build --symlink-install --packages-select rm_auto_aim_sim \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCONDA_PYTHON="$mujoco_python" \
      -DPython3_EXECUTABLE=/usr/bin/python3
  ctest --test-dir build/rm_auto_aim_sim --output-on-failure 
fi

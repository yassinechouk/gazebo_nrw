#!/usr/bin/env bash
# Build the Assemforchouk simulation workspace.
set -euo pipefail
cd "$(dirname "$0")"

source /opt/ros/jazzy/setup.bash

# Rebuild the per-link meshes from the SolidWorks STLs if they are missing.
if [ ! -f src/assemforchouk_sim/meshes/base_chain.stl ]; then
  echo ">> rebuilding meshes from ../meshes ..."
  python3 tools/build_meshes.py
fi

colcon build --symlink-install "$@"
echo
echo "Built.  Now run:"
echo "    source $(pwd)/install/setup.bash"
echo "    ros2 launch assemforchouk_sim gazebo.launch.py demo:=true"

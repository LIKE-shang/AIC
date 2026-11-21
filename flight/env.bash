#!/usr/bin/env bash
set -e
FULL_CHAIN_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /home/abot/window_pass_ground_env.bash
if [[ ! -f "$FULL_CHAIN_ROOT/vision_ws/devel/setup.bash" ]]; then
  echo "Full-chain vision has not been compiled. Run install.bash first." >&2
  return 2
fi
source "$FULL_CHAIN_ROOT/vision_ws/devel/setup.bash"
export ROS_PACKAGE_PATH="$FULL_CHAIN_ROOT/rgbd:$FULL_CHAIN_ROOT/vision_ws/src:/home/abot/window_pass_session_20261013${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
export PYTHONPATH="$FULL_CHAIN_ROOT/vendor:$FULL_CHAIN_ROOT/rgbd/src:$FULL_CHAIN_ROOT/vision_ws/src/full_chain_vision_20261025_v6_1/src${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1

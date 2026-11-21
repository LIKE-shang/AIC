#!/usr/bin/env bash
set -e
AUTO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /home/abot/window_pass_ground_env.bash
export ROS_PACKAGE_PATH="/home/abot/window_live_rgbd_20261020_v24_4:/home/abot/window_pass_session_20261013${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
WP_PKG="$(rospack find window_pass_test)"
export PYTHONPATH="/home/abot/window_live_rgbd_20261020_v24_4/src:$WP_PKG/src${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec python3 "$AUTO_ROOT/scripts/start_components.py" perception

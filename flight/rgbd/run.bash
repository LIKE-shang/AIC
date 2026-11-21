#!/usr/bin/env bash
set -euo pipefail
LIVE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/noetic/setup.bash
if [[ -f /home/abot/window_pass_ground_env.bash ]]; then
  source /home/abot/window_pass_ground_env.bash
fi
if [[ -f /home/abot/window_pass_test_ws/devel/setup.bash ]]; then
  source /home/abot/window_pass_test_ws/devel/setup.bash
fi
set -u
export ROS_PACKAGE_PATH="$LIVE_ROOT${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
export PYTHONPATH="$LIVE_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONUNBUFFERED=1
mkdir -p "$HOME/window_live_logs"
printf '%s\n' '实时窗框检测：使用当前图像，不需要 geometry；只发布建议，不输出飞控目标。'
exec roslaunch "$LIVE_ROOT/launch/live.launch" "$@"

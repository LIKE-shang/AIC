#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/noetic/setup.bash
[[ ! -f /home/abot/window_pass_ground_env.bash ]] || source /home/abot/window_pass_ground_env.bash
[[ ! -f /home/abot/window_pass_test_ws/devel/setup.bash ]] || source /home/abot/window_pass_test_ws/devel/setup.bash
set -u
export ROS_PACKAGE_PATH="$ROOT${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONUNBUFFERED=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
mkdir -p "$HOME/window_rgbd_logs"
if [[ "${1:-}" == base ]]; then
  shift
  if (( $# )); then
    printf '%s\n' 'base has no extra arguments; ground-only localization recovery.' >&2
    exit 2
  fi
  export ROS_PACKAGE_PATH="/home/abot/window_pass_session_20261013:$ROS_PACKAGE_PATH"
  exec python3 "$ROOT/scripts/recover_base.py"
fi
if [[ "${1:-}" == start || $# == 0 ]]; then
  [[ $# == 0 ]] || shift
  if (( $# )); then
    printf '%s\n' 'start uses the fixed current-stage profile; no extra arguments.' >&2
    exit 2
  fi
  exec roslaunch "$ROOT/launch/competition.launch"
fi
if [[ "${1:-}" == diag ]]; then
  shift
  exec python3 "$ROOT/tools/snapshot_rgbd.py" --verbose "$@"
fi
if [[ "${1:-}" == check ]]; then
  shift
  exec python3 "$ROOT/tools/snapshot_rgbd.py" "$@"
fi
printf '%s\n' 'RGB-D实时检测/几何/条件规划合并启动；持续录包默认关闭。只输出建议，不是A_READY。相机启动仅允许未解锁ON_GROUND，拒绝覆盖已有相机。'
exec roslaunch "$ROOT/launch/rgbd.launch" "$@"
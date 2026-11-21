#!/usr/bin/env bash
set -eo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /home/abot/window_pass_ground_env.bash
WP_PKG="$(rospack find window_pass_test)"
export PYTHONPATH="$WP_PKG/src${PYTHONPATH:+:$PYTHONPATH}"
exec python3 "$TASK_ROOT/scripts/auto_window.py" --require-experiment takeoff_land "$@"

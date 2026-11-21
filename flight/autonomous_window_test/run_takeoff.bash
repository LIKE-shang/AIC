#!/usr/bin/env bash
set -e
AUTO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /home/abot/window_pass_ground_env.bash
WP_PKG="$(rospack find window_pass_test)"
export PYTHONPATH="$WP_PKG/src${PYTHONPATH:+:$PYTHONPATH}"
exec python3 "$AUTO_ROOT/scripts/auto_window.py" --config "$AUTO_ROOT/config/takeoff_land.json" "$@" --require-experiment takeoff_land

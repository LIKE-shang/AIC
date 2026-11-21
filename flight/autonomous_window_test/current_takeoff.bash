#!/usr/bin/env bash
set -eo pipefail
AUTO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROFILE="$(cat /home/abot/autonomous_window_current_trial.path)"
test -f "$PROFILE"
bash "$AUTO_ROOT/run_takeoff.bash" --config "$PROFILE" --commissioning-takeoff "$@" 2>&1 | tee -a "$(dirname -- "$PROFILE")/mission.log"

#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 "$ROOT/verify_manifest.py"
source "$ROOT/env.bash"
exec python3 "$ROOT/autonomous_window_test/scripts/bind_full_chain_trial.py" "$@"

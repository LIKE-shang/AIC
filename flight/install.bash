#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 "$ROOT/verify_manifest.py"
source /home/abot/window_pass_ground_env.bash
cd "$ROOT/vision_ws"
catkin_make -DCMAKE_BUILD_TYPE=Release -j2 -l2 2>&1 | tee "$ROOT/board_build.log"
source "$ROOT/env.bash"
[[ "$(rospack find full_chain_vision_20261025_v6_1)" == "$ROOT/vision_ws/src/full_chain_vision_20261025_v6_1" ]]
[[ "$(rospack find full_chain_rgbd_20261025_v6_1)" == "$ROOT/rgbd" ]]
python3 - "$ROOT" <<'PY'
import ast,hashlib,json,platform,sys
from pathlib import Path
root=Path(sys.argv[1])
for folder in ['autonomous_window_test','rgbd/src','rgbd/scripts','vendor','vision_ws/src/full_chain_vision_20261025_v6_1/src','vision_ws/src/full_chain_vision_20261025_v6_1/scripts']:
    for p in (root/folder).rglob('*.py'):ast.parse(p.read_text(encoding='utf-8-sig'),filename=str(p),feature_version=(3,8))
binary=root/'vision_ws/devel/lib/full_chain_vision_20261025_v6_1/full_chain_target_laser_v6_1'
report={'python':sys.version,'machine':platform.machine(),'binary':str(binary),'sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'flight_executed':False}
(root/'BOARD_BUILD_VERIFIED.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
PY
printf '%s\n' 'Independent bundle compiled and sources resolved. This does not start nodes or prove flight/hit acceptance.'

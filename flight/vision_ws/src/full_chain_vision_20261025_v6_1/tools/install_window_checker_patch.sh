#!/usr/bin/env bash
# Run from the unpacked patch directory after sourcing the two ROS workspaces.
# This script installs ONLY three files and never starts/stops a ROS node.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
for cmd in python3 rospack rosnode sha256sum timeout; do
    command -v "$cmd" >/dev/null || { echo "Missing command: $cmd" >&2; exit 1; }
done
sha256sum -c SHA256SUMS.window_checker
PKG="$(readlink -f -- "$(rospack find robot_vision)")"
case "$PKG" in
    /home/abot/abot_ws/src/robot_vision|/home/abot/vision_camera_test_ws/src/robot_vision) ;;
    *) echo "Refusing unexpected active package path: $PKG" >&2; exit 1 ;;
esac
printf 'Active package: %s\n' "$PKG"
# Verify relay/overlay imports point at the same source tree BEFORE changing it.
python3 - "$PKG" <<'PY'
import ast, hashlib, pathlib, sys
import robot_vision_test.vision as vision
package = pathlib.Path(sys.argv[1])
expected = package/'src/robot_vision_test/vision.py'
if pathlib.Path(vision.__file__).resolve() != expected.resolve():
    raise SystemExit('Wrong Python overlay: '+str(vision.__file__))
baseline = {
    'src/robot_vision_test/vision.py': 'bdf87bc7747ffee6c23f53b44c425443c2e3e8c28ad855e90707f11a8bab040a',
    'scripts/window_test_node.py': '66ed8fd150480ecc20db92298276d933652312375273b6c6c43e1fa92f88e545',
    'config/window_test.yaml': '4b8cb5f8b9b21a459dfcb60af054e7ac03a034ac51f69a7cb8dca06c7fce410d',
}
for relative, original in baseline.items():
    target = package/relative
    if target.is_symlink() or target.resolve().parent != (package/relative).parent.resolve():
        raise SystemExit('Refusing symlink file: '+str(target))
    if package.resolve() not in target.resolve().parents:
        raise SystemExit('Target escapes package: '+str(target))
    source = pathlib.Path('robot_vision')/relative
    replacement = hashlib.sha256(source.read_bytes()).hexdigest()
    current = hashlib.sha256(target.read_bytes()).hexdigest()
    if current not in (original, replacement):
        raise SystemExit('Locally modified target, not overwritten: '+str(target))
    if relative.endswith('.py'):
        ast.parse(source.read_text(encoding='utf-8-sig'), filename=str(source))
print('PREFLIGHT_SOURCE_AND_OVERLAY_OK')
PY
# Fail closed if the master is unreachable. Never kill a node on the user's behalf.
NODES="$(timeout 15s rosnode list)" || { echo 'Cannot check ROS nodes; no files installed.' >&2; exit 1; }
if printf '%s\n' "$NODES" | grep -Fxq '/window_test'; then
    echo 'Stop ONLY the window_test detector launch first (Ctrl-C), then rerun.' >&2
    exit 1
fi
FILES=(src/robot_vision_test/vision.py scripts/window_test_node.py config/window_test.yaml)
BACKUP="$(mktemp -d /home/abot/window_checker_backup.XXXXXX)"
for rel in "${FILES[@]}"; do
    mkdir -p -- "$BACKUP/$(dirname -- "$rel")"
    cp -p -- "$PKG/$rel" "$BACKUP/$rel"
done
printf 'Backup: %s\n' "$BACKUP"
restore_on_error() {
    local code=$?
    trap - ERR
    echo 'Installation failed; attempting to restore all three original files.' >&2
    for rel in "${FILES[@]}"; do
        cp -p -- "$BACKUP/$rel" "$PKG/$rel" || echo "RESTORE FAILED: $rel" >&2
    done
    echo "Backup retained: $BACKUP" >&2
    exit "$code"
}
trap restore_on_error ERR
for rel in "${FILES[@]}"; do
    mode=644
    [[ "$rel" == scripts/window_test_node.py ]] && mode=755
    install -m "$mode" -- "$HERE/robot_vision/$rel" "$PKG/$rel"
done
python3 - "$PKG" "$HERE" <<'PY'
import hashlib, pathlib, sys
import numpy as np
import yaml
import robot_vision_test.vision as vision
package, patch = map(pathlib.Path, sys.argv[1:])
assert pathlib.Path(vision.__file__).resolve() == (package/'src/robot_vision_test/vision.py').resolve()
for relative in ('src/robot_vision_test/vision.py', 'scripts/window_test_node.py', 'config/window_test.yaml'):
    assert hashlib.sha256((package/relative).read_bytes()).digest() == hashlib.sha256((patch/'robot_vision'/relative).read_bytes()).digest(), relative
assert hasattr(vision, 'annotate_windows')
config = yaml.safe_load((package/'config/window_test.yaml').read_text())
assert config['checker_fallback'] and config['require_inner_frame']
targets, edges = vision.detect_windows(np.full((480, 640, 3), 235, np.uint8), config)
assert targets == [] and edges.dtype == np.uint8
print('WINDOW_CHECKER_INSTALL_OK (import + blank-image smoke test; NOT camera acceptance)')
PY
trap - ERR
printf 'Only three files installed. No node was started. Backup: %s\n' "$BACKUP"

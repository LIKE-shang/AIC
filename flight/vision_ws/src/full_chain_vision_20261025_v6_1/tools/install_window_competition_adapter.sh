#!/usr/bin/env bash
# Install a fixed file whitelist. Never builds or starts/stops a ROS node.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
for cmd in python3 rospack rosnode sha256sum timeout readlink install; do
    command -v "$cmd" >/dev/null || { echo "Missing command: $cmd" >&2; exit 1; }
done
sha256sum -c SHA256SUMS.window_competition_adapter
PKG="$(readlink -f -- "$(rospack find robot_vision)")"
case "$PKG" in
    /home/abot/abot_ws/src/robot_vision|/home/abot/vision_camera_test_ws/src/robot_vision) ;;
    *) echo "Refusing unexpected active package path: $PKG" >&2; exit 1 ;;
esac
printf 'Active package: %s\n' "$PKG"
python3 - "$PKG" "$HERE" <<'PY'
import ast
import hashlib
import importlib.util
import json
import pathlib
import sys
import xml.etree.ElementTree as ET

package, patch = map(pathlib.Path, sys.argv[1:])
manifest = json.loads((patch/'INSTALL_MANIFEST.json').read_text(encoding='utf-8'))
allowed = (
    'CMakeLists.txt', 'package.xml', 'scripts/window_test_node.py',
    'scripts/window_competition_adapter.py', 'src/robot_vision_test/competition_adapter.py',
    'config/window_competition_vision.yaml', 'config/window_competition_adapter.yaml',
    'launch/window_competition_adapter.launch', 'launch/window_competition_calibration.launch',
    'test/test_competition_adapter.py', 'test/test_node_callbacks.py', 'test/test_vision_tests.py',
    'docs/WINDOW_COMPETITION_ADAPTER_CN.md', 'docs/ROBOT_TEST_LOG.md',
    'docs/TEST_REPORT.md', 'docs/FILE_MANIFEST.md',
)
if [entry['path'] for entry in manifest['files']] != list(allowed):
    raise SystemExit('Unexpected installation whitelist')
spec = importlib.util.find_spec('robot_vision_test.vision')
if spec is None or pathlib.Path(spec.origin).resolve() != (package/'src/robot_vision_test/vision.py').resolve():
    raise SystemExit('Wrong Python overlay; source both ROS workspaces first')

def safe_target(relative):
    target = package/relative
    if target.is_symlink() or package.resolve() not in target.resolve().parents:
        raise SystemExit('Refusing symlink/escaping target: '+str(target))
    return target

for entry in manifest['prerequisites']:
    target = safe_target(entry['path'])
    if not target.is_file() or hashlib.sha256(target.read_bytes()).hexdigest() != entry['sha256']:
        raise SystemExit('Prerequisite mismatch; keep local changes and reconcile checker patch: '+str(target))
for entry in manifest['files']:
    relative = entry['path']
    source = patch/'robot_vision'/relative
    data = source.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if digest != entry['new_sha256']:
        raise SystemExit('Replacement checksum mismatch: '+relative)
    target = safe_target(relative)
    if target.exists():
        current = hashlib.sha256(target.read_bytes()).hexdigest()
        if current not in entry['allowed_previous_sha256'] + [digest]:
            raise SystemExit('Locally modified target, not overwritten: '+str(target))
    elif entry['allowed_previous_sha256']:
        raise SystemExit('Missing existing prerequisite file: '+str(target))
    if relative.endswith('.py'):
        ast.parse(data.decode('utf-8'), filename=relative)
    if relative.endswith(('.launch', '.xml')):
        ET.fromstring(data)
print('COMPETITION_ADAPTER_PREFLIGHT_OK')
PY
# Refuse hot replacement. Do not terminate a node on the operator's behalf.
NODES="$(timeout 15s rosnode list)" || { echo 'Cannot check ROS nodes; no files installed.' >&2; exit 1; }
while IFS= read -r node; do
    case "${node##*/}" in
        window_test|window_competition_adapter|uav_mission|agv_mission|complete_mission)
            echo "Stop detector/adapter/tasks first (Ctrl-C): $node" >&2
            exit 1 ;;
    esac
done <<< "$NODES"
FILES=(
    CMakeLists.txt package.xml scripts/window_test_node.py
    scripts/window_competition_adapter.py src/robot_vision_test/competition_adapter.py
    config/window_competition_vision.yaml config/window_competition_adapter.yaml
    launch/window_competition_adapter.launch launch/window_competition_calibration.launch
    test/test_competition_adapter.py test/test_node_callbacks.py test/test_vision_tests.py
    docs/WINDOW_COMPETITION_ADAPTER_CN.md docs/ROBOT_TEST_LOG.md
    docs/TEST_REPORT.md docs/FILE_MANIFEST.md
)
BACKUP="$(mktemp -d /home/abot/window_competition_backup.XXXXXX)"
for rel in "${FILES[@]}"; do
    mkdir -p -- "$BACKUP/$(dirname -- "$rel")"
    if [[ -f "$PKG/$rel" ]]; then
        cp -p -- "$PKG/$rel" "$BACKUP/$rel"
    else
        printf '%s\n' "$rel" >> "$BACKUP/new_files.txt"
    fi
done
printf 'Backup: %s\n' "$BACKUP"
restore_on_error() {
    local code="${1:-$?}"
    trap - ERR INT TERM
    echo 'Installation failed; attempting to restore the whitelisted files.' >&2
    for rel in "${FILES[@]}"; do
        if [[ -f "$BACKUP/$rel" ]]; then
            cp -p -- "$BACKUP/$rel" "$PKG/$rel" || echo "RESTORE FAILED: $rel" >&2
        else
            # rel is from the fixed whitelist, validated inside PKG above.
            rm -f -- "$PKG/$rel" || echo "CLEANUP FAILED: $rel" >&2
        fi
    done
    echo "Backup retained: $BACKUP" >&2
    exit "$code"
}
trap restore_on_error ERR
trap 'restore_on_error 130' INT
trap 'restore_on_error 143' TERM
for rel in "${FILES[@]}"; do
    mkdir -p -- "$PKG/$(dirname -- "$rel")"
    mode=644
    [[ "$rel" == scripts/*.py ]] && mode=755
    install -m "$mode" -- "$HERE/robot_vision/$rel" "$PKG/$rel"
done
python3 - "$PKG" "$HERE" <<'PY'
import hashlib
import json
import pathlib
import sys
import yaml
from geometry_msgs.msg import PointStamped
from robot_vision_test.competition_adapter import Confirmation, parse_window_status

package, patch = map(pathlib.Path, sys.argv[1:])
manifest = json.loads((patch/'INSTALL_MANIFEST.json').read_text())
for entry in manifest['files']:
    assert hashlib.sha256((package/entry['path']).read_bytes()).hexdigest() == entry['new_sha256'], entry['path']
vision = yaml.safe_load((package/'config/window_competition_vision.yaml').read_text())
field = yaml.safe_load((package/'config/window_test.yaml').read_text())
adapter = yaml.safe_load((package/'config/window_competition_adapter.yaml').read_text())
assert vision == field, 'Frozen field profile differs'
assert adapter['allow_legacy_output'] is False
assert adapter['publish_legacy_object_position'] is False
assert adapter['require_capture_stamp'] is True
assert parse_window_status('{}')[0] is None
gate = Confirmation(5, 3)
assert [gate.found() for _ in range(5)] == [False, False, False, False, True]
assert gate.missing() is False
assert PointStamped().point.z == 0.0
print('COMPETITION_ADAPTER_INSTALL_OK (source/import only; NOT ROS build/flight acceptance)')
PY
trap - ERR INT TERM
printf 'Installed %s whitelisted files. No node was started. Backup: %s\n' "${#FILES[@]}" "$BACKUP"
echo 'Next: rebuild ONLY /home/abot/vision_camera_test_ws and perform ground acceptance.'

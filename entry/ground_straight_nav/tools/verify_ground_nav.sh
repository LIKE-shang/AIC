#!/bin/bash
# Read-only verification of the installed packages. Never uploads or removes files.
set -euo pipefail
PFX=$(rospack find ground_straight_nav)
SLAM=$(rospack find slam_navigation)
python3 -m unittest discover -s "$PFX/test" -p 'test_*.py'
python3 -m unittest discover -s "$SLAM/test" -p 'test_localization.py'
python3 "$SLAM/tools/selfcheck_pointcloud.py"
bash "$PFX/tools/preflight_ground_nav.sh"
echo 'Software checks passed. A measured-start, full-footprint path check is still required; see README_CN.md.'

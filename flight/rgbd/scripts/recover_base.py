#!/usr/bin/env python3
"""Ground-only recovery of EXISTING localization tools; no controller or camera.

This does not redeploy or edit the legacy package. Its verified launch children
retain their deployed startup checks. No arm, mode request or control setpoint.
"""
import hashlib
import importlib.util
import os
from pathlib import Path
import sys
import uuid

LEGACY = Path('/home/abot/window_pass_session_20261013')
LEGACY_HASHES = {
    # 2026-10-18: session.py gained the autonomous-pass entry point and the
    # clearance-aware config checks, so the pinned digest moved with it.
    'scripts/session.py': '7494690b0daeb69704cc8c319e9b7c92fab9165042f061d593b5c24c91723d98',
    'session.bash': '109cb821ac311c6c5b2c2f55c85488a4093844ecd7e5b7ac33ef64df285d8e40',
    'launch/base.launch': 'b9097f8a7e682d6d36c92db44bf491d503dba98d05850f7e2d04f76249c8af0a',
}
CONTROL_TOPICS = (
    '/mavros/setpoint_raw/local', '/mavros/setpoint_position/local',
    '/mavros/setpoint_velocity/cmd_vel', '/mavros/setpoint_velocity/cmd_vel_unstamped',
    '/mavros/setpoint_raw/attitude', '/mavros/setpoint_attitude/attitude',
)


def verify_legacy(root):
    for name, expected in LEGACY_HASHES.items():
        path = root/name
        if not path.is_file():
            raise RuntimeError('EXISTING_SESSION_FILE_MISSING:'+str(path))
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError('EXISTING_SESSION_VERSION_MISMATCH:'+str(path))


def recovery_plan(legacy, publishers, existing_nodes=()):
    # Do not start a second bridge underneath any hold/align/controller session.
    legacy.no_control_session(publishers)
    active = {topic: publishers[topic] for topic in CONTROL_TOPICS if publishers.get(topic)}
    if active:
        raise RuntimeError('EXISTING_CONTROL_OUTPUT: finish the old ground controller first: '+repr(active))
    flags = legacy.component_plan(publishers, existing_nodes)
    flags['start_camera'] = False
    flags['start_detector'] = False
    if flags['start_pose'] and publishers.get('/mavros/vision_pose/pose'):
        raise RuntimeError('EXISTING_VISION_POSE_WITHOUT_BRIDGE_STATUS: do not duplicate localization')
    return flags


def recovery_command(root, flags, run_id):
    if set(flags) != {'start_mavros','start_lidar','start_lio','start_camera','start_detector','start_pose'}:
        raise RuntimeError('INCOMPATIBLE_BASE_COMPONENT_CONTRACT')
    if flags['start_camera'] or flags['start_detector'] or any(type(value) is not bool for value in flags.values()):
        raise RuntimeError('BASE_RECOVERY_LOCALIZATION_ONLY')
    if not run_id or not run_id.isalnum():
        raise RuntimeError('INVALID_RECOVERY_RUN_ID')
    return ['roslaunch', str(root/'launch/base.launch'), 'run_id:='+run_id] + [
        key+':='+str(flags[key]).lower() for key in sorted(flags)]


def main():
    import fcntl
    import rosgraph
    verify_legacy(LEGACY)
    # Same lock as the deployed wrapper: reuse its existing terminal, not a
    # second parent launch that might later be mistaken for the owning session.
    lock = (Path('/home/abot')/'.window_pass_session_launch.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('BASE_ALREADY_RUNNING: keep the existing healthy base terminal; do not restart it')
    os.set_inheritable(lock.fileno(), True)
    spec = importlib.util.spec_from_file_location('deployed_localization_session', LEGACY/'scripts/session.py')
    legacy = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(legacy)
    publishers, nodes = {}, set()
    if rosgraph.is_master_online():
        _, graph = legacy.tables()
        publishers = graph[0]
        nodes = {node for table in graph for row in table.values() for node in row}
    flags = recovery_plan(legacy, publishers, nodes)
    if publishers.get('/mavros/state'):
        legacy.require_ground(manual=True, landed=True)
    for needed in (legacy.POSE, Path('/home/abot/fast_lio_diagnostic_20261008_v1.launch')):
        if not needed.is_file():
            raise RuntimeError('EXISTING_LOCALIZATION_FILE_MISSING:'+str(needed))
    for key in ('mavros','lidar','lio','pose'):
        print(('START ' if flags['start_'+key] else 'REUSE ')+key, flush=True)
    print('LOCALIZATION_ONLY: keep this terminal. Wait for FORWARDING_CONTINUOUS_POSE. '
          'Old launch text mentioning geometry is historical: next use run_rgbd.bash, not geometry/air.', flush=True)
    command = recovery_command(LEGACY, flags, uuid.uuid4().hex)
    os.execvp(command[0], command)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('RGBD_BASE_STOP: '+str(exc), file=sys.stderr, flush=True)
        sys.exit(2)

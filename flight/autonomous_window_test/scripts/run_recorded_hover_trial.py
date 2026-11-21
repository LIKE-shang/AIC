#!/usr/bin/env python3
"""Run one existing hover trial and record telemetry. No control-law changes."""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path

TOPICS=['/mavros/timesync_status','/mavros/time_reference','/diagnostics','/mavros/state','/mavros/extended_state','/mavros/rc/in',
        '/mavros/local_position/odom','/window_pass_lio/Odometry','/livox/imu',
        # Do not subscribe to the control input: transport requires MAVROS
        # as its sole subscriber. Target readback and mission status retain
        # the requested positions without altering that handshake.
        '/mavros/imu/data','/mavros/setpoint_raw/target_local',
        '/mavros/vision_pose/pose','/mavros/estimator_status','/mavros/battery',
        '/mavros/statustext/recv','/autonomous_window_test/status',
        '/circle_pixel_preview/status','/hover_target_laser/status','/window_pass_competition/status',
        '/window_live/status','/window_live/geometry','/camera/rgb/camera_info','/camera/depth/camera_info']

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial-dir',type=Path,required=True)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    trial=args.trial_dir.resolve(strict=True)
    if trial.parent!=Path('/home/abot') or not trial.name.startswith(('hover_target1_trial_','full_chain_trial_')):
        raise ValueError('invalid_trial_directory')
    original=trial/'run.bash'
    result=subprocess.run(['bash',str(original)])
    if result.returncode or not args.execute:return result.returncode
    record=trial/('telemetry_'+str(time.monotonic_ns()))
    cfg=json.loads((trial/'config.json').read_text())
    duration=270 if cfg.get('mission',{}).get('second_window_return_enabled') else 150 if 'selected_target_ids' in cfg.get('full_chain',{}) else 120
    log=record.with_suffix('.log').open('ab')
    bag=subprocess.Popen(['rosbag','record','-O',str(record.with_suffix('.bag')),
                          '--duration='+str(duration)]+TOPICS,stdin=subprocess.DEVNULL,
                          stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        time.sleep(.5)
        if bag.poll() is not None:raise RuntimeError('telemetry_recorder_start_failed: '+str(record.with_suffix('.log')))
        print('TELEMETRY_RECORDING='+str(record.with_suffix('.bag')),flush=True)
        return subprocess.run(['bash',str(original),'--execute']).returncode
    finally:
        if bag.poll() is None:
            os.killpg(bag.pid,signal.SIGINT)
            try:bag.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(bag.pid,signal.SIGTERM)
                try:bag.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print('RECORDER_STILL_FLUSHING pid='+str(bag.pid),file=sys.stderr)
        log.close()

if __name__=='__main__':sys.exit(main())

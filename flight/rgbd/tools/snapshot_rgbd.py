#!/usr/bin/env python3
"""Bounded evidence snapshot, no flight actions and no waiting for READY."""
import argparse
import json
import math
from pathlib import Path
from window_live_vision.pipeline import summarize_frames
from window_live_vision.stage import stage_summary, display_lines
import subprocess
import tempfile
import threading
import time
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from mavros_msgs.msg import State, ExtendedState


def diagnostic_json(value):
    # Preserve sensor faults as evidence instead of failing to save on NaN.
    if isinstance(value, float) and not math.isfinite(value):
        return {'nonfinite_value': repr(value)}
    if isinstance(value, dict):
        return {key: diagnostic_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [diagnostic_json(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=3.)
    parser.add_argument('--verbose', action='store_true')
    args = parser.parse_args()
    if not 2 <= args.seconds <= 8:
        raise SystemExit('seconds must be [2,8]')
    rospy.init_node('window_rgbd_snapshot', anonymous=True)
    rows, lock = {}, threading.Lock()
    frames = {}
    received_counts, last_received = {}, {}
    def callback(topic, msg):
        with lock:
            if isinstance(msg, String):
                try:
                    row = json.loads(msg.data)
                except ValueError:
                    row = dict(invalid_json=msg.data)
            elif isinstance(msg, State):
                row = dict(connected=msg.connected, armed=msg.armed, mode=msg.mode, stamp=msg.header.stamp.to_sec())
            elif isinstance(msg, ExtendedState):
                row = dict(landed_state=msg.landed_state, stamp=msg.header.stamp.to_sec())
            else:
                row = dict(stamp=msg.header.stamp.to_sec(), frame_id=msg.header.frame_id,
                           width=msg.width, height=msg.height, K=list(msg.K), D=list(msg.D))
            rows[topic] = row
            received_counts[topic] = received_counts.get(topic, 0)+1
            last_received[topic] = time.monotonic()
            if topic == '/window_live/status':
                frame = row.get('last_frame_diagnostics') or {}
                runtime = row.get('runtime') or {}
                key = (runtime.get('instance_id'), frame.get('process_sequence'))
                if frame and key[1] is not None:
                    # Replace a frame with its completed-cycle timings on the
                    # next heartbeat, but count that frame only once.
                    frames[key] = frame
                    if len(frames) > 240:
                        del frames[next(iter(frames))]
    topics = [('/window_live/status', String), ('/window_live/geometry', String), ('/window_live/traversal', String),
              ('/mavros/state', State), ('/mavros/extended_state', ExtendedState),
              ('/window_pass_competition/status', String),
              ('/camera/rgb/camera_info', CameraInfo), ('/camera/depth/camera_info', CameraInfo)]
    subs = [rospy.Subscriber(topic, typ, lambda msg, topic=topic: callback(topic, msg), queue_size=1) for topic, typ in topics]
    extras = {}
    def camera_details():
        for service in ('get_device_info', 'get_camera_params'):
            try:
                result = subprocess.run(['rosservice', 'call', '/camera/'+service], stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, timeout=2., universal_newlines=True)
                value = result.stdout
            except Exception as exc:
                value = str(exc)
            with lock:
                extras['camera_'+service] = value
        try:
            code, _, graph = rospy.get_master().getSystemState()
            publishers = {topic: nodes for topic, nodes in graph[0] if topic.startswith('/camera/')}
            params = rospy.get_param('/camera/camera', {})
            with lock:
                extras.update(camera_publishers=publishers, camera_driver_params=params)
        except Exception as exc:
            with lock:
                extras['graph_error'] = str(exc)
    # The optional queries cannot block image evidence collection or exit.
    if args.verbose:
        threading.Thread(target=camera_details, daemon=True).start()
    deadline = time.monotonic()+args.seconds
    airborne_warning = False
    while time.monotonic() < deadline and not rospy.is_shutdown():
        with lock:
            state = rows.get('/mavros/state', {})
            extended = rows.get('/mavros/extended_state', {})
        if state.get('armed') or extended.get('landed_state') == ExtendedState.LANDED_STATE_IN_AIR:
            airborne_warning = True
            break
        time.sleep(.1)
    with lock:
        output = dict(rows)
        output.update(extras)
        output['processing_window'] = summarize_frames(list(frames.values()))
        output['frame_diagnostics_observed'] = list(frames.values())
        output['received_counts'] = dict(received_counts)
        output['last_received_age_s'] = {topic: time.monotonic()-wall for topic, wall in last_received.items()}
    output['actual_output_permitted'] = False
    output['snapshot_waited_for_ready'] = False
    output['airborne_warning'] = airborne_warning
    output['missing_topics'] = [topic for topic, _ in topics if topic not in output]
    root = Path.home()/'window_rgbd_logs'
    root.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix='check_', dir=str(root)))
    path = directory/'rgbd_inputs.json'
    path.write_text(json.dumps(diagnostic_json(output), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    summary = stage_summary(output, output['last_received_age_s'], rospy.Time.now().to_sec())
    print('\n'.join(display_lines(summary)))
    if args.verbose:
        live = output.get('/window_live/status', {})
        print(json.dumps({key: live.get(key) for key in ('version', 'reason', 'target_count', 'ready', 'confirmed_frames', 'tracking_reset_reason', 'depth_reason',
              'depth_stamp_delta_s', 'metric_geometry_ready', 'metric_geometry_reason', 'traversal_reason')}, ensure_ascii=False, indent=2))
        runtime = live.get('runtime') or {}
        last_frame = live.get('last_frame_diagnostics') or {}
        print('configuration: '+json.dumps(live.get('configuration'), ensure_ascii=False))
        print('streams: '+json.dumps(runtime.get('streams'), ensure_ascii=False))
        print('processing: '+json.dumps({key: runtime.get(key) for key in
              ('processed_rate_hz', 'processing_active', 'processing_stage', 'result_fresh', 'result_age_wall_s', 'result_age_ros_s')}, ensure_ascii=False))
        print('last_frame: '+json.dumps(last_frame, ensure_ascii=False))
        print('processing_window: '+json.dumps(output['processing_window'], ensure_ascii=False))
        pose = output.get('/window_pass_competition/status', {})
        print('localization: '+json.dumps({key: pose.get(key) for key in
              ('real_output_state','real_output_rate_hz','real_fault','horizontal_flags_valid')}, ensure_ascii=False))
        print('fcu: '+json.dumps(output.get('/mavros/state', {}), ensure_ascii=False))
        print('landed: '+json.dumps(output.get('/mavros/extended_state', {}), ensure_ascii=False))
    print('Saved: '+str(path))
    if airborne_warning:
        print('当前检测到解锁/空中：已立即保存，不继续等条件；按已验证流程人工接管降落，落地后再分析。')
    else:
        print('本次快照不授予飞行权限，不是A_READY；不要按A，不要为等待识别起飞。')


if __name__ == '__main__':
    main()
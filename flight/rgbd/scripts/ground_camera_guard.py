#!/usr/bin/env python3
"""Launch prefix: refuse camera start/reconfigure in flight or over another driver.

Read-only MAVROS subscriptions, USB discovery and ROS camera parameter setup.
No arm, mode or setpoint interface. Existing localization is not touched.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import threading
import time


def uvc_devices(root='/sys/class/video4linux'):
    found = {}
    for video in Path(root).glob('video*'):
        device = (video/'device').resolve()
        for parent in (device,)+tuple(device.parents):
            vendor, product = parent/'idVendor', parent/'idProduct'
            if vendor.is_file() and product.is_file():
                if vendor.read_text().strip().lower() == '2bc5':
                    pid = int(product.read_text().strip(), 16)
                    name = (parent/'product').read_text().strip() if (parent/'product').is_file() else 'Orbbec USB'
                    found[str(parent)] = dict(product_id=pid, product=name, usb_path=str(parent))
                break
    return list(found.values())


def duplicate_publishers(system_state, topics):
    return {topic: nodes for topic, nodes in system_state[0] if topic in topics and nodes}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera-name', default='camera')
    parser.add_argument('--uvc-product-id', type=lambda s: int(s, 0), default=0)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if not args.command or not os.path.isfile(args.command[0]):
        raise RuntimeError('camera executable missing')
    import rospy
    from mavros_msgs.msg import State, ExtendedState
    rospy.init_node('window_rgbd_ground_guard', anonymous=True, argv=[sys.argv[0]])
    rows, lock = {}, threading.Lock()
    def callback(key, msg):
        with lock:
            rows[key] = (msg, time.monotonic())
    rospy.Subscriber('/mavros/state', State, lambda msg: callback('state', msg), queue_size=1)
    rospy.Subscriber('/mavros/extended_state', ExtendedState, lambda msg: callback('extended', msg), queue_size=1)
    deadline = time.monotonic()+6.
    while time.monotonic() < deadline and not rospy.is_shutdown():
        with lock:
            sample = dict(rows)
        ready = True
        for key in ('state', 'extended'):
            if key not in sample:
                ready = False
                continue
            msg, wall = sample[key]
            age = rospy.Time.now().to_sec()-msg.header.stamp.to_sec()
            if msg.header.stamp.to_sec() <= 0 or not -.02 <= age <= 2.5 or time.monotonic()-wall > 1.5:
                ready = False
        if ready:
            state, extended = sample['state'][0], sample['extended'][0]
            if state.armed or extended.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
                raise RuntimeError('camera change refused: require disarmed + ON_GROUND; land first')
            if state.connected:
                break
        time.sleep(.1)
    else:
        raise RuntimeError('no fresh connected/disarmed/ON_GROUND FCU evidence; camera not started')
    code, message, system = rospy.get_master().getSystemState()
    if code != 1:
        raise RuntimeError('cannot inspect ROS camera ownership')
    topics = ['/'+args.camera_name+'/rgb/image_raw', '/'+args.camera_name+'/depth/image_raw']
    duplicates = duplicate_publishers(system, topics)
    if duplicates:
        raise RuntimeError('existing camera publisher; stop ONLY its old camera launch while disarmed: '+json.dumps(duplicates))
    devices = uvc_devices()
    if args.uvc_product_id:
        choices = [device for device in devices if device['product_id'] == args.uvc_product_id]
    else:
        choices = devices
    if len(choices) != 1:
        raise RuntimeError('cannot uniquely identify Orbbec UVC device: '+json.dumps(devices))
    selected = choices[0]
    rospy.set_param('/'+args.camera_name+'/camera/uvc_vendor_id', int('2bc5', 16))
    rospy.set_param('/'+args.camera_name+'/camera/uvc_product_id', selected['product_id'])
    print('GROUNDED_RGBD_CAMERA_START '+json.dumps(selected, ensure_ascii=False), flush=True)
    # Immediately recheck the received state after USB/master checks.
    with lock:
        state, wall = rows['state']
        extended, ext_wall = rows['extended']
    if state.armed or not state.connected or extended.landed_state != ExtendedState.LANDED_STATE_ON_GROUND or time.monotonic()-min(wall, ext_wall) > 1.5:
        raise RuntimeError('ground conditions changed; camera not started')
    rospy.signal_shutdown('guard complete; starting camera')
    os.execv(args.command[0], args.command)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('RGBD_CAMERA_REFUSED: '+str(exc), file=sys.stderr, flush=True)
        sys.exit(2)
#!/usr/bin/env python3
"""Gemini RGB hollow-window test. Publishes images/status only, never control."""
import math
import time
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from robot_vision_test.vision import WINDOW_DEFAULTS, detect_windows, checked_parameters, annotate_windows
from robot_vision_test.ros_images import read_bgr, test_topic, publish_image, publish_json


class WindowTest:
    def __init__(self):
        self.bridge = CvBridge()
        self.params = {k: rospy.get_param('~'+k, v) for k, v in WINDOW_DEFAULTS.items()}
        self.params = checked_parameters(WINDOW_DEFAULTS, self.params)
        self.max_processing_fps = float(rospy.get_param('~max_processing_fps', 10.0))
        if not math.isfinite(self.max_processing_fps) or self.max_processing_fps < 0:
            raise ValueError('max_processing_fps must be finite and >= 0 (0 disables rate limit)')
        self.next_frame_time = 0.0
        self.image_pub = rospy.Publisher(test_topic('output_image_topic', '/window_test/image', '/window_test'), Image, queue_size=1)
        self.edge_pub = rospy.Publisher(test_topic('edges_topic', '/window_test/edges', '/window_test'), Image, queue_size=1)
        self.status_pub = rospy.Publisher(test_topic('status_topic', '/window_test/status', '/window_test'), String, queue_size=1, latch=True)
        self.publish_edges = rospy.get_param('~publish_edges', True)
        topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        self.sub = rospy.Subscriber(topic, Image, self.image, queue_size=1, buff_size=16777216)
        rospy.loginfo('Window test: RGB input %s -> %s. No hardware/control interfaces.', rospy.resolve_name(topic), self.image_pub.resolved_name)

    def image(self, msg):
        now = time.monotonic()
        if now < self.next_frame_time:
            return  # Drop before conversion/processing; do not sleep in subscriber.
        self.next_frame_time = now + (1.0/self.max_processing_fps if self.max_processing_fps else 0.0)
        try:
            frame = read_bgr(self.bridge, msg)
            found, edges = detect_windows(frame, self.params)
            processing_ms = round((time.monotonic()-now)*1000.0, 2)
            publish_image(self.bridge, self.image_pub, annotate_windows(frame, found), msg.header)
            if self.publish_edges:
                publish_image(self.bridge, self.edge_pub, edges, msg.header, 'mono8')
            targets = [dict(method=t['method'], outline_kind=t['outline_kind'],
                            center_px=[round(float(v), 1) for v in t['center']],
                            width_px=round(t['width'], 1), height_px=round(t['height'], 1),
                            area_px=round(t['area'], 1)) for t in found]
            # Capture metadata lets the separate ground adapter reject stale
            # latched observations. It is NOT a flight-safety certification.
            capture_stamp = msg.header.stamp.to_sec()
            publish_json(self.status_pub, dict(state='found' if found else 'not_found',
                         count=len(found), targets=targets, processing_ms=processing_ms,
                         max_processing_fps=self.max_processing_fps,
                         input_encoding=msg.encoding, image_width=msg.width,
                         image_height=msg.height, frame_seq=msg.header.seq,
                         capture_stamp=capture_stamp,
                         camera_frame_id=msg.header.frame_id,
                         rgb_only=True, no_control=True))
        except Exception as exc:
            rospy.logerr_throttle(2.0, 'Window image rejected: %s', exc)
            publish_json(self.status_pub, dict(state='image_error', error=str(exc),
                         capture_stamp=0.0, no_control=True))


if __name__ == '__main__':
    rospy.init_node('window_test')
    try:
        WindowTest()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal('Window test startup failed: %s', exc)
        raise

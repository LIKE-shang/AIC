#!/usr/bin/env python3
"""Measured image reference and candidate feedback; no flight or laser control."""
import json
import threading
import time
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from robot_vision_test.air_target import air_parameters, detect_air_target, jsonable
from robot_vision_test.hover_detection import detect_hover_target
from robot_vision_test.pixel_preview import parameters, evaluate, draw, unavailable
from robot_vision_test.safety import FrameFreshness, ContentFreshness


class PixelPreview:
    def __init__(self):
        self.preview = parameters(rospy.get_param('~pixel_preview', {}))
        self.hover_servo = rospy.get_param('~hover_servo', False) is True
        self.detector = air_parameters(rospy.get_param('~detector', {}))
        self.timeout = float(rospy.get_param('~image_timeout', .35))
        self.freshness = FrameFreshness(self.timeout, .05, True)
        self.content = ContentFreshness(.25)
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.frame_lock = threading.Lock()
        self.stop = threading.Event()
        self.closed = False
        self.generation = 0
        self.last_wall = time.monotonic()
        self.last_stamp = None
        self.last_header = None
        self.stale = True
        self.input_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        self.image_pub = rospy.Publisher('/circle_pixel_preview/image', Image, queue_size=1)
        self.status_pub = rospy.Publisher('/circle_pixel_preview/status', String, queue_size=1)
        rospy.on_shutdown(self.close)
        self.emit_status(unavailable('await_first_frame'))
        self.watchdog = threading.Thread(target=self.watch, daemon=True)
        self.watchdog.start()
        self.sub = rospy.Subscriber(self.input_topic, Image, self.callback, queue_size=1,
                                    buff_size=2**21, tcp_nodelay=True)
        rospy.loginfo('Pixel preview reference offset=(%.3f, %.3f); no flight/PWM interfaces',
                      self.preview['offset_x_px'], self.preview['offset_y_px'])

    def emit_status(self, result, header=None, **extra):
        payload = dict(result, hardware_commands_sent=False, **extra)
        if header is not None:
            payload.update(capture_stamp=header.stamp.to_sec(), frame_seq=header.seq,
                           camera_frame_id=header.frame_id)
        self.status_pub.publish(String(data=json.dumps(jsonable(payload), allow_nan=False)))

    def invalidate(self, reason):
        with self.lock:
            if self.closed: return
            self.generation += 1
            self.stale = True
            result = unavailable(reason)
            frame = np.zeros((self.preview['image_height'], self.preview['image_width'], 3), np.uint8)
            draw(frame, {'targets': []}, result)
            message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            if self.last_header is not None: message.header = self.last_header
            self.image_pub.publish(message)
            self.emit_status(result, self.last_header)

    def callback(self, message):
        if not self.frame_lock.acquire(False): return
        try:
            began = time.monotonic()
            with self.lock:
                if self.closed: return
                generation = self.generation
                self.last_header = message.header
                stamp = message.header.stamp.to_sec()
                self.freshness.check(stamp, rospy.Time.now().to_sec())
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8').copy()
            self.content.check(frame.tobytes(), time.monotonic())
            if (frame.shape[:2] != (self.preview['image_height'], self.preview['image_width']) or
                    message.header.frame_id != self.preview['camera_frame_id']):
                self.invalidate('pixel_reference_camera_or_size_mismatch')
                return
            detector = detect_hover_target if self.hover_servo else detect_air_target
            detection, _ = detector(frame, self.detector)
            result = evaluate(frame, message.header.frame_id, detection, self.preview)
            draw(frame, detection, result)
            out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out.header = message.header
            with self.lock:
                if self.closed or generation != self.generation: return
                age = rospy.Time.now().to_sec()-stamp
                if not -.05 <= age <= self.timeout:
                    self.invalidate('capture_expired_during_processing')
                    return
                self.image_pub.publish(out)
                self.emit_status(result, message.header, capture_age_s=age,
                                 detector_reason=detection.get('reason',''),
                                 detector_mode=detection.get('hover_detection_mode','three_ring_original'),
                                 detector_ring_count=len(detection.get('rings',[])),
                                 detector_target_count=len(detection.get('targets',[])),
                                 processing_ms=(time.monotonic()-began)*1000,
                                 source_image_size_px=[frame.shape[1], frame.shape[0]])
                self.last_wall = time.monotonic()
                self.last_stamp = stamp
                self.stale = False
        except Exception as exc:
            self.invalidate('invalid_image:'+str(exc))
            rospy.logerr_throttle(2., 'Pixel preview image rejected: %s', str(exc))
        finally:
            self.frame_lock.release()

    def watch(self):
        while not self.stop.wait(.05):
            with self.lock:
                if self.closed: return
                expired = time.monotonic()-self.last_wall > self.timeout
                if self.last_stamp is not None:
                    age = rospy.Time.now().to_sec()-self.last_stamp
                    expired = expired or age > self.timeout or age < -.05
                if expired and not self.stale:
                    self.invalidate('image_stream_stale')

    def close(self):
        with self.lock:
            if self.closed: return
            self.closed = True
            self.stop.set()
            self.emit_status(unavailable('preview_shutdown'))


if __name__ == '__main__':
    rospy.init_node('circle_pixel_preview')
    cv2.setNumThreads(1)
    node = PixelPreview()
    try:
        rospy.spin()
    finally:
        node.close()

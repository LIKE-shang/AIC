#!/usr/bin/env python3
"""Optional depth preview ONLY. It is never input to the RGB window detector."""
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from robot_vision_test.vision import depth_to_gray
from robot_vision_test.safety import safe_test_topic


class DepthPreview:
    def __init__(self):
        self.bridge = CvBridge()
        self.near = float(rospy.get_param('~near_metres', 0.2))
        self.far = float(rospy.get_param('~far_metres', 5.0))
        self.scale = float(rospy.get_param('~uint16_scale', 0.001))
        output = safe_test_topic(rospy.resolve_name(rospy.get_param('~output_topic', '/window_test/depth_preview')))
        if not output.startswith('/window_test/'):
            raise ValueError('depth preview output must stay in /window_test/')
        self.pub = rospy.Publisher(output, Image, queue_size=1)
        self.sub = rospy.Subscriber(rospy.get_param('~image_topic', '/camera/depth/image_raw'), Image, self.callback, queue_size=1, buff_size=2**24)

    def callback(self, msg):
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            gray = depth_to_gray(raw, msg.encoding, self.near, self.far, self.scale)
            out = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')
            out.header = msg.header
            self.pub.publish(out)
        except Exception as exc:
            rospy.logerr_throttle(2.0, 'Depth preview rejected: %s' % exc)


if __name__ == '__main__':
    rospy.init_node('window_test_depth_preview')
    node = DepthPreview()
    rospy.spin()

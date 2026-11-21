"""ROS image helpers. Raw depth is deliberately NOT an input to these RGB tests."""
import json
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def test_topic(param, default, prefix):
    name = rospy.resolve_name(rospy.get_param('~'+param, default))
    if not name.startswith(prefix + '/'):
        raise ValueError('%s must stay under %s/ (mission topic isolation)' % (param, prefix))
    return name


def read_bgr(bridge, msg):
    if msg.encoding not in ('bgr8', 'rgb8', 'bgra8', 'rgba8', 'mono8', 'yuv422', 'yuv422_yuy2'):
        raise ValueError('Expected RGB/mono8, got %s. Do not remap raw 16UC1/32FC1 depth here.' % msg.encoding)
    return bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').copy()


def publish_image(bridge, publisher, frame, header, encoding='bgr8'):
    msg = bridge.cv2_to_imgmsg(frame, encoding=encoding)
    msg.header = header
    publisher.publish(msg)


def publish_json(pub, data):
    pub.publish(String(data=json.dumps(data, sort_keys=True)))

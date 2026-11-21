import os
import sys
import time
import unittest
from unittest.mock import patch
import test_node_boundary as boundary
sys.path.insert(0,os.path.join(os.path.dirname(__file__),'..','tools'))
import axis_check

class TestAxisCheck(unittest.TestCase):
    def test_stale_pose_stops_before_motion(self):
        probe = axis_check.Probe()
        probe.pose = (0,0,0,time.monotonic()-10)
        boundary.node_module.rospy.published[:] = []
        with self.assertRaises(RuntimeError): probe.send(0,0.06,1)
        cmds = [msg for topic,msg in boundary.node_module.rospy.published if topic == '/cmd_vel']
        self.assertTrue(cmds)
        self.assertTrue(all(msg.linear.y == 0 for msg in cmds))
    def test_interrupt_still_sends_zero(self):
        probe = axis_check.Probe()
        probe.pose = (0,0,0,time.monotonic())
        with patch.object(axis_check.time,'sleep',side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt): probe.send(0,0.06,1)
        msg = boundary.node_module.rospy.published[-1][1]
        self.assertEqual((msg.linear.x,msg.linear.y,msg.angular.z),(0,0,0))
    def test_unbounded_speed_refused(self):
        with self.assertRaises(SystemExit) as exc: axis_check.main(['--speed','10'])
        self.assertEqual(exc.exception.code,2)

if __name__ == '__main__': unittest.main()

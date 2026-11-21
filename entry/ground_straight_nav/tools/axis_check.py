#!/usr/bin/env python3
"""One-shot check that +linear.y moves the chassis to its own left.

The direction of ``linear.y`` on the starrobot base is a property of the
firmware and of ``e1Dir..e4Dir`` in ``robot_type.yaml``; nothing in this
workspace states it.  This tool commands a short, slow +y pulse and reports
which way the map pose actually moved, so the straight-run node can rely on
its own sign convention.  It moves the robot: keep the area clear.

It only ever commands ``angular.z = 0`` and a bounded pulse, and it publishes
zero on exit or Ctrl+C.
"""
import argparse
import math
import sys
import time

import rospy
import rosgraph
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class Probe(object):
    def __init__(self):
        self.pose = None
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/localization', Odometry, self._cb, queue_size=1)

    def _cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.to_sec()
        age = rospy.Time.now().to_sec() - stamp
        if (msg.header.frame_id != 'map' or stamp <= 0 or not -0.1 <= age <= 0.5 or
                not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)) or
                not 0.9 <= q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w <= 1.1):
            return
        if getattr(self, '_stamp', None) is not None and stamp <= self._stamp:
            return
        self._stamp = stamp
        self.pose = (p.x, p.y, quat_to_yaw(msg.pose.pose.orientation), time.monotonic())

    def wait_pose(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.pose is not None:
                return True
            time.sleep(0.05)
        return False

    def fresh(self):
        return self.pose is not None and 0 <= time.monotonic() - self.pose[3] <= 0.5

    def send(self, vx, vy, seconds):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = 0.0
        end = time.monotonic() + seconds
        try:
            while time.monotonic() < end and not rospy.is_shutdown():
                if not self.fresh():
                    raise RuntimeError('pose lost during axis pulse')
                self.pub.publish(cmd)
                time.sleep(0.05)
        finally:
            self.stop()

    def stop(self):
        self.pub.publish(Twist())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=float, default=0.06,
                        help='pulse speed, m/s (default 0.06)')
    parser.add_argument('--seconds', type=float, default=1.0)
    parser.add_argument('--settle', type=float, default=0.6)
    parser.add_argument('--yes', action='store_true',
                        help='skip the interactive confirmation')
    args = parser.parse_args(rospy.myargv(argv=[sys.argv[0]] + argv if argv is not None else None)[1:])
    if (not all(math.isfinite(v) for v in (args.speed,args.seconds,args.settle)) or
            not 0 < args.speed <= 0.1 or not 0 < args.seconds <= 2 or not 0 <= args.settle <= 5):
        parser.error('speed must be (0,0.1], seconds (0,2], settle [0,5]')

    rospy.init_node('ground_axis_check', anonymous=True)
    pubs = rosgraph.Master(rospy.get_name()).getSystemState()[0]
    if any(topic == rospy.resolve_name('/cmd_vel') and nodes for topic,nodes in pubs):
        raise SystemExit('another /cmd_vel publisher is running; stop it before the axis test')
    probe = Probe()
    rospy.on_shutdown(probe.stop)
    if not probe.wait_pose(10.0):
        print('no pose on /localization; start FAST-LIO + ICP alignment first')
        return 2

    x0, y0, yaw0, _ = probe.pose
    print('start pose: x=%.3f y=%.3f yaw=%.3f rad' % (x0, y0, yaw0))
    print('about to pulse +linear.y %.3f m/s for %.1f s '
          '(expected travel %.2f m)' % (args.speed, args.seconds,
                                        args.speed * args.seconds))
    if not args.yes:
        sys.stdout.write('type GO and press Enter: ')
        sys.stdout.flush()
        if sys.stdin.readline().strip() != 'GO':
            print('cancelled')
            return 1

    if not probe.fresh():
        raise SystemExit('pose became stale while awaiting confirmation')
    x0, y0, yaw0, _ = probe.pose
    probe.send(0.0, args.speed, args.seconds)
    time.sleep(args.settle)
    if not probe.fresh():
        raise SystemExit('no fresh pose after pulse; result cannot be trusted')
    x1, y1, yaw1, _ = probe.pose

    # Project the displacement onto the chassis-left axis at start heading.
    dx, dy = x1 - x0, y1 - y0
    left = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
    forward = math.cos(yaw0) * dx + math.sin(yaw0) * dy
    dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))

    print()
    print('moved: forward %+.3f m, left %+.3f m, yaw %+.4f rad' %
          (forward, left, dyaw))
    verdict = 'UNKNOWN'
    if abs(left) < 0.01 and abs(forward) < 0.01:
        verdict = 'NO_MOTION (base not listening, or e1Dir..e4Dir mixed up)'
    elif left > abs(forward):
        verdict = 'OK: +linear.y is chassis-left, straight_nav sign is valid'
    elif left < -abs(forward):
        verdict = 'INVERTED: +linear.y is chassis-right, negate vy in the node'
    else:
        verdict = 'CROSSTALK: motion went mostly forward, check the wheel wiring'
    print('verdict: %s' % verdict)
    if abs(dyaw) > 0.02:
        print('warning: yaw moved %.3f rad during a pure lateral pulse; the '
              'chassis cannot hold heading, lower --speed' % dyaw)
    return 0 if verdict.startswith('OK') else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(130)

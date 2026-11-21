"""Deterministic preparation waiting/diagnostic tests; no ROS or real waits."""
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

from ground_snapshot import ground_rejection, latest_messages


TOPICS = [(name, object) for name in
          ('/mavros/state', '/mavros/extended_state', '/mavros/rc/in')]


def msg(kind, stamp=10., caller='/mavros', **changes):
    fields = [dict(connected=True, armed=False, system_status=3, mode='MANUAL'),
              dict(landed_state=1), dict(channels=[1050]*8)][kind]
    fields.update(changes)
    return NS(header=NS(stamp=NS(to_sec=lambda: stamp)),
              _connection_header={'callerid': caller}, **fields)


class ScriptedROS:
    ROSException = RuntimeError
    ROSInterruptException = InterruptedError

    def __init__(self, events, ros_now=None):
        self.wall = 0.
        self.events = list(events)
        self.callbacks = {}
        self.closed = []
        self.now = ros_now or (lambda: 10.+self.wall)
        self.Time = NS(now=lambda: NS(to_sec=self.now))

    def Subscriber(self, topic, kind, callback, queue_size):
        self.callbacks[topic] = callback
        return NS(unregister=lambda: self.closed.append(topic))

    def is_shutdown(self):
        return False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def notify_all(self):
        pass

    def wait(self, duration):
        self.wall += duration
        while self.events and self.events[0][0] <= self.wall+1.e-9:
            _, index, message = self.events.pop(0)
            self.callbacks[TOPICS[index][0]](message)


class FreshSnapshotWaitTest(unittest.TestCase):
    def run_sample(self, ros, maximum=2., timeout=.2, modes=None):
        with patch('ground_snapshot.threading.Condition', return_value=ros), \
                patch('ground_snapshot.time.monotonic', side_effect=lambda: ros.wall):
            try:
                return latest_messages(ros, TOPICS, timeout=timeout, max_age=maximum,
                                       reject=lambda t,m: ground_rejection(t,m,modes))
            finally:
                self.assertEqual(ros.closed, [t for t,_ in TOPICS])

    def test_waits_for_replacement_after_all_three_arrive(self):
        latest = msg(0, stamp=10.09)
        ros = ScriptedROS([(.05,0,msg(0,stamp=7.)), (.05,1,msg(1)),
                           (.05,2,msg(2)), (.1,0,latest)])
        self.assertIs(self.run_sample(ros)[0], latest)
        self.assertAlmostEqual(ros.wall, .1)

    def test_wait_requires_all_three_fresh_at_same_time(self):
        ros = ScriptedROS([(.05,0,msg(0,stamp=8.06)), (.05,1,msg(1,stamp=7.)),
                           (.05,2,msg(2)), (.15,1,msg(1,stamp=10.15))])
        with self.assertRaisesRegex(RuntimeError, '/mavros/state: age_s='):
            self.run_sample(ros)
        self.assertAlmostEqual(ros.wall, .2)

    def test_timeout_contains_each_topic_age_stamp_and_caller(self):
        ros = ScriptedROS([(.05,i,msg(i,stamp=7.)) for i in range(3)])
        with self.assertRaises(RuntimeError) as error:
            self.run_sample(ros)
        text = str(error.exception)
        self.assertIn('fresh trusted MAVROS required', text)
        self.assertIn('age_s=3.200000000', text)
        self.assertIn('required=-0.02<=age_s<2.0', text)
        self.assertEqual(text.count("callerid='/mavros'"), 3)
        self.assertEqual(text.count('fresh=False'), 3)
        for topic,_ in TOPICS:
            self.assertIn(topic, text)
        self.assertAlmostEqual(ros.wall, .2)

    def test_missing_topic_diagnostic(self):
        ros = ScriptedROS([(.05,0,msg(0))])
        with self.assertRaisesRegex(RuntimeError, '/mavros/rc/in: missing'):
            self.run_sample(ros)

    def test_observed_armed_is_not_hidden_by_later_safe_message(self):
        ros = ScriptedROS([(.05,0,msg(0,armed=True)), (.05,0,msg(0))])
        with self.assertRaisesRegex(ValueError, 'armed=true') as error:
            self.run_sample(ros)
        self.assertIn('armed=True', str(error.exception))
        self.assertAlmostEqual(ros.wall, .05)

    def test_unsafe_messages_reject_without_waiting_for_other_topics(self):
        cases = [(0,msg(0,connected=False)), (0,msg(0,system_status=4)),
                 (1,msg(1,landed_state=2)), (2,msg(2,channels=[1050]*6)),
                 (2,msg(2,channels=[1050]*5+[1500,1050])),
                 (2,msg(2,channels=[1050]*6+[1500])), (1,msg(1,caller='/other'))]
        for index,message in cases:
            with self.subTest(index=index,message=message):
                ros = ScriptedROS([(.05,index,message)])
                with self.assertRaises(ValueError):
                    self.run_sample(ros)
                self.assertAlmostEqual(ros.wall, .05)

    def test_bad_source_cannot_be_hidden_by_following_good_source(self):
        ros = ScriptedROS([(.05,0,msg(0,caller='/other')), (.05,0,msg(0))])
        with self.assertRaisesRegex(ValueError, "callerid='/other'"):
            self.run_sample(ros)

    def test_only_transport_check_has_manual_posctl_requirement(self):
        events = [(.05,i,msg(i,**({'mode':'ALTCTL'} if i==0 else {}))) for i in range(3)]
        self.run_sample(ScriptedROS(events))
        with self.assertRaisesRegex(ValueError, 'mode must be one of'):
            self.run_sample(ScriptedROS(events),modes=('MANUAL','POSCTL'))

    def test_each_existing_age_limit_remains_strict(self):
        for maximum in (1.5,2.):
            for age,accepted in ((maximum-.001,True),(maximum,False),(-.019,True),(-.021,False)):
                with self.subTest(maximum=maximum,age=age):
                    ros = ScriptedROS([(.05,i,msg(i,stamp=10.-age)) for i in range(3)],ros_now=lambda:10.)
                    if accepted:
                        self.run_sample(ros,maximum=maximum)
                    else:
                        with self.assertRaises(RuntimeError):
                            self.run_sample(ros,maximum=maximum)

    def test_future_timestamp_can_be_replaced_but_not_accepted(self):
        ros = ScriptedROS([(.05,i,msg(i,stamp=50.)) for i in range(3)] +
                           [(.1,i,msg(i,stamp=10.1)) for i in range(3)])
        self.run_sample(ros)
        self.assertAlmostEqual(ros.wall,.1)

    def test_ros_clock_jump_does_not_extend_monotonic_deadline(self):
        ros = ScriptedROS([(.05,i,msg(i,stamp=10.)) for i in range(3)],ros_now=lambda:0.)
        with self.assertRaisesRegex(RuntimeError, 'age_s=-10.000000000'):
            self.run_sample(ros)
        self.assertAlmostEqual(ros.wall,.2)

    def test_fresh_messages_at_deadline_do_not_extend_wait_budget(self):
        ros=ScriptedROS([(.2,i,msg(i,stamp=10.2)) for i in range(3)])
        with self.assertRaisesRegex(RuntimeError,'wait timeout'):
            self.run_sample(ros)
        self.assertAlmostEqual(ros.wall,.2)

    def test_keeps_latest_updates_while_one_topic_is_stale(self):
        latest = msg(2,stamp=10.09)
        ros = ScriptedROS([(.05,0,msg(0,stamp=7.)),(.05,1,msg(1)),(.05,2,msg(2)),
                           (.1,2,latest),(.15,0,msg(0,stamp=10.15))])
        self.assertIs(self.run_sample(ros)[2],latest)


if __name__ == '__main__':
    unittest.main()

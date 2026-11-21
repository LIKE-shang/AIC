"""Concurrent receive tests; no ROS transport, vehicle calls or outputs."""
import threading
import time
import unittest
from types import SimpleNamespace

from ground_snapshot import latest_messages


class FakeROS:
    ROSException = RuntimeError
    ROSInterruptException = InterruptedError

    def __init__(self, shutdown=False, fail_topic=None):
        self.callbacks = {}
        self.closed = []
        self.shutdown = shutdown
        self.fail_topic = fail_topic
        self.ready = threading.Event()

    def Subscriber(self, topic, kind, callback, queue_size):
        if topic == self.fail_topic:
            raise ValueError('subscription failed')
        assert queue_size == 1
        self.callbacks[topic] = callback
        if len(self.callbacks) == 3:
            self.ready.set()
        return SimpleNamespace(unregister=lambda: self.closed.append(topic))

    def is_shutdown(self):
        return self.shutdown


TOPICS = [('state', object), ('extended', object), ('rc', object)]


class GroundSnapshotTest(unittest.TestCase):
    def collect(self, events):
        ros = FakeROS()
        failures = []

        def publish():
            if not ros.ready.wait(1):
                failures.append('subscriptions were not installed concurrently')
                return
            for topic, message in events:
                ros.callbacks[topic](message)

        worker = threading.Thread(target=publish)
        worker.start()
        try:
            result = latest_messages(ros, TOPICS, timeout=1)
        finally:
            worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertEqual(failures, [])
        self.assertCountEqual(ros.closed, ['state', 'extended', 'rc'])
        return result

    def test_all_subscribed_before_any_message(self):
        self.assertEqual(self.collect([('rc', 3), ('state', 1), ('extended', 2)]), (1, 2, 3))

    def test_replaces_early_message_while_waiting_for_rc(self):
        self.assertEqual(self.collect([('state', 'old'), ('extended', 2),
                                       ('state', 'latest'), ('rc', 3)]), ('latest', 2, 3))

    def test_does_not_filter_unsafe_or_stale_messages(self):
        unsafe = SimpleNamespace(armed=True, stamp=-100, callerid='/other')
        result = self.collect([('state', unsafe), ('extended', 2), ('rc', 3)])
        self.assertIs(result[0], unsafe)

    def test_return_in_requested_order(self):
        self.assertEqual(self.collect([('extended', 20), ('rc', 30), ('state', 10)]), (10, 20, 30))

    def test_missing_topic_times_out_and_cleans_up(self):
        ros = FakeROS()
        with self.assertRaisesRegex(RuntimeError, 'state, extended, rc'):
            latest_messages(ros, TOPICS, timeout=.01)
        self.assertCountEqual(ros.closed, ['state', 'extended', 'rc'])

    def test_shutdown_cleans_up(self):
        ros = FakeROS(shutdown=True)
        with self.assertRaises(InterruptedError):
            latest_messages(ros, TOPICS, timeout=1)
        self.assertEqual(len(ros.closed), 3)

    def test_subscription_error_cleans_up_existing_subscriber(self):
        ros = FakeROS(fail_topic='extended')
        with self.assertRaisesRegex(ValueError, 'subscription failed'):
            latest_messages(ros, TOPICS, timeout=1)
        self.assertEqual(ros.closed, ['state'])


if __name__ == '__main__':
    unittest.main()

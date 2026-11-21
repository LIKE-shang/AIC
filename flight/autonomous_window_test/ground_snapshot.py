"""Read the latest messages concurrently; callers retain their existing guards."""
import threading
import time


def ground_rejection(topic, message, allowed_modes=None):
    """Fail fast on the caller's existing source/vehicle/RC requirements."""
    if (message._connection_header or {}).get('callerid') != '/mavros':
        return 'callerid must be /mavros'
    if topic == '/mavros/state':
        if not message.connected:
            return 'connected=false'
        if message.armed:
            return 'armed=true'
        if message.system_status != 3:
            return 'system_status must be 3'
        if allowed_modes is not None and message.mode not in allowed_modes:
            return 'mode must be one of ' + repr(allowed_modes)
    elif topic == '/mavros/extended_state':
        if message.landed_state != 1:
            return 'landed_state must be 1'
    elif topic == '/mavros/rc/in':
        if len(message.channels) < 7:
            return 'RC requires at least 7 channels'
        if not all(900 <= message.channels[k] <= 1200 for k in (5, 6)):
            return 'CH6 released / CH7 off required'
    return None


def snapshot_details(rospy, topics, rows, max_age=None):
    """Only report data; never use diagnostics as a separate acceptance gate."""
    now = rospy.Time.now().to_sec()
    parts = ['ros_now=%.9f' % now]
    for (topic, _), message in zip(topics, rows):
        if message is None:
            parts.append(topic + ': missing')
            continue
        stamp = message.header.stamp.to_sec()
        age = now - stamp
        caller = (message._connection_header or {}).get('callerid')
        fields = ['age_s=%.9f' % age, 'stamp=%.9f' % stamp,
                  'callerid=' + repr(caller), 'source_ok=' + repr(caller == '/mavros')]
        if max_age is not None:
            fields.append('required=-0.02<=age_s<%s' % max_age)
            fields.append('fresh=' + repr(-.02 <= age < max_age))
        for key in ('connected', 'armed', 'system_status', 'mode', 'landed_state'):
            if hasattr(message, key):
                fields.append(key + '=' + repr(getattr(message, key)))
        if hasattr(message, 'channels'):
            fields.append('channels=' + repr(list(message.channels)))
        parts.append(topic + ': ' + ', '.join(fields))
    return '; '.join(parts)


def latest_messages(rospy, topics, timeout, max_age=None, reject=None, ready=None):
    """Wait within the original deadline for a simultaneously fresh snapshot.

    Existing callers still apply their original guards after return. Only stale
    or future timestamps may wait for replacement. An observed source/vehicle
    rejection is latched, so a later safe update cannot hide it while waiting.
    Omitting max_age/reject retains the original concurrent receive API.
    """
    rows = [None] * len(topics)
    condition = threading.Condition()
    subscribers = []
    failure = []

    def receive(index, message):
        with condition:
            rows[index] = message
            if reject is not None and not failure:
                reason = reject(topics[index][0], message)
                if reason:
                    failure.append((index, message, reason))
            condition.notify_all()

    try:
        for index, (topic, kind) in enumerate(topics):
            subscribers.append(rospy.Subscriber(
                topic, kind, lambda message, i=index: receive(i, message),
                queue_size=1))
        deadline = time.monotonic() + timeout
        with condition:
            while True:
                if rospy.is_shutdown():
                    raise rospy.ROSInterruptException('rospy shutdown')
                if failure:
                    index, message, reason = failure[0]
                    captured = list(rows)
                    captured[index] = message
                    raise ValueError('trusted ground snapshot rejected: ' +
                                     topics[index][0] + ': ' + reason + '; ' +
                                     snapshot_details(rospy, topics, captured, max_age))
                complete = all(row is not None for row in rows)
                remaining = deadline - time.monotonic()
                if complete and remaining > 0:
                    now = rospy.Time.now().to_sec() if max_age is not None else None
                    if max_age is None or all(
                            -.02 <= now - row.header.stamp.to_sec() < max_age
                            for row in rows):
                        if ready is None or ready(tuple(rows)):
                            return tuple(rows)
                if remaining <= 0:
                    if max_age is None:
                        missing = [topic for (topic, _), row in zip(topics, rows)
                                   if row is None]
                        raise rospy.ROSException('timeout waiting for messages: ' + ', '.join(missing))
                    raise rospy.ROSException('fresh trusted MAVROS required: wait timeout; ' +
                                             snapshot_details(rospy, topics, rows, max_age))
                condition.wait(min(remaining, 0.05))
    finally:
        for subscriber in subscribers:
            subscriber.unregister()


def await_mavros_startup(rospy, state_type, timeout=15):
    """New-process startup only; caller must still run all original guards.

    MAVROS advertises a latched disconnected State before its first heartbeat.
    Wait for connection in the existing startup budget; never waive source,
    arming or connected-state requirements and never restart an existing node.
    """
    def reject(topic, message):
        if (message._connection_header or {}).get('callerid') != '/mavros':
            return 'callerid must be /mavros'
        if message.armed:
            return 'armed=true'
        if message.connected:
            return ground_rejection(topic, message)
        return None
    try:
        return latest_messages(rospy, [('/mavros/state', state_type)],
                               timeout=timeout, max_age=2, reject=reject,
                               ready=lambda rows: rows[0].connected)[0]
    except rospy.ROSException as exc:
        raise rospy.ROSException('MAVROS startup connection not established; '
                                 'inspect heartbeat/framing diagnostics; ' + str(exc))

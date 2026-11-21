"""Explicit, disarmed-ground-only recovery from AUTO.LAND to ALTCTL."""
import math
import time

TOPICS = ('/mavros/state', '/mavros/extended_state', '/mavros/rc/in')
ACTIVE_CONTROL_TOPICS = (
    '/mavros/setpoint_raw/local',
    '/mavros/setpoint_position/local',
    '/mavros/setpoint_velocity/cmd_vel',
    '/mavros/setpoint_raw/attitude',
)
STARTUP_MODE_TIMEOUT_S = 6.


def refuse(reason):
    raise ValueError('PREPARE_CLASS=ground_mode_recovery_refused; ' + reason)


def validate_snapshot(rows, now):
    state, extended, rc = rows
    stamps = tuple(msg.header.stamp.to_sec() for msg in rows)
    if any((msg._connection_header or {}).get('callerid') != '/mavros'
           for msg in rows):
        refuse('untrusted_mavros_source')
    if not math.isfinite(now) or any(not math.isfinite(stamp) or
                                    not -.02 <= now - stamp < 2
                                    for stamp in stamps):
        refuse('stale_or_future_ground_feedback')
    if (not state.connected or state.armed or state.system_status != 3 or
            extended.landed_state != 1 or len(rc.channels) < 7 or
            any(not 900 <= rc.channels[k] <= 1200 for k in (5, 6))):
        refuse('connected_disarmed_landed_CH6_CH7_off_required')
    return {'mode': state.mode, 'stamps': stamps}


def validate_graph(pubs):
    if any(pubs.get(topic) != ['/mavros'] for topic in TOPICS):
        refuse('unique_mavros_telemetry_required')
    active = {topic: pubs[topic] for topic in ACTIVE_CONTROL_TOPICS + ('/cmd_vel',)
              if pubs.get(topic)}
    if active:
        refuse('control_publisher_present=' + repr(active))


def unresolved_mode(mode):
    """MAVROS may publish its first connected heartbeat before PX4 decoding."""
    return mode == '' or (mode.startswith('CMODE(') and mode.endswith(')') and
                          mode[6:-1].isdigit())


def ensure_startup_mode(read, graph, request, authorized=False,
                        sleep=time.sleep, clock=time.monotonic):
    """Read validates all three fresh ground messages on every call.

    The caller holds prepare's exclusive lock, after the old session stopped.
    Unknown startup names may wait, but never authorize readiness or a request.
    Even an existing ALTCTL needs two distinct state updates within six seconds.
    Only an explicit CLI flag permits a single ALTCTL request. Service ACK is
    insufficient: two distinct subsequent state messages must confirm ALTCTL.
    """
    def sample(timeout):
        validate_graph(graph())
        row = read(timeout)
        validate_graph(graph())
        return row

    startup_started = clock()
    startup_deadline = startup_started + STARTUP_MODE_TIMEOUT_S
    first = sample(3)
    initial = first
    previous_stamps = first['stamps']
    confirmations = int(first['mode'] == 'ALTCTL')
    def startup_timeout(row):
        raise ValueError('PREPARE_CLASS=ground_startup_mode_timeout; observed=' +
                         row['mode'] + '; required=two_distinct_ALTCTL_states; ' +
                         'confirmations=' + str(confirmations))

    while first['mode'] == 'ALTCTL' or unresolved_mode(first['mode']):
        remaining = startup_deadline - clock()
        if remaining <= 0:
            startup_timeout(first)
        if confirmations >= 2:
            return {'mode': 'ALTCTL', 'requested': False, 'evidence': first,
                    'initial_evidence': initial,
                    'distinct_ALTCTL_feedback': confirmations,
                    'startup_wait_s': clock() - startup_started}
        after = sample(min(2, remaining))
        if clock() >= startup_deadline:
            startup_timeout(after)
        if any(b < a for a, b in zip(previous_stamps, after['stamps'])):
            refuse('ground_timestamp_regressed')
        if confirmations and after['mode'] != 'ALTCTL':
            refuse('ALTCTL_not_stable; observed=' + after['mode'])
        if after['mode'] == 'ALTCTL' and after['stamps'][0] > previous_stamps[0]:
            confirmations += 1
        previous_stamps = after['stamps']
        first = after
        # Do not spin on a latched state, or accept late feedback as readiness.
        remaining = startup_deadline - clock()
        if remaining > 0 and confirmations < 2:
            sleep(min(.1, remaining))
    if not authorized:
        raise ValueError('PREPARE_CLASS=ground_startup_mode_required; observed=' +
                         first['mode'] + '; required=ALTCTL; ' +
                         'recovery_option=--recover-landed-mode')
    if first['mode'] != 'AUTO.LAND':
        refuse('only_AUTO.LAND_can_be_recovered; observed=' + first['mode'])

    started = clock()
    deadline = started + 6
    previous_stamps = first['stamps']
    while True:
        remaining = deadline - clock()
        if remaining <= 0:
            refuse('ground_proof_not_advancing')
        before = sample(min(2, remaining))
        if before['mode'] != 'AUTO.LAND':
            refuse('mode_changed_during_ground_proof; observed=' + before['mode'])
        if any(b < a for a, b in zip(previous_stamps, before['stamps'])):
            refuse('ground_timestamp_regressed')
        previous_stamps = before['stamps']
        if clock() - started >= 1 and all(b > a for a, b in
                                          zip(first['stamps'], before['stamps'])):
            break
        sleep(.1)

    validate_graph(graph())
    # One attempt only. A rejection or transport error must not become readiness.
    response = request()
    if not getattr(response, 'mode_sent', False):
        refuse('ALTCTL_request_rejected')
    deadline = clock() + 6
    last_stamp = before['stamps'][0]
    last_stamps = before['stamps']
    confirmations = 0
    while True:
        remaining = deadline - clock()
        if remaining <= 0:
            refuse('ALTCTL_feedback_timeout')
        after = sample(min(2, remaining))
        if any(b < a for a, b in zip(last_stamps, after['stamps'])):
            refuse('ground_timestamp_regressed')
        last_stamps = after['stamps']
        if after['mode'] not in ('AUTO.LAND', 'ALTCTL'):
            refuse('unexpected_mode_after_request; observed=' + after['mode'])
        if confirmations and after['mode'] != 'ALTCTL':
            refuse('ALTCTL_not_stable')
        stamp = after['stamps'][0]
        if stamp > last_stamp:
            last_stamp = stamp
            if after['mode'] == 'ALTCTL':
                confirmations += 1
                if confirmations >= 2:
                    return {'mode': 'ALTCTL', 'requested': True,
                            'previous_mode': 'AUTO.LAND',
                            'before': before, 'after': after,
                            'distinct_ALTCTL_feedback': confirmations}
        sleep(.1)


def prepare_startup_mode(rospy, master, authorized=False):
    from ground_snapshot import latest_messages, ground_rejection
    from mavros_msgs.msg import State, ExtendedState, RCIn
    from mavros_msgs.srv import SetMode

    topics = list(zip(TOPICS, (State, ExtendedState, RCIn)))

    def read(timeout):
        rows = latest_messages(rospy, topics, timeout=timeout, max_age=2,
                               reject=ground_rejection)
        return validate_snapshot(rows, rospy.Time.now().to_sec())

    def request():
        rospy.wait_for_service('/mavros/set_mode', timeout=2)
        # Refresh every requirement after service discovery, immediately before
        # the request. The aircraft must still be disarmed on the ground.
        current = read(2)
        pubs, _, services = master.getSystemState()
        validate_graph(dict(pubs))
        if current['mode'] != 'AUTO.LAND':
            refuse('mode_changed_before_request; observed=' + current['mode'])
        if dict(services).get('/mavros/set_mode') != ['/mavros']:
            refuse('unique_mavros_mode_service_required')
        return rospy.ServiceProxy('/mavros/set_mode', SetMode)(
            base_mode=0, custom_mode='ALTCTL')

    return ensure_startup_mode(read, lambda: dict(master.getSystemState()[0]),
                               request, authorized=authorized)

import sys
import types
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from prepare_mode import (ACTIVE_CONTROL_TOPICS, TOPICS, ensure_startup_mode,
                          prepare_startup_mode, validate_graph, validate_snapshot,
                          STARTUP_MODE_TIMEOUT_S)


def messages(now=10, mode='AUTO.LAND'):
    def message(**values):
        return NS(header=NS(stamp=NS(to_sec=lambda: now - .1)),
                  _connection_header={'callerid': '/mavros'}, **values)
    return (message(connected=True, armed=False, system_status=3, mode=mode),
            message(landed_state=1), message(channels=[1500]*5+[1050, 1050]))


def graph():
    return {topic: ['/mavros'] for topic in TOPICS}


class Scenario:
    def __init__(self, mode='AUTO.LAND'):
        self.t = 0
        self.mode = mode
        self.pubs = graph()
        self.requests = 0
        self.ack = True
        self.switch = True
        self.frozen = False
        self.reject_after = False
        self.read_count = 0

    def clock(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds

    def read(self, timeout):
        self.t += min(.6, timeout)
        self.read_count += 1
        if self.reject_after and self.requests:
            raise ValueError('unsafe ground feedback')
        return {'mode': self.mode,
                'stamps': (0, 0, 0) if self.frozen else (self.t,)*3}

    def request(self):
        self.requests += 1
        if self.switch:
            self.mode = 'ALTCTL'
        return NS(mode_sent=self.ack)

    def run(self, authorized=True):
        return ensure_startup_mode(self.read, lambda: self.pubs, self.request,
                                   authorized=authorized, clock=self.clock,
                                   sleep=self.sleep)


class ModePolicyTests(unittest.TestCase):
    def test_default_requires_explicit_authorization(self):
        sim = Scenario()
        with self.assertRaisesRegex(ValueError, 'ground_startup_mode_required'):
            sim.run(authorized=False)
        self.assertEqual(sim.requests, 0)

    def test_existing_ALTCTL_never_requests_mode(self):
        for authorized in (False, True):
            sim = Scenario('ALTCTL')
            self.assertFalse(sim.run(authorized)['requested'])
            self.assertEqual(sim.requests, 0)

    def test_only_AUTO_LAND_is_recoverable(self):
        for mode in ('OFFBOARD', 'AUTO.MISSION', 'POSCTL', 'MANUAL'):
            sim = Scenario(mode)
            with self.assertRaisesRegex(ValueError, 'only_AUTO.LAND'):
                sim.run()
            self.assertEqual(sim.requests, 0)

    def test_recovers_once_and_requires_two_distinct_state_updates(self):
        sim = Scenario()
        result = sim.run()
        self.assertEqual(sim.requests, 1)
        self.assertEqual(result['mode'], 'ALTCTL')
        self.assertEqual(result['distinct_ALTCTL_feedback'], 2)
        self.assertGreaterEqual(result['before']['stamps'][0] - .6, 1)

    def test_request_rejection_is_not_overridden_by_telemetry(self):
        sim = Scenario()
        sim.ack = False
        with self.assertRaisesRegex(ValueError, 'request_rejected'):
            sim.run()
        self.assertEqual(sim.requests, 1)

    def test_ack_without_ALTCTL_feedback_times_out_without_retry(self):
        sim = Scenario()
        sim.switch = False
        with self.assertRaisesRegex(ValueError, 'feedback_timeout'):
            sim.run()
        self.assertEqual(sim.requests, 1)

    def test_no_advancing_ground_feedback_prevents_request(self):
        sim = Scenario()
        sim.frozen = True
        with self.assertRaisesRegex(ValueError, 'proof_not_advancing'):
            sim.run()
        self.assertEqual(sim.requests, 0)

    def test_one_ALTCTL_frame_repeated_cannot_confirm(self):
        sim = Scenario()
        # Keep the original read callable, but freeze its state stamp after ACK.
        original_read = sim.read
        stamp = []
        def read(timeout):
            row = original_read(timeout)
            if sim.requests:
                if not stamp:
                    stamp.append(sim.t)
                row['stamps'] = (stamp[0],)*3
            return row
        with self.assertRaisesRegex(ValueError, 'feedback_timeout'):
            ensure_startup_mode(read, lambda: sim.pubs, sim.request, True,
                                clock=sim.clock, sleep=sim.sleep)
        self.assertEqual(sim.requests, 1)

    def test_unsafe_feedback_after_request_aborts(self):
        sim = Scenario()
        sim.reject_after = True
        with self.assertRaisesRegex(ValueError, 'unsafe ground'):
            sim.run()
        self.assertEqual(sim.requests, 1)

    def test_new_controller_appearing_during_proof_prevents_request(self):
        sim = Scenario()
        original = sim.read
        def read(timeout):
            row = original(timeout)
            if sim.read_count >= 2:
                sim.pubs['/mavros/setpoint_velocity/cmd_vel'] = ['/other']
            return row
        with self.assertRaisesRegex(ValueError, 'control_publisher_present'):
            ensure_startup_mode(read, lambda: sim.pubs, sim.request, True,
                                clock=sim.clock, sleep=sim.sleep)
        self.assertEqual(sim.requests, 0)

    def test_mode_change_during_proof_prevents_request(self):
        sim = Scenario()
        original = sim.read
        def read(timeout):
            row = original(timeout)
            if sim.read_count >= 2:
                row['mode'] = 'POSCTL'
            return row
        with self.assertRaisesRegex(ValueError, 'mode_changed_during_ground_proof'):
            ensure_startup_mode(read, lambda: sim.pubs, sim.request, True,
                                clock=sim.clock, sleep=sim.sleep)
        self.assertEqual(sim.requests, 0)


class StartupModeWaitTests(unittest.TestCase):
    def sequence(self, modes, authorized=False, transform=None):
        sim = Scenario(modes[0]);original = sim.read
        def read(timeout):
            sim.mode = modes[min(sim.read_count, len(modes)-1)]
            row = original(timeout)
            return transform(sim, row) if transform else row
        sim.read = read
        return sim, lambda: sim.run(authorized)

    def test_recorded_cmode_then_two_ALTCTL_frames_succeeds_without_request(self):
        for authorized in (False, True):
            sim, run = self.sequence(['CMODE(131072)', 'ALTCTL', 'ALTCTL'], authorized)
            out = run()
            self.assertEqual(sim.read_count, 3);self.assertEqual(sim.requests, 0)
            self.assertFalse(out['requested']);self.assertEqual(out['distinct_ALTCTL_feedback'], 2)
            self.assertEqual(out['initial_evidence']['mode'], 'CMODE(131072)')
            self.assertLess(out['startup_wait_s'], STARTUP_MODE_TIMEOUT_S)

    def test_existing_ALTCTL_also_requires_two_distinct_updates(self):
        sim, run = self.sequence(['ALTCTL'])
        self.assertEqual(run()['distinct_ALTCTL_feedback'], 2)
        self.assertEqual(sim.read_count, 2);self.assertEqual(sim.requests, 0)

    def test_other_unknown_numeric_names_and_empty_wait_without_being_accepted(self):
        for mode in ('', 'CMODE(393216)', 'CMODE(999999)'):
            sim, run = self.sequence([mode, mode, 'ALTCTL', 'ALTCTL'])
            self.assertEqual(run()['mode'], 'ALTCTL');self.assertEqual(sim.requests, 0)

    def test_unknown_name_persisting_has_bounded_timeout_and_never_requests_mode(self):
        for mode in ('', 'CMODE(131072)', 'CMODE(393216)'):
            for authorized in (False, True):
                sim, run = self.sequence([mode], authorized)
                with self.assertRaisesRegex(ValueError, 'ground_startup_mode_timeout'):run()
                self.assertLessEqual(sim.t, STARTUP_MODE_TIMEOUT_S)
                self.assertEqual(sim.requests, 0)

    def test_repeated_ALTCTL_stamp_cannot_supply_second_confirmation(self):
        def frozen(sim, row):
            if sim.read_count > 1:row['stamps'] = (1.3, 1.3, 1.3)
            return row
        sim, run = self.sequence(['CMODE(131072)', 'ALTCTL'], transform=frozen)
        with self.assertRaisesRegex(ValueError, 'ground_startup_mode_timeout'):run()
        self.assertEqual(sim.requests, 0)

    def test_same_stamp_with_changed_label_does_not_count_as_new_state(self):
        def stamp(sim, row):
            row['stamps'] = (1.,)*3 if sim.read_count < 3 else (float(sim.read_count),)*3
            return row
        sim, run = self.sequence(['CMODE(131072)', 'ALTCTL'], transform=stamp)
        self.assertEqual(run()['distinct_ALTCTL_feedback'], 2)
        self.assertEqual(sim.read_count, 4)

    def test_each_ground_timestamp_regression_is_rejected(self):
        for index in range(3):
            def regress(sim, row):
                if sim.read_count == 2:
                    stamps=list(row['stamps']);stamps[index]=.1;row['stamps']=tuple(stamps)
                return row
            sim, run = self.sequence(['CMODE(131072)', 'ALTCTL'], transform=regress)
            with self.assertRaisesRegex(ValueError, 'ground_timestamp_regressed'):run()
            self.assertEqual(sim.requests, 0)

    def test_ALTCTL_confirmation_cannot_hide_mode_change(self):
        for mode in ('CMODE(131072)', '', 'POSCTL', 'AUTO.LAND'):
            sim, run = self.sequence(['CMODE(131072)', 'ALTCTL', mode, 'ALTCTL'], True)
            with self.assertRaisesRegex(ValueError, 'ALTCTL_not_stable'):run()
            self.assertEqual(sim.requests, 0)

    def test_known_wrong_modes_are_not_retried_into_readiness(self):
        for mode in ('POSCTL', 'OFFBOARD', 'MANUAL', 'AUTO.MISSION', 'AUTO.LAND', 'CMODE(bad)'):
            sim, run = self.sequence(['CMODE(131072)', mode, 'ALTCTL'])
            with self.assertRaisesRegex(ValueError, 'ground_startup_mode_required'):run()
            self.assertEqual(sim.read_count, 2);self.assertEqual(sim.requests, 0)

    def test_authorized_recovery_remains_AUTO_LAND_only_after_unknown_name(self):
        sim, run = self.sequence(['CMODE(131072)', 'POSCTL'], True)
        with self.assertRaisesRegex(ValueError, 'only_AUTO.LAND'):run()
        self.assertEqual(sim.requests, 0)
        sim = Scenario();original=sim.read
        def read(timeout):
            row=original(timeout)
            if sim.read_count == 1:row['mode']='CMODE(100925440)'
            return row
        sim.read=read
        out=sim.run(True)
        self.assertTrue(out['requested']);self.assertEqual(sim.requests, 1)

    def test_ground_rejection_or_graph_change_during_wait_is_not_swallowed(self):
        def reject(sim, row):
            if sim.read_count == 2:raise ValueError('unsafe ground feedback')
            return row
        sim, run = self.sequence(['CMODE(131072)', 'ALTCTL'], transform=reject)
        with self.assertRaisesRegex(ValueError, 'unsafe ground'):run()
        self.assertEqual(sim.requests, 0)
        for topic in ('/cmd_vel',)+ACTIVE_CONTROL_TOPICS:
            def controller(sim, row):
                if sim.read_count == 2:sim.pubs[topic]=['/other']
                return row
            sim, run = self.sequence(['CMODE(131072)', 'ALTCTL'], transform=controller)
            with self.assertRaisesRegex(ValueError, 'control_publisher_present'):run()
            self.assertEqual(sim.requests, 0)

    def test_late_second_ALTCTL_does_not_pass_deadline(self):
        def late(sim, row):
            if sim.read_count == 2:sim.t=STARTUP_MODE_TIMEOUT_S
            return row
        sim, run = self.sequence(['ALTCTL'], transform=late)
        with self.assertRaisesRegex(ValueError, 'ground_startup_mode_timeout'):run()
        self.assertEqual(sim.requests, 0)


class SnapshotTests(unittest.TestCase):
    def test_original_ground_requirements_are_all_enforced(self):
        cases = [(0, 'connected', False), (0, 'armed', True),
                 (0, 'system_status', 4), (1, 'landed_state', 2),
                 (2, 'channels', [1500]*7), (2, 'channels', [1050]*6)]
        for index, key, value in cases:
            with self.subTest(key=key, value=value):
                rows = messages()
                setattr(rows[index], key, value)
                with self.assertRaisesRegex(ValueError, 'disarmed_landed'):
                    validate_snapshot(rows, 10)

    def test_stale_future_and_nonfinite_stamps_are_rejected(self):
        for stamp in (8, 10.03, float('nan'), float('inf')):
            rows = messages()
            rows[1].header.stamp.to_sec = lambda: stamp
            with self.assertRaisesRegex(ValueError, 'stale_or_future'):
                validate_snapshot(rows, 10)

    def test_foreign_caller_is_rejected_even_if_ground_fields_look_safe(self):
        rows = messages()
        rows[2]._connection_header = {'callerid': '/other'}
        with self.assertRaisesRegex(ValueError, 'untrusted_mavros'):
            validate_snapshot(rows, 10)

    def test_control_publishers_and_duplicate_mavros_are_rejected(self):
        for topic in ('/cmd_vel',) + ACTIVE_CONTROL_TOPICS:
            pubs = graph()
            pubs[topic] = ['/other']
            with self.assertRaisesRegex(ValueError, 'control_publisher'):
                validate_graph(pubs)
        pubs = graph()
        pubs[TOPICS[0]].append('/other')
        with self.assertRaisesRegex(ValueError, 'unique_mavros'):
            validate_graph(pubs)

    def test_mavros_setpoint_echo_topics_are_not_control_commands(self):
        pubs = graph()
        pubs.update({
            '/mavros/setpoint_raw/target_local': ['/mavros'],
            '/mavros/setpoint_raw/target_global': ['/mavros'],
            '/mavros/setpoint_raw/target_attitude': ['/mavros'],
            '/mavros/setpoint_trajectory/desired': ['/mavros'],
        })
        validate_graph(pubs)


class RosAdapterTests(unittest.TestCase):
    def startup_adapter(self, unsafe=None):
        fake_msg = types.ModuleType('mavros_msgs.msg')
        fake_msg.State = fake_msg.ExtendedState = fake_msg.RCIn = object
        fake_srv = types.ModuleType('mavros_msgs.srv');fake_srv.SetMode = object
        fake_ground = types.ModuleType('ground_snapshot');clock=[0.];reads=[]
        def latest(*args, **kwargs):
            clock[0] += .6;reads.append(kwargs)
            rows=messages(clock[0], 'CMODE(131072)' if len(reads)==1 else 'ALTCTL')
            if unsafe and len(reads)==2:unsafe(rows,clock[0])
            return rows
        fake_ground.latest_messages=latest;fake_ground.ground_rejection=lambda *args:None
        rospy=NS(Time=NS(now=lambda:NS(to_sec=lambda:clock[0])),
                 wait_for_service=Mock(),ServiceProxy=Mock())
        master=NS(getSystemState=lambda:(list(graph().items()),[],[]))
        modules={'mavros_msgs.msg':fake_msg,'mavros_msgs.srv':fake_srv,'ground_snapshot':fake_ground}
        original=ensure_startup_mode
        def fast(*args, **kwargs):
            return original(*args,**kwargs,clock=lambda:clock[0],
                            sleep=lambda dt:clock.__setitem__(0,clock[0]+dt))
        return rospy,master,modules,reads,fast

    def test_startup_adapter_waits_for_named_mode_without_discovering_mode_service(self):
        rospy,master,modules,reads,fast=self.startup_adapter()
        with patch.dict(sys.modules,modules),patch('prepare_mode.ensure_startup_mode',side_effect=fast):
            out=prepare_startup_mode(rospy,master)
        self.assertEqual(out['distinct_ALTCTL_feedback'],2);self.assertEqual(len(reads),3)
        self.assertTrue(all(x['max_age']==2 and x['reject'] is not None for x in reads))
        rospy.wait_for_service.assert_not_called();rospy.ServiceProxy.assert_not_called()

    def test_startup_adapter_revalidates_all_ground_fields_and_stamps_during_wait(self):
        cases=[lambda rows,now:setattr(rows[0],'connected',False),
               lambda rows,now:setattr(rows[0],'armed',True),
               lambda rows,now:setattr(rows[0],'system_status',4),
               lambda rows,now:setattr(rows[1],'landed_state',2),
               lambda rows,now:setattr(rows[2],'channels',[1500]*7),
               lambda rows,now:setattr(rows[2],'_connection_header',{'callerid':'/other'}),
               lambda rows,now:setattr(rows[1].header.stamp,'to_sec',lambda:now-2.),
               lambda rows,now:setattr(rows[1].header.stamp,'to_sec',lambda:now+.03)]
        for unsafe in cases:
            rospy,master,modules,reads,fast=self.startup_adapter(unsafe)
            with patch.dict(sys.modules,modules),patch('prepare_mode.ensure_startup_mode',side_effect=fast):
                with self.assertRaises(ValueError):prepare_startup_mode(rospy,master)
            self.assertEqual(len(reads),2)
            rospy.wait_for_service.assert_not_called();rospy.ServiceProxy.assert_not_called()

    def adapter(self, services=None, change_before_request=False):
        fake_msg = types.ModuleType('mavros_msgs.msg')
        fake_msg.State = fake_msg.ExtendedState = fake_msg.RCIn = object
        fake_srv = types.ModuleType('mavros_msgs.srv')
        fake_srv.SetMode = object
        fake_ground = types.ModuleType('ground_snapshot')
        clock = [0]
        requested = []
        discovered = [False]
        def latest(*args, **kwargs):
            clock[0] += .6
            rows = messages(clock[0], 'ALTCTL' if requested else 'AUTO.LAND')
            if change_before_request and discovered[0]:
                rows[0].armed = True
            return rows
        fake_ground.latest_messages = latest
        fake_ground.ground_rejection = lambda *args: None
        def call(**kwargs):
            requested.append(kwargs)
            return NS(mode_sent=True)
        rospy = NS(Time=NS(now=lambda: NS(to_sec=lambda: clock[0])),
                   wait_for_service=lambda *a, **k: discovered.__setitem__(0, True),
                   ServiceProxy=Mock(return_value=call))
        master = NS(getSystemState=lambda: (list(graph().items()), [], services))
        return rospy, master, requested, {
            'mavros_msgs.msg': fake_msg, 'mavros_msgs.srv': fake_srv,
            'ground_snapshot': fake_ground}

    def test_adapter_requests_only_ALTCTL_with_explicit_flag(self):
        rospy, master, calls, modules = self.adapter([('/mavros/set_mode', ['/mavros'])])
        original = ensure_startup_mode
        def fast(*args, **kwargs):
            sim = Scenario()
            return original(*args, **kwargs, clock=sim.clock,
                            sleep=lambda seconds: setattr(sim, 't', sim.t + 1))
        with patch.dict(sys.modules, modules), patch('prepare_mode.ensure_startup_mode', side_effect=fast):
            result = prepare_startup_mode(rospy, master, True)
        self.assertTrue(result['requested'])
        self.assertEqual(calls, [{'base_mode': 0, 'custom_mode': 'ALTCTL'}])

    def test_service_owner_and_ground_are_rechecked_after_discovery(self):
        for foreign, unsafe in ((True, False), (False, True)):
            services = [('/mavros/set_mode', ['/other' if foreign else '/mavros'])]
            rospy, master, calls, modules = self.adapter(services, unsafe)
            original = ensure_startup_mode
            def fast(*args, **kwargs):
                sim = Scenario()
                return original(*args, **kwargs, clock=sim.clock,
                                sleep=lambda seconds: setattr(sim, 't', sim.t + 1))
            with patch.dict(sys.modules, modules), patch('prepare_mode.ensure_startup_mode', side_effect=fast):
                with self.assertRaises(ValueError):
                    prepare_startup_mode(rospy, master, True)
            self.assertEqual(calls, [])


class EntryCliTests(unittest.TestCase):
    def test_flag_is_opt_in_and_passes_through_target_selection(self):
        import competition
        for explicit in (False, True):
            args = ['competition.py', 'prepare', '--targets', '2', '3',
                    '--current-flight-area-confirmed']
            if explicit:
                args.append('--recover-landed-mode')
            with patch.object(sys, 'argv', args), patch.object(competition, 'prepare') as prepare, \
                    patch.object(competition, 'configure_ros_package_path'):
                self.assertEqual(competition.main(), 0)
            prepare.assert_called_once_with([2, 3], True, recover_landed_mode=explicit)


if __name__ == '__main__':
    unittest.main()

"""Bounded-step and inherited hold transport checks; no ROS network or flight."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import competition_align as app
import test_competition_hold as hf


def config(yaw=.2):
    return dict(enable_motion=False,enable_traverse=False,local_frame='map',body_frame='base_link',
        optical_frame='camera_color_optical_frame',window_center_local_m=[1.7,0.,1.],window_yaw_rad=yaw,
        envelope_center_body_m=[0.,0.,-.09],camera_translation_body_m=[.08,0.,-.06],
        camera_rotation_body_from_optical_xyzw=[-.5,.5,-.5,.5],camera_info_sha256='a'*64,
        vision_profile_sha256='b'*64,algorithm_sha256='c'*64)


def status(stamp, lateral=-.0987, distance=1.675):
    return dict(state='OBSERVE',enable_motion=False,actual_output_permitted=False,graph_ok=True,
        confirmed=True,output_sent_count=0,measurement=dict(capture_stamp=stamp,lateral_error_m=lateral,
        vertical_error_m=-.0045,distance_m=distance))


class Fixture:
    def __init__(self, enabled=True, yaw=.2, lateral=-.0987):
        self.c = app.AlignCore(enabled,config(yaw))
        self.r,self.w,self.lateral,self.yaw = 100.,10.,lateral,yaw
        self.origin = (3.5,-2.,.6)
        self.feed(armed=False,landed=1)

    def feed(self,dt=.05,armed=True,mode='POSCTL',landed=2,p=None,geometry=True,send=True,
             graph=True,estimator=True,omit=()):
        self.r += dt; self.w += dt
        for name,value in dict(state=dict(connected=True,armed=armed,mode=mode),extended=landed,
                pose=hf.pose(p or self.origin,yaw=self.yaw),estimator=estimator).items():
            if name not in omit:
                self.c.update(name,self.r,value,self.r,self.w)
        if geometry:
            self.c.geometry_update(status(self.r,self.lateral),self.r,self.w)
        if graph:
            self.c.graph_update('',self.w)
        self.c.tick(self.r,self.w)
        if send and self.c.active:
            self.c.output_sent(self.w)
        return self

    def ready(self):
        for _ in range(26): self.feed()
        return self

    def capture(self):
        self.ready()
        ok,why=self.c.capture(self.r,self.w)
        if not ok: raise AssertionError(why)
        return self

    def holding(self):
        self.capture()
        for _ in range(36): self.feed()
        self.feed(mode='OFFBOARD')
        if self.c.phase != 'HOLDING': raise AssertionError(self.c.reason)
        return self


class CoreTests(unittest.TestCase):
    def test_default_has_no_output_and_no_ground_capture(self):
        for enabled in (False,True):
            f=Fixture(enabled)
            self.assertFalse(f.c.capture(f.r,f.w)[0])
            self.assertFalse(f.c.active)

    def test_capture_requires_fresh_confirmed_original_geometry(self):
        for change in ('stale','wrong_source','rejected','unconfirmed','output'):
            f=Fixture().ready(); s=status(f.r+.001)
            source=app.OBSERVER
            if change=='stale': s['measurement']['capture_stamp']=f.r-1
            if change=='wrong_source': source='/other'
            if change=='rejected': s['measurement']={'measurement_rejected':'survey_visual_disagreement'}
            if change=='unconfirmed': s['confirmed']=False
            if change=='output': s['output_sent_count']=1
            f.c.geometry_update(s,f.r,f.w,source)
            with self.subTest(change=change): self.assertFalse(f.c.capture(f.r,f.w)[0])

    def test_both_step_signs_and_yaw_keep_height_normal_fixed(self):
        for yaw in (0.,.11981015056970282,math.pi/2,-1.0):
            for lateral in (-.18,.18,-.0987):
                f=Fixture(yaw=yaw,lateral=lateral).capture()
                displacement=tuple(a-b for a,b in zip(f.c.goal,f.c.origin))
                with self.subTest(yaw=yaw,lateral=lateral):
                    self.assertAlmostEqual(app.dot(displacement,f.c.normal),0.,places=12)
                    self.assertEqual(f.c.goal[2],f.c.origin[2])
                    self.assertLessEqual(app.hold.norm(displacement),app.MAX_STEP+1e-12)
                    self.assertAlmostEqual(app.dot(displacement,f.c.lateral),max(-.1,min(.1,lateral)))
                    self.assertEqual(f.c.target,f.c.origin)
                    self.assertEqual(f.c.target_yaw,yaw)

    def test_near_window_or_excessive_error_refused(self):
        for distance,lateral in ((.8,-.1),(3.,-.1),(1.6,-.3),(1.6,.01)):
            f=Fixture().ready()
            f.c.geometry_update(status(f.r+.001,lateral,distance),f.r,f.w)
            self.assertFalse(f.c.capture(f.r,f.w)[0])

    def test_no_lateral_progress_in_prestream(self):
        f=Fixture().capture()
        for _ in range(36): f.feed()
        self.assertTrue(f.c.prestream_ready(f.w))
        self.assertEqual(f.c.progress_m,0.)
        self.assertEqual(f.c.target,f.c.origin)

    def test_rate_bounded_motion_and_completion_remain_holding(self):
        f=Fixture().holding()
        for _ in range(100):
            previous=f.c.target
            f.feed(mode='OFFBOARD',p=previous)
            self.assertLessEqual(app.hold.distance(previous,f.c.target),app.STEP_SPEED*.05+1e-12)
            self.assertEqual(f.c.target[2],f.c.origin[2])
        self.assertEqual(f.c.step_state,'COMPLETE_HOLD')
        self.assertTrue(f.c.active)
        self.assertEqual(f.c.target,f.c.goal)
        self.assertIsNotNone(f.c.completion_position)
        f.feed(mode='POSCTL',p=f.c.target)
        self.assertEqual(f.c.snapshot(f.r,f.w)['step_result'],'COMPLETE_HOLD')
        self.assertFalse(f.c.capture(f.r,f.w)[0])

    def test_no_tracking_freezes_at_lead_then_holds_after_timeout(self):
        f=Fixture().holding()
        for _ in range(180): f.feed(mode='OFFBOARD')
        self.assertLessEqual(app.hold.distance(f.c.target,f.c.origin),app.MAX_LEAD+1e-12)
        self.assertEqual(f.c.step_state,'FROZEN_HOLD')
        self.assertTrue(f.c.active)

    def test_later_visual_loss_does_not_cut_or_retarget_frozen_plan(self):
        f=Fixture().holding(); goal=f.c.goal
        f.c.geometry_update({},f.r,f.w)
        for _ in range(100): f.feed(mode='OFFBOARD',p=f.c.target,geometry=False)
        self.assertTrue(f.c.active)
        self.assertEqual(f.c.goal,goal)
        self.assertEqual(f.c.step_state,'COMPLETE_HOLD')

    def test_manual_takeover_disarm_estimator_loss_stop(self):
        cases=[dict(mode='POSCTL'),dict(mode='OFFBOARD',armed=False),
               dict(mode='OFFBOARD',estimator=False),dict(mode='OFFBOARD',landed=1)]
        for args in cases:
            f=Fixture().holding(); f.feed(**args)
            self.assertFalse(f.c.active)
            self.assertFalse(f.c.capture(f.r,f.w)[0])

    def test_timing_and_tracking_faults_stop(self):
        for fault in ('normal','height','tracking','gap','pose_stale'):
            f=Fixture().holding()
            if fault=='gap': f.feed(dt=.3,mode='OFFBOARD')
            elif fault=='pose_stale':
                for _ in range(8): f.feed(mode='OFFBOARD',omit=('pose',))
            else:
                d={'normal':tuple(.11*x for x in f.c.normal),'height':(0.,0.,.11),
                   'tracking':tuple(.15*x for x in f.c.lateral)}[fault]
                f.feed(mode='OFFBOARD',p=tuple(p+x for p,x in zip(f.origin,d)))
            with self.subTest(fault=fault): self.assertFalse(f.c.active)

    def test_prestream_expiry_does_not_retry(self):
        f=Fixture().capture()
        for _ in range(310): f.feed()
        self.assertEqual(f.c.reason,'PRESTREAM_EXPIRED_NO_OFFBOARD_OBSERVED')
        self.assertFalse(f.c.capture(f.r,f.w)[0])


class TransportTests(unittest.TestCase):
    def make(self,enabled=True,edit=None):
        r,g=hf.Ros(),hf.Graph(); cfg=config(0.)
        r.params[app.CONFIG_PARAM]=copy.deepcopy(cfg)
        g.pubs[app.OBSERVATION]=[app.OBSERVER]
        if edit: edit(r,g)
        n=app.AlignNode(r,g,hf.TYPES,cfg,enabled,lambda:r.w)
        self.addCleanup(n.close)
        self.feed(n,r,armed=False)
        return n,r,g

    def feed(self,n,r,armed=True,mode='POSCTL',p=None):
        r.r+=.05; r.w+=.05
        for key,msg in hf.messages(r.r,armed,mode,2 if armed else 1,p or (3.5,-2.,.6)).items():
            n.receive(key,msg)
        n.receive_geometry(NS(data=json.dumps(status(r.r)),_connection_header=dict(callerid=app.OBSERVER)))
        n.core.graph_update('',r.w)
        n.once()

    def test_only_align_status_at_start_no_control_publisher(self):
        n,r,g=self.make()
        self.assertIn('/window_pass_align/status',r.pubs)
        self.assertNotIn(app.hold.STATUS,r.pubs)
        self.assertNotIn(app.hold.OUTPUT,r.pubs)
        self.assertFalse(n.service('capture').success)

    def test_legacy_hold_and_wrong_profile_refused(self):
        for edit in (lambda r,g:g.pubs.update({app.hold.STATUS:['/old_hold']}),
                     lambda r,g:r.params[app.CONFIG_PARAM].update(window_yaw_rad=1.)):
            with self.assertRaises(ValueError): self.make(edit=edit)

    def test_mavros_frame_mask_and_no_control_until_capture(self):
        n,r,g=self.make()
        for _ in range(26): self.feed(n,r)
        self.assertNotIn(app.hold.OUTPUT,r.pubs)
        self.assertTrue(n.service('capture').success)
        for _ in range(36): self.feed(n,r)
        for _ in range(90): self.feed(n,r,mode='OFFBOARD',p=n.core.target)
        self.assertEqual(n.core.step_state,'COMPLETE_HOLD')
        out=r.pubs[app.hold.OUTPUT].messages
        self.assertGreater(len(out),100)
        for m in out:
            self.assertEqual(m.coordinate_frame,m.FRAME_LOCAL_NED)
            self.assertEqual(m.type_mask,2552)
            self.assertEqual(m.position.z,.6)
            self.assertEqual(m.yaw,0.)
            self.assertAlmostEqual(m.position.x,3.5)
        self.feed(n,r,mode='POSCTL',p=n.core.target)
        self.assertIsNone(n.pub)
        self.assertTrue(r.pubs[app.hold.OUTPUT].closed)

    def test_config_change_and_competing_publisher_fail_closed(self):
        n,r,g=self.make()
        r.params[app.CONFIG_PARAM]['window_yaw_rad']=1.
        self.assertEqual(n.graph_problem(g.getSystemState()),'OBSERVE_GEOMETRY_CONFIGURATION_CHANGED')
        r.params[app.CONFIG_PARAM]['window_yaw_rad']=0.
        g.pubs[app.hold.OUTPUT]=['/foreign']
        self.assertTrue(n.graph_problem(g.getSystemState()).startswith('COMPETING_CONTROL:'))

    def test_observe_without_cli_cannot_capture(self):
        n,r,g=self.make(False)
        for _ in range(26): self.feed(n,r)
        self.assertFalse(n.service('capture').success)
        self.assertNotIn(app.hold.OUTPUT,r.pubs)

class CaptureHelperTests(unittest.TestCase):
    def run_attempt(self,prestream,initial=None):
        sys.path.insert(0,str(ROOT/'tools'))
        import align_capture_once as helper
        healthy=dict(version='20261012_v1',entry='BOUNDED_LATERAL_STEP_ONLY',input_ready=True,
            connected=True,armed=True,airborne=True,output_enabled=True,startup_disarmed_seen=True,
            graph_fresh=True,stop_reason='',graph_error='',mode='POSCTL',phase='OBSERVE',capture_ready=True)
        if initial: healthy.update(initial)
        states=[healthy]+[dict(healthy,**s) for s in prestream]
        clock=[0.]; calls=[]; output=[]
        def read(seconds):
            clock[0]+=.05
            return states.pop(0) if states else dict(healthy,phase='STOPPED',stop_reason='expired')
        def capture():
            calls.append('capture')
            return NS(success=True,message='planned')
        try:
            result=helper.attempt(read,capture,clock=lambda:clock[0],emit=output.append)
            return result,calls,output
        except RuntimeError as exc:
            return exc,calls,output

    def test_first_send_wait_and_single_capture(self):
        result,calls,output=self.run_attempt([
            dict(phase='PRESTREAM',prestream_ready=False,output_active=False,output_sent_count=0),
            dict(phase='PRESTREAM',prestream_ready=True,output_active=True,planned_lateral_step_m=-.0987)])
        self.assertIsInstance(result,dict)
        self.assertEqual(calls,['capture'])
        self.assertTrue(any(x.startswith('A_READY:') for x in output))

    def test_mode_change_or_stopped_never_prints_ready(self):
        for update in (dict(phase='STOPPED'),dict(phase='PRESTREAM',mode='AUTO.LAND'),
                       dict(phase='PRESTREAM',output_active=False,output_sent_count=10)):
            result,calls,output=self.run_attempt([update])
            self.assertIsInstance(result,RuntimeError)
            self.assertEqual(calls,['capture'])
            self.assertFalse(any(x.startswith('A_READY:') for x in output))

    def test_ground_or_disabled_never_requests_capture(self):
        for initial in (dict(armed=False),dict(airborne=False),dict(output_enabled=False)):
            result,calls,output=self.run_attempt([],initial)
            self.assertIsInstance(result,RuntimeError)
            self.assertEqual(calls,[])


class CaptureTimeoutTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, str(ROOT/'tools'))
        import align_capture_once
        self.helper = align_capture_once
        self.healthy = dict(version='20261012_v1', entry='BOUNDED_LATERAL_STEP_ONLY',
            input_ready=True, connected=True, armed=True, airborne=True,
            output_enabled=True, startup_disarmed_seen=True, graph_fresh=True,
            stop_reason='', graph_error='', mode='POSCTL', phase='OBSERVE',
            capture_ready=True)
        self.clock = [0.]
        self.calls, self.output = [], []

    def capture(self):
        self.calls.append('capture')
        return NS(success=True, message='planned')

    def attempt(self, read):
        return self.helper.attempt(read, self.capture,
            clock=lambda: self.clock[0], emit=self.output.append)

    def test_geometry_unready_keeps_reason_and_never_captures(self):
        reason = 'FRESH_CONFIRMED_OBSERVE_GEOMETRY_REQUIRED'
        def read(seconds):
            self.clock[0] += .10
            return dict(self.healthy, capture_ready=False,
                        capture_reason=reason, geometry_reason=reason)
        with self.assertRaisesRegex(RuntimeError, 'CAPTURE_NOT_READY: ' + reason):
            self.attempt(read)
        self.assertEqual(self.calls, [])
        self.assertFalse(any(x.startswith('A_READY:') for x in self.output))

    def test_initial_status_missing_never_captures(self):
        def read(seconds):
            self.clock[0] += seconds
            raise self.helper.StatusTimeout('received_count=0')
        with self.assertRaisesRegex(RuntimeError, 'ALIGN_STATUS_NOT_RECEIVED'):
            self.attempt(read)
        self.assertEqual(self.calls, [])

    def test_status_interruption_keeps_last_geometry_reason(self):
        states = [dict(self.healthy, capture_ready=False,
                       capture_reason='LATERAL_ERROR_NOT_STABLE')]
        def read(seconds):
            if states:
                self.clock[0] += .1
                return states.pop(0)
            self.clock[0] += seconds
            raise self.helper.StatusTimeout('received_count=1')
        with self.assertRaisesRegex(RuntimeError,
                                    'CAPTURE_NOT_READY: LATERAL_ERROR_NOT_STABLE'):
            self.attempt(read)
        self.assertEqual(self.calls, [])

    def test_status_lost_after_capture_never_announces_a_ready(self):
        states = [self.healthy]
        def read(seconds):
            if states:
                self.clock[0] += .1
                return states.pop(0)
            self.clock[0] += seconds
            raise self.helper.StatusTimeout('last_phase=OBSERVE')
        with self.assertRaisesRegex(RuntimeError, 'PRESTREAM_STATUS_TIMEOUT_AFTER_CAPTURE'):
            self.attempt(read)
        self.assertEqual(self.calls, ['capture'])
        self.assertTrue(any(x.startswith('CAPTURE_OK:') for x in self.output))
        self.assertFalse(any(x.startswith('A_READY:') for x in self.output))

    def test_status_first_arriving_at_deadline_cannot_capture(self):
        def read(seconds):
            self.clock[0] += seconds
            return dict(self.healthy)
        with self.assertRaisesRegex(RuntimeError, 'CAPTURE_NOT_READY'):
            self.attempt(read)
        self.assertEqual(self.calls, [])

    def test_capture_rejection_is_not_retried(self):
        def read(seconds):
            self.clock[0] += .1
            return dict(self.healthy)
        def reject():
            self.calls.append('capture')
            return NS(success=False, message='GEOMETRY_CHANGED')
        with self.assertRaisesRegex(RuntimeError, 'CAPTURE_REJECTED: GEOMETRY_CHANGED'):
            self.helper.attempt(read, reject, clock=lambda: self.clock[0], emit=self.output.append)
        self.assertEqual(self.calls, ['capture'])
        self.assertEqual(self.output, [])


class StatusReaderTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, str(ROOT/'tools'))
        import align_capture_once
        self.helper = align_capture_once
        self.clock = [1.]
        self.subscriptions, self.unregistrations = [], []
        def subscribe(topic, message_type, callback, **kwargs):
            self.subscriptions.append((topic, callback, kwargs))
            return NS(unregister=lambda: self.unregistrations.append(topic))
        self.ros = NS(Subscriber=subscribe, is_shutdown=lambda: False)
        self.reader = self.helper.StatusReader(self.ros, NS, clock=lambda: self.clock[0])
        fixture = self
        class Condition:
            def __enter__(self): return self
            def __exit__(self, *args): return False
            def notify_all(self): pass
            def wait(self, seconds):
                fixture.clock[0] += seconds
                if fixture.on_wait:
                    action, fixture.on_wait = fixture.on_wait, None
                    action()
        self.reader.condition = Condition()
        self.on_wait = None

    def send(self, data, source='/window_pass_align_123'):
        self.reader.receive(NS(data=json.dumps(data), _connection_header={'callerid': source}))

    def tearDown(self):
        self.reader.close()

    def test_one_subscription_distinct_receipts_and_idempotent_close(self):
        self.send(dict(phase='OBSERVE', capture_ready=False))
        self.assertFalse(self.reader.read(.2)['capture_ready'])
        with self.assertRaises(self.helper.StatusTimeout):
            self.reader.read(.1)
        self.send(dict(phase='OBSERVE', capture_ready=True))
        self.assertTrue(self.reader.read(.2)['capture_ready'])
        self.assertEqual(len(self.subscriptions), 1)
        self.assertEqual(self.reader.sequence, 2)
        self.reader.close()
        self.reader.close()
        self.assertEqual(self.unregistrations, [self.helper.STATUS])

    def test_stale_cached_status_is_not_used(self):
        self.send(dict(phase='OBSERVE', capture_ready=True,
                       capture_reason='EXPIRED', geometry_reason='EXPIRED'))
        self.clock[0] += .31
        with self.assertRaises(self.helper.StatusTimeout) as caught:
            self.reader.read(.2)
        detail = json.loads(str(caught.exception).split(': ', 1)[1])
        self.assertEqual(detail['received_count'], 1)
        self.assertEqual(detail['last_geometry_reason'], 'EXPIRED')
        self.assertGreater(detail['last_received_age_s'], .30)
        self.assertEqual(self.reader.consumed, 0)

    def test_wait_can_receive_fresh_status_after_stale_cache(self):
        self.send(dict(capture_ready=False))
        self.clock[0] += .31
        self.on_wait = lambda: self.send(dict(capture_ready=True))
        self.assertTrue(self.reader.read(.2)['capture_ready'])
        self.assertEqual(len(self.subscriptions), 1)

    def test_missing_status_has_receipt_count_zero(self):
        with self.assertRaises(self.helper.StatusTimeout) as caught:
            self.reader.read(.2)
        detail = json.loads(str(caught.exception).split(': ', 1)[1])
        self.assertEqual(detail['received_count'], 0)
        self.assertIsNone(detail['last_received_age_s'])

    def test_bad_publisher_and_nonobject_json_fail_closed(self):
        for data, source in ((dict(capture_ready=True), '/foreign'),
                             ([], '/window_pass_align_123')):
            with self.subTest(source=source, data=data):
                self.reader.error = ''
                self.send(data, source)
                with self.assertRaisesRegex(RuntimeError, 'INVALID_ALIGN_STATUS'):
                    self.reader.read(.2)
                self.assertEqual(self.reader.sequence, 0)

    def test_invalid_json_fails_closed(self):
        self.reader.receive(NS(data='not json',
            _connection_header={'callerid': '/window_pass_align_123'}))
        with self.assertRaisesRegex(RuntimeError, 'INVALID_ALIGN_STATUS'):
            self.reader.read(.2)

    def test_close_or_ros_shutdown_prevents_cached_readiness(self):
        self.send(dict(capture_ready=True))
        self.ros.is_shutdown = lambda: True
        with self.assertRaisesRegex(RuntimeError, 'ALIGN_STATUS_READER_CLOSED'):
            self.reader.read(.2)
        self.ros.is_shutdown = lambda: False
        self.reader.close()
        with self.assertRaisesRegex(RuntimeError, 'ALIGN_STATUS_READER_CLOSED'):
            self.reader.read(.2)


class InputDiagnosticTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, str(ROOT/'tools'))
        import diagnose_align_inputs
        self.app = diagnose_align_inputs
        self.clock = [10.]
        self.snapshot = self.app.Snapshot(clock=lambda: self.clock[0])

    def feed_state(self, armed=False, connected=True, landed=1):
        self.snapshot.receive('state', NS(connected=connected, armed=armed,
            mode='POSCTL', system_status=3))
        self.snapshot.receive('extended', NS(landed_state=landed))

    def test_ground_report_requires_fresh_disarmed_landed_telemetry(self):
        self.feed_state()
        self.assertTrue(self.snapshot.report()['fresh_disarmed_on_ground_observed'])
        self.clock[0] += 1.51
        self.assertFalse(self.snapshot.report()['fresh_disarmed_on_ground_observed'])

    def test_airborne_armed_or_disconnected_are_not_ground_confirmation(self):
        for update in (dict(armed=True), dict(landed=2), dict(connected=False)):
            with self.subTest(update=update):
                self.feed_state(**update)
                self.assertFalse(self.snapshot.report()['fresh_disarmed_on_ground_observed'])

    def test_missing_state_is_reported_without_starting_ros(self):
        data = self.snapshot.report()
        self.assertTrue(data['read_only'])
        self.assertFalse(data['controls_vehicle'])
        self.assertFalse(data['fresh_disarmed_on_ground_observed'])
        self.assertEqual(data['topics']['state']['received_count'], 0)
        self.assertIsNone(data['topics']['state']['last_receipt_age_s'])

    def test_geometry_rejection_receipts_and_publisher_are_preserved(self):
        self.snapshot.receive('observer', NS(data=json.dumps(dict(state='OBSERVE',
            confirmed=False, hold_pose_fresh=True, graph_ok=True,
            measurement={'measurement_rejected': 'survey_visual_disagreement'})),
            _connection_header={'callerid': '/window_pass_controller'}))
        self.clock[0] += .1
        row = self.snapshot.report()['topics']['observer']
        self.assertEqual(row['received_count'], 1)
        self.assertEqual(row['decoded_count'], 1)
        self.assertEqual(row['publishers'], {'/window_pass_controller': 1})
        self.assertEqual(row['reason_counts'], {'survey_visual_disagreement': 1})
        self.assertFalse(row['latest']['confirmed'])
        self.assertEqual(row['last_decoded_age_s'], .1)

    def test_detector_no_target_is_separate_from_observer_pose_state(self):
        self.snapshot.receive('detector', NS(valid=False, target_count=0, reason='no_target',
            calibration_valid=False, detection_method='', geometry_kind='', processing_ms=50.))
        row = self.snapshot.report()['topics']['detector']
        self.assertEqual(row['reason_counts'], {'no_target': 1})
        self.assertFalse(row['latest']['valid'])

    def test_bad_json_is_reported_without_refreshing_decoded_status(self):
        self.snapshot.receive('align', NS(data=json.dumps(dict(phase='OBSERVE'))))
        self.clock[0] += 2.
        self.snapshot.receive('align', NS(data='invalid json'))
        row = self.snapshot.report()['topics']['align']
        self.assertEqual(row['received_count'], 2)
        self.assertEqual(row['decoded_count'], 1)
        self.assertEqual(row['latest']['phase'], 'OBSERVE')
        self.assertEqual(row['last_receipt_age_s'], 0.)
        self.assertEqual(row['last_decoded_age_s'], 2.)
        self.assertTrue(row['decode_errors'])

    def test_offline_check_and_python38_syntax_without_control_calls(self):
        import ast
        import subprocess
        path = ROOT/'tools/diagnose_align_inputs.py'
        text = path.read_text(encoding='utf-8-sig')
        ast.parse(text, feature_version=(3,8))
        self.assertNotIn('ServiceProxy', text)
        self.assertNotIn('Publisher(', text)
        self.assertNotIn('/mavros/set_mode', text)
        self.assertNotIn('/mavros/cmd/arming', text)
        result = subprocess.run([sys.executable, str(path), '--check'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout)
        self.assertFalse(data['ROS_started'])
        self.assertTrue(data['read_only'])


class OfflineTests(unittest.TestCase):
    def test_python38_and_no_runtime_mode_or_arming_service(self):
        import ast
        for path in (ROOT/'scripts/competition_align.py', ROOT/'tools/align_capture_once.py'):
            ast.parse(path.read_text(encoding='utf-8-sig'), feature_version=(3,8))
        text=(ROOT/'scripts/competition_align.py').read_text(encoding='utf-8-sig')
        self.assertNotIn('ServiceProxy',text)
        self.assertNotIn('/mavros/set_mode',text)
        self.assertNotIn('/mavros/cmd/arming',text)

    def test_helper_check_is_offline_and_has_no_flight_service(self):
        import subprocess
        script = ROOT/'tools/align_capture_once.py'
        text = script.read_text(encoding='utf-8-sig')
        self.assertNotIn('/mavros/set_mode', text)
        self.assertNotIn('/mavros/cmd/arming', text)
        self.assertNotIn('rospy.Publisher', text)
        for args in ([], ['--check']):
            result = subprocess.run([sys.executable, str(script)] + args,
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            data = json.loads(result.stdout)
            self.assertFalse(data['ROS_started'])
            self.assertEqual(data['helper_version'], '20261012_v2')
            self.assertTrue(data['persistent_status_subscription'])
            self.assertFalse(data['automatic_retry'])

    def test_default_is_offline_and_output_requires_run(self):
        import subprocess
        script=str(ROOT/'scripts/competition_align.py')
        r=subprocess.run([sys.executable,script],capture_output=True,text=True)
        self.assertEqual(r.returncode,0,r.stderr)
        self.assertFalse(json.loads(r.stdout)['ROS_started'])
        r=subprocess.run([sys.executable,script,'--enable-align-output'],capture_output=True,text=True)
        self.assertNotEqual(r.returncode,0)


if __name__ == '__main__':
    unittest.main()

"""Offline regression of startup sampling and refusal evidence; no ROS calls."""
import contextlib
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts'),str(ROOT.parent/'window_pass_test/src')]
from bind_hover_target_trial import TELEMETRY_TOPICS, require_sample_telemetry
from full_chain_ground import start_ground_pose, pose_bridge_evidence, require_existing_pose_bridge
from ground_snapshot import snapshot_details

LIO='/window_pass_lio/laserMapping'
BEFORE={'/window_pass_lio/Odometry':[LIO,'http://example.invalid/']}

def message(stamp=99.9,caller='/mavros'):
    return NS(header=NS(stamp=NS(to_sec=lambda:stamp)),_connection_header={'callerid':caller})

def startup_state(mode='ALTCTL',stamp=99.9,caller='/mavros'):
    value=message(stamp,caller)
    value.mode=mode
    value.connected=True
    value.armed=False
    value.system_status=3
    return value

def describe_state(state):
    ros=NS(Time=NS(now=lambda:NS(to_sec=lambda:100.)))
    return snapshot_details(ros,[('/mavros/state',object)],[state],2)

def startup_graph():
    return {'/window_pass_lio/Odometry':[LIO]}

def bridge_status(caller='/bridge',**changes):
    data={'raw_lio_motion_limits':{'max_pose_step_m':.35,'max_pose_speed_mps':2.2},
          'raw_lio_guard_scope':'RouteContinuity'}
    data.update(changes)
    return NS(_connection_header={'callerid':caller},data=json.dumps(data))

def streams():
    return {topic:[(9.9,message(caller=LIO if topic=='/window_pass_lio/Odometry' else '/mavros'))]
            for topic in TELEMETRY_TOPICS}

class TelemetryEvidenceTests(unittest.TestCase):
    def test_missing_local_odom_is_named_and_not_replaced_by_lio(self):
        values=streams();del values['/mavros/local_position/odom']
        with self.assertRaises(ValueError) as caught:
            require_sample_telemetry(values,BEFORE,100.,10.)
        text=str(caught.exception)
        self.assertTrue(text.startswith('missing telemetry: /mavros/local_position/odom;'))
        details=json.loads(text.split('; ',1)[1])
        self.assertEqual(details['/mavros/local_position/odom']['count'],0)
        self.assertEqual(details['/window_pass_lio/Odometry']['count'],1)

    def test_all_missing_streams_are_named(self):
        with self.assertRaises(ValueError) as caught:
            require_sample_telemetry({},BEFORE,100.,10.)
        for topic in TELEMETRY_TOPICS:self.assertIn(topic,str(caught.exception))

    def test_current_trusted_five_streams_pass(self):
        self.assertIsNone(require_sample_telemetry(streams(),BEFORE,100.,10.))

    def test_source_age_boundary_unchanged(self):
        for stamp,accepted in ((100.02,True),(100.021,False),(99.0001,True),(99.,False)):
            values=streams();values['/window_pass_lio/Odometry']=[(9.9,message(stamp,LIO))]
            with self.subTest(stamp=stamp):
                if accepted:require_sample_telemetry(values,BEFORE,100.,10.)
                else:
                    with self.assertRaisesRegex(ValueError,'stale:/window_pass_lio/Odometry'):
                        require_sample_telemetry(values,BEFORE,100.,10.)

    def test_stale_receipt_and_fresh_header_still_rejects(self):
        values=streams();values['/mavros/local_position/odom']=[(9.,message())]
        with self.assertRaises(ValueError) as caught:
            require_sample_telemetry(values,BEFORE,100.,10.)
        detail=json.loads(str(caught.exception).split('; ',1)[1])['/mavros/local_position/odom']
        self.assertEqual(detail['receipt_age_s'],1.)
        self.assertAlmostEqual(detail['source_age_s'],.1)
        self.assertEqual(detail['source_callers'],['/mavros'])

    def test_wrong_source_in_earlier_sample_still_rejects(self):
        values=streams();values['/mavros/state'].insert(0,(9.8,message(caller='/impostor')))
        with self.assertRaises(ValueError) as caught:
            require_sample_telemetry(values,BEFORE,100.,10.)
        self.assertIn('untrusted source:/mavros/state; expected=/mavros;',str(caught.exception))
        self.assertIn('/impostor',str(caught.exception))

    def test_lio_must_match_frozen_publisher(self):
        values=streams();values['/window_pass_lio/Odometry']=[(9.9,message(caller='/mavros'))]
        with self.assertRaisesRegex(ValueError,'untrusted source:/window_pass_lio/Odometry'):
            require_sample_telemetry(values,BEFORE,100.,10.)

class PoseStartupEvidenceTests(unittest.TestCase):
    def test_fresh_mode_sample_precedes_start(self):
        events=[]
        def ground():events.append('fresh_ground');return startup_state()
        def start(name,args):events.append((name,args));return 'started'
        self.assertEqual(start_ground_pose(ground,start,startup_graph,describe_state),'started')
        self.assertEqual(events[0],'fresh_ground')
        self.assertEqual(events[1][0],'guarded_pose')
        self.assertEqual(events[1][1][-1],'--run')
        self.assertEqual(len(events),2)

    def test_mode_changed_since_earlier_sample_refuses_start(self):
        earlier=startup_state();fresh=startup_state('MANUAL');started=[]
        self.assertEqual(earlier.mode,'ALTCTL')
        with self.assertRaisesRegex(ValueError,'observed=MANUAL'):
            start_ground_pose(lambda:fresh,lambda *args:started.append(args),startup_graph,describe_state)
        self.assertEqual(started,[])

    def test_current_altctl_does_not_use_earlier_mode(self):
        samples=iter((startup_state('MANUAL'),startup_state()))
        earlier=next(samples);started=[]
        self.assertEqual(earlier.mode,'MANUAL')
        start_ground_pose(lambda:next(samples),lambda *args:started.append(args),startup_graph,describe_state)
        self.assertEqual(len(started),1)

    def test_failed_fresh_ground_gate_never_starts(self):
        started=[]
        def ground():raise ValueError('not landed')
        with self.assertRaisesRegex(ValueError,'not landed'):
            start_ground_pose(ground,lambda *args:started.append(args),startup_graph,describe_state)
        self.assertEqual(started,[])

    def test_stopped_bridge_reports_fault_without_granting_readiness(self):
        name='/window_pass_competition_pose_1'
        pubs={'/window_pass_competition/status':[name]}
        msg=NS(_connection_header={'callerid':name},data=json.dumps({
            'state':'STOPPED:CLOCK_JUMP','real_fault':'LIO_GUARD_CLOCK_JUMP',
            'real_output_active':False,'local_odom_received':False}))
        data=json.loads(pose_bridge_evidence(pubs,lambda:msg))
        self.assertEqual(data['vision_publishers'],[])
        self.assertEqual(data['status']['real_fault'],'LIO_GUARD_CLOCK_JUMP')
        self.assertTrue(data['diagnostic_only'])
        self.assertEqual(pubs,{'/window_pass_competition/status':[name]})

    def test_untrusted_status_is_not_reported_as_bridge_fault(self):
        pubs={'/window_pass_competition/status':['/expected']}
        msg=NS(_connection_header={'callerid':'/other'},data='{"state":"READY"}')
        data=json.loads(pose_bridge_evidence(pubs,lambda:msg))
        self.assertEqual(data['status_error'],'untrusted status source')
        self.assertNotIn('status',data)

    def test_ambiguous_publishers_do_not_select_one_status(self):
        def read():self.fail('must not read an arbitrary source')
        data=json.loads(pose_bridge_evidence({'/window_pass_competition/status':['/one','/two']},read))
        self.assertEqual(data['status_publishers'],['/one','/two'])
        self.assertNotIn('status',data)

    def test_status_timeout_preserves_diagnostic_error(self):
        def read():raise TimeoutError('no fresh status received')
        data=json.loads(pose_bridge_evidence({'/window_pass_competition/status':['/one']},read))
        self.assertEqual(data['status_error'],'no fresh status received')
        self.assertNotIn('status',data)

class PoseStartupGraphTests(unittest.TestCase):
    def attempt(self,pubs,state=None):
        started=[]
        with self.assertRaises(ValueError) as error:
            start_ground_pose(lambda:state or startup_state(),lambda *args:started.append(args),
                              lambda:pubs,describe_state)
        self.assertEqual(started,[])
        return str(error.exception)

    def test_graph_is_read_after_fresh_sample(self):
        pubs=startup_graph();events=[]
        def ground():
            events.append('sample')
            pubs['/mavros/setpoint_raw/local']=['/controller']
            return startup_state()
        def graph():events.append('graph');return pubs
        with self.assertRaisesRegex(ValueError,'flight setpoint publisher present'):
            start_ground_pose(ground,lambda *args:self.fail('must not spawn'),graph,describe_state)
        self.assertEqual(events,['sample','graph'])

    def test_missing_or_multiple_lio_blocks_start(self):
        for sources in ([],[LIO,'/other_lio']):
            with self.subTest(sources=sources):
                self.assertIn('LIO source ambiguous',self.attempt({'/window_pass_lio/Odometry':sources}))

    def test_any_bridge_appearing_during_sampling_blocks_duplicate_start(self):
        for topics in (('/window_pass_competition/status',),('/mavros/vision_pose/pose',),
                       ('/window_pass_competition/status','/mavros/vision_pose/pose')):
            pubs=startup_graph();pubs.update({topic:['/new_bridge'] for topic in topics})
            with self.subTest(topics=topics):
                self.assertIn('pose bridge appeared during startup',self.attempt(pubs))

    def test_mode_error_includes_exact_sample_evidence(self):
        text=self.attempt(startup_graph(),startup_state('MANUAL',99.875))
        for value in ('observed=MANUAL','stamp=99.875000000','age_s=0.125000000',
                      "callerid='/mavros'","mode='MANUAL'",'fresh=True'):
            self.assertIn(value,text)

    def test_success_reports_the_accepted_sample(self):
        output=io.StringIO();started=[]
        with contextlib.redirect_stdout(output):
            start_ground_pose(lambda:startup_state(),lambda *args:started.append(args),
                              startup_graph,describe_state)
        self.assertEqual(len(started),1)
        text=output.getvalue()
        for value in ('POSE_STARTUP_SNAPSHOT','stamp=99.900000000',"callerid='/mavros'","mode='ALTCTL'"):
            self.assertIn(value,text)


class ExistingPoseBridgeTests(unittest.TestCase):
    def graph(self,status='/bridge',vision='/bridge'):
        return {'/window_pass_competition/status':[status],'/mavros/vision_pose/pose':[vision]}

    def test_matching_owner_and_original_scope_limits_pass(self):
        result=require_existing_pose_bridge(self.graph(),lambda:bridge_status())
        self.assertEqual(result['raw_lio_guard_scope'],'RouteContinuity')

    def test_different_status_and_vision_owners_refuse(self):
        with self.assertRaisesRegex(ValueError,'pose bridge partial/ambiguous'):
            require_existing_pose_bridge(self.graph(vision='/other'),lambda:bridge_status())

    def test_status_caller_must_match_graph_owner(self):
        with self.assertRaisesRegex(ValueError,'untrusted pose status source; expected=/bridge; observed=/other'):
            require_existing_pose_bridge(self.graph(),lambda:bridge_status('/other'))

    def test_original_motion_limits_remain_required(self):
        with self.assertRaisesRegex(ValueError,'pose limits require explicit'):
            require_existing_pose_bridge(self.graph(),lambda:bridge_status(raw_lio_motion_limits={
                'max_pose_step_m':.35,'max_pose_speed_mps':99.}))

    def test_original_route_scope_remains_required(self):
        with self.assertRaisesRegex(ValueError,'not full-chain guarded source'):
            require_existing_pose_bridge(self.graph(),lambda:bridge_status(raw_lio_guard_scope='Other'))


if __name__=='__main__':unittest.main()


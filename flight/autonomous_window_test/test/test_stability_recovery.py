"""Disturbance recovery reaches the real final state; no ROS or robot IO."""
import copy
import math
import sys
import threading
import types
import unittest
from dataclasses import replace
from unittest.mock import patch

from test_second_window_return import ReturnSim
import test_completion_priority as completion_tests
import test_auto_transport as transport_tests
app=transport_tests.app
from pass_terminal import TerminalCapture
from attitude_envelope import support
from window_pass.geometry import norm, sub, yaw, wrap


class Once:
    def __init__(self):self.done=False
    def is_set(self):return self.done
    def wait(self,timeout):self.done=True;return True


class RecoveryFlow(unittest.TestCase):
    def bounded_follow(self,s):
        # Resuming a paused plant must not teleport it to a reference 25 cm away.
        # These limits remain inside the unchanged 2.2 m/s / 0.2 rad input gates.
        previous=s.update
        def update(follow=True):
            saved=s.out
            if follow and saved and saved.get('position') is not None:
                delta=sub(saved['position'],s.f.p);distance=norm(delta)
                if distance:s.f.p=tuple(p+d*min(1.,.09/distance) for p,d in zip(s.f.p,delta))
            if saved and s.follow_yaw and saved.get('yaw') is not None:
                delta=wrap(saved['yaw']-yaw(s.f.q))
                s.out=dict(saved,yaw=yaw(s.f.q)+max(-.15,min(.15,delta)))
            previous(False)
            s.out=saved
        s.update=update

    def complete(self,s):
        s.until('DONE',limit=5500)
        self.assertTrue(s.m.return_journey.complete)
        self.assertEqual([r['target_id'] for r in s.m.full_chain.results],s.m.full_chain.selected)
        self.assertTrue(all(r['laser_off_confirmed'] for r in s.m.full_chain.results))
        self.assertEqual(s.actions,['OFFBOARD','ARM','LAND'])
        self.assertEqual(sum(e['new']=='TAKEOFF' for e in s.m.events),1)

    def second_adjust(self,s):
        s.start();s.until('YAW_TURN');s.until('SEARCH');s.until('ADJUST')
        self.assertEqual(s.m.return_journey.leg,2)

    def test_old_alignment_target_recovers_then_all_pairs_finish(self):
        for pair in ((1,2),(1,3),(2,3)):
            with self.subTest(pair=pair):
                s=ReturnSim(pair);self.second_adjust(s);m=s.m
                origin=m.origin;epoch=m.localization_epoch;identity=copy.deepcopy(m.session_window.identity)
                m.adjust_plan['target']=[100.,100.,100.]
                with patch.object(m.session_window,'route',side_effect=ValueError('temporary_departure_not_feasible')):
                    s.step()
                self.assertIsNotNone(m.recovery.active,m.reason)
                self.complete(s)
                self.assertEqual(m.origin,origin);self.assertEqual(m.localization_epoch,epoch)
                self.assertEqual(m.session_window.identity,identity)

    def test_predictive_corridor_failure_starts_bounded_inward_recovery(self):
        s,_=completion_tests.CompletionPriorityTests().recorded();m=s.m;c=m.session_window.scoped_control(m.core.c)
        axis=(0.,-1.,0.)
        required=support(c,m.heading,axis)+c['_session_field_fence_margin_m']-.02
        m.core.c['geofence_min_m'][1]=m.core.pose.p[1]-required+.002
        self.assertTrue(m.corridor(m.core.pose.p))
        self.assertFalse(m.return_journey.region(m,m.core.pose.p,1))
        self.assertTrue(m.recovery.begin_prepass(m,s.inputs(),'predicted_corridor'))
        self.assertGreater(m.recovery.active['target'][1],m.core.pose.p[1])
        self.assertLessEqual(norm(sub(m.recovery.active['target'],m.core.pose.p)),.35)
        self.assertTrue(m.recovery.holding_allowed(m,m.recovery.active['target']))

    def test_recovery_never_accepts_a_different_opening_or_epoch(self):
        s=ReturnSim();self.second_adjust(s);m=s.m
        self.assertTrue(m.recovery.begin_prepass(m,s.inputs(),'test'))
        g=copy.deepcopy(m.recovery.expected_geometry)
        g['center_enu_m'][0]+=.21
        with self.assertRaisesRegex(ValueError,'same_opening'):m.recovery.check_geometry(g)
        g=copy.deepcopy(m.recovery.expected_geometry);g['pose_epoch']+=1
        with self.assertRaisesRegex(ValueError,'epoch'):m.recovery.check_geometry(g)

    def test_recovery_rejects_actual_boundary_and_does_not_clear_fault(self):
        s=ReturnSim();self.second_adjust(s);m=s.m
        m.core.pose=replace(m.core.pose,p=tuple(m.core.c['geofence_min_m']))
        self.assertFalse(m.recovery.begin_prepass(m,s.inputs(),'actual_outside'))
        m.phase='FAULT'
        self.assertFalse(m.recovery.take(m,s.inputs(),'anything','never_rearm'))

    def test_actual_stop_18cm_short_of_endpoint_can_continue_all_pairs(self):
        for pair in ((1,2),(1,3),(2,3)):
            with self.subTest(pair=pair):
                s=ReturnSim(pair);s.start();s.until('POSTPASS');m=s.m
                plan=m.direct_plan;m.phase='PASS';m.terminal_capture=TerminalCapture()
                m.position=tuple(plan['endpoint'])
                actual=tuple(p-.18*d for p,d in zip(plan['endpoint'],plan['direction']))
                stamp=m.core.pose.stamp;base=s.inputs();m.entered=base.wall-7.
                stopped=False
                for n in range(18):
                    m.core.pose=replace(m.core.pose,p=actual,stamp=stamp+n*.05)
                    i=replace(base,wall=base.wall+n*.05,ros=base.ros+n*.05)
                    stopped=m.terminal_capture.update(m,i,.05)
                    if stopped:break
                self.assertTrue(stopped,m.terminal_capture.status())
                self.assertGreater(norm(sub(actual,m.position)),.12)
                m.direct_pass_completed=True;s.f.p=actual
                m.full_chain.begin(m,i)
                s.f.wall=i.wall;s.f.ros=i.ros;m.last_tick=i.wall;s.out=m.output()
                self.bounded_follow(s)
                self.assertEqual(m.position,actual)
                self.complete(s)

    def test_terminal_outside_capture_radius_does_not_complete(self):
        s=ReturnSim();s.start();s.until('POSTPASS');m=s.m
        m.phase='PASS';m.terminal_capture=TerminalCapture();m.position=tuple(m.direct_plan['endpoint'])
        p=tuple(x-.31*d for x,d in zip(m.position,m.direct_plan['direction']))
        m.entered=s.f.wall-7.;base=s.inputs()
        for n in range(16):
            m.core.pose=replace(m.core.pose,p=p,stamp=base.ros+n*.05)
            self.assertFalse(m.terminal_capture.update(m,replace(base,wall=base.wall+n*.05),.05))

    def transfer(self,s):
        s.start()
        for _ in range(2500):
            s.step()
            if s.m.phase=='POSTPASS' and s.m.full_chain.index==1 and s.m.full_chain.stage=='STATION_TRANSFER':return
        self.fail(s.m.reason)

    def test_delayed_transfer_pauses_trajectory_then_finishes(self):
        s=ReturnSim((1,3));self.transfer(s)
        for _ in range(65):
            s.step(False)
            self.assertEqual(s.m.phase,'POSTPASS',s.m.reason)
            self.assertLessEqual(norm(sub(s.m.position,s.f.p)),.25001)
        self.bounded_follow(s);self.complete(s)

    def test_transfer_old_reference_is_replanned_without_repeating_target(self):
        s=ReturnSim((2,3));self.transfer(s);m=s.m
        m.position=(s.f.p[0]+.4,s.f.p[1],s.f.p[2])
        s.step(False)
        self.assertEqual(m.position,tuple(s.f.p));self.assertEqual(m.full_chain.index,1)
        self.assertEqual(len(m.full_chain.results),1)
        self.complete(s)

    def test_slow_turn_retries_remaining_angle_then_finishes(self):
        s=ReturnSim((1,2));s.start();s.until('YAW_TURN');s.follow_yaw=False
        goal=s.m.return_journey.turn_goal
        for _ in range(430):s.step(False)
        self.assertEqual(s.m.phase,'YAW_TURN',s.m.reason)
        self.assertGreater(s.m.return_journey.turn_extra_s,0.)
        self.assertEqual(s.m.return_journey.turn_goal,goal)
        s.follow_yaw=True;self.bounded_follow(s);self.complete(s)

    def test_retry_budget_and_whole_mission_deadline_remain_finite(self):
        s=ReturnSim();self.second_adjust(s);m=s.m;i=s.inputs()
        self.assertTrue(m.recovery.take(m,i,'finite','test'))
        self.assertTrue(m.recovery.take(m,i,'finite','test'))
        self.assertFalse(m.recovery.take(m,i,'finite','test'))
        m.began=i.wall-241.
        self.assertFalse(m.recovery.take(m,i,'other','deadline'))
        s.step();self.assertIn(m.phase,('HANDOVER_REQUEST','PILOT_CONTROL','FAULT'))


class QueryRecovery(unittest.TestCase):
    def test_timed_aim_preflight_without_scoring_camera_keeps_light_ack_required(self):
        from test_takeoff import TakeoffSim
        s=TakeoffSim();r=transport_tests.Boundary().runtime(s)
        r.errors=[];r.takeoff_only=True;r.airborne_geometry=False;r.control=s.m.core.c
        r.params={k:[0,0.] for k in app.PARAMETERS}
        r.cfg={'mission':s.options,'acceptance':{k:True for k in app.TAKEOFF_EVIDENCE},
               'evidence_id':'SYNTHETIC_TEST_ONLY','expected_fcu_parameters':r.params}
        r.history=[(s.f.wall,.245)]*60;r.graph_reason='test'
        r.hover_policy={'fixed_position_timed':False,'timed_visual_aim':True}
        r.hover_laser_row=None;r.hover_visual_row=None
        with patch('hover_target.fresh_bridge',return_value={'state':'OFF_ACKNOWLEDGED'}):
            result=r.report()
            self.assertTrue(result['ready'],result['blockers'])
            r.hover_policy['timed_visual_aim']=False
            self.assertIn('hover_camera_status_missing',r.report()['blockers'])
        r.hover_policy['timed_visual_aim']=True
        with patch('hover_target.fresh_bridge',return_value=None):
            self.assertIn('hover_laser_initial_off_not_acknowledged',r.report()['blockers'])

    def test_parameter_query_failure_preserves_only_last_valid_age(self):
        s=ReturnSim();r=transport_tests.Boundary().runtime(s);r.stop=Once()
        r.params={k:[0,0.] for k in app.PARAMETERS};r.params_wall=s.f.wall
        r.cfg={'expected_fcu_parameters':copy.deepcopy(r.params)}
        r.ros.ServiceProxy=lambda *a:lambda **kw:(_ for _ in ()).throw(OSError('transport'))
        srv=types.ModuleType('mavros_msgs.srv');srv.ParamGet=object
        with patch.dict(sys.modules,{'mavros_msgs.srv':srv}):r.parameter_worker()
        self.assertEqual(r.params_wall,s.f.wall)
        self.assertTrue(r.parameters_ok(s.f.wall+4.9));self.assertFalse(r.parameters_ok(s.f.wall+5.1))

    def test_graph_transport_failure_does_not_refresh_or_erase_valid_result(self):
        s=ReturnSim();r=transport_tests.Boundary().runtime(s);r.stop=Once();r.graph_ok=True;r.graph_wall=s.f.wall
        r.master=types.SimpleNamespace(getSystemState=lambda:(_ for _ in ()).throw(OSError('transport')))
        r.graph_worker()
        self.assertTrue(r.graph_ok);self.assertEqual(r.graph_wall,s.f.wall)
        r.stop=Once();r.master.getSystemState=lambda:(_ for _ in ()).throw(ValueError('identity_changed'))
        r.graph_worker();self.assertFalse(r.graph_ok)

    def test_timed_aim_missing_visual_passes_real_graph_checks(self):
        from test_takeoff import TakeoffSim
        s=TakeoffSim();r=transport_tests.Boundary().runtime(s);r.stop=Once()
        r.takeoff_only=True;r.airborne_geometry=False;r.execute=True;r.motion=None;r.source_identity=None
        r.control=s.m.core.c;r.hover_policy={'fixed_position_timed':False,'timed_visual_aim':True}
        r.params={k:[0,0.] for k in app.PARAMETERS};r.params_wall=s.f.wall
        r.cfg={'expected_fcu_parameters':copy.deepcopy(r.params)}
        topics=('/mavros/state','/mavros/extended_state','/mavros/local_position/odom','/mavros/rc/in','/mavros/estimator_status','/mavros/battery')
        pubs=[[t,['/mavros']] for t in topics]+[['/hover_target_laser/status',['/hover_target_laser']]]
        r.master=types.SimpleNamespace(getSystemState=lambda:[pubs,[],[]])
        r.ros.get_param=lambda *a:False;r.ros.get_name=lambda:'/auto'
        r.core=types.SimpleNamespace(graph=lambda *a:None)
        with patch.object(app.time,'monotonic',return_value=s.f.wall):r.graph_worker()
        self.assertTrue(r.graph_ok,r.graph_reason)
        r.stop=Once();pubs.append(['/circle_pixel_preview/status',['/unexpected_source']])
        with patch.object(app.time,'monotonic',return_value=s.f.wall):r.graph_worker()
        self.assertFalse(r.graph_ok)


if __name__=='__main__':unittest.main()

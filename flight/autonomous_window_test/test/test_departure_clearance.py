"""Recorded second-window premature commitment; offline, no aircraft IO."""
import copy
import json
import math
import unittest
from dataclasses import replace
from pathlib import Path

from attitude_envelope import support
from pass_terminal import MotionWindow
from session_window import decorate
from straight_pass import advance_route
from test_second_window_return import ReturnSim
from window_pass.geometry import add, scale, norm


FIXTURE=json.loads((Path(__file__).parent/'fixtures/aperture_763140854933.json').read_text())


class DepartureClearance(unittest.TestCase):
    def recorded(self):
        d=copy.deepcopy(FIXTURE);s=ReturnSim((1,2))
        s.start();s.until('YAW_TURN');s.until('SEARCH');s.until('ADJUST')
        m=s.m;b=d['before'];p=d['before_pose'];cfg=d['config']
        m.core.c=decorate(copy.deepcopy(cfg['control']),cfg['session_window'])
        m.core.pose=replace(m.core.pose,p=tuple(p['p']),q=tuple(p['q']),stamp=p['stamp'])
        m.origin=tuple(b['full_chain']['takeoff_origin_local_m']);m.ground_z=b['height']['ground_z_local_m']
        m.position=tuple(b['position']);m.heading=b['yaw']
        m.adjust_plan=copy.deepcopy(b['prepass_adjustment']['plan'])
        m.session_window.snapshot=copy.deepcopy(b['session_window']['frozen_geometry'])
        m.session_window.plan=copy.deepcopy(m.adjust_plan)
        m.adjust_last_position=m.core.pose.p;m.adjust_path_m=0.
        m.return_journey.first_plan=copy.deepcopy(d['first_plan'])
        m.entered=s.f.wall-.5;m.field_alignment_replan_stamp=None
        m.field_alignment_motion=MotionWindow()
        for n in range(12):m.field_alignment_motion.update(p['stamp']-.55+.05*n,p['p'])
        return s,d

    def test_recorded_narrow_departure_continues_alignment(self):
        s,d=self.recorded();m=s.m
        plan=d['failed_route'];p=m.core.pose
        m.core.pose=replace(p,p=tuple(plan['actual_departure_position_local_m']),q=tuple(d['departure_pose']['q']))
        m.position=tuple(plan['start'])
        original=m.session_window.route(m.core.c,m.core.pose,m.position)
        self.assertLess(min(original['edge_clearance_after_budgets_m']),.012)
        out=m.align_session_window(s.inputs(),.02)
        self.assertEqual(out['phase'],'ADJUST',out['reason'])
        self.assertIsNone(out['action']);self.assertIsNone(m.recovery.active)
        self.assertFalse(m.departure_clearance_gate['ready'])
        self.assertAlmostEqual(m.departure_clearance_gate['required_margin_m'],.08)
        self.assertLess(norm(tuple(a-b for a,b in zip(m.position,plan['start']))),.002)

    def test_commit_entry_cannot_bypass_clearance_gate(self):
        s,d=self.recorded();m=s.m;route=d['failed_route']
        m.core.pose=replace(m.core.pose,p=tuple(route['actual_departure_position_local_m']),q=tuple(d['departure_pose']['q']))
        m.position=tuple(route['start'])
        out=m.commit_session_route(s.inputs())
        self.assertEqual(out['phase'],'ADJUST');self.assertFalse(m.adjust_completed)

    def test_recorded_departure_keeps_adjusting_then_releases_with_margin(self):
        s,d=self.recorded();m=s.m;old=d['failed_route']
        m.core.pose=replace(m.core.pose,p=tuple(old['actual_departure_position_local_m']),q=tuple(d['departure_pose']['q']))
        m.position=tuple(old['start'])
        origin=m.origin;epoch=m.localization_epoch;geometry=copy.deepcopy(m.session_window.snapshot)
        start=m.position
        for n in range(240):
            m.core.pose=replace(m.core.pose,p=tuple(m.position),stamp=m.core.pose.stamp+.05)
            out=m.align_session_window(replace(s.inputs(),wall=s.f.wall+n*.05),.05)
            self.assertIn(out['phase'],('ADJUST','PASS'),out['reason'])
            self.assertIsNone(m.recovery.active)
            if out['phase']=='PASS':break
        self.assertEqual(m.phase,'PASS',m.reason)
        self.assertGreater(n,1)
        self.assertLess(math.dist(start,m.direct_plan['start']),.20)
        self.assertGreaterEqual(m.departure_clearance_gate['current_margin_m'],.08-1e-9)
        self.assertEqual(m.origin,origin);self.assertEqual(m.localization_epoch,epoch)
        self.assertEqual(m.session_window.snapshot,geometry)
        self.assertEqual(json.loads(json.dumps(m.direct_plan['smooth_trajectory'])),old['smooth_trajectory'])

    def test_margin_ready_releases_without_exact_target_and_same_dynamics(self):
        s,d=self.recorded();m=s.m;target=tuple(m.adjust_plan['target'])
        # Deliberately remain 1 cm short of the selected centre.
        actual=(target[0],target[1]+.01,target[2])
        m.core.pose=replace(m.core.pose,p=actual);m.position=actual
        m.field_alignment_motion=MotionWindow()
        for n in range(12):m.field_alignment_motion.update(m.core.pose.stamp-.55+n*.05,actual)
        out=m.align_session_window(s.inputs(),.02)
        self.assertEqual(out['phase'],'PASS',out['reason'])
        self.assertTrue(m.departure_clearance_gate['ready'])
        self.assertGreater(min(m.direct_plan['edge_clearance_after_budgets_m']),.08)
        self.assertEqual(json.loads(json.dumps(m.direct_plan['smooth_trajectory'])),d['failed_route']['smooth_trajectory'])
        self.assertEqual(m.direct_plan['length_m'],3.)
        # Replay the original measured drift against the better departure ray.
        old=d['failed_route'];p=d['fault_pose']['p']
        from window_pass.geometry import sub,dot
        delta=sub(p,old['start']);progress=dot(delta,old['direction'])
        error=sub(delta,scale(old['direction'],progress))
        pose=replace(m.core.pose,p=add(add(actual,scale(m.direct_plan['direction'],progress)),error))
        advance_route(m.direct_plan,pose,actual,m.core.c,.02,.7)

    def test_limited_available_margin_does_not_require_unreachable_eight_cm(self):
        s,d=self.recorded();m=s.m;old=d['failed_route']
        m.core.pose=replace(m.core.pose,p=tuple(old['actual_departure_position_local_m']),q=tuple(d['departure_pose']['q']))
        m.position=tuple(old['start'])
        # Model a geometry-constrained case where the best feasible target is
        # already the current ray. The original departure checks still apply.
        m.adjust_plan['target']=list(m.core.pose.p)
        plan=m.session_departure_route()
        self.assertTrue(m.departure_clearance_gate['ready'])
        self.assertLess(m.departure_clearance_gate['required_margin_m'],.01)
        self.assertEqual(plan['length_m'],3.)

    def test_obsolete_target_does_not_reintroduce_old_corridor_abort(self):
        s,d=self.recorded();m=s.m;target=tuple(m.adjust_plan['target'])
        m.core.pose=replace(m.core.pose,p=target);m.position=target
        m.adjust_plan['target']=[100.,100.,100.]
        plan=m.session_departure_route()
        self.assertEqual(plan['length_m'],3.)
        self.assertIn('candidate_reason',m.departure_clearance_gate)

    def test_actual_aperture_failure_remains_rejected(self):
        s,d=self.recorded();m=s.m;route=d['failed_route']
        m.core.pose=replace(m.core.pose,p=(route['start'][0],route['start'][1]+.04,route['start'][2]))
        m.position=m.core.pose.p
        with self.assertRaises(ValueError):m.session_departure_route()


if __name__=='__main__':unittest.main()

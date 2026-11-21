"""Exact retransmissions and delayed OFF acknowledgments, without hardware."""
import ast
import copy
import types
import unittest
from dataclasses import replace

import test_auto_transport as transport_tests
from test_second_window_return import ReturnSim
from session_localization import Continuity
from window_pass.core import Pose


class InputStability(unittest.TestCase):
    def continuity(self):
        s=ReturnSim();c=Continuity({k:v for k,v in s.m.core.c.items() if not k.startswith('_session_')})
        p=Pose(100.,(0.,0.,0.),(0.,0.,0.,1.),'camera_init','body')
        self.assertTrue(c.observe(p,10,100.,10.,'/lio','/lio'))
        return c,p

    def test_exact_lio_duplicate_does_not_refresh_time_or_change_epoch(self):
        c,p=self.continuity();stream=c.raw.streams['pose']
        self.assertTrue(c.observe(p,10,100.1,10.1,'/lio','/lio'))
        self.assertEqual(c.raw.streams['pose'],stream);self.assertEqual(c.epoch,0)
        self.assertEqual(c.duplicate_messages,1)
        self.assertFalse(c.raw.fresh('pose',100.21,10.21,.2))

    def test_same_sequence_with_changed_payload_or_source_still_faults(self):
        c,p=self.continuity()
        self.assertFalse(c.observe(replace(p,stamp=100.05,p=(.01,0.,0.)),10,100.05,10.05,'/lio','/lio'))
        self.assertEqual(c.epoch,1)
        c,p=self.continuity()
        self.assertFalse(c.observe(p,10,100.05,10.05,'/different','/lio'))
        self.assertEqual(c.reason,'lio_source_changed')

    def test_exact_mavros_duplicate_preserves_last_accepted_sample(self):
        s=ReturnSim();s.start();s.until('SEARCH');r=transport_tests.Boundary().runtime(s)
        r.history=[];r.now=lambda:(s.f.ros,s.f.wall)
        p=s.m.core.pose
        msg=types.SimpleNamespace(
            _connection_header={'callerid':s.m.core.c['fcu_node']},
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda:p.stamp),frame_id=p.frame),
            child_frame_id=p.child,pose=types.SimpleNamespace(pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=p.p[0],y=p.p[1],z=p.p[2]),
                orientation=types.SimpleNamespace(x=p.q[0],y=p.q[1],z=p.q[2],w=p.q[3]))))
        stream=s.m.core.streams['pose'];r.pose(msg)
        self.assertEqual(s.m.phase,'SEARCH');self.assertEqual(r.duplicate_pose_messages,1)
        self.assertEqual(s.m.core.streams['pose'],stream)
        msg._connection_header={'callerid':'/unknown'};r.pose(msg)
        self.assertEqual(s.m.phase,'FAULT')

    def test_diagnostic_serialization_is_outside_control_lock(self):
        tree=ast.parse((transport_tests.app.ROOT/'scripts/auto_window.py').read_text())
        parents={child:node for node in ast.walk(tree) for child in ast.iter_child_nodes(node)}
        def name(n):
            if isinstance(n,ast.Name):return n.id
            if isinstance(n,ast.Attribute):return name(n.value)+'.'+n.attr
            return ''
        runtime=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Runtime')
        run=next(n for n in runtime.body if isinstance(n,ast.FunctionDef) and n.name=='run')
        calls=[n for n in ast.walk(run) if isinstance(n,ast.Call) and name(n.func)=='self.diag.publish']
        self.assertEqual(len(calls),1)
        node=calls[0]
        while node in parents:
            node=parents[node]
            if isinstance(node,ast.With):
                self.assertNotIn('self.lock',[name(i.context_expr) for i in node.items])


class OffAcknowledgment(unittest.TestCase):
    def test_delayed_target_off_reuses_request_then_completes(self):
        s=ReturnSim((1,2));s.start();s.until('HOVER')
        while s.m.hover_target.phase!='EXPOSE':s.step()
        s.allow_off=False
        while s.m.hover_target.phase!='OFF_WAIT':s.step()
        request=s.m.hover_target.request_id
        for _ in range(50):s.step()
        self.assertEqual(s.m.phase,'HOVER');self.assertEqual(s.m.hover_target.phase,'OFF_WAIT')
        self.assertEqual(s.m.hover_target.request_id,request)
        self.assertEqual(s.m.hover_target.off_ack_extra_s,2.)
        self.assertFalse(s.m.hover_target.off_confirmed)
        s.allow_off=True;s.until('DONE',limit=5500)
        self.assertEqual(len(s.m.full_chain.results),2)
        self.assertEqual(s.actions,['OFFBOARD','ARM','LAND'])

    def test_delayed_landing_off_requires_real_ack_before_done(self):
        s=ReturnSim((2,3));s.start();s.until('LANDING_LIGHT_OFF',limit=5500)
        s.allow_off=False;request=s.m.return_journey.request_id
        for _ in range(50):s.step()
        self.assertEqual(s.m.phase,'LANDING_LIGHT_OFF')
        self.assertEqual(s.m.return_journey.request_id,request)
        self.assertEqual(s.m.return_journey.landing_off_extra_s,2.)
        self.assertFalse(s.m.return_journey.complete)
        s.allow_off=True;s.until('DONE')
        self.assertTrue(s.m.return_journey.complete)


if __name__=='__main__':unittest.main()

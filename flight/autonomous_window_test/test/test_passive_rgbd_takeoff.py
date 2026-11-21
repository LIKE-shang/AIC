import unittest
from dataclasses import replace
from test_pilot_landing import PilotSim
from test_mission import Sim
from mission import Mission,settings
from test_auto_transport import app

class PassiveTakeoff(unittest.TestCase):
    def test_allowance_does_not_grant_traverse(self):
        s=PilotSim();s.m.o['takeoff_horizontal_error_m']=.18
        self.assertFalse(s.m.core.c['enable_traverse'])
        self.assertEqual(s.m.core.request_pass(),(False,'takeoff_test_cannot_commit_pass'))
        s.start();s.until('TAKEOFF');p=s.m.core.pose
        s.m.core.pose=replace(p,p=(s.m.origin[0]+.13,p.p[1],p.p[2]))
        self.assertEqual(s.m.tick(s.inputs())['phase'],'TAKEOFF')
        s.m.core.pose=replace(p,p=(s.m.origin[0]+.181,p.p[1],p.p[2]))
        self.assertEqual(s.m.tick(s.inputs())['phase'],'FAULT')
    def test_hover_xy_allowance_keeps_vertical_limit(self):
        s=PilotSim();s.m.o['takeoff_horizontal_error_m']=.18;s.start();s.until('HOVER');p=s.m.core.pose
        s.m.core.pose=replace(p,p=(s.m.origin[0]+.13,p.p[1],p.p[2]))
        self.assertEqual(s.m.tick(s.inputs())['phase'],'HOVER')
        s.m.core.pose=replace(p,p=(p.p[0],p.p[1],s.m.target_z+.121))
        self.assertEqual(s.m.tick(s.inputs())['phase'],'FAULT')
    def test_allowance_never_changes_traverse_limit(self):
        s=Sim();m=Mission(s.m.core.c,dict(s.options,takeoff_horizontal_error_m=.18))
        self.assertEqual(m.horizontal_tracking_limit(),.12)
    def test_allowance_must_fit_column(self):
        s=PilotSim()
        for v in (.201,.11,float('nan')):
            with self.assertRaises(ValueError):settings(dict(s.options,takeoff_horizontal_error_m=v))
    def test_passive_observation_still_uses_takeoff_commissioning(self):
        from test_position_goal_takeoff import PositionGoal
        c=PositionGoal().config();c['acquisition']='takeoff_rgbd_observe';c['mission']['takeoff_horizontal_error_m']=.18
        self.assertTrue(app.commissioning_deferrals(c,True))
        c['experiment']='traverse'
        with self.assertRaises(ValueError):app.commissioning_deferrals(c,True)

    def test_passive_callback_records_unique_frames_without_flight_authority(self):
        import threading,types
        from collections import deque
        r=app.Runtime.__new__(app.Runtime);r.lock=threading.RLock();r.passive_rgbd=True;r.airborne_geometry=False
        r.mission=types.SimpleNamespace(phase='HOVER');r.observation_rows=deque(maxlen=100);r.observation_started_ros=None
        r.ros=types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_sec=lambda:100.)))
        r.control={};r.core=types.SimpleNamespace(ingest_metric=lambda *a:self.fail('passive capture must not command geometry controller'))
        msg=types.SimpleNamespace(_connection_header={'callerid':'/window_live_vision'},data='{"source_stamp":100,"target_count":0,"ready":false}')
        r.rgbd(msg);r.rgbd(msg)
        self.assertEqual(len(r.observation_rows),1)
        self.assertNotEqual(r.observation_rows[0]['metric_validation'],'ok')
        self.assertEqual(r.mission.phase,'HOVER')
        msg._connection_header={'callerid':'/untrusted'};r.rgbd(msg)
        self.assertEqual(len(r.observation_rows),1)
    def test_passive_frames_before_hover_do_not_count(self):
        import threading,types
        from collections import deque
        r=app.Runtime.__new__(app.Runtime);r.lock=threading.RLock();r.passive_rgbd=True;r.airborne_geometry=False
        r.mission=types.SimpleNamespace(phase='TAKEOFF');r.observation_rows=deque();r.observation_started_ros=None
        r.rgbd(types.SimpleNamespace(_connection_header={'callerid':'/window_live_vision'},data='{"source_stamp":100}'))
        self.assertFalse(r.observation_rows)

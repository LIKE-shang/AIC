"""Approved ground profile through stable handoff into the existing supervisor."""
import copy
import math
from pathlib import Path
import unittest
import yaml
from ground_straight_nav.core import StraightRun
from policy import Handoff, ReturnHandoff


class SuccessfulGroundProfileTests(unittest.TestCase):
    def test_current_profile_then_single_supervisor_second_window_return(self):
        from test_second_window_return import ReturnSim
        raw = yaml.safe_load((Path(__file__).resolve().parents[1] / 'ground_stage_v6_yaw015.yaml').read_text())
        points = raw.pop('waypoints')
        self.assertEqual([raw[k] for k in ('max_speed_mps','final_max_speed_mps','accel_limit_mps2','decel_limit_mps2')], [1.5,.65,.8,.8])
        self.assertTrue(raw['heading_hold_enabled'])
        self.assertTrue(raw['fast_intermediate_enabled'])
        self.assertEqual(points[2]['map'], [.015790638438488862,-2.874492045599364])
        r = StraightRun(raw)
        r.pose_update(0.,0.,0.,0.)
        r.heading_update(0.,1.,0.)
        r.start(0.); r.begin(points)
        limits=[]
        for i in range(4):
            r.index=i; limits.append(r.active_speed_limit())
        r.index=0
        self.assertEqual(limits,[1.5,1.5,1.5,.65])
        h=Handoff(); x=y=0.
        for n in range(1,5000):
            t=n*.05
            r.heading_update(0.,1.+t,t)
            r.pose_update(x,y,0.,t)
            vx,vy=r.update(t); x+=vx*.05; y+=vy*.05
            if h.observe(r.summary(),0.,'own','own'):break
        self.assertTrue(h.done)
        self.assertLess(math.hypot(x,y),.05)
        saved={'identity':{'boot':'b','run':'r','source':'uri'},'position_local_m':[0.,0.,0.]}
        gate=ReturnHandoff(copy.deepcopy(saved))
        for n in range(50):
            t+=.05
            r.update(t)  # No identity/heading refresh after DONE.
            self.assertEqual(r.state,'DONE')
            if not gate.ready:gate.observe(saved,r.summary(),t,t)
        self.assertTrue(gate.ready)
        for ids in ([1,2],[1,3],[2,3]):
            sim=ReturnSim(ids); sim.start(); owner=sim.m.core
            sim.until('AUTO_LANDING',limit=4500)
            self.assertIs(sim.m.core,owner)
            self.assertEqual(sim.actions,['OFFBOARD','ARM','LAND'])
            self.assertTrue(sim.m.return_journey.second_pass_complete)
            self.assertTrue(sim.m.return_journey.origin_return_stopped)


if __name__ == '__main__':unittest.main()

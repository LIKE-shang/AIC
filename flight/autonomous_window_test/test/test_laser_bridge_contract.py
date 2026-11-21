"""Actual C++ bridge policy consumes supervisor output; no ROS or hardware IO."""
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from test_bounded_visual_aim import BoundedSim


class LaserBridgeContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler=shutil.which('g++')
        if not compiler:
            raise unittest.SkipTest('actual C++ bridge policy requires g++')
        flight=Path(__file__).resolve().parents[2]
        vision=flight/'vision_ws/src/full_chain_vision_20261025_v6_1'
        cls.temp=tempfile.TemporaryDirectory(prefix='bridge_contract_')
        cls.binary=Path(cls.temp.name)/'timed_visual_aim_guard'
        result=subprocess.run([compiler,'-std=c++11','-I'+str(vision/'src'),
            str(vision/'tests/timed_visual_aim_guard_test.cpp'),'-o',str(cls.binary)],
            stdout=subprocess.PIPE,stderr=subprocess.PIPE,universal_newlines=True,timeout=60)
        if result.returncode:
            raise AssertionError('C++ bridge compile failed: '+result.stderr)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def decisions(self,rows):
        result=subprocess.run([str(self.binary),'--json'],
            input=''.join(json.dumps(row)+'\n' for row in rows),stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,universal_newlines=True,timeout=30,check=True)
        return result.stdout.splitlines()

    def test_real_abort_three_second_contract_is_rejected_one_second_is_accepted(self):
        old=dict(bounded_visual_aim=True,timed_visual_aim=True,visual_target_hold=True,
            visual_aim_budget_s=3.,visual_aim_budget_elapsed=True,exposure_s=3.,
            fixed_position_timed=False,visual_servo_enabled=True,visual_calibration_mode='off',stage='ON_WAIT')
        self.assertEqual(self.decisions([old,dict(old,visual_aim_budget_s=1.)]),['REJECT','ACCEPT'])

    def test_actual_supervisor_both_targets_are_accepted_for_all_ON_heartbeats(self):
        for scenario in ('no_target','far_target','preexisting_reflection'):
            s=BoundedSim()
            if scenario=='no_target':s.draw_target=False
            if scenario=='far_target':s.reference_error=[35.,25.]
            if scenario=='preexisting_reflection':s.preexisting_spot=True
            s.start();rows=[];targets=set()
            for _ in range(7000):
                s.step();h=s.m.hover_target
                if h.request_on:
                    rows.append(copy.deepcopy(h.status()))
                    targets.add(s.m.full_chain.selected[s.m.full_chain.index])
                if s.m.phase=='AUTO_LANDING':s.airborne=False;s.armed=False;s.mode='AUTO.LAND'
                if s.m.phase in ('DONE','FAULT','TAKEOVER','PILOT_CONTROL'):break
            self.assertEqual(s.m.phase,'DONE',(scenario,s.m.reason))
            self.assertEqual(len(targets),2,scenario)
            self.assertTrue(rows)
            decisions=self.decisions(rows)
            self.assertEqual(len(decisions),len(rows))
            self.assertTrue(all(x=='ACCEPT' for x in decisions),(scenario,decisions))

    def test_original_bridge_rejection_tests_still_pass(self):
        subprocess.run([str(self.binary)],check=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=10)

    def test_paced_feedback_keeps_actual_cpp_contract_for_complete_course(self):
        from test_observed_servo_pacer import PacedSim
        for missing in (False,True):
            s=PacedSim();s.draw_target=not missing;s.spot_error=[20.,0.]
            s.start();rows=[];targets=set()
            for _ in range(7000):
                s.step();h=s.m.hover_target
                if h.request_on:
                    self.assertTrue(h.p['paced_visual_servo'])
                    rows.append(copy.deepcopy(h.status()))
                    targets.add(s.m.full_chain.selected[s.m.full_chain.index])
                if s.m.phase=='AUTO_LANDING':s.airborne=False;s.armed=False;s.mode='AUTO.LAND'
                if s.m.phase in ('DONE','FAULT','TAKEOVER','PILOT_CONTROL'):break
            self.assertEqual(s.m.phase,'DONE',s.m.reason)
            self.assertEqual(len(targets),2)
            self.assertTrue(rows)
            self.assertTrue(all(x=='ACCEPT' for x in self.decisions(rows)))


if __name__=='__main__':unittest.main()

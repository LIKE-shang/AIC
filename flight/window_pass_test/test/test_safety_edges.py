#!/usr/bin/env python3
"""Pure/Fake ROS fault injection. All dimensions here are synthetic fixtures."""
from pathlib import Path
import sys
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0,str(ROOT/'test'))
from fixtures import Feed,calibration
from window_pass.core import Controller
from window_pass.config import load_config


class SafetyEdgeTests(unittest.TestCase):
    def test_measurement_and_stopping_margin_are_mandatory(self):
        for k in ('position_uncertainty_m','stopping_margin_m'):
            c=calibration(True);c[k]=0.
            with self.subTest(key=k),self.assertRaises(ValueError):Controller(c)

    def test_invalid_configuration_limits(self):
        cases=[('vehicle_size_m',[-.62,.62,.35]),('window_yaw_rad',4.),
               ('stability_px',30.),('target_jump_px',100.),('opening_size_error_fraction',.9),
               ('max_total_time_s',999.),('max_traverse_distance_m',99.),
               ('max_setpoint_lead_m',.06),('degraded_hold_timeout_s',5.),
               ('hold_prestream_s',.5),('optical_frame','square'),('fcu_node','mavros')]
        for key,value in cases:
            with self.subTest(key=key),self.assertRaises(ValueError):load_config({key:value})
        with self.assertRaises(ValueError):load_config(None)

    def test_mode_exit_between_control_ticks_is_latched(self):
        f=Feed(True);f.ready()
        c=f.ctrl
        c.flight_state(f.ros+.001,True,True,'POSCTL',f.ros+.001,f.wall+.001)
        c.flight_state(f.ros+.002,True,True,'OFFBOARD',f.ros+.002,f.wall+.002)
        f.step()
        self.assertEqual(c.state,'RELEASED');self.assertIsNone(c.output(f.ros,f.wall))

    def test_disconnection_pulse_is_latched(self):
        f=Feed(True);f.ready();c=f.ctrl
        c.flight_state(f.ros+.001,False,True,'OFFBOARD',f.ros+.001,f.wall+.001)
        f.step()
        self.assertEqual(c.state,'RELEASED')

    def test_bad_image_between_ticks_is_latched(self):
        f=Feed(True);f.ready()
        f.ctrl.image(0.,640,480,f.ros+.001,f.wall+.001)
        f.step()
        self.assertEqual(f.ctrl.state,'FAULT')

    def test_graph_conflict_pulse_never_recovers_permission(self):
        f=Feed(True);f.ready()
        f.ctrl.graph(False,'second_controller',f.wall)
        f.ctrl.graph(True,'gone_again',f.wall+.001)
        f.step()
        self.assertEqual(f.ctrl.state,'RELEASED');self.assertFalse(f.ctrl.authorized)

    def test_prestream_counts_do_not_replace_elapsed_time(self):
        f=Feed(True);f.prime();self.assertTrue(f.ctrl.request_hold(f.ros,f.wall)[0])
        for _ in range(100):f.ctrl.output_sent(f.wall)
        self.assertFalse(f.ctrl.request_start(f.ros,f.wall)[0])

    def test_manual_drift_before_offboard_revokes_hold_stream(self):
        f=Feed(True);f.mode='POSCTL';f.prime()
        self.assertTrue(f.ctrl.request_hold(f.ros,f.wall)[0])
        f.p=(.06,0.,1.2);f.step(dt=.1)
        self.assertEqual(f.ctrl.state,'RELEASED')
        f.mode='OFFBOARD';f.step()
        self.assertIsNone(f.ctrl.output(f.ros,f.wall))

    def test_pose_loss_is_bounded_not_indefinite_keepalive(self):
        f=Feed(True);f.ready();f.ctrl.request_pass(f.ros,f.wall)
        for _ in range(6):f.step(pose=False)
        self.assertEqual(f.ctrl.state,'FAULT');fixed=f.ctrl.command
        for _ in range(20):f.step(pose=False)
        self.assertEqual(f.ctrl.state,'RELEASED');self.assertEqual(f.ctrl.command,fixed)
        self.assertIsNone(f.ctrl.output(f.ros,f.wall))

    def test_commit_refuses_insufficient_remaining_time(self):
        f=Feed();f.ready();f.ctrl.begin_wall=f.wall-f.cfg['max_total_time_s']+1.
        ok,why=f.ctrl.request_pass(f.ros,f.wall)
        self.assertFalse(ok);self.assertIn('remaining_time',why)

    def test_complete_still_monitors_pose_and_control_ownership(self):
        f=Feed(True);f.ready();f.ctrl.request_pass(f.ros,f.wall)
        for _ in range(400):f.step(follow=True)
        self.assertEqual(f.ctrl.state,'HOLD_COMPLETE')
        f.mode='POSCTL';f.step()
        self.assertEqual(f.ctrl.state,'RELEASED')

    def test_fault_in_advance_cannot_be_overwritten_by_completion(self):
        f=Feed();f.ready();f.ctrl.request_pass(f.ros,f.wall)
        def fault(candidate,ros,wall):f.ctrl.stop('injected_advance_fault',ros,wall)
        f.ctrl.advance=fault
        f.step()
        self.assertEqual(f.ctrl.state,'ABORT')
        self.assertEqual(f.ctrl.reason,'injected_advance_fault')


if __name__=='__main__':unittest.main(verbosity=2)

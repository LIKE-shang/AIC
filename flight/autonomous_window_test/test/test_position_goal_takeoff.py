"""Vendor-style final-height position takeoff; no motor or flight IO."""
import copy,types,unittest
from unittest.mock import Mock
from test_takeoff_velocity import VelocitySim,PositionTarget
import test_takeoff_velocity as velocity_tests
from test_takeoff import TakeoffSim
from test_mission import Sim
from test_auto_transport import app
from mission import Mission,settings

class GoalSim(VelocitySim):
    def __init__(self):
        super().__init__()
        self.options.update(takeoff_vertical_velocity_enabled=False,takeoff_position_goal_enabled=True)
        self.m=Mission(self.m.core.c,self.options,experiment='takeoff_land');self.f.ctrl=self.m.core
        for _ in range(30):self.update()

class PositionGoal(unittest.TestCase):
    def test_full_target_only_after_observed_arming(self):
        s=GoalSim();s.start();s.until('ARM')
        self.assertEqual(s.m.position,s.m.origin)
        s.until('TAKEOFF');self.assertAlmostEqual(s.m.position[2]-s.m.origin[2],.9)
        self.assertIsNone(s.out['vertical_velocity_mps'])
        s.step();self.assertEqual(s.m.phase,'TAKEOFF')
        self.assertGreater(s.m.output()['tracking']['error_norm_m'],.12)
        self.assertEqual(s.m.output()['tracking']['tracking_check'],'horizontal_and_height_overshoot')
    def test_lagged_climb_completes_without_false_vertical_abort(self):
        s=GoalSim();s.start();s.until('DONE',limit=1700)
        self.assertTrue(s.m.takeoff_hover_completed)
        self.assertEqual(s.actions,['OFFBOARD','ARM','LAND'])
        self.assertTrue(all(e['reason']!='tracking_error' for e in s.m.events))
    def test_no_climb_times_out_even_if_airborne_flag_is_wrong(self):
        s=GoalSim();s.freeze=True;s.start();s.until('TAKEOFF');s.airborne=True
        s.until('DONE');self.assertFalse(s.m.takeoff_hover_completed)
        self.assertIn('takeoff_timeout',[e['reason'] for e in s.m.events])
    def test_horizontal_drift_still_stops(self):
        s=GoalSim();s.start();s.until('TAKEOFF')
        for _ in range(30):
            s.f.p=(s.f.p[0]+.005,s.f.p[1],s.f.p[2]);r=s.step(follow=False)
            if r['phase']=='FAULT':break
        self.assertEqual(s.m.phase,'FAULT')
        self.assertIn('takeoff_horizontal_tracking_error',s.m.reason)
    def test_excess_height_still_stops(self):
        from dataclasses import replace
        s=GoalSim();s.start();s.until('TAKEOFF')
        p=s.m.core.pose;s.m.core.pose=replace(p,p=(p.p[0],p.p[1],s.m.target_z+.121))
        out=s.m.tick(s.inputs())
        self.assertEqual(out['phase'],'FAULT');self.assertIn('takeoff_height_overshoot',out['reason'])
    def test_hover_restores_three_dimensional_tracking(self):
        from dataclasses import replace
        s=GoalSim();s.start();s.until('HOVER')
        p=s.m.core.pose;s.m.core.pose=replace(p,p=(p.p[0],p.p[1],s.m.target_z-.121))
        out=s.m.tick(s.inputs());self.assertIn('tracking_error',out['reason'])
    def test_modes_cannot_mix(self):
        s=Sim()
        with self.assertRaises(ValueError):settings(dict(s.options,takeoff_position_goal_enabled=True,takeoff_vertical_velocity_enabled=True))
    def test_transport_uses_xyz_position_not_vertical_velocity(self):
        r=velocity_tests.VelocityTransport().runtime()
        r.publish({'position':(1.,2.,1.145),'yaw':.2,'phase':'TAKEOFF','vertical_velocity_mps':None})
        m=r.motion.publish.call_args[0][0]
        self.assertFalse(m.type_mask & PositionTarget.IGNORE_PZ)
        self.assertTrue(m.type_mask & PositionTarget.IGNORE_VZ)
        self.assertAlmostEqual(m.position.z,1.145)
    def config(self):
        s=GoalSim()
        return {'experiment':'takeoff_land','mission':s.options,'expected_fcu_parameters':{'MPC_Z_VEL_MAX_UP':[0,.4],'MPC_ACC_UP_MAX':[0,.4]},
                'commissioning_authorization':{'scope':'supervised_takeoff_hover_land_only','operator_accepts_unverified_tests':True,'deferred_checks':sorted(app.COMMISSIONING_DEFERRALS)}}
    def test_declared_fcu_climb_bounds_required(self):
        c=self.config();self.assertTrue(app.commissioning_deferrals(c,True))
        for name in ['MPC_Z_VEL_MAX_UP','MPC_ACC_UP_MAX']:
            for val in [None,[0,.5],[0,float('nan')],[0,0.],[1,0.]]:
                c=self.config()
                if val is None:c['expected_fcu_parameters'].pop(name)
                else:c['expected_fcu_parameters'][name]=val
                with self.subTest(name=name,val=val),self.assertRaises(ValueError):app.commissioning_deferrals(c,True)

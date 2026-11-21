"""Synthetic bounds/plant/corridor are OFFLINE fixtures, never flight validation."""
import copy
import math
import unittest
import numpy as np
from window_live_vision.geometry import quaternion_matrix
from window_live_vision.traversal import (Safety, PlanRejected, plan_route, swept_cells, evaluate_corridor,
    TraversalModel, evidence_ready, motion_profile, yaw_rotation)
from test_rgbd import geometry


def safety():
    return Safety(position_error_bound_m=.015, depth_geometry_error_bound_m=.015,
        extrinsic_error_bound_m=.01, tracking_error_bound_m=.03, stopping_margin_m=.025,
        frame_thickness_bound_m=.03, geofence_min_m=(-3.,-3.,-3.), geofence_max_m=(5.,3.,4.),
        yaw_error_bound_rad=math.radians(2.),tilt_bound_rad=math.radians(2.),bounds_source='OFFLINE_SYNTHETIC_NOT_FLIGHT')


def pose(stamp=10.,position=(0.,0.,1.),yaw=0.,epoch=0):
    return dict(position=np.array(position,dtype=float),rotation=yaw_rotation(yaw),stamp=stamp,frame_id='map',epoch=epoch)


def synthetic_evidence(plan,stamp=10.):
    return dict(plan_id=plan['plan_id'],local_frame_id=plan['local_frame_id'],pose_epoch=plan['pose_epoch'],
                source_stamp=stamp,required_cells=100,observed_free_cells=100,unknown_cells=0,occupied_cells=0,clear=True,
                fixture='SYNTHETIC_ALL_FREE_NOT_SENSOR_VALIDATION')


class PlanningTests(unittest.TestCase):
    def test_observed_polygon_not_fixed_one_meter_opening(self):
        for width,height in ((1.1,.95),(1.4,1.2),(2.,1.5)):
            with self.subTest(width=width,height=height):
                plan=plan_route(geometry(width=width,height=height),pose(),safety(),10.)
                self.assertGreater(len(plan['safe_center_polygon_uv_m']),2)
                self.assertGreater(plan['path_length_m'],.1)  # full traverse is not the one-step mode
                self.assertFalse(plan['actual_output_permitted']);self.assertFalse(plan['accepted_real_window_pass'])
                # Account for vehicle envelope center below MAVROS body origin.
                self.assertAlmostEqual(plan['body_origin_crossing_enu_m'][2],1.09,places=5)

    def test_full_vehicle_and_uncertainty_prevent_center_only_false_ready(self):
        for width,height in ((.60,1.5),(1.5,.32),(.90,.70)):
            with self.subTest(width=width,height=height),self.assertRaises(PlanRejected):
                plan_route(geometry(width=width,height=height),pose(),safety(),10.)
        bigger=safety();bigger.position_error_bound_m=.18
        with self.assertRaises(PlanRejected):plan_route(geometry(width=1.,height=.85),pose(),bigger,10.)

    def test_offset_height_local_z_and_negative_map_z(self):
        g=geometry();g['center_enu_m'][2]=-1.5
        for corner in g['corners_enu_m']:corner[2]-=2.5
        plan=plan_route(g,pose(position=(0.,0.,-1.5)),safety(),10.)
        self.assertAlmostEqual(plan['body_origin_crossing_enu_m'][2],-1.41,places=6)

    def test_rotated_window_and_convex_perspective_polygon(self):
        g=geometry();r=yaw_rotation(.55)
        for name in ('center_enu_m','normal_enu'):g[name]=r.dot(g[name]).tolist()
        g['corners_enu_m']=np.asarray(g['corners_enu_m']).dot(r.T).tolist()
        plan=plan_route(g,pose(),safety(),10.)
        self.assertAlmostEqual(plan['target_yaw_rad'],.55)
        # Normal traversing segments, not chase an RGB center toward background.
        d=np.asarray(plan['waypoints_enu_m'][4])-plan['waypoints_enu_m'][2]
        self.assertLess(np.linalg.norm(np.cross(d,g['normal_enu'])),1e-8)
        g=geometry();g['corners_enu_m'][0][1]+=.12
        plan=plan_route(g,pose(),safety(),10.)
        self.assertGreater(plan['safe_center_min_slack_m'],.03)

    def test_geofence_checks_body_not_just_path_center(self):
        limits=safety();limits.geofence_max_m=(3.,3.,1.30)
        with self.assertRaisesRegex(PlanRejected,'geofence'):plan_route(geometry(),pose(),limits,10.)

    def test_missing_bounds_do_not_get_verified_by_plane_residual(self):
        with self.assertRaisesRegex(PlanRejected,'missing_safety_bounds'):
            plan_route(geometry(),pose(),Safety(),10.)
        limits=safety();limits.bounds_source='UNMEASURED'
        with self.assertRaisesRegex(PlanRejected,'provenance'):plan_route(geometry(),pose(),limits,10.)
        limits=safety();limits.max_speed_mps=.20
        with self.assertRaisesRegex(PlanRejected,'design_limits'):plan_route(geometry(),pose(),limits,10.)

    def test_stale_session_foreground_nonconvex_and_start_too_close_reject(self):
        cases=[]
        g=geometry();g['source_stamp']=9.;cases.append((g,pose()))
        g=geometry();g['pose_epoch']=2;cases.append((g,pose()))
        g=geometry();g['opening_evidence']['possible_opening']=False;cases.append((g,pose()))
        g=geometry();g['corners_enu_m'][0][0]+=.2;cases.append((g,pose()))
        g=geometry();g['corners_enu_m'][1][1]=-1.2;cases.append((g,pose()))
        cases.append((geometry(),pose(position=(1.5,0.,1.))))
        cases.append((geometry(),pose(stamp=9.)))
        for g,p in cases:
            with self.subTest(g=g,p=p),self.assertRaises(PlanRejected):plan_route(g,p,safety(),10.)


class CorridorTests(unittest.TestCase):
    def setUp(self):
        self.plan=plan_route(geometry(),pose(),safety(),10.)
        self.k=np.array([[500.,0.,320.],[0.,500.,240.],[0.,0.,1.]])
        self.r=quaternion_matrix([-.5,.5,-.5,.5])
        self.origin=np.array([-2.5,0.,1.09]) # independent SYNTHETIC upstream view

    def check(self,depth,origin=None):
        return evaluate_corridor(self.plan,depth,self.k,np.zeros(5),self.r,
                                 self.origin if origin is None else origin,10.,10.)

    def test_swept_volume_clear_with_actual_valid_far_returns(self):
        result=self.check(np.full((480,640),8.))
        self.assertTrue(result['clear']);self.assertEqual(result['unknown_cells'],0)
        self.assertGreater(result['required_cells'],1000)
        self.assertFalse(result['flight_validated']);self.assertFalse(result['actual_output_permitted'])
        self.assertTrue(evidence_ready(result,self.plan,10.1))

    def test_invalid_depth_is_unknown_never_infinite_free_range(self):
        for value in (0.,float('nan'),float('inf'),9.):
            result=self.check(np.full((480,640),value))
            self.assertFalse(result['clear']);self.assertGreater(result['unknown_cells'],0)
            self.assertEqual(result['observed_free_cells'],0)

    def test_foreground_obstacle_and_unseen_exit_block(self):
        depth=np.full((480,640),8.);depth[180:300,290:350]=3.
        result=self.check(depth)
        self.assertFalse(result['clear']);self.assertGreater(result['occupied_cells']+result['unknown_cells'],0)
        result=self.check(np.full((480,640),8.),origin=[.08,0.,.94])
        self.assertFalse(result['clear']);self.assertGreater(result['unknown_cells'],0) # own rear/FOV not seen

    def test_stale_raw_distortion_and_wrong_scoped_evidence_block(self):
        with self.assertRaises(PlanRejected):evaluate_corridor(self.plan,np.full((480,640),8.),self.k,np.ones(5)*.01,self.r,self.origin,10.,10.)
        with self.assertRaises(PlanRejected):evaluate_corridor(self.plan,np.full((480,640),8.),self.k,np.zeros(5),self.r,self.origin,10.,11.)
        evidence=synthetic_evidence(self.plan)
        for change in ({'plan_id':'old'},{'source_stamp':9.},{'unknown_cells':1},{'observed_free_cells':99},{'pose_epoch':2},{'clear':False}):
            self.assertFalse(evidence_ready(dict(evidence,**change),self.plan,10.))

    def test_grid_contains_whole_body_near_plane_and_exit(self):
        cells,size=swept_cells(self.plan)
        self.assertEqual(size,.05)
        self.assertGreater(cells[:,0].max(),self.plan['waypoints_enu_m'][4][0]+.3)
        self.assertLess(cells[:,0].min(),-.3)
        with self.assertRaises(PlanRejected):swept_cells(self.plan,cell_m=.01)


class ProfileTests(unittest.TestCase):
    def test_analytic_profile_speed_accel_stop_and_no_ten_cm_restarts(self):
        for length in (.025,.10,1.5):
            times=np.arange(0.,70.,.05)
            rows=[motion_profile(length,t,.04,.02) for t in times]
            positions=np.array([r[0] for r in rows]);speeds=np.array([r[1] for r in rows])
            self.assertLessEqual(np.max(speeds),.0400001)
            self.assertLessEqual(np.max(np.abs(np.diff(speeds))/.05),.0200001)
            self.assertAlmostEqual(positions[-1],length)
            self.assertEqual(speeds[-1],0.)
            self.assertLessEqual(np.max(np.diff(positions))/.05,.0400001)

    def test_profile_invalid_inputs_reject(self):
        for args in ((-.1,1,.04,.02),(.1,float('nan'),.04,.02),(.1,1,0,.02)):
            with self.assertRaises(ValueError):motion_profile(*args)


class StateMachineTests(unittest.TestCase):
    def setup_model(self):
        plan=plan_route(geometry(),pose(),safety(),10.)
        model=TraversalModel(plan,pose(),10.,synthetic_evidence(plan),'OFFLINE_SIMULATION_ONLY')
        return model,plan

    def test_never_claims_a_real_flight_controller_or_permission(self):
        model,plan=self.setup_model()
        self.assertFalse(model.output()['actual_output_permitted'])
        self.assertEqual(model.output()['output_sent_count'],0)
        with self.assertRaisesRegex(PlanRejected,'not_a_flight_controller'):
            TraversalModel(plan,pose(),10.,synthetic_evidence(plan),'A_PRESSED')

    def test_complete_offline_pass_with_frame_loss_only_after_commit(self):
        model,plan=self.setup_model();states=[];targets=[model.target.copy()];speeds=[0.]
        now=10.
        for _ in range(3000):
            now+=.05
            p=pose(now,model.target,model.yaw)
            g=None if model.frozen_stamp else geometry(now)
            output=model.tick(p,now,g,synthetic_evidence(plan,now))
            states.append(output['state']);targets.append(model.target.copy());speeds.append(output['advisory_speed_mps'])
            if model.state in ('DONE','ABORT'):break
        self.assertEqual(model.state,'DONE',model.reason)
        self.assertTrue(set(('ALIGN','APPROACH','COMMIT','TRAVERSE','EXIT','DONE')).issubset(set(states)))
        self.assertFalse(output['accepted_real_window_pass'])
        np.testing.assert_allclose(model.target,plan['waypoints_enu_m'][4],atol=1e-6)
        speed=np.linalg.norm(np.diff(np.asarray(targets),axis=0),axis=1)/.05
        self.assertLessEqual(np.max(speed),.040001)
        self.assertLessEqual(np.max(np.abs(np.diff(speeds)))/.05,.020001)

    def test_geometry_loss_before_commit_cancels_not_blind_forward(self):
        model,plan=self.setup_model()
        output=model.tick(pose(10.05),10.05,None,synthetic_evidence(plan,10.05))
        self.assertEqual(output['state'],'ABORT');self.assertIsNone(output['advisory_target_position_enu_m'])

    def test_takeover_stale_pose_clock_jump_session_and_unknown_corridor(self):
        for change in ('takeover','stale_pose','clock','epoch','corridor','geometry','tilt'):
            model,plan=self.setup_model();p=pose(10.05);g=geometry(10.05);e=synthetic_evidence(plan,10.05);now=10.05;pilot=True
            if change=='takeover':pilot=False
            if change=='stale_pose':p['stamp']=9.
            if change=='clock':now=11.;p['stamp']=11.;e['source_stamp']=11.
            if change=='epoch':p['epoch']=2
            if change=='corridor':e['unknown_cells']=1
            if change=='geometry':g['corners_enu_m'][0][1]+=.04
            if change=='tilt':p['rotation']=np.array([[1,0,0],[0,math.cos(.1),-math.sin(.1)],[0,math.sin(.1),math.cos(.1)]])
            with self.subTest(change=change):self.assertEqual(model.tick(p,now,g,e,pilot)['state'],'ABORT')

    def test_failure_in_frame_never_asserts_blind_hold_or_retreat_safe(self):
        model,plan=self.setup_model();model.state='TRAVERSE';model.frozen_stamp=10.
        model.target=np.array(plan['body_origin_crossing_enu_m']);p=pose(10.05,model.target)
        e=synthetic_evidence(plan,10.05);e['unknown_cells']=1
        output=model.tick(p,10.05,None,e)
        self.assertEqual(output['state'],'ABORT');self.assertIn('EMERGENCY_MANUAL',output['abort_action'])
        self.assertIsNone(output['advisory_target_position_enu_m'])
    def test_constructor_refuses_stale_plan_or_different_start(self):
        _,plan=self.setup_model()
        for p,t in ((pose(epoch=2),10.),(pose(position=(.10,0,1)),10.),(pose(yaw=.1),10.),(pose(11.),11.)):
            with self.subTest(pose=p),self.assertRaises(PlanRejected):
                TraversalModel(plan,p,t,synthetic_evidence(plan,t),'OFFLINE_SIMULATION_ONLY')

    def test_malformed_live_data_returns_abort_and_never_a_target(self):
        for corrupt in ('stamp','corners','normal','center','evidence','pose','authorization'):
            model,plan=self.setup_model();p=pose(10.05);g=geometry(10.05);e=synthetic_evidence(plan,10.05);pilot=True
            if corrupt=='stamp':g['source_stamp']='invalid'
            if corrupt=='corners':g['corners_enu_m']=[[float('nan')]*3]*4
            if corrupt=='normal':g['normal_enu']=[0,0,1]
            if corrupt=='center':g['center_enu_m'][1]+=.1
            if corrupt=='evidence':e['source_stamp']=None
            if corrupt=='pose':p['rotation']=None
            if corrupt=='authorization':pilot='true'
            with self.subTest(corrupt=corrupt):
                out=model.tick(p,10.05,g,e,pilot)
                self.assertEqual(out['state'],'ABORT');self.assertIsNone(out['advisory_target_position_enu_m'])

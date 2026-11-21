import math
import unittest
from types import SimpleNamespace
import numpy as np
from window_live_vision.detector import Detection, quad_center
from window_live_vision.tracking import Tracker
from window_live_vision.geometry import (PoseBuffer, camera_model, matched_depth, estimate_plane,
    step_preview, preview_setpoint, quaternion_matrix, rays, border_samples)


def detection(dx=0):
    q=np.float32([[180+dx,130],[460+dx,130],[460+dx,350],[180+dx,350]])
    return Detection(q,quad_center(q),.9,'test',[dict(band_fraction=.10)]*4)


def camera(stamp=10.):
    return dict(stamp=stamp,frame_id='rgb_optical',width=640,height=480,
        K=[500.,0.,320.,0.,500.,240.,0.,0.,1.],D=[0.]*5,model='plumb_bob')


class TrackingTests(unittest.TestCase):
    def confirmed(self, tracker, dx=0):
        for i in range(3):
            out=tracker.update([detection(dx)],10.+i*.2,10.+i*.2,20.+i*.2,(480,640),[320,240])
        self.assertTrue(out['ready'])
        return out

    def test_confirmation_then_direction_follows_live_image(self):
        tracker=Tracker()
        left=self.confirmed(tracker,-45)
        self.assertEqual(left['visual_instruction']['direction'],'IMAGE_LEFT')
        right=tracker.update([detection(45)],10.6,10.6,20.6,(480,640),[320,240])
        if not right['ready']:
            tracker.update([detection(45)],10.8,10.8,20.8,(480,640),[320,240])
            right=tracker.update([detection(45)],11.,11.,21.,(480,640),[320,240])
        self.assertEqual(right['visual_instruction']['direction'],'IMAGE_RIGHT')
        self.assertIsNone(right['visual_instruction']['metric_delta_m'])

    def test_loss_and_multiple_cancel_immediately(self):
        for candidates in ([],[detection(),detection(90)]):
            tracker=Tracker();self.confirmed(tracker)
            out=tracker.update(candidates,10.6,10.6,20.6,(480,640))
            self.assertFalse(out['ready']);self.assertIsNone(out['visual_instruction'])
            out=tracker.update([detection()],10.8,10.8,20.8,(480,640))
            self.assertFalse(out['ready'])

    def test_watchdog_invalidates_when_ros_clock_stops(self):
        tracker=Tracker();self.confirmed(tracker)
        out=tracker.watchdog(21.)
        self.assertFalse(out['ready']);self.assertIsNone(tracker.last)

    def test_timestamp_failures_cancel(self):
        for stamp,now in ((10.4,10.4),(10.3,10.5),(10.6,11.2),(10.6,10.4),(float('nan'),11.),(0.,11.)):
            with self.subTest(stamp=stamp,now=now):
                tracker=Tracker();self.confirmed(tracker)
                out=tracker.update([detection()],stamp,now,20.6,(480,640))
                self.assertFalse(out['ready']);self.assertIsNone(out['visual_instruction'])

    def test_large_ros_clock_jump_requires_new_confirmation(self):
        tracker=Tracker();self.confirmed(tracker)
        self.assertFalse(tracker.update([detection()],20.,20.,20.6,(480,640))['ready'])

    def test_reacquisition_after_spatial_jump(self):
        tracker=Tracker();self.confirmed(tracker,-80)
        out=tracker.update([detection(150)],10.6,10.6,20.6,(480,640))
        self.assertFalse(out['ready'])

    def test_calibration_reference_and_vertical_no_motion(self):
        tracker=Tracker()
        for i in range(3):
            out=tracker.update([detection()],10.+.2*i,10.+.2*i,20.+.2*i,(480,640),[300,200])
        instruction=out['visual_instruction']
        self.assertEqual(instruction['error_px'],[20.,40.])
        self.assertFalse(instruction['altitude_change_requested'])
        self.assertFalse(instruction['yaw_change_requested'])
        self.assertFalse(instruction['forward_change_requested'])


class GeometryTests(unittest.TestCase):
    def test_calibration_validation(self):
        k,d=camera_model(camera(),'rgb_optical',640,480,10.)
        np.testing.assert_allclose(rays([[320,240]],k,d),[[0,0,1]])
        for change in ({'K':[0]*9},{'stamp':7.},{'stamp':float('nan')},{'frame_id':'depth'},
            {'D':[float('nan')]*5},{'binning_x':2},{'model':'equidistant'},{'width':320}):
            with self.subTest(change=change),self.assertRaises(ValueError):
                camera_model(dict(camera(),**change),'rgb_optical',640,480,10.)

    def test_depth_units_and_registration(self):
        d16=np.full((480,640),2000,np.uint16)
        values,k,d=matched_depth(d16,'16UC1',10.,'rgb_optical',camera(),camera(),10.,'rgb_optical',(480,640))
        self.assertEqual(values[0,0],2.)
        d32=np.full((480,640),2.,np.float32)
        values,_,_=matched_depth(d32,'32FC1',10.,'rgb_optical',camera(),camera(),10.,'rgb_optical',(480,640))
        self.assertEqual(values[0,0],2.)
        for stamp,frame,info,encoding in [(10.10,'rgb_optical',camera(),'16UC1'),(10.,'depth',camera(),'16UC1'),
            (10.,'rgb_optical',dict(camera(),K=[550,0,320,0,500,240,0,0,1]),'16UC1'),(10.,'rgb_optical',camera(),'8UC1')]:
            with self.subTest(stamp=stamp,frame=frame),self.assertRaises(ValueError):
                matched_depth(d16,encoding,stamp,frame,camera(),info,10.,'rgb_optical',(480,640))

    def test_depth_uses_border_not_background_inside_opening(self):
        k,d=camera_model(camera(),'rgb_optical',640,480,10.)
        depth=np.full((480,640),2.)
        depth[131:350,181:460]=5.5
        plane=estimate_plane(detection(),depth,k,d)
        np.testing.assert_allclose(plane['center_optical_m'],[0,0,2.],atol=1e-6)
        np.testing.assert_allclose(plane['normal_optical'],[0,0,1.],atol=1e-6)
        self.assertAlmostEqual(plane['corners_optical_m'][1,0]-plane['corners_optical_m'][0,0],1.12)

    def test_variable_real_scale_is_measured(self):
        k,d=camera_model(camera(),'rgb_optical',640,480,10.)
        a=estimate_plane(detection(),np.full((480,640),1.5),k,d)
        b=estimate_plane(detection(),np.full((480,640),3.),k,d)
        np.testing.assert_allclose(b['corners_optical_m'],2*a['corners_optical_m'],atol=1e-6)

    def test_missing_one_depth_side_or_disagreeing_planes_reject(self):
        k,d=camera_model(camera(),'rgb_optical',640,480,10.)
        pixels,labels=border_samples(detection(),(480,640))
        for value in (0.,float('nan'),3.):
            depth=np.full((480,640),2.)
            chosen=pixels[labels==2]
            depth[chosen[:,1],chosen[:,0]]=value
            with self.subTest(value=value),self.assertRaises(ValueError):
                estimate_plane(detection(),depth,k,d)

    def test_pose_interpolation_and_reset(self):
        poses=PoseBuffer()
        poses.add(10.,[0,0,0],[0,0,0,1],'map')
        poses.add(10.08,[.08,0,0],[0,0,0,1],'map')
        np.testing.assert_allclose(poses.at(10.04)['position'],[.04,0,0],atol=1e-6)
        for stamp in (9.99,10.081):
            with self.assertRaises(ValueError):poses.at(stamp)
        poses.add(1.,[0,0,0],[0,0,0,1],'map')
        self.assertEqual(len(poses.samples),1)
        with self.assertRaises(ValueError):poses.at(10.04)

    def test_pose_antipodal_quaternion_and_gap(self):
        poses=PoseBuffer()
        poses.add(10.,[0,0,0],[0,0,0,1],'map')
        poses.add(10.08,[0,0,0],[0,0,0,-1],'map')
        np.testing.assert_allclose(poses.at(10.04)['rotation'],np.eye(3))
        poses.add(11.,[0,0,0],[0,0,0,1],'map')
        with self.assertRaises(ValueError):poses.at(10.5)

    def test_preview_respects_bounds_height_yaw_and_plane_normal(self):
        k,d=camera_model(camera(),'rgb_optical',640,480,10.)
        for dx,yaw in [(100,0),(-100,.7),(0,-.4)]:
            with self.subTest(dx=dx,yaw=yaw):
                plane=estimate_plane(detection(dx),np.full((480,640),2.),k,d)
                rotation=quaternion_matrix([0,0,math.sin(yaw/2),math.cos(yaw/2)])
                pose=dict(rotation=rotation,position=np.array([1.,2.,-.8]),frame_id='map',stamp=10.)
                proposal=step_preview(plane,pose)
                delta=np.array(proposal['target_position_enu'])-proposal['origin_position_enu']
                self.assertLessEqual(np.linalg.norm(delta),.100000001)
                self.assertAlmostEqual(delta[2],0.)
                self.assertAlmostEqual(np.dot(delta,proposal['window_normal_enu']),0.)
                self.assertAlmostEqual(proposal['target_yaw_enu_rad'],yaw)
                self.assertEqual(proposal['max_target_speed_mps'],.04)
                self.assertFalse(proposal['actual_output_permitted'])
                self.assertFalse(proposal['clearance_verified'])
                if dx:self.assertEqual(np.sign(proposal['planned_lateral_step_m']),np.sign(dx))

    def test_plane_tilt_and_body_mount_transform(self):
        # An oblique plane is observable without supplying its normal/size.
        k,d=camera_model(camera(),'rgb_optical',640,480,10.)
        yy,xx=np.indices((480,640))
        n=np.array([.25,0.,1.]); n/=np.linalg.norm(n)
        depth=2./(n[0]*(xx-320)/500+n[2])
        plane=estimate_plane(detection(35),depth,k,d)
        np.testing.assert_allclose(plane['normal_optical'],n,atol=1e-6)
        p=step_preview(plane,dict(rotation=np.eye(3),position=np.zeros(3),frame_id='map',stamp=10.))
        np.testing.assert_allclose(np.dot(np.array(p['target_position_enu']),p['window_normal_enu']),0,atol=1e-6)


class PreviewProfileTests(unittest.TestCase):
    def test_frozen_ramp_is_bounded_not_repeated_ten_centimeter_steps(self):
        p=dict(origin_position_enu=[1.,2.,-.8],target_position_enu=[1.,2.1,-.8],target_yaw_enu_rad=.4)
        previous=np.array(p['origin_position_enu'])
        for elapsed in np.arange(.1,4.,.1):
            sample=preview_setpoint(p,elapsed)
            current=np.array(sample['target_position_enu'])
            self.assertLessEqual(np.linalg.norm(current-previous)/.1,.040000001)
            self.assertAlmostEqual(current[2],-.8)
            self.assertLessEqual(np.linalg.norm(current-p['origin_position_enu']),.100000001)
            self.assertEqual(sample['target_yaw_enu_rad'],.4)
            previous=current
        np.testing.assert_allclose(preview_setpoint(p,100.)['target_position_enu'],p['target_position_enu'])
        np.testing.assert_allclose(preview_setpoint(p,-1.)['target_position_enu'],p['origin_position_enu'])

    def test_profile_rejects_nonfinite_time_or_oversized_budget(self):
        p=dict(origin_position_enu=[0,0,0],target_position_enu=[0,.11,0],target_yaw_enu_rad=0)
        with self.assertRaises(ValueError):preview_setpoint(p,1.)
        p['target_position_enu']=[0,0,0]
        with self.assertRaises(ValueError):preview_setpoint(p,float('nan'))
        self.assertEqual(preview_setpoint(p,1.)['command_progress_m'],0.)

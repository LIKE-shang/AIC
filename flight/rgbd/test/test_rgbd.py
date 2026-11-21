import copy
import math
import importlib.util
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np
from window_live_vision.detector import Detector
from window_live_vision.geometry import PoseBuffer, quaternion_matrix, estimate_plane
from window_live_vision.rgbd import (DepthRegistrar, DepthOpeningDetector, MetricTracker,
    aperture_evidence, fuse_candidates, world_geometry)
from window_live_vision.astra_contract import factory_checked_raw_info, factory_checked_tf
from test_guidance import camera, detection


def depth_scene(background=4., missing=None):
    meters = np.full((480, 640), background, np.float64)
    meters[105:375, 155:485] = 2.
    meters[125:355, 175:465] = background
    if missing == 'top':
        meters[100:130, 155:485] = background
    return meters


def geometry(stamp=10., x=2., width=1.4, height=1.2):
    return dict(center_enu_m=[x, 0., 1.], normal_enu=[1., 0., 0.],
        corners_enu_m=[[x,-width/2,1+height/2],[x,width/2,1+height/2],[x,width/2,1-height/2],[x,-width/2,1-height/2]],
        local_frame_id='map', pose_epoch=0, source_stamp=stamp, plane_rms_m=.005,
        opening_evidence=dict(possible_opening=True, interior_unknown_fraction=0.))


class RegistrationTests(unittest.TestCase):
    def raw(self):
        rgb = camera()
        depth = dict(rgb, frame_id='depth_optical', width=320, height=240,
                     K=[250.,0.,160.,0.,250.,120.,0.,0.,1.])
        return rgb, depth

    def test_unequal_resolutions_explicit_reprojection(self):
        rgb, info = self.raw()
        registrar = DepthRegistrar()
        values, _, _ = registrar.register(np.full((240,320),2000,np.uint16),'16UC1',10.,'depth_optical',
            rgb, info, 10., 'rgb_optical',(480,640),np.eye(3),[.04,0.,0.])
        # Center raw pixel maps to x=330, not a resized x=320.
        self.assertEqual(values[240,330],2.)
        self.assertTrue(np.isnan(values[241,330]))  # no interpolation/hole fill
        self.assertTrue(np.isnan(values[240,0]))
        cached = registrar.cached_rays
        registrar.register(np.full((240,320),2000,np.uint16),'16UC1',10.,'depth_optical',
            rgb,info,10.,'rgb_optical',(480,640),np.eye(3),[.04,0.,0.])
        self.assertIs(cached,registrar.cached_rays)

    def test_units_zbuffer_and_rotated_camera(self):
        rgb, info = self.raw()
        # Very low RGB focal length deliberately maps adjacent depth pixels
        # onto one RGB pixel; nearer surface must win the z-buffer.
        rgb['K']=[50.,0.,320.,0.,50.,240.,0.,0.,1.]
        depth = np.full((240,320),3.,np.float32);depth[120,160]=2.
        values, _, _ = DepthRegistrar().register(depth,'32FC1',10.,'depth_optical',rgb,info,
            10.,'rgb_optical',(480,640),np.eye(3),[.04,0.,0.])
        self.assertEqual(values[240,321],2.)

    def test_rejects_bad_time_frames_baseline_calibration_and_units(self):
        rgb, info = self.raw()
        base = [np.full((240,320),2000,np.uint16),'16UC1',10.,'depth_optical',rgb,info,10.,'rgb_optical',(480,640),np.eye(3),[.04,0.,0.]]
        cases = [(2,10.1),(3,'rgb_optical'),(9,np.eye(3)*2),(10,[0.,0.,0.]),(10,[.4,0.,0.]),(1,'8UC1'),
                 (5,dict(info,frame_id='another')), (5,dict(info,K=[0.]*9))]
        for index,value in cases:
            args=list(base);args[index]=value
            with self.subTest(index=index,value=value),self.assertRaises(ValueError):DepthRegistrar().register(*args)


class DepthDetectionTests(unittest.TestCase):
    def setUp(self):
        self.k=np.array(camera()['K']).reshape(3,3);self.d=np.zeros(5)

    def test_untextured_rgb_uses_depth_hole(self):
        rgb=np.full((480,640,3),140,np.uint8)
        self.assertEqual(Detector().detect(rgb)[0],[])
        found=DepthOpeningDetector().detect(depth_scene(),self.k,self.d)
        self.assertEqual(len(found),1)
        self.assertEqual(found[0].method,'depth_four_border_plane_hole')
        plane=estimate_plane(found[0],depth_scene(),self.k,self.d)
        self.assertAlmostEqual(plane['plane_distance_m'],2.,places=2)
        self.assertTrue(aperture_evidence(found[0],depth_scene(),self.k,self.d,plane)['possible_opening'])

    def test_unknown_opening_is_candidate_not_free(self):
        meters=depth_scene(float('nan'))
        found=DepthOpeningDetector().detect(meters,self.k,self.d)
        self.assertEqual(len(found),1)
        plane=estimate_plane(found[0],meters,self.k,self.d)
        result=aperture_evidence(found[0],meters,self.k,self.d,plane)
        self.assertGreater(result['interior_unknown_fraction'],.99)
        self.assertFalse(result['unknown_is_free']);self.assertFalse(result['free_corridor_verified'])

    def test_no_depth_frame_and_broken_top_reject(self):
        for meters in (np.full((480,640),2.),np.full((480,640),np.nan),depth_scene(missing='top')):
            self.assertEqual(DepthOpeningDetector().detect(meters,self.k,self.d),[])

    def test_painted_wall_and_front_obstruction_reject_aperture(self):
        candidate=detection()
        for value in (2.,1.5):
            meters=np.full((480,640),2.);meters[140:340,190:450]=value
            plane=estimate_plane(candidate,meters,self.k,self.d)
            result=aperture_evidence(candidate,meters,self.k,self.d,plane)
            self.assertFalse(result['possible_opening'])

    def test_rgb_and_depth_fusion_does_not_hide_distinct_targets(self):
        rgb=detection();same=detection(1);other=detection(220)
        fused=fuse_candidates([rgb],[same,other])
        self.assertEqual(len(fused),2);self.assertIn('+depth_plane',fused[0].method)

    def test_metric_world_transform_uses_actual_pose_not_ground_height(self):
        plane=estimate_plane(detection(),np.full((480,640),2.),self.k,self.d)
        pose=dict(stamp=10.,frame_id='map',epoch=4,position=np.array([.1,.2,-1.6]),rotation=np.eye(3))
        result=world_geometry(plane,pose)
        self.assertEqual(result['pose_epoch'],4)
        self.assertAlmostEqual(result['center_enu_m'][0],2.18,places=6)
        self.assertAlmostEqual(result['center_enu_m'][2],-1.66,places=6)
        self.assertFalse(result['metric_accuracy_verified'])


class MetricTrackingTests(unittest.TestCase):
    def stable(self,tracker):
        for i in range(5):result=tracker.update(geometry(10.+.2*i),10.+.2*i)
        self.assertTrue(result['ready'])
        return result

    def test_five_time_spanning_frames_not_pixel_confirmation(self):
        tracker=MetricTracker()
        for i in range(4):self.assertFalse(tracker.update(geometry(10.+.2*i),10.+.2*i)['ready'])
        result=tracker.update(geometry(10.8),10.8)
        self.assertTrue(result['ready']);self.assertFalse(result['geometry']['scatter_is_accuracy_bound'])

    def test_loss_reset_stale_jumps_and_new_map_epoch(self):
        for change in ('stale','center','corners','epoch','clock','normal'):
            tracker=MetricTracker();self.stable(tracker);current=geometry(11.)
            now=11.
            if change=='stale':now=12.
            if change=='center':current['center_enu_m'][0]+=.12
            if change=='corners':current['corners_enu_m'][0][1]+=.2
            if change=='epoch':current['pose_epoch']=1
            if change=='clock':current['source_stamp']=10.6
            if change=='normal':current['normal_enu']=[0.,1.,0.]
            with self.subTest(change=change):self.assertFalse(tracker.update(current,now)['ready'])
        tracker=MetricTracker();self.stable(tracker)
        self.assertFalse(tracker.reset('depth_lost')['ready']);self.assertEqual(len(tracker.rows),0)

    def test_pose_reset_without_renaming_map_changes_epoch(self):
        buffer=PoseBuffer();buffer.add(10.,[0,0,0],[0,0,0,1],'map')
        buffer.add(10.02,[2,0,0],[0,0,0,1],'map')
        self.assertEqual(buffer.at(10.02)['epoch'],1)


class AstraContractTests(unittest.TestCase):
    def data(self):
        rgb=dict(camera(),frame_id='camera_color_optical_frame')
        info=dict(camera(),frame_id='',height=480)
        params=dict(success=True,l_intr_p=[500,500,320,240],r_intr_p=[500,500,320,240],
                    r2l_r=np.eye(3).ravel().tolist(),r2l_t=[40,0,0])
        return rgb,info,params

    def test_only_missing_raw_info_frame_is_repaired_after_factory_checks(self):
        rgb,info,params=self.data()
        result=factory_checked_raw_info(rgb,info,'camera_depth_optical_frame','camera_color_optical_frame',params,False)
        self.assertEqual(result['frame_id'],'camera_depth_optical_frame')
        self.assertEqual(info['frame_id'],'');self.assertEqual(result['K'],info['K'])
        self.assertFalse(result['distortion_and_temporal_accuracy_flight_verified'])

    def test_wrong_registration_metadata_factory_and_profile_fail_closed(self):
        rgb,info,params=self.data()
        base=[rgb,info,'camera_depth_optical_frame','camera_color_optical_frame',params,False]
        cases=[(5,True),(5,None),(2,'camera_color_optical_frame'),(1,dict(info,frame_id='color')),
               (1,dict(info,height=240)),(1,dict(info,K=[550,0,320,0,500,240,0,0,1])),
               (4,dict(params,success=False)),(4,dict(params,r2l_t=[0,0,0])),(4,dict(params,r2l_r=[0]*9))]
        for index,value in cases:
            args=list(base);args[index]=value
            with self.subTest(index=index),self.assertRaises(ValueError):factory_checked_raw_info(*args)

    def test_tf_must_match_driver_factory_chain_including_rotation_and_sign(self):
        _,_,params=self.data()
        self.assertFalse(factory_checked_tf(np.eye(3),[.04,0.,0.],params)['physical_extrinsics_verified'])
        with self.assertRaisesRegex(ValueError,'tf_disagrees'):
            factory_checked_tf(np.eye(3),[-.04,0.,0.],params)
        angle=.03
        raw=np.array([[math.cos(angle),0.,math.sin(angle)],[0.,1.,0.],[-math.sin(angle),0.,math.cos(angle)]])
        params['r2l_r']=raw.ravel().tolist()
        f=quaternion_matrix([-.5,.5,-.5,.5]);q=f.dot(raw.T).dot(f.T)
        expected_t=f.T.dot(q).dot(f).dot(q.T).dot([.04,0.,0.])
        factory_checked_tf(raw.T,expected_t,params)
        with self.assertRaisesRegex(ValueError,'tf_disagrees'):
            factory_checked_tf(raw,expected_t,params)


class GroundGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=Path(__file__).resolve().parents[1]/'scripts/ground_camera_guard.py'
        spec=importlib.util.spec_from_file_location('guard_test',path)
        cls.guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.guard)

    def test_duplicates_are_refused_not_killed(self):
        system=[[['/camera/rgb/image_raw',['/camera/camera']],['/other',['/foo']]],[],[]]
        self.assertEqual(self.guard.duplicate_publishers(system,['/camera/rgb/image_raw']),{'/camera/rgb/image_raw':['/camera/camera']})

    def test_real_uvc_product_discovery_not_pro_name_guess(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);usb=root/'usb';usb.mkdir()
            (usb/'idVendor').write_text('2bc5');(usb/'idProduct').write_text('0511');(usb/'product').write_text('Gemini test')
            video=root/'video0';video.mkdir();(video/'device').mkdir()
            (video/'device'/'idVendor').write_text('2bc5');(video/'device'/'idProduct').write_text('0511')
            (video/'device'/'product').write_text('Real USB model')
            result=self.guard.uvc_devices(str(root))
            self.assertEqual(len(result),1);self.assertEqual(result[0]['product_id'],0x0511)
class FastMetricConfirmationTests(unittest.TestCase):
    def test_three_distinct_frames_span_and_processing_gap(self):
        tracker=MetricTracker(fast_confirmation=True)
        for i,t in enumerate((10.,10.43,10.86)):
            result=tracker.update(geometry(t),t+.1)
            self.assertEqual(result['ready'],i==2)
        self.assertEqual(result['geometry']['confirmation_frames'],3)
        self.assertAlmostEqual(result['geometry']['confirmation_span_s'],.86)
    def test_fast_does_not_accept_stale_epoch_jump_or_duplicate(self):
        for case in ('stale','epoch','duplicate','gap','corner'):
            tracker=MetricTracker(fast_confirmation=True)
            tracker.update(geometry(10.),10.);tracker.update(geometry(10.4),10.4)
            g=geometry(10.8);now=10.8
            if case=='stale':now=11.4
            if case=='epoch':g['pose_epoch']=2
            if case=='duplicate':g['source_stamp']=10.4
            if case=='gap':g['source_stamp']=11.1;now=11.1
            if case=='corner':g['corners_enu_m'][0][1]+=.2
            with self.subTest(case=case):self.assertFalse(tracker.update(g,now)['ready'])
    def test_fast_requires_time_span_not_three_near_duplicates(self):
        tracker=MetricTracker(fast_confirmation=True)
        for t in (10.,10.05,10.1):self.assertFalse(tracker.update(geometry(t),t)['ready'])
    def test_latest_obstruction_not_averaged_away(self):
        tracker=MetricTracker(fast_confirmation=True)
        for t in (10.,10.2):tracker.update(geometry(t),t)
        g=geometry(10.4);g['opening_evidence']={'possible_opening':False}
        result=tracker.update(g,10.4)
        self.assertFalse(result['geometry']['opening_evidence']['possible_opening'])

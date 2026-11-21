"""ROS-facing plumbing checked with explicit fakes, NOT a ROS runtime test."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from scenes import scene
from window_live_vision.detector import Detector
from window_live_vision.tracking import Tracker
from window_live_vision.geometry import PoseBuffer
from window_live_vision.rgbd import DepthRegistrar, DepthOpeningDetector, MetricTracker
from window_live_vision.traversal import Safety
ROOT=Path(__file__).resolve().parents[1]


class Clock:
    now_value=10.
    def __init__(self,value=0.):self.value=value
    def to_sec(self):return self.value
    @classmethod
    def now(cls):return cls(cls.now_value)


class String:
    def __init__(self,data=''):self.data=data


class Pub:
    def __init__(self):self.rows=[]
    def publish(self,msg):self.rows.append(msg)


class Bridge:
    def imgmsg_to_cv2(self,msg,desired_encoding):return msg.array
    def cv2_to_imgmsg(self,frame,encoding):return types.SimpleNamespace(array=frame,encoding=encoding)


def load_node():
    fake=types.ModuleType('rospy')
    fake.Time=Clock
    fake.loginfo_throttle=lambda *a:None
    fake.logwarn_throttle=lambda *a:None
    fake_modules={'rospy':fake}
    bridge=types.ModuleType('cv_bridge');bridge.CvBridge=Bridge;fake_modules['cv_bridge']=bridge
    for package,names in [('std_msgs',['String']),('sensor_msgs',['Image','CameraInfo']),('geometry_msgs',['PoseStamped'])]:
        parent=types.ModuleType(package);message=types.ModuleType(package+'.msg')
        for name in names:setattr(message,name,String if name=='String' else type(name,(),{}))
        parent.msg=message;fake_modules[package]=parent;fake_modules[package+'.msg']=message
    with patch.dict(sys.modules,fake_modules):
        spec=importlib.util.spec_from_file_location('live_node_test',ROOT/'scripts/live_node.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


MODULE=load_node()


def node():
    n=object.__new__(MODULE.LiveNode)
    n.bridge=Bridge();n.detector=Detector();n.tracker=Tracker();n.pose=PoseBuffer()
    n.lock=threading.RLock();n.seq=0;n.status={};n.use_depth=False
    n.monitor=MODULE.PipelineMonitor();n.fps=5.;n.enable_traversal=True;n.factory_align=None
    n.info=None;n.depth_info=None;n.depth_info_msg=None;n.depths=[];n.last_pose_wall=None
    n.registrar=DepthRegistrar();n.metric_tracker=MetricTracker();n.depth_detector=DepthOpeningDetector()
    n.depth_candidates=True;n.registration_mode='strict_registered';n.safety=Safety()
    n.astra_factory=False;n.factory_params=None;n.transform_provider=None
    n.factory_wall=None;n.factory_thread=None;n.factory_attempt_wall=None
    n.calibration_signature=None;n.calibration_epoch=0;n.camera_prefix='camera'
    n.camera_xyz=[.08,0.,-.06];n.camera_q=[-.5,.5,-.5,.5]
    n.status_pub=Pub();n.proposal_pub=Pub();n.image_pub=Pub();n.source_pub=Pub()
    n.geometry_pub=Pub();n.traversal_pub=Pub();n.depth_source_pub=Pub();n.depth_source_info_pub=Pub()
    return n


def image(frame,stamp=10.,encoding='bgr8'):
    return types.SimpleNamespace(header=types.SimpleNamespace(stamp=Clock(stamp),frame_id='rgb_optical'),
        width=frame.shape[1],height=frame.shape[0],encoding=encoding,array=frame)


class NodeTests(unittest.TestCase):
    def test_rgb_stream_live_direction_and_loss_publish_cancel(self):
        n=node();frame,_=scene(center=(370,240))
        for i in range(3):
            stamp=10.+.2*i;Clock.now_value=stamp
            with patch.object(MODULE.time,'monotonic',return_value=20.+.2*i):n.process(image(frame,stamp),20.+.2*i)
        self.assertTrue(n.status['ready']);self.assertEqual(n.status['visual_instruction']['direction'],'IMAGE_RIGHT')
        self.assertIsNone(n.status['metric_preview']);self.assertFalse(n.status['actual_output_permitted'])
        self.assertEqual(n.status['output_sent_count'],0)
        Clock.now_value=10.6
        with patch.object(MODULE.time,'monotonic',return_value=20.6):n.process(image(np.full_like(frame,140),10.6),20.6)
        proposal=json.loads(n.proposal_pub.rows[-1].data)
        self.assertFalse(proposal['ready']);self.assertIsNone(proposal['visual_instruction'])
        self.assertEqual(n.source_pub.rows[-1].header.stamp.to_sec(),10.6)
        self.assertEqual(n.image_pub.rows[-1].header.stamp.to_sec(),10.6)

    def test_stale_image_rejected_before_detection(self):
        n=node();Clock.now_value=11.
        with patch.object(n.detector,'detect') as detect, self.assertRaises(ValueError):
            n.process(image(scene()[0],10.),20.)
        detect.assert_not_called()

    def test_processing_delay_cannot_publish_ready(self):
        n=node();Clock.now_value=10.
        wall=[20.]
        original=n.detector.detect
        def slow_detect(frame):
            result=original(frame)
            wall[0]=20.6
            return result
        with patch.object(MODULE.time,'monotonic',side_effect=lambda:wall[0]), patch.object(n.detector,'detect',side_effect=slow_detect):
            n.process(image(scene()[0],10.),20.)
        self.assertFalse(n.status['ready'])
        self.assertEqual(n.status['reason'],'processing_exceeded_freshness_limit')

    def test_header_and_encoding_validation(self):
        n=node();Clock.now_value=10.
        for encoding in ('mono8','32FC1'):
            with patch.object(MODULE.time,'monotonic',return_value=20.),self.assertRaises(ValueError):
                n.process(image(scene()[0],10.,encoding),20.)
        msg=image(scene()[0]);msg.header.frame_id=''
        with patch.object(MODULE.time,'monotonic',return_value=20.),self.assertRaises(ValueError):n.process(msg,20.)

    def test_depth_enabled_but_missing_never_invents_metric_command(self):
        n=node();n.use_depth=True
        for i in range(3):
            Clock.now_value=10.+.2*i
            with patch.object(MODULE.time,'monotonic',return_value=20.+.2*i):n.process(image(scene()[0],Clock.now_value),20.+.2*i)
        self.assertTrue(n.status['ready']);self.assertIsNone(n.status['metric_preview'])
        self.assertEqual(n.status['metric_reason'],'registered_depth_missing')

    def test_registered_depth_produces_preview_but_never_output_authorization(self):
        n=node();n.use_depth=True
        info=dict(stamp=10.,frame_id='rgb_optical',width=640,height=480,
            K=[500.,0.,320.,0.,500.,240.,0.,0.,1.],D=[0.]*5,model='plumb_bob')
        n.info=info;n.depth_info=info
        frame,_=scene(center=(370,240))
        for i in range(3):
            stamp=10.+i*.2;wall=20.+i*.2;Clock.now_value=stamp
            depths=np.full((480,640),2000,np.uint16)
            depths[140:340,230:510]=4000
            n.depths=[(image(depths,stamp,'16UC1'),wall)]
            n.pose.add(stamp,[0,0,-1.6],[0,0,0,1],'map');n.last_pose_wall=wall
            with patch.object(MODULE.time,'monotonic',return_value=wall):n.process(image(frame,stamp),wall)
        proposal=json.loads(n.proposal_pub.rows[-1].data)
        self.assertTrue(proposal['ready'])
        self.assertEqual(proposal['ready_scope'],'visual_tracking_only')
        self.assertFalse(proposal['actual_output_permitted'])
        self.assertIsNotNone(proposal['metric_preview'])
        self.assertAlmostEqual(proposal['metric_preview']['planned_lateral_step_m'],.1)
        self.assertAlmostEqual(proposal['metric_preview']['target_position_enu'][2],-1.6)
        self.assertFalse(proposal['metric_preview']['actual_output_permitted'])

    def feed_rgbd(self, n, count=9):
        from test_rgbd import depth_scene
        from test_guidance import camera
        n.use_depth=True;n.registration_mode='software_tf'
        n.transform_provider=lambda *args:(np.eye(3),np.array([.004,0.,0.]))
        rgb=np.full((480,640,3),140,np.uint8)
        for i in range(count):
            stamp=10.+i*.2;wall=20.+i*.2;Clock.now_value=stamp
            n.info=dict(camera(),stamp=stamp)
            n.depth_info=dict(n.info,frame_id='depth_optical')
            raw=image((depth_scene()*1000).astype(np.uint16),stamp,'16UC1');raw.header.frame_id='depth_optical'
            n.depths=[(raw,wall)];n.depth_info_msg=types.SimpleNamespace(header=raw.header)
            n.pose.add(stamp,[0.,0.,-1.6],[0.,0.,0.,1.],'map');n.last_pose_wall=wall
            with patch.object(MODULE.time,'monotonic',return_value=wall):n.process(image(rgb,stamp),wall)
        return rgb,stamp,wall

    def test_raw_depth_no_rgb_texture_reaches_metric_but_missing_budgets_block_plan(self):
        n=node();self.feed_rgbd(n)
        self.assertTrue(n.status['ready'])
        self.assertTrue(n.status['metric_geometry_ready'],n.status)
        self.assertTrue(n.status['metric_geometry']['opening_evidence']['possible_opening'])
        self.assertIn('missing_safety_bounds',n.status['traversal_reason'])
        self.assertIsNone(n.status['traversal_plan'])
        self.assertEqual(n.status['output_sent_count'],0)
        self.assertEqual(n.depth_source_pub.rows[-1].header.frame_id,'depth_optical')
        self.assertFalse(json.loads(n.geometry_pub.rows[-1].data)['actual_output_permitted'])

    def test_raw_depth_loss_and_wall_watchdog_cancel_every_metric_output(self):
        n=node();rgb,stamp,wall=self.feed_rgbd(n)
        self.assertTrue(n.status['metric_geometry_ready'])
        n.depths=[];stamp+=.2;wall+=.2;Clock.now_value=stamp
        with patch.object(MODULE.time,'monotonic',return_value=wall):n.process(image(rgb,stamp),wall)
        self.assertFalse(n.status['metric_geometry_ready']);self.assertIsNone(n.status['metric_preview'])
        n.cancel_locked('image_stream_stale')
        for publisher,key in ((n.geometry_pub,'geometry'),(n.traversal_pub,'plan')):
            self.assertIsNone(json.loads(publisher.rows[-1].data)[key])
        self.assertFalse(n.status['actual_output_permitted'])

    def test_calibration_change_and_depth_exception_cannot_reuse_old_geometry(self):
        n=node();rgb,stamp,wall=self.feed_rgbd(n)
        info=dict(n.info);info['K']=list(info['K']);info['K'][0]+=.1
        n.check_calibration_epoch(info,n.depth_info,np.eye(3),np.array([.004,0.,0.]))
        self.assertEqual(n.calibration_epoch,1);self.assertEqual(len(n.metric_tracker.rows),0)
        stamp+=.2;wall+=.2;Clock.now_value=stamp
        n.depths[-1][0].header.stamp=Clock(stamp);n.depths=[(n.depths[-1][0],wall)]
        with patch.object(n.depth_detector,'detect',side_effect=ValueError('depth_failed')),patch.object(MODULE.time,'monotonic',return_value=wall):
            n.process(image(rgb,stamp),wall)
        self.assertIsNone(n.status['metric_geometry']);self.assertIsNone(n.status['metric_preview'])
        self.assertEqual(n.status['depth_reason'],'depth_failed')

    def test_painted_rgb_rectangle_on_plane_has_no_step_preview(self):
        from test_guidance import camera
        n=node();n.use_depth=True;n.info=camera();n.depth_info=camera()
        for i in range(7):
            stamp=10.+i*.2;wall=20.+i*.2;Clock.now_value=stamp
            n.depths=[(image(np.full((480,640),2000,np.uint16),stamp,'16UC1'),wall)]
            n.pose.add(stamp,[0,0,0],[0,0,0,1],'map');n.last_pose_wall=wall
            with patch.object(MODULE.time,'monotonic',return_value=wall):n.process(image(scene()[0],stamp),wall)
        self.assertTrue(n.status['ready']);self.assertIsNone(n.status['metric_preview'])
        self.assertIn('not_a_clear_aperture',n.status['metric_reason'])
        self.assertFalse(n.status['metric_geometry']['opening_evidence']['possible_opening'])

    def test_stalled_factory_refresh_expires_without_blocking_image_worker(self):
        n=node();n.astra_factory=True
        n.factory_thread=types.SimpleNamespace(is_alive=lambda:True)
        n.factory_params={'success':True};n.factory_wall=10.
        with patch.object(MODULE.time,'monotonic',return_value=14.),self.assertRaisesRegex(ValueError,'fresh_factory'):
            n.factory_depth_info(None,None,'rgb','depth')

    def test_pose_epoch_reset_during_processing_cancels_metric_outputs(self):
        n=node();rgb,stamp,wall=self.feed_rgbd(n)
        original=MODULE.world_geometry
        def reset_after_geometry(*args):
            geometry=original(*args)
            n.pose.add(args[1]['stamp']+.02,[2,0,-1.6],[0,0,0,1],'map')
            return geometry
        stamp+=.2;wall+=.2;Clock.now_value=stamp
        n.depths[-1][0].header.stamp=Clock(stamp);n.depths=[(n.depths[-1][0],wall)]
        n.pose.add(stamp,[0,0,-1.6],[0,0,0,1],'map');n.last_pose_wall=wall
        with patch.object(MODULE,'world_geometry',side_effect=reset_after_geometry),patch.object(MODULE.time,'monotonic',return_value=wall):
            n.process(image(rgb,stamp),wall)
        self.assertFalse(n.status['ready']);self.assertIsNone(n.status['metric_geometry'])
        self.assertIsNone(n.status['metric_preview']);self.assertIsNone(n.status['traversal_plan'])
        self.assertEqual(n.status['reason'],'pose_reset_during_processing')

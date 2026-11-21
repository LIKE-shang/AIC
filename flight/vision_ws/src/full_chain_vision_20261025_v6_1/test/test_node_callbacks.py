#!/usr/bin/env python3
"""Exercise the real ROS node callbacks using a fake transport; NOT a ROS test."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch
import cv2
import numpy as np

from test_air_target import synthetic, calibrated_params

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))

class Stamp:
    def __init__(self, value=0): self.value=value
    def to_sec(self): return self.value
class Header:
    def __init__(self, value=0):
        self.stamp=Stamp(value); self.seq=42; self.frame_id='camera_optical'
class Image:
    def __init__(self, data=None, encoding='bgr8', stamp=0):
        self.data=data; self.encoding=encoding; self.header=Header(stamp)
        self.height,self.width=data.shape[:2] if data is not None else (0,0)
class String:
    def __init__(self, data=''): self.data=data
class LaserDecision:
    def __init__(self):
        self.header=Header(); self.enable_laser=False; self.request_on=False
        self.observation_valid=False; self.hit_count=0; self.miss_count=0; self.reason=''
class Publisher:
    def __init__(self, name, typ, **kwargs): self.resolved_name=name; self.messages=[]
    def publish(self, msg): self.messages.append(copy.deepcopy(msg))
class CvBridge:
    def imgmsg_to_cv2(self,msg,desired_encoding=None):
        if msg.data is None: raise ValueError('fake conversion error')
        return msg.data.copy()
    def cv2_to_imgmsg(self,data,encoding='bgr8'): return Image(data.copy(),encoding)

rospy=types.ModuleType('rospy')
params={}
rospy.get_param=lambda name,default=None: params if name=='~' else params.get(name[1:] if name.startswith('~') else name,default)
rospy.Publisher=Publisher
rospy.Subscriber=lambda *a,**kw: None
rospy.resolve_name=lambda s:s
rospy.on_shutdown=lambda f:None
rospy.Time=types.SimpleNamespace(now=lambda:Stamp(time.time()))
for name in ['loginfo','logwarn','logwarn_throttle','logerr','logerr_throttle','logfatal','signal_shutdown']:
    setattr(rospy,name,lambda *a,**kw:None)
mods={'rospy':rospy, 'cv_bridge':types.SimpleNamespace(CvBridge=CvBridge),
      'sensor_msgs':types.ModuleType('sensor_msgs'),'sensor_msgs.msg':types.SimpleNamespace(Image=Image),
      'std_msgs':types.ModuleType('std_msgs'),'std_msgs.msg':types.SimpleNamespace(String=String),
      'robot_vision':types.ModuleType('robot_vision'),'robot_vision.msg':types.SimpleNamespace(LaserDecision=LaserDecision)}
sys.modules.update(mods)
def load(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'scripts'/f'{name}.py')
    mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod); return mod
circle=load('circle_laser_test_node')
window=load('window_test_node')

def scene(target=True,double=False):
    if double:
        return np.minimum(synthetic((175,240)),synthetic((465,240)))
    return synthetic() if target else np.full((480,640,3),235,np.uint8)

class CallbackTests(unittest.TestCase):
    def setUp(self):
        params.clear(); params.update(calibrated_params()); params.update(on_frames=3,off_frames=1,image_timeout=0.25,
                                     max_image_age=0.5,require_ring=True)
        self.node=None
    def tearDown(self):
        if self.node:
            self.node.close(); self.node.watchdog.join(1)
    def make_node(self,real=False):
        params['enable_laser']=real
        self.node=circle.CircleLaserTest()
        if real:
            self.node.status_callback(String(json.dumps(dict(state='OFF_ACKNOWLEDGED',startup_off_acked=True,
                                      fault_latched=False,fcu_connected=True,fcu_armed=False))))
        return self.node
    def frame(self,node,target=True,double=False):
        node.fake_capture=max(time.time(),getattr(node,'fake_capture',0)+.01)
        msg=Image(scene(target,double),stamp=node.fake_capture); node.callback(msg); return msg
    def test_startup_dry_run_n_and_immediate_off(self):
        n=self.make_node()
        self.assertFalse(n.decision_pub.messages[0].request_on)
        self.assertEqual(json.loads(n.dry_status_pub.messages[-1].data)['state'],'DRY_RUN_OFF')
        for _ in range(2):
            self.frame(n); self.assertFalse(n.decision_pub.messages[-1].request_on)
        self.frame(n); self.assertTrue(n.decision_pub.messages[-1].request_on)
        self.assertFalse(n.decision_pub.messages[-1].enable_laser)
        self.frame(n,False); self.assertFalse(n.decision_pub.messages[-1].request_on)
        self.frame(n,False); self.assertFalse(n.decision_pub.messages[-1].request_on)
    def test_headers_preserved(self):
        n=self.make_node(); msg=self.frame(n)
        for pub in [n.image_pub,n.edges_pub,n.decision_pub]:
            h=pub.messages[-1].header
            self.assertEqual(h.stamp.value,msg.header.stamp.value)
            self.assertEqual(h.frame_id,msg.header.frame_id)
            self.assertEqual(h.seq,msg.header.seq)
    def test_conversion_error_off(self):
        n=self.make_node()
        for _ in range(3): self.frame(n)
        n.callback(Image(None,stamp=time.time()))
        self.assertFalse(n.decision_pub.messages[-1].request_on)
        self.assertFalse(n.decision_pub.messages[-1].observation_valid)
    def test_duplicate_stamp_off(self):
        n=self.make_node(); msg=self.frame(n); n.callback(msg)
        self.assertFalse(n.decision_pub.messages[-1].observation_valid)
    def test_multiple_targets_invalid(self):
        n=self.make_node(); self.frame(n,double=True)
        self.assertFalse(n.decision_pub.messages[-1].observation_valid)
        self.assertIn('multiple',n.decision_pub.messages[-1].reason)
    def test_watchdog_no_callbacks(self):
        n=self.make_node()
        for _ in range(3): self.frame(n)
        deadline=time.monotonic()+1
        while time.monotonic()<deadline and n.gate.on: time.sleep(.02)
        self.assertFalse(n.gate.on)
        self.assertEqual(n.decision_pub.messages[-1].reason,'image_timeout')
    def test_detector_shutdown_off(self):
        n=self.make_node()
        for _ in range(3): self.frame(n)
        n.close()
        self.assertFalse(n.decision_pub.messages[-1].request_on)
        self.assertEqual(n.decision_pub.messages[-1].reason,'detector_shutdown')
    def test_real_detector_only_publishes_intent(self):
        n=self.make_node(True)
        for _ in range(3): self.frame(n)
        self.assertTrue(n.decision_pub.messages[-1].enable_laser)
        self.assertTrue(n.decision_pub.messages[-1].request_on)
        self.assertIsNone(n.dry_status_pub)
        self.assertFalse(hasattr(n,'service_client'))
    def test_real_missing_calibration_and_simulated_time_refused(self):
        params['enable_laser']=True; params['calibration']['confirmed']=False
        with self.assertRaises(ValueError): circle.CircleLaserTest()
        params['calibration']['confirmed']=True; params['/use_sim_time']=True
        with self.assertRaises(ValueError): circle.CircleLaserTest()
    def test_real_stale_controller_revokes_on(self):
        n=self.make_node(True)
        for _ in range(3): self.frame(n)
        n.laser_status_wall=time.monotonic()-1
        self.frame(n)
        self.assertFalse(n.gate.on); self.assertTrue(n.gate.latched)
    def test_target_switch_resets_and_off(self):
        n=self.make_node()
        for _ in range(3): self.frame(n)
        n.callback(Image(synthetic(center=(355,240)),stamp=time.time()+.04))
        self.assertFalse(n.gate.on); self.assertTrue(n.gate.latched)
        self.assertEqual(n.decision_pub.messages[-1].reason,'target_changed')
    def test_stale_future_zero_frame_immediately_off(self):
        for delta in [-1,1]:
            n=self.make_node()
            for _ in range(3): self.frame(n)
            n.callback(Image(scene(),stamp=time.time()+delta))
            self.assertFalse(n.gate.on); self.assertFalse(n.decision_pub.messages[-1].observation_valid)
            n.close(); n.watchdog.join(1)
    def test_camera_background_spot_veto_before_on(self):
        params['feedback_mode']='camera'; params['spot'].update(color='red',validated=True)
        n=self.make_node()
        for i in range(3): n.callback(Image(synthetic(spot=(320,240)),stamp=time.time()+i*.01))
        self.assertFalse(n.gate.on); self.assertIn('background',n.gate.reason)
    def test_camera_out_of_bounds_spot_and_loss_immediate_off(self):
        params['feedback_mode']='camera'; params['spot'].update(color='red',validated=True)
        n=self.make_node()
        for _ in range(3): self.frame(n)
        n.callback(Image(synthetic(spot=(320,240)),stamp=time.time()+.04))
        self.assertTrue(n.gate.on)
        n.callback(Image(synthetic(spot=(365,240)),stamp=time.time()+.05))
        self.assertFalse(n.gate.on); self.assertEqual(n.gate.elapsed,0)
    def test_warning_header_not_fabricated_and_recovery_not_starved(self):
        n=self.make_node(); self.frame(n,False)
        time.sleep(.3)
        self.assertEqual(n.image_pub.messages[-1].header.stamp.value,n.fake_capture)
        original=circle.detect_air_target
        def slower(*a):
            time.sleep(.07); return original(*a)
        with patch.object(circle,'detect_air_target',slower): self.frame(n)
        self.assertEqual(n.gate.reason,'capture_or_processing_gap')
        with patch.object(circle,'detect_air_target',slower): self.frame(n)
        self.assertEqual(n.gate.hits,1) # subsequent fresh recovery must not be starved
    def test_overlay_failure_after_confirmation_sends_off(self):
        n=self.make_node()
        for _ in range(2): self.frame(n)
        with patch.object(n,'_overlay',side_effect=ValueError('injected render failure')): self.frame(n)
        self.assertFalse(n.decision_pub.messages[-1].request_on)
        self.assertEqual(n.gate.elapsed,0)
    def test_late_callback_after_shutdown_no_on(self):
        n=self.make_node(); n.close(); count=len(n.decision_pub.messages)
        self.frame(n); self.assertEqual(len(n.decision_pub.messages),count)
    def test_full_camera_callback_hold_ack_not_physical_acceptance(self):
        params['feedback_mode']='camera'; params['spot'].update(color='red',validated=True)
        n=self.make_node(True); n.stop.set(); n.watchdog.join(1)
        n.laser_status_wall=10.0
        for i in range(36):
            wall,stamp=10+i*.1,100+i*.1
            with patch.object(circle.time,'monotonic',return_value=wall), patch.object(circle.rospy.Time,'now',return_value=Stamp(stamp)):
                if i>=3:
                    n.status_callback(String(json.dumps(dict(state='ON_ACKNOWLEDGED',startup_off_acked=True,
                                              fault_latched=False,fcu_connected=True,fcu_armed=False))))
                image=synthetic(spot=(320,240) if i>=3 else None)
                image[0,0]=[i,20,30] # varying payload, not a frozen synthetic frame
                n.callback(Image(image,stamp=stamp))
        status=json.loads(n.status_pub.messages[-1].data)
        self.assertEqual(status['result'],'camera_observed_3s')
        self.assertFalse(n.gate.on); self.assertFalse(status['physical_hit_verified'])
        self.assertGreaterEqual(status['completed_seconds'],3.0)
        self.assertGreater(status['completed_capture_interval'][0],n.ack_ros)
    def test_rendering_past_capture_expiry_cannot_publish_on(self):
        n=self.make_node()
        for _ in range(2): self.frame(n)
        stamp=max(time.time(),n.fake_capture+.01)
        actual=n._overlay
        def age_during_output(*args):
            actual(*args)
            ros_clock.return_value=Stamp(stamp+n.freshness.max_age+.01)
        with patch.object(circle.rospy.Time,'now',return_value=Stamp(stamp)) as ros_clock, \
             patch.object(n,'_overlay',age_during_output):
            n.callback(Image(scene(),stamp=stamp))
        self.assertFalse(n.decision_pub.messages[-1].request_on)
        self.assertIn('expired',n.decision_pub.messages[-1].reason)
    def test_window_callback_and_header(self):
        n=window.WindowTest()
        data=scene(False); cv2.rectangle(data,(100,100),(530,380),(20,20,20),20)
        msg=Image(data,stamp=time.time()); n.image(msg)
        self.assertEqual(n.image_pub.messages[-1].header.stamp.value,msg.header.stamp.value)
        self.assertEqual(json.loads(n.status_pub.messages[-1].data)['state'],'found')
    def test_window_checker_callback_and_diagnostics(self):
        n=window.WindowTest()
        data=cv2.imread(str(ROOT/'test/artifacts/window_checker.png'))
        self.assertIsNotNone(data)
        msg=Image(data,stamp=time.time()); n.image(msg)
        status=json.loads(n.status_pub.messages[-1].data)
        self.assertEqual(status['state'],'found')
        self.assertEqual(status['count'],1)
        self.assertTrue(status['no_control'])
        self.assertTrue(status['rgb_only'])
        self.assertEqual(status['image_width'],640)
        self.assertEqual(status['image_height'],480)
        self.assertEqual(status['capture_stamp'],msg.header.stamp.value)
        self.assertEqual(status['frame_seq'],msg.header.seq)
        self.assertEqual(status['camera_frame_id'],msg.header.frame_id)
        self.assertEqual(status['targets'][0]['outline_kind'],'opening')
        self.assertGreaterEqual(status['processing_ms'],0)
        for pub in [n.image_pub,n.edge_pub]:
            self.assertEqual(pub.messages[-1].header.stamp.value,msg.header.stamp.value)
            self.assertEqual(pub.messages[-1].header.seq,msg.header.seq)
            self.assertEqual(pub.messages[-1].header.frame_id,msg.header.frame_id)
    def test_window_rate_limit_drops_without_sleep(self):
        params['max_processing_fps']=10.0
        n=window.WindowTest()
        msg=Image(scene(False),stamp=time.time())
        with patch.object(window.time,'monotonic',return_value=100.0):
            n.image(msg)
        with patch.object(window.time,'monotonic',return_value=100.01):
            with patch.object(window,'detect_windows') as detect:
                n.image(msg)
                detect.assert_not_called()
        self.assertEqual(len(n.image_pub.messages),1)
        with patch.object(window.time,'monotonic',return_value=100.11):
            n.image(msg)
        self.assertEqual(len(n.image_pub.messages),2)
    def test_window_invalid_rate_fails_at_startup(self):
        for value in [-1.0,float('nan'),float('inf')]:
            params['max_processing_fps']=value
            with self.assertRaises(ValueError): window.WindowTest()
    def test_window_raw_depth_rejected(self):
        n=window.WindowTest(); n.image(Image(np.zeros((480,640),np.uint16),'16UC1',time.time()))
        self.assertEqual(json.loads(n.status_pub.messages[-1].data)['state'],'image_error')
        self.assertEqual(len(n.image_pub.messages),0)
    def test_inflight_frame_after_watchdog_cannot_enable(self):
        n=self.make_node()
        for _ in range(3): self.frame(n)
        detect=circle.detect_air_target
        def slow(*args):
            time.sleep(.35); return detect(*args)
        with patch.object(circle,'detect_air_target',slow): self.frame(n)
        self.assertFalse(n.gate.on)
        self.assertFalse(n.decision_pub.messages[-1].request_on)

if __name__=='__main__': unittest.main(verbosity=2)

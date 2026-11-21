#!/usr/bin/env python3
"""Actual observer wrapper, simulated messages/OpenCV. NOT a detector regression.
No cv2, NumPy, ROS runtime, master or hardware libraries are executed here.
"""
import copy
import hashlib
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
import window_pass
N=types.SimpleNamespace


class Stamp:
    def __init__(self,v):self.v=v
    def to_sec(self):return self.v


class Point:
    def __init__(self):self.x=self.y=self.z=0.


class ObservationMessage:
    def __init__(self):
        self.valid=self.calibration_valid=False
        self.target_count=0
        self.center_px=Point()
        self.reason=self.detection_method=self.geometry_kind=self.camera_info_sha256=''
    def validate_stub_schema(self):
        assert len(self.opening_corners_px)==4 and len(self.optical_rays)==5
        for p in [self.center_px]+self.opening_corners_px+self.optical_rays:
            assert all(type(x) is float for x in (p.x,p.y,p.z))


class ObserverTransportTests(unittest.TestCase):
    def setUp(self):
        self.wall,self.ros=10.,1010.
        self.pubs=[];self.remaps={};self.bad_info=False
        self.found=[dict(method='checker_texture')]
        self.frame=N(shape=(480,640,3))
        def publisher(topic,typ,**kw):
            p=N(topic=topic,typ=typ,options=kw,messages=[])
            p.publish=lambda msg:p.messages.append(copy.deepcopy(msg))
            self.pubs.append(p);return p
        self.rospy=N(get_param=lambda n,d=None:d,resolve_name=lambda n:self.remaps.get(n,n),
            Publisher=publisher,Subscriber=lambda *a,**kw:N(),loginfo=lambda *a:None,
            logwarn_throttle=lambda *a:None,Time=N(now=lambda:Stamp(self.ros)))
        cv=N(polylines=lambda *a:None,drawMarker=lambda *a:None,putText=lambda *a:None,
            MARKER_CROSS=0,FONT_HERSHEY_SIMPLEX=0)
        np=N(asarray=lambda value,**kw:value,int32=int)
        bridge=N(imgmsg_to_cv2=lambda msg,**kw:self.frame,
                 cv2_to_imgmsg=lambda frame,encoding:N(encoding=encoding))
        fakevision=N(__file__=str(ROOT/'src/window_pass/vision_frozen.py'),WINDOW_DEFAULTS={},
            checked_parameters=lambda defaults,supplied:{},detect_windows=lambda frame,params:(self.found,None),
            annotate_windows=lambda frame,found:frame)
        def camera(*args):
            if self.bad_info:raise ValueError('missing_camera_info')
            return [],[],'a'*64
        fakegeometry=N(inner_opening=lambda *a:((320.,240.),((100.,90.),(540.,90.),(540.,390.),(100.,390.)),'approximate_inner_opening'),
            camera_rays=camera,undistorted_rays=lambda *a:[(0.,0.,1.)]*5,digest=lambda value:'b'*64)
        modules={'rospy':self.rospy,'cv2':cv,'numpy':np,'cv_bridge':N(CvBridge=lambda:bridge),
            'sensor_msgs.msg':N(Image=N,CameraInfo=N),'geometry_msgs.msg':N(Point32=Point,Vector3=Point),
            'window_pass_test.msg':N(WindowObservation=ObservationMessage),
            'window_pass.vision_frozen':fakevision,'window_pass.observation':fakegeometry}
        self.mods=patch.dict(sys.modules,modules);self.mods.start()
        self.attr=patch.object(window_pass,'vision_frozen',fakevision,create=True);self.attr.start()
        spec=importlib.util.spec_from_file_location('window_observer_stub_test',ROOT/'scripts/window_observer.py')
        self.mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.mod)
        self.clock=patch.object(self.mod.time,'monotonic',lambda:self.wall);self.clock.start()
        self.node=self.mod.Observer()
    def tearDown(self):
        self.clock.stop();self.attr.stop();self.mods.stop()
    def image(self,**kw):
        msg=N(header=N(stamp=Stamp(self.ros),frame_id='optical',seq=7),width=640,height=480,encoding='rgb8')
        for k,v in kw.items():setattr(msg,k,v)
        self.node.image_cb(msg)
        out=self.node.pub.messages[-1]
        out.validate_stub_schema()
        return out
    def test_structured_message_arrays_and_capture_time(self):
        o=self.image()
        self.assertTrue(o.valid and o.calibration_valid)
        self.assertEqual(o.header.stamp.to_sec(),1010.)
        self.assertEqual((o.center_px.x,o.center_px.y),(320.,240.))
        self.assertEqual(o.geometry_kind,'approximate_inner_opening')
        self.assertEqual(o.algorithm_sha256,hashlib.sha256((ROOT/'src/window_pass/vision_frozen.py').read_bytes()).hexdigest())
    def test_missing_calibration_never_fabricates_range(self):
        self.bad_info=True;o=self.image()
        self.assertTrue(o.valid);self.assertFalse(o.calibration_valid)
        self.assertTrue(all(p.z==0. for p in o.optical_rays))
    def test_multiple_targets_have_no_geometric_reference(self):
        self.found*=2;o=self.image()
        self.assertEqual(o.target_count,2);self.assertFalse(o.valid)
        self.assertEqual(o.reason,'multiple_targets')
    def test_no_target_fixed_arrays_still_serializable_in_stub(self):
        self.found=[]
        with patch.object(self.mod, 'camera_rays', side_effect=AssertionError('must not check calibration without a target')) as camera:
            o=self.image()
        camera.assert_not_called()
        self.assertFalse(o.valid);self.assertEqual(o.reason,'no_target')
        self.assertFalse(o.calibration_valid)
        self.assertEqual(o.camera_info_sha256, '')
        self.assertEqual(o.target_count, 0)
        self.assertEqual(len(self.node.image_pub.messages), 1)
    def test_replay_is_rejected_not_retimestamped(self):
        self.image();self.wall+=.2;o=self.image()
        self.assertFalse(o.valid);self.assertIn('duplicate',o.reason)
    def test_future_and_old_captures_are_rejected(self):
        for stamp in (0.,1000.,1020.):
            self.wall+=.2
            o=self.image(header=N(stamp=Stamp(stamp),frame_id='optical',seq=1))
            self.assertFalse(o.valid)
    def test_wrong_resolution_encoding_and_frame(self):
        for values in (dict(width=320),dict(encoding='mono8'),dict(header=N(stamp=Stamp(1010.),frame_id='',seq=1))):
            self.wall+=.2;self.node.last_stamp=0.
            self.assertFalse(self.image(**values).valid)
    def test_only_diagnostic_publishers_no_motion_interfaces(self):
        self.assertEqual({p.topic for p in self.pubs},{'/window_pass_test/observation','/window_pass_test/image'})
        self.remaps['/window_pass_test/observation']='/object_position'
        with self.assertRaises(ValueError):self.mod.Observer()


if __name__=='__main__':unittest.main(verbosity=2)

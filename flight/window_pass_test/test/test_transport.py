#!/usr/bin/env python3
"""Real wrapper with Fake ROS only. No ROS master/FCU connections."""
import copy
import importlib.util
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'test'))
from fixtures import Feed,calibration
from window_pass.graph import OUTPUT,POSE,STATE,EXTENDED,OBSERVATION,IMAGE,INFO
N=types.SimpleNamespace
class Stamp:
    def __init__(self,v=0.):self.v=v
    def to_sec(self):return self.v
class Message:
    FRAME_LOCAL_NED=1
    IGNORE_VX,IGNORE_VY,IGNORE_VZ,IGNORE_AFX,IGNORE_AFY,IGNORE_AFZ,IGNORE_YAW_RATE=8,16,32,64,128,256,2048
    LANDED_STATE_IN_AIR=2
    def __init__(self,*args,**kw):
        self.header=N(stamp=Stamp(),frame_id='',seq=0)
        self.position=N(x=0.,y=0.,z=0.);self.velocity=N(x=0.,y=0.,z=0.)
        self.twist=N(linear=N(x=0.,y=0.,z=0.),angular=N(x=0.,y=0.,z=0.))
        self.data=kw.get('data','')
        if args:
            self.success,self.message=args
class Pub:
    def __init__(self,name,typ,**kw):self.resolved_name=name;self.typ=typ;self.options=kw;self.messages=[];self.registered=True;self.connections=1
    def publish(self,m):self.messages.append(copy.deepcopy(m))
    def unregister(self):self.registered=False
    def get_num_connections(self):return self.connections


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.cfg={};self.pubs=[];self.services={};self.remaps={};self.wall=10.;self.ros=1010.;self.node=None;self.rivals=[]
        def publisher(*a,**kw):
            p=Pub(*a,**kw);self.pubs.append(p);return p
        self.rospy=N(get_param=lambda n,d=None:self.cfg if n=='~control' else (False if n.endswith('/tf/listen') else d),
            resolve_name=lambda n:self.remaps.get(n,n),get_name=lambda:'/window_pass_controller',
            Publisher=publisher,Subscriber=lambda *a,**kw:N(args=a),
            Service=lambda name,typ,cb:self.services.setdefault(name,cb),on_shutdown=lambda f:None,
            Time=N(now=lambda:Stamp(self.ros),from_sec=Stamp),
            logwarn=lambda *a:None,loginfo=lambda *a:None,logerr=lambda *a:None,logfatal=lambda *a:None)
        modules={'rospy':self.rospy,'rosgraph':N(Master=lambda n:N(getSystemState=self.graph)),
                 'geometry_msgs.msg':N(TwistStamped=Message),'mavros_msgs.msg':N(PositionTarget=Message,State=Message,ExtendedState=Message),
                 'nav_msgs.msg':N(Odometry=Message),'sensor_msgs.msg':N(Image=Message),
                 'std_msgs.msg':N(String=Message),'std_srvs.srv':N(Trigger=Message,TriggerResponse=Message),
                 'window_pass_test.msg':N(WindowObservation=Message)}
        self.modules=patch.dict(sys.modules,modules);self.modules.start()
        spec=importlib.util.spec_from_file_location('window_node_under_test',ROOT/'scripts/window_pass_node.py')
        self.mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.mod)
        self.clock=patch.object(self.mod.time,'monotonic',lambda:self.wall);self.clock.start()
    def tearDown(self):
        if self.node:self.node.shutdown()
        self.clock.stop();self.modules.stop()
    def graph(self):
        pubs=[(t,['/mavros']) for t in (POSE,STATE,EXTENDED)]
        pubs += [(OBSERVATION,['/window_pass_observer']),(IMAGE,['/camera']),(INFO,['/camera'])]
        pubs += [(p.resolved_name,['/window_pass_controller']) for p in self.pubs if p.registered]+self.rivals
        return pubs,[(OUTPUT,['/mavros'])],[]
    def make(self,motion=False):
        if motion:self.cfg=calibration(True)
        self.node=self.mod.Node(start_threads=False)
        return self.node
    def feed(self,motion=True,ready=False):
        f=Feed(motion)
        if ready:f.ready()
        else:f.prime()
        self.node.core=f.ctrl;self.wall,self.ros=f.wall,f.ros
        return f
    def test_shadow_constructor_has_no_real_publisher_or_hardware_services(self):
        n=self.make()
        for _ in range(5):n.control_once();n.watchdog_once()
        for action in ('authorize_hold','start_alignment','commit_pass','abort','release'):n.service(action)
        self.assertTrue(all(p.resolved_name.startswith('/window_pass_test/') for p in self.pubs))
        self.assertTrue(all(s.startswith('/window_pass_test/') for s in self.services))
        self.assertIsNone(n.motion_pub)
    def test_shadow_with_mavros_trajectory_visualization_never_creates_motion_output(self):
        n = self.make()
        self.rivals = [('/mavros/setpoint_trajectory/desired', ['/mavros']),
                       ('/mavros/trajectory/desired', ['/mavros'])]
        self.assertEqual(n.graph_once(), (True, 'graph_exclusive_snapshot'))
        for _ in range(5):
            n.control_once(); n.watchdog_once()
        self.assertFalse(n.service('authorize_hold').success)
        self.assertIsNone(n.motion_pub)
        self.assertEqual(n.core.output_count, 0)
        self.assertTrue(all(p.resolved_name.startswith('/window_pass_test/') for p in self.pubs))

    def test_feedback_exception_keeps_real_trajectory_controller_blocked(self):
        n = self.make(True); self.feed()
        self.rivals = [('/mavros/setpoint_trajectory/desired', ['/mavros']),
                       ('/mavros/setpoint_trajectory/local', ['/other_controller'])]
        self.assertFalse(n.graph_once()[0])
        self.assertFalse(n.service('authorize_hold').success)
        self.assertIsNone(n.motion_pub)

    def test_incomplete_motion_config_fails_before_any_publisher(self):
        self.cfg={'enable_motion':True}
        with self.assertRaises(ValueError):self.mod.Node(start_threads=False)
        self.assertEqual(self.pubs,[])
    def test_remaps_to_motion_or_legacy_are_refused(self):
        for name in (OUTPUT,'/window_pass_test/status'):
            self.remaps={name:'/cmd_vel'}
            with self.assertRaises(ValueError):self.mod.Node(start_threads=False)
        self.assertEqual(self.pubs,[])
    def test_explicit_hold_then_correct_position_mask_only(self):
        n=self.make(True);self.feed();n.graph_once()
        self.assertIsNone(n.motion_pub)
        answer=n.service('authorize_hold');self.assertTrue(answer.success,answer.message)
        n.control_once()
        self.assertEqual(len(n.motion_pub.messages),1)
        msg=n.motion_pub.messages[-1]
        self.assertEqual(msg.type_mask,2552);self.assertEqual(msg.coordinate_frame,1)
        self.assertEqual((msg.position.x,msg.position.y,msg.position.z),(0.,0.,1.2))
        self.assertFalse(n.core.started)
    def test_bad_graph_does_not_advertise_motion(self):
        n=self.make(True);self.feed();self.rivals=[('/mavros/setpoint_position/local',['/other'])];n.graph_once()
        self.assertFalse(n.service('authorize_hold').success)
        self.assertIsNone(n.motion_pub)
    def test_transport_disconnect_revokes(self):
        n=self.make(True);self.feed();n.graph_once();self.assertTrue(n.service('authorize_hold').success)
        n.motion_pub.connections=0;n.control_once()
        self.assertEqual(n.core.state,'RELEASED');self.assertEqual(n.motion_pub.messages,[])
    def test_actual_callbacks_preserve_original_capture_time(self):
        n=self.make();msg=Message();msg.header.stamp=Stamp(1009.9);msg.width=640;msg.height=480;msg.encoding='rgb8'
        n.image_cb(msg)
        self.assertEqual(n.core.streams['image'][0],1009.9)
        n.image_cb(msg)
        self.assertFalse(n.core.streams['image'][2])
    def test_trajectory_control_does_not_create_publisher_on_launch(self):
        n=self.make(True);self.feed(ready=True)
        n.control_once()
        self.assertIsNone(n.motion_pub)
        self.assertNotIn(OUTPUT,[p.resolved_name for p in self.pubs])
    def test_background_watchdog_fires_without_callbacks_or_ros_clock(self):
        n=self.make(True);f=self.feed(ready=True)
        self.clock.stop()
        offset=time.monotonic()-f.wall
        n.core.last_tick+=offset
        n.core.graph_wall+=offset
        n.core.streams={k:(s,w+offset,ok) for k,(s,w,ok) in n.core.streams.items()}
        n.core.begin_wall+=offset
        t=threading.Thread(target=n.watchdog_loop);t.start()
        try:
            time.sleep(.23)
            self.assertEqual(n.core.state,'FAULT')
        finally:n.stop_event.set();t.join(1.)
        self.clock=patch.object(self.mod.time,'monotonic',lambda:self.wall);self.clock.start()
    def test_abort_holds_only_when_previously_authorized(self):
        n=self.make(True);self.feed();n.graph_once();n.service('authorize_hold')
        self.assertTrue(n.service('abort').success);n.watchdog_once()
        self.assertEqual(n.core.state,'ABORT')
        self.assertTrue(n.motion_pub.messages)


if __name__=='__main__':unittest.main(verbosity=2)

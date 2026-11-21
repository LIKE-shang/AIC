"""Faults stop at pose forwarding before FCU conversion/publication."""
import threading,unittest
from types import SimpleNamespace as NS
from test_mission import Sim
from hover_pose_guard import guarded_bridge

class Parent:
    def __init__(self):
        self.lock=threading.RLock();self.now=100.;self.forwarded=[];self.real_fault=''
        self.ros=NS(Time=NS(now=lambda:NS(to_sec=lambda:self.now)))
        self.gm=NS(SOURCE_NODE='/lio')
    def on_odometry(self,msg):
        if not self.real_fault:self.forwarded.append(msg)
    def _disable_real(self,reason):self.real_fault=reason

class PoseGuardTests(unittest.TestCase):
    def setUp(self):
        self.bridge=guarded_bridge(NS(ContinuousPoseBridge=Parent),Sim().m.core.c)()
    def msg(self,p=(0.,0.,0.),seq=1):
        return NS(header=NS(seq=seq,stamp=NS(to_sec=lambda:self.bridge.now),frame_id='camera_init'),
            child_frame_id='body',pose=NS(pose=NS(position=NS(x=p[0],y=p[1],z=p[2]),
            orientation=NS(x=0.,y=0.,z=0.,w=1.))),_connection_header={'callerid':'/lio'})
    def test_good_data_reaches_existing_conversion(self):
        self.bridge.on_odometry(self.msg());self.assertEqual(len(self.bridge.forwarded),1)
    def test_huge_initial_origin_never_reaches_fcu(self):
        self.bridge.on_odometry(self.msg((1306.,-1083.,-7661.)))
        self.assertEqual(len(self.bridge.forwarded),0);self.assertTrue(self.bridge.real_fault)
    def test_jump_is_latched_before_forwarding_and_does_not_auto_resume(self):
        self.bridge.on_odometry(self.msg());self.bridge.now+=.02
        self.bridge.on_odometry(self.msg((.5,0.,0.),2));self.bridge.now+=.02
        self.bridge.on_odometry(self.msg((.001,0.,0.),3))
        self.assertEqual(len(self.bridge.forwarded),1);self.assertTrue(self.bridge.real_fault)
    def test_source_change_cannot_reach_conversion(self):
        m=self.msg();m._connection_header={'callerid':'/other'}
        self.bridge.on_odometry(m);self.assertEqual(len(self.bridge.forwarded),0)

if __name__=='__main__':unittest.main()

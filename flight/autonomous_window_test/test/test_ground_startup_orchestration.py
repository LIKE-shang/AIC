"""Exercise the real ground-recovery main() using fake ROS and spawn callbacks.

No ROS installation, network, subprocess, mode service or vehicle is used.
"""
import contextlib
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
import full_chain_ground as app

LIO='/window_pass_lio/laserMapping'
STATUS='/window_pass_competition/status'
VISION='/mavros/vision_pose/pose'
BOOT='offline-startup-test-boot'


class StopAfterPoseDecision(Exception):
    """Stop before unrelated target/laser/RGB-D startup stages."""


def sample(mode,stamp=99.9):
    def base(**values):
        return NS(header=NS(stamp=NS(to_sec=lambda:stamp)),
                  _connection_header={'callerid':'/mavros'},**values)
    return (base(connected=True,armed=False,mode=mode,system_status=3),
            base(landed_state=1),base(channels=[1050]*8))


def status_message(caller='/existing_bridge',**changes):
    data={'raw_lio_motion_limits':{'max_pose_step_m':.35,'max_pose_speed_mps':2.2},
          'raw_lio_guard_scope':'RouteContinuity'}
    data.update(changes)
    return NS(data=json.dumps(data),_connection_header={'callerid':caller})


class GroundScenario:
    def __init__(self,early='MANUAL',fresh='ALTCTL',after_lio=None,after_sample=None):
        self.early=early;self.fresh=fresh
        self.after_lio=after_lio;self.after_sample=after_sample
        self.pubs={'/mavros/state':['/mavros'],'/livox/lidar':['/livox']}
        self.samples=0;self.spawned=[];self.events=[];self.output=''
        self.stop_at_sample=3;self.status=status_message()
        self.handles=[];self.status_reads=0

    def getSystemState(self):
        self.events.append('graph')
        return (list(copy.deepcopy(self.pubs).items()),[],[])

    def latest(self,ros,topics,timeout,max_age,reject):
        self.samples+=1;self.events.append('sample%d'%self.samples)
        if timeout!=4 or max_age!=2:
            raise AssertionError('original ground sampling bounds changed')
        if self.samples>=self.stop_at_sample:
            raise StopAfterPoseDecision()
        mode=self.early if self.samples==1 else self.fresh
        values=sample(mode)
        for (topic,_),message in zip(topics,values):
            reason=reject(topic,message)
            if reason:raise ValueError(reason)
        if self.after_sample:self.after_sample(self,self.samples)
        return values

    def spawn(self,name,args):
        self.events.append('spawn:'+name);self.spawned.append((name,list(args)))
        if name=='lio':
            self.pubs['/window_pass_lio/Odometry']=[LIO]
            if self.after_lio:self.after_lio(self)
        elif name=='guarded_pose':
            self.pubs[STATUS]=['/new_bridge'];self.pubs[VISION]=['/new_bridge']
        else:
            raise AssertionError('unrelated or real startup attempted: '+name)
        return NS(pid=123)

    def wait_for_message(self,topic,kind,timeout):
        if topic!=STATUS or timeout!=3:
            raise AssertionError('unexpected message wait: '+topic)
        self.status_reads+=1
        return self.status

    def run(self):
        ros=ModuleType('rospy')
        ros.init_node=lambda *args,**kwargs:None
        ros.get_name=lambda:'/offline_ground_test'
        ros.Time=NS(now=lambda:NS(to_sec=lambda:100.))
        ros.wait_for_message=self.wait_for_message
        graph=ModuleType('rosgraph')
        graph.is_master_online=lambda:True
        graph.Master=lambda caller:self
        nodes=ModuleType('rosnode')
        nodes.get_node_names=lambda:sorted({n for names in self.pubs.values() for n in names})
        nodes.kill_nodes=lambda *args:(_ for _ in ()).throw(AssertionError('node stop not expected'))
        mavros=ModuleType('mavros_msgs.msg')
        mavros.State=type('State',(),{});mavros.ExtendedState=type('ExtendedState',(),{})
        mavros.RCIn=type('RCIn',(),{})
        std=ModuleType('std_msgs.msg');std.String=type('String',(),{})
        fcntl=ModuleType('fcntl');fcntl.LOCK_EX=1;fcntl.LOCK_NB=2
        fcntl.flock=lambda *args:None
        modules={'rospy':ros,'rosgraph':graph,'rosnode':nodes,'mavros_msgs.msg':mavros,
                 'std_msgs.msg':std,'fcntl':fcntl}
        original_read=Path.read_text;original_open=Path.open
        def read_text(path,*args,**kwargs):
            if path==Path('/proc/sys/kernel/random/boot_id'):return BOOT
            return original_read(path,*args,**kwargs)
        def tracked_open(path,*args,**kwargs):
            handle=original_open(path,*args,**kwargs)
            if path.name=='.ground_restore.lock':self.handles.append(handle)
            return handle
        output=io.StringIO()
        with tempfile.TemporaryDirectory(prefix='ground_startup_test_') as directory:
            try:
                with patch.dict(sys.modules,modules),patch.object(app,'BUNDLE',Path(directory)), \
                     patch.object(app,'latest_messages',side_effect=self.latest), \
                     patch.object(app,'spawn',side_effect=self.spawn), \
                     patch.object(app.sys,'argv',['full_chain_ground.py','--restore']), \
                     patch.object(Path,'read_text',read_text),patch.object(Path,'open',tracked_open), \
                     contextlib.redirect_stdout(output):
                    return app.main()
            finally:
                self.output=output.getvalue()
                for handle in self.handles:handle.close()

    def pose_starts(self):
        return [args for name,args in self.spawned if name=='guarded_pose']


class GroundStartupOrchestrationTests(unittest.TestCase):
    def test_cold_lio_old_manual_then_fresh_altctl_starts_once(self):
        scene=GroundScenario()
        with self.assertRaises(StopAfterPoseDecision):scene.run()
        self.assertEqual([name for name,_ in scene.spawned],['lio','guarded_pose'])
        self.assertLess(scene.events.index('sample1'),scene.events.index('spawn:lio'))
        self.assertLess(scene.events.index('spawn:lio'),scene.events.index('sample2'))
        self.assertLess(scene.events.index('sample2'),scene.events.index('spawn:guarded_pose'))
        self.assertEqual(len(scene.pose_starts()),1)
        self.assertIn("mode='ALTCTL'",scene.output)
        self.assertIn('POSE_STARTUP_SNAPSHOT',scene.output)

    def test_new_manual_mode_refuses_without_waiting_or_starting_pose(self):
        scene=GroundScenario(early='ALTCTL',fresh='MANUAL')
        with self.assertRaises(ValueError) as error:scene.run()
        self.assertEqual([name for name,_ in scene.spawned],['lio'])
        self.assertEqual(scene.samples,2)
        for value in ('observed=MANUAL','stamp=99.900000000','age_s=0.100000000',
                      "callerid='/mavros'","mode='MANUAL'"):
            self.assertIn(value,str(error.exception))

    def test_bridge_appearing_while_lio_starts_is_reused_without_duplicate(self):
        def appeared(scene):
            scene.pubs[STATUS]=['/existing_bridge'];scene.pubs[VISION]=['/existing_bridge']
            scene.stop_at_sample=2
        scene=GroundScenario(after_lio=appeared)
        with self.assertRaises(StopAfterPoseDecision):scene.run()
        self.assertEqual(scene.pose_starts(),[])
        self.assertEqual(scene.status_reads,1)

    def test_status_only_after_lio_starts_is_refused_without_duplicate(self):
        def appeared(scene):scene.pubs[STATUS]=['/existing_bridge']
        scene=GroundScenario(after_lio=appeared)
        with self.assertRaisesRegex(ValueError,'pose bridge partial/ambiguous'):scene.run()
        self.assertEqual(scene.pose_starts(),[])
        self.assertEqual(scene.samples,1)

    def test_controller_appearing_during_final_sample_blocks_pose(self):
        def appeared(scene,index):
            if index==2:scene.pubs['/mavros/setpoint_raw/local']=['/other_controller']
        scene=GroundScenario(after_sample=appeared)
        with self.assertRaisesRegex(ValueError,'flight setpoint publisher present'):scene.run()
        self.assertEqual(scene.pose_starts(),[])
        self.assertEqual(scene.samples,2)

    def test_second_lio_appearing_during_final_sample_blocks_pose(self):
        def appeared(scene,index):
            if index==2:scene.pubs['/window_pass_lio/Odometry'].append('/other_lio')
        scene=GroundScenario(after_sample=appeared)
        with self.assertRaisesRegex(ValueError,'LIO source ambiguous before pose startup'):scene.run()
        self.assertEqual(scene.pose_starts(),[])

    def test_bridge_appearing_during_final_sample_blocks_duplicate_pose(self):
        def appeared(scene,index):
            if index==2:
                scene.pubs[STATUS]=['/existing_bridge'];scene.pubs[VISION]=['/existing_bridge']
        scene=GroundScenario(after_sample=appeared)
        with self.assertRaisesRegex(ValueError,'pose bridge appeared during startup'):scene.run()
        self.assertEqual(scene.pose_starts(),[])

    def test_split_bridge_publishers_after_lio_start_refuse(self):
        def appeared(scene):
            scene.pubs[STATUS]=['/existing_bridge'];scene.pubs[VISION]=['/different_bridge']
        scene=GroundScenario(after_lio=appeared)
        with self.assertRaisesRegex(ValueError,'pose bridge partial/ambiguous'):scene.run()
        self.assertEqual(scene.pose_starts(),[])

    def test_existing_bridge_status_wrong_caller_is_refused(self):
        def appeared(scene):
            scene.pubs[STATUS]=['/existing_bridge'];scene.pubs[VISION]=['/existing_bridge']
            scene.status=status_message('/wrong_source')
        scene=GroundScenario(after_lio=appeared)
        with self.assertRaisesRegex(ValueError,'untrusted pose status source'):scene.run()
        self.assertEqual(scene.pose_starts(),[])


class LatchedBridgeReplacementTests(unittest.TestCase):
    NODE='/window_pass_competition_pose_4192_1319845'

    def scene(self,status=None,caller=None,vision=None,command=None,exits=True,setpoint=False,lio=1,ground_error=None):
        data={'real_output_active':False,'real_fault':'HOVER_RAW_LIO_lio_pose_discontinuity','raw_lio_guard_scope':'RouteContinuity','real_published':3613}
        data.update(status or {})
        s=NS(pubs={STATUS:[self.NODE],'/window_pass_lio/Odometry':[LIO]*lio},killed=[],ground_calls=0,alive=True)
        if vision is not None:s.pubs[VISION]=vision
        if setpoint:s.pubs['/mavros/setpoint_raw/local']=['/other_controller']
        message=NS(data=json.dumps(data),_connection_header={'callerid':caller or self.NODE})
        s.read=lambda:message
        def ground():
            s.ground_calls+=1
            if ground_error:raise ValueError(ground_error)
        s.ground=ground
        s.graph=lambda:copy.deepcopy(s.pubs)
        def kill(nodes):
            s.killed.extend(nodes)
            if exits:
                s.pubs.pop(STATUS,None);s.pubs.pop(VISION,None);s.alive=False
        s.kill=kill
        s.command=command or 'python3 '+str(app.BUNDLE/'autonomous_window_test/scripts/start_full_chain_pose.py')+' --run'
        ticks=iter(range(1000))
        def replace():
            return app.replace_latched_pose_bridge(copy.deepcopy(s.pubs),s.read,s.ground,s.graph,
                lambda node:(4192,s.command),s.kill,lambda pid:s.alive,sleep=lambda x:None,clock=lambda:next(ticks))
        s.replace=replace
        return s

    def test_latched_bridge_on_trusted_ground_is_replaced(self):
        s=self.scene()
        self.assertTrue(s.replace())
        self.assertEqual(s.killed,[self.NODE])
        self.assertGreaterEqual(s.ground_calls,1)

    def test_healthy_bridge_with_vision_publisher_is_not_touched(self):
        s=self.scene(vision=[self.NODE])
        self.assertFalse(s.replace())
        self.assertEqual(s.killed,[])

    def test_two_status_publishers_are_not_replaced(self):
        s=self.scene();s.pubs[STATUS]=[self.NODE,'/other_bridge']
        self.assertFalse(s.replace())
        self.assertEqual(s.killed,[])

    def test_untrusted_status_caller_is_not_replaced(self):
        s=self.scene(caller='/wrong_source')
        self.assertFalse(s.replace())
        self.assertEqual(s.killed,[])

    def test_foreign_node_name_is_not_replaced(self):
        s=self.scene();s.pubs[STATUS]=['/existing_bridge']
        self.assertFalse(s.replace())
        self.assertEqual(s.killed,[])

    def test_active_output_or_missing_fault_is_not_replaced(self):
        for changes in ({'real_output_active':True},{'real_fault':''}):
            s=self.scene(status=changes)
            self.assertFalse(s.replace())
            self.assertEqual(s.killed,[])

    def test_wrong_guard_scope_is_not_replaced(self):
        s=self.scene(status={'raw_lio_guard_scope':'HoverContinuity'})
        self.assertFalse(s.replace())
        self.assertEqual(s.killed,[])

    def test_flight_setpoint_publisher_refuses_before_kill(self):
        s=self.scene(setpoint=True)
        with self.assertRaisesRegex(ValueError,'flight setpoint publisher present'):s.replace()
        self.assertEqual(s.killed,[])

    def test_ambiguous_lio_refuses_before_kill(self):
        s=self.scene(lio=2)
        with self.assertRaisesRegex(ValueError,'LIO source ambiguous'):s.replace()
        self.assertEqual(s.killed,[])

    def test_untrusted_ground_refuses_before_kill(self):
        s=self.scene(ground_error='connected disarmed ground required')
        with self.assertRaisesRegex(ValueError,'disarmed ground'):s.replace()
        self.assertEqual(s.killed,[])

    def test_process_outside_this_bundle_is_not_killed(self):
        s=self.scene(command='python3 /home/abot/other_bundle/scripts/start_full_chain_pose.py --run')
        with self.assertRaisesRegex(ValueError,'not this bundle process'):s.replace()
        self.assertEqual(s.killed,[])

    def test_process_that_does_not_exit_is_reported(self):
        s=self.scene(exits=False)
        with self.assertRaisesRegex(ValueError,'did not exit'):s.replace()
        self.assertEqual(s.killed,[self.NODE])


if __name__=='__main__':unittest.main()

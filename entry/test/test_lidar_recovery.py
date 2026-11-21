import os
import unittest
from unittest.mock import patch

from competition import (ensure_unique_lidar,require_prepare_sources,
                         prepare_support_after_mavros,prompt_target_selection,
                         wait_for_unique_topic,configure_ros_package_path,FLIGHT)


class FakeMaster:
    def __init__(self, counts, topic='/livox/lidar', node='/livox_lidar_publisher2'):
        self.counts = iter(counts)
        self.topic = topic
        self.node = node

    def getSystemState(self):
        count = next(self.counts)
        if isinstance(count, Exception):
            raise count
        names = [self.node] * count
        pubs = [(self.topic, names)] if names else []
        return pubs, [], []


class LidarRecoveryTests(unittest.TestCase):
    def test_bridge_waits_for_delayed_status_publisher(self):
        master=FakeMaster([0,0,1],topic='/hover_target_laser/status',node='/hover_target_laser')
        clock=iter([0.,0.,.1,.2]).__next__
        self.assertEqual(wait_for_unique_topic(master,'/hover_target_laser/status',
                         sleep=lambda _s:None,clock=clock),'/hover_target_laser')

    def test_bridge_wait_rejects_ambiguous_publishers(self):
        master=FakeMaster([2],topic='/hover_target_laser/status',node='/hover_target_laser')
        with self.assertRaisesRegex(ValueError,'duplicate_status_publishers'):
            wait_for_unique_topic(master,'/hover_target_laser/status',sleep=lambda _s:None)

    def test_ros_package_path_prefers_aic_copy_and_removes_canonical_duplicate(self):
        local=[str(FLIGHT/'rgbd'),str(FLIGHT/'vision_ws/src'),
               '/home/abot/window_pass_session_20261013']
        canonical='/home/abot/full_chain_20261030_lidar_recovery/vision_ws/src'
        original=os.pathsep.join(local+[canonical,local[1],'/opt/ros/noetic/share',canonical])
        with patch.dict(os.environ,{'ROS_PACKAGE_PATH':original}):
            configure_ros_package_path()
            result=os.environ['ROS_PACKAGE_PATH'].split(os.pathsep)
        self.assertEqual(result[:3],local)
        self.assertEqual(result[3:],['/opt/ros/noetic/share'])

    def test_prompt_accepts_each_pair_and_sorts_selection(self):
        for value,expected in [('12',[1,2]),('13',[1,3]),('23',[2,3]),('3,1',[1,3]),('2 3',[2,3])]:
            prompts=[]
            self.assertEqual(prompt_target_selection(input_fn=lambda:value,
                             output_fn=prompts.append),expected)
            self.assertEqual(len(prompts),1)

    def test_prompt_retries_invalid_or_repeated_target(self):
        answers=iter(['123','11','24','13']);prompts=[]
        self.assertEqual(prompt_target_selection(input_fn=lambda:next(answers),
                         output_fn=prompts.append),[1,3])
        self.assertEqual(len(prompts),7)

    def test_explicit_targets_skip_interactive_prompt(self):
        def unexpected():raise AssertionError('prompt must not run')
        self.assertEqual(prompt_target_selection([2,1],input_fn=unexpected),[1,2])

    def test_unique_existing_publisher_is_reused(self):
        started = []
        self.assertEqual(ensure_unique_lidar(FakeMaster([1]), lambda: [],
                                              lambda: started.append(True)),
                         '/livox_lidar_publisher2')
        self.assertEqual(started, [])

    def test_missing_publisher_starts_driver_once(self):
        started = []
        clock = iter([0, 0, 0, 0, 0]).__next__
        result = ensure_unique_lidar(FakeMaster([0, 0, 1]), lambda: [],
                                     lambda: started.append(True), sleep=lambda _: None,
                                     clock=clock)
        self.assertEqual(result, '/livox_lidar_publisher2')
        self.assertEqual(started, [True])

    def test_existing_driver_is_not_duplicated(self):
        started = []
        clock = iter([0, 0, 0]).__next__
        result = ensure_unique_lidar(FakeMaster([0, 1]),
                                     lambda: ['/livox_lidar_publisher2'],
                                     lambda: started.append(True), sleep=lambda _: None,
                                     clock=clock)
        self.assertEqual(result, '/livox_lidar_publisher2')
        self.assertEqual(started, [])

    def test_duplicate_publishers_fail_closed(self):
        with self.assertRaisesRegex(ValueError, 'multiple /livox/lidar publishers'):
            ensure_unique_lidar(FakeMaster([2]), lambda: [], lambda: None)

    def test_transient_master_failure_is_retried(self):
        clock = iter([0, 0, 0]).__next__
        result = ensure_unique_lidar(FakeMaster([RuntimeError('starting'), 1]),
                                     lambda: [], lambda: None, sleep=lambda _: None,
                                     clock=clock)
        self.assertEqual(result, '/livox_lidar_publisher2')


class StartupMaster:
    def __init__(self,pubs=None):self.pubs=pubs or {}
    def getSystemState(self):return list(self.pubs.items()),[],[]
    def lookupNode(self,name):return 'http://127.0.0.1:42059/'


class PrepareBootstrapTests(unittest.TestCase):
    def setUp(self):
        self.processes=patch('competition.local_lio_processes',return_value=[])
        self.mock_processes=self.processes.start();self.addCleanup(self.processes.stop)

    def callbacks(self,events):
        return [lambda:events.append('start_mavros'),lambda:events.append('wait_connected'),lambda:events.append('support')]

    def test_master_online_without_mavros_starts_it_before_telemetry_wait(self):
        events=[]
        prepare_support_after_mavros(StartupMaster(),set(),*self.callbacks(events))
        self.assertEqual(events,['start_mavros','wait_connected'])

    def test_existing_support_migration_waits_for_new_mavros_connection(self):
        events=[];master=StartupMaster({'/hover_target_laser/status':['/hover_target_laser']})
        prepare_support_after_mavros(master,set(),*self.callbacks(events))
        self.assertEqual(events,['start_mavros','wait_connected','support'])

    def test_live_mavros_is_reused_but_still_needs_fresh_connected_state(self):
        events=[];master=StartupMaster({'/mavros/state':['/mavros']})
        prepare_support_after_mavros(master,{'/mavros'},*self.callbacks(events),probe=lambda uri:123)
        self.assertEqual(events,['wait_connected'])

    def test_failed_connection_does_not_migrate_support(self):
        events=[]
        def fail():raise RuntimeError('no heartbeat')
        with self.assertRaisesRegex(RuntimeError,'no heartbeat'):
            prepare_support_after_mavros(StartupMaster({'/hover_target_laser/status':['/hover_target_laser']}),set(),
                lambda:events.append('start_mavros'),fail,lambda:events.append('support'))
        self.assertEqual(events,['start_mavros'])

    def test_stale_mavros_and_foreign_lio_are_both_reported_before_any_start(self):
        events=[];master=StartupMaster({'/mavros/state':['/mavros'],'/Odometry':['/laserMapping']})
        def dead(uri):raise ConnectionRefusedError('Connection refused')
        with self.assertRaises(ValueError) as caught:
            prepare_support_after_mavros(master,{'/mavros','/laserMapping'},*self.callbacks(events),probe=dead)
        self.assertIn('mavros_registration_unreachable',str(caught.exception))
        self.assertIn('external_lio_present',str(caught.exception))
        self.assertIn('127.0.0.1:42059',str(caught.exception))
        self.assertEqual(events,[])

    def test_foreign_lio_node_is_refused_even_before_its_first_odometry(self):
        with self.assertRaisesRegex(ValueError,'external_lio_present'):
            require_prepare_sources(StartupMaster(),{'/laserMapping'})

    def test_competition_lio_is_accepted_and_other_mavros_owner_is_refused(self):
        self.mock_processes.return_value=[{'pid':123,'argv':['/known/fastlio_mapping']}]
        require_prepare_sources(StartupMaster({'/window_pass_lio/Odometry':['/window_pass_lio/laserMapping']}),{'/window_pass_lio/laserMapping'},probe=lambda uri:123)
        with self.assertRaisesRegex(ValueError,'mavros_source_conflict'):
            require_prepare_sources(StartupMaster({'/mavros/state':['/other']}),set())

    def test_active_flight_controller_prevents_bootstrap(self):
        events=[]
        with self.assertRaisesRegex(ValueError,'flight_controller_present'):
            prepare_support_after_mavros(StartupMaster({'/mavros/setpoint_raw/local':['/controller']}),set(),*self.callbacks(events))
        self.assertEqual(events,[])

    def test_orphan_lio_process_is_refused_even_when_master_has_no_registration(self):
        self.mock_processes.return_value=[{'pid':13772,'argv':['/home/abot/234471/devel/lib/fast_lio/fastlio_mapping']}]
        events=[]
        with self.assertRaisesRegex(ValueError,'external_lio_present') as caught:
            prepare_support_after_mavros(StartupMaster(),set(),*self.callbacks(events))
        self.assertIn('13772',str(caught.exception))
        self.assertEqual(events,[])


class CleanPrepareTests(unittest.TestCase):
    def row(self,pid,ppid,argv,start='100'):
        return dict(pid=pid,ppid=ppid,argv=argv,start=start)

    def test_ros_launchers_and_descendants_selected_but_ssh_and_system_preserved(self):
        from prepare_restart import robot_processes
        rows={1:self.row(1,0,['/sbin/init']),10:self.row(10,1,['/usr/sbin/sshd']),
              20:self.row(20,10,['/usr/bin/python3','/opt/ros/noetic/bin/roslaunch','uav_mission','uav_mission2026.launch']),
              21:self.row(21,20,['/home/abot/custom_robot_node']),
              30:self.row(30,1,['/home/abot/234471/devel/lib/fast_lio/fastlio_mapping']),
              40:self.row(40,10,['bash','-c','echo roslaunch'])}
        self.assertEqual(set(robot_processes(rows)),{20,21,30})

    def test_failed_ground_proof_prevents_inventory_shutdown(self):
        import prepare_restart as reset
        with patch.object(reset,'sample_ground',side_effect=RuntimeError('no ground proof')),patch.object(reset,'stop_inventory') as stop:
            with self.assertRaisesRegex(RuntimeError,'no ground proof'):reset.reset_proven_session()
        stop.assert_not_called()

    def test_unreachable_existing_mavros_is_checked_without_starting_duplicate(self):
        import prepare_restart as reset
        row=self.row(31,1,['/opt/ros/noetic/lib/mavros/mavros_node'])
        self.assertEqual(reset.mavros_bootstrap_decision(False,{31:row}),
                         'verify_existing_mavros_telemetry')

    def test_unreachable_existing_mavros_continues_to_proof_before_reset(self):
        import prepare_restart as reset
        row=self.row(31,1,['/opt/ros/noetic/lib/mavros/mavros_node'])

        class Master:
            def lookupNode(self,*_args):return 1,'','http://127.0.0.1:42059/'

        class Node:
            def __enter__(self):return self
            def __exit__(self,*_args):return False
            def getPid(self,*_args):return 0,'unreachable',0

        with patch.object(reset.sys,'argv',['prepare_restart.py']), \
             patch.object(reset,'processes',return_value={31:row}), \
             patch.object(reset,'master_online',return_value=True), \
             patch.object(reset,'master',return_value=Master()), \
             patch.object(reset.xmlrpc.client,'ServerProxy',return_value=Node()), \
             patch.object(reset,'spawn_bootstrap') as start, \
             patch.object(reset,'reset_proven_session') as reset_session:
            reset.main()

        start.assert_not_called()
        reset_session.assert_called_once_with()

    def test_missing_mavros_process_starts_only_minimal_bootstrap(self):
        import prepare_restart as reset
        self.assertEqual(reset.mavros_bootstrap_decision(False,{}),
                         'start_minimal_mavros')

    def test_reachable_registered_mavros_is_reused(self):
        import prepare_restart as reset
        row=self.row(31,1,['/opt/ros/noetic/lib/mavros/mavros_node'])
        self.assertEqual(reset.mavros_bootstrap_decision(True,{31:row}),
                         'reuse_registered_mavros')

    def test_registered_local_robot_node_is_included(self):
        from prepare_restart import robot_processes
        rows={20:self.row(20,1,['python3','/home/abot/custom_node.py']),
              30:self.row(30,1,['/usr/sbin/sshd'])}
        self.assertEqual(set(robot_processes(rows,[20,30])),{20})

    def test_reused_pid_cannot_be_signalled(self):
        import prepare_restart as reset
        old=self.row(20,1,['/robot'],start='100')
        with patch.object(reset,'processes',return_value={20:self.row(20,1,['/robot'],start='200')}),patch.object(reset.os,'kill') as kill:
            reset.signal_process(old,reset.signal.SIGINT)
        kill.assert_not_called()

    def test_normal_stop_uses_only_sigint(self):
        import prepare_restart as reset
        active=[True];row=self.row(20,1,['/robot']);signals=[]
        def stop(row,number):signals.append(number);active[0]=False
        with patch.object(reset,'same_process',side_effect=lambda row:active[0]),patch.object(reset,'signal_process',side_effect=stop):
            reset.stop_inventory({20:row})
        self.assertEqual(signals,[reset.signal.SIGINT])

    def test_unstoppable_process_prevents_success_after_escalation(self):
        import prepare_restart as reset
        row=self.row(20,1,['/robot'])
        phases=[(reset.signal.SIGINT,0),(reset.signal.SIGTERM,0),(reset.SIGNAL_KILL,0)]
        with patch.object(reset,'same_process',return_value=True),patch.object(reset,'signal_process') as send:
            with self.assertRaisesRegex(RuntimeError,'did not exit'):reset.stop_inventory({20:row},phases=phases)
        self.assertEqual([c.args[1] for c in send.call_args_list],[p[0] for p in phases])


if __name__ == '__main__':
    unittest.main()

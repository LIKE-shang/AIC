"""Offline startup convergence tests; no ROS nodes or services are started."""
import copy
import importlib.util
import json
from pathlib import Path
import threading
import types
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('startup_wait_app', ROOT/'scripts/auto_window.py')
app = importlib.util.module_from_spec(spec)
spec.loader.exec_module(app)


def report(*blockers):
    return dict(ready=not blockers, blockers=list(blockers), mode='ALTCTL',
                armed=False, pose_time_guard={'offset_span_ns':14539776})


class StartupWaitTests(unittest.TestCase):
    def simulate(self, read):
        self.t = 0.
        self.waits = []
        def wait(seconds):
            self.waits.append(seconds)
            self.t += seconds
        with patch('builtins.print'):
            return app.wait_readonly_timesync(read, wait, lambda:False, clock=lambda:self.t)

    def test_observed_failure_converges_in_same_observer(self):
        reads = []
        def read():
            reads.append(self.t)
            return report('pose_time_guard:timesync_offset_unstable') if self.t < 2. else report()
        result = self.simulate(read)
        self.assertTrue(result['ready'])
        self.assertLess(result['startup_timesync_wait']['elapsed_s'], 2.2)
        self.assertEqual(result['startup_timesync_wait']['initial_guard']['offset_span_ns'],14539776)
        self.assertGreater(len(reads), 2)

    def test_persistent_offset_failure_stops_after_15_seconds(self):
        result = self.simulate(lambda:report('pose_time_guard:timesync_offset_unstable'))
        self.assertFalse(result['ready'])
        self.assertEqual(result['blockers'], ['pose_time_guard:timesync_offset_unstable'])
        self.assertAlmostEqual(self.t, 15.)
        self.assertEqual(result['startup_timesync_wait']['outcome'], 'timeout')

    def test_other_blockers_are_never_waited_or_removed(self):
        for blocker in ('ground_state_not_ready','rc_not_ready','graph:foreign_source',
                        'estimator_not_ready:stale','pose_time_guard:rejected_pose_in_window',
                        'pose_time_guard:pose_future_headroom_insufficient',
                        'pose_time_guard:invalid_or_untrusted_timesync',
                        'pose_time_guard:timesync_rtt_excessive'):
            for blockers in ((blocker,),('pose_time_guard:timesync_offset_unstable',blocker)):
                original=report(*blockers)
                self.assertIs(self.simulate(lambda:original), original)
                self.assertEqual(self.waits, [])

    def test_new_ground_fault_ends_wait_immediately(self):
        def read():
            return report('ground_state_not_ready') if self.t>=.3 else report('pose_time_guard:timesync_offset_unstable')
        result=self.simulate(read)
        self.assertFalse(result['ready'])
        self.assertLess(self.t, .5)
        self.assertEqual(result['blockers'], ['ground_state_not_ready'])

    def test_ready_does_not_add_delay(self):
        original=report()
        self.assertIs(self.simulate(lambda:original),original)
        self.assertEqual(self.waits,[])

    def test_stop_during_wait_cannot_become_ready(self):
        read=Mock(side_effect=[report('pose_time_guard:timesync_offset_unstable'),report()])
        stopped=[False]
        with patch('builtins.print'), self.assertRaisesRegex(RuntimeError,'interrupted'):
            app.wait_readonly_timesync(read,lambda seconds:stopped.__setitem__(0,True),lambda:stopped[0],clock=lambda:0.)
        self.assertEqual(read.call_count,1)

    def test_read_failure_is_not_swallowed(self):
        read=Mock(side_effect=[report('pose_time_guard:timesync_offset_unstable'),ValueError('source changed')])
        with patch('builtins.print'), self.assertRaisesRegex(ValueError,'source changed'):
            app.wait_readonly_timesync(read,lambda seconds:None,lambda:False,clock=lambda:0.)

    def test_real_guard_requires_original_window_before_ready(self):
        from pose_time_guard import PoseTimeGuard
        from window_pass.core import Controller, Pose
        cfg=json.loads((ROOT/'config/full_chain_second_window_return.json').read_text())['control']
        guard=PoseTimeGuard(cfg,True)
        core=Controller(copy.deepcopy(cfg))
        last=[-1]
        def read():
            end=150+round(self.t*50)
            for n in range(last[0]+1,end+1):
                t=n*.02
                p=Pose(100.+t-.005,(0.,0.,1.),(0.,0.,0.,1.),'map','base_link')
                prior=core.pose;stamp=core.latest_stamps.get('pose',0.)
                accepted=core.pose_update(p,100.+t,10.+t)
                guard.pose(p,100.+t,10.+t,prior,stamp,accepted,'IDLE')
                if n%5==0:
                    offset=1e18+(14539776 if n<25 else 0)
                    guard.timesync(100.+t,10.+t,100.+t,offset,offset,1.,'/mavros')
            last[0]=end
            r=guard.ground(100.+end*.02,10.+end*.02)
            return dict(ready=r['ready'],blockers=['pose_time_guard:'+b for b in r['blockers']],pose_time_guard=r)
        result=self.simulate(read)
        self.assertTrue(result['ready'])
        self.assertGreaterEqual(self.t,.4)
        self.assertEqual(cfg['future_tolerance_s'],.02)
        self.assertLessEqual(result['pose_time_guard']['offset_span_ns'],5e6)

    def test_runtime_readonly_path_never_takes_flight_authority(self):
        r=app.Runtime.__new__(app.Runtime)
        r.execute=False;r.full_chain_enabled=True;r.lock=threading.RLock()
        r.stop=threading.Event();r.ros=types.SimpleNamespace(is_shutdown=lambda:False)
        r.preflight_report=Mock(return_value=report('pose_time_guard:timesync_offset_unstable'))
        r.report=Mock(return_value=report())
        r.request=Mock();r.prepare_ground_posctl=Mock();r.publish=Mock()
        with patch.object(app.time,'sleep'),patch.object(r.stop,'wait'),patch('builtins.print'):
            self.assertEqual(r.run(),0)
        r.report.assert_called_once()
        r.request.assert_not_called();r.prepare_ground_posctl.assert_not_called();r.publish.assert_not_called()

    def test_execute_path_does_not_use_startup_wait(self):
        r=app.Runtime.__new__(app.Runtime);r.execute=True;r.full_chain_enabled=True
        r.preflight_report=Mock(return_value=report('pose_time_guard:timesync_offset_unstable'))
        with patch.object(app.time,'sleep'),patch.object(app,'wait_readonly_timesync') as waiter,patch('builtins.print'):
            with self.assertRaisesRegex(RuntimeError,'PREFLIGHT_BLOCKED'):r.run()
        waiter.assert_not_called()


if __name__=='__main__':unittest.main()

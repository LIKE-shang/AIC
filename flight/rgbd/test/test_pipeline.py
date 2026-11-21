"""Regression checks for truthful stream status and independent output expiry."""
import unittest
from unittest.mock import patch
import numpy as np
from test_node import MODULE, Clock, node, image
from scenes import scene
from test_guidance import detection
from window_live_vision.tracking import Tracker
from window_live_vision.pipeline import PipelineMonitor, FrameProfile, summarize_frames


class PipelineRegressionTests(unittest.TestCase):
    def frame(self, n, stamp=10., wall=20., frame=None):
        Clock.now_value=stamp
        msg=image(scene()[0] if frame is None else frame,stamp)
        with patch.object(MODULE.time,'monotonic',return_value=wall):
            n.image_cb(msg)
            n.process(msg,wall)
        return msg

    def tick(self,n,ros,wall):
        Clock.now_value=ros
        with patch.object(MODULE.time,'monotonic',return_value=wall):
            n.watchdog_once()

    def test_fresh_blank_image_keeps_no_target_reason(self):
        n=node();self.frame(n,frame=np.full((480,640,3),140,np.uint8))
        self.assertEqual(n.status['reason'],'no_complete_frame')
        self.tick(n,10.1,20.1)
        self.assertEqual(n.status['reason'],'no_complete_frame')
        self.assertEqual(n.status['target_count'],0)
        self.assertTrue(n.status['runtime']['streams']['rgb']['receipt_fresh'])
        self.assertFalse(n.status['ready'])

    def test_confirming_frame_is_not_mislabeled_as_stream_loss(self):
        n=node();self.frame(n)
        self.tick(n,10.1,20.1)
        self.assertEqual(n.status['reason'],'confirming_current_target')
        self.assertEqual(n.status['confirmed_frames'],1)
        self.assertEqual(n.tracker.count,1)

    def test_exception_stays_visible_while_input_is_fresh(self):
        n=node();Clock.now_value=10.;msg=image(scene()[0],encoding='mono8')
        with patch.object(MODULE.time,'monotonic',return_value=20.):
            n.image_cb(msg)
            with self.assertRaisesRegex(ValueError,'invalid_image_header'):
                n.process(msg,20.)
        self.tick(n,10.1,20.1)
        self.assertIn('image_rejected:invalid_image_header',n.status['reason'])
        self.assertIn('image_rejected:invalid_image_header',n.status['last_frame_diagnostics']['reason'])
        self.assertIsNone(n.monitor.active)

    def test_real_input_loss_cancels_and_keeps_last_frame_evidence(self):
        n=node()
        for i in range(3):self.frame(n,10.+.2*i,20.+.2*i)
        self.assertTrue(n.status['ready'])
        self.tick(n,11.,21.)
        self.assertEqual(n.status['reason'],'image_stream_stale')
        self.assertFalse(n.status['ready']);self.assertIsNone(n.status['visual_instruction'])
        self.assertFalse(n.status['metric_geometry_ready']);self.assertIsNone(n.status['metric_geometry'])
        self.assertIsNone(n.status['metric_preview']);self.assertIsNone(n.status['traversal_plan'])
        self.assertEqual(n.tracker.count,0)
        self.assertEqual(n.status['last_frame_diagnostics']['reason'],'live_visual_tracking')
        self.assertFalse(n.status['actual_output_permitted'])

    def test_output_expiry_retains_history_for_fresh_inflight_frame(self):
        n=node()
        for i in range(3):self.frame(n,10.+.2*i,20.+.2*i)
        msg=image(scene()[0],10.6)
        with patch.object(MODULE.time,'monotonic',return_value=20.6):n.image_cb(msg)
        # Old result has expired, but a more recent image arrived while the
        # worker was busy. Revoke output, do not manufacture a stream dropout.
        self.tick(n,10.91,20.91)
        self.assertEqual(n.status['reason'],'processing_result_stale')
        self.assertFalse(n.status['ready']);self.assertIsNone(n.status['visual_instruction'])
        self.assertEqual(n.tracker.count,3)
        Clock.now_value=10.99
        with patch.object(MODULE.time,'monotonic',return_value=20.99):n.process(msg,20.6)
        self.assertTrue(n.status['ready']);self.assertEqual(n.status['confirmed_frames'],4)
        self.assertFalse(n.status['actual_output_permitted'])

    def test_real_source_gap_still_requires_new_confirmation(self):
        n=node()
        for i in range(3):self.frame(n,10.+.2*i,20.+.2*i)
        self.frame(n,10.85,20.85)
        self.assertFalse(n.status['ready']);self.assertEqual(n.status['confirmed_frames'],1)
        self.assertEqual(n.status['tracking_reset_reason'],'image_receipt_gap_exceeded')
        self.assertAlmostEqual(n.status['tracking_receipt_gap_s'],.45)
        self.assertEqual(n.status['configuration']['max_track_gap_s'],.4)
        self.assertEqual(n.status['configuration']['max_image_age_s'],.5)

    def test_slow_frame_reason_not_overwritten_by_fresh_stream_watchdog(self):
        n=node();Clock.now_value=10.;msg=image(scene()[0])
        wall=[20.];original=n.detector.detect
        def slow(frame):
            out=original(frame);wall[0]=20.6;Clock.now_value=10.6
            n.image_cb(image(frame,10.6))
            return out
        with patch.object(MODULE.time,'monotonic',side_effect=lambda:wall[0]),patch.object(n.detector,'detect',side_effect=slow):
            n.image_cb(msg);n.process(msg,20.)
        self.tick(n,10.7,20.7)
        self.assertEqual(n.status['reason'],'processing_exceeded_freshness_limit')
        self.assertEqual(n.status['last_frame_diagnostics']['reason'],'processing_exceeded_freshness_limit')
        self.assertIn('rgb_detection',n.status['last_frame_diagnostics']['stage_ms'])
        self.assertFalse(n.status['ready'])

    def test_no_camera_yet_is_waiting_not_old_target_failure(self):
        n=node();self.tick(n,10.,20.)
        self.assertEqual(n.status['reason'],'waiting_for_first_image')
        self.assertFalse(n.status['runtime']['streams']['rgb']['receipt_fresh'])

    def test_stopped_ros_clock_cannot_leave_output_ready(self):
        n=node()
        for i in range(3):self.frame(n,10.+.2*i,20.+.2*i)
        with patch.object(MODULE.time,'monotonic',return_value=20.95):n.image_cb(image(scene()[0],10.4))
        self.tick(n,10.4,21.)
        self.assertEqual(n.status['reason'],'processing_result_stale')
        self.assertFalse(n.status['ready'])

    def test_monitor_reports_callback_rates_and_wait_separately(self):
        monitor=PipelineMonitor()
        for i in range(4):monitor.receive('rgb',20.+i*.1,10.+i*.1)
        profile=FrameProfile(10.3,20.3,20.35);monitor.begin(profile)
        snapshot=monitor.snapshot(20.4,10.4,.5)
        self.assertTrue(snapshot['processing_active'])
        self.assertAlmostEqual(snapshot['streams']['rgb']['callback_rate_hz'],10.)
        self.assertFalse(snapshot['streams']['depth']['receipt_fresh'])
        result=monitor.finish({'reason':'confirming_current_target'},profile,20.45,10.45)
        self.assertAlmostEqual(result['queue_wait_ms'],50.)
        self.assertAlmostEqual(result['result_age_wall_s'],.15)
        self.assertTrue(monitor.snapshot(20.45,10.45,.5)['result_fresh'])


class ConfirmationEvidenceTests(unittest.TestCase):
    def test_target_jump_records_overlap_reset(self):
        t=Tracker();t.update([detection()],10.,10.,20.,(480,640))
        row=t.update([detection(180)],10.2,10.2,20.2,(480,640))
        self.assertFalse(row['ready'])
        self.assertEqual(row['tracking_reset_reason'],'target_overlap_below_threshold')
        self.assertLess(row['tracking_iou'],.5)

    def test_source_gap_records_source_reset(self):
        t=Tracker();t.update([detection()],10.,10.,20.,(480,640))
        row=t.update([detection()],10.45,10.45,20.2,(480,640))
        self.assertEqual(row['tracking_reset_reason'],'image_stamp_gap_exceeded')
        self.assertEqual(row['confirmed_frames'],1)


class AdditionalPipelineTests(unittest.TestCase):
    def test_backwards_clock_during_depth_stage_revokes_outputs(self):
        from test_guidance import camera
        n=node();n.use_depth=True;n.depth_candidates=False
        n.info=camera();n.depth_info=camera()
        rgb=scene()[0]
        for i in range(3):
            stamp=10.+.2*i;wall=20.+.2*i;Clock.now_value=stamp
            n.depths=[(image(np.full((480,640),2000,np.uint16),stamp,'16UC1'),wall)]
            n.pose.add(stamp,[0,0,0],[0,0,0,1],'map');n.last_pose_wall=wall
            with patch.object(MODULE.time,'monotonic',return_value=wall):n.process(image(rgb,stamp),wall)
        Clock.now_value=10.6;n.depths=[(image(np.full((480,640),2000,np.uint16),10.6,'16UC1'),20.6)]
        original=MODULE.estimate_plane
        def reset_clock(*args):
            plane=original(*args);Clock.now_value=9.;return plane
        with patch.object(MODULE.time,'monotonic',return_value=20.6),patch.object(MODULE,'estimate_plane',side_effect=reset_clock):
            n.process(image(rgb,10.6),20.6)
        self.assertFalse(n.status['ready']);self.assertIsNone(n.status['metric_geometry'])
        self.assertEqual(n.status['reason'],'processing_exceeded_freshness_limit')

    def test_true_stream_loss_requires_new_confirmation_after_return(self):
        test=PipelineRegressionTests();n=node()
        for i in range(3):test.frame(n,10.+.2*i,20.+.2*i)
        test.tick(n,11.,21.)
        test.frame(n,11.1,21.1)
        self.assertFalse(n.status['ready']);self.assertEqual(n.status['confirmed_frames'],1)
        self.assertEqual(n.status['tracking_reset_reason'],'image_stream_stale')

    def test_metric_and_route_expiry_keep_no_published_command(self):
        n=node();test=PipelineRegressionTests()
        for i in range(3):test.frame(n,10.+.2*i,20.+.2*i)
        # Exercise output revocation even when optional metric paths exist.
        n.status.update(metric_geometry={'source_stamp':10.4},metric_geometry_ready=True,
                        metric_preview={'target_position_enu':[0,0,0]},traversal_plan={'route':[]})
        with patch.object(MODULE.time,'monotonic',return_value=20.9):n.image_cb(image(scene()[0],10.9))
        test.tick(n,10.95,20.95)
        self.assertEqual(n.status['reason'],'processing_result_stale')
        self.assertFalse(n.status['metric_geometry_ready'])
        for key in ('metric_geometry','metric_preview','traversal_plan','visual_instruction'):
            self.assertIsNone(n.status[key])
        import json
        self.assertIsNone(json.loads(n.geometry_pub.rows[-1].data)['geometry'])
        self.assertIsNone(json.loads(n.traversal_pub.rows[-1].data)['plan'])
        self.assertFalse(json.loads(n.proposal_pub.rows[-1].data)['ready'])
        self.assertEqual(n.status['output_sent_count'],0)

    def test_frame_summary_distinguishes_resets_and_stage_timing(self):
        frames=[{'reason':'confirming_current_target','confirmed_frames':1,'tracking_reset_reason':'image_receipt_gap_exceeded',
                 'processing_ms':350.,'stage_ms':{'depth_registration':100.}},
                {'reason':'live_visual_tracking','confirmed_frames':3,'tracking_reset_reason':'',
                 'processing_ms':150.,'stage_ms':{'depth_registration':50.}}]
        report=summarize_frames(frames)
        self.assertEqual(report['observed_unique_frames'],2)
        self.assertEqual(report['processing_ms']['p50'],250.)
        self.assertEqual(report['processing_ms']['p95'],350.)
        self.assertEqual(report['tracking_resets'],{'image_receipt_gap_exceeded':1})
        self.assertEqual(report['confirmed_frames_max'],3)
        self.assertEqual(summarize_frames([])['processing_ms']['samples'],0)

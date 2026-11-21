#!/usr/bin/env python3
"""Latest-frame-only ROS RGB/RGB-D perception and conditional planning.

Never arms/switches modes or publishes a MAVROS setpoint. This is NOT A_READY.
"""
import copy
import json
import math
import threading
import time
from collections import deque
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from window_live_vision.detector import Detector, annotate
from window_live_vision.tracking import Tracker
from window_live_vision.pipeline import PipelineMonitor, FrameProfile
from window_live_vision.geometry import (PoseBuffer, camera_model, matched_depth,
                                         estimate_plane, step_preview, quaternion_matrix)
from window_live_vision.rgbd import (DepthRegistrar, DepthOpeningDetector, MetricTracker,
                                    aperture_evidence, fuse_candidates, world_geometry)
from window_live_vision.traversal import Safety, plan_route, evaluate_corridor
from window_live_vision.astra_contract import factory_checked_raw_info, factory_checked_tf

OUTPUTS = ('/window_live/status', '/window_live/proposal', '/window_live/image', '/window_live/source_image',
           '/window_live/geometry', '/window_live/traversal', '/window_live/source_depth', '/window_live/source_depth_info')


def info_dict(msg):
    return dict(stamp=msg.header.stamp.to_sec(), frame_id=msg.header.frame_id,
                width=msg.width, height=msg.height, K=list(msg.K), D=list(msg.D),
                model=msg.distortion_model, binning_x=msg.binning_x, binning_y=msg.binning_y,
                roi=(msg.roi.x_offset, msg.roi.y_offset, msg.roi.width, msg.roi.height))


class LiveNode:
    def __init__(self):
        cv2.setNumThreads(1)
        self.bridge = CvBridge()
        self.detector = Detector(int(rospy.get_param('~max_width', 640)))
        self.depth_detector = DepthOpeningDetector()
        self.registrar = DepthRegistrar()
        fast=rospy.get_param('~fast_confirmation',False)
        if type(fast) is not bool:raise ValueError('fast_confirmation must be boolean')
        self.metric_tracker = MetricTracker(fast_confirmation=fast)
        self.tracker = Tracker(max_gap_s=.60 if fast else .40)
        self.monitor = PipelineMonitor()
        self.pose = PoseBuffer()
        self.lock = threading.RLock()
        self.latest = self.info = self.depth_info = self.depth_info_msg = None
        self.depths = deque(maxlen=15)
        self.last_pose_wall = None
        self.seq = 0
        self.fps = float(rospy.get_param('~process_hz', 5.))
        if not math.isfinite(self.fps) or not 1 <= self.fps <= 15:
            raise ValueError('process_hz must be in [1,15]')
        self.use_depth = rospy.get_param('~registered_depth', False)
        self.depth_candidates = rospy.get_param('~depth_candidates', True)
        if type(self.use_depth) is not bool or type(self.depth_candidates) is not bool:
            raise ValueError('depth flags must be boolean')
        self.registration_mode = rospy.get_param('~registration_mode', 'strict_registered')
        if self.registration_mode not in ('strict_registered', 'software_tf'):
            raise ValueError('unknown_registration_mode')
        self.transform_provider = None
        self.astra_factory = bool(rospy.get_param('~astra_factory_contract', False))
        self.factory_params = None
        self.factory_wall = None
        self.factory_align = None
        self.factory_thread = None
        self.factory_attempt_wall = None
        self.factory_signature = None
        self.calibration_signature = None
        self.calibration_epoch = 0
        self.camera_prefix = rospy.get_param('~camera_prefix', 'camera')
        self.enable_traversal = rospy.get_param('~enable_traversal_planning', False)
        if type(self.enable_traversal) is not bool:
            raise ValueError('enable_traversal_planning must be boolean')
        if self.use_depth and self.registration_mode == 'software_tf':
            import tf2_ros
            self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(3.))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            self.transform_provider = self.raw_depth_transform
        self.camera_xyz = rospy.get_param('~camera_to_body_xyz', [.08, 0., -.06])
        self.camera_q = rospy.get_param('~camera_to_body_quaternion', [-.5, .5, -.5, .5])
        self.safety = Safety()
        safety_file = rospy.get_param('~safety_file', '')
        if safety_file and self.enable_traversal:
            with open(safety_file, encoding='utf-8') as handle:
                self.safety = Safety.from_dict(json.load(handle))
        pubs = []
        for i, topic in enumerate(OUTPUTS):
            if rospy.resolve_name(topic) != topic:
                raise ValueError('output remapping refused: '+topic)
            message_type = String if i in (0, 1, 4, 5) else (CameraInfo if i == 7 else Image)
            pubs.append(rospy.Publisher(topic, message_type, queue_size=1, latch=False))
        (self.status_pub, self.proposal_pub, self.image_pub, self.source_pub,
         self.geometry_pub, self.traversal_pub, self.depth_source_pub, self.depth_source_info_pub) = pubs
        self.status = dict(ready=False, reason='waiting_for_first_image', visual_instruction=None,
                           metric_preview=None, metric_geometry=None, metric_reason='registered_depth_disabled',
                           traversal_plan=None, traversal_reason='no_metric_geometry', source_stamp=None)
        self.subscribers = [
            rospy.Subscriber(rospy.get_param('~image_topic', '/camera/rgb/image_raw'), Image, self.image_cb, queue_size=1, buff_size=4194304),
            rospy.Subscriber(rospy.get_param('~camera_info_topic', '/camera/rgb/camera_info'), CameraInfo, self.info_cb, queue_size=1),
            rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb, queue_size=20)]
        if self.use_depth:
            topic = rospy.get_param('~depth_topic', '')
            info_topic = rospy.get_param('~depth_info_topic', '')
            if not topic or not info_topic:
                raise ValueError('depth requires explicit depth_topic and depth_info_topic')
            self.subscribers.extend([
                rospy.Subscriber(topic, Image, self.depth_cb, queue_size=1, buff_size=4194304),
                rospy.Subscriber(info_topic, CameraInfo, self.depth_info_cb, queue_size=1)])
        self.threads = [threading.Thread(target=self.worker, daemon=True), threading.Thread(target=self.watchdog, daemon=True)]
        for thread in self.threads:
            thread.start()
        rospy.loginfo('LIVE RGB-D: real-time aperture + conditional route; advisory only, NOT A_READY')

    def refresh_factory(self):
        try:
            from astra_camera.srv import GetCameraParams
            service = '/'+self.camera_prefix+'/get_camera_params'
            rospy.wait_for_service(service, timeout=.25)
            response = rospy.ServiceProxy(service, GetCameraParams)()
            params = {name: list(getattr(response, name)) for name in
                      ('l_intr_p', 'r_intr_p', 'r2l_r', 'r2l_t', 'l_k', 'r_k')}
            params['success'] = bool(response.success)
            align = rospy.get_param('/'+self.camera_prefix+'/camera/depth_align', None)
            # A stalled service runs only in this one daemon thread. The image
            # worker keeps publishing cancellation and the cache expires.
            with self.lock:
                self.factory_params, self.factory_wall, self.factory_align = params, time.monotonic(), align
        except Exception:
            with self.lock:
                self.factory_params, self.factory_wall, self.factory_align = None, None, None

    def factory_depth_info(self, info, depth_info, image_frame, depth_frame):
        if not self.astra_factory:
            return depth_info
        now = time.monotonic()
        with self.lock:
            if ((self.factory_thread is None or not self.factory_thread.is_alive()) and
                    (self.factory_attempt_wall is None or now-self.factory_attempt_wall >= 1.)):
                self.factory_attempt_wall = now
                self.factory_thread = threading.Thread(target=self.refresh_factory, daemon=True)
                self.factory_thread.start()
            if self.factory_params is None or self.factory_wall is None or now-self.factory_wall > 3.:
                raise ValueError('fresh_factory_calibration_pending_or_unavailable')
            params = copy.deepcopy(self.factory_params)
            align = self.factory_align
        checked = factory_checked_raw_info(info, depth_info, depth_frame, image_frame, params, align, self.camera_prefix)
        checked['factory_params_snapshot'] = params
        return checked

    def check_calibration_epoch(self, info, depth_info, rotation, translation):
        # Timestamp is deliberately excluded: refreshed CameraInfo timestamps
        # do not change calibration. Any actual model/TF change resets history.
        fields = ('frame_id', 'width', 'height', 'K', 'D', 'model', 'binning_x', 'binning_y', 'roi')
        values = [{key: data.get(key) for key in fields} for data in (info, depth_info)]
        values.extend((np.asarray(rotation).tolist(), np.asarray(translation).tolist()))
        values.append(depth_info.get('factory_params_snapshot'))
        signature = json.dumps(values, sort_keys=True, allow_nan=False)
        with self.lock:
            if self.calibration_signature is not None and signature != self.calibration_signature:
                self.calibration_epoch += 1
                self.metric_tracker.reset('camera_calibration_changed')
                self.tracker.cancel('camera_calibration_changed')
            self.calibration_signature = signature
        return self.calibration_epoch

    def raw_depth_transform(self, image_frame, depth_frame):
        # Only static optical-frame calibration: using Time(0) does NOT select
        # a latest body pose. The body pose is interpolated at image time below.
        transform = self.tf_buffer.lookup_transform(image_frame, depth_frame, rospy.Time(0), rospy.Duration(.02)).transform
        q, t = transform.rotation, transform.translation
        return quaternion_matrix([q.x, q.y, q.z, q.w]), np.array([t.x, t.y, t.z])

    def image_cb(self, msg):
        with self.lock:
            received = time.monotonic()
            self.latest = (msg, received)
            self.monitor.receive('rgb', received, msg.header.stamp.to_sec())

    def info_cb(self, msg):
        with self.lock:
            self.info = info_dict(msg)

    def depth_info_cb(self, msg):
        with self.lock:
            self.depth_info, self.depth_info_msg = info_dict(msg), msg

    def depth_cb(self, msg):
        with self.lock:
            received = time.monotonic()
            self.depths.append((msg, received))
            self.monitor.receive('depth', received, msg.header.stamp.to_sec())

    def pose_cb(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        try:
            with self.lock:
                self.pose.add(msg.header.stamp.to_sec(), [p.x, p.y, p.z], [q.x, q.y, q.z, q.w], msg.header.frame_id)
                self.last_pose_wall = time.monotonic()
        except ValueError:
            with self.lock:
                self.pose.samples.clear()
                self.pose.epoch += 1
                self.metric_tracker.reset('pose_rejected')
                self.last_pose_wall = None

    def emit_locked(self):
        self.seq += 1
        self.status.update(runtime=self.monitor.snapshot(time.monotonic(), rospy.Time.now().to_sec(), self.tracker.max_age_s),
                           last_frame_diagnostics=self.monitor.last_frame,
                           configuration=dict(process_hz=self.fps, depth_candidates=self.depth_candidates,
                               registered_depth=self.use_depth, registration_mode=self.registration_mode,
                               rgb_detection_max_width=self.detector.max_width, enable_traversal_planning=self.enable_traversal,
                               max_image_age_s=self.tracker.max_age_s, max_track_gap_s=self.tracker.max_gap_s,
                               fast_confirmation=self.metric_tracker.fast_confirmation,
                               metric_required_frames=self.metric_tracker.required_frames,
                               metric_minimum_span_s=self.metric_tracker.minimum_span_s))
        self.status.update(version='20261014_rgbd_v5', sequence=self.seq, ready_scope='visual_tracking_only',
                           output_active=False, output_sent_count=0, actual_output_permitted=False,
                           accepted_real_window_pass=False, requires_new_flight_integration=True,
                           registration_mode=self.registration_mode,
                           sensor_sync_and_accuracy_flight_verified=False)
        self.status_pub.publish(String(data=json.dumps(self.status, ensure_ascii=False, allow_nan=False)))
        self.proposal_pub.publish(String(data=json.dumps(dict(sequence=self.seq,
            source_stamp=self.status.get('source_stamp'), valid_for_s=self.tracker.max_age_s,
            ready=self.status['ready'], ready_scope='visual_tracking_only', visual_instruction=self.status.get('visual_instruction'),
            metric_preview=self.status.get('metric_preview'), actual_output_permitted=False), allow_nan=False)))
        self.geometry_pub.publish(String(data=json.dumps(dict(sequence=self.seq,
            geometry=self.status.get('metric_geometry'), reason=self.status.get('metric_geometry_reason', 'not_ready'),
            actual_output_permitted=False), allow_nan=False)))
        self.traversal_pub.publish(String(data=json.dumps(dict(sequence=self.seq,
            source_stamp=self.status.get('source_stamp'), plan=self.status.get('traversal_plan'),
            reason=self.status.get('traversal_reason', 'not_ready'), actual_output_permitted=False,
            output_sent_count=0, requires_new_flight_integration=True), allow_nan=False)))

    def process(self, msg, received):
        profile = FrameProfile(msg.header.stamp.to_sec(), received, time.monotonic())
        with self.lock:
            self.monitor.begin(profile)
        try:
            self.process_frame(msg, received, profile)
        except Exception as exc:
            with self.lock:
                self.cancel_locked('image_rejected:'+str(exc), emit=False)
                # An early rejection has no depth/geometry result for this frame.
                # Do not attribute a previous frame's successful measurements.
                self.monitor.finish(self.status, profile, time.monotonic(), rospy.Time.now().to_sec())
                self.emit_locked()
            raise

    def frame_fresh(self, stamp, received):
        ros_age = rospy.Time.now().to_sec()-stamp
        wall_age = time.monotonic()-received
        return (math.isfinite(ros_age) and math.isfinite(wall_age) and
                -.02 <= ros_age <= self.tracker.max_age_s and
                0 <= wall_age <= self.tracker.max_age_s)

    def process_frame(self, msg, received, profile):
        stamp = msg.header.stamp.to_sec()
        with self.lock:
            info, depth_info, depths = self.info, self.depth_info, list(self.depths)
            depth_info_msg = self.depth_info_msg
        if not self.frame_fresh(stamp, received):
            raise ValueError('stale_or_future_image')
        if not msg.header.frame_id or stamp <= 0 or msg.encoding not in ('bgr8', 'rgb8'):
            raise ValueError('invalid_image_header_or_encoding')
        with profile.stage('rgb_decode'):
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame.shape[:2] != (msg.height, msg.width):
            raise ValueError('image_dimensions_mismatch')
        with profile.stage('rgb_detection'):
            found, result = self.detector.detect(frame)
        calibration, calibration_reason, principal = False, '', None
        try:
            k, d = camera_model(info, msg.header.frame_id, msg.width, msg.height, stamp)
            principal = [float(k[0, 2]), float(k[1, 2])]
            calibration = True
        except (ValueError, TypeError) as exc:
            calibration_reason = str(exc)
        meters = None
        registration_contract = None
        depth_reason, depth_time_delta = 'registered_depth_disabled', None
        if self.use_depth and (len(found) == 1 or self.depth_candidates or self.enable_traversal):
            try:
                if not depths:
                    raise ValueError('registered_depth_missing')
                depth_msg, depth_received = min(depths, key=lambda row: abs(row[0].header.stamp.to_sec()-stamp))
                depth_time_delta = depth_msg.header.stamp.to_sec()-stamp
                if time.monotonic()-depth_received > self.tracker.max_age_s:
                    raise ValueError('depth_stream_stale')
                self.depth_source_pub.publish(depth_msg)
                if depth_info_msg is not None:
                    self.depth_source_info_pub.publish(depth_info_msg)
                with profile.stage('depth_decode'):
                    depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
                if self.registration_mode == 'software_tf':
                    with profile.stage('factory_calibration_and_tf'):
                        depth_info = self.factory_depth_info(info, depth_info, msg.header.frame_id, depth_msg.header.frame_id)
                        r, t = self.transform_provider(msg.header.frame_id, depth_msg.header.frame_id)
                    if self.astra_factory:
                        factory_checked_tf(r, t, depth_info['factory_params_snapshot'])
                    self.check_calibration_epoch(info, depth_info, r, t)
                    with profile.stage('depth_registration'):
                        meters, k, d = self.registrar.register(depth, depth_msg.encoding, depth_msg.header.stamp.to_sec(), depth_msg.header.frame_id,
                            info, depth_info, stamp, msg.header.frame_id, frame.shape, r, t)
                else:
                    self.check_calibration_epoch(info, depth_info, np.eye(3), np.zeros(3))
                    meters, k, d = matched_depth(depth, depth_msg.encoding, depth_msg.header.stamp.to_sec(), depth_msg.header.frame_id,
                        info, depth_info, stamp, msg.header.frame_id, frame.shape)
                registration_contract = dict(rgb_camera_info=info, depth_camera_info=depth_info,
                    depth_image_frame=depth_msg.header.frame_id, depth_encoding=depth_msg.encoding,
                    rotation_rgb_from_depth=r.tolist() if self.registration_mode == 'software_tf' else np.eye(3).tolist(),
                    translation_rgb_from_depth_m=t.tolist() if self.registration_mode == 'software_tf' else [0., 0., 0.],
                    accuracy_verified=False, timestamp_means_capture_time_verified=False)
                depth_reason = 'depth_projection_valid_sensor_validation_still_required'
                if depth_info and depth_info.get('metadata_repair'):
                    depth_reason += ':'+depth_info['metadata_repair']
                if self.depth_candidates:
                    with profile.stage('depth_candidates'):
                        extra = self.depth_detector.detect(meters, k, d)
                    found = fuse_candidates(found, extra)
                    result.update(depth_candidate_count=len(extra), target_count=len(found), valid=len(found) == 1,
                                  reason='' if len(found) == 1 else ('no_target' if not found else 'multiple_targets'),
                                  targets=[candidate.as_dict() for candidate in found])
            except Exception as exc:
                meters = None
                depth_reason = str(exc)
        elif self.use_depth:
            depth_reason = 'depth_waiting_for_unique_rgb_target'
        metric, candidate_geometry, pose = None, None, None
        metric_reason = depth_reason if meters is None else 'visual_target_not_unique'
        if meters is not None and len(found) == 1:
            try:
                with profile.stage('frame_plane_and_aperture'):
                    plane = estimate_plane(found[0], meters, k, d)
                    plane['opening_evidence'] = aperture_evidence(found[0], meters, k, d, plane)
                with self.lock:
                    if self.last_pose_wall is None or time.monotonic()-self.last_pose_wall > .20:
                        raise ValueError('local_pose_stale')
                    pose = self.pose.at(stamp)
                candidate_geometry = world_geometry(plane, pose, self.camera_xyz, self.camera_q)
                candidate_geometry['calibration_epoch'] = self.calibration_epoch
                if not plane['opening_evidence']['possible_opening']:
                    metric_reason = 'surface_or_foreground_not_a_clear_aperture'
                else:
                    # Perception remains useful even outside the legacy small
                    # step preview's distance/vertical-plane constraints.
                    metric = step_preview(plane, pose, self.camera_xyz, self.camera_q)
                    metric_reason = 'live_depth_geometry_preview_only'
            except (ValueError, TypeError) as exc:
                metric_reason = str(exc)
        with self.lock:
            if not self.frame_fresh(stamp, received):
                tracking = self.tracker.cancel('processing_exceeded_freshness_limit')
            else:
                tracking = self.tracker.update(found, stamp, rospy.Time.now().to_sec(), received, frame.shape, principal)
            if tracking.get('tracking_reset_reason'):
                self.metric_tracker.reset('visual_track_reset')
            # Confirm visual and metric observations together. Neither result
            # is exposed as ready until its original conditions hold.
            if tracking['reason'] in ('live_visual_tracking', 'confirming_current_target') and candidate_geometry is not None:
                metric_state = self.metric_tracker.update(candidate_geometry, rospy.Time.now().to_sec())
                if not tracking['ready'] and metric_state['ready']:
                    metric_state = dict(ready=False, reason='visual_confirmation_pending', geometry=None)
            else:
                metric_state = self.metric_tracker.reset('unique_fresh_rgbd_geometry_required')
        plan = None
        traversal_reason = 'stable_metric_geometry_required' if self.enable_traversal else 'not_requested_for_geometry_stage'
        if self.enable_traversal and metric_state['ready']:
            try:
                with profile.stage('route_planning'):
                    plan = plan_route(metric_state['geometry'], pose, self.safety, rospy.Time.now().to_sec())
                optical_r = pose['rotation'].dot(quaternion_matrix(self.camera_q))
                optical_origin = pose['position']+pose['rotation'].dot(np.asarray(self.camera_xyz))
                with profile.stage('corridor_evidence'):
                    evidence = evaluate_corridor(plan, meters, k, d, optical_r, optical_origin, stamp, rospy.Time.now().to_sec())
                plan.update(corridor=evidence, corridor_status=evidence['reason'])
                traversal_reason = 'conditional_plan_not_flight_authorization' if evidence['clear'] else evidence['reason']
            except Exception as exc:
                traversal_reason = str(exc)
                # Keep geometry, never publish an incomplete route on a failed check.
                plan = None
        with self.lock:
            if pose is not None and pose.get('epoch', 0) != self.pose.epoch:
                tracking = self.tracker.cancel('pose_reset_during_processing')
                metric_state = self.metric_tracker.reset('pose_reset_during_processing')
                plan = None
                traversal_reason = 'pose_reset_during_processing'
            if not self.frame_fresh(stamp, received):
                tracking = self.tracker.cancel('processing_exceeded_freshness_limit')
                metric_state = self.metric_tracker.reset('processing_exceeded_freshness_limit')
                plan = None
                traversal_reason = 'processing_exceeded_freshness_limit'
            self.status = dict(result, **tracking, source_stamp=stamp, optical_frame=msg.header.frame_id,
                image_width=msg.width, image_height=msg.height, calibration_valid=calibration,
                calibration_reason=calibration_reason, capture_age_s=rospy.Time.now().to_sec()-stamp,
                metric_preview=metric if tracking['ready'] else None, metric_reason=metric_reason,
                metric_geometry=metric_state['geometry'], metric_geometry_ready=metric_state['ready'],
                metric_geometry_reason=metric_state['reason'], calibration_epoch=self.calibration_epoch,
                depth_reason=depth_reason, depth_stamp_delta_s=depth_time_delta, registration_contract=registration_contract,
                traversal_plan=plan, traversal_reason=traversal_reason)
            self.monitor.finish(self.status, profile, time.monotonic(), rospy.Time.now().to_sec())
            self.emit_locked()
            direction = (tracking.get('visual_instruction') or {}).get('direction', tracking['reason'])
        with profile.stage('overlay_and_publish'):
            overlay = self.bridge.cv2_to_imgmsg(annotate(frame, found, direction+' / ADVISORY ONLY'), encoding='bgr8')
            overlay.header = copy.deepcopy(msg.header)
            self.image_pub.publish(overlay)
            self.source_pub.publish(msg)
        with self.lock:
            self.monitor.complete_cycle(profile)
        rospy.loginfo_throttle(2., 'LIVE %s | rgbd=%s | geometry=%s | plan=%s | NO FLIGHT OUTPUT',
                               direction, depth_reason, metric_state['reason'], traversal_reason)
        rospy.loginfo_throttle(2., 'PIPELINE ms=%.1f | confirmed=%s | reset=%s | source_gap=%s | receipt_gap=%s | stages_ms=%s',
                               profile.elapsed_ms(), tracking.get('confirmed_frames'), tracking.get('tracking_reset_reason'),
                               tracking.get('tracking_source_gap_s'), tracking.get('tracking_receipt_gap_s'), profile.stage_ms)

    def cancel_locked(self, reason, reset_history=True, emit=True):
        if reset_history:
            self.tracker.cancel(reason)
            self.metric_tracker.reset(reason)
        # Always revoke old proposals, even if the most recent frame was good.
        # Output expiry with a live input stream must not erase the independent
        # tracking histories: update() still enforces the original .40s gaps.
        self.status = dict(ready=False, reason=reason, confirmed_frames=0,
                           tracking_reset_reason=reason if reset_history else 'history_retained_pending_fresh_frame',
                           visual_instruction=None, source_stamp=None, target_count=None,
                           depth_reason=None, depth_stamp_delta_s=None,
                           metric_preview=None, metric_geometry=None, metric_geometry_ready=False,
                           metric_geometry_reason=reason, metric_reason=reason,
                           traversal_plan=None, traversal_reason=reason)
        if emit:
            self.emit_locked()

    def watchdog_once(self):
        with self.lock:
            runtime = self.monitor.snapshot(time.monotonic(), rospy.Time.now().to_sec(), self.tracker.max_age_s)
            rgb = runtime['streams']['rgb']
            if rgb['received_count'] == 0:
                self.cancel_locked('waiting_for_first_image')
            elif not rgb['receipt_fresh']:
                self.cancel_locked('image_stream_stale')
            elif not runtime['result_fresh'] and (self.status.get('ready') or
                    self.status.get('metric_geometry') is not None or
                    self.status.get('metric_preview') is not None or
                    self.status.get('traversal_plan') is not None):
                self.cancel_locked('processing_result_stale', reset_history=False)
            else:
                # Keep no-target/rejected/confirming reasons, plus current
                # stream and worker evidence, instead of labeling all as loss.
                self.emit_locked()

    def worker(self):
        while not rospy.is_shutdown():
            start = time.monotonic()
            with self.lock:
                row, self.latest = self.latest, None
            if row is not None:
                try:
                    self.process(*row)
                except Exception as exc:
                    rospy.logwarn_throttle(2., 'LIVE image rejected: %s', str(exc))
            time.sleep(max(.005, 1./self.fps-(time.monotonic()-start)))

    def watchdog(self):
        while not rospy.is_shutdown():
            self.watchdog_once()
            time.sleep(.10)


if __name__ == '__main__':
    rospy.init_node('window_live_vision')
    LiveNode()
    rospy.spin()
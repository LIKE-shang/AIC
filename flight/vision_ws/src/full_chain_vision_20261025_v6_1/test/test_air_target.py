#!/usr/bin/env python3
"""Synthetic/ROS-free contract tests. NEVER a physical laser acceptance test."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest
import cv2
import numpy as np
import yaml
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from robot_vision_test.air_target import (air_parameters, detect_air_target, assess_aim, detect_spot,
                                          assess_spot, draw_air_overlay, physical_radii, jsonable)
from robot_vision_test.safety import ContinuousHitGate, FrameFreshness, ContentFreshness


def synthetic(center=(320,240), shape=(480,640), missing=(), ratio=1.0, angle=0,
              diameters=(.35,.24,.12), definition='centerline', spot=None):
    frame=np.full((*shape,3),235,np.uint8)
    adjustment={'outer':-.008,'centerline':0,'inner':.008}[definition]
    for i,d in enumerate(diameters):
        if i in missing: continue
        radius=(d+adjustment)/2
        for r,color in [(radius+.004,(20,20,20)),(radius-.004,(235,235,235))]:
            cv2.ellipse(frame,center,(round(r*800),round(r*800*ratio)),angle,0,360,color,-1)
    if spot is not None: cv2.circle(frame,tuple(spot),2,(0,0,255),-1)
    return frame


def calibrated_params(frame=None, **overrides):
    """Fabricated measurements for tests ONLY. Not deployable calibration."""
    frame=synthetic() if frame is None else frame
    p=air_parameters(dict(dimensions_confirmed=True,diameter_definition='centerline',
                         operating_envelope_confirmed=True,working_distance_m=1.5,
                         live_camera_verified=True,prediction_only_acknowledged=True))
    d,_=detect_air_target(frame,p)
    outer=d['targets'][0]['rings'][0] if d['targets'] else dict(axes=np.array([140.,140.]),angle=0.)
    p['calibration'].update(confirmed=True,calibration_id='SYNTHETIC_NOT_HARDWARE',camera_frame_id='camera_optical',
                            image_width=frame.shape[1],image_height=frame.shape[0],offset_px=[.5,.5],
                            distance_m=1.5,outer_major_px=float(outer['axes'][0]),axis_ratio=float(outer['axes'][1]/outer['axes'][0]),
                            angle_deg=float(outer['angle']),spot_radius_m=.004,error_bound_m=.005)
    p.update(overrides)
    return p


class TripleRingTests(unittest.TestCase):
    def test_correct_ratio_and_boundary_dedup(self):
        d,_=detect_air_target(synthetic())
        self.assertTrue(d['valid'],d['reason']); self.assertEqual(len(d['rings']),3)
        self.assertEqual(len(d['targets']),1)
        self.assertLess(d['targets'][0]['ratio_error'],.01)
    def test_each_missing_ring_and_single_ring(self):
        for missing in [(0,),(1,),(2,),(0,1),(0,2),(1,2)]:
            with self.subTest(missing=missing): self.assertFalse(detect_air_target(synthetic(missing=missing))[0]['valid'])
    def test_incorrect_ratio(self):
        self.assertFalse(detect_air_target(synthetic(diameters=(.35,.28,.16)))[0]['valid'])
    def test_two_targets(self):
        frame=np.minimum(synthetic((200,240),(480,880)),synthetic((650,240),(480,880)))
        d,_=detect_air_target(frame); self.assertFalse(d['valid']); self.assertEqual(len(d['targets']),2)
    def test_extra_ring_or_partial_second_target_veto(self):
        frame=synthetic((200,240),(480,880)); cv2.circle(frame,(650,240),60,(20,20,20),6)
        self.assertFalse(detect_air_target(frame)[0]['valid'])
    def test_elliptic_target_rotated(self):
        for ratio,angle in [(.8,0),(.65,30),(.6,75)]:
            d,_=detect_air_target(synthetic(ratio=ratio,angle=angle))
            self.assertTrue(d['valid'],(ratio,angle,d['reason']))
    def test_mild_projective_and_extreme_pose_rejected(self):
        src=np.float32([[0,0],[639,0],[639,479],[0,479]])
        dst=np.float32([[35,30],[590,12],[620,450],[20,470]])
        warped=cv2.warpPerspective(synthetic(),cv2.getPerspectiveTransform(src,dst),(640,480),borderValue=(235,)*3)
        d,_=detect_air_target(warped); self.assertTrue(d['valid'],d['reason'])
        self.assertFalse(detect_air_target(synthetic(ratio=.35))[0]['valid'])
    def test_occlusion_and_glare_fail_closed(self):
        for color in [(235,)*3,(255,)*3,(20,)*3]:
            image=synthetic(); cv2.rectangle(image,(310,55),(335,245),color,-1)
            self.assertFalse(detect_air_target(image)[0]['valid'])
    def test_inner_obstruction_without_breaking_rings_rejected(self):
        frame=synthetic(); cv2.rectangle(frame,(305,225),(335,255),(20,20,20),-1)
        self.assertFalse(detect_air_target(frame)[0]['valid'])
    def test_border_and_low_contrast(self):
        self.assertFalse(detect_air_target(synthetic(center=(80,240)))[0]['valid'])
        self.assertFalse(detect_air_target(np.full((480,640,3),200,np.uint8))[0]['valid'])
    def test_noise_and_reflections_do_not_manufacture_target(self):
        rng=np.random.default_rng(107)
        image=np.full((480,640,3),220,np.uint8)
        for x,y in rng.integers([5,5],[635,475],size=(100,2)): cv2.circle(image,(int(x),int(y)),3,(255,)*3,-1)
        self.assertFalse(detect_air_target(image)[0]['valid'])
        image=synthetic().astype(np.int16)+rng.normal(0,3,(480,640,3)).astype(np.int16)
        self.assertTrue(detect_air_target(np.clip(image,0,255).astype(np.uint8))[0]['valid'])
    def test_all_diameter_definitions(self):
        for definition,free in [('outer',.052),('centerline',.056),('inner',.060)]:
            p=air_parameters(dict(diameter_definition=definition,dimensions_confirmed=True))
            self.assertAlmostEqual(physical_radii(p)[1],free)
            self.assertTrue(detect_air_target(synthetic(definition=definition),p)[0]['valid'])
    def test_line_width_is_not_diameter(self):
        with self.assertRaises(ValueError): air_parameters(dict(diameters_m=[.008]*3))
        with self.assertRaises(ValueError): air_parameters(dict(dimensions_confirmed=True))


class CalibrationAndSpotTests(unittest.TestCase):
    def setUp(self):
        self.frame=synthetic(); self.p=calibrated_params(); self.t=detect_air_target(self.frame,self.p)[0]['targets'][0]
    def aim(self,p=None,frame_id='camera_optical',shape=None):
        return assess_aim(self.t,shape or self.frame.shape,frame_id,p or self.p)
    def test_default_not_calibrated_and_unknown_definition_block(self):
        self.assertFalse(assess_aim(self.t,self.frame.shape,'camera_optical')['aim_valid'])
        p=copy.deepcopy(self.p); p['calibration']['confirmed']=False
        self.assertFalse(self.aim(p)['aim_valid'])
    def test_pixel_offset_not_image_center_assumption(self):
        self.assertTrue(self.aim()['aim_valid'])
        p=copy.deepcopy(self.p); p['calibration']['offset_px']=[70,0]
        a=self.aim(p); self.assertFalse(a['aim_valid']); self.assertEqual(a['predicted_point'],[389.5,239.5])
    def test_camera_resolution_distance_scale_and_pose_envelope(self):
        self.assertFalse(self.aim(frame_id='wrong')['calibration_valid'])
        self.assertFalse(self.aim(shape=(720,1280,3))['calibration_valid'])
        for patch in [dict(working_distance_m=1.8),dict(operating_envelope_confirmed=False)]:
            p=copy.deepcopy(self.p); p.update(patch); self.assertFalse(self.aim(p)['calibration_valid'])
        for k,v in [('outer_major_px',200),('axis_ratio',.7)]:
            p=copy.deepcopy(self.p); p['calibration'][k]=v; self.assertFalse(self.aim(p)['calibration_valid'])
    def test_boundary_margin_not_nominal_six_cm(self):
        a=self.aim(); self.assertAlmostEqual(a['effective_radius_m'],.042)
        p=copy.deepcopy(self.p); p['calibration']['offset_px']=[39,.5]
        self.assertFalse(self.aim(p)['aim_valid']) # nominal inner radius ~48px; effective is smaller
        p['calibration']['error_bound_m']=.06
        self.assertEqual(self.aim(p)['reason'],'no_safe_inner_area')
    def test_non_640_resolution(self):
        frame=synthetic((400,300),(600,800)); p=calibrated_params(frame)
        t=detect_air_target(frame,p)[0]['targets'][0]
        a=assess_aim(t,frame.shape,'camera_optical',p)
        self.assertTrue(a['aim_valid']); self.assertEqual(a['predicted_point'],[400,300])
    def test_spot_missing_actual_and_false_reflections(self):
        self.p['spot'].update(color='red',validated=True)
        a=self.aim()
        self.assertFalse(detect_spot(self.frame,self.p)['valid'])
        visible=synthetic(spot=(320,240)); spot=detect_spot(visible,self.p)
        self.assertTrue(assess_spot(spot,a,self.t,self.p))
        self.assertFalse(assess_spot(detect_spot(synthetic(spot=(380,240)),self.p),a,self.t,self.p))
        cv2.circle(visible,(350,240),2,(0,0,255),-1)
        self.assertFalse(detect_spot(visible,self.p)['valid'])
        white=self.frame.copy(); cv2.circle(white,(320,240),3,(255,255,255),-1)
        self.assertFalse(detect_spot(white,self.p)['valid'])
    def test_unvalidated_spot_never_claimed(self):
        p=copy.deepcopy(self.p); p['spot']['color']='red'
        self.assertFalse(assess_spot(detect_spot(synthetic(spot=(320,240)),p),self.aim(),self.t,p))
    def test_invalid_parameters(self):
        for patch in [dict(ratio_tolerance=float('nan')),dict(min_axis_ratio=0),dict(dimensions_confirmed='true')]:
            with self.assertRaises(ValueError): air_parameters(patch)


class ContinuousTimingTests(unittest.TestCase):
    def gate(self,mode='prediction_only'):
        return ContinuousHitGate(3,.35,3,mode,.3,5)
    def test_three_seconds_continuous_then_off(self):
        g=self.gate()
        for i in range(33): g.observe(True,10+i*.1,100+i*.1,acknowledged=True)
        self.assertFalse(g.on); self.assertTrue(g.latched); self.assertEqual(g.result,'predicted_only_3s')
        self.assertEqual(g.elapsed,0)
    def test_no_ack_no_hit_credit(self):
        g=self.gate()
        for i in range(35): g.observe(True,10+i*.1,100+i*.1)
        self.assertTrue(g.on); self.assertEqual(g.elapsed,0); self.assertEqual(g.result,'none')
    def test_interruptions_never_add_up(self):
        for reason in ['target_changed','target_missing','spot_outside','occluded','expired']:
            g=self.gate()
            for i in range(15): g.observe(True,10+i*.1,100+i*.1,acknowledged=True)
            g.observe(False,11.5,101.5,reason,True)
            self.assertFalse(g.on); self.assertEqual(g.elapsed,0)
            for i in range(16,60): g.observe(True,10+i*.1,100+i*.1,acknowledged=True)
            self.assertEqual(g.result,'none'); self.assertFalse(g.on)
    def test_acquisition_grace_only_before_first_observation(self):
        g=self.gate('camera')
        for i in range(3): g.observe(True,10+i*.1,100+i*.1,acknowledged=True)
        self.assertTrue(g.on); self.assertEqual(g.elapsed,0)
        g.observe(True,10.3,100.3,acknowledged=True,observed_hit=True)
        g.observe(True,10.4,100.4,acknowledged=True,observed_hit=False)
        self.assertFalse(g.on); self.assertTrue(g.latched)
    def test_missing_spot_times_out_no_fake_feedback(self):
        g=self.gate('camera')
        for i in range(8): g.observe(True,10+i*.1,100+i*.1,acknowledged=True)
        self.assertFalse(g.on); self.assertEqual(g.result,'none')
    def test_camera_observed_three_seconds(self):
        g=self.gate('camera')
        for i in range(34): g.observe(True,10+i*.1,100+i*.1,acknowledged=True,observed_hit=True)
        self.assertEqual(g.result,'camera_observed_3s'); self.assertFalse(g.on)
    def test_watchdog_stream_loss_and_max_exposure(self):
        g=self.gate()
        for i in range(3): g.observe(True,10+i*.1,100+i*.1)
        self.assertTrue(g.watchdog(10.56)); self.assertFalse(g.on)
        g=self.gate()
        for i in range(54): g.observe(True,10+i*.1,100+i*.1)
        self.assertFalse(g.on); self.assertTrue(g.latched)
    def test_replayed_backward_and_capture_gap(self):
        for wall,stamp in [(10.3,100.2),(10.3,100.1),(10.3,101),(11,100.3)]:
            g=self.gate()
            for i in range(3): g.observe(True,10+i*.1,100+i*.1,acknowledged=True)
            g.observe(True,wall,stamp,acknowledged=True)
            self.assertFalse(g.on); self.assertTrue(g.latched)
    def test_fast_replay_does_not_earn_wall_time(self):
        g=self.gate()
        for i in range(35): g.observe(True,10+i*.001,100+i*.1,acknowledged=True)
        self.assertLess(g.elapsed,.04); self.assertEqual(g.result,'none')
    def test_timestamp_high_water_mark(self):
        f=FrameFreshness(.35); f.check(100,100)
        for stamp in (99.9,100,99.95):
            with self.assertRaises(ValueError): f.check(stamp,100.1)
        for stamp,now in [(100.1,100.9),(0,100),(102,101)]:
            with self.assertRaises(ValueError): f.check(stamp,now)


class ReplayAndCalibrationToolsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import importlib.util
        def load(name):
            spec=importlib.util.spec_from_file_location(name,ROOT/'tools'/(name+'.py'))
            module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module); return module
        cls.replay=load('replay_air_target'); cls.calibrator=load('calibrate_air_target')
    def measurements(self):
        return dict(calibration_id='SYNTHETIC_TOOL_TEST',camera_frame_id='camera_optical',camera_settings_record='synthetic',
                    image_width=640,image_height=480,distance_m=1.5,distance_tolerance_m=.02,pixels_per_m_min=800,
                    measurement_error_px=.5,additional_error_m=.002,
                    samples=[dict(x_px=340+x,y_px=235,spot_radius_px=2,outer_major_px=140,axis_ratio=1,angle_deg=0,distance_m=1.5)
                             for x in [-1,0,1,-.5,.5]])
    def test_calibration_derivation_never_auto_confirms(self):
        p,report=self.calibrator.compute(self.measurements())
        np.testing.assert_allclose(p['calibration']['offset_px'],[20.5,-4.5])
        self.assertAlmostEqual(p['calibration']['error_bound_m'],1.5/800+.002)
        self.assertFalse(p['calibration']['confirmed']); self.assertFalse(p['live_camera_verified'])
        self.assertFalse(report['physical_measurements_verified'])
    def test_calibration_mixed_distances_refused(self):
        m=self.measurements(); m['samples'][-1]['distance_m']=2
        with self.assertRaises(ValueError): self.calibrator.compute(m)
        m=self.measurements(); m['samples']=m['samples'][:2]
        with self.assertRaises(ValueError): self.calibrator.compute(m)
    def test_manifest_preserves_and_rejects_duplicate_stale_timestamps(self):
        import tempfile
        with tempfile.TemporaryDirectory(dir=str(ROOT/'test/artifacts')) as td:
            folder=Path(td); cv2.imwrite(str(folder/'raw.png'),synthetic())
            records=[dict(file='raw.png',capture_stamp=s,received_stamp=t,camera_frame_id='camera_optical')
                     for s,t in [(100,100.1),(100,100.2),(100.1,101)]]
            (folder/'frames.json').write_text(json.dumps(records),encoding='utf-8')
            report=self.replay.main(['--manifest',str(folder/'frames.json'),'--output',str(folder/'result')])
            self.assertEqual([r['capture_valid'] for r in report],[True,False,False])
            self.assertTrue(all(not r['request_on'] and not r['physical_hit_verified'] for r in report))
    def test_still_image_labels_synthetic_clock(self):
        import tempfile
        with tempfile.TemporaryDirectory(dir=str(ROOT/'test/artifacts')) as td:
            folder=Path(td); cv2.imwrite(str(folder/'raw.png'),synthetic())
            report=self.replay.main(['--input',str(folder/'raw.png'),'--output',str(folder/'result')])
            self.assertTrue(report[0]['synthetic_clock']); self.assertTrue(report[0]['detection']['valid'])
    def test_frozen_content_with_new_timestamps(self):
        f=ContentFreshness(.25); f.check(b'image',10); f.check(b'image',10.2)
        with self.assertRaises(ValueError): f.check(b'image',10.26)
        f.check(b'new image',10.3)
    def test_template_cannot_be_used_as_measurement(self):
        m=json.loads((ROOT/'config/air_target_calibration_samples.template.json').read_text(encoding='utf-8-sig'))
        with self.assertRaises(ValueError): self.calibrator.compute(m)


class HardwareIsolationStaticTests(unittest.TestCase):
    def test_bridge_guards_and_hash_before_dlopen(self):
        s=(ROOT/'src/laser_test_bridge.cpp').read_text(encoding='utf-8-sig')
        for token in ['msg->target_valid','msg->aim_valid','msg->calibration_valid','SHA256_Final',
                      'independent_maximum_exposure','physical_isolation_confirmed','independent_cutoff_confirmed',
                      'output_mapping_confirmed','response.success','result.result!=0','start(false)',
                      'shutdownOff','service_call_timeout']:
            self.assertIn(token,s)
        self.assertLess(s.index('SHA256_Final'),s.index('handle_ = dlopen'))
        self.assertNotIn('fault_=false;',s) # declaration includes other variables, never clears a latched fault
        self.assertNotIn('const bool stable=desired_',s)
    def test_safe_templates(self):
        p=yaml.safe_load((ROOT/'config/circle_laser_test.yaml').read_text(encoding='utf-8-sig'))
        p=air_parameters(p)
        self.assertFalse(p['dimensions_confirmed']); self.assertFalse(p['calibration']['confirmed'])
        h=yaml.safe_load((ROOT/'config/laser_hardware.yaml').read_text(encoding='utf-8-sig'))
        for key in ['hardware_wiring_confirmed','physical_isolation_confirmed','independent_cutoff_confirmed','output_mapping_confirmed']:
            self.assertIs(h[key],False)
    def test_all_new_sources_parse_python38(self):
        import ast
        for folder in ['src/robot_vision_test','scripts','tools']:
            for p in (ROOT/folder).glob('*air*py'):
                ast.parse(p.read_text(encoding='utf-8-sig'),feature_version=(3,8))


def save_fixtures():
    out=ROOT/'test/artifacts/air_target'; out.mkdir(parents=True,exist_ok=True)
    for name,image in [('three_rings',synthetic()),('ellipse',synthetic(ratio=.65,angle=30)),
                       ('missing_ring',synthetic(missing=(1,))),('observed_spot',synthetic(spot=(320,240)))]:
        cv2.imwrite(str(out/(name+'.png')),image)
    image=synthetic(); p=calibrated_params(image); d,mask=detect_air_target(image,p)
    a=assess_aim(d['targets'][0],image.shape,'camera_optical',p)
    cv2.imwrite(str(out/'annotated.png'),draw_air_overlay(image.copy(),d,a,dict(candidates=[]),['SYNTHETIC ONLY - NOT physical acceptance']))
    cv2.imwrite(str(out/'mask.png'),mask)
    p['live_camera_verified']=False; p['prediction_only_acknowledged']=False
    (out/'synthetic_calibration.yaml').write_text(yaml.safe_dump(p,sort_keys=False),encoding='utf-8')
    p['feedback_mode']='camera'; p['spot'].update(color='red',validated=True)
    (out/'synthetic_camera_feedback.yaml').write_text(yaml.safe_dump(p,sort_keys=False),encoding='utf-8')
    (out/'geometry.json').write_text(json.dumps(jsonable(dict(detection=d,aim=a)),indent=2),encoding='utf-8')

if __name__=='__main__':
    save_fixtures(); unittest.main(verbosity=2)

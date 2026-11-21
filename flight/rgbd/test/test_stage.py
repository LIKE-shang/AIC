"""Tests of the competition milestone path, not flight control."""
import copy
import json
import types
import unittest
from unittest.mock import patch
import numpy as np
from pathlib import Path
import xml.etree.ElementTree as ET
from test_node import node, image, MODULE, Clock
from test_guidance import camera
from scenes import scene
from window_live_vision.stage import stage_summary, display_lines, evidence_json
ROOT=Path(__file__).resolve().parents[1]


class StageSummaryTests(unittest.TestCase):
    def sample(self):
        rows={
          '/mavros/state':dict(stamp=10.,connected=True,armed=False,mode='MANUAL'),
          '/mavros/extended_state':dict(stamp=10.,landed_state=1),
          '/window_pass_competition/status':dict(real_output_state='FORWARDING_CONTINUOUS_POSE',real_output_rate_hz=50.,real_fault='',horizontal_flags_valid=True),
          '/window_live/status':dict(version='test',ready=True,target_count=1,confirmed_frames=5,source_stamp=10.,
              metric_geometry_ready=True,metric_geometry={'opening_evidence':{'possible_opening':True}},
              metric_preview=dict(planned_lateral_step_m=.1,observed_opening_width_m=1.1,observed_opening_height_m=.8),
              runtime=dict(result_fresh=True,result_age_wall_s=.05,processed_rate_hz=5.,
                           streams={key:dict(receipt_fresh=True,callback_rate_hz=30.) for key in ('rgb','depth')}))}
        return rows,{key:.05 for key in rows}

    def test_ready_is_geometry_milestone_without_flight_permission(self):
        rows,ages=self.sample();result=stage_summary(rows,ages,10.1)
        self.assertEqual(result['code'],'GEOMETRY_READY');self.assertTrue(result['geometry_stage_ready'])
        self.assertFalse(result['actual_output_permitted']);self.assertFalse(result['small_step_completed'])
        self.assertEqual(result['preview']['lateral_step_cm'],10.)
        self.assertEqual(len(display_lines(result)),2)

    def test_no_full_traversal_budget_needed_for_geometry_stage(self):
        rows,ages=self.sample()
        rows['/window_live/status']['traversal_reason']='missing_safety_bounds:geofence'
        self.assertTrue(stage_summary(rows,ages,10.1)['geometry_stage_ready'])

    def test_old_status_cannot_present_old_ready(self):
        for age,now in ((1.1,10.1),(.1,10.6),(.49,10.1)):
            rows,ages=self.sample();ages['/window_live/status']=age
            self.assertFalse(stage_summary(rows,ages,now)['geometry_stage_ready'])

    def test_missing_telemetry_and_depth_show_one_action(self):
        for key,expected in (('/mavros/state','WAIT_FCU'),('/window_pass_competition/status','WAIT_LOCALIZATION'),('/window_live/status','WAIT_VISION')):
            rows,ages=self.sample();del rows[key];del ages[key]
            result=stage_summary(rows,ages,10.1);self.assertEqual(result['code'],expected)
        rows,ages=self.sample();rows['/window_live/status']['runtime']['streams']['depth']['receipt_fresh']=False
        self.assertEqual(stage_summary(rows,ages,10.1)['code'],'WAIT_DEPTH')

    def test_armed_message_ends_ground_only_panel(self):
        rows,ages=self.sample();rows['/mavros/state']['armed']=True
        result=stage_summary(rows,ages,10.1);self.assertEqual(result['code'],'LAND_AND_REVIEW')
        self.assertFalse(result['geometry_stage_ready'])

    def test_real_rejection_reason_is_preserved(self):
        rows,ages=self.sample();live=rows['/window_live/status']
        live.update(ready=False,reason='confirming_current_target',tracking_reset_reason='image_receipt_gap_exceeded')
        result=stage_summary(rows,ages,10.1)
        self.assertEqual(result['reason'],'image_receipt_gap_exceeded')
        live.update(ready=True,metric_geometry_ready=False,metric_reason='four_borders_do_not_share_depth_plane')
        self.assertEqual(stage_summary(rows,ages,10.1)['reason'],'four_borders_do_not_share_depth_plane')

    def test_surface_or_invalid_metric_never_ready(self):
        rows,ages=self.sample();live=rows['/window_live/status']
        live['metric_geometry']['opening_evidence']['possible_opening']=False
        self.assertFalse(stage_summary(rows,ages,10.1)['geometry_stage_ready'])
        live['metric_geometry']['opening_evidence']['possible_opening']=True
        live['metric_preview']['planned_lateral_step_m']=.11
        self.assertFalse(stage_summary(rows,ages,10.1)['geometry_stage_ready'])

    def test_nan_evidence_is_saved_not_hidden_or_crashed(self):
        result=evidence_json({'rate':float('nan'),'values':[float('inf')]})
        json.dumps(result,allow_nan=False)
        self.assertEqual(result['rate'],{'nonfinite_value':'nan'})


class StagePipelineTests(unittest.TestCase):
    def test_preview_path_never_calls_full_traversal_checks(self):
        n=node();n.enable_traversal=False;n.depth_candidates=False;n.use_depth=True
        n.registration_mode='strict_registered';n.info=camera();n.depth_info=camera()
        frame,_=scene(center=(370,240));reached=[]
        with patch.object(MODULE,'plan_route',side_effect=AssertionError('not requested')),patch.object(MODULE,'evaluate_corridor',side_effect=AssertionError('not requested')):
            for i in range(5):
                stamp=10.+.2*i;wall=20.+.2*i;Clock.now_value=stamp
                depth=np.full((480,640),2000,np.uint16);depth[140:340,230:510]=4000
                n.depths=[(image(depth,stamp,'16UC1'),wall)]
                n.pose.add(stamp,[0,0,-1.6],[0,0,0,1],'map');n.last_pose_wall=wall
                with patch.object(MODULE.time,'monotonic',return_value=wall):n.process(image(frame,stamp),wall)
                reached.append(n.status['metric_geometry_ready'])
        self.assertEqual(reached,[False,False,False,False,True])
        self.assertIsNotNone(n.status['metric_preview']);self.assertIsNone(n.status['traversal_plan'])
        self.assertEqual(n.status['traversal_reason'],'not_requested_for_geometry_stage')
        self.assertFalse(n.status['actual_output_permitted'])

    def test_no_target_skips_registration_but_never_returns_geometry(self):
        n=node();n.enable_traversal=False;n.depth_candidates=False;n.use_depth=True;Clock.now_value=10.
        with patch.object(n.registrar,'register',side_effect=AssertionError('no target')),patch.object(MODULE.time,'monotonic',return_value=20.):
            n.process(image(np.full((480,640,3),140,np.uint8)),20.)
        self.assertEqual(n.status['depth_reason'],'depth_waiting_for_unique_rgb_target')
        self.assertFalse(n.status['ready']);self.assertIsNone(n.status['metric_geometry'])

    def test_factory_contract_uses_fresh_async_parameter_cache(self):
        n=node();n.astra_factory=True;n.factory_wall=20.;n.factory_align=False
        n.factory_params={'success':True};n.factory_thread=types.SimpleNamespace(is_alive=lambda:True)
        with patch.object(MODULE,'factory_checked_raw_info',return_value={}) as checked,patch.object(MODULE.time,'monotonic',return_value=20.1):
            result=n.factory_depth_info({}, {}, 'rgb', 'depth')
        self.assertFalse(checked.call_args.args[5]);self.assertEqual(result['factory_params_snapshot'],{'success':True})
        with patch.object(MODULE.time,'monotonic',return_value=23.1),self.assertRaisesRegex(ValueError,'fresh_factory'):
            n.factory_depth_info({}, {}, 'rgb', 'depth')

    def test_competition_launch_keeps_base_and_flight_out_of_restart_group(self):
        launch=ET.parse(ROOT/'launch/competition.launch').getroot()
        self.assertEqual(len(launch.findall('include')),1)
        args={item.get('name'):item.get('value') for item in launch.find('include').findall('arg')}
        self.assertEqual(args['enable_traversal_planning'],'false')
        self.assertEqual(args['depth_candidates'],'false');self.assertEqual(args['record'],'false')
        self.assertEqual(args['depth_height'],'400')
        self.assertEqual([item.get('type') for item in launch.findall('node')],['stage_console.py'])
        self.assertNotIn('base.launch',(ROOT/'launch/competition.launch').read_text(encoding='utf-8-sig'))

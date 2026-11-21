import copy,math,unittest
from test_mission import Sim
from mission import Mission
from live_geometry import AirborneController,validate_metric,flight_blockers


def status(s,stamp=None):
    center=s.f.cfg['window_center_local_m'];x,y,z=center
    return dict(ready=True,metric_geometry_ready=True,target_count=1,calibration_valid=True,
        optical_frame='camera_optical',source_stamp=s.f.ros if stamp is None else stamp,
        runtime={'result_fresh':True},metric_geometry=dict(center_enu_m=center,
        corners_enu_m=[[x,y+.525,z+.4],[x,y-.525,z+.4],[x,y-.525,z-.4],[x,y+.525,z-.4]],
        normal_enu=[1.,0.,0.],source_stamp=s.f.ros if stamp is None else stamp,
        local_frame_id='map',pose_epoch=0,calibration_epoch=0,
        plane_rms_m=.002,depth_inlier_fraction=.95,confirmation_frames=5,
        temporal_corner_scatter_m=.003,metric_source='live_rgbd_four_frame_borders',
        opening_evidence={'possible_opening':True,'free_corridor_verified':False}))


class DynamicSim(Sim):
    def __init__(self,target=True):
        super().__init__(target)
        cfg=copy.deepcopy(self.m.core.c)
        cfg.update(window_center_local_m=[],geometry_verified=False,opening_geometry_verified=False)
        self.m=Mission(cfg,self.options,airborne_geometry=True)
        self.f.ctrl=self.m.core;self.dynamic=True;self.frames=0
        for _ in range(30):self.update()
    def update(self,follow=True):
        super().update(follow)
        if getattr(self,'dynamic',False):
            self.frames+=1
            if self.frames%4==0:
                value=status(self) if self.target else {'ready':False,'metric_geometry_ready':False}
                self.m.core.ingest_metric(value,self.f.wall)


class AirborneGeometry(unittest.TestCase):
    def test_takeoff_without_ground_target_then_measure_and_pass(self):
        s=DynamicSim(False);s.start();s.until('SEARCH')
        self.assertEqual(s.m.core.c['window_center_local_m'],[])
        self.assertFalse(s.m.core.live_bound)
        s.target=True;s.until('DONE')
        self.assertTrue(s.m.core.live_bound)
        self.assertEqual(s.actions,['OFFBOARD','ARM','LAND'])
    def test_no_airborne_detection_times_out_and_lands(self):
        s=DynamicSim(False);s.start();s.until('DONE')
        self.assertFalse(s.m.core.live_bound)
        self.assertNotIn('PASS',[x['new'] for x in s.m.events])
    def test_ground_metric_not_accepted_as_airborne(self):
        s=DynamicSim();c=s.m.core
        c.ingest_metric(status(s),s.f.wall)
        self.assertFalse(c.acquire(s.f.ros,s.f.wall,s.f.ros+.1))
        self.assertEqual(c.c['window_center_local_m'],[])
    def test_preexisting_window_coordinates_refused(self):
        s=Sim()
        with self.assertRaisesRegex(ValueError,'presurveyed'):AirborneController(s.m.core.c)
    def test_only_window_specific_gates_are_deferred(self):
        s=DynamicSim();cfg=dict(s.m.core.c,position_uncertainty_m=0)
        self.assertIn('missing:position_uncertainty_m',flight_blockers(cfg))
        self.assertNotIn('missing:window_center_local_m',flight_blockers(cfg))
        with self.assertRaises(ValueError):AirborneController(cfg)
    def test_stale_depth_result_cannot_authorize_pass(self):
        s=DynamicSim();g=status(s,s.f.ros-2)
        with self.assertRaisesRegex(ValueError,'fresh_after_takeoff'):
            validate_metric(g,s.m.core.c,s.f.ros,s.f.wall,s.f.wall,s.f.ros-3)
    def test_invalid_depth_geometry_rejected(self):
        s=DynamicSim()
        cases=[('plane_rms_m',.1),('depth_inlier_fraction',.1),('center_enu_m',[math.nan,0,1]),
               ('normal_enu',[0,0,1]),('local_frame_id','old_map'),('confirmation_frames',1),
               ('opening_evidence',{'possible_opening':False})]
        for key,value in cases:
            g=status(s);g['metric_geometry'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):
                validate_metric(g,s.m.core.c,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1)
    def test_epoch_change_during_alignment_lands_without_pass(self):
        s=DynamicSim();s.start();s.until('ALIGN')
        g=status(s);g['metric_geometry']['pose_epoch']=1
        s.m.core.ingest_metric(g,s.f.wall)
        out=s.m.tick(s.inputs())
        self.assertEqual(out['action'],'LAND');self.assertNotIn('PASS',[x['new'] for x in s.m.events])
    def test_geometry_loss_after_commit_does_not_retarget(self):
        s=DynamicSim();s.start();s.until('PASS');s.target=False
        s.until('DONE')
        self.assertEqual(s.actions.count('LAND'),1)
    def test_no_automatic_permission_from_possible_opening(self):
        s=DynamicSim();cfg=dict(s.m.core.c,corridor_verified=False)
        with self.assertRaisesRegex(ValueError,'corridor_verified'):AirborneController(cfg)

if __name__=='__main__':unittest.main(verbosity=2)

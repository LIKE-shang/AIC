import unittest,math,json
from pathlib import Path
from test_alignment_only import AlignOnlySim
from test_mission import Sim
from window_pass.core import Controller
from window_pass.geometry import norm,rotate,add
from envelope_fence import envelope_inside,envelope_margins

class EnvelopeFence(unittest.TestCase):
    def config(self):
        # Reproduce the refused trial geometry; no dependency on local logs.
        c=dict(AlignOnlySim().m.core.c)
        c.update(geofence_min_m=[-.5635349389131892,-3.0701414075856164,-.45114782159202876],
                 geofence_max_m=[4.385110364182243,.9372361448268796,1.6947032697685063],
                 position_uncertainty_m=.05,clearance_margin_m=.04)
        return c
    def test_logged_pose_full_box_fits_but_sphere_does_not(self):
        c=self.config();p=(.0002440364,-.0285918396,-.0572055243+.9);q=(0,0,math.sin(.060563804/2),math.cos(.060563804/2))
        self.assertTrue(envelope_inside(c,p,q));self.assertFalse(Controller(dict(c,enable_motion=False)).inside(p))
    def test_each_face_rejects_actual_box_violation(self):
        c=self.config();q=(0,0,0,1);center=[(a+b)/2 for a,b in zip(c['geofence_min_m'],c['geofence_max_m'])]
        for axis in range(3):
            for side in (-1,1):
                p=list(center);p[axis]=c['geofence_min_m'][axis] if side<0 else c['geofence_max_m'][axis]
                with self.subTest(axis=axis,side=side):self.assertFalse(envelope_inside(c,p,q))
    def test_rotated_corners_not_unrotated_box(self):
        s=AlignOnlySim();c=dict(s.m.core.c,geofence_min_m=[-.43,-.43,0],geofence_max_m=[.43,.43,2])
        self.assertTrue(envelope_inside(c,(0,0,1),(0,0,0,1)))
        self.assertFalse(envelope_inside(c,(0,0,1),(0,0,math.sin(math.pi/8),math.cos(math.pi/8))))
    def test_alignment_runtime_and_startup_use_same_model(self):
        s=AlignOnlySim();s.m.core.c.update(self.config());s.m.core.c['enable_motion']=True
        s.f.p=(.0002440364,-.0285918396,-.0572055243)
        s.m.core.pose=None
        for _ in range(30):s.update()
        s.start();self.assertEqual(s.m.phase,'PRESTREAM')
        self.assertTrue(all(s.out['startup_checks'].values()))
        self.assertTrue(s.m.core.inside((s.f.p[0],s.f.p[1],s.m.target_z)))
    def test_full_traverse_keeps_spherical_model(self):
        s=Sim();p=(0,0,1);c=s.m.core.c
        radius=norm(c['vehicle_size_m'])/2+norm(c['envelope_center_body_m'])+c['clearance_margin_m']+c['position_uncertainty_m']
        self.assertEqual(s.m.core.inside(p),all(a+radius<=v<=b-radius for a,b,v in zip(c['geofence_min_m'],c['geofence_max_m'],p)))
    def test_invalid_inputs_fail_closed(self):
        c=self.config()
        self.assertFalse(envelope_inside(c,(0,0,1),(0,0,0,0)))
        self.assertFalse(envelope_inside(c,(float('nan'),0,1),(0,0,0,1)))

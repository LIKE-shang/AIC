"""Ground-contact tolerance matches bounded stationary calibration variation."""
import copy,unittest
from test_takeoff import TakeoffSim
from mission import Mission
from field_site import bind_bounds
from window_pass.core import Pose

class GroundContact(unittest.TestCase):
    def bound(self):
        s=TakeoffSim()
        d={'boundary_distances_m':{'front':4.5,'back':.81,'left':1.,'right':3.3},'ceiling_above_ground_m':4.}
        g=bind_bounds(d,(0.,0.,.245),(0.,0.,0.,1.),s.options,s.m.core.c)
        c=copy.deepcopy(s.m.core.c);c.update({k:g[k] for k in ['geofence_min_m','geofence_max_m']})
        s.m=Mission(c,s.options,experiment='takeoff_land');s.f.ctrl=s.m.core
        for _ in range(30):s.update()
        return s,g
    def test_start_with_one_cm_ground_change_is_not_false_fence_rejection(self):
        s,g=self.bound();s.f.p=(0.,0.,.230)
        for _ in range(30):s.update()
        s.start();self.assertEqual(s.m.phase,'PRESTREAM',s.m.output())
        self.assertTrue(all(s.m.output()['startup_checks'].values()))
        self.assertAlmostEqual(g['floor_contact_tolerance_m'],.05)
        self.assertAlmostEqual(s.m.target_z-s.m.origin[2],.9)
    def test_excessive_ground_change_remains_rejected(self):
        s,g=self.bound();s.f.p=(0.,0.,.180)
        for _ in range(30):s.update()
        s.start();self.assertEqual(s.m.phase,'REFUSED')
        self.assertFalse(s.m.output()['startup_checks']['start_envelope_inside'])
    def test_ceiling_and_horizontal_bounds_are_preserved(self):
        s,g=self.bound()
        self.assertAlmostEqual(g['geofence_min_m'][0],-.81)
        self.assertAlmostEqual(g['geofence_max_m'][0],4.5)
        self.assertAlmostEqual(g['geofence_min_m'][1],-3.3)
        self.assertAlmostEqual(g['geofence_max_m'][1],1.)
        self.assertAlmostEqual(g['geofence_max_m'][2],2.)
        self.assertAlmostEqual(s.m.o['minimum_bottom_height_m'],.8)

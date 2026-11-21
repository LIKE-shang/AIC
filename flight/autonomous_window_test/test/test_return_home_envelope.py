"""Recorded 1914672461375 stop and bounded return-only cone slack; no IO."""
import copy,math,unittest
from dataclasses import replace
from unittest.mock import patch
from test_second_window_return import ReturnSim
from attitude_envelope import enclosed,support,cylinder_axis_support
from envelope_fence import envelope_inside,envelope_margins
from window_pass.geometry import rotate
from second_window_return import HOME_TILT_FENCE_SLACK_M

STOP=(.19085776805877686,-2.945363998413086,.7884054780006409)
Q=(-.00127531785680653,-.0013985332011908827,-.9999337701867184,.011358533526837342)
LO=[-.8223655922566979,-3.3012313683172367,-.4015352530845848]
HI=[4.130916365469133,.9929445224531406,1.6950310201794527]

class ReturnHomeEnvelopeTests(unittest.TestCase):
    def second_pass(self):
        s=ReturnSim();s.start()
        for _ in range(4500):
            s.step()
            if s.m.phase=='PASS' and s.m.return_journey.leg==2:return s
        self.fail(s.m.reason)

    def recorded_stop(self):
        s=self.second_pass();m=s.m;j=m.return_journey
        m.core.c.update(geofence_min_m=list(LO),geofence_max_m=list(HI))
        m.origin=(.000477654772112146,.01701381243765354,-.0702110007405281)
        m.target_z=.8297889992594719;m.ground_z=-.3152110007405281
        m.heading=-3.10557092088099
        j.first_plan=dict(geometry_center=[1.5802363867674052,.0552347918168502,.6841996026640028],
                          window_normal_local_m=[.9994881176679349,.009790199598851178,-.030457423272874533])
        s.f.p=STOP;s.f.q=Q;m.core.pose=replace(m.core.pose,p=STOP,q=Q);m.position=STOP
        return s

    def test_recorded_stop_enters_home_and_reference_moves_inward(self):
        s=self.recorded_stop();m=s.m;j=m.return_journey;c=copy.deepcopy(m.core.c)
        self.assertTrue(envelope_inside(c,STOP,Q))
        self.assertFalse(enclosed(c,STOP,m.heading))
        out=j.begin_home(m,s.inputs())
        self.assertEqual(out['phase'],'RETURN_HOME');self.assertIsNone(out['action'])
        self.assertEqual(m.core.c,c);self.assertTrue(j.region(m,STOP,-1))
        for _ in range(20):
            out=j.move(m,s.inputs(),.05,-1)
            self.assertEqual(out['phase'],'RETURN_HOME',out['reason'])
        self.assertGreater(m.position[1],STOP[1]);self.assertLess(m.position[0],STOP[0])
        self.assertEqual(j.status()['home_tilt_fence_slack_m'],.08)

    def test_eight_cm_cone_limit_is_bounded(self):
        s=self.recorded_stop();m=s.m;j=m.return_journey
        lower=LO[1]+support(m.core.c,m.heading,(0.,-1.,0.))+.04-.08
        p=(STOP[0],lower-.001,STOP[2]);m.core.pose=replace(m.core.pose,p=p,q=(0.,0.,0.,1.))
        cone=dict(m.core.c,geofence_min_m=[LO[0]-.08,LO[1]-.08,LO[2]],geofence_max_m=[HI[0]+.08,HI[1]+.08,HI[2]])
        self.assertFalse(enclosed(cone,p,m.heading))
        j.second_pass_complete=True
        self.assertFalse(j.region(m,p,-1))
        self.assertEqual(HOME_TILT_FENCE_SLACK_M,.08)

    def test_actual_safe_point_with_more_than_five_cm_cone_deficit_can_return(self):
        s=self.recorded_stop();m=s.m;j=m.return_journey
        p=(STOP[0],-2.950,STOP[2]);m.core.pose=replace(m.core.pose,p=p,q=(0.,0.,0.,1.))
        cone5=dict(m.core.c,geofence_min_m=[LO[0]-.05,LO[1]-.05,LO[2]],geofence_max_m=[HI[0]+.05,HI[1]+.05,HI[2]])
        self.assertFalse(enclosed(cone5,p,m.heading))
        self.assertTrue(envelope_inside(m.core.c,p,m.core.pose.q))
        out=j.begin_home(m,s.inputs());self.assertEqual(out['phase'],'RETURN_HOME')

    def test_recorded_stop_reaches_origin_and_auto_landing_in_simulation(self):
        s=self.recorded_stop();m=s.m;j=m.return_journey
        s.out=j.begin_home(m,s.inputs())
        self.assertEqual(s.out['phase'],'RETURN_HOME')
        s.until('AUTO_LANDING',limit=2000)
        self.assertTrue(j.origin_return_stopped)
        self.assertLess(math.dist(s.f.p[:2],m.origin[:2]),.081)
        self.assertEqual(s.actions,['OFFBOARD','ARM','LAND'])

    def test_actual_two_cm_clearance_is_still_required(self):
        s=self.recorded_stop();m=s.m;j=m.return_journey;j.second_pass_complete=True
        a=math.radians(-15)/2;p=(STOP[0],-2.960,STOP[2]);q=(math.sin(a),0.,0.,math.cos(a))
        m.core.pose=replace(m.core.pose,p=p,q=q)
        cone=dict(m.core.c,geofence_min_m=[LO[0]-.08,LO[1]-.08,LO[2]],geofence_max_m=[HI[0]+.08,HI[1]+.08,HI[2]])
        self.assertTrue(enclosed(cone,p,m.heading))
        self.assertFalse(envelope_inside(m.core.c,p,q));self.assertFalse(j.region(m,p,-1))

    def test_vertical_cone_limit_is_not_relaxed(self):
        s=self.recorded_stop();m=s.m;j=m.return_journey;j.second_pass_complete=True
        p=(STOP[0],0.,LO[2]+support(m.core.c,m.heading,(0.,0.,-1.))+.04-.001)
        m.core.pose=replace(m.core.pose,p=p,q=(0.,0.,0.,1.))
        self.assertTrue(envelope_inside(m.core.c,p,m.core.pose.q))
        self.assertFalse(j.region(m,p,-1))

    def test_other_stages_and_first_divider_keep_old_cone(self):
        for phase,leg,complete,side in [('PASS',2,False,-1),('RETURN_PREP',1,False,1),('YAW_TURN',1,False,1),('SEARCH',2,False,-1)]:
            s=self.recorded_stop();m=s.m;j=m.return_journey
            m.phase=phase;j.leg=leg;j.second_pass_complete=complete
            self.assertFalse(j.region(m,STOP,side),(phase,leg))
        s=self.recorded_stop();m=s.m;j=m.return_journey;j.second_pass_complete=True
        self.assertFalse(j.region(m,(1.58,-1.,STOP[2]),-1))

    def test_handover_action_survives_both_pass_completion_paths(self):
        for terminal in (True,False):
            s=self.second_pass();m=s.m
            if terminal:m.terminal_capture.update=lambda *args:True
            else:m.terminal_capture=None
            m.direct_arrival_since=s.f.wall-5;m.core.pose_speed=0.
            with patch('straight_pass.advance_route',return_value=(m.position,True)),patch.object(m.return_journey,'region',return_value=False):
                out=m.tick(s.inputs())
            self.assertEqual(out['phase'],'HANDOVER_REQUEST');self.assertEqual(out['action'],'POSCTL')
            self.assertFalse(m.core.authorized)

class SecondExitClearanceTests(unittest.TestCase):
    # Only 280532329379 supplies this fault pose, fence and frozen exit plane.
    P=(.676028847694397,-2.9528350830078125,.8001447916030884)
    Q=(-.05119913809735167,-.02005216923753554,-.9981915459098235,.024296478054772)
    LO=(-.863760525921005,-3.2986380748723843,-.4010847456807405)
    HI=(4.0956510829576835,1.0008518279861562,1.6947405706811696)

    def fault_model(self):
        s=ReturnHomeEnvelopeTests().second_pass();m=s.m
        m.core.c.update(geofence_min_m=list(self.LO),geofence_max_m=list(self.HI))
        m.core.pose=replace(m.core.pose,p=self.P,q=self.Q)
        m.direct_plan.update(geometry_center=[1.5208222324287881,-2.9060688732890165,.7459747375974828],
            window_normal_local_m=[-.9995372278816695,-.02370714840330131,-.019060461516220605],
            minimum_center_exit_signed_distance_m=.3962631525276158)
        m.heading=-3.1161739115101694
        return s

    def test_recorded_fault_passes_two_cm_guard_without_declaring_stop(self):
        s=self.fault_model();m=s.m;j=m.return_journey;c=copy.deepcopy(m.core.c)
        scoped=m.session_window.scoped_control(c)
        old=envelope_margins(scoped,self.P,self.Q)['lower_m'][1]
        self.assertAlmostEqual(old,-.004333985042116878,places=9)
        self.assertFalse(envelope_inside(scoped,self.P,self.Q))
        self.assertTrue(m.full_chain.clear_of_window(m,self.P))
        self.assertTrue(m.corridor(self.P));self.assertTrue(j.exit_clearance_active(m))
        self.assertEqual(m.phase,'PASS');self.assertFalse(j.second_pass_complete)
        self.assertEqual(m.core.c,c)
        self.assertAlmostEqual(old+.01,.005666014957883122,places=9)

    def test_before_second_exit_stale_clear_flag_does_not_relax_guard(self):
        s=self.fault_model();m=s.m;m.terminal_capture.clear=True
        p=(1.6,self.P[1],self.P[2]);m.core.pose=replace(m.core.pose,p=p)
        self.assertFalse(m.full_chain.clear_of_window(m,p))
        self.assertFalse(m.return_journey.exit_clearance_active(m));self.assertFalse(m.corridor(p))

    def test_other_stages_keep_original_actual_clearance(self):
        s=self.fault_model();m=s.m;j=m.return_journey
        for phase,leg,complete in [('PASS',1,False),('SEARCH',2,False),('ADJUST',2,False),
                ('RETURN_PREP',1,False),('YAW_TURN',1,False),('RETURN_HOME',2,False),('LANDING_LIGHT_ON',2,True)]:
            m.phase=phase;j.leg=leg;j.second_pass_complete=complete
            self.assertFalse(j.exit_clearance_active(m),(phase,leg,complete))
            self.assertFalse(m.corridor(self.P),(phase,leg,complete))

    def test_all_horizontal_faces_reject_below_two_cm_and_physical_crossing(self):
        s=self.fault_model();m=s.m;original=copy.deepcopy(m.core.c)
        axis=rotate(self.Q,(0.,0.,1.))
        for k in (0,1):
            for lower in (True,False):
                direction=tuple((-1. if lower else 1.)*float(a==k) for a in range(3))
                reach=cylinder_axis_support(original,direction,axis)
                for gap,accepted in ((.021,True),(.019,False),(-.001,False)):
                    m.core.c=copy.deepcopy(original)
                    key='geofence_min_m' if lower else 'geofence_max_m'
                    m.core.c[key][k]=self.P[k]+(-1. if lower else 1.)*(reach+gap)
                    self.assertEqual(m.corridor(self.P),accepted,(k,lower,gap))

    def test_vertical_reserve_remains_three_cm_pass_four_cm_home(self):
        s=self.fault_model();m=s.m;j=m.return_journey;original=copy.deepcopy(m.core.c)
        axis=rotate(self.Q,(0.,0.,1.))
        for phase,reserve in (('PASS',.03),('RETURN_HOME',.04)):
            m.phase=phase;j.second_pass_complete=phase=='RETURN_HOME'
            for lower in (True,False):
                direction=(0.,0.,-1. if lower else 1.);reach=cylinder_axis_support(original,direction,axis)
                for delta,accepted in ((.001,True),(-.001,False)):
                    m.core.c=copy.deepcopy(original)
                    key='geofence_min_m' if lower else 'geofence_max_m'
                    m.core.c[key][2]=self.P[2]+(-1. if lower else 1.)*(reach+reserve+delta)
                    self.assertEqual(m.corridor(self.P),accepted,(phase,lower,delta))

    def test_home_admission_uses_same_two_cm_guard_with_eight_cm_cone(self):
        s=self.fault_model();m=s.m;j=m.return_journey
        # Test admission in isolation; the real caller still requires stop.
        self.assertFalse(envelope_inside(m.core.c,self.P,self.Q))
        out=j.begin_home(m,s.inputs())
        self.assertEqual(out['phase'],'RETURN_HOME',out['reason'])
        self.assertTrue(m.corridor(self.P));self.assertTrue(j.region(m,self.P,-1))
        self.assertEqual(j.status()['second_exit_horizontal_clearance_m'],.02)

if __name__=='__main__':unittest.main(verbosity=2)

"""326247409240 regression and stage-boundary counterexamples; synthetic IO only."""
import copy, math, unittest
from dataclasses import replace
from test_second_window_return import ReturnSim
from attitude_envelope import enclosed, support, cylinder_axis_support
from envelope_fence import envelope_inside
from window_pass.geometry import rotate

class YawTurnCorridorTests(unittest.TestCase):
    P=(3.031494379043579,-2.8578107357025146,.867855429649353)
    Q=(.015460310757397155,-.012121556285647652,-.72196720942828,-.6916483734581425)
    LO=(-.787975833401604,-3.260408607456564,-.3958337616159373)
    HI=(4.085737615312623,.9647865194529466,1.6974851057430107)
    def recorded(self):
        s=ReturnSim((2,3));s.start();s.until('YAW_TURN');m=s.m;j=m.return_journey
        m.core.c.update(geofence_min_m=list(self.LO),geofence_max_m=list(self.HI))
        j.first_plan=dict(geometry_center=[1.5687553773302128,.07844286962086518,.7298727074189858],
                          window_normal_local_m=[.997969227020971,.0557144209284872,-.030875965082390593])
        s.f.p=self.P;s.f.q=self.Q;m.core.pose=replace(m.core.pose,p=self.P,q=self.Q)
        m.heading=1.8612893842273603
        return s

    def test_recorded_fault_passes_complete_turn_cone_and_tick_preserves_owner(self):
        s=self.recorded();m=s.m;j=m.return_journey;c=copy.deepcopy(m.core.c)
        self.assertTrue(envelope_inside(c,self.P,self.Q));self.assertFalse(enclosed(c,self.P,m.heading))
        turn=dict(c,_session_pass_tilt_limit_rad=m.attitude_limit())
        margin=self.P[1]-support(turn,m.heading,(0.,-1.,0.))-.04-self.LO[1]
        self.assertAlmostEqual(margin,.02151736070794641,places=10)
        self.assertTrue(j.region(m,self.P,1))
        origin=m.origin;owner=m.core;position=m.position
        out=j.tick(m,s.inputs(),.05)
        self.assertEqual(out['phase'],'YAW_TURN');self.assertIsNone(out['action'])
        self.assertEqual(m.origin,origin);self.assertIs(m.core,owner)
        self.assertEqual(m.position,position);self.assertEqual(m.core.c,c)
        self.assertEqual(j.leg,1);self.assertIs(j.first,m.session_window)
        self.assertFalse(j.second_pass_complete)

    def test_outside_complete_eight_degree_turn_cone_still_rejects(self):
        s=self.recorded();m=s.m;j=m.return_journey
        c=dict(m.core.c,_session_pass_tilt_limit_rad=m.attitude_limit())
        p=(self.P[0],self.LO[1]+support(c,m.heading,(0.,-1.,0.))+.04-.001,self.P[2])
        m.core.pose=replace(m.core.pose,p=p,q=(0.,0.,0.,1.))
        self.assertTrue(envelope_inside(m.core.c,p,m.core.pose.q))
        self.assertFalse(j.region(m,p,1))
        out=j.tick(m,s.inputs(),.05)
        self.assertEqual(out['phase'],'FAULT');self.assertIn('yaw_turn_outside_right_corridor',out['reason'])

    def test_actual_four_cm_reserve_still_rejects_on_all_six_faces(self):
        s=self.recorded();m=s.m;j=m.return_journey;original=copy.deepcopy(m.core.c)
        axis=rotate(self.Q,(0.,0.,1.))
        for k in range(3):
            for lower in (True,False):
                direction=tuple((-1. if lower else 1.)*float(a==k) for a in range(3))
                reach=cylinder_axis_support(original,direction,axis)
                for gap,accepted in ((.041,True),(.039,False),(-.001,False)):
                    m.core.c=copy.deepcopy(original)
                    key='geofence_min_m' if lower else 'geofence_max_m'
                    m.core.c[key][k]=self.P[k]+(-1. if lower else 1.)*(reach+gap)
                    self.assertEqual(j.field_inside(m,m.core.c,self.P),accepted,(k,lower,gap))
                    if not accepted:self.assertFalse(j.region(m,self.P,1),(k,lower,gap))

    def test_turn_tilt_over_eight_degrees_still_fails_common_guard(self):
        s=self.recorded();m=s.m
        a=math.radians(8.1)/2;m.core.pose=replace(m.core.pose,q=(math.sin(a),0.,0.,math.cos(a)))
        self.assertAlmostEqual(m.attitude_limit(),math.radians(8))
        self.assertEqual(m.common_error(s.inputs()),'attitude_limit')

    def test_preparation_and_second_search_still_require_fifteen_degree_cone(self):
        s=self.recorded();m=s.m;j=m.return_journey
        for phase,leg in (('RETURN_PREP',1),('SEARCH',2),('ADJUST',2),('PASS',2)):
            m.phase=phase;j.leg=leg
            c=m.session_window.scoped_control(m.core.c) if leg==2 else m.core.c
            boundary=self.LO[1]+support(c,m.heading,(0.,-1.,0.))+c.get('_session_field_fence_margin_m',c['clearance_margin_m'])
            p=(self.P[0],boundary-.001,self.P[2])
            m.core.pose=replace(m.core.pose,p=p,q=(0.,0.,0.,1.))
            self.assertTrue(envelope_inside(c,p,m.core.pose.q))
            self.assertFalse(j.region(m,p,1),(phase,leg))

    def test_divider_and_vertical_cone_are_still_enforced(self):
        s=self.recorded();m=s.m;j=m.return_journey
        p=tuple(j.first_plan['geometry_center']);m.core.pose=replace(m.core.pose,p=p)
        self.assertFalse(j.region(m,p,1))
        c=dict(m.core.c,_session_pass_tilt_limit_rad=m.attitude_limit())
        for lower in (True,False):
            p=list(self.P);w=(0.,0.,-1. if lower else 1.)
            p[2]=(self.LO[2]+support(c,m.heading,w)+.04-.001 if lower else self.HI[2]-support(c,m.heading,w)-.04+.001)
            m.core.pose=replace(m.core.pose,p=tuple(p),q=(0.,0.,0.,1.))
            self.assertFalse(j.region(m,tuple(p),1))

if __name__=='__main__':unittest.main(verbosity=2)

import unittest,types,copy,importlib.util
from pathlib import Path
spec=importlib.util.spec_from_file_location('indoor_policy',Path(__file__).resolve().parents[1]/'scripts/prepare_indoor_offboard_policy.py')
policy=importlib.util.module_from_spec(spec);spec.loader.exec_module(policy)
class IndoorPolicyGroundGuard(unittest.TestCase):
    def args(self):
        def msg(**kw):return types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda:100.)),_connection_header={'callerid':'/mavros'},**kw)
        return [msg(connected=True,armed=False,system_status=3,mode='MANUAL'),msg(landed_state=1),msg(channels=[1500]*6+[1050]),msg(pos_horiz_rel_status_flag=True,pos_vert_abs_status_flag=True),{'/mavros/vision_pose/pose':['/window_pass_competition_pose_session_20261013']},{'COM_OBL_RC_ACT':[4,0.],'COM_OBL_ACT':[0,0.],'NAV_RCL_ACT':[3,0.],'COM_RC_OVERRIDE':[3,0.]},100.1]
    def test_healthy_ground_only(self):self.assertEqual(policy.blockers(*self.args()),[])
    def test_armed_airborne_kill_or_missing_position_refused(self):
        for index,key,value in [(0,'armed',True),(0,'system_status',8),(1,'landed_state',2),(3,'pos_horiz_rel_status_flag',False)]:
            a=self.args();setattr(a[index],key,value);self.assertTrue(policy.blockers(*a))
    def test_stale_sources_and_competing_output_refused(self):
        a=self.args();a[-1]=102.;self.assertTrue(policy.blockers(*a))
        a=self.args();a[0]._connection_header={'callerid':'/other'};self.assertTrue(policy.blockers(*a))
        a=self.args();a[4]['/mavros/setpoint_raw/local']=['/other'];self.assertTrue(policy.blockers(*a))
    def test_unexpected_parameters_are_not_overwritten(self):
        a=self.args();a[5]['COM_OBL_RC_ACT']=[1,0.];self.assertIn('unexpected_parameter:COM_OBL_RC_ACT',policy.blockers(*a))
    def test_offboard_rc_switch_on_refused(self):
        a=self.args();a[2].channels[6]=1950;self.assertIn('A_off_required',policy.blockers(*a))

"""Ground reset must not arm or stream, and must observe new POSCTL reports."""
import unittest,types,threading,sys
from unittest.mock import patch,Mock
from test_auto_transport import app

class GroundIntentReset(unittest.TestCase):
    def runtime(self):
        r=app.Runtime.__new__(app.Runtime);r.lock=threading.RLock();r.motion=None;r.source_identity={'session':'test'}
        r.identity=lambda:{'session':'test'};r.core=types.SimpleNamespace(connected=True,armed=False,mode='POSCTL')
        r.i=types.SimpleNamespace(state_fresh=True,landed_fresh=True,landed=True,rc_ok=True,emergency=False,estimator_ok=True,graph_ok=True,battery_ok=True,wall=10)
        r.inputs=lambda:r.i;r.parameters_ok=lambda wall:True
        r.rows={'/mavros/state':(1.,types.SimpleNamespace(mode='POSCTL'))};r.calls=[];r.reports=0
        def wait(duration):
            r.reports+=1;r.rows['/mavros/state']=(1.+r.reports,types.SimpleNamespace(mode='POSCTL'))
        r.stop=types.SimpleNamespace(wait=wait)
        def proxy(name,kind):
            def call(**kw):r.calls.append((name,kw));return types.SimpleNamespace(mode_sent=True)
            return call
        r.ros=types.SimpleNamespace(wait_for_service=Mock(),ServiceProxy=proxy,loginfo=Mock())
        return r
    def invoke(self,r):
        srv=types.ModuleType('mavros_msgs.srv');srv.SetMode=object
        with patch.dict(sys.modules,{'mavros_msgs.srv':srv}):r.prepare_ground_posctl()
    def test_even_reported_posctl_requires_explicit_reset_and_new_messages(self):
        r=self.runtime();self.invoke(r)
        self.assertEqual(r.calls,[('/mavros/set_mode',{'base_mode':0,'custom_mode':'POSCTL'})])
        self.assertGreaterEqual(r.reports,2);self.assertIsNone(r.motion)
    def test_not_grounded_or_unhealthy_never_requests_mode(self):
        for attr in ('state_fresh','landed_fresh','landed','rc_ok','estimator_ok','graph_ok','battery_ok'):
            r=self.runtime();setattr(r.i,attr,False)
            with self.subTest(attr=attr),self.assertRaises(RuntimeError):self.invoke(r)
            self.assertFalse(r.calls)
    def test_arm_or_existing_output_prevents_reset(self):
        for change in ('armed','output','session','emergency'):
            r=self.runtime()
            if change=='armed':r.core.armed=True
            elif change=='output':r.motion=Mock()
            elif change=='session':r.identity=lambda:{'session':'changed'}
            else:r.i.emergency=True
            with self.subTest(change=change),self.assertRaises(RuntimeError):self.invoke(r)
            self.assertFalse(r.calls)
    def test_rejected_reset_stops_before_stream(self):
        r=self.runtime();r.ros.ServiceProxy=lambda *a:lambda **kw:types.SimpleNamespace(mode_sent=False)
        with self.assertRaisesRegex(RuntimeError,'rejected'):self.invoke(r)
        self.assertEqual(r.reports,0);self.assertIsNone(r.motion)
    def test_stale_report_cannot_confirm_reset(self):
        r=self.runtime();r.stop.wait=lambda duration:None
        clock=iter([0,0,1,2,3,4,5,6])
        with patch.object(app.time,'monotonic',side_effect=lambda:next(clock)):
            with self.assertRaisesRegex(RuntimeError,'not_observed'):self.invoke(r)
        self.assertIsNone(r.motion)

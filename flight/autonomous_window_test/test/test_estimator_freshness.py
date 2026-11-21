"""Reproduce measured 1 Hz jitter without suppressing estimator failures."""
import types,unittest,dataclasses
from unittest.mock import patch
from test_auto_transport import app
from test_takeoff import TakeoffSim

class EstimatorFreshness(unittest.TestCase):
    def runtime(self,age,horizontal=True,vertical=True,source='/mavros'):
        r=app.Runtime.__new__(app.Runtime)
        msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda:100.-age)),
            _connection_header={'callerid':source},pos_horiz_rel_status_flag=horizontal,pos_vert_abs_status_flag=vertical)
        r.rows={'/mavros/estimator_status':(100.-age,msg)}
        r.now=lambda:(100.,100.)
        r.ros=types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_sec=lambda:100.)))
        return r
    def test_measured_jitter_over_one_second_is_healthy(self):
        for age in [0.,.999,1.002185,1.003774,1.05,1.49]:
            r=self.runtime(age)
            with patch.object(app.time,'monotonic',return_value=100.):
                ok,detail=r.estimator_health()
            self.assertTrue(ok,detail);self.assertEqual(detail['reason'],'ok')
            self.assertAlmostEqual(detail['receipt_age_s'],age)
    def test_prolonged_outage_is_distinct_from_invalid_position(self):
        r=self.runtime(1.51)
        with patch.object(app.time,'monotonic',return_value=100.):ok,d=r.estimator_health()
        self.assertFalse(ok);self.assertEqual(d['reason'],'estimator_data_stale_or_invalid_stamp')
    def test_actual_flag_failures_remain_failures(self):
        for h,v,reason in [(False,True,'estimator_horizontal_invalid'),(True,False,'estimator_vertical_invalid')]:
            r=self.runtime(.02,h,v)
            with patch.object(app.time,'monotonic',return_value=100.):ok,d=r.estimator_health()
            self.assertFalse(ok);self.assertEqual(d['reason'],reason)
    def test_wrong_publisher_rejected(self):
        r=self.runtime(.01,source='/other')
        with patch.object(app.time,'monotonic',return_value=100.):ok,d=r.estimator_health()
        self.assertFalse(ok);self.assertEqual(d['reason'],'estimator_source_untrusted')
    def test_missing_message_explicit(self):
        r=self.runtime(0.);r.rows={}
        ok,d=r.estimator_health();self.assertFalse(ok);self.assertEqual(d['reason'],'estimator_data_missing')
    def test_fault_snapshot_preserves_cause_and_age(self):
        s=TakeoffSim();s.start();s.until('HOVER')
        evidence={'reason':'estimator_data_stale_or_invalid_stamp','receipt_age_s':1.51,'source_age_s':1.51,'timeout_s':1.5}
        i=dataclasses.replace(s.inputs(),estimator_ok=False,estimator_reason=evidence['reason'],estimator_evidence=evidence)
        out=s.m.tick(i)
        self.assertEqual(out['phase'],'FAULT');self.assertIn('estimator_data_stale',out['reason'])
        self.assertEqual(out['fault_snapshot']['estimator'],evidence)
    def test_actual_bad_flag_pulse_still_latches(self):
        from test_auto_transport import Boundary
        s=TakeoffSim();s.start();s.until('HOVER');r=Boundary().runtime(s);r.rows={}
        r.store('/mavros/estimator_status',types.SimpleNamespace(pos_horiz_rel_status_flag=False,pos_vert_abs_status_flag=True))
        r.store('/mavros/estimator_status',types.SimpleNamespace(pos_horiz_rel_status_flag=True,pos_vert_abs_status_flag=True))
        self.assertEqual(s.m.phase,'FAULT')

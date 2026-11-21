"""Preflight waits for real OFF ACK; pending/fault/stale data never grant readiness."""
import threading,types,unittest
from unittest.mock import patch
from test_auto_transport import app

class OffRefreshPreflight(unittest.TestCase):
    def runtime(self,ack_after=None,**fields):
        r=app.Runtime.__new__(app.Runtime);r.lock=threading.RLock();r.full_chain_enabled=True
        clock=[100.];calls=[]
        data=dict(scope='window_pass_single_target',fault_latched=False,state='OFF_PENDING',
                  startup_off_acked=True,requested_on=False,exposure_used=False,request_id=0)
        data.update(fields);r.hover_laser_row=(clock[0],data)
        def report():
            calls.append(clock[0]);ok=data['state']=='OFF_ACKNOWLEDGED' and not data['fault_latched']
            return dict(ready=ok,blockers=[] if ok else ['hover_laser_initial_off_not_acknowledged'])
        def wait(dt):
            clock[0]+=dt;r.hover_laser_row=(clock[0],data)
            if ack_after is not None and clock[0]>=100.+ack_after:data['state']='OFF_ACKNOWLEDGED'
        r.report=report;r.stop=types.SimpleNamespace(wait=wait)
        return r,clock,calls
    def test_refresh_waits_for_fresh_actual_ack(self):
        r,c,calls=self.runtime(.2)
        with patch.object(app.time,'monotonic',side_effect=lambda:c[0]):out=r.preflight_report()
        self.assertTrue(out['ready']);self.assertGreater(len(calls),1);self.assertGreaterEqual(c[0]-100.,.2)
    def test_persistent_pending_remains_blocked_after_bounded_wait(self):
        r,c,calls=self.runtime()
        with patch.object(app.time,'monotonic',side_effect=lambda:c[0]):out=r.preflight_report()
        self.assertFalse(out['ready']);self.assertGreaterEqual(c[0]-100.,2.);self.assertLess(c[0]-100.,2.1)
    def test_fault_used_on_request_wrong_scope_and_stale_do_not_wait_or_pass(self):
        for fields in [dict(fault_latched=True),dict(exposure_used=True),dict(requested_on=True),dict(scope='single_hover_target')]:
            r,c,calls=self.runtime(.1,**fields)
            with patch.object(app.time,'monotonic',side_effect=lambda:c[0]):out=r.preflight_report()
            self.assertFalse(out['ready']);self.assertEqual(len(calls),1)
        r,c,calls=self.runtime(.1);r.hover_laser_row=(99.,r.hover_laser_row[1])
        with patch.object(app.time,'monotonic',side_effect=lambda:c[0]):out=r.preflight_report()
        self.assertFalse(out['ready']);self.assertEqual(len(calls),1)
    def test_other_safety_blocker_is_not_retried_into_success(self):
        r,c,calls=self.runtime(.1);r.report=lambda:dict(ready=False,blockers=['rc_not_ready','hover_laser_initial_off_not_acknowledged'])
        with patch.object(app.time,'monotonic',side_effect=lambda:c[0]):out=r.preflight_report()
        self.assertFalse(out['ready']);self.assertEqual(c[0],100.)

if __name__=='__main__':unittest.main()

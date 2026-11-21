"""Explicit external-alarm policy; no hardware or motor commands."""
import unittest,types
from test_mission import Sim
from test_auto_transport import app
from mission import settings

class BatteryPolicy(unittest.TestCase):
    def test_default_still_blocks_bad_battery(self):
        s=Sim();s.healthy['battery_ok']=False;s.start()
        self.assertEqual(s.m.phase,'REFUSED')
    def test_external_alarm_allows_full_mission_without_battery(self):
        s=Sim();s.m.o['companion_battery_monitor_enabled']=False
        s.healthy['battery_ok']=False;s.start();s.until('DONE')
        self.assertNotIn('battery_low',[e['reason'] for e in s.m.events])
    def test_default_inflight_battery_abort_preserved(self):
        s=Sim();s.start();s.until('TAKEOFF');s.healthy['battery_ok']=False;s.step()
        self.assertEqual(s.m.reason,'battery_low')
    def test_invalid_policy_rejected(self):
        s=Sim()
        for value in ('false',0,None):
            with self.assertRaises(ValueError):settings(dict(s.options,companion_battery_monitor_enabled=value))
    def test_disabled_transport_never_reads_battery(self):
        r=app.Runtime.__new__(app.Runtime)
        r.cfg={'mission':{'companion_battery_monitor_enabled':False}}
        r.now=lambda:(10.,10.);r.graph_ok=True;r.graph_wall=10.
        def fresh(topic,*args):
            self.assertNotEqual(topic,'/mavros/battery')
            return None
        r.fresh=fresh
        self.assertTrue(r.inputs().battery_ok)

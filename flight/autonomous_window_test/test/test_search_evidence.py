import unittest
from test_alignment_only import AlignOnlySim

class SearchEvidence(unittest.TestCase):
    def test_timeout_retains_reason_before_handover(self):
        s=AlignOnlySim(False);s.start();s.until('PILOT_CONTROL')
        d=s.out['search_evidence'];self.assertIsNotNone(d)
        self.assertFalse(d['geometry_ready']);self.assertFalse(d['legacy_rgb_target_fresh'])
        self.assertGreater(len(s.out['search_events']),0)
        self.assertFalse(s.out['alignment_completed'])
    def test_valid_geometry_but_missing_legacy_target_is_distinguished(self):
        s=AlignOnlySim();s.start();s.until('SEARCH')
        s.m.core.live_bound=True;s.m.core.obs=None;s.m.core.samples.clear()
        s.m.tick(s.inputs());d=s.m.output()['search_evidence']
        self.assertTrue(d['geometry_ready']);self.assertEqual(d['reason'],'legacy_rgb_target_not_fresh')
    def test_slow_hold_refusal_reason_is_preserved(self):
        s=AlignOnlySim();s.start();s.until('SEARCH')
        s.m.core.live_bound=True;s.m.core.target_fresh=lambda *a:True;s.m.core.confirmed=lambda:True
        s.m.core.request_hold=lambda *a:(False,'not_stably_hovering')
        s.m.tick(s.inputs());self.assertEqual(s.m.output()['search_evidence']['reason'],'hold_refused:not_stably_hovering')
    def test_event_history_is_bounded(self):
        s=AlignOnlySim();s.start();s.until('SEARCH')
        for n in range(100):s.m.record_search_evidence(s.inputs(),False,'test_'+str(n))
        self.assertEqual(len(s.m.search_events),60)

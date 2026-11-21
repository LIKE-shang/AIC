import copy,math,unittest
from policy import ReturnHandoff,same_start
from test_policy import D
SAVED={'identity':{'boot':'b','run':'r','source':'uri'},'position_local_m':[0.,0.,0.]}
class ReturnHandoffTests(unittest.TestCase):
 def current(self,p):return dict(identity=copy.deepcopy(SAVED['identity']),position_local_m=list(p))
 def stable(self,p,ground=None):
  gate=ReturnHandoff(copy.deepcopy(SAVED))
  for n in range(12):
   if gate.observe(self.current(p),ground or D,10.+n*.05,20.+n*.05):break
  return gate
 def test_recorded_10p016cm_refusal_now_passes_after_stable_feedback(self):
  p=[.09184434130850197,-.039639983775585995,.005080223894019506]
  with self.assertRaisesRegex(ValueError,'prepared_start_moved'):same_start(SAVED,self.current(p))
  gate=self.stable(p);self.assertTrue(gate.ready);self.assertAlmostEqual(gate.metrics['horizontal_error_m'],.10003355109222767)
 def test_fifteen_cm_only_allowed_post_ground_not_prestart(self):
  with self.assertRaises(ValueError):same_start(SAVED,self.current([.15,0.,0.]))
  self.assertTrue(self.stable([.15,0.,0.]).ready)
 def test_twenty_cm_is_finite_horizontal_cap(self):
  self.assertTrue(self.stable([.20,0.,0.]).ready)
  gate=self.stable([.201,0.,0.]);self.assertFalse(gate.ready);self.assertIn('over20cm',gate.reason)
 def test_height_keeps_original_ten_cm_cap_not_twenty(self):
  self.assertFalse(self.stable([.10,0.,.101]).ready)
  self.assertTrue(self.stable([.10,0.,.099]).ready)
 def test_source_boot_run_change_never_waits_or_passes(self):
  for k in ('boot','run','source'):
   current=self.current([.1,0.,0.]);current['identity'][k]='new'
   with self.assertRaisesRegex(ValueError,'sources_changed'):ReturnHandoff(SAVED).observe(current,D,1.,1.)
 def test_requires_ground_DONE_zero_command_and_original_map_return(self):
  for ground in (dict(D,state='ABORT'),dict(D,dry_run=True),dict(D,pose=[.11,0.,0.]),dict(D,command={'vx':.01,'vy':0.,'wz':0.})):
   with self.assertRaises(ValueError):ReturnHandoff(SAVED).observe(self.current([.1,0.,0.]),ground,1.,1.)
 def test_no_release_on_single_instantaneous_DONE_sample(self):
  gate=ReturnHandoff(SAVED);self.assertFalse(gate.observe(self.current([.1,0.,0.]),D,1.,1.))
 def test_moving_feedback_does_not_pass_at_twenty_cm(self):
  gate=ReturnHandoff(SAVED)
  for n in range(20):gate.observe(self.current([n*.01,0.,0.]),D,1.+n*.05,1.+n*.05)
  self.assertFalse(gate.ready)
 def test_duplicate_pose_cannot_create_stable_window(self):
  gate=ReturnHandoff(SAVED)
  for n in range(40):gate.observe(self.current([.1,0.,0.]),D,1.,1.+n*.05)
  self.assertFalse(gate.ready);self.assertEqual(len(gate.rows),1)
 def test_fast_replayed_stamps_cannot_fake_half_second(self):
  gate=ReturnHandoff(SAVED)
  for n in range(10):gate.observe(self.current([.1,0.,0.]),D,1.+n*.1,1.+n*.001)
  self.assertFalse(gate.ready)
 def test_reversed_pose_time_refused(self):
  gate=ReturnHandoff(SAVED);gate.observe(self.current([.1,0.,0.]),D,1.,1.)
  with self.assertRaises(ValueError):gate.observe(self.current([.1,0.,0.]),D,.9,1.1)
 def test_invalid_feedback_refused(self):
  for p in ([float('nan'),0.,0.],[float('inf'),0.,0.],[True,0.,0.],[0.,0.]):
   with self.assertRaises(ValueError):ReturnHandoff(SAVED).observe(self.current(p),D,1.,1.)
 def test_confirmation_single_use_and_original_snapshot_immutable(self):
  saved=copy.deepcopy(SAVED);gate=self.stable([.15,0.,0.]);self.assertEqual(saved,SAVED)
  with self.assertRaises(ValueError):gate.observe(self.current([.15,0.,0.]),D,22.,22.)
 def test_out_of_band_feedback_restarts_stable_interval(self):
  gate=ReturnHandoff(SAVED)
  for n in range(6):gate.observe(self.current([.1,0.,0.]),D,1.+n*.05,1.+n*.05)
  gate.observe(self.current([.21,0.,0.]),D,1.3,1.3)
  self.assertFalse(gate.observe(self.current([.1,0.,0.]),D,1.35,1.35));self.assertEqual(len(gate.rows),1)
 def test_entry_update_requires_new_preparation_manifest(self):
  import competition
  from unittest.mock import patch
  with patch.object(competition,'digest',return_value='new'):
   with self.assertRaisesRegex(ValueError,'entry_changed'):competition.validate_entry_binding({'entry_build':competition.ENTRY_BUILD,'entry_manifest_sha256':'old'})
   competition.validate_entry_binding({'entry_build':competition.ENTRY_BUILD,'entry_manifest_sha256':'new'})
 def test_handoff_precedes_same_supervisor_full_flight_all_pairs(self):
  from test_second_window_return import ReturnSim
  for ids in ([1,2],[1,3],[2,3]):
   self.assertTrue(self.stable([.10,-.03,0.]).ready)
   sim=ReturnSim(ids);sim.start();owner=sim.m.core;sim.until('AUTO_LANDING',limit=4500)
   self.assertIs(sim.m.core,owner);self.assertEqual(sim.actions,['OFFBOARD','ARM','LAND'])
   self.assertTrue(sim.m.return_journey.second_pass_complete);self.assertTrue(sim.m.return_journey.origin_return_stopped)
if __name__=='__main__':unittest.main()

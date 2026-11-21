#!/usr/bin/env python3
"""Restore guarded single-target pose forwarding. No flight commands."""
import hashlib,importlib.util,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from hover_pose_guard import guarded_bridge
from full_chain_localization import RouteContinuity
POSE_ROOT=Path('/home/abot/window_pass_competition_pose_20261011_v1')
EXPECTED={
 'scripts/competition_pose.py':'0ccb415aace66b801c627f513728699c951cedb52ae9a8af97eb774bf949ea13',
 'scripts/competition_ground.py':'a71bf8f4389ca8483aabda7416827f516c80e677e7e71084312c59b28aaf6cf7',
 'competition/pose_guard_v1.py':'15dabf5e06f398f7c5756a259aed73f452d488bada6a53d0fa3dff8fe3195417',
 'config/competition_ground.json':'5401e6fa77f18a490e52fa716603c9b31493a1362d42b04f1dbc467a53cbd9d0'}

def main():
 if sys.argv[1:]!=['--run']:raise ValueError('explicit --run required')
 for name,digest in EXPECTED.items():
  if hashlib.sha256((POSE_ROOT/name).read_bytes()).hexdigest()!=digest:
   raise ValueError('original_pose_source_changed:'+name)
 spec=importlib.util.spec_from_file_location('hover_legacy_pose',POSE_ROOT/'scripts/competition_pose.py')
 legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
 control=json.loads((ROOT/'config/full_chain_single_target.json').read_text())['control']
 legacy.ContinuousPoseBridge=guarded_bridge(legacy,control,RouteContinuity,startup_mode='ALTCTL')
 return legacy.main(['--run','--continuous-pose'])

if __name__=='__main__':
 try:sys.exit(main())
 except Exception as exc:raise SystemExit('HOVER_POSE_STOP:'+str(exc))

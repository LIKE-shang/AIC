#!/usr/bin/env python3
import hashlib,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
result=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(ROOT/'test'),'-v'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
(ROOT/'OFFLINE_TEST_OUTPUT.txt').write_text(result.stdout,encoding='utf-8')
paths=[ROOT/'session_window.py',ROOT/'session_localization.py',ROOT/'config/session_window.json',ROOT/'mission.py',ROOT/'scripts/auto_window.py',ROOT/'config/airborne_rgbd.json',ROOT/'live_geometry.py',ROOT/'straight_pass.py',ROOT/'attitude_envelope.py',ROOT/'prepass_adjustment.py',ROOT/'straight_pass_policy.py',ROOT/'envelope_fence.py',ROOT/'alignment_policy.py',ROOT/'scripts/start_components.py',ROOT/'takeoff.py',ROOT/'field_site.py',ROOT/'scripts/bind_takeoff_site.py',ROOT/'config/takeoff_land.json',ROOT/'scripts/reboot_setup.py']
report=dict(ok=result.returncode==0,kind='synthetic_feedback_and_fake_ROS_services',
            real_flight=False,SITL=False,real_auto_takeoff_land=False,real_window_pass=False,
            sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
(ROOT/'OFFLINE_RESULTS.json').write_text(json.dumps(report,indent=2)+chr(10),encoding='utf-8')
print(result.stdout);sys.exit(result.returncode)

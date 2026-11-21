#!/usr/bin/env python3
"""Read-only console and bounded automatic evidence; no manual check chain."""
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import rospy
from std_msgs.msg import String
from mavros_msgs.msg import State, ExtendedState
from window_live_vision.stage import stage_summary, display_lines, evidence_json


def main():
    rospy.init_node('window_live_stage')
    rows, received, lock = {}, {}, threading.Lock()
    root=Path.home()/'window_rgbd_logs'
    root.mkdir(parents=True,exist_ok=True)
    directory=Path(tempfile.mkdtemp(prefix='stage_',dir=str(root)))
    print('自动诊断目录：'+str(directory),flush=True)
    def callback(topic,msg):
        try:
            if isinstance(msg,String):row=json.loads(msg.data)
            elif isinstance(msg,State):row=dict(connected=msg.connected,armed=msg.armed,mode=msg.mode,stamp=msg.header.stamp.to_sec())
            else:row=dict(landed_state=msg.landed_state,stamp=msg.header.stamp.to_sec())
        except (ValueError,TypeError):return
        with lock:rows[topic],received[topic]=row,time.monotonic()
    topics=[('/window_live/status',String),('/window_pass_competition/status',String),
            ('/mavros/state',State),('/mavros/extended_state',ExtendedState)]
    subs=[rospy.Subscriber(topic,typ,lambda msg,topic=topic:callback(topic,msg),queue_size=1) for topic,typ in topics]
    previous=None;last_print=last_save=0.;history_size=0;ready_saved=False;warned_io=False
    while not rospy.is_shutdown():
        now=time.monotonic()
        with lock:sample=dict(rows);ages={topic:now-stamp for topic,stamp in received.items()}
        summary=stage_summary(sample,ages,rospy.Time.now().to_sec())
        key=(summary['code'],summary['reason'])
        changed=key!=previous
        if changed or now-last_print>=3.:
            print('\n'.join(display_lines(summary)),flush=True);last_print=now
        if changed or now-last_save>=2.:
            payload=json.dumps(evidence_json(dict(summary=summary,received_age_s=ages,rows=sample)),ensure_ascii=False,allow_nan=False)
            try:
                temporary=directory/'latest.tmp'
                temporary.write_text(payload,encoding='utf-8')
                os.replace(str(temporary),str(directory/'latest.json'))
                if changed and history_size<2*1024*1024:
                    with (directory/'transitions.jsonl').open('a',encoding='utf-8') as handle:handle.write(payload+'\n')
                    history_size+=len(payload.encode('utf-8'))+1
                if summary['geometry_stage_ready'] and not ready_saved:
                    (directory/'first_geometry_ready.json').write_text(payload,encoding='utf-8');ready_saved=True
            except (OSError,ValueError) as exc:
                if not warned_io:print('自动保存失败：'+str(exc),flush=True);warned_io=True
            last_save=now
        previous=key
        if summary['code']=='LAND_AND_REVIEW':
            print('已保存当前证据，地面检查面板退出：'+str(directory),flush=True)
            break
        time.sleep(.2)


if __name__=='__main__':main()

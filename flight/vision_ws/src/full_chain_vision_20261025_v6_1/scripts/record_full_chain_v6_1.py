#!/usr/bin/env python3
"""Save a small raw-image burst during HOVER for later centring diagnosis."""
import json,threading,time,tempfile
from pathlib import Path
import cv2,rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

class Recorder:
    def __init__(self):
        self.lock=threading.Lock();self.stop=threading.Event();self.pending=None
        self.flight=None;self.visual=None;self.last_saved=0.;self.count=0;self.folder=None
        self.bridge=CvBridge()
        self.subs=[rospy.Subscriber('/autonomous_window_test/status',String,self.status,callback_args='flight',queue_size=1),
                   rospy.Subscriber('/circle_pixel_preview/status',String,self.status,callback_args='visual',queue_size=1),
                   rospy.Subscriber('/usb_cam/image_raw',Image,self.image,queue_size=1,buff_size=2**21,tcp_nodelay=True)]
        self.worker=threading.Thread(target=self.write,daemon=True);self.worker.start()
        rospy.on_shutdown(self.close)
    def status(self,msg,kind):
        expected='/autonomous_window_test' if kind=='flight' else '/circle_pixel_preview'
        if (msg._connection_header or {}).get('callerid')!=expected:return
        try:d=json.loads(msg.data)
        except (ValueError,TypeError):return
        if not isinstance(d,dict):return
        with self.lock:setattr(self,kind,(time.monotonic(),d))
    def image(self,msg):
        now=time.monotonic()
        with self.lock:
            if self.count>=360 or self.flight is None or now-self.flight[0]>.5 or self.flight[1].get('phase') not in ('TAKEOFF','SEARCH','ADJUST','PASS','POSTPASS','HOVER','HANDOVER_REQUEST') or now-self.last_saved<.25:return
            self.pending=(msg,self.flight[1],self.visual[1] if self.visual else None)
            self.last_saved=now
    def write(self):
        while not self.stop.wait(.02):
            with self.lock:row=self.pending;self.pending=None
            if row is None:continue
            try:
                msg,flight,visual=row
                if self.folder is None:
                    self.folder=Path(tempfile.mkdtemp(prefix='hover_vision_recording_',dir=str(Path.home())))
                    rospy.loginfo('HOVER_VISION_RECORDING %s',self.folder)
                image=self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                stem='%04d'%self.count
                if not cv2.imwrite(str(self.folder/(stem+'.jpg')),image,[cv2.IMWRITE_JPEG_QUALITY,95]):raise RuntimeError('image_write_failed')
                (self.folder/(stem+'.json')).write_text(json.dumps(dict(capture_stamp=msg.header.stamp.to_sec(),frame_id=msg.header.frame_id,flight=flight,visual=visual),ensure_ascii=False,indent=2))
                self.count+=1
            except Exception as exc:rospy.logwarn_throttle(2.,'Hover image recording: %s',exc)
    def close(self):self.stop.set()

if __name__=='__main__':
    rospy.init_node('record_hover_vision');cv2.setNumThreads(1);node=Recorder();rospy.spin()

#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from PIL import Image as PILImage
import io

class BarcodeDetector:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        # 创建cv_bridge用于ROS图像和OpenCV图像转换
        self.bridge = CvBridge()
        
        # 订阅RGB摄像头图像（和shape_detector相同的输入topic）
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
        
        # 可选：发布标注了条形码的图像
        self.image_pub = rospy.Publisher("barcode_detection_image", Image, queue_size=1)

        rospy.loginfo("Barcode detector node started!")
        rospy.loginfo("Subscribing to ROS image: /input_rgb_image")
        rospy.loginfo("Publishing annotated image: /barcode_detection_image")

    def image_callback(self, data):
        # 将ROS图像消息转换为OpenCV格式（BGR8）
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError as e:
            rospy.logerr(f"CVBridge转换错误: {e}")
            return

        # 复制原图像用于绘制标注
        annotated_frame = frame.copy()

        # --------------------------
        # 核心：从摄像头图像解码条形码
        # --------------------------
        # 1. OpenCV图像（BGR）转换为PIL图像（RGB）
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(rgb_frame)
        
        # 2. 解码条形码
        decoded_results = decode(pil_image)

        # 3. 处理识别结果并绘制标注
        for result in decoded_results:
            # 解析条形码信息
            barcode_data = result.data.decode('utf-8')
            barcode_type = result.type
            
            # 打印识别结果
            rospy.loginfo(f"识别内容: {barcode_data}")
            rospy.loginfo(f"条码类型: {barcode_type}")

            # 获取条形码边界框坐标并绘制
            (x, y, w, h) = result.rect
            cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # 绘制条形码文本信息
            text = f"{barcode_data} ({barcode_type})"
            cv2.putText(annotated_frame, text, (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 发布标注后的图像
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(f"图像发布错误: {e}")

    def cleanup(self):
        rospy.loginfo("Shutting down barcode detector node.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node("barcode_detector")
        BarcodeDetector()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down barcode detector node (KeyboardInterrupt).")
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupt exception occurred.")

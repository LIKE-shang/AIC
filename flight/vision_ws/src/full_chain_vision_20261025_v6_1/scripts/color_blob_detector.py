#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ColorBlobDetector:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        # 创建cv_bridge
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("color_blob_image", Image, queue_size=1)

        # 从参数服务器获取色块识别的颜色范围（HSV），可在launch文件中配置
        self.hue_min = rospy.get_param("~hue_min", 0)
        self.hue_max = rospy.get_param("~hue_max", 54)
        self.sat_min = rospy.get_param("~sat_min", 130)
        self.sat_max = rospy.get_param("~sat_max", 255)
        self.val_min = rospy.get_param("~val_min", 79)
        self.val_max = rospy.get_param("~val_max", 255)

        # 形态学操作参数，用于降噪
        self.kernel_size = rospy.get_param("~kernel_size", 5)
        self.kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)

        # 色块最小最大面积阈值（过滤过小/过大的噪点）
        self.min_area = rospy.get_param("~min_area", 100)
        self.max_area = rospy.get_param("~max_area", 10000)

        # 框选色块的颜色（BGR）
        self.color = (0, 255, 0)

        # 初始化图像订阅者，保持和原代码相同的输入接口
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)

    def image_callback(self, data):
        # 使用cv_bridge将ROS图像转换为OpenCV格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError as e:
            print (e)
            return

        # 色块识别核心处理
        result_image = self.detect_color_blob(frame)

        # 将识别后的图像发布为ROS消息
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))
        except CvBridgeError as e:
            print (e)

    def detect_color_blob(self, frame):
        # 1. 转换为HSV颜色空间（比BGR更适合颜色分割）
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2. 构建颜色掩码
        lower_color = np.array([self.hue_min, self.sat_min, self.val_min])
        upper_color = np.array([self.hue_max, self.sat_max, self.val_max])
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # 3. 形态学操作去除噪点
        mask = cv2.erode(mask, self.kernel, iterations=1)
        mask = cv2.dilate(mask, self.kernel, iterations=1)

        # 4. 查找轮廓
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 5. 筛选并绘制符合条件的色块
        result_frame = frame.copy()
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # 过滤面积不符合的轮廓
            if self.min_area < area < self.max_area:
                # 计算轮廓的外接矩形
                x, y, w, h = cv2.boundingRect(cnt)
                # 绘制矩形框
                cv2.rectangle(result_frame, (x, y), (x + w, y + h), self.color, 2)
                # 绘制轮廓中心点
                cx = x + w // 2
                cy = y + h // 2
                cv2.circle(result_frame, (cx, cy), 5, (0, 0, 255), -1)

        return result_frame

    def cleanup(self):
        print ("Shutting down color blob detector node.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node("color_blob_detector")
        ColorBlobDetector()
        rospy.loginfo("Color blob detector is started..")
        rospy.loginfo("Please subscribe the ROS image: /color_blob_image")
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down color blob detector node.")
        cv2.destroyAllWindows()

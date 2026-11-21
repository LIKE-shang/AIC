#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ShapeDetector:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        # 创建cv_bridge，复用原代码的图像格式转换逻辑
        self.bridge = CvBridge()
        # 发布识别后的图像
        self.image_pub = rospy.Publisher("shape_detection_image", Image, queue_size=1)

        # 形状识别参数（可在launch文件中配置）
        self.blur_kernel = rospy.get_param("~blur_kernel", 5)  # 模糊核大小
        self.canny_thresh1 = rospy.get_param("~canny_thresh1", 50)  # 边缘检测阈值1
        self.canny_thresh2 = rospy.get_param("~canny_thresh2", 150)  # 边缘检测阈值2
        self.epsilon_ratio = rospy.get_param("~epsilon_ratio", 0.02)  # 轮廓逼近系数

        # 订阅RGB图像（和原人脸识别代码相同的输入topic）
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)

        # 形状颜色配置
        self.shape_colors = {
            "circle": (0, 255, 255),    # 黄色
            "rectangle": (255, 0, 0),   # 蓝色
            "square": (0, 0, 255),      # 红色
            "triangle": (255, 255, 0),  # 青色
            "unknown": (128, 128, 128)  # 灰色
        }

    def image_callback(self, data):
        # 复用原代码的ROS图像转OpenCV格式逻辑（bgr8）
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError as e:
            rospy.logerr(f"CVBridge转换错误: {e}")
            return

        # 形状识别核心流程
        shape_frame = self.detect_shapes(frame.copy())

        # 发布识别后的图像（复用原代码的发布逻辑）
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(shape_frame, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(f"图像发布错误: {e}")

    def detect_shapes(self, frame):
        # 1. 预处理：灰度化、模糊、边缘检测（形状识别基础）
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (self.blur_kernel, self.blur_kernel), 0)
        edges = cv2.Canny(blurred, self.canny_thresh1, self.canny_thresh2)

        # 2. 查找轮廓
        contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 3. 遍历每个轮廓识别形状
        for cnt in contours:
            # 过滤小轮廓（去除噪声）
            area = cv2.contourArea(cnt)
            if area < 100:
                continue

            # 轮廓逼近（减少轮廓点数量）
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, self.epsilon_ratio * peri, True)
            x, y, w, h = cv2.boundingRect(approx)

            # 4. 形状判断
            shape = "unknown"
            # 三角形（3个顶点）
            if len(approx) == 3:
                shape = "triangle"
            # 矩形/正方形（4个顶点）
            elif len(approx) == 4:
                # 计算长宽比判断正方形/矩形
                aspect_ratio = w / float(h)
                if 0.95 <= aspect_ratio <= 1.05:
                    shape = "square"
                else:
                    shape = "rectangle"
            # 圆形（通过轮廓面积与外接圆面积的比值判断）
            else:
                (x_c, y_c), radius = cv2.minEnclosingCircle(cnt)
                circle_area = np.pi * (radius **2)
                # 轮廓面积与外接圆面积比值接近1则为圆形
                if 0.7 <= (area / circle_area) <= 1.2:
                    shape = "circle"

            # 5. 绘制形状轮廓和标签
            cv2.drawContours(frame, [approx], -1, self.shape_colors[shape], 2)
            # 在形状中心绘制标签
            cx = x + w // 2
            cy = y + h // 2
            cv2.putText(frame, shape, (cx - 20, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        return frame

    def cleanup(self):
        rospy.loginfo("Shutting down shape detector node.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        # 初始化ROS节点（保持和原代码一致的启动逻辑）
        rospy.init_node("shape_detector")
        ShapeDetector()
        rospy.loginfo("Shape detector is started..")
        rospy.loginfo("Please subscribe the ROS image: /shape_detection_image")
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down shape detector node (KeyboardInterrupt).")
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupt exception occurred.")

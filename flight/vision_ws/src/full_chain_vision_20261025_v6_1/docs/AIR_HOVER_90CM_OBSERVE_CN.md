# 0.9 米空中悬停观测

该入口复用现有 USB 三环识别节点，在飞机已经由既有穿框流程稳定悬停到约 0.9 米后采集完整靶面图像。它发布标注图、掩膜、检测状态和偏轴预测结果，默认不会启动激光桥，也不会发布飞控控制量。

启动前确认相机设备路径和唯一相机发布者。启动后保持飞机悬停，观察：

- `/circle_laser_test/image`
- `/circle_laser_test/detection_status`
- `/circle_laser_test/decision`

采集原始帧和头信息可使用现有只读工具：

```bash
rosrun robot_vision audit_air_camera.py \
  --topic /usb_cam/image_raw --seconds 10 \
  --output ~/air_hover_90cm_audit
```

只有在空中获得至少 5 份人工复核的完整三环样本后，才使用 `tools/calibrate_air_target.py` 生成未确认标定文件，并重新审阅相机帧、距离、偏心量和误差界。该过程不会自动把标定标记为可用于激光。

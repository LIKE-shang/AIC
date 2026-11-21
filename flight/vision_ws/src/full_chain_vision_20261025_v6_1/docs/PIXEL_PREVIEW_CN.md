# 640×360 像素偏移实时调试

本节点使用 usb_raw_rbANV8 八张原图的测量值，不发布 LaserDecision、飞控命令或 PWM。
用户实测的 7 cm 仍单独保留；没有把它当成两个轴各 7 cm，也没有换成机体运动量。

## 参数与画面

- 默认像素偏移：x=+53.25、y=+15.484375，相对几何画面中心 (319.5,179.5)。
- 默认参考点：(372.75,194.984375)，必须使用 640×360、circle_usb_optical_frame。
- 黄色叉 REF：测得的参考点；绿色十字：当前最内环内边界中心。
- 红色十字 SPOT：当前帧粉色光晕/亮核候选。丢失或多候选时不补造结果。
- REF dx/dy：参考点减靶心，像素 x 向右、y 向下。
- SPOT-REF dx/dy：当前亮点减参考点，用于观察参考点是否需要调整。
- radius：参考点在内环椭圆上的归一化半径，<1 表示点位于未涂黑的最内环开口内。边上不计入。
- inner_radius_ratio 默认1.0，调小可选择更小的显示容差区域。

SPOT-REF 接近零只是说明预测参考点与亮点一致，不能等同于亮点在靶心。
POINT INSIDE 只判断二维点在开口内，不包含真实光斑面积、相机外参、姿态变化或连续命中认证。
本节点不自动宣布实际测试通过，也不负责悬停或转移靶位。

## 相机已经运行时

Nano 上新终端：

```bash
source /opt/ros/noetic/setup.bash
source /home/abot/abot_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=192.168.0.100
roslaunch robot_vision air_hover_pixel_debug.launch
```

默认 start_camera=false，只使用已有 /usb_cam/image_raw，不争用摄像头。
旧观测 launch 若正在拥有相机，不要为了启动本节点而把它一起关掉。
只启动一份本调试节点，图像终端只订阅一个需要观看的图像话题。

## 相机没有运行时

```bash
roslaunch robot_vision air_hover_pixel_debug.launch start_camera:=true \
  video_device:=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0
```

## Linux 192.168.0.135 查看

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=192.168.0.135
rosrun rqt_image_view rqt_image_view
```

在话题下拉菜单中选择 `/circle_pixel_preview/image`。
数值状态：`rostopic echo /circle_pixel_preview/status`。

## 微调

参数在启动时读取。先停止本调试 launch，再用新的值启动；下列示例仍为当前测量值：

```bash
roslaunch robot_vision air_hover_pixel_debug.launch \
  offset_x_px:=53.25 offset_y_px:=15.484375 inner_radius_ratio:=1.0
```

例如 x 增加1，黄色参考点右移1像素；y 增加1，参考点下移1像素。
调整参考点应依据 SPOT-REF，不要仅为了让 REF INSIDE 而随意修改数值。
如果本 launch 原先使用 start_camera=true，新启动仍需带原相机参数。

## 验证范围

18项本地测试覆盖原图回放、偏移微调、边界、来源/分辨率不匹配、无靶、光斑丢失/多候选、过期与重复帧清除、节点回调和看门狗。
8张保存帧回放中 SPOT-REF 距离均<0.5像素，但参考点仍在最内环外。
颜色规则仅在本批正样本和合成负样本中检查，尚未完成现场各种反光背景验证。

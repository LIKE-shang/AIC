> **2026-10-05 手持三同心环测试更新**：USB/激光链路已改为三环、偏轴标定和连续 3 秒的一次性测试。请以 [AIR_TARGET_HANDHELD_CN.md](AIR_TARGET_HANDHELD_CN.md) 为操作入口；未确认直径定义/标定/硬件前仅 dry-run。本文 Gemini 部分保留。旧单圆 N/M 说明不再适用，不启动原矩形飞行演示。

# 独立 OpenCV 摄像头测试（不启动比赛任务）

本目录的测试只识别/显示图像；USB 测试另有可选 PWM 桥接器。**不是比赛任务改造**。

- 本地工程：`C:/Users/jiangzx/Desktop/src`
- 部署工作空间：`/home/abot/abot_ws`
- Gemini 结果：`/window_test/image`；边缘：`/window_test/edges`。
- USB 结果：`/circle_laser_test/image`；边缘：`/circle_laser_test/edges`。
- 激光决策：`/circle_laser_test/decision`；状态：`/circle_laser_test/laser_status`。
- 缺省 `enable_laser:=false`。dry-run **不发送任何硬件命令，包括启动/退出的 OFF**，只模拟应开/应关。
- 没有运动、位置、速度、解锁、模式、起飞、降落服务或发布器；不向 `/object_position` 发布。
- 两个测试的节点/输出命名空间互不重叠；不自动启动 MAVROS。真实桥固定节点名 `/circle_laser_bridge`，内部服务 worker 会再次检查该节点的 enable/接线确认参数；不要单独运行 `--command-worker` 或任意重命名真实桥。
- 不要同时运行旧任务、旧激光控制器或另一个争用同一摄像头/串口的驱动。测试不能阻止已在运行的其他节点运动。

## 1. 已核对的工程接口

### 1.1 原形状识别

阅读 `robot_vision/scripts/shape_detector.py`，复用其 `cv_bridge → bgr8 → 灰度 → GaussianBlur → Canny → findContours → approxPolyDP` 方法。
旧 `shape_detector.launch` 中 Haar/人脸参数与脚本不匹配，已去掉，替换为脚本实际读取的 `blur_kernel/canny_thresh1/canny_thresh2/epsilon_ratio` 及图像 remap。旧识别脚本本身不作改动，也不会被新测试启动。

新窗框检测要求：凸四边形、面积/边长/角度/长短边比/矩形填充度/对边变形满足阈值。普通路径要求有真正分离、包含于外框内的矩形内边缘；Canny 在同一条边产生的两条近乎重合轮廓不能充当空心证据。针对棋盘格边框，新增局部方差孔洞候选与四侧独立的亮暗/边缘/交替重复周期验证，保持 `require_inner_frame: true`，不放行任意四边形。棋盘路径的绿色轮廓和 OPENING W/H 表示内开口；普通配对路径显示外框。宽高均为**像素**。单张 RGB 不能证明真实可穿越空洞，矩形画框、屏幕、印刷图案仍可能相似，需现场验证。具体参数和补丁步骤见 [WINDOW_CHECKER_PATCH_CN.md](WINDOW_CHECKER_PATCH_CN.md)。

圆形检测组合轮廓圆度、外接圆填充率、椭圆轴比、径向离散度、面积/半径过滤和同心轮廓去重；可选 HoughCircles 作为额外验证。环形靶建议 `require_ring: true`，实心圆靶可设 false。不是二维码、AR、YOLO 或 Darknet。

### 1.2 Gemini Pro 的实际发布逻辑

ROS 包名是 **astra_camera**，磁盘目录名是 `ros_astra_camera-main`。

根据本工程源码（非真机 `rostopic` 结果）：

| 配置/源码 | 默认命名空间 camera 下的话题 |
|---|---|
| `gemini.launch` 缺省 `use_uvc_camera=true`，`uvc_camera_driver.cpp` 发布 `rgb/image_raw` | `/camera/rgb/image_raw` |
| UVC 彩色相机信息 | `/camera/rgb/camera_info` |
| `ob_camera_node.cpp` 的深度流 | `/camera/depth/image_raw`，需 `enable_depth=true` |
| 非 UVC 彩色流，`stream_name_[COLOR]="color"` | 可能为 `/camera/color/image_raw`，不是强行假定 rgb |
| `gemini.launch` 中 depth_registered 重映射 | **仅点云** `/camera/depth_registered/points`，不能据此断言存在 `/camera/depth_registered/image_raw` |

`gemini.launch` 的 `enable_depth` 当前是 **false**。本次矩形识别使用 **RGB**，不订阅深度，不测距，无需开启深度。相机包装 launch 支持显式启用深度和重映射话题，但不改动原驱动文件。

可选深度预览节点单独处理 `16UC1`（缺省毫米，乘 `0.001`）和 `32FC1`（米），先用固定米范围转 `uint8/mono8`；0/NaN/Inf 无效值显示黑色。该预览**不是窗框识别输入**，不要直接把原始深度 remap 给 RGB 检测器。源码深度配置为 1 mm 格式并发布 16UC1；部署版本和深度缩放仍需核对。原 gemini.launch 的 depth_format 参数为 Y11，驱动按其深度配置解释；如果真机报告不支持该流模式，应先检查驱动/固件支持的模式，不能凭预览图推断格式正确。

### 1.3 USB 摄像头

已读 `usb_cam/launch/usb_cam-test.launch` 和 `robot_vision/launch/usb_cam.launch`：旧配置固定 `/dev/video0`、640×480、yuyv。
`usb_cam_node.cpp` 使用私有 `NodeHandle("~")`，在节点私有空间发布 `image_raw`/`camera_info`。新包装节点名不同，故显式 remap 私有图像名到可配置的 `/usb_cam/image_raw`。
**/dev/video0 不是接线证明**。Gemini 的 UVC 彩色端也可能占用 video 设备，务必识别物理设备和端口。

### 1.4 PWM/激光核验（不是硬件接线核验）

已读以下文件，均不修改：

- `lib_library/include/lib_library.h`
- `uav_mission/src/uav_mission.cpp`
- `uav_mission/CMakeLists.txt`
- `uav_mission/lib/liblib.so`

头文件声明 `void lib_pwm_control(int pwm_channel_5, int pwm_channel_6)`，注释称辅助通道 5、6，参数 0–100 占空/位置单位。函数**没有返回值**。
原任务用 `mavros_msgs/CommandLong` 调 `/mavros/cmd/command`，但没有检查 `.call()`、`response.success` 和 `response.result`，所以打印“开启激光笔”并不能证明执行成功。

对交付二进制作只读 ELF/反汇编核验（不执行动态库）：

- 架构：AArch64 Linux，不能在本 Windows/x86 Python 中加载；C++ 在机器人侧加载。
- SHA-256：`e055c1e0468cc44c114c2ff408a6303bb9e0e9ece5fcf707fd3489dfb0e2734c`。
- 导出函数：`_Z15lib_pwm_controlii`；全局请求对象：`lib_ctrl_pwm`。
- 实际写入请求：`command=187`，`param1=p5/50.0-1`，`param2=p6/50.0-1`。
- `broadcast/confirmation/param3..param7` 不由函数设置，桥接器构造请求前清零，并在运行时检查已审计行为/布局。不自行替换为猜测的 servo 命令或微秒 PWM。
- 新桥接器调用**现有真实 C++ 函数**构造请求；Python 不调用 `.so`。C++ 不 include 定义运动全局对象的任务头文件，不链接任务可执行程序。

**必须现场确认，不能从仓库得出的信息：**

1. 辅助 5 或 6 哪一路连接激光，是否经过继电器/驱动板以及共地/电源。
2. 高/低电平有效、真实开光/关光值；`100`/`0` 只是旧代码用例，不是本测试缺省值。
3. 另一路是否接舵机/释放机构，安全保持值是什么。
4. 此飞控固件如何解释该命令、输出映射、是否允许锁定状态输出、failsafe 行为。`param3..param7=0` 在实际固件上的含义也需核对，不能据头文件认定只影响两根物理引脚。
5. 激光功率/安全操作范围，以及物理断电/急停/硬件失联关断措施。

旧 API 一次写**两路**，没有核实的“保持另一路不变”接口。桥接器只允许选 5 或 6，另一路必须填写已确认安全保持值。若另一路不能被写、或者没有安全保持值，就**不能使用当前双通道 API 做真实测试**；继续 dry-run，先由硬件/固件人员提供并验证单通道接口。不要猜 NaN、-1、1500 等“保持”值。

缺省 `laser_hardware.yaml` 的接线确认是 false，通道/值是 -1，真实启动会拒绝操作。参数或库自检失败时根本不建立硬件操作，不能声称这种启动失败已发送有效 OFF。**不能为通过校验而盲填参数**。

## 2. 部署与编译（机器人上执行）

**现有 Nano 会话已建好 `/home/abot/vision_camera_test_ws` 独立测试工作空间。只更新棋盘格补丁时遵循 WINDOW_CHECKER_PATCH_CN.md，不重跑下面首次部署的 catkin 命令，也不改变原工作空间白名单。**

只把 `robot_vision` 的本次代码同步到 `/home/abot/abot_ws/src/robot_vision`，不要用 Windows 的测试依赖覆盖机器人 Python。`.vision_test_deps` 不属于部署内容。

以下以 **ROS Noetic/Python3** 为示例。先核对：

```bash
ls /opt/ros
uname -m
source /opt/ros/noetic/setup.bash
python3 -c 'import rospy, cv2, numpy; from cv_bridge import CvBridge; print(cv2.__version__)'
```

若机器人是 Melodic/其他环境，必须先确认有与 ROS/相机图像消息兼容的 **Python3 cv_bridge**，不能盲用 Python2，也不能把 Windows wheel 安装进机器人；本地无法验证这项兼容性。

```bash
cd /home/abot/abot_ws
chmod +x src/robot_vision/scripts/window_test_node.py \
         src/robot_vision/scripts/circle_laser_test_node.py \
         src/robot_vision/scripts/depth_preview_test_node.py
# 只构建 robot_vision 及其构建依赖，不构建/运行完整任务。
catkin_make --only-pkg-with-deps robot_vision -DPYTHON_EXECUTABLE=/usr/bin/python3
source /home/abot/abot_ws/devel/setup.bash
```

相机驱动此前已可运行时不必重编；需要时另行构建 `astra_camera usb_cam`。`--only-pkg-with-deps` 的白名单可能保留在 CMake 缓存里，将来恢复全工程构建需明确传 `-DCATKIN_WHITELIST_PACKAGES=""`（本次不要求做）。编译不会启动任务。

每个新终端都执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/abot/abot_ws/devel/setup.bash
```

### 独立 ROS master 终端（两组测试共用）

为避免某个相机 launch 自动创建的 ROS master 随它退出而影响另一组，**先用单独终端 R0 启动 roscore 并保持运行**。若已有独立 roscore，不要再启动第二个。

```bash
roscore
```

R0 不属于任一测试的启动/关闭范围。关闭 Gemini 或 USB launch 不关闭 R0；这是两组可独立关闭的必要运行方式。两摄像头仍共享主机 USB 带宽/供电，物理总线故障不在进程隔离保证内。

## 3. Gemini 分终端测试

先停止旧任务/控制节点，确认机器人静止、飞控未解锁；不要为此运行比赛 TXT。先保持独立 R0 的 ROS master 运行。

**终端 G1：只开相机**

```bash
roslaunch robot_vision window_test_camera.launch camera_name:=camera enable_depth:=false
```

**终端 G2：先核对，再启动 RGB 窗框识别**

```bash
rostopic list
rostopic type /camera/rgb/image_raw
rostopic hz /camera/rgb/image_raw
# 确认后 Ctrl-C 退出 hz，再运行：
roslaunch robot_vision window_test.launch start_camera:=false image_topic:=/camera/rgb/image_raw
```

**终端 G3：查看输出（不是原图）**

```bash
rosrun rqt_image_view rqt_image_view /window_test/image
```

在 rqt 下拉框可切换 `/window_test/edges`。所有标注由 OpenCV 节点产生，rqt 只显示。

若不需要分开启动 G1/G2，可以单独运行：

```bash
roslaunch robot_vision window_test.launch
```

它只开 Gemini 和窗框识别；不要再重复开启 G1。

可选深度预览（不是验收必需项）：重启 G1 时加 `enable_depth:=true`，再另开终端：

```bash
rostopic list | grep depth
rostopic echo -n 1 /camera/depth/image_raw/encoding
rosrun robot_vision depth_preview_test_node.py _image_topic:=/camera/depth/image_raw \
  _uint16_scale:=0.001 _near_metres:=0.2 _far_metres:=5.0
rosrun rqt_image_view rqt_image_view /window_test/depth_preview
```

两个 `rosrun` 是两个前台进程，分别开终端执行。**不要假定 depth_registered/image_raw 存在**。

## 4. USB 手持三同心环/激光测试（新版入口）

完整分终端命令、现场 STOP-A/B/C/D 确认点、尺寸定义、偏轴标定、光斑/3 秒逻辑、截图/录包回放和三层验收表，见 [AIR_TARGET_HANDHELD_CN.md](AIR_TARGET_HANDHELD_CN.md)。

本次关键变化：

- `circle_usb_camera.launch` 必填现场确认的 `video_device`；不再暗示 `/dev/video0` 已核实。
- `circle_laser_test.launch` 默认 `start_camera=false`、`enable_laser=false`，只订阅先行启动并核验的 USB 图像。
- 默认模板尺寸定义未确认、偏轴为空；识别到三个环也不会开光。预测点不是图像中心假设，更不是物理光斑。
- 三环比例/环宽/椭圆/共心唯一性 + 标定/距离/姿态/安全有效区后，才允许 N 帧确认；`off_frames` 必须等于 1，首个坏帧撤销。
- 连续 3 秒计时分 prediction_only 与 camera 证据；中断清零并锁存。完成也关光，只能排错并重启独立测试再试。
- 真实桥保留启动 OFF/ACK/独立 watchdog/退出 OFF，增加库哈希、现场物理安全/光学确认和单次最大 5 秒保护。故障不再靠移走/放回靶子自动解除。
- 独立 `laser_mavros_only.launch` 不变：只通信，不发解锁/模式/飞行/运动命令，FCU URL 必须现场提供。
- 不允许用本文旧代码/其他文档中的 PWM、串口、分辨率或“开启激光”日志代替现场测量。

## 5. 检查实际节点、话题与隔离

```bash
rosnode list
rosnode info /window_test
rosnode info /circle_laser_test
rosnode info /circle_laser_bridge    # 只在真实模式存在
rostopic list
rostopic hz /camera/rgb/image_raw
rostopic hz /window_test/image
rostopic hz /usb_cam/image_raw
rostopic hz /circle_laser_test/image
rostopic echo /window_test/status
rostopic echo /circle_laser_test/detection_status
rostopic echo /circle_laser_test/laser_status
rostopic echo /circle_laser_test/decision
```

`hz`、`echo` 各自会占据终端，用 Ctrl-C 退出或分终端运行。输入配置改变后，命令里的图像话题也须相应替换。

核对测试节点 Publisher/Subscriber/Service 列表：不能有 `/object_position`、`/cmd_vel`、`/ctrl_mode`、`/move_base/goal`、`/mavros/setpoint_raw/local` 或解锁/起飞/降落/模式/导航服务客户端。rqt/相机发布图像不产生运动。

```bash
rostopic info /cmd_vel
rostopic info /ctrl_mode
rostopic info /object_position
rostopic info /move_base/goal
rostopic info /mavros/setpoint_raw/local
```

不存在这些话题是正常的。若存在，定位 Publisher 是否来自未停止的旧节点；本测试不能代表其他节点的安全性。

## 6. 参数与故障定位

手持三环参数和联锁以新版手册为准。`on_frames` 默认 5，`off_frames` 固定 1，默认图像超时 0.35 秒，watchdog 25 ms/10 ms 检查；不再允许 M 帧宽限。不要降低时间戳、尺寸/标定或物理安全检查来点亮。

参数 YAML 在启动时读取。修改 YAML 后重启相应识别 launch（保持 dry-run）；**不要期待运行中 rosparam set 自动重载算法阈值**。可通过 `config_file:=/绝对路径/自定义.yaml` 指定配置。

- **矩形误检**：提高 `min_area/min_side/min_rectangularity`，减小 `max_angle_cosine/max_opposite_side_ratio`，缩紧长短边比。保持 `require_inner_frame:true`，用边缘图核对内/外框而非两条重复边。大幅透视可能被拒绝，这是测试过滤边界，不提供姿态估计。
- **矩形漏检**：确认画面真是 RGB；观察 `/window_test/edges`，调整 Canny 与奇数 `blur_kernel`、`epsilon_ratio`。细窗框需要更接近 1 的 `max_inner_area_ratio`，但过高会误把 Canny 双边当空心；先离线验证。
- **三环误检/反光**：检查真实原图的三组材料边界、环宽/比例/共心、椭圆残差与额外候选；默认未知尺寸只预览。光斑检测先用 OFF/ON 原图验证。详见新版手册，不得切回单圆逻辑来开光。
- **设备选错/被占用**：比较 `v4l2-ctl --list-devices`、by-id/by-path 和画面；检查 `fuser /dev/videoX`，只停止自己确认的冲突进程；不要同时打开 Gemini UVC 与同一个 USB video 节点。
- **图像无输出**：先检查 input `rostopic type/hz`，再查 `rosnode info` 实际订阅；检查 video 权限/分辨率/pixel_format，camera_name/remap，Python3 cv_bridge。跨主机 ROS 还需 ROS_MASTER_URI/ROS_IP 和时间同步。结果 rqt 话题不是原图话题。
- **深度格式错误**：RGB 节点拒绝 16UC1/32FC1 是正确行为。启用相机深度流，单独预览，检查 encoding/dtype 和单位；不要直接 `astype(uint8)` 截断毫米深度。没有订阅者时驱动可能按需停止流，`rostopic hz`/预览会建立订阅。
- **激光服务失败**：看 bridge 的 transport/success/result/error；核对唯一 MAVROS、FCU 连接、AArch64 库与 SHA/ABI、串口、命令插件、已确认的接线和值。拒绝/超时后状态 UNKNOWN，不宣称关光成功。输出不动作不可用解锁解决。
- **目标消失未关**：先查检测状态是否仍误检、图像超时与物理链路延迟，再看 decision 是否 OFF、bridge watchdog/ACK/error。输入断流时不能等下一张图，两个 wall-clock watchdog 分别保护。若 OFF 已接受但光仍亮，排查开关极性、错误接线/通道或另一个控制器覆写，立即物理断电。
- **启动后一直不允许 ON**：可能尚未成功 OFF、接线未确认、帧重复/时间戳为零/过期、N 尚未满足、目标不唯一，或故障锁存。先先物理关断并排错，重新核验后重启独立测试；不降低保护绕过故障。

## 7. 关光保证的范围（必须理解）

识别端使用 `time.monotonic()` 的独立线程；C++ 桥接端有独立 steady-clock 看门狗，不依赖 `/clock` 推进。可处理摄像头正常断流、识别异常、识别进程被杀、常规 Ctrl-C/SIGTERM 和可捕获异常。

ROS 服务不能保证物理电平。**SIGKILL、主机掉电/死机、USB/串口断线、MAVROS/飞控失联、失灵的驱动板，或 OFF 被固件拒绝时，任何主机程序都不可能保证发送/执行 OFF**。服务阻塞时桥接器记录未知状态、终止超时调用并尝试 OFF；已经发往飞控的命令不能被软件撤销。停止整个 launch 时 MAVROS 若同时消失，也可能收不到 OFF。因此真实测试应让独立 L1 通信继续运行，先停 L2 并检查 OFF ACK/物理熄灭，再关 L1。

对这些故障需要常断型/失联自动关断硬件、飞控侧看门狗或独立物理断电。不可把本测试当作安全认证联锁。验收中的“节点退出发送 OFF”仅针对可处理退出以及通信仍可用的条件；绝不宣称能保证 kill -9 后单进程继续执行代码。

## 8. 验收清单（真实机器人待执行）

- [ ] Gemini 窗框进入画面：外框/内框、中心、像素宽高与 FOUND 状态正确；非矩形/噪声被过滤。
- [ ] Gemini 检测节点仅图像/状态发布；整个识别过程机器人完全不动作，也不控制激光。
- [ ] USB 三环/偏轴/光斑/计时/硬件的完整验收表见 AIR_TARGET_HANDHELD_CN.md，必须区分离线、ROS 真机、物理命中。
- [ ] 连续 N 帧有效才应开；N 未满或确认被打断均不应开；默认 N=5。
- [ ] 第一帧丢靶/越界/光斑丢失即请求 OFF 并取消计时；断流由独立 watchdog 撤销。
- [ ] 输入相机停止：无需下一帧，按配置超时发 OFF。可用 `rosnode kill /circle_usb_camera`；先在 dry-run 测，真光时准备物理断电。
- [ ] 单独结束识别进程：bridge 仍运行时超时 OFF；Ctrl-C 结束测试 launch：bridge 退出 OFF（先保持 MAVROS L1 运行）。
- [ ] 真实模式：启动第一条是 OFF，服务拒绝/断开后不继续开光，OFF 重试和 UNKNOWN 状态可观察。
- [ ] OFF/ON 的 ROS ACK 与物理光、电平分别观察记录；确认未改动另一路机构。
- [ ] 全程 `armed=false`，无起飞/降落/模式切换、运动/位置/速度控制。
- [ ] 保持独立 roscore 终端 R0，同时启动两组测试；只共用 ROS master，不共享结果话题。关闭任意一组不结束另一组，且不关闭 R0。

离线回归测试见 `test/`；实际执行记录及文件清单见本目录 `TEST_REPORT.md` 和 `FILE_MANIFEST.md`。Windows 离线通过不等于 catkin 编译或真机通过。

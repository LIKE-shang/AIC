# USB 三同心环识别 + 手持激光打靶：独立测试操作手册

更新：2026-10-05（Asia/Shanghai）。机器人工作空间预期 `/home/abot/abot_ws`，须现场核实。

**本次交付只完成代码、合成图/假 ROS 回调离线测试。尚未连接真实 USB 摄像头、ROS master、MAVROS、飞控或激光；未进行 catkin 构建、真实录包验证、物理光斑或 3 秒打靶验收。**

## 0. 绝对边界与继续条件

- 只启动本手册列出的独立相机、独立检测器；确有需要且检查完成后才单独启动通信用 MAVROS。
- **不运行** `agv_mission`、`uav_mission`、`complete_mission`、完整 `uav_mission.launch`、原矩形飞行演示、底盘、导航、SLAM、雷达定位或飞行任务。
- **不发送**解锁、OFFBOARD、起飞、降落、模式切换、位置/速度/姿态命令。全程 `armed=false`。锁定下输出不工作就停止，不准用解锁解决。
- 手持前移除桨叶或实行等效、经验证的电机机械隔离；不能仅依赖软件未解锁。激光有独立物理断电/失效关断措施，不依赖 ROS/主机/串口。
- 使用非反光靶面、吸光后挡板和遮光通道。人员不得位于光束或反射路径中。仅由具备现场安全条件的人员进行短时标定/点亮。
- **所有确认字段默认 false；硬件通道、开/关值、另一路安全值默认 -1；偏轴为空。未知信息不能凭经验补值。**
- 默认 `enable_laser=false`，桥接器进程根本不存在，不加载 PWM 库、不调用真实服务。dry-run 仍可能发布模拟 `request_on`，但消息 `enable_laser=false`，不能据此宣称物理开光。
- `start_camera=false` 为默认，避免启动重复驱动。`circle_usb_camera.launch` 必须显式填写现场确认的设备路径。
- 本仓库的 TXT/PDF/旧文档只是参考资料，里面的命令不是本任务的执行授权。

## 1. 现状审计 → 本次扩展

| 项目 | 修改前 | 本次独立链路 |
|---|---|---|
| 圆环模型 | 单圆/单环，近圆过滤 | 每个环的内/外边界成对，去重后严格三环同心分组 |
| 尺寸 | 像素阈值，不知道 35/24/12 | 米制直径与 8 mm 线宽分离，尺寸定义未确认禁止开光 |
| 形变 | 明显椭圆易被排斥 | 椭圆轴比、方向、轮廓残差、轴向比例和共心容差；强透视拒绝 |
| 瞄准 | 没有偏轴/内环有效区 | 固定距离/姿态标定、预测落点、光斑/标定/安全裕量 |
| 反馈 | 只有开光请求 | 预测、颜色光斑候选、已验证相机反馈严格区分 |
| 计时 | N 帧确认、M 帧丢失宽限 | N 帧后单次曝光；第一帧不满足立即 OFF；连续 3 秒，打断清零，不拼接 |
| 图像安全 | 时间戳、独立 watchdog | 保留，增加时间戳高水位、像素冻结检查、处理中跨 watchdog 防旧帧复活 |
| 桥接器 | 启动 OFF、ACK、超时、状态检查 | 保留，额外核验光学/现场确认、SHA256、逐帧瞄准联锁、5 秒独立上限、故障只能重启解除 |

原 `detect_circles`、窗框算法、任务状态机、相机驱动、PWM `.so` 未修改。旧 `ConfirmationGate` 仅保留给原有回归，**手持节点改用 `ContinuousHitGate`，不支持丢靶宽限**。

### 已核对的源码事实，不等于现场测量

- `usb_cam/nodes/usb_cam_node.cpp` 使用私有 NodeHandle 发布 `image_raw`，独立包装 remap 到配置话题。
- `usb_cam/src/usb_cam.cpp` 的 `grab_image(sensor_msgs::Image*)` 在抓图后调用 `ros::Time::now()`，**不是传感器硬件曝光时间**。底层 `read_frame()` 的返回值未用来决定上层是否重新发图；发布端新时间戳不能单独证明内容新鲜。本次没有修改驱动。
- 检测器检查帧龄、未来/重复/倒退时间戳、处理后帧龄、断流和超过 0.25 秒完全相同像素。后者保守拒绝静态逐字节重复画面，但不能排除加噪/交替重放，也不能测出全部相机缓存延迟。
- `v4l2` 可能协商不同格式；必须对照设备实际格式、ROS 图像尺寸和原图，不能仅相信参数。
- 只读 ELF/反汇编再次核对现有库：AArch64，`lib_pwm_control(int,int)` 构造 `command=187`、`param1=p5/50-1`、`param2=p6/50-1`；**一次写两路**。这不是激光接线/极性/固件映射的证据。
- 已审计库 SHA256：`e055c1e0468cc44c114c2ff408a6303bb9e0e9ece5fcf707fd3489dfb0e2734c`。桥接器在 `dlopen` 前用 OpenSSL 校验此哈希，之后保留 ABI/行为自检。版本不同必须重新审计，不提供“跳过哈希”开关；部署后不要在运行中更换库文件。

## 2. 首先确认直径定义，不能默认为中心线

**当前仍未知：35、24、12 cm 分别是环外径、环中心线直径，还是环内径。请向制靶者确认或用尺测量环的内外边界并保留照片。8 mm 是环线/环材径向宽度，不是三个环的直径。**

`diameter_definition` 允许 `outer / centerline / inner`，缺省 `unconfirmed`；只有实际确认后才填 `dimensions_confirmed:true`。可显示未确认的几何预览，但不允许开光。

令标称直径为 D，实测环线宽为 w：

| 直径定义 | 轮廓配对后的中心线半径 | 内环未涂线区域半径（D=12 cm、w=8 mm，仅按给定尺寸计算） |
|---|---|---|
| outer（外径） | (D-w)/2 | 52 mm |
| centerline（中心线） | D/2 | 56 mm |
| inner（内径） | (D+w)/2 | 60 mm |

三环比例以中心线直径校正后比较；未确认预览采用名义 `1 : 24/35 : 12/35`，不能当测量结论。

有效半径：

```text
R_effective = min(effective_radius_cap_m,
                  R_unpainted - measured_spot_radius_m
                              - calibration_error_bound_m - safety_margin_m)
```

缺省 `effective_radius_cap_m=0.045`、`safety_margin_m=0.005` 是可调软件保守量，不是硬件测量。光斑半径/误差均未填时拒绝开光。以实测内边界椭圆按 `R_effective/R_unpainted` 缩小，预测点和实际光斑中心都必须严格位于其中，**边界不算安全**。没有硬编码 `(320,240)` 判分；像素参考中心为 `((width-1)/2,(height-1)/2)`。

## 3. 识别、状态和限制

### 3.1 三环判别

对黑环/浅底（可配置 `polarity:bright`）二值分割，`RETR_TREE` 配对材料外边界与孔内边界；拟合椭圆，检查残差、面积填充、环宽、共心、轴比/方向与物理直径比例。环线两边不当成两个目标。唯一三环组且没有额外可疑完整环才合格；两个完整靶、缺环、额外环、触边、比例不符、低对比或严重形变不得 ON。内环内部可见深色/异色遮挡占据过多区域也拒绝（`max_inner_obstruction_fraction`）；与底色完全相同的遮挡不能凭图像保证识别。

- `ratio_tolerance` 是两个轴向半径比的**绝对容差**，不是百分比置信概率。
- `center_tolerance` 乘外环短半轴；`min_axis_ratio`、方向差、环宽偏差均可调。
- 适配中等椭圆/小幅透视；**不是任意透视单应标定或三维测距**。部分遮挡不补出不可见环；轮廓不充分就 OFF，不放宽保护以强行点亮。微小遮挡/完全不可见的第二靶无法靠图像保证辨认，现场应确保单靶、受控背景。
- 不同靶位置/尺度跳变清除确认；ON 后任何不满足都锁定 OFF。外观、位置和尺度完全相同的替换无法可靠区分身份，不声称目标 ID 追踪。

### 3.2 图像/数值输出

| 话题 | 内容 |
|---|---|
| 实测 USB 原图话题，默认映射 `/usb_cam/image_raw` | 唯一真实输入；保留原 header |
| `/circle_laser_test/image` | 外/中/内环两条边界、共同中心、黄色有效区、蓝色预测点、红色实测光斑候选、误差/几何评分/请求/桥状态 |
| `/circle_laser_test/edges` | 当前实现的环材料二值掩膜（名字沿用，不再是 Canny 边缘） |
| `/circle_laser_test/decision` | 带原图 header 的意图、目标/标定/瞄准联锁、连续时间、反馈模式 |
| `/circle_laser_test/detection_status` | JSON：输入话题/尺寸/帧号/时间戳/帧龄/处理耗时、三椭圆、预测像素/归一化误差/有效半径、光斑候选、计时/锁存/结果 |
| `/circle_laser_test/laser_status` | dry-run 状态，或桥接器 ACK、FCU 状态、错误/故障锁存；`hardware_light_verified` 不伪造为 true |

`geometry_score`（内部字段 `confidence`）只是比例拟合评分，不是统计置信概率。无有效标定不画假想“实际激光点”；预测与 observed 候选的颜色、标签不同。断流警告图是黑底，沿用旧 header，不伪装新相机图。

### 3.3 3 秒与反馈模式

- `prediction_only`：只有标定预测，真实模式还要求 `prediction_only_acknowledged:true`；必须由安全位置的观察者/靶面录像记录实际光斑。输出 `predicted_only_3s`，**不是命中**。
- `camera`：必须现场验证 `spot.validated:true` 和实际红/绿颜色参数。按 HSV 色彩/亮度/面积/形状检测；全图唯一候选、大小未超标、与预测偏差不超过标定误差、落在有效区才有效。纯白饱和点、看不见、多个彩色反射点都不伪造反馈。
- OFF 时若已有类似光斑的彩色物体，拒绝 ON。启动后只在收到 ON ACK 且图像 header 晚于 ACK 接收 ROS 时间时计入；ACK 本身不算物理证据。
- 首次相机光斑获取容许最多 0.30 秒（只允许“尚未看见”，不计命中）；看到越界/多个候选立即 OFF，不享受宽限。一旦进入可见命中计时，再丢一帧光斑就 OFF。该初始短窗口也必须有物理隔离和独立断电。
- 默认稳定 5 帧三环+标定+瞄准后请求 ON；保持时间同时受单调墙钟和递增图像时间约束，取较小值。间隔达到 `image_timeout` 也会打断，不把片段相加。
- 完成后立即请求 OFF 并锁存，输出 `camera_observed_3s` 或 `predicted_only_3s`；`completed_seconds` 和 `completed_capture_interval` 保留历史事件，活动 `continuous_seconds` 清零。历史结果不表示当前仍在命中。
- 每次 ON 最长 5 秒；Python 和 C++ 各自独立限制，包含等 ACK 时间。检测器 watchdog 25 ms 检查，桥 10 ms 检查；默认断流 0.35 秒、静帧 0.25 秒。首次坏帧处理结束立刻发 OFF；通信/调度/驱动延迟另计，不是硬实时关光保证。
- ON 后中断、异常或完成均不自动再点亮。只有先物理确认熄灭、排错、检查安全并**重启本独立测试 launch**才开始新一轮。不得运行中改参数绕过故障锁存。
- `camera_observed_3s` 是配置已校验条件下的相机证据，仍需真实靶面验收；dry-run 合成相机证据不是物理证据。离散图像不能证明任意两帧之间绝无瞬时越界。

## 4. 机器人部署/编译（只在机器人执行；本机未执行）

保留机器人原工程备份，只同步本次 manifest 中 `robot_vision` 的文件，不覆盖任务/驱动/库。消息定义改变，检测器和桥必须一起重新构建，不能混用旧 `LaserDecision`。代码要求 Python3 的 ROS1 环境；以下不猜发行版。

```bash
# R-build：填写实际安装环境，不要复制未核实的发行版名称
: "${ROS_SETUP:?先设置 ROS_SETUP 为实际 /opt/ros/<发行版>/setup.bash}"
source "$ROS_SETUP"
cd /home/abot/abot_ws
test -d /home/abot/abot_ws/src/robot_vision
rosdep check --from-paths /home/abot/abot_ws/src/robot_vision --ignore-src
# 需要 OpenSSL 开发头/libssl-dev、Python3 cv_bridge/OpenCV/numpy/yaml、MAVROS 消息等。
# 缺依赖先由操作员确认安装；本机没有替你安装机器人依赖。
catkin_make --only-pkg-with-deps robot_vision
source /home/abot/abot_ws/devel/setup.bash
python3 /home/abot/abot_ws/src/robot_vision/test/test_vision_tests.py
```

若 `usb_cam` 尚未在机器人正确构建，单独处理其构建依赖；不能通过启动完整比赛任务解决。AArch64 桥的编译/链接/ABI、实际 MAVROS 版本均待真机检查。本测试不会在 Windows 加载该 `.so`。

## 5. 分终端启动，严格依序

**每个新终端先加载已核实的 ROS 环境和 `/home/abot/abot_ws/devel/setup.bash`。当前 ROS 图里如果有其他运动/任务/飞行控制节点，先由操作员停止并确认，不能把本测试的隔离当成整个 ROS 图安全。**

### 第一阶段：只开 USB 相机（STOP-A：确认设备/图像事实）

**终端 R0：独立 master**（已有经检查的唯一 master 则不重复启动）

```bash
source /home/abot/abot_ws/devel/setup.bash
roscore
```

**终端 U0：只读设备核查**

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/ /dev/v4l/by-path/
# 用物理镜头/设备标识确认后，设置 CAM_DEVICE，绝不默认 /dev/video0。
: "${CAM_DEVICE:?填写已核实的 /dev/v4l/by-id/... 或实际 /dev/videoX}"
v4l2-ctl -d "$CAM_DEVICE" --list-formats-ext
v4l2-ctl -d "$CAM_DEVICE" --all
```

**终端 U1：相机 ONLY**。先从支持格式选出明确的 `CAM_WIDTH/CAM_HEIGHT/CAM_FPS/CAM_FORMAT`，不要把下列变量当作已测量值。

```bash
source /home/abot/abot_ws/devel/setup.bash
: "${CAM_DEVICE:?现场确认}" "${CAM_WIDTH:?现场确认}" "${CAM_HEIGHT:?现场确认}" \
  "${CAM_FPS:?现场确认}" "${CAM_FORMAT:?现场确认}"
roslaunch robot_vision circle_usb_camera.launch video_device:="$CAM_DEVICE" \
  image_width:="$CAM_WIDTH" image_height:="$CAM_HEIGHT" framerate:="$CAM_FPS" \
  pixel_format:="$CAM_FORMAT" image_topic:=/usb_cam/image_raw
```

**终端 U2：实际图像核验与原图留档**

```bash
source /home/abot/abot_ws/devel/setup.bash
rosnode info /circle_usb_camera
rostopic list
rostopic type /usb_cam/image_raw
rostopic info /usb_cam/image_raw
rostopic echo -n 1 /usb_cam/image_raw/header
rostopic echo -n 1 /usb_cam/image_raw/width
rostopic echo -n 1 /usb_cam/image_raw/height
rostopic hz /usb_cam/image_raw
# hz 用 Ctrl-C 结束后再继续；参数期望值不替代上述测量。
v4l2-ctl -d "$CAM_DEVICE" --get-fmt-video --get-parm
python3 /home/abot/abot_ws/src/robot_vision/tools/audit_air_camera.py \
  --topic /usb_cam/image_raw --seconds 10 --save-every 1 \
  --output /home/abot/abot_ws/air_camera_audit
```

保存 `audit.json`、`all_headers.json`、原图和 `manifest.json`。核对唯一发布者、图像格式/尺寸、真实 FPS、最大间隔、零/重复/倒退时间戳和帧龄。`--save-every 1` 会有磁盘负载，先关光执行；磁盘太慢要减小保存频率并说明样本缺口。

在关光状态，移动一张非反光标记/短暂遮盖镜头，核对画面跟随和延迟，观察冻结保护。不要用人体进入将来光路作测试。无法证明画面实时/延迟稳定就不填 `live_camera_verified`。仅靠驱动 `ros::Time::now()` 或 `rostopic hz` 不能完成此确认。

### 第二阶段：dry-run（STOP-B：尺寸、场景、偏轴均需检查）

**终端 D1：检测，不启动 MAVROS/桥**

```bash
source /home/abot/abot_ws/devel/setup.bash
roslaunch robot_vision circle_laser_test.launch start_camera:=false enable_laser:=false \
  image_topic:=/usb_cam/image_raw
```

模板只允许三环识别预览，不会通过未确认的标定/尺寸联锁。先检查正确三环、缺一个环、两靶、偏斜、遮挡、反光、帧中断和目标跳变。

**终端 D2：图像**

```bash
rosrun rqt_image_view rqt_image_view /circle_laser_test/image
```

**终端 D3：数值及图隔离**

```bash
rostopic echo /circle_laser_test/detection_status
# 另开终端观察下面任一项：
rostopic echo /circle_laser_test/decision
rosnode list
rosnode info /circle_laser_test
```

dry-run 图中不应存在 `/circle_laser_bridge`；检测器不能有任何 PWM 服务客户端。标注图/日志必须把 predicted 和 observed 分开。先在 dry-run 用 Ctrl-C 停 U1 验证断流 OFF，之后重启相机；不要从真实点亮开始测试未知失效模式。

### 第三阶段：固定距离偏轴标定（仍不可直接进入自动真光）

本检测器**不提供未标定时的“强制开光”后门**。由具备条件的操作员在封闭/吸光光路、拆桨/电机隔离下，用已核实的低风险、独立硬件瞬时按钮/受限电源短时点亮激光做测量；先验证其断电手段。若没有这样的安全标定设施，停在 dry-run，请提供独立测得的标定，不调用裸 PWM 服务绕过桥保护。

1. 固定摄像头与激光相对安装，靶板距离、姿态、曝光、增益、焦距、分辨率和裁剪。记录 by-id 与 `camera_frame_id`。禁止手持晃动状态下估算固定偏轴。
2. 同距离/姿态拍 **OFF → 短时 ON → OFF** 原图，多次重复（至少 5 组），标记光斑中心 `(x,y)`、包含完整光斑的上界半径。图像中心按 `(W-1)/2,(H-1)/2` 定义，偏移 `dx=x-center_x, dy=y-center_y`（右正、下正）。看不清光斑时由靶板标记/另一台安全观察相机确定，不能填零。
3. 用标定板已知长度求保守的 `pixels_per_m_min`；记录三环外环中心线长半轴 `outer_major_px`、椭圆轴比/角度（可从回放 JSON 读取）。`measurement_error_px` 是点位/半径测量误差；`additional_error_m` 要覆盖靶形拟合、镜头畸变、安装松动、姿态/距离允许范围内的剩余误差，不能凭“应当很准”填零。
4. 复制 `config/air_target_calibration_samples.template.json`，填实测数据，`samples` 至少五项，每项字段见模板。不同距离分开文件/配置，**当前实现不插值、不根据三环大小伪造实时距离**。
5. 生成待复核配置：

```bash
python3 /home/abot/abot_ws/src/robot_vision/tools/calibrate_air_target.py \
  --measurements /home/abot/abot_ws/air_target_measured_samples.json \
  --output /home/abot/abot_ws/air_target_verified.yaml
```

工具输出仍是 `confirmed:false`、`dimensions_confirmed:false`、`live_camera_verified:false`。它计算平均像素偏移、最大实测离散残差加测量/额外误差、最大光斑半径上界；输出文件名中的 verified 不构成验证。人工复核并在独立未参与拟合的样本上验证之后，才填上述确认字段、真实 `diameter_definition`、`working_distance_m`、`operating_envelope_confirmed`。

6. 限定工作距离和姿态，例如现场定位杆/尺度参照或独立测距；手持移动超出标定容差必须停光、重新定位或换已验证距离档。运行时还用外环尺寸、轴比/角度做拒绝性核查，但这些不是距离/姿态测量的替代。
7. 若启用 `feedback_mode:camera`，用真实 OFF/ON 数据测试色阈值、饱和点、面积、强反光、多光斑、越界/消失和颜色背景；无法可靠识别则保持 `prediction_only`，准备人工靶面记录。不能让 ON ACK 自动变为 observed hit。
8. 用实测配置重启 dry-run，先验证预测落点、有效区、尺度/距离联锁及错误输入 OFF：

```bash
roslaunch robot_vision circle_laser_test.launch start_camera:=false enable_laser:=false \
  image_topic:=/usb_cam/image_raw config_file:=/home/abot/abot_ws/air_target_verified.yaml
```

### 第四阶段：接线核验与独立 MAVROS（STOP-C：必须现场签字/记录）

确认激光到底接辅助 5 还是 6、开值/关值/极性、另一路设备和**安全保持值**、飞控硬件/固件及锁定状态的输出行为。旧函数会写两路，没有“另一通道不变”的已验证选项；不能把另一路擅自设为 0。`param3..param7=0` 的固件含义也要检查；不能凭注释假定只影响两根物理引脚。

```bash
cp /home/abot/abot_ws/src/robot_vision/config/laser_hardware.yaml \
   /home/abot/abot_ws/laser_hardware_verified.yaml
sha256sum /home/abot/abot_ws/src/uav_mission/lib/liblib.so
```

填写 `hardware_wiring_confirmed`、`laser_channel`、`laser_on_value`、`laser_off_value`、`other_channel_safe_value` 和新增 `physical_isolation_confirmed/independent_cutoff_confirmed/output_mapping_confirmed`。通道值单位是已审计 API 的 0–100，不是微秒。无法确认另一设备安全值或固件行为，不得进行真实测试。

**终端 M1：确有必要时只开 MAVROS 通信**。串口/波特率/目标系统组件都来自现场核对，不沿用旧任务示例。若已有经过核查的唯一通信实例则不重复启动。

```bash
source /home/abot/abot_ws/devel/setup.bash
: "${FCU_URL:?必须填写现场核实的飞控连接URL，不提供猜测串口}"
: "${FCU_SYSTEM:?实测目标系统ID}" "${FCU_COMPONENT:?实测组件ID}"
roslaunch robot_vision laser_mavros_only.launch fcu_url:="$FCU_URL" \
  tgt_system:="$FCU_SYSTEM" tgt_component:="$FCU_COMPONENT"
```

该 launch 只包含 MAVROS 通信，插件白名单 `sys_status`、`command`。部署版 MAVROS 的 launch/plugin 配置仍须现场核实；不启动任何 setpoint/飞行演示来“帮它连上”。

**终端 M2：只读核验**

```bash
rosservice type /mavros/cmd/command
rossrv show mavros_msgs/CommandLong
rostopic echo /mavros/state
# 持续观察 connected:true、armed:false；然后 Ctrl-C 结束 echo。
rosnode list
rosnode info /mavros
```

检查当前 ROS 图没有任务、控制或重复 PWM 节点。服务存在不等于可安全输出；**没有点亮前必须有可操作的独立断电措施**。

### 第五阶段：真实激光（STOP-D 后由操作员手动执行）

先结束 D1，保留 U1/R0/M1；检查唯一检测器/桥、已核实目标在受控光路内、人员安全、拆桨/隔离、电机不会旋转、全程未解锁。**这一步会产生真实硬件服务调用，不属于 dry-run。**

**终端 L1：只启动独立检测 + 桥**

```bash
source /home/abot/abot_ws/devel/setup.bash
roslaunch robot_vision circle_laser_test.launch start_camera:=false enable_laser:=true \
  image_topic:=/usb_cam/image_raw \
  config_file:=/home/abot/abot_ws/air_target_verified.yaml \
  laser_config_file:=/home/abot/abot_ws/laser_hardware_verified.yaml \
  pwm_library:=/home/abot/abot_ws/src/uav_mission/lib/liblib.so
```

1. **启动 L1 时先移走靶纸或让镜头不对准靶，使识别无效，防止核验前自动满足 ON。**先核对启动 OFF 的 `transport/success/result`、`OFF_ACKNOWLEDGED` 和**实际无光**，另一路设备保持安全。不能只看 ACK。
2. 手动抬起并缓慢对准固定单靶；不需要车体自行运动。满足全部联锁后只做一次最长 5 秒曝光；3 秒达标或任意中断后 OFF 并锁存。
3. 分别记录 `ON/OFF_PENDING / ACKNOWLEDGED / UNKNOWN`、相机 predicted/observed、实际靶面位置/3 秒录像，见验收表。
4. `OFF_ACKNOWLEDGED` 而仍有光，立即物理断电并停止排查；不得盲目反转极性继续试。任何故障都不得解锁。
5. 停止顺序：先物理确认熄灭/切断激光 → Ctrl-C L1（保留 MAVROS 让桥尽力多次 OFF）→ 核对状态/实际熄灭 → 再停止 M1/U1/R0。普通退出会尝试 OFF，强杀/掉电/失联不能保证执行，依赖独立失效关断。

## 6. 真实截图和录包回放接口

合成测试图/配置集中在 `test/artifacts/air_target`，标定 ID 为 SYNTHETIC，不能作为机器人实测配置。生成的合成配置保持 `live_camera_verified:false`，不能用于真实激光启动。

只读回放工具从不发布 ROS 消息，不开硬件、不执行袋内控制话题，只分析指定图像话题。它**不提供连续 3 秒/物理命中结论**，`request_on` 永远 false。实际节点的计时由独立单元和假 ROS 回调测试，以及现场实测覆盖。

```bash
# Windows/Linux 均可：原始截图（不是带标注截图）
python3 /home/abot/abot_ws/src/robot_vision/tools/replay_air_target.py \
  --input /home/abot/abot_ws/air_target_raw.png --frame-id '现场真实frame_id' \
  --config /home/abot/abot_ws/air_target_verified.yaml \
  --output /home/abot/abot_ws/air_target_replay

# audit_air_camera.py 生成的真实时间戳清单
python3 /home/abot/abot_ws/src/robot_vision/tools/replay_air_target.py \
  --manifest /home/abot/abot_ws/air_camera_audit/manifest.json \
  --config /home/abot/abot_ws/air_target_verified.yaml \
  --output /home/abot/abot_ws/air_target_manifest_replay

# 录包终端 B1：仅记录指定图像/测试状态/飞控状态；不要用 rosbag record -a
rosbag record -O /home/abot/abot_ws/air_target_test.bag \
  /usb_cam/image_raw /usb_cam/camera_info /circle_laser_test/decision \
  /circle_laser_test/detection_status /circle_laser_test/laser_status /mavros/state

# 只读 bag 回放：需要机器人侧 ROS1 rosbag + cv_bridge，不启动 roscore/rosbag play
python3 /home/abot/abot_ws/src/robot_vision/tools/replay_air_target.py \
  --bag /home/abot/abot_ws/air_target_test.bag --topic /usb_cam/image_raw \
  --config /home/abot/abot_ws/air_target_verified.yaml \
  --output /home/abot/abot_ws/air_target_bag_replay
```

单张图没有可信时间戳，报告明确 `synthetic_clock:true`；manifest/bag 保留 capture/received 时间，检测重复、过期、未来时间戳。输出逐帧标注/掩膜与 `report.json`，所有报告 `physical_hit_verified:false`。本机尚无真实 bag，也无 ROS Python 包，真实 bag 路径尚未实跑。禁止把 bag 发布到仍挂着真实激光桥的 ROS master。

## 7. 需要操作员提供的实测数据

- [ ] 35/24/12 cm 的外径/中心线/内径定义、实际三直径和环材宽度，带尺照片及规则/制靶者确认。
- [ ] 未标注原图和视频/图像 bag：清楚单靶、缺环、两靶、透视、各方向遮挡、反光、不同曝光；不能只有截图框线。
- [ ] 设备厂商/序列号、稳定 by-id、实际 `/dev/videoX`、分辨率/编码/FPS、图像真实话题/frame_id、`audit.json` 与延迟/冻结检查结果。
- [ ] 工作距离及容差、靶面姿态、相机/激光刚性安装状态、对焦/曝光/增益/裁剪，是否改变后需重标。
- [ ] 至少五组 OFF/短时 ON/OFF 原图、光斑中心/半径/误差、板上像素尺度、外环半长轴/轴比/角度、独立验证样本；如有多个距离，各自一份。
- [ ] 激光型号/光色/功率等级和风险控制条件，USB 是否可靠看到完整光斑；不能看到时的独立观察记录方法。
- [ ] 辅助 5 或 6 接线照片、开/关值/极性、另一路设备和安全保持值；所有命令字段的固件映射、锁定输出行为。
- [ ] 机器人架构、ROS/MAVROS/固件版本、实际 FCU URL/system/component、库 SHA256；不能用旧代码常量代替。
- [ ] 拆桨/电机隔离、遮光后挡、独立断电和失联关断的实际验证记录。
- [ ] 每次 `decision / laser_status / actual spot` 对照记录，3 秒连续靶面录像及观察者签名。

## 8. 验收矩阵（禁止混淆三层）

| 项目 | 离线代码/合成/假 ROS | ROS 真机 | 物理靶面 |
|---|---|---|---|
| 三环比例、去重、尺寸定义换算 | 已测试 | 待真实图像 | 待确认真实尺寸 |
| 缺环、两靶、椭圆/小透视、噪声反光、遮挡拒绝 | 已测试指定合成样本 | 待现场/真实录包 | 待实际受控场景验证 |
| 目标切换清除确认并 OFF | 已测试 | 待测试 | 待确认熄灭 |
| 原图话题/设备/实际 FPS/时间戳/延迟 | 审计脚本/静态检查，非相机测量 | **未执行** | 无硬件确认 |
| 过期/重复/倒退/未来时间戳、相同内容冻结 | 已测试 | 待测试 | 待确认坏画面时熄灭 |
| 断流/处理中超时/退出撤销 | Python 假 ROS 与计时测试通过 | C++/ROS watchdog 待执行 | 独立断电及实际熄灭待验 |
| 偏轴、距离/姿态/分辨率不符拒绝 | 合成标定测试通过 | 待实测标定与校验 | **无真实偏轴/光斑数据** |
| 内环线宽/光斑/误差安全裕量 | 公式与边界测试通过 | 待实测参数 | 待靶面完整光斑在有效区 |
| 预测 3 秒、不拼接、完成/中断 OFF | 纯逻辑测试通过 | 待真节点计时 | 不等于命中 |
| 可见光斑唯一、越界/消失拒绝、观察 3 秒 | 合成光斑通过 | 待真实 OFF/ON 数据验证 | **未进行物理 3 秒命中验收** |
| 启动 OFF、ACK 检查、故障锁存、库哈希、最大曝光 | C++ 源码静态检查/库只读审计 | **catkin/ABI/服务故障注入未执行** | ACK 与光分开验 |
| 原任务/驱动/PWM 二进制不变 | 349 个既有保护文件 SHA256 回归 | 部署时重核 | 不代表其它节点安全 |
| 全程未解锁、电机不转、独立失效关断 | 无硬件启动/控制调用 | 待现场检查 | 必须单独签字 |

### 每次现场试验记录（空白必须保留为未验）

```text
日期/操作者/观察者：
构建输出、部署版本、库SHA、ROS/MAVROS/固件：
直径定义、环宽、工作距离/姿态、标定ID：
相机by-id、话题/尺寸/FPS/frame_id、时延：
机械隔离和独立激光断电验证：
通道、开/关值、另一通道安全值（有测量依据）：
开始/结束 /mavros/state.connected / armed：
启动OFF：服务结果______；实际光______；另一设备状态______
ON：时间戳______；服务ACK______；实际位置/完整光斑边缘______
3秒证据：prediction_only / camera_observed / 独立靶面录像编号______
中断（目标/光斑/过期/断流）：决策OFF时间______；实际熄灭时间______
退出OFF：服务结果______；实际熄灭______；物理断电______
是否满足最内环有效区连续3秒：未验 / 不通过 / 通过（观察者签名）
```

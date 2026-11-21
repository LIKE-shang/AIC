# USB 三同心环识别与手持激光测试：完整交接方案

> **本地资料整理（2026-10-15）**：两轮共 14 张真实相机原图、JSON 审计/回放报告、交接快照和源代码均保留；可由原图或代码重新生成的标注图、掩膜图和合成图已清理。下文图片数量和快照描述仍是当时的历史记录，不代表当前保留的派生图片数量；实机状态和验收结论没有改动。详见 [清理记录](../../_diagnostics/WORKSPACE_CLEANUP_20261015.md)。

**交接日期：2026-10-14（Asia/Shanghai）。本子项目结束于交接，后续进入新的整合对话。**

本文件依据现有源码、不可变部署包、本机保存的原图/回放文件，以及用户在本对话
回传的 Nano 终端输出编写。本轮没有连接 Nano、运行 ROS 或触发任何硬件输出。
其他对话中的穿框/飞行调试不在本文件的验收范围内；其最新状态应读各自交接文档。
本文中的命令供后续有明确任务范围时参考，不是现在执行的指令。

原测试边界继续适用于本地面基线：不启动 `agv_mission`、`uav_mission`、
`complete_mission`、完整 `uav_mission.launch`、矩形飞行演示、底盘、导航、
SLAM 或定位任务；不发送解锁、OFFBOARD、起降、位置/速度/姿态控制。
用户当前要求暂不启动飞控；本文没有给出真实开光或飞行执行命令。

## 1. 接手者先读：已完成到哪里

| 项目 | 本分支交接状态 |
|---|---|
| 三环识别代码、独立 ROS 节点、安全逻辑、工具和文档 | 已实现，并在 Nano 部署 |
| Nano `robot_vision` 构建 | 消息生成及 `laser_test_bridge` 编译/链接成功；当时有时钟偏差警告 |
| Nano 已安装源码的专用离线测试 | 用户回传 40 项通过，0.518 秒 |
| USB 相机实际图像流 | 已审计，1920×1080/rgb8，约 20 FPS |
| 真实三环图像回放 | 加白色背板后 7/7 保存帧检出唯一三环靶 |
| 实时识别 | dry-run 短样本通过；未作长时间负载验收 |
| 遮挡、断流、恢复 | dry-run 状态符合预期；没有实际 ON→OFF 物理测量 |
| MAVROS、激光桥、PWM 输出 | 本分支未启动/未调用 |
| 偏轴标定、红光斑反馈、有效区实测 | 未完成 |
| 连续 3 秒实际命中 | 未验收 |
| 飞行中打靶、穿框和飞行整合 | 未实现/未验收；现版激光桥禁止在已解锁状态开光 |

**可以复用的是三环视觉及地面测试实现，不能把它认定为已完成的飞行打靶系统。**

此前 [AIR_TARGET_HANDHELD_CN.md](AIR_TARGET_HANDHELD_CN.md) 和
[AIR_TARGET_UPDATE_CN.md](AIR_TARGET_UPDATE_CN.md) 中“未编译/未上机”等表述是
早期交付时的历史状态。当前进展以本文件和 [ROBOT_TEST_LOG.md](ROBOT_TEST_LOG.md)
为准；部署包内的旧文档为保留包哈希没有重写。

## 2. 工作目录、设备与停下时的现场信息

| 项目 | 已知信息及边界 |
|---|---|
| Windows 工程 | `C:\Users\jiangzx\Desktop\src` |
| 本子项目 | `robot_vision`，不整体覆盖其他子项目 |
| Nano SSH | `abot@192.168.0.100` |
| Nano 工作空间 | `/home/abot/abot_ws`，源码包为 `/home/abot/abot_ws/src/robot_vision` |
| 历史 Linux 观察机 | `192.168.0.181`；后来用户不再使用，不是继续测试的必需设备 |
| 使用方式 | Nano 命令在 VS Code Remote SSH 终端；文件传输曾使用 Windows PowerShell `scp` |
| 机器人运行环境证据 | ROS Noetic、ros_comm 1.17.4、Python 3.8.10、AArch64；OpenSSL 1.1.1f |
| 开发包证据 | `libssl-dev:arm64=1.1.1f-1ubuntu2.24`，用户回传已安装 |
| ROS master | 本分支使用 `http://192.168.0.100:11311`；整合时须核实当前环境和唯一 master |
| 相机稳定路径 | `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0` |
| 相机历史设备号 | `/dev/video0`；重启后不能只依赖此编号 |
| 相机实际参数 | 1920×1080，MJPEG 采集，20 FPS，ROS 解码图像为 `rgb8` |
| 相机输入话题 | `/usb_cam/image_raw`，`sensor_msgs/Image` |
| 发布节点与坐标帧 | `/circle_usb_camera`；`circle_usb_optical_frame` |
| 另一相机 | 用户确认此前 `/dev/video2` 是另一台/深度相机，不应混作本 USB 相机 |
| 激光 | 用户说明为红光，安装在相机旁，接 AUX5；偏轴未知 |
| AUX6 | 用户明确回复“空着”；这不是固件输出映射或安全保持数值的验证 |
| 最后观察到的 USB 串口 | `/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_in_FS_Mode_395B37893233-if00` → `/dev/ttyACM0` |
| 串口物理身份 | 用户尚未回答“该线另一端是否确实连接飞控 USB 口”；波特率、固件、system/component 未确认 |

最后一次现场动作是列出串口、核对 PWM 库 SHA-256，之后用户要求交接。
不要假设 2026-10-14 的进程、接线、配置仍与 10 月 6–7 日相同；本轮没有查询或
停止 Nano 上的任何进程。其他对话可能已经调试飞行，整合前需重新核对，避免双重控制。

Nano 没有 RTC 电池，曾显示接近 1970 年的时间。用户已手工设过系统时钟，要求不再
处理 RTC。后续重启后应在图像会话前检查时钟；不要在运行检测时调整时钟或放宽
时间戳保护。ROS header 时间递增并不证明传感器曝光时间准确。

## 3. 代码范围与交付文件

下表路径均相对 `robot_vision/`，除非注明 `_deploy/`。

| 文件 | 作用、整合注意点 |
|---|---|
| `src/robot_vision_test/air_target.py` | ROS 无关的三环几何、光斑候选、偏轴预测、有效区与可视化算法 |
| `scripts/circle_laser_test_node.py` | 真实图像订阅、追踪连续性、决策/状态/标注发布和独立 watchdog；直接导入 `air_target` |
| `src/robot_vision_test/safety.py` | `FrameFreshness`、`ContinuousHitGate`、`ContentFreshness`；共享模块，修改后须回归使用它的其他节点 |
| `src/laser_test_bridge.cpp` | 地面未解锁安全桥：启动 OFF、ACK 检查、失效 OFF、故障锁存、退出 OFF、库 SHA/ABI 校验 |
| `msg/LaserDecision.msg` | 三环与瞄准决策消息；相较旧单圆消息新增字段，节点和桥必须使用同一构建版本 |
| `launch/circle_usb_camera.launch` | 单独相机启动；要求显式设备路径；只是 launch 包装，未改原 USB 驱动 |
| `launch/circle_laser_test.launch` | 默认 `start_camera=false`、`enable_laser=false`；后者为 false 时不创建激光桥 |
| `launch/laser_mavros_only.launch` | 预留单独通信启动；本分支未运行；不能直接取代飞行项目的 MAVROS 配置 |
| `config/laser_mavros_plugins.yaml` | 仅 `sys_status`、`command` 插件白名单，不包含飞行 setpoint 插件 |
| `config/circle_laser_test.yaml` | 通用保守模板；尺寸/标定等默认未确认 |
| `config/air_target_field_85cm.yaml` | 本现场 dry-run 参数：外径 35/24/12 cm、8 mm 环宽、目标距离 0.85 m、红色光斑 |
| `config/laser_hardware.yaml` | 硬件输出模板；确认全为 false，通道/开关/另一路值仍为 -1，没有自动填入用户口述值 |
| `config/air_target_calibration_samples.template.json` | 标定记录空模板，不能直接充当实测数据 |
| `tools/audit_air_camera.py` | 只读 ROS 图像审计，保存原图、header 清单、manifest 和统计 |
| `tools/replay_air_target.py` | 原图/manifest/rosbag 只读几何回放；不调用 PWM，不做真实 3 秒命中认证 |
| `tools/calibrate_air_target.py` | 从至少 5 份已审阅的测量计算偏移与误差；输出始终未确认 |
| `tools/audit_pwm_library.py` | 只读 ELF/反汇编工具，不能替代固件输出和接线确认 |
| `test/test_air_target.py` | 40 项专用合成图、标定、时间、隔离检查，并生成合成示例 |
| `test/test_node_callbacks.py` | 假 ROS 回调测试；本机历史记录可参考，不等于 Nano ROS 验收 |
| `CMakeLists.txt`、`package.xml` | 本次合并 OpenSSL 链接、工具安装、YAML/rosbag 依赖，保留既有窗框入口 |
| `setup.py` | 已有 Python 包配置保留，未在安装时替换 |
| `docs/AIR_TARGET_HANDHELD_CN.md` | 原详细算法/安全/操作手册，其历史验收表需结合本交接更新理解 |
| `docs/AIR_TARGET_UPDATE_CN.md`、`docs/AIR_TARGET_INSTALL_CN.md` | 分阶段部署和安装说明，不应整包无条件覆盖当前现场修改 |
| `docs/ROBOT_TEST_LOG.md`、`docs/FILE_MANIFEST.md` | 回传证据与文件索引 |
| `_deploy/build_air_target_update.py`、`_deploy/test_air_target_update_package.py` | 原 20 文件源码包构建及标准库包测试 |
| `_deploy/build_air_target_installer.py`、`_deploy/air_target_installer_core.py` | 按审阅基线校验、备份、安装 19 个必要文件并在失败时尝试回滚 |

本分支没有修改原任务状态机、USB 相机驱动或原 PWM 二进制。
Nano 的 `vision.py` 没有被三环部署包覆盖；本机该文件还含三环 API 转导出，
因此本机整个包与 Nano 整包不能视为完全一致。三环节点直接导入 `air_target`，
不依赖转导出。不要为整合三环而覆盖其他对话的窗框算法或配置。

2026-10-14 本地核对：19 个安装项经 CRLF→LF 归一化后均匹配安装包的目标 SHA。
这只校验本地与冻结安装包，没有重新读取 Nano 文件。详见
[air_target_handover_snapshot.json](air_target_handover_snapshot.json)。

## 4. 三环、偏轴和有效区的准确含义

### 4.1 尺寸与识别

用户明确确认 **35/24/12 cm 是外径，8 mm 是环材径向宽度**，不是三个环的直径。
外边界的半径比约为 `1 : 0.686 : 0.343`。算法把一条环的内外边界配对、去重后
比较中心线几何；本参数对应中心线半径为 `0.171 / 0.116 / 0.056 m`，比例应按
换算后的值计算，不能把原外径比硬套到中心线轮廓。

流程包括：黑环对比度分割、椭圆拟合、内外边界配对、环宽/圆心/轴比/角度检查、
三环同心分组，以及内环遮挡检查。缺环、多靶、额外不确定环、严重透视/遮挡时拒绝。
支持已测试的中等椭圆与小透视样本；没有实现任意透视重建，也不补造被挡住的环。

`confidence≈0.96` 是几何评分，不是 96% 命中概率。图像中的外观完全相同、位置
和尺度不变的靶替换不能被当前算法可靠识别；没有稳定的目标 ID 或跨相机追踪接口。

### 4.2 预测点与坐标

- 图像像素原点在左上，x 向右，y 向下。`rings[].axes` 是半长轴/半短轴像素值，
  不是整条直径；`angle` 是角度制、180° 周期。
- 预测光斑点为 `[(W-1)/2, (H-1)/2] + calibration.offset_px`。
  1920×1080 对应参考中心 `(959.5, 539.5)`，不是硬编码 `(320,240)`。
- `aim.error_px = predicted_point - 最内环内边界中心`；不代表机体系或世界系的
  位置误差。飞行控制的方向、尺度、外参映射尚未定义和验证。
- 曾识别到靶中心约 `(1060.8,519.0)`。**靶中心与图像中心的差不等于激光偏轴**，
  不能将它直接写入 `offset_px`。
- 当前 `offset_px=[]`、`calibration.confirmed=false`，因此不会输出可用瞄准解。

### 4.3 实际有效区

最内环外半径 0.060 m；扣除径向环宽 0.008 m 后，未被环材占据的内区半径
为 **0.052 m**。有效半径按下式再收缩：

```text
effective_radius = min(0.045,
                       0.052 - measured_spot_radius
                             - measured_error_bound - 0.005)   # 单位 m
```

0.045 m 上限和 0.005 m 附加裕量是现有配置，不是光斑大小或实际误差测量。
光斑半径和误差尚为 -1 占位，必须实测；边界判断用严格小于，边上不计命中。
不能仅检查圆心像素误差，更不能直接使用 6 cm 作为可接受的光斑中心半径。

### 4.4 距离与姿态

`working_distance_m=0.85` 是用户希望的现场工作距离，不是本系统实时测距值。
当前只支持固定距离/姿态配置，通过标定距离声明和图像尺度/轴比/角度检查限制范围；
未实现多距离插值或飞行测距融合。**定高不能保证与靶面的距离恒为 85 cm**。
飞行时的横移、俯仰、侧倾、镜头畸变、振动和安装变化均可能改变偏轴。

## 5. 对外接口与消费者应遵守的约定

### 5.1 当前数据流

```text
唯一 USB 驱动 → /usb_cam/image_raw → circle_laser_test
                                    ├─ /circle_laser_test/image
                                    ├─ /circle_laser_test/edges
                                    ├─ /circle_laser_test/detection_status
                                    └─ /circle_laser_test/decision

dry-run：检测器自己发布 /circle_laser_test/laser_status；没有激光桥。
未来地面真光：decision → 独立桥 → 已核验 MAVROS 服务 → AUX；桥回传 laser_status。
后半条链在本分支从未运行，不能认为已经可用。
```

| 话题 | 类型 | 含义与使用要求 |
|---|---|---|
| `/usb_cam/image_raw` | `sensor_msgs/Image` | 输入真实相机帧，保留 capture header |
| `/usb_cam/camera_info` | `sensor_msgs/CameraInfo` | 相机 launch 配套话题；内参文件缺失，未验收为有效标定 |
| `/circle_laser_test/image` | `sensor_msgs/Image`，bgr8 | 标注图；只能辅助观察，不能把“有图”当作当前有效决策 |
| `/circle_laser_test/edges` | `sensor_msgs/Image`，mono8 | 实际为检测二值掩膜/调试图，历史名称为 edges |
| `/circle_laser_test/decision` | `robot_vision/LaserDecision` | 带原图 header 的决策；不是位置、速度或姿态命令 |
| `/circle_laser_test/detection_status` | `std_msgs/String`，JSON | 几何、预测、光斑、耗时、有效性、计时与结果；没有独立 ROS Header，须查 JSON 时间字段 |
| `/circle_laser_test/laser_status` | `std_msgs/String`，JSON | dry-run 模拟状态或真实桥的指令/ACK 状态；不能据此确认物理光 |

输入话题可配置；输出被限制在 `/circle_laser_test/`。现版并未为多个检测器提供
完整命名空间实例隔离，桥也有固定节点/服务检查；不要仅通过重映射复用到控制话题。

### 5.2 LaserDecision 字段

完整定义见 [LaserDecision.msg](../msg/LaserDecision.msg)。

| 字段 | 含义 |
|---|---|
| `header` | 原始相机 header；断流时可能复用最后一帧，不代表新观测 |
| `enable_laser` | 节点真实输出开关；dry-run 为 false |
| `observation_valid` | 当前节点允许使用该观察；失效/超时为 false；不能替代目标和瞄准各项检查 |
| `request_on` | 当前请求；dry-run 在实测配置齐备时也可能模拟 true，因此不能单独驱动执行器 |
| `hit_count`、`miss_count` | 条件连续确认计数，不是得分或命中次数 |
| `target_valid` | 有且仅有符合几何检查的三环靶 |
| `calibration_valid` | 标定身份、距离/尺度/姿态范围与有效区计算通过，不单独表示预测在区内 |
| `aim_valid` | 在上述条件成立后，预测光斑位于收缩后的有效区 |
| `observed_hit` | 算法观察到合格光斑的条件状态；仍需区分 dry-run/实测，不是赛事验收结论 |
| `continuous_seconds` | 当前连续计时；失效或完成 OFF 时会归零 |
| `feedback_mode` | `prediction_only` 或 `camera` |
| `reason` | 当前拒绝、等待、计时或完成原因 |

### 5.3 JSON、完成结果与新鲜度

`detection_status` 常用字段为 `valid`、`target_valid`、`calibration_valid`、
`aim_valid`、`detection.targets`、`aim.predicted_point`、`aim.error_px`、
`spot`、`capture_stamp`、`frame_seq`、`capture_age_s`、`processing_ms`、
`dry_run`、`request_on`、`continuous_seconds`、`result`、`completed_seconds`、
`completed_capture_interval`、`session_latched`、`physical_hit_verified`。

消费者必须同时检查顶层有效性、接收超时、采集时间推进及模式。失效时部分嵌套
几何/时间字段可能保留最后观测，不能跳过顶层 false 而继续使用旧靶。不要只凭
`header.seq` 判新帧，相机重启可以重置序号；也不要把接收时刻伪装成曝光时刻。

- `result=none`：没有完成结果。
- `predicted_only_3s`：预测条件持续满足，不等于实际命中。
- `camera_observed_3s`：配置和光斑验证条件下的图像证据；真实靶面仍需核验。
- 成功触发后立即 OFF 并锁存，当前计时归零；完成摘要保存在 JSON 的
  `completed_seconds` 与 `completed_capture_interval`，不在 LaserDecision 中。
- `physical_hit_verified` 和 `hardware_light_verified` 在现实现中保持 false；
  人工观察/录像验收应单独记录，不能通过服务 ACK 将它们改为 true。

现有 `laser_status` 包含 `DRY_RUN_OFF/DRY_RUN_ON`，或真实桥的
`OFF_PENDING/ON_PENDING/OFF_ACKNOWLEDGED/ON_ACKNOWLEDGED/UNKNOWN` 等状态。
后者仅表明通信/请求状态，主机断电或失联后不能保证物理熄灭。laser_status 使用
latched 发布，订阅者重连会收到旧值；其 JSON 没有 capture header，消费者还必须
独立记录接收时间并检查心跳，不能把第一条 latched ACK 当作当前确认。
安全 OFF 消息可携带旧 header，不能因为它不算“新帧”就忽略撤销请求。

## 6. 当前安全逻辑及与飞行的冲突

现场配置：确认帧数 5、失效立即撤销（`off_frames=1`）、图像超时/最大年龄
0.35 s、未来容忍 0.05 s、相同内容超时 0.25 s、连续保持 3 s、最大请求曝光
5 s、相机光斑首次获取窗口 0.30 s。它们是软件参数，不是实测关光延迟上限。

- 连续计时同时受 monotonic 时间和 capture 时间限制，不能累加几段中断照射。
- 丢靶、换靶、越界、过期、重复/倒退时间戳、断流均撤销；重复像素保护只检测
  完全相同的 payload，不是通用视频防重放证明。
- ON 之后中断/完成会锁存。没有“下一靶自动再开”或复位服务；现场检查后重启
  独立进程才解除。此前断流恢复是在未开光状态，不能外推为真实模式可自动复光。
- 相机模式首次获取之前不记命中时间；已经识别到越界/多光斑则立即撤销，不享受
  获取宽限；没有看到光斑时不能伪造实际反馈。
- 桥初始化在保护条件成立后先发 OFF，检查 transport/success/result，定期刷新
  OFF、限制服务等待并在退出时尝试 OFF；未知和故障不会自动恢复开光。
- 加载库前验证 AArch64、固定 SHA 与 `CommandLong` ABI（大小/字段偏移），
  运行时自校验与真服务行为仍未在 Nano 执行。编译成功不证明运行时 ABI 一定通过。

**与未来定高飞行直接冲突的条件：**

1. `circle_laser_test_node.py` 的真实模式要求桥回报 `fcu_armed=false`。
2. `laser_test_bridge.cpp` 的 `step/state/request` 在 `armed=true` 时撤销 ON，
   需要连接正常、状态新鲜、未解锁才允许 ON。
3. 这是一项明确的地面手持边界，不是待删除的错误或简单参数。不能伪造
   `/mavros/state`、绕过桥服务或直接删除 `armed` 检查来适配飞行。

若新对话明确要在飞行中打靶，应将“飞行中激光执行”作为新范围单独设计：任务阶段
授权、唯一输出控制权、飞行状态/距离与光路条件、独立关断、失联策略、校准、机载
实际光斑记录和阶段验收。保留本地面版本作回归基线，先完成无输出的整合和地面实测。

## 7. 已有证据及准确的验收等级

| 验收项 | 离线/代码证据 | ROS/真实相机证据 | 物理激光证据 |
|---|---|---|---|
| 三环比例、边界去重、外径/环宽换算 | 40 项测试中的相应案例通过 | 7/7 原图回放及实时 3 条短样本通过 | 不涉及点亮；尺寸定义来自用户 |
| 缺环、双靶、额外环、椭圆/透视、噪声反光 | 指定合成样本通过 | 未对全部场景逐项现场验收 | 未验 |
| 遮挡拒绝 | 合成/回调逻辑有记录 | 完全遮挡回传 `missing_or_inconsistent_rings`，request=false | 测试时未开光，未测实际关光 |
| 实际图像格式、频率、header | 审计工具 | 199 帧/10 s，约 20 FPS，无零/重复/倒退 | 曝光时刻/传感器总延迟未验 |
| 断流 watchdog | 计时逻辑与假 ROS 测试 | 停相机回传 `image_timeout`、request=false、计时=0 | 未测 ON→OFF 与真实熄灭 |
| 断流后恢复 | 状态逻辑 | 重启相机后 3 帧 target=true，仍 `calibration_unconfirmed` | 无复光验证 |
| 时间戳过期/未来/回退、冻结画面、处理中超时 | 专用测试/历史回调记录 | 尚未逐项故障注入 | 未验 |
| 目标切换、三秒不中断累计、结束/中断锁存 | 单元/回调测试 | 当前未标定，不能进行真实瞄准计时 | 未验 |
| 偏轴、距离/分辨率不符拒绝、有效区边界 | 合成标定与边界测试 | 没有已确认实测标定 | 未验 |
| 红色光斑唯一性、消失/越界、相机三秒结果 | 合成光斑和计时案例 | 没有真实 OFF/ON 数据 | 未验 |
| 消息、Python 节点、C++ 桥 | 静态/历史离线检查 | Nano 编译/链接及新节点运行通过；有构建时钟警告 | 桥实际输出未验 |
| 启动/退出 OFF、ACK 拒绝、服务失联、最大曝光 | 源码静态与计时测试 | 未启动 MAVROS/桥，未作真实服务故障测试 | 未验 |
| 未解锁、电机隔离、独立断电 | 软件保护已实现 | 安装命令包含操作者确认，但不作为硬件传感证据 | 尚无独立验收记录 |
| 穿框/飞行整合 | 未实现 | 其他对话进展需另取证 | 本分支不覆盖 |

具体证据目录：

| 内容 | Nano 路径 | 本机副本 |
|---|---|---|
| 首轮原图，外环与黑支架/背景相连 | `/home/abot/air_camera_audit.1PJh8m` | `_deploy/air_camera_audit.1PJh8m` |
| 首轮回放，7 帧中 0 个有效靶 | `/home/abot/air_target_replay.vjxYiK` | `_deploy/air_target_replay.vjxYiK` |
| 加大哑光白背板后的原图 | `/home/abot/air_camera_audit.judOdL` | `_deploy/air_camera_audit.judOdL` |
| 原参数回放，7/7 唯一三环 | `/home/abot/air_target_replay.0qvvPe` | `_deploy/air_target_replay.0qvvPe` |
| 安装后实时相机审计 | `/home/abot/air_camera_live_audit.WfB10n` | 本机未找到该目录；只有本对话输出，后续可归档 |

第二批回放首帧中心约 `(1073.59,528.18)`、评分约 0.961；实时短样本中心约
`(1060.8,519.0)`、评分 0.956–0.959，处理约 53–55 ms，header 年龄约
0.185–0.219 s。两批尺寸/位置不同，不能混作偏轴标定数据。
最后实时审计最大 header 间隔 51.55 ms，接收减 header 年龄约 8.47–140.94 ms。

本机还保留 `test/artifacts/air_target_geometry_test_run.txt`（40 项）、
`air_target_test_run.txt`（历史综合 135 项）、`air_target_baseline_tests.txt`
（历史 84 项）及 `_deploy/air_target_update_20261006_v1_tests.txt`（7 项包检查）。
这些是历史运行记录，不能表述为 2026-10-14 重新运行全部测试或其他子项目也通过。
本轮仅做交接内容、路径和版本静态校验，没有重跑 OpenCV/ROS/catkin。

## 8. 部署与版本追溯

源码包已校验并在 Nano 暂存到 `/home/abot/air_target_stage.lQiA0E`；后续安装器
暂存到 `/home/abot/air_target_install_stage.3mnQrq`。安装器输出
`INSTALLED_SOURCE_ONLY`，写入 19 个项目，校验 78 个未改动文件。
目录和安装结果来自当时回传，重启/其他对话修改后需现场复核。

| 产物 | SHA-256 |
|---|---|
| `_deploy/air_target_update_20261006_v1.tar.gz` | `2747eb9e51064083a1566844d293726e78a593117b3106a1f2e269c59b5aea76` |
| `_deploy/air_target_installer_20261006_v1.tar.gz` | `8beedd41cc3341a51c7776404dcbf312234e39c15f6973fcff8d3b9c8fac631b` |
| `_deploy/robot_vision_before_airtest.sNw5nO.tar.gz` | `3620802c3443c7213af2ccaefd5f2bd5db864096f8d1edba33c7204e75be360c` |
| Nano `/home/abot/robot_vision_before_air_install.4x81q2lu.tar.gz` | 用户回传 `be0210ebe2b01671511e7abf954d644c5f696a06fb7e2e205ff8a549b73ae182`，本机未重新下载校验 |
| `uav_mission/lib/liblib.so` | `e055c1e0468cc44c114c2ff408a6303bb9e0e9ece5fcf707fd3489dfb0e2734c`，本机及用户回传 Nano 哈希一致 |

原源码包中的 `build_reference/` 仅供核对；正式安装器将明确审阅后的构建差异
合并，并保留既有窗框脚本和 setup。不要重新打包覆盖这些历史同名 v1 文件，也
不要向当前工作空间直接全量解压旧包。安装器拒绝未知文件改动是正常行为，发生时
应做差异审阅，不关闭哈希校验来强制覆盖其他对话的成果。

## 9. 后续必须补齐的测量与硬件事项

### 9.1 PWM：已知设计值与未验事项

本机 `uav_mission/src/uav_mission.cpp` 与原始 `dongbu-ros.zip` 内同文件的以下 PWM
调用和行号一致；不表示整份任务代码一致（本地任务航点已有差异，非本次修改）：

| 原始位置 | 内容 | 只能得出的结论 |
|---|---|---|
| 第 85 行 | `lib_pwm_control(20,20)` | 初始化设计，不是已验证的熄灭值 |
| 第 257/305 行 | `lib_pwm_control(100,100)`，随后日志“开启激光笔” | 原任务 ON 设计值为 100 |
| 第 268/315 行 | `lib_pwm_control(0,0)`，随后日志“关闭激光笔” | 原任务 OFF 设计值为 0 |
| 头文件第 48–50 行 | 通道 5/6、整数 0–100 | API 范围，不是 PWM 微秒值 |

已保存的库静态审计显示 command=187，`param1=p5/50-1`、`param2=p6/50-1`。
此前用户的“pwm50”没有明确是 ON、OFF 或其他设置，不得直接将 50 当开关值。
旧代码两路同时开/关；现桥需要另外给出 AUX6 固定保持值。用户确认 AUX6 空置，
但配置没有自动填 0，也没有验证固件的全部命令字段映射和未解锁输出行为。

仍需核实：USB 串口物理身份、固件版本、实际通信参数/system/component、
命令 187 的固件实现、AUX5 真实开/关极性、AUX6 保持行为、其他字段是否影响
别的输出、未解锁时能否输出、机械隔离及独立激光断电手段。
**不能用解锁解决锁定状态不输出，不能按原任务启动飞行来测试激光。**

`laser_mavros_only.launch` 要求显式 `fcu_url`，内部默认 target ID 1/1 只是默认值。
本分支没有准备可直接执行的实际连接参数，也没有启动该 launch。

### 9.2 偏轴标定与真实光斑

1. 固定相机/激光相对安装、分辨率/裁剪、对焦/曝光、靶距离与姿态；记录设备标识。
2. 在已确认的低风险封闭光路、非反光靶面、机械隔离和独立断电条件下，采集至少
   5 组 OFF→短时 ON→OFF 原图，另留验证数据。人员不得处在光束或反射路径。
3. 测量完整光斑中心与保守半径、板上像素尺度、测量误差、姿态/安装/镜头附加误差，
   填入模板。多个距离分别测量，不能混在一组平均。
4. 使用标定工具生成待复核 YAML；工具主动保持所有关键确认 false，须审阅真实
   测量后才逐项确认。照片一张通常不足以给出误差上界和可靠工作范围。
5. 先在 dry-run 验证预测点/有效区与独立样本，再验证红点误检、丢失、越界和反光。
   当前 HSV/面积阈值只经过合成样本，红色标签不代表真实光斑已经可可靠识别。
6. 摄像头无法可靠看到光斑时只输出预测并要求靶面观察/录像；不能给出
   `camera_observed_3s` 的实际验收结论。

**当前标定前不能用本自动桥点亮。** 它要求先确认标定，且没有标定“强制开光”
接口。原方案使用独立、已核验的瞬时硬件按钮/受限电源获得短时光斑测量；若现场
没有这种设施，应在新范围中设计受控的标定装置或独立流程，不能伪造标定参数、
裸调 MAVROS 服务、降低保护条件或开启原任务来绕过。此问题尚未解决。

## 10. 与穿框、飞行程序整合的建议顺序（尚未实现）

以下是后续工作方案，不表示现在已有总任务管理器或任务阶段控制接口。

### 阶段 A：整理各子项目与资源所有权

- 分别读取穿框、飞行项目最新交接、源码、现场记录；不要用本分支日志替代它们。
- 枚举相机硬件/驱动、节点名、话题、master、串口、MAVROS、消息版本、参数加载
  位置及飞行指令发布者。一个物理相机只起一个驱动；多个识别节点订阅同一图像。
- 激光/PWM 保持唯一写入者。搜索其他程序中的 `lib_pwm_control` 和命令服务调用，
  不能让旧任务在旁路修改 AUX5/6。
- 全流程只保留一个经核验的 MAVROS 连接所有者。此项目的仅两插件配置不是飞行
  MAVROS 配置；不能在运行飞行 MAVROS 时再启动本独立实例争抢串口。
- 现版节点启动时读取配置，没有运行中重新加载标定或阶段允许服务。不要假定
  `rosparam set` 会实时改变已加载的 Python 参数；变更后按流程重启并重新审计。

### 阶段 B：只整合视觉和状态记录

复用 `air_target` 的纯算法或现有 detector 输出。建议未来整合层自行分配试验/任务阶段 ID，
防止重启、重连后把旧完成摘要作为新一轮成功；现实现没有这样的会话授权接口。
未来可新增独立适配层，保留：
原图时间、有效性/拒绝原因、像素坐标系、几何尺度、标定 ID、预测/观察模式。
该适配层尚未实现；不要直接把测试图像像素作为飞行控制误差。

总流程应有明确的目标选择/阶段授权和接收超时，避免穿框画面中的环状物触发打靶。
推荐先记录“观察到唯一靶/标定是否适用/预测是否入区”，激光持续关闭。
相机 20 FPS 不代表检测器 20 FPS；现有 53–55 ms 短样本耗时已经接近单帧间隔。
在穿框、定位、飞行模块并行负载下重测队列、丢帧、延迟和 watchdog；不得靠放宽
超时来掩盖积压。若降分辨率/裁剪，应重新标定偏轴，而非按比例盲套参数。

### 阶段 C：完成地面独立激光验收

先解决第 9 节的标定、接线、独立关断和未解锁输出问题，再验证原有安全桥。
分别记录请求、服务 ACK 和物理光斑三条证据；包括启动 OFF、正常短时 ON/OFF、
丢靶/越界/断流/退出/通信故障，最后才验证完整光斑在有效区连续 3 秒。
没有物理反馈的项目继续标“未验”，不能因软件 OFF 或编译成功改为通过。

### 阶段 D：明确未来飞行阶段控制设计

若新任务明确授权飞行中打靶，另行设计可审阅的飞行激光执行条件与测试；不能
直接沿用本桥已解锁拒绝逻辑并宣称支持，也不能将保护删除后认为完成。
建议总流程显式区分“穿框阶段、靶点观察阶段、允许光斑验证的阶段、退出/故障阶段”，
由一个受控执行层决定输出。总流程退出、目标身份变化、控制权变化和时间失效应
撤销照射；一次失败不能自动重开光。每一阶段先无输出记录，再逐步验证。

高度不等于靶距离；需要独立的距离/姿态来源与可靠性边界，验证相机到机体系、
激光轴与目标平面关系、运动模糊和曝光延迟。当前代码没有飞行控制增益、方向、
距离估计或三维目标姿态能力，不要推测存在这些接口。

### 阶段 E：全流程验收

在前面独立项目通过后再做端到端调试。验收材料至少保存完整参数、软件哈希、
相机/位姿/决策/控制状态时间序列、实际靶面录像、阶段切换与故障处理记录。
软件成功、预测三秒、相机观察三秒、人工确认实际命中应保留为不同结果。

## 11. 后续复现的最小命令参考

**本轮不执行。** 以下只用于后续明确恢复相机/地面 dry-run；有其他任务运行时先
确认资源和节点所有权，不要擅自停止其他对话的进程或重复创建相机/master。
每次现场沟通仍按用户要求一次一步，不把整份手册一次性要求用户执行。

### 11.1 构建注意事项

已有 Nano 构建成功，无需为阅读交接而重建。以后代码确实变更时，先检查缓存：

```bash
grep '^CATKIN_WHITELIST_PACKAGES:' /home/abot/abot_ws/build/CMakeCache.txt
```

本分支曾因缓存为 `agv_mission` 导致 `catkin_make --pkg robot_vision` 找不到目标。
最后成功构建命令如下，会把缓存白名单持久改成 robot_vision：

```bash
cd /home/abot/abot_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=robot_vision --pkg robot_vision
source /home/abot/abot_ws/devel/setup.bash
rosmsg show robot_vision/LaserDecision
```

整合时必须维护新流程实际需要的包列表与 overlay，不能清空白名单无差别编译
整个老工程，也不要覆盖 root CMake 或清除其他项目 build/devel 来解决局部问题。
旧消息、旧桥二进制、新 Python 节点不能混用。

### 11.2 终端 A：仅在无既有 master 时启动

```bash
source /home/abot/abot_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=192.168.0.100
roscore
```

其他终端同样 source 工作空间并使用相同 ROS 网络设置。不要在检测运行中跳变系统时间。

### 11.3 终端 B：确认唯一驱动后启动 USB 相机

```bash
source /home/abot/abot_ws/devel/setup.bash
roslaunch robot_vision circle_usb_camera.launch \
  video_device:=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0 \
  image_width:=1920 image_height:=1080 framerate:=20 \
  pixel_format:=mjpeg image_topic:=/usb_cam/image_raw
```

上述路径来自已用实物确认的相机；重接后仍应核实。曾出现内参文件缺失、
`focus_auto` 和 swscaler 警告，随后图像审计成功。这不证明镜头已标定；也不应
将警告本身解释为真实光斑开关条件。与激光相关的是尚未完成的偏轴/误差验证。

### 11.4 终端 C：独立 detector dry-run

```bash
source /home/abot/abot_ws/devel/setup.bash
roslaunch robot_vision circle_laser_test.launch \
  start_camera:=false enable_laser:=false \
  image_topic:=/usb_cam/image_raw \
  config_file:=/home/abot/abot_ws/src/robot_vision/config/air_target_field_85cm.yaml
```

该命令不启动桥。合格靶的当前预期是 `target_valid=true`、
`reason=calibration_unconfirmed`、`request_on=false`、计时 0。
`start_camera=false` 时 launch 内 640×480 默认值不会改变已运行相机的实际图像。

### 11.5 终端 D：查看状态、保存证据

```bash
source /home/abot/abot_ws/devel/setup.bash
timeout 10s rostopic echo -n 3 /circle_laser_test/decision
```

需要图像/耗时 JSON 时再读 `detection_status`，输出较长，不必每轮都打印。
下列审计只订阅，自动结束，保存目录应每次新建：

```bash
audit_out="$(mktemp -d /home/abot/air_camera_audit.XXXXXX)"
python3 /home/abot/abot_ws/src/robot_vision/tools/audit_air_camera.py \
  --topic /usb_cam/image_raw --seconds 10 --output "$audit_out"
printf 'AUDIT=%s\n' "$audit_out"
```

**ROS 工具不要加 `python3 -E`**，它会忽略 ROS 所需的 PYTHONPATH，已经实际
导致 `No module named rospy`。纯离线合成测试曾使用 `-E`，与 ROS 工具不同。

```bash
python3 -E /home/abot/abot_ws/src/robot_vision/test/test_air_target.py
```

### 11.6 原图、审计 manifest、录包接口

以下只读回放保留当前参数，不启动 rosbag play 或 PWM：

```bash
replay_out="$(mktemp -d /home/abot/air_target_replay.XXXXXX)"
python3 /home/abot/abot_ws/src/robot_vision/tools/replay_air_target.py \
  --manifest /home/abot/air_camera_audit.judOdL/manifest.json \
  --config /home/abot/abot_ws/src/robot_vision/config/air_target_field_85cm.yaml \
  --output "$replay_out"
```

工具也接受 `--input <原图或图片目录>`，或 `--bag <bag文件> --topic /usb_cam/image_raw`。
单图没有真实时间证据，会标记 synthetic_clock；bag 模式依赖 source 后的 ROS
rosbag/cv_bridge，读取未压缩 `sensor_msgs/Image`。回放固定 request=false，不认证
连续曝光。相机审计默认每 30 帧保存一图，约 1.5 s 一张，不能证明连续 3 秒光斑。

需要完整 dry-run 记录时，可在容量允许且不会加重时延的前提下记录已存在的话题：

```bash
record_dir="$(mktemp -d /home/abot/air_target_record.XXXXXX)"
rosbag record --duration=10 -O "$record_dir/dry_run.bag" \
  /usb_cam/image_raw /circle_laser_test/decision \
  /circle_laser_test/detection_status /circle_laser_test/laser_status
```

此录包命令是接口参考，尚未作为本分支真机录包结果执行验收。原图约
6.22 MB/帧，20 FPS 约 124 MB/s 未压缩载荷；录包/远程图像传输的 CPU、带宽、
磁盘预算须纳入整合压测。不要默认 Wi-Fi 可无损传输所有原图。

## 12. 后续交接数据清单与验收记录模板

需要新对话补齐的材料：

- 穿框与飞行子项目最新入口、运行模式、控制权、话题/坐标系和验收记录。
- Nano 当前节点图、代码/二进制版本、实际相机和 MAVROS 所属进程、构建白名单。
- 当前安装距离/姿态范围、相机设置、至少 5 组可追溯光斑标定原图及独立验证图。
- 红点半径、偏轴离散误差、尺度/畸变/姿态剩余误差；不能取 0 作为缺失占位。
- AUX5/AUX6 现场接线、设计值与实测值分别记录，固件和命令字段映射证据。
- 实际飞控 USB 身份和通信参数；未解锁输出、机械隔离、独立断电测量记录。
- 启停、丢靶、丢光斑、越界、断流、通信/主机失效时，命令与实际光的对应录像。
- 完整光斑在有效区内连续 3 秒的证据；与预测/算法结果分开存放。

每次试验记录：

```text
日期/操作者/观察者/试验编号：
范围：离线 / ROS dry-run / 地面未解锁真光 / 另行批准的新范围
部署与构建版本、PWM库SHA、ROS/MAVROS/固件：
唯一相机驱动、实际话题/尺寸/FPS/frame_id、图像与决策年龄：
靶外径/环宽、距离与姿态范围、标定ID、实测偏轴/半径/误差：
激光通道、ON/OFF值、AUX6保持值、实际接线与映射依据：
机械隔离、独立断电手段及验证结果：
软件请求时间 / 服务ACK时间和结果 / 实际光变化时间：
有效区内完整光斑与连续3秒证据文件：
中断场景、撤销请求时间、物理熄灭时间、是否锁存：
结束状态：物理无光 / 未知；现场记录：
结论：离线通过__；ROS行为通过__；物理命中通过__；未验项__
```

## 13. 新对话的建议开场与本次收尾

可直接使用项目根目录 [AIR_TARGET_HANDOVER.md](../../AIR_TARGET_HANDOVER.md) 中
的开场文字。新对话应先做只读整合审阅，沿用“一次一步、用户回传”的现场节奏。

本次收尾仅新增交接文档、版本快照和索引记录；未修改运行算法/参数，未重建或
替换已上传包，未启动/停止机器人节点。用户已要求暂不启动飞控，本分支不再继续
实测，也没有创建后台继续任务。恢复硬件测试的条件应在新的整合任务中明确。

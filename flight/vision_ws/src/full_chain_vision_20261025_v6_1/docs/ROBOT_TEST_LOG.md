# Nano / Linux physical-host test record

> **资料整理说明（2026-10-15）**：下文是历史测试记录。真实相机原图、回归测试输入、原始日志及 JSON 摘要保留；可再生成的标注/掩膜/合成图片已清理，历史目录中不再保证留有这些输出图。详见 [清理记录](../../_diagnostics/WORKSPACE_CLEANUP_20261015.md)。

Date: 2026-10-04 (Asia/Shanghai). These are results supplied by the user,
not commands remotely executed by the coding agent.

Latest update: see the 2026-10-05 section below. The initial NOT FOUND results
and missing-original-image notes are historical; the user subsequently reported
stable live checker-window detection after installing the checker patch.

## Confirmed from user terminal output

- Upload archive SHA256 matched; all 29 deployed file checksums reported OK.
- Nano: aarch64, ROS Noetic, Python 3.8.10, OpenCV 4.10.0,
  NumPy 1.24.4. Python3 cv_bridge conversion test passed.
- Separate workspace `/home/abot/vision_camera_test_ws` build completed;
  LaserDecision message and robot_vision_test Python imports passed; executable
  presence checked. The original workspace whitelist remained `agv_mission`.
  No laser executable was run by that verification command.
- Gemini actual RGB: `/camera/rgb/image_raw`, sensor_msgs/Image, rgb8,
  approximately 30 FPS measured locally.
- Nano ROS address: 192.168.0.100. Linux viewer: 192.168.0.181, Noetic.
- Linux rqt subscribes `/window_test/image`; observed publisher
  `/window_test` at 192.168.0.100 and subscriber rqt at 192.168.0.181.
- Wi-Fi/node-ping latency and intermittent timeouts observed; transport works
  but stability is not yet accepted.
- Window output image contains `WINDOW: NOT FOUND (0) | RGB ONLY | NO CONTROL`.
- Nano `rostopic echo -n 1 /window_test/status` returned count=0,
  input_encoding=rgb8, no_control=true, rgb_only=true, state=not_found;
  exit status=0. This is a valid no-detection result, not missing local status.

## Earlier window-detection issue (before lossless RGB was supplied)

The real target uses a checker-pattern border with a highly textured reflective
background. The photographed edge display shows many small checker contours
and background edges. The target is now visibly inside the image, but the
existing hierarchical paired-quadrilateral detector does not find a window.
This is NOT a completed window-detection acceptance test.

Read-only exploratory analysis of the photographed edge display also produced
no accepted quadrilateral with the unmodified contour filter. Simple close/open
trials did not yield a valid nested frame pair. Those observations are based on
a rescaled photograph of a screen, not the original camera image, and must not
be represented as a replay of the Nano capture or a validated parameter fix.

Next needed evidence: a lossless original RGB image and its current parameters
from Nano, for offline replay and a checker-frame-specific regression case.
No runtime detector parameters or robot task code were changed during this
image-review turn.

## Unverified at the initial test stage (historical)

- Successful real checker-window detection and false-positive behavior.
- Physical no-motion observation (code has no motion interfaces, but hardware
  behavior must be separately observed by the operator).
- USB camera identification / circle detection on real images.
- Laser wiring, PWM mapping, service commands, physical illumination and OFF.

## 2026-10-04 原图证据与离线修复进展（不是真机验收）

上述“还需原始图像”的步骤现已完成：用户提供原始 window_rgb.png 和运行参数，
工程内无损副本为 robot_vision/test/artifacts/window_checker.png。
该副本 SHA-256 为 e7ca4d3b5920864301805bd0048261c3d0f1dbd695f7438dc71420cbc8e66f8a。

- 用户原参数/原 nested-only 检测路径在原图上返回 0；已复现。
- 新增四侧棋盘纹理证据路径后在此原图上返回 1，轮廓是内开口，不是外框。
- 本地 50 项离线回归通过；完整记录见 WINDOW_CHECKER_PATCH_CN.md。
- 新程序用 max_processing_fps=10 限制处理负荷，并通过 status 报告 processing_ms、method、outline_kind。
- 提供只更新三个窗框运行文件的补丁；没有远程安装，尚未收到 Nano 实时通过的证据。
- 本地修改不影响已经在 Nano 运行的旧进程；必须安装并仅重启识别节点后才生效。

仍需现场检查本页上面的 Still unverified 项；USB 和真实激光尚未验证，不能由窗框离线通过推导已通过。

## 2026-10-05 用户现场反馈与比赛接口适配

用户已回传：棋盘格补丁部署后“没有任何问题，在移动后也会稳定识别”。
据此更新窗户实时识别状态为**用户报告通过**；本轮未远程执行相机测试，
也未收到所有角度/光照/距离负样本统计，不能把这项反馈扩展成飞行或误检率验收。

- 棋盘格补丁 SHA256 校验及安装器的 import/空图 smoke test 曾由用户回传成功。
- Nano 和 Linux 物理机时钟已由用户检查；Nano NTP 服务 active 但 synchronized=no，
  不能仅凭两次相近的 date 输出宣称长期时钟同步已通过。
- Windows 工程已冻结 window_competition_vision.yaml，沿用现有 window_test.yaml 参数。
  代理没有读取 Nano 当前参数服务器；临时调参仍需导出核对。
- 新增 ground-only window_competition_adapter，默认只发两个 /window_test 隔离话题，
  输出候选 PointStamped 像素中心和有效性诊断，flight_ready 始终 false。
- 本轮 34 项适配器标准库/Fake ROS 回归通过，包含真正的 wall-clock 后台 watchdog、
  过期/未来/重放帧、首失效撤销与恢复重确认、隔离及旧接口启动冲突检查。
- 349 个受保护文件哈希保持一致，已验证任务代码未改动。
- 本轮完整 OpenCV 回归因本地依赖访问权限未完成，未通过其他工具绕过权限限制。
- 新适配器在 Nano 的编译、ROS 传输、断流/退出消息和物理 no-motion 验收尚未完成。

检查当前比赛状态机后确认：运行路径仍为
1 -> 2222 -> 3333 -> 4444 -> 5555 -> 17；对框 case 2 / 穿框 case 3 不在此路径。
旧任务没有目标新鲜度保护，不得因视觉已稳定而直接接入自动飞行。
USB/真实激光、控制方向、相机内外参、距离和穿框安全均仍需另行验证。
本轮操作手册见 WINDOW_COMPETITION_ADAPTER_CN.md。

## 2026-10-06 USB 三环首轮原图回放（未通过识别）

本节依据用户回传的 Nano 终端输出与下载到本机的原始数据；代理未连接机器人执行命令。

- 用户已备份原包，更新包整体及内部 SHA-256 校验通过；源码仍在
  `/home/abot/air_target_stage.lQiA0E`，尚未覆盖原工作空间。
- 用户在 Nano 暂存目录运行专用测试，回传 `Ran 40 tests in 0.518s / OK`。
  这是合成图与安全逻辑离线测试，不是实时三环或物理激光验收。
- 相机审计目录 `/home/abot/air_camera_audit.1PJh8m`：10 秒收到 200 帧，
  1920x1080/rgb8，发布者 `/circle_usb_camera`，frame_id 为
  `circle_usb_optical_frame`；到达频率约 20.012 FPS，header 间隔频率约
  20.002 FPS，无零、重复或倒退时间戳，最大 header 间隔约 51.43 ms。
- ROS 接收时刻减图像 header 时刻为约 9.35--146.04 ms；这不等于实测
  传感器曝光延迟。工具的设备物理身份与曝光时间戳确认字段仍为 false。
- 回放目录 `/home/abot/air_target_replay.vjxYiK`：7 张原图的时间戳检查通过，
  但 `unique_targets=0`；各帧均为 `missing_or_inconsistent_rings`，仅保留
  中环、内环两项候选。首帧候选半长轴约 127.34/61.46 px，所有请求为 OFF。
- 原图及二值掩膜显示，外环附近的纸边与黑色支架背景相接，不能获得独立的
  完整外环轮廓。下一步加更大的哑光白色背板、检查环线完整性后，用原参数
  重新采集和回放；改善效果尚未验证，不通过放宽容差或补造缺失环来开光。

本机证据保存在 `_deploy/air_camera_audit.1PJh8m/` 和
`_deploy/air_target_replay.vjxYiK/`。本轮只读取原图、JSON 和掩膜并记录结果，
未重新执行本地 OpenCV，未安装或启动新版三环 ROS 节点，未执行 catkin、
MAVROS、PWM、解锁或飞行命令。真实光斑、偏轴标定及连续 3 秒命中均未验收。

## 2026-10-06 USB 三环第二轮原图回放（几何通过，开光仍锁定）

- 用户在靶纸后增加白色背板后重新采集：199 帧，约 20.006 FPS；header 间隔
  频率约 20.000 FPS，最大间隔约 52.58 ms，时间戳无零、重复或倒退；接收年龄
  约 9.26--143.76 ms。发布者仍为 `/circle_usb_camera`，格式为 1920x1080/rgb8。
- 回放目录 `/home/abot/air_target_replay.0qvvPe` 的 7 帧全部为
  `three_rings_unique`，每帧 3 条环、1 个目标；首帧置信度约 0.961、比例误差约
  0.00217。标注图显示外/中/内环和共同中心与原图一致，未见把支架当外环的证据。
- 每帧 `capture_valid=true`，但 `calibration_valid=false`、`aim_valid=false`，
  原因是 `calibration_unconfirmed`；没有预测光斑点、请求开光或物理命中声明。

本机证据为 `_deploy/air_camera_audit.judOdL/` 和
`_deploy/air_target_replay.0qvvPe/`。本轮仅检查下载数据及标注，未重跑本地 OpenCV。
这轮只证明该组原图的三环几何与新鲜度回放通过；三环实时识别节点、相机--激光偏轴、
红色光斑反馈、PWM 接线和 3 秒物理命中仍未验收。

## 2026-10-06 编译依赖与 AUX5 接线信息

- 用户回传 `libssl-dev:arm64: install ok installed; version=1.1.1f-1ubuntu2.24`。
  OpenSSL 开发包已安装；新版消息与安全桥尚未编译，不能据此宣称构建通过。
- 用户明确说明激光接在 AUX5，由 PWM 值控制亮灭。AUX5 是操作员提供的
  接线信息；开值、关值、极性、未解锁时的实际输出行为和 AUX6 安全保持值
  仍未核验。此前的“pwm50”没有明确表示 ON 或 OFF，也不是微秒单位确认。
- 原库同时写两路，不能由 AUX5 接线信息推断 AUX6 可设为 0。模板和已有
  暂存包的实际输出允许字段继续保持 false；不以本条接线记录解除任何开光保护。
- 下一步读取已有原包备份，核对待覆盖文件后生成最小部署更新。当前仍停留在
  dry-run 准备阶段，没有执行 MAVROS、PWM、解锁或飞行操作。

## 2026-10-07 回传验收进展：源码安装、ROS dry-run 与断流

以下依据操作员逐步回传的 Nano 终端输出，包含 10 月 6 日晚的测试；代理没有
通过 SSH 执行机器人端命令。此前“尚未安装/编译”的条目是当时状态。

- 安装器校验通过并返回 `INSTALLED_SOURCE_ONLY`，安装 19 个文件，校验 78 个
  未改动文件；安装时 ROS 图只有 `/rosout`。安装前新备份为
  `/home/abot/robot_vision_before_air_install.4x81q2lu.tar.gz`，SHA-256 为
  `be0210ebe2b01671511e7abf954d644c5f696a06fb7e2e205ff8a549b73ae182`。
- 操作员在已安装路径运行 `test/test_air_target.py`，回传 40 项测试通过
  （0.518 秒）。仍属于离线合成图和安全逻辑测试。
- 首次 catkin 构建因缓存白名单为 `agv_mission` 失败；显式改为 `robot_vision`
  后输出消息生成、C++ 编译、链接及 `Built target laser_test_bridge`。构建当时
  有 clock-skew 警告，不将其描述成无警告构建；随后操作者手动设置系统时钟，
  `rosmsg show robot_vision/LaserDecision` 确认了新三环、标定和连续计时字段。
  Nano 无 RTC 电池，按操作者要求不再处理 RTC；图像会话中没有放宽时间戳保护。
- 审计目录 `/home/abot/air_camera_live_audit.WfB10n`：199 帧/10 秒，到达频率
  20.011 FPS，header 间隔频率 19.999 FPS，最大间隔 51.55 ms；无零、重复、
  倒退时间戳和工具错误。1920x1080/rgb8，发布者 `/circle_usb_camera`，frame_id
  `circle_usb_optical_frame`。接收减 header 时间约 8.47--140.94 ms，尚未证明
  真实传感器曝光延迟。ROS 工具必须保留 source 环境，使用 `python3`，不能带
  会忽略 ROS PYTHONPATH 的 `-E`；首次带 `-E` 的调用在导入 rospy 时失败。
- 独立 `circle_laser_test.launch` 使用 field_85cm 配置、`start_camera=false`、
  `enable_laser=false`，日志仅启动检测器。3 条实时样本均为一个三环目标，
  共同中心约 (1060.8, 519.0)，算法置信度 0.956--0.959，处理 53--55 ms，
  header 年龄 0.185--0.219 秒。它们是短样本证据，不能外推成长时间稳定通过。
  每条均为 `calibration_unconfirmed`、`request_on=false`、`DRY_RUN_OFF`，
  无预测光斑、光斑命中或三秒成绩。
- 完全遮挡靶纸后，回传 `missing_or_inconsistent_rings`，观察/目标/瞄准/标定/
  光斑字段均 false，请求 OFF，连续时间为 0。只证实遮挡后的软件状态，未测
  从遮挡开始到撤销的精确延迟，且当时本来就没有开光。
- Ctrl-C 停止 USB 相机后，检测器继续输出 `reason=image_timeout`，
  `enable_laser=false`、`observation_valid=false`、`request_on=false`、
  `hit_count=0`、`target_valid=false`、`observed_hit=false`、
  `continuous_seconds=0.0`。watchdog 的断流状态通过 dry-run 核对；不代表
  ON 到 OFF 的实际切换、物理激光关断或 0.35 秒上限的实测验收。

- 恢复同一 USB 相机后，回传 seq 17/18/19 三条新观测，时间戳递增；
  `observation_valid=true`、`target_valid=true`，原因重新变为
  `calibration_unconfirmed`。`enable_laser=false`、`request_on=false`、
  `aim_valid=false`、`calibration_valid=false`、`observed_hit=false`，连续计时仍为 0。
  这证明本次断流后短样本识别恢复且未解除未标定保护，不是激光重新点亮验收。

下一项仅核实已有的 PWM 开/关定义和 AUX6 接线/安全保持信息，不试扫输出。
相机--激光偏轴、实测距离范围、
红光斑识别、PWM 开/关值与 AUX6 保持值、未解锁输出、独立物理断电及连续
3 秒实际命中均仍待验收。MAVROS/激光桥未在本轮启动，未操作飞行任务。

## 2026-10-07 按操作员要求核对原始文件夹的 PWM 用法

只读核对工作区 `uav_mission/src/uav_mission.cpp` 及原始 `dongbu-ros.zip`
中同名文件，以下未注释调用与行号一致：

- 第 85 行 `lib_pwm_control(20,20)` 是初始化调用，不能作为激光关值依据。
- 第 257/305 行 `lib_pwm_control(100,100)` 后接服务调用及“开启激光笔”日志。
- 第 268/315 行 `lib_pwm_control(0,0)` 后接服务调用及“关闭激光笔”日志。
- 头文件第 48--50 行说明参数为辅助通道 5、6 的 0--100 API 值，非微秒。
  原库当前 SHA-256 与已保存的只读反汇编记录一致：
  `e055c1e0468cc44c114c2ff408a6303bb9e0e9ece5fcf707fd3489dfb0e2734c`。
  既有审计的构造关系为 `param1=p5/50-1`、`param2=p6/50-1`；50 是映射
  中点所对应的 API 输入值，不因除数是 50 就成为已验证的激光开关值。

可记录“原任务设计意图为 ON=100、OFF=0”，并结合操作员此前的 AUX5 接线
说明作为后续核验依据；不声称当前未解锁固件/接线/极性已实测。原代码同时
改变 AUX6，无法从这些源码确认 AUX6 当前接何设备及独立安全保持值。下一步
先由操作者在激光断电状态查看 AUX6 是否接设备；不试扫 PWM，不执行原任务。
本次未修改激光配置或解除任何确认字段，未运行或重新加载 PWM 库。

- 随后操作者确认 AUX6 空置。连同此前 AUX5 接激光的说明，记录为当前接线
  信息；源码中 ON=100、OFF=0 仍是待现场验证的设计值。空置不证明固件映射、
  未解锁输出或实际激光极性，硬件确认字段继续保持 false。
- 下一步只读取 Nano 的串口设备标识与原 PWM 库 SHA-256，准备核对独立通信
  所需信息；尚未启动 MAVROS，未发送任何 PWM 或飞行控制命令。

## 2026-10-14 子项目收尾与交接

用户明确要求暂不启动飞控，本三环/手持激光子项目到此交接，之后在新对话整合
其他对话调试的穿框与飞行程序。本轮未连接机器人，未启动/停止节点，没有执行
MAVROS、PWM、解锁或飞行，也未核查其他对话的最新运行状态。

- 补录末次串口清单：
  `/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_in_FS_Mode_395B37893233-if00`
  指向 `/dev/ttyACM0`。该设备是否确为飞控 USB 口仍待实物确认，不能据设备名称猜测。
- 用户回传 Nano PWM 库 SHA-256 为
  `e055c1e0468cc44c114c2ff408a6303bb9e0e9ece5fcf707fd3489dfb0e2734c`，
  与本地和既有只读审计一致；这不是运行时 ABI、固件映射或输出测试。
- 新增根目录 `AIR_TARGET_HANDOVER.md` 入口，以及
  `robot_vision/docs/AIR_TARGET_HANDOVER_CN.md` 完整交接。
  其中分开记录离线、ROS dry-run、物理激光三层验收，包含接口、计时/锁存语义、
  标定阻塞项、硬件未知项、构建注意点、复现命令和未来整合阶段。
- 新增 `robot_vision/docs/air_target_handover_snapshot.json`：本地文件/不可变部署包/
  证据文件的 SHA-256 清单。19 个安装目标经 LF 归一化与本机一致；这没有证明
  Nano 在其他对话调试后仍未改变。依赖中本机 `vision.py` 与原 Nano 备份不同，
  三环安装器刻意未覆盖该文件，不得为本项目整包覆盖窗框工作。
- 只读复核原任务：PWM 调用与原压缩包一致；整份本地任务另有航点差异，未由
  本轮改动或恢复。新文档不将原压缩包当作可无条件回滚的全工程基线。
- 关键交接限制：当前 detector/bridge 要求 `armed=false`，不能直接用于定高
  飞行中的激光控制，不能靠删除保护或伪造状态解决。若新任务明确进入飞行激光
  范围，须另行设计授权、输出控制权和安全验证；现代码继续保留为地面测试基线。
- 本轮只做文档与标准库静态检查，未重跑 OpenCV/catkin/真机测试；未改运行代码、
  硬件配置、PWM 库或已上传压缩包。真实偏轴/开关/光斑及连续 3 秒命中仍未验收。
- 交接文档收尾检查：78 项标准库静态核对通过（UTF-8、链接、文件清单、消息字段、
  launch 参数与安全命令范围、19 个部署项及归档哈希）。报告见
  `air_target_handover_checks.json`；这不是新增的算法/ROS/物理激光测试。

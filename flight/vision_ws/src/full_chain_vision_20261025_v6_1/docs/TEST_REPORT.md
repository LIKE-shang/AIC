# 验证报告

> **资料整理说明（2026-10-15）**：本文记录历史测试结果，不表示生成图片永久保留。合成/标注/掩膜图片现已清理，原始输入与测试代码保留；本次清理前后的独立离线回归结果见 [清理记录](../../_diagnostics/WORKSPACE_CLEANUP_20261015.md)，不替换本文原有实机结论。

日期：2026-10-05（Asia/Shanghai）

## 2026-10-05 手持三同心环独立测试

| 类别 | 结果 | 范围/限制 |
|---|---|---|
| 三环几何/尺寸定义/线宽分离 | **40/40 通过** | 合成 OpenCV；35/24/12 cm 按直径参数，8 mm 独立为环宽；真实靶纸未测 |
| 节点回调、时间戳、断流、渲染后过期 | **135/135 通过** | 假 ROS transport；包含目标切换、偏轴、相机光斑反馈、3 秒连续计时；不是 ROS 集成 |
| PWM ELF 只读审计 | 通过 | AArch64、command 187、双辅助通道、SHA-256；未加载/执行库 |
| 合成标注/只读回放 | 通过 | 输出在 `test/artifacts/air_target`；无真实相机时间戳、无硬件访问 |

本机没有 ROS/catkin、MAVROS、飞控和 USB 摄像头，因此未执行 catkin 编译、roslaunch、真实相机审计、MAVROS 服务、PWM 波形或靶面光斑验收。`prediction_only` 或 `camera_observed_3s` 均不能替代物理靶面记录。

> 后续用户回传的 Nano 编译、相机及分布式测试进展见 [ROBOT_TEST_LOG.md](ROBOT_TEST_LOG.md)。用户随后报告棋盘格补丁安装后静态和移动均稳定识别。本报告下方保留历次 Windows 交付的验证范围；新比赛适配器尚未在 Nano 编译或做飞行验收。

## 已执行

| 类别 | 结果 | 范围/限制 |
|---|---|---|
| 现有工程接口阅读 | 完成 | 原 shape_detector、两个 USB launch、Gemini launch/驱动发布源码、PWM 头文件/任务调用/CMake/原动态库 |
| Python 源码语法编译/AST | 通过 | 11 个新增 Python 文件；这不是 ROS catkin 或 C++ 编译 |
| launch/package XML、YAML 解析 | 通过 | 新测试与修正后的旧 shape launch；本地可解析 include 的参数名静态核对 |
| 离线回归测试 | **37/37 通过** | 真正 OpenCV 计算 + 假 ROS transport 调用实际 Python 节点；无 ROS master、无相机/激光 |
| 输入/输出 header | 模拟回调通过 | 图像、边缘图和 LaserDecision 保留 capture stamp/frame_id/seq |
| N/M 帧、断流、过期/重复帧、异常、退出 | 离线通过 | 单调时钟状态机、实际 Python watchdog、关闭回调；不代表飞控执行 OFF |
| 形状识别 | 合成图通过 | 空心框/适度透视框、实心矩形排除、圆环去重、Hough 联合、小反光噪点/椭圆排除 |
| 接口一致性/安全缺省 | 静态通过 | Python/C++ 使用同一个 LaserDecision；带 enable_laser 许可字段；true 分支才创建桥接器；硬件模板默认无效 |
| PWM 二进制 | 完成只读 ELF/反汇编 | AArch64，核实 command=187、param1=p5/50-1、param2=p6/50-1；未加载执行 .so |
| 原工程受保护文件 | **349 个哈希一致** | agv_mission/uav_mission/complete_mission/lib_library/ros_astra_camera-main/usb_cam；其中任务包 30 文件；未修改比赛流程 |
| 合成产物检查 | 已查看 | test/artifacts 下 4 张 PNG，非摄像头实拍 |

最终离线测试环境：Windows，Python 3.12，OpenCV 5.0.0，NumPy 2.5.3。机器人可能使用 OpenCV 3/4；代码兼容 findContours 返回值差异，但部署版本需再运行测试。

以上环境描述属于首次交付。2026-10-05 的适配器检查仅使用本机 Python 标准库，
没有成功加载本机 OpenCV 测试依赖；具体范围见本报告末尾的比赛视觉适配部分。

复现离线回归（机器人 Python3 已有 cv2、numpy、yaml 时也可用；不需要 ROS）：

```bash
python3 /home/abot/abot_ws/src/robot_vision/test/test_vision_tests.py
```

该回归中的受保护文件哈希检查针对本次输入版本；在另一版本工程上如有历史差异应逐个检查，不能通过更新基线掩盖意外改动。

只读库审计工具另外需要 pyelftools、capstone（不是机器人运行依赖）：

```bash
python3 /home/abot/abot_ws/src/robot_vision/tools/audit_pwm_library.py
sha256sum /home/abot/abot_ws/src/uav_mission/lib/liblib.so
```

预期 SHA-256：
`e055c1e0468cc44c114c2ff408a6303bb9e0e9ece5fcf707fd3489dfb0e2734c`

## 未执行，不能声称通过

- **catkin/C++ 编译和链接**：本机没有可用 ROS、catkin、ROS C++ 头文件/编译环境；`wsl.exe --list --quiet` 未列出发行版。只有 Python 语法与静态构建配置检查，不是 C++ 编译通过。
- **ROS launch 展开和 ROS transport 集成**：没有 roslaunch 运行；MAVROS 安装包的 node.launch 参数/插件名需在机器人核对。
- **相机实测话题/帧率/曝光/设备映射**：文档的话题依据本工程源码，不是已经在机器人观察到的话题。
- **Python3 cv_bridge 部署兼容性**：尤其非 Noetic 环境需确认。
- **原 AArch64 动态库真实加载与 ABI 自检**：仅静态核查，桥接器运行时还有架构/布局/行为校验。
- **真实服务调用、拒绝/延迟/失联行为、Ctrl-C 关光 ACK**：未访问 MAVROS/飞控。
- **真实 PWM 波形、哪路连接激光、极性、电源、另一通道机构状态、物理发光/熄灭**：均必须真机验证；绝不把服务 ACK 说成物理已验证。
- **机器人完全静止、两相机并行资源占用**：测试不产生运动指令，但需现场确认没有旧任务并行运行。

C++ 桥接器的 source review 不替代真机故障注入。软件无法在主机掉电、桥本身 SIGKILL、通信失联或硬件失效时保证关光，必须使用独立硬件失效关断/断电。所有真实验收仍为未勾选状态，见 CAMERA_TESTS_CN.md。

## 2026-10-04 原图棋盘格补丁（本地验证）

用户提供原图及实际参数后，已用保存的 640×480 PNG 重现漏检：
只使用原来的内外四边形层级配对路径时结果为 0。
增加四侧棋盘周期证据和局部方差孔洞候选后，本地回放检出 1 个内开口。

本次新增检查结果：
- **50/50 离线回归通过**，包含原有圆形/安全状态机与模拟节点回调。
- 原图、旋转/缩放/亮度派生图检出；四种缺边、指定非周期随机纹理、整片/实心棋盘负样本被拒绝。
- 640×480 原图 60 次算法计时：中位约 19 ms，P95 约 20 ms（本 Windows/OpenCV 5 环境，默认 20 工作线程，不代表 Nano）。
- 新/改 Python 文件通过 Python 3.8 语法解析检查；这不是 Python 3.8 上的实际执行。
- 与原部署包比较，detect_circles、depth_to_gray、公共边缘/轮廓函数内容保持不变。
- 349 个受保护原工程文件仍通过哈希检查。

运行时只更新 vision.py、window_test_node.py、window_test.yaml。
CMake/package/launch、任务程序、圆形控制节点、激光桥均未改。
部署工具只执行校验、备份和这三文件替换；本机没有 Bash/ROS，未在 Nano 执行该安装器。
更详细边界、文件清单、安装与调参见 WINDOW_CHECKER_PATCH_CN.md。

**尚未完成：本补丁在 Nano OpenCV 4.10 上的运行、真实连续识别稳定性、CPU/吞吐、场景误检率及现场无动作验收。**
之前用户的 Nano 编译通过不等于本次算法补丁已真机通过。未部署、未控制激光、未重编比赛任务。

## 2026-10-05 比赛视觉输入适配（本地验证）

用户在上一补丁安装后报告静态和移动均稳定识别，历史“未部署/识别未通过”描述
只表示此前交付时的状态。当前现场进展以 ROBOT_TEST_LOG.md 最新小节为准。

新增代码只适配像素接口，默认隔离，不启动或改动比赛任务：
- **34/34 标准库/Fake ROS 适配器回归通过**，包括真正的后台 wall-clock watchdog。
- N 帧确认、首失效撤销、M 帧缺失诊断、恢复重确认、过期/未来/重复帧拒绝通过。
- 默认话题隔离、双开关旧接口拒绝/显式接受、启动发布者/任务冲突和 remap 检查通过。
- PointStamped 类别、像素坐标、采集时间戳/序号、正常退出撤销模拟回调通过。
- Python3.8 AST 和 XML 静态检查、嵌套 launch 的 include 参数名核对通过。
- 冻结配置保持现有 window_test.yaml 全部值；349 个受保护文件仍与原基线一致。
- 部署包只包含 robot_vision 白名单更新文件，内容哈希、LF 校验表、零 tar mtime 已检查。

本轮检查没有执行 OpenCV：依赖目录读取权限受限，申请扩大权限未获准，
未用其他工具、复制依赖或重装依赖来绕过限制。完整算法/图像回调回归仍需在 Nano 重跑。
本机未执行安装器（没有 Bash/ROS），没有做新适配器的 catkin 编译、ROS 传输、
物理断流/退出消息和无动作验收，更没有飞行对框、测距或穿框验收。

当前矩形任务状态机和旧任务的目标新鲜度缺口已记录在 WINDOW_COMPETITION_ADAPTER_CN.md。
本轮不能宣布“直接用于比赛飞行校准已经可用”。

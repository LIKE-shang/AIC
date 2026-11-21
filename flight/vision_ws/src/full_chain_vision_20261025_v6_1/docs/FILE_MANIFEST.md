# 本次交付文件清单

> **清理后的文件说明（2026-10-15）**：此清单保留历史用途说明；其中派生 PNG（标注图、边缘图、合成图）已按要求删除，不是当前必须存在的文件。用户原始 `test/artifacts/window_checker.png`、真实相机原图、JSON 证据及全部源码仍保留。受保护基线与历史快照未改写。当前删除明细见 [清理记录](../../_diagnostics/WORKSPACE_CLEANUP_20261015.md)。

下列路径均在本次 Windows 工程中。机器人部署时 `C:/Users/jiangzx/Desktop/src` 对应 `/home/abot/abot_ws/src`。

| 类型 | 完整路径 | 用途 |
|---|---|---|
| 修改 | `C:/Users/jiangzx/Desktop/src/robot_vision/CMakeLists.txt` | catkin 消息生成、Python 安装和独立 C++ 桥接器构建；不链接任务程序 |
| 修改 | `C:/Users/jiangzx/Desktop/src/robot_vision/package.xml` | 补齐消息、MAVROS 消息、OpenCV/Python3、相机与显示运行依赖 |
| 修改 | `C:/Users/jiangzx/Desktop/src/robot_vision/launch/shape_detector.launch` | 修正旧人脸参数，改为原形状脚本实际读取的参数 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/setup.py` | 安装独立 robot_vision_test Python 模块 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/scripts/window_test_node.py` | Gemini RGB 窗框识别/像素尺寸/中心/状态标注；只发图像和状态 |
| 修改 | `C:/Users/jiangzx/Desktop/src/robot_vision/scripts/circle_laser_test_node.py` | USB 三同心环识别、偏轴/有效区判定、预测/相机光斑证据分离、连续 3 秒计时、dry-run 和断流 watchdog |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/scripts/depth_preview_test_node.py` | 可选 16UC1/32FC1 深度转换预览，绝不控制硬件 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/src/robot_vision_test/__init__.py` | 独立 Python 模块标识 |
| 修改 | `C:/Users/jiangzx/Desktop/src/robot_vision/src/robot_vision_test/vision.py` | ROS 无关旧窗框/圆形算法及三环识别、偏轴、光斑和标注辅助 API |
| 修改 | `C:/Users/jiangzx/Desktop/src/robot_vision/src/robot_vision_test/safety.py` | 三环连续命中门、单调时钟超时、图像时间戳/内容新鲜度和输出命名空间检查 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/src/robot_vision_test/ros_images.py` | cv_bridge RGB 转换、header 原样保留、图像和状态发布辅助 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/src/laser_test_bridge.cpp` | 真光模式 C++ 双路 PWM 请求构造与返回检查；独立 watchdog、隔离服务 worker、退出 OFF |
| 修改 | `C:/Users/jiangzx/Desktop/src/robot_vision/msg/LaserDecision.msg` | 增加目标/瞄准/标定/相机光斑证据、连续秒数和反馈模式字段；仅独立测试接口 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/launch/window_test_camera.launch` | 只开 Gemini、相机命名/话题/尺寸/可选深度流配置 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/launch/window_test.launch` | 只开窗框识别，可选一并开 Gemini |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/launch/circle_usb_camera.launch` | 只开对地 USB 驱动，设备/分辨率/格式/话题可配 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/launch/circle_laser_test.launch` | 只开圆形测试，可选 USB；缺省不创建真实 PWM 桥 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/launch/laser_mavros_only.launch` | 显式手动运行的 MAVROS 最小通信启动，无完整任务 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/config/window_test.yaml` | 窗框边缘、轮廓、矩形几何与内外框筛选参数 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/config/circle_laser_test.yaml` | 圆形/环形几何、Hough、去重、单目标与图像有效性参数 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/config/laser_hardware.yaml` | 缺省拒绝真实输出的接线确认、通道/开关值/另一路值模板 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/config/laser_mavros_plugins.yaml` | 仅 sys_status/command MAVROS 插件白名单 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/tools/audit_pwm_library.py` | 只读审计原始 ELF 动态库，不加载不执行 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/test/test_vision_tests.py` | 离线形状/状态机/配置隔离/受保护文件回归测试总入口 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/test/test_node_callbacks.py` | 模拟 ROS 传输调用实际节点代码；不是 ROS 集成测试 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/test/protected_files_baseline.json` | 原任务/库/原相机文件 SHA-256 基线，仅用于确认未修改 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/docs/CAMERA_TESTS_CN.md` | 完整中文部署、分终端启动、参数、排错、安全边界和验收手册 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/docs/TEST_REPORT.md` | 已验证/未验证项目和测试环境记录 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/docs/FILE_MANIFEST.md` | 本文件，绝对路径和用途清单 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/src/robot_vision_test/air_target.py` | 三环轮廓配对、比例/椭圆/遮挡检查、偏轴标定、有效区和光斑判定 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/tools/audit_air_camera.py` | 只读 ROS USB 图像话题、设备身份、格式/FPS/时间戳/帧龄审计 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/tools/replay_air_target.py` | PNG/manifest/ROS bag 只读回放、标注和新鲜度报告；永不发布/开光 |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/tools/calibrate_air_target.py` | 固定距离多样本偏轴/光斑误差计算；输出始终 unconfirmed |
| 新增 | `C:/Users/jiangzx/Desktop/src/robot_vision/docs/AIR_TARGET_HANDHELD_CN.md` | 本测试的安全边界、分终端命令、标定、回放、数据清单和三层验收表 |

## 离线生成产物

以下是合成图像，不是真实摄像头画面：
- `C:/Users/jiangzx/Desktop/src/robot_vision/test/artifacts/window_synthetic.png`
- `C:/Users/jiangzx/Desktop/src/robot_vision/test/artifacts/window_edges.png`
- `C:/Users/jiangzx/Desktop/src/robot_vision/test/artifacts/circle_synthetic.png`
- `C:/Users/jiangzx/Desktop/src/robot_vision/test/artifacts/circle_edges.png`

本地 `.vision_test_deps` 是只用于 Windows 离线测试的依赖缓存，不部署到机器人。Python `__pycache__` 为运行缓存，不是交付源代码。

未修改原 `agv_mission`、`uav_mission`、`complete_mission`、`lib_library`、`ros_astra_camera-main`、`usb_cam` 内的文件；未修改原 `shape_detector.py`。

## 棋盘格窗框后续补丁（2026-10-04）

本次运行补丁只替换前述 vision.py、window_test_node.py、window_test.yaml 三份。
其余为本地验证/交付辅助内容；不修改圆形/激光/任务构建。

| 完整路径 | 用途 |
|---|---|
| `C:/Users/jiangzx/Desktop/src/robot_vision/tools/replay_window_image.py` | 纯离线原图回放与计时，不使用 ROS |
| `C:/Users/jiangzx/Desktop/src/robot_vision/tools/install_window_checker_patch.sh` | 本次三文件补丁的安全检查、备份、安装工具；不启动节点 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/docs/WINDOW_CHECKER_PATCH_CN.md` | 补丁范围、参数含义、已测/未测、Nano 更新步骤 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/test/artifacts/window_checker.png` | 用户提供的原始图像 fixture，非合成图 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/test/artifacts/window_checker_result.png` | 对此原图离线标注的结果，不是现场实时通过证明 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/test/artifacts/window_checker_edges.png` | 对此原图离线生成的 Canny 图 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/test/artifacts/window_checker_replay.json` | 此次回放坐标、纹理证据和 Windows 耗时 |
| `C:/Users/jiangzx/Desktop/src/_deploy/window_checker_patch_20261004_v1.tar.gz` | 三运行文件、安装器、校验表及补丁说明；不含原任务或本地 Python 依赖 |
| `C:/Users/jiangzx/Desktop/src/_deploy/window_checker_patch_20261004_v1.tar.gz.sha256` | 包整体 SHA-256 |
| `C:/Users/jiangzx/Desktop/src/_deploy/build_window_checker_patch.py` | 按白名单生成补丁的本地工具，不在 Nano 运行 |

同时更新本地 test_vision_tests.py、test_node_callbacks.py、CAMERA_TESTS_CN.md、TEST_REPORT.md、ROBOT_TEST_LOG.md 和本清单。

## 比赛视觉输入适配（2026-10-05）

完整绝对路径、部署与验收命令见
`C:/Users/jiangzx/Desktop/src/robot_vision/docs/WINDOW_COMPETITION_ADAPTER_CN.md`。

新增冻结视觉配置、安全缺省的独立适配器、两份独立 launch 和标准库回归。
本轮只对 window_test_node.py 增加采集元数据；CMake/package 增加消息依赖和脚本安装。
默认只发布 /window_test/mission_target 与 /window_test/mission_target_status；
校准 launch 强制关闭旧任务话题输出。未修改实际任务或控制代码。

部署产物：
- `C:/Users/jiangzx/Desktop/src/_deploy/window_competition_adapter_20261005_v1.tar.gz`
- `C:/Users/jiangzx/Desktop/src/_deploy/window_competition_adapter_20261005_v1.tar.gz.sha256`
- `C:/Users/jiangzx/Desktop/src/_deploy/build_window_competition_adapter.py`
- `C:/Users/jiangzx/Desktop/src/robot_vision/tools/install_window_competition_adapter.sh`

补丁白名单只含 robot_vision 适配代码、构建文件、测试及文档。
不含任务源码、动态库、相机驱动、本机依赖缓存或飞行启动文件。

## Nano 三环测试分阶段部署（2026-10-06）

- `C:/Users/jiangzx/Desktop/src/robot_vision/scripts/circle_laser_test_node.py`
  改为直接导入 air_target，避免为部署三环功能而覆盖 Nano 的窗框 vision.py。
- `C:/Users/jiangzx/Desktop/src/robot_vision/docs/AIR_TARGET_UPDATE_CN.md`
  记录真实相机话题/FPS、用户备份路径和分阶段更新范围。
- `C:/Users/jiangzx/Desktop/src/_deploy/build_air_target_update.py`
  标准库白名单打包工具；仅整理文件，不安装、编译或连接机器人。
- `C:/Users/jiangzx/Desktop/src/_deploy/test_air_target_update_package.py`
  7 项部署结构、SHA-256、可复现打包、默认隔离和导入边界测试。
- `C:/Users/jiangzx/Desktop/src/_deploy/air_target_update_20261006_v1.tar.gz`
  20 份三环链路源码/配置/文档，加 3 份仅供核对的构建文件与清单。
- `C:/Users/jiangzx/Desktop/src/_deploy/air_target_update_20261006_v1.tar.gz.sha256`
  压缩包整体 SHA-256，Linux LF 格式。
- `C:/Users/jiangzx/Desktop/src/_deploy/air_target_update_20261006_v1_tests.txt`
  本轮 7 项标准库测试执行记录；不代表 ROS/相机识别/物理命中通过。

包不含窗框文件、原任务、相机驱动或 PWM 二进制；build_reference 不能直接覆盖。
此时 Nano 尚未安装新版，未执行 catkin、MAVROS 或激光控制。

## 三环/手持激光子项目交接（2026-10-14）

用户要求暂不启动飞控，本分支以文档收尾，后续由新对话整合穿框与飞行子项目。
下列文件仅为说明与版本追溯，不是新的硬件输出配置：

- `C:/Users/jiangzx/Desktop/src/AIR_TARGET_HANDOVER.md`：根目录入口，包含新对话开场文字。
- `C:/Users/jiangzx/Desktop/src/robot_vision/docs/AIR_TARGET_HANDOVER_CN.md`：完整交接，
  覆盖文件作用、ROS/JSON 接口、实际证据、未完成标定、硬件待确认、分阶段整合方案。
- `C:/Users/jiangzx/Desktop/src/robot_vision/docs/air_target_handover_snapshot.json`：
  本地与不可变安装包的 SHA-256 核对、历史日志和图像证据索引；没有重新验证 Nano 当前状态。
- `C:/Users/jiangzx/Desktop/src/robot_vision/docs/air_target_handover_checks.json`：
  本轮文档链接/命令范围/版本一致性检查结果，不是 OpenCV/ROS/物理测试报告。
- 同步追加 `ROBOT_TEST_LOG.md`，不重写早期记录，不重打包已上传 v1 归档。

当前已完成源码安装、Nano 编译、40 项离线与相机 dry-run 部分验证；偏轴、
MAVROS/PWM 服务、实际红光斑、三秒物理命中未验。地面激光桥的未解锁要求与
未来飞行激光任务不兼容，必须在新任务中另行审阅，不能移除保护直接使用。

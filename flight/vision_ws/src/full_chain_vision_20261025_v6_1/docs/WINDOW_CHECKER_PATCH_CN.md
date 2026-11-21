# 棋盘格窗框补丁：原图回放与 Nano 更新

> **资料整理说明（2026-10-15）**：下文 `window_checker_result.png` 和 `window_checker_edges.png` 等派生输出已清理，原始无损 `test/artifacts/window_checker.png` 和 `window_checker_replay.json` 保留。需要预览时，可使用 `tools/replay_window_image.py` 从原图离线重建；这不影响回归测试。详见 [清理记录](../../_diagnostics/WORKSPACE_CLEANUP_20261015.md)。

日期：2026-10-04。**这是 RGB 识别补丁，不是飞行/运动/激光功能。**

## 1. 问题与修改范围

用户提供的 640×480 原始 RGB PNG 已保存为：
`C:/Users/jiangzx/Desktop/src/robot_vision/test/artifacts/window_checker.png`。
SHA-256：`e7ca4d3b5920864301805bd0048261c3d0f1dbd695f7438dc71420cbc8e66f8a`。

使用用户给出的原参数，原来只依赖内外四边形层级配对的路径返回 0。
棋盘格将外框边缘分割成许多小轮廓；存在内开口轮廓不等于存在完整的内外框配对。
不是 RGB 订阅中断，也不是必须打开深度流。本补丁仍只用 RGB。

运行时只更新以下三个文件；没有修改 CMake、package.xml 或 launch，也没有修改圆形检测函数、激光节点或任何任务程序：

1. `C:/Users/jiangzx/Desktop/src/robot_vision/src/robot_vision_test/vision.py`：保留原配对检测；增加纹理孔洞候选和四侧棋盘纹理验证，共用标注函数。
2. `C:/Users/jiangzx/Desktop/src/robot_vision/scripts/window_test_node.py`：内开口/外框标签、像素宽高、方法与耗时状态；处理帧率上限。
3. `C:/Users/jiangzx/Desktop/src/robot_vision/config/window_test.yaml`：显式新增参数；原用户几何阈值保持不变，`require_inner_frame: true`。

额外配套文件（测试/文档，不参与机器人控制）：
- `C:/Users/jiangzx/Desktop/src/robot_vision/tools/replay_window_image.py`：无 ROS 的图像回放/计时。
- `C:/Users/jiangzx/Desktop/src/robot_vision/tools/install_window_checker_patch.sh`：补丁包安装器，校验、核对 overlay、备份，只替换三个运行文件；不启停节点。
- `C:/Users/jiangzx/Desktop/src/robot_vision/test/test_vision_tests.py` 和 `C:/Users/jiangzx/Desktop/src/robot_vision/test/test_node_callbacks.py`：新增回归用例。
- 本文以及原有 `C:/Users/jiangzx/Desktop/src/robot_vision/docs/CAMERA_TESTS_CN.md`、`C:/Users/jiangzx/Desktop/src/robot_vision/docs/TEST_REPORT.md`、`C:/Users/jiangzx/Desktop/src/robot_vision/docs/ROBOT_TEST_LOG.md`、`C:/Users/jiangzx/Desktop/src/robot_vision/docs/FILE_MANIFEST.md`：说明和检查范围更新。

## 2. 检测依据与边界

优先保留正常空心框的层级配对路径。附加路径从 Canny 四边形或局部灰度方差掩膜的孔洞提取候选；孔洞凸包需满足填充度，所有候选仍需通过面积、凸性、角度、对边比、宽高比等几何检查。

候选四条外侧带必须**分别**通过边缘占比、亮暗像素占比、标准差，以及棋盘格沿边方向的交替/重复周期检查。
不是只看一个四边形，也不是只检测一片黑白纹理。识别成功后，同一位置只保留一个结果。

- `method=checker_texture` 或 `checker_contour` 时，绿色轮廓和 `OPENING W/H` 表示近似内开口。
- `method=nested_frame` 时，绿色轮廓为外框、青色线为内框。
- W/H 是更接近图像水平方向/竖直方向的两组对边平均长度，单位 px；不是实物尺寸。
- 单张 RGB 不能证明真实可穿越空洞，印刷图案、相似背景仍可能误检。本补丁不测距、不定位、不产生控制输出。
- `/window_test/edges` 仍是原始 Canny 边缘图，不是纹理掩膜；棋盘格边缘碎裂本身不代表新路径失败。

## 3. 已完成 / 尚未完成

Windows 离线环境：Python 3.12、OpenCV 5.0.0、NumPy 2.5.3。

- 50/50 离线测试通过，包括假 ROS transport 回调；不是 ROS transport 集成测试。
- 原图检出 1 个内开口；约中心 `(313.8, 285.3)`，宽 288 px、高 221 px。
- 原图派生的旋转 ±8°（同时缩放 0.85）、缩放 0.65/0.8、亮度乘 0.8/1.15 均检出 1 个。
- 分别遮去四侧任意一侧、整幅棋盘、实心棋盘板、指定随机纹理孔洞负样本均为 0。这些不是现场误检率测量。
- 合成普通窗框与棋盘窗框可同时检出并去重。
- 本地 60 次 640×480 原图算法计时：中位约 19 ms、P95 约 20 ms；OpenCV 默认 20 工作线程。**不能外推成 Nano 帧率**，也不包括网络、绘图、ROS 序列化延迟。
- 新 Python 文件通过 Python 3.8 语法兼容性检查；受保护的 349 个原工程文件哈希通过。
- 对比上次部署包，`detect_circles`、深度转换、公共 Canny/轮廓函数未改；圆形/激光回归仍通过。
- 用户此前回传的 Nano 独立工作空间编译已通过，但**本补丁尚未在 Nano 运行/验收**。本地未重做 catkin/C++ 编译。
- 真实连续识别稳定性、不同距离/角度/曝光、Nano CPU/处理耗时、无线延迟、机器人实际保持静止仍需现场验证。

预览：`C:/Users/jiangzx/Desktop/src/robot_vision/test/artifacts/window_checker_result.png`。
计时及候选信息：`C:/Users/jiangzx/Desktop/src/robot_vision/test/artifacts/window_checker_replay.json`。

## 4. 部署（不要停止相机，不要运行完整任务）

### Windows PowerShell：只上传补丁

```powershell
scp "C:\Users\jiangzx\Desktop\src\_deploy\window_checker_patch_20261004_v1.tar.gz" "C:\Users\jiangzx\Desktop\src\_deploy\window_checker_patch_20261004_v1.tar.gz.sha256" abot@192.168.0.100:/home/abot/
```

### Nano：先停止仅运行识别节点的那个终端

在此前 `roslaunch robot_vision window_test.launch start_camera:=false ...` 的终端 Ctrl-C。
不要停止 roscore 或相机的独立终端。若相机与识别是同一个 launch 启动的，先不要直接关闭整个 launch，需拆为原说明中的两个终端。

Nano 新终端执行（安装块使用子 shell，不会因失败退出外层终端）：

```bash
source /opt/ros/noetic/setup.bash
source /home/abot/abot_ws/devel/setup.bash
source /home/abot/vision_camera_test_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=192.168.0.100
(
  set -e
  cd /home/abot
  sha256sum -c window_checker_patch_20261004_v1.tar.gz.sha256
  STAGE="$(mktemp -d /home/abot/window_checker_patch.XXXXXX)"
  tar -xzf /home/abot/window_checker_patch_20261004_v1.tar.gz -C "$STAGE"
  bash "$STAGE/install_window_checker_patch.sh"
)
```

只有看到 `WINDOW_CHECKER_INSTALL_OK` 再继续。安装器会：

1. 校验包内文件；核对当前 `rospack find robot_vision` 及 Python import 指向同一源码。
2. 仅接受 `/home/abot/abot_ws/src/robot_vision` 或 `/home/abot/vision_camera_test_ws/src/robot_vision`（解析 symlink 后）。
3. 拒绝覆盖与上次部署包和本补丁均不一致的本地文件，保护用户自己做过的修改；此时不要强行绕过校验。
4. 检查 `/window_test` 已停止。ROS master 不可达时中止，不自动启动 master。
5. 把三份旧文件备份到 `/home/abot/window_checker_backup.XXXXXX`，替换后做 import/空白图 smoke test；失败时尝试恢复，保留备份。
6. 设置窗户脚本为 755。没有修改原工作空间 `agv_mission` 编译白名单，没有调用 catkin，也没有启动任何节点。

这三份是已有 Python/YAML 文件，在上述已核对源码 overlay 的 devel 模式下不需要重新编译 C++；重启 Python 识别节点即可。若 overlay 检查失败，先贴出输出，不要在原工作空间盲目重新 catkin_make。

### Nano：仅重启识别

```bash
roslaunch robot_vision window_test.launch \
  start_camera:=false image_topic:=/camera/rgb/image_raw
```

另外一个已 source 同样环境的 Nano 终端：

```bash
rosparam get /window_test/checker_fallback
rosparam get /window_test/require_inner_frame
timeout 15s rostopic echo -n 1 /window_test/status
timeout 10s rostopic hz /window_test/image
rosnode info /window_test
```

头两个参数都应为 `true`。目标完整进入画面时，期望状态包含 `state: found`、`count: 1`、`targets`、`outline_kind: opening` 和 `processing_ms`；实时结果以现场输出为准。
`timeout ... hz` 到时以 124 结束是人为结束持续测量，不是单独的图像故障依据。

### Linux 物理机：查看

```bash
source /opt/ros/noetic/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=192.168.0.181
rosrun rqt_image_view rqt_image_view
```

下拉选择 `/window_test/image`。已有 rqt 可以继续用，不必重复启动。
可选 `/window_test/edges`；不要把 `/camera/rgb/image_raw` 的未标注画面当成识别结果。

## 5. 调参 / 排错

参数只在节点启动时加载。修改 YAML 后只重启识别节点；单独 `rosparam set` 不会刷新已经缓存的算法参数。
不要同时跑两个 `/window_test`，不要切到 `/object_position`。

| 现象 | 操作 |
|---|---|
| 没有新增参数或 status 没有 targets | 核对当前 `rospack find robot_vision`、Python 模块实际路径、是否真的重启、是否用了旧 config_file |
| 目标太远太小 | 首先让完整窗框占据足够画幅；附加路径要求面积占比至少 `checker_min_area_fraction=0.08`。确有远距离样本再逐步调低，并重跑负样本，不要先禁用 require_inner_frame |
| 靠近导致窗框出画 | 四边必须完整可见；不应通过放开边界检查把缺边当成完整窗框 |
| 棋盘太大/太小或曝光偏移 | 保存新的原图再核对 `checker_min_cell_px/max_cell_px`、`checker_dark_threshold/light_threshold`、亮暗占比。不要一次放宽所有阈值 |
| 反光背景误检 | 保持内外框要求；可提高最小面积/占比、`checker_min_alternation` 或 `checker_min_repeat`，结合误检原图回放；提高过头会漏检 |
| 普通单色窗框 | 原 nested_frame 路径仍工作；必要时 `checker_fallback: false` 关闭附加路径，不影响普通配对检测 |
| 结果变慢 | 默认 `max_processing_fps: 10.0`，故结果不再要求达到相机 30 FPS；查看 Nano `processing_ms`，它是转换+算法耗时，不是端到端延迟；可调 5.0 降 CPU，0.0 取消上限但不保证更快 |
| Nano 有状态，Linux echo 超时 | 优先排查之前已出现的无线延迟和双向 ROS 网络，不把远端 echo 一次超时直接判为算法失败 |
| 深度格式错误 | 窗户节点输入继续用已确认的 RGB `rgb8`；不把 16UC1/32FC1 深度 remap 给它，不需要开启深度 |

继续实机验收时至少记录：完整目标进入/退出画面的状态变化、10 秒结果帧率、若干 processing_ms、倾斜与不同曝光下的原图、缺边/无目标时有无误检。
全程仅识别/标注/显示，不发送运动、任务、MAVROS 或激光命令；现场还需确认无其他旧任务在运行。

# 棋盘格窗户参数与比赛接口适配

日期：2026-10-05，Asia/Shanghai。

## 结论和本轮范围

可以复用本次成功的 RGB 窗框识别算法与参数，作为比赛视觉输入的候选配置。
**目前不能直接作为自动飞行对框、穿框的已验收配置。**
用户已报告：Nano 安装棋盘格补丁后，静态和移动相机时均能稳定识别。
这是用户提供的现场结果，不是本轮代理远程执行的验收。本轮没有连接 Nano，
没有启动相机、MAVROS 或任何任务节点，也没有执行解锁、飞行或激光命令。

本轮只做三件事：冻结识别配置、适配旧比赛像素消息格式、提供地面视觉校准启动。
未修改 agv_mission、uav_mission、complete_mission 的源码、launch、状态切换或控制参数。
此前已经验证的地面矩形、飞行矩形和降落流程不受本补丁修改。

## 参数来源

`window_competition_vision.yaml` 完整复制当前 `window_test.yaml` 的参数值，
保留 Canny 50/150、epsilon_ratio=0.02、min_area=1800、空心框限制、
四侧棋盘纹理证据以及 max_processing_fps=10。没有重新猜一套阈值。

已收到的实际相机信息是 `/camera/rgb/image_raw`、rgb8、640x480、约 30 FPS。
算法仍只处理 RGB，不读取深度、不测距离；棋盘检测路径的轮廓是近似内开口。
默认保留 nested_frame 路径，它输出外框中心。诊断记录 method 和 outline_kind，
现场要核对不同路径是否造成中心偏移，不能将二者未经核对地视为同一穿框参考。

**未读取 Nano 当前参数服务器。** 如果成功识别时还用 rosparam 调过参数，
先在仍运行的检测节点上保存实际生效值，不要用本地配置覆盖现场调参：

```bash
rosparam dump /home/abot/window_field_verified.yaml /window_test
rosparam get /window_test
```

停止检测后这些私有参数可能被 roslaunch 删除。重新启动时可用
`vision_config:=/home/abot/window_field_verified.yaml` 加载已保存配置。
每次更换镜头、曝光、安装角度、尺寸、场地光照都需要重新检查误检和漏检。

## 为什么还不能直接飞

本地代码检查发现：

- `uav_mission/src/uav_mission.cpp` 当前主路径是
  `1 -> 2222 -> 3333 -> 4444 -> 5555 -> 17`；最后进入降落。
- 对框 `case 2` 和穿框 `case 3` 不在这条已验证路径里。修改视觉 YAML 不会自动启用它们。
- 旧 `object_recognize_track()` 接收 `geometry_msgs/PointStamped`，
  `frame_id="square"`，x/y 是图像像素；横向中心硬编码为 320。
- 旧追踪函数没有消息时间戳/断流检查，会使用缓存目标。它只修正横向 Y，
  不是完整的相机内参、外参、距离和飞行闭环校准。
- 现有调用中的 20 像素、0.10 位置步长和 reverse=1 不是此次视觉测试测得的控制参数。

因此不能只把新识别节点接到 `/object_position` 就起飞。尤其不能仅凭
`horizontal_centered=true` 决定向前穿框。RGB 像素宽高不是米制宽高，z=0 不是零米距离。

## 新接口与默认安全行为

`window_competition_adapter` 订阅 `/window_test/status`，默认只发布：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/window_test/mission_target` | geometry_msgs/PointStamped | 比赛格式候选像素坐标，隔离输出 |
| `/window_test/mission_target_status` | std_msgs/String，JSON | 有效性、原因、像素误差和确认计数 |
| `/window_test/image` | sensor_msgs/Image | 原检测节点的标注画面 |

单一有效目标连续 5 个新的有效采集帧后，输出 square；x/y 是像素中心，
保留采集时间戳和帧序号。相同时间戳、乱序、零时间戳、未来/过期帧、多目标、
不符尺寸/编码和不允许的检测方法都会被拒绝。

**首次失效即发布 `frame_id="no object"` 并清空坐标**，不等待 3 帧才撤销。
off_frames=3 仅表示诊断中“确认消失”，恢复始终需要重新确认 5 帧。
这里的 N 帧只表示连续单目标有效，不证明是同一个窗户，也不证明几何抖动足够小。

独立 wall-clock watchdog 在状态断流超过 1.0 秒，或采集年龄超过 0.8 秒时撤销坐标。
定时等待不用 ROS 时间，即使 /clock 停止也检查断流；时间戳仍必须有效。
默认未来容差 0.2 秒。参数存于 `window_competition_adapter.yaml`。

adapter.launch 的同名启动参数优先于 YAML，包含 on_frames/off_frames、
status_timeout、max_capture_age/future_tolerance、预期尺寸和话题/双开关。
只改 YAML 中这些字段不会覆盖 launch 默认值；单独启动适配器时用对应 launch 参数调整。
min_width_px/min_height_px、allowed_methods 等没有同名启动参数的字段由 YAML 生效。
calibration.launch 则固定采用安全默认值和双开关 false。

启动、正常退出发送 no object。节点 SIGKILL、主机断电、网络中断不能保证最后一条
消息被接收；**任务端仍必须独立检查目标新鲜度并执行失联处理。**
此适配器不是飞行安全控制器。所有诊断始终包含 `flight_ready=false`。

默认不发 `/object_position`，不发任何运动话题，不调用任何服务，不控制激光。

## 完整文件清单

下面相对于 Windows 工程的绝对路径；部署后相应包路径为
`/home/abot/abot_ws/src/robot_vision`，独立 overlay 中通常是此包的软链接。

| 完整路径 | 变更及用途 |
|---|---|
| `C:/Users/jiangzx/Desktop/src/robot_vision/config/window_competition_vision.yaml` | 新增，冻结已使用的棋盘格识别配置 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/config/window_competition_adapter.yaml` | 新增，接口确认/超时/尺寸/隔离参数 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/src/robot_vision_test/competition_adapter.py` | 新增，无 ROS 的验证和连续帧逻辑 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/scripts/window_competition_adapter.py` | 新增，独立比赛像素接口适配节点 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/launch/window_competition_adapter.launch` | 新增，只启动适配器 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/launch/window_competition_calibration.launch` | 新增，Gemini/检测/适配器的地面校准，强制隔离 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/scripts/window_test_node.py` | 修改，status 增加采集时间/尺寸/序号，不改变检测阈值 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/CMakeLists.txt` | 修改，geometry_msgs 依赖和新脚本安装 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/package.xml` | 修改，geometry_msgs 和 rosgraph 依赖 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/test/test_competition_adapter.py` | 新增，标准库/Fake ROS 回归，不访问硬件 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/test/test_node_callbacks.py` | 修改，验证采集元数据 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/test/test_vision_tests.py` | 修改，总入口加入适配器测试 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/tools/install_window_competition_adapter.sh` | 新增，白名单/哈希/备份安装器，不启动节点 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/docs/WINDOW_COMPETITION_ADAPTER_CN.md` | 新增，本说明 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/docs/ROBOT_TEST_LOG.md` | 更新，保留历史并记录用户现场成功和新适配状态 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/docs/TEST_REPORT.md` | 更新，本轮已测/未测记录 |
| `C:/Users/jiangzx/Desktop/src/robot_vision/docs/FILE_MANIFEST.md` | 更新，指向本轮完整清单 |
| `C:/Users/jiangzx/Desktop/src/_deploy/build_window_competition_adapter.py` | 新增，白名单部署包构建工具 |
| `C:/Users/jiangzx/Desktop/src/_deploy/window_competition_adapter_20261005_v1.tar.gz` | 生成，小范围源码更新包，不含任务或本地依赖 |
| `C:/Users/jiangzx/Desktop/src/_deploy/window_competition_adapter_20261005_v1.tar.gz.sha256` | 生成，LF 换行整体校验表 |

`vision.py`、`window_test.yaml` 以及圆形/激光运行文件本轮未修改。
小补丁要求已安装上一版 window_checker 补丁；它不覆盖当前 window_test.yaml。

## 上传与安装

安装时停止窗框检测、适配器和全部任务，机器人保持未解锁。相机可以保留运行。
安装器只检查节点，不会自动结束它们；ROS master 必须可达。master 尚未运行时，
单独执行 roscore，不运行完整 bringup。

Windows PowerShell 上传：

```powershell
scp C:/Users/jiangzx/Desktop/src/_deploy/window_competition_adapter_20261005_v1.tar.gz C:/Users/jiangzx/Desktop/src/_deploy/window_competition_adapter_20261005_v1.tar.gz.sha256 abot@192.168.0.100:/home/abot/
```

Nano 安装终端：

```bash
source /opt/ros/noetic/setup.bash
source /home/abot/abot_ws/devel/setup.bash
source /home/abot/vision_camera_test_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=192.168.0.100
cd /home/abot
sha256sum -c window_competition_adapter_20261005_v1.tar.gz.sha256
PATCH_DIR=$(mktemp -d /home/abot/window_competition_patch.XXXXXX)
tar -xzf window_competition_adapter_20261005_v1.tar.gz -C "$PATCH_DIR"
cd "$PATCH_DIR"
sha256sum -c SHA256SUMS.window_competition_adapter
bash -n install_window_competition_adapter.sh
bash install_window_competition_adapter.sh
```

哈希不符或出现“Locally modified target”时停止，不强行覆盖。新包校验文件已使用 LF。
安装器备份到打印的 `/home/abot/window_competition_backup.*`，不会编译或启动节点。

## 编译与离线检查

**只在独立 overlay 编译，不改变原 abot_ws 的 agv_mission 白名单。**

```bash
source /opt/ros/noetic/setup.bash
source /home/abot/abot_ws/devel/setup.bash
PKG=$(rospack find robot_vision)
chmod +x "$PKG/scripts/window_test_node.py" "$PKG/scripts/window_competition_adapter.py"
cd /home/abot/vision_camera_test_ws
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_WHITELIST_PACKAGES=robot_vision
source /home/abot/vision_camera_test_ws/devel/setup.bash
rospack find robot_vision
python3 -c 'import robot_vision_test.competition_adapter; from geometry_msgs.msg import PointStamped; print("ADAPTER_IMPORT_OK")'
python3 /home/abot/abot_ws/src/robot_vision/test/test_competition_adapter.py
python3 /home/abot/abot_ws/src/robot_vision/test/test_vision_tests.py
grep '^CATKIN_WHITELIST_PACKAGES:' /home/abot/abot_ws/build/CMakeCache.txt
```

最后一行仍应为 agv_mission。如果 hash 测试提示 Nano 的历史任务版本与本地基线不同，
核对对应文件，不能通过更新基线掩盖变化。完整算法回归需要机器人已有的 cv2/numpy/yaml。

## 分终端启动

每个 Nano 新终端先执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/abot/abot_ws/devel/setup.bash
source /home/abot/vision_camera_test_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=192.168.0.100
```

终端 1（Nano）：master 没有运行时执行 `roscore`，已运行就不用再启动。

终端 2（Nano，全停状态，包含相机）：

```bash
roslaunch robot_vision window_competition_calibration.launch image_topic:=/camera/rgb/image_raw
```

该 launch 只有 Gemini、window_test、window_competition_adapter。
不要与旧 window_test.launch 同时运行。
如果只有相机已经运行，追加 `start_camera:=false`。
若相机和新 window_test 均已运行，只启动适配器：

```bash
roslaunch robot_vision window_competition_adapter.launch
```

适配器依赖新增 status 采集字段。旧检测进程没有该字段时必须仅重启检测进程。
使用保存的现场参数时，终端 2 的命令为：

```bash
roslaunch robot_vision window_competition_calibration.launch vision_config:=/home/abot/window_field_verified.yaml image_topic:=/camera/rgb/image_raw
```

终端 3（Linux 物理机，192.168.0.181）：

```bash
source /opt/ros/noetic/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=192.168.0.181
rosrun rqt_image_view rqt_image_view /window_test/image
```

终端 4（Nano 诊断，避免 Wi-Fi 延迟影响结论）：

```bash
rosnode list
rosnode info /window_competition_adapter
rostopic info /window_test/mission_target
rostopic echo /window_test/mission_target_status
```

另开诊断终端可查看坐标：`rostopic echo /window_test/mission_target`。
不要运行 uav_mission/agv_mission/complete_mission、object_darknet 或 MAVROS 来做此验收。

## 地面验收标准

1. 节点只有 master、相机、检测、适配器和查看器；无任务/运动节点。
2. 窗户左移、右移时，error_x_px 符号正确；中心约在 (320,240)。记录跳变和抖动。
3. 单目标连续 5 个新有效帧后 output_valid=true，frame_id=square，stamp 随采集更新。
4. 移走窗户或出现多目标，首个失效帧即 output_valid=false，坐标消息为 no object。
5. 仅 Ctrl-C 停止相机，适配器不退出并自动报 stale_capture 或 status_timeout、no object。
6. 恢复相机后需要重新 5 帧；停止适配器前保持诊断订阅，检查正常退出消息。
7. /object_position 没有本适配器发布者，机器人完全不动作、无解锁/模式切换/激光命令。

图像正常而适配器 no_target 时先查 reason：unexpected_image_width/height 要核对尺寸；
missing_capture_stamp/missing_image_dimensions 表示检测进程仍旧或 header 不合格；
stale_capture/future_capture 要查 Nano 时钟和采集时间，不能关闭时间戳检查来绕过；
method_not_allowed/target_absent_or_ambiguous 要核对算法路径及目标数。
不要简单放宽超时掩盖相机断流或处理过载。

## 可选旧接口检查，禁止据此直接飞行

独立 adapter.launch 有双开关的旧格式检查功能，默认都为 false。
仅在地面未解锁、没有任何任务节点/旧检测发布者时，替代默认适配器启动：

```bash
roslaunch robot_vision window_competition_adapter.launch allow_legacy_output:=true publish_legacy_object_position:=true output_topic:=/object_position
```

它要求严格 640x480，并在启动时查询 master；发现旧话题已有发布者或已知任务节点
就拒绝启动。**此检查只在启动时进行，不能阻止随后启动的、重命名的任务或发布者。**
操作者必须保证全程不运行任务。不要将这条命令写进当前 complete_mission 或无人机启动文件。

## 真正接入飞行前仍需完成

- 任务端独立检查采集时间和接收间隔，目标失效后撤销旧目标、悬停/安全退出，
  不因适配器退出或通信中断继续追踪缓存坐标。
- 审核如何从已验证矩形流程进入对框状态，不能悄悄把 2222 改成 2。
- 核对相机镜像、安装姿态、坐标系和左右修正方向；小幅受控响应中证明修正会减小误差。
- 单独验证相机内外参、距离/开口实际尺寸、机体安全间隙和穿框路径。
- 验证检测负荷、延迟、目标抖动、失联/多目标处理，逐级测试，保留人工接管和急停。

本轮 Windows 验证：34 项适配器标准库/Fake ROS 回归通过，Python3.8 语法兼容静态检查、
launch/package XML 静态检查和 349 个受保护文件哈希检查通过。
本轮 OpenCV 完整回归因依赖目录权限限制未完成；历史 50 项通过不代表此次新代码已完整回归。
新增代码在 Nano 的 catkin 编译、ROS transport、断流与退出验收、真实飞行均尚未完成。

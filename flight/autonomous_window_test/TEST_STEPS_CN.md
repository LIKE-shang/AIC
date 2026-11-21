# 自动穿框测试操作手册

适用：Nano 192.168.0.100；Linux 看图机 192.168.0.135；独立包 autonomous_window_test_20261019_v5。
日期：2026-10-19（v5 更新）。

独立自动起降入口已补齐，详见 [独立起降步骤](TAKEOFF_TEST_CN.md)，先做起降再做完整穿框。

## 先说明现在能做到哪一步

程序已实现自动流程并通过离线测试，Nano 已验证传感器和只读预检。
**最近预检为 ready=false：当前还不能直接执行真实飞行。**
尚缺本轮物理围栏、定位/停止误差预算、飞控失联策略清单与实测验收，
以及 RGB-D 测量链、仿真、自动起降等验收记录。
这不是需要你把车抬起来；默认模式不要求地面识别窗框，也不使用旧 geometry 文件。

本文把“现在可以操作的地面步骤”和“完成首次验收后才可操作的飞行步骤”分开。
不要通过批量把 verified 改 true、填写零误差或复制旧坐标来消除 ready=false。

## A. 首次安装 v5（本轮已安装，可跳过本节）

2026-10-19 已上传并校验 v5，当前 Nano 可以直接使用。
以下命令仅供后续重新部署到未安装的 Nano；当前不必重复解包。Windows PowerShell：

~~~powershell
scp.exe "C:/Users/jiangzx/Desktop/src/_deploy/autonomous_window_test_20261019_v5.tar.gz" "C:/Users/jiangzx/Desktop/src/_deploy/autonomous_window_test_20261019_v5.tar.gz.sha256" abot@192.168.0.100:/home/abot/
ssh.exe abot@192.168.0.100
~~~

进入 Nano 后，上传成功再执行：

~~~bash
cd /home/abot
sha256sum -c autonomous_window_test_20261019_v5.tar.gz.sha256 && tar --keep-old-files --no-same-owner -xzf autonomous_window_test_20261019_v5.tar.gz
~~~

必须看到校验 OK 且解包无错误。若目录已存在而报拒绝覆盖，先核对版本，不删除目录强行重装。
这一步仅解包，不启动任何节点；不影响已有基础定位和相机。

## 终端安排

| 终端 | 所在机器 | 用途 | 是否保持运行 |
|---|---|---|---|
| Nano 1 | Windows 开 SSH 登录 Nano | 雷达、定位、MAVROS | 整轮保持 |
| Nano 2 | Windows 另开 SSH 登录 Nano | RGB、深度、窗框识别 | 整轮保持 |
| Nano 3 | Windows 第三个 SSH 登录 Nano | 模式准备、预检、启动任务、记录结果 | 操作窗口 |
| 看图窗口（可选） | Linux 192.168.0.135 本机桌面 | 显示相机画面 | 按需保持 |

每个 Nano 终端首先在 Windows PowerShell 执行：

~~~powershell
ssh.exe abot@192.168.0.100
~~~

看到 abot@abot 提示符后，才执行本文的 Bash 命令。

## 0. 当前已经开机运行：先不要重复启动

如果仍是这次会话，定位、相机、RGB-D 和原 RGB 观察器已经运行，直接跳到第 3 步。
此前我启动的 RGB-D 在后台运行，所以即使没有它的前台终端，也不要再启动一份。
当前后台日志：/home/abot/window_rgbd_auto_ground_4156f987.log。

不确定是否运行时，在 Nano 3 输入：

~~~bash
source /home/abot/window_pass_ground_env.bash
rosnode list
~~~

已有 /mavros、/window_pass_lio/laserMapping、/window_pass_competition_pose_session_20261013、
/camera/camera、/window_live_vision、/window_pass_observer 时保留已有进程。
出现部分节点缺失/报错时先处理该组件；不要在已有系统上盲目重新执行 base。

## 1. 重新上电后：启动基础定位

只在基础系统尚未启动时使用。本轮车放在真实起飞地面，机头正对目标窗；
遥控器开启，A 关闭，机体未解锁。初始化时不要移动机体。

Nano 1：

~~~bash
bash /home/abot/autonomous_window_test_20261019_v5/start_base.bash
~~~

该入口只启动基础定位，不启动相机，不解锁、不起飞。保持窗口运行。
若提示 MASTER_ALREADY_RUNNING/版本不匹配，先停在此处，保留原输出。

### 定位桥等待 MANUAL 时

无需遥控器必须有 MANUAL 档。在 Nano 3 先查看：

~~~bash
source /home/abot/window_pass_ground_env.bash
timeout 5 rostopic echo -n 1 /mavros/state
timeout 5 rostopic echo -n 1 /mavros/extended_state
~~~

仅在实际落地、connected=True、armed=False、system_status=3、landed_state=1，
且 A 关闭时执行下面的地面模式请求：

~~~bash
rosservice call /mavros/set_mode "{base_mode: 0, custom_mode: 'MANUAL'}"
timeout 5 rostopic echo -n 3 /mavros/state
~~~

通过条件：读回 mode=MANUAL、armed=False、system_status=3。
mode_sent=True 只表示请求已发出，不代替模式读回。
若出现 system_status=8，不反复请求模式；先核对实际急停状态和飞控日志。

然后确认定位：

~~~bash
timeout 5 rostopic echo -n 1 /window_pass_competition/status
~~~

通过条件：real_output_state=FORWARDING_CONTINUOUS_POSE，real_fault 为空，
horizontal_flags_valid=true，real_output_rate_hz 至少 20。
若启动等待超时或节点已退出，不继续后面的测试；把错误保留用于处理。

## 2. 重新上电后：启动 RGB-D 与识别

Nano 2：

~~~bash
bash /home/abot/autonomous_window_test_20261019_v5/start_perception.bash
~~~

它同时启动 RGB、640×400 深度、RGB-D 检测和旧 RGB 观察器；保持终端运行。
这是感知入口，不是飞行控制入口。
若提示 ALREADY_RUNNING_OR_CONTROL_PRESENT，说明已有相关节点；不要重复启动。

Nano 3 检查真实数据到达：

~~~bash
timeout 5 rostopic echo -n 1 /camera/rgb/image_raw/header
timeout 5 rostopic echo -n 1 /camera/depth/image_raw/header
~~~

两条都应返回 header。最终以第 3 步预检对数据新鲜度、来源和标定的判断为准。
**车在地面显示 no_target 或 no_complete_frame 是允许的，不用垫高，也不需要运行 geometry。**

### 可选：Linux 看图

在 192.168.0.135 的本机桌面终端执行，不要在 Nano 的 SSH 提示符里执行：

~~~bash
source /opt/ros/noetic/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=192.168.0.135
rosrun rqt_image_view rqt_image_view /window_live/image
~~~

下拉选 /window_live/image 看 RGB-D 标注；/window_pass_test/image 看旧 RGB 观察器标注；
/camera/rgb/image_raw 看原图；/camera/depth/image_raw 看深度（可用 Dynamic Range）。
看图机不启动 roscore 或相机驱动。

## 3. 地面预检：现在可以执行

在 Nano 3 执行下面一整块：

~~~bash
RUN_LOG_DIR="$(mktemp -d /home/abot/window_auto_test_XXXXXX)"
export RUN_LOG_DIR
set -o pipefail
bash /home/abot/autonomous_window_test_20261019_v5/run.bash 2>&1 | tee "$RUN_LOG_DIR/preflight.txt"
~~~

默认命令只读检查：不解锁、不请求飞行模式、不发布位置目标。
输出 ground_window_detection_required=false 是正确状态。

- ready=false：停止在地面，根据 blockers 处理具体缺项；返回码 2 是预检未通过，不是起飞失败。
- ready=true 且 blockers=[]：表示这次软件预检通过；还需已经完成第 4 步首次验收。
- ERROR/Traceback：程序错误，不当作检查通过，保留日志。

**当前交付配置预计仍为 ready=false。不要直接跳到第 5 步。**

## 4. 首次真实飞行前的验收：一次性工作，不是每次抬车

| 项目 | 要完成的工作 | 当前情况 |
|---|---|---|
| 安装与高度 | 核对 base_link/EKF 原点与飞控壳体中心关系；复核相机外参 | 24.5 cm 已确认，定位参考点关系待核验 |
| 空中测量链 | 验证 RGB-D 配准、时间同步和米制精度；无需录入每轮窗坐标 | 真实数据流已到，空中测量准确性未验收 |
| 物理围栏/落点 | 用本轮 local 系配置真实起飞柱、窗前/窗后区域和落点 | 尚未填写 |
| 误差预算 | 用实测填写定位误差与停止余量 | 静止采样已有，动态误差和刹停未验收 |
| 失效处理 | 确认当前固件的 Offboard 失联、RC 失联、接管及降落行为 | 参数已读取，不能仅凭参数值认定完成 |
| 自动起降 | 先独立验证自动起飞、超时降落，再允许完整穿框 | 未完成；v5 新增 run_takeoff.bash 独立起降入口 |
| 仿真/故障注入 | 补实际 PX4 环境验证 | 71 项离线测试不是 PX4 SITL |

以上需要结合现场量测和试验由我配合完成，不能靠一条“全部设 true”的命令解决。
v5 已新增独立起降入口，操作见 TAKEOFF_TEST_CN.md；不要把完整穿框入口作为首次自动起飞验收的替代。
未完成时，本文不能提供一条现在就能安全放飞的快捷命令。

## 5. 完成首次验收后的正式自动测试：当前暂不执行

前提：第 4 步验收已完成、现场配置已按证据更新，且本轮第 3 步 ready=true、blockers=[]。
车在地面、起飞和落地区域清空；遥控器留作接管，飞手不同时操作电脑。
保持 Nano 1、Nano 2 运行。Nano 3 使用刚才生成的 RUN_LOG_DIR，执行：

~~~bash
test -n "$RUN_LOG_DIR" && test -d "$RUN_LOG_DIR" && bash /home/abot/autonomous_window_test_20261019_v5/run.bash --execute 2>&1 | tee "$RUN_LOG_DIR/mission.txt"
~~~

**这条命令是真实运动入口：预检通过后会自动请求模式和解锁，不会再询问一次。**
当前配置有运动开关关闭和验收缺项，应该拒绝执行；不要为尝试这条命令绕过检查。

正常阶段依次为：

| 屏幕阶段 | 程序动作 | 操作者动作 |
|---|---|---|
| PRESTREAM / MODE / ARM | 建立设定流，请求 OFFBOARD、解锁并确认反馈 | 飞手观察，无需按 A 启动 |
| TAKEOFF | 保持起点 XY/航向，车底目标约 0.90 m | 准备接管 |
| SEARCH | 稳定后等待空中窗框与深度几何，最多 10 秒 | 不再输入距离或高度 |
| ALIGN | 自动对齐，满足稳定条件后提交有限路径 | 观察飞行 |
| PASS | 穿越并在窗后停稳 | 观察飞行 |
| LAND_REQUEST / LANDING | 请求降落并监视实际模式、落地、解锁状态 | 保持遥控器开启 |
| DONE | 已观察到落地且未解锁 | 再进行收尾 |

SEARCH 超时直接进入降落流程，不穿框。识别成功必须是持续确认且测量检查通过，
不是画面上偶尔出现一个框就前冲。

## 6. 异常时怎样处理

- REFUSED：未通过启动条件，留在地面读取 reason。
- TAKEOVER：程序已锁定交出控制，不会自动恢复任务。
- FAULT：不能当作“已经悬停/已经降落”，飞手按已验证的接管流程处理。
- 位姿或 RC 丢失、控制源冲突、窗内穿越故障，不承诺靠 AUTO.LAND 一条指令解决。
- **Ctrl+C、关闭 SSH、关掉定位进程都不是降落命令。** 飞行中不要靠这些动作结束测试。
- 不把急停当作正常降落操作；正常任务结束由自动降落流程完成。

## 7. 收尾与重试

落地后在 Nano 3 读回：

~~~bash
source /home/abot/window_pass_ground_env.bash
timeout 5 rostopic echo -n 1 /mavros/state
timeout 5 rostopic echo -n 1 /mavros/extended_state
~~~

确认 armed=False、landed_state=1 后才能靠近、调整或停止相关进程。
保留 RUN_LOG_DIR 中的 preflight.txt 和 mission.txt。失败后先看 reason，不在空中重新执行任务。
再次测试需要新的单次任务进程和地面预检；基础定位健康时可保留，不随意重启或混用旧本地坐标。

## 不再使用的旧操作

本自动模式不使用 session.bash geometry/trial/air/autonomous-pass，也不要求手动调用
/window_pass_test/authorize_hold、start_alignment 或 commit_pass。
旧程序仍保留供历史测试，但不能与本自动控制器并行运行。

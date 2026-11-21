# 室内穿窗飞行项目交接（截至2026-10-21，Asia/Shanghai）

## 当前结论和现场状态

用户目标是：从地面位置目标起飞，RGB-D识别窗框并进行一次有界对框，冻结当前定位会话中的窗几何，用MID360/FAST-LIO位置反馈完成直线穿越，正常结束请求POSCTL并观察到真实POSCTL后交还用户手动降落/上锁。任务不自动降落、不自动解锁重试。

截至本交接，最新独立软件为v24.8.1：PASS标称推进0.30m/s，session快速模式独立前向领先上限0.12m；70cm最低机底，90cm起飞参考，3m总路线；一次对框20cm总位移/20cm活动柱。PASS横截面偏差最多8cm，并逐窗边确认冻结几何中的4cm刚性净空没有耗尽。0.30m/s是位置目标推进参考，不是实测实际飞行速度；和12cm总位置跟踪限额共享预算，侧向误差越大可用前向领先越小。用户希望1.5m/s只在最后竞赛速度调优时再讨论；现版本校验上限0.30m/s，未实现/验收1.5m/s。

Nano最新读到boot_id=95d9b4b7-638d-467b-b967-f1454807f685，ROS master及飞行/感知节点未运行。SSH最近再次超时；当前实时状态未知。不要把旧会话的ready结果或trial配置用于此boot。连接恢复后先查boot/run/nodes，再按缺失情况恢复。历史身份仅作记录：boot=95d9b4b7-638d-467b-b967-f1454807f685；该会话没有run_id证据，不可沿用。

v24.8.1独立部署目录：/home/abot/autonomous_window_test_20261021_v24_8_1，归档SHA256 eb7f8e7a91c2df6099a1633bd9dcdeec9266fabd03db9d14d366c6659870e13d；368项离线回归通过。没有该新boot绑定/预检通过记录，没有可用飞行命令。前一个绑定目录属于不同boot，不能复用。

本地为速度调参准备的只读预检助手：_diagnostics/prepare_speed_trial_v24_8_1.py。它读取模板里的traverse_speed_mps、要求显式确认起点/20cm区/3m出口、创建独立绑定和试验目录、运行无--execute预检并只打印飞行命令。最后通过scp更新Nano助手时SSH超时；**不可假定Nano上已有此文件或其哈希**。联网恢复后先检查/install/hash验证该助手，再使用。

## 目标控制参数在哪里

独立候选模板：/home/abot/autonomous_window_test_20261021_v24_8_1/config/session_window.json。

`control.traverse_speed_mps`是PASS推进参考速度，当前有效硬上限0.30m/s。新绑定配置会从模板复制速度；修改模板不会改变已生成的`trial/config.json`。`session_window.faster_pass=true`启用独立前向领先0.12m；通用`control.max_setpoint_lead_m=0.04`应保持原值。`straight_pass.py`把冻结的command speed和lead写入路线，每个周期按速度步进，但受实际进度+lead约束。

每次准备速度trial都选0.12~0.30m/s范围内的值，只改一个速度参数，保持20cm对框/70cm底高/3m路线/净空等其他参数不动。每次均要当前boot/run重新绑定、只读预检；检查输出里的`pass_command_speed_mps`和`pass_maximum_forward_lead_m`。如果将来要超过0.30m/s，不能只编辑JSON；现有任务授权上限、共享配置硬上限和速度/制动/轨迹保护须重新评估和验证。计划在比赛最终速度优化阶段再做，不在当前流程中调。

JSON修改无需C++编译；Python为解释执行。改Python源码则需本地跑相关完整回归、打独立包、manifest哈希验证、Nano py_compile，再重新地面预检。

## v24.8.1运行流程

先连接Nano并确认网络/系统时间仅作诊断；Nano历史显示1970时间，程序按ROS时间和monotonic运行，不要任务中设置系统时间。源ROS环境：

~~~bash
source /home/abot/window_pass_ground_env.bash
~~~

按实际节点状态判断，不重复启动已存在节点。基础系统由v24.8.1`start_base.bash`经锁和版本校验启动既有10Hz MID360/LIO/MAVROS基础；节点核心应唯一：`/mavros`、`/livox_lidar_publisher2`、`/window_pass_lio/laserMapping`。换电或重启生成新的run_id；不可用固定run_id。地面未解锁MANUAL条件满足后运行pose bridge `/window_pass_competition_pose_*`，只恢复缺失桥；不用已有点云图推断已经map relocalization。

在落地、未解锁、系统状态健康、CH7/A关闭时运行v24.8.1`reboot_setup.bash manual`准备地面MANUAL；确认两个新飞控状态消息MANUAL再继续。感知仅启动一次：`bash /home/abot/autonomous_window_test_20261021_v24_8_1/start_perception.bash`。它检查落地未解锁/A关闭和无重复视觉/控制发布者，启动相机、`/window_live_vision`和`/window_pass_observer`，并设置三项计算线程=1。视觉应读回fast_confirmation=true、3个米制帧、0.35秒跨度、相邻帧间隔0.60秒、单帧新鲜度仍0.50秒。

节点齐全、RC急停释放、A关闭、状态健康、定位静置采样完成、参数新读回后，物理确认本次同起点/朝向、20cm活动区、完整3m通道和末端停止区空旷。然后用运动禁用模板生成当前boot/run/source身份和新地面边界绑定；运行只读预检，`ready=true`和`blockers=[]`后才由用户现场运行生成的trial命令`bash <trial目录>/run.bash --execute`。命令执行前不提前手动解锁；异常由用户接管。正常结束程序必须观察到POSCTL，之后用户手动落地/上锁。PILOT_CONTROL只表示实际POSCTL已观察，不代表已落地。

参数每次上电读回。历史已批准：COM_LOW_BAT_ACT=0（仅警告），COM_OBL_RC_ACT=0、NAV_RCL_ACT=3，MPC_Z_VEL_MAX_UP/MPC_ACC_UP_MAX约0.40；其他完整21项期望见新模板。不要远程更改FCU参数。CH6=1050通常为急停释放、1950为急停；CH7=1050为A关闭。以新鲜RC消息为准。

## 本次飞行与问题记录

用户报告v24.7实体穿框成功。对应supervisor日志实际因PASS横截面偏差超过8cm进入FAULT，未报告计划3m走完、straight_pass_completed或程序POSCTL交还；物理穿过窗与监督器路线完成是两个独立事实。应保留用户的实体观察，同时不把日志FAULT写成软件完整成功。

v24.5/早期版本进入ADJUST曾有高度目标截低问题，后修复为保留起飞高度参考。70cm最低机底在v24.7模板启用，所有高度用ground z和旋转整机包络换算，不是local z直接比较。90cm只是起飞参考，不是全程强制高度。v24.8.1保留这一高度策略。

用户报告PASS约停顿，代码审计发现PASS每周期目标增量=min(设定速度×dt, 实际沿程进度+前向领先余量)，旧领先4cm可能在落后时造成停等。高速profile将推进参考提高到0.30m/s、前向领先0.12m，但共享12cm总跟踪向量预算。尚无v24.8.1实飞完成证据，不能宣称顿挫已解决。

最近用户提供v24.5日志显示程序曾进入PASS但在约0.92m时因70cm底高保护FAULT；v24.6候选保留高度目标并提出3m路线。v24.7候选完整离线回归358项。v24.8候选364项。v24.8.1前向领先12cm候选368项。

## 验收和边界

RGB-D、相机外参、body origin、FAST-LIO精度上界、FCU失效行为、SITL/仿真、真实穿框仍未全部验收；必须保持false，不用软件预检替代飞行验收。4cm净空是几何刚性净空，不是相机/定位误差证明。地图文件曾有完整性校验，但当前会话没有旧地图重定位发布者，窗几何只在当前boot/run/pose epoch有效。

3m是从PASS提交位置起的总线段，不是穿窗后再走3m。速度提高后路线仍受3m端点/围栏/窗边净空/70cm底高/8cm横截面/12cm总体跟踪限制；如果真实姿态或定位失效则可能FAULT，不承诺还能保持位置。有限直线运动时间预算55秒，全任务上限75秒。

## 证据索引

- v24.8.1地面/参数和绑定：`_diagnostics/v24_8_confirmed_preflight_20261021.txt`、`_diagnostics/v24_8_bound_config_20261021.json`。
- 用户v24.8.1飞行日志：`_diagnostics/v24_8_first_flight_20261021.log`（以附件归档名实际为准；若不存在，请核对`_diagnostics/v24_8_first_flight_20261020.log`）。
- v24.8.1离线测试：`_diagnostics/v24_8_1_tests_20261021.txt`；包哈希和部署：`_diagnostics/v24_8_1_deployment_20261021.txt`。
- v24.5最终70cm/3m版本说明：`autonomous_window_test/FLOOR_HEIGHT_V24_7_CN.md`。
- 视觉快速确认：`autonomous_window_test/VISION_CONFIRMATION_V24_4_CN.md`。
- 调整阈值：`autonomous_window_test/DISPLACEMENT_ARRIVAL_V24_5_CN.md`。
- 本摘要为当前最终交接：`autonomous_window_test/HANDOFF_FINAL_20261021_CN.md`。

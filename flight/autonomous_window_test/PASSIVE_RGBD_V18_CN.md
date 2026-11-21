# v18 原地空中 RGB-D 采集试验（2026-10-19）

独立部署：/home/abot/autonomous_window_test_20261019_v18。
试验：/home/abot/rgbd_observe_trial_998h_m3n/config.json。
入口：bash /home/abot/rgbd_observe_trial_998h_m3n/run.bash，默认只读；--execute 只由用户现场执行。
原 autonomous_window_current_trial.path 仍指向 v16 使用的 takeoff_trial_1ktk5o40，本试验不用该指针。

用户在上次实飞后确认已回原测量起点且朝向一致，已重新绑定当前会话。原验收false字段保持，未扩大起降commissioning豁免到穿框。

飞行范围：完整高度目标自动起飞，固定起点XY，车底约0.90m；稳定后HOVER采集识别结果约5秒，然后请求POSCTL，必须实际观察到该模式才进入PILOT_CONTROL，用户手动降落/上锁。感知结果不生成运动目标，不进入SEARCH/ALIGN/PASS，enable_traverse和pass_stage_approved保持false。无目标也按悬停计时交还，异常仍保留原故障处理。

仅起降测试水平容差从12cm单独设为18cm；垂直容差12cm，20cm起飞柱、围栏、姿态四元数有效性、定位与估计器时效/有效性、独占控制等保留。完整任务/穿窗仍用原12cm跟踪阈值，未修改对准/净空预算。起降角度豁免沿用既有授权；本轮没有向穿框扩展。

acquisition=takeoff_rgbd_observe。预检要求新鲜RGB-D状态和深度、calibration_valid，以及感知节点来源唯一且安装参数一致；无需地面识别完整窗框。HOVER期间记录去重source_stamp、target_count、检测原因、metric_geometry及validate_metric结果；只验证现有检查条件，不声明米制精度验收。

最终日志增加rgbd_observation，含unique_samples、validated_metric_samples及samples。位置目标仅由TakeoffCore支持的固定柱生成，状态采集不能调用几何控制器。空中采集通过也不授予穿越权限。

离线158项通过（包括原地18cm范围、垂直保护、完整任务阈值隔离、被动回调去重/来源/阶段隔离），不是SITL或实飞。已完成绑定预检ready=true/blockers=[]；实际飞行尚未执行。

## v18.1 预流退出调查与修复

首次v18执行在PRESTREAM、尚未请求OFFBOARD/ARM时，收到mode=OFFBOARD，按保护锁存TAKEOVER。没有解锁，没有采集空中数据。预流约0.68秒后退出，定位误差约0.58cm，非18cm跟踪阈值触发。

事后只读NSH实测：commander_state.main_state=7，vehicle_status.nav_state=2、failsafe=true，offboard_control_signal_lost=true；RC CH6/CH7=1050，无其他控制目标发布者/任务进程。强烈支持上次Offboard失流后内部主模式仍保留OFFBOARD意图，外部显示的POSCTL是失流降级；新的预流恢复后导航重新进入OFFBOARD。没有录制当时所有命令来源，不能声称完全排除瞬时外部请求。

在落地、未解锁、status=3、急停释放、A关闭、无目标发布者时显式请求POSCTL，读到3条新POSCTL；随后NSH main_state=2、failsafe=false。未写参数，无解锁/目标发布。

v18.1启动修改：仅--execute在预检通过后、创建目标发布者前，执行受保护显式POSCTL请求，必须读到两条新状态确认。默认只读入口不请求模式；启动条件变化/拒绝/未确认即停止。PRESTREAM中意外模式切换仍锁存，不放宽保护。

163项离线回归通过。运行文件已备份为Nano scripts/auto_window.v18_before_intent_reset.py，更新后runtime_build=20261019_passive_rgbd_takeoff_v18_1_ground_intent_reset，预检ready=true、blockers=[]。试验入口及配置未改变；尚未实飞。证据：_diagnostics/v18_prestream_exit_audit_20261019.json、v18_prestream_console_20261019.json、v18_commander_internal_before_20261019.json、v18_explicit_ground_posctl_20261019.json、v18_commander_internal_after_20261019.json、v18_1_final_preflight_20261019.txt。

## 用户附件确认的本次空中采集结果（2026-10-19）

用户已将Nano关机换电；不可复用旧boot/run/node绑定或旧窗中心局部坐标。当前无需SSH探测关机设备。

已解析用户完整附件并本地保存终态：_diagnostics/v18_1_airborne_capture_terminal_20261019.json；统计：_diagnostics/v18_1_airborne_capture_result_20261019.json。

结论：已取得空中窗框米制数据，后续无需再专门重复这一轮原地采集飞行，可以进入下一阶段窗前识别/对框测试准备。本次不是完整穿框验收，metric_accuracy_verified/free_corridor_verified仍false，不修改原验收字段。

85条状态记录（不是85帧有效几何），37个通过现有validate_metric的不同时间戳样本，跨度13.448秒。测量开口宽1.06984~1.07370m、高0.80458~0.81224m，分别接近用户提供1.05/0.80m。平面RMS5.63~7.91mm、深度内点96.53~99.86%；这些是内部拟合质量，不是绝对测量精度证明。

41条processing_result_stale，另有1条processing_exceeded_freshness_limit；有效样本最大间隔1.760秒，实时连续性需要处理后再考虑穿越。旧live_geometry.acquire遇到metric_geometry_ready=false会清空连续确认，因此即使本次可采集，不代表当前对框控制能稳定持续获取几何。不得简单增大新鲜度阈值绕过。

已观察HOVER；约15秒后因hover_stability_timeout进入HANDOVER_REQUEST，实际观察POSCTL后PILOT_CONTROL，handover_confirmed=true。takeoff_hover_completed=false，说明未达到代码要求的连续稳定悬停5秒，不能因几何采集成功改成true。仅终态缺少速度/高度时间序列，尚不能断定稳定性超时的唯一触发量。日志终态仍armed=true/airborne=true，不以用户换电报告替代本次遥测落地证明。

用户意图：既然采集成功，后续跳过重复采集，继续下一阶段。下次上电后只做必要的新会话核对、感知恢复与现场归位绑定，不机械重复本次独立采集；仍需解决实时数据间歇过期及窗前对准验证，不能直接用旧坐标下发穿越。

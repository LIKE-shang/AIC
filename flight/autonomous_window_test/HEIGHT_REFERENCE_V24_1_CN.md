# v24.1高度参考衔接修复（2026-10-20）

用户最新明确：按其提供的规则，80cm是最低底高，不要求固定90cm。90cm保留为起飞初始参考，不是SEARCH到达精度、对框或PASS的必须高度。对框采用本次深度测得的开口与中心；平放35cm高机体对中1.05m窗中心时机底为0.875m，允许低于0.90m，但仍检查实际/计划姿态下最低点不低于0.80m。此处不是本轮已测得现场窗高，也不是新增加到达条件。

v24本轮日志：747.947 TAKEOFF，756.010 SEARCH，756.313因session_search_height_floor请求POSCTL，756.953实际观察POSCTL。无ADJUST/PASS，无冻结窗几何。两条几何帧在SEARCH起点时间之前而被跳过，不足以断定相机识别失败。

到达时目标local z=0.856201706，实测z=0.766479254，相差0.089722452m；当时瞬时机底检查刚达到80cm。源代码在进入SEARCH时将完整目标改成全部实测pose，确实下调了约9cm高度指令。随后触发最低高度保护。可确定该指令衔接缺陷，单凭终态日志不能证明后续全部下降只由该缺陷引起。

修复：进入SEARCH仅重设保持XY和yaw，Z继续保持原完整起飞目标，不再向低处截断；依旧20cm到达带、解锁请求后3秒/观察airborne，无新速度或精密稳定门槛。可信窗框获取后的对中目标仍由几何决定，可低于90cm。真实80cm最低点保护保留。

补充height诊断及handover_snapshot：保存触发交还前的ground z、实际/目标最低底高、相对80cm余量、原目标及误差，避免HANDOVER将目标改为当前位置后丢失原指令证据。

317项离线测试通过；新增记录中8.972cm提前到达仍保持完整参考、真实79.9cm底高触发并保留原目标快照、87.5cm对框完整通过合成闭环。不是SITL或实飞验收。证据_diagnostics/v24_first_flight_20261020.log和v24_1_height_reference_tests_20261020.txt。

复核现场已POSCTL、armed=false、landed_state=1、CH6=1950、CH7=1050，system_status=8结合本次主动急停解释，不能直接当作新故障。本次不远程执行--execute，不自动重试。原v24试验入口仍是旧代码，后续需确认本轮归位及急停释放后新绑定和预检，不给旧命令冒充修复版。

已独立部署/home/abot/autonomous_window_test_20261020_v24_1，全量manifest哈希及Nano Python编译通过。归档SHA256=817fdd2bb36ff5d7ee9cdeec47263ba3373e7e740150489128e8b6dedd7f105f。旧v24试验目录未改，无新现场执行命令，无修复版实飞结果。

## v24.1重新归位确认与预检通过（2026-10-20）

用户“是”明确确认本次已回到原测量起点和朝向、调整区域与窗后通道仍空旷，承接上一条开关释放/要求测试。使用独立v24.1部署重新采样绑定，新试验/home/abot/v24_1_session_pass_trial_k4vn25x0/config.json。未复用旧local起点或旧场地绑定。

只读预检ready=true、blockers=[]、exit=0；runtime_build=20261020_session_window_v24_1_height_reference；POSCTL、armed=false；定位连续性ok/epoch0，地面高度跨度0.012088m，21项参数读回匹配。session_window已启用，简化起飞已启用且额外XY/yaw/tilt/超调起飞门槛false。未完成的body_origin/RGB-D精度/仿真/FCU故障行为验收仍false，不宣称飞行验收。

新现场命令：bash /home/abot/v24_1_session_pass_trial_k4vn25x0/run.bash --execute。仅用户执行，远程本轮只运行无--execute的预检，无解锁/起飞/自动重试。保持A关闭，不提前手动解锁；观察POSCTL交还后用户手动降落、上锁。80cm是最低底高，90cm只是起飞参考；进入SEARCH不再将高度目标下调到早到达反馈，后续对框仍由实测几何选高。

证据_diagnostics/v24_1_bound_preflight_20261020.txt、v24_1_bound_config_20261020.json。等待v24.1本次实飞结果；旧v24_session_pass_trial_bqt3_x83命令不使用。

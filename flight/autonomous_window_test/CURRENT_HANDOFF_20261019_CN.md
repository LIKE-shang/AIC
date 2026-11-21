# 2026-10-19 v16 实飞与后续工作交接

## 操作者操作习惯

用户明确：每次飞行结束后会按下 CH6 急停，防止意外起飞。之后读到 system_status=8 时，优先检查是否为操作者主动急停，结合新鲜 RC CH6、armed、landed_state 及飞控日志确认；不能只凭 status=8 判断飞控新故障，也不能忽略其他故障可能性。历史通道定义为 CH6 1950 急停、1050 释放。保持急停可继续只读诊断；后续需要健康地面状态的准备操作前再由用户释放。

## 当前绑定及实飞结果

配置指针已更新 /home/abot/takeoff_trial_1ktk5o40/config.json。
当前 boot ID 为 26a25702-6aba-4742-a1c6-95dd6fce79b4；run ID 为 b7fa9f36-f00c-4eb8-b26c-bfd788d5009a。只作历史记录，重启或定位会话变化后不可沿用。

用户现场执行 v16，OFFBOARD/ARM 均接受并观察到解锁。约16秒后横向误差0.121657m超过现有0.12m阈值，在TAKEOFF阶段进入FAULT；目标高度0.847493m、实际0.837044m（局部坐标，不等同离地高度）。估计器有效，接收年龄约0.909秒，未触发估计器超时。程序停止目标发布，退出时飞控POSCTL、armed=true。
用户确认成功人工接管并稳定悬停。不是程序 HANDOVER_REQUEST/PILOT_CONTROL 路径验收，takeoff_hover_completed=false、handover_confirmed=false。之后只读遥测观察到POSCTL、armed=false、landed_state=1、system_status=8；用户说明结束后主动急停的习惯，下一轮核对CH6。

用户希望开始穿框程序测试，并认为原地起飞12cm阈值偏严。尚未修改阈值。应区分起飞保持阈值与穿窗对准/误差预算，不将全地图尺度作为窗洞碰撞余量；肉眼与定位位移差异需检查真实运动和坐标漂移。

## 穿框准备检查发现

现有完整穿框配置为config/airborne_rgbd.json，启用运动/穿越的开关仍关闭，验收字段尚有缺项。当前ROS节点没有相机、RGB-D或RGB观察器。需先核对/启动感知组件，避免重复节点；start_perception.bash 的既有地面启动检查要求system_status=3，当前急停状态下不应绕过。

现有 mission.py 仅在 takeoff_land 支持 pilot_landing_enabled；完整穿框正常完成及部分超时仍请求AUTO.LAND。用户此前要求正常任务结束人工降落，下一阶段必须先解决该策略差异，不能直接发旧完整穿框执行命令。起飞姿态检查豁免不延伸到穿框。

后续先做空中识别/米制几何及对框验证，再做真实穿越；不得伪造验收字段或自动重复飞行。现场解锁/起飞仍由用户执行。

## 本地证据

- _diagnostics/v16_handoff_state_audit_20261019.json
- _diagnostics/v16_rebound_preflight_20261019.txt
- _diagnostics/v16_current_rebound_preflight_20261019.txt
- _diagnostics/v16_postflight_20261019.txt

## 新电池开机恢复（本轮最新状态）

新boot_id=73705d1a-c4c7-4eda-a488-b24497004d1c，新run_id=ae0b86adc6824e2ca7ba5c966f887306。旧试验绑定全部失效。当前未建立新飞行配置，未解锁或飞行。

初连无ROS master，已用已有v14 base_10hz_reboot.launch生成新的UUID启动基础定位；启动包装 /home/abot/start_v19_base_ground.bash，日志 /home/abot/v19_base_ground_20261019.log。未使用带固定旧run_id的v14启动脚本，也未启动50Hz旧入口。

遥控稍后由用户打开。原_pose启动因等待CH7超时已退出，确认无pose发布者后在落地未解锁、status=3、CH6/CH7=1050且无控制源条件下请求MANUAL，读回MANUAL后恢复唯一pose bridge。日志 /home/abot/v19_pose_ground_20261019.log。感知已启动，日志 /home/abot/v19_perception_ground_20261019.log。不要重复启动。

最新12秒采样：MANUAL、armed=false、landed=1、system_status=3；pose bridge FORWARDING_CONTINUOUS_POSE、horizontal_flags_valid=true、10.01Hz，FCU odom约30Hz、XYZ跨度0.02330/0.00941/0.01946m。RGB-D calibration_valid=true、处理约4.96Hz，地面no_complete_frame。无setpoint_raw/local、mavlink/to、hold/align控制发布者。

识别优化版本已核对：Nano已经包含project_rgb_pixels，不应重复部署同一优化。Nano合成目标（不是实景回放）480宽检测中位138.8ms、深度配准90.6ms、平面/开口37.9ms；未增大任何新鲜度期限，尚未修复实景过期问题。合成基准仅提示耗时规模，不能证明真实故障唯一原因。

证据：_diagnostics/v19_restored_ground_audit_20261019.json、v19_synthetic_pipeline_480_timing_20261019.json。换电后的物理归位尚未由用户明确确认：当前“完成”只针对打开遥控器。新的场地绑定前应确认起点和朝向；不能复用先前飞前确认。

上一轮37条有效空中几何样本的结论仍有效，不必机械重复原地采集；当前未实现/授权可执行的独立对框止步配置，不能给用户旧v18或完整穿框入口冒充下一阶段。

最新后续状态见 ALIGNMENT_V19_CN.md：用户已确认本轮归位；对框候选已部署并绑定，但运动仍禁用。视觉重复候选优化已部署。

用户已确认相机前8cm/下6cm/朝向一致；有限对框独立试验政策和待批准配置已准备，详见ALIGNMENT_SCOPE_V19_1_CN.md。该安装确认不等于接受新增未验收项范围；运动仍关闭，待明确的新范围接受。

用户已明确接受v19.1有限对框scope。已建alignment_trial_skvpshd7，预检ready=true；尚未执行飞行。详情见ALIGNMENT_SCOPE_V19_1_CN.md末尾。

2026-10-20 v19.1实飞因起飞航向5.14度触发heading_changed。用户要求放宽起飞偏差，已完成v20阶段容差修改并独立部署、195项离线测试通过。当前急停/落地未解锁，仅运动禁用模板待归位/释放急停后重绑；详见TAKEOFF_TRANSIENT_V20_CN.md。

2026-10-20 用户已归位解除急停，v20新试验alignment_v20_trial_3ofy6env重新绑定后ready=true，待用户现场执行。详见TAKEOFF_TRANSIENT_V20_CN.md末尾。

2026-10-20 v20启动被外接球围栏拒绝，未飞。已修复align_only完整旋转机体包络模型一致性，v20.1独立部署、新试验alignment_v20_1_trial_30btzh_c预检通过，201项测试通过，待用户现场执行。详见TAKEOFF_TRANSIENT_V20_CN.md末尾。

2026-10-20 v20.1真实起飞25秒稳定性超时后POSCTL交还成功，未进入识别/对框。已下载匹配本轮ULog，发现高度/速度稳定判据波动及飞控危险低电告警；详见TAKEOFF_TIMEOUT_V20_1_CN.md。不重复试飞。当前急停、落地已上锁；本地新增诊断203项通过尚未部署。

用户要求按商家原约束。已读取商家CPP/H，完成VENDOR_CONSTRAINT_AUDIT_20261020_CN.md；本地新增vendor_takeoff_arrival_enabled用20cm/解锁请求后3秒替代精密稳定起飞判据，207项通过尚未部署。当前Nano仍v20.1，不能说全部额外约束已解除。

2026-10-20用户明确授权取消低电自动动作；Nano新boot39473f7f-c1cd-480d-b401-0033f963cdfa，已地面修改COM_LOW_BAT_ACT=2->0读回并param save、仅告警，其余保护未改。当前无RC数据，基础10Hz已启动，旧绑定失效。见BATTERY_ACTION_WARNING_ONLY_20261020_CN.md。

## 2026-10-20 v21准备状态

用户“继续”后读回RC已恢复，MANUAL、armed=false、landed=1、status=3，CH6/7=1050。pose桥此前因等待RC超时退出，已仅恢复缺失pose桥，日志/home/abot/vendor_arrival_pose_20261020.log；启动相机/识别，日志/home/abot/vendor_arrival_perception_20261020.log，无重复基础节点。

本轮boot=39473f7f-c1cd-480d-b401-0033f963cdfa、run=12c5b155d483480685d7b3d63b0a01b6。10秒odom XYZ跨度1.04/0.99/1.38cm，bridge持续10.03Hz、horizontal_flags_valid=true、无fault；感知calibration_valid=true、地面no_complete_frame；无控制目标发布者。COM_LOW_BAT_ACT读回0。

v21已独立部署/home/abot/autonomous_window_test_20261020_v21，207项离线测试通过，上传sha256确认。包含商家式起飞到达策略可选项及原始起飞目标/稳定判据诊断。预检报告明确显示takeoff_arrival_policy，避免误解仍用0.04m/s到达门槛。开启该策略后的takeoff_progress不把速度门槛当作有效起飞到达条件，连续稳定需求0秒；实际解锁请求计时>3秒、高度误差<0.20m、观察到离地才推进，16秒无条件推进未搬入。

已生成/home/abot/alignment_v21_prepare_1q4tajjb/config.motion_disabled.json：vendor_takeoff_arrival_enabled=true、COM_LOW_BAT_ACT期望0；enable_motion=false、site_identity=None、围栏清空、本轮地面场地核对字段false。因为用户尚未在此次新boot/换电之后明确确认物理起点/朝向，未绑定或给飞行命令。此前的“已完成”属于上一boot，不能代替本轮确认。下一步只需确认新开机后原位/原方向，再采样绑定、预检；不重复专门空中采集，不扩大对框为穿越。

当前仍有限对框：SEARCH/ALIGN旧的姿态、跟踪、速度/2.5cm对框与有限路径限制仍存在，不应说全部已完全按商家原算法。已如实向用户说明这一差异。飞控低电仅告警，与用户授权一致。

证据_diagnostics/v21_new_boot_audit_20261020.txt、vendor_arrival_final_tests_20261020.txt。

## v21新会话绑定与最终预检通过（2026-10-20）

用户已明确确认本次重启后仍原测量起点/朝向。已创建/home/abot/alignment_v21_trial_vcpqon1p/config.json，新绑定boot39473f7f-c1cd-480d-b401-0033f963cdfa、run12c5b155d483480685d7b3d63b0a01b6及现行节点身份。入口 bash /home/abot/alignment_v21_trial_vcpqon1p/run.bash --execute，仅用户现场执行；默认无execute只读。没有远程飞行。

最终ready=true、blockers=[]，MANUAL、armed=false、landed=1、status=3、CH6/7=1050，COM_LOW_BAT_ACT=0，vendor_height_20cm_after_arm_request_3s，takeoff_stability_speed_gate_enabled=false，window_traversal_permitted=false。原始未验收字段保持false、有限对框commissioning声明继续限定不穿越。

代码最终衔接原有最低飞行高度：厂家20cm到达范围可能提前于车底0.8m，vendor到达必须同时满足当前/目标旋转机体最低点>=原0.8m底高；否则继续TAKEOFF爬升，而非刚进入SEARCH就触发flight_height_floor交还。此为保留已有规则约束、不是新增精密稳定要求。208项离线测试通过（新增最低高度衔接用例）。最新mission.py已复制Nano v21目录；打包归档为最后小修之前版本，后续以现行文件/最新哈希为准。

对框阶段5度、12cm跟踪、2.5cm目标精度、0.04m/s稳定等仍原规则；商家式起飞到达不代表全部控制策略等同厂家程序。等待首次v21实飞结果。证据_diagnostics/v21_final_preflight_20261020.txt、v21_final_tests_20261020.txt。

2026-10-20 v21已起飞进入SEARCH，但10秒后recognition_timeout交还POSCTL。感知同时间段确有stable_live_metric_geometry和16~49连续视觉确认，不能说相机没识别；实际卡住的几何接纳/旧RGB/hold门槛未记录，尚不确定。本地补充search_evidence诊断212项通过未部署；Nano已落地上锁急停。见SEARCH_TIMEOUT_V21_CN.md。

2026-10-20用户要求取消反复对框，改固定高度航向判定后直穿。已实现独立straight_pass模式、3/5最近1秒有效帧且当前必须可行、完整箱体扫窗+有限路径，230项回归通过。Nano独立v22候选和只读shadow验证完成；运动/穿越仍false，旧有限对框例外不可用于新穿越。当前落地上锁急停；详见STRAIGHT_PASS_V22_CN.md。

2026-10-20用户已明确授权开始直穿：判定通过执行，拒绝交还。v22.1独立scope和拒绝直接交还逻辑已部署，237测试通过。当前仍急停已落地，仅运动禁用模板straight_pass_v22_1_prepare_mswzkoix，待现场归位/急停释放及窗后通道确认后重绑。详见STRAIGHT_PASS_V22_CN.md末尾。

## v23已实现并部署，等待本轮现场归位（2026-10-20）
用户要求修正保守包络并自动小幅上下左右修正。独立/home/abot/autonomous_window_test_20261020_v23：连续姿态按边方向支撑，保留位置5cm/跟踪3cm/净空4cm/姿态5度；最多一次预穿越修正，总位移20cm、竖直15cm、速度0.08m/s、8秒、实际累计路径30cm；限已确认起飞柱与最低80cm底高，不向前、不转向。候选3/5确认、ADJUST到位后重新获取移动后新帧3/5，再PASS；无法修正/几何失效/超时交还，真实输入故障保留原逻辑。265项离线测试通过，Nano纯数学耗时约8ms内(非实时保证)，编译/部署哈希通过。详情BOUNDED_ADJUSTMENT_V23_CN.md。
当前落地未解锁、CH6=1950急停、A1050；只读预检阻止仅运动/穿越开关false及ground_state_not_ready。未远程起飞、未给旧入口作新测试。模板/home/abot/v23_motion_disabled_template_20261020.json需新绑定，site_identity=None，现场/地面证据字段false待本轮归位确认；原安装/仿真/精度延期字段保持false。用户物理归位、释放急停、确认小范围修正区域/窗后空旷后，新建v23试验并只读预检，再给现场命令。无需重复授权有界修正。
最终归档SHA256 3c3365d687fcc5d958b47fd146bcfd59a414dd4e2b4aaeb3e1198e59ea9a184a；证据_diagnostics/v23_nano_final_readonly_preflight_20261020.txt、v23_directional_adjustment_tests_20261020.txt。

## 换电重启后v23本轮已绑定预检通过（2026-10-20）
新boot bb35efce-b1e5-4946-ae2f-e8e3966b4b66。最初SSH/ROS未就绪，恢复10Hz基础程序，目录/home/abot/v23_ground_reboot_QH2mPm，session run_id=716d5a9bf5104a4698a95958369ac5b5。初次pose等待RC超时退出，RC恢复后仅重启pose，未重复LIO/MAVROS。落地未解锁/状态3/急停释放/A关闭时已按既有授权请求MANUAL，桥接启动成功；相机和识别经防重复入口恢复。
20秒601个地面odom样本，XYZ跨度0.01859/0.00877/0.02222m；桥接10Hz、无故障、水平有效。用户“完成，我重新更换了电池”及“开始”承接现场归位/朝向/调整区域/窗后空旷确认。
新试验/home/abot/v23_adjust_pass_trial_cpo0f3vw/config.json；现场命令bash /home/abot/v23_adjust_pass_trial_cpo0f3vw/run.bash --execute。新静置绑定未复用换电前局部坐标。只读预检ready=true、blockers=[]、runtime v23、adjustment enabled=true、COM_LOW_BAT_ACT=0及COM_OBL_RC_ACT=0，MANUAL/armed=false，ground_z跨度0.00748m。仅用户现场执行起飞，未远程--execute。
本轮一次左右上下修正后重新识别再穿越，成功/正常拒绝均实际观察POSCTL后交还人工降落，真实故障保留FCU/接管逻辑；首次真实调整/穿越仍未验收。
证据_diagnostics/v23_reboot_bound_preflight_20261020.txt、v23_reboot_stationary_readonly_20261020.json。不要运行旧v22入口或重新启动基础节点。

## 2026-10-20 最新接续：v24仅本地候选，Nano未上电

以SESSION_WINDOW_V24_CN.md为本轮最新实现说明。用户交接提出一次深度几何获取→一次对中→本会话冻结直线→MID360/FAST-LIO反馈穿越→观察POSCTL交还。本地已接入session_window.py、session_localization.py；新增显式session_window配置及simplified_takeoff_enabled。水平31cm有限圆柱+4cm刚性净空，保留35cm标称高度、FCU偏置和连续倾斜占用，不再额外叠加5cm位置+3cm跟踪几何预算；4cm不是已验证误差上界。

新模式起飞取消额外30cm XY、30°yaw、15°tilt、20cm超调门槛；仍完整高度目标、20cm到达带/解锁请求后3秒/观察离地/最低底高、真实输入/场地/超时。一次对中仍限20cm总位移、15cm垂向、不转向不前探；不能说已支持任意距离对框。冻结后无持续视觉依赖；boot/run/source/定位连续性异常使路线失效。314项离线测试通过，不是SITL或实飞。

本轮未SSH、未部署、未起节点、未解锁/起飞，远端仍以历史v23部署为记录，实时状态未知。config/session_window.json为运动禁用、无场地绑定的新模板。旧v23_adjust_pass_trial_cpo0f3vw命令不要运行。等上电再读回参数/节点/遥控/急停/落地未解锁/静置定位，重新确认场地并绑定，检查已有点云地图是否真在用；绝不从已有地图推断已重定位或复用旧窗坐标。

本轮上电后已部署v24但SSH随后断开，最新实时证据与待办见V24_POWERON_20261020_CN.md。基础日志/home/abot/v24_ground_reboot_zamXNE，禁止不检查就重复启动。尚无本轮预检通过或飞行命令。

同轮SSH已恢复且boot未变：v24感知运行、21项FCU参数新读回匹配；原始LIO约10Hz、12秒XYZ跨度1.93/1.20/2.40cm；pose桥/FCU定位尚未恢复，故未预检通过。CH6=1950急停、ALTCTL/落地未解锁，等待本次物理归位/空旷确认与急停释放；详见V24_POWERON_20261020_CN.md末节。

## v24当前会话绑定与只读预检通过（2026-10-20）

用户“已完成”明确承接本次急停释放、原测量起点/朝向及调整区域/窗后通道空旷确认。重新遥测CH6=1050、CH7=1050、landed=1、armed=false、status=3；地面MANUAL请求后最初快速重复读未观察到，后续独立新状态确认为MANUAL，未以service accepted替代观察。仅恢复缺失pose桥，未重启MAVROS/LIO/相机。桥节点/window_pass_competition_pose_v24_recovery，日志/home/abot/v24_ground_reboot_zamXNE/pose_restore.log，输出约10.02Hz、无fault、horizontal_flags_valid=true。

完整12秒FCU odom采364点、约29.99Hz，XYZ跨度0.01550/0.01951/0.02232m；原始LIO约9.98Hz。21项参数新读回匹配。证据_diagnostics/v24_restored_pose_audit_20261020.json。

本次boot仍4229dc11-2d55-49ed-912e-b290e5abcfe6、run_id=8cf638b5e90e4e029fd84576c6ce1ae0。新试验/home/abot/v24_session_pass_trial_bqt3_x83/config.json，绑定本次节点身份/起点/围栏，未复用旧local坐标。场地相关验收依据用户本次确认和静置采样记录；body_origin、RGB-D精度、仿真、FCU失效行为等延期验收保持false。

只读预检ready=true、blockers=[]、exit=0，MANUAL/armed=false，localization_continuity.ok=true/epoch=0，ground_z跨度0.010645m。session_window.enabled=true，horizontal_radius_m=.31/rigid_clearance_m=.04，simplified_takeoff_enabled=true，额外起飞XY/yaw/tilt/超调门槛false，无低速稳定起飞门槛，完整高度位置目标。旧prepass_adjustment.enabled=false正常：本次用session_window的一次对中。

仅给用户现场执行的新命令：bash /home/abot/v24_session_pass_trial_bqt3_x83/run.bash --execute。默认不带--execute为只读。远程没有运行--execute、没有解锁/起飞，未收到本次实飞结果。完成或正常拒绝后观察POSCTL才交还，用户手动降落/上锁；PILOT_CONTROL不代表已落地。换电/定位重启/移动起点朝向后此配置不能复用。

证据_diagnostics/v24_bound_preflight_20261020.txt、_diagnostics/v24_bound_config_20261020.json。本轮仍未在旧点云地图中重定位，穿越使用当前定位会话。Nano时钟1970问题尚未调整，程序使用ROS数据时间/monotonic并检查跳变；不要在任务运行中调整系统时间。

## v24首次实飞与v24.1高度参考修复（2026-10-20）
本次起飞进入SEARCH约0.30秒即因最低底高交还，实际确认POSCTL，无对框/穿越。已发现进入SEARCH将Z目标从0.8562降至0.7665的代码衔接缺陷；v24.1改为保持完整Z参考、实测对框再按几何选高。用户明确80cm为最低、90cm不是对框必须高度，未新增精密起飞门槛。317项离线测试通过；已落地上锁急停，不自动重飞。详见HEIGHT_REFERENCE_V24_1_CN.md。

用户已释放开关并要求测试v24.1。最新只读核对同boot/run、POSCTL/armed=false/landed=1/status=3、CH6/7=1050，21项参数一致；12秒FCU odom XYZ跨度2.04/0.88/1.53cm，桥约9.96Hz无fault，原节点保留。v24.1部署哈希再次匹配，准备新运动禁用模板，未绑定或给飞行命令：本条用户仅明确释放开关，上次已飞行落地，仍需本次原起点/朝向与调整区/窗后空旷确认。证据v24_1_before_test_20261020.json、v24_1_pending_site_20261020.txt。

## v24.1重新归位确认与预检通过（2026-10-20）

用户“是”明确确认本次已回到原测量起点和朝向、调整区域与窗后通道仍空旷，承接上一条开关释放/要求测试。使用独立v24.1部署重新采样绑定，新试验/home/abot/v24_1_session_pass_trial_k4vn25x0/config.json。未复用旧local起点或旧场地绑定。

只读预检ready=true、blockers=[]、exit=0；runtime_build=20261020_session_window_v24_1_height_reference；POSCTL、armed=false；定位连续性ok/epoch0，地面高度跨度0.012088m，21项参数读回匹配。session_window已启用，简化起飞已启用且额外XY/yaw/tilt/超调起飞门槛false。未完成的body_origin/RGB-D精度/仿真/FCU故障行为验收仍false，不宣称飞行验收。

新现场命令：bash /home/abot/v24_1_session_pass_trial_k4vn25x0/run.bash --execute。仅用户执行，远程本轮只运行无--execute的预检，无解锁/起飞/自动重试。保持A关闭，不提前手动解锁；观察POSCTL交还后用户手动降落、上锁。80cm是最低底高，90cm只是起飞参考；进入SEARCH不再将高度目标下调到早到达反馈，后续对框仍由实测几何选高。

证据_diagnostics/v24_1_bound_preflight_20261020.txt、v24_1_bound_config_20261020.json。等待v24.1本次实飞结果；旧v24_session_pass_trial_bqt3_x83命令不使用。

## v24.1再次实飞结果与v24.2（2026-10-20）
本次实际底高约89.4cm，SEARCH持续10秒，拒绝为精确窗中心候选低于80cm/超出修正范围，不是实际飞行低于80cm。相机持续有效米制几何，无ADJUST/PASS，实际POSCTL交还，后已落地上锁急停。用户允许必要时降低下限。本地v24.2先把精确居中硬条件改为在完整开口/净空/80cm底高/原运动范围内选最接近中心的可行位置，不盲目下调下限；323项离线测试通过，新增候选几何/底高/四边诊断。详见FEASIBLE_APERTURE_V24_2_CN.md。旧v24.1命令不代表新算法。

已独立部署/home/abot/autonomous_window_test_20261020_v24_2，全量哈希/编译通过；归档SHA256 ddf0032c39dbfe82a9d8f2a7c853097ff14d185af71290c5b59cb19460b89304。Nano上25次纯数学合成选点检查：最大约3.30ms、平均2.76ms；构造的0.95m中心对应精确居中最坏底高0.748914m，新选择最坏底高0.80m、目标[0,0.1,1.071086]。这是合成数据，不是本次现场窗坐标、也不是实时性保证，无ROS输出或飞行。证据_diagnostics/v24_2_deployment_math_20261020.txt。

当前没有新现场命令；保持既有节点，不用旧v24.1入口重试。下一次现场归位/朝向/空旷区和急停释放确认后，按当前会话新绑定并只读预检，再由用户执行新版本。最低底高仍0.80m，本轮没有将用户条件性放宽意向写成已验证安全的更低值。

## v24.2本次归位后绑定、只读预检通过（2026-10-20）

用户“已完成”承接本次归位、原测量朝向、调整区/窗后通道空旷及急停释放确认。只读遥测：同boot4229dc11-2d55-49ed-912e-b290e5abcfe6/run8cf638b5e90e4e029fd84576c6ce1ae0，POSCTL、armed=false、landed=1、status=3，CH6/7=1050，桥约9.97Hz无fault，21项参数匹配；未重复启动任何节点。

独立v24.2新绑定/home/abot/v24_2_session_pass_trial_k0qpw_5m/config.json，未复用旧local起点。只读预检ready=true、blockers=[]、exit=0，build=20261020_session_window_v24_2_feasible_aperture，定位连续性ok/epoch0，地面z跨度0.008555m。原未完成验收保持false；运动仅在用户现场显式--execute后启动，本轮没有远程飞行。

新的用户现场命令：bash /home/abot/v24_2_session_pass_trial_k0qpw_5m/run.bash --execute。A保持关闭，不提前手动解锁；实际观察POSCTL后交还人工降落/上锁。旧v24/v24.1入口不使用。当前80cm最低底高未降低，v24.2在完整包络可行的前提下允许偏离精确窗中心的一次对框。等待首次v24.2实飞结果。

证据_diagnostics/v24_2_before_test_20261020.json、v24_2_bound_preflight_20261020.txt、v24_2_bound_config_20261020.json。

## v24.2实飞ADJUST退出与v24.3新要求（2026-10-20）
v24.2进入ADJUST（计划6.9cm）后因直线横截面误差3.1296cm交还，无PASS；已落地上锁急停。用户要求更大调整范围及放宽PASS3cm。v24.3新模板：一次修正≤50cm/垂向15cm/8cm每秒/15秒/累计75cm，机心活动柱60cm待现场确认；修复ADJUST截低高度指令，对框使用既有12cm目标跟踪和活动区域约束。PASS上限8cm且冻结窗框逐边净空检查，4cm刚性净空与80cm最低点保留。337项离线测试，旧v24.2命令不使用。详见EXPANDED_ALIGNMENT_V24_3_CN.md。

已独立部署/home/abot/autonomous_window_test_20261020_v24_3，全部manifest哈希及Nano编译通过。包SHA256=39cf11299fc38b64d4799cc6934c6ac5a930be7bf9f073ab23e9d79fa1397302。尚未扩大区域确认/新绑定/预检，无新现场命令，无远程试飞。

## v24.3换电新boot恢复与待扩大区域确认（2026-10-20）

用户换电归原位，后“已连接”确认网络恢复。新boot=13ce4365-b6f7-4456-b9ae-952bc81d5fc2，run=606d03a6576244ff82445f6a4a11d21f。原节点全无，使用既有10Hz launch和新run+锁恢复基础，日志/home/abot/v24_3_ground_reboot_8iCaLl。MANUAL请求后已通过持续订阅观察不同时间戳状态确认；单次wait_for_message重复缓存的准备脚本未启动组件，随后修正为订阅验证。仅恢复一次pose桥/window_pass_competition_pose_v24_3及感知，未飞行/解锁/自动重试。

最新只读：MANUAL/armed=false/status3/landed1，CH6/7=1050，bridge10.03Hz无fault，FCU odom30Hz、12秒XYZ跨度2.18/1.17/1.55cm，21项参数均匹配。感知calibration_valid/result_fresh=true，RGB/depth约30Hz、处理5Hz，地面无完整框。仍无地图重定位，旧地图坐标不可复用。证据_diagnostics/v24_3_poweron_audit_20261020.json。

v24.3全量部署hash再次匹配。根据用户本次归原位完成地面采样绑定，/home/abot/v24_3_bound_pending_area_ysjeuv1a/config.motion_disabled.json。所有运动/穿越开关false，扩大区域确认false，未宣称最终飞行预检通过。采样XYZ跨度1.70/.51/1.65cm。本轮仍需用户明确机心60cm活动范围及桨叶扫过空间空旷（含窗后通道）；“已连接”不能冒充这个确认。确认后可生成新现场试验配置及只读预检，再给用户新--execute命令。原未验收项保持false。不得运行旧v24.2命令或重启现有节点。

## 用户撤回扩大对框范围：v24.3.1（2026-10-20）

用户明确改回20cm，指出后方无60cm活动范围。本轮最新配置覆盖前文50cm/60cm要求：alignment_max_distance_m=.20、alignment_clear_column_radius_m=.20、max_align_distance_m=.20、max_align_path_m=.30、max_align_time_s=8。仍垂向15cm、速度8cm/s、一次对框；保留v24.3连续指令高度衔接、对框12cm目标跟踪、PASS最高8cm且逐边4cm净空检查。无需再确认60cm空旷区域，不将20cm误写为3cm偏差。

337项离线测试通过，包括当前模板实际距离/柱范围、30cm累计路径、8秒时限和PASS8cm值核对。新运行标识20261020_session_window_v24_3_1_20cm_alignment。模板仍运动禁用/无绑定，原安装/精度/仿真验收未改。本轮用户换电归位/开关准备和当前恢复20cm范围指令承接原范围现场准备，后续只读核对新会话后新绑定预检，不复用旧飞行命令。

v24.3.1已独立部署/home/abot/autonomous_window_test_20261020_v24_3_1，hash/compile通过，包SHA256 a2abc30b0bde4c72e03ac4d8b05b781a2431bf7a9eb9015ea040c4a36a8da884。同新boot13ce4365-b6f7-4456-b9ae-952bc81d5fc2/run606d03a6576244ff82445f6a4a11d21f，未重复启动节点。最新地面MANUAL/armed=false/landed1/status3/CH6和7=1050，21参数匹配。

按本轮归位准备及明确恢复原20cm范围生成新绑定/home/abot/v24_3_1_session_pass_trial_dco4r64q/config.json。只读预检ready=true、blockers=[]、exit0，build=20261020_session_window_v24_3_1_20cm_alignment，session_window明确max_distance=.20/clear_column=.20/PASS_limit=.08，ground z跨度0.009804m，定位连续性ok/epoch0。保留原未验收字段false。

新现场命令bash /home/abot/v24_3_1_session_pass_trial_dco4r64q/run.bash --execute，仅用户现场执行；远程未执行飞行。旧v24.2/v24.3扩大范围入口不再使用。等待首次v24.3.1实际结果。证据v24_3_1_before_trial_20261020.json、v24_3_1_bound_preflight_20261020.txt、v24_3_1_bound_config_20261020.json。

## v24.3.1首飞确认超时（2026-10-20）
本次SEARCH10秒超时，无ADJUST/PASS；最后仅两帧safe，实际底高88.8cm，不是高度/修正距离触发。感知处理310~465ms、帧间隔>.4秒反复重置确认。发现恢复直接调用Python入口漏原三项线程=1环境设置，已修复start_components.py并在落地上锁急停后仅重启视觉节点；相机/定位未动。337项回归通过，但尚未验证完整空中RGB-D耗时改善。新视觉PID11576使旧绑定失效，无新命令；原部署manifest有一处已记录补丁。详见ACQUISITION_TIMEOUT_V24_3_1_CN.md。

## v24.4视觉确认简化（2026-10-20）
用户要求检查视觉代码并减少帧数。已实现显式fast_confirmation：米制5帧/.7秒→3帧/.35秒、视觉/米制间隔.4→.6秒、任务3/5→2/5（1.2秒窗口），当前结果新鲜度仍.5秒，几何/前景/会话检查保留。视觉144项、任务342项离线通过，非实飞。预检需实际感知配置匹配。20cm修正和8cm PASS净空联动不变。详见VISION_CONFIRMATION_V24_4_CN.md。

已保存Nano独立候选目录/home/abot/autonomous_window_test_20261020_v24_4和/home/abot/window_live_rgbd_20261020_v24_4；后者复制当前视觉基线，仅更新已逐行核对的rgbd.py/live_node.py/rgbd.launch及测试。全量任务manifest与视觉patch manifest哈希/编译通过。本轮没有切换正在运行的节点、没有给新飞行命令、没有远程试飞。当前运行视觉仍是上一轮已修复线程环境的旧确认策略；新策略未现场启用。候选部署证据_diagnostics/v24_4_candidate_deployment.txt。任务包SHA256 2e9b3ae859790aaa51a2da2c626d3780f378583fdbf95ed752423e6cf688c39d，视觉补丁632fd351652affa3548ca95fbd151a77efbaa7d793489a51586574e7226e6190。

后续地面切换视觉必须确认落地未解锁/无控制发布者，只替换视觉节点，保留相机和定位；指定新src PYTHONPATH、三项线程=1及_fast_confirmation:=true。验证status.configuration实际匹配，再按新任务配置新绑定和预检。不可运行旧v24.3.1试验命令冒充v24.4。

## v24.4地面切换完成（2026-10-20）

用户“可行，开始”授权推进。初次读取POSCTL/armed=false/landed1/status8/CH6=1950，符合地面急停状态；安全核对后仅停止旧/window_live_vision进程，启动独立/home/abot/window_live_rgbd_20261020_v24_4/scripts/live_node.py，_fast_confirmation:=true且三个计算线程变量均1。PID22638，日志/home/abot/v24_3_ground_reboot_8iCaLl/vision_v24_4.log。相机/MAVROS/LIO/pose bridge保持原节点URI，未发飞行模式或解锁/起飞命令。

随后12秒只读采样已读到POSCTL/armed=false/landed1/status3、CH6/CH7=1050（用户在现场释放）；视觉实际configuration fast_confirmation=true,metric_required_frames=3,metric_minimum_span_s=.35,max_track_gap_s=.6,max_image_age_s=.5，calibration_valid/result_fresh=true。桥FORWARDING_CONTINUOUS_POSE无fault，FCU odom30Hz、XYZ跨度.02420/.00902/.01902m。21项飞控参数读回匹配。任务/视觉候选manifest在切换前全量匹配。

本次尚未建立新运动配置、未完整飞行预检通过；需要确认上一轮飞行落地后已回到原测量起点和朝向、原20cm活动范围及出口通道空旷，再新绑定。不能把“开始”当作实测归位证明，不能复用v24.3.1旧试验命令。证据_diagnostics/v24_4_vision_switch.txt、v24_4_ground_audit.json。

## v24.4本次现场确认、新绑定及预检通过（2026-10-20）

用户“已完成”承接本次归位/原测量朝向、20cm调整区域及窗后通道空旷确认。最新boot13ce4365-b6f7-4456-b9ae-952bc81d5fc2/run606d03a6576244ff82445f6a4a11d21f，POSCTL/armed=false/status3/landed1，CH6/7=1050。按新视觉节点身份及新采样起点创建/home/abot/v24_4_session_pass_trial_s26f7onm/config.json，不复用旧窗/场地绑定。

只读预检ready=true、blockers=[]、exit0，runtime_build=20261020_session_window_v24_4_fast_confirmation；实际视觉快确认配置契约检查通过，session_window fast=true/修正.20/活动柱.20/PASS上限.08；定位连续性ok/epoch0，地面z跨度.011716m，21项参数匹配。原安装/精度/仿真/FCU失效验收仍false；不是实飞穿越验收。

用户现场新命令：bash /home/abot/v24_4_session_pass_trial_s26f7onm/run.bash --execute。默认无--execute仅只读；远程本轮未起飞或解锁、未自动重试。A保持关闭，不提前手动解锁，实际观察POSCTL后用户降落/上锁。旧v24.3.1命令不使用。等待v24.4首次实际飞行结果。

证据_diagnostics/v24_4_bound_preflight_20261020.txt、v24_4_bound_config_20261020.json。

## v24.4首飞结果（2026-10-20）
视觉确认已在SEARCH后.657秒完成，进入ADJUST；约5.965秒后反馈累计路程.302386m触发30cm累计上限，实际净位移仅.10865m、目标误差.0478m、底高.89327m。实际POSCTL交还，无PASS；对框1.5cm/4cm每秒/.5秒完成门槛仍在，末态未满足。原窗框同航向平移回算各边余量正但非提交验收。未修改保护或自动重试；SSH reset后超时，当前落地状态未知。详见ADJUSTMENT_PATH_V24_4_CN.md。

## v24.5新boot地面恢复完成，待本次归位确认（2026-10-20）

用户重连后boot48619c2b-0561-43f5-8d05-24000e7b4e42，无ROS进程；恢复已有10Hz基础入口，明确start_pose=false先待地面核对，新run1365bd8f95d84bcaa43f347d6128a79c，日志/home/abot/v24_5_ground_reboot_NkloVj。独立/home/abot/autonomous_window_test_20261020_v24_5，原包hash d397f9c4f2c917d3e1f4affaf6b7035769e2a77deece6e57b412177fd4559c75，manifest/编译通过。

start_perception.bash有CRLF启动失败且未创建节点，已本地与远端规范为LF、bash -n通过，随后防重复恢复视觉。补丁记录远端SHELL_LINE_ENDING_PATCH.json，文件新hash74ebfafdff893d93b8bdafcd049045b5cb4125251d135cce91ad9150da64f967。原manifest该文件记录旧换行hash，不能再声称未注明补丁的全量一致。感知用独立v24.4视觉包，fast配置读回3帧/.35秒/.6间隔/.5新鲜度，RGB/depth新鲜，未重写识别算法。

初始CH6=1950，后只读采样观察CH6/7=1050、落地未解锁，地面请求MANUAL并持续订阅确认两个新状态；仅恢复pose桥/window_pass_competition_pose_v24_5，未重启既有LIO/MAVROS/相机。未解锁/起飞、未运行--execute、未自动重试。完整地面证据_diagnostics/v24_5_full_ground_audit.json。

本次未取得换电后原测量起点/朝向及20cm调整区域/出口通道重新确认，故未生成新场地绑定或现场起飞命令；不将“现在连上了”冒充归位确认。用户确认后以新boot/run和节点身份采样绑定、只读预检再给新命令。旧v24.4入口不可复用。原未验收精度/安装/仿真/FCU行为继续false。

## v24.5本次确认、重新绑定与最终预检通过（2026-10-20）

用户“确定”明确确认换电后原测量起点和朝向、20cm调整区域及窗后通道仍空旷。新boot48619c2b-0561-43f5-8d05-24000e7b4e42/run1365bd8f95d84bcaa43f347d6128a79c持续不变；本次现场MANUAL/armed=false/landed1/status3/CH6=1050/CH7=1050。新绑定/home/abot/v24_5_session_pass_trial_p5nsj17y/config.json，未复用旧boot/local起点。

只读预检ready=true、blockers=[]、exit0，build=20261020_session_window_v24_5_displacement_arrival，session_window relaxed_alignment=true、fast_confirmation=true、20cm修正/20cm柱/PASS8cm。准备脚本在生成前直接调用settings/alignment_policy断言有效到位误差.06、settle0、cumulative_path_gate_enabled=false、arrival_speed_gate_enabled=false，全部通过。预检中的prepass_adjustment为enabled=false旧分支显示的1.5cm/.5秒字段，不是新session分支生效条件；本次真实session策略以上述代码断言和运行输出为准。

21项参数匹配、定位连续性ok/epoch0、地面z跨度.015410m。实际视觉快确认配置契约已通过。原body_origin/精度/仿真/FCU故障验收保持false，不把预检当实飞验收。

新的用户现场命令bash /home/abot/v24_5_session_pass_trial_p5nsj17y/run.bash --execute。只由用户现场执行，本轮远程没有解锁/起飞/自动重试。A保持关闭，不提前手动解锁，实际观察POSCTL后人工降落/上锁。旧v24.4及之前命令不可复用。等待v24.5首次实飞结果。证据_diagnostics/v24_5_bound_preflight_20261020.txt、v24_5_bound_config_20261020.json。

## v24.5首次PASS中途FAULT及v24.6候选（2026-10-20）
用户要求3m并报告框前停留数秒人工接管。实际日志原路线2.372m，仅进约.918m便触发straight_pass_height_floor；actual_bottom=.7996435，非走完路线、无程序POSCTL交还确认。后续只读已落地上锁急停，随后SSH再次超时。发现PASS提交将Z从原参考.833169降至实际.783886；本地v24.6已修复保持参考并双路线验证，显式3m总长度，PASS55秒/任务75秒，速度与4cm领先不改。352项离线通过，尚未部署/预检或给新命令，未自动重飞。详见THREE_METRE_V24_6_CN.md。

## 2026-10-21 v24.7当前方向
用户授权最低底高70cm，随后建议95cm/1m高度替代。代码已支持70cm、起飞参考保留90cm。旧帧回放95cm/1m上边净空分别不足1.69/6.70cm，不采用固定升高方案。358项离线测试通过，包含早到达衔接修复、3m总路线/PASS55秒/保持原Z参考。新boot2d32a7e1-dc0c-400a-bf6d-1daa6745ed27，基础10Hz已恢复run2497de3907794af6b1429fc4b14016cf，日志v24_7_ground_reboot_MEUCaD；最近落地未解锁ALTCTL、CH6急停。无新绑定/执行命令，不能声称走走停停已解决。详见FLOOR_HEIGHT_V24_7_CN.md。

v24.7已独立部署/home/abot/autonomous_window_test_20261021_v24_7，manifest/编译/bash语法通过，包SHA25698fb98814c16148229057b2a65c5f0c3094397d750ac72d44241a86a2a3421fb。防重复入口恢复感知，仅地面运行。未启动pose桥、未绑定或发执行命令；最新实时状态及参数见_diagnostics/v24_7_ground_audit_20261021.json。测试通过不等于现场可飞或满足比赛规则。后续需释放急停/恢复pose桥、确认本次起点朝向和完整3m路线末端停止区域空旷，再绑定预检；不可复用旧命令。

## v24.7测试前检查（2026-10-21）

用户“开始测试”后实际读回CH6/CH7=1050、落地未解锁ALTCTL。同boot2d32a7e1-dc0c-400a-bf6d-1daa6745ed27/run2497de3907794af6b1429fc4b14016cf。检查无控制发布者，仅恢复缺失pose桥：明确请求MANUAL并持续订阅确认两个新状态后启动/window_pass_competition_pose_v24_7，未重启LIO/MAVROS/相机/视觉。日志/home/abot/v24_7_ground_reboot_MEUCaD/pose.log。

随后12秒完整采样MANUAL/armed=false/landed1/status3，桥9.99Hz无fault，FCU odom30Hz、XYZ跨度2.03/1.38/2.64cm；21项参数匹配，视觉快确认配置正确。证据_diagnostics/v24_7_full_ground_audit_20261021.json。v24.7全部manifest匹配，新运动禁用模板/home/abot/v24_7_site_pending_ci5f4kar/config.motion_disabled.json。

尚未将“开始测试”当作本次物理归位/3米完整出口及停止区确认；未生成绑定或飞行命令。最新3m长于此前2.37m规划，需现场确认完整通道，旧boot/短路线确认不能直接复用；后续确认后按当前会话新采样绑定/预检，远程不执行--execute。

## v24.7本次现场确认、新绑定及最终预检（2026-10-21）

用户“没问题”明确承接原测量起点/朝向、20cm调整区域、从PASS提交点起完整3米路线及末端停止区域空旷确认。当前boot2d32a7e1-dc0c-400a-bf6d-1daa6745ed27/run2497de3907794af6b1429fc4b14016cf，地面MANUAL/armed=false/landed1/status3、CH6/7=1050，未重启已有节点。

新采样绑定/home/abot/v24_7_session_pass_trial_hmtft6aq/config.json，只读预检ready=true、blockers=[]、exit0；runtime_build=20261021_session_window_v24_7_70cm_floor，fixed_pass_distance_m=3.0、minimum_bottom_height_m=.7，relaxed_alignment/fast_confirmation=true、修正20cm/PASS偏差8cm。准备脚本断言有效6cm到位、settle0、取消累计路径/到位速度门槛，PASS55秒/起飞参考90cm。预检旧prepass_adjustment enabled=false显示的1.5cm不是session实际到位条件。21项参数匹配、定位连续性ok/epoch0，ground z跨度.009071m。原未验收项保持false。

新用户现场命令：bash /home/abot/v24_7_session_pass_trial_hmtft6aq/run.bash --execute。仅用户现场执行；远程本轮只读预检，无解锁/起飞。正常完成实际观察POSCTL才交还人工降落/上锁；真实FAULT仍需人工/飞控处置。走走停停尚未被实飞证明已解决，4cm领先机制未修改。旧v24.5等入口不复用。

证据_diagnostics/v24_7_bound_preflight_20261021.txt、v24_7_bound_config_20261021.json。等待首次v24.7实飞结果。

v24.8最终0.30m/s、PASS专用0.10m前向领先候选已部署/home/abot/autonomous_window_test_20261021_v24_8，hash/compile/bash检查通过，364项离线测试通过。归档SHA256 f95b30156dcde48cbb374d16d60777973cd37c8644c2fb9e2897c5eae8827e8c。

准备绑定期间发现boot已变成a201f929-601a-47ef-86da-23a3465d6914且无ROS master，停止未完成绑定；/home/abot/v24_8_session_pass_trial_n_5aq2p1仅有binding_template，无可执行试验配置，不能称预检通过。随后恢复既有10Hz launch新run8ed777835316404284546c7ac5cd7330，目录/home/abot/v24_8_ground_reboot_FIna8F；地面落地未解锁、CH6/7=1050，明确MANUAL两个新状态确认后仅恢复缺失pose桥/window_pass_competition_pose_v24_8和感知。无远程飞行/自动重试。

完整静置证据_diagnostics/v24_8_newboot_audit_20261021.json。当前需用户确认这次新boot后仍原起点/朝向、20cm调整区与3m通道/末端停止区空旷，再新绑定/预检，不能用重启前“已归位”直接代替新会话确认。未给新执行命令。用户报告v24.7实物穿框成功保留，程序v24.7仍因8cm横截面偏差FAULT/未完成3m与正常交还，不宣称整个流程验收成功。

## v24.8新boot现场确认及预检通过（2026-10-21）

用户“是的”明确确认本次重启后原起点/朝向、20cm调整区和完整3m通道/末端停止区空旷。实读同boota201f929-601a-47ef-86da-23a3465d6914/run8ed777835316404284546c7ac5cd7330，MANUAL/armed=false/landed1/status3/CH6和7均1050。新采样绑定/home/abot/v24_8_session_pass_trial_umjofg95/config.json，未复用重启前坐标。

只读预检ready=true、blockers=[]、exit0，build=20261021_session_window_v24_8_faster_pass，pass_command_speed_mps=.3、pass_maximum_forward_lead_m=.1、fixed_pass_distance=3m、minimum_bottom=.7；20cm调整/活动柱、8cm横截面、实际逐边4cm净空与总跟踪12cm仍保留。定位连续性ok/epoch0，地面Z跨度.011416m、21参数匹配。原未验收字段保持false。

新的现场命令bash /home/abot/v24_8_session_pass_trial_umjofg95/run.bash --execute。仅用户执行，远程本轮无解锁起飞。A关闭、不提前手动解锁；正常观察POSCTL后人工降落上锁。实际速度待验证，v24.7用户实物穿框成功但程序横截面FAULT未完成3m/正常交还这一事实不改写。旧失败目录n_5aq2p1不可使用。

证据_diagnostics/v24_8_confirmed_preflight_20261021.txt、v24_8_bound_config_20261021.json。等待首次v24.8结果。

## v24.8.1前向领先提高到12cm（2026-10-21）
用户指出10cm领先可能慢。已仅将PASS faster profile max lead升至.12m，0.30m/s不变；与12cm三维总跟踪误差共用，横偏8cm时有效forward约8.94cm。保持8cm横截面限制/每边4cm净空/70cm最低等检查，普通max_setpoint_lead=.04不变。368项测试通过；尚未部署/实飞。详见FASTER_PASS_V24_8_1_CN.md。

v24.8.1 lead12cm独立包已传入并部署源码目录/home/abot/autonomous_window_test_20261021_v24_8_1；全manifest哈希、Python编译和bash语法通过，归档SHA256 eb7f8e7a91c2df6099a1633bd9dcdeec9266fabd03db9d14d366c6659870e13d。368项离线测试通过。仅PASS快模式前向领先上限.12m，速度.30m/s不变、共用12cm跟踪向量预算；其他净空/偏差/高度保护不变。

Nano现boot=95d9b4b7-638d-467b-b967-f1454807f685，无ROS master/相关进程。文件已部署但未运行/预检/绑定。旧v24.8绑定仍引用原build，不用。下次必须先恢复新boot基础/桥/视觉、确认站点和3m通道，重绑预检后才给新命令。

截至2026-10-21完整交接以HANDOFF_FINAL_20261021_CN.md为准。已冻结PASS速度调优至最后阶段；1.5m/s未实现，v24.8.1最高0.30m/s。Nano最新已知boot95d9b4b7-638d-467b-b967-f1454807f685无ROS，需重新连接核对。speed prep助手更新后的上传未获确认，不假定Nano有最新版本。

# v20 起飞阶段容差调整（2026-10-20）

## 本次真实触发原因

v19.1在TAKEOFF刚开始约1.28秒后，heading_error=0.08974746rad=约5.142度，超过原5度航向检查，进入FAULT。横向误差约0.03046m，未触发12cm横向限值；大约0.893m正高度目标差是正常起飞目标差，不是本次失败原因。估计器fresh、有效位正常。终态退出时POSCTL、armed=true，不等于落地。

用户明确要求取消起飞过程的小偏差限制，保留大偏差时人工接管，并要求对照厂家。已重新阅读厂家快照：完整高度目标、各轴约20cm到达判定，未发现对应5度航向/12cm横向起飞中止。厂家原程序没有某检查不等于当前场地/定位可直接取消所有保护。

当前只读遥测落地未解锁、CH6=1950、status=8、CH5=1500、模式AUTO.LAND。急停/落地状态不支持直接重新试飞，不发送解锁或切换模式请求。

## 修改范围

新增mission.takeoff_transient_tolerance_enabled（默认false），仅align_only使用，独立alignment commissioning声明需operator_accepts_takeoff_transient_tolerance=true。本轮用户明确要求放宽起飞小偏差，已保存该请求来源；既有有限对框授权保留。

TAKEOFF以及ARM已观察解锁后的过渡期间：水平目标误差上限0.30m、航向相对初始方向上限30度、倾角上限15度、高度超调上限0.20m。不是全部取消检查。无效四元数、相邻姿态跳变、位置跳变/速度、定位时效、估计器有效位、地理围栏全包络、RC/急停、独占控制、超时、飞控保护仍原规则。

进入SEARCH前必须回到原严格条件：水平偏差<=0.12m、倾角<=原5度、航向偏差<=原5度，同时原高度误差/速度和连续稳定时间条件满足。保持最初位置/航向目标，不自动把漂移后位置或航向写为新参考。否则在起飞25秒超时后可信输入下请求POSCTL交还，不携带大偏差进入对框。SEARCH/ALIGN姿态和12cm跟踪保护立即恢复；禁止穿越。

## 验证与部署

195项离线测试执行通过，包括日志5.14度姿态不再在爬升阶段误中止、0.301m/31度/16度仍故障、高度超调0.201m仍故障、不恢复精度就不进入SEARCH、SEARCH恢复严格航向检查、估计器失效不绕过、独立声明与模式范围。

独立部署 /home/abot/autonomous_window_test_20261020_v20_candidate，上传SHA256已校验。只准备运动禁用模板 /home/abot/alignment_v20_prepare_7_ysa3m7/config.motion_disabled.json；当前场地绑定已清空，ground现场核对字段等待重绑后按证据恢复，不复用飞前旧坐标。没有改飞控参数，没有远程起飞。

下一步需用户确认本次中止后的归位/朝向并释放急停，再核对遥控/状态/定位，重新绑定运行预检，最后才给用户新的现场执行命令。不要运行旧alignment_trial_skvpshd7重试；该入口仍v19.1严格起飞检查。

## v20 新试验已绑定并通过只读预检（2026-10-20）

用户“已完成”确认归位、原朝向、急停释放、A关闭。读取boot仍73705d1a-c4c7-4eda-a488-b24497004d1c；起初实际模式AUTO.LAND、未解锁、落地、status=3、CH5=1500、CH6/7=1050。按既有授权在地面新鲜遥测/无控制源条件下显式请求POSCTL，读到3条新的POSCTL后继续静态采样；最终仍POSCTL、未解锁，未发送解锁或位置目标。

静态样本FCU odom约30Hz，XYZ跨度1.99/1.08/2.15cm；LIO约10Hz，定位桥10.03Hz、horizontal_flags_valid=true、real_fault为空；感知calibration_valid=true、地面no_complete_frame。没有重复节点或控制目标发布者。

新试验 /home/abot/alignment_v20_trial_3ofy6env/config.json；用户现场入口 bash /home/abot/alignment_v20_trial_3ofy6env/run.bash --execute；默认无--execute只读。原旧试验指针未替换。新绑定采样约121帧，当前地面高度/原测量起降点依据用户确认与绑定全包络检查恢复为ground_height/takeoff_column/takeoff_landing_site_verified=true；其余未完成验收字段保持false。

v20只读预检ready=true/blockers=[]，ground z span约0.89cm；COM_OBL_RC_ACT=0、position goal/pilot landing开启、姿态检查开启、穿越许可false。预检takeoff_horizontal_error_limit_m=0.12是正常阶段配置显示，运行TAKEOFF由Mission.transient_takeoff切换为0.30m；不是v20未生效。当前hash与本地完全一致，mission 94a4ef74c7aa14dca1dfb424830831301215fe4a950f4e64d22d8708bb212412。

目前尚未实飞v20。成功对框证据仍alignment_completed=true/handover_confirmed=true/PILOT_CONTROL及用户实际接管落地。失败不能自动重复试飞。证据_diagnostics/v20_ground_prepare_audit_20261020.json、v20_bound_preflight_20261020.txt。

## v20.1 启动围栏模型一致性修复（2026-10-20）

v20现场执行在PRESTREAM前REFUSED，未请求OFFBOARD/ARM。startup_checks前三项true，target_origin_sphere_inside=false。事后已观察POSCTL、armed=false、landed=1、status=3；无本次飞行，无需因该次拒绝自动重绑。

原Controller.inside使用所有姿态外接球，半径sqrt(.62²+.62²+.35²)/2+.07+.04+.05约.632m；当前围栏后界约-.5635m，起点x约0，球检测失败。Mission.corridor按实际旋转机体八顶点+相同.09m余量，起点和目标均可放入。现统一只对align_only的控制器inside使用全旋转机体包络；Mission起飞/实际点检查共用envelope_fence.py，Controller姿态/位置更新、health、advance均调用同一inside。姿态仍实时受保护，倾角大幅变化下实际包络会重算。完整traverse入口保留外接球模型，不扩展穿越授权。

不扩大围栏、不减少position_uncertainty=.05或clearance=.04、不改v20起飞容差，不跳过真实边界检查。预检提前计算当前起点和目标包络余量，避免ready=true后启动才暴露同一几何问题。startup_checks字段改名target_control_envelope_inside以准确表示模型。

201项离线测试通过，包括复现本轮球拒绝/箱通过、六个面真实越界拒绝、旋转箱角越界拒绝、启动和控制统一模型、完整穿越保留原球、不合法姿态失效关闭。

独立部署 /home/abot/autonomous_window_test_20261020_v20_1；新试验 /home/abot/alignment_v20_1_trial_30btzh_c/config.json，入口 bash /home/abot/alignment_v20_1_trial_30btzh_c/run.bash --execute，默认只读。沿用本次地面会话绑定（本次拒绝没有飞行或节点变化），实时身份复核通过。预检ready=true/blockers=[]，POSCTL未解锁；当前后方最小包络余量约0.1435m、顶部目标余量约0.6529m，均已扣原.09m预算。地面3秒Z跨度约0.00635m。没有远程飞行。

注意30cm是水平偏差最大中止阈值，不代表所有方向都可偏移30cm；实际当前围栏后方余量约14cm，围栏仍可更早中止。不要宣称改模型就保证不再发生其他故障。证据_diagnostics/v20_1_preflight_20261020.txt和v20_1_test_output_20261020.txt。

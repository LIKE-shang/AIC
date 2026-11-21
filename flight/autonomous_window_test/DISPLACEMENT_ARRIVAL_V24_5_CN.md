# v24.5净位移与宽松对框到位（2026-10-20）

用户明确要求删除累计折线路程退出，改按实际相对调整起点距离；放宽1.5cm/4cm每秒精密对框要求。新显式relaxed_alignment=true配置：累计反馈路程保留诊断但不作为退出门槛；实际相对调整起点三维净位移上限20cm（不再额外加3cm），同时保留原20cm起飞活动柱、整机围栏、最低80cm底高、前窗距离、定位/RC/模式及8秒时间限制。

到位误差1.5cm→6cm，取消对框4cm/s低速门槛及连续0.5秒稳定要求。必须先让已发参考指令完成低速推进到选定目标；实际进入6cm范围后，以当前实际位置/姿态对本次固化窗框重新进行完整路线判定，成功才冻结PASS。失败请求POSCTL，不以6cm误差直接授权穿越。6cm是本轮工程到位阈值，不是测量精度证明；未取消位姿有效性/突变检测和真实机体高度约束。

保留v24.4快速视觉确认、一次几何固化、无反复视觉对框、PASS最高8cm且逐边4cm净空检查、高度目标连续衔接、正常结束实际观察POSCTL再人工降落。旧模式默认不启用relaxed_alignment，旧算法累计/低速门槛未被静默全局删除。

348项离线测试通过：累计诊断超过30cm仍可继续；5cm误差/8cm每秒不需精密保持即完整重验进入PASS；实际20.1cm净位移拒绝；到位但当前路线擦框拒绝；6cm外继续等待且8秒超时；完整旧模式回归。不是SITL或实飞验收。

用户换电后新boot48619c2b-0561-43f5-8d05-24000e7b4e42。前两次后续SSH超时，后来恢复连接，确认当时没有基础ROS节点，旧绑定失效。现场归位/朝向和空旷区域不可由新定位原点反推。远程不执行--execute。

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

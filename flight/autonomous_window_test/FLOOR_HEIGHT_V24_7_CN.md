# v24.7最低底高70cm及高度方案复核（2026-10-21）

用户明确要求把飞行最低机底高度从80cm放宽到70cm，保留70cm以下检查；随后询问若不易修改能否提高到95cm/1m以满足规则并穿框。已完成70cm实现，保留90cm起飞参考，不将用户条件性替代建议当作必须固定1m飞行。

旧帧回放（v24.5捕获窗口、同XY/航向、圆柱高度35cm/半径31cm、允许倾斜和4cm净空）：机底90cm上边余量+3.3078cm，95cm为-1.6945cm，1m为-6.6969cm。95cm/1m在这份近似几何下不能满足净空，不能宣称提高就保证规则和穿框。不是本次现场坐标或实测精度验收；规则合规未独立核查，70cm按用户本轮试验授权，不保证符合比赛规则。证据_diagnostics/v24_7_height_replay_20261021.json。

新模板mission.minimum_bottom_height_m=.70；起飞、SEARCH、ADJUST、PASS当前/目标最低点均引用统一值，未改成local z=.70。通用旧模式最低仍80cm；session模式明确允许70cm。机体尺寸/偏置/倾角仍参与底高，不把FCU高度当机底。起飞参考仍90cm，不主动命令下降到70cm。

回归发现：最低70cm使厂家到达带可能在剩余向上参考近20cm时进入SEARCH，旧12cm三维跟踪门槛会误判。因此SEARCH/ADJUST中明确正在追踪的向上参考允许垂向最多20cm，水平跟踪仍12cm；SEARCH继续发送完整目标，直到回到既有12cm目标跟踪范围再允许横向对框；不要求低速或连续精密稳定，8/10秒原超时保留，实际70cm底高优先检查，不能因继续爬升分支掩盖低于底线。

包含上一轮本地v24.6候选：3m总路线（从PASS提交点算）、PASS55秒/全任务75秒，PASS保持对框原高度参考并验证实际射线+参考高度射线；不会再次将Z下调到偏低反馈。3m端点仍检查围栏和出口，尚需本次完整3m出口/停止区确认。速度.12m/s和4cm参考领先暂未改动；故不声称已解决走走停停，缺完整动态时序，不能把静止或定位噪声当作唯一原因。

358项离线回归通过，包括75cm PASS不触发旧80cm、69.9cm仍FAULT、旧模式拒绝70cm、早到达衔接继续爬升不精密等待、低于70cm不能被衔接分支隐藏，及3m端点/双高度路线检查。非SITL/实飞验收，无自动重试。

现场新boot2d32a7e1-dc0c-400a-bf6d-1daa6745ed27。初无基础ROS程序，已按既有10Hz launch新run2497de3907794af6b1429fc4b14016cf恢复，start_pose=false等待地面检查；日志/home/abot/v24_7_ground_reboot_MEUCaD。最近读回ALTCTL/armed=false/landed1/status3，CH6=1950急停，CH7=1050。不复用旧绑定，不远程--execute。

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

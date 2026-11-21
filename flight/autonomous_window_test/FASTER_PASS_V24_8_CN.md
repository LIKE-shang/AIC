# v24.8提速试验（2026-10-21）

用户现场报告v24.7穿框成功，要求提速并已归回原处/遥控复位。下载完整任务日志发现程序仍以FAULT退出：2356.165 PASS，2398.352 straight_cross_track_exceeds_geometry_budget，未完成3m/未程序确认POSCTL交还。保留用户实物穿框成功与程序完成false这两个不同结论。末端前向参考仍约领先4cm，约42.2秒走到local x2.03m，不能仅凭末态证明所有缓慢都由领先限制造成。

用户进一步要求更快后最终选择PASS参考速度0.30m/s（原0.12）、前向领先上限0.10m（原0.04）；对框速度仍0.08。无速度前馈，未修改飞控参数，实际飞行速度需现场验证，不能承诺3米10秒。保留PASS横截面8cm、每条边4cm刚性净空、总跟踪12cm、70cm最低点、定位/RC/模式/独占及3m整机围栏检查。上一轮横截面FAULT不因提速而视为已解决。

前向领先与横截面误差共享12cm总体跟踪限额，实际可用lead=min(.10,sqrt(.12²-cross_track²))，不增加新的目标使三维误差预算超限。速度/领先写入冻结路线，运行时配置不能放大冻结值。仅session_window.faster_pass=true生效，通用配置max_setpoint_lead_m仍.04，旧起飞/对框逻辑不全局扩大。

364项离线测试：0.30m/s步进、0.10m领先、停滞反馈不会无限推进、冻结上限不变、总三维误差组合、8cm横截面仍拒绝、独立授权范围、3m完整合成PASS/POSCTL。非SITL/实飞验收。新模板仍运动禁用/无绑定，不重用旧现场坐标。

v24.8最终0.30m/s、PASS专用0.10m前向领先候选已部署/home/abot/autonomous_window_test_20261021_v24_8，hash/compile/bash检查通过，364项离线测试通过。归档SHA256 f95b30156dcde48cbb374d16d60777973cd37c8644c2fb9e2897c5eae8827e8c。

准备绑定期间发现boot已变成a201f929-601a-47ef-86da-23a3465d6914且无ROS master，停止未完成绑定；/home/abot/v24_8_session_pass_trial_n_5aq2p1仅有binding_template，无可执行试验配置，不能称预检通过。随后恢复既有10Hz launch新run8ed777835316404284546c7ac5cd7330，目录/home/abot/v24_8_ground_reboot_FIna8F；地面落地未解锁、CH6/7=1050，明确MANUAL两个新状态确认后仅恢复缺失pose桥/window_pass_competition_pose_v24_8和感知。无远程飞行/自动重试。

完整静置证据_diagnostics/v24_8_newboot_audit_20261021.json。当前需用户确认这次新boot后仍原起点/朝向、20cm调整区与3m通道/末端停止区空旷，再新绑定/预检，不能用重启前“已归位”直接代替新会话确认。未给新执行命令。用户报告v24.7实物穿框成功保留，程序v24.7仍因8cm横截面偏差FAULT/未完成3m与正常交还，不宣称整个流程验收成功。

## v24.8新boot现场确认及预检通过（2026-10-21）

用户“是的”明确确认本次重启后原起点/朝向、20cm调整区和完整3m通道/末端停止区空旷。实读同boota201f929-601a-47ef-86da-23a3465d6914/run8ed777835316404284546c7ac5cd7330，MANUAL/armed=false/landed1/status3/CH6和7均1050。新采样绑定/home/abot/v24_8_session_pass_trial_umjofg95/config.json，未复用重启前坐标。

只读预检ready=true、blockers=[]、exit0，build=20261021_session_window_v24_8_faster_pass，pass_command_speed_mps=.3、pass_maximum_forward_lead_m=.1、fixed_pass_distance=3m、minimum_bottom=.7；20cm调整/活动柱、8cm横截面、实际逐边4cm净空与总跟踪12cm仍保留。定位连续性ok/epoch0，地面Z跨度.011416m、21参数匹配。原未验收字段保持false。

新的现场命令bash /home/abot/v24_8_session_pass_trial_umjofg95/run.bash --execute。仅用户执行，远程本轮无解锁起飞。A关闭、不提前手动解锁；正常观察POSCTL后人工降落上锁。实际速度待验证，v24.7用户实物穿框成功但程序横截面FAULT未完成3m/正常交还这一事实不改写。旧失败目录n_5aq2p1不可使用。

证据_diagnostics/v24_8_confirmed_preflight_20261021.txt、v24_8_bound_config_20261021.json。等待首次v24.8结果。

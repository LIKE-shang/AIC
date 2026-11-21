# v24.3扩大对框范围、连续高度参考和穿越偏差（2026-10-20）

用户明确要求：一次对框不能只允许几厘米，起点可能明显偏离窗中心；穿越3cm直线偏差也过严，应容忍轻微晃动。当前实现将新模板的一次对框总位移上限从20cm扩大至50cm，PASS总横向/竖向合成直线偏差上限从3cm改为8cm，并额外逐边检验冻结窗洞净空。

## 本次v24.2证据

日志2518.597 SEARCH，2519.657 ADJUST，2520.430因adjustment_cross_track交还，2520.985实际POSCTL。无PASS。原计划调整6.906cm，不是只能调整3cm；3cm是调整直线的横截面偏差阈值。触发时横截面误差3.1296cm，其中前向约2.3745cm、垂向约-2.0385cm。actual_bottom=0.799014m也略低于80cm；错误先被cross_track分支报告，不能说只有横向误差。

已获取可信窗框，中心距地约0.988932m，原实际射线四边在扣除包络/4cm净空后仍有0.12485/0.23326/0.16895/0.08460m。这些是本次近似RGB-D几何计算，不是已验收精度。

找到第二处高度衔接缺陷：SEARCH保留的完整Z参考0.852751m在进入ADJUST时又被写成反馈Z=0.777612m，选点也倾向于贴最低界。新实现始终从上一条已发布位置指令平滑移动，不在ADJUST入口重置为反馈；优先保持既有指令高度、横向靠近窗中心，只有开口确有需要才在允许范围内上下调整。仍允许低于90cm的可行路线，80cm最低点未降低。

对本次保存几何的离线回放：新目标[0.06715,-0.07209,0.85275]，最坏底高0.873914m，最紧边扣除包络和4cm净空后剩约0.051726m。保持完整高度在该快照上可行，不代表实际动态跟踪/新一轮必能通过。证据v24_2_first_flight_analysis_20261020.json与v24_3_recorded_geometry_replay_20261020.json。

## 一次对框的新限值

- 总目标位移≤50cm，垂向仍≤15cm；速度≤8cm/s；最多15秒，反馈累计路径≤75cm，仍最多一次调整，不主动向窗前探进、不转航向。
- 机心活动柱半径扩大到60cm，以容纳起飞偏移和侧向修正；整机包络、场地围栏、窗前距离检查不放宽。下一轮必须现场确认扩大区域及桨叶扫过空间空旷，旧20cm柱确认不自动等同新区域确认。
- 对框由前一指令缓慢移向目标，保留既有12cm三维目标跟踪上限，以及实际姿态、围栏、活动柱、最低底高和路径/超时检查；不再用PASS式的3cm直线偏差拦截起飞爬升余量和小范围纠偏。
- 保留原对框到位/速度停稳要求，不把它们加回起飞到达条件。几何仍一次可信获取、固化当前定位会话；对框后按实际出发位置重新校验冻结几何再提交PASS。

## 穿越偏差上限8cm，逐边净空联动

新显式session_window.pass_cross_track_limit_m=.08，不将window_pass通用配置原3cm字段直接改为超出其范围的值；旧模式仍有原语义。冻结路线记录本轮上限，运行中配置变化不能扩大已冻结上限。

实际偏移e=pose-start-progress*direction。对每条冻结窗边，检查原扣减后净余量+投影梯度·e>0。原净余量已经扣除了允许连续姿态范围内圆柱占用和4cm刚性净空，所以侧偏8cm只有在所有边仍满足净空时才能继续；例如上方只有5cm余量，向上偏5cm会提前拒绝，而向下同等偏移不必消耗上边余量。沿路线前进不消耗侧向净空。若偏移总量>8cm，或任何边净空耗尽，或冻结边数据缺失，则停止自动穿越；实际底高/定位/RC/接管机制仍运行。

这不是把8cm误差额外叠加到每边预算，也不是取消4cm净空；4cm及在线偏差阈值仍不能代替外参/定位/延迟精度证明。失去定位时不能保证悬停，正常结束仍观察POSCTL再交还人工降落/上锁。

## 验证和待办

337项离线测试：40cm修正到冻结直穿/POSCTL、超范围拒绝、SEARCH→ADJUST不截低高度、对框12cm跟踪、实际底高保护、扩大场地确认缺失拒绝、PASS 6cm可行侧偏允许、8cm硬上限、净空不足提前拒绝、符号方向和纵向进度、冻结上限不能变大等。不是SITL或实飞验收。

新模板config/session_window.json仍运动禁用、site_identity=null、alignment_area_confirmation.operator_confirmed_clear=false。原未完成精度/安装/仿真/FCU故障验收未改为true。待现场确认扩大后的活动区域、归位/朝向/出口空旷、释放急停后新绑定和只读预检。仅用户现场执行--execute，不自动重试，旧v24.2命令不使用。

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

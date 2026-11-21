# v22 固定高度/航向通行判定与直穿候选（2026-10-20）

## 用户要求与结果

用户明确请求简化流程：保持高度，用深度相机判定当前路线能否容纳整机，允许一定数量有效帧投票后直接向前飞，删除反复对框，并检查本次SEARCH卡点。已实现独立experiment=straight_pass，保留旧align_only用于回退。当前代码候选/影子验证完成，实际穿越尚未启用/执行。

## 当前问题证据

v21在1373.227进入SEARCH、1383.244超时交还。实际识别日志1374~1384多次stable_live_metric_geometry，连续视觉确认16~49，处理约242~283ms。旧任务链依次要求RGB-D几何接纳、自带旧RGB观察器target_fresh/confirmed、request_hold中0.04m/s速度检查，后续才ALIGN。识别成功不等于所有额外入口条件均通过。

旧终态未记录每层阻止原因，不能追溯证明唯一是旧RGB或速度。此次新增search_evidence诊断保留到终态。新straight_pass彻底不调用旧RGB target_fresh/confirmed/request_hold/request_start/request_pass，不订阅旧RGB observation，不使用其8帧和精密悬停准入。这解决了结构上的额外串联依赖，不能声称已直接复现并唯一证明本次旧流程触发点。

## 新流程

PRESTREAM -> MODE -> ARM -> TAKEOFF -> SEARCH（通行判定）-> PASS（冻结直线）-> HANDOVER_REQUEST -> PILOT_CONTROL。

起飞继续商家式20cm高度到达范围和解锁请求后3秒；保留原正常最低底高0.80m。判定使用当前实际高度/航向，不再对准窗中心，不横向/竖直修正。不能容纳当前直线则等待新结果或超时POSCTL交还。

最近1秒最多5个不同source_stamp结果，其中3个可行即可，不要求连续全部可行；缺目标/心跳空状态不清空所有近期好帧，但当前帧必须独立可行。重复/倒序时间戳不能计票，过期票移除，epoch变化清空，目标中心变化超过原预算重计。投票的对象是已有RGB-D管线输出；上游原有至少5帧几何确认仍保留，没有声称整条管线只处理3张原图。

## 几何判定

使用测得4个边角和窗平面；按当前冻结航向生成整机62x62x35cm、包络中心相对FCU下7cm的箱体。将其8个角沿前进方向投影到窗框厚度前/后两平面，检查投影相对真实凸四边形每条边的最小余量。不是中心点深度或外接矩形尺寸检查。

保留配置中的position_uncertainty+clearance_tracking_budget与clearance_margin，加入保留航向/倾角范围内的姿态包络扩张；预算仍只是既有配置值，未将其称为实测精度证明。名义平放箱体绕当前固定yaw放置，整体扩张覆盖允许yaw/tilt变化，避免以已倾斜箱体为中心却少算姿态变化。

路径从当前位置到窗后整机通过+停止余量的有限端点，始终同z、同yaw；完整箱体+预算两端位于凸场地围栏内，从而覆盖整条直线。保留起始距离、总路径长度/时间、窗后区域验收。未知深度不视为空旷，possible_opening不等于free_corridor_verified。不能宣称只要给坐标就必然穿过。

## 提交后的控制

确认一次后冻结start/endpoint/direction/height/yaw/geometry_stamp，不因新识别结果改目标；框离开视野不重对框。每周期按原traverse_speed向前推进，保持原max_setpoint_lead，反馈落后只保持不倒退。横向/垂直偏移不得超出已计入几何的clearance_tracking_budget；定位/模式/估计器/围栏等真实异常保留。窗后到位低速停稳后观察实际POSCTL才交还；不自动降落、不重新解锁。

## 权限与部署

旧起降/有限对框commissioning例外不能用于straight_pass，测试覆盖拒绝这种越权。straight_pass运行仍要求独立enable_motion/enable_traverse/pass_stage_approved及相应真实验收；旧有限对框scope没有被静默扩大。用户当前指令授权实现新模式，未用它伪造原安装/米制精度验收为true。

独立部署/home/abot/autonomous_window_test_20261020_v22_candidate；只读配置/home/abot/straight_pass_v22_shadow/config.motion_disabled.json，所有运动/穿越开关false、site_identity=None。当前预检ready=false符合预期；不是可飞试验。源文件最终哈希见_diagnostics/v22_final_nano_preflight_20261020.txt，包归档后有两份源文件更新，以最终文件为准。旧现场v21入口不变。

新增scripts/check_straight_pass.py只订阅pose/status，无发布者/服务客户端。Nano8秒观察40个不同结果，当前地面无完整窗框，所有判定rgbd_target_not_confirmed、未生成路线；没有解锁或运动。Nano当前落地未解锁、CH6=1950、status=8；保持急停即可继续只读检查。

## 测试与实测边界

230项离线测试通过，包括模拟完整直穿到POSCTL（旧RGB功能被Mock为调用即失败）、直线全程不进ALIGN、图像离开后不重定向、固定z/yaw、有缺帧3/5、当前坏帧不可被历史好票覆盖、重复/过期/epoch、完整箱体/净空/窗后围栏、实际偏航/过线/定位故障、不得借用align_only例外。不是SITL/实飞穿越验收。

历史v18.1几何做过非同步示意输入检查（用终态位置而非每帧同步位姿），只能用于数学负例，不能判断当前现场能否直穿或授权复用旧坐标。不能拿该回放否认当前新路线。

下一步若推进实飞：保持新直线模式的范围，核对现场实际高度/航向是否能容纳路线、窗后通道/停止空间与新会话身份，准备新独立配置；不能发旧对框命令冒充直穿，也不能把当前ready=false说成通过。无需重做已经取得的“能否采到窗框”独立飞行。

## 用户授权开始直穿测试；v22.1已准备（2026-10-20）

用户明确指令：识别后判定通过直接穿框，没通过交还。这是新的straight_pass试验授权，已单独记录，不沿用align_only禁止穿越的声明。新增straight_pass_policy.py、CLI --commissioning-straight-pass，只允许固定高度/航向有限直线；延期字段与原已说明的安装/精度/仿真等未完成项保持false，独立scope=supervised_fixed_height_straight_pass，不修改成已验收。实时RC/定位/估计器/姿态/围栏/独占参数等不在延期清单。要求现场确认窗后停止区域清空。

判定策略补齐：有效几何明确拒绝（擦框、路线越界、距离/时间预算不足、开口被前景阻挡）时，Mission立即进入HANDOVER_REQUEST，保持当前位置请求POSCTL，实际确认后PILOT_CONTROL；不是继续对框或等满搜索超时。仅数据未取得/不够帧数时最多等10秒，超时交还。当前无法通过的帧不能被历史票数覆盖。

直线模式不再要求窗平面法向与机头误差5度；这属于旧对框判断。现在用完整机体沿当前机头方向穿过窗厚的投影检查实际是否够宽，仍保留原min_ray_plane_cos用于避免近切向无效交点，实际身体偏航/倾角仍受运行约束。增加用例：斜窗无需转机头，物理投影有余量则直穿。

237项离线测试通过；独立部署/home/abot/autonomous_window_test_20261020_v22_1。新增独立scope相关6+1项测试包括无授权拒绝、未验收状态不被改写、不能跳过额外检查、拒绝即交还、实际通过后停稳交还、旧RGB无依赖及斜窗投影。

仅生成运动禁用模板 /home/abot/straight_pass_v22_1_prepare_mswzkoix/config.motion_disabled.json，scope授权已true，但enable_motion/enable_traverse/pass_stage_approved=false；site_identity=None、围栏清空、飞后场地核对字段false；exit_corridor_confirmation.operator_confirmed_clear=false等待本轮确认。没有可执行飞行命令，还未绑定/预检通过。

最新Nano遥测同boot39473f7f-c1cd-480d-b401-0033f963cdfa，POSCTL、armed=false、landed=1、status=8，CH6=1950、CH7=1050。因此当前只等待现场操作：把车放回测量起点/朝向、释放急停、A关闭，确认窗洞及窗后直行/停止区域空旷。无需用户重复授权试验scope。确认这些物理状态后才能新绑定并给用户现场execute命令，不能远程试飞或直接运行旧v21对框入口。

证据_diagnostics/v22_1_straight_trial_tests_20261020.txt、v22_1_straight_trial_preparation_20261020.json。现场deployment中straight_pass.py在打包后有“按实际斜窗投影而非先卡5度法向”的更新，以当前源文件为准；备份原版本均保留。

## 本轮现场归位确认后已绑定并通过只读预检（2026-10-20）
用户回复“完成”，确认已归位原朝向、释放急停、A关闭及窗洞/窗后直行停止区空旷。实时POSCTL、armed=false、landed=1、system_status=3，CH6/CH7均1050，无位置目标发布者。未远程解锁/起飞。
新独立试验：/home/abot/straight_pass_trial_frh4v4es/config.json；现场入口 run.bash --execute。旧模板/旧局部坐标未复用，重新静置采样绑定，预检ready=true/blockers=[]，地面z跨度0.00350m、90样本，COM_LOW_BAT_ACT=0、COM_OBL_RC_ACT=0。scope为straight_pass；原安装/精度/仿真延期字段仍false。地面/起飞柱/落地区域字段依据用户本次现场确认及实际静置绑定，不是新飞行验收。
判定通过冻结直线直接PASS；确认不可行立即请求POSCTL；缺少有效几何最多10秒交还。完成穿越停稳后也POSCTL交还人工降落/上锁。预检不代表空中路线已判定可穿，首次实际穿越仍未验证。不得远程运行--execute或自动重试。
完整证据：_diagnostics/v22_1_current_trial_binding_preflight_20261020.txt；配置快照_diagnostics/v22_1_current_bound_trial_20261020.json。

## 首次直穿判定实飞后复核（2026-10-20）
本轮起飞成功，识别后几何包络拒绝fixed_height_heading_box_would_clip_window，实际POSCTL交还已确认；未PASS。当前落地未解锁且用户急停CH6=1950/status8。发现现有全方向姿态/误差预算合计每边约20.8cm，标称居中余量横向约0.7cm/竖向约1.7cm，保守判定不等于已证明实物擦框。本轮缺拒绝帧角点/同步pose，无法确定哪条边。已补诊断geometry_diagnostics（角点、pose、每边原始空间与预算扣减），238项离线测试通过，并部署到v22.1；仅记录变更，未改判定条件，未重复飞行。详情STRAIGHT_PASS_FIRST_FLIGHT_REVIEW_20261020_CN.md。

# v19.1 有限对框试验范围（待用户明确接受）

当前不是可执行试验。配置 /home/abot/alignment_scope_draft_dqwlm8w9/config.pending_approval.json 中 enable_motion=false、operator_accepts_unverified_alignment_tests=false；严格预检ready=false。没有解锁、飞行或伪造验收字段。

## 已确认的事实

用户确认换电后回到原测量起点/朝向；相机光心相对飞控中心约前8cm、下6cm、镜头朝向与机头一致。此信息记为现场确认近似安装，不等于精密外参标定或测量精度验收。上次37条有效空中几何数据不重复专门采集，不复用旧窗坐标。

## 可审查的试验行为

自动从地面起飞到车底约0.90m；新空中识别窗框，窗前仅横向/竖直修正；最大离对框起点0.20m、累计路径0.30m、修正速度上限0.08m/s、距窗至少0.80m；对框完成后请求POSCTL，实际确认后停止发布，用户手动降落上锁。无识别/可信输入超时也交还。真实定位失效、接管等仍原故障逻辑，不保证任何故障都定点。

enable_traverse/pass_stage_approved必须false，Mission不调用穿越提交；限定控制器也拒绝request_pass。不得将该试验扩展到穿越。

姿态保护开启，起降的角度豁免不扩展。原12cm跟踪误差、新鲜度、三维围栏、最低高度、估计器真实有效位、RC、独占控制、参数清单均保留。本次没有将18cm起降测试阈值扩大至对框。

## 新范围需要单独接受的未完成验收

acceptance字段：body_origin_verified、fcu_failsafe_verified、simulation_verified、rgbd_geometry_verified。
control字段：camera_verified、pose_verified、direction_verified、hold_verified、failsafe_verified、latency_verified、corridor_verified、single_controller_verified。

含义：安装/米制精度、动态定位与停止预算、连续稳定对框、完整仿真/失效动作验收尚不完整；图源独占性、时效、姿态、围栏等实时检查仍必须通过。部分字段对应的行为已有历史/当前证据，但不得把缺少完整验收直接改成true。

新政策通过独立参数 --commissioning-alignment 和明确scope=supervised_alignment_only_no_traverse、operator_accepts_unverified_alignment_tests=true共同启用；目前该接受声明为false。原起降commissioning例外仍仅起降，不能借用到此入口。ground_height/takeoff_column/takeoff_landing_site不在延期列表，执行前必须按本次现场确认和重新绑定证据核对，不能用该政策跳过。

## 验证和限制

179项离线回归通过，包含有限范围限制、拒绝额外延期、没有用户接受时拒绝、安装声明不匹配拒绝、姿态检查必须保留、无穿越入口、真实状态故障保留、未验收字段不改写。独立部署 /home/abot/autonomous_window_test_20261019_v19_1_candidate。不是SITL/实飞通过。

视觉提前去重优化已有140项测试和Nano合成性能对比；空中实景连续性尚未实测证明，因此不得因优化结果直接认定过期问题完全解决。

后续用户若明确接受以上有限对框未完成验收范围，再生成独立批准配置、重新核对实时状态/绑定/地面证据并运行只读预检；仍由用户现场执行真实起飞命令。不能把对安装位置的“对”当成上述范围接受。

## 2026-10-19 用户明确批准有限对框范围

用户对上一条完整有限对框范围及未完成验收项说明回答“可以”。这是新增独立范围的明确接受，不是仅相机安装确认。

已生成 /home/abot/alignment_trial_skvpshd7/config.json、入口 /home/abot/alignment_trial_skvpshd7/run.bash。默认只读；带--execute才真实请求地面POSCTL准备、OFFBOARD和解锁，由用户现场执行。部署仍/home/abot/autonomous_window_test_20261019_v19_1_candidate。未远程执行飞行。

本次当前boot与原确认一致，新采样绑定当前节点身份；用户已确认换电后起点/朝向未变。ground_height沿用用户实测0.245m。takeoff_column_verified和takeoff_landing_site_verified按本次已确认的原测量起降位置、连续静态采样、旋转全包络及起点/目标/凸围栏内柱检查记录为地面场地核对通过；这两项不是飞行/米制精度验收。全部新增scope中延期的acceptance/control字段仍false，记录在ground_site_acceptance_evidence和alignment_commissioning_authorization。

只读预检ready=true、blockers=[]；MANUAL、armed=false、landed=1、status=3、CH6/7=1050，COM_OBL_RC_ACT=0。地面3秒z跨度约1.01cm。179项离线监督测试通过。见_diagnostics/v19_1_approved_alignment_preflight_20261019.txt。

本轮最大修正速度0.08m/s、离对框起点0.20m、累计路径0.30m、窗前距离0.8~2m、正常结束实际观察POSCTL后PILOT_CONTROL。窗口局部坐标空中重新获取，穿越禁止。姿态保护开启，跟踪阈值仍12cm（18cm原地起降阈值不扩展）。独立几何过期问题仅有耗时优化证据，未声称实景完全修复；超时/丢失按既有逻辑交还或故障。

成功证据需alignment_completed=true、handover_confirmed=true、phase=PILOT_CONTROL以及用户实际接管/落地确认。无目标/过期导致交还不是对框成功。当前等待用户现场执行后结果。

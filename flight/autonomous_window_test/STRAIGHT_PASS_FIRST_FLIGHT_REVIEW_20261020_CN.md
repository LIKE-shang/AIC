# 首次straight_pass拒绝复核（2026-10-20）

本轮入口/home/abot/straight_pass_trial_frh4v4es/run.bash。4169.999 TAKEOFF，4179.099 SEARCH，4179.569 HANDOVER_REQUEST（fixed_height_heading_box_would_clip_window），4180.006观察到POSCTL并进入PILOT_CONTROL。没有PASS，frozen_straight_route=null，straight_pass_completed=false，handover_confirmed=true。已采证，不自动重试。

用户日志表明两个早期几何帧被after_takeoff新鲜度规则跳过，重复帧未计票；第三个不同帧通过上游米制几何校验，进入真实整机投影判定并拒绝。不是起飞跟踪误差触发，也不是识别超时。PIPELINE在09:09:40有stable_live_metric_geometry；不能把地面后续无完整框作为空中识别失败依据。

只读实时复核：POSCTL、armed=false、landed=1、system_status=8、CH6=1950、CH7=1050；与用户落地后按急停习惯一致。

发现现有全方向姿态预算很保守。机体角点最大半径sqrt(.31²+.31²+.245²)约0.502m；yaw和tilt各5°，姿态扩张2*r*sin(5°)约.0875m；再加位置.05、跟踪.03、净空.04，正向每侧预算约.2075m。标称1.05x.80窗、.62x.35机体截面，居中名义余量横向约.0075m/竖向约.0175m。真实斜窗可能更少。属于保守上界判定，不能直接等同真实碰撞；不能据此静默清零预算。

本轮没有保存拒绝帧角点、对应pose、边余量。现有bag时间1667~1890，不覆盖本轮4179，stage记录也无对应帧。不能用旧bag重建本轮几何。终态位置是交还保持位置，不是几何判定同步pose。现有证据无法确定实际哪边、差多少。

已补充诊断而未更改判定或保护：RouteRejection携带拒绝帧geometry/pose时间、位置/四元数、窗角点/中心/基向量、每边原始空间/预算/扣减后余量、各预算分项。PassageDecision终态新增geometry_diagnostics。新增回归用例核对负边余量、分项一致、重复帧不替换证据；238项离线测试通过，非SITL/实飞验收。

已部署到原v22.1目录，原文件备份straight_pass.py.before_rejection_diagnostics_20261020。新文件SHA256 ac647945e0687374b2ec767b635012e8a0e287764e3058c3019202fcd538bd69，两端一致，Nano编译通过。没有远程启动飞行。本轮原始记录_diagnostics/v22_1_first_straight_flight_20261020.log。

后续应先核对当前路线的真实几何偏差及按方向计算姿态占用的合理性；本次未授权/未实施自动对框、调整起飞高度、放宽/删除净空预算或重复试飞。已知输入/安装精度未验收状态不变。

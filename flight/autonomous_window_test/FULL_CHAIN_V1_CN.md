# 穿框→悬停→单靶完整链路 v1（2026-10-24）

当前状态：本地实现与离线验证候选版。整体比赛测试仍未通过。Nano在本轮SSH检查时超时，尚未上传、板端编译、恢复节点、新会话绑定或完成实际地面预检；没有远程起飞或ON测试。当前本地477项飞行侧回归（含23项新增完整链路/绑定测试）、7项识别及18项像素反馈回归通过；20张保存JPEG重放18张有效。这些结果不等于实飞验收。记录与最终验证结果见 `_diagnostics/full_chain_20261024/VALIDATION.json`。

## 已实现

同一个 `Mission` / `Runtime`，唯一 `/mavros/setpoint_raw/local` 发布者：

`PRESTREAM → MODE → ARM → TAKEOFF → SEARCH → ADJUST → PASS → POSTPASS(HEIGHT_HOLD→STATION_TRANSFER) → HOVER(CENTER→ON_WAIT→EXPOSE→OFF_WAIT) → HANDOVER_REQUEST → PILOT_CONTROL`。

- 复用现有session-window v24穿框：本轮当前会话获取窗几何、有界对框、冻结约3米路线，仍需整机离框、路线到达和原停止稳定时间。PASS成功不立即交还；不启动另一条起飞命令，不重新解锁，不重置ground_z、起飞原点或定位epoch。
- 完整链路保留12厘米起飞水平中止，包含原simplified分支；没有宣称起飞侧移已解决。新取景/航向规则仍需现场验证，不能因此跳过当前失败。
- POSTPASS使用冻结窗平面、连续姿态整机支撑包络、实测围栏检查实际位置与每个新目标，防止靠回窗内。先在离框柱内到约0.9米机底，再向明确的单靶站点限速移动并停稳1秒。
- 站点为“冻结穿框路线末端 + 前/左米制偏移”，按该路线yaw换算至local。独立`aim_anchor`，15厘米视觉修正围绕靶站；地面起飞点仍保留。
- v1站点转移不超过1米，速度≤0.08m/s；离框调高≤0.10m/s；POSTPASS总超时25秒。超过范围需依据已确认几何扩展profile，不能把导航藏入15厘米微调。通用跟踪12厘米、普通前导4厘米不放宽。
- USB沿用640×360/20FPS、参考(+53.25,+15.484375)像素、最大修正0.15米/0.04m/s与v7恶化暂停。目标高度0.9米；进入打靶要求实际机底≥0.8米、到高误差≤0.04米、站点位置误差≤0.04米且低速持续1秒。
- 定时开灯：CENTER等待2秒后请求最大AUX5=100，AUX6=0；ON ACK后曝光3秒，独立最长5秒，结束AUX5/AUX6均0；沿用商家库哈希、ABI、worker截止、连接/模式/图像新鲜度和调用者检查。
- 新scope `window_pass_single_target`，激光只在顶层HOVER且单靶子阶段、穿框和停稳完成时允许ON；每进程仍仅一次ON。新trial要求未用过的episode和初始OFF ACK，不通过飞行中重启执行器来重置。
- 正常结束与导航健康的相机/目标任务中止都先等OFF ACK，再请求POSCTL并确认真实模式。OFF超时会明确报告硬件未知、流程未完成并请求交还，避免无限占用OFFBOARD；失定位/急停/外部模式接管走原停止/独立租约关灯路径，不假装拿到OFF确认。
- 新路线定位守卫以首次可信原始LIO为锚，范围按3米路线及单靶转移余量派生；保留原始序号/时间/步长/速度/来源锁存，首次巨大原点在转发前阻止。不是把旧单靶固定5米绝对范围原封不动用于整条路线。
- 完成字段拆开：`pass_complete/postpass_hold_complete/target_acquired/aim_converged/laser_exposure_complete/laser_off_confirmed/handover_confirmed/mission_complete`。`mission_complete`只表示穿框到曝光、关灯、交还流程，`physical_hit_verified=false`、`precision_acceptance=not_verified`始终明确。无靶或未居中不取消正常定时曝光；因此流程成功可同时精度未通过。
- 原图记录覆盖TAKEOFF、SEARCH、ADJUST、PASS、POSTPASS、HOVER、HANDOVER_REQUEST，每0.25秒最多360帧。遥测不直接订阅控制输入，继续使用target_local和监督器状态。

## 独立包

本地候选目录 `_deploy/full_chain_20261024_v1`，归档同名`.tar.gz`。安装目标 `/home/abot/full_chain_20261024_v1`；保留历史穿框、v6视觉、v7和旧trial。

USB包名 `full_chain_vision_20261024_v1`，二进制 `full_chain_target_laser_v1`，视觉入口 `circle_full_chain_servo_v1.py`，原图入口 `record_full_chain_v1.py`；RGB-D包 `full_chain_rgbd_20261024_v1`、入口`live_full_chain_v1.py`。Python路径显式前置包内USB/RGB-D源码与固定window_pass纯控制源码，包manifest记录SHA256。不能同时开旧支持组占用USB/激光/同名节点。`ground.bash`输出节点真实进程路径，安装检查`rospack find`。

板端依赖仍来自已有ROS/相机/MAVROS/LIO环境和已固定哈希的原定位转换；不是重新部署飞控固件。原始10Hz基础启动入口仍使用已知历史路径，不自动改用可能50Hz的旧start_base。新支持组初始化会真实发OFF；地面恢复可能在着地未解锁时请求MANUAL以满足首次定位转发条件，但不会ARM、OFFBOARD或发布飞行目标。

## 地面恢复、重新绑定与命令

以下是**待Nano再次上电、实际上传并编译后**使用的具体入口；当前不能称这些步骤已完成。

1. 独立上传解包后在Nano编译（不启动飞行/ON）：

```bash
bash /home/abot/full_chain_20261024_v1/install.bash
bash /home/abot/full_chain_20261024_v1/ground.bash
bash /home/abot/full_chain_20261024_v1/ground.bash --restore
```

先status核对新boot/run，按缺失情况restore。脚本只在无控制发布者、着地未解锁、CH6急停释放及CH7 A关闭时恢复；无ROS master时采用新的随机run_id启动已知10Hz基础；已有master不重启LIO。已有定位/支持组来源错误或只启动一半时会明确停止，不能重叠启动。恢复后再次status，检查LIO约10Hz、guard来源和USB/RGB-D真实路径。已启动进程留PID/boot标记，防止短时未出话题时重复启动。

2. 本轮确实缺少已验证的穿框末端到靶站米制关系。查到的图纸：`_diagnostics/map_alignment_20261018/nominal_field_draft/field_geometry.json`，为DRAFT_NOT_APPROVED_FOR_MOTION，三个右侧靶草图中心(4,3.8)/(4,2.5)/(4,1.2)米，无批准的map/field变换；不能直接拿来导航。原3米路线还从实时对框后的提交点计，不能用起飞点平移3米等同靶心。

获得当前实际偏移后（前为正，左为正，右为负；若确实在末端正下方才填0/0），在Nano执行：

```bash
bash /home/abot/full_chain_20261024_v1/bind.bash \
  --station-forward-m <实测前后米数> \
  --station-left-m <实测左右米数> \
  --station-evidence '<现场测量或已核实图纸依据>' \
  --current-flight-area-confirmed
```

这里的尖括号是必须补全的输入，不是可以原样执行的飞行命令。当前area确认覆盖已测起点/朝向、20厘米对框空间、完整3米路线及停止区、到单靶的移动与15厘米微调柱、交还降落区。无需重复提供相机/PWM/安装偏移或因地面看不全靶重新标定。

绑定采集8秒新鲜FCU和原始LIO，核对新boot/run及RGB-D来源，使用原实测边界生成新围栏，保留历史未验收标志为false，创建全新`/home/abot/full_chain_trial_随机串/`并自动执行只读完整链路预检。绑定阶段不ARM、不发飞行setpoint、不请求ON。

3. 只有真实预检通过时，输出该次**完整**用户飞行命令：

```text
USER_FLIGHT_COMMAND=bash /home/abot/full_chain_trial_本次新随机串/run_recorded.bash --execute
```

`run_recorded.bash`无`--execute`仍仅预检；带`--execute`才由用户现场执行，整个链路只有一个Mission。不能换成旧hover_target1_trial，也不能穿框命令之后接单靶起飞命令。更换boot/run、定位重启、移动起点/朝向或节点来源变化后重新绑定。执行器曝光用过后，只能着地未解锁时检查并恢复支持组，再绑定新trial。

## 验证边界与下一步

离线回归验证逻辑连接和失败处理，不验证真实飞行、安装轴响应或命中。本轮Nano仍不可达，板端编译与实际预检结果必须留空，不能填ready=true。起飞水平偏移尚未物理归因，也未证明v7空中收敛。

现场按新bag/原图逐项判定：稳定起飞→窗几何与穿框→整机离框停稳→0.9米单靶站点→真实视觉修正及误差曲线→ON实际光点→OFF ACK→POSCTL实模式→用户遥控降落。中心命中需原图/现场结果单独验收，定时曝光完成不等于命中。靶间移动与随机抽两个本版不实现。

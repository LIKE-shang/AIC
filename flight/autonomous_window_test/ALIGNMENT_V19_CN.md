# v19 窗前对框候选与当前阻止项（2026-10-19）

用户已确认本次换电后回到测量起点且朝向一致。本轮boot=73705d1a-c4c7-4eda-a488-b24497004d1c、run=ae0b86adc6824e2ca7ba5c966f887306。最新候选配置 /home/abot/align_only_prepare_djabfm_d/config.json，绑定包含重启后识别节点新身份；此前71eynobi配置因识别节点重启不再用。

部署 /home/abot/autonomous_window_test_20261019_v19_candidate。
experiment=align_only；支持从地面起飞、空中实时采集窗几何、保持窗前法向距离做横向/竖直对框；到核心PREPASS后直接POSCTL交还。禁止enable_traverse/pass_stage_approved，绝不调用request_pass；若进入PASS立即故障。必须启用pilot_landing，正常交还须实际观察POSCTL。真实输入故障仍原处理。姿态检查开启。当前候选运动开关保持false，未扩展起降commissioning例外，不能执行真实对框。

170项飞行监督回归通过，包含不提交穿越、对框后交还、未识别超时、不接受穿越权限、安装/运动门槛保留、真实估计器故障、不扩大起降豁免。不是SITL或实飞。

只读预检：地面、传感器、参数、图、时效、会话与围栏配置均通过；ready=false，因为enable_motion=false及安装/校准/动态预算/失效验收字段仍false。对框入口不再要求自动降落验收和窗后落地区域（该模式不使用），其余字段保持真实状态。绑定没有改变验收字段。

实时性能优化：window_live_vision detector提前跳过最终NMS一定会丢弃的重叠候选，选中候选仍走完整四边证据检查。按输出四边形面积排序后执行，与原先完整支持检查后NMS逐项对比相同；140项视觉回归通过（包括真实历史图片、合成透视、多目标、缺边、原版本逐项比对）。Nano合成480宽场景检测中位137.7ms->90.9ms，最终角点/得分一致。没有降低分辨率或增加新鲜度阈值。不能据此宣称空中实景过期已解决。

在落地未解锁、status=3、CH6/7=1050、无控制发布者时仅更新detector.py并重启/window_live_vision。原文件备份 /home/abot/window_live_rgbd_20261014_v5/src/window_live_vision/detector.before_v19_early_suppression.py。新节点PID6509，日志 /home/abot/v19_detector_optimized.log。定位桥、MAVROS、相机未重启。后续重启感知入口仍载入优化文件。

上次37个有效空中几何样本足以避免重复专门采集飞行，但metric_accuracy_verified、free_corridor_verified仍false；尚需安装外参/定位原点、动态误差与实时几何连续性等核验，不可批量改true或把已有起降例外扩展到对框。当前没有向用户发可执行对框/穿越命令。

资料：_diagnostics/v19_final_alignment_preflight_20261019.txt、v19_detector_comparison_20261019.json、v19_test_output_20261019.txt、v19_vision_tests_20261019.txt。

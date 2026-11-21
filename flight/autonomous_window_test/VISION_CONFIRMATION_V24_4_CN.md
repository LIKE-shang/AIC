# v24.4视觉确认链审计与简化（2026-10-20）

用户要求检查视觉代码，适当减少帧数/放宽阈值。已逐层审计RGB Tracker、MetricTracker、LiveNode、任务validate_metric与PassageDecision。现有project_rgb_pixels及缓存射线优化与Nano代码一致，不重复实现或宣称本轮新增配准优化。

旧确认：RGB3帧/.18秒；米制至少5帧/.7秒（与RGB同时累计，不是简单相加），视觉或米制源间隔>.4秒重置；任务再取3/5个独立可行结果。上次日志实际310~465ms处理和>.4秒间隔导致计数重置，最后只累积2票，10秒到期。上一轮漏线程环境已修复，但未有同场景完整RGB-D A/B实测，不能归为唯一原因。

新增显式fast_confirmation配置，旧视觉默认仍false，可回退。新任务模板true：
- RGB仍3帧/.18秒；RGB与米制允许相邻结果间隔≤.60秒。
- 米制3个不同源帧、跨度≥.35秒；最多保留5帧且源时间窗口≤1.2秒；角点散布2.5cm、中心散布2cm、中心跳变6cm、角点跳变8cm、法向跳变4°及epoch连续性原样保留。
- 任务接纳米制≥3帧且校验span .35~1.2秒，随后最近1.2秒5个结果中2个可行且当前可行才固化。两份几何平均历史可重叠，正常最少约4张底层不同原图即可产生两次可信结果，不能宣传为独立5帧或单帧高精度。
- 当前图像/米制结果新鲜度仍≤.5秒，receipt新鲜度、35ms RGB-depth对齐、同步位姿、唯一完整框、平面/角点有效、开口前景、会话失效等均不放宽。允许.6秒处理间隔不等于允许使用.6秒旧图像。
- 最近观测到的前景证据不做历史平均掩盖。确认样本减少会减少平均样本量，不代表精度已验收。

任务只读预检新增实际视觉配置校验；必须收到fast_confirmation=true、metric_required_frames=3、span=.35、max_track_gap=.6、max_image_age=.5，否则拒绝以旧视觉运行新任务。任务与感知版本分别独立目录，旧启动指针不自动改成新入口。所有运动开关默认false、site_identity=null。

20cm一次修正/20cm活动柱、8cm PASS最高偏差逐边4cm净空、80cm最低底高、高度指令连续衔接和POSCTL实际确认均保留。没有增加起飞精密稳定条件，没有自动起飞/重试。

验证：视觉144项、任务342项离线通过。新增3帧/.35秒确认、.43秒间隔不重置、过期/epoch/跳变/重复仍拒绝、前景不平均、任务两票与三帧米制契约、完整合成获取→一次对框→冻结PASS→POSCTL以及视觉丢失不重采样。不是SITL或实飞验收。源代码对比证据_diagnostics/diff_vision_v24_4.py表明本地相对现行Nano的rgbd.py/live_node.py仅本轮确认策略/诊断变更。

本轮只完成代码审计、独立候选及离线验证；是否已经实际运行新视觉参数必须以现场后续status读回为准，不能把本说明写成当前预检通过。新版本启用后旧节点URI绑定失效，需重新绑定；不提供旧试验入口作为新版本命令。

已保存Nano独立候选目录/home/abot/autonomous_window_test_20261020_v24_4和/home/abot/window_live_rgbd_20261020_v24_4；后者复制当前视觉基线，仅更新已逐行核对的rgbd.py/live_node.py/rgbd.launch及测试。全量任务manifest与视觉patch manifest哈希/编译通过。本轮没有切换正在运行的节点、没有给新飞行命令、没有远程试飞。当前运行视觉仍是上一轮已修复线程环境的旧确认策略；新策略未现场启用。候选部署证据_diagnostics/v24_4_candidate_deployment.txt。任务包SHA256 2e9b3ae859790aaa51a2da2c626d3780f378583fdbf95ed752423e6cf688c39d，视觉补丁632fd351652affa3548ca95fbd151a77efbaa7d793489a51586574e7226e6190。

后续地面切换视觉必须确认落地未解锁/无控制发布者，只替换视觉节点，保留相机和定位；指定新src PYTHONPATH、三项线程=1及_fast_confirmation:=true。验证status.configuration实际匹配，再按新任务配置新绑定和预检。不可运行旧v24.3.1试验命令冒充v24.4。

## v24.4地面切换完成（2026-10-20）

用户“可行，开始”授权推进。初次读取POSCTL/armed=false/landed1/status8/CH6=1950，符合地面急停状态；安全核对后仅停止旧/window_live_vision进程，启动独立/home/abot/window_live_rgbd_20261020_v24_4/scripts/live_node.py，_fast_confirmation:=true且三个计算线程变量均1。PID22638，日志/home/abot/v24_3_ground_reboot_8iCaLl/vision_v24_4.log。相机/MAVROS/LIO/pose bridge保持原节点URI，未发飞行模式或解锁/起飞命令。

随后12秒只读采样已读到POSCTL/armed=false/landed1/status3、CH6/CH7=1050（用户在现场释放）；视觉实际configuration fast_confirmation=true,metric_required_frames=3,metric_minimum_span_s=.35,max_track_gap_s=.6,max_image_age_s=.5，calibration_valid/result_fresh=true。桥FORWARDING_CONTINUOUS_POSE无fault，FCU odom30Hz、XYZ跨度.02420/.00902/.01902m。21项飞控参数读回匹配。任务/视觉候选manifest在切换前全量匹配。

本次尚未建立新运动配置、未完整飞行预检通过；需要确认上一轮飞行落地后已回到原测量起点和朝向、原20cm活动范围及出口通道空旷，再新绑定。不能把“开始”当作实测归位证明，不能复用v24.3.1旧试验命令。证据_diagnostics/v24_4_vision_switch.txt、v24_4_ground_audit.json。

## v24.4本次现场确认、新绑定及预检通过（2026-10-20）

用户“已完成”承接本次归位/原测量朝向、20cm调整区域及窗后通道空旷确认。最新boot13ce4365-b6f7-4456-b9ae-952bc81d5fc2/run606d03a6576244ff82445f6a4a11d21f，POSCTL/armed=false/status3/landed1，CH6/7=1050。按新视觉节点身份及新采样起点创建/home/abot/v24_4_session_pass_trial_s26f7onm/config.json，不复用旧窗/场地绑定。

只读预检ready=true、blockers=[]、exit0，runtime_build=20261020_session_window_v24_4_fast_confirmation；实际视觉快确认配置契约检查通过，session_window fast=true/修正.20/活动柱.20/PASS上限.08；定位连续性ok/epoch0，地面z跨度.011716m，21项参数匹配。原安装/精度/仿真/FCU失效验收仍false；不是实飞穿越验收。

用户现场新命令：bash /home/abot/v24_4_session_pass_trial_s26f7onm/run.bash --execute。默认无--execute仅只读；远程本轮未起飞或解锁、未自动重试。A保持关闭，不提前手动解锁，实际观察POSCTL后用户降落/上锁。旧v24.3.1命令不使用。等待v24.4首次实际飞行结果。

证据_diagnostics/v24_4_bound_preflight_20261020.txt、v24_4_bound_config_20261020.json。

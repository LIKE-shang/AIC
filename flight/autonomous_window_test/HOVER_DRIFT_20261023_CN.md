# 2026-10-23 单靶起飞漂移调查

实际飞行日志进入OFFBOARD并确认解锁，TAKEOFF约2.42秒后水平误差0.12139米触发FAULT。
未进入HOVER，摄像头未修改水平目标，也未请求激光ON。
目标FCU本地坐标约[1306.937,-1083.729,-7660.333]，该新启动本地定位试验中的千米级坐标本应阻止起飞。
之前binder只取4秒静置跨度；原始LIO连续性检查仅在session_window启用，单靶模式遗漏。

后续地面未解锁只读4秒：原始LIO位置从[10674.543,-15497.566,-97975.910]变为[11397.591,-16247.919,-102735.770]，XYZ跨度[723.047,750.353,4759.860]米。
飞控本地位置也异常，定位桥仍声称FORWARDING_CONTINUOUS_POSE/horizontal_flags_valid=true。
这些标志和新鲜时间戳不能替代定位物理一致性。原日志持续No Effective Points，具体传感器/匹配根因尚未确证。

已确认着地未解锁后停止/window_pass_competition_pose_session_20261013，禁用/home/abot/hover_target1_trial_k0aczn5e/run.bash。
不放宽0.12米跟踪阈值，不通过POSCTL强行继续，不修改飞控参数。

v4：单靶接入session_localization.Continuity原始LIO来源/时效/位姿跳变/速度/桥状态检查。
本模式明确是新初始化局部原点的单靶测试，不支持任意偏移地图坐标：原始LIO与FCU位置必须在局部原点5米内，异常锁存。
地面绑定同时要求原始LIO至少50样本、XYZ跨度不超过原4厘米阈值；原FCU静置和当前来源身份检查保留。
新增8个故障回归检查，全套435项离线测试通过；不代表实飞定位验收。
已安装/home/abot/hover_target_single_20261023_v4_localization并编译成功。没有创建新飞行试验。
仅重启FAST-LIO计算，定位转发保持关闭，等待只读采样结果。

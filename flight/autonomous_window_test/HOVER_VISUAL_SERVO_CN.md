# 2026-10-23 v6 悬停视觉微调

v5实飞日志：target_detected=false、error_px=null、observed_hit_frames=0，定时ON/OFF完成。
没有靶心识别就没有发生视觉修正，completed不代表命中中心。

v6复用0.9米起飞、定位守卫、PWM开关和3秒定时亮灯；明确visual_servo_enabled=true。
开灯前等待2秒，按已测偏移预测落点微调；ON确认后的EXPOSE阶段，优先按实际光点减靶心误差持续微调。
光点不可见但靶心有效时，仍使用已测预测点；靶心无效时保持原位置，不猜测修正方向，也不因此取消定时开灯。
微调使用0.6比例增益、2像素死区，按新帧间隔限速4cm/s，位置目标不超过原15cm范围和4cm前导。
目标从当前测得位置计算后限速，避免把滞后的图像误差持续积分到移动目标。

视觉节点单独采用比赛识别：原三环检测优先；较宽松的唯一三环组；再尝试几何比例匹配的中环和内环。
双环模式使用实际测得内环椭圆；不把单个圆环或多个完整靶随意认作靶心。
实时状态报告detector_reason、detector_mode、detector_ring_count、detector_target_count。
悬停日志增加correction_updates、aim_source、aim_error_px、correction_reason。
新增只读画面记录器，仅HOVER期间每0.25秒保存一张原图和当时状态，总量上限120张。
原图目录/home/abot/hover_vision_recording_*，日志会打印实际目录。

离线验证：451项飞行逻辑、7项比赛识别（含旧8帧相机图）、18项预览节点测试通过。
模拟有偏移的实际光点反馈能在亮灯期间收敛；不是硬件中心命中保证。
实际光照、运动模糊和本轮现场未识别原因仍需以新原图验证。

本次Nano新boot=523c469a-4806-441e-af45-fd0d53fb95aa。
基础服务复用原base_10hz_reboot.launch，原始LIO8秒82帧，各轴跨度1.07/0.87/1.13厘米。
当前着地未解锁、ALTCTL、status3、CH6=1950、CH7=1050；未请求起飞或ON。
地面绑定采样调整为8秒，以便10Hz输入有至少50个原始LIO样本。

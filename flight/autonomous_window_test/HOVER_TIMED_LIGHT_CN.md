# 2026-10-23 v5 单靶定时开灯

v4本次实飞已成功悬停，但全程camera_identity_or_target_invalid，没有发ON请求。
原原因把目标识别失败与相机身份错误合并；当前地面读回确认相机身份尺寸正确，图像新鲜，但await_unique_three_ring_target。

新试验配置明确trigger_policy=hover_timed：进入HOVER后等待1秒，收到新鲜且身份/尺寸/偏移一致的相机消息后发出ON请求。
无需完整识别三环、无需落入中心45%区域、无需满足4cm/s与2.5cm的额外开灯门槛，也不以光点识别判定能否保持亮灯。
若开灯前识别到完整靶，仍按现有偏移做有限微调；亮灯期间保持原位置目标。
收到ON确认后定时3秒，OFF确认后交回POSCTL，飞手降落。
无靶/未命中记为得分未知，不得伪称命中成功。completed只说明定时亮灯流程完成；physical_hit_verified保持false。
相机身份/流时效、真实OFFBOARD/armed、定位连续性、遥控接管、5秒独立最长ON、ON/OFF确认仍保留。
原vision_confirmed策略保留为可选兼容模式；新模板显式选择hover_timed。

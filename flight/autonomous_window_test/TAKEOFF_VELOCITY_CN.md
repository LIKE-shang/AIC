2026-10-19 v13：受监督原地起降的垂直速度控制

前提：用户确认相同载荷和桨叶配置此前能够遥控起飞悬停。本次修改不改飞控悬停推力、电机输出、保护参数或解锁策略，不代表动力与控制已验收。

原程序起飞只发布三维位置目标，高度参考最多领先反馈4cm。读取MPC_Z_P=1时，该参考通常只请求约4cm/s爬升；日志显示电机已进入推力爬升，但没有持续离地。这个限制影响自动起飞响应，不能认定为唯一原因。

v13配置mission.takeoff_vertical_velocity_enabled=true，仅允许experiment=takeoff_land，爬升速度设置不得高于0.10m/s。PRESTREAM/MODE/ARM保持位置控制；仅在观察到已解锁并进入TAKEOFF后，XY继续位置控制，垂直发送ENU正向上升速度，并使用IGNORE_PZ而解除IGNORE_VZ。上升速度min(配置爬升速度,剩余高度/1秒)，接近目标时减速，目标以上不继续请求上升。稳定达到车底目标0.90m后，HOVER恢复XYZ位置保持，再按原流程自动降落。垂直速度命令不等于保证实际速度；由飞控闭环响应。

位置参考仍保留用于误差、围栏和日志检查；12cm跟踪阈值、水平20cm起飞柱、实际旋转包络围栏、80cm空中最低点、起飞/悬停超时、定位数据有效性和遥控接管保持。不修改跟踪误差阈值，起飞没有爬升或airborne标志误报不能通过悬停验收。

manual准备入口适配当前10Hz定位：最低读回输出频率由20Hz改为8Hz，仍须新鲜位姿、飞控落地未解锁、status=3、A关闭。当前10Hz是诊断启动参数，重启旧base会恢复50Hz，不应直接复用旧坐标或原地起飞。

离线测试覆盖带1秒准备延迟及0.4秒速度惯性的合成模型、目标高度减速与悬停降落、没有爬升但airborne误报时超时降落、横向漂移中止、ENU速度符号/掩码以及非起飞阶段禁止速度输出。不是SITL/实飞验收。

现场准备模板需要显式启用新策略，保持其他授权与未验证项。使用v13重新绑定/预检，不能仅把旧v12起飞命令重跑。急停解除后：

bash /home/abot/autonomous_window_test_20261019_v13/reboot_setup.bash manual
bash /home/abot/autonomous_window_test_20261019_v13/reboot_setup.bash trial --same-start-confirmed

只有地面准备通过、现场飞手就绪后，才由操作者单独执行current_takeoff.bash --execute。本修改不授权穿框。

# 本次低速原地起降试验（2026-10-19，v6）

用户明确要求精简验收并推进测试。本轮采用单独的受监督起降试验授权，
不把未完成项目写成 verified=true，也不开放穿框。

## 本轮范围

车底目标不超过 0.90 m，上升目标推进速度不超过 0.10 m/s，稳定悬停不超过 5 秒，
然后请求 AUTO.LAND。水平目标固定在起点；允许的反馈偏移半径不超过 0.20 m。
场地边界、安装高度、已批准的失联/低电量降落配置沿用当前会话记录。

以下三项保持“未验证”，仅本次受监督起降试验接受：
- 完整机体定位原点标定（body_origin_verified）；
- 实际飞行中的失效动作试验（fcu_failsafe_verified）；
- PX4 SITL（simulation_verified）。

参数值比对、定位新鲜度、估计器、遥控在线与接管、落地未解锁、围栏、
单一控制源、电池门槛及超时保护仍是硬检查；不能用本授权跳过。

## 使用

v6 已增加 --commissioning-takeoff。它必须同时匹配配置里的明确试验授权，
并且任务类型必须为 takeoff_land；传入完整穿框配置会拒绝。
只带 --commissioning-takeoff 仍然只预检；再加 --execute 才会请求真实模式和解锁。

本轮配置保留在 /home/abot/takeoff_test_aOaGrX/config.commissioning_20261019.json，
原始 config.json 保持不变。不得将这些未验收状态当作后续穿框已通过。

预检：

~~~bash
bash /home/abot/autonomous_window_test_20261019_v6/run_takeoff.bash --config /home/abot/takeoff_test_aOaGrX/config.commissioning_20261019.json --commissioning-takeoff
~~~

飞手就位、场地保持清空后，由现场操作者执行实际试验：

~~~bash
set -o pipefail
bash /home/abot/autonomous_window_test_20261019_v6/run_takeoff.bash --config /home/abot/takeoff_test_aOaGrX/config.commissioning_20261019.json --commissioning-takeoff --execute 2>&1 | tee /home/abot/takeoff_test_aOaGrX/commissioning_flight_20261019.log
~~~

正常阶段 PRESTREAM → MODE → ARM → TAKEOFF → HOVER → LAND_REQUEST → LANDING → DONE。
只有 DONE 且 takeoff_hover_completed=true 表示程序观察到完整起降；
异常降落后 DONE 且该标志为 false 不算起降测试通过。
异常按已验证的遥控接管流程处理；Ctrl+C/关闭 SSH 不是降落命令。

## 验证边界

76 项离线测试通过，包含试验授权不可用于穿框、不可跳过实时检查、不可扩大试验限值、
保留未验收标志以及现有起降故障测试。没有声称完成 PX4 SITL 或本次真实起飞。

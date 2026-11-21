# 独立自动起降测试（v5，2026-10-19）

## 这次入口做什么

固定起点 XY 和航向 → 自动请求 OFFBOARD/解锁并读回 → 车底目标 0.90 m →
到位稳定 2 秒 → 连续稳定悬停 5 秒 → 请求 AUTO.LAND → 确认落地且未解锁。

起降入口不依赖相机、不需要看到窗框、不调用 geometry，不允许对齐或穿框。
即使误传穿框配置，也会因入口/任务类型不匹配而拒绝。
若起飞或悬停超时，会在有效定位和已确认起降柱内请求降落；此时不算测试成功。
定位/遥控/控制源失效、人工接管等故障仍交由经验证的 FCU 失效策略与飞手处理。

默认命令只做地面预检。交付配置的 enable_motion=false，不会自动飞。
此前缺少的独立起降代码现已补齐，71 项离线测试通过，Nano 默认只读预检已运行；
尚未做 PX4 SITL 或实际自动起降验收。v5 本轮已部署，无需再上传。

## 1. 保留基础定位

已有 MAVROS、雷达、定位桥时保留原终端。车放在之前测量边界的同一个起飞位置，
车头朝向不变，保持未解锁、A 关闭。已有视觉可以继续运行，也可不启动；起降测试不依赖视觉。
重新上电的基础启动方式见 TEST_STEPS_CN.md，不要在已有系统上重复启动。

## 2. 绑定本轮起降坐标（不解锁、不切模式）

不用重新量距离或抬车。确认车仍在测量时的起飞点，然后在 Nano SSH 终端执行：

~~~bash
TAKEOFF_DIR="$(mktemp -d /home/abot/takeoff_test_XXXXXX)"
export TAKEOFF_DIR
bash /home/abot/autonomous_window_test_20261019_v5/prepare_takeoff.bash \
  --same-start-confirmed --output "$TAKEOFF_DIR/config.json"
~~~

看到 SITE_PROFILE_WRITTEN_MOTION_DISABLED 才继续。
它采集 4 秒地面位置，把已提供的前 4.5 m、后参考点 0.81 m（后缘净空 0.50 m）、
左 1 m、右 3.3 m 边界转换到本轮 local 系。顶部实际 4 m，软件包络上界最多取 2 m，
起飞目标仍为车底 0.90 m。旋转边界取保守内接轴对齐框，不使用会扩张区域的外接框。
地面数值下界含接地测量容差；程序不会指令起飞点以下的位置，下降由 AUTO.LAND 处理。

新 profile 绑定 boot_id、run_id 和实际定位进程地址；重启定位后须重新绑定。
--same-start-confirmed 表示你确认没有改变相对于场地边界的起飞位置，不是飞行批准。
此步骤保留所有验收标志，不把采集成功当作安全验收。

## 3. 只读预检

~~~bash
set -o pipefail
bash /home/abot/autonomous_window_test_20261019_v5/run_takeoff.bash \
  --config "$TAKEOFF_DIR/config.json" 2>&1 | tee "$TAKEOFF_DIR/preflight.txt"
~~~

正确的任务标识：experiment=takeoff_land、acquisition=none、window_traversal_permitted=false。
这里不再检查“自动起降已成功”，也不需要 RGB-D/窗后落点/窗框验收，因此没有循环前提。
仍检查机体参考点、起飞落地柱、飞控失效保护、仿真等基础证据。

当前交付标志仍有未完成项，预期 ready=false；不可批量改 true。
你已经批准的四项参数及先前核对的基线写入 expected_fcu_parameters，运行时逐项比对。
参数读取独立于控制源检查线程，慢服务不会堵住控制循环；缓存超过 5 秒会拒绝继续。

position_uncertainty_m=0.05、stopping_margin_m=0.10 是明确标记的保守试验预算，
不是已测出的动态误差上界或刹停距离。相关验收没有因此自动通过。

## 4. 真实测试入口（仅基础验收完成、预检通过后）

只有本轮 ready=true、blockers=[]，且现场基础验收已经完成，才允许执行：

~~~bash
bash /home/abot/autonomous_window_test_20261019_v5/run_takeoff.bash \
  --config "$TAKEOFF_DIR/config.json" --execute 2>&1 | tee "$TAKEOFF_DIR/mission.txt"
~~~

这条命令通过预检后会直接请求解锁/起飞；没有第二次询问。飞手应专注现场观察与接管。
不要把 Ctrl+C、关闭 SSH、停止定位当作降落命令。

正常阶段：PRESTREAM → MODE → ARM → TAKEOFF → HOVER → LAND_REQUEST → LANDING → DONE。
只有 DONE 且 takeoff_hover_completed=true 才表示程序观察到完成了完整起飞/悬停/落地过程。
如果未升到目标或悬停不稳定，可能降落后也是 DONE，但 takeoff_hover_completed=false，不能算通过。
这仍是机载估计器反馈，不是外部真值精度或所有故障保护的证明。

## 5. 结束

确认机体实际落地且 armed=false 后再靠近。保留 config.json、preflight.txt、mission.txt。
完成一次起降不会自动解锁穿框，也不会自动改变 auto_takeoff_land_verified；由本轮证据审核后记录。

本次代码与离线测试完成，不表示本次已执行真实飞行。

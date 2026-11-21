# 完整单靶链路 v1.1：图纸估算靶站（2026-10-24）

用户已确认Nano上电、场地严格按图纸摆放、靶1接近计划3米穿框停点，授权按现有规格估算。v1.1延续v1单一监督器与所有关灯/交还/定位保护，没有远程起飞。

图纸数据：起点(0.85,3.85)，右上靶1(4.0,3.8)米，图纸右边为机头前方。因此从起飞点估算靶站为前3.15米、右0.05米；未对框且航向不变时，距3米停点约前0.15米、右0.05米。此为用户批准的估算，不将其写为实测精度或旧地图重定位成功。

为避免对框横移后给整个靶站追加同样偏移，新增`station_reference=takeoff_body`，靶站以本次起飞origin和初始yaw固定。穿框后的heading可以更新，靶站仍固定在本次初始坐标；15厘米微调围绕该站点。实际冻结路线停点到站点若超过1米，或任何整机包络/窗平面/围栏检查不满足，则不进入打靶。旧`route_endpoint`配置仍保留兼容。

流程与限制详见包内`autonomous_window_test/FULL_CHAIN_V1_CN.md`的v1设计；本v1.1说明覆盖其中“米制数据缺失/待Nano上电”的历史状态。新包目录`/home/abot/full_chain_20261024_v1_1`，USB包`full_chain_vision_20261024_v1_1`，RGB-D包`full_chain_rgbd_20261024_v1_1`。只将本包加入本次ROS路径。

板端编译及地面恢复：

```bash
bash /home/abot/full_chain_20261024_v1_1/install.bash
bash /home/abot/full_chain_20261024_v1_1/ground.bash
bash /home/abot/full_chain_20261024_v1_1/ground.bash --restore
```

新会话绑定与只读预检（不ARM、不OFFBOARD、不起飞、不ON）：

```bash
bash /home/abot/full_chain_20261024_v1_1/bind.bash \
  --station-reference takeoff_body \
  --station-forward-m 3.15 --station-left-m -0.05 \
  --station-evidence '2026-10-24 operator-approved drawing estimate; forward=drawing right; start(0.85,3.85), target1(4.0,3.8); accuracy unverified' \
  --boundaries /home/abot/full_chain_20261024_v1_1/FIELD_BOUNDARY_CURRENT_20261024.json \
  --current-flight-area-confirmed
```

必须采集本次新boot/run与8秒双定位静置；真实预检通过才打印新的`full_chain_trial_.../run_recorded.bash --execute`供用户现场执行。不能沿用旧trial。相机画面不要求地面完整看靶。

起飞侧移根因仍未解决，12厘米中止保留；流程完成与中心命中分开。v1.1回归增加了实际3米、0.30m/s、12厘米前导与对框横移后靶站仍固定的验证。实际上传、板端编译、恢复和预检的状态以本轮新证据为准，不能只凭文档命令认为这些动作已成功。

现场恢复修复：显式加入原session包ROS路径；仅补缺失MAVROS/LIO，保留已运行10Hz雷达与新run。周期OFF刷新造成的OFF_PENDING，地面预检最多等待2秒获取新的新鲜OFF_ACK；不接受pending、旧ACK、故障或已用曝光。预检明确报告12厘米起飞水平中止与本轮靶站策略。

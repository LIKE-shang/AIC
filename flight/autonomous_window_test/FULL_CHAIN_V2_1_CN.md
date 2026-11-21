# 完整链路 v2.1：离线修订，等待Nano重新上电

2026-10-24用户贴出的日志明确显示：PASS于6208.476232开始，6209.148640因attitude_limit中止，间隔0.672408秒；实测0.1437000086437999rad≈8.2334°，超过v2的8°。不是本次已达到1.5m/s的证据；按指令曲线此时速度约0.54m/s，真实速度需下一次取bag核对。其余故障快照：高度余量约11.82cm，纵向指令跟踪误差约11cm，航向误差约0.058°，没有进入悬停、视觉修正或开灯。

本地变更：

- 新增明确session_window.pass_tilt_limit_rad=15°，仅smooth session的PASS生效；其他阶段control.tilt_limit_rad仍为8°。
- 连续圆柱几何支持函数按15°完整姿态锥预留占用；SEARCH/ADJUST选点与最终冻结路线均重算。不是只把故障阈值提高而沿用8°净空。
- PASS上限15°，超过仍立即中止；逐边4cm刚性净空、8cm横向/垂向偏离、真实场地围栏、0.7m机底高度、35cm纵向跟踪和末端停稳/整机离框条件保留。
- 保留1.5m/s峰值、1m/s²加速度、4m/s³加加速度，三米零速起止轨迹；本次不修改飞控参数，原MPC_TILTMAX_AIR=15必须新上电读回。
- 实时状态输出effective_tilt_limit_rad和passage_geometry_tilt_limit_rad；FAULT快照保留触发时角度和当时生效阈值，避免终态标签丢失故障现场条件。
- 用户要求删除的简化起飞12cm水平中止继续关闭。

日志中的冻结框几何已整理为test/pass_tilt_v2_log_fixture.json，只用于离线回归，禁止加载成新飞行坐标。新包：`/home/abot/full_chain_20261024_v2_1`（计划路径，尚未上传）。

Nano当前断电，本轮没有连接、上传、改参数或执行飞行。原trial的ready状态已过期；旧命令不得用于下一次上电。

重新上电后先取得新boot/run/节点来源，恢复相关节点，检查当前姿态、定位、USB/RGBD与激光OFF确认，再按新模板生成全新trial，完成只读地面预检后给出新命令。届时的恢复入口为：

```bash
bash /home/abot/full_chain_20261024_v2_1/install.bash
bash /home/abot/full_chain_20261024_v2_1/ground.bash --restore
bash /home/abot/full_chain_20261024_v2_1/bind.bash --station-reference takeoff_body --station-forward-m 3.15 --station-left-m -0.05 --station-evidence '2026-10-24 operator approved drawing estimate; full field follows drawing' --current-flight-area-confirmed --boundaries /home/abot/full_chain_20261024_v2_1/FIELD_BOUNDARY_CURRENT_20261024.json
```

这些是部署后使用的地面命令，本轮没有执行。未提供可用实飞trial。离线回放/语法通过不等于实际穿框或中心命中通过；整体测试仍未通过。

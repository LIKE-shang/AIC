# 完整链路v3：穿框制动、停稳与单靶稳定曝光

2026-10-24本次bag telemetry_980398050848.bag显示：PASS开始1021.792916512、FAULT1024.998639712，故障straight_progress_outside_route；实际沿路线3.163884m，原阈值3.15m。故障时前进速度估计0.577m/s，尚未停稳；不是打靶节点拒绝，而是未进入POSTPASS。峰值沿线速度估计1.742m/s，超过参考1.5m/s，说明制动动态未实测闭环。

v3仍由同一个监督器一次起飞，固定三米路线、15度PASS包络与8度其他阶段、起飞12cm中止关闭。高速S曲线在2.625米结束（3.5秒，峰值1.5m/s）；最后0.375米只以≤0.15m/s移动位置参考，速度前馈为0，留给飞控制动和终点捕获。最终路线端点仍为三米，不把新几何当作已经测准。

完全离开冻结框的整机支持半空间之前，原8cm横向/高度跟踪与逐边4cm刚性净空保留。整机已离框后，终点捕获区域改为相对路线30cm侧向/高度、最多50cm前向超调、65cm纵向跟踪误差，同时必须位于原真实场地围栏；一旦已确认离框，若重新侵入框占用区域会中止。捕获最多12秒，不无限等待。仅允许末端参考到达、反馈回到30cm端点区域、实际-指令≤12cm、倾角回到8度以内、0.5秒净位移速度≤0.04m/s且窗口跨度≤4cm持续1.5秒后，才确认stop并移交POSTPASS。后续固定到实测停止位置而非瞬间跳回名义端点，起飞原点/高度/航向和冻结几何不重置。

POSTPASS先垂向回到机底约0.9米，再低速移动到起飞体轴靶1估算(3.15,-0.05)米。转移仍≤1m、≤0.08m/s，瞄准范围仍半径15cm；所有实际/参考保持离框和真实围栏。靶站稳定也使用0.5秒净位移及跨度，防止30Hz逐点差分把小定位抖动误判为未停稳。

新require_stable_target_before_light只在此模板开启：真实新鲜USB靶心识别、预测参考点进入内环归一化半径0.45以内、定位/指令误差≤4cm、净位移停稳持续0.6秒，且悬停至少2秒，才能请求AUX5=100。15秒内不满足则请求OFF、确认后POSCTL，不能无靶按时亮灯。亮灯后继续小幅视觉调整，必须新鲜观测光点与靶心误差/内环短轴≤0.45、光点在内环、停稳连续3秒才完成视觉曝光。中断则重新计时；最多5秒硬件ON限制保持，未达成不报成功。此视觉曝光依据不是外部中心命中验收。OFF确认和实际POSCTL回读仍必需，AUX6=0，飞手降落。

新版包目录（部署目标）：/home/abot/full_chain_20261024_v3。不能使用旧trial命令。部署后重新读boot/run、FCU参数与来源并绑定：

```bash
bash /home/abot/full_chain_20261024_v3/ground.bash
bash /home/abot/full_chain_20261024_v3/bind.bash --station-reference takeoff_body --station-forward-m 3.15 --station-left-m -0.05 --station-evidence '2026-10-24 operator approved drawing estimate; full field follows drawing' --current-flight-area-confirmed --boundaries /home/abot/full_chain_20261024_v3/FIELD_BOUNDARY_CURRENT_20261024.json
```

若节点来源/boot/run/起点变化需重绑。仅新地面预检ready后，由用户现场运行新run_recorded.bash --execute；代理不远程起飞、不测试ON。本轮保留健康v2.1定位/视觉支持节点（消息协议相同），新监督器独立v3；准确来源须保存在新快照里。录制器严禁订阅/mavros/setpoint_raw/local，只订阅target_local。

上述制动与视觉条件已做离线回归，实际穿框末端制动、视觉方向/光点稳定和中心命中仍需现场实飞验收；不能保证仅改条件就已稳定命中。

# 穿框→单靶完整链路 v2（2026-10-24）

本版本删除完整链路简化起飞阶段的12厘米水平偏差中止条件。仍使用原定点起飞、原高度到达判据和同一个飞行监督器。起飞后的真实场地围栏、输入新鲜度、遥控接管保留。旧 trial 禁止复用。

穿框使用三米固定方向、固定高度的解析七段 S 曲线：最高1.5m/s，加速度≤1m/s²，加加速度≤4m/s³，三米轨迹3.75秒。加速1.3125米、匀速0.375米、减速1.3125米。1.5m/s是最高轨迹速度；框较近时，经过框平面的速度低于该值。冻结的三米路线不因视觉更新而重规划。位置与速度同时发送；PASS之外不发送该速度前馈。

为适配此速度：沿路线跟踪偏差独立上限35厘米，移除旧12厘米前导量对PASS的限速；横向/高度偏离仍≤8厘米且逐边满足实际冻结框净空。连续倾斜包络由5度调整为8度，框边净空重新计算（不直接放行原路线）；机身31厘米半径、4厘米刚性净空保留。10Hz定位单步上限35厘米、速度上限2.2m/s；时间戳、帧、来源、会话变化和异常跳变仍中止并锁存。末端允许最多15厘米纵向超调，但只有回到3.5厘米到达范围、实际速度≤0.04m/s持续1.5秒且整机离框后，才继续POSTPASS。

飞控只修改 MPC_XY_VEL_MAX：0.7→1.5。MPC_XY_CRUISE保留；实时读回MPC_ACC_HOR=1、MPC_ACC_HOR_MAX=1、MPC_JERK_AUTO=4、MPC_TILTMAX_AIR=15，并和原21个参数一起预检。参数工具仅在可信、未解锁、CH6释放、CH7关闭的地面状态执行，先备份再写入并读回；不解锁、不发送起飞或激光开启命令。

Nano目录：`/home/abot/full_chain_20261024_v2`。部署后必须执行地面准备和新绑定，禁止沿用v1.1命令：

```bash
source /home/abot/full_chain_20261024_v2/env.bash
python3 /home/abot/full_chain_20261024_v2/autonomous_window_test/scripts/prepare_smooth_pass_speed.py
bash /home/abot/full_chain_20261024_v2/ground.bash --restore --upgrade-pose
bash /home/abot/full_chain_20261024_v2/bind.bash --station-reference takeoff_body --station-forward-m 3.15 --station-left-m -0.05 --station-evidence '2026-10-24 operator approved drawing estimate; target1 near nominal 3m endpoint; full field follows drawing' --current-flight-area-confirmed --boundaries /home/abot/full_chain_20261024_v2/FIELD_BOUNDARY_CURRENT_20261024.json
```

只有新绑定输出ready=true后，用户在现场执行它打印的新run_recorded.bash --execute。代理不远程起飞。重启、来源改变或移动起点后重新绑定。

穿框后同监督器停稳、到0.9米、转移到起飞体轴估算靶1(3.15,-0.05)米、USB视觉微调、AUX5=100定时亮灯、AUX5=0且AUX6=0、关灯确认、请求并观察POSCTL，由飞手降落。录制器只订阅target_local，禁止订阅setpoint_raw/local。

回归/地面预检不是飞行验收。速度实际响应、过框平稳性、视觉收敛及中心命中仍需现场证据；未承诺已经命中。

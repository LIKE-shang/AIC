# 起飞跟踪诊断与遥控器设置（2026-10-19）

Nano 当前关机。v10 仅在本地准备，未部署、未进行实飞。v10 保留 v9 已批准的起降测试角度豁免和伴随程序电池豁免；没有删除定位、位置跟踪、围栏、速度、超时和遥控接管检查。穿框例外未扩大。

## 本次修改

tracking 每帧包含目标与实际局部坐标、目标减实际的三轴偏差、水平偏差、垂直偏差、总偏差和限制值。fault_snapshot 在 fail() 时冻结故障阶段、原因、时间和当时的位置证据，后续自动降落的坐标更新不会覆盖它。终端 position=null 表示停止发布目标，不能据此认为故障时没有目标坐标。

起飞目标本来就会从起飞点高度逐渐上升，起飞位移不是自动触发 tracking_error 的条件。判断使用目标与实际位置的差；当前现场配置限制为 0.12 m。缺少上一轮故障瞬间的分轴数据，不能判断是实际漂移、目标跟踪不足或定位变化，不能保证放宽阈值就能起飞。

## 遥控器：按历史参数，重新上电后读回确认

| 项目 | 准备状态与用途 |
|---|---|
| 模式开关 CH5 | 选择已确认的 Position 档，读回 /mavros/state 为 POSCTL。历史映射：低档对应槽1 Position，中档槽4 Land，高档槽6 Altitude；没有 MANUAL 档。不要用 Land 档准备起飞。 |
| Offboard 开关 A / CH7 | 保持关闭，当前程序要求通道值 900–1200（常见约1050）。程序自行请求 OFFBOARD，不需要拨 A 启动。运行中拨动 A 会取消任务；不要作为试启动动作。 |
| 急停 / CH6 | 准备时保持经过确认的非急停位置。不能只根据1050/1950猜测开关方向。急停可能停桨，不用于普通接管或正常降落。 |
| 解锁 | 以 /mavros/state 的 armed=false 为准备条件；不要提前手动解锁，程序自行请求。历史 RC_MAP_ARM_SW=0，所谓遥控器“解锁档”不能代替飞控状态。 |
| 摇杆 | 自动阶段保持已校准中位。历史 COM_RC_OVERRIDE=3、COM_RC_STICK_OV=30；摇杆接管是已配置能力，需结合本机既有验证方式操作，不能假设一个未知开关保证接管。 |

以下只读取状态，不起飞、不改参数。在基础系统启动后执行：

~~~bash
source /home/abot/window_pass_ground_env.bash
timeout 5 rostopic echo -n 1 /mavros/state
timeout 5 rostopic echo -n 1 /mavros/extended_state
timeout 5 rostopic echo -n 1 /mavros/rc/in
for name in RC_MAP_FLTMODE COM_FLTMODE1 COM_FLTMODE4 COM_FLTMODE6 RC_MAP_KILL_SW RC_KILLSWITCH_TH RC6_REV RC_MAP_OFFB_SW RC_MAP_ARM_SW COM_RC_OVERRIDE COM_RC_STICK_OV; do
  printf '\n%s\n' "$name"
  rosservice call /mavros/param/get "param_id: '$name'"
done
~~~

车辆放回原调查起点、车头保持原方向、落地未解锁时，保持车辆静止再做地面绑定。重新上电不得直接沿用旧坐标和旧 current_trial。当前已安装 v9 的基础与重新绑定命令：

窗口1：
~~~bash
bash /home/abot/autonomous_window_test_20261019_v9/start_base.bash
~~~

窗口2：
~~~bash
bash /home/abot/autonomous_window_test_20261019_v9/reboot_setup.bash manual
bash /home/abot/autonomous_window_test_20261019_v9/reboot_setup.bash trial --same-start-confirmed
~~~

上述 manual 仅在落地未解锁时准备飞控模式；trial 创建本次坐标并预检，均不请求解锁。v10 尚未传到 Nano，这里不提供 v10 起飞命令。待安装、核对本次配置并完成预检后再安排受监督起降。

历史证据：STEP4_REVIEW_20261018.json、STEP4_REVIEW_CN.md、_diagnostics/step4_fcu_readback_20261018.json；定制固件行为不能仅凭官方参数定义认定已验证。

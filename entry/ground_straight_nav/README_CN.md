# 地面直线任务：20261018_audit_v2

底盘入口已更正，详见 ../_deploy/CHASSIS_ENTRY_CORRECTION_20261018_CN.md。已发布 v2 归档内旧启动说明不再适用。

本包在地图中以全向平移方式经过航点，angular.z 始终为 0。**零角速度指令不能保证实际车头完全不漂移**；超过 yaw_abort_rad 时中止，必须显式重新启动。

## 定位与控制

- 默认使用 /localization，必须为 map 帧，时间戳必须新鲜且递增，四元数和坐标有效。
- 修复后的 transform_fusion 在没有 ICP 结果时不发布 /localization；里程计超过 0.5 秒或 ICP 校正超过 6 秒时停发。导航在位姿超时后中止，恢复数据不会自动续跑。
- 首次启动和重启均锁定当时起点，relative 航点据此转换成固定地图点；每次发送速度时使用当前 yaw 转到车体系。
- 超过 0.15 m + 2 × max_speed_mps × 采样间隔的单次位置变化会中止。该阈值是保护阈值，需要结合实机噪声验证。
- 启动服务拒绝已有 /cmd_vel 其他发布者；运行期间不可再启动遥控、move_base 或其他速度节点。完成/中止后本节点持续发零，交接穿框控制前须结束本节点。
- 控制、位姿、开始/停止服务使用同一把锁，避免 stop 返回后旧控制周期再发非零速度。

## 上车流程

先按部署包 INSTALL_CN.md 安装修复版定位脚本和本包。源工作空间为 ~/abot_ws 时：


默认 config/straight_run.yaml 仍是“前进 1 米再返回”的演示，必须先审核现场航点。

1. source /opt/ros/noetic/setup.bash 与 ~/abot_ws/devel/setup.bash。
2. roslaunch starrobot_bringup minimal.launch（厂家 rosserial 入口，节点名 rosserial_starrobot；不要使用 minimal_uart.launch）。完整静态 TF 链见厂家 abot_bringup/agv_base.launch。
3. **另外启动 Mid360 驱动**；slam_location.launch 不含雷达驱动。确认 /livox/lidar 和 /livox/imu 持续输出。
4. roslaunch slam_navigation slam_location.launch。
5. rosrun slam_navigation publish_initial_pose.py X Y Z YAW PITCH ROLL，均为地图坐标/弧度，六个值由现场给定；该节点保持锁存，初始化完成后 Ctrl+C。
6. 确认 /map_to_odom 有更新，/localization 与实测地图位置一致，再运行 bash $(rospack find ground_straight_nav)/tools/preflight_ground_nav.sh。
7. 用以下命令审核路径（参数不得照抄占位符）：

```bash
rosrun ground_straight_nav check_waypoints.py --config $(rospack find ground_straight_nav)/config/straight_run.yaml --start-x X --start-y Y --start-yaw YAW --robot-radius R
```

R 是**以定位 body 原点为圆心**包住整车的实测半径加净空余量，单位米。本检查考虑整车圆形包络、占用/未知/地图边界和地图 origin yaw。替代了不存在的地图图像路径时会返回失败。

8. 单独运行轴向检查（会移动车）：rosrun ground_straight_nav axis_check.py --speed 0.06 --seconds 1.0；确认无其他速度发布者并保持急停可用。若方向不符，先检查坐标外参及底盘配置，不应直接取反某个误差项掩盖问题。
9. roslaunch ground_straight_nav straight_run.launch。
10. rosservice call /ground_straight_nav/start；停车使用 rosservice call /ground_straight_nav/stop。状态见 /ground_straight_nav/status。

## 本地验证与边界

python3 -m unittest discover -s test -p 'test_*.py'

python3 tools/simulate_run.py --config config/straight_run.yaml --start-yaw 1.5707963268

离线测试不能验证 Open3D 实际配准、地图正确性、雷达与底盘外参、轮系方向或 MCU 停车行为。本节点不做动态避障；max_run_distance_m 是相对起点的**半径围栏**，不是累计路程或失定位后的行驶距离保险。前版误审的是另一个自定义串口驱动；厂家的 rosserial/MCU 超时停车实现尚未取得，必须现场验证断流/进程崩溃时可靠停车后再落地运行。横移能力与轴向需通过正确驱动验证，不能由旧 C++ 分支推断。

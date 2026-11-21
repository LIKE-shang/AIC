# 三环测试源码安装器（Nano）

这个安装器只更新 `robot_vision` 的三环测试源码和两个明确审阅过的构建元数据
文件（`CMakeLists.txt`、`package.xml`）。默认是只读检查；不会编译、启动或停止
ROS 节点，不会连接 MAVROS，不会加载 PWM 动态库，也不会调用激光服务。

## 安装前必须满足

- 仅保留用户明确启动的 `/rosout` 和 `/circle_usb_camera`；停止三环检测器、桥接器、
  MAVROS、任务和其他会写入 `robot_vision` 的进程。
- 飞控未解锁，桨叶已移除或有等效机械隔离，激光物理断电/断开。
- 已保留 `/home/abot/robot_vision_before_airtest*.tar.gz` 备份；安装器还会在写入前
  生成一次完整 `robot_vision` 快照。
- 暂存目录与现有包分开，例如 `/home/abot/air_target_stage.*`。

安装器只接受构建包对应的固定 SHA-256 版本，并且会拒绝备份之后的未知文件改动、
符号链接/重解析点和非规范路径。校验或安装失败时不会继续写入；写入中验证失败
会尝试恢复到本次快照并报告备份路径。

## 操作

```bash
cd /home/abot/air_target_install_stage
(cd /home/abot/air_target_install_stage && sha256sum -c SHA256SUMS.air_target_install)
python3 -E ./install_air_target.py \
  --stage /home/abot/air_target_install_stage \
  --package /home/abot/abot_ws/src/robot_vision \
  --backup-dir /home/abot
```

只读检查显示的 `ros_nodes` 只能是 `/rosout` 与 `/circle_usb_camera`。确认检查无误，
再在**同一现场状态**执行：

```bash
python3 -E ./install_air_target.py \
  --stage /home/abot/air_target_install_stage \
  --package /home/abot/abot_ws/src/robot_vision \
  --backup-dir /home/abot \
  --apply --confirm-idle
```

`--confirm-idle` 是操作者确认，不是软件对飞控或激光状态的证明。安装结束后仍需
人工检查输出中的备份路径和 `mode=INSTALLED_SOURCE_ONLY`；随后**另行**按仓库手册
审阅合并后的 CMake/package 内容，再按需执行 catkin 编译。安装器不会自动编译。

## 回滚

安装器失败会自动尝试回滚并报告 `rollback=complete` 或失败项。不要删除失败现场；
保留备份、终端输出和工作空间状态，先停止任何可能的写入进程，再人工恢复和复核。

此文件、安装器输出、离线测试和 ROS ACK 都不构成真实光斑命中或物理关断验收。

# 单靶 0.9 米悬停、相机微调与激光输出

本次新增的是实际任务控制分支，替代之前仅预览的调试方式。它不自动启动或切换正在运行的基础定位节点。

## 自动流程

1. 复用原 Mission 的唯一飞控发布者、起飞位置目标和 0.9 m 高度。
2. 地面仅要求相机状态持续发布，不要求完整靶面；起飞进入 HOVER 后采集新的完整三环。
3. 使用实测参考点 (372.75,194.984375)，像素偏移 (+53.25,+15.484375)。画面上方对应机头前方，右方对应机体右方。
4. 根据当前最内环尺度和 5.2 cm 内开口半径计算水平修正，保持高度和航向；最多偏离起点15cm，速度4cm/s，保留原跟踪/包络/定位/RC检查。没有跨靶移动。
5. 参考点进入内环半径45%以内并稳定0.6秒，才请求 AUX5=100；AUX6保持0。商家库/ABI/SHA验证沿用原实现。
6. 摄像头连续3秒观察亮点在最内环开口内且与参考点相符，发关光请求；若光点缺失、越界、相机失流或稳定性丢失，提前关光并终止本次。
7. 关光请求确认后请求 POSCTL，必须实际读回 POSCTL 才停止飞控位置发布，由用户降落。回执失败时结果标为未完成，不把回执当作物理发光证明。

独立执行器只接受 /autonomous_window_test 的单靶 HOVER 状态，在 connected/armed/OFFBOARD、当前图像时间戳和心跳有效时允许ON；每次进程仅允许一次曝光，独立最长5秒。启动首先OFF。原地面桥不修改，不允许二者同时驱动输出。

掉电或命令失联不能用软件保证物理熄灭，保留用户已说明的现场独立关断保障。程序不在退出时解锁、不自动再次接管。

## 使用现有起飞配置

新的 config/hover_target.json 是运动禁用的模板，不伪造本次定位绑定或原未验收字段。用户这次是在靶1，不能复用穿框旧起点的本地边界坐标。

若已有**本次靶1位置**绑定过并通过检查的 takeoff_land 配置，可创建独立配置：

```bash
python3 <本单靶程序目录>/scripts/prepare_hover_target.py \
  --base <本次靶1起点的takeoff_land配置.json> \
  --output <新的单靶配置.json>
```

它保留原 motion 开关与真实验收记录，只添加 hover_target 参数、禁用穿越及设置人工降落；不会从旧穿框配置自动转权限。不完整模板仍会在原预检中列明缺项。

## 图像和实际PWM执行器

相机正在运行时，先退出旧的独立 pixel debug launch（保留相机拥有者），再运行：

```bash
source /home/abot/abot_ws/devel/setup.bash
roslaunch robot_vision hover_target_support.launch \
  enable_laser:=true operator_pwm_verified:=true
```

这个命令启动相机分析和实际激光执行器，但不启动飞行程序；执行器初始只发OFF。
如相机没运行，附加 start_camera:=true video_device:=<已确认设备路径>。
查看话题仍是 /circle_pixel_preview/image。不要同时启动原 circle_laser_bridge。

## 起飞命令入口

```bash
bash <本单靶程序目录>/run_hover_target.bash --config <新的单靶配置.json>
```

默认只读预检；只有 ready=true 后，由用户现场添加 --execute 执行真实起飞。该程序执行期间会通过请求使激光实际开启，不是仅显示图形。远程部署不执行此命令。

由于尚未取得本次靶1的飞行绑定配置，不能将以上占位符当成当前已就绪的现场命令。

## 调整偏移

偏移需要同时在相机支持launch和独立任务JSON中一致修改，否则程序报告camera_reference_offset_mismatch，不起光。例如launch参数 offset_x_px:=53.25 offset_y_px:=15.484375；任务JSON的hover_target.offset_x_px/offset_y_px对应相同数值。图像 x向右、y向下。修改后需重启本次独立节点，不自动复用正在飞行中的配置。

若真正观察到明显光点偏差，先完成交还并降落，再改偏移重试。不通过修改容差将越界结果伪装成完成。

## 本轮验证

- 13项单靶合成测试；407项飞行总回归通过。
- 新激光执行器在Nano独立目录C++编译通过，未执行。
- 原图算法此前八帧回放验证；无新真实起飞、PWM或命中验收。

## 从穿框源码自动提取起飞设置（2026-10-22）

已定位本地穿框入口 `run.bash -> scripts/auto_window.py -> mission.py`，
配置来源 `config/session_window.json`。不再需要用户提供起飞命令。

新增命令支持从该文件直接生成单靶模板：

```bash
python3 scripts/prepare_hover_target.py --from-traverse-template \
  --output config/hover_target_from_traverse.json
```

本地已生成上述文件，命令以独占创建方式写入，已有文件不会覆盖。
复制了0.9米位置目标、商家式高度到达判据、0.245米机底/飞控偏置、
POSCTL后人工降落、原飞控参数期望值和定位坐标帧/跟踪设置。
保存来源SHA256和复制字段清单。

明确差异：session_window特有的 simplified_takeoff_enabled 仍关闭；
本单靶模板保留 takeoff_land 的0.8米最低机底条件，而穿框模板为0.7米。
所以这是复用起飞实现与参数，不是声称完整照搬穿框全部起飞豁免。
旧boot/run、旧起点边界、穿越权限、窗框安装和通道确认不被复制。
当前生成文件仍运动禁用、未绑定靶1，不能直接 --execute。

新增5项配置转换测试与13项单靶控制测试全部通过。
本次只修改本地准备脚本、测试、模板和说明，未上传或操作Nano。

## 2026-10-23 本次靶1试验准备完成

用户已确认当前靶1中心、机头方向及15cm修正/上升空间。
独立部署 `/home/abot/hover_target_single_20261023`，当前新绑定试验目录 `/home/abot/hover_target1_trial_37t050p_`。
地面只读预检ready=true、blockers=[]；OFF回执成功，未执行飞行或ON。
记录在 `../_diagnostics/hover_target1_trial_37t050p_/FIELD_HANDOFF_CN.md`。
新增独立 --commissioning-hover-target 范围，保留历史未完成验收false，不使用 --commissioning-takeoff 来放行水平修正。
现场使用本次trial/run.bash --execute，A关闭，实际POSCTL后人工降落。
启动脚本ROS环境兼容和LF换行已修复，运行中的基础、相机、定位桥和OFF执行器保留。

## 2026-10-23 首次实飞中止与 v2 修正

首次实飞进入 TAKEOFF 约1.35秒即因航向误差5.069度超过5度而 FAULT。
当时估计机底仍在地面，未进入HOVER、未发出激光ON请求。
ARM成功和飞控airborne标志不能代替真实高度到达；本次测试未完成。

v2保留0.9米位置目标、0.8米最低机底判据及当前像素偏移。
只在单靶、位置目标、商家到达判据且已解锁起飞阶段，使用已有起飞瞬态策略中的30度航向范围。
并未打开 simplified_takeoff；倾角、水平跟踪、超高、包络、定位、RC及控制源检查仍保留。
进入HOVER前必须恢复原5度航向范围，HOVER后仍按5度检查。
未恢复时继续保持原高度与航向目标，按原起飞超时处理；不触发激光。
状态增加 heading_limit_rad、hover_takeoff_heading_tolerance_active 和到达阻塞项 hover_heading_recovered。

故障后只读飞控确认 landed_state=1、armed=false、POSCTL，但system_status=8。
该状态不满足原有起飞就绪条件，v2不会绕过该检查，也没有远程重试飞行。

# 换电后重新执行原地起降试验

2026-10-19，v6 已安装；保留上次批准的受监督低速起降范围。无需重新安装或测窗。
车放回之前量过边界的起飞位置，车头方向不变。遥控开启，A 关闭、未解锁。
本轮不穿框；车底目标约 0.90 m，稳定悬停 5 秒后降落。

## 窗口 1：基础定位

Windows PowerShell：

~~~powershell
ssh.exe abot@192.168.0.100
~~~

Nano：

~~~bash
bash /home/abot/autonomous_window_test_20261019_v6/start_base.bash
~~~

保持这个窗口运行。初期提示等待 MANUAL 时，继续窗口 2。
如果报 BASE_ALREADY_RUNNING/MASTER_ALREADY_RUNNING，保留已有基础系统，不重复启动。

## 窗口 2：模式准备和本轮试验

另开 PowerShell 并 SSH：

~~~powershell
ssh.exe abot@192.168.0.100
~~~

先做地面模式准备（仅落地未解锁、状态 3、A 关闭时请求 MANUAL）：

~~~bash
bash /home/abot/autonomous_window_test_20261019_v6/reboot_setup.bash manual
~~~

看到 BASE_READY 后，车保持静止，在同一终端运行：

~~~bash
bash /home/abot/autonomous_window_test_20261019_v6/reboot_setup.bash trial --same-start-confirmed
~~~

这条命令只采集地面位置、创建新的试验配置并预检，不会起飞。
它会打印新日志目录。只有看到 ready=true、blockers=[] 和 TRIAL_READY，才继续。
它不会把未做的仿真/完整标定/实际失效试验改成已验收，仍显示本轮接受的未验证项。
若失败，停在此处保留原输出，不运行下一条。

飞手和现场准备好后，仍在窗口 2 执行真实起降：

~~~bash
bash /home/abot/autonomous_window_test_20261019_v6/current_takeoff.bash --execute
~~~

**这条会自动请求解锁起飞，不会再询问。无需按 A 启动。**
固定 XY/航向 → 车底约 0.90 m → 稳定悬停 5 秒 → 自动降落。
以 DONE 且 takeoff_hover_completed=true 为完整起降结果。
日志自动追加保存到本轮目录中的 mission.log；不要靠关闭 SSH 或 Ctrl+C 当作降落。
异常按已验证的遥控接管方式处理。落地且未解锁后再回传最终结果。

## 可选窗口 3：起飞前启动相机与识别

独立起降无需相机；如果需要画面，在起飞前另开 Nano SSH 终端：

~~~bash
bash /home/abot/autonomous_window_test_20261019_v6/start_perception.bash
~~~

已运行相机/识别时不要重复执行。保持窗口运行。

Linux 192.168.0.135 的本机桌面终端看图：

~~~bash
source /opt/ros/noetic/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=192.168.0.135
rosrun rqt_image_view rqt_image_view /window_live/image
~~~

地面看不到窗框不影响本轮原地起降。不要执行旧 geometry/trial/air 命令。

# 比赛调试简化入口 v5（2026-10-14）

当前阶段：实时RGB找框、深度测量、稳定几何与10cm以内横向目标预览。v5简化计算和人工操作；实际小步/穿框控制尚未接入这条RGB-D链路，不会自动解锁、切模式或发送飞控目标。v5已通过138项离线测试，尚未部署Nano验证。

## 日常只保留两个终端

Nano终端1：已有健康定位就继续使用。只有换电后基础系统未运行时执行：

```bash
bash ~/window_live_rgbd_20261014_v5/run_rgbd.bash base
```

Nano终端2：地面未解锁，A关闭，完整框稳定入镜，执行：

```bash
bash ~/window_live_rgbd_20261014_v5/run_rgbd.bash start
```

这一条启动相机、RGB-D检测和状态显示，自动保存诊断。无需手输窗框尺寸/旧map坐标，无需第三个终端反复check。终端1单独保留，更新/停止识别不会顺带关闭持续定位。

默认：RGB640×480；本设备已实帧验证的深度640×400；RGB找框内部宽480像素，角点换回原图坐标做标定深度计算；关闭额外全图深度候选、完整穿框规划、持续bag录制。

## 这次更新一次

如果v4还没上传，直接跳到v5。

Windows PowerShell：

```powershell
scp C:/Users/jiangzx/Desktop/src/_deploy/window_live_rgbd_20261014_v5.tar.gz C:/Users/jiangzx/Desktop/src/_deploy/window_live_rgbd_20261014_v5.tar.gz.sha256 abot@192.168.0.100:/home/abot/
```

Nano终端2：仅Ctrl+C结束原RGB-D组，返回shell后：

```bash
cd ~ && sha256sum -c window_live_rgbd_20261014_v5.tar.gz.sha256 && tar -xzf window_live_rgbd_20261014_v5.tar.gz && bash ~/window_live_rgbd_20261014_v5/run_rgbd.bash start
```

现有定位终端和Linux192.168.0.135看图窗口保留。旧v3/v4归档不覆盖，旧competition_align.py和部署过的飞控辅助工具不用重传。

## 只看一处结果

启动后每3秒显示一行概况和一个当前动作建议；状态变化会立即显示，例如以下为格式示意，不是实机结果：

```text
定位50.0Hz | RGB30.0 / 深度30.0Hz | 处理5.0Hz | 等待稳定窗框 (confirming_current_target)
下一步：保持完整窗框稳定入镜，程序自动完成连续确认。
```

程序只优先显示当前阻塞原因。若识别/深度几何成立，则显示“识别阶段通过（仅预览）”和实时开口尺寸、横向目标预览。完整穿框误差预算、geofence和扫掠体积验证不参与这个阶段的判定。预览仍不是A_READY，小步没有因此被执行。

自动保存目录在启动时打印：`~/window_rgbd_logs/stage_*`。

- `latest.json`：每2秒更新当前完整诊断。
- `transitions.jsonl`：状态变化记录，上限约2MB，不持续录大图像。
- `first_geometry_ready.json`：首次通过识别阶段的证据，单次保存。

一般只需回传屏幕上的两行和自动日志目录，不再整段粘贴几十项字段。如果后续需要完整快照，已有可选命令：

```bash
bash ~/window_live_rgbd_20261014_v5/run_rgbd.bash diag
```

`check`仍可用，但只输出简短结果；并非日常必做步骤。

## 具体删减与优化

1. 本阶段关闭完整穿框路线、机体扫掠体积和未测误差预算检查的调用；以后执行穿框再处理它们。不把未知/未测伪装成通过。
2. 默认使用RGB候选和深度四边测量，关闭耗时的全图深度候选搜索；无唯一RGB目标时跳过没有用途的深度重投影。无纹理窗框仅靠深度发现的能力仍留在完整模式，本模式不宣称覆盖该情况。
3. RGB搜索从宽640减到480，深度仍基于原相机内参和原图坐标；没有resize深度或填洞。480分辨率跑了与640相同的真实图片、缺边、多目标和背景误检回归。
4. 将视觉确认和米制确认并行积累，去掉先等待视觉完成再开始米制确认的串行等待；仍保留各自帧数、跨度、间隔和稳定条件。在标准0.2s间隔合成样例里，第5帧即可达到几何确认。
5. 相机depth_align参数与工厂参数一起异步读取，图像线程不再每帧同步查询ROS参数服务器；3秒缓存失效与标定一致性检查仍有效。
6. 沿用v4的看门狗修正、明确确认重置原因和等价快速投影。原来图像流状态被目标历史覆盖的错误已经在代码中修正。

本阶段仍需要真实有效图像/深度、可用定位、唯一可测开口和未过期结果。人工不再逐项确认，程序自动处理。控制层原有遥控接管、单次10cm/4cm每秒上限和固定高度/朝向/窗法向约束没有修改；新RGB-D链路尚未接入实际控制层。

## 复现与边界

`_diagnostics/window_live_rgbd_20261014_v5/test_results.json`和test_output.txt记录138项离线测试及代码哈希。归档含MANIFEST和OFFLINE_RESULTS。构建另验证Python3.8语法、所有launch XML、归档路径/执行位/manifest。

验证环境为Windows本机Python/OpenCV/NumPy，不是真实ROS/Nano性能测试。RGB640和RGB480同套场景通过，不等于所有比赛框必识别。旧完整穿框算法仍在代码中，只有显式enable_traversal_planning:=true的完整模式才运行，缺失条件仍会拒绝。

当前不声称小步成功或完成穿框。用v5先把现场阻塞缩成一处明确状态，避免反复人工执行检查链。

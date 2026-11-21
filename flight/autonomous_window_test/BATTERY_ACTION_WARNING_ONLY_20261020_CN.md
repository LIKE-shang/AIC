# 用户批准的飞控低电自动动作变更（2026-10-20）

用户明确授权取消飞控低电自动保护动作，并报告起飞前自行测电压。解释与实施范围：COM_LOW_BAT_ACT从2改为0，即仅告警；没有关闭电压/电量遥测，没有伪造电压，没有取消RC失联、Offboard失流、急停或其他硬件保护。负载压降既可能真实也可能有仪表误差，本次不声称低电告警已证明是误报。

Nano新boot_id=39473f7f-c1cd-480d-b401-0033f963cdfa，连接时ROS master未启动。已使用原start_v19_base_ground.bash以唯一新run ID恢复基础10Hz系统，没有启动50Hz旧入口。当前MANUAL、armed=false、landed=1、system_status=3，未收到RC消息，pose桥等待遥控；无控制目标发布者。

在新鲜且来源为/mavros的落地未解锁遥测、无setpoint/mavlink/hold/align控制源条件下，备份后通过/mavros/param/set只改COM_LOW_BAT_ACT。读回前[2,0.]、后[0,0.]。同时读回对照以下均未变：COM_OBL_RC_ACT=0、COM_OBL_ACT=0、NAV_RCL_ACT=3、COM_BAT_ACT_T=10、BAT_LOW_THR≈.15、BAT_CRIT_THR≈.07、BAT_EMERGEN_THR≈.05。

执行NSH param save，随后param show COM_LOW_BAT_ACT返回0、没有未保存*标记。未重启飞控验证重启持久性；下次开机继续读回。不应称“任何低压都不影响飞行”，该变更只取消这个参数控制的低电自动动作，硬件欠压及其他保护不属于它。

远端备份 /home/abot/battery_action_warning_only_ugy6zv7m/：before.json、report.json和旧current trial配置备份。旧指针/home/abot/autonomous_window_current_trial.path对应takeoff_trial_1ktk5o40已标记enable_motion=false，因为新boot与参数策略变化。所有其他旧试验也不可复用，boot会话检查应拒绝。后续生成新配置必须声明COM_LOW_BAT_ACT=[0,0.]，不得直接用期望2的旧配置。

本地三个新配置模板takeoff_land/airborne_rgbd/align_only已更新期望清单为0；历史部署版本不覆盖（仅当后续独立版本部署/生成新trial时使用新清单）。此前商家式起飞到达判据仍仅本地修改、尚未部署。本次未发送模式/解锁/起飞请求。

证据：_diagnostics/battery_warning_only_applied_20261020.json、battery_policy_nano_backup_20261020.json、battery_warning_only_param_save_20261020.json；参数脚本set_battery_warning_only_ground_20261020.py。下一步需打开遥控器保持A关闭/急停释放，并按新会话重新核对地面定位和绑定后才能继续飞行准备。

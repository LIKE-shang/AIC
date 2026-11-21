# 解锁拒绝诊断补丁（2026-10-19，v7）

现场日志：OFFBOARD 已读回，随后 ARM_service_rejected_or_failed，本次未完成起飞。
远程读回显示 armed=false、landed_state=1、无 setpoint 发布者，模式为 AUTO.LAND。
这与结束 Offboard 目标流后的模式变化一致，不表示已经发生真实飞行或完成降落验收。

原程序只保存 success 布尔值，丢弃了 CommandBool 的 result 字段；现有日志不能确定根因。
只读检查没有获得本次拒绝的 FCU 文本。不能凭 GPS 无卫星、油门居中或相机日志猜测根因。

v7 的变化仅为诊断：
- 打印 FCU_COMMAND action=ARM accepted=... mav_result=...；
- 区分服务异常与 FCU 拒绝；
- 订阅 /mavros/statustext/recv，保存最近文本；
- 失败时立即注销位置输出，再以只读方式收集 2 秒迟到提示；
- 最终 JSON 带 last_command_feedback、recent_fcu_text、fcu_state_at_exit；
- 明确 ready=true 只是伴随计算机预检通过，FCU 尚未接受解锁。

未修改飞控参数、解锁保护、速度/高度/围栏门槛，也没有自动重试或强制解锁。
原地未解锁、没有其他控制器时，可用 reboot_setup.bash manual 恢复 MANUAL；
它会读回真实地面条件。然后由现场操作者在飞手准备好后执行一次起降命令采集反馈。
该命令若解锁成功仍会真正起飞，不是纯诊断命令。

# v24.8.1 PASS前向领先12cm（2026-10-21）

用户认为10cm前向领先仍可能使位置目标停等，要求再增加。此次仅将 faster_pass 的PASS最大前向领先从0.10m增加到0.12m；指令速度维持0.30m/s。普通控制字段max_setpoint_lead_m=.04保持原样，session profile的独立值覆盖PASS，不影响起飞/对框/旧模式。

前向领先与三维最大跟踪误差0.12m共享：每周期lead=min(profile lead, sqrt(max(0, .12²-cross_track²))-epsilon)。因此无侧偏时最多12cm；侧偏8cm时最多sqrt(.12²-.08²)=约8.94cm，避免组合误差超出12cm。8cm横截面硬限制、冻结窗框逐边净空（4cm刚性净空）、定位/高度检查均未放宽。没以松开这些保护换速度。

独立冻结路线同时保存0.30m/s、0.12m领先，运行中配置增大不能抬高已冻结值。status/preflight报告pass_maximum_forward_lead_m同步显示.12。用户此前报告物理穿框成功；程序上次仍因横截面>8cm进入FAULT，未完成3米及程序POSCTL交还。此次不是新实飞结果，也不能保证不会停。

368项离线测试通过：最大12cm有效领先、横向偏差消耗共同余量、控制配置改变不能扩大冻结值、8cm横向硬限制及3米合成全流程。非SITL/实飞验收。新配置模板version仍motion-disabled/site_identity=null；需重新绑定/预检，旧v24.8路径绑定不会自动继承新源码/前向值。

v24.8.1 lead12cm独立包已传入并部署源码目录/home/abot/autonomous_window_test_20261021_v24_8_1；全manifest哈希、Python编译和bash语法通过，归档SHA256 eb7f8e7a91c2df6099a1633bd9dcdeec9266fabd03db9d14d366c6659870e13d。368项离线测试通过。仅PASS快模式前向领先上限.12m，速度.30m/s不变、共用12cm跟踪向量预算；其他净空/偏差/高度保护不变。

Nano现boot=95d9b4b7-638d-467b-b967-f1454807f685，无ROS master/相关进程。文件已部署但未运行/预检/绑定。旧v24.8绑定仍引用原build，不用。下次必须先恢复新boot基础/桥/视觉、确认站点和3m通道，重绑预检后才给新命令。

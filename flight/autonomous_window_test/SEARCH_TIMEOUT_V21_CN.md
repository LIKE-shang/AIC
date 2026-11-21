# v21 SEARCH超时调查（2026-10-20）

本次观察：1364.988578 TAKEOFF，1373.227060 SEARCH，1383.243908识别超时主动交还，1383.982156实际POSCTL确认，handover_confirmed=true、alignment_completed=false。商家式起飞判据生效，没有进入ALIGN。

现场事后遥测POSCTL、armed=false、landed=1、status=8，CH6=1950、CH7=1050，符合落地后急停。不要自动重试。

已提取同一会话/时间段感知日志保存_diagnostics/v21_search_perception_evidence_20261020.txt。1374.091127~1384.346559多次geometry=stable_live_metric_geometry、LIVE IMAGE_RIGHT；PIPELINE确认计数16、25、33、41、49，source gap约0.199~0.300秒；单帧完成约242~283ms。不能把本次recognition_timeout解释成相机未发现窗框，也不能声称是同上次完全相同的过期原因。日志是节流摘要，不代表每一帧完全无异常。

代码路径有三层：RGB-D validate_metric/acquire -> 独立旧RGB target_fresh/confirmed -> request_hold（其内部要求speed<=0.04m/s、geometry/legacy投影一致等）。任一未通过都会留SEARCH直到10秒超时。本次终态只保存最终reason，覆盖了SEARCH内部失败原因，未存legacy observation状态、geometry live_reason或hold refusal；现有证据不足以确定唯一阻止项。

本地新增search_evidence/search_events诊断，逐项记录实时RGB-D ready/fresh/几何原因、几何接纳确认数、旧RGB是否fresh/confirmed/样本数、位置差分速度与hold拒绝原因。记录原因变更日志SEARCH_GATE并保留最后60个变更，交还后仍保留，不改变任何控制判据/阈值/超时。不因仅有RGB-D ready就忽略原RGB确认或降低检查。

212项离线回归通过，覆盖无几何超时保留原因、有几何而无legacy RGB、hold拒绝、事件长度。新增诊断仅本地修改，尚未部署或执行新飞行。Nano当前现场仍v21。

后续应先利用已有可用历史图像/数据做影子诊断或地面回放，若必须新飞行需用户归位/解急停后准备；不机械重复飞行、更不能直接删确认条件。上次37帧有效采集历史仍保留，无需再验证“能不能采到”。

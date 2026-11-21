2026-10-19 v12：修复ROS Master并发查询

故障发生在实际创建控制目标发布者和请求OFFBOARD/ARM之前。主线程执行identity()，后台graph_worker也在同一个rosgraph.Master实例上查询；Python XML-RPC传输复用HTTP连接，并发调用触发CannotSendRequest: Request-sent。

修复：ThreadLocalMaster为每个调用线程建立独立ROS Master客户端，线程内复用。图检查、节点来源唯一性和会话身份复核全部保留；不把通信异常忽略或视为检查通过。不改飞行高度、跟踪阈值、围栏、飞控参数、解锁流程或定位转发设置。

新增测试用真实本机XML-RPC服务模拟图查询和节点地址查询，6线程各15轮混合请求，确保无共享HTTP连接错误。节点查找失败仍向上传递。全部离线测试不是SITL或实飞验收。

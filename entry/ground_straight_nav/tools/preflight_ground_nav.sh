#!/bin/bash
# ground_straight_nav 开跑前检查 — Nano Bash，只读，不发任何速度。
# 用法： bash preflight_ground_nav.sh
set -u

PKG=${PKG:-ground_straight_nav}
PASS=0
FAIL=0

ok()   { printf '  [ok]   %s\n' "$*"; PASS=$((PASS+1)); }
bad()  { printf '  [FAIL] %s\n' "$*"; FAIL=$((FAIL+1)); }
warn() { printf '  [warn] %s\n' "$*"; }

echo "=== 1. ROS 与包 ==="
if [ -z "${ROS_MASTER_URI:-}" ]; then
  echo "  ROS_MASTER_URI 未设置，先 source /opt/ros/noetic/setup.bash"
  exit 1
fi
if ! timeout 5 rosnode list >/dev/null 2>&1; then
  bad "roscore 未运行（rosnode list 无法连接 master）"
  echo
  echo "先启动底盘与定位，再回来跑本检查。"
  exit 1
fi
ok "roscore 可达（ROS_MASTER_URI=$ROS_MASTER_URI）"
PFX=$(rospack find "$PKG" 2>/dev/null)
if [ -n "$PFX" ]; then ok "包路径 $PFX"; else bad "$PKG 不在 rospack 路径（需 catkin_make 并 source）"; fi

echo
echo "=== 2. 里程计位姿 ==="
FRAME=$(timeout 6 rostopic echo -n1 /localization/header/frame_id 2>/dev/null | head -1)
if [ -z "$FRAME" ]; then
  bad "/localization 无输出（slam_location + transform_fusion 未起，或 ICP 未完成）"
else
  case "$FRAME" in
    map|'"map"') ok "/localization frame_id = $FRAME" ;;
    *)     bad "/localization frame_id = $FRAME，不是 map，节点会拒收" ;;
  esac
  HZ=$(timeout 6 rostopic hz /localization 2>/dev/null | grep -m1 average | awk '{print $3}')
  if [ -n "$HZ" ]; then
    ok "/localization 频率 ${HZ} Hz"
  else
    warn "/localization 频率读不到（roslaunch 还在启动？）"
  fi
fi

echo
echo "=== 3. 底盘速度订阅方 ==="
# /cmd_vel is a topic, not a node: `rosnode info` only ever answers
# "unknown node" for it. Read the Subscribers block from `rostopic info`.
SUBS=$(timeout 6 rostopic info /cmd_vel 2>/dev/null | sed -n '/^Subscribers:/,$p' | grep '^ *\*' | awk '{print $2}' | sort -u | tr '\n' ' ')
if [ -z "$SUBS" ]; then
  bad "/cmd_vel 没有订阅者（starrobot_robot_node 未起）"
else
  ok "/cmd_vel 订阅者: $SUBS"
fi
NPUB=$(timeout 6 rostopic info /cmd_vel 2>/dev/null | sed -n '/Publishers:/,/Subscribers:/p' | grep -c '\*')
if [ "$NPUB" -gt 1 ]; then
  bad "/cmd_vel 有 $NPUB 个发布者，必须只有一个控制输出"
elif [ "$NPUB" -eq 1 ]; then
  ok "/cmd_vel 当前 1 个发布者"
else
  ok "/cmd_vel 当前无发布者（本节点启动后会成为唯一发布者）"
fi

echo
echo "=== 4. 静止 yaw 抖动（5 秒，车必须停着）==="
if timeout 12 python3 - <<'PY'
import math, rospy
from nav_msgs.msg import Odometry
ys, xs, stamps = [], [], []
invalid = []
def cb(m):
    stamp = m.header.stamp.to_sec()
    age = rospy.Time.now().to_sec() - stamp
    q = m.pose.pose.orientation
    if (m.header.frame_id != 'map' or stamp <= 0 or not -0.1 <= age <= 0.5 or
            not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w, m.pose.pose.position.x, m.pose.pose.position.y)) or
            not 0.9 <= q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w <= 1.1):
        invalid.append(True)
        return
    if stamps and stamp <= stamps[-1]:
        return
    stamps.append(stamp)
    y = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    ys.append(y); xs.append((m.pose.pose.position.x, m.pose.pose.position.y))
rospy.init_node('ground_yaw_jitter_probe', anonymous=True)
sub = rospy.Subscriber('/localization', Odometry, cb, queue_size=50)
rospy.sleep(5.0)
sub.unregister()
if invalid:
    print('  [FAIL] invalid or stale localization')
    raise SystemExit(1)
if len(ys) < 10:
    print('  [warn] 样本不足 %d 个' % len(ys))
    raise SystemExit(1)
def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))
ref = ys[0]
drift = max(abs(wrap(y - ref)) for y in ys)
mx = max(x for x, _ in xs) - min(x for x, _ in xs)
my = max(y for _, y in xs) - min(y for _, y in xs)
print('  样本 %d，%.1f Hz' % (len(ys), len(ys)/5.0))
print('  静止 yaw 最大偏移 %.4f rad（阈值 0.05 warn / 0.15 abort）' % drift)
print('  静止位置抖动 x %.3f m, y %.3f m' % (mx, my))
if max(mx, my) > 0.10:
    print('  [FAIL] stationary position drift exceeds 0.10 m')
    raise SystemExit(1)
if drift > 0.15:
    print('  [FAIL] 静止 yaw 抖动已超过 abort 阈值，先查出漂移原因')
    raise SystemExit(1)
elif drift > 0.05:
    print('  [warn] 静止 yaw 抖动偏大，实际运行中会更容易触发告警')
else:
    print('  [ok] 静止 yaw 稳定')
PY
then ok "静止位姿检查完成"; else bad "静止位姿检查失败或超时"; fi

echo
echo "=== 结果 ==="
echo "  通过 $PASS 项，失败 $FAIL 项"
if [ "$FAIL" -eq 0 ]; then
  echo "  可以继续：先跑 check_waypoints.py，再启动 straight_run.launch"
  exit 0
fi
echo "  有阻塞项，先处理上面的 [FAIL]"
exit 1

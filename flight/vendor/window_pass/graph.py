"""Pure ROS graph policy. Discovery is monitoring, NOT an atomic graph lock."""
OUTPUT = '/mavros/setpoint_raw/local'
OBSERVATION = '/window_pass_test/observation'
STATE = '/mavros/state'
EXTENDED = '/mavros/extended_state'
POSE = '/mavros/local_position/odom'
IMAGE = '/camera/rgb/image_raw'
INFO = '/camera/rgb/camera_info'


def check_graph(state, own_name, cfg, has_publisher=False):
    pubs, subs, services = state
    p, s = dict(pubs), dict(subs)
    for topic, nodes in pubs:
        if topic in ('/mavros/rc/override','/mavros/manual_control/send','/mavlink/to','/mavros/mavlink/to') and nodes:
            return False,'competing_direct_control:'+topic
        if topic.startswith('/mavros/setpoint_') or topic.startswith('/mavros/trajectory/') or topic.startswith('/mavros/actuator_control'):
            # MAVROS telemetry reports are not external setpoint publishers.
            telemetry = topic in ('/mavros/setpoint_raw/target_local','/mavros/setpoint_raw/target_global','/mavros/setpoint_raw/target_attitude','/mavros/setpoint_trajectory/desired','/mavros/trajectory/desired')
            others = set(nodes)-({cfg['fcu_node']} if telemetry else set())
            allowed = {own_name} if topic == OUTPUT and has_publisher else set()
            if others-allowed: return False,'competing_setpoint:'+topic+':'+','.join(sorted(others-allowed))
    names = {n for _, nodes in pubs+subs+services for n in nodes}
    forbidden = ('uav_mission','agv_mission','complete_mission','offboard_single_position','offboard_multi_position','px4ctrl','object_darknet')
    if any(n.rsplit('/',1)[-1] in forbidden for n in names): return False,'known_competing_node'
    for topic in (POSE,STATE,EXTENDED):
        if set(p.get(topic,[])) != {cfg['fcu_node']}: return False,'unexpected_source:'+topic
    if set(p.get(OBSERVATION,[])) != {cfg['detector_node']}: return False,'unexpected_observation_source'
    for topic in (IMAGE,INFO):
        if len(p.get(topic,[])) != 1: return False,'ambiguous_camera_source:'+topic
    if has_publisher:
        if set(p.get(OUTPUT,[])) != {own_name}: return False,'output_not_exclusive'
        if set(s.get(OUTPUT,[])) != {cfg['fcu_node']}: return False,'unexpected_output_subscriber'
    return True,'graph_exclusive_snapshot'

"""Takeoff-only commissioning policy. No camera, opening or pass permissions.
Original traversal entry remains unchanged; this policy never enables it.
"""
import math
from window_pass.config import load_config
from window_pass.core import Controller
from window_pass.geometry import vector

EVIDENCE=('body_origin_verified','ground_height_verified','takeoff_column_verified',
          'takeoff_landing_site_verified','fcu_failsafe_verified','simulation_verified')


def takeoff_blockers(c):
    errors=[]
    for k in ('local_frame','body_frame','evidence_id'):
        if not c[k].strip():errors.append('missing:'+k)
    for k in ('envelope_center_body_m','geofence_min_m','geofence_max_m'):
        if not vector(c[k]):errors.append('missing:'+k)
    if vector(c['geofence_min_m']) and vector(c['geofence_max_m']):
        if any(a>=b for a,b in zip(c['geofence_min_m'],c['geofence_max_m'])):
            errors.append('invalid:geofence')
    for k,maximum in [('position_uncertainty_m',.1),('stopping_margin_m',.2)]:
        if not 0<c[k]<maximum:errors.append('missing:'+k)
    if c['enable_traverse'] or c['pass_stage_approved']:
        errors.append('takeoff_test_requires_pass_disabled')
    return errors


class TakeoffCore(Controller):
    def __init__(self,control,attitude_checks_enabled=True):
        if type(attitude_checks_enabled) is not bool:raise ValueError("attitude policy must be boolean")
        c=load_config(control)
        errors=takeoff_blockers(c)
        if errors:raise ValueError('takeoff configuration refused: '+','.join(errors))
        super().__init__(dict(c,enable_motion=False))
        self.c=c
        if not attitude_checks_enabled:
            # Quaternion angular distance is at most pi. Keep validity, position
            # jumps, speed and timestamp checks; only angular jumps are exempt.
            self.c["max_attitude_step_rad"]=math.pi
        self.blockers=[]
    def inside(self, position):
        # A takeoff column starts at the floor; the generic all-attitude sphere
        # includes fictitious space below that floor. Use the same rotated box
        # and uncertainty reserves as Mission.corridor, not a larger fence.
        from envelope_fence import envelope_inside
        return self.pose is not None and envelope_inside(self.c,position,self.pose.q)
    def measurement(self):
        raise ValueError('takeoff_test_has_no_window_geometry')
    def request_hold(self,*args):return False,'takeoff_test_has_no_alignment'
    def request_start(self,*args):return False,'takeoff_test_has_no_alignment'
    def request_pass(self,*args):return False,'takeoff_test_cannot_commit_pass'


def check_takeoff_graph(state,own_name,cfg,has_publisher=False):
    # Same control-source exclusions as the original graph policy, without
    # requiring perception publishers for a vertical-only test.
    pubs,subs,services=state;p,s=dict(pubs),dict(subs)
    output='/mavros/setpoint_raw/local'
    feedback={'/mavros/setpoint_raw/target_local','/mavros/setpoint_raw/target_global',
              '/mavros/setpoint_raw/target_attitude','/mavros/setpoint_trajectory/desired','/mavros/trajectory/desired'}
    direct={'/mavros/rc/override','/mavros/manual_control/send','/mavlink/to','/mavros/mavlink/to'}
    for topic,nodes in pubs:
        if topic in direct and nodes:return False,'competing_direct_control:'+topic
        if topic.startswith(('/mavros/setpoint_','/mavros/trajectory/','/mavros/actuator_control')):
            others=set(nodes)-({cfg['fcu_node']} if topic in feedback else set())
            allowed={own_name} if topic==output and has_publisher else set()
            if others-allowed:return False,'competing_setpoint:'+topic
    names={n for _,nodes in pubs+subs+services for n in nodes}
    forbidden={'uav_mission','agv_mission','complete_mission','offboard_single_position',
               'offboard_multi_position','px4ctrl','object_darknet'}
    if any(n.rsplit('/',1)[-1] in forbidden for n in names):return False,'known_competing_node'
    for topic in ('/mavros/local_position/odom','/mavros/state','/mavros/extended_state'):
        if set(p.get(topic,[]))!={cfg['fcu_node']}:return False,'unexpected_source:'+topic
    if has_publisher:
        if set(p.get(output,[]))!={own_name}:return False,'output_not_exclusive'
        if set(s.get(output,[]))!={cfg['fcu_node']}:return False,'unexpected_output_subscriber'
    return True,'takeoff_control_graph_exclusive'

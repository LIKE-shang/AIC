"""Transport boundary tests with fake ROS services: never connect to hardware."""
import sys,unittest,types,threading,importlib.util,json,hashlib
from pathlib import Path
from unittest.mock import patch,Mock
from test_mission import Sim,ROOT
spec=importlib.util.spec_from_file_location('auto_window_transport',ROOT/'scripts/auto_window.py')
app=importlib.util.module_from_spec(spec);spec.loader.exec_module(app)

class ImmediateThread:
    def __init__(self,target,**kw):self.target=target
    def start(self):self.target()

class Boundary(unittest.TestCase):
    def runtime(self,s):
        r=app.Runtime.__new__(app.Runtime)
        r.lock=threading.RLock();r.mission=s.m;r.core=s.m.core;r.inputs=s.inputs
        r.calls=[]
        r.takeoff_only=False;r.params_wall=s.f.wall
        def proxy(name,typ):
            def invoke(**kw):
                r.calls.append((name,kw))
                return types.SimpleNamespace(success=True,mode_sent=True)
            return invoke
        r.ros=types.SimpleNamespace(ServiceProxy=proxy,logerr=lambda *a:None,loginfo=lambda *a:None,logwarn=lambda *a:None)
        return r
    def request(self,r,action):
        srv=types.ModuleType('mavros_msgs.srv');srv.SetMode=object;srv.CommandBool=object
        pkg=types.ModuleType('mavros_msgs');pkg.srv=srv
        with patch.dict(sys.modules,{'mavros_msgs':pkg,'mavros_msgs.srv':srv}),patch.object(app.threading,'Thread',ImmediateThread):
            r.request(action)
    def test_arm_only_after_observed_offboard(self):
        s=Sim();s.start();s.until('ARM');s.armed=False;s.update();r=self.runtime(s)
        self.request(r,'ARM')
        self.assertEqual(r.calls,[('/mavros/cmd/arming',{'value':True})])
    def test_no_arm_on_ground_idle(self):
        s=Sim();r=self.runtime(s);self.request(r,'ARM');self.assertEqual(r.calls,[])
    def test_no_late_arm_after_takeover(self):
        s=Sim();s.start();s.until('ARM');s.m.phase='TAKEOVER';r=self.runtime(s)
        self.request(r,'ARM');self.assertEqual(r.calls,[])
    def test_no_arm_when_pose_stale(self):
        s=Sim();s.start();s.until('ARM');s.armed=False;s.update()
        s.m.core.streams['pose']=(s.f.ros,s.f.wall,False);r=self.runtime(s)
        self.request(r,'ARM');self.assertEqual(r.calls,[])
    def test_land_uses_mode_not_disarm(self):
        s=Sim(False);s.start();s.until('LAND_REQUEST');r=self.runtime(s)
        self.request(r,'LAND')
        self.assertEqual(r.calls,[('/mavros/set_mode',{'base_mode':0,'custom_mode':'AUTO.LAND'})])
    def test_publish_refuses_missing_mavros_subscriber(self):
        s=Sim();r=self.runtime(s);r.motion=Mock();r.motion.get_num_connections.return_value=0
        with self.assertRaises(RuntimeError):r.publish({'position':[0,0,1]})
        r.motion.publish.assert_not_called()
    def test_terminal_output_does_not_publish(self):
        s=Sim();r=self.runtime(s);r.motion=Mock()
        r.publish({'position':None});r.motion.publish.assert_not_called()
    def test_previous_boot_geometry_is_rejected(self):
        raw=b'{}';meta={'sha256':hashlib.sha256(raw).hexdigest(),'identity':{'boot_id':'old'}}
        p=Mock();p.read_bytes.return_value=raw;p.with_suffix.return_value.read_text.return_value=json.dumps(meta)
        with patch.object(app,'Path') as P:
            P.return_value.read_text.return_value='new'
            with self.assertRaisesRegex(ValueError,'previous_boot'):app.geometry_identity(p,Mock(),Mock())
    def test_modified_geometry_rejected(self):
        p=Mock();p.read_bytes.return_value=b'{}';p.with_suffix.return_value.read_text.return_value=json.dumps({'sha256':'wrong','identity':{}})
        with self.assertRaisesRegex(ValueError,'hash_changed'):app.geometry_identity(p,Mock(),Mock())
    def test_emergency_pulse_latches_even_if_next_sample_recovers(self):
        s=Sim();s.start();s.until('SEARCH');r=self.runtime(s);r.rows={}
        r.store('/mavros/rc/in',types.SimpleNamespace(channels=[1500]*6+[1950]))
        r.store('/mavros/rc/in',types.SimpleNamespace(channels=[1500]*6+[1050]))
        self.assertEqual(s.m.phase,'TAKEOVER')
    def test_readonly_entry_never_constructs_motion_runtime(self):
        r=app.Runtime.__new__(app.Runtime);r.lock=threading.RLock();r.execute=False
        r.report=lambda:{'ready':False,'blockers':['geometry_from_previous_boot']}
        r.request=Mock();r.publish=Mock()
        with patch.object(app.time,'sleep'),patch('builtins.print'):
            self.assertEqual(r.run(),2)
        r.request.assert_not_called();r.publish.assert_not_called()
        self.assertFalse(hasattr(r,'motion'))
    def test_delivered_config_does_not_authorize_flight(self):
        c=json.loads((ROOT/'config/field.json').read_text())
        self.assertFalse(c['control_overrides']['enable_motion'])
        self.assertFalse(c['control_overrides']['enable_traverse'])
        self.assertTrue(all(v is False for v in c['acceptance'].values()))
        self.assertEqual(c['expected_fcu_parameters'],{})
        self.assertIsNone(c['mission']['ground_fcu_height_m'])
    def test_ground_no_target_is_not_airborne_preflight_blocker(self):
        from test_airborne_geometry import DynamicSim
        sim=DynamicSim(False)
        r=self.runtime(sim)
        r.errors=[];r.airborne_geometry=True;r.control=sim.m.core.c
        r.params={k:[0,0.] for k in app.PARAMETERS}
        r.cfg={'mission':sim.options,'acceptance':{k:True for k in app.EVIDENCE},
               'evidence_id':'SYNTHETIC_TEST_ONLY','expected_fcu_parameters':r.params}
        r.cfg['acceptance']['rgbd_geometry_verified']=True
        r.history=[(sim.f.wall,.245)]*60;r.graph_reason='test'
        r.rgbd_row=(sim.f.wall,{'calibration_valid':True,'ready':False,'metric_geometry_ready':False})
        r.depth_row=(sim.f.wall,sim.f.ros)
        report=r.report()
        self.assertTrue(report['ready'],report['blockers'])
        self.assertFalse(report['ground_window_detection_required'])
        self.assertFalse(sim.m.core.live_bound)
    def test_takeoff_preflight_does_not_require_previous_takeoff_or_camera(self):
        from test_takeoff import TakeoffSim
        sim=TakeoffSim();r=self.runtime(sim)
        r.errors=[];r.takeoff_only=True;r.airborne_geometry=False;r.control=sim.m.core.c
        r.params={k:[0,0.] for k in app.PARAMETERS}
        r.cfg={'mission':sim.options,'acceptance':{k:True for k in app.TAKEOFF_EVIDENCE},
               'evidence_id':'SYNTHETIC_TEST_ONLY','expected_fcu_parameters':r.params}
        r.cfg['acceptance'].update(auto_takeoff_land_verified=False,rgbd_geometry_verified=False,postpass_landing_site_verified=False)
        r.history=[(sim.f.wall,.245)]*60;r.graph_reason='test'
        result=r.report()
        self.assertTrue(result['ready'],result['blockers'])
        self.assertEqual(result['experiment'],'takeoff_land')
        self.assertFalse(result['window_traversal_permitted'])
    def test_takeoff_entry_cannot_load_traverse_config(self):
        self.assertEqual(app.verify_experiment({'experiment':'takeoff_land'},'takeoff_land'),'takeoff_land')
        with self.assertRaises(ValueError):app.verify_experiment({'experiment':'traverse'},'takeoff_land')
    def test_parameter_cache_age_and_float_transport(self):
        s=Sim();r=self.runtime(s)
        r.params={k:[0,0.] for k in app.PARAMETERS}
        r.cfg={'expected_fcu_parameters':{k:list(v) for k,v in r.params.items()}}
        r.cfg['expected_fcu_parameters']['MPC_LAND_SPEED']=[0,.3]
        r.params['MPC_LAND_SPEED']=[0,.3000000119]
        self.assertTrue(r.parameters_ok(s.f.wall))
        self.assertFalse(r.parameters_ok(s.f.wall+5.01))
        r.params['MPC_LAND_SPEED']=[0,.7]
        self.assertFalse(r.parameters_ok(s.f.wall))
    def test_parameter_rpc_stall_does_not_block_graph_checks(self):
        from test_takeoff import TakeoffSim
        import time
        sim=TakeoffSim();r=self.runtime(sim)
        entered=threading.Event();release=threading.Event();graph_seen=threading.Event()
        r.stop=threading.Event();r.takeoff_only=True;r.airborne_geometry=False;r.execute=True
        r.control=sim.m.core.c;r.motion=None;r.source_identity=None
        r.params={k:[0,0.] for k in app.PARAMETERS};r.params_wall=time.monotonic()
        r.cfg={'expected_fcu_parameters':r.params};r.graph_ok=False
        pubs=[[topic,['/mavros']] for topic in ('/mavros/state','/mavros/extended_state','/mavros/local_position/odom','/mavros/rc/in','/mavros/estimator_status','/mavros/battery')]
        r.master=types.SimpleNamespace(getSystemState=lambda:[pubs,[],[]])
        def pending(**kw):
            entered.set();release.wait(2)
            return types.SimpleNamespace(success=True,value=types.SimpleNamespace(integer=0,real=0.))
        r.ros=types.SimpleNamespace(ServiceProxy=lambda *a:pending,get_param=lambda *a:False,get_name=lambda:'/auto')
        r.core=types.SimpleNamespace(graph=lambda *a:graph_seen.set())
        srv=types.ModuleType('mavros_msgs.srv');srv.ParamGet=object
        with patch.dict(sys.modules,{'mavros_msgs.srv':srv}):
            pt=threading.Thread(target=r.parameter_worker);gt=threading.Thread(target=r.graph_worker)
            try:
                pt.start();self.assertTrue(entered.wait(1));gt.start()
                self.assertTrue(graph_seen.wait(1));self.assertTrue(r.graph_ok)
            finally:
                r.stop.set();release.set();pt.join(2)
                if gt.ident is not None:gt.join(2)
    def test_invalid_pose_pulse_latches_before_recovery(self):
        from test_takeoff import TakeoffSim
        sim=TakeoffSim();sim.start();sim.until('HOVER');r=self.runtime(sim)
        r.history=[];r.now=lambda:(sim.f.ros,sim.f.wall)
        p=types.SimpleNamespace(x=float('nan'),y=0.,z=1.145)
        q=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.)
        msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda:sim.f.ros),frame_id='map'),child_frame_id='base_link',pose=types.SimpleNamespace(pose=types.SimpleNamespace(position=p,orientation=q)))
        r.pose(msg);sim.step()
        self.assertEqual(sim.m.phase,'FAULT');self.assertIsNone(sim.out['position'])
    def test_20ms_future_pose_tolerance_preserves_other_pose_guards(self):
        from window_pass.core import Controller,Pose
        import copy
        cfg=json.loads((Path(__file__).resolve().parents[1]/'config/full_chain_second_window_return.json').read_text())['control']
        self.assertEqual(cfg['future_tolerance_s'],.02)
        def pose(stamp=100.019,p=(0.,0.,1.),q=(0.,0.,0.,1.)):
            return Pose(stamp,p,q,'map','base_link')
        new=Controller(copy.deepcopy(cfg))
        self.assertTrue(new.pose_update(pose(),100.,10.))
        old=Controller(copy.deepcopy(cfg))
        self.assertFalse(old.pose_update(pose(100.020165205),100.,10.))
        self.assertFalse(new.pose_update(pose(),100.01,10.01)) # duplicate stamp
        self.assertFalse(Controller(copy.deepcopy(cfg)).pose_update(pose(100.031),100.,10.))
        self.assertFalse(Controller(copy.deepcopy(cfg)).pose_update(pose(99.84),100.,10.))
        for bad in (pose(100.04,p=(.36,0.,1.)),pose(100.04,q=(0.,0.,.2,.9797958971132712))):
            core=Controller(copy.deepcopy(cfg));self.assertTrue(core.pose_update(pose(100.),100.,10.))
            self.assertFalse(core.pose_update(bad,100.04,10.04))
        core=Controller(copy.deepcopy(cfg));self.assertTrue(core.pose_update(pose(100.),100.,10.))
        self.assertFalse(core.pose_update(pose(100.01,p=(.03,0.,1.)),100.01,10.01))

    def commissioning_config(self,sim):
        cfg={'experiment':'takeoff_land','mission':sim.options,
             'commissioning_authorization':{'scope':'supervised_takeoff_hover_land_only',
             'operator_accepts_unverified_tests':True,'deferred_checks':sorted(app.COMMISSIONING_DEFERRALS)}}
        return cfg
    def test_commissioning_never_grants_traversal(self):
        from test_takeoff import TakeoffSim
        sim=TakeoffSim();cfg=self.commissioning_config(sim)
        cfg['experiment']='traverse'
        with self.assertRaises(ValueError):app.commissioning_deferrals(cfg,True)
    def test_commissioning_cannot_skip_rc_or_fence(self):
        from test_takeoff import TakeoffSim
        cfg=self.commissioning_config(TakeoffSim())
        cfg['commissioning_authorization']['deferred_checks'].append('takeoff_column_verified')
        with self.assertRaises(ValueError):app.commissioning_deferrals(cfg,True)
    def test_commissioning_limits_cannot_be_increased(self):
        from test_takeoff import TakeoffSim
        for key,value in [('target_bottom_height_m',1.),('climb_speed_mps',.15),('hover_time_s',10.),('landing_radius_m',.3)]:
            cfg=self.commissioning_config(TakeoffSim());cfg['mission']=dict(cfg['mission'],**{key:value})
            with self.subTest(key=key),self.assertRaises(ValueError):app.commissioning_deferrals(cfg,True)
    def test_commissioning_requires_both_declaration_and_explicit_request(self):
        from test_takeoff import TakeoffSim
        cfg=self.commissioning_config(TakeoffSim())
        self.assertFalse(app.commissioning_deferrals(cfg,False))
        self.assertEqual(app.commissioning_deferrals(cfg,True),app.COMMISSIONING_DEFERRALS)
        cfg.pop('commissioning_authorization')
        with self.assertRaises(ValueError):app.commissioning_deferrals(cfg,True)
    def test_commissioning_report_keeps_unverified_flags_and_live_gates(self):
        from test_takeoff import TakeoffSim
        sim=TakeoffSim();r=self.runtime(sim)
        r.errors=[];r.takeoff_only=True;r.airborne_geometry=False;r.control=sim.m.core.c
        r.params={k:[0,0.] for k in app.PARAMETERS};r.cfg=self.commissioning_config(sim)
        r.cfg.update(acceptance={k:k not in app.COMMISSIONING_DEFERRALS for k in app.TAKEOFF_EVIDENCE},
                     evidence_id='SYNTHETIC_ONLY',expected_fcu_parameters=r.params)
        r.deferred_checks=app.commissioning_deferrals(r.cfg,True)
        r.history=[(sim.f.wall,.245)]*60;r.graph_reason='test'
        result=r.report();self.assertTrue(result['ready'],result['blockers'])
        self.assertEqual(set(result['unverified_checks_accepted_for_this_trial']),app.COMMISSIONING_DEFERRALS)
        self.assertFalse(r.cfg['acceptance']['simulation_verified'])
        sim.healthy['rc_ok']=False
        self.assertIn('rc_not_ready',r.report()['blockers'])
        sim.healthy['rc_ok']=True;r.control['enable_motion']=False
        self.assertIn('disabled:enable_motion',r.report()['blockers'])
    def test_arm_denial_preserves_mavlink_result(self):
        s=Sim();s.start();s.until('ARM');s.armed=False;s.update();r=self.runtime(s)
        r.ros.ServiceProxy=lambda *a:lambda **kw:types.SimpleNamespace(success=False,result=1)
        self.request(r,'ARM')
        self.assertEqual(s.m.last_command_feedback['mav_result'],1)
        self.assertFalse(s.m.last_command_feedback['accepted'])
        s.step(respond=False)
        self.assertIn('rejected_mav_result_1',s.m.reason)
    def test_arm_transport_error_is_distinct_from_fcu_rejection(self):
        s=Sim();s.start();s.until('ARM');s.armed=False;s.update();r=self.runtime(s)
        def fail(**kw):raise RuntimeError('transport failed')
        r.ros.ServiceProxy=lambda *a:fail
        self.request(r,'ARM')
        self.assertIsNone(s.m.last_command_feedback['mav_result'])
        self.assertIn('transport failed',s.m.last_command_feedback['error'])
    def test_terminal_unregisters_before_late_diagnostic_wait(self):
        s=Sim();r=self.runtime(s);r.motion=Mock();publisher=r.motion;r.fcu_text=[]
        def wait(seconds):
            self.assertIsNone(r.motion)
            publisher.unregister.assert_called_once()
            received=threading.Event()
            def late():
                with r.lock:r.fcu_text.append({'text':'late prearm reason'})
                received.set()
            t=threading.Thread(target=late);t.start()
            self.assertTrue(received.wait(.5));t.join()
        r.stop=types.SimpleNamespace(wait=wait)
        terminal={'phase':'FAULT'}
        with patch('builtins.print'):self.assertEqual(r.finish_terminal(terminal),3)
        self.assertEqual(terminal['recent_fcu_text'],[{'text':'late prearm reason'}])
    def test_statustext_ignores_other_publishers(self):
        s=Sim();r=self.runtime(s);r.fcu_text=[]
        msg=types.SimpleNamespace(_connection_header={'callerid':'/other'},text='ignored',severity=2,header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda:1.)))
        r.statustext(msg);self.assertFalse(r.fcu_text)
        msg._connection_header={'callerid':'/mavros'};r.statustext(msg)
        self.assertEqual(r.fcu_text[0]['text'],'ignored')
    def test_syntax_python38(self):
        import ast
        for p in (ROOT/'mission.py',ROOT/'live_geometry.py',ROOT/'scripts/auto_window.py',ROOT/'scripts/start_components.py',ROOT/'takeoff.py',ROOT/'field_site.py',ROOT/'scripts/bind_takeoff_site.py'):
            ast.parse(p.read_text(),feature_version=(3,8))


class LandingHandoverTransport(unittest.TestCase):
    runtime=Boundary.runtime
    request=Boundary.request
    def test_scoped_landing_ON_ACK_allows_real_LAND_service_transport(self):
        from test_second_window_return import ReturnSim
        s=ReturnSim();s.start();s.until('LANDING_HANDOVER',limit=4500);r=self.runtime(s)
        self.request(r,'LAND')
        self.assertEqual(r.calls,[('/mavros/set_mode',{'base_mode':0,'custom_mode':'AUTO.LAND'})])
    def test_landing_LAND_transport_requires_ON_ACK(self):
        from test_second_window_return import ReturnSim
        s=ReturnSim();s.start();s.until('LANDING_HANDOVER',limit=4500);s.m.return_journey.on_ack=False;r=self.runtime(s)
        self.request(r,'LAND');self.assertEqual(r.calls,[])
    def test_landing_LAND_transport_requires_active_illumination(self):
        from test_second_window_return import ReturnSim
        s=ReturnSim();s.start();s.until('LANDING_HANDOVER',limit=4500);s.m.return_journey.request_on=False;r=self.runtime(s)
        self.request(r,'LAND');self.assertEqual(r.calls,[])

if __name__=='__main__':unittest.main(verbosity=2)

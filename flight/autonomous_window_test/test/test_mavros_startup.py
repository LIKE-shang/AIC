"""New-process handshake waits; existing running-node guards stay strict."""
import ast
from pathlib import Path
from unittest.mock import patch
from test_ground_snapshot_wait import ScriptedROS,msg
import unittest
from ground_snapshot import await_mavros_startup


class MavrosStartupTest(unittest.TestCase):
    def run_startup(self,events,timeout=.2):
        ros=ScriptedROS(events)
        with patch('ground_snapshot.threading.Condition',return_value=ros), \
                patch('ground_snapshot.time.monotonic',side_effect=lambda:ros.wall):
            try:return await_mavros_startup(ros,object,timeout=timeout)
            finally:self.assertEqual(ros.closed,['/mavros/state'])

    def test_initial_disconnected_then_valid_heartbeat(self):
        latest=msg(0,stamp=10.1)
        self.assertIs(self.run_startup([(.05,0,msg(0,connected=False,system_status=0)),(.1,0,latest)]),latest)

    def test_persistent_disconnect_times_out(self):
        with self.assertRaisesRegex(RuntimeError,'startup connection not established.*connected=False'):
            self.run_startup([(.05,0,msg(0,connected=False,system_status=0))])

    def test_missing_heartbeat_times_out(self):
        with self.assertRaisesRegex(RuntimeError,'/mavros/state: missing'):
            self.run_startup([])

    def test_armed_or_bad_source_never_waits_for_safe_replacement(self):
        for change in (dict(armed=True),dict(caller='/other'),dict(connected=True,system_status=4)):
            with self.subTest(change=change),self.assertRaises(ValueError):
                self.run_startup([(.05,0,msg(0,**change)),(.05,0,msg(0))])

    def test_connected_but_stale_is_not_accepted(self):
        with self.assertRaises(RuntimeError):self.run_startup([(.05,0,msg(0,stamp=1.))])

    def test_startup_wait_only_inside_own_spawn_branch(self):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'scripts/full_chain_ground.py').read_text())
        call=next(n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)
                  and n.func.id=='await_mavros_startup')
        parent=next(n for n in ast.walk(tree) if isinstance(n,ast.If)
                    and any(isinstance(s,ast.Expr) and s.value is call for s in n.body))
        self.assertEqual(ast.dump(parent.test),ast.dump(ast.parse("'/mavros' not in names",mode='eval').body))
        self.assertEqual(ast.literal_eval(call.keywords[0].value),15)


if __name__=='__main__':unittest.main()

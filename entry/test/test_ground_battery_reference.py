"""Execute the actual entry guard with battery invalid and other inputs varied."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest


class GroundBatteryReference(unittest.TestCase):
    def guard(self, **changes):
        tree=ast.parse((Path(__file__).resolve().parents[1]/'competition.py').read_text())
        node=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='ground_guard')
        values=dict(state_fresh=True,landed_fresh=True,landed=True,rc_ok=True,emergency=False,
                    estimator_ok=True,graph_ok=True,battery_ok=False,wall=10.,
                    estimator_reason='',estimator_evidence={})
        values.update(changes)
        i=NS(**values)
        runtime=NS(inputs=lambda:i,core=NS(armed=False),graph_reason='ok',graph_wall=10.)
        import json
        env=dict(runtime=runtime,sources=lambda:None,json=json)
        module=ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[]))
        exec(compile(module,'<actual-ground-guard>','exec'),env)
        return env['ground_guard']()

    def test_battery_invalid_is_reference_only(self):
        self.guard(battery_ok=False)

    def test_localization_and_graph_still_block_and_report_cause(self):
        for change in (dict(estimator_ok=False,estimator_reason='estimator_data_stale_or_invalid_stamp'),
                       dict(graph_ok=False)):
            with self.subTest(change=change),self.assertRaisesRegex(ValueError,'estimator_ok.*graph_reason'):
                self.guard(**change)

    def test_original_state_and_rc_protection_remains(self):
        for key,value in (('state_fresh',False),('landed_fresh',False),('landed',False),
                          ('rc_ok',False),('emergency',True)):
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'ground state/RC changed'):
                self.guard(**{key:value})

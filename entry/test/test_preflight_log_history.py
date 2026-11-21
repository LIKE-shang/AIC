"""Run only a temporary fake CLI to check independent logs. No robot code."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


@unittest.skipIf(os.name=='nt','Bash harness runs on Nano; no Windows Bash required')
class LogHistoryTests(unittest.TestCase):
    def test_failed_then_successful_check_retains_both_attempts(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);entry=root/'entry';entry.mkdir();flight=root/'flight';flight.mkdir()
            (entry/'competition.bash').write_bytes((Path(__file__).resolve().parents[1]/'competition.bash').read_bytes())
            (entry/'verify.py').write_text('print("FAKE_INTEGRITY_OK")\n')
            (flight/'env.bash').write_text('true\n')
            (entry/'competition.py').write_text('import os,sys\nprint("PRECHECK_WAIT fake_test_only")\nprint("PRECHECK_READY" if os.environ["FAKE_OK"]=="1" else "ValueError: FAKE_FAILURE")\nsys.exit(0 if os.environ["FAKE_OK"]=="1" else 2)\n')
            for ok in ('0','1'):
                p=subprocess.run(['bash',str(entry/'competition.bash'),'check'],env=dict(os.environ,FAKE_OK=ok),stdout=subprocess.PIPE,stderr=subprocess.PIPE,universal_newlines=True,timeout=10)
                self.assertEqual(p.returncode,0 if ok=='1' else 2)
                self.assertIn('PRECHECK_WAIT fake_test_only',p.stderr)
            unique=list(entry.glob('preflight_check.*.log'))
            self.assertEqual(len(unique),2)
            self.assertEqual(sum('FAKE_FAILURE' in p.read_text() for p in unique),1)
            self.assertEqual(sum('PRECHECK_READY' in p.read_text() for p in unique),1)
            self.assertIn('PRECHECK_READY',(entry/'preflight_check.log').read_text())


if __name__=='__main__':unittest.main()

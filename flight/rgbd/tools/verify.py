"""Offline verification entry, Python deps come from existing local installation."""
import sys
import runpy
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
local_deps = ROOT.parent/'.vision_test_deps'
# That folder holds a Windows build of numpy/cv2. Putting it on the path on
# Linux makes `import cv2` fail inside numpy's DLL shim, so only use it where
# it can actually load. The Nano and any Linux host use the system packages.
if local_deps.exists() and sys.platform.startswith('win'):
    sys.path.insert(0, str(local_deps))
sys.path.insert(0, str(ROOT/'src'))
if '--projection-benchmark' in sys.argv:
    runpy.run_path(str(ROOT/'tools/benchmark_projection.py'), run_name='__main__')
elif '--rgbd-report' in sys.argv:
    runpy.run_path(str(ROOT/'tools/report_rgbd.py'), run_name='__main__')
elif '--report' in sys.argv:
    runpy.run_path(str(ROOT/'tools/report.py'), run_name='__main__')
elif '--probe' in sys.argv:
    runpy.run_path(str(ROOT/'tools/probe.py'), run_name='__main__')
else:
    import unittest
    import json
    import platform
    import io
    import hashlib
    import cv2
    import numpy as np
    suite=unittest.defaultTestLoader.discover(str(ROOT/'test'))
    buffer=io.StringIO()
    result=unittest.TextTestRunner(stream=buffer,verbosity=2).run(suite)
    text=buffer.getvalue();print(text)
    out=ROOT.parent/'_diagnostics/window_live_rgbd_20261014_v5'
    out.mkdir(parents=True,exist_ok=True)
    (out/'test_output.txt').write_text(text,encoding='utf-8')
    code_hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_text(encoding='utf-8-sig').replace('\r\n', '\n').encode('utf-8')).hexdigest()
        for folder in ('src', 'scripts', 'test', 'launch', 'config', 'tools') for p in (ROOT/folder).rglob('*')
        if p.is_file() and '__pycache__' not in p.parts and p.suffix in ('.py', '.launch', '.json')}
    code_hashes['run_rgbd.bash'] = hashlib.sha256((ROOT/'run_rgbd.bash').read_bytes()).hexdigest()
    (out/'test_results.json').write_text(json.dumps(dict(code_sha256=code_hashes,tests=result.testsRun,errors=len(result.errors),failures=len(result.failures),passed=result.wasSuccessful(),Python=sys.version,OpenCV=cv2.__version__,NumPy=np.__version__,platform=platform.platform(),ROS_runtime_tested=False,Nano_deployed=False,flight_output=False),indent=2),encoding='utf-8')
    sys.exit(0 if result.wasSuccessful() else 1)

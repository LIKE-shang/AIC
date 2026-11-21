"""Offline image probe. No ROS, network, or vehicle access."""
from pathlib import Path
import sys
import json
import cv2
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from window_live_vision.detector import Detector, annotate
OUT = ROOT.parent/'_diagnostics/window_live_20261013'
OUT.mkdir(exist_ok=True, parents=True)
rows = {}
for name, path in [('field', ROOT.parent/'window_pass_test/test/window_checker_field_20261012.jpg'),
                   ('previous', ROOT.parent/'window_pass_test/test/window_checker.png')]:
    frame = cv2.imread(str(path))
    found, result = Detector().detect(frame, debug=True)
    mask = result.pop('texture_mask')
    proposals = result.pop('proposal_quads')
    result['proposals'] = [p.tolist() for p in proposals]
    cv2.imwrite(str(OUT/(name+'_texture.png')), mask)
    overlay = annotate(frame, found)
    cv2.imwrite(str(OUT/(name+'_live.jpg')), overlay)
    for p in proposals:
        cv2.polylines(frame, [p.astype('int32')], True, (255, 0, 0), 1)
    cv2.imwrite(str(OUT/(name+'_proposals.jpg')), frame)
    rows[name] = result
(OUT/'probe.json').write_text(json.dumps(rows, indent=2), encoding='utf-8')
print(json.dumps({k: {x:v[x] for x in ('target_count','processing_ms','proposal_count')} for k,v in rows.items()}, indent=2))

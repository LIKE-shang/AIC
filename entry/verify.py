import hashlib,json
from pathlib import Path
root=Path(__file__).resolve().parent
for name,want in json.loads((root/'manifest.json').read_text()).items():
 if hashlib.sha256((root/name).read_bytes()).hexdigest()!=want:raise ValueError('competition source changed:'+name)
print('Competition entry source manifest verified')

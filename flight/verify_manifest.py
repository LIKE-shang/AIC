#!/usr/bin/env python3
import hashlib,json
from pathlib import Path
root=Path(__file__).resolve().parent
manifest=json.loads((root/'MANIFEST_SHA256.json').read_text())
for relative,digest in manifest.items():
    path=root/relative
    if hashlib.sha256(path.read_bytes()).hexdigest()!=digest:raise SystemExit('SOURCE_CHANGED:'+relative)
print('Full-chain source manifest verified: '+str(len(manifest))+' files')

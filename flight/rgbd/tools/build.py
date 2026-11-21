"""Build deterministic, standalone ROS source package; never uploads or runs ROS."""
import ast
import gzip
import hashlib
import io
import json
from pathlib import Path
import tarfile
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
DEPLOY=ROOT.parent/'_deploy'
REPORT=ROOT.parent/'_diagnostics/window_live_rgbd_20261014_v5'
PREFIX='window_live_rgbd_20261014_v5'


def main():
    results=json.loads((REPORT/'test_results.json').read_text(encoding='utf-8'))
    if not results['passed'] or results['errors'] or results['failures']:
        raise SystemExit('Refusing to package failed tests')
    sources={}
    for path in ROOT.rglob('*'):
        if not path.is_file() or '__pycache__' in path.parts or path.suffix=='.pyc' or path.name in ('MANIFEST.json','OFFLINE_RESULTS.json'):
            continue
        rel=path.relative_to(ROOT).as_posix()
        text=path.read_text(encoding='utf-8-sig').replace('\r\n','\n')
        path.write_bytes(text.encode('utf-8'))
        if path.suffix=='.py':ast.parse(text,filename=rel,feature_version=(3,8))
        if path.suffix in ('.xml','.launch'):ET.fromstring(text)
        sources[rel]=text.encode('utf-8')
    for name, digest in results.get('code_sha256', {}).items():
        if name not in sources or hashlib.sha256(sources[name]).hexdigest() != digest:
            raise SystemExit('Code changed since offline tests: '+name)
    if not results.get('code_sha256'):
        raise SystemExit('Missing code/test provenance')
    results.update(archive_contains_rgbd_perception_and_offline_planner=True,old_flight_tools_reuploaded=False,
                   actual_output_permitted=False,archive_prefix=PREFIX)
    sources['OFFLINE_RESULTS.json']=(json.dumps(results,indent=2)+'\n').encode('utf-8')
    manifest={name:hashlib.sha256(data).hexdigest() for name,data in sorted(sources.items())}
    sources['MANIFEST.json']=(json.dumps(manifest,indent=2)+'\n').encode('utf-8')
    data=io.BytesIO()
    with tarfile.open(fileobj=data,mode='w') as tar:
        for name,payload in sorted(sources.items()):
            item=tarfile.TarInfo(PREFIX+'/'+name)
            item.size=len(payload);item.mtime=0;item.uid=0;item.gid=0
            item.mode=0o755 if name.endswith('.bash') or (name.startswith('scripts/') and name.endswith('.py')) else 0o644
            tar.addfile(item,io.BytesIO(payload))
    payload=gzip.compress(data.getvalue(),mtime=0)
    with tarfile.open(fileobj=io.BytesIO(payload),mode='r:gz') as tar:
        for item in tar.getmembers():
            assert not item.issym() and not item.islnk() and item.name.startswith(PREFIX+'/') and '..' not in item.name.split('/')
            name=item.name[len(PREFIX)+1:]
            assert tar.extractfile(item).read()==sources[name]
            if name.endswith('.bash') or name.startswith('scripts/'):
                assert item.mode & 0o111
    archive=DEPLOY/(PREFIX+'.tar.gz')
    archive.write_bytes(payload)
    sha=hashlib.sha256(payload).hexdigest()
    Path(str(archive)+'.sha256').write_bytes((sha+'  '+archive.name+'\n').encode('ascii'))
    (ROOT/'OFFLINE_RESULTS.json').write_bytes(sources['OFFLINE_RESULTS.json'])
    (ROOT/'MANIFEST.json').write_bytes(sources['MANIFEST.json'])
    (DEPLOY/'WINDOW_LIVE_RGBD_20261014_V5_CN.md').write_bytes(sources['README_CN.md'])
    print(json.dumps(dict(archive=str(archive),bytes=len(payload),sha256=sha,tests=results['tests'],
        archive_verified=True,Nano_deployed=False),indent=2))


if __name__=='__main__':main()

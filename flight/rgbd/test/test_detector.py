import ast
import hashlib
import json
from pathlib import Path
import sys
import time
import unittest
import xml.etree.ElementTree as ET
import cv2
import numpy as np
from scenes import scene
from window_live_vision.detector import Detector, quad_center, polygon_iou
ROOT=Path(__file__).resolve().parents[1]
ART=ROOT.parent/'_diagnostics/window_live_20261013'


class DetectorTests(unittest.TestCase):
    def assert_single(self, image, expected, label):
        found, detail=Detector().detect(image)
        self.assertEqual(len(found),1,(label,detail['reason']))
        self.assertLess(np.linalg.norm(found[0].center-quad_center(expected)), max(8.,.04*np.sqrt(abs(cv2.contourArea(expected)))),label)
        self.assertGreater(polygon_iou(found[0].corners,expected),.80,label)
        return found[0]

    def test_existing_field_images(self):
        for name,expected in [('window_checker_field_20261012.jpg', [[225,146],[475,140],[480,334],[225,338]]),
                              ('window_checker.png', [[160,173],[454,159],[466,400],[164,408]])]:
            with self.subTest(name=name):
                image=cv2.imread(str(ROOT.parent/'window_pass_test/test'/name))
                self.assert_single(image,np.float32(expected),name)

    def test_scale_shape_and_cell_size_are_observed(self):
        for width,height,cell,border in [(170,120,5,20),(300,220,9,30),(440,330,13,35),(170,280,7,26),(390,130,11,22)]:
            with self.subTest(size=(width,height,cell,border)):
                image,q=scene(width,height,cell=cell,border=border)
                self.assert_single(image,q,'synthetic size')

    def test_translation_updates_current_center(self):
        for dx,dy in [(-80,-50),(65,45),(30,-40)]:
            with self.subTest(offset=(dx,dy)):
                image,q=scene(center=(320+dx,240+dy))
                self.assert_single(image,q,'translation')

    def test_perspective(self):
        image,q=scene()
        target=np.float32([[180,135],[475,165],[450,360],[165,350]])
        matrix=cv2.getPerspectiveTransform(q,target)
        warped=cv2.warpPerspective(image,matrix,(640,480),borderValue=(155,155,155))
        self.assert_single(warped,target,'perspective')

    def test_lighting_and_jpeg(self):
        image,q=scene()
        for gain,bias in [(.40,25),(.75,35),(1.0,15)]:
            with self.subTest(gain=gain):
                varied=np.clip(image.astype(float)*gain+bias,0,255).astype(np.uint8)
                ok,encoded=cv2.imencode('.jpg',varied,[cv2.IMWRITE_JPEG_QUALITY,70])
                self.assertTrue(ok)
                self.assert_single(cv2.imdecode(encoded,cv2.IMREAD_COLOR),q,'lighting')

    def test_solid_border_of_unknown_size(self):
        image,q=scene(260,200,solid=True,border=24)
        self.assert_single(image,q,'plain nested frame')

    def test_backgrounds_are_rejected(self):
        yy,xx=np.indices((480,640))
        rng=np.random.RandomState(4)
        field=cv2.imread(str(ROOT.parent/'window_pass_test/test/window_checker_field_20261012.jpg'))
        backgrounds=dict(blank=np.full((480,640,3),140,np.uint8), noise=rng.randint(0,256,(480,640,3)).astype(np.uint8),
            filled_checker=np.repeat((25+((xx//7+yy//7)%2)*200).astype(np.uint8)[:,:,None],3,axis=2),
            curtain=cv2.resize(field[165:310,240:465],(640,480)))
        for label,image in backgrounds.items():
            with self.subTest(label=label):
                self.assertEqual(Detector().detect(image)[0],[],label)

    def test_filled_rectangle_is_not_a_window(self):
        image=np.full((480,640,3),150,np.uint8)
        cv2.rectangle(image,(160,130),(480,350),(25,25,25),-1)
        self.assertEqual(Detector().detect(image)[0],[])

    def test_missing_border_cannot_be_invented(self):
        for side in range(4):
            image,q=scene()
            if side==0: image[98:145,120:510]=155
            elif side==1: image[90:390,462:515]=155
            elif side==2: image[340:390,120:510]=155
            else: image[90:390,120:180]=155
            with self.subTest(side=side):
                self.assertEqual(Detector().detect(image)[0],[])

    def test_multiple_frames_are_reported(self):
        image,_=scene(170,130,center=(160,240),cell=6,border=20)
        other,_=scene(170,130,center=(480,240),cell=6,border=20)
        image[:,320:]=other[:,320:]
        self.assertEqual(len(Detector().detect(image)[0]),2)

    def test_clipped_frame_has_no_target(self):
        image,_=scene(300,220,center=(70,240))
        self.assertEqual(Detector().detect(image)[0],[])

    def test_resolution(self):
        image,q=scene()
        for scale in (.65,1.5):
            with self.subTest(scale=scale):
                resized=cv2.resize(image,None,fx=scale,fy=scale)
                self.assert_single(resized,q*scale,'resolution')

    def test_invalid_arrays(self):
        for image in (None,np.zeros((100,100)),np.zeros((40,60,3),np.uint8)):
            with self.assertRaises(ValueError): Detector().detect(image)

    def test_real_field_variations(self):
        image=cv2.imread(str(ROOT.parent/'window_pass_test/test/window_checker_field_20261012.jpg'))
        base=np.float32([[225,146],[475,140],[480,334],[225,338]])
        for scale,dx,dy,gain in [(.75,-20,0,1.),(.90,30,-10,.65),(1.03,-10,0,1.)]:
            with self.subTest(scale=scale,gain=gain):
                m=np.float32([[scale,0,320*(1-scale)+dx],[0,scale,240*(1-scale)+dy]])
                variant=cv2.warpAffine(image,m,(640,480),borderValue=(130,130,130))
                variant=np.clip(variant.astype(float)*gain,0,255).astype(np.uint8)
                expected=cv2.transform(base.reshape(1,-1,2),m)[0]
                self.assert_single(variant,expected,'real field variation')


class CompetitionResolutionTests(DetectorTests):
    """Run the same scene/false-positive suite at the faster search resolution."""
    def setUp(self):
        from unittest.mock import patch
        original=Detector
        self.override=patch(__name__+'.Detector',side_effect=lambda:original(480))
        self.override.start()
        self.addCleanup(self.override.stop)


class StaticTests(unittest.TestCase):
    def test_python38_and_xml(self):
        for path in ROOT.rglob('*.py'):
            ast.parse(path.read_text(encoding='utf-8-sig'),feature_version=(3,8))
        ET.parse(ROOT/'launch/live.launch'); ET.parse(ROOT/'package.xml')

    def test_frozen_pipeline_unchanged(self):
        for file,digest in [('src/window_pass/vision_frozen.py','c585040f506561888c8cb6098aaf15676ad8a842c9f099e7d690300bf717f658'),
                            ('config/vision_frozen.yaml','254b0a0b5d5d52bf672c59b6c8a6f8ea9631362760aa300ccd564b6ab1d71ff8'),
                            ('scripts/competition_align.py','68210a9cede6f75ef00933a98059cce6519e64dca1e2b0924ef6ab97956e0251')]:
            self.assertEqual(hashlib.sha256((ROOT.parent/'window_pass_test'/file).read_bytes()).hexdigest(),digest)

    def test_no_flight_api_and_no_survey_dependency(self):
        for folder in ('src','scripts'):
            for path in (ROOT/folder).rglob('*.py'):
                text=path.read_text(encoding='utf-8-sig')
                if path.name == 'recover_base.py':
                    # This read-only graph guard names setpoint topics to refuse
                    # active controllers. Permit only that literal deny-list;
                    # it must have no publishing/service APIs of its own.
                    recovery = ast.parse(text)
                    guards = [n for n in recovery.body if isinstance(n, ast.Assign) and
                              any(isinstance(t, ast.Name) and t.id == 'CONTROL_TOPICS' for t in n.targets)]
                    self.assertEqual(len(guards), 1)
                    self.assertTrue(all(t.startswith('/mavros/setpoint') for t in ast.literal_eval(guards[0].value)))
                    calls = [n for n in ast.walk(recovery) if isinstance(n, ast.Call)]
                    self.assertFalse(any(isinstance(n.func, ast.Attribute) and n.func.attr in
                                         ('Publisher','ServiceProxy','publish','set_mode','arm') for n in calls))
                    rows = text.splitlines()
                    text = '\n'.join(rows[:guards[0].lineno-1] + rows[guards[0].end_lineno:])
                for forbidden in ('SetMode(', 'CommandBool(', '/mavros/setpoint', 'window_pass_observe_field', 'survey_opening_size'):
                    self.assertNotIn(forbidden,text,str(path))
        node=ast.parse((ROOT/'scripts/live_node.py').read_text(encoding='utf-8-sig'))
        publishers=[x for x in ast.walk(node) if isinstance(x,ast.Call) and isinstance(x.func,ast.Attribute) and x.func.attr=='Publisher']
        self.assertEqual(len(publishers),1)  # loop over a fixed, non-remappable whitelist
        constants = [x for x in node.body if isinstance(x, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'OUTPUTS' for t in x.targets)]
        outputs = ast.literal_eval(constants[0].value)
        self.assertEqual(len(outputs), 8)
        self.assertTrue(all(t.startswith('/window_live/') for t in outputs))
        services = [x for x in ast.walk(node) if isinstance(x, ast.Call) and isinstance(x.func, ast.Attribute) and x.func.attr == 'ServiceProxy']
        self.assertEqual(len(services), 1)
        self.assertEqual(services[0].args[1].id, 'GetCameraParams')  # read-only factory query
        launch=ET.parse(ROOT/'launch/live.launch').getroot()
        self.assertEqual({n.get('pkg') for n in launch.iter('node')},{'window_live_vision','rosbag'})

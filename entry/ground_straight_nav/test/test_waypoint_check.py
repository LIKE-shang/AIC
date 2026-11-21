import math
import os
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,os.path.join(os.path.dirname(__file__),'..','tools'))
import check_waypoints as wc

class TestMapCheck(unittest.TestCase):
    def pgm(self, data):
        f = tempfile.NamedTemporaryFile(delete=False)
        f.write(data)
        f.close()
        self.addCleanup(os.unlink,f.name)
        return f.name
    def test_truncated_header_rejected(self):
        for data in (b'P5\n#unfinished',b'P5 3',b'P5 3 3 255'):
            with self.assertRaises(ValueError): wc.read_pgm(self.pgm(data))
    def test_crlf_header_does_not_shift_pixels(self):
        self.assertEqual(wc.read_pgm(self.pgm(b'P5\r\n2 1\r\n255\r\n'+bytes([10,254])))[2],bytes([10,254]))
    def test_rotated_map_origin(self):
        p = self.pgm(b'P5\n2 1\n255\n'+bytes([254,0]))
        occ = wc.Occupancy({'resolution':1,'origin':[10,20,math.pi/2]},p)
        self.assertEqual(occ.value(9.5,20.5),occ.FREE)
        self.assertEqual(occ.value(9.5,21.5),occ.OCCUPIED)
    def test_centerline_clear_but_chassis_hits_wall(self):
        pixels = bytearray([254]*100)
        for col in range(10): pixels[3*10+col] = 0
        p = self.pgm(b'P5\n10 10\n255\n'+pixels)
        occ = wc.Occupancy({'resolution':0.1,'origin':[0,0,0]},p)
        self.assertEqual(occ.value(0.5,0.5),occ.FREE)
        self.assertFalse(occ.footprint_clear(0.5,0.5,0.2))
        self.assertTrue(occ.footprint_clear(0.5,0.5,0.03))
    def test_start_and_radius_are_required(self):
        with self.assertRaises(SystemExit) as exc:
            wc.main(['--config','missing.yaml'])
        self.assertEqual(exc.exception.code,2)
    def test_nonpositive_step_refused(self):
        with self.assertRaises(SystemExit):
            wc.main(['--config','missing.yaml','--start-x','0','--start-y','0','--start-yaw','0','--robot-radius','0.3','--step','0'])

if __name__ == '__main__': unittest.main()

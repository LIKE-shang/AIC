from pathlib import Path
import unittest


class ManualGroundModeTests(unittest.TestCase):
    def test_ground_restore_does_not_request_mode_change(self):
        source = (Path(__file__).resolve().parents[1] /
                  'scripts' / 'full_chain_ground.py').read_text()
        self.assertIn('manually select ALTCTL before starting the vision-pose bridge', source)
        self.assertNotIn('mavros_msgs.srv import SetMode', source)
        self.assertNotIn("ServiceProxy('/mavros/set_mode'", source)


if __name__ == '__main__':
    unittest.main()

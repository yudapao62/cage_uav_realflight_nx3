#!/usr/bin/env python3
import importlib.util
import os
import unittest

path = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'realflight_web_supervisor.py')
spec = importlib.util.spec_from_file_location('realflight_web_supervisor', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SupervisorProfileTest(unittest.TestCase):
    def test_missing_or_unknown_profile_is_rejected_before_start(self):
        self.assertEqual((False, 'INPUT_PROFILE_REQUIRED'), module.start_stack(None))
        self.assertEqual((False, 'INPUT_PROFILE_REQUIRED'), module.start_stack('legacy'))

    def test_non_boolean_controls_are_rejected(self):
        self.assertEqual((False, 'FEEDING_AND_RVIZ_MUST_BE_BOOLEAN'),
                         module.start_stack('PHOTO_COORDS', feeding='true'))


if __name__ == '__main__': unittest.main()

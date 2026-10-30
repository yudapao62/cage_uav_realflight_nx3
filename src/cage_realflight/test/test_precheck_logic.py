#!/usr/bin/env python3
import importlib.util
import os
import unittest
from types import SimpleNamespace


path = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'realflight_precheck.py')
spec = importlib.util.spec_from_file_location('realflight_precheck', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class GroundGPSHealthTest(unittest.TestCase):
    def channel(self):
        return module.GroundGPSHealth('A', min_rate_hz=1.5, stale_timeout=1.5,
            stable_window=5.0, min_valid_samples=5, ok_hold=2.0,
            fail_hold=1.0, min_satellites=6)

    @staticmethod
    def fix(seq, valid=True):
        return SimpleNamespace(anchor_id='A', lat=22.0, lon=114.0,
            altitude=5.0, valid=valid, satellites=10, seq=seq)

    def test_requires_advancing_sequence_and_hold(self):
        channel = self.channel()
        for index in range(5): channel.update(self.fix(index), now=1.0+index*0.5)
        self.assertFalse(channel.snapshot(now=3.0)['healthy'])
        for index in range(5, 9): channel.update(self.fix(index), now=1.0+index*0.5)
        self.assertTrue(channel.snapshot(now=5.0)['healthy'])
        count = channel.snapshot(now=5.0)['new_samples_in_window']
        channel.update(self.fix(8), now=5.2)
        self.assertEqual(channel.snapshot(now=5.2)['new_samples_in_window'], count)

    def test_out_of_order_sequence_does_not_refresh_freshness(self):
        channel = self.channel()
        channel.update(self.fix(10), now=1.0)
        channel.update(self.fix(9), now=2.0)
        snapshot = channel.snapshot(now=2.0)
        self.assertEqual(snapshot['seq'], 10)
        self.assertEqual(snapshot['new_samples_in_window'], 1)
        self.assertEqual(snapshot['age_sec'], 1.0)

    def test_soft_failure_is_debounced(self):
        channel = self.channel()
        for index in range(5): channel.update(self.fix(index), now=1.0+index*0.5)
        self.assertFalse(channel.snapshot(now=3.0)['healthy'])
        for index in range(5, 9): channel.update(self.fix(index), now=1.0+index*0.5)
        self.assertTrue(channel.snapshot(now=5.0)['healthy'])
        self.assertTrue(channel.snapshot(now=6.6)['healthy'])
        self.assertFalse(channel.snapshot(now=7.7)['healthy'])

    def test_invalid_flag_is_hard_failure(self):
        channel = self.channel()
        for index in range(5): channel.update(self.fix(index), now=1.0+index*0.5)
        self.assertFalse(channel.snapshot(now=3.0)['healthy'])
        for index in range(5, 9): channel.update(self.fix(index), now=1.0+index*0.5)
        self.assertTrue(channel.snapshot(now=5.0)['healthy'])
        channel.update(self.fix(9, valid=False), now=5.1)
        self.assertFalse(channel.snapshot(now=5.1)['healthy'])


class PrestartModeTest(unittest.TestCase):
    def test_posctl_and_altctl_are_allowed_before_upload(self):
        allowed = ['POSCTL', 'ALTCTL']
        self.assertTrue(module.Precheck._prestart_mode_ok(True, 'POSCTL', allowed))
        self.assertTrue(module.Precheck._prestart_mode_ok(True, 'ALTCTL', allowed))

    def test_auto_modes_and_disconnected_fcu_are_rejected(self):
        allowed = ['POSCTL', 'ALTCTL']
        self.assertFalse(module.Precheck._prestart_mode_ok(True, 'AUTO', allowed))
        self.assertFalse(module.Precheck._prestart_mode_ok(True, 'AUTO.MISSION', allowed))
        self.assertFalse(module.Precheck._prestart_mode_ok(False, 'POSCTL', allowed))


if __name__ == '__main__': unittest.main()

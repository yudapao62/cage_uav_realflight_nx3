#!/usr/bin/env python3
import os
import sys
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.realpath(os.path.join(HERE, '..', 'scripts')))

from web_bridge_node import WebBridge


class FrozenSubmissionTest(unittest.TestCase):
    def test_confirmed_plan_is_copied_without_replanning(self):
        payload = {'task_source': 'GPS_GRID', 'grid_rows': 2, 'grid_cols': 3}
        frozen = [{'seq': 0, 'lat': 27.5, 'lon': 120.5, 'alt': 10.0}]
        result = WebBridge.confirmed_frozen_plan(dict(payload), dict(payload), frozen)
        self.assertEqual(result, frozen)
        self.assertIsNot(result, frozen)
        self.assertIsNot(result[0], frozen[0])
        result[0]['lat'] = 0.0
        self.assertEqual(frozen[0]['lat'], 27.5)

    def test_changed_parameters_require_new_confirmed_preview(self):
        with self.assertRaisesRegex(ValueError, 'CONFIRMED_FROZEN_PREVIEW_REQUIRED'):
            WebBridge.confirmed_frozen_plan(
                {'grid_rows': 3}, {'grid_rows': 2}, [{'seq': 0}])

    def test_empty_frozen_preview_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'CONFIRMED_FROZEN_PREVIEW_REQUIRED'):
            WebBridge.confirmed_frozen_plan({'grid_rows': 2}, {'grid_rows': 2}, [])

    def test_grid_feeding_duration_must_be_shorter_than_hold(self):
        bridge = object.__new__(WebBridge)
        bridge.input_profile = 'GROUND_GPS'
        payload = {'task_id': 'timing-test', 'task_source': 'GPS_GRID',
                   'grid_rows': 3, 'grid_cols': 3,
                   'feeding_dwell_sec': 10.0, 'waypoint_hold_sec': 10.0}
        with mock.patch('web_bridge_node.rospy.Time.now') as now, \
                mock.patch('web_bridge_node.rospy.get_param',
                           side_effect=lambda _name, default: default):
            now.return_value.to_sec.return_value = 1.0
            with self.assertRaisesRegex(ValueError, 'shorter than waypoint hold'):
                bridge.normalized_payload(payload)

    def test_grid_feeding_and_hold_are_frozen_parameters(self):
        bridge = object.__new__(WebBridge)
        bridge.input_profile = 'GROUND_GPS'
        payload = {'task_id': 'timing-test', 'task_source': 'GPS_GRID',
                   'grid_rows': 3, 'grid_cols': 3,
                   'feeding_dwell_sec': 5.0, 'waypoint_hold_sec': 10.0}
        with mock.patch('web_bridge_node.rospy.Time.now') as now, \
                mock.patch('web_bridge_node.rospy.get_param',
                           side_effect=lambda _name, default: default):
            now.return_value.to_sec.return_value = 1.0
            result = bridge.normalized_payload(payload)
        self.assertEqual(result['feeding_dwell_sec'], 5.0)
        self.assertEqual(result['waypoint_hold_sec'], 10.0)

    def test_input_profile_cannot_select_other_source(self):
        bridge = object.__new__(WebBridge)
        bridge.input_profile = 'PHOTO_COORDS'
        with mock.patch('web_bridge_node.rospy.Time.now') as now, \
                mock.patch('web_bridge_node.rospy.get_param',
                           side_effect=lambda _name, default: default):
            now.return_value.to_sec.return_value = 1.0
            with self.assertRaisesRegex(ValueError, 'INPUT_PROFILE_TASK_SOURCE_MISMATCH'):
                bridge.normalized_payload({'task_source': 'GPS_GRID'})

    def test_quick_photo_assets_do_not_require_large_cage(self):
        bridge = object.__new__(WebBridge)
        bridge.input_profile = 'PHOTO_COORDS'
        payload = {'task_id': 'quick-photo', 'task_source': 'PHOTO_POINTS',
                   'photo_asset_ids': ['asset-1', 'asset-2'],
                   'feeding_dwell_sec': 5.0, 'waypoint_hold_sec': 10.0}
        with mock.patch('web_bridge_node.rospy.Time.now') as now, \
                mock.patch('web_bridge_node.rospy.get_param',
                           side_effect=lambda _name, default: default):
            now.return_value.to_sec.return_value = 1.0
            result = bridge.normalized_payload(payload)
        self.assertEqual(['asset-1', 'asset-2'], result['photo_asset_ids'])
        self.assertEqual('', result['large_cage_id'])

    def test_quick_photo_duplicate_asset_is_rejected(self):
        bridge = object.__new__(WebBridge)
        bridge.input_profile = 'PHOTO_COORDS'
        with mock.patch('web_bridge_node.rospy.Time.now') as now, \
                mock.patch('web_bridge_node.rospy.get_param',
                           side_effect=lambda _name, default: default):
            now.return_value.to_sec.return_value = 1.0
            with self.assertRaisesRegex(ValueError, 'duplicate photo assets'):
                bridge.normalized_payload({'task_source': 'PHOTO_POINTS',
                    'photo_asset_ids': ['asset-1', 'asset-1']})


if __name__ == '__main__':
    unittest.main()

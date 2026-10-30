#!/usr/bin/env python3
import threading
import unittest
from cage_msgs.msg import MissionWaypoint
from cage_mavros_adapter.adapter import MavrosAdapter

class AdapterTest(unittest.TestCase):
    def test_set_current_prepares_mission_without_mode_change(self):
        class Result:
            success = True
        adapter = object.__new__(MavrosAdapter)
        adapter.mission_seq_offset = 1
        calls = []
        adapter._set_current = lambda wp_seq: calls.append(wp_seq) or Result()
        self.assertTrue(adapter.set_mission_current())
        self.assertEqual(calls, [1])

    def _upload_adapter(self):
        adapter = object.__new__(MavrosAdapter)
        adapter.mission_home_item_required = False
        adapter.mission_seq_offset = 0
        adapter.feed_point_fcu_hold_sec = 20.0
        adapter.final_cell_rtl_guard_sec = 5.0
        adapter.global_position = None
        adapter.waypoints = []
        adapter._waypoint_event = threading.Event()
        adapter.last_upload_error = ''
        adapter.last_upload_count = 0
        adapter.upload_verified = False
        adapter.set_mission_current = lambda: True
        return adapter

    def test_upload_clears_pushes_pulls_compares_and_selects(self):
        class Result:
            def __init__(self, success=True, count=0):
                self.success = success
                self.wp_transfered = count
                self.wp_received = count
        adapter = self._upload_adapter()
        clears = []
        adapter.clear_mission = lambda: clears.append(True) or True
        captured = []
        adapter._push = lambda start_index, waypoints: (
            captured.extend(waypoints) or Result(True, len(waypoints)))
        def pull():
            adapter.waypoints = list(captured)
            adapter._waypoint_event.set()
            return Result(True, len(captured))
        adapter._pull = pull
        item = MissionWaypoint(seq=0, lat=22.0, lon=114.0, alt=10.0, yaw=0,
                               acceptance_radius=3.0, speed=5.0,
                               waypoint_type='TAKEOFF')
        self.assertTrue(adapter.upload_mission([item], timeout=0.1))
        self.assertEqual(len(clears), 1)
        self.assertEqual(adapter.last_upload_count, 1)
        self.assertTrue(adapter.upload_verified)

    def test_pull_mismatch_clears_partial_mission(self):
        class Result:
            def __init__(self, success=True, transferred=0, received=0):
                self.success = success
                self.wp_transfered = transferred
                self.wp_received = received
        adapter = self._upload_adapter()
        clears = []
        adapter.clear_mission = lambda: clears.append(True) or True
        adapter._push = lambda start_index, waypoints: Result(True, len(waypoints), 0)
        adapter._pull = lambda: Result(True, 0, 0)
        item = MissionWaypoint(seq=0, lat=22.0, lon=114.0, alt=10.0, yaw=0,
                               acceptance_radius=3.0, speed=5.0,
                               waypoint_type='TAKEOFF')
        self.assertFalse(adapter.upload_mission([item], timeout=0.1))
        self.assertEqual(adapter.last_upload_error, 'MISSION_PULL_COUNT_0/1')
        self.assertEqual(len(clears), 2)

    def test_takeoff_conversion_uses_relative_frame(self):
        item = MissionWaypoint(seq=0, lat=22.0, lon=114.0, alt=20.0, yaw=90,
                               acceptance_radius=3, speed=5, waypoint_type='TAKEOFF')
        wp = MavrosAdapter.to_mavros_waypoint(item)
        self.assertEqual(wp.frame, wp.FRAME_GLOBAL_REL_ALT)
        self.assertEqual(wp.command, 22)
        self.assertTrue(wp.is_current)
    def test_feed_point_sets_mavlink_hold_time(self):
        item = MissionWaypoint(seq=2, lat=22.0, lon=114.0, alt=15.0, yaw=0,
                               acceptance_radius=1.5, speed=5, hold_time=5.0,
                               waypoint_type='FEED_POINT')
        wp = MavrosAdapter.to_mavros_waypoint(item)
        self.assertEqual(wp.command,16)
        self.assertEqual(wp.param1,5.0)

    def test_feed_point_uses_independent_fcu_hold_time(self):
        item = MissionWaypoint(seq=2, lat=22.0, lon=114.0, alt=15.0, yaw=0,
                               acceptance_radius=1.5, speed=5, hold_time=10.0,
                               waypoint_type='FEED_POINT')
        wp = MavrosAdapter.to_mavros_waypoint(item, feed_point_fcu_hold_sec=20.0)
        self.assertEqual(wp.param1, 20.0)

    def test_non_feed_waypoint_keeps_task_hold_time(self):
        item = MissionWaypoint(seq=1, lat=22.0, lon=114.0, alt=25.0, yaw=0,
                               acceptance_radius=3.0, speed=5, hold_time=0.0,
                               waypoint_type='TRANSIT')
        wp = MavrosAdapter.to_mavros_waypoint(item, feed_point_fcu_hold_sec=20.0)
        self.assertEqual(wp.param1, 0.0)

    def test_final_cell_hold_has_rtl_guard_extension(self):
        item = MissionWaypoint(seq=11, lat=22.0, lon=114.0, alt=5.0, yaw=0,
                               acceptance_radius=3.0, speed=5.0, hold_time=10.0,
                               waypoint_type='CELL_TARGET', cell_id='R03C03')
        wp = MavrosAdapter.to_mavros_waypoint(
            item, feed_point_fcu_hold_sec=20.0, hold_extension_sec=5.0)
        self.assertEqual(wp.param1, 15.0)

if __name__ == '__main__': unittest.main()

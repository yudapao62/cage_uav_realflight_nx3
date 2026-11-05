#!/usr/bin/env python3
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.realpath(os.path.join(HERE, '..', 'scripts')))

from cage_geo.geodesy import LocalENU
from mission_modes import battery_status, plan_gps_grid, plan_photo_points


class MissionModePlannerTest(unittest.TestCase):
    HOME = (27.5, 120.5, 10.0)

    def test_grid_centers_and_serpentine_order(self):
        local = LocalENU(*self.HOME)
        a = local.to_wgs84(100.0, 20.0, 0.0)
        b = local.to_wgs84(160.0, 40.0, 0.0)
        plan, details = plan_gps_grid(self.HOME, a, b, 2, 3,
                                      10.0, 12.0, 8.0, 5.0)
        cells = [point for point in plan if point['type'] == 'CELL_TARGET']
        self.assertEqual([p['cell_id'] for p in cells],
                         ['R01C01', 'R01C02', 'R01C03',
                          'R02C03', 'R02C02', 'R02C01'])
        first = local.to_enu(cells[0]['lat'], cells[0]['lon'], self.HOME[2])
        self.assertAlmostEqual(first[0], 113.333333, places=3)
        self.assertAlmostEqual(first[1], 21.666667, places=3)
        side = math.hypot(60.0, 20.0) / math.sqrt(2.0)
        self.assertAlmostEqual(details['length_m'], side, places=3)
        self.assertAlmostEqual(details['width_m'], side, places=3)
        self.assertAlmostEqual(details['cell_length_m'], side / 3.0, places=3)
        self.assertAlmostEqual(details['cell_width_m'], side / 2.0, places=3)
        self.assertEqual(details['geometry_definition'],
                         'A_B_OPPOSITE_CORNERS_SQUARE')
        self.assertEqual(len(plan), 9)
        self.assertEqual(plan[-1]['type'], 'CELL_TARGET')
        self.assertNotIn('GRID_EXIT', [point['type'] for point in plan])

    def test_nearly_north_south_diagonal_still_builds_square(self):
        local = LocalENU(*self.HOME)
        a = local.to_wgs84(0.0, 0.0, 0.0)
        b = local.to_wgs84(0.6, 15.4, 0.0)
        plan, details = plan_gps_grid(self.HOME, a, b, 3, 3,
                                      10.0, 12.0, 8.0, 5.0)
        cells = [point for point in plan if point['type'] == 'CELL_TARGET']
        self.assertEqual(len(cells), 9)
        self.assertAlmostEqual(details['length_m'], details['width_m'], places=6)
        self.assertGreater(details['length_m'], 10.0)
        self.assertAlmostEqual(details['cell_length_m'],
                               details['cell_width_m'], places=6)

        # All nine centers form a rotated square, not a thin east/north box.
        enu = [local.to_enu(p['lat'], p['lon'], self.HOME[2]) for p in cells]
        east_span = max(p[0] for p in enu) - min(p[0] for p in enu)
        north_span = max(p[1] for p in enu) - min(p[1] for p in enu)
        self.assertGreater(east_span, 9.0)
        self.assertGreater(north_span, 9.0)

    def test_photo_points_use_xy_and_parameter_altitude(self):
        local = LocalENU(*self.HOME)
        near = local.to_wgs84(20.0, 0.0, 800.0)
        far = local.to_wgs84(80.0, 0.0, -200.0)
        plan, details = plan_photo_points(self.HOME, [
            {'id': 'far', 'label': 'B-02', 'lat': far[0], 'lon': far[1]},
            {'id': 'near', 'label': 'B-01', 'lat': near[0], 'lon': near[1]},
        ], 10.0, 12.0, 7.5, 4.0)
        targets = [point for point in plan if point['type'] == 'CELL_TARGET']
        self.assertEqual([point['cell_id'] for point in targets], ['B-01', 'B-02'])
        self.assertTrue(all(math.isclose(point['alt'], 7.5) for point in targets))
        self.assertEqual(plan[-1]['type'], 'CELL_TARGET')
        self.assertNotIn('GRID_EXIT', [point['type'] for point in plan])
        self.assertTrue(details['photo_altitude_ignored'])
        self.assertEqual(details['altitude_source'], 'WEB_PARAMETER_RELATIVE_ALT')

    def test_grid_rejects_more_than_200_cells(self):
        local = LocalENU(*self.HOME)
        a = local.to_wgs84(10.0, 0.0, 0.0)
        b = local.to_wgs84(110.0, 50.0, 0.0)
        with self.assertRaises(ValueError):
            plan_gps_grid(self.HOME, a, b, 20, 20,
                          10.0, 12.0, 8.0, 5.0)

    def test_manual_database_order_is_preserved(self):
        local = LocalENU(*self.HOME)
        far = local.to_wgs84(80.0, 0.0, 0.0)
        near = local.to_wgs84(20.0, 0.0, 0.0)
        plan, details = plan_photo_points(self.HOME, [
            {'id': 'far', 'label': 'FAR', 'lat': far[0], 'lon': far[1]},
            {'id': 'near', 'label': 'NEAR', 'lat': near[0], 'lon': near[1]},
        ], 10.0, 12.0, 7.5, 4.0, preserve_order=True)
        targets = [point for point in plan if point['type'] == 'CELL_TARGET']
        self.assertEqual(['FAR', 'NEAR'], [point['cell_id'] for point in targets])
        self.assertEqual('MANUAL', details['order_mode'])

    def test_grid_rejects_coincident_diagonal_pair(self):
        local = LocalENU(*self.HOME)
        a = local.to_wgs84(10.0, 0.0, 0.0)
        b = local.to_wgs84(10.2, 0.1, 0.0)
        with self.assertRaises(ValueError):
            plan_gps_grid(self.HOME, a, b, 2, 3,
                          10.0, 12.0, 8.0, 5.0)

    def test_battery_unknown_stale_and_low_are_truthful(self):
        unknown = battery_status(float('nan'), 47.2, 10.0, 10.5, 3.0, 0.2)
        self.assertIsNone(unknown['percentage'])
        self.assertEqual(47.2, unknown['voltage'])
        self.assertTrue(unknown['fresh'])
        stale = battery_status(0.7, 47.2, 10.0, 14.0, 3.0, 0.2)
        self.assertFalse(stale['fresh'])
        low = battery_status(0.2, 44.0, 10.0, 10.1, 3.0, 0.2)
        self.assertTrue(low['low'])


if __name__ == '__main__':
    unittest.main()

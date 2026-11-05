#!/usr/bin/env python3
import json
import os
import threading
import tempfile
import urllib.request
import unittest

from http.server import ThreadingHTTPServer

os.environ.setdefault('CAGE_SITE_DB_DATA', tempfile.mkdtemp(prefix='cage_site_api_test_'))

from cage_site_db.db import init_db, row, transaction
from cage_site_db.service import Handler


class ServiceApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        init_db(); cls.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        cls.base = 'http://127.0.0.1:%d/api/site-db' % cls.server.server_address[1]
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True); cls.thread.start()

    @classmethod
    def tearDownClass(cls): cls.server.shutdown(); cls.server.server_close()

    def post(self, path, payload):
        request = urllib.request.Request(self.base+path, data=json.dumps(payload).encode('utf-8'),
            headers={'Content-Type': 'application/json'}, method='POST')
        with urllib.request.urlopen(request) as response:
            return json.loads(response.read().decode('utf-8'))

    def get(self, path):
        with urllib.request.urlopen(self.base+path) as response:
            return json.loads(response.read().decode('utf-8'))

    def test_bulk_manual_approve_and_mission_points(self):
        site = self.post('/sites', {'name': 'api-large-cage'})['id']
        bulk = self.post('/subcages/bulk', {'site_id': site, 'prefix': 'Z-',
            'start': 1, 'count': 200, 'padding': 3})
        self.assertEqual(200, bulk['count'])
        cage = bulk['created'][0]['id']
        candidate = self.post('/points/manual-candidate', {'site_id': site,
            'subcage_id': cage, 'reviewer': 'tester', 'lat': 27.58,
            'lon': 120.56, 'note': 'verified'})['candidate_point_id']
        approved = self.post('/points/'+candidate+'/approve',
                             {'reviewer': 'tester', 'review_note': 'approved'})
        self.assertTrue(approved['coordinate_approved'])
        points = self.get('/large-cages/'+site+'/mission-points')['points']
        self.assertEqual(1, len(points))
        self.assertEqual('MANUAL_WGS84', points[0]['source_method'])
        versions = self.get('/subcages/'+cage+'/point-versions')['versions']
        self.assertEqual(1, versions[0]['version'])
        params = {'takeoff_relative_alt': 10.0, 'transit_relative_alt': 8.0,
                  'feed_relative_alt': 5.0, 'feeding_dwell_sec': 5.0,
                  'waypoint_hold_sec': 10.0, 'speed': 5.0,
                  'acceptance_radius': 3.0}
        snapshot = self.post('/mission-snapshots', {'mission_id': 'api-mission',
            'input_profile': 'PHOTO_COORDS', 'site_id': site,
            'home': {'lat': 27.57, 'lon': 120.55, 'alt': 12.0},
            'parameters': params, 'preview_hash': 'hash-1',
            'targets': [{'subcage_id': cage, 'approved_version_id': approved['id'],
                         'label': 'Z-001', 'lat': 27.58, 'lon': 120.56}]})
        self.assertTrue(snapshot['id'].startswith('snapshot_'))
        self.post('/mission-snapshots/api-mission/state',
                  {'state': 'RTL_CONFIRMED', 'details': {'attempts': 1}})
        with transaction() as conn:
            saved = row(conn.execute("SELECT state,home_lat FROM mission_snapshots WHERE mission_id='api-mission'"))
        self.assertEqual('RTL_CONFIRMED', saved['state'])
        self.assertAlmostEqual(27.57, saved['home_lat'])


if __name__ == '__main__': unittest.main(verbosity=2)

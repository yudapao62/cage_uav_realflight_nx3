#!/usr/bin/env python3
import glob
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rospy
from mavros_msgs.msg import State, WaypointList
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_msgs.msg import Empty, Float64, Header, String
from cage_msgs.msg import (CageGeometry, FeedingTaskRequest, FeedingTaskStatus,
                           MissionRequest, MissionState, MissionWaypoint,
                           MissionWaypointArray)

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from mission_modes import TASK_SOURCES, battery_status, plan_gps_grid, plan_photo_points


class BagRecorder:
    TOPICS = [
        '/mission/state', '/mission/waypoints', '/mission/direct_request',
        '/feeding/task_status',
        '/feeder/command', '/feeder/status', '/cage/geometry',
        '/cage/beacon/status', '/cage/realflight/precheck',
        '/farm/scene_1/anchor_A/fix', '/farm/scene_1/anchor_B/fix',
        '/mavros/state', '/mavros/global_position/global',
        '/mavros/global_position/rel_alt', '/mavros/mission/waypoints',
        '/mavros/mission/reached', '/mavros/rc/in', '/mavros/battery'
    ]

    def __init__(self):
        self.root = rospy.get_param('/web_bridge/bag_root', '/home/nx3/cage_uav_ws/bags')
        self.process = None
        self.directory = ''
        self.started_at = 0.0
        self.error = ''
        self.log_handle = None
        self.lock = threading.RLock()

    def _alive(self):
        return self.process is not None and self.process.poll() is None

    def _files(self):
        return sorted(glob.glob(os.path.join(self.directory, '*.bag*'))) if self.directory else []

    def status(self):
        with self.lock:
            alive = self._alive()
            files = self._files()
            size = sum(os.path.getsize(path) for path in files if os.path.isfile(path))
            return {'active': alive, 'directory': self.directory, 'files': files,
                    'pid': self.process.pid if alive else None,
                    'duration_sec': max(0.0, time.time()-self.started_at) if self.started_at else 0.0,
                    'size_bytes': size, 'error': self.error}

    def start(self, snapshot):
        with self.lock:
            if self._alive():
                return False, 'ROSBAG_ALREADY_RECORDING'
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            self.directory = os.path.join(self.root, 'realflight_'+stamp)
            os.makedirs(self.directory, exist_ok=False)
            with open(os.path.join(self.directory, 'status_before.json'), 'w') as handle:
                json.dump(snapshot, handle, indent=2, ensure_ascii=False)
            try:
                subprocess.run(['rosparam', 'dump', os.path.join(self.directory, 'rosparams.yaml')], check=True, timeout=10)
                self.log_handle = open(os.path.join(self.directory, 'rosbag.log'), 'ab', buffering=0)
                command = ['rosbag', 'record', '--lz4', '-O', os.path.join(self.directory, 'realflight.bag')] + self.TOPICS
                self.process = subprocess.Popen(command, stdout=self.log_handle,
                                                stderr=subprocess.STDOUT, start_new_session=True)
                self.started_at = time.time()
                self.error = ''
            except Exception as exc:
                self.error = str(exc)
                return False, self.error
        deadline = time.time() + 4.0
        while time.time() < deadline:
            if not self._alive():
                self.error = 'ROSBAG_PROCESS_EXITED'
                return False, self.error
            if self._files():
                return True, ''
            time.sleep(0.1)
        self.error = 'ROSBAG_FILE_NOT_CREATED'
        self.stop({}, force=True)
        return False, self.error

    def stop(self, snapshot, force=False):
        with self.lock:
            if not self._alive():
                return False, 'ROSBAG_NOT_RECORDING'
            process = self.process
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
            except ProcessLookupError:
                pass
            with open(os.path.join(self.directory, 'status_after.json'), 'w') as handle:
                json.dump(snapshot, handle, indent=2, ensure_ascii=False)
            if self.log_handle:
                self.log_handle.close()
                self.log_handle = None
            self.process = None
            return True, ''

    def recent(self):
        os.makedirs(self.root, exist_ok=True)
        return sorted([path for path in glob.glob(os.path.join(self.root, 'realflight_*'))
                       if os.path.isdir(path)], reverse=True)[:10]


class WebBridge:
    COMMON_CONFIRMATION_KEYS = ('takeoff_area_clear', 'h12_takeover_verified',
                                'pilot_ready', 'site_coordinates_verified')

    def __init__(self):
        self.geometry_mode = str(rospy.get_param('/cage/geometry_mode', 'rectangular')).strip().lower()
        if self.geometry_mode not in ('rectangular', 'circular'):
            raise ValueError('invalid /cage/geometry_mode: {}'.format(self.geometry_mode))
        self.status = FeedingTaskStatus(task_state='TASK_IDLE')
        self.input_profile = str(rospy.get_param('/realflight/input_profile', '')).upper()
        if self.input_profile not in ('GROUND_GPS', 'PHOTO_COORDS'):
            raise ValueError('invalid /realflight/input_profile')
        self.mission = MissionState(state='WAIT_FCU')
        self.fcu = State()
        self.uav_gps = None
        self.battery = None; self.battery_received_at = 0.0
        self.return_control = {'state': 'IDLE', 'reason': '', 'requested_at': None, 'attempts': 0}
        self.relative_alt = None
        self.beacon = {}; self.precheck = {}; self.geometry = None
        self.waypoints = None
        self.preview_payload = None
        self.preview_confirmed = False
        self.preview_pending = False
        self.preview_details = {}
        self.direct_plan = []
        self.frozen_direct_plan = []
        self.frozen_preview_payload = None
        self.fcu_waypoints = WaypointList()
        self.submission_lock = threading.RLock()
        self.submission = {'state': 'IDLE', 'mission_id': '', 'error': '',
                           'expected_fcu_waypoints': 0, 'requested_at': 0.0}
        self.direct_plan_pub = rospy.Publisher('/mission/waypoints', MissionWaypointArray, queue_size=1, latch=True)
        # Direct grid/photo missions must bypass waypoint_planner_node.  That
        # node owns /mission/request and always generates the circular 3-point
        # plan, which would otherwise race the frozen direct plan.
        self.direct_request_pub = rospy.Publisher('/mission/direct_request', MissionRequest, queue_size=1)
        self.bag = BagRecorder()
        self.preview_pub = rospy.Publisher('/feeding/task_preview', FeedingTaskRequest, queue_size=1)
        self.start_pub = rospy.Publisher('/feeding/task_request', FeedingTaskRequest, queue_size=1)
        self.cancel_pub = rospy.Publisher('/feeding/task_cancel', Empty, queue_size=1)
        self.return_pub = rospy.Publisher('/mission/return_now', Empty, queue_size=1)
        rospy.Subscriber('/feeding/task_status', FeedingTaskStatus, lambda m: setattr(self, 'status', m), queue_size=1)
        rospy.Subscriber('/mission/state', MissionState, self.on_mission, queue_size=1)
        rospy.Subscriber('/mission/waypoints', MissionWaypointArray, self.on_waypoints, queue_size=1)
        rospy.Subscriber('/mavros/state', State, lambda m: setattr(self, 'fcu', m), queue_size=1)
        rospy.Subscriber('/mavros/mission/waypoints', WaypointList,
                         lambda m: setattr(self, 'fcu_waypoints', m), queue_size=1)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, lambda m: setattr(self, 'uav_gps', m), queue_size=1)
        rospy.Subscriber('/mavros/global_position/rel_alt', Float64, lambda m: setattr(self, 'relative_alt', m.data), queue_size=1)
        rospy.Subscriber('/mavros/battery', BatteryState, self.on_battery, queue_size=1)
        rospy.Subscriber('/mission/return_status', String, self.on_return_status, queue_size=1)
        rospy.Subscriber('/cage/beacon/status', String, self.on_beacon, queue_size=1)
        rospy.Subscriber('/cage/realflight/precheck', String, self.on_precheck, queue_size=1)
        rospy.Subscriber('/cage/geometry', CageGeometry, lambda m: setattr(self, 'geometry', m), queue_size=1)

    def on_battery(self, msg):
        self.battery = msg; self.battery_received_at = time.time()

    def on_return_status(self, msg):
        try: self.return_control = json.loads(msg.data)
        except ValueError: self.return_control = {'state': 'FAILED', 'reason': 'RETURN_STATUS_INVALID', 'requested_at': None, 'attempts': 0}

    def on_beacon(self, msg):
        try: self.beacon = json.loads(msg.data)
        except ValueError: self.beacon = {}

    def on_precheck(self, msg):
        try: self.precheck = json.loads(msg.data)
        except ValueError: self.precheck = {}

    def on_mission(self, msg):
        changed = msg.state != self.mission.state or msg.reason != self.mission.reason
        self.mission = msg
        if changed and self.input_profile == 'PHOTO_COORDS' and self.frozen_preview_payload:
            threading.Thread(target=self.update_snapshot,
                args=(msg.state, {'reason': msg.reason, 'current_waypoint': msg.current_waypoint,
                                  'current_cell_id': msg.current_cell_id,
                                  'battery': self.battery_json()}), daemon=True).start()

    def on_waypoints(self, msg):
        self.waypoints = msg
        if msg.mission_id == 'PREVIEW' and msg.waypoints:
            self.preview_pending = False

    def reset_submission(self):
        with self.submission_lock:
            self.submission = {'state': 'IDLE', 'mission_id': '', 'error': '',
                               'expected_fcu_waypoints': 0, 'requested_at': 0.0}

    def fail_submission(self, error):
        with self.submission_lock:
            self.submission['state'] = 'FAILED'
            self.submission['error'] = str(error)

    def begin_submission(self, mission_id, expected_fcu_waypoints=0):
        with self.submission_lock:
            self.submission = {'state': 'REQUESTED', 'mission_id': mission_id,
                               'error': '',
                               'expected_fcu_waypoints': int(expected_fcu_waypoints),
                               'requested_at': time.time()}

    def freeze_confirmed_preview(self):
        if self.preview_payload is None or self.waypoints is None or not self.waypoints.waypoints:
            raise ValueError('PREVIEW_NOT_AVAILABLE')
        self.frozen_preview_payload = dict(self.preview_payload)
        source = self.preview_payload['task_source']
        if source in ('GPS_GRID', 'PHOTO_POINTS'):
            if not self.direct_plan:
                raise ValueError('DIRECT_PREVIEW_NOT_AVAILABLE')
            self.frozen_direct_plan = [dict(item) for item in self.direct_plan]
        else:
            self.frozen_direct_plan = []

    def submission_status(self):
        with self.submission_lock:
            state = self.submission['state']
            expected = int(self.submission.get('expected_fcu_waypoints', 0))
            actual = len(self.fcu_waypoints.waypoints)
            if state == 'REQUESTED':
                if self.mission.state in ('ERROR', 'FAILSAFE', 'MANUAL_OVERRIDE'):
                    self.submission['state'] = 'FAILED'
                    self.submission['error'] = self.mission.reason or self.mission.state
                elif self.mission.state == 'WAIT_ARM':
                    if expected <= 0 and self.waypoints is not None and self.waypoints.waypoints:
                        expected = len(self.waypoints.waypoints) + (
                            1 if rospy.get_param('/vehicle/mission_home_item_required', True) else 0)
                        self.submission['expected_fcu_waypoints'] = expected
                    if expected > 0 and actual == expected:
                        self.submission['state'] = 'VERIFIED'
                    # A stale WaypointList may arrive between clear/push/pull.
                    # Keep waiting for the adapter-verified list; the bounded
                    # submission timeout turns a persistent mismatch into a
                    # hard failure without creating a callback-order race.
            result = dict(self.submission)
        result['fcu_waypoint_count'] = len(self.fcu_waypoints.waypoints)
        result['upload_verified'] = result['state'] == 'VERIFIED'
        result['unlock_ready'] = bool(result['upload_verified'] and
                                      self.mission.state == 'WAIT_ARM' and
                                      result['fcu_waypoint_count'] == result['expected_fcu_waypoints'])
        result['auto_switch_allowed'] = result['unlock_ready']
        result['manual_auto_switch'] = True
        result['unsafe_auto_mode'] = bool(self.fcu.mode == 'AUTO.MISSION' and
                                          self.mission.state not in
                                          ('TAKEOFF', 'TRANSIT', 'FEEDING',
                                           'FEEDING_DWELL', 'EXIT', 'RTL', 'LAND'))
        return result

    def wait_for_submission(self):
        timeout = float(rospy.get_param('/web_bridge/submission_timeout_sec', 30.0))
        deadline = time.monotonic() + max(5.0, timeout)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            status = self.submission_status()
            if status['state'] == 'VERIFIED':
                return True, status
            if status['state'] == 'FAILED':
                return False, status
            time.sleep(0.1)
        self.fail_submission('MISSION_UPLOAD_VERIFICATION_TIMEOUT')
        return False, self.submission_status()

    def normalized_payload(self, payload):
        defaults = {'task_id': 'web-task-{}'.format(int(rospy.Time.now().to_sec())),
            'cage_id': 0, 'repeat_count': 1,
            'task_source': 'GPS_CENTER',
            'takeoff_relative_alt': rospy.get_param('/vehicle/takeoff_relative_alt', 10.0),
            'transit_relative_alt': rospy.get_param('/feeding/transit_relative_alt', 8.0),
            'feed_relative_alt': rospy.get_param('/feeding/feed_relative_alt', 5.0),
            'feeding_dwell_sec': rospy.get_param('/cage/circular/feeding_dwell_sec', 10.0),
            'waypoint_hold_sec': rospy.get_param('/mission/grid_waypoint_hold_sec', 10.0),
            'grid_rows': 6, 'grid_cols': 10, 'large_cage_id': '',
            'route_order_mode': 'NEAREST', 'photo_point_ids': [],
            'speed': rospy.get_param('/mission/speed', 5.0),
            'acceptance_radius': rospy.get_param('/mission/waypoint_acceptance_radius', 3.0),
            'max_leg_distance': rospy.get_param('/safety/max_leg_distance', 5000.0)}
        out = {key: payload.get(key, value) for key, value in defaults.items()}
        out['task_id'] = str(out['task_id']).strip()[:64]
        out['task_source'] = str(out['task_source']).strip().upper()
        out['cage_id'] = int(out['cage_id']); out['repeat_count'] = int(out['repeat_count'])
        for key in ('takeoff_relative_alt', 'transit_relative_alt', 'feed_relative_alt',
                    'feeding_dwell_sec', 'waypoint_hold_sec', 'speed',
                    'acceptance_radius', 'max_leg_distance'):
            out[key] = float(out[key])
        if not out['task_id'] or not 1 <= out['repeat_count'] <= 20:
            raise ValueError('invalid task id or repeat count')
        if out['task_source'] not in TASK_SOURCES:
            raise ValueError('task_source must be GPS_CENTER, GPS_GRID or PHOTO_POINTS')
        compatible = ((self.input_profile == 'GROUND_GPS' and out['task_source'] in ('GPS_CENTER','GPS_GRID')) or
                      (self.input_profile == 'PHOTO_COORDS' and out['task_source'] == 'PHOTO_POINTS'))
        if not compatible:
            raise ValueError('INPUT_PROFILE_TASK_SOURCE_MISMATCH')
        for key in ('takeoff_relative_alt', 'transit_relative_alt', 'feed_relative_alt'):
            if not 3.0 <= out[key] <= 120.0: raise ValueError(key+' must be 3..120 m')
        if not 1.0 <= out['feeding_dwell_sec'] <= 120.0:
            raise ValueError('feeding_dwell_sec must be 1..120 s')
        if not 2.0 <= out['waypoint_hold_sec'] <= 300.0:
            raise ValueError('waypoint_hold_sec must be 2..300 s')
        if not 0.5 <= out['speed'] <= 20.0: raise ValueError('speed must be 0.5..20 m/s')
        if not 0.5 <= out['acceptance_radius'] <= 30.0: raise ValueError('acceptance_radius must be 0.5..30 m')
        if not 10.0 <= out['max_leg_distance'] <= 20000.0: raise ValueError('max_leg_distance must be 10..20000 m')
        if (out['task_source'] in ('GPS_GRID', 'PHOTO_POINTS') and
                out['feeding_dwell_sec'] >= out['waypoint_hold_sec']):
            raise ValueError('feeding duration must be shorter than waypoint hold time')
        if out['task_source'] == 'GPS_GRID':
            out['grid_rows'] = int(out['grid_rows']); out['grid_cols'] = int(out['grid_cols'])
            if not (1 <= out['grid_rows'] <= 50 and 1 <= out['grid_cols'] <= 50 and
                    out['grid_rows']*out['grid_cols'] <= 200):
                raise ValueError('grid rows*cols must be 1..200')
        ids = out.get('photo_point_ids', [])
        if not isinstance(ids, list):
            raise ValueError('photo_point_ids must be a list')
        out['photo_point_ids'] = [str(value) for value in ids]
        out['large_cage_id'] = str(out.get('large_cage_id') or '')
        out['route_order_mode'] = str(out.get('route_order_mode') or 'NEAREST').upper()
        if out['route_order_mode'] not in ('NEAREST','MANUAL'):
            raise ValueError('route_order_mode must be NEAREST or MANUAL')
        if out['task_source'] == 'PHOTO_POINTS':
            if not out['large_cage_id']: raise ValueError('select one large cage')
            if not out['photo_point_ids']: raise ValueError('select at least one approved database point')
        return out

    def make_request(self, payload, auto_start):
        data = self.normalized_payload(payload)
        mode = 'CENTER_FEED' if self.geometry_mode == 'circular' else str(payload.get('mode', 'FULL_GRID')).upper()
        return FeedingTaskRequest(header=Header(stamp=rospy.Time.now()), task_id=data['task_id'],
            cage_id=data['cage_id'], mode=mode,
            rows=[] if self.geometry_mode == 'circular' else [int(v) for v in payload.get('rows', [])],
            cols=[] if self.geometry_mode == 'circular' else [int(v) for v in payload.get('cols', [])],
            selected_cell_ids=[] if self.geometry_mode == 'circular' else [str(v).upper() for v in payload.get('selected_cell_ids', [])],
            repeat_count=data['repeat_count'], auto_start=bool(auto_start),
            feed_relative_alt=data['feed_relative_alt'], transit_relative_alt=data['transit_relative_alt'],
            takeoff_relative_alt=data['takeoff_relative_alt'], feeding_dwell_sec=data['feeding_dwell_sec'])

    @staticmethod
    def _json_url(url, timeout=3.0):
        with urllib.request.urlopen(url, timeout=timeout) as response:
            return json.loads(response.read().decode('utf-8'))

    @staticmethod
    def _post_json(url, payload, timeout=3.0):
        request = urllib.request.Request(url, data=json.dumps(payload).encode('utf-8'),
                                         headers={'Content-Type': 'application/json'}, method='POST')
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode('utf-8'))

    def large_cages(self):
        try:
            return [item for item in self._json_url('http://127.0.0.1:8090/api/site-db/sites')
                    if item.get('status') == 'ACTIVE']
        except Exception as exc:
            rospy.logwarn_throttle(10.0, 'Large cage database unavailable: %s', exc)
            return []

    def approved_photo_points(self):
        try:
            data = self._json_url('http://127.0.0.1:8090/api/site-db/map')
            points = data.get('approved', [])
            return [{'id': str(point['id']), 'label': str(point.get('label') or point['id']),
                     'lat': float(point['lat']), 'lon': float(point['lon']),
                     'site_id': str(point.get('site_id') or ''),
                     'subcage_id': str(point.get('subcage_id') or ''),
                     'source_method': str(point.get('source_method') or ''),
                     'version': int(point.get('version', 0)),
                     'approved_at': point.get('approved_at', '')}
                    for point in points if point.get('active', 1)]
        except Exception as exc:
            rospy.logwarn_throttle(10.0, 'Photo point database unavailable: %s', exc)
            return []

    def save_snapshot(self):
        if self.input_profile != 'PHOTO_COORDS':
            return None
        data = self.frozen_preview_payload
        points = {item['id']: item for item in self.approved_photo_points()}
        execution_ids = [item['approved_version_id'] for item in self.preview_details.get('order', [])]
        selected = [points[item] for item in execution_ids if item in points]
        if len(selected) != len(data['photo_point_ids']):
            raise ValueError('APPROVED_POINT_VERSION_CHANGED')
        home = self.preview_details['home']
        digest = hashlib.sha256(json.dumps({'payload': data, 'plan': self.frozen_direct_plan},
            sort_keys=True, separators=(',', ':')).encode('utf-8')).hexdigest()
        return self._post_json('http://127.0.0.1:8090/api/site-db/mission-snapshots', {
            'mission_id': data['task_id'], 'input_profile': self.input_profile,
            'site_id': data['large_cage_id'],
            'home': home,
            'parameters': data, 'preview_hash': digest, 'state': 'PREVIEW_CONFIRMED',
            'details': self.preview_details,
            'targets': [{'subcage_id': point['subcage_id'], 'approved_version_id': point['id'],
                         'label': point['label'], 'lat': point['lat'], 'lon': point['lon']}
                        for point in selected]})

    def verify_frozen_versions(self):
        if self.input_profile != 'PHOTO_COORDS': return True
        current = {item['id']: item for item in self.approved_photo_points()}
        return all(point_id in current and current[point_id]['site_id'] == self.frozen_preview_payload['large_cage_id']
                   for point_id in self.frozen_preview_payload['photo_point_ids'])

    def update_snapshot(self, state, details=None):
        if self.input_profile != 'PHOTO_COORDS' or not self.frozen_preview_payload:
            return
        mission_id = urllib.parse.quote(self.frozen_preview_payload['task_id'], safe='')
        try:
            self._post_json('http://127.0.0.1:8090/api/site-db/mission-snapshots/{}/state'.format(mission_id),
                            {'state': state, 'details': details or {}})
        except Exception as exc:
            rospy.logwarn('Mission snapshot state update failed: %s', exc)

    def _home(self):
        if (self.uav_gps is None or self.uav_gps.status.status < NavSatStatus.STATUS_FIX or
                not all(math.isfinite(v) for v in (self.uav_gps.latitude,
                                                   self.uav_gps.longitude,
                                                   self.uav_gps.altitude))):
            raise ValueError('UAV_GPS_NOT_VALID')
        return (self.uav_gps.latitude, self.uav_gps.longitude, self.uav_gps.altitude)

    def build_direct_plan(self, data):
        home = self._home()
        common = (data['takeoff_relative_alt'], data['transit_relative_alt'],
                  data['feed_relative_alt'], data['waypoint_hold_sec'],
                  data['acceptance_radius'], data['speed'], data['max_leg_distance'])
        if data['task_source'] == 'GPS_GRID':
            if self.geometry is None or not self.beacon.get('geometry_valid', False):
                raise ValueError('BEACON_GEOMETRY_INVALID')
            plan, details = plan_gps_grid(home,
                (self.geometry.endpoint_a_lat, self.geometry.endpoint_a_lon, self.geometry.endpoint_a_alt),
                (self.geometry.endpoint_b_lat, self.geometry.endpoint_b_lon, self.geometry.endpoint_b_alt),
                data['grid_rows'], data['grid_cols'], *common)
        elif data['task_source'] == 'PHOTO_POINTS':
            approved = {point['id']: point for point in self.approved_photo_points()
                        if point['site_id'] == data['large_cage_id']}
            missing = [point_id for point_id in data['photo_point_ids'] if point_id not in approved]
            if missing:
                raise ValueError('APPROVED_PHOTO_POINTS_REQUIRED:'+','.join(missing))
            selected = [approved[point_id] for point_id in data['photo_point_ids']]
            plan, details = plan_photo_points(home, selected, *common,
                preserve_order=data['route_order_mode'] == 'MANUAL')
            details['large_cage_id'] = data['large_cage_id']
            details['selected_versions'] = [{'id': p['id'], 'subcage_id': p['subcage_id'],
                                              'version': p['version']} for p in selected]
        else:
            raise ValueError('direct planner only supports GPS_GRID and PHOTO_POINTS')
        if data['repeat_count'] > 1:
            cells = [dict(item) for item in plan if item['type'] == 'CELL_TARGET']
            if len(cells)*data['repeat_count'] > 200:
                raise ValueError('repeated target count exceeds 200')
            repeated = [dict(item) for item in plan[:3]]
            for repeat_index in range(1, data['repeat_count'] + 1):
                for item in cells:
                    copy = dict(item)
                    copy['cell_id'] = '{}@R{}'.format(copy['cell_id'], repeat_index)
                    repeated.append(copy)
            for seq, item in enumerate(repeated):
                item['seq'] = seq
            plan = repeated
            details['repeat_count'] = data['repeat_count']
            details['target_count_per_repeat'] = len(cells)
            details['target_count'] = len(cells)*data['repeat_count']
        details['waypoint_hold_sec'] = data['waypoint_hold_sec']
        details['feeding_duration_sec'] = data['feeding_dwell_sec']
        details['home'] = {'lat': home[0], 'lon': home[1], 'alt': home[2]}
        return plan, details

    @staticmethod
    def ros_waypoints(plan, mission_id, cage_id):
        message = MissionWaypointArray(header=Header(stamp=rospy.Time.now()),
                                       mission_id=mission_id, cage_id=cage_id)
        message.waypoints = [MissionWaypoint(seq=item['seq'], lat=item['lat'], lon=item['lon'],
            alt=item['alt'], yaw=item['yaw'], acceptance_radius=item['acceptance_radius'],
            speed=item['speed'], hold_time=item['hold_time'], waypoint_type=item['type'],
            cell_id=item.get('cell_id', '')) for item in plan]
        return message

    def publish_direct_preview(self, data):
        self.direct_plan, self.preview_details = self.build_direct_plan(data)
        self.direct_plan_pub.publish(self.ros_waypoints(self.direct_plan, 'PREVIEW', data['cage_id']))

    @staticmethod
    def confirmed_frozen_plan(data, frozen_payload, frozen_plan):
        if data != frozen_payload or not frozen_plan:
            raise ValueError('CONFIRMED_FROZEN_PREVIEW_REQUIRED')
        return [dict(item) for item in frozen_plan]

    def start_direct_mission(self, data):
        # Use the exact human-confirmed preview.  Live UAV/A/B drift must not
        # silently move any waypoint between confirmation and upload.
        plan = self.confirmed_frozen_plan(
            data, self.frozen_preview_payload, self.frozen_direct_plan)
        mission_id = '{}-{}'.format(data['task_id'], int(time.time()))
        request = MissionRequest(header=Header(stamp=rospy.Time.now()), mission_id=mission_id,
            target_cage_id=data['cage_id'], start=True, resume_after_override=False,
            rtl_after_mission=True, feed_relative_alt=data['feed_relative_alt'],
            transit_relative_alt=data['transit_relative_alt'],
            takeoff_relative_alt=data['takeoff_relative_alt'],
            feeding_dwell_sec=data['feeding_dwell_sec'])
        message = self.ros_waypoints(plan, mission_id, data['cage_id'])
        expected = len(plan) + (1 if rospy.get_param('/vehicle/mission_home_item_required', True) else 0)
        self.begin_submission(mission_id, expected)
        self.direct_request_pub.publish(request)
        rospy.Timer(rospy.Duration(0.35), lambda _event: self.direct_plan_pub.publish(message), oneshot=True)
        return mission_id

    def start_blockers(self):
        blockers = list(self.precheck.get('blockers', []))
        if not self.precheck.get('ready', False) and not blockers: blockers.append('PRECHECK_NOT_READY')
        if self.fcu.armed: blockers.append('AIRCRAFT_ALREADY_ARMED')
        if self.mission.state != 'IDLE': blockers.append('FSM_NOT_IDLE')
        if self.mission.manual_override or self.precheck.get('manual_override_latched', False): blockers.append('MANUAL_OVERRIDE_LATCHED')
        if not rospy.get_param('/realflight/site_alignment_confirmed', False): blockers.append('SITE_ALIGNMENT_NOT_CONFIRMED')
        if not self.preview_confirmed: blockers.append('PREVIEW_NOT_CONFIRMED')
        if not self.bag.status()['active']: blockers.append('ROSBAG_NOT_RECORDING')
        if self.preview_payload:
            try: self.normalized_payload(self.preview_payload)
            except ValueError: blockers.append('INPUT_PROFILE_TASK_SOURCE_MISMATCH')
        return sorted(set(blockers))

    def battery_json(self):
        timeout = float(rospy.get_param('/safety/battery_telemetry_timeout_sec', 3.0))
        threshold = float(rospy.get_param('/safety/low_battery_threshold', 0.20))
        return battery_status(None if self.battery is None else self.battery.percentage,
                              None if self.battery is None else self.battery.voltage,
                              self.battery_received_at, time.time(), timeout, threshold)

    def waypoint_json(self):
        if self.waypoints is None: return []
        return [{'seq': w.seq, 'type': w.waypoint_type, 'lat': w.lat, 'lon': w.lon,
                 'alt': w.alt, 'acceptance_radius': w.acceptance_radius,
                 'speed': w.speed, 'hold_time': w.hold_time,
                 'cell_id': w.cell_id} for w in self.waypoints.waypoints]

    def status_json(self):
        target = None
        if self.geometry_mode == 'circular' and self.geometry is not None:
            target = {'lat': self.geometry.center_lat, 'lon': self.geometry.center_lon,
                      'alt': self.geometry.center_alt, 'diameter_m': self.geometry.diameter_m,
                      'radius_m': self.geometry.radius_m}
        manual_latched = bool(self.mission.manual_override or self.precheck.get('manual_override_latched', False))
        blockers = self.start_blockers()
        task_source = (self.preview_payload or {}).get('task_source',
                      rospy.get_param('/realflight/task_source_mode', 'GPS_CENTER'))
        progress_index = int(self.mission.current_waypoint)
        waypoints = self.waypoint_json()
        completed = len(waypoints) if self.mission.state in ('RTL', 'LAND', 'FINISHED') else max(0, min(len(waypoints), progress_index + 1))
        uav_position = None
        if self.uav_gps is not None:
            uav_position = {'lat': self.uav_gps.latitude, 'lon': self.uav_gps.longitude,
                            'absolute_alt': self.uav_gps.altitude,
                            'relative_alt': self.relative_alt}
        submission = self.submission_status()
        return {'geometry_mode': self.geometry_mode, 'task_source': task_source,
            'input_profile': self.input_profile,
            'task_mode': 'CENTER_DWELL' if task_source == 'GPS_CENTER' else task_source,
            'feeding_enabled': bool(rospy.get_param('/feeding/enabled', True)), 'feeding_target': target,
            'center_dwell_sec': rospy.get_param('/cage/circular/feeding_dwell_sec', 5.0),
            'task_state': self.status.task_state, 'task_id': self.status.task_id, 'mode': self.status.mode,
            'cage_id': self.status.cage_id, 'current_repeat': self.status.current_repeat,
            'total_repeat': self.status.total_repeat, 'completed_targets': self.status.completed_targets,
            'total_targets': self.status.total_targets, 'current_cell_id': self.status.current_cell_id,
            'message': self.status.message, 'fsm': self.mission.state, 'fsm_reason': self.mission.reason,
            'manual_override': manual_latched, 'real_flight': bool(rospy.get_param('/web_bridge/real_flight', False)),
            'fcu_label': rospy.get_param('/web_bridge/fcu_label', ''), 'fcu_connected': self.fcu.connected,
            'armed': self.fcu.armed, 'fc_mode': self.fcu.mode or self.precheck.get('fcu_mode', ''),
            'auto_arm': bool(rospy.get_param('/vehicle/auto_arm', False)),
            'site_alignment_confirmed': bool(rospy.get_param('/realflight/site_alignment_confirmed', False)),
            'preview_confirmed': self.preview_confirmed, 'preview_pending': self.preview_pending,
            'preview_parameters': self.preview_payload, 'preview_waypoints': self.waypoint_json(),
            'preview_details': self.preview_details, 'uav_position': uav_position,
            'submission': submission, 'battery': self.battery_json(),
            'return_control': self.return_control,
            'mission_progress': {'current_waypoint': progress_index,
                'completed_waypoints': completed, 'total_waypoints': len(waypoints),
                'current_cell_id': self.mission.current_cell_id,
                'state': self.mission.state, 'reason': self.mission.reason},
            'rosbag': self.bag.status(), 'start_ready': not blockers,
            'unlock_ready': submission['unlock_ready'],
            'auto_switch_allowed': submission['auto_switch_allowed'],
            'start_blockers': blockers,
            'beacon': self.beacon, 'precheck': self.precheck}

    def grid_json(self):
        if self.geometry_mode == 'circular':
            return {'geometry_mode': 'circular', 'cage_id': int(rospy.get_param('/cage_grid/cage_id', 0)), 'rows': 0, 'cols': 0, 'cell_ids': []}
        rows = int(rospy.get_param('/cage_grid/rows', 6)); cols = int(rospy.get_param('/cage_grid/cols', 10))
        return {'geometry_mode': 'rectangular', 'cage_id': int(rospy.get_param('/cage_grid/cage_id', 0)), 'rows': rows, 'cols': cols,
                'cell_ids': ['R{:02d}C{:02d}'.format(r, c) for r in range(1, rows+1) for c in range(1, cols+1)]}

    def confirm_site(self, payload):
        required = list(self.COMMON_CONFIRMATION_KEYS)
        required.append('anchors_fixed' if self.input_profile == 'GROUND_GPS' else 'database_points_verified')
        missing = [key for key in required if payload.get(key) is not True]
        if missing: return False, missing
        rospy.set_param('/realflight/site_alignment_confirmed', True)
        return True, []


def handler_factory(bridge, web_root):
    class Handler(BaseHTTPRequestHandler):
        def send_json(self, value, code=200):
            body = json.dumps(value).encode('utf-8')
            self.send_response(code); self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store'); self.send_header('Content-Length', str(len(body)))
            self.end_headers(); self.wfile.write(body)

        def do_GET(self):
            parsed = urllib.parse.urlparse(self.path); path = parsed.path
            query = urllib.parse.parse_qs(parsed.query)
            if path == '/api/status': return self.send_json(bridge.status_json())
            if path == '/api/grid': return self.send_json(bridge.grid_json())
            if path == '/api/task-sources/large-cages':
                return self.send_json({'large_cages': bridge.large_cages(), 'input_profile': bridge.input_profile})
            if path in ('/api/task-sources/photo-points','/api/task-sources/database-points'):
                site_id = (query.get('large_cage_id') or [''])[0]
                points = [point for point in bridge.approved_photo_points()
                          if not site_id or point['site_id'] == site_id]
                return self.send_json({'points': points, 'mode': 'APPROVED_DATABASE_POINTS_ONLY',
                                       'altitude_source': 'WEB_PARAMETER_RELATIVE_ALT',
                                       'photo_altitude_ignored': True})
            if path == '/api/realflight/precheck': return self.send_json(bridge.precheck)
            if path == '/api/rosbag/status': return self.send_json(bridge.bag.status())
            if path == '/api/rosbag/list': return self.send_json({'directories': bridge.bag.recent()})
            path = os.path.join(web_root, 'index.html')
            with open(path, 'rb') as handle: body = handle.read()
            self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Cache-Control', 'no-store'); self.send_header('Content-Length', str(len(body)))
            self.end_headers(); self.wfile.write(body)

        def do_POST(self):
            try:
                length = int(self.headers.get('Content-Length', '0'))
                payload = json.loads(self.rfile.read(length).decode('utf-8') or '{}') if length else {}
            except (ValueError, TypeError): return self.send_json({'error': 'invalid JSON'}, 400)
            try:
                if self.path == '/api/preview':
                    bridge.reset_submission()
                    bridge.preview_payload = bridge.normalized_payload(payload)
                    task_source = bridge.preview_payload['task_source']
                    beacon_fresh = rospy.Time.now().to_sec()-float(bridge.beacon.get('published_at', 0)) <= float(rospy.get_param('/beacon/receive_timeout_sec', 2.0))
                    if (task_source in ('GPS_CENTER', 'GPS_GRID') and
                            rospy.get_param('/web_bridge/real_flight', False) and
                            (not bridge.beacon.get('geometry_valid', False) or not beacon_fresh)):
                        return self.send_json({'error': 'BEACON_GEOMETRY_INVALID'}, 409)
                    bridge.preview_confirmed = False; bridge.preview_pending = True; bridge.waypoints = None
                    bridge.preview_details = {}; bridge.direct_plan = []
                    bridge.frozen_direct_plan = []; bridge.frozen_preview_payload = None
                    bridge.direct_plan_pub.publish(MissionWaypointArray(
                        header=Header(stamp=rospy.Time.now()), mission_id='PREVIEW',
                        cage_id=bridge.preview_payload['cage_id']))
                    rospy.set_param('/realflight/task_source_mode', task_source)
                    rospy.set_param('/realflight/grid_route_valid', False)
                    rospy.set_param('/realflight/photo_route_valid', False)
                    if task_source == 'GPS_CENTER':
                        bridge.preview_pub.publish(bridge.make_request(bridge.preview_payload, False))
                    else:
                        bridge.publish_direct_preview(bridge.preview_payload)
                        rospy.set_param('/realflight/grid_route_valid', task_source == 'GPS_GRID')
                        rospy.set_param('/realflight/photo_route_valid', task_source == 'PHOTO_POINTS')
                    return self.send_json({'ok': True, 'parameters': bridge.preview_payload,
                                           'details': bridge.preview_details})
                if self.path == '/api/realflight/confirm-preview':
                    if bridge.preview_pending or bridge.waypoints is None or not bridge.waypoints.waypoints:
                        return self.send_json({'error': 'PREVIEW_NOT_AVAILABLE'}, 409)
                    bridge.freeze_confirmed_preview()
                    snapshot = bridge.save_snapshot()
                    bridge.preview_confirmed = True
                    return self.send_json({'ok': True, 'snapshot': snapshot})
                if self.path in ('/api/start', '/api/realflight/start-circle'):
                    requested = bridge.normalized_payload(payload)
                    if requested != bridge.preview_payload:
                        bridge.preview_confirmed = False
                        return self.send_json({'error': 'PARAMETERS_CHANGED_REPREVIEW_REQUIRED'}, 409)
                    if not bridge.verify_frozen_versions():
                        bridge.preview_confirmed = False
                        return self.send_json({'error': 'APPROVED_POINT_VERSION_CHANGED_REPREVIEW_REQUIRED'}, 409)
                    blockers = bridge.start_blockers() if rospy.get_param('/web_bridge/real_flight', False) else []
                    if blockers: return self.send_json({'error': 'MISSION_START_BLOCKED', 'blockers': blockers}, 409)
                    if requested['task_source'] == 'GPS_CENTER':
                        bridge.begin_submission(requested['task_id'])
                        bridge.start_pub.publish(bridge.make_request(requested, True))
                        mission_id = requested['task_id']
                    else:
                        mission_id = bridge.start_direct_mission(requested)
                    ok, submission = bridge.wait_for_submission()
                    return self.send_json({'ok': ok, 'mission_id': mission_id,
                                           'submission': submission,
                                           'error': '' if ok else submission.get('error', 'MISSION_UPLOAD_FAILED')},
                                          200 if ok else 409)
                if self.path == '/api/realflight/confirm-site':
                    ok, missing = bridge.confirm_site(payload)
                    return self.send_json({'ok': ok, 'missing': missing}, 200 if ok else 400)
                if self.path == '/api/realflight/revoke-site-confirmation':
                    rospy.set_param('/realflight/site_alignment_confirmed', False); return self.send_json({'ok': True})
                if self.path == '/api/realflight/return-now':
                    bridge.cancel_pub.publish(Empty())
                    bridge.preview_confirmed = False; bridge.reset_submission()
                    bridge.update_snapshot('USER_RETURN_REQUESTED', {
                        'requested_at': time.time(), 'fcu_mode': bridge.fcu.mode,
                        'armed': bridge.fcu.armed, 'battery': bridge.battery_json(),
                        'position': None if bridge.uav_gps is None else {
                            'lat': bridge.uav_gps.latitude, 'lon': bridge.uav_gps.longitude,
                            'alt': bridge.uav_gps.altitude}})
                    bridge.return_pub.publish(Empty())
                    return self.send_json({'ok': True, 'return_control': bridge.return_control}, 202)
                if self.path == '/api/rosbag/start':
                    ok, error = bridge.bag.start(bridge.status_json())
                    return self.send_json({'ok': ok, 'error': error, 'rosbag': bridge.bag.status()}, 200 if ok else 409)
                if self.path == '/api/rosbag/stop':
                    if bridge.fcu.armed: return self.send_json({'error': 'AIRCRAFT_ARMED'}, 409)
                    ok, error = bridge.bag.stop(bridge.status_json())
                    return self.send_json({'ok': ok, 'error': error, 'rosbag': bridge.bag.status()}, 200 if ok else 409)
                if self.path == '/api/cancel':
                    if bridge.fcu.armed or bridge.mission.state in (
                            'TAKEOFF','TRANSIT','FEEDING','FEEDING_DWELL','EXIT','RTL','USER_RETURN','LAND'):
                        return self.send_json({'error': 'AIRBORNE_USE_RETURN_OR_H12'}, 409)
                    bridge.cancel_pub.publish(Empty()); bridge.reset_submission()
                    return self.send_json({'ok': True})
            except (ValueError, TypeError) as exc:
                if self.path == '/api/start':
                    bridge.fail_submission(str(exc))
                    rospy.logerr('Web mission submission failed: %s', exc)
                return self.send_json({'error': str(exc)}, 400)
            return self.send_json({'error': 'not found'}, 404)

        def log_message(self, fmt, *args): rospy.loginfo('web_bridge ' + fmt, *args)
    return Handler


if __name__ == '__main__':
    rospy.init_node('web_bridge_node')
    bridge = WebBridge()
    web_root = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'www')
    host = rospy.get_param('/web_bridge/host', '0.0.0.0'); port = int(rospy.get_param('/web_bridge/port', 8080))
    server = ThreadingHTTPServer((host, port), handler_factory(bridge, web_root))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    rospy.on_shutdown(lambda: bridge.bag.stop(bridge.status_json(), force=True) if bridge.bag.status()['active'] else None)
    rospy.loginfo('Cage realflight Web UI listening on http://%s:%d', host, port)
    rospy.spin(); server.shutdown()

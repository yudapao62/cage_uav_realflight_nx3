#!/usr/bin/env python3
"""Continuous fail-closed gate for starting a real vehicle mission."""
from collections import deque
import json
import math
import time

import rosgraph
import rosnode
import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from mavros_msgs.msg import State, RCIn
from cage_msgs.msg import CageGeometry, CageGrid, MissionState
from cage_geo.geodesy import LocalENU


class GroundGPSHealth:
    """Health based only on genuinely new AnchorFix sequence numbers."""
    def __init__(self, role, min_rate_hz, stale_timeout, stable_window,
                 min_valid_samples, ok_hold, fail_hold, min_satellites):
        self.role = role
        self.min_rate_hz = float(min_rate_hz)
        self.stale_timeout = float(stale_timeout)
        self.stable_window = float(stable_window)
        self.min_valid_samples = int(min_valid_samples)
        self.ok_hold = float(ok_hold)
        self.fail_hold = float(fail_hold)
        self.min_satellites = int(min_satellites)
        self.msg = None
        self.received_at = 0.0
        self.last_seq = None
        self.seq_times = deque(maxlen=max(200, self.min_valid_samples * 4))
        self.healthy = False
        self.good_since = None
        self.bad_since = None

    def update(self, msg, now=None):
        now = time.monotonic() if now is None else now
        # Duplicate or out-of-order packets must not refresh either the rate or
        # freshness gates. Only a strictly advancing sequence is new data.
        if self.last_seq is None or msg.seq > self.last_seq:
            self.msg = msg
            self.received_at = now
            self.last_seq = msg.seq
            self.seq_times.append(now)

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        while self.seq_times and now - self.seq_times[0] > self.stable_window:
            self.seq_times.popleft()
        age = None if not self.received_at else max(0.0, now - self.received_at)
        rate = None
        if len(self.seq_times) >= 2 and self.seq_times[-1] > self.seq_times[0]:
            rate = (len(self.seq_times) - 1) / (self.seq_times[-1] - self.seq_times[0])
        msg = self.msg
        finite = bool(msg and all(math.isfinite(v) for v in (msg.lat, msg.lon, msg.altitude)))
        identity_ok = bool(msg and str(msg.anchor_id).upper() == self.role)
        valid = bool(msg and msg.valid and finite and identity_ok)
        enough_samples = len(self.seq_times) >= self.min_valid_samples
        raw_ok = bool(valid and msg.satellites >= self.min_satellites and
                      age is not None and age < self.stale_timeout and
                      enough_samples and rate is not None and rate >= self.min_rate_hz)
        if raw_ok:
            self.bad_since = None
            if self.good_since is None:
                self.good_since = now
            if now - self.good_since >= self.ok_hold:
                self.healthy = True
        else:
            self.good_since = None
            if self.bad_since is None:
                self.bad_since = now
            hard_invalid = bool(msg and (not msg.valid or not finite or not identity_ok))
            if hard_invalid or now - self.bad_since >= self.fail_hold:
                self.healthy = False
        return {
            'healthy': self.healthy, 'raw_healthy': raw_ok, 'rate_hz': rate,
            'age_sec': age, 'seq': None if msg is None else int(msg.seq),
            'valid': False if msg is None else bool(msg.valid),
            'satellites': None if msg is None else int(msg.satellites),
            'lat': None if msg is None else float(msg.lat),
            'lon': None if msg is None else float(msg.lon),
            'new_samples_in_window': len(self.seq_times),
        }


class Precheck:
    def __init__(self):
        self.input_profile = str(rospy.get_param(
            '/realflight/input_profile', 'GROUND_GPS')).strip().upper()
        if self.input_profile not in ('GROUND_GPS', 'PHOTO_COORDS'):
            raise ValueError('invalid /realflight/input_profile: {}'.format(
                self.input_profile))
        self.geometry_mode = str(rospy.get_param('/cage/geometry_mode', 'rectangular')).strip().lower()
        if self.geometry_mode not in ('rectangular', 'circular'):
            raise ValueError('invalid /cage/geometry_mode: {}'.format(self.geometry_mode))
        cfg = rospy.get_param('/ground_gps', {})
        args = (cfg.get('min_rate_hz', 0.3), cfg.get('stale_timeout_sec', 3.5),
                cfg.get('stable_window_sec', 10.0), cfg.get('min_valid_samples', 3),
                rospy.get_param('/precheck/gps_ok_hold_sec', 2.0),
                rospy.get_param('/precheck/gps_fail_hold_sec', 2.0),
                rospy.get_param('/beacon/min_satellites', 6))
        self.ground = {'A': GroundGPSHealth('A', *args), 'B': GroundGPSHealth('B', *args)}
        self.fcu = None; self.fcu_at = 0.0
        self.gps = None; self.gps_at = 0.0
        self.rc = None; self.rc_at = 0.0
        self.battery = None; self.battery_at = 0.0
        self.grid = None; self.home = None; self.geometry = None
        self.feeder_status_at = 0.0
        self.fsm = None; self.last_fsm_state = None
        self.beacon = {}; self.beacon_at = 0.0
        self.enable_feeding = bool(rospy.get_param('/feeding/enabled', True))
        self.pub = rospy.Publisher('/cage/realflight/precheck', String, queue_size=1, latch=True)
        rospy.Subscriber('/mavros/state', State, self.on_fcu, queue_size=1)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.on_gps, queue_size=1)
        rospy.Subscriber('/mavros/rc/in', RCIn, self.on_rc, queue_size=1)
        rospy.Subscriber('/mavros/battery', BatteryState, self.on_battery, queue_size=1)
        if self.input_profile == 'GROUND_GPS':
            from gps_receiver.msg import AnchorFix
            rospy.Subscriber(rospy.get_param('/beacon/input_topic_a', '/farm/scene_1/anchor_A/fix'),
                             AnchorFix, lambda msg: self.ground['A'].update(msg), queue_size=20)
            rospy.Subscriber(rospy.get_param('/beacon/input_topic_b', '/farm/scene_1/anchor_B/fix'),
                             AnchorFix, lambda msg: self.ground['B'].update(msg), queue_size=20)
        rospy.Subscriber('/cage/grid_info', CageGrid, lambda m: setattr(self, 'grid', m), queue_size=1)
        rospy.Subscriber('/cage/geometry', CageGeometry, lambda m: setattr(self, 'geometry', m), queue_size=1)
        rospy.Subscriber('/cage/home', NavSatFix, lambda m: setattr(self, 'home', m), queue_size=1)
        rospy.Subscriber('/mission/state', MissionState, self.on_fsm, queue_size=1)
        rospy.Subscriber('/cage/beacon/status', String, self.on_beacon, queue_size=1)
        rospy.Subscriber('/feeder/status', Bool, self.on_feeder_status, queue_size=1)
        rospy.Timer(rospy.Duration(0.5), self.tick)

    def on_fcu(self, msg): self.fcu = msg; self.fcu_at = time.monotonic()
    def on_gps(self, msg): self.gps = msg; self.gps_at = time.monotonic()
    def on_rc(self, msg): self.rc = msg; self.rc_at = time.monotonic()
    def on_battery(self, msg): self.battery = msg; self.battery_at = time.monotonic()
    def on_fsm(self, msg):
        if (self.last_fsm_state is not None and msg.state != self.last_fsm_state and
                msg.state in ('FINISHED', 'FAILSAFE', 'ERROR', 'MANUAL_OVERRIDE')):
            rospy.set_param('/realflight/site_alignment_confirmed', False)
        self.last_fsm_state = msg.state
        self.fsm = msg
    def on_beacon(self, msg):
        try: self.beacon = json.loads(msg.data); self.beacon_at = time.monotonic()
        except (TypeError, ValueError): self.beacon = {}
    def on_feeder_status(self, _msg): self.feeder_status_at = time.monotonic()

    @staticmethod
    def _mode_conflict(names):
        conflict_nodes = sorted(n for n in names if 'uav_control_bridge' in n.lower() or 'offboard' in n.lower())
        publishers = []
        try:
            system_state = rosgraph.Master('/realflight_precheck').getSystemState()
            for topic, nodes in system_state[0]:
                if topic == '/mavros/setpoint_raw/local':
                    publishers = sorted(nodes)
                    break
        except Exception:
            return True, conflict_nodes, publishers, 'ROS_GRAPH_UNAVAILABLE'
        unexpected = [n for n in publishers if n != '/mavros']
        return bool(conflict_nodes or unexpected), conflict_nodes, publishers, ''

    @staticmethod
    def _passes(value):
        return value is True or value == 'SKIPPED'

    @staticmethod
    def _prestart_mode_ok(fcu_ok, mode, allowed_start_modes):
        return bool(fcu_ok and mode in set(allowed_start_modes))

    @staticmethod
    def _profile_compatible(input_profile, task_source):
        return ((input_profile == 'GROUND_GPS' and task_source in ('GPS_CENTER', 'GPS_GRID')) or
                (input_profile == 'PHOTO_COORDS' and task_source == 'PHOTO_POINTS'))

    def tick(self, _event):
        now = time.monotonic()
        default_source = 'GPS_CENTER' if self.input_profile == 'GROUND_GPS' else 'PHOTO_POINTS'
        task_source = str(rospy.get_param('/realflight/task_source_mode', default_source)).strip().upper()
        if task_source not in ('GPS_CENTER', 'GPS_GRID', 'PHOTO_POINTS'):
            task_source = default_source
        ground_required = self.input_profile == 'GROUND_GPS'
        profile_task_compatible = self._profile_compatible(self.input_profile, task_source)
        uav_gps_ok = (self.gps is not None and now-self.gps_at <= float(rospy.get_param('/safety/gps_timeout', 2.0))
            and self.gps.status.status >= NavSatStatus.STATUS_FIX
            and all(math.isfinite(v) for v in (self.gps.latitude, self.gps.longitude, self.gps.altitude))
            and (self.gps.latitude, self.gps.longitude) != (0.0, 0.0))
        fcu_ok = bool(self.fcu and self.fcu.connected and now-self.fcu_at <= float(rospy.get_param('/safety/fcu_timeout', 2.0)))
        beacon_ok = (self.beacon.get('geometry_valid') is True and
            now-self.beacon_at <= float(rospy.get_param('/beacon/receive_timeout_sec', 2.0)))
        ground = {role: channel.snapshot(now) for role, channel in self.ground.items()}
        baseline = self.beacon.get('baseline_m')
        grid_alignment = bool(self.grid and baseline is not None and
            abs(self.grid.cols*self.grid.cell_size_x-float(baseline)) <=
            float(rospy.get_param('/cage_grid/max_grid_length_error_m', 5.0)))
        center_distance = None; center_ok = False
        dwell = float(rospy.get_param('/cage/circular/feeding_dwell_sec', 5.0))
        if self.geometry_mode == 'circular' and self.geometry and uav_gps_ok:
            values = (self.geometry.center_lat, self.geometry.center_lon, self.geometry.center_alt)
            if (self.geometry.geometry_mode == 'circular' and all(math.isfinite(v) for v in values) and
                    -90 <= values[0] <= 90 and -180 <= values[1] <= 180 and self.geometry.radius_m > 0):
                local = LocalENU(self.gps.latitude, self.gps.longitude, self.gps.altitude)
                east, north, _ = local.to_enu(*values)
                center_distance = math.hypot(east, north)
                center_ok = center_distance <= float(rospy.get_param('/safety/max_leg_distance', 5000.0))
        rc_ok = self.rc is not None and bool(self.rc.channels) and now-self.rc_at <= 2.0
        battery_ok = (self.battery is not None and now-self.battery_at <=
                      float(rospy.get_param('/safety/battery_telemetry_timeout_sec', 3.0)))
        try:
            names = set(rosnode.get_node_names())
            simulation_absent = not any('gazebo' in n or 'feeder_simulator' in n or 'simulation_acceptance' in n for n in names)
        except Exception:
            names = set(); simulation_absent = False
        conflict, conflict_nodes, setpoint_publishers, graph_error = self._mode_conflict(names)
        feeder_real = rospy.get_param('/feeding/feeder_mode', '') == 'real'
        feeder_fresh = now-self.feeder_status_at <= 2.0
        allowed_start_modes = set(rospy.get_param('/vehicle/mission_start_modes', ['POSCTL', 'ALTCTL']))
        prestart_mode_ok = self._prestart_mode_ok(fcu_ok, self.fcu.mode if self.fcu else '',
                                                  allowed_start_modes)
        checks = {
            'FCU': fcu_ok, 'UAV_GPS': uav_gps_ok,
            'PRESTART_MODE': prestart_mode_ok, 'SIMULATION_ABSENT': simulation_absent,
            'RC_INPUT': rc_ok if rospy.get_param('/vehicle/require_rc_ready', True) else True,
            'BATTERY_TELEMETRY': battery_ok,
            'GROUND_GPS_A': ground['A']['healthy'] if ground_required else 'SKIPPED',
            'GROUND_GPS_B': ground['B']['healthy'] if ground_required else 'SKIPPED',
            'GEOMETRY': beacon_ok if ground_required else 'SKIPPED',
            'INPUT_PROFILE_TASK_COMPATIBLE': profile_task_compatible,
            'ACFLY_MODE_CONFLICT_ABSENT': not conflict,
            'FSM_READY': bool(self.fsm and not self.fsm.manual_override and self.fsm.state in ('IDLE', 'FINISHED')),
            'AUTO_ARM_FALSE': rospy.get_param('/vehicle/auto_arm', True) is False,
            'FEEDER_REAL': feeder_real if self.enable_feeding else 'SKIPPED',
            'FEEDER_STATUS': feeder_fresh if self.enable_feeding else 'SKIPPED',
            'SITE_ALIGNMENT_CONFIRMED': rospy.get_param('/realflight/site_alignment_confirmed', False) is True,
        }
        if task_source == 'GPS_GRID':
            checks['GRID_ROUTE'] = rospy.get_param('/realflight/grid_route_valid', False) is True
        elif task_source == 'PHOTO_POINTS':
            checks['PHOTO_ROUTE'] = rospy.get_param('/realflight/photo_route_valid', False) is True
        elif self.geometry_mode == 'rectangular':
            checks['GRID'] = bool(self.grid and self.home and self.grid.rows > 0 and self.grid.cols > 0 and '/cage_grid_node' in names)
            checks['GRID_ALIGNMENT'] = grid_alignment
        else:
            checks['CIRCULAR_TARGET'] = bool(center_ok and '/cage_grid_node' in names)
            checks['CIRCULAR_DWELL'] = bool(math.isfinite(dwell) and dwell > 0)
        ready = all(self._passes(value) for value in checks.values())
        resume_checks = dict(checks)
        resume_checks['FSM_READY'] = bool(self.fsm and self.fsm.manual_override)
        resume_ready = all(self._passes(value) for value in resume_checks.values())
        rospy.set_param('/realflight/start_ready', ready)
        rospy.set_param('/realflight/resume_ready', resume_ready)
        blockers = [name for name, value in checks.items() if not self._passes(value)]
        lines = ['REALFLIGHT PRECHECK']
        for name, value in checks.items():
            label = value if isinstance(value, str) else ('OK' if value else 'FAIL')
            lines.append('{:<29}{}'.format(name, label))
        for role in ('A', 'B'):
            info = ground[role]
            lines.append('GROUND GPS {} RATE / AGE    {} / {}'.format(role, info['rate_hz'], info['age_sec']))
            lines.append('GROUND GPS {} SEQ / SAT     {} / {}'.format(role, info['seq'], info['satellites']))
        lines.append('BASELINE M                    {}'.format(baseline))
        if self.geometry_mode == 'circular': lines.append('CENTER DISTANCE M              {}'.format(center_distance))
        if conflict: lines.append('ACFLY_MODE_CONFLICT            {}'.format(conflict_nodes or setpoint_publishers))
        lines.append('RESULT                        {}'.format('READY' if ready else 'FAIL'))
        rospy.loginfo_throttle(5.0, '\n'.join(lines))
        payload = {
            'ready': ready, 'resume_ready': resume_ready, 'checks': checks,
            'blockers': blockers, 'ground_gps': ground, 'beacon': self.beacon,
            'geometry_mode': self.geometry_mode, 'task_source': task_source,
            'input_profile': self.input_profile,
            'ground_gps_required': ground_required, 'center_distance_m': center_distance,
            'fcu_mode': self.fcu.mode if self.fcu else '',
            'manual_override_latched': bool(self.fsm and self.fsm.manual_override),
            'feeding_enabled': self.enable_feeding,
            'feeder_mode': rospy.get_param('/feeding/feeder_mode', ''),
            'auto_arm': rospy.get_param('/vehicle/auto_arm', None),
            'site_alignment_confirmed': rospy.get_param('/realflight/site_alignment_confirmed', False),
            'acfly_mode_conflict': conflict, 'conflict_nodes': conflict_nodes,
            'setpoint_publishers': setpoint_publishers, 'graph_error': graph_error,
        }
        self.pub.publish(String(data=json.dumps(payload, allow_nan=False)))


if __name__ == '__main__':
    rospy.init_node('realflight_precheck')
    Precheck()
    rospy.spin()

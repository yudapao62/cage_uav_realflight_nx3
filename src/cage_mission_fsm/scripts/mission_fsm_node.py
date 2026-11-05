#!/usr/bin/env python3
import math
import json
import time
import rospy
from mavros_msgs.msg import RCIn, State, WaypointReached
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Empty, Header, String
from cage_msgs.msg import FeederCommand, MissionRequest, MissionState, MissionWaypointArray
from cage_mavros_adapter.adapter import MavrosAdapter
from cage_mission_fsm.core import MissionFSM, RtlRequestController, States
from cage_geo.geodesy import LocalENU


class MissionFSMNode:
    def __init__(self):
        self.fsm = MissionFSM(); self.fsm.initialize()
        self.adapter = MavrosAdapter()
        self.auto_arm = rospy.get_param('/vehicle/auto_arm', False)
        self.auto_mode = rospy.get_param('/vehicle/auto_mode', 'AUTO.MISSION')
        self.manual_auto_mode_switch = bool(
            rospy.get_param('/vehicle/manual_auto_mode_switch', True))
        self.prearm_mode = rospy.get_param('/vehicle/prearm_mode', 'STABILIZE')
        self.rtl_mode = rospy.get_param('/vehicle/rtl_mode', 'AUTO.RTL')
        self.require_auto_permission_switch = bool(
            rospy.get_param('/vehicle/require_auto_permission_switch', False))
        self.auto_permission_channel_index = int(
            rospy.get_param('/vehicle/auto_permission_channel_index', 4))
        self.auto_permission_threshold = int(
            rospy.get_param('/vehicle/auto_permission_threshold', 1650))
        self.auto_permission_rc_timeout = float(
            rospy.get_param('/vehicle/auto_permission_rc_timeout', 1.0))
        if (self.auto_permission_channel_index < 0 or
                not 800 <= self.auto_permission_threshold <= 2200 or
                self.auto_permission_rc_timeout <= 0.0):
            raise ValueError('invalid AUTO permission switch configuration')
        self.enable_feeding = bool(rospy.get_param('/feeding/enabled', True))
        self.input_profile = str(rospy.get_param('/realflight/input_profile', 'GROUND_GPS')).strip().upper()
        if self.input_profile not in ('GROUND_GPS', 'PHOTO_COORDS'):
            raise ValueError('invalid /realflight/input_profile')
        self.geometry_mode = str(rospy.get_param('/cage/geometry_mode', 'rectangular')).strip().lower()
        if self.geometry_mode not in ('rectangular', 'circular'):
            raise ValueError('invalid /cage/geometry_mode: {}'.format(self.geometry_mode))
        self.circular_dwell_sec = float(rospy.get_param('/cage/circular/feeding_dwell_sec', 5.0))
        self.circular_altitude_tolerance = float(rospy.get_param('/cage/circular/feeding_altitude_tolerance_m', 3.0))
        self.circular_seq_fallback_enabled = bool(
            rospy.get_param('/cage/circular/current_seq_fallback_enabled', True))
        self.circular_fallback_radius = float(
            rospy.get_param('/cage/circular/feeding_fallback_radius_m', 5.0))
        self.circular_fallback_altitude_tolerance = float(
            rospy.get_param('/cage/circular/feeding_fallback_altitude_tolerance_m', 5.0))
        if self.geometry_mode == 'circular' and self.circular_dwell_sec <= 0:
            raise ValueError('circular feeding_dwell_sec must be positive')
        if (self.circular_fallback_radius <= 0.0 or
                self.circular_fallback_altitude_tolerance <= 0.0):
            raise ValueError('circular feeding fallback tolerances must be positive')
        self.fsm.MANUAL_MODES = set(rospy.get_param('/vehicle/manual_modes', list(self.fsm.MANUAL_MODES)))
        self.gps_timeout = rospy.get_param('/safety/gps_timeout', 2.0)
        self.fcu_timeout = rospy.get_param('/safety/fcu_timeout', 2.0)
        self.low_battery_threshold = rospy.get_param('/safety/low_battery_threshold', 0.20)
        self.require_beacon = rospy.get_param('/safety/require_beacon_before_start', False)
        self.beacon_loss_timeout = float(rospy.get_param('/safety/beacon_loss_during_active_mission/timeout_sec', 5.0))
        self.beacon_prestart_loss_timeout = float(
            rospy.get_param('/safety/beacon_loss_before_takeoff_timeout_sec', self.beacon_loss_timeout))
        self.beacon_loss_action = rospy.get_param('/safety/beacon_loss_during_active_mission/action', 'RTL')
        self.last_beacon_valid = 0.0
        self.last_beacon_report = 0.0
        self.beacon_current_valid = False
        self.timeouts = {States.WAIT_ARM: rospy.get_param('/mission/arm_timeout',60.0),
                         States.TAKEOFF: rospy.get_param('/mission/takeoff_timeout',60.0),
                         States.TRANSIT: rospy.get_param('/mission/transit_timeout',600.0),
                         States.FEEDING: rospy.get_param('/mission/feeding_timeout',180.0),
                         States.FEEDING_DWELL: rospy.get_param('/mission/feeding_timeout',180.0),
                         States.EXIT: rospy.get_param('/mission/feeding_timeout',180.0),
                         States.RTL: rospy.get_param('/mission/rtl_timeout',600.0),
                         States.USER_RETURN: rospy.get_param('/mission/rtl_timeout',600.0),
                         States.LAND: rospy.get_param('/mission/rtl_timeout',600.0)}
        self.feeder_off_timeout = rospy.get_param('/mission/timeout_recovery/feeder_off_timeout_sec', 2.0)
        rtl_cfg = rospy.get_param('/mission/timeout_recovery/rtl', {})
        self.rtl_retries = int(rtl_cfg.get('retries', 3))
        self.rtl_request_interval = float(rtl_cfg.get('request_interval_sec', 1.0))
        self.rtl_confirm_timeout = float(rtl_cfg.get('mode_confirm_timeout_sec', 5.0))
        self.normal_rtl = RtlRequestController(self.rtl_retries, self.rtl_request_interval,
                                               self.rtl_confirm_timeout)
        self.recovery_entry_time = 0.0
        self.last_recovery_rtl_request = 0.0
        self.recovery_rtl_attempts = 0
        self.recovery_stage = ''
        now = time.monotonic()
        self.last_fcu, self.last_gps, self.state_since = now, 0.0, now
        self.last_state, self.waypoints = self.fsm.state, []
        self.upload_attempted = self.arm_attempted = self.auto_attempted = self.rtl_attempted = False
        self.last_arm_attempt = self.last_auto_attempt = 0.0
        self.last_prearm_mode_attempt = 0.0
        self.stream_rate_requested = False
        self.feeder_status, self.feeder_command_time = False, 0.0
        self.current_gps = None
        self.rc_channels = []
        self.last_rc = 0.0
        self.dwell_started_at = 0.0
        self.grid_feeding_duration_sec = 0.0
        self.grid_cell_started_at = 0.0
        self.grid_cell_index = -1
        self.return_requested_at = 0.0
        self.return_requested_monotonic = 0.0
        self.return_initial_mode = ''
        self.return_explicit = False
        self.return_clear_done = False
        self.return_status = {'state': 'IDLE', 'reason': '', 'requested_at': None,
                              'attempts': 0}
        self.state_pub = rospy.Publisher('/mission/state', MissionState, queue_size=1, latch=True)
        self.feeder_pub = rospy.Publisher('/feeder/command', FeederCommand, queue_size=1, latch=True)
        self.return_status_pub = rospy.Publisher('/mission/return_status', String,
                                                 queue_size=1, latch=True)
        rospy.Subscriber('/mavros/state', State, self.on_state, queue_size=1)
        rospy.Subscriber('/mavros/rc/in', RCIn, self.on_rc, queue_size=1)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.on_gps, queue_size=1)
        rospy.Subscriber('/mavros/mission/reached', WaypointReached, self.on_reached, queue_size=10)
        rospy.Subscriber('/mavros/battery', BatteryState, self.on_battery, queue_size=1)
        rospy.Subscriber('/mission/request', MissionRequest, self.on_request, queue_size=1)
        rospy.Subscriber('/mission/direct_request', MissionRequest, self.on_request, queue_size=1)
        rospy.Subscriber('/mission/return_now', Empty, self.on_return_now, queue_size=1)
        rospy.Subscriber('/mission/waypoints', MissionWaypointArray, self.on_waypoints, queue_size=1)
        rospy.Subscriber('/feeder/status', Bool, self.on_feeder_status, queue_size=1)
        if self.require_beacon:
            rospy.Subscriber('/cage/beacon/status', String, self.on_beacon_status, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self.tick)
        self.publish_return_status()
        self.publish_state()

    def on_state(self, msg):
        now = time.monotonic()
        if msg.connected: self.last_fcu = now
        self._check_manual_takeover(msg.mode)

    def on_rc(self, msg):
        self.rc_channels = list(msg.channels)
        self.last_rc = time.monotonic()

    def auto_permission_granted(self, now=None):
        if not self.require_auto_permission_switch:
            return True
        now = time.monotonic() if now is None else now
        index = self.auto_permission_channel_index
        return bool(now-self.last_rc <= self.auto_permission_rc_timeout and
                    index < len(self.rc_channels) and
                    self.rc_channels[index] > self.auto_permission_threshold)

    def _check_manual_takeover(self, mode):
        # A RETURN pressed while already in POSCTL/ALTCTL deliberately asks the
        # FCU to leave that initial mode.  Allow the first RTL request to be
        # processed, but never ignore a new pilot-selected manual mode after
        # AUTO.RTL has been confirmed or after leaving a non-manual mode.
        if (self.fsm.state == States.USER_RETURN and not self.normal_rtl.confirmed and
                mode in self.fsm.MANUAL_MODES and mode == self.return_initial_mode and
                time.monotonic()-self.return_requested_monotonic <= self.rtl_confirm_timeout):
            return False
        if self.fsm.mode_changed(mode, self.auto_mode):
            rospy.logwarn('Manual takeover detected (%s); NX relinquishes control', mode)
            self.command_feeder(False, 'MANUAL_OVERRIDE')
            if self.return_explicit:
                self.set_return_status('MANUAL_OVERRIDE', 'H12_MANUAL_TAKEOVER')
            self._state_changed()
            return True
        return self.fsm.manual_override_latched

    def automatic_control_allowed(self, action='automatic control'):
        allowed = self.fsm.automatic_control_allowed()
        if not allowed:
            rospy.logwarn_throttle(5.0, 'AUTO CONTROL BLOCKED: MANUAL_OVERRIDE_LATCHED (%s)', action)
        return allowed

    def on_gps(self, msg):
        if msg.status.status >= NavSatStatus.STATUS_FIX and all(math.isfinite(v) for v in (msg.latitude,msg.longitude,msg.altitude)) and (msg.latitude != 0 or msg.longitude != 0):
            self.current_gps = msg
            self.last_gps = time.monotonic()

    def on_battery(self, msg):
        if math.isfinite(msg.percentage) and 0 <= msg.percentage < self.low_battery_threshold:
            self.command_feeder(False, 'LOW_BATTERY')
            if self.fsm.manual_override_latched:
                self.set_return_status('MANUAL_OVERRIDE', 'LOW_BATTERY_H12_CONTROL_ACTIVE')
                return
            if self.fsm.state == States.IDLE:
                self.fsm.failsafe('LOW_BATTERY'); self._state_changed()
            elif self.fsm.state in self.fsm.ACTIVE and self.automatic_control_allowed('low battery RTL'):
                self.on_return_now(Empty())
                if self.return_status['state'] == 'REQUESTED':
                    self.set_return_status('REQUESTED', 'LOW_BATTERY_RETURN_REQUESTED')

    def set_return_status(self, state, reason=''):
        self.return_status.update({
            'state': str(state), 'reason': str(reason),
            'requested_at': (self.return_requested_at
                             if self.return_requested_at > 0.0 else None),
            'attempts': int(self.normal_rtl.attempts),
        })
        self.publish_return_status()

    def publish_return_status(self):
        self.return_status_pub.publish(String(data=json.dumps(self.return_status)))

    def on_return_now(self, _msg):
        """Fail-safe, idempotent operator command to abandon and return."""
        if self.return_status['state'] in ('REQUESTED', 'RTL_CONFIRMED'):
            self.publish_return_status()
            return
        self.command_feeder(False, 'USER_RETURN')
        self.return_requested_at = time.time()
        self.return_requested_monotonic = time.monotonic()
        self.return_initial_mode = self.adapter.get_mode()
        self.return_explicit = True
        self.return_clear_done = False
        if not self.adapter.is_connected():
            self.fsm.mission_aborted = True
            self.set_return_status('FAILED', 'FCU_DISCONNECTED_MANUAL_TAKEOVER_REQUIRED')
            self._state_changed()
            return
        if not self.adapter.state.armed:
            self.fsm.user_return(False)
            try:
                self.return_clear_done = bool(self.adapter.clear_mission())
            except rospy.ServiceException:
                self.return_clear_done = False
            self.set_return_status('LANDED', 'TASK_CANCELLED_DISARMED')
            self._state_changed()
            return
        if self.current_gps is None or time.monotonic()-self.last_gps > self.gps_timeout:
            self.fsm.mission_aborted = True
            self.set_return_status('FAILED', 'UAV_GPS_INVALID_MANUAL_TAKEOVER_REQUIRED')
            self._state_changed()
            return
        if not self.fsm.user_return(True):
            self.set_return_status('MANUAL_OVERRIDE', self.fsm.reason)
            self._state_changed()
            return
        self.normal_rtl.reset(time.monotonic())
        self.set_return_status('REQUESTED', 'USER_RETURN_REQUESTED')
        self._state_changed()

    def on_request(self, msg):
        if not msg.start: return
        if self.fsm.manual_override_latched and not msg.resume_after_override:
            rospy.logerr('Mission request rejected: MANUAL_OVERRIDE_LATCHED'); return
        ready_param = '/realflight/resume_ready' if msg.resume_after_override else '/realflight/start_ready'
        if self.require_beacon and not rospy.get_param(ready_param, False):
            rospy.logerr('Realflight mission rejected: %s is false', ready_param); return
        if msg.resume_after_override and self.adapter.get_mode() in self.fsm.MANUAL_MODES:
            rospy.logerr('Explicit resume rejected while FC mode is manual: %s', self.adapter.get_mode()); return
        rtl_after = True if not hasattr(msg, 'rtl_after_mission') else msg.rtl_after_mission
        if self.fsm.request(msg.mission_id, msg.resume_after_override, rtl_after):
            self.upload_attempted = self.arm_attempted = self.auto_attempted = self.rtl_attempted = False
            self.last_arm_attempt = self.last_auto_attempt = self.last_prearm_mode_attempt = 0.0
            self.dwell_started_at = 0.0
            self.grid_feeding_duration_sec = float(msg.feeding_dwell_sec)
            self.grid_cell_started_at = 0.0
            self.grid_cell_index = -1
            self.return_explicit = False
            self.return_clear_done = False
            self.return_requested_at = 0.0
            self.return_requested_monotonic = 0.0
            self.set_return_status('IDLE', '')
            self._state_changed()
        else: rospy.logwarn('Mission request rejected in state %s', self.fsm.state)

    def on_waypoints(self, msg):
        if not self.automatic_control_allowed('waypoint handling'): return
        if self.fsm.state != States.PLAN or msg.mission_id != self.fsm.mission_id: return
        if not self.valid_waypoints(msg):
            self.fsm.transition(States.ERROR, 'INVALID_MISSION'); self._state_changed(); return
        self.waypoints = list(msg.waypoints); self.fsm.planned(); self._state_changed()

    def valid_waypoints(self, msg):
        kinds = [w.waypoint_type for w in msg.waypoints]
        legacy = ['TAKEOFF','TRANSIT','CAGE_ENTER','FEED_START','FEED_END','CAGE_EXIT']
        new_grid = (len(kinds) >= 4 and
                    kinds[:3] == ['TAKEOFF','TRANSIT','GRID_ENTER'] and
                    all(k == 'CELL_TARGET' for k in kinds[3:]))
        legacy_grid = (len(kinds) >= 5 and
                       kinds[:3] == ['TAKEOFF','TRANSIT','GRID_ENTER'] and
                       kinds[-1] == 'GRID_EXIT' and
                       all(k == 'CELL_TARGET' for k in kinds[3:-1]))
        grid_ok = new_grid or legacy_grid
        circular_ok = kinds == ['TAKEOFF','TRANSIT','FEED_POINT'] and msg.waypoints[-1].hold_time > 0
        if kinds != legacy and not grid_ok and not circular_ok: return False
        numerically_valid = all(
            w.seq == i and -90 <= w.lat <= 90 and -180 <= w.lon <= 180 and w.alt > 0
            and all(math.isfinite(v) for v in
                    (w.lat,w.lon,w.alt,w.yaw,w.acceptance_radius,w.speed,w.hold_time))
            for i,w in enumerate(msg.waypoints))
        if not numerically_valid:
            return False
        if new_grid:
            return (self.grid_feeding_duration_sec > 0.0 and
                    all(w.hold_time > self.grid_feeding_duration_sec
                        for w in msg.waypoints[3:]))
        return True

    def on_reached(self, msg):
        if not self.automatic_control_allowed('waypoint reached'): return
        mission_index = msg.wp_seq - self.adapter.mission_seq_offset
        if mission_index < 0 or mission_index >= len(self.waypoints): return
        old_feeder = self.fsm.feeder_enabled
        reached_wp = self.waypoints[mission_index]
        self.fsm.waypoint_reached(mission_index, reached_wp.waypoint_type, reached_wp.cell_id,
                                  enable_feeding=self.enable_feeding)
        if reached_wp.waypoint_type == 'FEED_POINT' and self.fsm.state == States.FEEDING_DWELL:
            self._begin_circular_dwell(mission_index, 'MISSION_ITEM_REACHED')
        if self.fsm.feeder_enabled != old_feeder and reached_wp.waypoint_type != 'FEED_POINT':
            self.command_feeder(self.fsm.feeder_enabled, self.waypoints[mission_index].waypoint_type)
        if reached_wp.waypoint_type == 'CELL_TARGET':
            self._begin_grid_cell(mission_index, 'MISSION_ITEM_REACHED')
        self._state_changed()

    def on_feeder_status(self, msg): self.feeder_status = msg.data

    def beacon_required_for_active_task(self):
        """Only the immutable GROUND_GPS startup profile may require A/B."""
        return bool(self.require_beacon and self.input_profile == 'GROUND_GPS')

    def on_beacon_status(self, msg):
        try:
            self.beacon_current_valid = json.loads(msg.data).get('geometry_valid') is True
            self.last_beacon_report = time.monotonic()
            if self.beacon_current_valid:
                self.last_beacon_valid = time.monotonic()
        except (ValueError, TypeError):
            pass

    def command_feeder(self, enable, reason):
        if enable and (not self.enable_feeding or not self.automatic_control_allowed('feeder enable')):
            rospy.logwarn('Feeder enable blocked (%s): feeding_enabled=%s manual_override_latched=%s',
                          reason, self.enable_feeding, self.fsm.manual_override_latched)
            enable = False
        self.fsm.feeder_enabled = bool(enable)
        self.feeder_command_time = time.monotonic()
        self.feeder_pub.publish(FeederCommand(header=Header(stamp=rospy.Time.now()),
                                mission_id=self.fsm.mission_id, enable=enable, reason=reason))

    def tick(self, _event):
        now = time.monotonic()
        # Redundant polling closes any transport/callback race: manual mode is
        # always evaluated before mission progression or dwell completion.
        self._check_manual_takeover(self.adapter.get_mode())
        if self.fsm.manual_override_latched:
            self.fsm.feeder_enabled = False
            self.publish_state()
            return
        if self.fsm.state == States.WAIT_FCU and self.adapter.is_connected():
            if rospy.get_param('/vehicle/request_stream_rate', True) and not self.stream_rate_requested:
                self.stream_rate_requested = self.adapter.request_stream_rate(rospy.get_param('/vehicle/stream_rate', 10))
            self.fsm.fcu_connected(); self._state_changed()
        if self.fsm.state == States.WAIT_GPS and now-self.last_gps <= self.gps_timeout:
            self.fsm.gps_valid(); self._state_changed()
        self._maybe_begin_circular_dwell(now)
        self._maybe_progress_grid_mission(now)
        self._update_grid_timers(now)
        if self.fsm.state in self.fsm.ACTIVE:
            if now-self.last_fcu > self.fcu_timeout:
                self.fsm.failsafe('FCU_DISCONNECTED'); self.command_feeder(False,'FCU_DISCONNECTED')
                if self.return_explicit: self.set_return_status('FAILED','FCU_DISCONNECTED_MANUAL_TAKEOVER_REQUIRED')
                self._state_changed()
            elif now-self.last_gps > self.gps_timeout:
                self.fsm.failsafe('GPS_LOST'); self.command_feeder(False,'GPS_LOST')
                if self.return_explicit: self.set_return_status('FAILED','UAV_GPS_LOST_MANUAL_TAKEOVER_REQUIRED')
                self._state_changed()
        beacon_required = self.beacon_required_for_active_task()
        if (beacon_required and self.fsm.state in (States.PLAN,States.UPLOAD_MISSION,States.WAIT_ARM) and
                (self.last_beacon_valid <= 0.0 or
                 now-self.last_beacon_valid > self.beacon_prestart_loss_timeout)):
            self.fsm.failsafe('BEACON_LOST_BEFORE_TAKEOFF')
            self.command_feeder(False,'BEACON_LOST')
            self._state_changed()
        if (beacon_required and self.beacon_loss_action == 'RTL' and
                self.fsm.state in (States.TAKEOFF,States.TRANSIT,States.FEEDING,States.FEEDING_DWELL,States.EXIT) and
                now-self.last_beacon_valid > self.beacon_loss_timeout):
            self.fsm.timeout_abort('BEACON_LOST')
            self._enter_timeout_abort(now)
            self._state_changed()
        if self.fsm.state == States.UPLOAD_MISSION and not self.upload_attempted:
            self.upload_attempted = True
            upload_ok = self.adapter.upload_mission(self.waypoints)
            self.fsm.upload_result(upload_ok)
            if not upload_ok and self.adapter.last_upload_error:
                self.fsm.reason = self.adapter.last_upload_error
                rospy.logerr('Mission upload rejected: %s', self.adapter.last_upload_error)
            self._state_changed()
        if self.fsm.state == States.WAIT_ARM:
            mode = self.adapter.get_mode()
            auto_permission = self.auto_permission_granted(now)
            beacon_permission = not beacon_required or self.beacon_current_valid
            auto_entry_permission = auto_permission and beacon_permission
            # After a previous mission ends in LAND/RTL, ArduPilot may refuse arming
            # until we explicitly return to the pre-arm mode again.
            if self.auto_arm and not self.adapter.state.armed and mode != self.prearm_mode and now-self.last_prearm_mode_attempt >= 2.0:
                self.last_prearm_mode_attempt = now
                if not self.adapter.set_mode(self.prearm_mode): rospy.logwarn('Pre-arm mode request rejected; remaining in WAIT_ARM')
                mode = self.adapter.get_mode()
            if self.auto_arm and not self.adapter.state.armed and mode == self.prearm_mode and now-self.last_arm_attempt >= 2.0:
                self.arm_attempted = True; self.last_arm_attempt = now
                # TODO_K9_VERIFY: automatic arming stays disabled in real-vehicle configuration.
                if not self.adapter.arm(): rospy.logwarn('ARM rejected; remaining in WAIT_ARM until timeout')
            if (not self.manual_auto_mode_switch and
                    self.fsm.should_request_auto(self.adapter.state.armed, auto_entry_permission) and
                    now-self.last_auto_attempt >= 2.0):
                self.auto_attempted = True; self.last_auto_attempt = now
                if not self.adapter.start_mission(self.auto_mode):
                    rospy.logwarn('%s request rejected; remaining in WAIT_ARM until timeout', self.auto_mode)
            elif (self.manual_auto_mode_switch and self.adapter.state.armed and
                    auto_entry_permission and mode != self.auto_mode):
                rospy.logwarn_throttle(2.0,
                    'Mission verified and armed; waiting for pilot to switch H12 to %s',
                    self.auto_mode)
            elif self.adapter.state.armed and not auto_permission:
                rospy.logwarn_throttle(2.0,
                    'AUTO.MISSION blocked: move H12 CH%d/E switch to RIGHT (PWM > %d)',
                    self.auto_permission_channel_index+1, self.auto_permission_threshold)
            elif self.adapter.state.armed and not beacon_permission:
                rospy.logwarn_throttle(2.0, 'AUTO.MISSION blocked: ground GPS geometry is not currently valid')
            if self.fsm.confirm_auto_mode(self.adapter.state.armed and auto_entry_permission,
                                          self.adapter.get_mode(), self.auto_mode):
                self._state_changed()
        if self.fsm.state in (States.RTL, States.USER_RETURN):
            self._handle_normal_rtl(now)
        if (self.fsm.state in (States.RTL, States.USER_RETURN) and
                self.normal_rtl.confirmed and
                self.adapter.relative_alt is not None and self.adapter.relative_alt <= 0.5):
            self.fsm.landed(); self._state_changed()
        if self.fsm.state == States.LAND and not self.adapter.state.armed:
            self.fsm.landed(); self._state_changed()
        if (self.return_explicit and not self.adapter.state.armed and
                self.fsm.state in (States.LAND, States.FINISHED) and
                not self.return_clear_done):
            try:
                self.return_clear_done = bool(self.adapter.clear_mission())
            except rospy.ServiceException:
                self.return_clear_done = False
            self.set_return_status(
                'LANDED',
                'RETURN_COMPLETE_MISSION_CLEARED' if self.return_clear_done
                else 'RETURN_COMPLETE_MISSION_CLEAR_FAILED')
        limit = self.timeouts.get(self.fsm.state)
        if limit and now-self.state_since > limit:
            if self.fsm.state in (States.TAKEOFF, States.TRANSIT, States.FEEDING,
                                  States.EXIT, States.RTL, States.USER_RETURN,
                                  States.LAND):
                if self.return_explicit:
                    self.set_return_status('FAILED', self.fsm.state+'_TIMEOUT')
                self.fsm.timeout_abort(self.fsm.state+'_TIMEOUT'); self._enter_timeout_abort(now); self._state_changed()
            else:
                self.fsm.failsafe(self.fsm.state+'_TIMEOUT'); self.command_feeder(False,'MISSION_TIMEOUT'); self._state_changed()
        if self.enable_feeding and self.fsm.feeder_enabled and not self.feeder_status and now-self.feeder_command_time > 2.0:
            self.fsm.failsafe('FEEDER_FAILURE'); self.command_feeder(False,'FEEDER_FAILURE'); self._state_changed()
        if self.fsm.state == States.FEEDING_DWELL and self.dwell_started_at and now-self.dwell_started_at >= self.circular_dwell_sec:
            if self.fsm.feeding_dwell_complete():
                rospy.loginfo('[CIRCULAR] target dwell complete; feeding=%s -> %s', self.enable_feeding, self.fsm.state)
                self.command_feeder(False, 'CIRCULAR_DWELL_COMPLETE')
                self._state_changed()
        self._handle_timeout_recovery(now)
        self.publish_state()

    def _maybe_begin_circular_dwell(self, now):
        if self.geometry_mode != 'circular' or self.fsm.state not in (States.TAKEOFF, States.TRANSIT):
            return
        feed_index = next((i for i,w in enumerate(self.waypoints) if w.waypoint_type == 'FEED_POINT'), None)
        if feed_index is None or self.current_gps is None:
            return
        fcu_feed_seq = feed_index + self.adapter.mission_seq_offset
        seq_fallback = (self.circular_seq_fallback_enabled and
                        self.adapter.current_mission_seq == fcu_feed_seq)
        reached_progress = self.fsm.current_waypoint >= feed_index - 1
        if not reached_progress and not seq_fallback:
            return
        # current_seq means the FCU is targeting the feed item, not necessarily
        # that it has arrived.  Never enable the feeder from sequence alone:
        # require live AUTO flight, fresh UAV GPS, valid cage geometry, and
        # independent horizontal/vertical proximity checks.
        if seq_fallback:
            if (not self.adapter.state.armed or
                    self.adapter.get_mode() != self.auto_mode or
                    now-self.last_gps > self.gps_timeout or
                    (self.beacon_required_for_active_task() and not self.beacon_current_valid)):
                return
        waypoint = self.waypoints[feed_index]
        local = LocalENU(self.current_gps.latitude, self.current_gps.longitude, self.current_gps.altitude)
        east, north, _ = local.to_enu(waypoint.lat, waypoint.lon, self.current_gps.altitude)
        radius = (max(waypoint.acceptance_radius, self.circular_fallback_radius)
                  if seq_fallback else waypoint.acceptance_radius)
        altitude_tolerance = (self.circular_fallback_altitude_tolerance
                              if seq_fallback else self.circular_altitude_tolerance)
        altitude_ready = (self.adapter.relative_alt is not None and
                          abs(self.adapter.relative_alt-waypoint.alt) <= altitude_tolerance)
        distance = math.hypot(east,north)
        if distance <= radius and altitude_ready:
            if self.fsm.feed_point_arrived(feed_index, enable_feeding=self.enable_feeding):
                source = ('FCU_CURRENT_SEQ_PROXIMITY' if seq_fallback
                          else 'ACCEPTANCE_RADIUS')
                rospy.logwarn('[CIRCULAR] feed fallback accepted: source=%s '
                              'fcu_seq=%d expected=%d distance=%.2fm/%.2fm '
                              'relative_alt=%.2fm target=%.2fm tolerance=%.2fm',
                              source, self.adapter.current_mission_seq, fcu_feed_seq,
                              distance, radius, self.adapter.relative_alt,
                              waypoint.alt, altitude_tolerance)
                self._begin_circular_dwell(feed_index, source)
                self._state_changed()
        elif seq_fallback:
            rospy.logwarn_throttle(2.0,
                '[CIRCULAR] feed fallback waiting: fcu_seq=%d distance=%.2fm/%.2fm '
                'relative_alt=%s target=%.2fm tolerance=%.2fm',
                self.adapter.current_mission_seq, distance, radius,
                'UNKNOWN' if self.adapter.relative_alt is None else
                '{:.2f}m'.format(self.adapter.relative_alt),
                waypoint.alt, altitude_tolerance)

    def _maybe_progress_grid_mission(self, now):
        """Guarded K9 fallback for grid/photo missions without REACHED events.

        ACFly can advance WaypointList.current_seq without publishing
        MISSION_ITEM_REACHED.  A sequence advance proves earlier items passed;
        the currently targeted CELL_TARGET/GRID_EXIT is accepted only with
        independent GPS and altitude proximity checks.
        """
        if (not self.waypoints or
                not any(w.waypoint_type == 'CELL_TARGET' for w in self.waypoints) or
                self.fsm.state not in (States.TAKEOFF, States.TRANSIT, States.FEEDING, States.EXIT)):
            return
        if (not self.adapter.state.armed or self.adapter.get_mode() != self.auto_mode or
                self.current_gps is None or now-self.last_gps > self.gps_timeout or
                (self.beacon_required_for_active_task() and not self.beacon_current_valid)):
            return
        current = self.adapter.current_mission_seq-self.adapter.mission_seq_offset
        if current < 0 or current >= len(self.waypoints):
            return
        # current_seq targeting item N means every prior item has completed.
        for index in range(self.fsm.current_waypoint+1, current):
            old_feeder = self.fsm.feeder_enabled
            waypoint = self.waypoints[index]
            self.fsm.waypoint_reached(index, waypoint.waypoint_type, waypoint.cell_id,
                                      enable_feeding=False)
            if self.fsm.feeder_enabled != old_feeder:
                self.command_feeder(self.fsm.feeder_enabled,
                                    'SEQ_ADVANCE_'+waypoint.waypoint_type)
            self._state_changed()
        if current <= self.fsm.current_waypoint:
            return
        waypoint = self.waypoints[current]
        if waypoint.waypoint_type not in ('CELL_TARGET', 'GRID_EXIT'):
            return
        local = LocalENU(self.current_gps.latitude, self.current_gps.longitude,
                         self.current_gps.altitude)
        east, north, _ = local.to_enu(waypoint.lat, waypoint.lon,
                                      self.current_gps.altitude)
        distance = math.hypot(east, north)
        altitude_tolerance = max(3.0, self.circular_altitude_tolerance)
        altitude_ready = (self.adapter.relative_alt is not None and
                          abs(self.adapter.relative_alt-waypoint.alt) <= altitude_tolerance)
        if distance <= waypoint.acceptance_radius and altitude_ready:
            old_feeder = self.fsm.feeder_enabled
            self.fsm.waypoint_reached(current, waypoint.waypoint_type,
                                      waypoint.cell_id,
                                      enable_feeding=self.enable_feeding)
            if self.fsm.feeder_enabled != old_feeder:
                self.command_feeder(self.fsm.feeder_enabled,
                                    'PROXIMITY_'+waypoint.waypoint_type)
            if waypoint.waypoint_type == 'CELL_TARGET':
                self._begin_grid_cell(current, 'FCU_CURRENT_SEQ_PROXIMITY')
            rospy.logwarn('[GRID] waypoint fallback accepted: index=%d type=%s '
                          'distance=%.2fm/%.2fm relative_alt=%.2fm target=%.2fm',
                          current, waypoint.waypoint_type, distance,
                          waypoint.acceptance_radius, self.adapter.relative_alt,
                          waypoint.alt)
            self._state_changed()

    def _begin_grid_cell(self, mission_index, source):
        if mission_index == self.grid_cell_index:
            return
        self.grid_cell_index = mission_index
        self.grid_cell_started_at = time.monotonic()
        waypoint = self.waypoints[mission_index]
        rospy.loginfo('[GRID] cell dwell started: index=%d cell=%s source=%s '
                      'feed=%.1fs hold=%.1fs final=%s',
                      mission_index, waypoint.cell_id, source,
                      self.grid_feeding_duration_sec, waypoint.hold_time,
                      mission_index == len(self.waypoints)-1)

    def _update_grid_timers(self, now):
        if (self.grid_cell_started_at <= 0.0 or self.grid_cell_index < 0 or
                self.grid_cell_index >= len(self.waypoints) or
                self.fsm.state != States.FEEDING):
            return
        waypoint = self.waypoints[self.grid_cell_index]
        if waypoint.waypoint_type != 'CELL_TARGET':
            return
        elapsed = now-self.grid_cell_started_at
        if (self.fsm.feeder_enabled and
                elapsed >= self.grid_feeding_duration_sec):
            self.command_feeder(False, 'GRID_FEED_DURATION_COMPLETE')
            rospy.loginfo('[GRID] feeder duration complete: index=%d cell=%s '
                          'elapsed=%.1fs/%.1fs', self.grid_cell_index,
                          waypoint.cell_id, elapsed,
                          self.grid_feeding_duration_sec)
            self._state_changed()
        if (self.grid_cell_index == len(self.waypoints)-1 and
                elapsed >= waypoint.hold_time):
            if self.fsm.feeder_enabled:
                self.command_feeder(False, 'GRID_FINAL_CELL_COMPLETE')
            if self.fsm.grid_complete():
                rospy.loginfo('[GRID] final cell dwell complete: index=%d '
                              'elapsed=%.1fs; requesting %s',
                              self.grid_cell_index, elapsed, self.rtl_mode)
                self.grid_cell_started_at = 0.0
                self._state_changed()

    def _begin_circular_dwell(self, mission_index, source):
        if self.dwell_started_at:
            return
        self.dwell_started_at = time.monotonic()
        self.circular_dwell_sec = float(self.waypoints[mission_index].hold_time or self.circular_dwell_sec)
        rospy.loginfo('[CIRCULAR] target reached via %s; feeding=%s; dwell %.1f sec',
                      source, self.enable_feeding, self.circular_dwell_sec)
        if self.enable_feeding:
            self.command_feeder(True, 'FEED_POINT')

    def _enter_timeout_abort(self, now):
        self.recovery_entry_time = now
        self.last_recovery_rtl_request = 0.0
        self.recovery_rtl_attempts = 0
        self.recovery_stage = 'FEEDER_OFF'
        self.command_feeder(False, 'TIMEOUT_ABORT')

    def _handle_normal_rtl(self, now):
        action = self.normal_rtl.step(now, self.adapter.get_mode(), self.rtl_mode)
        if action == RtlRequestController.REQUEST:
            self.rtl_attempted = True
            accepted = self.adapter.request_rtl(self.rtl_mode)
            self.fsm.reason = 'RTL_REQUESTED_{}_{}'.format(
                self.normal_rtl.attempts, 'ACCEPTED' if accepted else 'REJECTED')
            rospy.logwarn('RTL request %d/%d: mode=%s service_accepted=%s; waiting for FC mode confirmation',
                          self.normal_rtl.attempts, self.rtl_retries, self.rtl_mode, accepted)
            if self.return_explicit:
                self.set_return_status(
                    'REQUESTED',
                    'RTL_REQUEST_{}_{}'.format(
                        self.normal_rtl.attempts,
                        'ACCEPTED' if accepted else 'REJECTED'))
        elif (action == RtlRequestController.CONFIRMED and
              self.fsm.reason not in ('RTL_MODE_CONFIRMED',
                                      'USER_RETURN_RTL_CONFIRMED')):
            self.fsm.reason = ('USER_RETURN_RTL_CONFIRMED'
                               if self.return_explicit else 'RTL_MODE_CONFIRMED')
            rospy.loginfo('RTL confirmed by /mavros/state: %s', self.rtl_mode)
            if self.return_explicit:
                self.set_return_status('RTL_CONFIRMED', 'FCU_MODE_AUTO_RTL')
        elif action == RtlRequestController.FAILED:
            self.fsm.transition(States.ERROR, 'RTL_MODE_NOT_CONFIRMED_AFTER_{}_ATTEMPTS'.format(self.rtl_retries))
            if self.return_explicit:
                self.set_return_status('FAILED', self.fsm.reason)
            self._state_changed()

    def _handle_timeout_recovery(self, now):
        if not self.automatic_control_allowed('timeout recovery'):
            return
        if self.fsm.state == States.TIMEOUT_ABORT:
            if self.recovery_stage != 'FEEDER_OFF':
                self.recovery_stage = 'FEEDER_OFF'
                self.recovery_entry_time = now
                self.command_feeder(False, 'TIMEOUT_ABORT')
            if self.feeder_status is False or now - self.recovery_entry_time >= self.feeder_off_timeout:
                self.fsm.recovery_start()
                self.recovery_stage = 'RTL_REQUEST'
                self._state_changed()
        elif self.fsm.state == States.RECOVERY_RTL:
            if now - self.last_recovery_rtl_request >= self.rtl_request_interval:
                if self.recovery_rtl_attempts >= self.rtl_retries:
                    self.fsm.failsafe('RTL_REQUEST_FAILED')
                    self._state_changed()
                    return
                self.last_recovery_rtl_request = now
                self.recovery_rtl_attempts += 1
                if self.adapter.request_rtl(self.rtl_mode):
                    self.fsm.transition(States.RECOVERY_RTL_VERIFY, 'RTL_REQUESTED')
                    self.recovery_stage = 'RTL_VERIFY'
                    self._state_changed()
                elif self.recovery_rtl_attempts >= self.rtl_retries:
                    self.fsm.failsafe('RTL_REQUEST_FAILED')
                    self._state_changed()
        elif self.fsm.state == States.RECOVERY_RTL_VERIFY:
            if self.adapter.is_rtl_mode(self.rtl_mode):
                self.fsm.recovery_rtl_verified()
                self.recovery_stage = 'RETURNING'
                self._state_changed()
            elif now - self.state_since > self.rtl_confirm_timeout:
                self.fsm.transition(States.RECOVERY_RTL, 'RTL_CONFIRM_TIMEOUT')
                self._state_changed()
        elif self.fsm.state == States.RECOVERY_RETURNING:
            if self.adapter.relative_alt is not None and self.adapter.relative_alt <= 0.5:
                self.fsm.recovery_landing()
                self.recovery_stage = 'LANDING'
                self._state_changed()
        elif self.fsm.state == States.RECOVERY_LANDING:
            if self.adapter.is_landed():
                self.fsm.recovery_landed()
                self.recovery_stage = 'LANDED'
                self._state_changed()

    def _state_changed(self):
        if self.fsm.state != self.last_state:
            if self.fsm.state in (States.RTL, States.USER_RETURN):
                self.normal_rtl.reset(time.monotonic())
                self.rtl_attempted = False
            rospy.loginfo('FSM %s -> %s (%s)', self.last_state, self.fsm.state, self.fsm.reason)
            self.last_state, self.state_since = self.fsm.state, time.monotonic()
        self.publish_state()

    def publish_state(self):
        self.state_pub.publish(MissionState(header=Header(stamp=rospy.Time.now()), mission_id=self.fsm.mission_id,
            state=self.fsm.state, reason=self.fsm.reason, current_waypoint=self.fsm.current_waypoint,
            feeder_enabled=self.fsm.feeder_enabled, manual_override=self.fsm.manual_override_latched,
            current_cell_id=self.fsm.current_cell_id))


if __name__ == '__main__':
    rospy.init_node('mission_fsm_node'); MissionFSMNode(); rospy.spin()

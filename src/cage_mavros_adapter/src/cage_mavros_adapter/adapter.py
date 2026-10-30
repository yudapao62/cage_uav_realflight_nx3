"""The only project layer allowed to call MAVROS APIs."""
import math
import threading
import rospy
from mavros_msgs.msg import State, Waypoint, WaypointList
from mavros_msgs.srv import CommandBool, SetMode, StreamRate, WaypointClear, WaypointPull, WaypointPush, WaypointSetCurrent
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


class MavrosAdapter:
    def __init__(self, namespace='/mavros'):
        self.ns = namespace.rstrip('/')
        self.state, self.global_position, self.relative_alt = State(), None, None
        self.waypoints = []
        # ACFly/K9 advances WaypointList.current_seq but does not emit
        # MISSION_ITEM_REACHED on /mavros/mission/reached.  Keep the FCU-side
        # sequence so the mission FSM can use it as a guarded fallback.
        self.current_mission_seq = -1
        self._waypoint_event = threading.Event()
        self.last_upload_error = ''
        self.last_upload_count = 0
        self.upload_verified = False
        self.mission_home_item_required = rospy.get_param('/vehicle/mission_home_item_required', True)
        self.mission_seq_offset = 1 if self.mission_home_item_required else 0
        # Keep the FCU at FEED_POINT longer than the FSM's feeder dwell.  This
        # gives the FSM time to close the feeder and request RTL while the FCU
        # is still in AUTO.MISSION instead of completing the final waypoint.
        self.feed_point_fcu_hold_sec = float(
            rospy.get_param('/cage/circular/fcu_feed_point_hold_sec', 20.0))
        self.final_cell_rtl_guard_sec = float(
            rospy.get_param('/mission/final_cell_rtl_guard_sec', 5.0))
        if not math.isfinite(self.feed_point_fcu_hold_sec) or self.feed_point_fcu_hold_sec < 0.0:
            raise ValueError('/cage/circular/fcu_feed_point_hold_sec must be finite and non-negative')
        if (not math.isfinite(self.final_cell_rtl_guard_sec) or
                self.final_cell_rtl_guard_sec < 1.0):
            raise ValueError('/mission/final_cell_rtl_guard_sec must be at least 1.0 s')
        rospy.Subscriber(self.ns+'/state', State, self._on_state, queue_size=1)
        rospy.Subscriber(self.ns+'/global_position/global', NavSatFix, self._on_gps, queue_size=1)
        rospy.Subscriber(self.ns+'/global_position/rel_alt', Float64, self._on_rel_alt, queue_size=1)
        rospy.Subscriber(self.ns+'/mission/waypoints', WaypointList, self._on_waypoints, queue_size=1)
        # TODO_K9_VERIFY: K9 support for these standard MAVROS/MAVLink services must be bench-tested.
        self._arm = rospy.ServiceProxy(self.ns+'/cmd/arming', CommandBool)
        self._mode = rospy.ServiceProxy(self.ns+'/set_mode', SetMode)
        self._clear = rospy.ServiceProxy(self.ns+'/mission/clear', WaypointClear)
        self._push = rospy.ServiceProxy(self.ns+'/mission/push', WaypointPush)
        self._pull = rospy.ServiceProxy(self.ns+'/mission/pull', WaypointPull)
        self._set_current = rospy.ServiceProxy(self.ns+'/mission/set_current', WaypointSetCurrent)
        self._stream_rate = rospy.ServiceProxy(self.ns+'/set_stream_rate', StreamRate)

    def _on_state(self, msg): self.state = msg
    def _on_gps(self, msg): self.global_position = msg
    def _on_rel_alt(self, msg): self.relative_alt = msg.data
    def _on_waypoints(self, msg):
        self.waypoints = list(msg.waypoints)
        self.current_mission_seq = int(msg.current_seq)
        self._waypoint_event.set()

    def is_connected(self): return bool(self.state.connected)
    def get_mode(self): return self.state.mode
    def get_current_mode(self): return self.state.mode
    def get_global_position(self): return self.global_position
    def get_relative_altitude(self): return self.relative_alt
    def arm(self): return bool(self._arm(True).success)
    def disarm(self): return bool(self._arm(False).success)
    def set_mode(self, mode): return bool(self._mode(0, mode).mode_sent)
    def rtl(self, mode='AUTO.RTL'): return self.set_mode(mode)
    def request_rtl(self, mode='AUTO.RTL'): return self.set_mode(mode)
    def is_rtl_mode(self, mode='AUTO.RTL'): return self.state.mode == mode
    def is_landed(self):
        return not self.state.armed and self.relative_alt is not None and self.relative_alt <= 0.5
    def request_stream_rate(self, hz=10):
        # TODO_K9_VERIFY: K9 telemetry stream-rate handling may differ from ArduPilot.
        try:
            self._stream_rate(stream_id=StreamRate._request_class.STREAM_ALL,
                              message_rate=hz, on_off=True)
            return True
        except rospy.ServiceException:
            return False
    def start_mission(self, auto_mode='AUTO'):
        # TODO_K9_VERIFY: K9 mission/set_current support and sequence convention require bench testing.
        if not self.set_mission_current(): return False
        return self.set_mode(auto_mode)
    def set_mission_current(self):
        """Prepare the first executable mission item without changing flight mode."""
        try:
            return bool(self._set_current(wp_seq=self.mission_seq_offset).success)
        except rospy.ServiceException as exc:
            rospy.logerr('Mission set_current failed: %s', exc)
            return False
    def clear_mission(self): return bool(self._clear().success)

    @staticmethod
    def to_mavros_waypoint(item, feed_point_fcu_hold_sec=None,
                           hold_extension_sec=0.0):
        wp = Waypoint()
        wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp.command = 22 if item.waypoint_type == 'TAKEOFF' else 16
        wp.is_current = item.seq == 0
        wp.autocontinue = True
        # NAV_WAYPOINT param1 is the FCU-side hold time.  The task waypoint's
        # hold_time remains the FSM feeder dwell; FEED_POINT can deliberately
        # use a longer FCU hold so RTL is requested before mission completion.
        base_hold = (float(feed_point_fcu_hold_sec)
                     if item.waypoint_type == 'FEED_POINT' and feed_point_fcu_hold_sec is not None
                     else float(item.hold_time))
        wp.param1 = base_hold + float(hold_extension_sec)
        wp.param2 = item.acceptance_radius
        wp.param3 = 0.0
        wp.param4 = item.yaw
        wp.x_lat, wp.y_long, wp.z_alt = item.lat, item.lon, item.alt
        return wp

    def upload_mission(self, items, timeout=10.0):
        """Clear, push, pull, compare and select the mission; never arm on failure."""
        self.last_upload_error = ''
        self.last_upload_count = 0
        self.upload_verified = False
        if not items:
            self.last_upload_error = 'EMPTY_MISSION'
            return False
        final_cell_index = (len(items)-1
                            if items[-1].waypoint_type == 'CELL_TARGET' else -1)
        outbound = [self.to_mavros_waypoint(
            item, self.feed_point_fcu_hold_sec,
            self.final_cell_rtl_guard_sec if index == final_cell_index else 0.0)
            for index, item in enumerate(items)]
        if self.mission_home_item_required:
            if self.global_position is None:
                self.last_upload_error = 'FCU_HOME_POSITION_UNAVAILABLE'
                return False
            home = Waypoint(frame=Waypoint.FRAME_GLOBAL, command=16, is_current=True, autocontinue=True,
                            x_lat=self.global_position.latitude, y_long=self.global_position.longitude,
                            z_alt=self.global_position.altitude)
            for wp in outbound: wp.is_current = False
            outbound.insert(0, home)
        try:
            if not self.clear_mission():
                self.last_upload_error = 'MISSION_CLEAR_FAILED'
                return False
            rospy.loginfo('Mission clear verified; uploading %d FCU items', len(outbound))
            response = self._push(start_index=0, waypoints=outbound)
            if not response.success or response.wp_transfered != len(outbound):
                self.last_upload_error = 'MISSION_PUSH_INCOMPLETE_{}/{}'.format(
                    response.wp_transfered, len(outbound))
                self._clear_after_failed_upload()
                return False
            self._waypoint_event.clear()
            pulled = self._pull()
            if not pulled.success or pulled.wp_received != len(outbound):
                self.last_upload_error = 'MISSION_PULL_COUNT_{}/{}'.format(
                    pulled.wp_received, len(outbound))
                self._clear_after_failed_upload()
                return False
            if not self._waypoint_event.wait(timeout):
                self.last_upload_error = 'MISSION_WAYPOINT_LIST_TIMEOUT'
                self._clear_after_failed_upload()
                return False
            if not self._same_mission(outbound, self.waypoints, self.mission_home_item_required):
                self.last_upload_error = 'MISSION_CONTENT_MISMATCH'
                self._clear_after_failed_upload()
                return False
            if not self.set_mission_current():
                self.last_upload_error = 'MISSION_SET_CURRENT_FAILED'
                self._clear_after_failed_upload()
                return False
            self.last_upload_count = len(outbound)
            self.upload_verified = True
            rospy.loginfo('Mission upload verified: %d FCU items; current seq=%d',
                          self.last_upload_count, self.mission_seq_offset)
            return True
        except rospy.ServiceException as exc:
            self.last_upload_error = 'MISSION_SERVICE_ERROR:{}'.format(exc)
            self._clear_after_failed_upload()
            return False

    def _clear_after_failed_upload(self):
        """Best-effort removal prevents a partial/stale mission being flown manually."""
        try:
            if not self.clear_mission():
                rospy.logerr('Failed upload cleanup: FCU mission clear was rejected')
        except rospy.ServiceException as exc:
            rospy.logerr('Failed upload cleanup: %s', exc)

    def download_mission(self):
        self._waypoint_event.clear()
        response = self._pull()
        if response.success: self._waypoint_event.wait(5.0)
        return list(self.waypoints) if response.success else []

    @staticmethod
    def _same_mission(expected, actual, skip_home=False):
        if len(expected) != len(actual): return False
        start = 1 if skip_home else 0
        for lhs, rhs in zip(expected[start:], actual[start:]):
            if lhs.command != rhs.command or not math.isclose(lhs.x_lat, rhs.x_lat, abs_tol=2e-6) or \
               not math.isclose(lhs.y_long, rhs.y_long, abs_tol=2e-6) or \
               not math.isclose(lhs.z_alt, rhs.z_alt, abs_tol=0.2) or \
               not math.isclose(lhs.param1, rhs.param1, abs_tol=0.1): return False
        return True

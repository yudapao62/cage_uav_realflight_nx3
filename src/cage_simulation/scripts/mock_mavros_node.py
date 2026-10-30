#!/usr/bin/env python3
"""ARM-safe MAVROS transport stub for Jetson simulation smoke tests.

It exercises the Cage task/FSM message and service interfaces without claiming
to be a flight controller. The real vehicle launch always uses MAVROS/K9.
"""
import threading
import rospy
from mavros_msgs.msg import State, WaypointList, WaypointReached
from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse
from mavros_msgs.srv import StreamRate, StreamRateResponse, WaypointClear, WaypointClearResponse
from mavros_msgs.srv import WaypointPull, WaypointPullResponse, WaypointPush, WaypointPushResponse
from mavros_msgs.srv import WaypointSetCurrent, WaypointSetCurrentResponse
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_msgs.msg import Float64


class MockMavros:
    def __init__(self):
        self.state = State(connected=True, armed=False, mode='STABILIZE')
        self.lat = float(rospy.get_param('~latitude', -35.363261))
        self.lon = float(rospy.get_param('~longitude', 149.165230))
        self.alt = float(rospy.get_param('~altitude', 584.0)); self.home_alt = self.alt
        self.relative_alt = 0.0; self.waypoints = []; self.current = 0; self.last_reached = -1
        self.arrival_at = None
        self.lock = threading.RLock()
        self.state_pub = rospy.Publisher('/mavros/state', State, queue_size=1, latch=True)
        self.gps_pub = rospy.Publisher('/mavros/global_position/global', NavSatFix, queue_size=1, latch=True)
        self.rel_pub = rospy.Publisher('/mavros/global_position/rel_alt', Float64, queue_size=1, latch=True)
        self.wp_pub = rospy.Publisher('/mavros/mission/waypoints', WaypointList, queue_size=1, latch=True)
        self.reached_pub = rospy.Publisher('/mavros/mission/reached', WaypointReached, queue_size=1)
        self.battery_pub = rospy.Publisher('/mavros/battery', BatteryState, queue_size=1, latch=True)
        rospy.Service('/mavros/cmd/arming', CommandBool, self.arm)
        rospy.Service('/mavros/set_mode', SetMode, self.mode)
        rospy.Service('/mavros/mission/clear', WaypointClear, self.clear)
        rospy.Service('/mavros/mission/push', WaypointPush, self.push)
        rospy.Service('/mavros/mission/pull', WaypointPull, self.pull)
        rospy.Service('/mavros/mission/set_current', WaypointSetCurrent, self.set_current)
        rospy.Service('/mavros/set_stream_rate', StreamRate, lambda _req: StreamRateResponse())
        rospy.Timer(rospy.Duration(0.2), self.tick)

    def arm(self, req):
        with self.lock: self.state.armed = bool(req.value)
        return CommandBoolResponse(success=True, result=0)

    def mode(self, req):
        with self.lock: self.state.mode = req.custom_mode
        return SetModeResponse(mode_sent=True)

    def clear(self, _req):
        with self.lock:
            self.waypoints = []; self.current = 0; self.last_reached = -1; self.arrival_at = None
        self.wp_pub.publish(WaypointList())
        return WaypointClearResponse(success=True)

    def push(self, req):
        with self.lock:
            self.waypoints = list(req.waypoints); self.current = 0; self.last_reached = -1; self.arrival_at = None
        self.wp_pub.publish(WaypointList(waypoints=self.waypoints, current_seq=0))
        return WaypointPushResponse(success=True, wp_transfered=len(self.waypoints))

    def pull(self, _req):
        return WaypointPullResponse(success=True, wp_received=len(self.waypoints))

    def set_current(self, req):
        self.current = int(req.wp_seq); self.arrival_at = None
        return WaypointSetCurrentResponse(success=True)

    def tick(self, _event):
        stamp = rospy.Time.now()
        if self.state.armed and self.state.mode == 'AUTO' and self.waypoints and self.current < len(self.waypoints):
            wp = self.waypoints[self.current]
            self.lat, self.lon = wp.x_lat, wp.y_long
            self.relative_alt = float(wp.z_alt)
            self.alt = self.home_alt + self.relative_alt
            if self.arrival_at is None:
                self.arrival_at = stamp.to_sec()
            if stamp.to_sec() - self.arrival_at >= max(0.0, float(wp.param1)):
                self.last_reached = self.current
                self.reached_pub.publish(WaypointReached(wp_seq=self.current))
                self.current += 1
                self.arrival_at = None
        if self.state.mode == 'RTL' and self.waypoints:
            self.relative_alt = 0.0; self.alt = self.home_alt; self.state.armed = False
        gps = NavSatFix(); gps.header.stamp = stamp; gps.status.status = NavSatStatus.STATUS_FIX
        gps.status.service = NavSatStatus.SERVICE_GPS; gps.latitude = self.lat; gps.longitude = self.lon; gps.altitude = self.alt
        self.state_pub.publish(self.state); self.gps_pub.publish(gps); self.rel_pub.publish(Float64(self.relative_alt))
        self.battery_pub.publish(BatteryState(percentage=1.0))


if __name__ == '__main__':
    rospy.init_node('mock_mavros')
    MockMavros()
    rospy.spin()

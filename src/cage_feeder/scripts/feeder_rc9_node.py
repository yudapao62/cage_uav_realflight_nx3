#!/usr/bin/env python3
"""Drive the real feeder through FCU PWM output 9."""
import math
import threading
import time

import rospy
from cage_feeder.rc9_logic import FeederCycle
from cage_msgs.msg import FeederCommand
from cage_msgs.srv import RunFeeder, RunFeederResponse
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Bool


class FeederPWMNode:
    MAV_CMD_DO_SET_SERVO = 183

    def __init__(self):
        prefix = '/feeding/'
        legacy_channel = int(rospy.get_param(prefix + 'rc_channel', 9))
        legacy_pwm_on = int(rospy.get_param(prefix + 'rc_pwm_on', 1841))
        legacy_pwm_off = int(rospy.get_param(prefix + 'rc_pwm_off', 1137))
        self.channel = int(rospy.get_param(prefix + 'pwm_output_channel', legacy_channel))
        self.pwm_on = int(rospy.get_param(prefix + 'pwm_on', legacy_pwm_on))
        self.pwm_off = int(rospy.get_param(prefix + 'pwm_off', legacy_pwm_off))
        self.update_rate = float(rospy.get_param(prefix + 'command_update_rate_hz', 20.0))
        self.status_rate = float(rospy.get_param(prefix + 'status_publish_rate_hz', 5.0))
        self.close_hold_sec = float(rospy.get_param(prefix + 'close_hold_sec', 1.0))
        self.max_duration_sec = float(rospy.get_param(prefix + 'max_run_duration_sec', 300.0))
        self.require_fcu = bool(rospy.get_param(prefix + 'require_fcu_connection', True))
        self._validate_config()

        self.lock = threading.RLock()
        self.cycle = FeederCycle(self.close_hold_sec)
        self.fcu_connected = False
        self.last_status = None
        self.last_status_publish = 0.0
        self.pending_off = False

        self.command_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        self.status_pub = rospy.Publisher('/feeder/status', Bool, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher('/feeder/state', Bool, queue_size=1, latch=True)
        rospy.Subscriber('/mavros/state', State, self.on_fcu_state, queue_size=1)
        rospy.Subscriber('/feeder/command', FeederCommand, self.on_command, queue_size=5)
        self.service = rospy.Service('/feeder/run_for_duration', RunFeeder, self.on_run)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.tick)
        rospy.on_shutdown(self.on_shutdown)
        self.publish_status(False, force=True)
        rospy.loginfo(
            'PWM feeder ready: output=%d on=%d off=%d max_duration=%.1fs',
            self.channel, self.pwm_on, self.pwm_off, self.max_duration_sec)

    def _validate_config(self):
        if not 1 <= self.channel <= 16:
            raise ValueError('feeding/pwm_output_channel must be in [1, 16]')
        if not 800 <= self.pwm_off <= 2200 or not 800 <= self.pwm_on <= 2200:
            raise ValueError('feeder PWM values must be in [800, 2200]')
        if self.pwm_on == self.pwm_off:
            raise ValueError('feeder ON and OFF PWM values must differ')
        if not math.isfinite(self.update_rate) or self.update_rate <= 0.0:
            raise ValueError('feeding/command_update_rate_hz must be positive')
        if not math.isfinite(self.status_rate) or self.status_rate <= 0.0:
            raise ValueError('feeding/status_publish_rate_hz must be positive')
        if not math.isfinite(self.close_hold_sec) or self.close_hold_sec <= 0.0:
            raise ValueError('feeding/close_hold_sec must be positive')
        if not math.isfinite(self.max_duration_sec) or self.max_duration_sec <= 0.0:
            raise ValueError('feeding/max_run_duration_sec must be positive')

    def on_fcu_state(self, message):
        with self.lock:
            was_connected = self.fcu_connected
            self.fcu_connected = bool(message.connected)
            if not was_connected and self.fcu_connected:
                # Establish a known safe output after every MAVROS reconnect.
                self.pending_off = True
            if was_connected and not self.fcu_connected and self.cycle.enabled:
                self.cycle.begin_closing(time.monotonic())
                self.pending_off = True
                self.publish_status(False)
                rospy.logerr('FCU disconnected while feeder was ON; command cancelled')

    def set_pwm(self, value):
        try:
            response = self.command_service(
                broadcast=False,
                command=self.MAV_CMD_DO_SET_SERVO,
                confirmation=0,
                param1=float(self.channel),
                param2=float(value),
                param3=0.0,
                param4=0.0,
                param5=0.0,
                param6=0.0,
                param7=0.0)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logerr('Failed to set PWM%d=%d: %s', self.channel, value, exc)
            return False
        if not response.success or response.result != 0:
            rospy.logerr('FCU rejected PWM%d=%d: success=%s result=%d',
                         self.channel, value, response.success, response.result)
            return False
        return True

    def on_run(self, request):
        duration = float(request.duration_sec)
        if not math.isfinite(duration) or duration <= 0.0 or duration > self.max_duration_sec:
            return RunFeederResponse(
                success=False,
                message='duration_sec must be > 0 and <= {:.1f}'.format(self.max_duration_sec))
        with self.lock:
            if self.require_fcu and not self.fcu_connected:
                return RunFeederResponse(success=False, message='FCU is not connected')
            if not self.set_pwm(self.pwm_on):
                return RunFeederResponse(success=False, message='FCU rejected feeder ON command')
            self.pending_off = False
            self.cycle.start_timed(duration, time.monotonic())
            self.publish_status(True)
        rospy.logwarn('Feeder ON for %.3f seconds (PWM%d=%d)',
                      duration, self.channel, self.pwm_on)
        return RunFeederResponse(
            success=True,
            message='feeder ON; automatic OFF in {:.3f} seconds'.format(duration))

    def on_command(self, message):
        with self.lock:
            enable = bool(message.enable)
            if enable and self.require_fcu and not self.fcu_connected:
                self.cycle.begin_closing(time.monotonic())
                self.pending_off = True
                self.publish_status(False)
                rospy.logerr('Feeder ON rejected: FCU is not connected (%s)', message.reason)
                return
            target_pwm = self.pwm_on if enable else self.pwm_off
            if not self.set_pwm(target_pwm):
                if enable:
                    self.cycle.begin_closing(time.monotonic())
                    self.pending_off = True
                    self.publish_status(False)
                return
            self.pending_off = False
            self.cycle.set_enabled(enable, time.monotonic())
            self.publish_status(enable)
        rospy.loginfo('Feeder %s from mission command: %s',
                      'ON' if enable else 'OFF', message.reason)

    def publish_status(self, enabled, force=False):
        enabled = bool(enabled)
        if force or enabled != self.last_status:
            self.last_status = enabled
            self.last_status_publish = time.monotonic()
            self.status_pub.publish(Bool(data=enabled))
            self.state_pub.publish(Bool(data=enabled))

    def tick(self, _event):
        with self.lock:
            now = time.monotonic()
            transition = self.cycle.update(now)
            state = self.cycle.state
            if transition and transition[1] == FeederCycle.CLOSING:
                self.pending_off = True
                self.publish_status(False)
                rospy.logwarn('Feeder timed run complete; setting PWM%d=%d',
                              self.channel, self.pwm_off)
            if now - self.last_status_publish >= 1.0 / self.status_rate:
                self.publish_status(state == FeederCycle.ON, force=True)
            if self.fcu_connected and self.pending_off and self.set_pwm(self.pwm_off):
                self.pending_off = False
                rospy.loginfo('Feeder OFF confirmed: PWM%d=%d', self.channel, self.pwm_off)

    def on_shutdown(self):
        try:
            if self.fcu_connected:
                self.set_pwm(self.pwm_off)
        except Exception:  # Shutdown is best effort and must not block process exit.
            pass


if __name__ == '__main__':
    rospy.init_node('feeder_rc9_node')
    FeederPWMNode()
    rospy.spin()

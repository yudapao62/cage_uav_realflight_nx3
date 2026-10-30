"""Deterministic safety policy for the mission FSM, independent of ROS transport."""
class States:
    INIT='INIT'; WAIT_FCU='WAIT_FCU'; WAIT_GPS='WAIT_GPS'; IDLE='IDLE'; PLAN='PLAN'
    UPLOAD_MISSION='UPLOAD_MISSION'; WAIT_ARM='WAIT_ARM'; TAKEOFF='TAKEOFF'
    TRANSIT='TRANSIT'; FEEDING='FEEDING'; FEEDING_DWELL='FEEDING_DWELL'; EXIT='EXIT'; RTL='RTL'; LAND='LAND'
    FINISHED='FINISHED'; MANUAL_OVERRIDE='MANUAL_OVERRIDE'; FAILSAFE='FAILSAFE'; ERROR='ERROR'
    TIMEOUT_ABORT='TIMEOUT_ABORT'; RECOVERY_RTL='RECOVERY_RTL'; RECOVERY_RTL_VERIFY='RECOVERY_RTL_VERIFY'
    RECOVERY_RETURNING='RECOVERY_RETURNING'; RECOVERY_LANDING='RECOVERY_LANDING'; RECOVERY_LANDED='RECOVERY_LANDED'


class RtlRequestController:
    """Side-effect-free retry/confirmation policy for normal mission RTL."""
    WAIT = 'WAIT'; REQUEST = 'REQUEST'; CONFIRMED = 'CONFIRMED'; FAILED = 'FAILED'

    def __init__(self, retries=3, request_interval=1.0, confirm_timeout=5.0):
        self.retries = max(1, int(retries))
        self.request_interval = max(0.0, float(request_interval))
        self.confirm_timeout = max(0.0, float(confirm_timeout))
        self.reset(0.0)

    def reset(self, now):
        self.attempts = 0
        self.last_request = float(now) - self.request_interval
        self.confirmed = False

    def step(self, now, current_mode, expected_mode):
        now = float(now)
        if current_mode == expected_mode:
            self.confirmed = True
            return self.CONFIRMED
        if self.confirmed:
            return self.CONFIRMED
        if self.attempts < self.retries and now - self.last_request >= self.request_interval:
            self.attempts += 1
            self.last_request = now
            return self.REQUEST
        if self.attempts >= self.retries and now - self.last_request >= self.confirm_timeout:
            return self.FAILED
        return self.WAIT


class MissionFSM:
    ACTIVE = {States.UPLOAD_MISSION, States.WAIT_ARM, States.TAKEOFF, States.TRANSIT,
              States.FEEDING, States.FEEDING_DWELL, States.EXIT, States.RTL, States.LAND}
    CONTROLLED = ACTIVE | {States.TIMEOUT_ABORT, States.RECOVERY_RTL, States.RECOVERY_RTL_VERIFY,
                            States.RECOVERY_RETURNING, States.RECOVERY_LANDING}
    NON_TAKEOVER_STATES = {States.INIT, States.WAIT_FCU, States.WAIT_GPS, States.IDLE}
    MANUAL_MODES = {'STABILIZE','LOITER','POSHOLD','POSCTL','ALT_HOLD','ALTCTL','MANUAL'}

    def __init__(self):
        self.state, self.reason = States.INIT, ''
        self.mission_id, self.current_waypoint = '', -1
        self.current_cell_id, self.rtl_after_mission = '', True
        self.feeder_enabled, self.auto_control_started = False, False
        self.mission_aborted, self.recovery_active = False, False
        self.manual_override_latched = False

    def transition(self, state, reason=''):
        self.state, self.reason = state, reason
        if state in (States.MANUAL_OVERRIDE, States.FAILSAFE, States.ERROR):
            self.feeder_enabled = False
        if state == States.FINISHED:
            self.recovery_active = False

    def initialize(self): self.transition(States.WAIT_FCU)
    def fcu_connected(self):
        if self.state == States.WAIT_FCU: self.transition(States.WAIT_GPS)
    def gps_valid(self):
        if self.state == States.WAIT_GPS: self.transition(States.IDLE)
    def request(self, mission_id, resume=False, rtl_after_mission=True):
        if self.manual_override_latched and not resume: return False
        if resume and not self.manual_override_latched: return False
        if self.state not in (States.IDLE, States.MANUAL_OVERRIDE, States.FINISHED): return False
        if resume:
            self.manual_override_latched = False
        self.mission_id, self.current_waypoint = mission_id, -1
        self.current_cell_id, self.rtl_after_mission = '', bool(rtl_after_mission)
        self.auto_control_started = False
        self.mission_aborted, self.recovery_active = False, False
        self.transition(States.PLAN, 'explicit resume' if resume else '')
        return True
    def planned(self):
        if self.state == States.PLAN: self.transition(States.UPLOAD_MISSION)
    def upload_result(self, success):
        if self.state == States.UPLOAD_MISSION:
            self.transition(States.WAIT_ARM if success else States.ERROR,
                            '' if success else 'MISSION_UPLOAD_FAILED')
    def armed_and_auto(self):
        if self.state == States.WAIT_ARM:
            self.auto_control_started = True; self.transition(States.TAKEOFF)
    def should_request_auto(self, armed, auto_permission=True):
        return bool(self.state == States.WAIT_ARM and armed and auto_permission and
                    not self.auto_control_started and self.automatic_control_allowed())
    def confirm_auto_mode(self, armed, mode, expected_mode):
        if self.state == States.WAIT_ARM and armed and mode == expected_mode:
            self.armed_and_auto()
            return True
        return False
    def mode_changed(self, mode, expected_mode=None):
        # POSCTL/ALTCTL are expected before automatic control starts. Once
        # AUTO.MISSION has been confirmed, a pilot-selected manual mode latches
        # takeover and the NX must never request automatic control again.
        if self.manual_override_latched:
            return False
        if (self.auto_control_started and self.state not in self.NON_TAKEOVER_STATES and
                mode in self.MANUAL_MODES and mode != expected_mode):
            self.manual_override_latched = True
            self.mission_aborted = True
            self.recovery_active = False
            self.transition(States.MANUAL_OVERRIDE, 'MANUAL_MODE_'+mode); return True
        return False
    def automatic_control_allowed(self):
        return not self.manual_override_latched
    def waypoint_reached(self, seq, waypoint_type, cell_id='', enable_feeding=True):
        if self.state not in self.ACTIVE or not self.automatic_control_allowed(): return
        self.current_waypoint = seq
        if cell_id: self.current_cell_id = cell_id
        if waypoint_type in ('TAKEOFF','TRANSIT','CAGE_ENTER','GRID_ENTER'): self.transition(States.TRANSIT)
        elif waypoint_type == 'FEED_START': self.feeder_enabled=bool(enable_feeding); self.transition(States.FEEDING)
        elif waypoint_type == 'CELL_TARGET': self.feeder_enabled=bool(enable_feeding); self.transition(States.FEEDING)
        elif waypoint_type == 'FEED_POINT': self.feed_point_arrived(seq, enable_feeding)
        elif waypoint_type == 'FEED_END': self.feeder_enabled=False; self.transition(States.EXIT)
        elif waypoint_type == 'CAGE_EXIT': self.feeder_enabled=False; self.transition(States.RTL)
        elif waypoint_type == 'GRID_EXIT':
            self.feeder_enabled=False; self.current_cell_id=''
            self.transition(States.RTL if self.rtl_after_mission else States.FINISHED)
    def feed_point_arrived(self, seq, enable_feeding=True):
        if (self.state not in self.ACTIVE or self.state == States.FEEDING_DWELL or
                not self.automatic_control_allowed()): return False
        self.current_waypoint = seq
        self.feeder_enabled = bool(enable_feeding)
        self.transition(States.FEEDING_DWELL, 'CIRCULAR_FEED_POINT_REACHED')
        return True
    def feeding_dwell_complete(self):
        if self.state != States.FEEDING_DWELL: return False
        self.feeder_enabled = False
        self.transition(States.RTL if self.rtl_after_mission else States.FINISHED,
                        'CIRCULAR_FEEDING_COMPLETE')
        return True
    def grid_complete(self):
        if self.state != States.FEEDING or not self.automatic_control_allowed():
            return False
        self.feeder_enabled = False
        self.current_cell_id = ''
        self.transition(States.RTL if self.rtl_after_mission else States.FINISHED,
                        'GRID_FINAL_CELL_DWELL_COMPLETE')
        return True
    def landed(self):
        if self.state == States.RTL: self.transition(States.LAND)
        elif self.state == States.LAND: self.transition(States.FINISHED)
    def timeout_abort(self, reason):
        if self.state not in self.ACTIVE or not self.automatic_control_allowed(): return
        self.feeder_enabled = False
        self.mission_aborted = True
        self.recovery_active = True
        self.transition(States.TIMEOUT_ABORT, reason)
    def recovery_start(self):
        if self.automatic_control_allowed() and self.state == States.TIMEOUT_ABORT: self.transition(States.RECOVERY_RTL, 'TIMEOUT_RTL_REQUESTED')
    def recovery_rtl_verified(self):
        if self.state == States.RECOVERY_RTL_VERIFY: self.transition(States.RECOVERY_RETURNING, 'TIMEOUT_RTL_CONFIRMED')
    def recovery_landing(self):
        if self.state == States.RECOVERY_RETURNING: self.transition(States.RECOVERY_LANDING, 'TIMEOUT_RETURNING')
    def recovery_landed(self):
        if self.state == States.RECOVERY_LANDING:
            self.recovery_active = False
            self.transition(States.FINISHED, 'RECOVERY_COMPLETE')
    def failsafe(self, reason):
        if self.manual_override_latched:
            return
        self.mission_aborted = True
        self.recovery_active = False
        self.transition(States.FAILSAFE, reason)

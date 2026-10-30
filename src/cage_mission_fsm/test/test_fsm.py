#!/usr/bin/env python3
import unittest
from cage_mission_fsm.core import MissionFSM, RtlRequestController, States

class FSMTest(unittest.TestCase):
    def active_fsm(self):
        f=MissionFSM(); f.initialize(); f.fcu_connected(); f.gps_valid(); f.request('m1'); f.planned(); f.upload_result(True); f.armed_and_auto(); return f
    def test_normal_progress(self):
        f=self.active_fsm(); f.waypoint_reached(3,'FEED_START'); self.assertTrue(f.feeder_enabled)
        f.waypoint_reached(4,'FEED_END'); self.assertFalse(f.feeder_enabled)
        f.waypoint_reached(5,'CAGE_EXIT'); self.assertEqual(f.state, States.RTL)
        f.landed(); f.landed(); self.assertEqual(f.state, States.FINISHED)
    def test_manual_override_is_latched_and_feeder_off(self):
        f=self.active_fsm(); f.waypoint_reached(3,'FEED_START'); self.assertTrue(f.mode_changed('LOITER'))
        self.assertEqual(f.state, States.MANUAL_OVERRIDE); self.assertTrue(f.manual_override_latched); self.assertFalse(f.feeder_enabled)
        self.assertFalse(f.automatic_control_allowed())
        self.assertFalse(f.mode_changed('AUTO'))
        self.assertTrue(f.manual_override_latched)
        self.assertFalse(f.request('m2', resume=False)); self.assertEqual(f.state, States.MANUAL_OVERRIDE)
        self.assertTrue(f.request('m2', resume=True)); self.assertEqual(f.state, States.PLAN)
        self.assertFalse(f.manual_override_latched)
    def test_resume_without_latch_is_rejected(self):
        f=MissionFSM(); f.initialize(); f.fcu_connected(); f.gps_valid()
        self.assertFalse(f.request('m1', resume=True))
    def test_manual_takeover_all_task_and_recovery_states(self):
        states=(States.TAKEOFF,States.TRANSIT,States.FEEDING,States.FEEDING_DWELL,States.EXIT,
                States.RTL,States.LAND,States.TIMEOUT_ABORT,States.RECOVERY_RTL,
                States.RECOVERY_RTL_VERIFY,States.RECOVERY_RETURNING,
                States.RECOVERY_LANDING,States.RECOVERY_LANDED)
        for state in states:
            f=MissionFSM(); f.auto_control_started=True; f.transition(state)
            self.assertTrue(f.mode_changed('POSCTL', 'AUTO.MISSION'), state)
            self.assertEqual(f.state,States.MANUAL_OVERRIDE,state)
            self.assertTrue(f.manual_override_latched,state)
    def test_prestart_manual_modes_do_not_latch(self):
        for state in (States.PLAN, States.UPLOAD_MISSION, States.WAIT_ARM):
            for mode in ('POSCTL', 'ALTCTL'):
                f=MissionFSM(); f.transition(state)
                self.assertFalse(f.mode_changed(mode, 'AUTO.MISSION'), (state, mode))
                self.assertFalse(f.manual_override_latched, (state, mode))
    def test_auto_request_requires_arm_and_exact_mode_confirmation(self):
        f=MissionFSM(); f.transition(States.WAIT_ARM)
        self.assertFalse(f.should_request_auto(False))
        self.assertFalse(f.should_request_auto(True, auto_permission=False))
        self.assertTrue(f.should_request_auto(True))
        self.assertFalse(f.confirm_auto_mode(True, 'AUTO', 'AUTO.MISSION'))
        self.assertEqual(f.state, States.WAIT_ARM)
        self.assertTrue(f.confirm_auto_mode(True, 'AUTO.MISSION', 'AUTO.MISSION'))
        self.assertEqual(f.state, States.TAKEOFF)
        self.assertTrue(f.auto_control_started)
    def test_manual_takeover_blocks_future_auto_requests(self):
        f=self.active_fsm()
        self.assertTrue(f.mode_changed('ALTCTL', 'AUTO.MISSION'))
        self.assertFalse(f.should_request_auto(True))
        self.assertFalse(f.automatic_control_allowed())
    def test_non_task_states_do_not_latch_manual_mode(self):
        for state in (States.INIT,States.WAIT_FCU,States.WAIT_GPS,States.IDLE):
            f=MissionFSM(); f.transition(state)
            self.assertFalse(f.mode_changed('LOITER'),state)
            self.assertFalse(f.manual_override_latched,state)
    def test_latch_blocks_task_and_recovery_progression(self):
        f=self.active_fsm(); self.assertTrue(f.mode_changed('LOITER'))
        f.waypoint_reached(3,'FEED_START')
        f.timeout_abort('SHOULD_NOT_RUN'); f.recovery_start()
        self.assertEqual(f.state,States.MANUAL_OVERRIDE)
        self.assertFalse(f.feeder_enabled)
        self.assertFalse(f.recovery_active)
    def test_disconnect_failsafe_turns_feeder_off(self):
        f=self.active_fsm(); f.waypoint_reached(3,'FEED_START'); f.failsafe('FCU_DISCONNECTED')
        self.assertEqual(f.state, States.FAILSAFE); self.assertFalse(f.feeder_enabled)
    def test_grid_chain_can_hold_without_rtl_for_repeat(self):
        f=MissionFSM(); f.initialize(); f.fcu_connected(); f.gps_valid(); f.request('m1', rtl_after_mission=False); f.planned(); f.upload_result(True); f.armed_and_auto()
        f.waypoint_reached(2,'GRID_ENTER'); f.waypoint_reached(3,'CELL_TARGET','R01C01')
        self.assertEqual(f.current_cell_id, 'R01C01'); self.assertTrue(f.feeder_enabled)
        f.waypoint_reached(4,'GRID_EXIT')
        self.assertEqual(f.state, States.FINISHED); self.assertFalse(f.feeder_enabled)
    def test_grid_final_cell_timer_completes_to_rtl(self):
        f=self.active_fsm(); f.waypoint_reached(11,'CELL_TARGET','R03C03')
        self.assertTrue(f.feeder_enabled)
        self.assertTrue(f.grid_complete())
        self.assertEqual(f.state, States.RTL)
        self.assertEqual(f.reason, 'GRID_FINAL_CELL_DWELL_COMPLETE')
        self.assertFalse(f.feeder_enabled)
        self.assertEqual(f.current_cell_id, '')
    def test_grid_final_cell_can_finish_without_rtl(self):
        f=MissionFSM(); f.initialize(); f.fcu_connected(); f.gps_valid()
        f.request('m1', rtl_after_mission=False); f.planned(); f.upload_result(True); f.armed_and_auto()
        f.waypoint_reached(3,'CELL_TARGET','R01C01')
        self.assertTrue(f.grid_complete())
        self.assertEqual(f.state, States.FINISHED)
    def test_timeout_recovery_chain(self):
        f=MissionFSM(); f.initialize(); f.fcu_connected(); f.gps_valid(); f.request('m1'); f.planned(); f.upload_result(True); f.armed_and_auto()
        f.waypoint_reached(3,'FEED_START'); f.timeout_abort('FEEDING_TIMEOUT')
        self.assertEqual(f.state, States.TIMEOUT_ABORT); self.assertTrue(f.mission_aborted); self.assertTrue(f.recovery_active)
        f.recovery_start(); self.assertEqual(f.state, States.RECOVERY_RTL)
        f.transition(States.RECOVERY_RTL_VERIFY, 'RTL_REQUESTED'); f.recovery_rtl_verified(); self.assertEqual(f.state, States.RECOVERY_RETURNING)
        f.recovery_landing(); self.assertEqual(f.state, States.RECOVERY_LANDING)
        f.recovery_landed(); self.assertEqual(f.state, States.FINISHED); self.assertFalse(f.recovery_active)
    def test_circular_dwell_then_rtl(self):
        f=self.active_fsm(); self.assertTrue(f.feed_point_arrived(2))
        self.assertEqual(f.state,States.FEEDING_DWELL); self.assertTrue(f.feeder_enabled)
        self.assertTrue(f.feeding_dwell_complete())
        self.assertEqual(f.state,States.RTL); self.assertFalse(f.feeder_enabled)
    def test_circular_dwell_without_feeder(self):
        f=self.active_fsm(); self.assertTrue(f.feed_point_arrived(2,enable_feeding=False))
        self.assertEqual(f.state,States.FEEDING_DWELL); self.assertFalse(f.feeder_enabled)
        self.assertTrue(f.feeding_dwell_complete()); self.assertEqual(f.state,States.RTL)
    def test_manual_takeover_during_circular_dwell(self):
        f=self.active_fsm(); f.feed_point_arrived(2); self.assertTrue(f.mode_changed('LOITER'))
        self.assertEqual(f.state,States.MANUAL_OVERRIDE); self.assertFalse(f.feeder_enabled)
    def test_anchor_failure_during_circular_dwell(self):
        f=self.active_fsm(); f.feed_point_arrived(2); f.timeout_abort('BEACON_LOST')
        self.assertEqual(f.state,States.TIMEOUT_ABORT); self.assertFalse(f.feeder_enabled)

    def test_rtl_request_retries_and_requires_exact_mode(self):
        rtl=RtlRequestController(retries=3,request_interval=1.0,confirm_timeout=5.0)
        rtl.reset(10.0)
        self.assertEqual(rtl.step(10.0,'AUTO.MISSION','AUTO.RTL'),rtl.REQUEST)
        self.assertEqual(rtl.step(10.5,'AUTO.MISSION','AUTO.RTL'),rtl.WAIT)
        self.assertEqual(rtl.step(11.0,'AUTO.MISSION','AUTO.RTL'),rtl.REQUEST)
        self.assertEqual(rtl.step(12.0,'AUTO.MISSION','AUTO.RTL'),rtl.REQUEST)
        self.assertEqual(rtl.step(16.9,'AUTO.MISSION','AUTO.RTL'),rtl.WAIT)
        self.assertEqual(rtl.step(17.0,'AUTO.MISSION','AUTO.RTL'),rtl.FAILED)

    def test_rtl_request_confirms_from_fcu_mode(self):
        rtl=RtlRequestController(retries=3,request_interval=1.0,confirm_timeout=5.0)
        rtl.reset(1.0)
        self.assertEqual(rtl.step(1.0,'AUTO.MISSION','AUTO.RTL'),rtl.REQUEST)
        self.assertEqual(rtl.step(1.2,'AUTO.RTL','AUTO.RTL'),rtl.CONFIRMED)
        self.assertTrue(rtl.confirmed)

if __name__ == '__main__': unittest.main()

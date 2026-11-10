"""Synthetic encoder/motor and serial failure injection, not hardware qualification."""
import importlib.util
import json
import math
import os
from pathlib import Path
import pty
import select
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CLOCK_MOD = 1 << 30
fake_time = types.SimpleNamespace(
    ticks_diff=lambda a, b: (a - b + CLOCK_MOD // 2) % CLOCK_MOD - CLOCK_MOD // 2)
with patch.dict(sys.modules, {"utime": fake_time, "machine": types.ModuleType("machine"),
                             "micropython": types.ModuleType("micropython")}):
    fw = load("pico_position", ROOT / "pico/main.py")


def inertial_trial(kd):
    """Synthetic first-order plant: 100 mm/s full duty, 180 ms time constant.

    This is a repeatable comparison, not an identified model of the real lift.
    """
    clock, count, pulses, output = [0], [0], [0], [0, 0]
    controller = fw.PositionController(
        lambda d, u: output.__setitem__(slice(None), [d, u]), lambda: clock[0],
        lambda: (count[0], pulses[0]), mm_per_count=.1, ab_verified=True)
    clock[0] = 400
    controller.command({"cmd": "zero", "at_lower_reference": True})
    controller.command({"cmd": "arm"})
    controller.command({"cmd": "move_to", "target_mm": 100, "max_duty": .25,
                        "kp": .006, "kd": kd})
    position = velocity = peak = 0.0
    approach_speed = None
    for step in range(6000):
        velocity += (100 * output[0] * output[1] - velocity) * .01 / .18
        position += velocity * .01
        peak = max(peak, position)
        raw = round(position / .1)
        pulses[0] += abs(raw - count[0]); count[0] = raw
        clock[0] += 10; controller.update()
        if approach_speed is None and position >= 98:
            approach_speed = velocity
        if step % 5 == 0:
            controller.command({"cmd": "heartbeat"})
        if controller.mode == "idle":
            break
    return {"kd": kd, "seconds": (step + 1) * .01, "position_mm": position,
            "approach_speed_mm_s": approach_speed, "peak_mm": peak,
            "event": controller.event, "fault": controller.fault}


class PositionTests(unittest.TestCase):
    def setUp(self):
        self.time = self.raw = self.pulses = 0
        self.outputs = []
        self.c = fw.PositionController(
            lambda direction, duty: self.outputs.append((direction, duty)),
            lambda: self.time % CLOCK_MOD, lambda: (self.raw, self.pulses),
            mm_per_count=0.1, encoder_sign=1, ab_verified=True)

    def cmd(self, cmd, **values):
        self.c.command(dict(cmd=cmd, **values))

    def step(self, ms=20, counts=0, activity=None, heartbeat=False):
        self.time += ms
        self.raw = (self.raw + counts) % fw.COUNTER_MODULUS
        self.pulses = (self.pulses + (abs(counts) if activity is None else activity)) % fw.COUNTER_MODULUS
        self.c.update()
        if heartbeat:
            self.cmd("heartbeat")

    def home(self):
        self.step(400)
        self.cmd("zero", at_lower_reference=True)
        self.assertTrue(self.c.homed)

    def simulate_to_rest(self):
        remainder = 0.0
        for _ in range(4000):
            # Ideal synthetic plant: 100 mm/s at full duty, encoder 0.1 mm/count.
            remainder += self.c.direction * self.c.duty * 1000 * .02
            counts = math.trunc(remainder)
            remainder -= counts
            self.step(counts=counts, heartbeat=True)
            if self.c.mode == "idle":
                return
        self.fail("Synthetic plant did not stop")

    def test_boot_requires_calibration_zero_and_arm(self):
        self.assertIsNone(self.c.position_mm)
        self.assertFalse(self.c.status()["at_min"])
        self.cmd("move_to", target_mm=200)
        self.assertEqual(self.outputs[-1], (0, 0))
        self.cmd("arm")
        self.cmd("move_to", target_mm=200)
        self.assertIsNone(self.c.target_mm)
        self.cmd("disarm")
        self.c.mm_per_count = None
        self.step(400)
        self.cmd("zero", at_lower_reference=True)
        self.assertFalse(self.c.homed)

    def test_manual_zero_requires_reference_disarmed_and_no_recent_motion(self):
        self.step(400)
        self.cmd("zero")
        self.assertFalse(self.c.homed)
        self.cmd("arm")
        self.cmd("zero", at_lower_reference=True)
        self.assertFalse(self.c.homed)
        self.cmd("disarm")
        self.step(counts=1)
        self.cmd("zero", at_lower_reference=True)
        self.assertFalse(self.c.homed)
        self.step(400)
        self.cmd("zero", at_lower_reference=True)
        self.assertEqual(self.c.position_mm, 0)

    def test_200_to_300_and_back_without_rehoming(self):
        self.home()
        self.step(counts=2000)
        self.assertEqual(self.c.position_mm, 200)
        self.cmd("arm")
        self.cmd("move_to", target_mm=300)
        self.simulate_to_rest()
        self.assertFalse(self.c.fault)
        self.assertEqual(self.c.event, "target_reached")
        self.assertLessEqual(abs(self.c.position_mm - 300), fw.TOLERANCE_MM + fw.POSITION_EPSILON_MM)
        self.assertEqual(self.outputs[-1], (0, 0))
        self.cmd("move_to", target_mm=200)
        self.simulate_to_rest()
        self.assertEqual(self.c.event, "target_reached")
        self.assertLessEqual(abs(self.c.position_mm - 200), fw.TOLERANCE_MM + fw.POSITION_EPSILON_MM)

    def test_reset_reason_is_optional_and_watchdog_is_identified(self):
        self.assertEqual(fw.reset_reason(), "unknown")
        with patch.object(fw.machine, "reset_cause", create=True, return_value=3), \
             patch.object(fw.machine, "WDT_RESET", 3, create=True):
            self.assertEqual(fw.reset_reason(), "watchdog")
        c=fw.PositionController(lambda *args:None,lambda:0,lambda:(0,0),boot_reason="watchdog")
        self.assertEqual(c.status()["boot_reason"],"watchdog")
        self.assertEqual(c.duty,0)
        self.assertFalse(c.armed)

    def test_resolution_rejects_unreachable_target_without_energizing(self):
        self.c.mm_per_count = .7
        self.home(); self.cmd("arm")
        self.cmd("move_to", target_mm=100.45)
        self.assertEqual(self.c.event, "target_encoder_resolution")
        self.assertEqual(self.outputs[-1], (0, 0))
        self.assertEqual(self.c.mode, "idle")
        self.cmd("move_to", target_mm=100.1)
        self.assertEqual(self.c.mode, "moving")

    def test_two_tenths_boundary_handles_float_rounding_both_directions(self):
        for counts, target in [(1998,200), (1002,100)]:
            self.setUp(); self.home(); self.step(counts=counts); self.c.reset_velocity()
            self.cmd("arm"); self.cmd("move_to", target_mm=target)
            self.step(heartbeat=True)
            self.assertEqual(self.c.mode,"settling")
            self.assertEqual(self.outputs[-1],(0,0))

    def test_endpoint_margin_rejects_without_moving_and_physical_flags_remain(self):
        self.home(); self.cmd("arm")
        for target in (0, 9.99, 620.01, 630):
            self.cmd("move_to", target_mm=target)
            self.assertEqual(self.c.event, "target_near_endpoint")
            self.assertIsNone(self.c.target_mm)
            self.assertEqual(self.outputs[-1], (0, 0))
        self.assertTrue(self.c.status()["at_min"])
        self.cmd("move_to", target_mm=10)
        self.simulate_to_rest()
        self.assertEqual(self.c.event, "target_reached")
        self.assertFalse(self.c.status()["at_min"])
        self.assertEqual(self.c.status()["work_max_mm"], 620)

    def test_range_invalid_values_and_bool_rejected(self):
        for target in [-1, 630.01, float("nan"), float("inf"), True, "300"]:
            with self.subTest(target=target):
                self.setUp()
                self.home()
                self.cmd("arm")
                self.cmd("move_to", target_mm=target)
                self.assertEqual(self.c.fault, "bad_target")
                self.assertFalse(self.c.armed)
                self.assertEqual(self.outputs[-1], (0, 0))

    def test_soft_limits_block_outward_jog_but_allow_retreat(self):
        self.home()
        self.cmd("arm")
        self.cmd("jog", direction=-1, duty=.2)
        self.assertEqual(self.c.event, "soft_min")
        self.assertEqual(self.outputs[-1], (0, 0))
        self.cmd("jog", direction=1, duty=.2)
        self.assertEqual(self.c.direction, 1)
        self.step(counts=6300)
        self.assertEqual(self.c.event, "soft_max")
        self.cmd("jog", direction=1, duty=.2)
        self.assertEqual(self.c.direction, 0)
        self.cmd("jog", direction=-1, duty=.2)
        self.assertEqual(self.c.direction, -1)

    def test_midstroke_stall_is_fault_never_min_max_or_zero(self):
        self.home()
        self.step(counts=2000)
        self.c.reset_velocity()
        self.cmd("arm")
        self.cmd("move_to", target_mm=300)
        for _ in range(41):
            self.step(heartbeat=True)
        self.assertEqual(self.c.fault, "encoder_no_pulse")
        self.assertFalse(self.c.homed)
        self.assertFalse(self.c.status()["at_min"])
        self.assertFalse(self.c.status()["at_max"])
        self.assertEqual(self.outputs[-1], (0, 0))
        self.cmd("clear")
        self.cmd("arm")
        self.cmd("move_to", target_mm=300)
        self.assertIsNone(self.c.target_mm)

    def test_jog_renewals_and_reversals_do_not_mask_stall(self):
        self.cmd("arm")
        self.cmd("jog", direction=1, duty=.2)
        for i in range(40):
            self.step()
            self.cmd("jog", direction=1 if i < 20 else -1, duty=.2)
        self.assertEqual(self.c.fault, "jog_reverse_requires_stop")
        self.assertEqual(self.outputs[-1], (0, 0))

    def test_late_heartbeat_stops_and_does_not_restart(self):
        self.home()
        self.cmd("arm")
        self.cmd("move_to", target_mm=300)
        self.step(20)
        self.time += fw.LEASE_MS
        self.cmd("heartbeat")
        self.assertEqual(self.c.fault, "command_timeout")
        self.assertFalse(self.c.armed)
        self.assertEqual(self.outputs[-1], (0, 0))
        self.cmd("arm")
        self.assertFalse(self.c.armed)

    def test_passive_motion_is_tracked_and_out_of_range_invalidates_zero(self):
        self.home()
        self.step(counts=1000)
        self.assertEqual(self.c.position_mm, 100)
        self.step(counts=5520)
        self.assertEqual(self.c.fault, "position_out_of_range")
        self.assertFalse(self.c.homed)

    def test_wrong_encoder_direction_stops_and_requires_new_zero(self):
        self.home()
        self.step(counts=2000)
        self.cmd("arm")
        self.cmd("move_to", target_mm=300)
        # Allow the measured velocity from manual pre-positioning to decay;
        # direction checking applies once the controller actually drives upward.
        for _ in range(30):
            self.step(heartbeat=True)
            if self.c.direction == 1:
                break
        self.assertEqual(self.c.direction, 1)
        for _ in range(6):
            self.step(counts=-2, heartbeat=True)
        self.assertEqual(self.c.fault, "encoder_direction_mismatch")
        self.assertFalse(self.c.homed)

    def test_overshoot_is_not_success_and_settling_drift_is_fault(self):
        self.home()
        self.step(counts=2000)
        self.cmd("arm")
        self.cmd("move_to", target_mm=300)
        self.step(counts=1020, heartbeat=True)
        self.assertEqual(self.c.fault, "target_overshoot")
        self.assertNotEqual(self.c.event, "target_reached")
        self.cmd("clear")
        self.cmd("arm")
        self.cmd("move_to", target_mm=302)
        self.step(heartbeat=True)
        self.assertEqual(self.c.mode, "settling")
        self.step(counts=20, heartbeat=True)
        self.assertEqual(self.c.fault, "target_not_settled")

    def test_repeat_target_does_not_reset_motion_timeout(self):
        self.home()
        self.cmd("arm")
        self.cmd("move_to", target_mm=300)
        started = self.c.started
        self.step(counts=1, heartbeat=True)
        self.cmd("move_to", target_mm=300)
        self.assertEqual(self.c.started, started)
        self.assertEqual(self.c.event, "busy_stop_before_new_target")
        with patch.object(fw, "MOVE_TIMEOUT_MS", 100):
            for _ in range(5):
                self.step(counts=1, heartbeat=True)
        self.assertEqual(self.c.fault, "move_timeout")

    def test_counter_and_clock_wrap_and_negative_encoder_sign(self):
        self.raw = fw.COUNTER_MODULUS - 2
        self.c.raw = self.raw
        self.time = CLOCK_MOD - 100
        self.c.last_update = self.c.last_pulse = self.time
        self.home()
        self.step(counts=5)
        self.assertAlmostEqual(self.c.position_mm, .5)
        self.c.encoder_sign = -1
        self.c.origin_count = self.c.count
        self.step(counts=-100)
        self.assertAlmostEqual(self.c.position_mm, 10)

    def test_calibration_validation_and_recalibration_loses_zero(self):
        self.home()
        for factor in [0, -1, 1.1, True, float("nan")]:
            self.cmd("calibrate", mm_per_count=factor, encoder_sign=1, ab_verified=True)
            self.assertEqual(self.c.fault, "bad_calibration")
            self.cmd("clear")
        self.cmd("calibrate", mm_per_count=.2, encoder_sign=-1, ab_verified=True)
        self.assertTrue(self.c.calibrated)
        self.assertFalse(self.c.homed)
        self.assertEqual(self.outputs[-1], (0, 0))

    def test_full_pwm_allowed_and_above_full_rejected(self):
        self.home()
        self.cmd("arm")
        self.cmd("jog", direction=1, duty=1.0)
        self.assertEqual(self.outputs[-1], (1, 1.0))
        self.cmd("stop")
        self.cmd("move_to", target_mm=300, max_duty=1.0)
        self.assertEqual(self.c.max_duty, 1.0)
        self.cmd("stop")
        self.cmd("jog", direction=1, duty=1.01)
        self.assertEqual(self.c.fault, "bad_jog")
        self.assertEqual(self.outputs[-1], (0, 0))

    def test_calibration_persists_across_reboot_but_zero_and_motion_do_not(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "calibration.json")
            store = fw.CalibrationStore(path, device_id="pico-a")
            self.c.calibration_store = store
            self.step(400)
            self.cmd("calibrate", mm_per_count=.2, encoder_sign=-1, ab_verified=True)
            self.assertTrue(self.c.status()["calibration_saved"])
            self.cmd("zero", at_lower_reference=True)
            self.cmd("arm")
            self.cmd("move_to", target_mm=300)
            reboot = fw.PositionController(
                lambda *args: self.outputs.append(args), lambda: self.time,
                lambda: (self.raw, self.pulses), calibration_store=store)
            self.assertTrue(reboot.calibrated)
            self.assertTrue(reboot.calibration_saved)
            self.assertEqual(reboot.mm_per_count, .2)
            self.assertEqual(reboot.encoder_sign, -1)
            self.assertEqual(reboot.event, "saved_calibration_loaded")
            self.assertFalse(reboot.homed)
            self.assertFalse(reboot.armed)
            self.assertIsNone(reboot.target_mm)
            self.assertIsNone(reboot.position_mm)
            self.assertEqual(self.outputs[-1], (0, 0))
            reboot.command({"cmd": "arm"})
            reboot.command({"cmd": "move_to", "target_mm": 300})
            self.assertIsNone(reboot.target_mm)
            data = json.loads(Path(path).read_text())
            self.assertNotIn("origin_count", data)
            self.assertNotIn("position_mm", data)

    def test_save_failure_stops_and_does_not_report_new_calibration(self):
        self.home()
        self.c.calibration_store = types.SimpleNamespace(save=lambda *args: (_ for _ in ()).throw(OSError(28)))
        self.cmd("calibrate", mm_per_count=.2, encoder_sign=-1, ab_verified=True)
        self.assertEqual(self.c.fault, "calibration_save_failed")
        self.assertEqual(self.c.mm_per_count, .1)
        self.assertFalse(self.c.calibration_saved)
        self.assertFalse(self.c.homed)
        self.assertEqual(self.outputs[-1], (0, 0))

    def test_pd_damps_motion_toward_target_without_active_reverse(self):
        self.home()
        self.cmd("arm")
        self.cmd("move_to", target_mm=100, max_duty=.8, kp=.006, kd=.004)
        self.c.velocity = 0
        stationary = self.c.position_duty(10)
        self.c.velocity = 20
        approaching = self.c.position_duty(10)
        self.c.velocity = -20
        retreating = self.c.position_duty(10)
        self.assertLess(approaching, stationary)
        self.assertGreater(retreating, stationary)
        self.c.velocity = 100
        self.assertEqual(self.c.position_duty(10), 0)  # No minimum-duty override.
        self.c.move_direction = -1
        self.c.velocity = -100
        self.assertEqual(self.c.position_duty(-10), 0)
        self.c.velocity = 0
        self.assertLessEqual(self.c.position_duty(-500), self.c.max_duty)

    def test_velocity_filter_keeps_same_millisecond_pulses_and_is_sample_rate_independent(self):
        def estimate(update_ms):
            self.setUp()
            for tick in range(1, 401, update_ms):
                counts = 2 if (tick + update_ms - 1) % 20 == 0 else 0
                self.step(ms=update_ms, counts=counts)
            return self.c.velocity
        regular, fast = estimate(20), estimate(1)
        self.assertAlmostEqual(regular, fast)
        self.assertGreater(regular, 9)
        self.assertLessEqual(regular, 10)
        self.setUp()
        self.step(ms=0, counts=1)
        self.step(ms=20, counts=1)
        split = self.c.velocity
        self.setUp()
        self.step(ms=20, counts=2)
        self.assertEqual(split, self.c.velocity)

    def test_gain_validation_cannot_change_running_motion_or_hide_expired_lease(self):
        for gains in [(0, .001), (.2, .001), (.006, -.1), (.006, .2),
                      (True, .001), (.006, float("nan"))]:
            with self.subTest(gains=gains):
                self.setUp(); self.home(); self.cmd("arm")
                self.cmd("move_to", target_mm=100, kp=gains[0], kd=gains[1])
                self.assertEqual(self.c.fault, "bad_pd_gains")
                self.assertEqual(self.outputs[-1], (0, 0))
        self.setUp(); self.home(); self.cmd("arm")
        self.cmd("move_to", target_mm=100, kp=.01, kd=.002)
        self.cmd("move_to", target_mm=200, kp=.02, kd=.003)
        self.assertEqual((self.c.kp, self.c.kd), (.01, .002))
        self.time += fw.LEASE_MS
        self.cmd("move_to", target_mm=200, kp=.02, kd=.003)
        self.assertEqual(self.c.fault, "command_timeout")
        self.assertEqual((self.c.kp, self.c.kd), (.01, .002))

    def test_kd_zero_allows_explicit_p_comparison_without_target_derivative_kick(self):
        self.home(); self.cmd("arm")
        self.cmd("move_to", target_mm=100, kd=0)
        self.c.velocity = 100
        no_d = self.c.position_duty(10)
        self.c.velocity = 0
        self.assertEqual(no_d, self.c.position_duty(10))
        self.cmd("stop")
        self.cmd("move_to", target_mm=600, kd=.004)
        self.assertEqual(self.c.velocity, 0)
        self.assertEqual(self.c.status()["position_control"], "PD")

    def test_approach_envelope_settles_with_or_without_damping(self):
        proportional, damped = inertial_trial(0), inertial_trial(.001)
        for result in (proportional, damped):
            self.assertEqual(result["fault"], "")
            self.assertEqual(result["event"], "target_reached")
        for result in (proportional, damped):
            self.assertLess(result["approach_speed_mm_s"], 5)
            self.assertLessEqual(abs(result["position_mm"] - 100), 1)

    def test_arrival_drift_survives_disarm_but_new_motion_releases_monitor(self):
        self.home(); self.step(counts=2000); self.c.reset_velocity()
        self.cmd("arm"); self.cmd("move_to", target_mm=200)
        for _ in range(20): self.step(heartbeat=True)
        self.assertEqual(self.c.arrival_target, 200)
        self.cmd("disarm")
        for _ in range(21): self.step(counts=-1)
        self.assertEqual(self.c.fault, "position_drift")
        self.assertEqual(self.outputs[-1], (0, 0))
        self.assertFalse(self.c.armed)
        self.assertTrue(self.c.homed)
        self.cmd("clear"); self.cmd("arm"); self.cmd("move_to", target_mm=300)
        self.assertIsNone(self.c.arrival_target)
        self.assertEqual(self.c.mode, "moving")

    def test_pulse_chatter_with_zero_progress_stops_even_with_heartbeats(self):
        self.home(); self.cmd("arm"); self.cmd("move_to", target_mm=100, max_duty=1)
        for _ in range(100): self.step(activity=2, heartbeat=True)
        self.assertEqual(self.c.fault, "motion_no_progress")
        self.assertFalse(self.c.homed)
        self.assertEqual(self.outputs[-1], (0, 0))

    def test_fine_approach_accepts_slow_net_counts_but_not_chatter(self):
        self.home(); self.step(counts=1995); self.c.reset_velocity()
        self.cmd("arm"); self.cmd("move_to", target_mm=200)
        for i in range(75):
            self.step(counts=1 if i in (20,40) else 0, activity=2, heartbeat=True)
        self.assertFalse(self.c.fault)
        self.assertEqual(self.c.mode,"moving")
        self.step(counts=1, activity=2, heartbeat=True)
        self.assertEqual(self.c.mode,"settling")

    def test_fine_approach_still_rejects_zero_net_motion(self):
        self.home(); self.step(counts=1995); self.c.reset_velocity()
        self.cmd("arm"); self.cmd("move_to", target_mm=200)
        for _ in range(90): self.step(activity=2, heartbeat=True)
        self.assertEqual(self.c.fault,"motion_no_progress")
        self.assertEqual(self.outputs[-1],(0,0))

    def test_coasting_does_not_spend_powered_no_pulse_budget(self):
        self.home(); self.cmd("arm"); self.cmd("move_to", target_mm=100, max_duty=1)
        # An externally maintained velocity estimate requests coasting. Check the
        # timing contract separately from any assumed physical motor response.
        for _ in range(60):
            self.c.velocity = 1000
            self.step(heartbeat=True)
        self.assertFalse(self.c.fault)
        self.assertEqual(self.c.duty, 0)
        self.c.velocity = 0
        self.step(heartbeat=True)
        self.assertFalse(self.c.fault)
        for _ in range(41): self.step(heartbeat=True)
        self.assertEqual(self.c.fault, "encoder_no_pulse")

    def test_approach_envelope_reduces_drive_in_both_directions(self):
        self.c.move_direction = 1; self.c.velocity = 10
        self.assertEqual(self.c.approach_duty(1, 2), 0)
        self.assertGreater(self.c.approach_duty(1, 100), 0)
        self.c.move_direction = -1; self.c.velocity = -10
        self.assertEqual(self.c.approach_duty(1, 2), 0)
        self.c.velocity = 0
        self.assertLess(self.c.approach_duty(1, 2), self.c.approach_duty(1, 100))

    def test_position_speed_limit_is_continuous_at_velocity_threshold(self):
        remaining = 2.0
        a, reaction = fw.APPROACH_DECEL_MM_S2, fw.APPROACH_REACTION_S
        ceiling = math.sqrt((a * reaction) ** 2 + 2 * a * (remaining - fw.TOLERANCE_MM)) - a * reaction
        for direction in (-1, 1):
            self.c.move_direction = direction
            self.c.velocity = direction * (ceiling - .001)
            below = self.c.approach_duty(.8, remaining, floor=fw.POSITION_DRIVE_BIAS, smooth=True)
            self.c.velocity = direction * (ceiling + .001)
            above = self.c.approach_duty(.8, remaining, floor=fw.POSITION_DRIVE_BIAS, smooth=True)
            self.assertGreater(above, 0)
            self.assertLess(above, below)
            self.assertLess(below - above, .001)
            self.assertLessEqual(below, .8)
            # Large overspeed still permits zero output, without a bias override.
            self.c.velocity = direction * 100
            self.assertEqual(self.c.approach_duty(.8, remaining, smooth=True), 0)

    def test_smooth_approach_respects_output_cap_and_fine_boundary(self):
        self.c.move_direction = 1
        self.c.velocity = 4
        below = self.c.approach_duty(.8, 9.999, floor=.2, smooth=True)
        above = self.c.approach_duty(.8, 10.001, floor=.2, smooth=True)
        self.assertLess(abs(above - below), .001)
        for velocity in (0, 3, 10, 50):
            self.c.velocity = velocity
            for duty in (0, .1, .25, .6, 1):
                output = self.c.approach_duty(duty, 2, floor=.2, smooth=True)
                self.assertGreaterEqual(output, 0)
                self.assertLessEqual(output, duty)

    def test_tuning_save_restore_without_restoring_zero_arm_or_target(self):
        with tempfile.TemporaryDirectory() as directory:
            store = fw.CalibrationStore(str(Path(directory)/"cal.json"), device_id="test")
            store.save(.1, 1); self.c.calibration_store=store; self.c.calibration_saved=True
            self.home()
            self.cmd("save_control", kp=.012, kd=.003, max_duty=.4)
            self.assertEqual(self.c.event, "control_saved")
            reboot = fw.PositionController(lambda *args: None, lambda:self.time,
                lambda:(self.raw,self.pulses), calibration_store=store)
            self.assertEqual((reboot.kp,reboot.kd,reboot.max_duty), (.012,.003,.4))
            self.assertFalse(reboot.homed); self.assertFalse(reboot.armed)
            self.assertIsNone(reboot.target_mm)
            self.cmd("arm")
            self.cmd("save_control", kp=.02,kd=.004,max_duty=.5)
            self.assertEqual(self.c.event, "control_save_requires_stationary")
            self.assertEqual(store.load()["control"]["kp"], .012)
            self.cmd("disarm")
            with patch.object(fw.os, "rename", side_effect=OSError(28)):
                self.cmd("save_control", kp=.02,kd=.004,max_duty=.5)
            self.assertEqual(self.c.fault,"control_save_failed")
            self.assertEqual(store.load()["control"]["kp"],.012)

    def test_reference_can_be_forgotten_only_when_disarmed_and_stationary(self):
        self.home(); self.cmd("arm"); self.cmd("forget_zero")
        self.assertTrue(self.c.homed)
        self.assertEqual(self.c.event, "reference_clear_requires_stationary")
        self.cmd("disarm"); self.cmd("forget_zero")
        self.assertFalse(self.c.homed)
        self.assertEqual(self.c.event, "reference_cleared")
        self.cmd("arm"); self.cmd("jog", direction=-1, duty=.2)
        self.assertEqual(self.c.direction, -1)

    def test_invalid_saved_tuning_keeps_valid_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"cal.json"
            store=fw.CalibrationStore(str(path),device_id="test")
            data=store.record(.1,1);data["control"]={"kp":True,"kd":.1,"max_duty":1}
            path.write_text(json.dumps(data))
            self.assertEqual(store.load(),store.record(.1,1))


class CalibrationStorageTests(unittest.TestCase):
    def test_atomic_save_keeps_previous_file_on_interruption_and_skips_duplicates(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "calibration.json")
            store = fw.CalibrationStore(path, device_id="pico-a")
            self.assertIsNone(store.load())
            store.save(.2, 1)
            old = Path(path).read_bytes()
            with patch.object(fw.os, "rename", side_effect=OSError(5)):
                store.save(.2, 1)  # Already saved: no flash rewrite/rename.
                with self.assertRaises(OSError):
                    store.save(.3, -1)
            self.assertEqual(Path(path).read_bytes(), old)
            self.assertFalse(Path(path + ".tmp").exists())
            self.assertEqual(store.load()["mm_per_count"], .2)
            store.save(.3, -1)
            self.assertEqual(store.load()["encoder_sign"], -1)

    def test_corrupt_or_wrong_hardware_records_never_restore_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.json"
            store = fw.CalibrationStore(str(path), device_id="pico-a")
            valid = store.record(.2, 1)
            bad = ["{partial"]
            for key, value in [("version", 2), ("device_id", "pico-b"),
                               ("decoder", "x4"), ("encoder_pins", [1, 2]),
                               ("stroke_mm", 700), ("mm_per_count", 0),
                               ("mm_per_count", True), ("encoder_sign", True),
                               ("ab_verified", False)]:
                bad.append(json.dumps(dict(valid, **{key: value})))
            for text in bad:
                with self.subTest(text=text):
                    path.write_text(text)
                    outputs = []
                    c = fw.PositionController(lambda *args: outputs.append(args),
                                              lambda: 0, lambda: (0, 0), calibration_store=store)
                    self.assertFalse(c.calibrated)
                    self.assertFalse(c.homed)
                    self.assertFalse(c.calibration_saved)
                    self.assertEqual(c.event, "calibration_storage_invalid")
                    self.assertEqual(outputs[-1], (0, 0))


class PositionGuiTests(unittest.TestCase):
    def setUp(self):
        import tkinter as tk
        sys.path.insert(0, str(ROOT / "raspberry_pi/lift/position"))
        from lift_position import PositionUI
        try:
            self.root = tk.Tk()
        except tk.TclError:
            self.skipTest("Tk display unavailable")
        self.root.withdraw()
        self.master, self.slave = pty.openpty()
        os.set_blocking(self.master, False)
        self.temp_diagnostics = tempfile.TemporaryDirectory()
        self.app = PositionUI(self.root, os.ttyname(self.slave))
        self.app.diag_path = Path(self.temp_diagnostics.name) / "diagnostics.jsonl"
        self.app.connect()
        self.commands()
        c = fw.PositionController(lambda *args: None, lambda: 0, lambda: (0, 0),
                                  mm_per_count=.1, ab_verified=True)
        self.frame = c.status()

    def tearDown(self):
        if hasattr(self, "app"):
            self.root.update_idletasks()
            self.app.close()
            os.close(self.master)
            os.close(self.slave)
            self.temp_diagnostics.cleanup()

    def commands(self):
        data = b""
        # PTYs may deliver consecutive serial writes in separate readable chunks.
        while select.select([self.master], [], [], .02)[0]:
            data += os.read(self.master, 16384)
        return [json.loads(line) for line in data.splitlines()]

    def receive(self, **changes):
        frame = dict(self.frame, **changes)
        os.write(self.master, (json.dumps(frame) + "\n").encode())
        self.app.receive()

    def arm_and_home(self):
        self.receive(homed=True, state="IDLE", position_mm=200)
        self.app.arm()
        self.commands()
        self.receive(homed=True, state="IDLE", position_mm=200, armed=True)
        self.assertTrue(self.app.armed)

    def test_protocol_validation_and_target_heartbeat_stop_reboot(self):
        self.receive(protocol="yor-official-pwm-v1")
        self.assertFalse(self.app.known)
        self.receive(homed=True, position_mm=float("nan"))
        self.assertFalse(self.app.known)
        self.arm_and_home()
        self.app.target.set("300")
        self.app.move_to()
        self.assertEqual(self.commands(), [{"cmd": "move_to", "target_mm": 300, "max_duty": 1.0,
                                              "kp": .04, "kd": .001}])
        self.app.tick_extra()
        self.assertEqual(self.commands(), [{"cmd": "heartbeat"}])
        self.receive(homed=True, state="IDLE", position_mm=300, armed=True, event="target_reached")
        self.assertIn("300.0", self.app.position_text.get())
        self.assertIn("목표 도달", self.app.event_text.get())
        self.receive(armed=False, homed=False, state="UNHOMED", position_mm=None)
        self.assertFalse(self.app.armed)
        self.assertIn("원점 미등록", self.app.position_text.get())
        self.commands()
        self.app.tick_extra()
        self.assertEqual(self.commands(), [])

    def test_invalid_target_is_not_transmitted_and_stale_transport_clears_position(self):
        self.arm_and_home()
        for value in ["631", "-1", "nan", "abc"]:
            self.app.target.set(value)
            self.app.move_to()
            self.assertEqual(self.commands(), [])
        self.app.last_rx = 0
        self.app.tick()
        self.assertIsNone(self.app.serial)
        self.assertIsNone(self.app.latest)
        self.assertIn("알 수 없음", self.app.position_text.get())

    def test_calibration_and_manual_zero_require_explicit_reference_confirmation(self):
        self.receive()
        self.app.factor.set("0.15")
        self.app.encoder_sign.set("-1")
        with patch("lift_position.messagebox.askyesno", return_value=True):
            self.app.calibrate()
        self.assertEqual(self.commands()[-1], {"cmd": "calibrate", "mm_per_count": .15,
                                               "encoder_sign": -1, "ab_verified": True})
        self.app.mark_zero()
        dialog = next(w for w in self.root.winfo_children() if w.winfo_class() == "Toplevel")
        buttons = [w for frame in dialog.winfo_children() for w in frame.winfo_children()]
        next(w for w in buttons if w.cget("text") == "취소").invoke()
        self.assertNotIn("zero", [m["cmd"] for m in self.commands()])
        self.app.mark_zero()
        dialog = next(w for w in self.root.winfo_children() if w.winfo_class() == "Toplevel")
        buttons = [w for frame in dialog.winfo_children() for w in frame.winfo_children()]
        next(w for w in buttons if w.cget("text") == "확인 · 0 mm 설정").invoke()
        self.assertEqual(self.commands()[-1], {"cmd": "zero", "at_lower_reference": True})

    def test_move_button_arms_only_for_requested_move_and_waits_for_ack(self):
        self.receive(homed=True, state="IDLE", position_mm=200)
        self.app.target.set("300")
        self.app.move_to()
        self.assertEqual(self.commands(), [{"cmd": "arm"}])
        self.receive(homed=True, state="IDLE", position_mm=200, armed=True)
        self.assertEqual(self.commands(), [{"cmd": "move_to", "target_mm": 300, "max_duty": 1.0,
                                              "kp": .04, "kd": .001}])
        self.app.disarm()
        self.commands()
        self.receive(homed=True, state="IDLE", position_mm=200, armed=False, fault="encoder_no_pulse")
        self.app.move_to()
        self.assertEqual(self.commands(), [])

    def test_zero_without_confirmation_from_pico_reports_failure(self):
        self.receive()
        self.app.confirm_zero()
        self.commands()
        self.app.zero_deadline = time.monotonic() - 1
        self.app.tick_extra()
        self.assertIn("0 mm 설정 실패", self.app.event_text.get())
        self.assertFalse(self.app.latest["homed"])

    def test_held_button_release_before_arm_ack_never_starts_motor(self):
        self.receive()
        self.app.start_manual(1)
        self.assertEqual(self.commands(), [{"cmd": "arm"}])
        self.app.release()
        self.assertEqual(self.commands(), [{"cmd": "stop"}])
        self.receive(armed=True)
        self.app.tick_extra()
        self.assertTrue(all(cmd["cmd"] == "heartbeat" for cmd in self.commands()))
        self.assertFalse(self.app.held)

    def test_stop_and_timeout_cancel_pending_target(self):
        self.receive(homed=True, state="IDLE", position_mm=200)
        self.app.move_to()
        self.commands()
        self.app.disarm()
        self.commands()
        self.receive(homed=True, state="IDLE", position_mm=200, armed=True)
        self.assertNotIn("move_to", [cmd["cmd"] for cmd in self.commands()])
        self.app.armed = False
        self.app.move_to()
        self.commands()
        self.app.action_deadline = 0
        self.receive(homed=True, state="IDLE", position_mm=200, armed=True)
        self.app.tick_extra()
        self.assertNotIn("move_to", [cmd["cmd"] for cmd in self.commands()])

    def test_wizard_records_counts_and_checks_both_directions(self):
        self.receive(calibrated=False, raw_count=2)
        self.app.setup_next()
        self.commands()
        self.receive(calibrated=False, raw_count=502)
        self.app.distance.set("100")
        self.app.setup_next()
        self.commands()
        self.assertEqual(self.app.measure_phase, "down")
        self.app.start_manual(-1)
        self.commands()
        self.receive(calibrated=False, raw_count=602, armed=True)
        self.app.release()
        self.commands()
        self.receive(calibrated=False, raw_count=602)
        self.app.setup_next()
        self.assertNotIn("calibrate", [cmd["cmd"] for cmd in self.commands()])
        self.receive(calibrated=False, raw_count=402)
        with patch("lift_position.messagebox.askyesno", return_value=True):
            self.app.setup_next()
        self.assertEqual(self.commands()[-1], {"cmd": "calibrate", "mm_per_count": .2,
                                               "encoder_sign": 1, "ab_verified": True})
        self.receive(calibrated=True, raw_count=402, event="calibrated_unhomed")
        self.assertEqual(self.app.measure_phase, "done")
        self.assertIn("설정 완료", self.app.setup_text.get())

    def test_disconnect_destroys_incomplete_measurement_and_pending_motion(self):
        self.receive(calibrated=False)
        self.app.setup_next()
        self.commands()
        self.app.start_manual(1)
        self.commands()
        self.app.disconnect("test")
        self.assertIsNone(self.app.measure_start)
        self.assertIsNone(self.app.pending_action)
        self.assertEqual(self.app.measure_phase, "idle")

    def test_stop_cancels_calibration_wait_without_late_reapply(self):
        import time
        self.receive(calibrated=False, raw_count=402,
                     event="calibration_requires_disarmed_stationary")
        self.app.measure_phase = "applying"
        self.app.calibration_retry = (.2, 1, 402, time.monotonic() + 1)
        self.app.tick_extra()
        self.assertEqual(self.commands(), [{"cmd": "calibrate", "mm_per_count": .2,
                                           "encoder_sign": 1, "ab_verified": True}])
        self.app.disarm()
        self.commands()
        self.app.tick_extra()
        self.assertEqual(self.commands(), [])
        self.assertEqual(self.app.measure_phase, "down")

    def test_keyboard_full_pwm_both_keys_release_and_typing_exclusion(self):
        def key(name, widget=None):
            return types.SimpleNamespace(keysym=name, widget=widget or self.app.canvas)
        self.receive()
        self.assertTrue(self.root.bind("<KeyPress-Up>"))
        entry = types.SimpleNamespace(winfo_class=lambda: "TEntry")
        self.app.key_press(key("Up", entry))
        self.assertEqual(self.commands(), [])
        self.app.pwm.set(100)
        self.app.key_press(key("Up"))
        self.assertEqual(self.commands(), [{"cmd": "arm"}])
        self.receive(armed=True)
        self.assertEqual(self.commands(), [{"cmd": "jog", "direction": 1, "duty": 1.0}])
        self.app.key_press(key("Down"))
        self.assertEqual(self.commands(), [{"cmd": "stop"}])
        self.app.key_release(key("Up"))
        self.root.after(65, self.root.quit); self.root.mainloop()
        self.assertEqual(self.app.direction, -1)
        self.assertIn({"cmd": "jog", "direction": -1, "duty": 1.0}, self.commands())
        self.app.key_release(key("Down"))
        self.root.after(65, self.root.quit); self.root.mainloop()
        self.assertFalse(self.app.held)
        self.assertEqual(self.commands()[-1], {"cmd": "stop"})

    def test_reboot_and_repeat_cannot_restart_held_keyboard(self):
        key = types.SimpleNamespace(keysym="Up", widget=self.app.canvas)
        self.receive()
        self.app.key_press(key)
        self.commands()
        self.receive(armed=True)
        self.commands()
        self.receive(armed=False, event="boot_unhomed")
        self.commands()
        self.app.key_release(key)
        self.app.key_press(key)  # X11 repeat pair, while physical key is still held.
        self.app.tick_extra()
        self.assertEqual(self.commands(), [])
        self.assertFalse(self.app.held)
        self.app.key_release(key)
        self.root.after(65, self.root.quit); self.root.mainloop()
        self.app.key_press(key)
        self.assertEqual(self.commands(), [{"cmd": "arm"}])

    def test_saved_calibration_prompts_only_zero_without_auto_motion(self):
        self.receive(calibrated=True, calibration_saved=True, event="saved_calibration_loaded")
        self.assertIn("0 mm", self.app.setup_text.get())
        self.assertIn("저장", self.app.setup_text.get())
        self.assertEqual(str(self.app.zero_button["state"]), "normal")
        self.assertEqual(str(self.app.move_button["state"]), "disabled")
        self.assertEqual(self.commands(), [])

        self.receive(calibrated=True, calibration_saved=True, event="disarmed")
        self.assertIn("0 mm", self.app.event_text.get())
        self.app.disconnect("test")
        self.app.connect()
        self.commands()
        self.receive(calibrated=True, calibration_saved=True, event="saved_calibration_loaded")
        self.assertEqual(self.app.measure_phase, "idle")
        self.assertEqual(self.commands(), [])

    def test_pd_settings_are_snapshotted_and_old_firmware_cannot_claim_pd(self):
        self.receive(homed=True, state="IDLE", position_mm=200)
        self.app.kp.set("0.01"); self.app.kd.set("0.002")
        self.app.pwm.set(60)
        self.app.move_to()
        self.assertEqual(self.commands(), [{"cmd": "arm"}])
        self.app.kp.set("0.02")  # Editing while waiting cannot change the queued move.
        self.app.pwm.set(90)
        self.receive(homed=True, state="IDLE", position_mm=200, armed=True)
        command = self.commands()[-1]
        self.assertEqual((command["kp"], command["kd"], command["max_duty"]), (.01, .002, .6))
        self.app.latest.pop("position_control")
        self.app.move_to()
        self.assertEqual(self.commands(), [])
        self.assertIn("펌웨어", self.app.event_text.get())

    def test_startup_defaults_and_edits_are_not_overwritten_by_saved_device_tuning(self):
        profile={"kp":.012,"kd":.003,"max_duty":.4}
        self.receive(calibration_saved=True,event="saved_calibration_loaded",saved_control=profile)
        self.assertEqual(float(self.app.kp.get()),.04)
        self.assertEqual(self.app.pwm.get(),100)
        self.app.kp.set("0.02")
        self.receive(calibration_saved=True,event="saved_calibration_loaded",saved_control=profile)
        self.assertEqual(self.app.kp.get(),"0.02")
        self.receive(event="control_saved",saved_control=profile)
        self.app.kp.set("0.03")
        self.receive(event="control_saved",saved_control=profile)
        self.assertEqual(self.app.kp.get(),"0.03")

    def test_invalid_editable_settings_never_arm_or_start_a_move(self):
        self.receive(homed=True, state="IDLE", position_mm=200)
        for kp, kd in (("nan", "0.001"), ("0", "0.001"), ("0.04", "-1"), ("abc", "0.001")):
            self.app.kp.set(kp); self.app.kd.set(kd)
            self.app.move_to()
            self.assertEqual(self.commands(), [])

    def test_editable_pwm_applies_to_manual_move_without_automatic_start(self):
        self.receive()
        self.app.pwm.set(35)
        self.assertEqual(self.commands(), [])
        self.app.start_manual(1)
        self.assertEqual(self.commands(), [{"cmd": "arm"}])
        self.receive(armed=True)
        self.assertEqual(self.commands(), [{"cmd": "jog", "direction": 1, "duty": .35}])
        self.app.release()
        self.assertIn({"cmd": "stop"}, self.commands())

    def test_pico_fault_log_preserves_the_requested_kp_and_is_not_a_disconnect(self):
        self.receive(homed=True,state="IDLE",position_mm=0)
        self.app.kp.set("0.015");self.app.pwm.set(100);self.app.target.set("200")
        self.app.move_to();self.commands()
        self.receive(homed=True,state="IDLE",position_mm=0,armed=True)
        self.commands()
        self.receive(homed=True,state="MOVING",position_mm=199.0,target_mm=200,
                     armed=True,duty=.1,direction=1,kp=.015)
        self.receive(homed=False,state="FAULT",position_mm=None,target_mm=None,
                     fault="encoder_no_pulse",event="encoder_no_pulse",armed=False)
        record=json.loads(self.app.diag_path.read_text().splitlines()[-1])
        self.assertEqual(record["kind"],"pico_fault")
        self.assertEqual(record["reason"],"encoder_no_pulse")
        self.assertEqual(record["last_command"]["kp"],.015)
        self.assertEqual(record["last_command"]["target_mm"],200)
        self.assertEqual(record["last_command"]["max_duty"],1)
        self.assertEqual(record["recent_status"][-2]["position_mm"],199.0)
        self.assertIsNotNone(self.app.serial)
        self.assertIn("무펄스",self.app.event_text.get())

    def test_watchdog_reset_during_session_is_logged_and_does_not_rearm(self):
        self.arm_and_home()
        self.receive(armed=False,homed=False,state="UNHOMED",position_mm=None,
                     event="saved_calibration_loaded",boot_reason="watchdog")
        self.assertFalse(self.app.armed)
        self.assertIn("감시 타이머",self.app.event_text.get())
        record=json.loads(self.app.diag_path.read_text().splitlines()[-1])
        self.assertEqual((record["kind"],record["reason"]),("pico_connected","watchdog"))
        self.assertNotIn("arm",[cmd["cmd"] for cmd in self.commands()])

    def test_timeout_logs_recent_status_and_rejected_frames_then_disarms(self):
        self.receive(homed=True,state="MOVING",position_mm=199.0,target_mm=200,armed=True,
                     duty=.12,direction=1,kp=.015,kd=.001)
        self.receive(homed=True,state="MOVING",position_mm=199.0,raw_count="bad")
        self.app.last_rx=time.monotonic()-1
        self.app.tick()
        self.assertIsNone(self.app.serial)
        self.assertIsNone(self.app.latest)
        self.assertIn("연결이 끊겼어요",self.app.event_text.get())
        record=json.loads(self.app.diag_path.read_text().splitlines()[-1])
        self.assertEqual(record["kind"],"connection_lost")
        self.assertEqual(record["reason"],"피코 응답 만료")
        self.assertEqual(record["invalid_frames"]["position_fields"],1)
        self.assertEqual(record["recent_status"][-1]["position_mm"],199.0)
        self.assertEqual(record["recent_status"][-1]["kp"],.015)
        self.assertIn({"cmd":"disarm"},self.commands())

    def test_write_error_is_not_overwritten_by_timeout(self):
        self.receive()
        self.app.last_rx=time.monotonic()-1
        with patch.object(self.app.serial,"write",side_effect=OSError("usb write failed")):
            self.app.tick()
        self.assertIsNone(self.app.serial)
        self.assertIn("송신 실패",self.app.status.get())
        record=json.loads(self.app.diag_path.read_text().splitlines()[-1])
        self.assertIn("usb write failed",record["reason"])
        self.assertEqual(record["kind"],"connection_lost")

    def test_diagnostic_write_failure_cannot_prevent_disconnect(self):
        self.receive()
        self.app.diag_path=Path("/dev/null/not-a-directory/diag.jsonl")
        self.app.disconnect("test")
        self.assertIsNone(self.app.serial)
        self.assertTrue(self.app.diag_log_error)

    def test_watchdog_boot_is_logged_without_automatic_motion(self):
        self.receive(event="saved_calibration_loaded",calibration_saved=True,boot_reason="watchdog")
        self.assertIn("감시 타이머",self.app.event_text.get())
        record=json.loads(self.app.diag_path.read_text().splitlines()[-1])
        self.assertEqual(record["kind"],"pico_connected")
        self.assertEqual(record["reason"],"watchdog")
        self.assertEqual(self.commands(),[])

    def test_fault_reason_survives_disarm_and_reconnect_until_cleared(self):
        self.receive(calibration_saved=True, homed=False, state="FAULT", position_mm=None,
                     event="encoder_no_pulse", fault="encoder_no_pulse")
        self.assertIn("무펄스", self.app.event_text.get())
        self.assertTrue(self.app.fault_button.winfo_manager())
        self.receive(calibration_saved=True, homed=False, state="FAULT", position_mm=None,
                     event="disarmed", fault="encoder_no_pulse")
        self.assertIn("무펄스", self.app.event_text.get())
        self.receive(calibration_saved=True, homed=False, state="UNHOMED", position_mm=None,
                     event="cleared", fault="")
        self.assertFalse(self.app.fault_button.winfo_manager())
        self.assertIn("0 mm", self.app.event_text.get())

    def test_old_position_firmware_cannot_start_automatic_or_manual_motion(self):
        self.receive(homed=True,state="IDLE",position_mm=200)
        self.app.latest.pop("control_revision")
        self.app.move_to()
        self.assertEqual(self.commands(),[])
        self.assertIn("펌웨어",self.app.event_text.get())
        self.app.start_manual(1)
        self.assertFalse(self.app.held)
        self.assertEqual(self.commands(),[])

    def test_forget_reference_requires_confirmation_and_supported_firmware(self):
        self.receive(homed=True,state="IDLE",position_mm=200,reference_clear_available=True)
        with patch("lift_position.messagebox.askyesno",return_value=True):
            self.app.forget_zero()
        self.assertEqual(self.commands()[-1],{"cmd":"forget_zero"})
        self.app.latest.pop("reference_clear_available")
        with patch("lift_position.messagebox.askyesno",return_value=True):
            self.app.forget_zero()
        self.assertEqual(self.commands(),[])

    def test_save_settings_disarms_and_snapshots_tuning(self):
        self.receive(calibration_saved=True,control_storage_available=True)
        self.app.save_control()
        commands=self.commands()
        self.assertEqual(commands[-1],dict(cmd="save_control",kp=.04,kd=.001,max_duty=1.0))
        self.assertIn(dict(cmd="disarm"),commands)
        self.app.latest.pop("control_storage_available")
        self.app.save_control()
        self.assertEqual(self.commands(),[])



if __name__ == "__main__":
    unittest.main()

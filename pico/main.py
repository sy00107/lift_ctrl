"""MicroPython lift position control. Manual zero; encoder soft limits 0..690 mm.

This is a separate firmware/protocol from the existing jog/PWM versions.
No physical end-stop sensors are assumed. Never home by driving into a stall.
"""
import json
import math
import os
import select
import sys
import machine
import micropython
import utime

RPWM_PIN, LPWM_PIN, R_EN_PIN, L_EN_PIN = 2, 3, 4, 5
ENC_A_PIN, ENC_B_PIN = 27, 28
# Set only after measuring THIS mechanism with A-rising-edge (x1) decoding.
# Runtime calibration is saved on the Pico. Zero/current position are never saved.
MM_PER_COUNT = None
ENCODER_SIGN = 1
ENCODER_AB_VERIFIED = False
PROTOCOL = "yor-lift-position-v1"
STROKE_MM = 690.0
MAX_DUTY = 1.0
MIN_POSITION_DUTY = 0.10
# Provisional breakaway compensation; measure the real lift before tuning further.
POSITION_DRIVE_BIAS = 0.20
POSITION_KP = 0.006
POSITION_KD = 0.001  # Initial value only; duty per (mm/s), requires hardware tuning.
MAX_POSITION_KP = 0.1
MAX_POSITION_KD = 0.1
VELOCITY_SAMPLE_MS = 20
VELOCITY_FILTER_MS = 80
DUTY_RAMP_PER_SECOND = 0.5
TOLERANCE_MM = 0.2
# Keep end-of-travel and plausibility checks independent of arrival accuracy.
END_TOLERANCE_MM = 1.0
POSITION_EPSILON_MM = 1e-9
SETTLE_SPEED_MM_S = 2.0
SETTLE_MS = 200
STATIONARY_MS = 300
LEASE_MS = 300
NO_PULSE_MS = 800
MOVE_TIMEOUT_MS = 120000
# Commissioning defaults, not a measured stopping-distance guarantee.
WORK_MARGIN_MM = 10.0
APPROACH_DECEL_MM_S2 = 12.0
APPROACH_REACTION_S = 0.2
# Duty correction per mm/s of approach speed error. Simulation-tuned starting value.
APPROACH_SPEED_GAIN = 0.06
APPROACH_FINE_DISTANCE_MM = 10.0
APPROACH_FINE_SPEED_RATIO = 0.30
MAX_SPEED_MM_S = 38.1
ARRIVAL_DRIFT_MM = 2.0
PROGRESS_WINDOW_MS = 1500
PROGRESS_MIN_MM = 0.5
# Keep ISR arithmetic inside MicroPython's small-int range even before masking.
COUNTER_MODULUS = 1 << 29


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def valid_control(kp, kd, duty):
    return (number(kp) and 0 < kp <= MAX_POSITION_KP
            and number(kd) and 0 <= kd <= MAX_POSITION_KD
            and number(duty) and MIN_POSITION_DUTY <= duty <= MAX_DUTY)


def reset_reason():
    """Report the latest MCU reset cause when supported by this MicroPython port."""
    try:
        cause = machine.reset_cause()
        for name, label in (("WDT_RESET", "watchdog"), ("PWRON_RESET", "power_on"),
                            ("HARD_RESET", "hard_reset"), ("SOFT_RESET", "soft_reset")):
            value = getattr(machine, name, None)
            if value is not None and cause == value:
                return label
        return "other"
    except (AttributeError, OSError, ValueError):
        return "unknown"


class CalibrationStore:
    """Persist only calibration, bound to this Pico and encoder decoding scheme."""

    def __init__(self, path="lift_calibration.json", device_id="", sync=None):
        self.path, self.device_id, self.sync = path, device_id, sync

    def record(self, factor, sign):
        return {"version": 1, "device_id": self.device_id, "decoder": "a_rising_b_x1",
                "encoder_pins": [ENC_A_PIN, ENC_B_PIN], "stroke_mm": STROKE_MM,
                "mm_per_count": factor, "encoder_sign": sign, "ab_verified": True}

    def validate(self, data):
        if (not isinstance(data, dict) or type(data.get("version")) is not int
                or data.get("version") != 1 or not self.device_id
                or data.get("device_id") != self.device_id
                or data.get("decoder") != "a_rising_b_x1"
                or data.get("encoder_pins") != [ENC_A_PIN, ENC_B_PIN]
                or data.get("stroke_mm") not in (630.0, STROKE_MM)
                or not number(data.get("mm_per_count")) or not 0 < data["mm_per_count"] <= 1
                or type(data.get("encoder_sign")) is not int or data["encoder_sign"] not in (-1, 1)
                or data.get("ab_verified") is not True):
            raise ValueError("Invalid lift calibration")
        result = self.record(data["mm_per_count"], data["encoder_sign"])
        control = data.get("control")
        # Corrupt optional tuning must not invalidate a good encoder calibration.
        if isinstance(control, dict) and valid_control(control.get("kp"), control.get("kd"), control.get("max_duty")):
            result["control"] = {key: control[key] for key in ("kp", "kd", "max_duty")}
        return result

    def load(self):
        try:
            with open(self.path) as source:
                return self.validate(json.load(source))
        except OSError as error:
            if error.args and error.args[0] == 2:  # ENOENT: first use, not a fault.
                return None
            raise

    def save(self, factor, sign):
        self.save_record(self.record(factor, sign))

    def save_control(self, kp, kd, duty):
        if not valid_control(kp, kd, duty):
            raise ValueError("Invalid tuning")
        data = self.load()
        if data is None:
            raise ValueError("Save calibration before tuning")
        data["control"] = {"kp": kp, "kd": kd, "max_duty": duty}
        self.save_record(data)

    def save_record(self, data):
        data = self.validate(data)
        try:
            if self.load() == data:
                return  # Repeated stationary retries must not wear the flash.
        except (ValueError, OSError):
            pass
        temporary = self.path + ".tmp"
        try:
            with open(temporary, "w") as output:
                json.dump(data, output)
                output.flush()
            if self.sync:
                self.sync()
            os.rename(temporary, self.path)
            if self.sync:
                self.sync()
        finally:
            try:
                os.remove(temporary)
            except OSError:
                pass


class PositionController:
    """Hardware-independent controller; encoder() returns (signed raw, activity)."""

    def __init__(self, drive, now, encoder, mm_per_count=MM_PER_COUNT,
                 encoder_sign=ENCODER_SIGN, ab_verified=ENCODER_AB_VERIFIED,
                 calibration_store=None, boot_reason="unknown"):
        self.drive, self.now, self.encoder = drive, now, encoder
        self.drive(0, 0)
        self.calibration_store = calibration_store
        self.boot_reason = boot_reason
        self.calibration_saved = False
        self.saved_control = None
        self.mm_per_count = mm_per_count
        self.encoder_sign = encoder_sign
        self.ab_verified = ab_verified
        self.armed = False
        self.fault = ""
        self.event = "boot_unhomed"
        self.mode = "idle"
        self.direction, self.duty = 0, 0.0
        self.target_mm = None
        self.origin_count = None
        self.raw, self.pulses = encoder()
        self.count = 0
        self.velocity = 0.0
        self.kp, self.kd = POSITION_KP, POSITION_KD
        self.wrong_distance = 0.0
        self.move_direction = 0
        self.max_duty = 0.25
        self.last_update = self.last_pulse = self.last_command = now()
        self.velocity_time, self.velocity_count = self.last_update, self.count
        self.started = self.settle_start = self.last_update
        self.arrival_target = None
        self.progress_drive_ms = 0
        self.progress_count = self.count
        self.progress_direction = 0
        self.no_pulse_drive_ms = 0
        self.jog_direction, self.jog_duty = 0, 0.0
        if calibration_store is not None:
            try:
                saved = calibration_store.load()
                if saved is not None:
                    self.mm_per_count = saved["mm_per_count"]
                    self.encoder_sign, self.ab_verified = saved["encoder_sign"], True
                    self.calibration_saved = True
                    self.saved_control = saved.get("control")
                    if self.saved_control:
                        self.kp, self.kd = self.saved_control["kp"], self.saved_control["kd"]
                        self.max_duty = self.saved_control["max_duty"]
                    self.event = "saved_calibration_loaded"
            except (OSError, ValueError, TypeError):
                self.mm_per_count, self.ab_verified = None, False
                self.event = "calibration_storage_invalid"

    @property
    def calibrated(self):
        return (self.ab_verified is True and number(self.mm_per_count)
                and 0 < self.mm_per_count <= 1.0
                and type(self.encoder_sign) is int and self.encoder_sign in (-1, 1))

    @property
    def homed(self):
        return self.origin_count is not None and self.calibrated

    @property
    def position_mm(self):
        if not self.homed:
            return None
        return (self.count - self.origin_count) * self.mm_per_count * self.encoder_sign

    def output(self, direction, duty):
        self.direction, self.duty = direction, duty
        self.drive(direction, duty)

    def stop(self, event="stopped"):
        self.output(0, 0)
        if event == "target_reached":
            self.arrival_target = self.target_mm
        self.mode, self.target_mm = "idle", None
        self.event = event

    def fail(self, reason, lose_zero=False):
        self.stop(reason)
        self.armed = False
        self.fault = reason
        self.arrival_target = None
        if lose_zero:
            self.origin_count = None

    def reset_velocity(self):
        self.velocity = 0.0
        self.velocity_time, self.velocity_count = self.now(), self.count

    def position_duty(self, error):
        # D on measured velocity avoids a derivative kick when the target changes.
        forward = self.move_direction * (self.kp * error - self.kd * self.velocity)
        if forward <= 0:
            return 0.0  # D may coast, but never reverse the motor within one move.
        # Existing minimum-drive compensation overcomes friction only when PD asks
        # for forward motion. It must not override a zero/negative PD demand.
        return min(self.max_duty, forward + POSITION_DRIVE_BIAS)

    def approach_duty(self, duty, remaining, tolerance=TOLERANCE_MM,
                      floor=MIN_POSITION_DUTY, smooth=False):
        # v*t_reaction + v^2/(2*a) <= remaining. Drive limiting, NOT a brake.
        available = max(0.0, remaining - tolerance)
        reaction = APPROACH_DECEL_MM_S2 * APPROACH_REACTION_S
        ceiling = min(MAX_SPEED_MM_S,
                      math.sqrt(reaction * reaction + 2 * APPROACH_DECEL_MM_S2 * available)
                      - reaction)
        if smooth and ceiling < MAX_SPEED_MM_S:
            # A hard velocity threshold alternates drive/coast as quantized
            # encoder speeds cross it. Use continuous negative speed feedback
            # instead; large overspeed can still coast, with no forced floor.
            forward_speed = max(0.0, self.velocity * self.move_direction)
            # Blend extra coast/quantization headroom into the last 10 mm,
            # continuously; do not introduce a new switch at the fine boundary.
            blend = min(1.0, max(0.0, remaining) / APPROACH_FINE_DISTANCE_MM)
            desired_speed = ceiling * (APPROACH_FINE_SPEED_RATIO
                                      + (1 - APPROACH_FINE_SPEED_RATIO) * blend)
            cap = floor + (1 - floor) * desired_speed / MAX_SPEED_MM_S
            cap += APPROACH_SPEED_GAIN * (desired_speed - forward_speed)
            return max(0.0, min(duty, cap))
        if self.velocity * self.move_direction >= ceiling:
            return 0.0
        if ceiling >= MAX_SPEED_MM_S:
            return duty
        return min(duty, floor + (1 - floor) * ceiling / MAX_SPEED_MM_S)

    def reset_progress(self, direction):
        self.progress_drive_ms = 0
        self.progress_count = self.count
        self.progress_direction = direction

    def output_jog(self, pos):
        duty = self.jog_duty
        self.move_direction = self.jog_direction
        if pos is not None:
            remaining = (STROKE_MM - WORK_MARGIN_MM - pos if self.jog_direction > 0
                         else pos - WORK_MARGIN_MM)
            duty = self.approach_duty(duty, remaining, END_TOLERANCE_MM)
        self.output(self.jog_direction if duty > 0 else 0, duty)

    def update(self):
        now = self.now()
        dt = max(0, utime.ticks_diff(now, self.last_update))
        raw, pulses = self.encoder()
        delta = ((raw - self.raw + COUNTER_MODULUS // 2) % COUNTER_MODULUS
                 - COUNTER_MODULUS // 2)
        self.raw, self.count = raw, self.count + delta
        had_pulse = pulses != self.pulses
        if had_pulse:
            self.last_pulse = now
            self.no_pulse_drive_ms = 0
        self.pulses = pulses
        distance = delta * self.mm_per_count * self.encoder_sign if self.calibrated else 0.0
        sample_ms = utime.ticks_diff(now, self.velocity_time)
        if sample_ms >= VELOCITY_SAMPLE_MS:
            # Accumulate counts across fast/repeated command updates (including
            # pulses arriving in the same millisecond); filter by elapsed time.
            travel = ((self.count - self.velocity_count) * self.mm_per_count * self.encoder_sign
                      if self.calibrated else 0.0)
            measured = travel * 1000 / sample_ms
            alpha = sample_ms / (VELOCITY_FILTER_MS + sample_ms)
            self.velocity += alpha * (measured - self.velocity)
            self.velocity_time, self.velocity_count = now, self.count
        self.last_update = now
        pos = self.position_mm
        if pos is not None and not -END_TOLERANCE_MM <= pos <= STROKE_MM + END_TOLERANCE_MM:
            self.fail("position_out_of_range", lose_zero=True)
            return
        if self.arrival_target is not None and pos is not None and not self.fault:
            if abs(pos - self.arrival_target) > ARRIVAL_DRIFT_MM:
                self.fail("position_drift")
                return
        active = self.mode in ("jog", "moving", "settling")
        if not active:
            return
        if utime.ticks_diff(now, self.last_command) >= LEASE_MS:
            self.fail("command_timeout")
            return
        if utime.ticks_diff(now, self.started) >= MOVE_TIMEOUT_MS:
            self.fail("move_timeout")
            return
        # Known encoder boundary is a normal directional stop, even when the
        # last powered interval also reaches the no-pulse timeout. Unknown
        # position and mid-stroke stalls still invalidate the reference.
        if self.mode == "jog" and pos is not None:
            if ((self.jog_direction < 0 and pos <= WORK_MARGIN_MM + END_TOLERANCE_MM)
                    or (self.jog_direction > 0 and pos >= STROKE_MM - WORK_MARGIN_MM - END_TOLERANCE_MM)):
                self.stop("soft_min" if self.jog_direction < 0 else "soft_max")
                return
        if self.direction:
            if self.calibrated:
                self.wrong_distance = (self.wrong_distance + abs(distance)
                                       if distance * self.direction < 0 else
                                       0.0 if distance else self.wrong_distance)
                if self.wrong_distance >= max(1.0, 3 * self.mm_per_count):
                    self.fail("encoder_direction_mismatch", lose_zero=True)
                    return
            if not had_pulse:
                self.no_pulse_drive_ms += dt
            if self.no_pulse_drive_ms >= NO_PULSE_MS:
                # Could be jam, disconnected encoder, or an end stop: never re-zero.
                self.fail("encoder_no_pulse", lose_zero=True)
                return
            if self.calibrated:
                # Powered time only; coasting does not erase accumulated time.
                self.progress_drive_ms += dt
                if self.progress_drive_ms >= PROGRESS_WINDOW_MS:
                    progress = ((self.count - self.progress_count) * self.mm_per_count
                                * self.encoder_sign * self.progress_direction)
                    required = max(PROGRESS_MIN_MM, self.mm_per_count)
                    if self.mode == "moving" and abs(self.target_mm - pos) <= 1.0:
                        # The fine approach may have less than 0.5 mm left.
                        # Still require a net count; chatter alone does not pass.
                        required = self.mm_per_count
                    if progress + POSITION_EPSILON_MM < required:
                        self.fail("motion_no_progress", lose_zero=True)
                        return
                    self.reset_progress(self.progress_direction)
        if self.mode == "jog":
            self.output_jog(pos)
            return
        if self.mode == "settling":
            if abs(self.target_mm - pos) > TOLERANCE_MM + POSITION_EPSILON_MM:
                self.fail("target_not_settled")
            elif abs(self.velocity) > SETTLE_SPEED_MM_S:
                self.settle_start = now
            elif utime.ticks_diff(now, self.settle_start) >= SETTLE_MS:
                self.stop("target_reached")
            return
        error = self.target_mm - pos
        if abs(error) <= TOLERANCE_MM + POSITION_EPSILON_MM:
            self.output(0, 0)
            self.mode, self.settle_start = "settling", now
        elif error * self.move_direction < 0:
            self.fail("target_overshoot")
        else:
            desired = self.approach_duty(self.position_duty(error), abs(error),
                                         floor=POSITION_DRIVE_BIAS, smooth=True)
            duty = min(desired, self.duty + DUTY_RAMP_PER_SECOND * dt / 1000)
            self.output(self.move_direction if duty > 0 else 0, duty)

    def command(self, message):
        # A late heartbeat must not conceal an expired motion lease.
        self.update()
        if not isinstance(message, dict):
            self.fail("bad_command")
            return
        cmd = message.get("cmd")
        if cmd == "stop":
            self.stop()
        elif cmd == "disarm":
            self.stop("disarmed")
            self.armed = False
        elif cmd == "clear":
            if self.mode == "idle":
                self.fault = ""
                self.armed = False
                self.event = "cleared"
        elif cmd == "arm":
            if not self.fault and self.mode == "idle":
                self.armed = True
                self.last_command = self.now()
        elif cmd == "heartbeat":
            if self.armed and not self.fault:
                self.last_command = self.now()
        elif cmd == "save_control":
            if (self.armed or self.fault or self.mode != "idle" or not self.calibrated
                    or utime.ticks_diff(self.now(), self.last_pulse) < STATIONARY_MS):
                self.event = "control_save_requires_stationary"
                return
            if not self.calibration_saved:
                self.event = "control_save_requires_calibration"
                return
            kp, kd, duty = message.get("kp"), message.get("kd"), message.get("max_duty")
            if not valid_control(kp, kd, duty):
                self.event = "bad_pd_gains"
                return
            if self.calibration_store is None:
                self.event = "control_storage_unavailable"
                return
            try:
                self.calibration_store.save_control(kp, kd, duty)
            except (OSError, ValueError, TypeError):
                self.fail("control_save_failed")
                return
            self.kp, self.kd, self.max_duty = kp, kd, duty
            self.saved_control = {"kp": kp, "kd": kd, "max_duty": duty}
            self.event = "control_saved"
        elif cmd == "calibrate":
            factor, sign = message.get("mm_per_count"), message.get("encoder_sign")
            if (self.armed or self.fault or self.mode != "idle"
                    or utime.ticks_diff(self.now(), self.last_pulse) < STATIONARY_MS):
                self.event = "calibration_requires_disarmed_stationary"
                return
            if (not number(factor) or not 0 < factor <= 1.0
                    or type(sign) is not int or sign not in (-1, 1)
                    or message.get("ab_verified") is not True):
                self.fail("bad_calibration")
                return
            if self.calibration_store is not None:
                try:
                    self.calibration_store.save(factor, sign)
                except (OSError, ValueError, TypeError):
                    self.fail("calibration_save_failed", lose_zero=True)
                    return
                self.calibration_saved = True
            self.mm_per_count, self.encoder_sign, self.ab_verified = factor, sign, True
            self.saved_control = None
            self.kp, self.kd, self.max_duty = POSITION_KP, POSITION_KD, 0.25
            self.arrival_target = None
            self.origin_count = None
            self.reset_velocity()
            self.event = "calibrated_unhomed"
        elif cmd == "zero":
            if (self.armed or self.fault or self.mode != "idle" or not self.calibrated
                    or utime.ticks_diff(self.now(), self.last_pulse) < STATIONARY_MS
                    or message.get("at_lower_reference") is not True):
                self.event = "zero_requires_calibration_disarmed_stationary_and_reference"
                return
            self.origin_count = self.count
            self.arrival_target = None
            self.reset_velocity()
            self.event = "manual_zero_set"
        elif cmd == "forget_zero":
            if (self.armed or self.fault or self.mode != "idle"
                    or utime.ticks_diff(self.now(), self.last_pulse) < STATIONARY_MS):
                self.event = "reference_clear_requires_stationary"
                return
            self.origin_count = None
            self.arrival_target = None
            self.reset_velocity()
            self.event = "reference_cleared"
        elif cmd == "move_to":
            target, duty = message.get("target_mm"), message.get("max_duty", self.max_duty)
            kp, kd = message.get("kp", self.kp), message.get("kd", self.kd)
            if (not number(target) or not 0 <= target <= STROKE_MM
                    or not number(duty) or not MIN_POSITION_DUTY <= duty <= MAX_DUTY):
                self.fail("bad_target")
                return
            if (not number(kp) or not 0 < kp <= MAX_POSITION_KP
                    or not number(kd) or not 0 <= kd <= MAX_POSITION_KD):
                self.fail("bad_pd_gains")
                return
            if not WORK_MARGIN_MM <= target <= STROKE_MM - WORK_MARGIN_MM:
                self.event = "target_near_endpoint"
                return
            if not self.armed or self.fault or not self.homed:
                self.event = "move_requires_armed_and_homed"
                return
            if self.mode != "idle":
                # Repeating a target does not restart timers or restart settling.
                self.event = "busy_stop_before_new_target"
                return
            nearest = math.floor(target / self.mm_per_count + 0.5) * self.mm_per_count
            if abs(nearest - target) > TOLERANCE_MM + POSITION_EPSILON_MM:
                # Refuse a target that the calibrated encoder cannot represent
                # within the requested arrival tolerance.
                self.event = "target_encoder_resolution"
                return
            self.target_mm, self.max_duty = target, duty
            self.arrival_target = None
            self.kp, self.kd = kp, kd
            self.move_direction = 1 if target >= self.position_mm else -1
            self.mode = "moving"
            self.event = "moving"
            self.started = self.last_command = self.last_pulse = self.now()
            self.no_pulse_drive_ms = 0
            self.wrong_distance = 0.0
            self.reset_progress(self.move_direction)
        elif cmd == "jog":
            direction, duty = message.get("direction"), message.get("duty")
            if (type(direction) is not int or direction not in (-1, 1)
                    or not number(duty) or not 0 < duty <= MAX_DUTY):
                self.fail("bad_jog")
                return
            if not self.armed or self.fault:
                return
            if self.mode not in ("idle", "jog"):
                self.event = "busy_stop_before_jog"
                return
            pos = self.position_mm
            if pos is not None and ((direction < 0 and pos <= WORK_MARGIN_MM + END_TOLERANCE_MM)
                                    or (direction > 0 and pos >= STROKE_MM - WORK_MARGIN_MM - END_TOLERANCE_MM)):
                self.stop("soft_min" if direction < 0 else "soft_max")
                return
            if self.mode == "idle":
                self.started = self.last_pulse = self.now()
                self.no_pulse_drive_ms = 0
                self.wrong_distance = 0.0
                self.arrival_target = None
                self.reset_progress(direction)
            elif direction != self.progress_direction:
                self.fail("jog_reverse_requires_stop")
                return
            self.last_command = self.now()
            self.mode, self.event = "jog", "jog"
            self.jog_direction, self.jog_duty = direction, duty
            self.output_jog(pos)
        else:
            self.fail("bad_command")

    def status(self):
        pos = self.position_mm
        return {"protocol": PROTOCOL, "armed": self.armed,
                "state": "FAULT" if self.fault else self.mode.upper() if self.mode != "idle"
                else "IDLE" if self.homed else "UNHOMED",
                "fault": self.fault, "event": self.event,
                "direction": self.direction, "duty": self.duty,
                "pulses": self.pulses, "raw_count": self.count,
                "calibrated": self.calibrated, "homed": self.homed,
                "calibration_saved": self.calibration_saved,
                "position_control": "PD", "control_revision": 3, "kp": self.kp, "kd": self.kd,
                "boot_reason": self.boot_reason,
                "arrival_tolerance_mm": TOLERANCE_MM,
                "arrival_target_mm": self.arrival_target,
                "saved_control": self.saved_control,
                "control_storage_available": self.calibration_store is not None,
                "reference_clear_available": True,
                "work_min_mm": WORK_MARGIN_MM, "work_max_mm": STROKE_MM - WORK_MARGIN_MM,
                "position_mm": pos, "target_mm": self.target_mm,
                "velocity_mm_s": self.velocity, "min_mm": 0.0, "max_mm": STROKE_MM,
                "at_min": pos is not None and pos <= END_TOLERANCE_MM,
                "at_max": pos is not None and pos >= STROKE_MM - END_TOLERANCE_MM,
                "limit_source": "encoder_soft", "physical_limits_available": False}


def run():
    micropython.alloc_emergency_exception_buf(100)
    ren = machine.Pin(R_EN_PIN, machine.Pin.OUT, value=0)
    len_ = machine.Pin(L_EN_PIN, machine.Pin.OUT, value=0)
    rpwm, lpwm = machine.PWM(machine.Pin(RPWM_PIN)), machine.PWM(machine.Pin(LPWM_PIN))
    rpwm.freq(20000)
    lpwm.freq(20000)
    rpwm.duty_u16(0)
    lpwm.duty_u16(0)
    ren.value(1)
    len_.value(1)
    output_direction = 0
    raw_count, pulses = 0, 0
    a = machine.Pin(ENC_A_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
    b = machine.Pin(ENC_B_PIN, machine.Pin.IN, machine.Pin.PULL_UP)

    def drive(direction, duty):
        nonlocal output_direction
        if direction != output_direction:
            rpwm.duty_u16(0)
            lpwm.duty_u16(0)
            # Include stop -> reverse; a preceding stop must not bypass deadtime.
            utime.sleep_ms(10)
        if direction == 1:
            lpwm.duty_u16(0)
            rpwm.duty_u16(int(65535 * duty))
        elif direction == -1:
            rpwm.duty_u16(0)
            lpwm.duty_u16(int(65535 * duty))
        else:
            rpwm.duty_u16(0)
            lpwm.duty_u16(0)
        output_direction = direction

    def pulse(_):
        nonlocal raw_count, pulses
        # x1: direction from B at A's rising edge, including unpowered motion.
        raw_count = (raw_count + (1 if b.value() else -1)) & (COUNTER_MODULUS - 1)
        pulses = (pulses + 1) & (COUNTER_MODULUS - 1)

    def snapshot():
        irq = machine.disable_irq()
        result = (raw_count, pulses)
        machine.enable_irq(irq)
        return result

    a.irq(trigger=machine.Pin.IRQ_RISING, handler=pulse)
    device_id = "".join("%02x" % byte for byte in machine.unique_id())
    store = CalibrationStore(device_id=device_id, sync=getattr(os, "sync", None))
    controller = PositionController(drive, utime.ticks_ms, snapshot,
                                    calibration_store=store, boot_reason=reset_reason())
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    buffer, discard = "", False
    last_report = utime.ticks_ms()
    watchdog = machine.WDT(timeout=1000)
    try:
        while True:
            controller.update()
            for _ in range(128):
                if not poll.poll(0):
                    break
                char = sys.stdin.read(1)
                if not char:
                    break
                if char == "\n":
                    if buffer and not discard:
                        try:
                            message = json.loads(buffer)
                        except (ValueError, TypeError):
                            controller.fail("bad_json")
                        else:
                            controller.command(message)
                    buffer, discard = "", False
                elif char != "\r" and not discard:
                    buffer += char
                    if len(buffer) > 256:
                        controller.fail("line_too_long")
                        buffer, discard = "", True
            controller.update()
            now = utime.ticks_ms()
            if utime.ticks_diff(now, last_report) >= 100:
                last_report = now
                print(json.dumps(controller.status()))
            watchdog.feed()
            utime.sleep_ms(10)
    finally:
        drive(0, 0)


if __name__ == "__main__":
    run()

#!/usr/bin/env python3
"""Local Tk control for pico_lift_position, manual zero and 0..690 mm targets."""
import argparse
import glob
from collections import deque
from datetime import datetime
import json
import math
import os
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk, messagebox

from lift_joystick import Joystick

PROTOCOL = "yor-lift-position-v1"
STROKE_MM = 690.0
WORK_MIN_MM, WORK_MAX_MM = 10.0, 680.0
DEFAULT_KP, DEFAULT_KD = 0.04, 0.001
DEFAULT_PWM = 100
EVENTS = {
    "boot_unhomed": "처음 설정을 마친 뒤 맨 아래에서 0 mm를 설정하세요",
    "manual_zero_set": "현재 위치를 0 mm로 등록함",
    "calibrated_unhomed": "보정 완료 · 하단 원점 등록 필요",
    "saved_calibration_loaded": "저장된 보정값을 불러왔어요. 맨 아래에서 0 mm만 설정하세요.",
    "calibration_storage_invalid": "저장된 설정을 읽지 못했어요. 처음 보정을 다시 진행하세요.",
    "calibration_save_failed": "보정값 저장 실패 · 저장 공간을 확인하고 다시 보정하세요.",
    "moving": "목표 높이로 이동 중",
    "target_reached": "목표 도달 · 정착 확인",
    "position_drift": "도착 후 위치 이탈 · 하중/브레이크 확인 후 오류를 해제하세요",
    "motion_no_progress": "실제 진행 없음 · 걸림/엔코더 확인 · 원점 재등록 필요",
    "target_near_endpoint": "끝단 여유 구간입니다. 자동 이동 목표는 10–680 mm로 입력하세요",
    "target_encoder_resolution": "엔코더 해상도로 이 목표를 0.2 mm 이내에 맞출 수 없어요",
    "jog_reverse_requires_stop": "방향 전환 전 정지하세요 · 오류 해제 후 다시 조작하세요",
    "control_save_requires_stationary": "멈춘 뒤 잠시 기다렸다가 설정 저장을 다시 누르세요",
    "control_save_requires_calibration": "먼저 엔코더 보정값을 저장하세요",
    "control_storage_unavailable": "설정 저장을 지원하는 새 Pico 펌웨어가 필요해요",
    "control_save_failed": "이동 설정 저장 실패 · 저장 공간과 보정값을 확인하세요",
    "control_saved": "PD 계수와 출력 상한을 저장했어요. 재시작 후에도 복원됩니다",
    "reference_cleared": "원점이 해제됐어요. 하단 확인 전에는 위치 제한이 적용되지 않습니다",
    "reference_clear_requires_stationary": "먼저 멈춘 뒤 원점 해제를 다시 누르세요",
    "soft_min": "하단 여유 구간 도달 · 하강 차단",
    "soft_max": "상단 여유 구간 도달 · 상승 차단",
    "encoder_no_pulse": "엔코더 무펄스 · 걸림/센서 확인 · 원점 재등록 필요",
    "encoder_direction_mismatch": "엔코더 방향 불일치 · 배선/부호 확인",
    "command_timeout": "통신 명령 만료 · 정지",
    "target_overshoot": "목표 초과 이동 · 정지",
    "target_not_settled": "정지 후 목표 오차 초과 · 정지",
    "bad_pd_gains": "PD 계수 범위를 확인하세요",
    "position_out_of_range": "측정 범위 이탈 · 원점 재등록 필요",
    "move_requires_armed_and_homed": "먼저 맨 아래에서 0 mm를 설정하세요",
    "busy_stop_before_new_target": "이동 중 · 정지 후 새 목표를 입력하세요",
    "busy_stop_before_jog": "목표 이동 중 · 정지 후 조그를 사용하세요",
    "calibration_requires_disarmed_stationary": "비활성화 후 정지 상태에서 보정하세요",
    "zero_requires_calibration_disarmed_stationary_and_reference": "보정·비활성화·정지·최하단 확인 필요",
    "jog": "버튼을 놓으면 멈춰요",
    "stopped": "정지",
    "disarmed": "멈춤",
    "cleared": "오류를 해제했어요. 원하는 동작을 다시 눌러주세요.",
}


def fit_calibration(samples):
    """Independent upward intervals (signed encoder counts, measured mm)."""
    if len(samples) != 3:
        raise ValueError("낮음·중간·높음 세 구간을 모두 측정하세요")
    if any(type(n) is not int or n == 0 or not math.isfinite(d)
           or not 30 <= d <= STROKE_MM for n, d in samples):
        raise ValueError("각 구간에서 30 mm 이상 올라간 실제 거리를 입력하세요")
    sign = 1 if samples[0][0] > 0 else -1
    if any(n * sign <= 0 for n, _ in samples):
        raise ValueError("상승 방향 카운트가 일치하지 않아요. 배선과 이동 방향을 확인하세요")
    factors = [d / abs(n) for n, d in samples]
    if any(not 0 < f <= 1 for f in factors):
        raise ValueError("거리와 센서 값이 맞지 않아요. 다시 측정하세요")
    factor = sum(abs(n)*d for n, d in samples) / sum(n*n for n, _ in samples)
    spread = (max(factors)-min(factors)) / factor
    # Provisional gross-error gate, not a claim about measurement accuracy.
    if spread > .05:
        raise ValueError("구간별 환산값 차이가 5%를 넘어요. 자·측정 기준·엔코더를 확인하고 다시 시작하세요")
    residual = max(abs(abs(n)*factor-d) for n, d in samples)
    return factor, sign, spread, residual


class PositionUI(Joystick):
    STATUS_PROTOCOL = PROTOCOL
    PWM_MAX = 100
    PWM_DEFAULT = DEFAULT_PWM
    WINDOW_TITLE = "Lift · 0–690 mm position control"
    WINDOW_GEOMETRY = "560x880"
    JOYSTICK_SCALE = 0.6
    HEADING = "리프트 높이 제어 · 0–690 mm"
    INITIAL_TELEMETRY = "원점 미등록 · 실제 끝단 센서 없음"

    def __init__(self, root, port, demo=False):
        # A small in-memory ring; disk is written only on faults/disconnects.
        self.diag_frames = deque(maxlen=50)
        self.diag_invalid = {}
        self.diag_last_command = None
        self.diag_last_fault = ""
        self.diag_log_error = None
        self.diag_path = Path(os.environ.get(
            "YOR_LIFT_LOG_DIR", str(Path.home() / ".local/state/yor-lift"))) / "diagnostics.jsonl"
        super().__init__(root, port, demo)

    def invalid_state(self, reason):
        self.diag_invalid[reason] = self.diag_invalid.get(reason, 0) + 1

    def record_diagnostic(self, kind, reason):
        record = {"time": datetime.now().astimezone().isoformat(timespec="seconds"),
                  "kind": kind, "reason": str(reason),
                  "port": self.port.get(),
                  "last_valid_age_ms": round(max(0, time.monotonic() - self.last_rx) * 1000),
                  "invalid_frames": dict(self.diag_invalid),
                  "last_command": self.diag_last_command,
                  "recent_status": list(self.diag_frames)}
        try:
            self.diag_path.parent.mkdir(parents=True, exist_ok=True)
            with self.diag_path.open("a", encoding="utf-8") as output:
                output.write(json.dumps(record, ensure_ascii=False) + "\n")
        except (OSError, ValueError) as error:
            # Diagnostics must never interfere with the stop/disconnect path.
            self.diag_log_error = str(error)

    def build_main_controls(self, panel, ui_family):
        # Transport/key handling is shared with the old joystick; no joystick is shown.
        self.canvas = tk.Canvas(panel)
        self.knob = self.canvas.create_oval(0, 0, 1, 1)
        self.root.bind_all("<ButtonRelease-1>", self.release)
        self.root.bind("<Escape>", lambda _event: self.disarm())
        self.root.bind("<FocusOut>", self.focus_out)
        for key in ("Up", "Down"):
            self.root.bind("<KeyPress-" + key + ">", self.key_press)
            self.root.bind("<KeyRelease-" + key + ">", self.key_release)
        self.build_extra_controls(panel)

    def build_extra_controls(self, panel):
        self.latest = None
        self.pending_action = None
        self.action_deadline = 0
        self.manual_widget = None
        self.blocked_keys = set()
        self.measure_start = self.measure_end = None
        self.measure_phase = "idle"
        self.measure_candidate = None
        self.measure_samples = []
        self.down_seen = False
        self.calibration_retry = None
        self.zero_deadline = None
        self.position_text = tk.StringVar(value="현재 높이: 알 수 없음")
        self.event_text = tk.StringVar(value="연결한 뒤 처음 설정을 진행하세요")
        self.target = tk.StringVar(value="200")
        self.factor = tk.StringVar(value="")
        self.encoder_sign = tk.StringVar(value="1")
        self.distance = tk.StringVar(value="")
        self.kp = tk.StringVar(value=str(DEFAULT_KP))
        self.kd = tk.StringVar(value=str(DEFAULT_KD))
        self.setup_text = tk.StringVar(value="낮음·중간·높음에서 각각 30 mm 이상 상승량을 측정해요.")
        ttk.Label(panel, text="리프트", font=("TkDefaultFont", 24, "bold")).pack(anchor="w")
        row = ttk.Frame(panel); row.pack(fill="x", pady=8)
        ttk.Label(row, textvariable=self.status, wraplength=340).pack(side="left", expand=True, fill="x")
        ttk.Button(row, text="연결", command=self.connect).pack(side="right")
        self.fault_button = ttk.Button(row, text="오류 해제", command=self.clear_error)
        ttk.Label(panel, textvariable=self.position_text, font=("TkDefaultFont", 20, "bold")).pack(pady=(12, 2))
        ttk.Label(panel, text="맨 아래 기준 · 총 행정 690 mm · 자동 이동 10–680 mm").pack()
        box = ttk.LabelFrame(panel, text="높이 이동", padding=12); box.pack(fill="x", pady=12)
        row = ttk.Frame(box)
        row.pack(fill="x")
        ttk.Entry(row, textvariable=self.target, width=10).pack(side="left")
        ttk.Label(row, text="mm").pack(side="left", padx=8)
        self.move_button = ttk.Button(row, text="이동", command=self.move_to, state="disabled")
        self.move_button.pack(side="right")
        ttk.Label(box, text="목표 오차 ±0.2 mm · 엔코더 기준").pack(anchor="w", pady=(6, 0))
        row = ttk.Frame(panel); row.pack(fill="x")
        for direction, title in ((1, "▲ 올리기"), (-1, "▼ 내리기")):
            button = ttk.Button(row, text=title)
            button.pack(side="left", fill="x", expand=True, padx=3, ipady=8)
            button.bind("<ButtonPress-1>", lambda event, d=direction: self.start_manual(d, event.widget))
        ttk.Label(panel, text="버튼 또는 ↑/↓ 키를 누르는 동안 이동 · 놓으면 정지").pack(pady=4)
        tk.Button(panel, text="정지  (Esc)", command=self.disarm, bg="#b42318", fg="white",
                  activebackground="#912018", activeforeground="white", relief="flat",
                  font=("TkDefaultFont", 15, "bold")).pack(fill="x", ipady=8, pady=8)
        self.zero_button = ttk.Button(panel, text="지금이 맨 아래예요 · 0 mm 설정", command=self.mark_zero,
                                      state="disabled")
        self.zero_button.pack(fill="x", ipady=4)
        ttk.Label(panel, textvariable=self.event_text, wraplength=460).pack(fill="x", pady=8)
        self.setup_box = ttk.LabelFrame(panel, text="높이 보정 · 세 구간 측정", padding=10)
        self.setup_box.pack(fill="x", pady=4)
        ttk.Label(self.setup_box, textvariable=self.setup_text, wraplength=490).pack(anchor="w")
        self.distance_row = ttk.Frame(self.setup_box)
        ttk.Label(self.distance_row, text="실제 올라간 거리").pack(side="left")
        ttk.Entry(self.distance_row, textvariable=self.distance, width=8).pack(side="left", padx=6)
        ttk.Label(self.distance_row, text="mm").pack(side="left")
        self.setup_button = ttk.Button(self.setup_box, text="설정 시작", command=self.setup_next)
        self.setup_button.pack(fill="x", pady=(8, 0))
        ttk.Button(self.setup_box, text="보정 취소 / 처음부터", command=self.cancel_setup).pack(fill="x", pady=(4, 0))
        self.show_details = tk.BooleanVar(value=False)
        ttk.Checkbutton(panel, text="상세 설정", variable=self.show_details,
                        command=self.toggle_details).pack(anchor="w", pady=6)
        self.details = ttk.Frame(panel)
        row = ttk.Frame(self.details); row.pack(fill="x")
        ttk.Entry(row, textvariable=self.port).pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="오류 해제", command=self.clear_error).pack(side="left", padx=4)
        ttk.Button(row, text="원점 해제", command=self.forget_zero).pack(side="left", padx=4)
        tk.Scale(self.details, from_=self.PWM_MIN, to=self.PWM_MAX, orient="horizontal",
                 variable=self.pwm, label="PWM 상한 (%) · 수동 이동에도 적용").pack(fill="x")
        row = ttk.Frame(self.details); row.pack(fill="x", pady=4)
        for label, variable in (("Kp", self.kp), ("Kd", self.kd)):
            ttk.Label(row, text=label).pack(side="left", padx=(0, 4))
            ttk.Entry(row, textvariable=variable, width=8).pack(side="left", padx=(0, 8))
        ttk.Label(self.details, text="변경값은 다음 이동부터 적용 · 다시 실행하면 0.04 / 0.001 / 100%",
                  wraplength=460).pack(anchor="w")
        ttk.Label(self.details, textvariable=self.telemetry, wraplength=460).pack(anchor="w")
        ttk.Label(self.details, text="끝단은 엔코더 계산값입니다. 실제 끝단 센서는 없습니다.",
                  wraplength=460).pack(anchor="w")
        ttk.Label(self.details, text="통신·Pico 오류 기록: ~/.local/state/yor-lift/diagnostics.jsonl",
                  wraplength=460).pack(anchor="w")

    def toggle_details(self):
        if self.show_details.get():
            self.details.pack(fill="x")
            self.root.geometry("560x1040")
        else:
            self.details.pack_forget()
            self.root.geometry(self.WINDOW_GEOMETRY)

    def clear_error(self):
        self.disarm()
        self.send("clear")

    def forget_zero(self):
        if not (self.known and self.latest and self.latest["homed"]
                and self.latest["state"] == "IDLE" and not self.latest["fault"]):
            self.event_text.set("원점이 등록된 상태에서 먼저 정지하세요")
            return
        if self.latest.get("reference_clear_available") is not True:
            self.event_text.set("원점 해제 기능이 있는 새 Pico 펌웨어가 필요해요")
            return
        self.disarm()
        if messagebox.askyesno("원점 해제", "원점을 해제하면 위치 제한 없이 수동으로 움직입니다.\n"
                "다시 최하단을 직접 확인해 0 mm를 설정해야 합니다.\n원점을 해제할까요?",
                default="no", parent=self.root):
            self.send("forget_zero")

    def save_control(self):
        if not (self.known and self.latest and self.latest["calibrated"]
                and self.latest["state"] in ("IDLE", "UNHOMED") and not self.latest["fault"]
                and self.measure_phase in ("idle", "done")):
            self.event_text.set("보정을 마친 뒤 멈춘 상태에서 저장하세요")
            return
        if not self.latest.get("control_storage_available"):
            self.event_text.set("설정 저장을 지원하는 새 Pico 펌웨어가 필요해요")
            return
        if not self.latest.get("calibration_saved"):
            self.event_text.set("먼저 처음 설정에서 엔코더 보정값을 저장하세요")
            return
        try:
            kp, kd, duty = float(self.kp.get()), float(self.kd.get()), self.pwm.get()/100
        except (ValueError, tk.TclError):
            kp, kd, duty = 0, -1, 0
        if not (math.isfinite(kp) and 0 < kp <= .1 and math.isfinite(kd)
                and 0 <= kd <= .1 and .1 <= duty <= 1):
            self.event_text.set("PD 계수와 출력 상한을 확인하세요")
            return
        self.disarm()
        self.send("save_control", kp=kp, kd=kd, max_duty=duty)

    def setup_next(self):
        if not self.known or self.demo or not self.latest:
            self.setup_text.set("먼저 리프트를 연결하세요")
            return
        state = self.latest
        if state["fault"] or state["state"] not in ("IDLE", "UNHOMED"):
            self.setup_text.set("먼저 정지하세요. 오류가 있으면 상세 설정에서 원인을 확인하고 해제하세요.")
            return
        if self.measure_phase in ("idle", "done"):
            self.measure_samples = []
            self.measure_candidate = None
            self.measure_phase = "ready"
            self.setup_text.set("1/3 낮은 구간: 하단에서 조금 올라온 곳으로 수동 이동하세요.\n끝단을 피하고, 멈춘 뒤 ‘낮은 구간 측정 시작’을 누르세요.")
            self.setup_button.configure(text="낮은 구간 측정 시작")
            self.disarm()
        elif self.measure_phase == "ready":
            self.disarm()
            self.measure_start = state["raw_count"]
            self.measure_phase = "up"
            self.distance.set("")
            self.distance_row.pack(fill="x", pady=6, before=self.setup_button)
            self.setup_button.configure(text="이 구간 측정 저장")
            name = ("낮은", "중간", "높은")[len(self.measure_samples)]
            self.setup_text.set(f"{len(self.measure_samples)+1}/3 {name} 구간: 시작 위치를 자로 기록하세요.\n위로 30 mm 이상 이동 후 정지하고, 두 위치의 차이를 입력하세요. 끝단은 피하세요.")
        elif self.measure_phase == "up":
            try:
                distance = float(self.distance.get())
            except ValueError:
                distance = 0
            delta = ((state["raw_count"] - self.measure_start + (1 << 28)) % (1 << 29)) - (1 << 28)
            if not math.isfinite(distance) or not 30 <= distance <= STROKE_MM or delta == 0:
                self.setup_text.set("상승 전후 자의 차이(30 mm 이상)를 입력하세요. 현재 높이 전체를 입력하지 마세요.")
                return
            if not 0 < distance / abs(delta) <= 1:
                self.setup_text.set("거리와 센서 값이 맞지 않아요. 입력값을 확인하세요.")
                return
            if self.measure_samples and delta * self.measure_samples[0][0] <= 0:
                self.setup_text.set("상승 카운트 방향이 앞 구간과 달라요. 이동 방향과 배선을 확인하세요.")
                return
            self.disarm()
            self.measure_samples.append((delta, distance))
            self.distance_row.pack_forget()
            if len(self.measure_samples) < 3:
                name = ("중간", "높은")[len(self.measure_samples)-1]
                self.measure_phase = "ready"
                self.setup_button.configure(text=f"{name} 구간 측정 시작")
                self.setup_text.set(f"{len(self.measure_samples)}/3 저장됨 · {distance:g} mm / {abs(delta)} 카운트\n{name} 구간으로 수동 이동 후 정지하고 측정 시작을 누르세요. 구간 사이 이동은 보정에 포함되지 않아요.")
                return
            try:
                factor, sign, spread, residual = fit_calibration(self.measure_samples)
            except ValueError as error:
                self.reset_measurement(str(error))
                return
            self.measure_end = state["raw_count"]
            self.measure_candidate = (factor, sign)
            self.measure_phase = "down"
            self.down_seen = False
            self.setup_button.configure(text="세 구간 보정 적용")
            self.setup_text.set(f"3/3 완료 · 구간 차이 {spread*100:.2f}% · 최대 거리 잔차 {residual:.2f} mm\n내리기를 짧게 눌렀다 놓고 적용하세요. 잔차는 정확도 보증값이 아니에요.")
        elif self.measure_phase == "down":
            factor, sign = self.measure_candidate
            reverse = (((state["raw_count"] - self.measure_end + (1 << 28)) % (1 << 29)) - (1 << 28)) * sign
            if not self.down_seen or reverse > -3:
                self.setup_text.set("내리기를 조금 누른 뒤 놓으세요. 반대 방향 움직임을 확인해야 해요.")
                return
            self.disarm()
            if messagebox.askyesno("움직임 확인", "세 구간의 실제 상승량을 측정했고, 올리기·내리기 방향도 맞나요?\n적용하면 기존 원점이 해제됩니다.",
                                   default="no", parent=self.root):
                self.factor.set(str(factor)); self.encoder_sign.set(str(sign))
                self.record_diagnostic("three_region_calibration", json.dumps({
                    "samples": self.measure_samples, "mm_per_count": factor,
                    "encoder_sign": sign}, ensure_ascii=False))
                self.send("calibrate", mm_per_count=factor, encoder_sign=sign, ab_verified=True)
                self.calibration_retry = (factor, sign, state["raw_count"], time.monotonic() + 1.2)
                self.measure_phase = "applying"
                self.setup_text.set("설정을 적용하고 있어요…")

    def request_action(self, action):
        if not self.known or not self.latest or self.latest["fault"]:
            self.event_text.set("먼저 연결하세요. 오류가 있으면 상세 설정에서 확인하세요.")
            return
        if self.latest.get("control_revision") != 3:
            self.event_text.set("새 Pico 펌웨어를 올린 뒤 다시 연결하세요")
            return
        self.pending_action = action
        self.action_deadline = time.monotonic() + .6
        if self.armed:
            self.dispatch_action()
        else:
            # Arm only for an explicit move/held-button action; do not clear faults.
            if self.send("arm"):
                self.pending_arm = True

    def dispatch_action(self):
        action, self.pending_action = self.pending_action, None
        if (not action or not self.known or not self.armed or not self.latest
                or self.latest["fault"] or time.monotonic() > self.action_deadline):
            return
        if action[0] == "manual" and self.held and self.latest["state"] in ("IDLE", "UNHOMED", "JOG"):
            self.send("jog", direction=self.direction, duty=self.pwm.get() / 100)
        elif action[0] == "move" and self.latest["homed"] and self.latest["state"] == "IDLE":
            self.diag_last_command = {"cmd": "move_to", "target_mm": action[1],
                                      "max_duty": action[4],
                                      "kp": action[2], "kd": action[3]}
            self.send("move_to", target_mm=action[1], max_duty=action[4],
                      kp=action[2], kd=action[3])

    def start_manual(self, direction, widget=None):
        self.release()
        if self.latest and self.latest.get("control_revision") != 3:
            self.event_text.set("새 Pico 펌웨어를 올린 뒤 다시 연결하세요")
            return "break"
        if self.measure_phase == "applying":
            self.event_text.set("설정 적용이 끝날 때까지 잠시 기다려주세요.")
            return "break"
        if not self.latest or self.latest["state"] not in ("IDLE", "UNHOMED", "JOG"):
            self.event_text.set("먼저 정지한 뒤 올리기·내리기를 사용하세요.")
            return "break"
        self.held, self.direction, self.input_source = True, direction, "mouse"
        if self.measure_phase == "down" and direction == -1:
            self.down_seen = True
        self.manual_widget = widget
        if widget:
            widget.focus_set(); widget.grab_set()
        self.request_action(("manual",))
        return "break"

    def release(self, event=None):
        if event is not None and self.input_source == "keyboard":
            return
        self.blocked_keys.update(self.pressed_keys)
        if getattr(self, "pending_action", None) and self.pending_action[0] == "manual":
            self.pending_action = None
        widget = getattr(self, "manual_widget", None)
        if widget and widget.grab_current() is widget:
            widget.grab_release()
        self.manual_widget = None
        super().release(event)

    def key_press(self, event):
        if self.latest and self.latest.get("control_revision") != 3:
            self.event_text.set("새 Pico 펌웨어를 올린 뒤 다시 연결하세요")
            return "break"
        if event.keysym in self.blocked_keys:
            pending = self.key_release_jobs.pop(event.keysym, None)
            if pending is not None:
                self.root.after_cancel(pending)
            return "break"
        if (self.input_source == "mouse" or event.widget.winfo_class() in (
                "Entry", "TEntry", "Text", "Spinbox", "TSpinbox", "TCombobox", "Scale", "TScale")):
            return
        if (self.measure_phase == "applying" or not self.known or not self.latest
                or self.latest["fault"] or self.latest["state"] not in ("IDLE", "UNHOMED", "JOG")):
            return
        pending = self.key_release_jobs.pop(event.keysym, None)
        if pending is not None:
            self.root.after_cancel(pending)
        if event.keysym in self.pressed_keys:
            return "break"
        self.pressed_keys.add(event.keysym)
        self.input_source = "keyboard"
        self.update_keys()
        return "break"

    def key_release(self, event):
        if event.keysym in self.blocked_keys:
            pending = self.key_release_jobs.pop(event.keysym, None)
            if pending is not None:
                self.root.after_cancel(pending)

            def unblock(key=event.keysym):
                self.blocked_keys.discard(key)
                self.key_release_jobs.pop(key, None)

            # X11 autorepeat release/press pairs must not re-arm after a stop/reset.
            self.key_release_jobs[event.keysym] = self.root.after(50, unblock)
            return "break"
        return super().key_release(event)

    def update_keys(self):
        super().update_keys()
        if self.direction and self.held:
            if self.measure_phase == "down" and self.direction == -1:
                self.down_seen = True
            self.request_action(("manual",))
        elif self.pending_action and self.pending_action[0] == "manual":
            self.pending_action = None

    def validate_extra_state(self, state):
        saved = state.get("saved_control")
        if saved is not None:
            if not isinstance(saved, dict):
                return False
            for key, lo, hi in (("kp", 0, .1), ("kd", -1, .1), ("max_duty", .099, 1)):
                value = saved.get(key)
                if type(value) not in (int, float) or not math.isfinite(value) or not lo < value <= hi or value < 0:
                    return False
        if "position_control" in state:
            if state["position_control"] != "PD":
                return False
            for key, minimum in (("kp", 0.0), ("kd", -1.0)):
                value = state.get(key)
                if (type(value) not in (int, float) or not math.isfinite(value)
                        or not minimum < value <= .1 or value < 0):
                    return False
        if "calibration_saved" in state and type(state["calibration_saved"]) is not bool:
            return False
        if any(type(state.get(key)) is not bool for key in (
                "calibrated", "homed", "at_min", "at_max", "physical_limits_available")):
            return False
        if (state.get("state") not in ("IDLE", "UNHOMED", "JOG", "MOVING", "SETTLING", "FAULT")
                or not isinstance(state.get("event"), str)
                or type(state.get("raw_count")) is not int
                or state.get("min_mm") != 0 or state.get("max_mm") != STROKE_MM
                or state.get("limit_source") != "encoder_soft"
                or state["physical_limits_available"]):
            return False
        pos, target = state.get("position_mm"), state.get("target_mm")
        if state["homed"]:
            if (not state["calibrated"] or type(pos) not in (int, float)
                    or not math.isfinite(pos) or not -1 <= pos <= STROKE_MM + 1):
                return False
        elif pos is not None or state["at_min"] or state["at_max"]:
            return False
        return target is None or (type(target) in (int, float) and math.isfinite(target)
                                  and 0 <= target <= STROKE_MM)

    def describe_state(self, state):
        return "{} · 카운트 {} · PWM {:.0f}% · {}".format(
            state.get("position_control", "이전 펌웨어"), state["raw_count"], state["duty"] * 100, state["state"])

    def handle_extra_state(self, state):
        previous = self.latest
        self.latest = state
        if state["homed"]:
            self.zero_deadline = None
        boot_events = ("boot_unhomed", "saved_calibration_loaded", "calibration_storage_invalid")
        boot_observed = (previous is None or (state["event"] in boot_events
                         and previous["event"] != state["event"]))
        fields = ("state", "fault", "event", "armed", "homed", "position_mm",
                  "target_mm", "raw_count", "pulses", "direction", "duty",
                  "velocity_mm_s", "kp", "kd", "boot_reason")
        self.diag_frames.append({"monotonic_s": round(time.monotonic(), 3),
                                 **{key: state.get(key) for key in fields}})
        if boot_observed and state.get("boot_reason") not in (None, "unknown"):
            self.record_diagnostic("pico_connected", state["boot_reason"])
        if state["fault"] and state["fault"] != self.diag_last_fault:
            self.record_diagnostic("pico_fault", state["fault"])
        self.diag_last_fault = state["fault"]
        if state["homed"]:
            endpoint = " · 계산상 하한" if state["at_min"] else " · 계산상 상한" if state["at_max"] else ""
            self.position_text.set("현재 높이: {:.1f} mm{}".format(state["position_mm"], endpoint))
        else:
            self.position_text.set("현재 높이: 알 수 없음 · 원점 미등록")
        if previous is None or (previous["event"], previous["fault"]) != (state["event"], state["fault"]):
            message = state["fault"] or state["event"]
            self.event_text.set(EVENTS.get(message, message))
        self.status.set("오류 · 상세 설정에서 확인하세요" if state["fault"] else
                        "이동 중" if state["state"] in ("MOVING", "JOG") else
                        "도착 확인 중" if state["state"] == "SETTLING" else "연결됨")
        if state["fault"]:
            if not self.fault_button.winfo_manager():
                self.fault_button.pack(side="right", padx=(0, 5))
            self.pending_action = None
            if self.measure_phase not in ("idle", "done"):
                self.reset_measurement("설정이 중단됐어요. 오류 원인을 확인한 뒤 다시 시작하세요.")
        else:
            self.fault_button.pack_forget()
        if state["event"] in boot_events and previous and previous["event"] != state["event"]:
            self.pending_action = None
            self.reset_measurement("장치가 다시 켜졌어요. 맨 아래에서 0 mm를 설정하세요.")
        if self.measure_phase == "applying" and state["calibrated"] and state["event"] == "calibrated_unhomed":
            self.calibration_retry = None
            self.measure_phase = "done"
            self.kp.set(str(DEFAULT_KP))
            self.kd.set(str(DEFAULT_KD))
            self.pwm.set(DEFAULT_PWM)
            self.setup_button.configure(text="다시 설정")
            self.setup_text.set("설정 완료! 보정값을 저장했어요. 다음부터는 0 mm만 설정하세요."
                                if state.get("calibration_saved") else
                                "설정 완료! 현재 펌웨어는 임시 적용이에요. 저장 기능이 있는 펌웨어로 갱신하세요.")
        if self.measure_phase in ("idle", "done") and state.get("calibration_saved") and state["calibrated"]:
            self.setup_button.configure(text="다시 보정하기")
            self.setup_text.set("보정값 저장됨 · 거리 측정은 다시 하지 않아도 돼요."
                                if state["homed"] else
                                "보정값 저장됨 · 맨 아래에 맞추고 ‘0 mm 설정’만 누르세요.")
            if not state["fault"] and not state["homed"] and state["event"] in boot_events + ("disarmed", "cleared"):
                self.event_text.set("저장된 보정값이 있어요. 맨 아래에서 0 mm만 설정하세요.")
        if boot_observed and not state["fault"] and state.get("boot_reason") == "watchdog":
            self.event_text.set("Pico의 최근 재시작 원인은 감시 타이머예요. 배선·전원·진단 기록을 확인하세요.")
        if self.pending_action and time.monotonic() <= self.action_deadline and self.armed and not state["fault"]:
            self.dispatch_action()
        setup_ready = self.measure_phase in ("idle", "done")
        self.move_button.configure(state="normal" if state["homed"] and setup_ready
                                   and state["state"] == "IDLE" and not state["fault"]
                                   and self.pending_action is None else "disabled")
        self.zero_button.configure(state="normal" if state["calibrated"] and setup_ready
                                   and state["state"] in ("IDLE", "UNHOMED")
                                   and not state["fault"] else "disabled")

    def cancel_setup(self):
        self.disarm()
        self.reset_measurement("보정을 취소했어요. 적용 전 측정은 버리고 기존 보정값을 유지해요.")

    def reset_measurement(self, text="낮음·중간·높음에서 각각 30 mm 이상 상승량을 측정해요."):
        self.calibration_retry = None
        self.measure_phase = "idle"
        self.measure_start = self.measure_end = self.measure_candidate = None
        self.measure_samples = []
        self.down_seen = False
        self.distance_row.pack_forget()
        self.setup_button.configure(text="설정 시작")
        self.setup_text.set(text)

    def move_to(self):
        if not (self.known and self.latest and self.latest["homed"]
                and self.measure_phase in ("idle", "done")
                and self.latest["state"] == "IDLE" and not self.latest["fault"]):
            self.event_text.set("처음 설정과 0 mm 설정을 마치고, 정지 상태에서 이동하세요")
            return
        try:
            target = float(self.target.get())
        except ValueError:
            target = float("nan")
        if not math.isfinite(target) or not WORK_MIN_MM <= target <= WORK_MAX_MM:
            self.event_text.set("끝단 여유를 위해 목표 높이는 10–680 mm로 입력하세요")
            return
        if self.latest.get("position_control") != "PD":
            self.event_text.set("PD 위치 제어용 새 Pico 펌웨어를 먼저 올려주세요.")
            return
        try:
            kp, kd, duty = float(self.kp.get()), float(self.kd.get()), self.pwm.get() / 100
        except (ValueError, tk.TclError):
            kp, kd, duty = 0, -1, 0
        if not (math.isfinite(kp) and 0 < kp <= .1 and math.isfinite(kd)
                and 0 <= kd <= .1 and .1 <= duty <= 1):
            self.event_text.set("Kp는 0 초과 0.1 이하, Kd는 0–0.1, PWM 상한은 10–100%로 입력하세요.")
            return
        self.release()
        self.move_button.configure(state="disabled")
        self.request_action(("move", target, kp, kd, duty))

    def mark_zero(self):
        if self.demo:
            self.event_text.set("데모는 실제 원점을 등록하지 않습니다")
            return
        if not (self.known and self.latest and self.latest["calibrated"]
                and self.measure_phase in ("idle", "done")):
            self.event_text.set("아래 ‘처음 설정’을 먼저 완료하세요")
            return
        self.disarm()
        dialog = tk.Toplevel(self.root)
        dialog.title("최하단 기준 확인")
        dialog.transient(self.root)
        ttk.Label(dialog, text="리프트가 실제 최하단 기준에 있고 정지했나요?\n"
                  "센서가 없어 소프트웨어는 이 위치를 확인할 수 없습니다.\n"
                  "확인한 현재 위치를 0 mm로 등록합니다.", padding=15).pack()
        buttons = ttk.Frame(dialog, padding=(10, 0, 10, 10)); buttons.pack()
        def finish(confirmed):
            dialog.grab_release()
            dialog.destroy()
            if confirmed:
                self.confirm_zero()
        ttk.Button(buttons, text="취소", command=lambda: finish(False)).pack(side="left", padx=5)
        ttk.Button(buttons, text="확인 · 0 mm 설정", command=lambda: finish(True)).pack(side="left", padx=5)
        dialog.protocol("WM_DELETE_WINDOW", lambda: finish(False))
        dialog.grab_set()

    def confirm_zero(self):
        if not (self.known and self.latest and self.latest["calibrated"]
                and not self.latest["fault"] and self.latest["state"] in ("IDLE", "UNHOMED")):
            self.event_text.set("0 mm 설정 실패 · 연결과 정지 상태를 확인하세요")
            return
        if self.send("zero", at_lower_reference=True):
            self.zero_deadline = time.monotonic() + 1.5
            self.event_text.set("0 mm 설정 요청 중 · 피코 응답을 기다려요")
        else:
            self.event_text.set("0 mm 설정 실패 · 피코 연결을 확인하세요")

    def calibrate(self):
        if not self.known or self.demo:
            self.event_text.set("실제 피코 연결 후 실측값을 적용하세요")
            return
        try:
            factor, sign = float(self.factor.get()), int(self.encoder_sign.get())
        except ValueError:
            factor, sign = 0, 0
        if not math.isfinite(factor) or not 0 < factor <= 1 or sign not in (-1, 1):
            self.event_text.set("실측 mm/count는 0 초과 1 이하, 부호는 1 또는 -1을 입력하세요")
            return
        self.disarm()
        if messagebox.askyesno("실측 보정 확인", "엔코더 A/B 위상과 상승 시 카운트 부호를 확인했고,\n"
                "실제 상판 이동거리로 mm/count를 측정했나요?\n"
                "이전 원점은 해제됩니다. 실측하지 않은 기본값은 사용하지 마세요.",
                default="no", parent=self.root):
            self.send("calibrate", mm_per_count=factor, encoder_sign=sign, ab_verified=True)

    def tick_extra(self):
        if self.zero_deadline and time.monotonic() > self.zero_deadline:
            self.zero_deadline = None
            if not self.latest or not self.latest.get("homed"):
                reason = self.latest.get("event", "") if self.latest else ""
                self.record_diagnostic("zero_failed", reason or "no_response")
                self.event_text.set("0 mm 설정 실패 · " + EVENTS.get(reason, reason or "피코 응답 없음"))
        if self.calibration_retry:
            factor, sign, count, deadline = self.calibration_retry
            if (time.monotonic() > deadline or not self.latest or self.latest["fault"]
                    or self.latest["raw_count"] != count):
                self.calibration_retry = None
                self.measure_phase = "down"
                self.setup_text.set("먼저 멈춘 뒤 ‘설정 적용’을 다시 눌러주세요.")
            elif self.latest["event"] == "calibration_requires_disarmed_stationary":
                # Firmware requires 300 ms without pulses; wait via its idle replies.
                self.send("calibrate", mm_per_count=factor, encoder_sign=sign, ab_verified=True)
        if self.pending_action and time.monotonic() > self.action_deadline:
            self.disarm()
            self.event_text.set("장치 응답을 확인하지 못해 이동을 취소했어요. 다시 눌러주세요.")
        if self.armed and self.serial:
            self.send("heartbeat")

    def disarm(self):
        self.pending_action = None
        if getattr(self, "calibration_retry", None):
            self.calibration_retry = None
            self.measure_phase = "down"
            self.setup_text.set("설정 적용을 취소했어요. 멈춘 상태에서 다시 적용할 수 있어요.")
        super().disarm()
        if self.serial and self.known:
            self.status.set("연결됨 · 정지")
        elif not self.demo and not self.serial:
            self.status.set("연결 안 됨 · 연결 버튼을 누르세요")
        if hasattr(self, "move_button"):
            self.move_button.configure(state="disabled")

    def disconnect(self, reason):
        had_serial = self.serial is not None
        super().disconnect(reason)
        if had_serial:
            self.record_diagnostic("connection_lost", reason)
        self.fault_button.pack_forget()
        self.latest = None
        self.pending_action = None
        self.zero_deadline = None
        self.reset_measurement("연결이 끊겼어요. 연결 후 설정을 다시 시작하세요.")
        self.position_text.set("현재 높이: 알 수 없음 · 연결 끊김")
        self.event_text.set("통신 연결이 끊겼어요. 모터 정지 후 원인을 확인하고 다시 연결하세요.")
        self.move_button.configure(state="disabled")

    def connect(self):
        self.diag_frames.clear()
        self.diag_invalid.clear()
        self.diag_last_command = None
        self.diag_last_fault = ""
        self.latest = None
        self.reset_measurement()
        self.position_text.set("현재 높이: 알 수 없음 · 응답 대기")
        super().connect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    ports = glob.glob("/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_*-if00")
    parser.add_argument("--port", default=ports[0] if len(ports) == 1 else "/dev/ttyACM0")
    parser.add_argument("--demo", action="store_true", help="UI only; no motor or simulated zero")
    args = parser.parse_args()
    root = tk.Tk()
    PositionUI(root, args.port, args.demo)
    root.mainloop()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Local Tk joystick for firmware/pico_lift/main.py; never auto-arms a device."""
import argparse
import json
import math
import time
import tkinter as tk
from tkinter import ttk, font as tkfont

PROTOCOL = "yor-lift-jog-v1"
KEY_RELEASE_DELAY_MS = 50


class Joystick:
    STATUS_PROTOCOL = PROTOCOL
    PWM_MIN = 10
    PWM_MAX = 35
    PWM_DEFAULT = 25
    COUNTER_LABEL = "A 펄스"
    WINDOW_TITLE = "Lift · Pico manual jog"
    WINDOW_GEOMETRY = "520x760"
    HEADING = "리프트 수동 조그"
    INITIAL_TELEMETRY = "위치 제어 없음 · A 펄스 활동만 표시"
    JOYSTICK_SCALE = 1.0

    def __init__(self, root, port, demo=False):
        self.root, self.demo = root, demo
        self.serial = None
        self.buffer = b""
        self.last_rx = 0.0
        self.known = self.armed = self.held = False
        self.direction = 0
        self.input_source = None
        self.pressed_keys = set()
        self.key_release_jobs = {}
        self.pending_arm = False
        available_fonts = set(tkfont.families(root))
        ui_family = next((name for name in (
            "Noto Sans CJK KR", "Noto Sans KR", "NanumGothic",
            "Noto Sans CJK JP", "Malgun Gothic", "Apple SD Gothic Neo",
        ) if name in available_fonts), tkfont.nametofont("TkDefaultFont").actual("family"))
        for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont"):
            tkfont.nametofont(name).configure(family=ui_family)
        root.title(self.WINDOW_TITLE)
        root.geometry(self.WINDOW_GEOMETRY)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.port = tk.StringVar(value=port)
        self.status = tk.StringVar(value="데모 · 모터 연결 없음" if demo else "연결 대기")
        self.telemetry = tk.StringVar(value=self.INITIAL_TELEMETRY)
        self.pwm = tk.IntVar(value=self.PWM_DEFAULT)
        panel = ttk.Frame(root, padding=20)
        panel.pack(fill="both", expand=True)
        self.build_main_controls(panel, ui_family)
        self.tick_job = root.after(50, self.tick)

    def build_main_controls(self, panel, ui_family):
        ttk.Label(panel, text=self.HEADING, font=(ui_family, 22, "bold")).pack()
        row = ttk.Frame(panel)
        row.pack(fill="x", pady=16)
        ttk.Entry(row, textvariable=self.port, width=27).pack(side="left", expand=True, fill="x")
        ttk.Button(row, text="연결", command=self.connect).pack(side="left", padx=5)
        ttk.Button(panel, text="조그 활성화 / 오류 해제", command=self.arm).pack(fill="x")
        ttk.Label(panel, textvariable=self.status, wraplength=450).pack(pady=10)
        self.canvas = tk.Canvas(panel, width=int(300 * self.JOYSTICK_SCALE),
                                height=int(300 * self.JOYSTICK_SCALE),
                                bg="#f2f4f7", highlightthickness=0)
        self.canvas.pack()
        self.canvas.create_line(150, 65, 150, 235, fill="#c4cbd5", width=8)
        self.canvas.create_text(150, 24, text="▲ UP", font=(ui_family, 16))
        self.canvas.create_text(150, 278, text="▼ DOWN", font=(ui_family, 16))
        self.knob = self.canvas.create_oval(121, 121, 179, 179, fill="#7b8794", outline="")
        self.canvas.scale("all", 0, 0, self.JOYSTICK_SCALE, self.JOYSTICK_SCALE)
        self.canvas.bind("<ButtonPress-1>", self.press)
        self.canvas.bind("<B1-Motion>", self.drag)
        self.root.bind_all("<ButtonRelease-1>", self.release)
        self.root.bind("<Escape>", lambda _event: self.disarm())
        self.root.bind("<FocusOut>", self.focus_out)
        for key in ("Up", "Down"):
            self.root.bind("<KeyPress-" + key + ">", self.key_press)
            self.root.bind("<KeyRelease-" + key + ">", self.key_release)
        tk.Scale(panel, from_=self.PWM_MIN, to=self.PWM_MAX, orient="horizontal", variable=self.pwm,
                 label="PWM (%) · 조이스틱은 방향만 선택").pack(fill="x", pady=8)
        ttk.Button(panel, text="정지 / 비활성화 (Esc)", command=self.disarm).pack(fill="x", ipady=8)
        ttk.Label(panel, textvariable=self.telemetry, wraplength=450).pack(pady=10)
        ttk.Label(panel, text="↑/↓ 키 또는 조이스틱을 누르는 동안 이동\n놓으면 정지 · ↑↓ 동시 입력은 정지 · Esc 비활성화",
                  justify="center").pack()
        self.build_extra_controls(panel)

    def build_extra_controls(self, panel):
        pass

    def validate_extra_state(self, state):
        return True

    def invalid_state(self, reason):
        """Optional diagnostic hook; invalid data never extends the link lease."""
        pass

    def describe_state(self, state):
        return "{}: {} · PWM: {:.0f}% · 명령 방향: {}".format(
            self.COUNTER_LABEL, state["pulses"], state["duty"] * 100, state["direction"])

    def handle_extra_state(self, state):
        pass

    def tick_extra(self):
        pass

    def position_knob(self, y):
        scale = self.JOYSTICK_SCALE
        self.canvas.coords(self.knob, 121 * scale, (y - 29) * scale,
                           179 * scale, (y + 29) * scale)

    def send(self, cmd, **values):
        if self.demo:
            return True
        if self.serial is None:
            return False
        try:
            data = (json.dumps(dict(cmd=cmd, **values)) + "\n").encode()
            if self.serial.write(data) != len(data):
                raise OSError("incomplete serial write")
            return True
        except (OSError, self.serial_error) as error:
            self.disconnect("송신 실패: " + str(error))
            return False

    def connect(self):
        self.disarm()
        if self.demo:
            self.known = True
            self.status.set("데모 · 활성화 후 조작하세요")
            return
        if self.serial:
            self.serial.close()
            self.serial = None
        try:
            import serial
            self.serial_error = serial.SerialException
            self.serial = serial.Serial(self.port.get(), 115200, timeout=0,
                                        write_timeout=0.05, exclusive=True)
        except (ImportError, OSError) as error:
            self.status.set("연결 실패: " + str(error))
            return
        self.known = False
        self.buffer = b""
        self.last_rx = time.monotonic()
        self.status.set("전용 피코 펌웨어 응답 대기…")
        self.send("disarm")

    def arm(self):
        self.release()
        if self.demo:
            self.known = self.armed = True
            self.status.set("데모 · 조그 활성화")
        elif self.known:
            self.send("stop")
            self.send("clear")
            if self.send("arm"):
                self.pending_arm = True
                self.status.set("활성화 응답 대기…")

    def press(self, event):
        if not self.armed:
            return
        self.release()
        self.input_source = "mouse"
        self.canvas.focus_set()
        self.canvas.grab_set()
        self.held = True
        self.drag(event)

    def drag(self, event):
        if not self.held or self.input_source != "mouse":
            return
        y = max(65, min(235, event.y / self.JOYSTICK_SCALE))
        self.direction = 1 if y < 125 else (-1 if y > 175 else 0)
        self.position_knob(y)
        self.canvas.itemconfigure(self.knob, fill="#2878cc" if self.direction else "#7b8794")
        if self.direction == 0:
            self.send("stop")

    def key_press(self, event):
        if (not self.armed or self.input_source == "mouse"
                or event.widget.winfo_class() in ("Entry", "TEntry", "Text", "Spinbox", "TSpinbox")):
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
        if event.keysym not in self.pressed_keys:
            return
        pending = self.key_release_jobs.pop(event.keysym, None)
        if pending is not None:
            self.root.after_cancel(pending)
        # X11 repeat release/press pairs can be separated by an idle/frame cycle.
        # Allow 50 ms for the repeat press, rather than stopping between frames.
        # Focus loss, Escape and the stop button still stop immediately.
        self.key_release_jobs[event.keysym] = self.root.after(KEY_RELEASE_DELAY_MS,
            lambda key=event.keysym: self.finish_key_release(key))
        return "break"

    def finish_key_release(self, key):
        self.key_release_jobs.pop(key, None)
        self.pressed_keys.discard(key)
        if self.input_source == "keyboard":
            self.update_keys()

    def update_keys(self):
        old = self.direction
        self.direction = (1 if self.pressed_keys == {"Up"} else
                          -1 if self.pressed_keys == {"Down"} else 0)
        self.held = bool(self.pressed_keys)
        if not self.pressed_keys:
            self.input_source = None
        y = 65 if self.direction == 1 else 235 if self.direction == -1 else 150
        self.position_knob(y)
        self.canvas.itemconfigure(self.knob, fill="#2878cc" if self.direction else "#7b8794")
        if old and not self.direction:
            self.send("stop")

    def release(self, _event=None):
        if _event is not None and self.input_source == "keyboard":
            return
        active = self.held or self.direction != 0
        for pending in self.key_release_jobs.values():
            self.root.after_cancel(pending)
        self.key_release_jobs.clear()
        self.pressed_keys.clear()
        self.input_source = None
        self.held = False
        self.direction = 0
        self.position_knob(150)
        self.canvas.itemconfigure(self.knob, fill="#7b8794")
        if self.canvas.grab_current() is self.canvas:
            self.canvas.grab_release()
        if active:
            self.send("stop")

    def disarm(self):
        self.armed = self.pending_arm = False
        self.release()
        self.send("disarm")
        self.status.set("비활성화 · 활성화 버튼으로 다시 시작")

    def focus_out(self, _event):
        self.root.after_idle(self.check_focus)

    def check_focus(self):
        if self.root.focus_displayof() is None:
            self.disarm()

    def disconnect(self, reason):
        # Clear transport first: release() must not recursively write to a failed port.
        old, self.serial = self.serial, None
        if old:
            try:
                old.close()
            except OSError:
                pass
        self.known = self.armed = self.pending_arm = False
        self.release()
        self.status.set(reason + " · 수동 재연결 필요")

    def receive(self):
        try:
            self.buffer += self.serial.read(4096)
        except (OSError, self.serial_error) as error:
            self.disconnect("수신 실패: " + str(error))
            return
        if len(self.buffer) > 16384:
            self.disconnect("수신 데이터 초과")
            return
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            try:
                state = json.loads(line)
                if not isinstance(state, dict) or state.get("protocol") != self.STATUS_PROTOCOL:
                    self.invalid_state("protocol_or_type")
                    continue
                if (type(state.get("armed")) is not bool
                        or type(state.get("direction")) is not int
                        or state["direction"] not in (-1, 0, 1)
                        or type(state.get("duty")) not in (int, float)
                        or not math.isfinite(state["duty"])
                        or not 0 <= state["duty"] <= self.PWM_MAX / 100
                        or type(state.get("pulses")) is not int
                        or not isinstance(state.get("fault"), str)):
                    self.invalid_state("common_fields")
                    continue
                if not self.validate_extra_state(state):
                    self.invalid_state("position_fields")
                    continue
            except (ValueError, UnicodeError):
                self.invalid_state("malformed_json")
                continue
            self.last_rx = time.monotonic()
            self.known = True
            self.telemetry.set(self.describe_state(state))
            if state["fault"]:
                self.armed = self.pending_arm = False
                self.release()
                self.status.set("피코 오류: " + state["fault"] + " · 원인 확인 후 오류 해제")
            elif state["armed"] and self.pending_arm:
                self.armed, self.pending_arm = True, False
                self.status.set("조그 활성화 · 누르는 동안 이동")
            elif not state["armed"] and self.armed:
                self.armed = False
                self.release()
                self.status.set("피코 비활성화 · 다시 활성화하세요")
            elif not state["armed"] and not self.pending_arm:
                self.status.set("연결됨 · 조그 활성화 버튼을 누르세요")
            self.handle_extra_state(state)

    def tick(self):
        if self.tick_job is not None:
            self.root.after_cancel(self.tick_job)
            self.tick_job = None
        try:
            if self.serial:
                self.receive()
                if self.serial and time.monotonic() - self.last_rx > 0.6:
                    self.send("disarm")
                    # send() may already have disconnected with a more precise
                    # write error. Preserve that cause rather than overwriting it.
                    if self.serial:
                        self.disconnect("피코 응답 만료")
            if self.armed and self.held and self.direction:
                self.send("jog", direction=self.direction, duty=self.pwm.get() / 100)
                if self.demo:
                    self.telemetry.set("데모 · {} · PWM {}%".format(
                        "UP" if self.direction == 1 else "DOWN", self.pwm.get()))
            self.tick_extra()
        finally:
            self.tick_job = self.root.after(50, self.tick)

    def close(self):
        if self.tick_job is not None:
            self.root.after_cancel(self.tick_job)
            self.tick_job = None
        self.disarm()
        if self.serial:
            self.serial.close()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--demo", action="store_true", help="UI only; no serial port opened")
    args = parser.parse_args()
    root = tk.Tk()
    Joystick(root, args.port, args.demo)
    root.mainloop()


if __name__ == "__main__":
    main()

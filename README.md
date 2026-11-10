# Lift Controller · Raspberry Pi + Pico

630 mm 행정 리프트의 수동 이동과 엔코더 기준 절대위치 PD 제어 코드입니다.
모바일 구동·조향 ROS 패키지와 별도로 실행합니다. ROS나 rover-mapping 저장소가 필요하지 않습니다.

## 파일 구성

```text
pico/main.py                              # Pico에서 실행할 MicroPython 펌웨어
raspberry_pi/lift/start_position.sh        # 라파 UI 실행
raspberry_pi/lift/position/
  lift_position.py                        # 높이 제어 UI
  lift_joystick.py                        # 공통 직렬 통신·키보드 처리
scripts/install_pi.sh                     # 라파 ~/lift에 UI 설치 (자동 구동 없음)
scripts/flash_pico.py                     # 백업·검증 후 Pico 펌웨어 교체
simulation/                              # GPIO·USB 없이 실행하는 시뮬레이션
tests/                                   # 가상 시리얼·제어기 테스트
docs/simulation/                          # 최근 감속 수정 비교 결과
```

UI 시작 기본값은 **Kp 0.04, Kd 0.001, PWM 상한 100%**이며, **상세 설정**에서 바꿀 수 있습니다.
자동 이동 중 실제 출력은 PD와 속도 제한에 따라 달라집니다. 수동 올리기·내리기에도 선택한 PWM이 적용됩니다.
UI를 다시 실행하면 위 기본값으로 시작합니다. 설정을 편집하는 것만으로 모터가 켜지지는 않습니다.

## 버전과 현재 장치의 차이

이 저장소는 2026-11-10 최신 로컬 버전입니다. 라파에서 사용하던 디렉터리 구조를 그대로 유지했습니다.
이전 실물 배포본에 비해 **UI 설정값 편집 복구**와 **자동 위치 이동의 연속 접근 속도 피드백**이 추가되어 있습니다.
이 두 최신 변경은 저장소를 만들 당시 실제 라파/Pico에는 미적용 상태였습니다.
설치와 펌웨어 교체를 각각 실행해야 반영됩니다. 실물에서 끊김이 없어졌다고 검증한 버전은 아닙니다.

## 1. 다른 컴퓨터에서 내려받기

```bash
git clone https://github.com/sy00107/lift_ctrl.git
cd lift_ctrl
```

비공개 저장소는 GitHub 인증이 필요합니다. 이후 같은 컴퓨터에서는 `git pull --ff-only`로 갱신합니다.
소스를 받는 것은 clone/pull이며, 다른 컴퓨터에서 수정한 코드를 올릴 때 push를 사용합니다.
제어기와 UI는 이 저장소만으로 사용할 수 있습니다. 단, Pico에는 MicroPython이 이미 설치돼 있어야 합니다.

## 2. 라파에 UI 설치

라파에서 저장소를 clone한 다음 실행합니다. Linux와 Python 3.10 이상을 기준으로 작성했습니다.

```bash
sudo apt update
sudo apt install -y git python3-tk python3-venv
git clone https://github.com/sy00107/lift_ctrl.git
cd lift_ctrl
bash scripts/install_pi.sh
```

이미 clone했다면 두 번째 clone은 생략합니다. 설치 위치는 다음과 같습니다.

```text
~/lift/
  start_position.sh
  .venv/
  position/lift_position.py
  position/lift_joystick.py
  position/firmware/main.py     # 펌웨어 사본일 뿐, Pico에는 아직 미업로드
```

기존 파일은 `~/lift/backups.install.*`에 백업합니다. 설치는 UI나 모터를 시작하지 않으며,
피코 내부의 `lift_calibration.json`도 수정하지 않습니다.
이미 시스템에 tkinter·pyserial이 설치되어 있다면 `bash scripts/install_pi.sh --no-venv`도 가능합니다.
설치 위치를 바꾸려면 `--prefix /원하는/폴더`를 지정합니다.

USB 권한 오류가 발생하면 라파 사용자가 `dialout` 그룹에 있는지 `groups`로 확인합니다.
필요하다면 `sudo usermod -aG dialout "$USER"` 후 로그아웃·로그인합니다. 장치 번호는 재연결 시 바뀔 수 있어
UI는 가능한 경우 `/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_...-if00` 경로를 자동 선택합니다.
피코가 여러 개라면 실행 시 `--port`로 올바른 보드를 지정해야 합니다.

## 3. Pico 펌웨어 설치·갱신

**모터 24V 전원을 끄고, 라파/Pico USB 전원은 유지합니다. 실행 중인 리프트 UI는 닫습니다.**
이 절차는 MicroPython 위의 `main.py`를 교체하며 MicroPython 자체의 UF2 설치는 하지 않습니다.

라파에서 저장소 폴더로 이동한 뒤:

```bash
~/lift/.venv/bin/python -m pip install -r requirements-pico.txt
~/lift/.venv/bin/python scripts/flash_pico.py --motor-power-off
```

보드가 여러 개라면 `--port /dev/serial/by-id/실제피코경로`도 전달합니다.
스크립트는 기존 `main.py`와 보정 파일을 저장소의 `backups/`에 복사하고, 새 파일을 임시 이름으로 올립니다.
읽어온 내용이 원본과 일치하는지 확인한 뒤 교체하고 다시 검증합니다. 그 후 Pico를 재시작합니다.
**arm/jog/move_to 명령은 보내지 않습니다.** 보정 파일은 보존하고 현재 높이·원점은 복원하지 않습니다.
실패하면 모터 24V를 계속 끄고 백업과 오류 메시지를 확인합니다.

Pico의 GPIO는 RPWM=2, LPWM=3, R_EN=4, L_EN=5, 엔코더 A=27, B=28입니다.
A 상승 에지에서 B 위상을 읽는 x1 방식입니다. 다른 드라이버·핀·센서를 쓰면 그대로 실행하지 말고 맞춰야 합니다.

## 4. 화면 실행

라파에 모니터가 연결돼 있으면 라파 터미널에서:

```bash
~/lift/start_position.sh
```

Linux 노트북에 창을 띄우려면 노트북 터미널에서:

```bash
ssh -X pi@raspberrypi.local
# 접속된 라파 터미널에서
~/lift/start_position.sh
```

`no display name and no $DISPLAY`는 화면 전달 없이 접속한 경우입니다.
`exit`로 나온 뒤 `ssh -X`로 다시 연결합니다. Windows/macOS는 별도 X 서버나 라파 데스크톱 원격 접속이 필요합니다.
SSH에서 `raspberrypi.local`을 찾지 못하면 같은 네트워크인지 확인하고 실제 라파 IP로 접속합니다.

## 5. 조작 순서

1. **연결**을 누릅니다. 연결만으로 모터는 움직이지 않습니다.
2. 처음 보드이거나 저장된 보정값이 없으면 **설정 시작**으로 실제 상승량을 자로 측정해 보정합니다.
   같은 보드의 보정값이 유효하게 저장돼 있다면 다시 측정할 필요가 없습니다.
3. 실제 최하단 기준을 직접 확인하고 정지한 상태에서 **지금이 맨 아래예요 · 0 mm 설정**을 누릅니다.
   확인창에서 확인한 뒤 화면이 `현재 높이: 0.0 mm`가 되는지 확인합니다.
4. 목표 높이 **10–620 mm**를 입력하고 **이동**을 누릅니다.
   예를 들어 200→300 mm는 100 mm 상승, 300→200 mm는 100 mm 하강입니다.
5. 수동 조작은 **올리기/내리기 버튼 또는 ↑/↓ 키를 누르는 동안**입니다. 놓으면 정지합니다.
   **정지/Esc**는 정지·비활성화합니다. 별도 활성화 버튼은 없습니다.

전원이 켜질 때 20초 하강하는 자동 원점 찾기는 구현하지 않았습니다.
피코 재시작 후에는 실제 기준을 확인하고 원점을 다시 등록해야 합니다.
이미 원점이 있는 상태에서 최하단까지 수동으로 다시 맞출 때는, 정지 후 **상세 설정 → 원점 해제**를 사용합니다.
원점이 해제되면 위치 기반 제한이 없으므로 실제 하단을 직접 확인해야 합니다.
상·하단 센서는 없으며 엔코더 계산에 의한 여유 구간만 존재합니다. 자동으로 0 mm까지 보내지 않습니다.

도착 허용오차 ±0.2 mm는 엔코더 기준입니다. 자로 측정한 실제 높이의 ±0.2 mm 정확도를 보장하지 않습니다.
엔코더 해상도·mm/count는 단위마다 실측해야 하며 장치별 보정 파일은 다른 Pico로 복사하지 않습니다.
275 lb 하중 스펙은 두 기둥 책상 전체 기준이며 단일 리프트의 허용 하중으로 간주하지 않습니다.

## 오류 확인

- `encoder_no_pulse`: 출력이 켜진 누적 800 ms 동안 엔코더 변화 없음. 출력 부족·걸림·센서 유실을 구분할 필요가 있습니다.
- `command_timeout`: 피코가 제때 heartbeat를 받지 못해 정지. Kp 부족과 같은 오류가 아닙니다.
- `피코 응답 만료`: UI가 600 ms 동안 유효한 상태를 받지 못함. USB·장치 재시작·UI 지연을 확인합니다.
- 오류 후 자동 재출발하지 않습니다. 무펄스 등 일부 오류는 원점을 무효화합니다.
- 진단 기록은 `~/.local/state/yor-lift/diagnostics.jsonl`입니다. 오류·연결 이벤트 때 주변 상태만 저장하므로
  정상 이동 전체를 기록하는 블랙박스는 아닙니다.

## 테스트와 시뮬레이션 (실물 없이)

```bash
python3 -m pip install -r requirements.txt
python3 -m unittest discover -s tests -p 'test_*.py'
python3 simulation/sim_lift_smooth.py
```

GUI 테스트에는 디스플레이가 필요합니다. 헤드리스 Linux에서는 Xvfb를 설치한 뒤
`xvfb-run -a python3 -m unittest discover -s tests -p 'test_*.py'`로 실행합니다.
시뮬레이션은 GPIO/USB/SSH를 사용하지 않으며 실제 기계의 부하·마찰·정지거리를 보장하지 않습니다.
[감속 변경 비교 결과](docs/simulation/report.md)에서 한계를 함께 확인할 수 있습니다.

## 업데이트

```bash
cd ~/lift_ctrl
git pull --ff-only
bash scripts/install_pi.sh
# 펌웨어 변경도 있다면 모터 전원 OFF 후 위 Pico 업로드 절차를 별도로 실행
```

UI 설치와 Pico 업로드는 별도 단계입니다. 저장소를 갱신하는 것만으로 피코 펌웨어가 바뀌지는 않습니다.

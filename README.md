# Lift Controller · Raspberry Pi + Pico

FLEXISPOT E6 리프트를 Raspberry Pi 화면에서 조작하는 독립 실행형 제어기입니다.
ROS나 다른 로봇 저장소 없이 이 저장소만 설치하면 됩니다.

현재 버전은 첫 배포 코드의 **엔코더 x1 방식**을 유지하며, **총 행정 약 690 mm**, **자동 이동 10–680 mm**, **낮음·중간·높음 세 구간 거리 보정 UI**를 적용했습니다.
690 mm는 실측한 대략의 행정입니다. 이 숫자로 엔코더 거리 환산값을 계산하지 않습니다.

## 1. 라파에 설치

라파 터미널에서 실행합니다. Raspberry Pi OS/Linux, Python 3.10 이상이 필요합니다.

```bash
sudo apt update
sudo apt install -y git python3-tk python3-venv
# 비공개 저장소라면 먼저 GitHub 계정 접근 권한과 인증을 준비하세요.
git clone https://github.com/sy00107/lift_ctrl.git
cd lift_ctrl
bash scripts/install_pi.sh
```

설치 결과는 `~/lift`에 생성됩니다. 기존 UI 파일은 `~/lift/backups.install.*`에 백업합니다.
설치만으로 UI나 모터가 시작되지 않으며 피코 펌웨어도 바뀌지 않습니다.
이미 clone했다면 다시 clone하지 말고 아래 업데이트 절차를 사용하세요.

## 2. 피코에 펌웨어 업로드

**모터 24V 전원을 끄고, 라파와 피코 USB 전원은 유지하세요. 실행 중인 리프트 UI는 닫으세요.**
피코에는 MicroPython이 먼저 설치되어 있어야 합니다. 이 스크립트는 MicroPython UF2 설치가 아니라 피코의 `main.py`를 교체합니다.
MicroPython이 없는 새 피코는 [공식 Raspberry Pi Pico MicroPython 안내](https://www.raspberrypi.com/documentation/microcontrollers/micropython.html)를 따라 먼저 준비하세요.

저장소 폴더에서 실행합니다.

```bash
cd ~/lift_ctrl
~/lift/.venv/bin/python -m pip install -r requirements-pico.txt
~/lift/.venv/bin/python scripts/flash_pico.py --motor-power-off
```

여러 피코가 연결되어 있으면 다음 명령으로 포트를 확인하고 `--port`를 추가하세요.

```bash
ls -l /dev/serial/by-id/
# 예: 위에서 확인한 실제 경로로 바꾸세요.
~/lift/.venv/bin/python scripts/flash_pico.py --motor-power-off --port /dev/serial/by-id/실제피코경로
```

스크립트는 기존 펌웨어·보정 파일을 저장소의 `backups/`에 백업하고,
업로드한 파일을 다시 읽어 원본과 일치하는지 검증한 뒤 피코를 재시작합니다.
모터를 활성화하거나 이동하는 명령은 보내지 않습니다. 실패하면 모터 전원을 계속 끄고 오류와 백업을 확인하세요.
같은 피코의 기존 x1 거리 보정값·PD 설정은 유지하지만, 원점은 재설정해야 합니다.
x4 또는 다른 펌웨어의 보정 파일은 호환되지 않습니다. 보정 파일을 다른 피코로 복사하지 마세요.

## 3. 화면 실행

라파에 화면이 연결되어 있으면:

```bash
~/lift/start_position.sh
```

Linux 노트북에서 라파의 창을 띄우려면 **노트북** 터미널에서:

```bash
ssh -X 사용자명@라파주소
```

접속된 **라파** 터미널에서:

```bash
~/lift/start_position.sh
```

예를 들어 라파 계정이 `pi`, 호스트 이름이 `raspberrypi.local`이면 `ssh -X pi@raspberrypi.local`입니다.
실제 계정과 주소를 사용하세요. Windows/macOS는 별도 X 서버 또는 라파 데스크톱 원격 접속이 필요합니다.
`no display name and no $DISPLAY`가 나오면 SSH에서 나온 후 `ssh -X`로 다시 연결하거나 라파 데스크톱에서 실행하세요.
피코를 지정하려면 `~/lift/start_position.sh --port /dev/serial/by-id/실제피코경로`를 사용합니다.

## 4. 처음 사용하는 피코: 세 구간 거리 보정

펌웨어 업로드가 끝나면 모터 전원을 복구하고 주변 이동 공간을 확보하세요.
UI에서 **연결**을 누르고, **높이 보정 · 세 구간 측정 → 설정 시작**을 누릅니다.
UI는 측정을 위해 자동 이동하지 않습니다. **올리기/내리기 버튼을 누르는 동안만 이동**하고, 놓으면 정지합니다.

각 구간에서는 자를 같은 기준점에 대고, **측정 전후 두 길이의 차이**를 기록하세요.

| 순서 | 할 일 | 입력하는 값 |
| --- | --- | --- |
| 낮은 구간 | 하단 끝단을 피한 낮은 위치로 이동해 멈춘 뒤 **낮은 구간 측정 시작** → 시작 길이를 자로 기록 → 위로 30 mm 이상 이동 → 정지 후 끝 길이 측정 | 끝 길이 − 시작 길이 |
| 중간 구간 | 중간 위치로 이동해 멈춘 뒤 **중간 구간 측정 시작** → 같은 방법으로 상승·측정 | 그 중간 구간에서 올라간 거리만 |
| 높은 구간 | 상단 끝단까지 여유를 남긴 높은 위치에서 **높은 구간 측정 시작** → 같은 방법으로 상승·측정 | 그 높은 구간에서 올라간 거리만 |

**두 번째 입력은 원점 기준도, 첫 번째 구간 기준도 아닙니다.**
예를 들어 중간 구간의 측정 시작 길이가 300 mm, 끝 길이가 380 mm라면 **80 mm**를 입력합니다.
구간 사이에 위치를 바꾸기 위해 움직인 거리는 측정에 포함하지 않습니다.
낮음·중간·높음은 작업자가 실제 위치를 보고 선택하며 UI가 자동 판별하지 않습니다.
각 구간은 최소 30 mm 이상, 가능하면 끝단을 피하면서 더 긴 거리를 같은 방향으로 재세요.

세 구간을 모두 저장하면 UI가 구간별 환산값 차이와 최대 거리 잔차를 보여줍니다.
**내리기를 짧게 눌렀다 놓아 반대 방향을 확인한 뒤 ‘세 구간 보정 적용’**을 누릅니다.
적용 확인창에 동의하면 피코에 보정값이 저장되고 기존 원점이 해제됩니다.

- 세 구간의 엔코더 변화량과 실측 상승량으로 하나의 mm/count를 최소제곱 계산합니다.
- 구간별 환산값의 최대–최소 차이가 종합값의 5%를 넘으면 적용하지 않고 재측정을 안내합니다.
  5%는 큰 측정 오류를 거르는 임시 기준이며 정확도 보증이 아닙니다.
- 최대 거리 잔차는 실측값과 계산값의 차이이며, 리프트의 실제 위치 정확도를 보증하지 않습니다.
- 측정 중 **보정 취소 / 처음부터**를 누르거나 연결이 끊기면 미완료 측정은 버립니다.
  **보정 적용 전까지 기존 피코 보정값은 유지됩니다.** 적용 후 취소로 이전 값을 복원하지는 않습니다.
- 구간마다 별도의 비선형 보정값을 피코에 저장하는 방식은 아닙니다.
  구간 차이가 크면 자의 기준점·측정 방법·배선·센서부터 확인하세요.
- 세 구간 실측값과 적용 요청은 `~/.local/state/yor-lift/diagnostics.jsonl`에 기록합니다.

## 5. 원점 설정과 이동

1. 실제 최하단에 맞추고 정지합니다. 필요한 경우 **상세 설정 → 원점 해제** 후 수동으로 조심히 맞춥니다.
2. **지금이 맨 아래예요 · 0 mm 설정**을 누르고 실제 하단임을 확인합니다.
3. `현재 높이: 0.0 mm`가 확인되면 목표 높이 **10–680 mm**를 입력하고 **이동**을 누릅니다.
4. 수동 이동은 버튼 또는 ↑/↓ 키를 누르는 동안입니다. **정지/Esc**는 정지·비활성화합니다.

전원을 껐다 켜거나 피코를 재시작하면 **최하단 원점을 다시 설정**해야 합니다.
저장된 보정값이 유효하면 세 구간 측정은 매번 반복하지 않습니다.
이 버전은 **자동 원점 찾기를 지원하지 않습니다.**

원점이 있으면 하단 약 11 mm 이하에서 추가 하강, 상단 약 679 mm 이상에서 추가 상승을 정상 차단하며 반대 방향 이동은 가능합니다.
이것은 엔코더 기준 소프트 제한입니다. **원점이 없으면 위치 제한이 적용되지 않습니다.**
바닥·걸림·센서 단선을 무펄스만으로 구분할 수 없으므로 하단을 밀어 자동으로 0점 등록하지 않습니다.
정확한 하단 감지가 필요하면 별도의 하단 리미트 스위치를 추가해야 합니다.

기본 UI 설정은 Kp 0.04, Kd 0.001, PWM 상한 100%입니다. 상세 설정에서 조절할 수 있습니다.
목표 허용오차 ±0.2 mm는 **엔코더 계산값 기준**이며, 자로 잰 실제 위치의 ±0.2 mm 정확도를 뜻하지 않습니다.
상·하단에서 실제 멈추는 위치와 하중에 따른 오차는 실물에서 별도 확인해야 합니다.

## 하드웨어와 배선

FLEXISPOT E6 3단 기둥, BTS7960 드라이버, Raspberry Pi Pico를 대상으로 합니다.

| 신호 | Pico GPIO |
| --- | --- |
| RPWM / LPWM | 2 / 3 |
| R_EN / L_EN | 4 / 5 |
| 엔코더 A / B | 27 / 28 |

엔코더는 A 상승 에지와 B 방향 판정을 사용하는 **x1** 방식입니다.
다른 핀·모터 드라이버·센서를 사용한다면 그대로 실행하지 말고 배선을 맞추세요.
제조사의 두 기둥 책상 전체 허용하중을 단일 리프트의 허용하중으로 간주하지 마세요.

## 연결·오류 해결

- **USB 권한 오류:** `groups`로 `dialout` 포함 여부를 확인합니다.
  필요하면 `sudo usermod -aG dialout "$USER"` 후 로그아웃·로그인하세요.
- **포트가 없음:** 피코 USB 데이터 케이블과 MicroPython 설치를 확인하고 `ls -l /dev/serial/by-id/`를 실행하세요.
- **포트 사용 중:** 다른 UI나 `mpremote`를 닫으세요. 업로드 중에는 UI를 실행하지 마세요.
- **ModemManager 간섭:** 피코에 `AT` 문자가 들어가는 경우 `bash scripts/ignore_modemmanager.sh`를 한 번 실행하고 피코 USB를 다시 연결하세요. sudo가 필요합니다.
- **`encoder_no_pulse`:** 출력이 켜진 누적 800 ms 동안 펄스가 없습니다. 원점 미등록 상태에서 실제 바닥을 계속 누르거나, 기구 걸림·센서 유실일 수 있습니다. 원인을 확인하고 오류 해제 후 원점을 다시 설정하세요. 자동 재출발하지 않습니다.
- **`motion_no_progress` / `encoder_direction_mismatch`:** 실제 진행 또는 엔코더 방향 문제입니다. 배선·기구를 확인하고 원점을 다시 설정하세요.
- **`command_timeout`:** 통신 heartbeat가 300 ms 이상 끊겨 정지했습니다.
- **`target_overshoot` / `target_not_settled`:** 목표를 초과했거나 정착하지 못했습니다. 하중·PD 설정을 확인하세요.
- **원점 설정 실패:** 정지 후 잠시 기다리고, 보정 완료 여부와 실제 하단을 확인하세요.
- **진단 로그:** `~/.local/state/yor-lift/diagnostics.jsonl`

## 업데이트

실행 중인 UI를 닫고, 저장소 폴더에서:

```bash
cd ~/lift_ctrl
git pull --ff-only
bash scripts/install_pi.sh
```

피코 코드가 바뀌었다면 **모터 24V 전원 OFF, 피코 USB 유지** 후 2번의 업로드 명령도 별도로 실행하세요.
`git pull`이나 UI 설치만으로 피코 내부 펌웨어가 바뀌지는 않습니다.

## 테스트와 파일 구성

실물 없이 테스트합니다. GUI 테스트까지 실행하려면 디스플레이 또는 Xvfb가 필요합니다.

```bash
sudo apt install -y xvfb
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements.txt
xvfb-run -a .venv/bin/python -m unittest discover -s tests -p 'test_*.py'
.venv/bin/python simulation/sim_lift_smooth.py
```

```text
pico/main.py                              # Pico MicroPython 펌웨어
raspberry_pi/lift/start_position.sh        # UI 실행 스크립트
raspberry_pi/lift/position/lift_position.py # 높이 제어·세 구간 보정 UI
raspberry_pi/lift/position/lift_joystick.py # 공통 통신·키보드 처리
scripts/install_pi.sh                     # ~/lift에 UI 설치·기존 파일 백업
scripts/flash_pico.py                     # Pico 백업·업로드·읽기 검증
scripts/ignore_modemmanager.sh            # 필요 시 USB 간섭 방지 설정
tests/                                   # 가상 제어기·직렬 통신·Tk UI 테스트
simulation/                              # 실물 없는 구동 시뮬레이션
docs/simulation/                          # 과거 감속 비교 결과(현 버전 실물 검증 아님)
```

테스트와 시뮬레이션은 실제 하중·마찰·정지거리·위치 정확도를 보장하지 않습니다.
장치별 보정 파일·USB 장치 ID·백업·개인 진단 로그는 저장소에 공유하지 않습니다.

이번 공유본의 검증: 자동 테스트 75개 통과, 임시 경로 UI 설치 확인.
가상 구동 비교에서는 수정 제어기 75개 조건 중 60개가 목표에 도달했고,
나머지 15개는 높은 마찰 가정에서 `encoder_no_pulse`로 정지했습니다.
이는 자동 테스트 실패와 별개인 시뮬레이션의 구동 한계이며, 실물에서 모든 하중이 구동된다는 뜻은 아닙니다.

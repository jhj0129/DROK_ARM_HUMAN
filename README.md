# DROK ARM HUMAN

DROK 6-DOF 로봇팔의 HumanArm Mode, 전원 재인가 기준점 처리, 실제팔 Home 이동, 1 kHz 부드러운 관절 궤적, Full-Pose DLS IK 기반 Pick 테스트를 위한 ROS 2 Humble 워크스페이스입니다.

Repository
https://github.com/jhj0129/DROK_ARM_HUMAN

---

## 1. 현재 구현 기능

- 실제 RMD 모터 RAW angle 모니터링
- can10 / can11 SocketCAN 사용
- J1~J6 HumanArm Home
- J6 전원 재인가 Session Reference
- J7 전원 재인가 FULL OPEN Session Reference
- 전원 ON 후 처진 자세 -> 기존 Legacy Home -> HumanArm Home 자동 이동
- ARM_BASE_LINK 기준 Full-Pose DLS IK
- 1 kHz 관절 명령 스트리밍
- 9th-order smootherstep trajectory
- J5 미세 헌팅 감소용 저속/고해상도 명령
- 그리퍼 FULL OPEN -> 접근 -> CLOSE -> 물체를 잡은 채 HumanArm Home 복귀
- 테스트 Pick target: ARM_BASE_LINK 기준 X = +0.60 m

---

## 2. 기준 환경

- Ubuntu Linux
- ROS 2 Humble
- SocketCAN
- CAN interface:
  - can10
  - can11
- CAN bitrate:
  - 1 Mbps

Python/ROS dependency는 build.sh에서 rosdep으로 설치하도록 구성되어 있습니다.

IK 및 기존 실제팔 bridge는 아래 저장소의 검증된 commit을 build.sh가 자동으로 받아옵니다.

DROK_ARM_IK
https://github.com/jhj0129/DROK_ARM_IK

Pinned commit:
26be1ec8480b0b44a26221fec2810bd5a38951be

따라서 사용자가 별도로 DROK_ARM_IK를 clone할 필요는 없습니다.

---

## 3. Clone

```bash
cd ~
git clone https://github.com/jhj0129/DROK_ARM_HUMAN.git
cd ~/DROK_ARM_HUMAN
```

---

## 4. Build

최초 1회:

```bash
cd ~/DROK_ARM_HUMAN
bash build.sh
```

build.sh가 자동으로 수행하는 작업:

1. ROS 2 Humble 환경 source
2. 검증된 DROK_ARM_IK commit 다운로드
3. drok_arm_kinematics 연결
4. drok_real_arm_bridge 연결
5. rosdep dependency 설치
6. colcon build --symlink-install
7. 실행 shell script에 실행 권한 부여

빌드 후 수동 source가 필요한 경우:

```bash
source /opt/ros/humble/setup.bash
source ~/DROK_ARM_HUMAN/install/setup.bash
```

---

## 5. CAN 확인

로봇 실행 전 can10 / can11이 존재하고 UP 상태여야 합니다.

```bash
ip -details link show can10
ip -details link show can11
```

정상 예:

```text
can state ERROR-ACTIVE
bitrate 1000000
```

CAN interface 설정은 사용하는 USB-CAN 장치에 맞게 별도로 수행합니다.

---

## 6. 실제 모터 Mapping

Logical joints:

- J1: Shoulder Yaw
- J2: Shoulder Pitch
- J3: Elbow Pitch
- J4: Wrist Yaw
- J5: Wrist Pitch
- J6: Wrist Roll
- J7: Gripper

Physical CAN mapping:

```text
J1       can10  0x141
J2_MAIN  can10  0x142
J2_SLAVE can10  0x143
J3       can10  0x144

J4       can11  0x141
J5       can11  0x142
J6       can11  0x143
J7       can11  0x144
```

Gear / angle convention used by HumanArm:

```text
J1  36:1
J2  36:1
J3  36:1
J4   1:1
J5   1:1
J6   6:1
J7   6:1
```

---

## 7. 중요한 J6 / J7 전원 ON 규칙

J6과 J7은 이전 부팅의 고정 RAW angle을 절대적인 물리 자세 기준으로 사용하지 않습니다.

전원을 넣을 때 실제 로봇은 반드시 다음 상태로 시작합니다.

```text
J6 = 기존 Legacy 방향
J7 = Gripper FULL OPEN
```

전원이 켜지면 현재 RAW를 이번 세션의 기준점으로 저장합니다.

J6:

```text
이번 부팅의 Legacy RAW
        ↓
시계방향 90 deg
        ↓
HumanArm J6 Home
```

software relation:

```text
J6 HumanArm Home RAW
=
J6 Boot Legacy RAW - 90 deg
```

J7:

```text
전원 ON 시 현재 RAW = FULL OPEN
```

그리퍼 닫힘은 FULL OPEN 기준 상대 이동으로 계산합니다.

```text
J7 close travel ≈ +87.7211 deg
```

Session file:

```text
~/.cache/drok_arm_human/session_reference.yaml
```

이 파일은 매 전원 ON 세션마다 새로 생성됩니다.

---

## 8. 전원 ON 후 HumanArm Home 이동

전원을 켰을 때 J2/J3가 중력 때문에 아래로 처져 있어도 괜찮습니다.

실행:

```bash
cd ~/DROK_ARM_HUMAN
./go_humanarm_home.sh
```

최초 build 직후 실행권한 문제가 있으면:

```bash
bash go_humanarm_home.sh
```

동작 순서:

```text
POWER ON
   ↓
현재 J6 Legacy RAW 캡처
현재 J7 FULL OPEN RAW 캡처
   ↓
MOVE_ARM 확인
   ↓
현재 처진 자세
   ↓
Legacy Home
J1 HOLD
J2~J5 Legacy Home
J6 HOLD
   ↓
Legacy bridge 종료
   ↓
HumanArm Home
J1~J5 calibrated Home
J6 = 이번 세션 Legacy - 90 deg
J7 = FULL OPEN 유지
```

실행 중:

```text
실행하려면 MOVE_ARM 입력:
```

이 나오면 주변 상태를 확인하고:

```text
MOVE_ARM
```

을 입력합니다.

---

## 9. HumanArm RAW Monitor

수동으로 모터 RAW 값을 보고 싶을 때:

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/DROK_ARM_HUMAN/install/setup.bash

ros2 run humanarm_mode humanarm_motor_monitor
```

Terminal 2:

```bash
ros2 topic echo /humanarm/raw_motor_deg
```

배열 순서:

```text
[
  J1,
  J2_MAIN,
  J2_SLAVE,
  J3,
  J4,
  J5,
  J6,
  J7
]
```

---

## 10. HumanArm 상태 확인

Raw monitor가 실행 중인 상태에서:

```bash
ros2 run humanarm_mode humanarm_session_control status
```

HumanArm Home에서는 J1~J6 HOME_ERR가 거의 0 deg 근처여야 합니다.

---

## 11. 60 cm IK Pick 테스트

먼저 반드시 현재 전원 세션에서 HumanArm Home 이동을 완료합니다.

```bash
cd ~/DROK_ARM_HUMAN
./go_humanarm_home.sh
```

그 다음:

```bash
./pick_60cm.sh
```

Pick sequence:

```text
HumanArm Home
      ↓
J7 FULL OPEN
      ↓
Full-Pose DLS IK
ARM_BASE_LINK 기준 TCP X = +0.60 m
현재 Y/Z 및 TCP orientation 유지
      ↓
1 kHz smooth joint trajectory
      ↓
J7 CLOSE / GRASP
      ↓
물체 파지 유지
      ↓
HumanArm Home 복귀
      ↓
J7 CLOSED 유지
```

---

## 12. IK 방식

IK solver:

```text
Full-Pose Damped Least Squares
```

Target:

```text
Base frame : ARM_BASE_LINK
Tool frame : gripper_tcp

Target X   : 0.600 m
Target Y   : 현재 TCP Y 유지
Target Z   : 현재 TCP Z 유지
Target RPY : 현재 TCP orientation 유지
```

현재 HumanArm Home에서 실제 확인된 예:

```text
Start TCP X ≈ 0.3603 m
Target TCP X = 0.6000 m
```

따라서 현재 데모에서는 약 24 cm 정도 앞으로 뻗습니다.

---

## 13. Smooth Motion

실제 모터 이동 완성도를 위해 관절 궤적은 다음 설정을 사용합니다.

```text
Control / stream rate : 1000 Hz
Nominal period        : 1.000 ms
Trajectory            : 9th-order smootherstep
```

9th-order smootherstep은 시작/종료에서 다음 값이 0이 되도록 구성했습니다.

```text
velocity
acceleration
jerk
snap
```

J5는 손목 하중과 작은 setpoint에서 보였던 미세 흔들림을 줄이기 위해:

```text
minimum protocol speed = 3
command quantum        = 1 encoder count
                       ≈ 0.01 deg
```

을 사용합니다.

또한 encoder count 명령은 진행 방향에 대해 monotonic하게 제한합니다.

---

## 14. 실제 측정된 1 kHz timing 예

실제 Pick 테스트에서 측정:

Forward:

```text
Average period : 1.0000 ms
Maximum period : 1.1246 ms
> 1.5 ms       : 0 / 12000
> 2.0 ms       : 0 / 12000
```

Return:

```text
Average period : 1.0000 ms
Maximum period : 1.0662 ms
> 1.5 ms       : 0 / 12000
> 2.0 ms       : 0 / 12000
```

따라서 해당 Ubuntu 환경에서는 Python 기반 trajectory streamer로도 1 kHz 주기가 안정적으로 확인되었습니다.

---

## 15. 실제 확인된 Pick 결과 예

IK result:

```text
Success        : true
Iterations     : 16
Position error : 0.000000131 m
```

Target:

```text
ARM_BASE_LINK X = 0.600 m
```

Gripper:

```text
FULL OPEN
24.96 deg

→

CLOSED
112.68 deg
```

동작 종료 후 로봇은 HumanArm Home으로 돌아오고 J7은 닫힌 상태를 유지합니다.

---

## 16. 주요 파일

```text
DROK_ARM_HUMAN/
├── build.sh
├── go_humanarm_home.sh
├── pick_60cm.sh
└── src/
    └── humanarm_mode/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── config/
        │   ├── humanarm_home.yaml
        │   ├── humanarm_mapping.yaml
        │   └── robot_geometry.yaml
        └── humanarm_mode/
            ├── common.py
            ├── raw_monitor.py
            ├── boot_reference.py
            ├── session_control.py
            ├── legacy_home_once.py
            └── pick_60cm.py
```

build.sh 실행 후 자동으로 다음 검증된 dependency가 `.deps/`에 생성됩니다.

```text
.deps/DROK_ARM_IK
```

그리고 아래 ROS package가 workspace의 src에 연결됩니다.

```text
drok_arm_kinematics
drok_real_arm_bridge
```

---

## 17. 주요 명령 요약

Build:

```bash
cd ~/DROK_ARM_HUMAN
bash build.sh
```

HumanArm Home:

```bash
cd ~/DROK_ARM_HUMAN
./go_humanarm_home.sh
```

60 cm Pick:

```bash
cd ~/DROK_ARM_HUMAN
./pick_60cm.sh
```

RAW monitor:

```bash
ros2 run humanarm_mode humanarm_motor_monitor
```

Status:

```bash
ros2 run humanarm_mode humanarm_session_control status
```

J6만 HumanArm Home:

```bash
ros2 run humanarm_mode humanarm_session_control j6-home
```

J6만 이번 세션 Legacy 위치:

```bash
ros2 run humanarm_mode humanarm_session_control j6-legacy
```

---

## 18. 실행 시 주의

동시에 둘 이상의 위치제어 CAN writer를 실행하지 마십시오.

예:

```text
moveit_to_rmd_bridge
joystick_to_rmd_control
drokck
HumanArm direct controller
```

Pick runner는 일부 대표적인 충돌 node를 검사해서 실행을 차단합니다.

Raw monitor는 read request를 보내고 feedback을 받는 용도입니다.

로봇을 실제로 구동하기 전:

```text
can10 UP
can11 UP
J6 power-on Legacy orientation
J7 FULL OPEN
주변 작업공간 확인
비상정지 사용 가능 상태
```

를 확인합니다.

---

## 19. 현재 기준

이 저장소의 현재 기준 기능:

```text
Power ON
→ Sagged Arm
→ Legacy Home
→ HumanArm Home
→ J7 Open
→ Full-Pose IK to X=0.60 m
→ J7 Grasp
→ HumanArm Home
```

HumanArm 동작의 핵심 기준은 다음입니다.

```text
J1~J5 : calibrated static HumanArm Home
J6    : power-on session-relative Home
J7    : power-on session-relative FULL OPEN
```

J6/J7의 이전 부팅 absolute RAW 값은 물리적 절대 자세 기준으로 사용하지 않습니다.

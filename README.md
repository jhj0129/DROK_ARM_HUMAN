# DROK_ARM_Sim_only

DROK ARM용 **ROS 2 Humble + MuJoCo 시뮬레이션 / 실시간 RC 개발 워크스페이스**입니다.

현재 개발의 중심 목표는 **Meta Quest 3 기반 초저지연 VR RC 제어**입니다. 정밀한 자동화용 trajectory tracking보다 사람이 화면을 보면서 즉시 조작하는 Human-in-the-loop RC를 우선하며, 위치·자세 오차는 비교적 러프하게 허용하고 **응답성, 1:1 추종감, 낮은 제어 지연**을 최우선으로 둡니다.

---

## Current verified status

현재 확인된 항목:

- ROS 2 Humble + MuJoCo simulation backend
- JOINT1 ~ JOINT6 arm simulation
- JOINT7 gripper interface
- `/joint_states` feedback
- `/drok_arm/joint_command` direct command
- calibrated HumanArm pose 기반 테스트
- endpoint FK / IK
- 1 kHz realtime RC controller
- 1:1 relative Cartesian VR mapping
- Geometric Jacobian
- Differential resolved-rate control
- latest-pose-only QoS (`KEEP_LAST(1)`, `BEST_EFFORT`)
- VR Cartesian velocity feed-forward
- direct 6x6 Jacobian solve in normal operation
- geometric arm/wrist singularity monitoring
- Singularity Firewall v1
- emergency-only DLS fallback
- fake 120 Hz VR keyboard input for Quest-free testing

실시간 RC 초기 시험에서 기존 Adaptive-DLS 버전은 일반적으로 다음 계산 성능을 확인했습니다.

```text
Control period avg      ~= 1000 us
Controller compute avg  ~= 25 ~ 40 us
Typical compute max     ~= 70 ~ 230 us
VR sample age           ~= 0 ~ 8 ms @ 120 Hz fake input
```

현재 Fast-Path + Singularity Firewall 버전에서는 normal path의 eigen decomposition / sigma_min 계산 / 상시 DLS를 제거하고 direct solve를 사용합니다.

---

## VR RC design goal

목표는 다음과 같습니다.

```text
Quest controller 10 cm 이동
        ->
Robot TCP 약 10 cm 이동

Quest controller 20 deg 회전
        ->
Robot wrist 약 20 deg 회전
```

기본 mapping:

```text
Translation scale = 1.0
Rotation scale    = 1.0
```

RC는 높은 정밀도보다 사람이 실제 로봇을 보면서 즉시 보정하는 방식에 맞춰 설계합니다.

우선순위:

```text
1. Responsiveness
2. Low latency
3. Natural 1:1 tracking
4. Singularity prevention / safe fallback
5. Precision
```

---

## Realtime RC architecture

```text
Meta Quest 3 / Fake VR input
          |
          | latest pose only
          v
+----------------------------------+
| Realtime RC Controller : 1000 Hz |
|                                  |
| Relative 1:1 VR target           |
|        |                         |
| Cartesian error + feed-forward   |
|        |                         |
| Geometric Jacobian               |
|        |                         |
| FAST direct solve                |
|        |                         |
| Singularity Firewall             |
|        |                         |
| Joint velocity clamp             |
|        |                         |
| q_cmd += qdot * 0.001            |
+----------------------------------+
          |
          v
/drok_arm/joint_command
          |
          v
MuJoCo / future real RMD-CAN bridge
```

Normal operation에서는 iterative pose IK를 사용하지 않습니다.

```text
No 8 / 16 / N iteration IK convergence loop
No trajectory queue for VR tracking
No heavy smoothing buffer
No old-pose processing
```

매 제어 cycle에서 최신 VR target을 기준으로 한 번 계산하고 바로 다음 joint command를 생성합니다.

---

## FAST solve + Singularity Firewall

### Normal region

정상 영역에서는 damping 없이 direct solve를 사용합니다.

```text
Geometric Jacobian J
      |
      v
J * qdot = desired_twist
      |
      v
FAST direct solve
```

상시 SVD / eigen decomposition / Adaptive-DLS를 사용하지 않습니다.

### Known singularity metrics

Arm/elbow singularity metric:

```text
arm_metric = |u_upper x u_forearm|
```

- `1.0`에 가까움: 링크 방향이 충분히 다름
- `0.0`에 가까움: arm links가 일직선 / anti-parallel에 가까움

Wrist singularity metric:

```text
wrist_metric = |axis_J4 x axis_J6|
```

- `1.0`에 가까움: J4/J6 world axis가 충분히 다름
- `0.0`에 가까움: J4/J6 world axis가 평행 / 반평행

### Firewall behavior

현재 v1 Firewall은 30 ms look-ahead를 사용해 known singular geometry로 더 가까워지는 방향인지 확인합니다.

```text
SAFE
  -> VR command 그대로 통과

WARNING
  -> singularity 방향 component만 점진 감쇠

HARD boundary
  -> 해당 singularity 쪽 component 차단

Away from singularity
  -> 제한하지 않음
```

기본값:

```yaml
firewall_arm_warning_metric: 0.15
firewall_arm_hard_metric: 0.05

firewall_wrist_warning_metric: 0.20
firewall_wrist_hard_metric: 0.10

firewall_lookahead_sec: 0.030
```

> 현재 Firewall v1은 DROK ARM의 알려진 arm/wrist singular geometry를 대상으로 한 **geometric heuristic prevention layer**입니다. Formal Control Barrier Function의 forward-invariance를 수학적으로 증명한 구현은 아직 아닙니다. 이후 필요하면 gradient/CBF projection 기반 Firewall v2로 확장합니다.

### Emergency DLS

DLS는 normal path에서 사용하지 않고 다음 경우에만 fallback으로 사용합니다.

```text
Jacobian rank loss
Direct solution NaN / Inf
Raw joint velocity abnormal spike
```

기본값:

```yaml
emergency_dls_lambda: 0.080
emergency_raw_qdot_rad_s: 20.0
```

---

## ROS 2 topics

```text
VR pose        : /vr/right_hand_pose
VR relink      : /vr/relink
Robot command  : /drok_arm/joint_command
Robot feedback : /joint_states
```

Realtime RC 관련 topic은 stale command 누적을 막기 위해 기본적으로:

```text
KEEP_LAST(1)
BEST_EFFORT
```

를 사용합니다.

RC에서는 과거 pose를 순서대로 처리하는 것보다 **가장 최신 pose 하나만 사용하는 것**이 중요합니다.

---

## Repository structure

```text
DROK_ARM_Sim_only/
├── README.md
├── config/
├── docs/
├── src/
│   ├── drok_arm_description/
│   ├── drok_arm_kinematics/
│   ├── drok_arm_mujoco/
│   └── drok_realtime_rc/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/
│       │   └── realtime_rc.yaml
│       └── src/
│           └── realtime_rc_controller.cpp
└── tools/
    ├── setup.sh
    ├── source_env.sh
    ├── run_sim.sh
    ├── go_home.sh
    └── fake_vr_keyboard.py
```

---

## Build

```bash
cd ~/DROK_ARM_Sim_only

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source ~/DROK_ARM_Sim_only/install/setup.bash
```

Realtime RC만 다시 빌드할 경우:

```bash
cd ~/DROK_ARM_Sim_only
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select drok_realtime_rc

source ~/DROK_ARM_Sim_only/install/setup.bash
```

---

## Run realtime RC test without Quest 3

### Terminal 1 - MuJoCo

```bash
cd ~/DROK_ARM_Sim_only
source /opt/ros/humble/setup.bash
source install/setup.bash

./tools/run_sim.sh
```

### Terminal 2 - Realtime RC controller

```bash
cd ~/DROK_ARM_Sim_only
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run drok_realtime_rc realtime_rc_controller \
  --ros-args \
  --params-file \
  /home/$USER/DROK_ARM_Sim_only/src/drok_realtime_rc/config/realtime_rc.yaml
```

정상 시작 예:

```text
Control       : 1000 Hz
Mapping       : translation 1.00 : 1 / rotation 1.00 : 1
Jacobian      : geometric
IK            : FAST direct solve + geometric firewall
Emergency     : DLS fallback only
QoS           : KEEP_LAST(1), BEST_EFFORT
```

### Terminal 3 - Fake VR input

```bash
cd ~/DROK_ARM_Sim_only
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 tools/fake_vr_keyboard.py
```

Keyboard mapping:

```text
W / S : +X / -X
A / D : +Y / -Y
R / F : +Z / -Z

U / O : +Roll  / -Roll
I / K : +Pitch / -Pitch
J / L : +Yaw   / -Yaw

0     : Relink current VR pose to current robot TCP
Q     : Quit
```

Fake VR publishes at 120 Hz.

---

## Diagnostic log

Realtime controller prints approximately once per second:

```text
period avg/max=... us
compute avg/max=... us
VR age=... ms
err=... mm / ... deg
arm=...
wrist=...
mode=...
q track max=... deg
```

`mode` values:

```text
FAST
FIREWALL_ARM
FIREWALL_WRIST
FIREWALL_ARM+WRIST
DLS_EMERGENCY
```

Normal teleoperation에서는 대부분 `FAST`가 유지되는 것이 목표입니다.

---

## Current controller philosophy

정밀 자동화와 VR RC는 같은 controller requirement를 갖지 않습니다.

### Autonomous / endpoint motion

사용 가능:

```text
Endpoint IK
Trajectory interpolation
Completion tolerance
Settle check
```

### VR RC

우선 사용:

```text
Latest pose only
1:1 relative mapping
Velocity feed-forward
Geometric Jacobian
One solve per 1 ms cycle
Loose error acceptance
Human visual feedback
Singularity prevention firewall
Emergency-only DLS
```

---

## Next development steps

현재 다음 우선순위로 진행합니다.

```text
1. Fast-Path + Singularity Firewall timing/behavior 검증
2. Actual q feedback 기반 TCP tracking 분리 측정
3. Command lead limiter 추가
4. Linux realtime scheduling / CPU affinity 적용
5. Meta Quest 3 actual pose stream 연결
6. Real RMD/CAN bridge 연결
7. 실물 VR RC latency measurement
8. 필요 시 CBF-style Singularity Firewall v2
```

최종 목표는 **사람이 VR 컨트롤러를 움직이는 순간 DROK ARM이 가능한 한 같은 움직임을 즉시 따라가는 초저지연 Human-in-the-loop RC 시스템**입니다.

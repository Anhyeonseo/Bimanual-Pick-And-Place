# 단계 5 STM32 arm/gripper Action·backend 잠금 시험 결과

- 날짜: 2026-07-24
- 판정: **PASS — Gate 5B, 통합 selector 및 mock vertical slice 완료**
- baseline commit: `c556a26d2e1066e79d5e2da052aaec6879ec9f93`
- working tree: ROS Action adapter, backend 잠금과 단계 5 문서 변경 미commit
- host: Ubuntu 24.04.4 LTS, x86_64
- Python: 3.12.3
- ROS environment: Jazzy — Action integration과 package build에 사용
- 실제 STM32/servo/serial: **비활성**

## 구현 범위

- 순수 Python `action_validation` 계층
- calibration raw endpoint에서 project radian safe limit 생성
- arm 5-joint contract와 입력 순서 재정렬
- single-point 및 `300..2000 ms` duration 제한
- NaN/Inf, 누락·중복·미지 joint와 잘못된 position count 거부
- multi-point와 non-monotonic timestamp 거부
- 지원하지 않는 velocity, acceleration과 effort field 거부
- arm/gripper verified safe range 밖 target 거부, clamp 없음
- hardware gripper에서 simulation `open=1.91986 rad` 거부
- process lifetime의 단일 SO-101 backend runtime lease
- lock owner backend, PID와 ROS domain 진단 정보 기록
- STM32 serial POSIX exclusive open
- `/left_arm_controller/follow_joint_trajectory` ActionServer
- arm 5축 target과 최신 gripper feedback을 하나의 6축 packet으로 결합
- fresh feedback 없음, READ_ONLY, blocked core와 동시 goal 사전 거부
- Action success, feedback, cancel, abort와 connection-loss result 전파
- Action 활성 중 legacy `/joint_command` 이중 진입 차단
- 2-thread executor와 serial transaction lock
- Jazzy `ParallelGripperCommand`의 `JointState` command/result/feedback 적용
- gripper target 전송 중 arm 5축 최신 feedback 보존
- arm/gripper 공용 arbiter로 양방향 동시 Action acceptance 차단
- gripper 고정 duration `1000 ms`, velocity/effort 입력 reject
- 단일 `backend:=mock|isaac|stm32` MoveIt bringup entrypoint
- launch process가 node 시작 전에 획득하는 runtime backend lease
- 검증된 PID/backend/ROS domain 기반 STM32 child lock handoff
- 기존 mock/Isaac launch의 고정 backend 호환 wrapper
- STM32 `allow_motion=false` 기본값 유지

추가로 pure `MotionExecutionCore`가 다음을 소유한다.

- verified firmware/protocol/joint count/calibration identity gate
- setpoint acceptance sequence와 calibration identity 검사
- final error `20 raw` 이하에서만 success
- cancel → SAFE_STOP과 late result 무효화
- 통신 단절 → abort/block, reconnect 후 stale goal 미재전송
- 명시적 recovery에서 identity와 stop latch 재검사

ROS Jazzy의 실제 ActionClient와 ActionServer를 연결했지만 transport는
메모리 fake다. 따라서 ROS interface 전파 증거이며 실제 STM32/servo
end-to-end 증거는 아니다.

## 실행과 결과

### 새 Action validation 시험

```bash
python3 -m unittest discover \
  -s tests \
  -p 'test_single_arm_action_validation.py' \
  -v
```

- 결과: **15/15 PASS**

### 기존 bridge와 calibration 회귀

```bash
python3 -m unittest discover \
  -s tests \
  -p 'test_single_arm_bridge_core.py' \
  -v

python3 -m unittest discover \
  -s tests \
  -p 'test_joint_calibration.py' \
  -v
```

- bridge core: **11/11 PASS**
- calibration: **5/5 PASS**

### Action execution 상태 머신

```bash
python3 -m unittest discover \
  -s tests \
  -p 'test_single_arm_action_execution.py' \
  -v
```

- 결과: **15/15 PASS**
- calibration/firmware identity mismatch에서 setpoint 0회
- cancel 후 late success 무효화 PASS
- connection loss와 explicit recovery 사이 stale goal 재전송 0회
- transport 응답 불확실 시 SAFE_STOP best-effort PASS

### Backend exclusivity

```bash
python3 -m unittest discover \
  -s tests \
  -p 'test_single_arm_backend_exclusivity.py' \
  -v
```

- 결과: **7/7 PASS**
- 두 번째 backend non-blocking reject PASS
- lock 거부 시 기존 owner 정보 보존 PASS
- 정상 release 후 새 backend 재획득 PASS
- owner process 종료 후 OS 자동 lock 회수 PASS
- 잘못된 backend와 ROS domain에서 lock file 생성 0회
- lock file mode `0600` PASS
- STM32 serial `exclusive=True` PASS

### ROS FollowJointTrajectory integration

```bash
source /opt/ros/jazzy/setup.bash
python3 -m unittest discover \
  -s tests \
  -p 'test_single_arm_follow_joint_trajectory_ros.py' \
  -v
```

- 결과: **6/6 PASS**
- 성공 status/result와 feedback 전파 PASS
- 최신 gripper feedback을 6번째 target으로 보존 PASS
- invalid, READ_ONLY와 feedbackless goal에서 transport call 0회
- cancel → `STATUS_CANCELED`, SAFE_STOP 1회 PASS
- firmware failure → `STATUS_ABORTED` PASS
- connection loss → abort, stale resend 0회 PASS
- 동시 두 번째 goal reject, transport call 증가 0회

### ROS ParallelGripperCommand와 공용 arbiter integration

```bash
source /opt/ros/jazzy/setup.bash
python3 -m unittest discover \
  -s tests \
  -p 'test_motion_goal_arbiter.py' \
  -v
python3 -m unittest discover \
  -s tests \
  -p 'test_single_arm_parallel_gripper_ros.py' \
  -v
```

- arbiter 결과: **4/4 PASS**
- gripper ROS 결과: **8/8 PASS**
- hardware-safe gripper success/result/feedback 전파 PASS
- arm 5축 feedback 보존과 gripper 1축 target 결합 PASS
- simulation open `1.91986 rad`에서 transport call 0회
- cancel, firmware failure와 connection loss 전파 PASS
- arm active → gripper reject, gripper active → arm reject PASS
- 양방향 reject에서 두 번째 transport call 0회

### 통합 backend selector와 mock vertical slice

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch so101_bringup so101_moveit.launch.py backend:=mock
```

- selector unit: **5/5 PASS**
- launch-to-child lease handoff: **3/3 PASS**
- invalid `backend:=mock,isaac`: exit 1, provider process 0개
- 기존 `isaac` lock 보유 중 두 번째 mock launch: exit 1, provider process 0개
- 정상 종료 후 lock 재획득 PASS, lock file mode `0600`
- mock hardware `left_system`: initialize/configure/activate PASS
- `left_arm_controller`: active
- `left_gripper_controller`: active
- `joint_state_broadcaster`: active
- arm/gripper 표준 Action 각각 하나 확인
- 실제 serial/STM32 open 0회
- headless 검증의 RViz display error는 GUI가 없는 시험 환경에 의한 것으로,
  controller/MoveIt backend 판정과 분리

### ROS package build

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-up-to so101_bringup
```

- 결과: **PASS — 5 packages finished**

### 전체 host 회귀

```bash
python3 -m unittest discover -s tests -v
```

- 결과: **104/104 PASS** — ROS Jazzy source 환경

### 문법과 형식

- `compileall`: PASS
- `git diff --check`: PASS
- 변경 Python 파일 88자 초과 줄: 0
- project `flake8` config: **PASS — selector/lease 변경 파일**

## 확인된 차단 동작

| 입력 | 결과 |
|---|---|
| NaN, `+Inf`, `-Inf` | reject |
| missing, unknown, duplicate arm joint | reject |
| empty 또는 position count 불일치 | reject |
| 2개 이상 point | reject |
| non-monotonic timestamp | reject |
| `300 ms` 미만 또는 `2000 ms` 초과 | reject |
| verified safe range 밖 arm target | reject, clamp 없음 |
| hardware gripper `1.91986 rad` | reject |
| unsupported dynamic field | reject |
| READ_ONLY, blocked core 또는 fresh feedback 없음 | reject |
| scheduled start 또는 custom tolerance | reject |
| 이미 arm goal 실행 중인 두 번째 goal | reject |
| arm 실행 중 gripper 또는 gripper 실행 중 arm | reject |
| gripper velocity/effort 또는 scheduled command | reject |

## 다음 검증 항목

- Gate 5D 실제 STM32 READ_ONLY identity와 6축 feedback
- READ_ONLY에서 motion command 0회 확인
- 별도 명시 승인 후 Gate 5E 제한 동작

Gate 5B, arm/gripper ROS fake integration과 mock 통합 vertical slice는
완료됐다. 실제 STM32 검증은 Gate 5D READ_ONLY를 먼저 모두 통과한 뒤 별도 motion 승인으로 진행한다.

현재 실제 STM32/servo motion은 계속 허용하지 않는다.

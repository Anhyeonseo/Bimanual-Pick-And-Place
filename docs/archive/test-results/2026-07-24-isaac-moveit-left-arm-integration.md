# 왼팔 Isaac Sim·MoveIt 통합 시험 결과

## 시험 범위

- 날짜: 2026-07-24
- 대상: SO-ARM101 왼팔 arm 5-DOF + gripper 1-DOF
- backend: mock, Isaac Sim
- 실제 STM32/servo: 비활성
- 결과: **통과**

고장 난 반대편 팔을 제외하고 정상인 왼팔 하나로 URDF → MoveIt →
controller action → Isaac articulation의 vertical slice를 검증했다.

## 환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu 24.04.4 LTS |
| ROS 2 | Jazzy |
| MoveIt | 2.12.4 |
| Isaac Sim | 6.0.1 |
| DDS | Cyclone DDS, domain 30, subnet discovery |
| USD | `isaac_sim/assets/so101_new_calib/so101_new_calib.usda` |

## 결과

| ID | 검증 | 관찰 결과 | 판정 |
|---|---|---|---|
| SIM-ENV-001 | Isaac GUI/empty stage/Play/Stop | crash·freeze 없음 | 통과 |
| SIM-ROS-001 | ROS 2 Bridge startup | ROS 환경 source 후 extension 유지 | 통과 |
| MODEL-001 | URDF/Xacro visual 및 TF | RViz와 Isaac에서 왼팔 표시 | 통과 |
| MODEL-002 | q0와 축 방향 | 5 arm joint와 gripper 방향 사용자 관찰로 확정 | 통과 |
| MOVEIT-001 | SRDF와 collision config | setup assistant 및 q0 validity 정상 | 통과 |
| MOVEIT-002 | mock arm trajectory | Plan/Execute animation 정상 | 통과 |
| MOVEIT-003 | mock gripper trajectory | open/closed Plan/Execute 정상 | 통과 |
| ISAAC-001 | Articulation/drive 안정성 | 6 joint 속도 0으로 안정 유지 | 통과 |
| ISAAC-002 | Joint States OmniGraph | state publish와 command subscribe 정상 | 통과 |
| MAP-001 | project↔Isaac mapping | pytest 3개 통과 | 통과 |
| ISAAC-ACT-001 | direct arm q0 action | `SUCCEEDED` | 통과 |
| ISAAC-ACT-002 | direct gripper q0 action | `SUCCEEDED`, `reached_goal=true` | 통과 |
| E2E-001 | MoveIt arm → Isaac | random valid pose Plan/Execute `SUCCEEDED` | 통과 |
| E2E-002 | MoveIt gripper → Isaac | open/closed Plan/Execute `SUCCEEDED` | 통과 |
| E2E-003 | home 복귀 | 6 joint 모두 `0.03 rad` 이내 | 통과 |

## 축 방향 관찰

Isaac joint에 작은 각도를 적용하고 gripper 끝단의 변화를 관찰했다.

| Isaac 조작 | 관찰 | 확정한 project 의미 |
|---|---|---|
| `shoulder_pan -5 deg` | 끝단이 왼쪽 | `left_base_joint +q` → `+Y` |
| `shoulder_lift -5 deg` | 끝단이 위 | `left_shoulder_joint +q` → `+Z` |
| `elbow_flex -5 deg` | 끝단이 위 | `left_elbow_joint +q` → `+Z` flex |
| `wrist_flex -5 deg` | 끝단이 위 | `left_wrist_flex_joint +q` → `+Z` |
| `wrist_roll -5 deg` | 시계 방향 | `left_wrist_roll_joint +q` → `+X` RH |
| `gripper +q` | jaw 열림 | `left_gripper_joint +q` → open |

arm 5축은 sign inversion이 필요했고 gripper는 sign을 유지하되 project
zero를 맞추기 위해 `+10 deg` offset이 필요했다.

## 최종 상태

MoveIt에서 gripper `closed`와 arm `home`을 실행한 뒤 수신한 project joint
position은 다음과 같았다.

```text
[0.0, -0.0059, -0.0097, -0.0012, 0.0001, 0.0000329]
```

절댓값 최댓값은 약 `0.0097 rad`로 bridge의 `0.03 rad` goal tolerance
이내다. 작은 shoulder/elbow 오차는 중력 하에서 drive가 유지하는
정상상태 오차이며 joint velocity가 0으로 안정된 것을 함께 확인했다.

gripper open 실행에서는 project 약 `1.91973 rad`가 Isaac 약
`1.7452 rad` (`100 deg`)에 대응했다. 이는
`q_project = q_isaac + 10 deg` mapping과 일치한다.

## 발견한 문제와 해결

### ROS 2 Bridge가 켜졌다 꺼짐

- 증상: `[Error][isaacsim.ros2.core.impl.extenstion] ROS2 Bridge startup failed`
- 원인: desktop에서 시작한 Isaac process가
  `libament_index_cpp.so`를 찾지 못함
- 해결: `/opt/ros/jazzy/setup.bash`를 source한 terminal에서 Isaac Sim
  6.0.1을 `--enable isaacsim.ros2.bridge`로 시작

### Isaac 6.0.1 Python API 차이

- 증상: `pxr.UsdPhysics`에 `JointStateAPI`가 없음
- 해결: 저장된 drive target과 ROS 2 OmniGraph 기반 state/command 경로를
  사용

### gripper action feedback schema

- 증상: Jazzy `ParallelGripperCommand.Feedback`에 존재하지 않는
  `stalled`/`reached_goal` field 접근
- 해결: feedback은 `state`만 채우고 final result에서 goal 도달 상태를
  보고하도록 adapter 수정

### bridge 종료 중 이중 shutdown

- 증상: ROS context가 이미 종료된 뒤 다시 shutdown 시 예외
- 해결: `ExternalShutdownException` 처리와 `rclpy.ok()` guard 추가

## 안전 판정

- `so101_isaac_bridge`는 serial device를 열지 않는다.
- `single_arm_bridge`는 이 시험 동안 실행하지 않았다.
- 실제 STM32와 servo에는 trajectory를 보내지 않았다.
- 단계 5 시작 전까지 이 결과를 실제 hardware 동작 허가로 사용하지 않는다.

## 관련 파일

- `docs/checklists/PHASE_4_ISAAC_MOVEIT_INTEGRATION.md`
- `ros2_ws/src/so101_description`
- `ros2_ws/src/so101_moveit_config`
- `ros2_ws/src/so101_bringup`
- `ros2_ws/src/so101_isaac_bridge`
- `isaac_sim/assets/so101_new_calib`

# Raspberry Pi ROS 2 host bridge 실기 결과

## 검증 대상

- Raspberry Pi 5 / Ubuntu 24.04 / ROS 2 Jazzy
- NUCLEO-G474RE firmware `0x00020700`
- protocol v1 / calibration hash `0x3DB42B48`
- package: `single_arm_bridge` 0.1.0

## 검증 결과

| 요구사항 | 실기 증거 | 판정 |
|---|---|---|
| 실제 위치 feedback | `RAW_POSITION_FEEDBACK` 6축 수신 후 `/joint_states` radian 발행 | 통과 |
| 기본 무동작 | `allow_motion=false`에서 `JointTrajectory` 거부 및 실제 무동작 | 통과 |
| 재접속 | STM32가 이미 binary mode여도 물리 RESET 없이 HELLO 재접속 | 통과 |
| heartbeat 안전 | host 중단 뒤 STM32 stop latch, 재시작 시 `MOTION_BLOCKED_LATCHED` | 통과 |
| 명시적 복구 | `/clear_fault` 호출 시 STM32 위치 검사 후 ARM/ENABLE | 통과 |
| ROS 실제 이동 | base 약 2° 이동과 0° 복귀, 각각 최종 완료 응답 수신 | 통과 |
| 최종 실패 전파 | 비동기 `SETPOINT_STATUS`를 보존하며 status 7~9는 SAFE_STOP 처리 | 자동시험 통과 |

## 측정값

- 이동 명령: base `0.0349066 rad`, duration `1200ms`
- 이동·복귀 결과: 두 명령 모두 `motion completed`
- 최대 최종 오차: 17 raw. 주된 오차는 중력 하중을 받는 ELBOW의 home 주변 오차다.
- motion 중 position polling을 정지하고 heartbeat는 100ms 주기로 유지했다. 종료 300ms 후 실제 feedback을 재개했으며 transient timeout은 재발하지 않았다.

## 남은 제한

현재 node는 6축 실제 상태와 안전한 단일 waypoint만 처리한다. 여러 waypoint, `FollowJointTrajectory` action, URDF 기반 limit, controller manager 연동은 아직 구현하지 않았다. 다음 단계에서 SO-ARM101 URDF의 joint 이름·축·방향·limit을 현재 calibration과 대조한 뒤 `ros2_control` hardware backend로 확장한다.

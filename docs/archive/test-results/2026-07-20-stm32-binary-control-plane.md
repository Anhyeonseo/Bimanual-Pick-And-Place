# STM32 바이너리 제어 경로 검증 결과

- 시험 날짜: 2026-07-20
- 보드: NUCLEO-G474RE
- 펌웨어: `0x00020500`
- 통신 규격: protocol v1, COBS framing, CRC-32C
- 서보 bus: STS3215 actuator 6개, ID 1~6
- 보정 정보 hash: `0x3DB42B48`

## PC frame 변환 시험

- Python 통신 규격 단위 시험: 6/6 통과
- COBS 왕복 변환: 통과
- CRC-32C 알려진 시험값: 통과
- 손상된 frame 거부: 통과
- Byte stream에서 frame 경계 복구: 통과
- 메시지 manifest 일치: 통과

## 실제 하드웨어 연동 결과

아래는 초기 바이너리 기본 점검(smoke test) 출력이다. 이후 모듈화 회귀 시험에서는 펌웨어 `0x00020500`을 다시 확인했다.

~~~text
BINARY_SMOKE_OK
PROTOCOL_VERSION=1
JOINT_COUNT=6
FIRMWARE_VERSION=0x00020000
CALIBRATION_HASH=0x3DB42B48
CAPABILITIES=0x00000007
STOP_LATCHED=0
HEARTBEAT_COUNT=1
CRC_REJECT_COUNT=1
LAST_HEARTBEAT_MS=2303
SAFE_STOP_LATCH_CLEAR=OK
~~~

확인한 동작:

1. STM32와 상위 제어기가 protocol v1 frame을 정상적으로 주고받았다.
2. CRC를 일부러 손상시킨 frame은 모터를 움직이지 않고 거부했다.
3. 정상 heartbeat가 계수되는 것을 확인했다.
4. `SAFE_STOP`은 모터 이동 명령 없이 제어 명령 경로를 잠갔다.
5. `CLEAR_FAULT`는 6축 위치를 모두 읽고 설정된 raw 안전 범위를 통과한 뒤에만 잠금을 해제했다.
6. 일시적인 servo 읽기 실패가 발생하면 정지 상태를 유지하고, 안전 위치를 다시 읽는 데 성공한 뒤에만 복구했다.
7. 바이너리 모드에 들어간 뒤 ASCII 진단 출력을 막아 VCP에서 서로 다른 frame 형식이 섞이지 않게 했다.
8. Heartbeat가 500ms 동안 끊기면 정지 상태가 잠겼다. Heartbeat를 다시 보내고 6축 위치 검사를 통과해야 잠금을 해제할 수 있었다.

## 이후 확장 작업

- 여러 sample과 적용 시각을 담는 setpoint queue, 오래된 값과 queue 고갈 처리
- URDF 관절 좌표계에 맞춘 최종 관절 원점·방향 보정
- Raspberry Pi 전송 계층 통합과 재연결 방식

## 바이너리 setpoint 초기 구동 결과

- 6축 `+52155 urad` 명령을 한 번에 받아 동시에 실행했다.
- 양의 관절 목표는 raw `[2082, 2082, 2014, 2014, 2082, 2014]`로 변환됐다.
- 첫 바깥 방향 이동 시험의 최대 오차는 19 raw였다.
- 6축을 0rad 홈으로 복귀시킨 시험의 최대 오차는 5 raw였다.
- 동작 실행기는 20ms 주기로 비차단식으로 동작했고, 이동 중에도 상위 제어기의 heartbeat가 계속 처리됐다.

## 이동 중 SAFE_STOP과 복구

~~~text
PREFLIGHT_POSITIONS=[2052, 2050, 2051, 2045, 2053, 2044]
MOTION_ACCEPTED_STOP_SCHEDULED_400MS
SAFE_STOP_SENT_DURING_MOTION
BINARY_MOTION_SAFE_STOP_OK
RESET_THEN_RUN_HOME_RECOVERY

PREFLIGHT_POSITIONS=[2053, 2050, 2051, 2045, 2054, 2044]
MOTION_ACCEPTED MODE=home SAMPLES=1 STATE=3
BINARY_MOTION_OK MODE=home TARGET_URAD=[0, 0, 0, 0, 0, 0] MAX_ERROR_RAW=10
~~~

확인한 동작:

1. `SAFE_STOP`이 실행 중인 6축 trajectory를 완료 전에 중단했다.
2. 동작 실행기가 정지 상태를 보고하고 명령 경로의 잠금을 유지했다.
3. Reset 후 6축 모두 보정된 0rad 홈으로 돌아왔다.
4. 최종 복구 최대 오차는 10 raw였으며 servo 출력축 기준 약 0.88°다.

## 모듈화 후 회귀 시험

하나의 파일에 모여 있던 코드를 `servo_bus`, `binary_control`, `single_arm_app`, `single_arm_config`로 분리했다. CubeMX가 생성하는 `main.c`에는 HAL 초기화와 애플리케이션 시작·반복 호출만 남겼다.

~~~text
BINARY_SMOKE_OK
PROTOCOL_VERSION=1
JOINT_COUNT=6
FIRMWARE_VERSION=0x00020500
CALIBRATION_HASH=0x3DB42B48
CAPABILITIES=0x00000007
STOP_LATCHED=0
HEARTBEAT_COUNT=1
CRC_REJECT_COUNT=1

PREFLIGHT_POSITIONS=[2051, 2050, 2051, 2047, 2053, 2044]
MOTION_ACCEPTED_STOP_SCHEDULED_800MS TARGET_DEG=8
SAFE_STOP_SENT_DURING_MOTION
BINARY_MOTION_SAFE_STOP_OK

PREFLIGHT_POSITIONS=[2101, 2097, 2011, 2002, 2099, 1998]
MOTION_ACCEPTED MODE=home SAMPLES=1 STATE=3
BINARY_MOTION_OK MODE=home TARGET_URAD=[0, 0, 0, 0, 0, 0] MAX_ERROR_RAW=5
~~~

회귀 시험 결과, protocol 식별값과 보정 정보 hash가 유지됐다. 6축 이동, 이동 중 `SAFE_STOP`, 정지 후 홈 복구도 모듈 분리 전과 동일하게 정상 동작했다.

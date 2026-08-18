# STM32 액추에이터 제어기 (양팔, protocol v2)

이 디렉터리에는 특정 보드에 의존하지 않는 C11 공통 core가 들어 있다. 실제 서보에 전원을 넣기 전에 PC에서 통신 규격, 안전 상태 전환과 크기가 제한된 setpoint buffer를 시험할 수 있도록 STM32 HAL 및 RTOS 관련 코드와 분리했다. 이 core가 그대로 `firmware/stm32_g474_single_arm` 보드 프로젝트에 링크돼 배포된 양팔 resident firmware(F8.9 `0x00024809`)를 만든다 — 디렉터리 이름은 단일팔 시절 그대로지만 내용은 양팔이다.

## 구현된 공통 core

- COBS frame 구성과 CRC-32C 오류 검출
- protocol v2 frame 검증과 byte stream의 frame 경계 복구(`ACTUATOR_PROTOCOL_VERSION=2`)
- 이상이 생기면 안전한 쪽으로 닫히는 `BOOT → SAFE_DISABLED → ARMED → ACTIVE` 상태 머신
- heartbeat가 끊기면 `HOLD`로 전환
- 해제 명령 전까지 유지되는 `FAULT`와 물리 정지 상태 `ESTOPPED`
- 12관절(양팔) sample을 공통 `apply_tick`으로 원자 적용하는 stream queue(`stream_contract_v2`, `stream_session_v2`, `stream_executor_v2`) — 자세한 wire 계약은 [`protocol/README.md` §6](../../protocol/README.md#6-stream-계약-v2--stream_open--setpoint_batchsplice)
- 좌우 6축을 각각 독립 servo bus로 라우팅하는 `bimanual_dispatch`/`bimanual_goal_map`
- shoulder unwrap(`joint_unwrap`)과 operational-limit 검사
- `protocol/message_ids.json`에서 생성하는 message ID(session/state_control/feedback). v2 stream 메시지(40~47, 58~62)는 `stream_contract_v2.h`에 직접 정의돼 있고 이 생성기 대상이 아니다 — [`protocol/README.md` §2.1](../../protocol/README.md#21-메시지-id) 참고

이 코드는 CubeIDE 보드 프로젝트(`firmware/stm32_g474_single_arm`)에 외부 파일(linked resource)로 연결되며 PC에서도 별도로 시험한다. STM32 시작 코드, HAL, STS3215 bus 접근과 flash 설정, 좌우 servo bus 인스턴스(`servo_bus.c`/`right_servo_bus.c`), 5 ms 제어 tick(`control_tick`, TIM6 ISR)은 보드 프로젝트에 남긴다. 관절 단위와 서보 raw 값 사이의 보정(calibration)은 이 공통 core가 담당한다.

legacy `setpoint_queue`/`buffered_command_route`(6축, 우팔 슬롯 미지원)도 이 디렉터리에 여전히 있다 — 초기 단일팔 구동 경로였고 resident v2 stream이 대체했다. 새 코드에서 참조하지 않는다.

## PC에서 빌드

~~~powershell
cmake -S firmware/stm32_actuator -B build/stm32_actuator-host
cmake --build build/stm32_actuator-host --config Debug
ctest --test-dir build/stm32_actuator-host -C Debug --output-on-failure
~~~

Windows에서 CMake가 `PATH`에 없다면 Visual Studio Developer PowerShell에서 실행한다.

## 보드와 공통 core의 경계

- 상위 제어기/Pi 연결: NUCLEO 기본 `LPUART1` 경로(`PA2/PA3`)의 STLINK-V3E VCP
- 좌팔 servo bus: `USART1`(`PC4/PC5`). 우팔 servo bus: `UART4`(`PC10/PC11`) — 독립된 두 bus
- 제어 주기: resident 경로는 TIM6 하드웨어 타이머 ISR이 5 ms마다 갱신. legacy `buffered_executor`(20 ms 비차단 실행기)는 남아있지만 배포 firmware가 쓰지 않는다
- UART 수신: frame 검증과 상태 처리는 `binary_control` 모듈에서 수행
- 서보 송수신: 명시적인 timeout을 두고 한 번에 처리량이 제한된 통신 사용

배선하거나 CubeMX 설정을 다시 생성하기 전에는 실제 NUCLEO 보드 revision과 pin 경로를 확인해야 한다.

## 자동으로 생성되는 protocol header

~~~powershell
python tools/setup/firmware/generate_protocol_header.py
python tools/setup/firmware/generate_protocol_header.py --check
~~~

저장소에 포함된 header는 사람이 직접 수정하는 문서가 아니다. `protocol/message_ids.json`에 있는 기계 판독용 정의와 항상 일치해야 한다.

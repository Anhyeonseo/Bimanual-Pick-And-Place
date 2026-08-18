# 단계 5 STM32 READ_ONLY 실기 결과

- 날짜: 2026-07-25
- 판정: **PARTIAL — 통신·identity·feedback PASS, strict position gate 미통과**
- 대상: 왼팔 SO-ARM101, Raspberry Pi 5 `pi5-chess`
- Pi: aarch64, ROS 2 Jazzy
- serial: `/dev/ttyACM0`, ST-LINK by-id 후보 정확히 1개
- bridge: `allow_motion=false`, `feedback_rate_hz=5.0`
- 실제 motion command: **0회**

## 환경과 identity

- firmware: `0x00020700`
- protocol: version 1, 6 joints
- calibration: `0x3DB42B48`
- ROS joint contract:
  1. `left_base_joint`
  2. `left_shoulder_joint`
  3. `left_elbow_joint`
  4. `left_wrist_flex_joint`
  5. `left_wrist_roll_joint`
  6. `left_gripper_joint`

최초 READ_ONLY 연결은 identity 검증 뒤 `GET_STATE status=2`로 중단됐다.
이는 setpoint나 motion 명령 실패가 아니라 6축 position read 실패였다.
bridge를 종료하고 servo power와 보드 상태를 재확인한 뒤, motion/ARM/ENABLE을
보내지 않는 ASCII `S` 진단에서 6축이 모두 응답했다.

## 최종 READ_ONLY 측정

| joint | raw | strict raw | 판정 |
|---|---:|---:|---|
| BASE | 2070 | 2048..2389 | strict 안 |
| SHOULDER | 2043 | 2048..2162 | strict 밖, recovery 안 |
| ELBOW | 2041 | 1934..2048 | strict 안 |
| WRIST_FLEX | 2071 | 1934..2048 | strict 밖, recovery 안 |
| WRIST_ROLL | 2080 | 2048..2219 | strict 안 |
| GRIPPER | 1965 | 1934..2048 | strict 안 |

모든 축은 firmware clear-stop 조건인 `strict range ±40 raw` 안이다.
하지만 SHOULDER와 WRIST_FLEX는 strict command range 밖이므로 Gate 5D 전체
PASS로 판정하지 않는다. 수동 미세 조정은 관절 coupling으로 다른 축까지
움직였으므로 추가 조정을 중단했다.

ROS `/joint_states` 한 sample:

```text
position:
- 0.0337475773334841
- -0.007669903939428206
- 0.010737865515199488
- -0.035281558121369745
- 0.047553404424454875
- 0.12732040539450823
```

- 60초 관측 rate: 평균 `5.000 Hz`
- sample interval: `0.199..0.201 s`
- sample count: 318개 이상
- motion Action: 0개 — READ_ONLY 의도와 일치
- 비정상 소음·진동·의도하지 않은 움직임: 없음

bridge 종료 뒤 `HELLO`, `GET_STATE`, `HEARTBEAT`만 호출하는
`tools/stm32_read_only_counters.py`로 별도 60초 counter 진단을 수행했다.

```text
HELLO protocol=1 joints=6 firmware=0x00020700 calibration=0x3DB42B48 stop_latched=0
START status=0 stop_latched=0 heartbeat=0 rejected=0
END status=0 stop_latched=0 heartbeat=599 rejected=0
DELTA heartbeat=599 rejected=0 heartbeat_increased=True rejected_delta_zero=True
```

따라서 heartbeat 증가, rejected-frame delta 0과 stop latch 0은 PASS다.

## 경계 복구 software gate

실측 경계 상태에서 수동 raw 맞춤 대신 다음 단방향 복구 정책을 host에
추가했다.

- 현재 feedback은 firmware와 같은 `strict ±40 raw` 범위만 복구 후보로 인정
- 현재 joint 하나라도 strict 밖이면 일반 arm goal 전부 거부
- all-zero q0 arm goal만 허용
- q0 recovery duration은 정확히 `2000 ms`만 허용
- gripper goal은 arm feedback이 strict 안으로 복귀할 때까지 거부
- 실제 전송 target은 기존 strict range를 계속 적용하며 clamp 없음
- recovery 범위를 1 raw라도 벗어나면 goal reject

workstation fake transport 검증:

- 일반 arm goal: reject, transport call 0회
- q0 1000 ms: reject, transport call 0회
- q0 2000 ms: accept, transport call 정확히 1회
- firmware recovery 경계 밖 feedback: reject
- arm recovery 전 gripper goal: reject, transport call 0회
- 전체 test: **109/109 PASS**
- ROS package build: **5/5 PASS**

이 결과는 실제 motion 증거가 아니다. 수정 source를 Pi에 배포하고
`allow_motion=false` READ_ONLY를 다시 확인한 뒤, 사용자가 작업 공간·무부하·
전원 차단 수단을 확인하고 별도 motion 승인을 해야 q0 복귀를 실행할 수 있다.

## q0 recovery 실제 Action

별도 사용자 승인 문구를 받은 뒤 왼팔 5축 all-zero q0, `2000 ms` goal을
정확히 1회 실행했다.

- Action ID: `3c635b0591cb441680e1d0a6edef6a62`
- goal acceptance: PASS
- terminal state: `SUCCEEDED`
- result: `error_code=0`, final error tolerance 안
- 관찰: 소각도 이동, 약 2초 뒤 정지, gripper 유지
- 비정상 소음·진동·잘못된 방향: 없음
- 종료 후 command process와 serial owner: 0개

최종 READ_ONLY raw 환산값은 다음과 같다.

| joint | raw | q0 error raw | strict 판정 |
|---|---:|---:|---|
| BASE | 2051 | 3 | 안 |
| SHOULDER | 2043 | 5 | 밖 |
| ELBOW | 2051 | 3 | 밖 |
| WRIST_FLEX | 2057 | 9 | 밖 |
| WRIST_ROLL | 2053 | 5 | 안 |
| GRIPPER | 1965 | 유지 | 안 |

q0가 one-sided strict range의 끝점이므로 정상 정지 오차가 반대편에 남았다.
최대 9 raw는 기존 Action 성공 허용치 `20 raw` 안이다. 반복 q0는 실행하지
않았다. host gate를 다음처럼 분리했다.

- strict 밖 `1..20 raw`: 완료 잔차로 인정, 다음 arm target은 strict 필수
- strict 밖 `21..40 raw`: all-zero q0, `2000 ms`만 허용
- strict 밖 `40 raw` 초과: reject
- gripper goal: 보존할 arm position이 strict 밖이면 계속 reject
- clamp: 없음

실측 최종 raw를 사용하는 fake transport 회귀를 추가했고 전체
**109/109 PASS**, ROS package **5/5 build PASS**다. 실제 후속 motion은
새 사용자 승인 전까지 허용하지 않는다.

## BASE 단일 관절 소각도 실제 Action

final-error 잔차 gate를 Pi에 배포하고 READ_ONLY 회귀를 통과한 뒤, 별도 사용자
승인 아래 BASE `+0.05 rad`, `2000 ms` goal을 정확히 1회 실행했다.

- Action ID: `723144efc5d343678dfd253bc62ed1d9`
- terminal state: `SUCCEEDED`
- BASE raw: `2051 → 2077`
- 목표 raw: 약 `2081`
- BASE 최종 오차: `4 raw`
- SHOULDER: `2043 → 2043`
- ELBOW: `2051 → 2051`
- WRIST_FLEX: `2057 → 2057`
- WRIST_ROLL: `2053 → 2053`
- GRIPPER: `1965 → 1965`
- 비정상 소음·진동·잘못된 방향: 없음
- 종료 후 command process와 serial owner: 0개

따라서 strict target 전송, final-error 잔차 수락과 비명령 관절 보존을 실제
hardware에서 확인했다.

별도 재승인 아래 BASE q0 `2000 ms` 복귀를 정확히 1회 실행했다. Action ID는
`2336387893604acfaf66300cfe260f1d`, terminal state는 `SUCCEEDED`였다.
최종 raw는 `2051, 2043, 2052, 2057, 2053, 1965`이고 최대 q0 오차는
9 raw였다. 물리 복귀와 gripper 유지가 정상이었고 종료 후 process와 serial
owner는 0개였다. BASE 왕복 시험은 PASS다.

## 팔 5축 동시 소각도 실제 Action

별도 사용자 승인 아래 팔 5축 각각 `+0.05 rad`, `2000 ms` goal을 정확히
1회 실행했다.

- terminal state: `SUCCEEDED`
- BASE: raw `2077`, target error 4
- SHOULDER: raw `2069`, target error 12
- ELBOW: raw `2033`, target error 18
- WRIST_FLEX: raw `2025`, target error 10
- WRIST_ROLL: raw `2075`, target error 6
- GRIPPER: raw `1965`, 위치 유지
- 팔 5축 strict range: 모두 안
- 비정상 소음·진동·잘못된 방향: 없음

motion 직후 `timeout waiting for STATE_FEEDBACK` 경고가 2회 있었으나 각각
`transient feedback delay (1/3)`에서 다음 sample로 회복됐고 fault, stop
latch 또는 Action 실패로 이어지지 않았다. 이후 READ_ONLY 60초 측정은
평균 `5.000 Hz`, interval `0.199..0.201 s`, 313개 이상 sample과 경고
0회였다. 지속 통신 장애로 판정하지 않되 이후 motion에서도 재발을 감시한다.

현재 팔 5축이 모두 strict 안이므로 gripper adapter가 이 값을 보존해 strict
target으로 전송할 수 있다. q0 복귀를 먼저 하면 one-sided boundary 잔차 때문에
gripper goal이 다시 차단될 수 있어, 다음 승인 항목은 gripper safe subrange로
순서를 조정한다.

## Gripper safe subrange 실제 Action

팔 5축이 모두 strict 안인 상태에서 별도 사용자 승인 아래 gripper
`0.08 rad`, adapter 고정 `1000 ms` goal을 정확히 1회 실행했다.

- terminal state: `SUCCEEDED`
- result position: `0.0874369 rad`
- result raw: 약 `1991`
- target raw: 약 `1996`
- final error: 약 `5 raw`
- `stalled=false`
- `reached_goal=true`
- 팔 5축 움직임: 없음
- 비정상 소음·걸림: 없음

completion 직후 `transient feedback delay (1/3)`이 다시 1회 발생했다.
Action과 물리 동작은 성공했지만 같은 시점의 재현이므로 후속 motion을
중단했다. 원인은 pending `GET_STATE`가 terminal `SETPOINT_STATUS`를 먼저
수집한 뒤 원래 STATE 응답을 기다릴 시간이 남지 않는 response ordering으로
분석했다.

첫 수정은 terminal result 뒤 기존 STATE 응답을 같은 timeout만큼 더 기다렸다.
Pi 배포와 READ_ONLY 재검증 뒤 gripper `0.13 rad`, `1000 ms` 복귀를 1회
실행했고 `SUCCEEDED`, result position `0.1196505 rad`, `stalled=false`,
`reached_goal=true`, 팔 5축 유지와 비정상 동작 없음이었다. 그러나 completion
직후 같은 `transient feedback delay (1/3)`가 재현되어 첫 수정은 FAIL이다.

실제 패킷 순서를 다시 분석한 결과, terminal status와 겹친 기존 `GET_STATE`
응답 자체가 누락될 수 있으므로 단순 대기는 충분하지 않다. `GET_STATE`를 fresh
sequence로 1회 즉시 재요청하는 두 번째 수정도 Pi에 배포했다. READ_ONLY PASS 뒤
별도 승인 아래 gripper `0.08 rad`, `1000 ms`를 실행했고 Action ID
`0e061e7216384339a0dfa07ecdede639`, result `0.0874369 rad`, `SUCCEEDED`,
팔 5축 유지와 비정상 동작 없음이었다. 그러나 같은 completion 경고가 다시
발생해 즉시 재요청 역시 final verification busy window와 겹치는 것으로 판정했다.

세 번째 수정은 terminal result와 겹친 feedback cycle만 연기했지만 원인 가정이
틀렸다. Pi 배포와 READ_ONLY PASS 뒤 별도 승인 아래 gripper `0.13 rad`,
`1000 ms`를 실행했다. Action ID `24c7d6c327d5439b8e7c1176acc8193a`, result
`0.1227185 rad`, `SUCCEEDED`, 팔 5축 유지와 비정상 동작 없음이었다. 경고 시각
`1784916484.432`는 완료 시각 `1784916485.357`보다 약 0.93초 빨라 active
trajectory 도중 발생했다. completion ordering이 아니라 STM32가 20 ms trajectory
control과 servo safety 작업을 수행하는 동안 host의 position read가 겹친 것이
원인이다.

네 번째 수정은 execution core가 active인 동안 `GET_STATE`를 보내지 않는다.
Action poll은 UART buffer에 이미 도착한 unsolicited terminal result만 수집한다.
완료 뒤 첫 정규 5 Hz cycle부터 physical position feedback을 재개한다. 일반
STATE timeout 경고·fault 기준과 heartbeat는 유지하며 motion goal 재전송은 없다.
GET_STATE 0회 상태의 unsolicited completion 수집 회귀를 추가했고 전체
**111/111 PASS**, ROS package **5/5 build PASS**다. Pi 재배포 뒤
READ_ONLY를 다시 통과했고, 별도 승인 아래 gripper `0.08 rad`,
`1000 ms`를 정확히 1회 실행했다. 사용자 관찰상 그리퍼만 정상 동작했고 팔
5축·소음·진동에 이상이 없었다. bridge의 동작 중·완료 직후 WARN/ERROR는
0회였다. 완료 후 `/joint_states --once`는 6축을 정상 출력했으며 gripper
`0.0874369 rad`, 팔 5축 유지가 확인됐다. active trajectory feedback pause와
완료 후 자동 복귀 검증은 **PASS**다.

## Action cancel, SAFE_STOP과 명시적 recovery

SAFE_STOP acknowledgement와 terminal result가 교차하는 회귀와 one-shot cancel
도구를 배포했다. 별도 승인 아래 gripper `0.13 rad`, cancel delay `300 ms`를
정확히 1회 실행했다.

- cancel request: accepted
- Action result: status `5` (`CANCELED`)
- `reached_goal=0`, `stalled=0`
- script verdict: `CANCEL_TEST_PASS SAFE_STOP_EXPECTED_LATCHED`
- bridge: `safety latch error: STM32 stop is latched` 확인
- `STATE_FEEDBACK timeout`: 0회
- 팔 5축 이상 동작: 없음

별도 승인으로 `/clear_fault`를 정확히 1회 호출했고 bridge는
`fault cleared; commands enabled`를 출력했다. 이전 goal 재개나 물리 동작은
없었다. 이후 `/joint_states --once`는 6축을 정상 출력했고 gripper
`0.0874369 rad`, 팔 5축 유지가 확인됐다. 안전 종료와 servo power OFF까지
완료했다. cancel, SAFE_STOP latch와 명시적 recovery는 **PASS**다.

## Process reconnect와 stale goal 방지

cancel과 명시적 recovery 시험 뒤 bridge를 종료하고 servo power를 OFF했다.
다음 실제 시험에서 servo power를 다시 켜고 Pi bridge를
`allow_motion:=true`로 시작한 뒤 5초간 새 goal을 보내지 않았다.

- 연결 identity: firmware `0x00020700`, calibration `0x3DB42B48`
- mode: `MOTION_ENABLED`
- 이전 cancel goal 재개·재전송: 0회
- 명령하지 않은 arm/gripper 동작: 0회
- bridge WARN/ERROR: 0회
- `/single_arm_bridge`: 정확히 1개

process reconnect 뒤 stale goal 미재개는 **PASS**다.

## MoveIt → Pi bridge → STM32 end-to-end

워크스테이션과 Pi는 ROS domain `30`, Cyclone DDS를 사용했다. Pi만 serial과
`single_arm_bridge`를 소유하고 워크스테이션은
`external_stm32_moveit.launch.py`로 RSP, MoveGroup과 RViz만 실행했다.
워크스테이션에 두 번째 hardware provider는 없었다.

확인된 Action은 다음과 같다.

- `/execute_trajectory` — `moveit_msgs/action/ExecuteTrajectory`
- `/left_arm_controller/follow_joint_trajectory` —
  `control_msgs/action/FollowJointTrajectory`
- `/left_gripper_controller/gripper_cmd` —
  `control_msgs/action/ParallelGripperCommand`

초기 B안의 single-point 제한을 유지하기 위해 실제 명령은
`tools/ros_moveit_execute_once.py`가 MoveIt `/execute_trajectory`에 정확히 한
point만 보냈다. 일반 OMPL multi-point `Plan & Execute`는 이 milestone의 지원
범위가 아니다. 외부 STM32 전용 MoveGroup의 start tolerance는 `0.20 rad`이며,
실제 target은 Action adapter의 더 엄격한 calibration 범위에서 다시 검증된다.

### 실제 home

- arm target: 5축 모두 `0.00 rad`, `2000 ms`
- MoveIt Action: accepted
- terminal status: `4` (`SUCCEEDED`)
- MoveIt error code: `1` (`SUCCESS`)
- 물리 q0 방향 이동과 정지: 정상
- gripper 움직임: 없음
- Pi WARN/ERROR: 0회

### 실제 representative arm

- arm target: 5축 모두 `0.05 rad`, `2000 ms`
- terminal status/error: `4` / `1`
- final feedback rad:
  `0.0444854, 0.0322136, 0.0230097, 0.0352816, 0.0414175`
- final error raw: BASE `4`, SHOULDER `12`, ELBOW `18`,
  WRIST_FLEX `10`, WRIST_ROLL `6`
- 모든 축 허용치 `20 raw` 안
- gripper `0.0874369 rad` 유지
- 잘못된 방향·소음·걸림과 Pi WARN/ERROR: 0회

### 실제 hardware-safe gripper

- gripper target: `0.08 rad`, `1000 ms`
- terminal status/error: `4` / `1`
- final gripper feedback: `0.0874369 rad`
- arm 5축 위치 보존
- Pi WARN/ERROR: 0회

### 육안 확인용 실제 arm

- arm target: 5축 모두 `0.10 rad`, `2000 ms`
- calibrated raw target: `2113, 2113, 1983, 1983, 2113`
- terminal status/error: `4` / `1`
- final feedback rad:
  `0.0905049, 0.0859029, 0.0766990, 0.0828350, 0.0905049`
- final error raw: BASE `6`, SHOULDER `9`, ELBOW `15`,
  WRIST_FLEX `11`, WRIST_ROLL `6`
- 움직임 육안 확인, 전 축 `20 raw` 안, gripper 유지

마지막으로 MoveIt home을 정확히 1회 실행해 q0로 원복했다. MoveIt/RViz,
Pi bridge 순서로 종료하고 servo power를 OFF했으며 원복·종료 이상은 없었다.

## 최종 판정

- 단계 5 초기 B안 single-point hardware milestone: **PASS**
- 단계 5 진행률: **100%**
- 전체 Python 회귀: **116/116 PASS**
- ROS workspace build: **6 packages PASS**
- 다음 단계: 단계 6 Top 카메라 인식

multi-point planning execution, preempt와 ros2_control hardware interface 전환은
검증 범위를 넓히는 후속 작업이며 이번 완료 판정에 포함하지 않는다.

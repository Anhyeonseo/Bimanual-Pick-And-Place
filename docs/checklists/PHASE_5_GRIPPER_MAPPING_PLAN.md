# 단계 5 — 왼팔 gripper mapping 측정 계획

- 상태: **APPROVED — 계획 승인, motion 미승인**
- 작성일: 2026-07-24
- 승인일: 2026-07-24
- 대상: 정상인 왼팔 `left_gripper_joint`
- 실행 host: STM32가 연결된 Raspberry Pi 5 / Ubuntu 24.04 / ROS 2 Jazzy
- 현재 motion 허용: **아니오**

이 문서는 측정 계획만 정의한다. 이 계획을 승인하는 것은 실제 servo motion
승인과 다르다. motion은 단계 5 READ_ONLY gate가 PASS하고 사용자가 그
시험을 별도로 명시 승인한 뒤에만 수행한다.

## 해결할 문제

현재 세 계층이 같은 `left_gripper_joint` 이름을 사용하지만 검증 범위가
다르다.

| 계층 | q=0 | +q | 현재 범위 |
|---|---|---|---|
| URDF/MoveIt | 거의 닫힌 moving jaw pose | jaw 열림 | `0..1.91986 rad` |
| Isaac | `-10 deg` drive pose | jaw 열림 | project `0..1.91986 rad` |
| STM32/servo | raw `2048` | raw 감소 | project 약 `0..0.17487 rad` |

URDF의 joint는 `gripper_link`에 대한 `moving_jaw_link`의 revolute angle이다.
현재 host와 firmware는 이 project angle을 servo shaft delta와 1:1로
변환한다.

```text
q_project = (2048 - raw) * 2π / 4096
```

이 1:1 가정을 그대로 적용하면 MoveIt full-open `1.91986 rad`은 이론상 raw
약 `796`이다. 이는 firmware verified range `1934..2048` 밖이므로 명령하지
않는다. 이번 측정은 기존 verified range 안에서만 1:1 가정, 열림 방향과
jaw gap 변화를 확인하며 safe range를 확장하지 않는다.

## 승인된 명령 제한 원칙

실제 STM32 backend에서는 다음 세 계층이 모두 verified safe range를
적용한다.

```text
MoveIt hardware 전용 planning limit
→ single_arm_bridge Action adapter range reject
→ STM32 firmware raw safe limit
```

- `backend:=stm32`일 때만 hardware 전용 MoveIt limit overlay를 적용한다.
- mock과 Isaac은 URDF의 simulation 범위 `0..1.91986 rad`을 유지한다.
- STM32 overlay의 초기 상한은 현재 verified 값 `0.174873810 rad`이며 실제
  측정 결과 없이 늘리지 않는다.
- MoveIt의 기존 `open=1.91986 rad` named state는 STM32 mode에서 범위 밖으로
  판정되어 실행되지 않아야 한다.
- 측정 후 `hardware_safe_open` named state를 별도로 정의한다.
- MoveIt 검사를 우회한 직접 Action goal도 adapter와 firmware에서 거부한다.

## 측정 전에 충족할 gate

다음 중 하나라도 충족하지 않으면 motion 측정을 시작하지 않는다.

- STM32 serial 후보가 정확히 하나
- STM32 backend 외 command backend가 모두 종료됨
- `allow_motion=false` READ_ONLY에서 firmware `0x00020700`, protocol v1,
  calibration hash `0x3DB42B48` 확인
- 6개 joint feedback과 gripper raw 확인
- gripper가 physical q0에 있고 raw가 `2048` 근처인지 사용자 확인
- fault와 stop 상태 확인
- 작업 공간을 비우고 gripper에 물체를 넣지 않음
- servo 전원을 즉시 차단할 수 있는 수단이 손이 닿는 곳에 있음
- arm은 안정된 지지 상태이며 gripper 외 joint의 이동 계획이 없음
- 사용자가 제한 motion을 별도로 명시 승인

physical E-stop이 없으므로 즉시 사용할 수 있는 전원 차단 수단이 없으면
READ_ONLY 기준점만 기록하고 motion 측정은 보류한다.

## 측정 도구와 기준점

- jaw gap은 두 jaw의 동일한 평행 면 사이를 mm로 측정한다.
- 동일한 자 또는 caliper와 동일한 접촉 위치를 모든 sample에 사용한다.
- 사진 제출은 요구하지 않는다. 사용자가 실제 상태를 판단하고 수치와
  관찰 결과를 기록한다.
- torque가 활성화된 동안 jaw를 손으로 밀거나 억지로 벌리지 않는다.

## M0 — READ_ONLY 기준점

motion command 없이 다음을 기록한다.

| 항목 | 기록값 |
|---|---|
| firmware / protocol / calibration hash | 실행 시 기록 |
| feedback gripper raw | 실행 시 기록 |
| feedback `left_gripper_joint` rad | 실행 시 기록 |
| q0 jaw gap mm | 사용자가 측정 |
| jaw contact 여부 | 접촉 / 거의 닫힘 / 간격 있음 |
| feedback baseline noise | 일정 시간 min..max |

M0는 mapping을 확정하지 않는다. q0와 측정 기준을 고정하는 단계다.

## M1 — 기존 safe range 안의 제한 motion

M0가 PASS한 뒤에도 사용자의 별도 motion 승인을 받기 전에는 실행하지
않는다. 다른 다섯 arm joint는 q=0으로 덮어쓰지 않고 그 시점의 실제
feedback 위치를 hold한다.

각 sample은 한 번에 하나씩, 무부하, 낮은 속도, 충분한 실행 시간으로
보낸다. 한 sample의 feedback과 최종 상태를 확인하기 전에 다음 sample로
넘어가지 않는다.

| 순서 | target raw | 예상 project q | servo delta | 용도 |
|---:|---:|---:|---:|---|
| 0 | 2048 | `0.000000000 rad` | `0.00000 deg` | 기준점 |
| 1 | 2020 | `0.042951462 rad` | `2.46094 deg` | 최초 방향 확인 |
| 2 | 1991 | `0.087436905 rad` | `5.00977 deg` | 중간점 |
| 3 | 1963 | `0.130388367 rad` | `7.47070 deg` | 중간점 |
| 4 | 1934 | `0.174873810 rad` | `10.01953 deg` | 현재 safe 끝점 |

상승 방향이 모두 PASS한 경우에만 같은 점을 역순으로 거쳐 raw `2048`로
돌아오며 hysteresis를 기록한다. raw `1934`보다 작은 값은 사용하지 않는다.

각 sample에서 다음을 기록한다.

- target raw와 project q
- 시작/종료 feedback raw와 project q
- jaw gap mm
- +q에서 실제로 열렸는지
- motion completion status와 final error raw
- fault, stop, load/current protection event 유무
- 다른 다섯 arm joint의 최대 feedback drift
- 소음, 걸림, 진동과 물리 접촉에 대한 사용자 판정

## 즉시 중단 조건

다음 중 하나가 발생하면 다음 sample을 보내지 않는다.

- +q에서 jaw가 닫히거나 예상과 반대로 움직임
- gripper가 아닌 arm joint가 움직임
- feedback 중단 또는 raw 불일치
- raw가 `1934..2048` 밖으로 나감
- 걸림, 충돌, 비정상 소음 또는 진동
- fault, stop latch 또는 load/current protection event
- command cancel/timeout 또는 final failure

중단 후 자동 recovery나 goal 재전송을 하지 않는다. `SAFE_STOP` 확인과 원인
검토 전에는 `/clear_fault`도 호출하지 않는다.

## 데이터 판정

최소 다섯 점의 상승 데이터와 안전하게 가능할 경우 하강 데이터를 비교한다.

1. +q가 jaw open과 일치하는지 판정
2. servo delta와 URDF moving-jaw angle이 1:1인지 판정
3. jaw gap과 q의 관계가 선형인지, monotonic table이 필요한지 판정
4. 상승/하강 jaw gap 차이로 hysteresis 기록
5. physical q0가 MoveIt `closed=0`으로 사용 가능한지 판정
6. 현재 safe 끝점에서 실제 Pick and Place에 충분한 gap인지 판정

baseline noise와 측정 도구 오차를 먼저 기록한 뒤 허용 오차를 정한다. 실제
데이터 없이 1:1, 선형 또는 zero-offset mapping을 미리 채택하지 않는다.

## 측정 후 가능한 결정

### 1:1 mapping이 확인된 경우

- hardware backend는 project radian을 그대로 사용한다.
- firmware verified limit `0..0.17487 rad`은 유지한다.
- full-open 범위 확장은 별도 mechanical range 시험으로 분리한다.
- 필요하면 MoveIt에 hardware-safe named state를 추가하되 simulation의 물리
  모델 범위를 임의로 축소하지 않는다.

### scale, offset 또는 비선형 mapping이 필요한 경우

- mapping은 STM32 backend 아래 calibration 계층에 둔다.
- MoveIt, task node와 policy에는 raw 또는 linkage 계산을 노출하지 않는다.
- mapping table/version과 firmware safe limit을 구분한다.
- host/firmware calibration hash 변경은 별도 승인과 regression 후 수행한다.

### 유효한 mapping을 만들 수 없는 경우

- 실제 gripper Action 구현을 차단 상태로 유지한다.
- arm 5-DOF Action 검증과 gripper 검증을 분리한다.
- 구조 또는 기구 측정 없이 URDF, firmware limit이나 zero를 변경하지 않는다.

## 산출물과 승인

측정 결과는 다음 형식으로 별도 기록한다.

```text
docs/archive/test-results/YYYY-MM-DD-left-gripper-mapping.md
```

결과에는 repository commit, Pi 환경, firmware/protocol/calibration version,
모든 sample, 중단 여부와 최종 mapping 결정을 포함한다. 측정 전에는 URDF,
SRDF, Isaac mapping, firmware safe limit과 calibration hash를 변경하지 않는다.

이 계획은 2026-07-24 사용자가 승인했다. 단계 5 Gate 5A의
`gripper mapping 측정 계획 승인`은 PASS다. 이 승인은 실제 motion을
허용하지 않는다.

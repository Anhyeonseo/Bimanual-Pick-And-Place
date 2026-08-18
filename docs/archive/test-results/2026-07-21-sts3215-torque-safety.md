# STS3215 축별 torque 및 동작 보호 실기 결과

## 검증 대상

- 보드: NUCLEO-G474RE
- 서보: STS3215 12V, ID 1~6
- 펌웨어: `0x00020600`
- 전원: 12V 10A adapter
- 목적: SO-ARM101에서 하중이 큰 SHOULDER, ELBOW, WRIST_FLEX의 torque를 조정하고 동작 중 과부하 감시를 확인한다.

## 적용 기준

| 축 | ID | torque limit |
|---|---:|---:|
| BASE | 1 | 400 |
| SHOULDER | 2 | 650 |
| ELBOW | 3 | 550 |
| WRIST_FLEX | 4 | 400 |
| WRIST_ROLL | 5 | 250 |
| GRIPPER | 6 | 150 |

- 100ms마다 선택된 축의 load와 current를 읽는다.
- load magnitude 800 또는 current raw 320 이상이 2회 연속 측정되면 동작을 중단하고 stop latch를 건다.
- current raw 1은 약 6.5mA로 계산한다.
- 보호 중단 시 마지막 명령 위치를 유지하며 자동 torque-off는 하지 않는다. 갑작스러운 낙하를 막기 위한 선택이다.

## 실기 결과

| 축 | 동작 결과 | 최종/복귀 오차 | 정지 load | 정지 current | 판정 |
|---|---|---:|---:|---:|---|
| ELBOW(ID3) | raw 2197에서 home 2048로 4400ms 이동 | home 직후 20 raw, 안정 후 16 raw | 106 | 5 raw, 약 32mA | 통과 |
| SHOULDER(ID2) | raw 2043→2082→2048 | 7 raw / 3 raw | magnitude 40 | 1 raw, 약 6mA | 통과 |
| WRIST_FLEX(ID4) | raw 2055→2014→2048 | 11 raw / 1 raw | 56 | 1 raw, 약 6mA | 통과 |

세 축 모두 갑작스러운 torque release나 safety abort 없이 이동하고 복귀했다. 전압은 12.2~12.5V, 최고 온도는 35°C였다.

## 판정과 제한

현재 홈 주변의 무부하 단일 팔 시험에서는 설정한 torque가 충분하고 보호 telemetry가 정상 동작한다. 이 결과는 팔을 길게 편 자세, 물체를 든 자세 또는 장시간 반복 동작까지 보장하지 않는다. Pick and Place payload가 정해지면 동일한 항목을 최악 자세에서 다시 측정한다.

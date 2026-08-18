# 카메라 decode·DDS 부하와 STM32 제어 격리 결과

- 날짜: 2026-07-21
- 대상: Raspberry Pi 5 4GB, USB MJPEG 카메라 3대, NUCLEO-G474RE
- ROS 2: Jazzy
- 시험 시간: warm-up 10초 + 측정 120초
- 카메라 phase: `DUAL_PRIVATE`
- STM32 bridge: `READ_ONLY`, feedback 5Hz, heartbeat 10Hz

## 시험 목적

세 카메라가 RGB 영상을 동시에 디코딩하고 ROS 2 DDS로 전달하는 동안에도 STM32 heartbeat와 joint feedback이 밀리지 않는지 확인한다. 동시에 Pi 전체 CPU, memory, 온도와 swap 사용을 기록한다.

단순히 image publisher만 실행하면 subscriber가 없어 DDS 전달 비용이 빠질 수 있다. `tools/camera_control_load_test.py`가 세 `sensor_msgs/Image` topic을 실제로 구독하고, 전달된 image memory를 읽도록 하여 다음 부하를 포함했다.

- MJPEG capture와 최신 frame 교체
- 선택적 JPEG RGB decode
- `sensor_msgs/Image` 생성
- local DDS 전달과 Python 역직렬화
- 수신 image memory 접근
- STM32 binary heartbeat와 position feedback

실제 object detection inference와 MoveIt planning 부하는 포함하지 않았다.

## 카메라와 DDS 결과

| 카메라 | 목표/실측 decode | RGB DDS payload | decode 실패 | frame age p95 | decode p95 |
|---|---:|---:|---:|---:|---:|
| Top | 6.00/6.00Hz | 44.23Mbps | 0 | 31.07ms | 2.99ms |
| Wrist A | 5.00/4.90Hz | 36.12Mbps | 0 | 32.44ms | 2.83ms |
| Wrist B | 5.00/4.93Hz | 36.37Mbps | 0 | 27.81ms | 4.53ms |

세 RGB topic의 application payload 합계는 약 116.72Mbps다. 이는 USB MJPEG 대역폭이 아니라 decode 뒤 ROS message data 크기를 기준으로 계산한 local DDS payload다.

모든 카메라가 `STREAMING`을 유지했고 decode 실패가 없었다. frame age p95 200ms와 decode p95 50ms 기준도 충족했다.

## STM32 제어 격리 결과

부하 전 `/joint_states` 기준선은 5.000Hz였으며 120초 부하 중 결과는 다음과 같다.

| 지표 | 결과 |
|---|---:|
| 수신 joint state | 601개 |
| 실측 rate | 5.008Hz |
| feedback 간격 p95 | 200.52ms |
| feedback 최대 간격 | 201.30ms |
| bridge heartbeat/feedback 오류 | 0회 |
| 시험 후 `STOP_LATCHED` | 0 |
| 시험 후 heartbeat count | 3,737 |

시험 후 protocol smoke도 통과했다.

```text
BINARY_SMOKE_OK
STOP_LATCHED=0
HEARTBEAT_COUNT=3737
RAW_POSITION_FEEDBACK=[2051, 2019, 2007, 2075, 2079, 1965]
```

따라서 실제 RGB decode·DDS 소비 부하가 STM32 제어 경로를 방해하지 않았다고 판정한다.

## Raspberry Pi 자원 결과

| 지표 | 결과 | 현재 기준 | 판정 |
|---|---:|---:|---|
| CPU 평균 | 6.38% | 70% 이하 | 통과 |
| CPU p95 | 7.98% | 참고값 | 통과 |
| CPU 1초 최댓값 | 8.73% | 90% 미만 | 통과 |
| memory 사용 최댓값 | 465.3MB | 3,000MB 이하 | 통과 |
| memory 가용 최솟값 | 3,515.0MB | 700MB 이상 | 통과 |
| CPU 온도 최댓값 | 33.6°C | 80°C 미만 | 통과 |
| swap in/out | 0/0 | 0/0 | 통과 |

시험 종료 후 camera phase가 `STANDBY`로 복귀한 것도 확인했다.

## 판정

- `CAM-003 카메라 부하 중 제어 격리`: 통과 재확인
- `RES-001 Pi 자원 한도` 중 capture + decode + DDS 하위 gate: 통과
- `RES-001` 전체: 부분 통과 유지

`RES-001` 전체 통과를 위해서는 실제 perception inference, MoveIt planning, 30분 burn-in과 단계 9의 8시간/24시간 시험이 추가로 필요하다.

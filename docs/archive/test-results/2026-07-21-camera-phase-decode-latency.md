# 카메라 phase scheduler와 선택적 JPEG decode 실기 결과

- 날짜: 2026-07-21
- 대상: Raspberry Pi 5, USB MJPEG 카메라 3대
- ROS 2: Jazzy
- package: `manipulation_camera_manager`
- capture 조건: `MJPEG 640x480/30 FPS`
- 통계 창: phase가 바뀔 때 초기화되는 최근 300개 sample

## 구현 범위

- 세 카메라는 독립 V4L2 thread에서 계속 capture
- 카메라마다 압축 최신 frame 한 장만 유지
- `/camera_phase` 명령에 따라 필요한 카메라만 JPEG decode
- 디코딩 영상은 `/camera/<name>/image_raw`에 `rgb8`로 발행
- 단일 scheduler callback에서 decode하여 동시 decode 수를 1로 제한
- phase별 frame age와 JPEG decode 시간의 p50/p95/max 기록
- 잘못된 phase와 JPEG 오류를 process 종료 대신 diagnostics에 기록
- phase별 전체 inference 예산을 12Hz 이하로 시작 시 검증

## 단위시험

Pi에서 clean build 후 전체 package 시험이 통과했다.

```text
Summary: 14 tests, 0 errors, 0 failures, 0 skipped
```

시험 범위에는 최신 frame slot, phase 규칙 검증, rolling percentile, 정상 JPEG decode와 손상 JPEG 거부가 포함된다.

## STANDBY 결과

`STANDBY`에서는 top만 1Hz로 decode하고 두 wrist 카메라는 압축 frame만 갱신했다.

| 카메라 | 목표 decode | decode 횟수 | 실패 | frame age p95/max | decode p95/max |
|---|---:|---:|---:|---:|---:|
| Top | 1Hz | 15 | 0 | 15.48/15.89ms | 4.90/5.04ms |
| Wrist A | 0Hz | 0 | 0 | 미측정 | 미측정 |
| Wrist B | 0Hz | 0 | 0 | 미측정 | 미측정 |

비활성 카메라의 통계가 `-1`인 것은 오류가 아니라 decode하지 않았다는 뜻이다.

## 작업 phase별 결과

각 phase를 8초 이상 유지하며 목표 속도, 실측 속도, 실패 횟수, 지연시간을 기록했다.

| Phase | 카메라 | 목표/실측 decode | 실패 | frame age p95/max | decode p95/max |
|---|---|---:|---:|---:|---:|
| SEARCH | Top | 10/10Hz | 0 | 27.72/27.87ms | 4.80/5.00ms |
| APPROACH_RIGHT | Top | 6/6Hz | 0 | 25.08/25.42ms | 4.84/4.88ms |
| APPROACH_RIGHT | Wrist B | 8/8Hz | 0 | 30.99/31.97ms | 6.10/6.24ms |
| VISUAL_ALIGN_RIGHT | Top | 2/2Hz | 0 | 22.94/23.02ms | 4.88/4.90ms |
| VISUAL_ALIGN_RIGHT | Wrist B | 12/12Hz | 0 | 31.60/35.98ms | 6.07/6.31ms |
| TRANSFER_RIGHT | Top | 3/3Hz | 0 | 20.20/20.56ms | 4.91/5.03ms |
| TRANSFER_RIGHT | Wrist B | 2/2Hz | 0 | 33.10/33.57ms | 5.87/6.15ms |
| VERIFY_RIGHT | Top | 6/6Hz | 0 | 18.02/18.37ms | 4.92/4.99ms |
| VERIFY_RIGHT | Wrist B | 6/6Hz | 0 | 30.23/31.08ms | 5.62/6.05ms |
| DUAL_PRIVATE | Top | 6/6Hz | 0 | 15.44/16.07ms | 5.01/5.17ms |
| DUAL_PRIVATE | Wrist A | 5/5Hz | 0 | 30.44/31.77ms | 4.43/4.47ms |
| DUAL_PRIVATE | Wrist B | 5/5Hz | 0 | 27.72/30.96ms | 5.71/5.79ms |
| POLICY_ASSIST | Top | 6/6Hz | 0 | 15.90/16.61ms | 4.97/5.04ms |
| POLICY_ASSIST | Wrist B | 8/8Hz | 0 | 32.00/35.80ms | 6.10/6.19ms |

표에 없는 카메라는 해당 phase에서 목표와 실측이 모두 0Hz였고 decode 실패도 없었다.

## 판정

- 모든 phase에서 목표 decode rate와 실측값 일치
- 전체 JPEG decode 실패: 0회
- frame age 최악 p95: 33.10ms
- frame age 전체 최댓값: 35.98ms
- JPEG decode 최악 p95: 6.10ms
- JPEG decode 전체 최댓값: 6.31ms
- frame age p95 허용 기준 200ms 충족
- JPEG decode p95 허용 기준 50ms 충족
- 시험 종료 후 `STANDBY` 복귀 성공

따라서 `CAM-005 frame 최신성`을 통과 처리한다. 이 결과는 JPEG decode까지만 포함하며 실제 object detection inference의 CPU·memory·온도 검증은 `RES-001`에서 별도로 수행한다.

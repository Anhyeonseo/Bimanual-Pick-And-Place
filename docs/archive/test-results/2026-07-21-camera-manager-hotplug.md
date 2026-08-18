# 카메라 manager와 hot-plug 실기 결과

- 날짜: 2026-07-21
- 대상: Raspberry Pi 5, USB MJPEG 카메라 3대
- ROS 2: Jazzy
- package: `manipulation_camera_manager`
- 수집 조건: `MJPEG 640x480/30 FPS`

## 구현 범위

- C++17 V4L2 mmap capture
- 카메라별 독립 수집 thread
- 카메라별 압축 최신 frame 한 장만 유지
- 오래된 frame queue 없음
- USB 물리 경로 고정
- 500ms 간격 자동 재연결
- 연결 상태, frame 수, 실측 FPS, driver drop, 재연결 횟수, frame age 진단
- 원본 영상 DDS publish 없음

최신 frame slot 단위시험과 Pi package test가 통과했다.

```text
100% tests passed, 0 tests failed out of 1
Summary: 3 tests, 0 errors, 0 failures, 0 skipped
```

## 실제 FPS 보정

첫 장시간 시험에서 상단과 Wrist B는 약 30 FPS였지만 Wrist A는 약 16.7 FPS였다. V4L2 설정은 30 FPS였으나 `exposure_dynamic_framerate=1`이어서 어두운 영상의 자동 노출이 실제 frame rate를 낮췄다.

Wrist A에서 이 control을 0으로 바꾼 뒤 약 36초 동안 1,062 frame을 받아 약 29.5 FPS로 복구됐다. manager가 카메라를 열거나 재연결할 때 `V4L2_CID_EXPOSURE_AUTO_PRIORITY=0`을 적용하도록 고정했다. 해당 control이 없는 카메라는 설정 실패를 무시하고 수집을 계속한다.

## hot-plug 결과

카메라를 한 대씩 분리하고 같은 USB port에 다시 연결했다. 다른 두 카메라와 manager process는 계속 실행했다.

| 카메라 | 분리 감지 | 재연결 중 stale 차단 | fresh stream 복구 | reconnect count |
|---|---|---|---|---:|
| Top | 통과 | 통과 | 약 1초 이내 | 1 |
| Wrist A | 통과 | 통과 | 약 1초 이내 | 1 |
| Wrist B | 통과 | `CONNECTING` 표시 | 약 1초 이내 | 1 |

여기서 복구 시간은 장치가 다시 보이기 시작한 뒤 1Hz diagnostics에서 `STREAMING`을 확인할 때까지의 관측 상한이다. manager의 실제 open 재시도 간격은 500ms다.

분리 중에는 이전 frame을 계속 정상값으로 사용하지 않고 `DISCONNECTED`, `CONNECTING` 또는 `STALE_FRAME`을 냈다. 재연결 뒤 첫 새 frame이 들어오면 오류 문자열을 비우고 `STREAMING`으로 돌아왔다.

## 최종 상태

| 카메라 | 실측 FPS | frame age | reconnect | driver drop | 오류 |
|---|---:|---:|---:|---:|---|
| Top | 30.00 | 7ms | 1 | 1 | 없음 |
| Wrist A | 29.00 | 25ms | 1 | 2 | 없음 |
| Wrist B | 30.00 | 10ms | 1 | 2 | 없음 |

`latest_generation`과 `frames_received`가 카메라별로 같아 새 frame이 들어올 때마다 최신 slot이 교체되는 것을 확인했다. driver drop은 최초 stream 시작 때 1회, 손목 카메라는 hot-plug 뒤 1회 추가됐으며 장시간 실행 중 계속 증가하지 않았다.

## 판정과 후속 검증

- `CAM-002 자동 재연결`: 통과
- `CAM-005 frame 최신성`: 이 시험 시점에는 부분 통과

이 시험에서는 frame age와 stale 차단까지만 검증했다. 이후 phase scheduler와 선택적 JPEG decode를 추가하여 상태별 frame age와 decode 시간의 p95/max를 측정했고, [phase scheduler·선택적 decode 결과](2026-07-21-camera-phase-decode-latency.md)에서 최종 통과했다.

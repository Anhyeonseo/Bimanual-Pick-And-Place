# 카메라 대역폭과 STM32 제어 격리 실기 결과

- 날짜: 2026-07-21
- 대상: Raspberry Pi 5, USB 카메라 3대, NUCLEO-G474RE
- 영상 조건: `MJPEG`, `640x480`, `30 FPS`
- ROS 2: Jazzy, `single_arm_bridge`, feedback `5 Hz`

## 연결 구조

- ST-LINK VCP는 USB 2.0 `Bus 002`에 단독 연결됐다.
- 카메라 3대는 USB 2.0 `Bus 004`의 같은 Genesys Logic hub를 공유했다.
- 세 카메라는 모두 `MJPEG 640x480/30`, `1280x720/30`, `1920x1080/30`을 지원했다.
- 현재 고정 장치 경로는 다음과 같다.

| 역할 | 고정 경로 |
|---|---|
| 상단 카메라 | `/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.1:1.0-video-index0` |
| 손목 카메라 A | `/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.2:1.0-video-index0` |
| 손목 카메라 B | `/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.3:1.0-video-index0` |

손목 카메라 두 대는 VID/PID가 같고 한 대만 고유 serial을 제공하므로 USB hub 물리 port를 고정 식별자로 사용한다. 실제 좌우 역할은 장착 후 표기한다.

## 대역폭 측정

카메라 영상을 decode하거나 다시 압축하지 않고 `ffmpeg -c:v copy -f null`로 수집했다. 아래 값은 USB protocol overhead를 제외한 MJPEG payload 기준이다.

| 카메라 | 60초 수신량 | 평균 payload |
|---|---:|---:|
| 상단 | 43,560 KiB | 약 5.95 Mbps |
| 손목 A | 9,216 KiB | 약 1.26 Mbps |
| 손목 B | 163,281 KiB | 약 22.29 Mbps |
| 합계 | 216,057 KiB | 약 29.50 Mbps |

세 카메라를 동시에 60초 실행한 결과는 다음과 같다.

- 세 process 종료 코드 모두 `0`
- 세 카메라 모두 `drop_frames=0`
- 세 카메라 모두 `dup_frames=0`
- 실행 속도 `0.999x~1.01x`
- 세 stream 모두 약 60초에 정상 종료

손목 카메라 A는 stream을 열 때 첫 JPEG frame에서 `EOI missing, emulating`이 1회 발생했다. 이후 반복 오류나 frame drop은 없었다. 카메라 manager에서는 시작 frame을 버리고 이 경고 횟수를 진단값으로 기록한다.

## STM32 제어 격리

서보를 `MOTION_ENABLED`로 유지한 상태에서 카메라 3대를 동시에 60초 실행했다. 관절 이동 명령은 보내지 않았다.

- `/joint_states` 평균: `5.000 Hz`
- 측정 주기 범위: `0.199~0.201초`
- 표준편차: 약 `0.00050초`
- 카메라 실행 중 bridge heartbeat/latch 오류: 0회
- 수정 후 clean shutdown과 binary 재접속 뒤 `STOP_LATCHED=0`

시험 중 발견한 host 문제와 수정 내용은 다음과 같다.

1. 기존 400ms feedback timeout은 500ms firmware heartbeat 제한과 간격이 너무 작았다.
2. 반복 `GET_STATE` 응답 제한을 120ms로 줄여 heartbeat가 늦어지는 상한을 낮췄다.
3. 서보 설정이 필요한 ARM은 별도로 1.5초를 허용했다.
4. `Ctrl+C`가 feedback frame 중간에서 발생해도 input fragment를 버리고 heartbeat 후 DISABLE을 재시도하도록 했다.
5. 이미 binary mode인 STM32에 smoke 도구가 RESET 없이 재접속하도록 수정했다.

## 판정과 다음 작업

- `CAM-001 카메라 3대 capture`: 통과
- `CAM-003 제어 격리`: 통과
- 단계 3 전체: 아직 진행 중

현재 기본 운용값은 카메라 3대 모두 `MJPEG 640x480/30 FPS`로 둔다. USB payload에는 여유가 있지만, 실제 perception에는 JPEG decode, ROS message copy, 최신 frame buffer와 추론 부하가 추가된다. 따라서 `720p`나 `1080p`를 기본값으로 올리기 전에 decode·frame age·CPU·memory·온도를 함께 측정한다.

다음 단계 3 작업은 카메라 hot-plug 자동 재연결(`CAM-002`), 최신 frame buffer, frame age 측정(`CAM-005`)과 실제 decode/추론 자원 시험(`RES-001`)이다.

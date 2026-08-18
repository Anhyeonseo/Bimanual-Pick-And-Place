# 하드웨어 인벤토리

상태 표기: `확정`(값이 고정되고 검증됨), `알려진 제약`(운용 중이지만 아직 안 채운 항목, 시스템 동작을 막지 않음)

## 컴퓨팅

| 항목 | 값 | 상태 | 증거/비고 |
|---|---|---|---|
| 주 제어 컴퓨터(SBC) | Raspberry Pi 5 4GB | 확정 | 실제 보유 |
| SBC 운영체제 | Ubuntu Server 24.04 | 확정 | 설치 완료 |
| ROS | ROS 2 Jazzy | 확정 | 설치 완료 |
| Pi 전원 | 공식 27W USB-C | 확정 | 실제 보유 |
| Pi 냉각 | 외부 선풍기 | 확정 | 장시간 시험 전 능동 냉각 재검토 |
| Pi 저장장치 | microSD | 확정 | journald/rosbag 쓰기량 제한 필요 |
| 개발 PC | Windows 11 / Ubuntu 24.04 dual boot | 확정 | RTX 5070 Ti |

## 로봇팔과 서보

| 항목 | 값 | 상태 | 증거/비고 |
|---|---|---|---|
| 로봇팔 | 기본형 SO-ARM101 follower × 2 | 확정 | 공식 손목 카메라 mount 적용 |
| Base 설치 방향 | 양팔 동일 방향 | 확정 |  |
| Base 중심 거리 | 232.064146 mm (Y축) | 확정 | `so101_dual_preview.urdf.xacro`의 `right_mount_xyz`, eye-to-hand 보정으로 확정 |
| 자세 관절 | 팔당 5개 | 확정 | gripper 제외 |
| Gripper 축 | 팔당 1축 | 확정 |  |
| 장착 서보 | STS3215 12V × 12 | 확정 |  |
| 예비 서보 | STS3215 12V × 2 | 확정 |  |
| 서보 ID | 각 버스 1~6 | 확정 | 우팔 R0/R1.1에서 ID 1~6 응답과 관절별 +8 raw 방향 확인 |
| 왼팔 관절 원점/방향 | 프로젝트 q0 raw 2048 기준 | 확정 | 2026-07-26 실기 재정렬 기록 |
| 오른팔 관절 원점/방향 | q0 전 축 raw 2048, 좌팔과 동일 조립 방향 | 확정 | 전원 재인가 후 q0 기록 및 R1.1 전 축 +8 raw 응답 확인 |
| Raw 위치 범위 | 좌우 각 5축+gripper, 팔별 독립 값으로 확정 | 확정 | `config/bimanual_operational_limits.json`(SHA256 `436a5cfd...`)이 단일 canonical 표. resident v2 경로가 이 표로 운용 중 |
| Feedback 항목 | position/speed/load/voltage 확인 | 확정 | 단일 시험 팔 ID 1~6 실기 검증 |

## 전원과 서보 버스

| 항목 | 값 | 상태 | 증거/비고 |
|---|---|---|---|
| 왼팔 전원 | 12V 10A | 확정 | 실제 출력 전압 측정 필요 |
| 오른팔 전원 | 12V 10A | 확정 | 실제 출력 전압 측정 필요 |
| 서보 전원 분리 | 좌우 독립 | 확정 |  |
| 서보 bus driver | Waveshare Bus Servo Adapter (A) × 2 | 확정 | 왼팔 L ×1, 오른팔 R ×1; UART 논리측↔12 V servo bus 경계 |
| 서보 bus | 좌우 독립 | 확정 | ID 중복 허용, arm namespace 필수 |
| 물리 E-stop | 미설치 | 알려진 제약 | 소프트웨어 STOP/fault latch로 운용 중이며 물리 E-stop은 별도 마지막 수단으로 예정(§PROJECT_CHARTER 안전 원칙) |
| 분기 퓨즈 | 미기록 | 알려진 제약 | 배선 사진과 정격 기록은 후속 작업 |
| 접지 연결 구조 | 미기록 | 알려진 제약 | Pi/STM32/좌우 adapter 기준 전위 문서화는 후속 작업 |

## MCU

| 항목 | 값 | 상태 | 증거/비고 |
|---|---|---|---|
| 보드 | NUCLEO-G474RE | 확정 | MB1367-G474RE-D01 |
| 상위 제어기 연결 | On-board ST-LINK VCP | 확정 | 현재 COM3 실기 검증, Pi 연결 시 udev 식별값 확인 필요 |
| 실행 구조 | STM32 HAL 기반 main loop | 확정 | 현재 FreeRTOS 미사용 |
| 현재 서보 연결 | 좌팔 USART1 + 우팔 UART4 독립 bus | 확정 | 양쪽 STS3215 ID 1~6 read-only 동시 점검 통과; 각 팔 Waveshare driver 1개 |
| 양팔 확장 연결 | 우팔 UART4 PC10/CN7-1 TX→driver R TX, PC11/CN7-2 RX←driver R RX | 확정 | Bus Servo Adapter (A)는 같은 이름끼리 연결; 좌우 driver 각 1개; 공통 GND와 3.3 V IO 호환 실측 필요; 12 V bus의 MCU 핀 직접 연결 금지 |
| 현재 배포 펌웨어 | resident F8.9 / protocol v2 / `0x00024809` (12 joints) | 확정 | no-motion, current-pose hold 2회, 좌→우 Top-camera 펜 전달 실기 통과 |
| 우팔 commissioning 상태 | resident adapter 경로로 양팔 motion 승인·운용 중 | 확정 | legacy `single_arm_bridge` 일반 trajectory backend는 비승인, resident adapter 경로만 사용 |

## 카메라

| 위치 | 장치 | 상태 | 확인할 항목 |
|---|---|---|---|
| 상단(Top) | XPCAM HD 1080p USB webcam | 확정 | VID/PID, serial, UVC format, focus, exposure |
| 왼쪽 손목 | Innomaker 1080p USB 2.0 UVC | 확정 | VID/PID, serial, UVC format, focus, exposure |
| 오른쪽 손목 | Innomaker 1080p USB 2.0 UVC | 확정 | VID/PID, serial, UVC format, focus, exposure |
| USB hub | 전원 공급형 USB 3.0 hub, 5V 3A | 확정 | 연결 구조, 역전원, 카메라 3대 안정성 확인 |

## 작업 환경

| 항목 | 값 | 상태 | 증거/비고 |
|---|---|---|---|
| 첫 물체 | 검은색 마커펜 | 확정 |  |
| 목적지 | 펜꽂이 | 확정 |  |
| 조명 | 고정 환경 | 확정 |  |
| 작업대 크기/높이 | 사전 고정값 없음 | 확정 | Top-camera homography로 매 실행 동적 검출. 고정 치수에 의존하지 않는다 |
| 펜 크기 | 사전 고정값 없음 | 확정 | YOLO-OBB가 매 실행 영상에서 위치/yaw를 검출한다 |
| 펜꽂이 입구 | 사전 고정값 없음 | 확정 | place 목표는 workcell 좌표계 고정 지점이며 입구 치수 측정에 의존하지 않는다 |

# 현재 상태

- 기준일: 2026-08-16
- 기준 firmware: F8.9 `0x00024809`, protocol v2, 12축 resident
- 수락점: Top-camera 기반 왼팔→오른팔 펜 연속 pick-and-place 완주

이 저장소는 이 지점에서 동결됐다. 캔→쓰레기통, 수건 닦기·접기 등 후속
매니퓰레이션 태스크는 별도 저장소에서 이어간다.

## 완료

**하드웨어 제어**

- STM32 한 대가 좌팔(USART1)과 우팔(UART4) SO-ARM101을 각각 6축씩 독립 제어한다.
- 공통 5 ms executor로 동작하며 paired DMA dispatch, operational limits,
  shoulder unwrap, measured tracking, coordinated stop을 갖췄다.

**통신 계층**

- Pi의 resident adapter만 serial backend를 소유한다.
- ROS로 12축 finite command, fresh anchor, terminal feedback, owner/epoch
  상태를 공개한다.

**안전 한계**

- 팔 tracking 한계는 그대로 유지하고, 그리퍼 접촉에만 완화된 한계를 적용한다:
  terminal/route `150,000 µrad`, firmware hard cap `160,000 µrad`.
- motion-disabled gate와 current-pose hold 2회를 통과했다.

**카메라·계획**

- 상단 카메라/작업대 보정과 오른팔 data-fit URDF를 적용했다.
- 계획 스키마 12가 화면축 보정을 plan SHA에 고정한다.
  - 왼팔: 화면 오른쪽 13.72 mm
  - 오른팔: 화면 왼쪽 29.47 mm

**실행 결과**

- fresh left plan → 검증 → 실행 → fresh right plan → 검증 → 실행을 자동
  재시도 없이 완주했고, 최종 READY/HOLD를 유지했다.
- 최종 증거: [F8.9 resident와 양팔 펜 전달 수락 결과](archive/test-results/2026-08-16-f89-bimanual-pen-transfer.md)

## 운영 불변식

1. motion은 `ready`, `owner=null`, `arbiter_epoch=0`,
   `motion_authorized=true`인 새 session에서 시작한다.
2. STOP/FAULTED session은 재사용하지 않고 STM32 reset과 resident 재시작 후
   새 anchor를 얻는다.
3. 한 팔 동작도 반대 팔 hold를 포함한 12축 command로 제출한다.
4. firmware와 resident의 measured terminal 판정 전에는 성공으로 보지 않는다.
5. dispatch, heartbeat, unwrap, operational-limit 및 arm tracking fault는
   자동 재시도하지 않는다.
6. plan-only/validate-only와 실제 실행을 분리하고 plan SHA와 최대 age를 검사한다.

현재 wrist-roll 한계:

| arm | lower | upper | span |
|---|---:|---:|---:|
| left | -128.41° | +69.43° | 197.84° |
| right | -114.17° | +81.04° | 195.21° |

양팔 모두 180°보다 넓어 무방향 장축의 모든 yaw를 표현할 수 있지만,
수직 접근·충돌·관절 결합 가능성은 각 plan에서 별도 검증한다.

## 관련 문서

- [양팔 상단 애플리케이션 인터페이스](BIMANUAL_UPPER_APPLICATION_INTERFACE.md)
- [Top-camera resident Pick/Place](TOP_CAMERA_RESIDENT_PICK_PLACE_APPLICATION.md)
- [F8.7 이전 수락 결과](archive/test-results/2026-08-15-f87-resident-top-camera-pick-place.md)
- [F8.9 최종 수락 결과](archive/test-results/2026-08-16-f89-bimanual-pen-transfer.md)

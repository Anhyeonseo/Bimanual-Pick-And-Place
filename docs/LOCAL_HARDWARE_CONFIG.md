# 로컬 하드웨어 설정

이 문서는 공개 저장소에 개인 장치 식별자를 넣지 않으면서 개발 PC와 Raspberry Pi의 실행 설정을 유지하는 방법을 설명한다.

## 기본 원칙

- 공개 가능한 공통 설정은 tracked YAML에 둔다.
- 장치 serial, 실험실별 port와 개인 경로는 `*.local.yaml`에 둔다.
- `*.local.yaml`은 `.gitignore`로 제외한다.
- launch는 공통 YAML을 먼저 읽고 local YAML이 있으면 나중에 읽어 같은 parameter를 덮어쓴다.
- calibration과 안전 limit은 개인 편의 설정이 아니므로 검증된 공통 파일에서 관리한다.

## STM32 serial 설정

공통 파일:

```text
ros2_ws/src/single_arm_bridge/config/bimanual_stream.yaml
```

기본 `serial_device` 값은 `auto`다(`device_discovery.resolve_serial_device`).

- 단일 ST-LINK by-id를 탐색한다.
- 못 찾으면 `/dev/ttyACM0`를 fallback으로 쓴다.
- ST-LINK가 둘 이상 발견되면 잘못된 팔을 제어하지 않도록 실행을 거부한다.

장치를 명시적으로 고정하려면 `serial_device`를 launch argument로 넘긴다.
양팔 resident adapter는 별도 local YAML 오버레이 파일을 쓰지 않는다.

```bash
ls -l /dev/serial/by-id/
ros2 launch single_arm_bridge bimanual_stream.launch.py serial_device:=/dev/serial/by-id/<실제 값>
```

## 실행

```bash
ros2 launch single_arm_bridge bimanual_stream.launch.py
```

legacy `single_arm_bridge` 일반 trajectory backend(`bridge.launch.py`)는
비승인이다. 양팔 motion은 resident adapter(`bimanual_stream.launch.py`)
경로만 사용한다.

## 새 PC 또는 새 Pi로 이동할 때

Git clone에는 local YAML이 포함되지 않는다. 다음 순서로 복구한다.

1. tracked example을 local YAML로 복사
2. 새 장치의 by-id를 측정해 입력
3. package 다시 build
4. `allow_motion:=false`로 먼저 `/joint_states` 확인
5. protocol smoke를 통과한 뒤에만 motion 허용

local YAML을 별도로 백업할 수는 있지만 공개 GitHub나 issue 본문에는 올리지 않는다.

## 카메라 경로

카메라는 `manipulation_camera_manager/config/cameras.yaml`의 USB 물리 port
기반 경로를 사용한다. 카메라나 hub port를 바꾸면:

```bash
ls -l /dev/v4l/by-path/
```

로 경로를 다시 확인한 뒤 YAML을 수정한다.

카메라 경로에는 인증정보가 없다. 다만 port 역할이 바뀌면 `top`, `wrist_a`,
`wrist_b` 영상이 뒤바뀔 수 있으므로, 변경 후에는 `/camera_diagnostics`와
실제 영상을 함께 확인한다.

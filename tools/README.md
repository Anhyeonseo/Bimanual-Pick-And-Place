# tools/

이 레포는 펜 연속 pick&place 한 가지만 다룬다. 폴더가 "뭘 실행해야 하는지"를
말해준다.

```text
tools/
├── run/                     실제로 펜 pick&place를 돌릴 때 실행하는 것
├── lib/                     직접 실행하지 않음 — run/·setup/이 import하는 라이브러리
├── setup/                   새 하드웨어에서 이 시스템을 재현하려면 한 번씩 실행
│   ├── stm32/               STM32/서보 기본 점검
│   ├── right_arm/           오른팔 bring-up
│   ├── resident_gate/       resident adapter pre-flight
│   ├── camera_calibration/  카메라 캘리브레이션
│   ├── pen_detector_training/  펜 검출기(YOLO-OBB) 재학습
│   └── firmware/            firmware/하드웨어 기준선
├── diagnostics/             범용 하드웨어 진단(특정 실패 재현용, 태스크 무관)
└── contract_evidence/       직접 실행 대상 아님 — 아래 설명 참고
```

## run/ — 골든 패스

`run_left_right_pen_transfer_once.py`가 아래 두 개를 subprocess로 호출한다.

- `run_left_right_pen_transfer_once.py` — 오케스트레이터
- `plan_top_camera_pick_place_once.py` — 카메라 기반 plan-only
- `run_top_pick_place_application_once.py` — resident adapter 실행
- `validate_protocol_manifest.py`, `validate_camera_schedule.py` — README가
  명시하는 자동 판정 게이트

## contract_evidence/ — 직접 실행 대상 아님

`tests/test_f7_bimanual_dma_candidate.py`, `test_f8_bimanual_tracking_feedback.py`,
`test_bimanual_resident_finite_completion.py`, `test_f81_bimanual_feedback_snapshot.py`,
`test_top_pick_place_application.py`가 이 폴더 파일들의 **소스코드 자체**를
읽어서 fail-closed 동작을 검증한다. 지우면 저 테스트들이 깨진다 — 수정할
때도 대응 테스트를 먼저 확인할 것.

## 폴더를 옮기거나 파일을 추가할 때

- 각 하위 폴더는 `__init__.py`가 있는 진짜 Python 패키지다. `from
  tools.lib.actuator_protocol import ...` 처럼 점(.) 경로로 import하는 코드가
  있다(`tools/lib/joint_calibration.py`, `tools/setup/stm32/*.py`).
- `pytest.ini`의 `pythonpath = ... tools/lib`가 `tools/lib/*.py`의 bare-name
  import(`from top_pick_place_application import ...`)를 성립시키는 근거다.
- `tools/run/*.py`에서 `tools/lib/*.py`를 bare-name import하는 곳은 파일
  안에서 직접 `sys.path`에 `tools/lib`를 추가한다(다른 폴더로 옮기면 같이
  옮길 것).
- 폴더 깊이가 바뀌면 각 파일의 `Path(__file__).resolve().parents[N]` 같은
  루트 계산도 같이 맞춰야 한다 — 한 번 전부 어긋났던 적이 있다(실측 확인됨).

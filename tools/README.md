# tools/ 분류

이 레포는 펜 연속 pick&place 한 가지만 다룬다. 65개 스크립트가 평평하게
있지만 역할은 다섯 갈래다.

## 1. 실행 — 골든 패스

실제로 펜 pick&place를 돌릴 때 실행하는 것. 순서: `run_left_right_pen_transfer_once.py`가
아래 두 개를 subprocess로 호출한다.

- `run_left_right_pen_transfer_once.py` — 오케스트레이터
- `plan_top_camera_pick_place_once.py` — 카메라 기반 plan-only
- `run_top_pick_place_application_once.py` — resident adapter 실행
- `validate_protocol_manifest.py`, `validate_camera_schedule.py` — README가 명시하는 자동 판정 게이트

## 2. 라이브러리 — 직접 실행하지 않음, import됨

`top_pick_place_application.py`, `grasp_yaw_kinematics.py`, `joint_calibration.py`,
`actuator_protocol.py`, `__init__.py`

## 3. 셋업 — 새 하드웨어에서 이 시스템을 재현하려면 한 번씩 실행

- **STM32/서보**: `stm32_motion_stop_test.py`, `stm32_setpoint_motion_test.py`,
  `stm32_protocol_smoke.py`, `stm32_raw_range_observer.py`, `stm32_read_only_counters.py`
- **오른팔 bring-up**: `right_arm_home.py`, `enable_right_arm_torque_once.py`,
  `execute_right_arm_jog_once.py`, `execute_right_arm_bounded_home_once.py`,
  `capture_right_arm_configuration_read_only.py`, `configure_right_arm_from_left_once.py`,
  `soak_right_arm_read_only_discovery.py`, `derive_bimanual_operational_limits_plan_only.py`
- **resident adapter pre-flight**: `validate_resident_bimanual_adapter_no_motion.py`,
  `execute_resident_bimanual_current_pose_hold_twice.py`
- **카메라 캘리브레이션**: `calibrate_top_homography.py`, `top_homography_capture.py`,
  `calibrate_top_base_table.py`, `assemble_top_base_table_session.py`,
  `monitor_top_base_table_gridboard.py`, `solve_top_eye_to_hand.py`,
  `capture_top_eye_to_hand_sample.py`, `assemble_top_eye_to_hand_session.py`,
  `monitor_top_eye_to_hand_gridboard.py`, `solve_top_base_visual_registration.py`,
  `capture_top_frame.py`, `detect_top_tcp_marker.py`, `top_camera_qos_relay.py`,
  `monitor_top_yolo_obb.py`, `generate_top_eye_to_hand_gridboard.py`,
  `render_top_eye_to_hand_target_pdf.py`, `generate_planar_aruco_gridboard.py`,
  `render_camera_calibration_print_pack.py`
- **펜 검출기(YOLO-OBB) 재학습**: `bootstrap_top_pen_obb_labels.py`,
  `build_top_pen_obb_training_manifest.py`, `train_export_top_pen_yolo_obb.py`,
  `validate_top_pen_obb_training_dataset.py`, `evaluate_top_pen_yolo_obb.py`,
  `evaluate_top_pen_detection_baseline.py`(위 evaluate 툴이 import하는 의존 모듈)
- **firmware/하드웨어 기준선**: `generate_protocol_header.py`, `validate_phase0.py`,
  `generate_overhead_top_hinge_removed.py`

## 4. 진단 (범용)

특정 실패를 재현할 때 씀. 캔/펜 등 태스크에 종속되지 않는 범용 하드웨어 진단.

- `diagnose_bimanual_shadow_anchor_once.py`
- `capture_top_shadow_target_once.py`

## 5. 안전성 계약 근거 — 직접 실행 대상 아님

`tests/test_f7_bimanual_dma_candidate.py`, `test_f8_bimanual_tracking_feedback.py`,
`test_bimanual_resident_finite_completion.py`, `test_f81_bimanual_feedback_snapshot.py`,
`test_top_pick_place_application.py`가 이 파일들의 **소스코드 자체**를 읽어서
fail-closed 동작을 검증한다. 지우면 저 테스트들이 깨진다 — 수정할 때도 대응
테스트를 먼저 확인할 것.

`execute_f7_bimanual_base_small_roundtrip_once.py`,
`execute_f7_bimanual_current_pose_hold_once.py`,
`execute_f8_bimanual_tracking_hold_once.py`,
`execute_resident_bimanual_base_small_roundtrip_once.py`,
`execute_resident_bimanual_rolling_base_small_roundtrip_once.py`,
`execute_resident_bimanual_rolling_horizon_no_motion_once.py`,
`execute_resident_right_arm_q0_once.py`,
`validate_f7_bimanual_dma_no_output.py`,
`validate_f7_bimanual_right_dma_fault_stop_once.py`,
`validate_f8_bimanual_tracking_fault_stop_once.py`,
`validate_f8_bimanual_tracking_no_output.py`

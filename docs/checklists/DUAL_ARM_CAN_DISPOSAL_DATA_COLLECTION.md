# 양팔 캔 폐기용 OBB 데이터 수집

이 단계는 Top 카메라의 원본 영상과 캔 중심·장축 라벨을 저장한다. 로봇
publisher, Action, service, serial 연결을 만들지 않으며 어떤 동작도 승인하지
않는다.

## 현장 준비

1. `config/can_disposal_capture.json`의 `instance_id`, 캔 지름·길이·무게를
   실측값으로 바꾼다. 설정을 바꾼 뒤에는 새 dataset directory를 사용한다.
2. Top 카메라와 작업대 보정판을 움직이지 않는다.
3. 캔 중심의 `top_board` X/Y는 검증된 눈금판에서 읽고 미터 단위로 입력한다.
4. 누운 캔의 yaw는 긴 축의 방향이며 180도 대칭인 0~180도로 기록한다.
   그리퍼는 이 장축에 수직인 방향으로 닫힌다.
5. 왼팔·오른팔 작업영역과 중앙 handoff 경계 주변을 모두 수집한다.

## 한 조건 수집

~~~bash
python3 tools/capture_object_pose_dataset.py \
  --config config/can_disposal_capture.json \
  --dataset-root datasets/dual_arm_can_disposal_20260815 \
  --capture-id left_center_yaw045_trial01 \
  --state lying \
  --position-label left_center \
  --ground-truth-x-m 0.000 \
  --ground-truth-y-m 0.000 \
  --ground-truth-yaw-deg 45 \
  --background marble_table \
  --lighting room_light \
  --glare low
~~~

각 실행은 `captures/<capture-id>/` 아래 PNG 5장과 `capture.json`을 만들고,
dataset root의 `dataset.json`에 SHA-256과 함께 등록한다. 같은 capture ID는
덮어쓰지 않는다.

## 권장 수집량

- 위치: 좌측 3곳, 우측 3곳, 중앙 handoff 경계
- 누운 장축 방향: 0, 30, 60, 90, 120, 150도
- 누운 캔 150장, absent·distractor 30장, 반사·가장자리 20장
- 조건당 PNG 5장, 총 200장

이후 개발 장소에서는 저장된 PNG만으로 캔 OBB·중심·장축 검출 코드를
작성한다. 캔 위치가 오른쪽이면 오른팔이 바로 오른쪽 쓰레기통으로 이동하고,
왼쪽이면 왼팔이 고정 handoff 위치로 옮긴 뒤 오른팔이 인계받는 것은 후속
MoveIt 단계다.

## 학습 모델 검출 확인

캔 OBB 모델을 학습한 뒤 저장 이미지 한 장의 중심·장축과 그리퍼 가로 방향을
다음 명령으로 확인한다. 이 도구는 로봇이나 ROS에 연결하지 않는다.

~~~bash
python3 tools/detect_can_obb_image.py \
  --model artifacts/can_obb/best.pt \
  --image datasets/dual_arm_can_disposal_20260815/captures/left_center_yaw045_trial01/frame_000.png \
  --output artifacts/can_obb/detection.json \
  --overlay artifacts/can_obb/detection_overlay.png
~~~

출력의 `long_axis_yaw_deg`는 캔의 긴 축이고,
`gripper_closing_yaw_deg`는 그 축에 수직인 파지 방향이다. 학습 이미지와
Ultralytics OBB label은 별도의 라벨링 단계에서 만들어야 하며, 사전학습
모델만으로 이 명령을 실행해 캔 검출이 된다고 가정하지 않는다.

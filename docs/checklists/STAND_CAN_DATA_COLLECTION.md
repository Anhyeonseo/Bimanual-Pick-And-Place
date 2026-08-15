# 캔 자세 데이터 수집

이 단계는 Top 카메라의 원본 영상과 수동 위치·방향 라벨을 저장한다. 로봇
publisher, Action, service, serial 연결을 만들지 않으며 어떤 동작도 승인하지
않는다.

## 현장 준비

1. `config/can_pose_capture.json`의 `instance_id`, 캔 지름·길이·무게를 실측값으로
   바꾼다. 설정을 바꾼 뒤에는 새 dataset directory를 사용한다.
2. Top 카메라와 작업대 보정판을 움직이지 않는다.
3. 캔 중심의 `top_board` X/Y는 검증된 눈금판에서 읽고 미터 단위로 입력한다.
4. 누운 캔은 중심에서 **캔 바닥면 방향**으로 향하는 각도를 기록한다. 이 값은
   0~360도 방향성이 있으며, 캔을 거꾸로 세우는 실패를 구분하기 위해 필요하다.
   장축의 0~180도 yaw는 이 값에서 자동으로 유도된다.
5. upright 기준 영상은 캔 윗면이 위로 향한 정상 상태만 수집한다.

## 한 조건 수집

~~~bash
python3 tools/capture_object_pose_dataset.py \
  --config config/can_pose_capture.json \
  --dataset-root datasets/stand_can_pose_20260815 \
  --capture-id center_yaw045_trial01 \
  --state lying \
  --position-label center \
  --ground-truth-x-m 0.000 \
  --ground-truth-y-m 0.000 \
  --bottom-end-yaw-deg 225 \
  --background marble_table \
  --lighting room_light \
  --glare low
~~~

각 실행은 `captures/<capture-id>/` 아래 PNG 5장과 `capture.json`을 만들고,
dataset root의 `dataset.json`에 SHA-256과 함께 등록한다. 같은 capture ID는
덮어쓰지 않는다.

## 권장 수집량

- 위치: front, center, back, left, right
- 누운 장축 방향: 0, 45, 90, 135도
- 각 배치에서 중심→캔 바닥면 방향을 0~360도로 별도 기록
- 조건당 PNG 5장: 기본 100장
- 별도로 upright와 absent 조건도 수집

이후 개발 장소에서는 저장된 PNG만으로 검출·좌표 변환 코드를 작성한다.

# parallel_parking

ROS1 rule-based one-shot parallel parking을 ROS2 Humble과 현재 실차 제원에
맞게 이식한 패키지입니다.

상태 흐름:

`IDLE → APPROACH → SETTLE → MANEUVER → FINAL_ALIGN → PARKED
→ EXIT_MANEUVER → EXIT_COMPLETE → DONE`

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select parallel_parking
source install/setup.bash
```

## Run

센서, `/scan_parking`, `/wheel_odom` 및 차량 명령 브리지를 먼저 실행한 뒤:

```bash
ros2 launch parallel_parking parking_parallel_oneshot.launch.py
ros2 topic pub -1 /parking_start std_msgs/msg/Bool "{data: true}"
```

주차 완료 후 노드는 `PARKED`에서 정지하고 `/parking_parked=true`를
발행합니다. 출차 시 현재 정차 자세에서 기존 경로의 역주행을 먼저
검증하고, 정차 오차 때문에 위험하면 `저속 후진 중 헤딩 보정 → 슬롯
반대쪽 전진 원호 → 복귀 전진 원호`의 S자 경로를 새로 탐색합니다. 두
전진 원호의 각도는 독립적으로 계산하여 시작 헤딩으로 복귀합니다. 첫
출차 시험은 다음 수동 트리거를 사용합니다.

```bash
ros2 topic pub -1 /parking_exit_start std_msgs/msg/Bool "{data: true}"
```

검증 후 자동 출차는 다음과 같이 켭니다.

```bash
ros2 launch parallel_parking parking_parallel_oneshot.launch.py \
  auto_exit:=true parking_hold_sec:=3.0
```

차체 전체가 슬롯 밖으로 빠지고 시작 헤딩으로 정렬되면
`/parking_exit_done=true`, `/parking_done=true`,
`/parking_active=false`, `/parking_request_stop=false`를 발행하고
`/cmd_*` 발행을 중단하여 웨이포인트 제어권을 반환합니다.

기본 설정은 차량 진행 방향 기준 우측 평행주차입니다. T자 주차는 좌측,
평행주차는 우측이라는 현재 코스 구성을 반영했습니다.

## Safety

- 기본값은 `auto_start: false`입니다.
- 첫 실차 시험 전 좌/우 최대 조향각과 실제 최소 회전반경을 측정하십시오.
- 반드시 구동 바퀴를 띄우거나 저속의 통제된 공간에서 토픽과 조향 부호를
  먼저 검증하십시오.
- `parking_parallel_right.yaml`의 슬롯 크기와 안전 여유는 실제 코스 실측 후
  조정해야 합니다.

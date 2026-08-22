# 실행 방법 전체 정리

모든 명령어는 `source ~/ros2_ws/install/setup.bash` 이후 기준
(워크스페이스 빌드 안 했으면 `cd ~/ros2_ws && colcon build` 먼저).

## 0. CAN 인터페이스 켜기 (실제 차량 제어 시 필수)

```bash
sudo ip link set can0 up type can bitrate 500000
```

껐다 켜기(문제 있을 때):
```bash
sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 500000
```

상태 확인:
```bash
ip -details link show can0   # state ERROR-ACTIVE면 정상, BUS-OFF면 문제
```

## 1. GPS/RTK 켜기

```bash
ros2 launch f9p_bringup f9p_rover.launch.py
```
`ublox_gps_node` + `rtk_bridge.py`(NTRIP RTCM) 같이 켜줌. 로그에
`configured successfully` 뜨고 Float/Fixed 될 때까지 대기(콜드리셋 포함
~15초). **ZED 카메라 USB 꽂기 전에 이거 먼저 해서 Fixed 잡는 게 안전**
(USB3-EMI, `03_TROUBLESHOOTING.md` 참고).

GPS 단독으로만 켜려면 이걸로 충분. `/fix` 리맵됨 →
**`/ublox_gps_node/fix`가 실제 토픽 이름** (이 launch로 켰을 때).

## 2. GPS 품질 모니터 (선택, EMI/RTK 디버깅용)

```bash
ros2 run waypoint_follower gps_quality_monitor
```
`carrSoln`(Fixed/Float), `h_acc`(cm), `jam_ind`/`jamming_state`(재밍
지표) 한 줄에 라벨 붙여서 출력.

## 3. 웨이포인트 기록

```bash
ros2 run waypoint_follower waypoint_recorder_node --ros-args \
  -p gps_topic:=/ublox_gps_node/fix \
  -p output_file:=/home/a/ros2_ws/src/waypoint_follower/waypoints/path_$(date +%Y%m%d_%H%M%S).csv \
  -p min_waypoint_distance_m:=1.0
```
Ctrl+C로 저장. `gps_topic`은 GPS를 어떻게 켰냐에 따라 다름 (`/fix` 단독
실행 시, `/ublox_gps_node/fix`는 `f9p_rover.launch.py`로 켰을 때).

## 4. 통합 주행 (GPS+카메라+arbiter+LiDAR+신호등, 메인 사용법)

GPS를 1번으로 이미 켠 상태에서:

```bash
ros2 launch waypoint_follower post_gps_drive.launch.py \
  waypoints_file:=/home/a/ros2_ws/src/waypoint_follower/waypoints/<파일명>.csv \
  enable_control:=true \
  cruise_rpm:=130 \
  curve_lead_margin:=1.2 \
  camera_mode_rpm:=130.0 \
  camera_can_target_rpm:=130 \
  base_steer_lowpass_alpha:=0.3
```

**주요 launch 인자** (`04_PARAMETERS_REFERENCE.md`에 전체 표):
| 인자 | 기본값 | 의미 |
|---|---|---|
| `waypoints_file` | (지정 필요) | 추종할 웨이포인트 CSV |
| `enable_control` | false | true여야 실제 CAN 송신 |
| `cruise_rpm` | 140 | GPS 순항 rpm 상한 |
| `curve_lead_margin` | 1.2 | 코너 예견 반응 배율 (크면 일찍 꺾음) |
| `camera_mode_rpm`/`camera_can_target_rpm` | 130.0 / 130 | 카메라 주행 rpm 관련 (아래 참고, 값 맞춰서 같이 넘길 것) |
| `base_steer_lowpass_alpha` | 0.3 (이 launch 파일 기준) | 조향 스무딩, 1.0=꺼짐 |
| `gps_priority_check_traffic_light` | false (이 launch 파일 기준) | OAK-D 없으면 false |
| `loop_waypoints` | false | 코스 끝나면 처음으로 루프 여부 |

**한 launch 파일 안에서 뜨는 것들**: `waypoint_follower_node`,
`yolopv2_zed_rpm_node`(카메라), `control_arbiter`, `traffic_light_node`,
`obstacle_avoid_node`, ZED 래퍼(`zed_wrapper`), 주차 노드들
(`event_zones`에 정의된 것만 실제로 관여함).

**GPS까지 같이 켜는 버전** (한방에, EMI 순서 안 지켜도 될 때):
```bash
ros2 launch waypoint_follower integrated_drive.launch.py \
  waypoints_file:=... enable_control:=true ...  # 인자는 위와 거의 동일
```

## 5. 카메라 단독 실행 (디버깅용)

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```
켜지면 다른 터미널에서:
```bash
ros2 run zed_camera yolopv2_zed_rpm_node --ros-args \
  -p can_enable:=false \
  -p auto_speed:=true \
  -p can_target_rpm:=130
```
확인: `ros2 topic hz /yolopv2_zed_node/lane_valid`,
`ros2 topic echo /yolopv2_zed_node/steering_deg`

## 6. 주차 단독 테스트 (arbiter 없이)

```bash
ros2 launch waypoint_follower parking_t_left.launch.py direct_cmd_output:=true auto_start:=true
ros2 launch waypoint_follower parking_parallel_right.launch.py direct_cmd_output:=true auto_start:=true
```
LiDAR/IMU는 각자 launch 파일로 따로 켜야 함
(`obstacle_avoidance/launch/rplidar_s2.launch.py`,
`handsfree_ros2_imu/launch/handsfree_imu.launch.py`).

**주의**: `parallel_parking/launch/parking_parallel_oneshot.launch.py`,
`t_parking/launch/parking_left_oneshot.launch.py`는 **깨져있음** —
존재하지 않는 패키지(`sllidar_ros2`, `my_first_pkg`) 참조함. 쓰지 말 것.

## 7. 시각화

```bash
ros2 run waypoint_follower mpl_viz_node
```
matplotlib 기반 대체 시각화 (RViz 없이도 경로/차량위치/조향각 확인 가능).

## 8. 상태 확인용 명령어 모음

```bash
ros2 topic hz /ublox_gps_node/fix                  # 20Hz 나와야 정상
ros2 topic echo /navpvt --field flags               # 1=noRTK 64=Float 128/131=Fixed
ros2 param get /control_arbiter event_zones          # 지금 떠있는 존 설정 확인 (재시작 확인용)
ros2 topic echo /control_arbiter/active_source       # 지금 누가 운전 중인지
```

## 자주 하는 실수

- **launch 파일 수정 후 재시작 안 함**: ROS2 launch는 그 순간에만 파일을
  읽음. 코드/설정 바꿨으면 Ctrl+C로 완전히 끄고 다시 켜야 반영됨
  (`ros2 param get`으로 확인 가능).
- **GPS 토픽 이름 헷갈림**: `f9p_rover.launch.py`로 켜면
  `/ublox_gps_node/fix`, `ros2 run ublox_gps ublox_gps_node`로 단독
  실행하면 `/fix` (리맵 없음).
- **ZED 카메라 USB3가 GPS EMI에 영향** — 실행 순서/배선 신경 쓸 것
  (`03_TROUBLESHOOTING.md`).

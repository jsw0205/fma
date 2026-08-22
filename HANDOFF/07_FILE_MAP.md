# 파일 맵 — 패키지별 주요 파일 설명

## `waypoint_follower/` (핵심 패키지)

| 파일 | 역할 |
|---|---|
| `waypoint_follower/arbiter_node.py` | **`control_arbiter`** — 카메라/GPS/LiDAR/주차/신호등 우선순위 중재 + CAN 유일 송신자. 제일 중요한 파일. |
| `waypoint_follower/waypoint_follower_node.py` | GPS 웨이포인트 추종, Stanley 컨트롤러, IMU/지자기 헤딩융합, CAN 폴백 송신(옵션) |
| `waypoint_follower/stanley_controller.py` | Stanley 제어법칙 + 커브 예견(anticipatory) 로직, `lookahead_path_curve()` |
| `waypoint_follower/can_driver.py` | CAN 프로토콜 구현 전체 (5개 메시지 pack/parse) — `05_CAN_PROTOCOL.md` 참고 |
| `waypoint_follower/geo_utils.py` | 위경도↔로컬ENU 좌표변환 (Web Mercator 기반) |
| `waypoint_follower/waypoint_recorder_node.py` | GPS로 웨이포인트 CSV 기록 |
| `waypoint_follower/gps_quality_monitor.py` | **(신규, 2026-08-19)** RTK Fixed/Float + EMI 진단용 실시간 모니터 |
| `waypoint_follower/mpl_viz_node.py` | matplotlib 기반 시각화 (RViz 대체) |
| `waypoint_follower/status_monitor_node.py` | 상태 출력용 |
| `waypoint_follower/gps_node.py`, `fake_gps_node.py` | GPS 관련 보조/테스트 노드 |
| `waypoint_follower/lidar_front_distance_node.py` | 전방 LiDAR 거리 보조 노드 |
| `waypoint_follower/mag_calibrate.py`, `imu_drift_test.py` | IMU/지자기 캘리브레이션·테스트 도구 |
| `launch/integrated_drive.launch.py` | GPS+카메라+arbiter+LiDAR+신호등 전부 한방에 |
| `launch/post_gps_drive.launch.py` | 위와 동일하되 GPS 빼고 (GPS 따로 먼저 켤 때, EMI 회피용) — **제일 많이 쓰는 launch 파일** |
| `launch/parking_t_left.launch.py`, `parking_parallel_right.launch.py` | 주차 단독 테스트 |
| `launch/gps_rtk_waypoint.launch.py` | GPS+웨이포인트 추종만 (카메라/arbiter 없음, 구버전 스타일) |
| `waypoints/*.csv` | 기록된 코스들. 최신: `path_20260818_145848.csv` |
| `henes_can.dbc` | CANoe용 DBC 파일 |
| `README_CAN_PROTOCOL.md` | CAN 프로토콜 원본 문서 (`HANDOFF/05_CAN_PROTOCOL.md`가 이걸 정리한 버전) |

## `zed_camera/` — 카메라 차선인식

| 파일 | 역할 |
|---|---|
| `zed_camera/yolopv2_zed_rpm_node.py` | **실제 쓰는 노드** — YOLOPv2 차선인식, `LaneTracker`, BEV, 커브기반 rpm 스케일링, `~/rpm_target` publish |
| `zed_camera/yolopv2_zed_node.py` | 구버전(슬라이딩윈도우), 안 씀, 참고용으로만 남음 |
| `zed_camera/utils/utils.py` | YOLOPv2 모델 유틸 (원본 CAIC-AD/YOLOPv2 레포에서 가져옴) |
| `weights/yolopv2.pt` | 모델 가중치 (150MB, git엔 안 올라감 — `.gitignore`) |

## `f9p_bringup/` — GPS/RTK

| 파일 | 역할 |
|---|---|
| `launch/f9p_rover.launch.py` | `ublox_gps_node` + `rtk_bridge.py` 통합 실행, `/fix`→`/ublox_gps_node/fix` 리맵 |
| `config/ublox_rover.yaml` | u-blox 드라이버 설정 (publish 옵션: pvt/sat/status/hw 등 다 켜져 있음) |
| `f9p_bringup/gps_to_odom.py` | GPS→오도메트리 변환 |
| `f9p_bringup/rtcm_serial_bridge.py` | (참고용, 실제로는 `rtk_bridge.py` 씀) |

## `rtk_bridge/` — NTRIP 중계 (ROS 패키지 아님, 순수 스크립트)

| 파일 | 역할 |
|---|---|
| `rtk_bridge.py` | NTRIP에서 RTCM 받아 시리얼로 직접 write. **실제 계정정보 들어있음, git엔 `.example`만 커밋됨** |
| `plot_nmea_log.py` | NMEA 로그 시각화 도구 |

## `traffic_light/` — OAK-D 신호등 인식

| 파일 | 역할 |
|---|---|
| `traffic_light/traffic_light_node.py` (`test_sunny_node`) | OAK-D+YOLO 신호등 인식, `/traffic_light`(GO/STOP) publish. **자체 while-loop 스레드**로 동작 (ROS 타이머 아님). depthai/OAK-D 없으면 idle 상태로 안전하게 대기 |

## `obstacle_avoidance/` — LiDAR 회피

| 파일 | 역할 |
|---|---|
| `obstacle_avoidance/obstacle_avoid_node.py` | CLEAR→AVOID→PASS→RETURN 상태머신, RPLiDAR A3M1 기반 |
| `launch/rplidar_s2.launch.py` | RPLiDAR 드라이버 단독 실행 |

## `t_parking/` — T자 후진주차

| 파일 | 역할 |
|---|---|
| `t_parking/rule_based_t_parking_node.py` | 메인 주차 로직 |
| `t_parking/slot_detector.py`, `cone_detector.py`, `two_arc_planner.py`, `geometry.py` | 슬롯탐지/경로계획 보조 모듈 |
| `t_parking/wheel_odom_front_axle_node.py` | 휠 오도메트리 |
| `t_parking/README.md` | 이 패키지 자체 상세 문서 |

## `parallel_parking/` — 평행주차

| 파일 | 역할 |
|---|---|
| `parallel_parking/rule_based_parallel_parking_node.py` | 메인 평행주차 로직 (odom 콜백 기반, 자체 타이머 없음) |
| `parallel_parking/README.md` | 이 패키지 자체 상세 문서 |

## `parking_bridge/` — 주차 관련 브릿지

| 파일 | 역할 |
|---|---|
| `parking_bridge/wheel_odom_pcan_node.py` | PCAN 기반 휠 오도메트리 브릿지 |

## `handsfree_ros2_imu/` — IMU

| 파일 | 역할 |
|---|---|
| `handsfree_ros2_imu/hfi_a9_ros2.py` | HandsFree HFI-A9 IMU 드라이버, `/handsfree/imu`+`/handsfree/mag` publish |

## `pcan_tools/` — 수동 CAN 테스트 도구 (ROS 아님)

`pcan_jetson_live.py`, `pcan_tc275_linux.py` 등 — PS2 수동조작/CAN 직접
테스트용 스크립트. `stop_mode=2`(hill)는 여기서만 씀 (자율주행
소프트웨어는 hill 안 씀).

## `fma/` — 펌웨어 (별도 gitlink)

AURIX TC275 소스. 이 워크스페이스 안에 gitlink(서브모듈 아님, 그냥
중첩 git repo)로 존재. **이 저장소 자체가 지금 우리 ROS2 워크스페이스도
브랜치로 같이 담고 있음** (`ros2-software` 브랜치가 이 워크스페이스,
`main`이 펌웨어).

## 기타

- `ydlidar_ros2_driver/` — YDLidar 드라이버 (현재 안 씀, RPLiDAR로 대체됨)
- `zed-ros2-wrapper/` — Stereolabs 공식 ZED2i 드라이버, 소스빌드

# ros2_ws

> 🇰🇷 한국어 번역본입니다. 원문(영어)은 [README.md](README.md)를 참고하세요. 코드 블록, 명령어, 파일 경로, 파라미터 이름은 원문 그대로 두었습니다.

_마지막 업데이트: 2026-08-03_

로버(rover)용 GPS/RTK 웨이포인트 추종 시스템입니다. u-blox ZED-F9P + NTRIP RTK, IMU(자이로+자력계) 헤딩 융합, 예측 코너링 + 물리적 실현 가능성을 고려한 코너 단축(corner-cutting)을 갖춘 Stanley 컨트롤러, CAN 모터 인터페이스, 그리고 (2026-07-26/27) 여러 제어 소스가 한 대의 차량을 안전하게 공유할 수 있게 해주는 카메라/GPS/LiDAR 회피 우선순위 중재기(arbiter)로 구성됩니다. 2026-07-28 기준으로 카메라 쪽도 실제로 동작합니다. ZED SDK + `zed_camera`(YOLOPv2 차선 추종)가 설치·빌드되었고, GPS+RTK 및 arbiter와 함께 하나의 launch 파일로 통합되었습니다. 또한 새로운 `traffic_light` 패키지(OAK-D + YOLO)가 코드만으로 `control_arbiter`에 연결되어 있습니다(OAK-D를 연결해서 테스트하지는 못함). 아래 "2026-07-28 세션"을 참고하세요. GPS는 2026-07-28에 잠시 `num_sv=0`(위성 0개)이 되었다가 같은 날 늦게 완전히 복구되었습니다(단독 실행과 GPS+ZED 통합 launch 모두에서 Float 확인, `UBX-MON-HW`로 안테나/RF 경로 정상 확인). 우연한 전원 재인가로 해결된 일시적 오류로 보며, 지속적인 하드웨어/소프트웨어 문제는 아닙니다. 아래 "GPS `num_sv=0`"을 참고하세요.

## 패키지

- **f9p_bringup** — GPS/RTK 구동(`ublox_gps_node` + `rtk_bridge.py` + 오도메트리/TF).
- **waypoint_follower** — 웨이포인트 기록/추종, Stanley 컨트롤러, CAN 드라이버, `control_arbiter`(카메라/GPS/회피 우선순위 전환), 시각화, 주행 로깅. 통합 launch 파일이 두 개 있습니다. `integrated_drive.launch.py`(전부 한 번에)와 `post_gps_drive.launch.py`(2026-07-29 신규, GPS를 제외한 전부 — ZED의 USB3 EMI가 GPS 수신을 떨어뜨리는 것을 피하려고 GPS를 먼저 단독으로 띄우기 위한 용도, 아래 "전체 실행 방법" 참고).
- **rtk_bridge** — 독립 실행형 NTRIP→시리얼 RTCM 중계 스크립트(ROS 패키지가 아니며 `package.xml` 없음). `f9p_bringup`에서 사용합니다.
- **obstacle_avoidance** — 실제 HENES 차량용 LiDAR(RPLiDAR A3M1) 장애물 회피 상태 머신(CLEAR→AVOID→PASS→RETURN→CLEAR). `~/Downloads/obstacle_avoidance`에서 가져와 2026-07-27에 `control_arbiter`와 통합했습니다(아래 참고). 하드웨어/실행 방법은 해당 패키지의 `README.md`에 자세히 있습니다.
- **handsfree_ros2_imu** — HandsFree HFI-A9 IMU 드라이버로, [3bdul1ah/handsfree_ros2_imu](https://github.com/3bdul1ah/handsfree_ros2_imu)에서 clone했습니다. `/handsfree/imu`로 `sensor_msgs/Imu`를, `/handsfree/mag`로 `sensor_msgs/MagneticField`를 발행합니다.
- **zed_camera** (2026-07-28 신규) — ZED2i + YOLOPv2 차선 추종 노드. `yolopv2_zed_rpm_node`(현재 사용 중, `LaneTracker` 기반: EMA 방식 반차선 폭 학습, 좌/우 히스테리시스, 차선이 하나만 보여도 처리, 커브 자동 감속, `~/lane_valid`와 `~/steering_deg`를 고정 20Hz 타이머로 발행)가 실제로 실행되는 노드이고, `yolopv2_zed_node`(예전 슬라이딩 윈도우 방식)는 남겨뒀지만 쓰지 않습니다. 가중치는 `src/zed_camera/weights/yolopv2.pt`에 있습니다. 설치/빌드 과정은 아래 "2026-07-28 세션"을 참고하세요.
- **traffic_light** (2026-07-28 신규) — OAK-D + YOLO 신호등 검출기. 독립 스크립트(`~/Downloads/test__sunny/test_sunny.py`)를 정식 ROS2 노드(`traffic_light_node.py`의 `test_sunny_node`)로 옮겼습니다. `control_arbiter`의 새 `traffic_light` 이벤트 존 타입이 쓰도록 `/traffic_light`(`std_msgs/String`, `"GO"`/`"STOP"`)를 발행합니다. **아직 하드웨어 테스트 전**입니다. 이번 세션에는 OAK-D가 연결되어 있지 않았고 코드만 통합했습니다(depthai/ultralytics 미설치. 둘 중 하나가 없거나 카메라를 찾지 못해도 launch가 죽지 않고 정상적으로 대기 상태로 들어갑니다). 아래 "2026-07-28 세션"을 참고하세요.
- **zed-ros2-wrapper** — Stereolabs 공식 ZED2i ROS2 드라이버로, [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper)에서 clone해 소스로 빌드했습니다. `zed_camera`가 구독하는 카메라 토픽을 발행합니다.
- **pcan_tools**, **fma**, **ydlidar_ros2_driver** — 이번 세션에서는 건드리지 않았습니다.

## 전체 실행 방법

각 구성 요소는 단독으로 실행해도 잘 동작합니다(아래 "개별 실행" 참고). 필요한 가시성에 맞는 방식을 고르세요.

### GPS + RTK (필수)

```bash
source ~/ros2_ws/install/setup.bash
ros2 run ublox_gps ublox_gps_node --ros-args \
  --params-file /home/a/ros2_ws/src/f9p_bringup/config/ublox_rover.yaml
```
다음 구성 요소를 시작하기 전에 로그에 `configured successfully`가 뜰 때까지 기다리세요(콜드 리셋 후 재연결은 정상이며 약 15초 걸릴 수 있습니다).

```bash
python3 /home/a/ros2_ws/src/rtk_bridge/rtk_bridge.py
```
이렇게 실행하면 NavSatFix를 리맵 없이 `/fix`로 발행합니다.

또는 `f9p_bringup`의 launch로 둘을 함께 실행할 수 있습니다(`ublox_gps_node`를 시작하고 2초 뒤에 `rtk_bridge`를 시작하며, `/fix`를 `/ublox_gps_node/fix`로 리맵합니다).
```bash
ros2 launch f9p_bringup f9p_rover.launch.py
```

### IMU (선택, 빠른 헤딩 갱신 + 즉시 부트스트랩 추가)

```bash
ros2 run handsfree_ros2_imu hfi_a9_ros2 --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=921600
```
`waypoint_follower_node`는 IMU가 실행 중이면 자동으로 사용합니다(기본값 `use_imu:=true`, `use_magnetometer:=true`). 아래 IMU 섹션을 참고하세요. 필수는 아닙니다. IMU가 없으면 `waypoint_follower_node`는 IMU가 생기기 전과 똑같이 순수 GPS 이동 기반 헤딩으로 되돌아갑니다.

### 웨이포인트 기록

```bash
ros2 run waypoint_follower waypoint_recorder_node --ros-args \
  -p gps_topic:=/fix \
  -p output_file:=/home/a/ros2_ws/src/waypoint_follower/waypoints/new_path.csv \
  -p min_waypoint_distance_m:=1.0
```
Ctrl+C를 누르면 저장됩니다. `output_file`이 이미 있으면 백업 없이 덮어씁니다.

`waypoints/`에 현재 기록된 코스는 다음과 같습니다. `my_path.csv`(67개 점, 첫 실제 기록), `new_path.csv`(25개 점, 작은 L자 코너 코스로 이번 세션의 컨트롤러 튜닝 테스트 대부분에 사용), `sample_waypoints.csv`(실제 장소가 아닌 더미 자리표시 값이므로 이 코스로는 주행하지 마세요).

### 웨이포인트 추종

```bash
ros2 run waypoint_follower waypoint_follower_node --ros-args \
  -p gps_topic:=/fix \
  -p waypoints_file:=/home/a/ros2_ws/src/waypoint_follower/waypoints/new_path.csv \
  -p enable_control:=true
```
(GPS를 `f9p_bringup`의 launch로 시작했다면 리맵되므로 `gps_topic`은 `/ublox_gps_node/fix`입니다.) `enable_control:=false`(기본값)는 CAN 명령을 보내지 않는 시각화 전용 모드입니다. 1초에 한 번씩 다음과 같이 출력합니다.
`target_idx=... steer=...deg speed=...m/s | CAN: rpm=... enable=... stop_mode=...`.
종료할 때 주행 로그 CSV를 `~/drive_log_<timestamp>.csv`로 저장합니다.

### 시각화 (선택)

```bash
ros2 run waypoint_follower mpl_viz_node
```
RViz 대신 쓸 수 있는 순수 matplotlib 방식입니다. 경로, 웨이포인트, 목표점, 차량 위치/헤딩, 조향각을 보여줍니다. `waypoint_follower/markers`를 읽습니다.

### 묶음 launch

`ros2 launch waypoint_follower gps_rtk_waypoint.launch.py`는 `f9p_bringup`의 GPS+RTK 스택 + `waypoint_follower_node` + `mpl_viz_node`를 함께 시작합니다(IMU 드라이버는 제외이므로 위에 나온 대로 따로 시작하세요). 인자: `waypoints_file:=...`(기본값 `waypoints/my_path.csv`), `enable_control:=true`.

**멀티 소스(카메라/GPS/회피) 구성**(2026-07-27, 자세한 내용은 아래 참고)에는 대신 `integrated_drive.launch.py`를 쓰세요. 이 launch는 `control_arbiter`를 추가하고, GPS 노드를 `publish_can_directly:=false`로 실행해서 arbiter만 CAN에 쓰도록 합니다.
```bash
ros2 launch waypoint_follower integrated_drive.launch.py
```

**ZED의 USB3 링크가 올라오기 전에 GPS가 Float/Fixed 상태로 깨끗하게 동작해야 하는 경우**(EMI 문제로 확인됨, 아래 "USB3-EMI 우려" 참고)에는 GPS를 먼저 단독으로 시작하고, fix를 잡은 뒤에 나머지를 올립니다.
```bash
ros2 launch f9p_bringup f9p_rover.launch.py
# Float/Fixed가 될 때까지 기다린 다음, 다른 터미널에서(GPS는 계속 실행 상태로 둠):
ros2 launch waypoint_follower post_gps_drive.launch.py
```
`post_gps_drive.launch.py`(2026-07-29 신규)는 `integrated_drive.launch.py`에서 GPS/RTK 구동만 뺀 것입니다. 인자는 같고 나머지 노드도 같습니다. GPS를 재시작하지 않으므로 잃을 fix가 없습니다(u-blox 수신기는 호스트가 시리얼 포트를 열어두었는지와 상관없이 계속 추적합니다). 실행하기 전에 ZED가 물리적으로 꽂혀 있는지 확인하세요. `zed_wrapper`는 카메라 열기를 약 28초 동안 재시도한 뒤 자동 재시작 없이 영구적으로 종료됩니다.

### CAN (`enable_control:=true`일 때만 필요)

```bash
sudo ip link set can0 up type can bitrate 500000
```
`can_driver.open_bus()`는 이것이 이미 올라와 있다고 가정하며 직접 설정하지 않습니다.

### 상태 확인

```bash
ros2 topic hz /ublox_gps_node/fix        # should be 20Hz (or /fix if run standalone)
ros2 topic echo /navpvt --field flags    # 1=no RTK, 64=Float, 128/131=Fixed
ros2 topic echo /navpvt --field num_sv   # satellite count
ros2 topic hz /rxmrtcm                   # confirms RTCM is actually reaching the receiver
```

## GPS/RTK 파이프라인 — 이번 세션에서 찾은 근본 원인

**기본 `ros-humble-ublox-gps` 드라이버에는 RTCM용 ROS subscriber가 아예 없습니다**(`rtcm_msgs` 의존성이 없음을 `package.xml`과 헤더에서 직접 확인했습니다). 그래서 `/rtcm` 토픽에 발행하는 ROS 네이티브 NTRIP 클라이언트는 아무에게도 닿지 않습니다. 해결책은 `rtk_bridge.py`입니다. NTRIP에서 RTCM을 받아 바이트를 그대로 시리얼 포트에 쓰는 단순 스크립트로, 이 세션 전에 RTK Fixed(`flags=131`)가 확인된 검증된 조합입니다. `f9p_bringup/f9p_rover.launch.py`는 `ublox_gps_node`와 `rtk_bridge.py`를 함께 실행하며, `rtk_bridge.py`는 `TimerAction`으로 `ublox_gps_node` 2초 뒤에 시작합니다. "configured successfully"를 기다릴 필요는 없는 것으로 확인했지만, 둘을 정확히 같은 순간(지연 0)에 시작하면 `ublox_gps_node`의 시리얼 읽기가 안정적으로 깨집니다(확인: 몇 초 안에 `End of file` 에러가 폭주). 따라서 동시에 포트를 여는 충돌을 피하려면 짧은 고정 지연이 여전히 필요합니다. 또 `rtk_bridge.py`의 초기 GGA 읽기(`read_latest_gga_from_gps`)는 이제 `SerialException`을 잡아서 크래시하는 대신 하드코딩된 `INIT_LAT`/`INIT_LON` 위치로 대체합니다. 일시적인 시리얼 읽기 실패가 치명적이어서는 안 되기 때문입니다.

`f9p_bringup/config/ublox_rover.yaml`에서는:
- `rate: 20.0`과 `save:` 블록을 추가해 EEPROM에 유지되도록 했습니다(없었기 때문에 전원을 껐다 켤 때마다 20Hz가 1Hz로 되돌아갔습니다).
- `usb.in`/`usb.out` 프로토콜 마스크 — 수신기는 USB로 연결되는데 USB는 `uart1`과 프로토콜 설정이 따로 있습니다. `uart1.in`에만 RTCM3가 켜져 있어서 USB 포트 수준에서도 RTCM이 조용히 버려지고 있었습니다.
- 동작이 확인된 baud는 **19200**입니다.
- **`device`는 이제 고정된 by-id 심볼릭 링크를 가리킵니다**
  (`/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00`.
  `rtk_bridge.py`의 `GPS_PORT`와 `f9p_rover.launch.py`의 `device` 인자 기본값에도 같은 수정을 적용했습니다). 일반 `/dev/ttyACM0` 대신 이렇게 한 이유는, USB 연결이 끊기면(아래 "EOF" 참고) `ttyACM1`, `ttyACM2` 등으로 다시 잡힐 수 있고, 그러면 일반 경로는 아무것도 가리키지 않게 되는 반면 by-id 심볼릭 링크는 실제 장치를 계속 따라가기 때문입니다. IMU(`/dev/ttyUSB0`)도 같은 종류의 위험이 있지만 아직 이 수정이 필요했던 적은 없습니다. 그런 일이 생기면 IMU의 by-id 경로는
  `/dev/serial/by-id/usb-Silicon_Labs_HandsFree_IMU_USB_to_UART_Bridge_Controller_0001-if00-port0`입니다.
- **`U-Blox ASIO input buffer read error: End of file, 0`**: 확인된 원인은 USB 연결이 실제로 끊겼다가 새로운 `/dev/ttyACM*` 번호로 다시 연결되는 것입니다(실제 장애 중에 `lsusb`/`ls /dev/serial/by-id/`로 확인). 위의 by-id 수정 덕분에 *다음* 실행에서는 장치를 자동으로 다시 찾지만, 끊김이 처음부터 발생하는 것 자체를 막지는 못합니다. 이것이 계속 반복된다면 케이블/연결/전원 문제이므로 물리적으로 추적해볼 가치가 있습니다(허브 없이 직접 USB 연결을 시도해 보세요).

## 웨이포인트 추종기 — 컨트롤러 변경 사항

이번 업데이트(2026-07-25) 기준 현재 기본값입니다(**2026-07-26에 대체됨 — `steer_limit_deg=14.3`, `curve_lookahead_m=6.0`과 그 이유는 아래 "2026-07-26/27 세션" 섹션을 참고하세요**): `stanley_k=0.3`, `cruise_rpm=140`, `min_curve_rpm=50`, `steer_limit_deg=30`, `steer_sign=-1`, `wheelbase_m=0.735`, `curve_lookahead_m=2.0`, `curve_lead_margin=1.5`, `curve_angle_for_min_rpm_deg=40`, `heading_lookback_m=0.15`.

- `steer_limit_deg` → **30** (사용자가 확인한 차량의 실제 최대 조향각. 그 전에는 보수적인 20이었음). **알고 보니 틀렸습니다.** 차량의 *실제* 최대 조향각은 14.3°이며, "30"은 펌웨어 자체의(보정이 잘못된) 최대 조향 개념이었지 실제 바퀴 각도가 아니었습니다. 아래를 참고하세요.
- `steer_sign` → **-1** (실제 차량에서 조향이 반대로 되어 있었고, 이 파라미터가 그것을 뒤집기 위해 존재합니다).
- `waypoint_arrival_radius_m` → **0.5** (*마지막* 웨이포인트의 도착/정지 판단에만 영향을 주며, 중간 구간의 추종 정밀도에는 영향이 없습니다).
- `target_idx`/`steer`/`speed`와 실제 CAN으로 나가는 값(`rpm`/`enable`/`stop_mode`)을 터미널에 1초에 한 번씩 출력하는 로그를 추가했습니다.
- **직진 폴백(Straight-driving fallback)**: `self.yaw`가 아직 `None`(아직 헤딩 없음)일 때는 `stop_mode=1`로 영원히 멈춰 있지 않고 직진(`steer=0`, `cruise_rpm`)합니다. 이전에는 정지해 있는 차량이 전혀 움직일 수 없었습니다. 헤딩이 GPS 이동으로만 얻어지는데 주행하려면 먼저 헤딩이 필요했기 때문입니다(교착 상태). 지금은 IMU/자력계가 헤딩을 즉시 부트스트랩해 주므로(IMU 섹션 참고) 대부분 의미가 없어졌지만, 둘 다 실행되지 않을 때를 위한 폴백으로 남겨두었습니다.
- `mpl_viz_node.py`(순수 matplotlib으로 만든 RViz 대체)를 추가하고, `gps_rtk_waypoint.launch.py`를 전부 묶도록 다시 작성했습니다(위의 "전체 실행 방법" 참고).

### 곡률 기반 속도 + 예측 조향

`stanley_controller.py`의 `lookahead_path_curve()`는 현재 목표점에서 **고정 거리**(`curve_lookahead_m`, 미터 단위이며 고정 웨이포인트 개수가 아님)만큼 앞으로 걸어가며 헤딩 변화를 누적합니다. 그래서 짧은 거리에 몰려 있는 실제 코너는 웨이포인트가 얼마나 촘촘하게 나뉘어 있든 급하게 읽히고, 같은 거리에 걸쳐 퍼져 있는 완만한 굴곡은 그렇게 읽히지 않습니다. 고정 점 개수 방식의 lookahead는 이런 성질이 없습니다(테스트 결과: 개수를 늘리면 완만한 굴곡의 각도까지 부풀려져 의미가 없어짐).

이 곡률 측정값을 서로 독립적인 두 곳에서 사용합니다.
- **`_curvature_scaled_rpm()`** (waypoint_follower_node.py): 앞으로 나올 각도가 `curve_angle_for_min_rpm_deg`에 가까워질수록 `cruise_rpm`을 `min_curve_rpm` 쪽으로 낮춥니다.
- **예측 헤딩 블렌딩** (`stanley_control()`의 `turning_radius_m` 파라미터, `wheelbase_m`/`tan(steer_limit_deg)`로 계산 — `_turning_radius_m()`): 각도 θ만큼의 선회를 그 반경으로 마치는 데 필요한 선행 거리는 `radius * θ`이고, 여기에 `curve_lead_margin`을 곱합니다(>1.0이면 순수 물리적 최소치보다 더 일찍 꺾기 시작해서 액추에이터/필터 지연의 여유를 둠 — 아래 참고). 급한 굴곡에서 그 거리 안으로 들어오면 **헤딩 오차 항에 대해서만** 선회 후 방향으로 블렌딩합니다(횡방향 오차(cross-track error)는 여전히 실제 현재 구간의 헤딩을 사용하며 바뀌지 않음). 기본 Stanley는 예측(preview)이 없어서 속도가 있으면 "너무 늦게" 꺾고, 이것이 코너 바깥쪽으로 크게 벗어나는 현상으로 나타납니다.

**실제 주행 분석**(`drive_log_20260724_171240.csv`, 당시 cruise_rpm=140, stanley_k=0.5, curve_lead_margin=1.1): 코스는 완주했지만 계획보다 17% 길게 달렸고, 횡방향 오차 RMSE가 0.57m였으며 코너에 집중되어 있었습니다(코너에서 평균 0.63m, 다른 곳은 0.39~0.49m). 조향이 ±30° 한계에 붙어 있던 시간이 10.5%였습니다. 진동이 아니라 "크게 벗어나고 늦게 반응한다"는 전형적인 양상입니다(조향은 샘플당 ≤9° 변화, 횡방향 오차 부호는 전체 주행에서 단 한 번만 바뀜). 대응: `curve_lead_margin` 1.1→1.5, `curve_angle_for_min_rpm_deg` 60→40(이 코스의 코너는 lookahead 거리에 따라 ~40~57°까지만 나와서 60은 `min_curve_rpm` 하한을 거의 발동시키지 못했음), `stanley_k`는 0.3으로 되돌림(이 속도에서는 0.5를 테스트한 적이 없고, 게인은 횡방향 오차 항에만 영향을 주고 헤딩 오차에는 영향이 없어서 "늦게 반응한다"의 해결책은 엄밀히 아니었음 — margin/angle 변경만으로 크게 도는 현상이 해결됐는지 게인과 별개로 재테스트가 필요함).

**알려져 있지만 아직 처리하지 않은 실제 환경의 간극**: `required_lead_m`은 순수한 기하학(`radius * angle * margin`)이고 속도 항이 없습니다. 하지만 지령한 조향각에 *도달*하는 데는 실제 시간이 걸립니다. `_apply_lowpass()`의 소프트웨어 평활화(`lowpass_fc_hz=2.0` 기준 ~95% 안정까지 ~0.24초)와 물리 액추에이터 자체의 응답 지연이 모두 있습니다. 속도가 높을수록 같은 시간 지연이 바퀴가 목표 각도에 실제로 도달하기 전에 더 많은 거리를 소모하므로, 고정 margin만이 아니라 속도가 높을수록 선행 거리도 늘어나야 합니다(`required_lead_m += v * settling_time`). 아직 구현하지 않았습니다. 관련 사항: (실시간 무슬립/운동학 논의를 통해) 자전거 모델에서 선회 반경 자체는 속도와 무관함을 확인했습니다. 실제로 속도가 중요해지는 것은 반응 시간을 거리로 환산할 때입니다. 기하학 자체 때문은 아닙니다.

**역시 빠져 있는 것**: 실현 가능성 검사가 없습니다. "이 코너는 이 속도/반경으로는 말 그대로 돌 수 없다"를 감지해서 보정하는(예: 복구 전략으로 일부러 코너를 가로지르는) 로직이 없으며, 같은 비례 법칙을 그대로 적용하기 때문에 정말 불가능한 경우 추종이 나빠집니다. `_curvature_scaled_rpm()`의 감속은 휴리스틱(임계 각도/하한값을 임의로 선택)이며, 이 코너를 완주하는 데 필요한 실제 최소 속도 계산에서 도출한 것이 아닙니다.

**아직 고쳐지지 않은 것**: `stanley_control()`의 최근접 웨이포인트 탐색은 여전히 전역입니다(매 주기마다 전체 경로 탐색). 코스가 자기 자신과 교차하면(예: 교차로를 두 번 지남) `target_idx`가 경로 순서와 무관하게 기하학적으로 가장 가까운 점으로 튈 수 있습니다. 이 노드는 카메라/차선 추종 시스템이 추종을 잃었을 때의 폴백으로 쓰이기 때문에 코스 중간의 임의 위치에서 스스로 위치를 찾아야 한다는 점에서 중요합니다. 논의된 해결책: 이전 `target_idx`가 없을 때(첫 활성화)에만 전체 탐색을 하고, 그 이후 주기에서는 이전 `target_idx` 주변의 윈도우만 탐색합니다.

### 헤딩 추정 — 샘플 개수 기반이 아니라 거리 기반

`_smoothed_heading()`은 이전에 정확히 `heading_window_n`개 GPS 샘플 전(고정 개수)과 위치를 비교하고, 신뢰하려면 순변위가 `heading_min_movement_m` 이상이어야 했습니다. 원래 기본값(n=4, 1.0m)에서는 아예 동작하는 데만도 ~5 m/s가 필요했습니다. 두 파라미터를 하나의 **`heading_lookback_m`**(기본값 0.15m)로 대체했습니다. `position_history`를 거슬러 올라가며 그만큼 이상 떨어진 점을 찾고 그 방위를 사용합니다. 위의 `curve_lookahead_m`과 같은 이유입니다. 최소 속도 요구가 없고, 실제로 움직이는 동안은 모든 GPS fix마다 갱신됩니다("lookback"은 매 주기 새로 계산되는 슬라이딩 거리 윈도우이며 "N cm 기다렸다가 한 번 갱신"하는 스로틀이 아닙니다). 정지해 있는 동안에는 매 주기 같은 답으로 계속 다시 계산합니다(고정된 `position_history[-1]` + 고정된 lookback 점 → 같은 결과). 실질적으로 "고정"과 동등하지만, 건너뛰는 것이 아니라 *재계산*이라는 점은 알아둘 만합니다.

참고: `position_history` 자체의 추가 임계값(`on_fix`, 1mm)은 실제 GPS 노이즈(RTK Fixed는 현실적으로 수 mm~1~2cm)보다 훨씬 작습니다. 정확히 중복된 점만 걸러낼 뿐 노이즈 필터가 아닙니다. 실제 노이즈 제거는 1mm 추가 임계값이 아니라, 그 노이즈 바닥보다 훨씬 큰 `heading_lookback_m`(15cm)에서 나옵니다.

## IMU (헤딩 융합)

헤딩은 이전에는 GPS 이동 기반으로만 계산했습니다. 이제 IMU가 GPS fix 사이에 빠른 갱신을 제공하고, (신규) 자력계를 통해 정지 상태에서도 즉시 헤딩을 제공합니다.

- 하드웨어: **HandsFree HFI-A9**, `/dev/ttyUSB0` (`HandsFree_IMU_USB_to_UART_Bridge_Controller`로 인식되며 고정 by-id 경로도 있음 — 위의 GPS 섹션 참고). Baud **921600**.
- 드라이버: GitHub에서 `handsfree_ros2_imu`를 clone했습니다(apt 패키지는 없음). 포트에서 받은 원시 바이트와 대조해 프레임 프로토콜(`0xAA 0x55` 헤더, 길이 바이트가 타입 ID를 겸함, CRC-16/MODBUS 체크섬)을 역으로 검증했고, 드라이버 소스와 정확히 일치합니다.
- `python3-transforms3d`(apt, 0.3.1, 제거된 NumPy 1.x API를 사용)가 이 장비의 pip NumPy 2.2.6에서 깨지는 문제를 고쳐야 했습니다. matplotlib 수정과 같은 종류의 문제입니다. `pip install --user --upgrade transforms3d`(→0.4.2) + `sudo apt install ros-humble-tf-transformations`(설치되어 있지 않았음)로 해결했습니다.
- **자이로**(`angular_velocity.z`, `on_imu()`에서 적분): GPS fix 사이를 메웁니다. `self.yaw`가 이미 초기화되어 있어야 합니다(자이로 각속도만으로는 절대 헤딩을 부트스트랩할 수 없음). 정지 상태 벤치 테스트: ~10분 실행에서 ±0.3° 이내, 두 번째 ~5분 실행에서 -0.16°~+0.13° 이내 — 정지 시 원시 각속도는 0 근처(~±0.001~0.005 rad/s)이며 눈에 띄는 일정한 바이어스는 없었습니다. **실제 주행 중에는 테스트하지 않았습니다**(진동 때문에 아마 더 나빠질 것).
- **자력계**(`on_mag()`, 신규): 정지 상태에서도 즉시 *절대* 헤딩을 제공합니다. GPS 이동 헤딩과 달리 부트스트랩 문제가 없습니다. Imu 메시지에 들어 있는 IMU 자체 온보드 융합 `orientation`도 자력계를 기준으로 하지만, 이번에는 의도적으로 *사용하지 않았습니다*. 원시 `MagneticField`를 직접 써서 이 코드베이스 자체의 편각 보정과 캘리브레이션 파라미터가 깔끔하게 적용되게 했습니다. 자북 ≠ 진북(GPS/웨이포인트 좌표계)이므로 `magnetic_declination_deg`(기본값 -8.0, 대한민국 대략값 — **정확한 장소/날짜로 확인하세요**, 예: NOAA 편각 계산기)가 이를 보정합니다. `mag_offset_x`/`mag_offset_y`는 hard-iron 캘리브레이션입니다(아래 `mag_calibrate.py` 참고). `mag_mount_offset_deg`는 IMU 장착 정렬 오차/축 규약 오차를 흡수하며, 알려진 방위를 향하게 한 뒤 `self.yaw`가 일치할 때까지 조정하는 방식으로 경험적으로 튜닝합니다.
  - 사용자 확인: 이 IMU는 **모터와 완전히 분리되어** 장착되어 있습니다(동적 전류 EMI 우려 없음). **강철 프레임** 위나 근처에 있는 것은 다른 문제로, 대체로 정적인 왜곡입니다(전류에 의한 것이 아니라 철 재질에서 오는 hard-iron 오프셋 + soft-iron 휘어짐). IMU가 프레임에 대해 단단히 고정되어 있고 캘리브레이션을 이미 최종 장착 위치에서 한다면 제대로 된 캘리브레이션으로 *대부분 보정할 수 있습니다*. Soft-iron(타원형 왜곡이며 회전/스케일 행렬이 필요)은 아직 구현하지 않았고 hard-iron(오프셋)만 구현했습니다.
  - `mag_calibrate.py`(`ros2 run waypoint_follower mag_calibrate`): 차량을 한 바퀴(360°) 돌리거나 주행시키면서(yaw만 — 차량이 수평을 유지하므로 기울이거나 뒤집을 필요가 없고, 8자 운동은 손에 들고 하는 3축 캘리브레이션용이므로 불필요) `/handsfree/mag`에서 x/y의 최솟값/최댓값을 기록합니다. Ctrl+C를 누르면 `mag_offset_x`/`mag_offset_y`에 넣을 오프셋을 출력하고, x/y 편차 비율로 보아 한 바퀴를 다 돌지 못했거나 soft-iron 왜곡이 있을 것으로 보이면 경고도 같이 출력합니다.
  - `imu_drift_test.py`(`ros2 run waypoint_follower imu_drift_test`): GPS 보정이 전혀 없는 순수 자이로 적분 드리프트 벤치 테스트 독립 실행 도구입니다. 위의 정지 상태 벤치 수치에 사용했습니다.
- `use_imu`/`use_magnetometer`(둘 다 기본값 `true`), `imu_topic`(`/handsfree/imu`), `mag_topic`(`/handsfree/mag`)이 이 모두를 제어합니다. 둘 중 하나를 `false`로 하면 남은 것으로 깔끔하게 폴백합니다(둘 다 끄면 순수 GPS 헤딩). IMU/자력계가 아예 연결되지 않아도 노드가 잘 실행됨을 확인했습니다(구독이 그냥 한 번도 발화되지 않을 뿐 에러는 없음).

## 2026-07-26/27 세션: 조향 캘리브레이션, 코너 물리, 멀티 소스 arbiter

### 조향이 잘못 보정되어 있었음 — 실제 최대 조향각은 30°가 아니라 ~14.3°

`Steering.c`(AURIX 펌웨어, `STM_Interrupt_1_KIT_TC275_LK` 프로젝트 — 별도의 Windows 장비에 있고 AURIX Development Studio에서 편집하며 **이 저장소에는 없음**)에는 `STEER_MAX_ANGLE = 30.0f`가 하드코딩되어 있고, 이를 `POT_LEFT..POT_CENTER..POT_RIGHT` 전체 포텐셔미터 범위에 1:1로 매핑합니다. `POT_CENTER`도 틀려 있었습니다(`2048`이었고 실제 중앙이 아니었음).

다음 순서로 진단하고 수정했습니다.
1. **`POT_CENTER` 재보정**(`2048` → `~1700`, 정확한 값은 사용자가 Windows 쪽 `Steering.c`에 설정): 주행 로그 방식을 사용했습니다. 기존 중앙값으로 직진 주행해서 직선을 유지하는 데 필요한 정상 상태 P 컨트롤러 보정량(옛 단위로 ~-6.33°)을 찾고, *옛* 상수로 `new_center = angleToPot(-6.33)`을 계산해서 사용했습니다. 동작 확인: 이후 직선 주행 로그에서 횡방향 오차 RMSE **0.105m**가 나왔습니다(재중앙 전에는 ~1.07m).
2. **실제 최대 조향각을 실측**: 풀 락(full lock) 상태로 두 바퀴를 돌며 `waypoint_recorder_node`로 GPS를 기록하고, 점들에 원을 피팅했습니다(Kasa 대수 피팅). 반경 **2.874m**(잔차 ~1cm, 두 바퀴가 0.5% 이내로 일치). `atan(wheelbase_m / radius) = atan(0.735 / 2.874) = 14.35°` — 즉 "30"(펌웨어 자체의 풀 락 개념)을 지령해도 실제 바퀴 각도는 30°가 아니라 **14.3°**밖에 나오지 않습니다. 이것은 왼쪽 방향에 대해서만 검증했습니다(재중앙 때 `POT_LEFT`는 건드리지 않았음). **오른쪽은 별도로 검증하지 않았고**, 재중앙 후 두 구간(`POT_LEFT-POT_CENTER` vs `POT_CENTER-POT_RIGHT`)이 비대칭이 되었다면 다를 수 있습니다.
3. **`POT_LEFT`/`POT_RIGHT` 레이블을 제대로 교체하려고 시도**했습니다(실제 차량과 반대로 되어 있음 — 확인됨). 하지만 이렇게 하니 `angleToPot`/`potToAngle`의 분기 로직(숫자상 `LEFT > CENTER > RIGHT`를 가정)이 깨졌고 연결하자마자 바퀴가 바로 돌아버렸습니다. **그대로 두었습니다**: 레이블은 현실과 반대지만 값은 동작하므로, 분기 로직도 같은 변경에서 함께 고치지 않는 한 레이블을 "고치지" 마세요.

**ROS/Linux 쪽에 적용한 수정**(`can_driver.py`):
```python
FIRMWARE_STEER_MAX_ANGLE_DEG = 30.0
TRUE_STEER_MAX_ANGLE_DEG = 14.3
CAN_STEER_SCALE = FIRMWARE_STEER_MAX_ANGLE_DEG / TRUE_STEER_MAX_ANGLE_DEG  # ≈2.098
```
`waypoint_follower_node`의 `steer_limit_deg` 기본값은 이제 `TRUE_STEER_MAX_ANGLE_DEG`(14.3, 이전에는 30)이므로 모든 Stanley/선회 반경 계산이 **실제 물리 각도(degree)** 기준으로 이루어집니다. 모든 CAN 송신은 `can_driver.send_control_true_deg(bus, rpm, true_steer_deg, enable, stop_mode)`를 거치며, 이 함수가 프레임을 패킹하기 직전에 `CAN_STEER_SCALE`을 곱합니다. 이 변환이 일어나는 곳은 *오직 여기뿐*입니다. **이 차량의 조향과 통신하는 새 코드는 반드시 `send_control_true_deg`를 써야 하며, 원시 각도로 `send_control`을 직접 호출하면 안 됩니다.** 그렇지 않으면 의도한 각도의 대략 절반만 조용히 지령하게 됩니다. 예외: 조향값이 이미 원시 펌웨어 스케일인 코드(예: `obstacle_avoidance`의 `avoid_steer_left/right = -30/30`)는 의도적으로 보정 없이 `send_control`로 보내며, 이들도 `send_control_true_deg`를 거치게 해서는 **안 됩니다**(보정이 이중으로 적용되거나 잘못 적용됨). 아래 "obstacle_avoidance 통합"을 참고하세요.

**`Steering.c`를 올바른 `STEER_MAX_ANGLE`로 다시 플래시하면**(14.3, 또는 비대칭이면 오른쪽 실제 값), 이 CAN_STEER_SCALE 관련 코드는 전부 무의미해지므로 죽은 코드로 남겨두지 말고 삭제해야 합니다. `CAN_STEER_SCALE`/`TRUE_STEER_MAX_ANGLE_DEG`를 검색하세요.

**실제로 있지만 아직 고치지 않은 조향 거칠음**: `Steering.c`의 `POT_TOLERANCE=30` + `MIN_PWM_PERCENT=15.0f`가 함께 작용해서 데드밴드 경계에서 눈에 띄는 "뱅뱅(bang-bang)" 떨림을 일으킵니다(아주 작은 초과 오차가 부드럽게 올라가는 대신 곧바로 15% PWM으로 튐). 진단: 먼저 `MIN_PWM_PERCENT`를 낮추고(떨림의 *강도*에 대한 더 직접적인 해결), 그 후에도 떨림의 *빈도*가 문제일 때만 `POT_TOLERANCE`를 건드립니다. 아직 적용하지 않았습니다.

### GPS 속도 계산 버그 (수정됨)

`waypoint_follower_node.on_fix()`가 `dt`를 GPS 메시지 자체의 `header.stamp`가 아니라 `self.get_clock().now()`(ROS 콜백이 실제로 실행된 시점의 벽시계 시간)로 계산했습니다. ROS 콜백 스케줄링 지터(fix가 연달아 처리되거나 흩어져서 처리됨) 때문에 일부 샘플에서 `dt`가 너무 작아져 `self.speed`가 부풀려졌습니다. 실제 로그에서 확인: 실제 최대 속도가 ~8km/h인 차량에서 **샘플의 34%가 8km/h 초과**로 읽혔고, 불가능한 11.5km/h까지 치솟았습니다. 이것은 Stanley의 횡방향 오차 보정(`v`가 보정 항의 분모에 있음)을 직접 약화시키므로 단순히 로깅 표시만 틀린 버그가 아니었습니다. 수정: `now = Time.from_msg(msg.header.stamp)`.

### 코너링: 물리적 실현 가능성 검사 + 풀 락 오버라이드 + S커브 수정

기존의 예측 조향 블렌딩(위의 "곡률 기반 속도 + 예측 조향" 참고)은 코너를 돌 수 있다고 가정하고 그저 더 일찍 꺾기 시작할 뿐이며, "이 차량이 이 코너를 애초에 돌 수 있는가"는 묻지 않습니다. `stanley_controller.py`에 실제 답을 추가했습니다.

- `lookahead_path_curve()`가 이제 걸어간 호(arc) 거리도 함께 반환하므로, 호출자가 `path_radius_m = arc_dist_m / turn_angle_rad`를 계산할 수 있습니다. 기록된 경로 자체의 국소 곡률입니다.
- `path_radius_m < turning_radius_m`이면(이제 올바른 `steer_limit_deg=14.3`에서 나온 차량의 실제 최소값) 해당 코너는 **어떤 예측 타이밍으로도 물리적으로 불가능**합니다. 차량이 돌 수 있는 것보다 더 급한 선회는 "더 일찍 꺾기 시작"해도 해결되지 않습니다. 이 경우 예측 선행 거리 안에 들어오면 `stanley_control()`은 비례 법칙을 완전히 건너뛰고 선회 방향으로 **풀 스티어링 락**을 지령합니다. 달성 가능한 최선의 응답임이 증명되며, 도달 불가능한 목표와 싸우는 대신 자연스럽게 코너 안쪽을 가로지르게 됩니다. 실제 주행 로그에서 검증: 풀 락 분기가 정확히 코너 진입 시점(idx 11-14, 18-19)에, 횡방향 오차가 커지기 *전에*(0.02-0.14m에서 시작) 발동했습니다. 즉 설계대로 늦게 반응하는 대신 일찍 확정합니다.
- **S커브 버그를 찾아서 수정**: 선회 각도 누적이 방향과 상관없이 모든 구간의 헤딩 변화에 `abs()`를 씌워서 더했기 때문에, 오른쪽 20°→왼쪽 20° 굴곡이 상쇄되어 ~0이 되는 대신 "같은 방향 40°"로 읽혔습니다. 급함을 과대평가하고, *다음* 굴곡의 풀 락 방향을 잘못 고를 가능성도 있었습니다. 수정: 이제 선회 방향이 반전되는 순간 누적을 멈춥니다(단위 테스트로 검증: 30° 다음 (-40)° 합성 굴곡이 이제 70°가 아니라 올바르게 30°로 보고됨).
- `curve_lookahead_m`을 **2.0 → 6.0**으로 올렸습니다. 스캔 윈도우는 기록된 웨이포인트에서 코너가 물리적으로 *퍼져 있는* 길이 이상이어야 하고, 그렇지 않으면 전체 선회 각도를 적게 셉니다. `new_path.csv`의 실제 코너로 확인: ~5.6m에 걸쳐 ~89° 퍼져 있음(급한 점 회전이 아니라 꽤 완만한 곡선) — 2.0m로는 전체를 보기에 한참 부족했습니다.
- 위의 모든 수정 이후 실제 결과(`new_path.csv`, 전체 코스): **처음으로 완주했습니다**(이전 시도들은 마지막 웨이포인트에서 멈췄음). 횡방향 오차는 여전히 코너 중간에서 ~1.2-1.3m까지 흔들리지만, 이제 이후에 부드럽게 *회복*됩니다(끝에는 ~0.2m까지 내려감). 재중앙 전처럼 영구적인 오프셋에서 정체되지 않습니다.

### RTK 마운트포인트: VRS가 아니라 `RTK-RTCM32` 유지

초기 Fixed 획득이 느린 문제를 고치려고 `rtk_bridge.py`의 `MOUNTPOINT`를 `VRS-RTCM34`(NGII 캐스터)로 바꿔봤습니다. 그 문제는 해결됐지만 새 문제가 생겼습니다. VRS는 우리가 마지막으로 보낸 GGA 위치(`GGA_INTERVAL_SEC=10.0`) 주변에서 보정값을 합성하므로, *움직이는* 로버가 마지막으로 보고한 자기 위치를 앞질러 가서 Fixed를 얻자마자 거의 바로 잃어버립니다. `RTK-RTCM32`(물리적 기준국 하나, 초기 Fix는 더 느리지만 주행 중에는 열화되지 않음)로 되돌렸습니다. VRS를 다시 검토한다면 `GGA_INTERVAL_SEC`을 2~3초로 낮추는 것을 가장 먼저 시도해 볼 만합니다.

### `control_arbiter` — 카메라 / GPS / idx 이벤트 존 / LiDAR 회피 우선순위 전환

새 노드(`waypoint_follower/arbiter_node.py`)입니다. 이제 여러 구성 요소가 같은 차량을 운전하려고 하는데 **CAN 제어 프레임을 쓰는 프로세스는 단 하나여야** 하기 때문입니다. 독립된 두 쓰기 주체는 서로 싸웁니다. 이번 세션 초반 GPS 시리얼 포트에서 힘들게 알게 된 것과 같은 종류의 버그입니다. 나머지는 모두 자신의 의도를 토픽으로 발행만 하고, `can_driver.send_control*`를 호출하는 것은 `control_arbiter`뿐입니다.

**우선순위(높은 순):**
1. **GPS idx 이벤트 존**(`event_zones` ROS 파라미터, `"idx_start:idx_end:type"` 문자열 리스트. `idx`는 GPS 노드가 사용하는 `waypoints_file`의 행 번호이며 헤더 제외 0부터 시작). `stop` / `gps_priority` / `avoid`는 하나의 평평한 계층입니다(서로 순위를 매기지 않음). 모두 "이 idx에서 특정한 일을 한다"는 것이며, 강제 정지든, GPS가 직접 통과해야 하는 교차로든, LiDAR 회피 존이든 마찬가지입니다. 카메라가 지금 운전 중인지와 상관없이 활성화됩니다.
2. **카메라 차선 추종** — `camera_steer_topic`(기본값 `/yolopv2_zed_node/steering_deg`)에 (a) `camera_timeout_sec` 이내의 메시지(노드가 아예 죽은 경우를 위한 데드맨 스위치) **그리고** (b) 프레임 수 히스테리시스가 모두 있을 때만 사용합니다. `camera_bad_frames_to_disable`(10)개의 연속 NaN 프레임이면 빠지고, `camera_good_frames_to_enable`(3)개의 연속 정상 프레임이면 다시 들어옵니다. arbiter 틱이 아니라 구독 콜백에서 실제 카메라 프레임(카메라는 30-70fps) 단위로 세며, 네 가지 전이를 모두 다루는 단위 테스트로 검증했습니다.
3. **GPS 웨이포인트 추종 폴백** — GPS 노드가 스스로 유효하다고 보고하는 경우(`gps_control/valid` 토픽, `self.pos`/`self.yaw`가 둘 다 알려져 있음을 반영).
4. **안전 정지**(`rpm=0, enable=0, stop_mode=1`) — 위에서 유효한 것이 없을 때.

**GPS 쪽**(`waypoint_follower_node.py`): 새 `publish_can_directly` 파라미터(기본값 `true`라서 단독 실행에는 영향 없음)를 arbiter 아래에서 실행할 때 `false`로 설정하면, 노드가 CAN을 직접 쓰는 대신 매 주기마다 `gps_control/steer_deg`, `gps_control/rpm`, `gps_control/target_idx`, `gps_control/valid`(Float32/Float32/Int32/Bool)를 발행합니다.

**카메라 쪽**: 아직 테스트할 수 없지만(이 워크스페이스에는 패키지가 없음 — 위의 패키지 참고), arbiter는 이미 `/yolopv2_zed_node/steering_deg`에 연결되어 준비되어 있습니다.

**obstacle_avoidance 통합**(2026-07-27): `obstacle_avoid_node.py`에도 같은 처리를 했습니다. 새 `write_can_directly` 파라미터(기본값 `true`)를 arbiter 아래에서는 `false`로 설정합니다. CAN 피드백(0x101/0x102)은 계속 직접 읽지만(자체 `pass_progress`/`steer_fault_check`에 필요 — CAN 버스를 여러 *읽기* 주체가 동시에 쓰는 것은 괜찮고, 충돌하는 것은 쓰기뿐), *쓰기*는 하지 않습니다. 기존 모니터링 토픽(`/avoid/state`, `/avoid/cmd_steer`, `/avoid/cmd_rpm`)이 대신 arbiter의 입력이 됩니다. arbiter는 `/can_bridge/enable`을 발행하고(GPS idx가 `avoid` 존 안에 있는 동안에만 True — `obstacle_avoid_node` 자체의 arm/disarm 토픽 및 disarm 시 `CLEAR`로 리셋하는 기존 동작과 일치), 그 `cmd_steer`/`cmd_rpm`을 **보정 없이**(원시 펌웨어 스케일, 예: -30/30 — 위의 CAN_STEER_SCALE 경고 참고) `can_driver.send_control`로 그대로 전달합니다. 회피 노드가 발행하지 않으면(실행 중이 아니거나 `avoid_timeout_sec`=1.0 초과) 존을 눈 감고 통과하는 대신 안전하게 정지로 폴백합니다.

**리뷰 중에 지적되었지만 아직 적용하지 않은 것**:
- `obstacle_avoidance`의 `turn_radius_override` 파라미터(기본값 `0`, `wheelbase/tan(30°)=1.273m`의 *이론적* 반경 사용을 의미)는 **실측값** `2.874m`로 설정해야 합니다. 현재는 차량이 실제보다 두 배 넘게 급하게 돌 수 있다고 가정하고 있어서, 장애물을 피하는 데 필요한 조향각(`alpha`)을 과소평가합니다.
- `obstacle_avoidance`의 `avoid_rpm=30`은 현재 `cruise_rpm=15`보다 *더 빠릅니다*. 안전 관점에서는 거꾸로입니다(실제 회피 기동 중에 더 느려야 반응 여유가 커지며, 위 코너링 섹션의 "반응 시간이 거리를 소모한다"는 지적과 일치). 어차피 알려진 S커브 테스트 코스에서만 실행되므로, 회피 상태 머신 전체에서 하나의 느린 일정 속도로 통일하는 것을 논의했지만 아직 적용하지 않았습니다.
- `obstacle_avoidance/README.md`에 문서화된 "음수/왼쪽 조향이 실제 차량에서 동작하지 않는다"는 문제(2026-07-24자, 이번 세션의 재보정 이전) — **이번 세션의 주행 로그에서 이미 실제 음수 `current_angle` 피드백이 정상적으로 변하는 것을 볼 수 있어서**(예: `-23.7~13.2`, `-30.0~12.9`), `POT_CENTER` 재중앙의 부수 효과로 이미 해결되었을 가능성이 매우 높습니다. obstacle_avoidance 코드로 특별히 독립 재검증하지는 않았습니다.

**새 launch 파일**: `waypoint_follower/launch/integrated_drive.launch.py` — GPS+RTK + `waypoint_follower_node`(`publish_can_directly:=false`) + `control_arbiter` + `mpl_viz_node`를 하나의 launch에 담았습니다. 상단 근처의 `EVENT_ZONES` 리스트가 idx 존을 설정하는 곳입니다(기본값은 빈 리스트). 카메라 `Node(...)` 블록은 안내와 함께 전부 주석 처리되어 있으며, `zed_camera` 패키지가 생기면 주석을 풀기만 하면 됩니다.

### 두 번째 차량("노란 차") — 조향 중앙 캘리브레이션 요청됨, 아직 하지 않음

두 번째의 다른 차량에서도 같은 `POT_CENTER` 오보정이 의심됩니다. 요청된 절차: 조향을 0°로 지령한 채(해당 차량의 저수준 제어를 어떤 방식으로든 사용) 직진 주행하면서 `waypoint_recorder_node`로 GPS를 기록한 다음, 그 CSV를 보내주면 됩니다. 위와 같은 원/곡률 피팅 방식으로 중앙이 얼마나 틀어져 있는지 역산합니다. **막힌 이유**: 해당 차량의 `POT_LEFT`/`POT_CENTER`/`POT_RIGHT`/`wheelbase_m` 값(아직 제공되지 않음)과 실제 시험 주행/로그입니다.

## 2026-07-28 세션: ZED SDK 설치, 카메라 통합, launch 크래시 수정, GPS `num_sv=0` 장애

### NVIDIA 드라이버 우회

ZED SDK 5.4.1은 CUDA ≥13.0이 필요해서 드라이버를 올려야 했습니다. 드라이버 595로 올렸더니 `gdm3`가 깨졌습니다(`prime-select on-demand` 상태였고 디스플레이 매니저가 시작되지 않음). 드라이버 **580**으로 다시 내려서 복구했는데, 580이 이미 CUDA 13.0/13.3을 문제없이 지원했기 때문에 595는 결국 필요 없었습니다. 나중에 다시 해야 한다면 595로 바로 뛰지 말고, 현재 드라이버가 이미 지원하는 것부터 먼저 시도해 보세요.

### ZED SDK + 카메라 파이프라인 설치

- ZED SDK 5.4.1을 설치했고, 진단 도구로 ZED2i가 감지되고 CUDA가 동작함을 확인했습니다.
- YOLOPv2 추론용으로 `torch`/`torchvision` 2.13.0+cu130을 설치했고, GPU에서 실행됨을 확인했습니다(처음에는 CPU에 머물러 있었음 — 아래 수정 참고).
- `zed-ros2-wrapper`를 공식 Stereolabs 저장소에서 clone해서 소스로 빌드했습니다(위의 패키지 참고).
- **NumPy를 1.26.4로 고정(2.x에서 다운그레이드)**: `cv_bridge` + `python3-opencv`는 numpy<2가 필요한데, `pyzed`/`transforms3d`(IMU 스택)는 numpy≥2를 원합니다. 둘을 모두 만족하는 버전을 하나 고르는 방식으로는 해결할 수 없는 실제 충돌입니다. IMU 작업은 일단 우선순위를 낮추라는 명시적 지시("IMU는 나중에 생각하자 GPS만 잘 되면 됨")에 따라 numpy<2 쪽으로 해결했습니다. **이는 numpy를 다시 올리기 전까지 IMU/헤딩 융합 스택(`handsfree_ros2_imu`, `transforms3d`)이 깨져 있을 가능성이 높다는 뜻이며**, 재검증하지 않았습니다.
- `setuptools` 회귀: torch를 설치하면서 `setuptools`가 조용히 83.0.0으로 올라가 `colcon build`가 깨졌습니다(`error: option --uninstall not recognized`). 수정: `pip3 install "setuptools<80"`(79.0.1로 정착). 앞으로 더 새로운 setuptools를 의존성으로 끌어오는 패키지를 `pip install`하면 이 문제가 다시 생길 수 있습니다. `colcon build`가 갑자기 `--uninstall` 에러와 함께 실패하기 시작하면 `pip3 show setuptools`부터 확인하세요.

### `zed_camera` 패키지 조립

소스 코드는 사용자가 제공한 zip(`~/Downloads`의 `zed_camera(3).zip`)에서 가져왔습니다. 앞선 두 사본인 `zed_camera.zip`과 `zed_camera (2).zip`은 손상되었거나 잘린 다운로드였고(`unzip -t`가 "End-of-central-directory signature not found"를 보고해서 확인), `(3)`이 유효하고 가장 최신이었습니다. 정식 ROS2 패키지로 조립했습니다.

- `zed_camera/utils/utils.py`가 zip에 빠져 있었습니다. 원래 들어 있어야 할 것과 바이트 단위로 동일하게, 업스트림 [CAIC-AD/YOLOPv2](https://github.com/CAIC-AD/YOLOPv2) 저장소(`utils/utils.py`)에서 그대로 가져왔습니다.
- **`setup.cfg`가 없었습니다.** `ros2 run zed_camera yolopv2_zed_rpm_node`가 "No executable found"를 보고한 근본 원인입니다. 이것이 없으면 colcon이 console_scripts를 `ros2 run`이 찾는 `install/zed_camera/lib/zed_camera/`가 아니라 `install/zed_camera/bin/`에 설치합니다. 표준적인 두 줄 수정(`[develop] script_dir=...` / `[install] install_scripts=...`, 둘 다 `$base/lib/zed_camera`를 가리킴)을 추가하고 클린 리빌드했습니다.
- **카메라가 CPU에 머무름(~70fps 대신 ~2fps)**: 명령줄에서 `-p device:=cuda`를 넘긴 것이 원인입니다. YOLOPv2 utils의 `select_device()`는 `CUDA_VISIBLE_DEVICES`를 파라미터 문자열 그대로 설정하므로, `"cuda"`는 유효한 장치 인덱스가 아니라서 모든 GPU를 조용히 숨깁니다. 파라미터를 아예 생략해서 해결했습니다(기본값은 실제 GPU 인덱스인 `"0"`).
- `weights/yolopv2.pt`를 명시적 요청에 따라 `~/Downloads/yolopv2.pt`에서 `src/zed_camera/weights/`로 (복사가 아니라) 이동했습니다.
- `_infer_loop()` 안에 FPS 상한(`max_fps` 파라미터, 기본값 50)을 추가했습니다. 루프 한 번을 시간 측정해서, 추론이 일찍 끝나면 `1/max_fps`의 나머지를 sleep으로 채우므로 빠른 GPU에서도 노드가 필요 이상으로 빨리 돌지 않습니다.
- 차선 하나만 보이는 경우 + 카메라 끊김 처리(둘 다 구현 전에 논의된 미해결 질문이었음): `yolopv2_zed_rpm_node`의 `LaneTracker` 방식은 이미 차선 하나가 보이지 않는 경우를 처리합니다(EMA로 각 차선의 반폭을 학습하고, 매 프레임 두 차선이 모두 필요하다고 요구하는 대신 보이는 쪽 + 그 폭으로 보이지 않는 쪽을 추정). 카메라 끊김은 한 계층 위의 `control_arbiter`의 기존 프레임 수 히스테리시스로 처리합니다(연속 10개의 나쁜 프레임이면 카메라에서 폴백되며, ~50fps에서 ~0.2초입니다. 원래 요청된 ~0.5초보다 즉각에 가깝지만, 더 빠른 폴백이 더 느린 것이 아니라 엄밀히 더 안전하므로 그대로 두었습니다).

### `integrated_drive.launch.py` — 한 번의 launch로 전체 스택

최종 구조입니다(권위 있는 버전은 파일 자체인 [integrated_drive.launch.py](src/waypoint_follower/launch/integrated_drive.launch.py)를 참고): ZED 래퍼가 즉시 시작하고, GPS+RTK(`f9p_bringup`)는 `TimerAction(period=20.0)`으로 감싸서 그 뒤에 시작하며, `waypoint_follower_node`는 `publish_can_directly:=false`로 실행하고, `camera_node`는 `can_enable:=false`로 `yolopv2_zed_rpm_node`를 실행하며, `control_arbiter` + `mpl_viz_node`가 전체를 묶습니다. 2026-07-26/27 arbiter 설계를 그대로 따르되, 이제는 주석 처리된 자리표시자 대신 실제 카메라 노드가 들어갑니다.

**버그를 찾아서 수정: 타입이 있는 launch 인자에는 `ParameterValue`가 필요**합니다. `LaunchConfiguration(...)`은 항상 문자열로 해석됩니다. parameters 딕셔너리에 그대로 넘기면 DOUBLE이나 BOOL 파라미터용 launch 인자가 STRING으로 도착하고, rclpy가 시작 시점에 이를 거부합니다(`InvalidParameterTypeException`). 노드가 launch 후 1초 안에 죽습니다. `camera_mode_rpm`(float, `arbiter_node.py`)에서 이 문제를 겪었고, 고치기 전에 직접 만든 `--params-file`로 정확한 예외를 재현해서 확인했습니다. 고치는 과정에서 `enable_control`(bool, `waypoint_follower_node.py`)에서도 똑같은 잠재 버그를 발견했습니다. 관찰된 실제 실행 한 번에서는 우연히 크래시하지 않았지만 STRING-BOOL 불일치 자체는 있었으므로, 실제로 크래시할 때까지 기다리지 않고 선제적으로 고쳤습니다. 이제 둘 다 감쌌습니다.
```python
"camera_mode_rpm": ParameterValue(LaunchConfiguration("camera_mode_rpm"), value_type=float),
"enable_control": ParameterValue(LaunchConfiguration("enable_control"), value_type=bool),
```
8초간의 실제 launch 테스트로 검증했습니다. 로그에 `died`/`InvalidParameterType`/`Traceback`이 없었고, `arbiter_node`가 예상대로 `[MODE CHANGE] None -> safe_stop`, 이어서 `[MODE CHANGE] safe_stop -> camera`를 로그에 남겼습니다. **문자열이 아닌 ROS 파라미터에 값을 넘기는 새 `LaunchConfiguration`은 모두 같은 `ParameterValue(..., value_type=...)` 래핑이 필요합니다.** 이런 종류의 버그는 다시 만들어내기 쉽습니다.

### `status_check.sh` — 한 번에 보는 상태 스냅샷

새 스크립트 [scripts/status_check.sh](src/waypoint_follower/scripts/status_check.sh)입니다. RTK fix 상태(`/navpvt` flags), GPS 웨이포인트 추종기 유효성/목표 idx, 카메라 차선 유효성/조향각, 장애물 회피 상태, arbiter의 `active_source`를 출력합니다. 예전에는 터미널 5개 이상이 필요했던 것을 `ros2 topic echo --once` 한 번의 훑기로 보여줍니다. `ros2 topic echo` 자체의 "WARNING: topic [...] does not appear to be published yet"(stderr가 아니라 stdout으로 나감)는 깔끔한 `(no data)`로 걸러냅니다. `control_arbiter`에도 `active_source` 토픽(String, 매 주기 발행)과, 활성 소스가 실제로 바뀔 때마다 스로틀 없이 남기는 `[MODE CHANGE] {old} -> {new} | {detail}` 로그(그 외에는 1초에 한 번 하트비트로 스로틀)를 추가했습니다. 위의 launch 크래시 검증을 빠르게 확인할 수 있었던 것이 이것 덕분입니다.

### GPS `num_sv=0` — 발견했지만 아직 해결하지 못함

통합 launch를 테스트하던 중 GPS가 fix를 잡지 못했습니다. 처음 가설: 앞서의 GPS/RTK 브릿지 동시 USB 시작 문제와 같은 종류의 버그이고, 이번에는 GPS와 ZED 자체의 USB 열거/CUDA 초기화가 충돌하는 것이라고 봤습니다. launch를 시차를 두어 실행해서 "고쳤습니다"(앞선 ZED 지연 시도에서 순서를 바꿔달라는 명시적 요청에 따라 `f9p_bringup`을 ZED보다 20초 늦게 시작. 이 launch 파일 변경은 아직 적용되어 있음).

**이 가설은 그 후 반증되었습니다**: GPS를 완전히 단독으로(카메라 없이, launch 파일 없이, `ublox_gps_node` + `rtk_bridge.py`를 직접 실행) 테스트해도 하늘이 트인 야외에서 여전히 `num_sv=0`이었습니다. 이번에는 소프트웨어/USB 경합 문제가 아님을 확인했습니다.
- USB 장치 경로/by-id 심볼릭 링크: 정상.
- `ublox_gps_node`와 `rtk_bridge.py`: 둘 다 실행 중이고, `/navpvt`와 `/fix`가 실제로 20Hz로 발행됨(멈추거나 걸려 있지 않음).
- RTCM 보정: NTRIP 캐스터에서 ~7Hz로 흐르고 있음(수신기에 도달).
- 안테나 케이블: 사용자가 헐거워지지 않았음을 확인.
- 그런데도 줄곧 `num_sv=0`.

**미해결로 남김** — 제안된 수신기 전원 재인가를 시도하기 전에 사용자가 노트북 충전을 위해 자리를 떴습니다. 재개할 때의 다음 단계: u-blox 수신기 전원을 먼저 껐다 켭니다(배제하기 가장 쉬운 것). 그래도 안 되면 안테나/커넥터 자체(헐겁지 않다고 해도 접촉이 불안정한 연결이 명백히 헐거워 보이지는 않을 수 있음)나 수신기 내부 오류를 의심합니다. `integrated_drive.launch.py`의 GPS 20초 지연 변경은 아마 유지하는 것이 맞고(해롭지 않으며, 이번 세션 초반의 다른 실제 버그에 대한 합리적인 수정이었음) 이 문제의 해결책은 **아닙니다**.

**제기되었다가 같은 날 배제된 가설**: 역전류 / 안테나 바이어스 라인 단락(수신기가 같은 SMA 동축을 통해 능동 안테나에 3-5V를 공급함)이 안테나의 LNA나 수신기의 안테나 전원 출력단을 태웠을 수 있다는 가설입니다. 관찰된 증상 패턴(통신은 정상이고 RF/위성 추적만 죽음)을 설명하고, 수신기를 한 번 떨어뜨린 적이 있다는 점과도 맞아떨어졌을 것입니다. u-blox가 스스로 보고하는 `UBX-MON-HW` 메시지(`a_status`/`a_power`)로 확인할 수 있습니다. `f9p_bringup/config/ublox_rover.yaml`에서 명시적으로 켰고(`publish.mon.hw: true`) `status_check.sh`에 추가했습니다. **첫 시도에서는 토픽 이름이 틀렸습니다**(`/ublox_gps_node/monhw` — ROS2는 토픽에 노드 이름을 자동으로 접두하지 않고 네임스페이스만 붙입니다. 이미 접두사 없이 쓰이는 `/navpvt` 등과 같이 올바른 `/monhw`로 수정).

**수정 후 결과**: `a_status=1`(UNKNOWN), `a_power=2`(UNKNOWN) — *동시에 실제 Float fix를 얻는 중*(`flags=67`). 이 조합은 오히려 역전류/손상 가설을 다시 배제합니다. 여기서 `UNKNOWN`은 안테나 감시 모니터 기능 자체가 켜져 있지 않다는 뜻이고(`UBX-CFG-ANT`를 설정한 적이 없음 — 안테나 감지 회로가 없는 보드나 수동 안테나 구성에서는 흔함), 무언가가 손상되었다는 뜻이 아닙니다. 또한 RF 프런트엔드가 정말로 탔다면 Float(실제 반송파 위상 신호 수신이 필요함)에 도달하는 것은 손상 여부와 관계없이 물리적으로 아예 불가능합니다. **따라서 이 테스트 기준으로 안테나/RF 경로는 정상임이 확인되었습니다.** 어제의 `num_sv=0`의 원인이 무엇이든 영구적인 하드웨어 손상이 아니라 일시적인 것(콜드 스타트 TTFF, 그 순간의 하늘 가시성, 또는 수신기 내부 오류)이었을 가능성이 매우 높습니다. 그대로 두었습니다(`UBX-CFG-ANT` 활성화를 더 쫓지 않음 — 필요하다는 증거가 없음).

**USB3-EMI 우려(ZED가 GPS를 방해) — 2026-07-29 확인됨**: ZED2i는 USB3 SuperSpeed 카메라이며, GPS L1(1575.42MHz)과 겹치는 1~6GHz 전자기 간섭의 알려진 실제 원인입니다. GPS가 단독으로 동작하게 된 뒤에도 *통합* launch에서는 실제 위험으로 지적되었습니다. 같은 날(2026-07-28)의 첫 테스트에서는 해소된 것처럼 보였습니다. 통합 launch에서 GPS가 Float에 도달했고 단독 실행보다 나쁘지 않았지만, 그 테스트는 "GPS가 fix를 *하나라도* 얻는가"만 확인했을 뿐 같은 품질에 도달하는지는 확인하지 않았습니다. **후속 테스트(2026-07-29)에서 실제 효과가 발견되었습니다**: GPS 단독으로 안정적으로 Fixed(`flags=131`)에 도달하는 지점에서, 통합 GPS+ZED launch는 Float(`flags=67`)에 머물렀고 Fixed로 진행하지 않았습니다. 즉 EMI가 fix 자체를 아예 막은 것이 아니라 SNR을 충분히 떨어뜨려서 반송파 위상 모호정수 결정(Fixed)을 막았고, 노이즈에 더 관대한 Float는 여전히 잘 동작한 것입니다. **해결 확인**: GPS 안테나 케이블과 ZED의 USB 케이블을 물리적으로 더 멀리 떨어뜨려 배선하자 해결되었고, 분리하자 Fixed가 정상적으로 잡혔습니다. **조치 사항**: 테스트 중만이 아니라 최종 차량 배선/하네스 배치에서도 이 두 케이블을 실용적으로 가능한 한 멀리 떨어뜨려 두세요.

**`num_sv=0` 최종 정리(2026-07-28)**: 근본 원인은 직접 포착하지 못했지만(발생한 순간에 실시간 진단이 돌고 있지 않았음), 정황 증거는 **우연한 전원 재인가**가 실제 해결책이었음을 가리킵니다. 사용자가 노트북을 충전하러 자리를 뜨기 전 마지막 제안이 "수신기 전원을 껐다 켜보라"였고, 그 휴식 시간 동안 GPS의 USB 케이블을 뽑았다 꽂았다면 정확히 그렇게 되었을 것입니다. 이후 모든 것을 다시 테스트했을 때 깨끗했습니다. 단독 Float(야외에서 거의 즉시), `UBX-MON-HW`로 안테나/RF 하드웨어 정상 확인, 통합 GPS+ZED launch에서도 다시 Float. 해결된 것으로 처리했습니다. **`num_sv=0`이 재발하면**: 안테나 하드웨어나 EMI를 다시 조사하기 전에 GPS의 USB 연결을 먼저 뽑았다 꽂으세요(저렴하고, 여기서 정황상 효과가 있었던 것과 일치).

### 2026-07-29 후속: 실제 테스트 중 발견해서 수정한 실제 버그 두 개

- **`mpl_viz`에서 차량/웨이포인트가 ~14,000km 떨어져 보임**: 근본 원인은 `waypoint_follower_node.on_fix()`에 있었습니다. `self.frame`(로컬 ENU 원점)이 실제 fix가 있는지와 상관없이 받은 *첫 번째* `NavSatFix` 메시지에 고정되었습니다. `ublox_gps_node`는 위성을 하나도 잡기 전에도 `lat=lon=0.0`(Null Island)과 `status.status=STATUS_NO_FIX`로 `NavSatFix`를 계속 발행합니다. 그 첫 메시지가 GPS가 fix를 잡기 전에 도착하면(흔함 — 노드 시작 후 GPS는 시간이 필요함) 원점이 (0,0)에 영구적으로 고정되고, `self.frame`은 한 번만 설정되므로 이후 실행 내내 그 원점 기준으로 그려지는 모든 웨이포인트가 ~14,000km 떨어진 곳에 찍혔습니다. 차량 마커는 그 순간 자기 위치도 *마찬가지로* (0,0)이었기 때문에 "멀쩡해" 보였을 뿐입니다(옳아서가 아니라 우연). **수정**: `on_fix()`가 이제 `msg.status.status < NavSatStatus.STATUS_FIX`이면 `self.frame`을 건드리지 않고 일찍 반환하므로, 원점은 실제 fix가 있는 메시지로만 설정됩니다. Float와 Fixed를 모두 받아들임을 확인했습니다(ublox의 `ublox_firmware7plus.hpp`는 Float RTK를 `STATUS_FIX`로, Fixed RTK를 `STATUS_GBAS_FIX`로 매핑하며 둘 다 `>= STATUS_FIX`이고 진짜 `STATUS_NO_FIX`만 거부됨). 따라서 Fixed가 아니라 Float를 얻는 즉시 플롯이 시작됩니다.
- **`control_arbiter`가 `LaneLost`에서 카메라 모드를 실제로 벗어나지 않음**: `arbiter_node.py`의 프레임 수 히스테리시스는 카메라의 `steering_deg`에 `math.isnan()`을 검사해서 "차선을 잃었는가"를 판단했는데, `yolopv2_zed_rpm_node`의 `_publish_timer_cb()`는 발행 전에 항상 `NaN`을 `0.0`으로 바꿔 넣습니다(다운스트림이 선로상에서 NaN을 보지 않게 하려고). 즉 arbiter의 NaN 검사는 절대 걸릴 수 없었고, `camera_bad_streak`은 한 번도 증가하지 않았으며, 차선이 죽었음이 확인된 상태에서도 카메라를 계속 신뢰하고(steer=0.0, 즉 직진) 운전했습니다. 실시간으로 발견: 사용자가 카메라를 손으로 들고 `/yolopv2_zed_node/lane_valid`(별도의, 올바르게 보고되는 Bool 토픽)가 `False`가 되는 것을 지켜봤지만 `active_source`는 `camera`로 유지되었습니다. **수정**: arbiter가 이제 `camera_lane_valid_topic`(새 파라미터, 기본값 `/yolopv2_zed_node/lane_valid`)을 직접 구독하고, 조향 값에서 추론하는 대신 그 Bool로 good/bad 연속 횟수 히스테리시스를 구동합니다. `_on_camera_steer`는 이제 신선도 데드맨 스위치의 타임스탬프만 갱신합니다. 같은 손으로 든 테스트로 수정 후 검증: 카메라에서 벗어나는 모드 전환이 이제 올바르게 일어납니다.
- **USB3-EMI(ZED vs. GPS) 실제로 확인됨** — 전체 기록은 위의 "GPS `num_sv=0`" 섹션을 참고하세요. 케이블 분리로 2026-07-28 A/B 테스트가 놓쳤던 "Float에 갇혀 Fixed에 도달하지 못함" 증상이 해결되었습니다(그 테스트는 "fix를 *하나라도* 얻는가"만 확인했고 fix 품질은 보지 않았음).

### `traffic_light` 패키지 — OAK-D + YOLO 빨간불 검출, `control_arbiter`와 통합 (코드만 통합, 하드웨어 테스트 전)

사용자에게는 이미 작성해서 내려받은 별도의 독립 스크립트가 있었습니다(`~/Downloads/test__sunny/test_sunny.py` + `test_sunny.pt` 가중치 + 사용 문서). OAK-D 카메라(ZED가 아니라 `depthai` 사용) + `ultralytics` YOLO로 빨간불을 검출하고, 30프레임 버퍼로 디바운스하며(최근 30프레임 중 25개 이상에서 높은 신뢰도의 빨간불이 보이면 STOP), `cv2.imshow` + CSV 로깅과 함께 블로킹 `while True` 루프로 실행될 예정이었고 ROS는 전혀 관여하지 않았습니다.

**이번 세션에는 OAK-D가 연결되어 있지 않았습니다.** 명시적으로 코드 통합만 하는 작업이었고(`depthai`/`ultralytics`도 설치되어 있지 않았음) 하드웨어 테스트가 아니었습니다. 새 패키지로 옮겼습니다.

- **새 패키지 `traffic_light`**(`src/traffic_light/`): `zed_camera`에 합치지 않았습니다. 카메라 하드웨어(OAK-D vs ZED2i)와 pip 의존성 집합(`depthai`, `ultralytics` vs `pyzed`/`torch`)이 다르므로 의도적으로 분리해 두었습니다.
- `traffic_light_node.py`(실행 파일 `test_sunny_node`, 원래 문서의 이름과 일치): 원래 스크립트와 같은 버퍼/임계값 로직을 쓰지만, depthai 프레임 획득 + YOLO 추론 루프를 ROS spin 스레드를 막는 대신 백그라운드 스레드에서 돌리고(`yolopv2_zed_rpm_node`의 `_infer_loop` 패턴을 따름), `cv2.imshow` 창에 그리기만 하는 대신 매 프레임 `/traffic_light`(`std_msgs/String`, `"GO"`/`"STOP"`)를 발행합니다. `cv2.imshow`는 여전히 있지만 `show_debug` 파라미터(기본값 `false`) 뒤에 두어 기본적으로 헤드리스로 실행됩니다. CSV 로깅(`log_csv` 파라미터, 기본값 `true`)은 원본과 같습니다. 모델 가중치는 `src/traffic_light/weights/test_sunny.pt`로 복사했습니다.
- **하드웨어/의존성이 없을 때 강하게 실패하지 않고 부드럽게 실패**: `ultralytics`나 `depthai`를 import할 수 없거나 `dai.Device(...)`가 OAK-D를 찾지 못하면, 노드는 크래시하는 대신 경고를 로그에 남기고 대기 상태로 머뭅니다(발행하지 않음). `arbiter_node.py`의 "CAN bus not available, log-only" 폴백으로서의 `can_driver`와 같은 이유입니다. 카메라가 꽂혀 있지 않고 pip 의존성도 아직 설치되지 않았는데도 `integrated_drive.launch.py`에 바로 연결해도 안전했던 이유입니다. **의존성 설치 완료**(2026-07-28, OAK-D는 여전히 물리적으로 연결 안 됨): `depthai==2.32.0.0`, `ultralytics==8.4.110`. 설치하면서 두 가지 함정을 만났고 둘 다 수정했습니다.
  - `pip3 install depthai ultralytics`는 **기본적으로 depthai 3.x를 설치**하는데, 이는 원래 스크립트(그리고 이 노드)가 작성된 v2 API와 호환되지 않는 변경입니다. v3의 `dai.Pipeline()`은 기본값이 `createImplicitDevice=True`이고 *생성 시점에* 장치에 연결하려고 하므로, 파이프라인 설명을 만드는 것만으로도(v2에서는 장치가 필요 없음) 곧바로 `RuntimeError: No available devices`가 발생합니다. 수정: 이 코드가 쓰는 v2의 `dai.Pipeline()` → `dai.Device(pipeline)` 2단계 흐름에 맞추려고 `pip3 install "depthai<3"`으로 고정했습니다(2.32.0.0에 정착). **depthai를 재설치/업그레이드하면 `<3`으로 다시 고정하세요.** 그렇지 않으면 v2 스타일의 파이프라인 코드를 v3 API에 맞게 다시 써야 합니다.
  - `ultralytics`는 전이 의존성으로 `opencv-python`(자체 pip 설치 cv2이며 현재는 완전히 다른 메이저 버전)을 끌어오고 `numpy`도 다시 2.x로 올립니다. 둘 다 `cv_bridge`가 필요로 하는 numpy<2/apt-cv2 고정과 **직접 충돌**합니다(위의 "ZED SDK + 카메라 파이프라인 설치" 참고). depthai/ultralytics를 설치한 뒤 `pip3 uninstall -y opencv-python`(그러면 `import cv2`가 `/usr/lib/python3/dist-packages/cv2...so`의 apt `python3-opencv`, 버전 4.5.4로 폴백)을 하고 `pip3 install "numpy<2"`로 다시 1.26.4에 고정하는 방식으로 해결했습니다. 이 조합에서 `cv_bridge`, `pyzed`, `torch`+CUDA가 모두 여전히 import됨을 이후에 확인했습니다. **이 정확한 순서(depthai/ultralytics 설치 → pip의 opencv-python 제거 → numpy<2 재고정)는 이들을 재설치할 때마다 반복해야 합니다.**
  - 하드웨어 없이 엔드투엔드로 동작 확인: 파이프라인이 빌드되고, 아무것도 꽂혀 있지 않으면 `dai.Device(pipeline)`가 약 5.6초(장치 검색 타임아웃) 뒤 `RuntimeError: Cannot find any device with given deviceInfo`로 깔끔하게 실패하며, `traffic_light_node.py`의 try/except가 이를 잡아서 경고를 로그에 남기고 대기 상태로 머뭅니다. 크래시 없음. 모델(`test_sunny.pt`)은 `ultralytics YOLO(...)`로 문제없이 로드되고, 클래스는 하나(`{0: 'red'}`)로 코드의 `label.lower() == "red"` 검사와 일치합니다.

  **실제 테스트 전에 아직 필요한 것**: OAK-D를 연결하고 `python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"`로 확인합니다(`[]` 대신 장치가 나와야 함).

**`control_arbiter`에 새 이벤트 존 타입 `"traffic_light"`가 추가됨**(`arbiter_node.py`): 요청받은 내용에 정확히 맞춰 설계했습니다. 기존 `stop` 타입처럼 "idx가 범위에 들어오는 순간 즉시 정지"하는 단순한 존이 아니라, idx 범위를 인식하는 동작입니다.
- 존 스펙은 여전히 `"idx_start:idx_end:traffic_light"`(다른 존 타입과 같은 3필드 형식)이지만, 이 타입에서는 `idx_end`를 특별히 **정지선** idx로 재사용합니다.
- `idx_start <= gps_idx < idx_end`(**접근 중**): `/traffic_light`가 `STOP`이면 속도를 줄입니다(`traffic_light_approach_rpm_scale` 파라미터, 기본값 `0.5`배). 하지만 조향은 건드리지 않습니다. `control_arbiter`가 원래 운전하던 카메라나 GPS(어느 쪽이든)가 정상적으로 계속 조향하고, 속도만 수정됩니다.
- `gps_idx >= idx_end`(**정지선에 도달/통과**): 여전히 `STOP`이면 완전 정지(`rpm=0`, `enable=0`, `stop_mode=1`). `GO`이면 정상 속도로 완전히 재개합니다. "그 특정 idx에서 빨간불이 아니면 다시 원래 속도로"라는 요청과 정확히 일치합니다(신호가 초록일 때 존을 평소처럼 통과하는 것과 다르지 않음).
- 이것은 다른 이벤트 존 타입과 구조적으로 다릅니다. `stop`/`gps_priority`/`avoid`는 자체 steer/rpm을 완전히 결정하지만, `traffic_light`는 카메라/GPS가 *원래 지령했을* 값을 먼저 알아야 합니다(존을 통과하는 동안 조향이 계속 동작하도록). `on_timer()` 맨 위에 `base_steer`/`base_rpm`/`base_source` 계산(camera_ok → camera, elif gps_ok → gps_fallback, 아니면 `None`)을 추가해서 `traffic_light`가 이를 읽도록 했고, 아래쪽의 기존 단순 카메라/GPS 폴백 분기도 이를 재사용하도록 단순화했습니다.
- `traffic_light_node`가 발행하지 않으면(`traffic_light_timeout_sec`, 기본값 1.0초, 데드맨 스위치) **GO가 아니라 RED로 안전하게 실패**합니다. 의도적인 것입니다. 설정되지 않았거나 크래시한 신호등 노드는, 빨간불일 수도 있는 교차로를 조용히 전속력으로 통과하는 대신 설정된 모든 존에서 차량이 서행하다가 멈추게 합니다. **명시적으로 알려둡니다**: 이 때문에 노드가 실제로 실행되어 연결되어 있지 않으면 `traffic_light` 존은 테스트에 쓸 수 없습니다(영구적으로 서행 후 정지만 함). 안전을 위한 의도적인 동작이지만, 존이 "멈춰 있는" 것처럼 보일 때 알아두면 좋습니다.
- `ros2 topic pub`으로 검증했습니다(실제 카메라/하드웨어 없이 상태 머신만 실행). 세 가지 전이가 모두 올바르게 로그에 남는 것을 확인했습니다. `RED ahead - slowing (gps_fallback) steer=2.0deg rpm=40`(가짜 80 rpm의 절반), 정지선 idx에서 `RED - stopped ... rpm=0 enable=0 stop_mode=1`, 그리고 `GO (gps_fallback) ... rpm=80`으로 전속 재개입니다. 또한 존 중간에 GPS 유효성도 만료되면 우선순위 체인을 따라 올바르게 `traffic_light_no_driver` → 안전 정지로 떨어짐을 확인했습니다.
- `integrated_drive.launch.py`에 연결: 새 `traffic_light_node` + `traffic_light_model_arg`(기본값 `src/traffic_light/weights/test_sunny.pt`)를 `LaunchDescription`에 추가했습니다. `EVENT_ZONES`는 기본적으로 빈 상태로 둡니다. 테스트할 실제 교차로 idx가 생기면(그리고 OAK-D가 실제로 연결되면) `"90:95:traffic_light"` 같은 항목을 추가하세요.

### `control_arbiter` — 카메라에 대한 GPS 횡방향 오차 거부권(veto) (2026-07-29)

실차 테스트 중에 제기된 동기가 된 문제: 카메라/GPS 우선순위 전환은 "카메라가 차선이 보인다고 *생각하는가*"(`lane_valid`)만 물었고, 카메라가 *자신 있게 틀리는* 경우(무언가를 차선으로 잘못 검출하고 높은 자기 보고 신뢰도로 기록된 선에서 벗어나 주행)를 잡을 방법이 없었습니다. `lane_valid=True`는 단지 "차선 모양의 픽셀을 찾았다"는 뜻이지 "이것이 올바른 차선이다"라는 뜻이 아닙니다.

**수정**: `waypoint_follower_node`가 이제 `gps_control/cross_track_error_m`도 발행합니다(이전에는 매 주기 주행 로그 CSV용으로 계산됐지만 토픽에는 올라가지 않았음). `control_arbiter`는 카메라가 적극적으로 운전 중일 때도 이것을 계속 감시하고, 실제 차량 위치가 기록된 웨이포인트 선에서 `camera_max_deviation_m`(기본값 반경 1.0m) 이상 벗어나면 카메라 자신의 주장과 상관없이 GPS로 강제 전환합니다.

- **단일 임계값이 아니라 히스테리시스**(`arbiter_node.py`의 `_camera_deviation_ok()`): 카메라를 다시 신뢰하려면 `camera_deviation_reenter_m`(기본값 0.5m — 진입 임계값 1.0m보다 더 좁음) 안으로 다시 들어와서 `camera_deviation_reenter_streak`개의 연속 주기(기본값 20 @ 20Hz = ~1초) 동안 유지해야 합니다. 이것이 없으면 예전의 NaN 기반 카메라 검사가 그랬을 수 있듯이 경계에서 바로 깜빡거릴(flap) 것입니다.
- **반복 위반 잠금(lockout)**: 거부권이 `camera_deviation_lockout_window_sec`(기본값 20초) 안에 `camera_deviation_lockout_count`회(기본값 3) 발동하면, 카메라를 `camera_deviation_lockout_sec`(기본값 15초) 동안 완전히 잠급니다. 근본 오류가 지속되는 카메라는 그렇지 않으면 GPS가 경계 안으로 끌어들였다가 제어를 돌려줄 때마다 즉시 다시 벗어나는 일을, 더 느리지만 여전히 반복되는 주기로 되풀이합니다. 잠금은 한 주기짜리 깜빡임만 막는 것이 아니라 그런 톱니 패턴을 멈춥니다.
- **GPS도 과신하지 않음**(설계 중에 명시적으로 제기된 우려 — GPS 자체도 일부 구간에서는 Fixed가 아니라 Float일 뿐이라 위치에 실제 오차가 있음): `cross_track_error_m`이 최근에 발행되지 않았거나(`cross_track_timeout_sec`, 기본값 1.0초) NaN이면, `_camera_deviation_ok()`는 정보가 없다는 이유로 기존 오버라이드를 해제하지도 *않고*, 새 오버라이드를 발동하지도 않습니다. 오래되었거나 없는 데이터를 기준으로 어느 쪽으로도 판단하지 않고 보수적으로 유지합니다. 고정 상수 대신 GPS 자체가 보고하는 정확도(`NavSatFix`의 `position_covariance`에서 나오는 `h_acc`, 또는 Fixed/Float 플래그)로 임계값을 스케일하는 방법도 검토했지만(구현하지 않음, 필요하면 나중을 위해 표시해 둠), 단순하게 유지하려고 지금은 하지 않기로 했습니다. Float 구간에서의 오탐 거부/오신뢰가 실제로 문제가 되면 다시 검토하세요.
- `ros2 topic pub` 가짜 데이터로 검증했습니다(실제 카메라/GPS 없이). 정상 차선 데이터에서 `gps_fallback -> camera`가 되고, 가짜 1.5m 횡방향 오차를 발행하는 즉시(1.0m 진입 임계값 초과) `camera -> gps_fallback`이 되는 것을 확인했습니다. 모드 전환 로그 줄이 예상대로 즉시 찍혔습니다.

## 2026-07-30 ~ 08-03 세션: 장애물 회피 실시간 튜닝, GPS/카메라 보강, 차선 검출 재설계, 곡률 피팅

며칠에 걸친 대규모 실차 테스트 세션입니다. 시간순이 아니라 영역별로 묶었습니다.

### `obstacle_avoidance` — 최종 튜닝 값 (실제 HENES 차량에서 실시간 튜닝)

현재 [config](src/obstacle_avoidance/config/obstacle_avoid.yaml): `front_angle_deg=50.0`, `max_consider_range=2.5`, `vehicle_width=0.8`, `turn_radius_override=2.874`(정밀한 원 피팅 측정값), `safety_margin=0.0`, `reaction_margin=0.0`, `avoid_rpm=80`, `cruise_rpm=30`, `alpha_extra_deg=0.0`, `obstacle_length=1.43`, `pass_margin=0.0`, `gps_side_bias_pts=5`.

- **`write_can_directly: false` 추가** — 아예 빠져 있어서 위험한 기본값 `true`가 적용되고 있었습니다(이 노드가 CAN을 직접 쓰고 arbiter도 CAN을 쓰면 서로 싸움). arbiter 없이 `obstacle_avoidance`를 단독 실행할 때만 `true`로 설정하세요.
- **`vehicle_width=0.0` 버그(발견하고 수정)**: 앞선 "전부 0으로 설정" 실험이 이 값을 0으로 남겨두었는데, 그러면 `_decide()`의 여유 거리 검사가 항상 통과(`C <= 0`)해서 상태 머신이 CLEAR를 절대 벗어나지 않습니다. 바로 앞에 장애물이 있어도 회피가 조용히 발동하지 않았습니다. 실제 값 0.8m로 복원했습니다.
- **`obstacle_length=0.0` 버그(발견하고 수정)**: `_solve_avoid_alpha`의 `clearance(alpha) = R*(1-cosα) + D*sinα`는 `D = obstacle_length + pass_margin`을 선회가 넘어야 하는 실제 장애물 크기로 사용합니다. `obstacle_length`가 0으로 남아 있으면 `pass_margin`만 늘렸을 때 직관과 달리 *더 약한* 선회가 계산됩니다(PASS 중의 뒤늦은 직선 드리프트에 기대는데, 이는 장애물을 실제로 지나가는 순간에는 도움이 되지 않음). 실제 값 1.43m로 복원했습니다.
- **AVOID_LEFT vs AVOID_RIGHT에 대한 GPS 기반 결정 보조**(`gps_side_bias_pts`, `gps_cross_track_topic`, `gps_bias_timeout_sec`, [obstacle_avoid_node.py](src/obstacle_avoidance/obstacle_avoidance/obstacle_avoid_node.py)): GPS 횡방향 오차가 "기록된 선으로 돌아가는 쪽"이라고 말하는 쪽에 `gps_side_bias_pts`개의 가상 LiDAR 점을 추가합니다. 원시 LiDAR의 좌/우 점 개수가 비슷할 때만 동점 해소용으로 쓰며, 명확한 LiDAR 신호를 절대 덮어쓰지 않습니다.
- RPLiDAR S2를 `rplidar_ros`(apt)로 구동 — 래퍼 launch 파일 [rplidar_s2.launch.py](src/obstacle_avoidance/launch/rplidar_s2.launch.py)가 `serial_port`를 by-id 경로로 고정합니다. 이 차량에서는 기본값 `/dev/ttyUSB0`이 실제로는 AURIX 조향 보드이기 때문입니다(LiDAR는 `/dev/ttyUSB1`).

### GPS/웨이포인트 추종기 보강

- **`stanley_v_min`(기본값 0.5)**: Stanley의 횡방향 항에 쓰이는 속도(`atan2(k*cte, max(v, v_min))`)에 하한을 두어, 정지 상태의 차량이 선에서 1cm만 벗어나도 *0이 아닌 어떤* 오차에 대해서든 풀 스티어링 락을 계산하는 버그(분모가 0에 가까움)를 고쳤습니다.
- **GPS 이상치 제거 연속 수용**(`gps_outlier_streak_accept`, 기본값 3): `gps_outlier_threshold_m`보다 큰 GPS 점프가 한 번 발생하면 필터링된 위치가 영원히 얼어붙었습니다. 진짜 점프(모드 전환 인계, GPS가 다시 따라잡음) 이후의 모든 실제 위치가 *역시* 얼어붙은 오래된 점에서 멀었기 때문에 계속 거부되었고, 필터가 빠져나올 수 없는 루프였습니다. 이제 이만큼 연속으로 거부되면 그 점프를 실제로 받아들입니다.
- **장애물 회피 후 `stanley_k` 부스트**(`stanley_k_boost=1.0`, `stanley_k_boost_duration_sec=2.0`): `obstacle_avoid_node`의 RETURN 상태가 끝난 직후 횡방향 이득을 일시적으로 올려서, 순항용으로 튜닝된 느린 기어들어가기 대신 기록된 선으로 더 빨리 복귀합니다.
- **`loop_waypoints` 파라미터**(기본값 `false`): 폐곡선 코스(`halla1_closed.csv`)용입니다. 마지막 웨이포인트에서의 도착-정지 검사를 건너뛰어 차량이 계속 주행합니다. `stanley_control()`은 이미 매 주기 전역에서 가장 가까운 웨이포인트를 다시 찾으므로(단조 증가 인덱스가 아님) idx가 자연스럽게 0 쪽으로 되감깁니다. 다른 변경은 필요 없습니다.
- **`halla1_closed.csv`**: 실제로 기록한 한 바퀴(`halla1.csv`, 2440점)에는 루프의 시작/끝이 이어지지 않는 ~55m 틈이 있었습니다. 직선으로 추측하는 대신 사용자가 기록한 실제 구간(`halla1_gap.csv`)을 그 틈에 이어 붙였습니다. 이음매 거리가 작고(1.96m / 0.41m), 합친 경로의 다른 곳에는 비정상적인 점프가 없음을 확인했습니다.
- **카메라가 경로를 벗어나는 것이 확인된 구간에 대한 `gps_priority` 이벤트 존**: 주행 후 횡방향 오차 분석(기록된 코스를 주행 로그와 비교)에서 `halla1_closed.csv`의 idx 1427-1465와 1535-1557이 카메라가 기록된 선이 아닌 다른 곳으로 이어지는 차선을 자신 있게 따라가서 횡방향 오차가 최악의 지점에서 5.25m까지 벌어지는 구간임을 찾았습니다. LOST/차선 없음 상황이 아니라, 카메라가 실제로 존재하지만 잘못된 차선 위를 적극적으로 운전하고 있었습니다. `post_gps_drive.launch.py`의 `EVENT_ZONES`가 이제 이를 덮도록 `1440:1464`와 `1520:1560`에서 GPS 전용으로 강제합니다.

### `zed_camera` — 차선 검출 오탐 제거 + 곡률 피팅

[yolopv2_zed_rpm_node.py](src/zed_camera/zed_camera/yolopv2_zed_rpm_node.py)의 `LaneTracker.update()`를 연석, 횡단보도 줄무늬, 과속방지턱 선을 실제 차선 가장자리로 오인하지 않도록 재설계했습니다.
1. 연결 성분별 후보(미리 좌/우로 나눈 풀이 아님).
2. 간격이 그럴듯한 차선 폭보다 작은 인접 후보를 병합하고, 이미지 중앙에 더 가까운 후보를 남김(연석은 실제 차선 바로 바깥, 즉 프레임 가장자리에 더 가까움).
3. 정리된 후보 중에서 폭을 검증하는 쌍 탐색(EMA로 학습한 반차선 폭에 가장 가까운 쌍이 선택됨).
4. 단일 측면 폴백은 그 측면에 정확히 하나의 후보만 남았을 때만. 그 외에는 추측하지 않고 LOST.
5. `lane_max_candidates`(기본값 4) — 그럴듯한 차선 범위 안에 이 개수만큼의 후보가 남아 있으면 잡동사니(횡단보도/과속방지턱)로 보고, 하나를 고르려 하지 않고 곧바로 LOST로 처리.

**속도 기반 조향 감쇠**(`speed_damp_gain=0.15`, `speed_damp_min_scale=0.4`): `self.lane.max_steer_deg`는 현재 속도(ZED VIO보다 GPS 속도 우선, GPS 속도가 오래되면 폴백)를 이용해 고정된 기준값에서 매 프레임 다시 스케일됩니다. GPS의 Stanley v-분모 감쇠를 따라 한 것이며, 높은 rpm에서 카메라가 차선 쪽으로 급하게 꺾으며 오버슈트하는 문제를 고칩니다.

**2차 곡선 피팅 / 곡률 예측**(`lane_num_bands=4`, `lane_min_fit_bands=3`, `lane_curvature_gain_deg=8.0`, `lane_curvature_max_deg=10.0`): 위의 단일 밴드 후보 검출 로직을 `_scan_region()`으로 분리해서 ROI의 가로 밴드마다 실행할 수 있게 했습니다. `_compute_curvature_term()`은 `lane_num_bands`개의 밴드를 스캔해 (y, lane_center_x) 점을 모으고, 2차 다항식(점이 2개뿐이면 1차)을 피팅해서, 근거리 기울기를 일반 위치 항 위에 더해지는 작은 예측 조향 항으로 바꿉니다. "GPS가 날카로운 각도로 카메라에 인계하면 카메라가 급하게 꺾으며 차선을 잃는" 실패 양상을 겨냥한 것으로, 잡음 많은 근거리 점 하나에 반응하는 대신 밴드 전반의 추세에 기댑니다. 유효한 점이 있는 밴드가 너무 적으면 0으로 폴백합니다(예전 단일 항 동작과 동일).

### 하드웨어 구동

- **RPLiDAR S2**를 `rplidar_ros`(apt)로 구동 — 포트가 뒤바뀐 문제는 위의 obstacle_avoidance 섹션을 참고하세요.
- **Taobotics/HandsFree IMU**(`mrpt_sensor_imu_taobotics`)를 구동했고 `/taobotics/sensor`로 발행합니다(`obstacle_avoid_node`의 AVOID/RETURN yaw 진행 판단이 사용).
- **OAK-D udev 권한**: `lsusb`에는 장치(`03e7:2485`)가 보이는데도 `dai.Device.getAllAvailableDevices()`가 빈 값을 반환했습니다. 수동 udev 규칙 `/etc/udev/rules.d/80-movidius.rules`로 해결했습니다: `SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"`.

### 첫 git 백업

이 세션 전까지는 `src/` 전체가 추적되지 않은 상태였고, 처음으로 커밋했습니다(`a155558`). `.gitignore`에 `__pycache__`/`*.pyc`를 추가했습니다. `fma`/`handsfree_ros2_imu`/`ydlidar_ros2_driver`/`zed-ros2-wrapper`는 gitlink로 커밋되어 있습니다(외부 clone이며 직접 만든 작업이 아니므로 예상된 것). `rtk_bridge.py`에는 실제 NTRIP 자격 증명이 평문으로 들어 있으며, 명시적 지시에 따라 그대로 커밋했습니다.

## 2026-08-05: 주차 통합 (parallel_parking / t_parking) — 코드만 통합, 하드웨어 테스트 안 함

미리 만들어진 규칙 기반 원샷 주차 패키지 두 개(`parallel_parking` - 오른쪽 평행주차, `t_parking` - 왼쪽 T자/직각 후진)입니다. HENES의 실제 차량 제원(L=1.410, W=0.800, WB=0.735 — `obstacle_avoid.yaml`과 정확히 일치)에 맞춰 이미 보정되어 있으며, 단독 실행하는 대신 arbiter/CAN 소유권 아키텍처에 통합했습니다.

**도착했을 때와 달라진 점**(둘 다 자체 LiDAR 드라이버(`sllidar_ros2`), 자체 IMU 노드, 이 워크스페이스에 없는 `my_first_pkg` 패키지, 그리고 CAN을 직접 쓰는 CAN 브릿지를 원했습니다. 세 가지 모두 "control_arbiter만이 CAN을 쓴다"와 "중복 센서 노드를 띄우지 않는다"와 충돌합니다):
- **새 패키지 `parking_bridge`**(`src/parking_bridge/`) — `wheel_odom_pcan_node`: CAN *피드백 전용*(엔코더 0x102 + 조향 0x101)이며, 이미 실행 중인 taobotics IMU(`/taobotics/sensor`)와 융합해 yaw를 구하고 두 주차 노드가 기대하는 `/wheel_odom`으로 `nav_msgs/Odometry`를 발행합니다. `enable_command_tx`의 기본값은 `false`이고 CAN을 직접 쓰지 않습니다. 실제 CAN 프레임 구성에는 별도로 다시 구현하지 않고 `waypoint_follower.can_driver`(이 코드베이스의 다른 곳에서 이미 검증된 같은 TX 0x200/RX 0x101/0x102)를 재사용하고, 각도/쿼터니언 보조 함수도 중복하지 않고 `t_parking.geometry`를 재사용합니다. 새로 작성해야 했던 것은 `blend_angle`(원형 yaw 융합)과 `wrapped_delta`(엔코더 랩어라운드에 안전한 델타)뿐입니다. `encoder_meter_per_count`의 기본값은 원본 스크립트에 들어 있던 자리표시자가 아니라 `obstacle_avoid.yaml` 자체의 측정된 wheel_diameter/encoder_counts_per_rev를 사용합니다.
- **중복 LiDAR/IMU 없음**: `t_parking`의 `scan_parking_filter` 노드는 이미 차량 좌표계 각도 윈도우로 `/scan` -> `/scan_parking`을 리맵할 뿐이고, 그 `input_scan` 기본값도 이미 `/scan`입니다. obstacle_avoidance가 이미 쓰는 기존 RPLiDAR S2 피드입니다. 두 주차 노드의 `imu_topic`은 새 launch 파일에서 `/taobotics/sensor`를 가리키도록 했습니다. `sllidar_ros2`도 `my_first_pkg`도 존재할 필요가 없습니다.
- **두 패키지 모두 소스에 절대 토픽 이름이 하드코딩되어 있음**(`/parking_start`, `/parking/cmd_rpm`, `/parking_active`, ...) — 한 번에 하나만 쓸 때는 괜찮지만, T존과 평행주차존이 *둘 다* 있는 코스에서는 두 주차 노드가 주행 내내 동시에 실행되어야 합니다(그 순간 차량이 어느 존에 있느냐에 따라 실제로는 하나만 발동됨 — 아래 "왜 필요할 때 launch하지 않는가" 참고). 같은 절대 이름의 인스턴스 두 개를 실행하면 충돌하므로, 각 launch 파일이 `remappings=`로 자기 주차 노드(및 `scan_parking_filter`의 `output_scan`)의 토픽을 각자의 접두사 아래로 리맵합니다. `parking_t_left.launch.py` -> `/parking_t/...`, `parking_parallel_right.launch.py` -> `/parking_r/...`이며, 두 패키지의 소스는 수정하지 않습니다.
- **`control_arbiter`에 새 이벤트 존 타입 두 개가 추가됨: `"parking_left"`(t_parking)와 `"parking_right"`(parallel_parking)**(`arbiter_node.py`, `_handle_parking_zone()`): 두 패키지의 `direct_cmd_output`은 `false`로 유지됩니다(`/parking/cmd_rpm` 등은 이미 "다른 무언가가 이것을 CAN으로 중계한다"는 용도로 존재). 두 주차 패키지 자체에는 소스 변경이 필요 없습니다. arbiter는 각 측면의 상태를 독립적으로 추적합니다(`self.parking["left"]`/`self.parking["right"]`, 그 측면의 리맵된 토픽에서 읽음). 존에 진입하면(`event_zones` 항목 `"start:end:parking_left"` 또는 `"...:parking_right"`) arbiter가 그 측면의 `parking_start=true`를 한 번 에지 트리거로 보냅니다. **그 측면이 `mapping=true`를 보고하는 동안**(자체 APPROACH 상태 — 아직 슬롯을 스캔/고정하는 중), **GPS가 존을 곧바로 직진 통과**하며 주차 노드 자체의 접근 컨트롤러를 신뢰하지 않습니다(`"avoid"` 존의 CLEAR 상태에서 GPS가 운전하는 것과 같은 자세 — 주차 노드는 계속 `/scan_parking`+`/wheel_odom`을 관찰하고 누가 실제로 운전하든 상관없이 계획을 세우며, 구동 주체만 바뀜. 이 단계의 rpm은 일반 GPS 순항 rpm이 아니라 그 측면의 `parking_{left,right}_approach_rpm` 파라미터, 즉 각 패키지 자체의 `pre_straight_rpm`(기본값 t_parking=30/parallel_parking=20)입니다. 슬롯 검출이 그 속도에서 보정되었기 때문입니다). 슬롯을 고정하고(`mapping=false`) 적극적으로 기동하기 시작하면(`active=true`), 명령이 최신인 동안 그 측면의 `cmd_rpm`/`cmd_steer`/`cmd_enable`을 그대로 CAN에 중계하는 것으로 제어가 전환됩니다(`obstacle_avoid`의 조향처럼 원시 펌웨어 스케일이며 `send_control_true_deg`를 거치지 *않음*). 그 측면의 주차 노드가 두 신호 중 어느 것이라도 발행하지 않으면 안전하게 실패(정지)합니다. `done=true`가 되면 제어가 카메라/GPS로 돌아갑니다. 기동 중에 차량이 슬롯에 들어가거나 나오기 위해 기록된 선을 물리적으로 벗어나기 때문에, GPS idx가 존 범위를 벗어났는지가 아니라 특별히 그 플래그를 조건으로 합니다.
- **새 launch 파일**: [parking_t_left.launch.py](src/waypoint_follower/launch/parking_t_left.launch.py), [parking_parallel_right.launch.py](src/waypoint_follower/launch/parking_parallel_right.launch.py) — 각각 `scan_parking_filter` + `parking_bridge`의 odom 노드 + 주차 노드 자체만 시작하며(각각 자체 `remappings=` 포함), 스택의 나머지(`post_gps_drive.launch.py`: RPLiDAR S2, taobotics IMU, CAN, arbiter)는 이미 실행 중이라고 가정합니다. 두 존 타입이 모두 있는 코스에서는 둘 다 *함께* 실행해야 합니다. 차량이 존에 진입하는 순간에 맞춰 필요할 때 주차 노드를 launch하는 방안은 검토했다가 기각했습니다. 초기화에 몇 초가 걸리므로(주차 노드가 구독/시작하기 전의 기존 `TimerAction(period=2.0)` 지연이 이미 이를 반영하고 있음) 필요할 때 launch하면 존에 진입하는 순간에 준비되지 않았을 위험이 있고, 프로세스를 동적으로 시작/중지하는 것은 그 자체로 추가적인 실패 모드이기 때문입니다(제때 반영되지 않은 중지는 이 리맵이 피하려던 바로 그 토픽 충돌을 다시 만듦). 두 노드가 발행/트리거 없이 대기하는 데는 사실상 비용이 들지 않으므로, 둘 다 미리 시작하는 것이 더 단순하고 안전한 선택입니다.

**아직 하드웨어 테스트 전** — 이것은 코드 통합 작업일 뿐이고 실제 주차 시도는 한 번도 실행하지 않았습니다. 구동 바퀴를 지면에 닿게 한 채로 어느 쪽이든 실행하기 전에, (이 통합이 바꾸지 않은 각 패키지 자체 README의 "실차 투입 전 반드시 할 것" / "Safety" 섹션에 따라) 다음을 확인하세요.
- LiDAR 장착 자세 가정(`laser_yaw`/`laser_angle_sign`/정적 TF `x=1.175 y=0.0 z=scan_height yaw=pi roll=pi`)이 실제 RPLiDAR S2 장착과 일치하는지 — 이 값들은 패키지가 튜닝되었던 차량에서 온 것이므로, 신뢰하기 전에 `obstacle_avoid.yaml` 자체의 장착 설명("그릴 장착, 지상고 12.5cm")과 맞춰 보세요.
- 조향 부호/최대 각도 캘리브레이션(`forward_turn_sign`, `max_steer_angle_left/right_deg`, 반경 테이블)을 새로운 실차 호(arc) 테스트로 확인 — 제공된 값은 과거 특정 HENES 테스트 로그를 인용하지만 다시 확인해야 합니다.
- `parking_bridge`의 `encoder_meter_per_count`/`wheel_base`/`steer_to_yaw_sign`을 실제 직선/제자리 회전 테스트로 확인(10m 직진 후 `/wheel_odom`의 x를 확인).
- 첫 시도는 두 패키지 자체 안전 섹션에 따라 바퀴를 지면에서 띄운 상태이거나, 장애물이 없는 탁 트인 공간에서 하세요.

### `t_parking`에 자체 직진 탈출 로직 추가

T자 슬롯 주차는 앞쪽이 바깥을 향하도록 후진해서 들어가므로, `parallel_parking`(빠져나오려면 안전이 검증된 역방향 되짚기나 새로운 S커브가 필요)과 달리 탈출은 그냥 앞으로 주행하면 됩니다. 새 경로 탐색이 필요 없습니다. `rule_based_t_parking_node.py`의 새 상태들:

- **`REVERSE_STRAIGHT`의 완료**가 더 이상 곧바로 `DONE`으로 가지 않습니다(예전에는 차량이 여전히 슬롯 안에서 앞쪽이 안쪽을 향하고 기록된 선에서 벗어난 상태로 제어를 카메라/GPS에 즉시 돌려줬음). 대신 **`PARKED`**로 갑니다: 정지 상태이며 `done`은 `False`로 유지되므로 `control_arbiter`는 돌려주지 않고 계속 중계/안전 실패 상태에 있습니다.
- **`PARKED`**는 `/parking_exit_start`(또는 `parallel_parking`과 같은 패턴의 `auto_exit:=true` + `parking_hold_sec`)를 기다린 뒤 `go_settle('EXIT_STRAIGHT')`를 합니다.
- **`EXIT_STRAIGHT`**는 `APPROACH`와 정확히 같은 시작 헤딩 유지 컨트롤러(`approach_cmd()`/새 `exit_cmd()`, 단지 `pre_straight_rpm` 대신 `exit_forward_rpm`을 사용 — `sequence_start_yaw`가 이미 후진해서 들어가기 전의 차선 헤딩이므로 새 헤딩 목표가 필요 없음)로 앞으로 주행합니다. 완료 판정은 새 **`entrance_clearance()`** 헬퍼가 합니다. 후방 범퍼 모서리를 입구 중점에 대한 *이미 고정된*(odom에 고정된) 슬롯의 깊이 축으로 투영하며, 이는 `rear_clearance()`가 이미 먼 벽에 대해 쓰던 것과 같은 모서리 계산이고 단지 입구에서부터 측정하는 것입니다. 입구를 지나(음수가) `exit_clear_margin`만큼 되면 `DONE` + `/parking_exit_done=true`가 됩니다. **새로운 LiDAR 읽기는 필요하지 않습니다.** 앞으로 주행해서 빠져나가는 동안에는 슬롯이 시야에 들어오지도 않고, 처음 고정할 때의 기하 정보가 이미 정확하므로 다시 투영하는 것으로 충분하고 유일한 방법이기도 합니다.
- 새 토픽: `/parking_exit_start`(입력), `/parking_exit_done`(출력) — `parallel_parking`이 이미 쓰던 것과 같은 이름이며 이제 두 패키지가 공유합니다. `parking_t_left.launch.py`의 토픽 리맵 목록에 이를 포함하도록 갱신했습니다.

### `control_arbiter` 재트리거 방지(주차 존)

단순한 존 기반 에지 트리거(`gps_idx`가 `"parking_left"`/`"parking_right"` 존에 진입할 때 `/parking_start`가 한 번 발동)만으로는 충분하지 않습니다. 기동 중에 차량은 의도적으로 기록된 선을 벗어나므로, `stanley_control`의 전역 최근접 웨이포인트 탐색이 `gps_idx`를 존의 `[start, end]` 바로 바깥으로 잠깐 튀었다가 되돌릴 수 있습니다(존 안의 어떤 점보다 다른 웨이포인트가 순간적으로 더 가까워짐). 방지 장치가 없으면 이것은 "존을 떠났다가 다시 진입한 것"과 똑같아 보여서, 첫 번째 시도가 아직 끝나는 중이거나 `DONE` 직후에 두 번째 `/parking_start`가 발동될 것입니다. 측면별 `completed_pending_clear` 플래그로 수정했습니다. 한 측면이 끝나면(`done=True`), `gps_idx`가 그 특정 완료된 존의 `[start, end]`(`parking_retrigger_clear_idx_margin`, 기본값 30 idx만큼 패딩)에서 확실히 벗어나기 전까지는 새 트리거가 발동하지 않습니다. 한 주기 동안 "현재 일치하지 않음"이라는 것만으로는 안 됩니다. 그래서 순환 코스(`loop_waypoints:=true`)에서 진짜 두 번째 바퀴는 차량이 실제로 멀어졌다가 돌아온 뒤에 정상적으로 다시 트리거되지만, 끝난 직후의 경계 깜빡임으로는 곧바로 재시작할 수 없습니다. (주차 노드 자체도 `state in (IDLE, DONE, ABORT)`이 아니면 `/parking_start`를 독립적으로 무시합니다. 이것은 그 위에 얹은 두 번째의 중복된 계층이며 유일한 계층이 아닙니다.)

### 이제 하나의 launch 파일로 전체 스택 시작

`post_gps_drive.launch.py`는 이전에는 RPLiDAR S2/taobotics IMU/`obstacle_avoidance`가 이미 실행 중이라고 가정했습니다(README의 "개별 실행"에 따라). 이제 이들이 통합되었고(각각 토글 가능: `enable_lidar`, `enable_imu`, `enable_obstacle_avoid`, `enable_parking_left`, `enable_parking_right`), 따라서 `traffic_light` + `parking_left`/`parking_right` + `avoid`를 함께 쓰는 코스는 다음만 있으면 됩니다.
```bash
ros2 launch waypoint_follower post_gps_drive.launch.py enable_control:=true \
  waypoints_file:=...
```
`imu_serial_port`의 기본값은 이 README의 다른 곳에서 이미 사용한 by-id 경로입니다. 신뢰하기 전에 실제 장치와 일치하는지 확인하세요.

## 2026-08-07: `t_parking_yellow.zip` 병합 (두 번째 차량에서 동작이 검증된 레퍼런스)

사용자가 `~/Downloads/t_parking_yellow.zip`을 제공했습니다. 다른("노란") 차량에서 이미 동작이 확인된 `t_parking` 빌드이며, 원본을 최대한 그대로 유지하고 차량 고유의 것(조향 기하)만 바꿔달라는 명시적 요청이 있었습니다.

**채택하지 않은 것**: zip의 `CMakeLists.txt`/`ament_cmake` 빌드(자체 `sllidar_ros2` C++ SDK 사본을 포함해 컴파일하고, 자체 `handsfree_imu_a9_node.py`/`wheel_odom_pcan_node.py`/`pcan_protocol.py`/`static_tf_publisher_node.py`를 번들하며, 독립 실행형 `launch/parking_left_oneshot.launch.py`를 제공함) — 이것은 그 다른 차량의 자체 완결형 단일 파일 테스트 리그입니다. HENES에는 이 모든 것이 이미 메인 스택의 일부로 실행되고 있습니다(`rplidar_ros`를 통한 RPLiDAR S2, taobotics IMU, 그리고 `parking_bridge`의 `wheel_odom_pcan_node` — CAN 피드백 전용이며 이미 동작이 검증됨). 기존 구현 옆에 검증되지 않은 CAN 쓰기/LiDAR 구동 구현을 하나 더 바꿔 넣는 것은 이득 없이 위험만 더합니다. 여기의 `t_parking`은 원래 그랬던 일반 `ament_python` 패키지로 유지됩니다. `pcan_protocol.py`는 어느 쪽이든 `waypoint_follower/can_driver.py`의 struct 레이아웃과 바이트 단위로 동일함을 확인했습니다(두 차량이 펌웨어 프로토콜을 공유하므로 변환은 애초에 필요 없었음).

**채택한 것**(순수 알고리즘 파일/값, 빌드 시스템 변경 없음): `geometry.py`, `two_arc_planner.py`, `temporal_gap_detector.py`, `scan_parking_filter_node.py`, `slot_detector.py`는 우리 사본과 zip의 것이 이미 바이트 단위로 동일했습니다(이전 세션에서 이미 병합됨). `cone_detector.py`와 `rule_based_t_parking_node.py`에는 실제 차이가 있었습니다. zip의 것을 통째로 가져온 다음, zip이 그 기능보다 먼저 만들어져서 없는, 이번 세션 자체의 2026-08-05 탈출 로직 추가(`PARKED`/`EXIT_STRAIGHT` 상태, `entrance_clearance()`, `exit_cmd()` — 위 참고)를 그 위에 다시 적용했습니다. 이후 병합 결과를 병합 전 파일과 diff해서, 변경 사항이 의도한 튜닝 개선과 탈출 로직의 깔끔한 재적용뿐임을 확인했습니다. 조용히 사라진 것은 없습니다.

zip에서 가져온 튜닝/알고리즘 개선(`config/parking_left.yaml`):
- `max_cluster_points: 60` → **400** — S2 DenseBoost는 가까운 콘 하나에 수백 개의 빔을 맞출 수 있습니다. 상한이 낮으면 실제 콘을 벽으로 오인합니다. 실제 벽 제거는 여전히 `n_exp_*_ratio`/폭 게이트가 처리합니다.
- 새 `far_track_range`(3.0m)/`far_min_track_hits`(2)/`far_track_miss_sec`(2.5) — 먼 콘은 매 프레임 보이지 않습니다(가림/약한 반사). 이 범위를 넘어서는 곳에서는 근거리와 같은 노이즈 임계값을 쓰는 대신 더 빨리 확정하고 더 오래 유지합니다.
- `entry_max_width: 2.20` → **2.60**.
- 새 `gap_side_min_lateral`(기본값 `0.5*vehicle_width + self_mask_extra` = 여기서는 0.48m) — 차량 자체의 몸체 폭 안에 있는 반사는 외부 경계일 수 없습니다. 예전의 고정 0.31m 하한을 대체합니다.
- `assumed_angle_increment_deg: 0.25` → **0.1125**(RPLiDAR S2의 실제 DenseBoost 각 해상도 — 예전 값은 다른 LiDAR용의 일반적인/틀린 자리표시자였고 시작 배너의 검출 범위 추정에만 쓰임).
- `cone_detector.py`의 `ConeTracker.update()`가 이제 각 검출의 거리를 받아서 위의 far-track hit/타임아웃 오버라이드를 적용합니다.

**조향 기하 보정**(병합의 진짜 목적 — zip의 `max_steer_angle_left/right_deg: 30.0`은 *다른* 차량을 위한 `[!!]` 자리표시자로 명시되어 있었음): HENES의 실제 측정값으로 설정했으며, `parallel_parking/config/parking_parallel_right.yaml`이 이미 쓰는 값과 일치합니다. `max_steer_angle_left_deg: 18.84`, `max_steer_angle_right_deg: 18.05`(cmd -30 → R=2.154m, cmd +30 → R=2.255m). `wheel_odom_front_axle`의 같은 두 파라미터 사본도 수정했고(현재 launch에서는 쓰지 않음 — 대신 `parking_bridge`의 노드를 사용 — 하지만 다시 돌아오는 사람을 위해 일관되게 유지), `log_root`(다른 차량의 `/home/jetson/...` 경로로 하드코딩되어 있었음)는 `~/.ros/parking_logs`로 바꿨습니다.

검증: `colcon build --symlink-install --packages-select t_parking`이 성공하고, `rule_based_t_parking_node.py`가 `rclpy` 아래에서 깨끗하게 import되며, YAML이 파싱됩니다. **이 특정 값들로는 아직 하드웨어 테스트 전**입니다. 원래 2026-08-05 통합과 같은 "실차 투입 전 반드시 할 것" 주의 사항(LiDAR 장착 자세, 조향 부호, 반경 보정)이 여전히 적용됩니다.

## 2026-08-07: 주차 존 속도 램프 (존 진입 전 부드러운 감속)

실차에서 확인: `control_arbiter`가 `gps_idx`가 존의 시작 idx에 도달하는 순간 rpm을 순항 속도(`gps_rpm`, ~130)에서 주차 존의 `approach_rpm`(`parking_left`/t_parking은 30, `parking_right`/parallel_parking은 20)으로 곧바로 잘라버렸고, 거의 급정거처럼 느껴졌습니다("rpm이 130에서 30으로 바로 확 죽거든").

`arbiter_node.py`의 새 `_parking_ramped_rpm(raw_rpm)` 헬퍼는 원시 rpm 값을 아직 활성화되지 않은 주차 존의 `approach_rpm` 쪽으로 블렌딩합니다. 순전히 그 측면의 램프 윈도우 `[zone_start-margin, zone_start)` 안에서 `gps_idx`의 *현재 위치*의 함수이며, "시각 T에 램프가 시작됨"이라는 상태는 전혀 없습니다. (같은 날의 이전 버전은 윈도우 진입 시 rpm을 한 번 스냅샷으로 저장하고 고정된 벽시계 시간 동안 감쇠시켰습니다. 후속 피드백("구간 해서 그 안에 계속... 중간에 하게 해도 되게")에 따라 폐기했습니다. `gps_idx`가 첫 idx가 아니라 윈도우 중간에 진입하면(예: 재시작 후) 잘못된 답이 나왔기 때문입니다. 대신 위치 기반으로 했습니다. `gps_idx`가 윈도우 어디에 있든 rpm은 정확히 그에 해당하는 블렌딩이며, 차량이 언제/어디서 진입했든 올바릅니다.)

- 새 파라미터 `parking_{side}_ramp_idx_margin`(기본값 **7**) — 램프 윈도우는 존의 시작보다 이만큼 idx 전에 시작합니다. `parking_left`의 현재 존 시작(idx 58)에서는 idx 51이며, 명시적 요청("신호등 뒤부터 바로 줄이고")에 따라 `traffic_light` 존이 idx 50에서 끝난 직후입니다. 일반 `declare_parameter`이므로 재launch 없이 실시간으로 튜닝할 수 있습니다: `ros2 param set /control_arbiter parking_left_ramp_idx_margin 7`.
- **`gps_idx`가 램프 윈도우 안에 있는 동안 램프되지 않은 rpm을 보내게 될 모든 곳에서 호출됩니다** — 일반 카메라/GPS 폴백 경로(`base_rpm`)만이 아닙니다. 이것은 구체적으로 중요했습니다. 오늘의 코스는 `parking_left`의 램프 윈도우(idx 51-57)가 `gps_priority` 존(idx 49-57) *안에서* 시작하고, 그 존의 분기는 `base_rpm`을 거치지 않고 `self.gps_rpm`을 직접 보내기 때문에 자체 `_parking_ramped_rpm()` 호출이 필요했습니다. 그렇지 않으면 램프가 처음 몇 개 idx 동안 조용히 아무 일도 하지 않았을 것입니다. 같은 이유로 `avoid` 존의 CLEAR 상태 GPS 구동 분기에도 같은 수정을 적용했습니다(오늘의 코스에서는 idx가 겹치지 않아 상관없지만, 앞으로의 코스에서 존이 겹치면 조용히 깨질 수 있음).
- rpm에만 영향을 줍니다. 조향은 평소에 운전하는 쪽(카메라 또는 GPS)에서 그대로 옵니다.
- `gps_idx`가 존에 실제로 도달하면 `_handle_parking_zone`이 완전히 인계받습니다(위의 모든 것을 무시하고 `approach_rpm`을 직접 강제). 그때쯤이면 램프가 이미 속도를 낮춰 놓았다는 것은 그 인계가 더 이상 급격한 단차가 아니라는 뜻일 뿐 기능적 의존성은 아닙니다.

**이 특정 margin/윈도우 배치로는 아직 실시간 재테스트 전**입니다.

## 2026-08-07: 주차 제어권 버그 (기동 도중 GPS가 다시 가져감)

**실차에서 확인**: GPS 구동으로 `parking_left`에 진입하면 슬롯을 찾고 기동을 시작하지만, 중간에 마무리하지 않고 다시 GPS 직진 주행을 재개했습니다("공간 잡고 틀려고 하고 하는데 그냥 gps쭉 하는듯"). 정지 상태에서(슬롯에서 얼마나 떨어져 있든) `/parking_start`를 수동으로 트리거하면 매번 잘 동작했습니다. 차이는 특히 GPS 구동 진입이었습니다.

**근본 원인**: `on_timer`의 디스패치 체인이 `_handle_parking_zone()`을 순전히 `_zone_at(self.gps_idx)`의 현재 존 일치 여부로만 게이트했습니다. 바로 위의 재트리거 방지 주석이 "이 존의 특별 처리를 실제로 끝내는 것은 idx가 존을 벗어나는 것이 아니라 `self.parking[side]["done"]`이다"라고 명시하고 있음에도 그렇게 되어 있었고, 문서화된 의도와 실제 코드 사이의 실제 모순이었습니다. 능동적인 기동 중에는 차량이 의도적으로 기록된 선을 벗어나므로(슬롯으로 후진), `stanley_control`의 전역 최근접 웨이포인트 탐색이 `gps_idx`를 존의 `[start, end]` 바깥으로 실제로 보낼 수 있고(재트리거 방지가 이미 처리한 잠깐의 깜빡임만이 아니라), 그런 일이 일어나는 순간 `zone`이 더 이상 `"parking_left"`가 아니게 되며 제어가 조용히 일반 GPS/카메라 구동으로 떨어져 기동 도중에 포기했습니다. 정지 상태의 수동 트리거에는 이 문제가 없습니다. 차량이 기록된 GPS 선을 따라가고 있지 않으므로, 기동 자체가 주차 노드의 자체 odom 기반 제어를 통해 차량을 움직이기 시작하기 전까지는 헤맬 `gps_idx`가 아예 없기 때문입니다.

**수정**: `self.parking[side]`에 측면별 `engaged` 래치를 새로 추가했습니다. `/parking_start`가 발동하는 바로 그 틱에 `True`로 설정하고, `state["done"]`이 관찰될 때만 `False`로 설정합니다(재트리거 방지 자체의 `completed_pending_clear` 트리거 시점과 같음). `on_timer`는 이제 `engaged_side`를 다른 모든 존 타입(`stop`, `gps_priority`, `avoid`, `traffic_light`)보다 **먼저** 확인합니다. 한 측면이 engaged되면 `gps_idx`가 숫자상 어떻게 움직이든 주차 노드 자체가 완료를 보고할 때까지 그 측면이 제어를 소유합니다. 예전의 idx로 게이트하는 `elif zone in ("parking_left", "parking_right")` 분기는 죽은 방어용 폴백으로 남겨 둡니다(도달할 수 없어야 함 — `engaged`는 그 분기의 조건이 처음 참이 되는 바로 그 틱에 설정되므로).

`ABORT`를 거쳐 래치되는 경우의 실패 모드 안전성을 확인했습니다. 노드의 `ABORT` 상태는 매 틱 `publish_zero(True, False, False, True)`를 계속 호출하고, 이는 내부적으로 `publish_cmd(0, 0, 1)`을 호출하므로 `cmd_rpm`/`cmd_steer`는 `0`으로 유지되며 `cmd_last_time`을 계속 갱신하고, `active`는 `True`로, `done`은 `False`로 유지됩니다. `engaged` 상태에서는 arbiter가 살아 있는 `rpm=0, steer=0, enable=1`(모터는 물려 있고 정지 상태로 유지)을 CAN으로 무한정 계속 중계한다는 뜻입니다. 구조상 안전하지만(멈췄거나 중단된 기동을 눈 감고 GPS로 통과하는 쪽으로 조용히 폴백하지 않음), 복구하려면 수동 `/parking_reset`이 필요하다는 뜻이기도 합니다(중단 후 그 측면은 스스로 GPS 구동을 재개하지 않음). 허용 가능하다고 판단했습니다. 주차 중단은 멈추고 사람을 기다려야 하며, 조용히 계속 진행해서는 안 됩니다.

**아직 실시간 재테스트 전** — 그리고 바로 다음 섹션에 따르면 이 특정 수정은 (전체) 이야기가 아닌 것으로 밝혀졌습니다. 아래를 참고하세요.

## 2026-08-07: 진짜 근본 원인 — 기동 내내 `mapping`이 True로 유지됨

위의 `engaged` 수정은 실제 수정이었지만 충분하지 않았습니다. 다음 실차 테스트에서도 그 수정을 넘어선 뒤 똑같은 증상이 보였습니다("reverse arc 뜨고 각도 떠도 걍 rpm 30으로 앞으로만 가는데?" — 노드 자체의 상태는 올바르게 진행되었고, SETUP_ARC → REVERSE_ARC로 각도도 제대로 로그에 찍혔지만, 실제 차량은 내내 `approach_rpm`=30으로 직진만 계속했음).

**근본 원인, `rule_based_t_parking_node.py`의 `on_timer`를 한 줄씩 읽어서 발견**: `SETUP_ARC`, `REVERSE_ARC`, `REVERSE_STRAIGHT`가 *전부* 매 틱 `self.pub_mapping.publish(Bool(data=True))`를 호출하고 있었습니다. 즉 슬롯이 고정되고 실제 2-호(arc) 기동이 시작된 뒤에도 `mapping`이 실제로는 `False`가 된 적이 없습니다. `control_arbiter`의 `_handle_parking_zone`은 `active`보다 *먼저* `mapping`을 확인하고, `True`이면 노드가 그 밖에 무엇을 하든 상관없이 무조건 `approach_rpm`으로 일반 GPS 구동을 합니다. 그래서 SETUP_ARC/REVERSE_ARC/REVERSE_STRAIGHT 동안 이 노드가 계산한 모든 `cmd_rpm`/`cmd_steer` 값은(자체 로그된 상태/각도에 따르면 올바르게) 기동 내내 매 틱 조용히 버려졌습니다. 2026-08-05 탈출 로직 병합이나 2026-08-07 `t_parking_yellow.zip` 병합이 도입한 것이 아니라 둘 다에 있었고, 노드 자체 상태 머신의 원래 부주의였습니다(mapping은 "아직 슬롯을 찾는 중"을 뜻해야 하는데 그 시점을 한참 지나서까지 `True`로 남아 있었음).

**수정**: `SETUP_ARC`/`REVERSE_ARC`/`REVERSE_STRAIGHT`는 이제 `pub_mapping.publish(Bool(data=False))`를 발행합니다. 여전히 `True`를 발행하는 것은 `APPROACH`뿐이며(올바름 — 슬롯을 아직 찾거나 향해 추적하는 유일한 상태), 다른 어떤 상태도 엉뚱한 `True`를 발행하지 않음을 확인했습니다(`grep pub_mapping.publish`).

이것과 위의 `engaged` 수정을 합치면 의도한 흐름은 이제 다음과 같아야 합니다. `APPROACH`(mapping=True, GPS가 approach_rpm으로 직진 구동) → 슬롯 고정 → `SETUP_ARC`/`REVERSE_ARC`/`REVERSE_STRAIGHT`(mapping=False, `engaged`가 idx와 상관없이 arbiter를 이 측면에 고정, 노드 자체의 `cmd_rpm`/`cmd_steer`가 CAN으로 중계됨) → `PARKED`/`EXIT_STRAIGHT`(탈출 로직) → `DONE`(`engaged` 해제, 제어가 돌아감).

**아직 실시간 재테스트 전** — 다음 섹션을 참고하세요. 이것도 전체 수정은 아니었습니다.

## 2026-08-07: 위 수정보다 한 상태 앞에서 일어난 같은 `mapping` 버그

`SETUP_ARC`/`REVERSE_ARC`/`REVERSE_STRAIGHT` 수정 이후의 실시간 재테스트에서도 여전히 내내 직진만 하고 멈추지조차 않았습니다("setup arc 뜰 때 멈춰야 하는거 아님? 지금 주차 공간 탐지해서 멈추는 기준이 뭐야"). 존의 시작 idx에서 정지 상태로 캡처한 결과, 콘 검출/추적 자체는 정상임을 확인했습니다(`cones=5/5 tracks=5`, `state=SEEK_BOUNDARY` — 시간적 간격 검출기가 경계 거리가 열리고 닫히는 것을 보려면 실제 전진 운동이 필요하므로 움직이지 않는 동안에는 예상된 상태).

**근본 원인 — 같은 버그, 한 상태 앞**: `APPROACH`도 `self.locked`와 상관없이 *상태 전체* 동안 `pub_mapping.publish(Bool(data=True))`를 무조건 발행하고 있었습니다. 하지만 실제 설계 의도(그리고 `_handle_parking_zone` 자체의 docstring)는 "mapping = 아직 찾는 중이므로 이 노드 자체의 접근 컨트롤러를 아직 신뢰하지 마라"이며, 이것은 상태 이름이 `SETUP_ARC`로 바뀔 때만이 아니라 슬롯이 고정되는 순간 참이 아니게 되어야 합니다. 고정된 후에도 `APPROACH`는 곧바로 전이하지 않습니다. 계속 스스로(자체 `approach_cmd()`로) 호 계획의 시작점을 향해 주행하다가 `publish_stop()`을 호출하고 `SETTLE`(`vx`가 실제로 `stop_speed_thresh` 아래로 떨어질 때까지 기다린 뒤 `SETUP_ARC`로 진행)을 거칩니다. `mapping`이 그 모든 과정 동안 `True`로 붙어 있으면 `control_arbiter`는 고정, 시작점까지의 주행, 정지 시도를 거치는 내내 `approach_rpm`으로 GPS 구동을 계속하므로 `vx`가 `SETTLE`을 진행시킬 만큼 낮아지지 않았고 차량은 `SETUP_ARC`에 도달할 수가 없었습니다. 이것이 바로 차량이 "그냥 직진하고 멈추지 않았던" 이유입니다. 내부에서 완벽하게 올바른 고정이 일어나도, 그 이후의 어떤 것도 CAN으로 중계될 수 없었으므로 아무 의미가 없었습니다.

**수정**: `APPROACH`는 이제 하드코딩된 `True`/`False` 대신 `pub_mapping.publish(Bool(data=not self.locked))`와 `pub_active.publish(Bool(data=self.locked))`를 발행합니다. 둘 다 상태 이름이 아니라 고정 상태를 직접 따라갑니다. `SETTLE` 자체는 여전히 어느 토픽도 건드리지 않습니다(거기서는 변경이 필요 없음 — `APPROACH`가 마지막으로 발행한 것을 그대로 유지하는 것이 맞고, `SETTLE`에 도달할 때쯤에는 이미 `mapping=False, active=True`입니다. `self.locked`는 새 `begin_sequence`/`reset_all`에서만 `False`로 돌아가므로).

앞선 두 수정과 합치면 전체 의도된 인계는 이제 다음과 같습니다. `APPROACH` 미고정(mapping=True, active=False, GPS 구동) → 고정 → `APPROACH` 고정/시작점으로 주행 중 그리고 `SETTLE`(mapping=False, active=True, 노드 자체의 `approach_cmd()`/정지가 CAN으로 중계됨) → `SETUP_ARC`/`REVERSE_ARC`/`REVERSE_STRAIGHT`(동일, 노드의 호/후진 명령이 중계됨) → `PARKED`/`EXIT_STRAIGHT` → `DONE`.

**아직 실시간 재테스트 전** — 그리고 아직도 정확히 이것은 아니었습니다. 아래를 참고하세요.

## 2026-08-07: 같은 계열의 네 번째 버그 — `SETUP_ARC`가 `active=False`를 하드코딩

위 수정 이후의 실시간 재테스트: 차량이 이제 실제로 올바르게 멈추고(SETTLE 동작), 상태가 `SETUP_ARC`로 진행되었지만(`/parking_t/parking_status`로 확인: `SETUP_ARC: steer=30 turned=-0.1/12.2deg moved=0.003`), 그 후 그냥 가만히 있었고 `moved`는 늘지 않았습니다. 토픽별로 직접 캡처하자 한 번에 정확히 짚였습니다. `/parking_t/parking_active=false`인데 `/parking_t/cmd_rpm=30`, `cmd_steer=30`, `cmd_enable=1`이 모두 동시에 살아 있고 올바른 상태였습니다. 사용자 본인의 후속 캡처가 정확한 전이를 확인해 주었습니다. `parking_active`는 주행/탐색 중에는 `false`(올바름 — 미고정)로 읽히고, 차량이 슬롯에 가까워지는 즉시 `true`로 바뀌며(올바름 — 위의 `APPROACH`의 고정 연동 수정), **멈추는 순간 다시 `false`로 바뀌고 돌아오지 않았습니다.**

**근본 원인**: `SETUP_ARC`가 `self.pub_active.publish(Bool(data=False))`를 호출하고 있었습니다. 하드코딩된 값이고, `mapping` 수정이 건드리지 않은 다른 줄입니다(빠르게 스크롤하며 지나가면 놓치기 쉬움). `mapping`도 이제 올바르게 `False`가 되었으므로 `control_arbiter`의 `_handle_parking_zone`은 더 이상 "mapping" 분기를 타지 않았지만, `active=False`라서 `fresh and state["active"]` 검사에도 실패하고 곧바로 안전 정지 분기로 떨어졌습니다. 그래서 이 상태가 계산한 모든 `cmd_rpm`/`cmd_steer`(자체 로그상 올바르게)는 대신 `rpm=0, enable=0`이 CAN으로 전송되었습니다. `REVERSE_ARC`/`REVERSE_STRAIGHT`/`EXIT_STRAIGHT`는 이미 올바랐고(`active=True`), 이것은 `SETUP_ARC` 하나에만 국한되었습니다. 사용자가 상태 로그의 텍스트를 믿는 대신 원시 토픽을 직접 캡처했기 때문에 비로소 발견되었습니다(그 텍스트는 `active`를 전혀 언급하지 않으므로 지금까지의 어떤 로그 캡처에서도 보이지 않았음).

**수정**: `SETUP_ARC`는 이제 `pub_active.publish(Bool(data=True))`를 발행합니다.

이것은 이번 세션에서 같은 계열의 네 번째 버그입니다(`engaged` idx 래치, `SETUP_ARC`/`REVERSE_ARC`/`REVERSE_STRAIGHT`의 mapping, 고정에 연동된 `APPROACH`의 mapping/active, 그리고 이것). 모두 "arbiter의 mapping/active 게이트가 노드의 상태 머신이 실제로 하던 일과 맞지 않았다"의 변형입니다. 발견될 때마다 하나씩 고치는 대신 다음 실차 테스트 전에 `pub_mapping`/`pub_active` 호출의 상태별 전체 감사를 권장합니다. `pub_mapping.publish\|pub_active.publish`를 grep해서 각 상태의 쌍이 `_handle_parking_zone`의 세 가지 경우(탐색 중 → mapping=True, 능동 기동 중 → mapping=False, active=True, 완료/중단/주차 대기 중 → 둘 다 False, 또는 `publish_zero` 호출 지점이 정하는 대로)에 비추어 맞는지 확인하세요.

**아직 실시간 재테스트 전.**

## 현재 상태 / 아직 하지 않은 것

- **RTK**: 2026-07-27 야외 실측 테스트에서 Float(`flags=67`)와 Fixed(`flags=131`, `flags=131` = Fixed + DGPS + valid) 도달을 확인했고, 같은 날의 `num_sv=0` 소동 이후 2026-07-28 늦게 다시 Float를 확인했습니다(위의 "2026-07-28 세션" 참고 — 소프트웨어/USB 경합 문제가 아니었고, 안테나/RF 하드웨어는 `UBX-MON-HW`로 정상 확인되어 일시적인 것으로 처리). 같은 날 늦게 야외에서 단독으로 다시 테스트했을 때 Float를 거의 즉시 얻었고(2026-07-28 이전의 정상 동작과 일치), `integrated_drive.launch.py`로 GPS+ZED를 모두 실행했을 때도 결과가 같았습니다. 하늘 시야와 NTRIP 네트워크 안정성에 좌우됩니다(테스트에서 캐스터 연결이 끊겼다가 재연결되고 체크섬 오류가 나타났음 — 이 코드가 아니라 캐스터/네트워크의 문제). 2026-07-28 기준 **해결된 것으로 간주**합니다. `num_sv=0`이 재발하면 GPS의 USB 연결을 먼저 뽑았다 꽂아 보세요(위의 2026-07-28 세션 섹션의 "최종 정리" 참고).
- **traffic_light 패키지 — 의존성은 설치됨, 하드웨어 테스트는 여전히 전무**: `depthai==2.32.0.0`(<3으로 고정, 위의 함정 참고)과 `ultralytics==8.4.110`이 이제 설치되어 있고 `cv_bridge`/`pyzed`/`torch`와 충돌 없이 함께 import됨을 확인했으며, OAK-D가 없으면 노드가 깨끗하게 시작하고 대기합니다. 하지만 이번 세션에는 OAK-D를 실제로 연결한 적이 없습니다. 노드 자체와 `control_arbiter`의 `traffic_light` 이벤트 존 로직은 가짜 `ros2 topic pub` 데이터로만 실행했을 뿐(위의 "2026-07-28 세션" 참고) 실제 카메라나 실제 빨간불/초록불에 대해 실행한 적이 없습니다. 차량에서 신뢰하기 전에: OAK-D를 연결하고, 현장에서 검출 정확도/지연을 확인하고, 기본값 `traffic_light_approach_rpm_scale=0.5` / `traffic_light_timeout_sec=1.0`을 실제 동작에 비추어 점검하세요.
- **USB 끊김/재번호 부여 — 하드웨어 문제로 확인됨, 해결 안 됨**: by-id 심볼릭 링크(GPS 섹션 참고)는 재연결 후 장치 경로가 바뀌는 증상은 고치지만, 끊김 자체는 실제이고 해결되지 않았습니다. 실시간 장애 중의 `sudo dmesg | grep -iE "usb|reset"`에서 GPS 수신기 포트에 `device descriptor read/64, error -71`과 `usb usbN-portM: attempt power cycle`이 반복되었고, 전형적인 접촉 불량의 징후(나쁜 케이블, 손상되었거나 헐거운 커넥터, 또는 고장 나는 포트)이며 ~10-50초마다 반복됩니다. 사용자가 이 GPS 장치를 한 번 떨어뜨린 적이 있다고 언급했는데, 간헐적으로 고장 나는 커넥터와 맞아떨어집니다(깔끔하게 항상 켜짐/항상 꺼짐으로 끊기는 것이 아니라 어떤 때는 고장 나고 어떤 때는 동작함 — 한 테스트에서는 `End of file` 에러가 폭주했고 다음 테스트에서는 깨끗하게 실행된 것과 일치). 코드를 더 고쳐서 해결할 수 있는 문제가 아닙니다. 다른 케이블, 다른 USB 포트, 허브 없이 시도하고 수신기의 포트를 물리적으로 점검하세요. 그동안 (재연결 후의) *장치 경로* 문제는 by-id 심볼릭 링크로 해결되지만, 세션 도중의 끊김은 여전히 포트를 잡고 있던 것을 죽이므로 수동 재시작이 필요합니다(자동 재시도 래퍼를 제안했지만 만들지는 않았음 — 끊김이 계속 방해가 된다면 요청하세요).
- **헤딩**: 자이로 + 자력계 + GPS 융합, 캘리브레이션 상태와 주의 사항은 IMU 섹션을 참고하세요. 실제 주행 중에는 테스트하지 않았습니다(자이로 드리프트와 자력계 soft-iron/간섭 모두).
- **코너링**: 이번 세션에서 적극적으로 튜닝 중(곡률 감속 + 예측 조향) — 최신 분석과 남은 간극(선행 거리에 속도 항 없음, 실현 불가능/복구 처리 없음)은 위의 "웨이포인트 추종기 — 컨트롤러 변경 사항"을 참고하세요.
- **CAN 제어**: 패킷 형식(ID `0x200`, `<hhBBH` rpm/steer/enable/stop_mode, 피드백 ID `0x101`/`0x102`)이 실제 하드웨어와 일치함을 확인했습니다(동작하는 수동 키보드 텔레옵 스크립트인 `pcan_tools/pcan_jetson_live.py`와 일치). 이번 세션에 실제 차량에서 `enable_control:=true`로 여러 번 실행했습니다. `cruise_rpm`→실제 m/s 캘리브레이션은 아직 측정하지 않았습니다.
- **NTRIP 자격 증명**(`rtk_bridge.py`, `f9p_bringup/launch/f9p_rover.launch.py`)은 저장소 안에 평문으로 들어 있습니다. `ros2_ws/src`가 아직 git에 커밋되지 않았으므로 지금은 괜찮지만, 어딘가에 push하기 전에 이를 반드시 점검하세요.
- `waypoint_follower/gps_node.py`, `fake_gps_node.py`, `f9p_bringup/f9p_bringup/rtcm_serial_bridge.py`는 이번 세션 초반 시도에서 남은 죽은 코드입니다(직접 만든 NMEA 파싱과 직접 만든 ROS 쪽 RTCM 브릿지, 둘 다 대체됨). 삭제하지 않고 쓰지 않는 채로 남겨 두었습니다.

## 2026-08-12 세션: 코스 재기록, `parallel_parking.zip` 재병합,
## params_file launch 충돌, stop_mode 덜컹거림 수정, 실시간 평행주차
## 디버깅 (vx 노이즈, SETTLE 멈춤, 안전 여유, 횡방향 미스)

새로 다시 기록한 테스트 코스에서 한 긴 실시간 디버깅 세션입니다. 코드/설정에서 실제로 바뀐 것의 요약이며, 정확한 줄은 git 이력/diff를 참고하세요.

**새 코스 + EVENT_ZONES**: 웨이포인트를 다시 기록했습니다(201개 점, idx 0-200). `waypoint_follower/launch/post_gps_drive.launch.py`의 `EVENT_ZONES`를 다시 구성했습니다. `10:29:traffic_light:18`, `30:37:gps_priority`, `38:89:parking_left`, `90:114:gps_priority_slow`(새 존 타입 — `gps_priority`처럼 GPS 전용이지만 새 `gps_priority_slow_rpm` 파라미터로 rpm이 제한됨. T주차 탈출에서 평행주차 진입까지의 이동 구간을 위해 추가), `115:142:parking_right`, `143:200:avoid`. 정확한 idx 경계를 미리 알 수 없었기 때문에 이를 시각적으로 고정하기 위해 일회성 대화형 웨이포인트 idx 뷰어 아티팩트를 만들었습니다 — 저장소에는 저장하지 않았으며, 다시 필요하면 요청하세요(`recorded_waypoints.csv`로 빠르게 다시 만들 수 있음).

**치명적 버그를 찾아서 수정 — `params_file` launch 인자 이름 충돌**: `parking_t_left.launch.py`와 `parking_parallel_right.launch.py`는 둘 다 예전에 일반 `DeclareLaunchArgument('params_file', ...)`을 선언했습니다. `DeclareLaunchArgument`/`LaunchConfiguration` 이름은 포함된 파일별로 범위가 정해지는 것이 아니라 *전체* launch 트리에서 전역입니다. `post_gps_drive.launch.py`가 둘 다 포함하므로 먼저 포함된 쪽이 두 노드 모두에 대해 조용히 "이겼습니다". `ps aux | grep rule_based_parallel`로 실시간 확인한 결과, 실제 실행 중인 프로세스의 `--params-file`이 parallel_parking 자체 설정이 아니라 t_parking의 `parking_left.yaml`을 가리키고 있었습니다. 이것이 `pre_straight_abort_yaw_deg: 0.0` 설정 오버라이드가 조용히 적용되지 않았던 이유입니다. 패키지 범위의 인자 이름 `t_parking_params_file` / `parallel_parking_params_file`로 이름을 바꿔서 **수정**했습니다.

**주차 정지 덜컹거림("덜컹덜컹")의 근본 원인을 AURIX 펌웨어 소스에서 발견**(`STM_Interrupt_1_KIT_TC275_LK_1.zip`, 별도의 Windows 쪽 저장소, 읽기 전용 참고): CAN `0x200`의 byte5 `stop_mode`는 장식이 아닙니다. `0`(일반)은 정지 상태에서 헌팅/진동할 수 있는 폐루프 `StopHoldEnable=TRUE` 위치 유지 PID를 켜고, `1`(flat)은 이를 끄므로(`StopHoldEnable=FALSE`) 깨끗하게 해제됩니다. PS2 컨트롤러 자체의 네모(square) 버튼 동작과 일치합니다. `control_arbiter`의 주차 중계는 예전에는 `cmd_enable==0`일 때만 `stop_mode=1`을 보냈는데, 이제 중계되는 `cmd_rpm==0`일 때도 보냅니다(`arbiter_node.py`). `parking_bridge/wheel_odom_pcan_node.py` 자체의 직접 CAN 경로에는 이미 동등한 로직(`auto_flat_stop_on_zero_rpm`, 기본값 True)이 있어서 단독 모드 테스트는 이 특정 간극의 영향을 받지 않았습니다.

**`parallel_parking.zip` 재병합**(새 2026-08-12 버전, 언더스코어 없는 파일 이름, 두 번째 차량에서 동작 확인됨)을 이번 세션 자체의 수정 위에 적용했습니다. `max_steer_angle_left/right_deg` 코드 기본값 30.0/30.0 -> 18.84/18.05(설정에는 이미 실제 HENES 값이 있어서 영향 없음), 새 `exit_reverse_only` 파라미터(기본값 True) + `build_reverse_handoff_plan()` — 탈출은 이제 슬롯을 벗어날 만큼만 후진하고 완전한 자체 구동 S커브 탈출 대신 웨이포인트 추종기에 인계합니다(단독 테스트에서 중요: 단독 실행은 차선까지 몰아낼 것이 남아 있지 않으므로 "탈출을 끝내지 못하는" 것처럼 보임 — 버그가 아니라 설계), `parallel_max_length` 4.00 -> 5.00.

**`wheel_odom_pcan_node.py`(t_parking과 parallel_parking이 공유 — 하나의 수정이 양쪽을 모두 커버) — 보고된 `vx`가 필터링 없는 단일 샘플의 순간 `ds/dt`였음**: 느린 주차 속도에서는 ~20ms 샘플 하나가 엔코더 카운트 1 미만으로 움직이는 경우가 많아서, `vx`가 카운트 양자화 노이즈에 지배되고 실제로 정지해 있어도 0에서 벗어난 값에 붙어 있을 수 있었습니다(그러면 `SETTLE`이 영원히 기다림). 샘플 하나 대신 마지막 `vx_window_samples`(기본값 7)개의 `(ds, dt)` 쌍에 걸쳐 `vx`를 평균내는 것으로 수정했습니다. 위치/yaw 적분(`self.x`/`self.y`/`self.yaw`)은 건드리지 않았고, *보고되는* `vx`만 평활화했습니다. **여전히 완전히 설명되지 않음** — 이 수정 후의 실시간 테스트에서 거의 평평한 지면에서 바퀴가 눈에 띄게 거의 움직이지 않는데도 정지 상태의 `vx`가 실행마다 달랐습니다(0.081 / 0.099 / 0.16 m/s). 이는 "소프트웨어 노이즈"(candump에서 `encoder_count`의 첫 2바이트가 "정지"된 동안 한 방향으로 진짜로 꾸준히 변하고 있었음 — 오래된 데이터가 아닌 실제 신호)에도, 일관된 "경사 크리프" 설명(실행마다 크기가 너무 일관되지 않음)에도 깔끔하게 맞지 않습니다. 순수 소프트웨어 버그가 아니라 엔코더의 전기적/배선 노이즈일 가능성이 높으며, 근본 원인은 찾지 못하고 우회만 했습니다(아래 참고).

**위에 대한 실용적 우회책 — `SETTLE`에 이제 하드 타임아웃이 있음**(새 `settle_timeout_sec` 파라미터, 기본값 5.0, 실시간 테스트를 위해 `parking_parallel_right.yaml`에서는 현재 3.0)이 `rule_based_parallel_parking_node.py`와 `rule_based_t_parking_node.py` 모두에 들어갔습니다. `SETTLE`에서 이 초 수를 넘기면 `vx`가 무엇을 하든 상관없이 영원히 멈춰 있는 대신 다음 상태로 진행합니다. `stop_hold_sec`(1.0초)은 이것이 세기 시작하기 전에 이미 지나므로, "깨끗한" vx 읽기를 기다리지 않아서 생기는 정확도 손실은 작을 것으로 예상합니다. 또한 "빠른 경로"(타임아웃이 필요 없이 바로 통과)로서 두 주차 설정에서 `stop_speed_thresh`를 0.05 -> 0.10으로 올렸습니다. vx의 크기가 계속 달라지는 점을 감안하면 이 정확한 숫자가 최종이라고 기대하지 마세요.

**실시간 테스트 중의 안전 여유 튜닝**(`parking_parallel_right.yaml`): `reverse_cone_safety_stop_margin` 0.10 -> 0.07(실제 `MANEUVER` 후진이 `cone=0.099`에서 중단됨, 예전 여유보다 1mm 안쪽, `collision=False`)과 `swept_path_boundary_margin` 0.10 -> 0.07(같은 기동이 다음에는 이 *다른* 여유에 대해 `inner=0.093`에서 중단됨 — 둘은 혼동하기 쉬움. `reverse_safety_stop()`의 중단 로그가 예전에는 콘 검사에만 정확한 하나의 공유 `margin=` 값을 출력했는데, 이제 각 검사 자체의 여유를 측정값 옆에 출력하도록 수정). 이것들은 버그가 아니라 실제 안전 임계값이며, 실제 기하가 정당하게 그만큼 가깝게 가기 때문에 낮췄습니다. 하지만 실시간 튜닝의 추측일 뿐이며 새로운 전체 실행으로 재검증하지 않았습니다.

**이 세션 후에도 미해결로 열려 있음(비 때문에 실시간 테스트가 중단됨)**:
- **`FINAL_ALIGN lateral miss q=0.425 exceeds 0.100`**: 실제 실행이 `MANEUVER`를 거쳐 `FINAL_ALIGN`(마지막 중앙 정렬 단계)까지 모두 갔다가, 차량이 슬롯 중앙에서 횡방향으로 42.5cm 벗어난 채 끝나서 거기서 중단되었습니다. `FINAL_ALIGN`은 직선 운동으로 횡방향 오차를 보정하는 것을 의도적으로 거부합니다(과거 버전이 시도해서 아슬아슬한 미스를 후방 벽 정지로 바꿔버렸음). 따라서 여기서 허용 오차를 키우는 것이 아니라 더 앞선 호/후진 구간이 더 정확해야 합니다. 유력한 용의자: 아래에 언급된 같은 조향각 캘리브레이션 간극입니다. 모든 호 구간의 계획된 선회 반경은 `max_steer_angle_left/right_deg`(18.84/18.05, 덜 엄밀한 이전 호 로그 테스트에서 나옴)에서 나오는 반면, `can_driver.py`의 `TRUE_STEER_MAX_ANGLE_DEG`(14.3, 2바퀴 GPS 원 피팅 테스트에서 나옴)가 더 신뢰할 만한 측정이지만, *왼쪽* 선회 방향에 대해서만 검증되었고 parallel_parking의 호는 양방향을 모두 씁니다. 다음 단계: `parallel_parking`의 설정에서 14.3으로 교체하는 것을 신뢰하기 전에 *오른쪽* 선회에 대한 전용 풀 락 원 피팅 테스트를 합니다. `~/.ros/parking_logs/rule_parallel_simple_<timestamp>/` 아래의 실행별 `parallel_control.csv`/`parallel_candidates.csv` 로그를 오차가 어느 구간에서 왔는지 보는 방법으로 확인했지만, 이번 세션에서 확인한 실행들에서는 비어 있었습니다(헤더 행만 있고 데이터 행 없음). 이런 사후 분석에 유용해지려면 로거의 쓰기 조건을 살펴봐야 합니다.
- `parking_parallel_oneshot.launch.py` / `t_parking/launch/parking_left_oneshot.launch.py`는 원래 레퍼런스 zip에서 온 **망가진, 적응되지 않은 잔재**입니다. 이 워크스페이스의 실제 패키지(`obstacle_avoidance/launch/rplidar_s2.launch.py`를 통한 `rplidar_ros`, `mrpt_sensor_imu_taobotics`, `parking_bridge`) 대신 이 워크스페이스에 존재하지 않는 패키지(`sllidar_ros2`, `my_first_pkg`)를 참조합니다. 그대로 쓰지 마세요. arbiter 없는 진짜 단독 테스트를 하려면, 대신 실제 lidar/imu launch 파일과 `waypoint_follower/launch/parking_parallel_right.launch.py direct_cmd_output:=true auto_start:=true`를 launch하세요(이미 검증된 실제 구성 요소를 재사용하고, GPS/arbiter를 기다리는 것만 건너뜀).
- 위의 `reverse_cone_safety_stop_margin`/`swept_path_boundary_margin`/`settle_timeout_sec`/`stop_speed_thresh` 값은 모두 날씨 때문에 중단된 하룻저녁의 실시간 튜닝에서 나온 것입니다. 최종이라고 가정하지 말고 다음 맑은 날 실행에서 재검증하세요.
- `vx` 노이즈의 근본 원인(위 참고)은 실제로 찾지 못했고 `SETTLE`의 타임아웃으로만 우회했습니다. `SETTLE` 외에 `abs(vx) < threshold`로 게이트하는 다른 곳에서 문제가 된다면 같은 타임아웃 백스톱 패턴을 거기에도 적용해야 할 수 있습니다.

## 2026-08-17 session: CAN 진단 프로토콜(0x104/0x203) 설계, `arbiter_node.py` `base_steer` 로우패스 필터 추가

- **CAN 진단 프로토콜 신규 설계** (`can_driver.py`, `arbiter_node.py`, `README_CAN_PROTOCOL.md`,
  `henes_can.dbc`): `0x203 CONTROL_META`(TX, 로깅/CANoe 가시성 전용, 실제 제어엔 안 쓰임 -
  `0x200`과 같은 틱에 rpm/steer/stop_mode 미러링 + `controller_id`/`seq` 추가)와
  `0x104 DIAG_STATUS`(RX, 펌웨어 실측 - `applied_stop_mode`/`fault_flags`/`steer_pwm_duty`/
  `supply_voltage_mV`/`rx_seq_echo`). 설계 당시엔 펌웨어 미구현이었으나, 8/16 저녁 CANoe
  로그(`Logging_2.asc`) 확인 결과 팀원이 이미 구현해서 실제로 송신 중임을 확인함
  (`can_driver.py`의 "NOT YET SENT BY FIRMWARE" 주석은 갱신 필요, 아직 안 함).
  8/16 로그 분석 중 발견한 것들:
  - `supply_voltage_mV`가 17623개 프레임 전부 0 - 미배선/스텁 의심, 펌웨어팀 확인 필요
  - `rx_seq_echo` 라운드트립 0건 이상 없음 - seq echo 메커니즘 정상 동작 확인
  - `applied_stop_mode`가 대부분 1(flat)/2(hold)이고 0(disabled)은 거의 안 잡힘 - 요청
    (`0x200.stop_mode`)은 계속 0(normal)이었는데, 의미가 다르다는 건 알지만(README 참고)
    한 번 팀원 확인 필요

- **8/16 22:23 실주행 로그 분석** (`arbiter_can_20260816_222302.csv` +
  `drive_log_20260816_222303.csv`, `Logging_2.asc`와 시간대 겹침 확인:
  CAN 로그 절대시간 22:23:27~22:24:55 = ROS 로그 t=25.1~113.2s, 두 로그가 카메라→
  GPS_FALLBACK 전환 시점에서 서로 일치함):
  - **카메라 주행 중 rpm이 항상 130.0 고정, 분산 0** (커브에서 최대 -14.3°까지 꺾여도
    안 바뀜) - `yolopv2_zed_rpm_node.py` 자체엔 커브 감속 로직(`_speed_for_steer`,
    `auto_speed`/`steer_deadzone_deg=2.0`/`steer_full_deg=18.0`(30°-스케일 기준,
    물리각 환산 시 약 8.58°)/`rpm_turn_scale=0.8`)이 있지만, 이 런에서
    `can_enable=false`였어서 그 블록 자체가 실행 안 됐고(카메라는 steer 토픽만 publish),
    실제 CAN을 쏘는 `arbiter_node.py`가 카메라 주행 시 `base_rpm`을 자기 파라미터
    `camera_mode_rpm` 고정값으로만 쓰기 때문 - 카메라의 곡률 계산 결과 자체가 rpm에는
    전혀 반영 안 되는 구조. 필요하면 이거 arbiter에서 걸 수 있음 (현재는 보류).
  - `yolopv2_zed_rpm_node` 프로세스가 시작 302.0초 후 **`exit code -6`(SIGABRT)로 크래시**
    (`~/.ros/log/2026-08-16-22-23-02-*/launch.log`) - drive_log 마지막 t_s(300.96s)와
    거의 일치. "로그를 한참 뒤에 껐다"가 아니라 크래시로 끝난 것. 원인 미조사.
  - GPS `cruise_rpm=140`/`min_curve_rpm=50` 커브 기반 감속 확인 (`_curvature_scaled_rpm`) -
    직선 구간에서 135~140대 찍히는 게 정상 동작.
  - `camera_ok` 판정 3조건 정리: ① `camera_active`(lane_valid 프레임 히스테리시스,
    `camera_bad_frames_to_disable=10`/`camera_good_frames_to_enable=3`, 순수 프레임
    카운트로 시간 무관 - 실측 20Hz 설계 대비 어제 실제 13~15fps로 밀림, 10프레임이면
    설계상 500ms인데 실측 기준 670~770ms), ② freshness dead-man's switch
    (`camera_timeout_sec=1.0s`), ③ GPS cross-track veto(`camera_max_deviation_m=2.5m`
    넘으면 강제 스왑, 재진입은 `camera_deviation_reenter_m=2.5m` 안으로
    `camera_deviation_reenter_streak=20`틱 연속 필요, `camera_deviation_lockout_count=3`번
    /`camera_deviation_lockout_window_sec=20s` 안에 반복 트립되면
    `camera_deviation_lockout_sec=15s` 강제 락아웃 - 이땐 차선 바로 보여도 안 풀림).

- **`arbiter_node.py`에 `base_steer` EMA 로우패스 필터 추가** (신규 파라미터
  `base_steer_lowpass_alpha`, 기본값 `1.0` = 필터 끔, 기존 동작과 100% 동일 - opt-in):
  `filtered = alpha*현재값 + (1-alpha)*이전값`. `base_steer`(평상시 카메라/gps_fallback
  주행값 - traffic_light 구간, 아직 안 맞물린 상태로 GPS가 주차존 그냥 통과하는 구간에서도
  재사용됨)가 계산되는 딱 한 지점에 적용해서 하위 소비처 전부에 자동 반영되게 함. 소스가
  camera<->gps_fallback로 전환될 때도 그대로 블렌딩됨(의도된 동작 - 전환 시 튀는 것도
  같이 완화하려는 목적). `base_source`가 `None`(카메라·GPS 둘 다 무효)이 되면 필터 상태
  리셋 - safe_stop/이벤트존 통과 후 재진입 시 오래된 값에서 블렌딩 시작하는 걸 방지.
  avoid/parking-engaged/event-stop 등 자체 정밀 조향 로직을 쓰는 구간은 이 필터 영향 안 받음
  (그쪽은 각자 이미 별도로 값 관리함). 아직 실차 테스트 안 됨 - alpha 튜닝값 미정.

## 2026-08-17 (이어서): GPS 커브 감속에 데드존 추가 (`curve_deadzone_angle_deg`)

교수님 피드백: 각도(커브 곡률) 따라 속도 줄이는 로직에 "특정 각도 이내면
그냥 최대 속도, 그 이후 선형적으로 내리다 최소값 되면 유지"하는 형태로
만들라는 지시. 기존 `_curvature_scaled_rpm()`은 `turn_angle > 0`이면 바로
선형 감속이 시작되는 구조였음(데드존 없음) - `curve_deadzone_angle_deg`
파라미터(기본값 **5.0**, 사용자 판단 근거로 채택) 추가해서
`0~5° -> cruise_rpm(140, 평평) -> 선형 감속 -> curve_angle_for_min_rpm_deg
(40°) 이상 -> min_curve_rpm(50, 평평)` 구조로 수정. 기존 대비: `frac`
계산식이 `turn_angle / max_angle`에서 `(turn_angle - deadzone) /
(max_angle - deadzone)`로 바뀜. 실차 테스트 아직 안 함.

## 2026-08-17 (이어서): 카메라 곡률기반 rpm을 arbiter가 실제로 쓰도록 배선

이전까지 카메라 노드(`yolopv2_zed_rpm_node.py`)엔 곡률기반 rpm 스케일링
로직(`_speed_for_steer`, `auto_speed`/`steer_deadzone_deg=2.0`/
`steer_full_deg=18.0`/`rpm_turn_scale=0.8` - 상한(데드존, 각도 작으면
그냥 max)/하한(풀커브, `rpm_turn_scale`로 clamp) 둘 다 이미 있었음)이
있었지만 두 겹으로 막혀서 실제로는 전혀 안 쓰이고 있었음:
1. `integrated_drive.launch.py`/`post_gps_drive.launch.py`가 `can_enable:
   False`로 카메라 노드를 띄우는데(arbiter가 유일한 CAN 송신자여야 해서
   맞는 설정), `_speed_for_steer` 계산 자체가 `if self.can is not None:`
   블록 안에 있어서 통째로 안 돌았음
2. 설령 돌았어도 arbiter가 카메라 주행 시 rpm은 자기 고정 파라미터
   `camera_mode_rpm`만 썼음 - 카메라 계산 결과 자체를 안 받음

**수정:**
- `yolopv2_zed_rpm_node.py`: rpm_target 계산(+스텝 제한)을 `can_enable`
  게이트 밖으로 빼서 항상 실행, `~/rpm_target` 토픽으로 항상 publish
  (`~/motor_rpm`은 그대로 둠 - 그건 오도메트리 기반 속도추정값이라 다른
  의미)
- `arbiter_node.py`: `camera_rpm_topic`(기본
  `/yolopv2_zed_node/rpm_target`) 구독 추가, `_on_camera_rpm` 콜백,
  카메라 주행 시 `base_rpm = self.camera_rpm`으로 교체 (기존
  `camera_mode_rpm` 파라미터는 첫 메시지 오기 전 seed 값/폴백으로만 남음)
- `integrated_drive.launch.py`/`post_gps_drive.launch.py`: 카메라 노드에
  `auto_speed: true`, `can_target_rpm`(신규 인자 `camera_can_target_rpm`,
  기본 130 - `camera_mode_rpm`이랑 값은 맞췄지만 int 타입 캐스팅 문제
  때문에 별도 인자로 분리, `int("130.0")`이 에러나서) 추가

`can_enable: False`는 그대로 유지 - CAN은 여전히 arbiter만 씀, 이번
수정은 "계산은 하되 CAN 전송은 안 함, 결과만 토픽으로 넘김" 구조.
아직 실차 테스트 안 함.

## 2026-08-18: "stop" 이벤트존에 타이밍 정지(hold_sec) 지원 추가

Hill_Stop(언덕정지, idx 정지 + stop_mode 전환)이 아직 미구현이라, 그
자리에 우선 "idx에서 N초 정지 후 자동 재출발"만 되는 간단 버전을 넣어서
테스트해보려는 목적. `stop_mode` 전환은 안 함 - 순수 타이밍 정지만.

- `parse_event_zones`/`_zone_at`: 4번째 필드(`extra`)를 이제 zone
  kind별로 다르게 해석 - `traffic_light`는 여전히 stopline(없으면
  `end`로 폴백, 기존과 동일), `stop`은 hold_sec(없으면 무한정지, 기존과
  동일 - 하위호환).
- `"stop"` 브랜치: `hold_sec`이 있으면 그 zone에 처음 진입한 시각을
  기록해두고, 경과시간이 `hold_sec` 넘으면 `base_steer`/`base_rpm`(카메라
  또는 gps_fallback, 그 순간 뭐가 유효하냐에 따라)으로 자동 재개. idx가
  실제로 그 존을 벗어나면(차가 다시 움직이면서) 상태 리셋 - 다음 랩에
  같은 존 다시 만나면 또 처음부터 정지.
- 포맷: `"start:end:stop:hold_sec"`, 예: `"44:44:stop:3"` (idx 44에서
  3초 정지 후 재출발). `hold_sec` 생략하면 기존처럼 무한정지.
- `post_gps_drive.launch.py`의 `EVENT_ZONES`를 `["44:44:stop:3"]`으로
  임시 설정 (테스트용, 8/18 저녁 새로 기록한 코스
  `path_20260818_145848.csv` 기준). 실차 테스트 예정.

## 2026-08-18 (이어서): 타이밍 정지에 stop_mode=2(hill) 반영

바로 위 hold_sec 타이밍정지 테스트에서, 정지 중 stop_mode를 1(flat)
대신 **2(hill)**로 보내도록 수정 - 사용자 요청("의도완 다른데 암튼
작동하는거아니까"). 정확한 설계는 아님(진짜 Hill_Stop은 stop 존의
변형이 아니라 별도 state가 돼야 함, 다이어그램에도 그렇게 그려둠) -
CAN 레벨에서 stop_mode=2 동작 자체를 실차로 확인해보려는 임시 테스트.
정지 중(hold 안 끝난 동안)만 stop_mode=2, hold_sec 없는 기존 무한정지
zone은 그대로 stop_mode=1 유지.

## 2026-08-18 (이어서): 커브 조기 꺾임 - `curve_lead_margin` 1.5→1.2, launch 인자로 노출

실차 테스트 중 커브에서 너무 일찍 꺾이는 증상 발견 - Stanley의 예견
(anticipatory) blend(`stanley_control()`의 `curve_lead_margin`,
`required_lead_m = turning_radius_m * radians(turn_angle_deg) *
curve_lead_margin`)가 원인으로 추정됨. 이 값은 7/30ish 세션에 "반응이
너무 늦어서 코너 바깥으로 밀린다"는 반대 증상을 고치려고 1.1→1.5로
올렸던 건데, 지금은 과하게 일찍 반응하는 쪽으로 넘어간 상태로 보임.

- `waypoint_follower_node.py`의 `curve_lead_margin` 기본값 1.5→**1.2**로
  낮춤
- `integrated_drive.launch.py`/`post_gps_drive.launch.py`에 `
  curve_lead_margin` launch 인자로 노출 - 재빌드 없이 실차에서 바로
  튜닝 가능 (`curve_lead_margin:=1.0`, 더 낮춰보거나 `curve_lead_margin:
  =0.0`으로 예견 blend 자체를 꺼볼 수도 있음 - 0이면 `required_lead_m`도
  0이 돼서 무조건 blend=0, 순수 반응형 Stanley로 돌아감. 단, "코너가
  회전반경보다 급해서 물리적으로 못 돈다"는 별도 안전장치(full-lock
  폴백)는 이 값과 무관하게 계속 작동함)

stop_mode=2(hill)는 CAN 인코딩 자체(호스트 쪽 struct.pack)는 문제없음
재확인 - 펌웨어가 hill 모드 액추에이션을 아직 구현 안 했을 가능성이
높음(기존에 StopHoldEnable 하나로 normal/hill 구분 못한다고 기록해둔
내용과 일치), 펌웨어팀 확인 필요.

## 2026-08-18 (이어서): 타이밍 정지 무한반복 버그 수정 (`_stop_hold_fired_once`)

실차 테스트 중 발견: stop_mode=2가 실제로 언덕에서 차를 못 붙잡아서
정지 중 차가 밀려 내려감(펌웨어 hill-hold 미구현 - 위 항목 참고) ->
GPS idx가 그 여파로 다시 zone 안으로 들어옴 -> 정지 재발동 ->
또 밀림 -> 무한반복.

`_stop_hold_fired_once`(zone (start,end) 집합, 이번 실행 동안 영구
유지 - idx가 zone을 벗어나도 리셋 안 됨, 기존 `_stop_hold_key` 등
3개와 다름) 추가 - 한 번 hold_sec 다 채우고 정상 완료된 zone은
그 세션 동안 다시는 안 걸리고 그냥 통과함. 재출발 안전장치와 별개로,
"한 번 시도했으면 그걸로 끝"이라는 사용자 요청 반영.

## 2026-08-18 (이어서): 타이밍 정지 무한반복 버그 재수정 - fired_once를 완료 시점이 아니라 진입 시점에 마킹

바로 위 수정(`_stop_hold_fired_once`)이 실차에서 여전히 무한반복함 -
"밀렸다 앞으로 갔다"를 계속 반복. 원인: `fired_once`를 hold_sec **다
채운 뒤에만** 추가하고 있었는데, 언덕에서 진짜로 안 붙잡혀서(펌웨어
hill-hold 미구현) hold_sec 채우기 전에 idx가 zone 밖으로 밀려나가면
`_stop_hold_key`/`_stop_hold_done`이 리셋되면서 `fired_once`엔 아무것도
안 남고, 다시 idx 44 들어오면 완전히 처음부터 재시도 - 못 버티는 상황
자체가 반복되니 똑같이 계속 반복됨.

**수정**: zone에 **처음 진입하는 순간** 바로 `fired_once`에 추가하도록
변경(성공적으로 hold_sec 다 채웠는지와 무관). 이제 "한 번 시도(끝까지
버텼든 밀려서 중간에 빠졌든)하면 이번 실행 동안은 그걸로 끝" - 사용자가
원한 정확히 그 동작.

## 2026-08-18 (이어서): 타이밍 정지 hold 중 enable=1로 시험

hold 중(정지 유지하는 3초 동안) `enable=0`으로 보내던 걸 **`enable=1`**로
바꿔서 시험 - 사용자 가설: hill-hold는 모터를 완전히 꺼서(freewheel)
버티는 게 아니라, 모터를 켠 채로 target rpm=0을 유지하면서 중력에
반하는 토크를 실제로 걸어야(액티브 홀드) 되는 거 아니냐는 것.
`stop_mode=2`는 그대로. hold 끝나고 재출발/스킵 분기는 안 건드림.
실차 테스트 예정 - 이것도 효과 없으면 펌웨어 액추에이션 자체를
다시 봐야 함.

## 2026-08-18 (이어서): "한 번만" 정지존 되돌림 - idx 44 조기 오탐으로 매번 스킵되던 문제

`_stop_hold_fired_once`("세션당 한 번만") 완전히 되돌림. 실차 로그
확인 결과, launch 재시작 직후부터 이미 "already held once - skipping"
로 찍힘 - GPS idx가 실제로 그 zone에 도달하기 전에, 시작 시점의
nearest-waypoint 탐색 튐(jitter)으로 잠깐 idx=44를 스쳤다가 곧바로
`fired_once`에 마킹돼버려서, 정작 차가 진짜로 그 구간에 도달했을 땐
매번 무조건 스킵되고 있었음.

사용자 판단: 반복 트리거 방지보다 "일단 정지가 되는 것" 자체가 우선.
`_stop_hold_key`/`_stop_hold_start_time`/`_stop_hold_done`(idx가 zone을
벗어나면 리셋되는 기존 3개)만 남기고, 매번 재진입할 때마다 다시
정지를 시도하는 원래 방식으로 되돌림 - 무한 반복(밀림-재시도) 가능성은
다시 있음, 감수하기로 함. `enable=1`/`stop_mode=2` 변경은 그대로 유지.

## 2026-08-18 (이어서): gps_priority 존 2개 추가 (idx 30-35, 65-73)

`waypoint_idx_viewer.png`로 오늘 새로 기록한 코스(`path_20260818_145848.csv`,
233개 웨이포인트)의 idx 위치 확인 후, `post_gps_drive.launch.py`의
`EVENT_ZONES`에 두 구간 추가:
- `"30:35:gps_priority"`
- `"65:73:gps_priority"`

기존 `"44:44:stop:3"`(타이밍 정지 테스트)는 그대로 유지 - 세 구간
idx 범위가 서로 안 겹침. gps_priority는 그 구간 동안 카메라(ZED)를
배제하고 GPS만 운전하는 존 - 이 구간에서 슬롯 탐지 같은 걸 카메라가
방해하지 않게 하려는 용도(코드/설계는 기존 그대로, 새 코스에 맞춰
idx만 새로 지정).

## 2026-08-18 (이어서): 카메라 곡률기반 rpm을 arbiter가 실제로 쓰게 배선 (auto_speed 활성화)

새 CAN 메시지 `0x204 GPS_NAV_STATUS` 추가 (host → AURIX, 로깅/CANoe
가시성 전용, `0x203 CONTROL_META`와 같은 취지) - "카메라 주행 중 GPS
경로에서 너무 멀어지면 GPS로 복귀한다"는 그 판단 근거를 CAN에도 실어서
CANoe에서 볼 수 있게 함:

```
byte0-1 : gps_idx               (uint16) - 지금 추종 중인 웨이포인트 인덱스
byte2-3 : cross_track_error_cm  (int16, cm) - 경로 대비 부호 있는 횡오차
byte4-7 : 미사용
```

`arbiter_node.py`가 실제 CAN 프레임(`0x200`) 보낼 때마다 `0x203`이랑
같이 이것도 쏨 (`_log_can()` 안에서). `cross_track_error_cm=-32768`이면
"값 없음"(NaN/stale) - 0이랑 구분하려고 별도 sentinel 씀. 펌웨어가
이 ID를 구독 안 해도(파싱 안 해도) **차량 동작엔 전혀 지장 없음** -
CAN 브로드캐스트 특성상 관심 없는 ID는 그냥 무시되는 게 정상 동작.

`can_driver.py`/`henes_can.dbc`/`README_CAN_PROTOCOL.md` 셋 다 갱신.
겸사겸사 `can_driver.py`의 `parse_diag_status()` 주석에 남아있던 stale
"NOT YET SENT BY FIRMWARE"도 8/16 로그로 실제 송신 확인된 내용으로
갱신함.

## 2026-08-18 (이어서): gps_priority 존 폭 확대 (30-35 → 28-40), 신호등 체크 토글 추가

**신호등 체크 토글**: `gps_priority`/`gps_priority_slow`의 2026-08-16 red-light
fail-safe(`traffic_light_node` 안 돌고 있으면 무조건 RED로 간주 -> 정지)가
OAK-D 없이 테스트할 때 zone이 고장난 것처럼 보이게 만듦(GPS로 안 넘어가고
그냥 멈춰버림). `gps_priority_check_traffic_light` 파라미터(기본
`True`, `post_gps_drive.launch.py`의 launch 인자는 이 런처의 실제 사용
맥락 맞춰 기본 `False`)로 토글 가능하게 함 - OAK-D 붙이면 다시 true로.

**gps_priority 존 폭**: 실차 테스트에서 idx 30~35 구간이 91초 동안
`camera`/`event_zone_gps_priority` 사이를 계속 플랩핑함(로그 확인:
`t=68~159s` 구간에서 camera 1513틱 vs gps_priority 193틱) - idx가 좁은
구간 경계를 GPS 지터로 계속 넘나든 것으로 추정(zone 자체 로직은 정상
동작 확인됨). 구간을 **28~40으로 넓힘** (기존 30~35).

이날 밤 테스트는 언덕 지나서 정지, 수동 조종으로 복귀하며 종료.

## 2026-08-18 (이어서): gps_priority에도 base_steer_lowpass_alpha 필터 적용

실차 CAN 로그(`arbiter_can_20260818_183017.csv`)로 확인: `gps_priority`
구간 진입 직후(t=36~42s) `self.gps_steer`(Stanley 원본값)가 필터 하나도
안 거치고 그대로 CAN에 나가서 **±14.3° 풀락을 6초 안에 여러 번** 왔다갔다함
(사용자가 실차에서 "왼쪽 오른쪽 한번씩 확 땡겼다"고 느낀 순간과 일치).
`base_steer_lowpass_alpha` 필터는 `base_steer`(평상 카메라/gps_fallback
주행) 한 곳에만 걸려있었고 `gps_priority`/`gps_priority_slow`는 처음부터
범위 밖이었음.

**수정**: `self._filtered_gps_steer`라는 독립된 필터 상태 추가 (같은
`base_steer_lowpass_alpha` 파라미터 재사용, `gps_ok` 기준으로 리셋 -
`base_source`/카메라 상태와 무관하게 계속 동작해야 해서 `base_steer`
필터 상태랑은 분리함). `gps_priority`/`gps_priority_slow` 두 분기 4곳
전부 `self.gps_steer` → `self._filtered_gps_steer`로 교체.

`arbiter_node.py` 자체 기본값은 여전히 `1.0`(꺼짐, 다른 데 영향 없음).
**`post_gps_drive.launch.py`는 이번 확인된 실제 문제 때문에 launch
기본값을 `0.3`으로 잡음** (20Hz 기준 시상수 약 0.2초) - 실차에서
`base_steer_lowpass_alpha:=` 값 바꿔가며 튜닝 가능.

## 2026-08-18 (이어서): 타이밍 정지를 순수 시간 기반으로 재작성 (idx 벗어나도 hold_sec 다 채움)

실차 확인: `44:44` 같은 웨이포인트 1개짜리 zone은 130rpm 관성으로
0.4초 만에 통과해버려서, hold_sec(3초) 로직이 `zone=="stop"`(idx가 그
1개 지점에 머무는 동안)에만 걸려있던 기존 구조로는 3초를 채울 수가
없었음 - "멈추려는 시도조차 없어 보인다"는 실측 결과.

**재설계**: `_stop_hold_active_until`(시각) 도입 - 한 번 hold가
시작되면, idx/zone이 뭐가 됐든 상관없이 **on_timer 맨 앞에서 무조건
`stop_mode=2` 명령을 그 시각까지 계속 강제로 보냄** (`stop_holding`
플래그로 `engaged_side`/zone 분기 전체보다 우선). `_stop_hold_done_key`로
"이번 zone 방문에서 이미 hold 시작했음"을 표시해서, hold_sec 지난 뒤에도
idx가 아직 그 zone 안이면 재시작 안 하고 그냥 통과 - idx가 실제로
zone을 벗어나야 리셋되어 다음 방문(다음 랩) 때 다시 작동.

여전히 CAN 명령(`stop_mode=2`, `enable=1`) 자체가 펌웨어에서 실제
제동으로 이어지는지는 별개 문제 (펌웨어 hill-hold 미구현, 별도 확인
필요) - 이번 수정은 "호스트가 최소 3초는 확실히 그 명령을 계속
보낸다"는 것만 보장함.

## 2026-08-19: `gps_quality_monitor` 노드 추가 (EMI 테스트용 실시간 GPS 품질 모니터)

**배경**: ZED 카메라 전원만 넣어도(코드 안 돌려도), 심지어 GPS랑 완전히
분리된 다른 노트북에서 테스트해도 GPS Fixed에 영향이 간다는 게 확인됨
(2026-08-18 밤, 노트북 2대로 분리 테스트) - 기존에 알던 "USB3 케이블
근접 커플링" 모델만으론 설명 안 되는 **방사(radiated) EMI** 가능성.
코드 실행(실제 스트리밍) 시 영향이 더 심해짐 - 대기전력 노이즈 +
활동량 비례 노이즈 둘 다 있는 걸로 보임. 그런데 테스트 당시 "무슨 값이
0.0001에 들어와야 fixed"라는 관찰이 있었는데 정확히 어떤 지표를 보고
있었는지 사후에 특정 불가능했음(모니터링을 명확히 안 하고 있었음).

**해결**: `gps_quality_monitor` 노드 신설
(`waypoint_follower/gps_quality_monitor.py`, `ros2 run waypoint_follower
gps_quality_monitor`) - `/navpvt`(carrSoln, h_acc, pDOP, numSV)와
`/monhw`(jam_ind, jamming_state, noise_per_ms)를 한 줄에 라벨 붙여서
0.5초마다 출력. 필드명 정정: 이 ROS2 빌드의 `ublox_msgs/NavPVT`는
camelCase(`hAcc`)가 아니라 snake_case(`h_acc`, mm 단위) - 문서 찾다가
확인함. 앞으로 EMI 격리 테스트(전원만 vs 스트리밍, 카메라-안테나 거리
등) 할 때 이거 하나 띄워두면 어떤 값을 보고 있었는지 헷갈릴 일 없음.

패키지 의존성: `waypoint_follower/package.xml`에 `ublox_msgs` 추가.
아직 실제 EMI 테스트에 써보진 않음 - 다음에 카메라 전원 on/off 반복하며
`jamming_state`/`h_acc` 변화 직접 확인해보면 좋을 것 같음.

## 2026-08-19: 인수인계 패키지 (`HANDOFF/` 폴더) 작성

프로젝트 인수인계를 위해 `~/ros2_ws/HANDOFF/`에 주제별로 정리된 문서
8개 + `henes_can.dbc` 사본 작성함. 이 `README.md`(세션별 연대기 로그)는
"왜 이렇게 했는지" 상세 기록용으로 그대로 유지하고, `HANDOFF/`는 "지금
상태가 어떤지" 스냅샷 — 새로 맡는 사람(사람이든 AI든)이 처음부터 여기
읽으면 됨. `HANDOFF/00_START_HERE.md`가 진입점.

구성: 00_START_HERE(개요+읽는 순서), 01_ARCHITECTURE(시스템 구조+제어
우선순위), 02_HOW_TO_RUN(실행 명령어 전부), 03_TROUBLESHOOTING(겪은
문제+해결법), 04_PARAMETERS_REFERENCE(튜닝 파라미터 전체),
05_CAN_PROTOCOL(CAN 메시지 상세, README_CAN_PROTOCOL.md보다 최신 상태
반영), 06_KNOWN_LIMITATIONS_TODO(미해결 항목 우선순위), 07_FILE_MAP
(패키지별 파일 설명).

이 `HANDOFF/` 폴더는 2026-08-19 기준 스냅샷이라 그 이후 변경사항은 이
`README.md`(세션 로그)에서 계속 찾아야 함 - `HANDOFF/` 폴더도 주기적으로
최신화 필요.

## 2026-08-19 (이어서): gps_priority 진입 시 정착(settle-in) 블렌딩 추가

실차 로그로 원인 확인: `gps_priority` 진입 직전 5초+ 동안 카메라가
몰고 있었고, 그동안 GPS(Stanley)는 실제로 운전 안 하면서 "카메라가
끌고 간 궤적"을 보고 배경 계산만 계속 하고 있었음 - 그 계산이 실제
피드백 루프 없이 방치돼서 값이 불안정해지고, `gps_priority` 진입
순간 그 값이 처음 실제로 CAN에 실리면서 풀락 스윙으로 나타난 것.
GPS-only로 쭉 달릴 때는 계산-실제결과가 계속 맞물려있어서 이 문제가
없음(자연스러운 피드백 안정화).

**해결**: `_gps_priority_settled_steer()` 추가 - `gps_priority`/
`gps_priority_slow` 진입 시, 그 직전에 **실제로 나가고 있던 값**
(`_last_sent_steer`, 보통 카메라 값)에서 시작해서, `gps_priority_settle_sec`
(기본 2.0초) 동안 `gps_priority_settle_alpha`(기본 0.15, 평소
`base_steer_lowpass_alpha`보다 훨씬 느림)로 목표값(`_filtered_gps_steer`)
쪽으로 서서히 블렌딩. settle_sec 지나면 평소처럼 `_filtered_gps_steer`
그대로 씀. `gps_priority`↔`gps_priority_slow` 전환은 같은 정착 상태
유지(리셋 안 됨), 두 zone 다 벗어나야 리셋. `settle_sec<=0`이면 이 기능
자체가 꺼짐(기존 동작).

시뮬레이션 확인: -14.3°에서 시작해도 2초 안에 목표값(+5° 가정) 근처로
수렴함. 아직 실차 테스트 안 함.

## 2026-08-19 (이어서): 헤딩 계산 근본 수정 - EMA 블렌딩 + 거리가중 버퍼, IMU 미배선 확인

**IMU가 실제로 안 쓰이고 있었음을 확인**: `post_gps_drive.launch.py`가
띄우는 IMU는 `mrpt_sensor_imu_taobotics`(우리 `handsfree_ros2_imu` 패키지
아님, 시스템 apt 패키지) - `obstacle_avoid_node`/`parking_bridge`한테만
쓰이고 자기 토픽에 publish함. `waypoint_follower_node`의
`imu_topic`/`mag_topic`(기본 `/handsfree/imu`/`/handsfree/mag`)엔 아무도
publish 안 함 - `on_imu()`/`on_mag()`가 이 세션 내내 한 번도 안 불렸을
것으로 확인됨. 즉 `self.yaw`는 순수 GPS 움직임 기반 헤딩만으로 결정돼왔음
(자이로/지자기 보강 전혀 없이).

**수정 1 - EMA 블렌딩**: `on_fix()`가 `self.yaw = heading`으로 매번
완전히 덮어쓰던 걸 `heading_correction_alpha`(기본 1.0=기존과 동일)로
`self.yaw += alpha*normalize_angle(heading - self.yaw)` 방식으로 변경
- 각도라 단순 뺄셈 아니라 `normalize_angle`로 wraparound 처리.

**수정 2 - `_smoothed_heading()` 거리가중 버퍼로 재작성**: 기존엔 딱
2점(지금 위치 ↔ `heading_lookback_m` 뒤 점)만으로 베어링 계산 - 노이즈에
매우 취약. 이제 뒤로 걸어가며 지나친 모든 점 각각의 "지금→그 점" 베어링을
그 점까지의 거리로 가중(멀수록 가중치 큼, 짧은 baseline일수록 노이즈에
상대적으로 더 민감하므로)해서 벡터합으로 결합. **`heading_buffer_window_mult`
(신규 파라미터, 기본 5.0)이 핵심** - `heading_lookback_m` 채우자마자
멈추면(기존 동작, 실제 주행속도에서 점 2~3개밖에 못 모음, 개선 거의
없음) 의미 없고, 그 문턱의 5배까지 계속 더 모아야 실제로 효과가 큼
(시뮬레이션: 3cm GPS 지터 기준 헤딩오차 표준편차가 lookback=0.15m에서
9.0°→2.6°, lookback=1.0m에서 1.4°→0.3°로 개선 - `heading_lookback_m`
자체를 키우는 것도 별도로 도움 됨).

두 수정 다 IMU 없이 순수 GPS 이동거리만으로 동작. 아직 실차 테스트 안 됨.

## 2026-08-19 (이어서): `gps_quality_live_plot` 노드 추가 - EMI 차폐 테스트용 실시간 그래프

알루미늄 호일 등으로 ZED 감싸서 EMI 차폐 효과 테스트할 때, 텍스트
모니터(`gps_quality_monitor`)로는 변화 속도/패턴을 직관적으로 보기
어려워서 실시간 그래프 버전 추가.

`waypoint_follower/gps_quality_live_plot.py` (`ros2 run waypoint_follower
gps_quality_live_plot`) - matplotlib `FuncAnimation`으로 0.5초마다
갱신, 최근 `window_sec`(기본 120초) 롤링 윈도우:
1. `carrSoln` 계단식 (NONE/FLOAT/FIXED)
2. `h_acc`(cm, log 스케일)
3. `jam_ind` + `jamming_state` 배경색(초록/노랑/빨강)
4. `noise_per_ms` + `numSV`(트윈 축)

가짜 데이터(재밍 심한 구간 → 개선된 구간)로 렌더링 로직은 검증함(PNG
저장해서 확인) - 실제 `/navpvt`/`/monhw` 토픽 연결한 실차 테스트는
아직 안 함. rclpy 스핀은 백그라운드 스레드, matplotlib는 메인 스레드
(GUI 필요, 디스플레이 있는 환경에서 실행).

## 2026-08-23 `gps_quality_live_plot.py` 버그 2개 수정

사용자 리포트: "한글도 깨지는거 해결해주고 실시간으로 안됨?" - 코드
재검토로 원인 확인, 3곳 수정.

1. **한글 폰트 미설정** - 이 파일만 다른 시각화 스크립트들과 다르게
   `matplotlib.font_manager` 폴백 설정이 아예 빠져있었음(제목/라벨의
   한글이 네모로 깨지는 원인). `NanumGothic`/`Noto Sans CJK KR`/`Noto
   Sans KR`/`Malgun Gothic` 순으로 폴백 추가 - 이 머신엔
   `NanumGothicCoding`이 매칭돼서 잡힘(고정폭이라 살짝 투박하지만 안
   깨짐).
2. **`fig.texts.clear()`가 제목까지 지움** - 매 프레임(0.5초)마다
   "현재값" 텍스트를 `fig.text()`로 새로 만들기 전에 `fig.texts.clear()`
   로 전체를 지웠는데, `fig.suptitle()`도 내부적으로 `fig.texts`에
   들어가있어서 **첫 프레임 이후로 제목이 사라지는 버그**였음. 텍스트
   객체를 한 번만 만들어두고 이후엔 `.set_text()`로만 갱신하도록 수정.
3. **`FuncAnimation` 객체가 로컬 변수로만 존재** - matplotlib의 잘
   알려진 함정: `ani = animation.FuncAnimation(...)` 를 강한 참조 없이
   두면 일부 백엔드/GC 타이밍에서 애니메이션이 첫 프레임 이후 멈출 수
   있음. `fig._anim_ref = ani`로 `fig`에 참조를 붙여서 `plt.show()` 도는
   동안 확실히 살아있게 함. "실시간으로 안됨" 리포트의 유력 원인으로
   추정(다른 후보였던 스레드 안전성/백엔드 문제는 코드상 특별한 이상
   없음을 확인함).

검증: `Agg` 백엔드로 가짜 데이터 11프레임 애니메이션 렌더 → 매 프레임
갱신 확인, 제목/텍스트 끝까지 유지 확인, PNG로 한글 안 깨지는 것도
직접 확인. **단, 실제 `TkAgg`+디스플레이 환경에서의 진짜 실시간 동작은
아직 실차 테스트 안 됨** - 다음 EMI 차폐 테스트 때 확인 필요.

## 2026-08-23 (이어서) `gps_quality_live_plot.py` "실시간으로 안됨" 진짜 원인 발견 - 스레드 레이스 컨디션 크래시

사용자가 "실시간 안되는데?"라고 재확인 - 위 3개 수정으로는 실제
GPS(`f9p_rover.launch.py` 켜진 상태)로 직접 돌려서 재현해봄. **진짜
원인은 멈춤이 아니라 몇 초 만에 조용히 죽는 크래시였음**:

```
ValueError: x and y must have same first dimension, but have shapes (64,) and (65,)
```

`_on_navpvt`(ROS 백그라운드 스레드)가 `t_pvt`/`carr`/`h_acc_cm`/
`num_sv`/`p_dop` 5개 deque에 순서대로 `append()`하는 도중, 메인
스레드의 `redraw()`가 `list(buf["t_pvt"])`와 `list(buf["h_acc_cm"])`를
**따로따로** 읽다가 그 append 중간 타이밍에 끼어들면 한쪽은 65개, 다른
쪽은 64개로 길이가 어긋나서 `ax.plot(x, y)`가 `ValueError`로 죽음 -
Tkinter는 콜백 예외를 잡아서 로그만 찍고 애니메이션 타이머를 멈춰버려서
"창은 떠있는데 안 흐르는" 것처럼 보였던 것.

**수정**: `threading.Lock()` 도입 - `_on_navpvt`/`_on_monhw`의 append
묶음 전체를 락으로 원자화하고, `redraw()`도 락 안에서 8개 deque를 한
번에 스냅샷(`list()`)만 뜨고 바로 락을 풀어서 이후 플로팅은 락 없이
로컬 리스트로 처리(ROS 콜백 스레드를 오래 안 막으려고).

**검증**: 실제 `f9p_rover.launch.py` GPS 켜둔 상태로 `TkAgg` +
`DISPLAY=:0` 환경에서 직접 실행 - 수정 전엔 시작 직후 위 트레이스백과
함께 죽었고, 수정 후엔 1분 47초 동안 에러 없이 계속 실행됨(로그에
트레이스백 없음, 프로세스 살아있음 확인) → **실제 크래시 재현 및 수정
확인됨**. 화면 스크린샷 도구가 없어서 그래프가 시각적으로 흘러가는
것까지는 이미지로 확인 못했지만, 크래시 없이 장시간 도는 것 자체가
"실시간 안됨" 리포트의 근본 원인 해결을 의미함.

## 2026-08-25 HANDOFF에 ZED SDK 버전 요구사항 문서화 (팀원 컴퓨터에서 실제로 걸림)

팀원 컴퓨터에 프로젝트 넘겼는데 "SDK 버전이 안 맞다"며 카메라가 안
켜지는 문제 발생. 확인해보니 실제로 HANDOFF 문서 어디에도 ZED SDK
정확한 버전(5.4.1, 2026-07-28에 설치했던 그 버전) 및 CUDA 13.0+
요구사항이 안 적혀있었음 - 이 노트북 세팅할 때 겪었던 걸 문서화 안
해놓은 채로 넘긴 것. `zed-ros2-wrapper`는 소스에서 빌드되면서 설치된
ZED SDK를 링크하는데 메이저 버전 안 맞으면 빌드/런타임에 바로 거부함.

`HANDOFF/02_HOW_TO_RUN.md`에 "-1. 사전 요구사항" 섹션 신설(ZED SDK
5.4.1 고정 + CUDA 13.0+ + 버전 확인 명령어), `03_TROUBLESHOOTING.md`
카메라/GPU 항목에도 짧게 링크 추가. **Desktop zip은 기존 지시대로
갱신 안 함** - git으로만 반영.

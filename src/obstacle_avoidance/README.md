# obstacle_avoidance

HENES 실차(운전면허 시험장 S자 코스) 라이다(RPLiDAR A3M1) 기반 장애물 회피.
CAN(PCAN-USB)으로 실제 구동 컨트롤러와 직접 통신한다.

## 구성 노드

| 노드 | 역할 | 포트 권한 필요 |
|---|---|---|
| `sllidar_ros2 / sllidar_node` | 라이다 드라이버, `/scan` 발행 | O (`ttyUSB*`, dialout 그룹) |
| `mrpt_sensor_imu_taobotics / mrpt_sensor_imu_taobotics_node` | IMU 드라이버, `/taobotics/sensor`(`sensor_msgs/Imu`) 발행 | O (`ttyUSB*`, dialout 그룹) |
| `obstacle_avoidance / obstacle_avoid_node` | `/scan`+`/taobotics/sensor` 판단 + CAN 송수신까지 전부 처리 (단일 프로세스) | `can0` 인터페이스 올릴 때만 sudo 필요 |

`can_bridge_node`는 더 이상 없음 (CAN 통신 로직이 `obstacle_avoid_node` 안에 병합됨).

## 0. 매 터미널 공통

```bash
source /opt/ros/humble/setup.bash
source /home/a/ros2_ws/install/setup.bash
```

## 1. 라이다 실행

**먼저 어느 포트인지 확인** (재연결/재부팅마다 `ttyUSB` 번호가 바뀔 수 있음):
```bash
for dev in /dev/ttyUSB*; do echo "-- $dev --"; udevadm info -a -n $dev 2>/dev/null | grep -E "idVendor|idProduct"; done
```
`idVendor==10c4`, `idProduct==ea60` (Silicon Labs CP210x) 인 게 라이다.

```bash
ros2 launch sllidar_ros2 sllidar_a3_launch.py serial_port:=/dev/ttyUSB이거확인한번호
```

**포트 권한 에러(`Error, unexpected error, code: 80008004`) 날 때:**
- 해당 포트가 `dialout` 그룹 소유인데 계정이 그 그룹에 없으면 발생.
- `sudo usermod -aG dialout $USER` 후 재로그인 필요. 재로그인해도 안 되면(터미널 환경에 따라 그룹이 안 갱신될 수 있음) 아래처럼 감싸서 실행:
  ```bash
  sg dialout -c "bash -c 'source /opt/ros/humble/setup.bash && source /home/a/ros2_ws/install/setup.bash && ros2 launch sllidar_ros2 sllidar_a3_launch.py serial_port:=/dev/ttyUSB이거확인한번호'"
  ```

**`/scan` 이 안 나올 때 (프로세스는 떠있는데 `ros2 topic hz /scan` 이 아무것도 안 잡음):**
- 드라이버가 내부적으로 멈춘 것. `ros2 service call /stop_motor std_srvs/srv/Empty` 시도 후 프로세스 죽이고(`kill -INT <pid>`, 안 죽으면 `kill -9`) 재실행.

**종료할 때 (모터 계속 도는 것 방지):**
```bash
ros2 service call /stop_motor std_srvs/srv/Empty
```
그 다음 `Ctrl+C` (또는 `kill -INT <pid>`) 로 정상 종료. `kill -9` 로 강제종료하면 모터가 계속 돌 수 있음.

## 2. IMU 실행 (TAOBOTICS HFI, AVOID/RETURN 진행 판단용)

라이다와 마찬가지로 포트가 재연결/재부팅마다 바뀔 수 있음 - 같은 방법으로 확인
(라이다와 IMU 둘 다 CP210x라 `idVendor==10c4`,`idProduct==ea60`로 여러 개 나올 수
있으니, 실제로 하나씩 켜보면서 어느 게 라이다이고 어느 게 IMU인지 구분할 것).

```bash
sg dialout -c "bash -c 'source /opt/ros/humble/setup.bash && ros2 launch mrpt_sensor_imu_taobotics mrpt_sensor_imu_taobotics.launch.py serial_port:=/dev/ttyUSB이거확인한번호'"
```

`sensor_model` 파라미터 기본값은 `hfi-a9`. 지원 모델은 `hfi-b6`/`hfi-a9` 뿐이라
"HF1" 장비가 다른 프로토콜이면 이 값을 조정해야 할 수 있음 (2026-07-25 기준
기본값으로 정상 데이터 나오는 것 확인됨).

확인:
```bash
ros2 topic echo /taobotics/sensor --once
```
`orientation`(쿼터니언), `angular_velocity`, `linear_acceleration`(z축 ≈9.8~10, 중력) 값이
정상 범위로 나오면 OK.

미설치 시: `sudo apt install ros-humble-mrpt-sensor-imu-taobotics`

## 3. CAN 인터페이스 올리기 (한 번만, PC/어댑터 재연결 시 다시)

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
```

확인:
```bash
ip -details -statistics link show can0
```
`state UP`, `bitrate 500000`, `can state ERROR-ACTIVE`, `berr-counter tx 0 rx 0` 나오면 정상.
`ERROR-PASSIVE`나 `berr-counter`가 계속 올라가면 상대(차량 컨트롤러)가 ACK를 안 하고 있다는 뜻 — 아래 "CAN 문제 해결" 참고.

`obstacle_avoid_node`의 `auto_setup_interface: false`(기본값)라 노드가 이 단계를 대신 안 해줌 — 백그라운드/비대화형 실행에서는 sudo 비밀번호 입력이 안 되기 때문에 매번 직접 해줘야 함. 인터랙티브 터미널에서 직접 `ros2 launch`를 실행하는 경우엔 `auto_setup_interface: true`로 바꾸면 노드가 대신 해줌.

## 4. 회피 로직 + CAN 통신 실행

```bash
ros2 launch obstacle_avoidance obstacle_avoid.launch.py
```

기동 로그에 `CAN 연결 성공: can0` 뜨면 정상. `enable` 이 꺼져있는 동안은(기본값) 실제 차는 안 움직임.
IMU 가 아직 안 켜져 있어도 노드는 죽지 않고 뜸 (`IMU yaw 없음 - AVOID 진행 판단 불가` 경고만 찍힘 -
이 경우 AVOID 에 들어가도 진행이 안 되고 계속 머무름, 실제 주행 전엔 반드시 IMU 켜둘 것).

## 5. 모니터링

```bash
ros2 topic echo /avoid/state
ros2 topic echo /avoid/cmd_steer
ros2 topic echo /can/steer_current_angle_deg
ros2 topic echo /can/rpm_actual
ros2 topic echo /taobotics/sensor
candump can0
```

## 6. 실제 구동 ON/OFF ⚠️

**차 움직이게 하기 (arm) — 반드시 주변 안전 확인 후:**
```bash
ros2 topic pub --once -w 1 /can_bridge/enable std_msgs/msg/Bool "{data: true}"
```

**정지시키기 (disarm):**
```bash
ros2 topic pub --once -w 1 /can_bridge/enable std_msgs/msg/Bool "{data: false}"
```

`-w 1` 은 구독자(`obstacle_avoid_node`)가 붙을 때까지 기다렸다 발행하는 옵션. 이거 없이 `--once` 만 쓰면 타이밍이 꼬여 메시지가 씹힐 수 있음.

## 상태머신 요약

```
CLEAR --(장애물 감지)--> AVOID_LEFT/RIGHT --(회전각 도달)--> PASS --(통과거리 도달)--> RETURN --(원헤딩 복귀)--> CLEAR
                                                                                                   |
                                                                                                   +--(너무 가깝거나 넓음 / CAN 피드백 유실 / 조향 이상감지)--> STOP
```

- `AVOID/PASS/RETURN` 중에는 `/scan` 재판단을 잠근다 (회피 도중 연석 등이 다시 장애물로 잡혀 오작동하는 것 방지).
- `PASS` 는 회피각만 채우고 바로 복귀하면 장애물 뒷부분과 부딪힐 수 있어서, `obstacle_length + pass_margin` 만큼 더 직진(CAN 엔코더 거리 누적)한 뒤에야 `RETURN` 시작.
- **AVOID 진입 시 IMU yaw(`yaw_start`)를 기록**해두고, `AVOID`는 `|현재yaw - yaw_start| >= alpha_target`이 되면 `PASS`로, `RETURN`은 그 값이 `yaw_tol_deg` 이내로 다시 좁혀지면(=시작 헤딩으로 복귀) `CLEAR`로 전환한다. `yaw_start`는 AVOID→PASS→RETURN 내내 그대로 유지됨.
- `RETURN` 이 끝나면 `CLEAR` 로 돌아가 스캔 재판단이 다시 켜지므로, S자 코스에 장애물이 여러 개(2~4개) 있어도 순서대로 이어서 회피한다.
- `CLEAR` 상태에서는 `cruise_rpm` 으로 순항한다 (STOP 은 항상 rpm=0, 안전정지 유지).
- CAN 송수신·IMU 구독 모두 `obstacle_avoid_node`의 15Hz 타이머/콜백 안에서 직접 처리됨 (별도 노드 없음).

## 파라미터 즉석 조정 (재시작 없이)

```bash
ros2 param set /obstacle_avoid_node avoid_rpm 15
ros2 param set /obstacle_avoid_node safety_margin 0.2
ros2 param set /obstacle_avoid_node pass_margin 0.5
```

단, 이미 `AVOID/PASS/RETURN` 에 진입한 뒤면 그 사이클엔 반영 안 될 수 있음 (진입 시점 값으로 이미 고정됨). 확실히 하려면 노드 재시작(`Ctrl+C` 후 다시 `ros2 launch`)으로 `CLEAR` 상태부터 시작할 것.

## 영구 설정 변경

- [config/obstacle_avoid.yaml](config/obstacle_avoid.yaml) — 회피 로직 + CAN 통신 파라미터 전부 (스캔 범위, 차량 제원, 안전마진, CAN 채널/ID 등)

## 알려진 임시값 (실차 튜닝 필요)

| 파라미터 | 현재값 | 비고 |
|---|---|---|
| `safety_margin` | 0.15 m | 실험하며 조정 예정 |
| `reaction_margin` | 0.15 m | 실험하며 조정 예정 |
| `pass_margin` | 0.4 m | 실험하며 조정 예정 |
| `cruise_rpm` | 15 | CLEAR 순항 속도, 실험하며 조정 예정 |
| `yaw_tol_deg` | 4.0° | RETURN 완료로 볼 yaw 오차, 실험하며 조정 예정 |
| `front_angle_deg` | 35° | 임시값 |
| `max_consider_range` | 1.8 m | 임시값 |
| `min_range` | 0.20 m | 차체/그릴 가림 범위 확인 후 조정 |

## 캘리브레이션 (실측 확인됨)

- `angle_offset_deg: 180` — raw 180도가 정면. 자외선 때문에 라이다를 뒤집어 그릴에 장착.
- `lateral_sign: 1.0` — 2026-07-24 최종 실측값 (통행인 없는 상태에서 좌/우 각각 재확인). **라이다 마운트를 다시 만지거나 재장착하면 반드시 재검증할 것** (이번 세션 중 여러 번 바뀐 적 있음).

재검증 방법: 라이다 전방 창(±`front_angle_deg`) 안, 왼쪽/오른쪽 한 쪽에만 물체를 두고 `/avoid/state` 가 반대쪽으로 회피하는지(`AVOID_LEFT`/`AVOID_RIGHT`) 확인. 안 맞으면 물체를 치운 스캔과 놓은 스캔을 비교(diff)해서 실제 raw 각도를 찾아 `angle_offset_deg`/`lateral_sign` 재조정.
**주의: 주변에 사람이 지나다니면 스캔이 오염돼서 결과가 뒤죽박죽 나옴 — 반드시 통행 없는 상태에서 확인할 것.**

## CAN 문제 해결

`candump can0`로 봤을 때 우리가 보낸 `0x200`만 보이고 차량 쪽 `0x101`/`0x102`가 안 보이거나, `ip -s link show can0`에서 `ERROR-PASSIVE`/`berr-counter`가 계속 올라가면:

1. `can0` 다시 내렸다 올리기 (섹션 2)
2. PCAN-USB를 물리적으로 뽑았다 다시 꽂기 (인터페이스 재시작만으론 어댑터 내부 상태가 안 풀릴 수 있음)
3. 컨트롤러 보드와 PCAN-USB 사이 CAN-H/CAN-L 배선, 종단저항(120Ω) 확인
4. 컴퓨터 재부팅 (한 번 효과 있었음)
5. **그래도 안 되면 구동/조향 컨트롤러 보드 자체를 전원 껐다 켜기.** PC/어댑터/케이블 다 정상이어도 그 보드가 이전 세션에서 이상 상태로 남아있으면 `0x101`/`0x102`를 아예 안 보낼 수 있음 — 이번 세션에서 실제로 이걸로 해결됨 (다른 조치 다 해봐도 안 됐는데 보드 재부팅 후 바로 정상화됨).

`OSError: [Errno 105] No buffer space available` 이 나면 `obstacle_avoid_node`가 자동으로 3번 재시도하지만, 재시도로도 안 되면 위 순서대로 하드웨어 점검 필요. 코드가 원인이 아님을 확인했음 — 원본 `pcan_jetson_live.py` 스크립트로도 동일하게 재현됨.

## 알려진 이슈 (미해결)

- **왼쪽 조향(음수 steer 값)이 실차에서 안 먹힘 (2026-07-24 확인).** `AVOID_LEFT`(steer=-30)로 진입하면 `candump`으로 직접 확인해도 `0x200` 프레임에 `-30`이 정확히 인코딩되어 나가는데(`E2 FF`), 컨트롤러가 응답하는 `0x101` 피드백의 `target_angle`이 계속 `0.0`으로 나오고 실제 조향각도 전혀 안 움직임. **오른쪽 조향(steer=+30, `AVOID_RIGHT`/`RETURN`)은 정상 작동 확인됨.** 저희 코드/CAN 프레임 인코딩은 100% 정상임을 확인했으므로, 컨트롤러 펌웨어가 음수 조향값 처리에 문제가 있는 것으로 보임 — 펌웨어 담당자 확인 필요. 이 문제가 있는 동안은 `enable_steer_fault_check` 때문에 `AVOID_LEFT`가 계속 STOP으로 튕겨나감(정상적인 안전동작).

## 안전 유의사항

- `enable=True` 로 arm 하기 전 항상 차량 주변 안전 확인.
- CAN 엔코더 피드백이 1초 이상 끊기면 자동 `STOP` (단, 세션 시작 후 피드백을 한 번도 못 받은 경우엔 이 안전장치가 작동하지 않음 — CAN 없이 판단 로직만 테스트할 때를 위한 것). **CAN이 불안정하면 회피가 시작된 뒤 자동으로 안 끝날 수 있음 — enable을 바로 끌 수 있게 준비해둘 것.**
- IMU 피드백도 동일하게 1초(`imu_feedback_timeout_sec`) 이상 끊기면 AVOID/RETURN 중엔 자동 `STOP` (역시 한 번도 못 받은 경우는 예외 — IMU 없이 조향값만 테스트할 때를 위한 것). **IMU가 아예 없거나 죽은 채로 AVOID에 들어가면 진행이 안 되고 계속 그 조향/rpm을 유지한 채 멈추지 않으니, 실제 주행 전엔 반드시 `ros2 topic echo /taobotics/sensor`로 IMU가 살아있는지 확인할 것.**
- 조향 피드백이 명령과 15°(`steer_fault_tol_deg`) 이상, 0.5초(`steer_fault_hold_sec`) 넘게 차이나면 자동 `STOP` (단, `enable=False` 일 때는 이 검사를 하지 않음 — 정지 상태에서 안 움직이는 게 정상이므로).
- 조이스틱 수동 조작과 CAN 자동 제어가 동시에 같은 차량 컨트롤러에 명령을 보내면 충돌할 수 있음. 조이스틱 테스트 시엔 `enable` 을 반드시 False 로 둘 것.

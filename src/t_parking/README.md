# t_parking — 좌측 T자 후진 주차 (ROS2, 실차)

ROS1 미니카 성공판(`rule_based_t_parking_node.py`)의 **설계 원칙과 상태 흐름을 그대로 유지**하고,
플랫폼 차이만 반영한 ROS2 패키지.

```
IDLE → APPROACH → SETTLE → SETUP_ARC → SETTLE → REVERSE_ARC → SETTLE → REVERSE_STRAIGHT → DONE
```

- 후방 블라인드 전제: 슬롯이 옆으로 보이는 접근 구간에서만 관측하고 odom 에 고정
- 시간/순서 패턴 금지 — 세그먼트 전환은 **측정 yaw 도달**로만 판정
- 2원호 닫힌 공식 (`theta1 + dtheta2 = 90°`)

## 무엇이 바뀌었나

| # | 항목 | 내용 |
|---|------|------|
| 1 | ROS1 → ROS2 | rclpy. `tf.transformations` 제거(쿼터니언 직접 구현), latched → `TRANSIENT_LOCAL` |
| 2 | 차량 제원 | L=1.410 W=0.800 WB=0.735, base_link=뒤 차축, 뒤축→앞범퍼 1.175 / 뒤범퍼 0.230 |
| 3 | 주차 방향 | 우측 → **좌측** (`parking_side: left`, ROI `y>0`) |
| 4 | LiDAR 뒤집힘 | 회전만으로 표현 불가 → **반사(mirror)** 포함: 유효각 `= laser_yaw + sign·raw`, `sign=-1` |
| 5 | 엔코더 앞 차축 | `wheel_odom_front_axle_node.py` 신설. `ds_rear = ds_front·cos δ` |
| 6 | 콘 검출 | 전면 재작성 (`cone_detector.py`) — 고정 임계값 → 물리 모델 |
| 7 | 슬롯 검출 | 전면 재작성 (`slot_detector.py`) — 차선축 RANSAC + 내부 공실 검사 |
| 8 | 2원호 | 목표 깊이 분해 (`two_arc_planner.py`) — **아래 참조** |

### 2원호가 그대로는 안 되는 이유 (중요)

원본 공식의 실현가능 조건은 `y_a + z_goal ≤ R_rev` 다. 실차 수치를 넣으면:

```
R_rev = 0.735 / tan(30°) = 1.273 m
z_goal = 2.00(슬롯깊이) − 0.230(뒤범퍼) − 0.20(여유) = 1.570 m
y_a ≈ 1.00 m
→ 2.571 ≤ 1.273  ✗  단발 2원호 해가 수학적으로 존재하지 않는다
```

그래서 목표 깊이를 둘로 나눈다. 원호가 끝나는 시점에 차량은 이미 슬롯과 평행하므로,
남은 깊이는 원본에도 있던 `REVERSE_STRAIGHT` 가 그대로 처리한다.
**상태 흐름과 판정 방식은 원본과 동일하고, 목표 분배만 바뀐다.**

```
z_arc      = min(z_goal, R_rev − y_a − arc_depth_margin)   ← 원호 담당
z_straight = z_goal − z_arc                                 ← 직진 후진 담당
```

## 콘 검출을 "타이트하게" 잡은 방법

라이다를 높이 달아 콘이 잘 안 잡히는 문제는 **임계값을 조여서** 풀 수 없다.
조이면 먼 콘이 죽고, 풀면 노이즈가 통과한다. 그래서 거리마다 기대치를 계산한다.

```
w_exp = cone_base_diameter · (1 − scan_height / cone_height)      # 스캔면 단면폭
n_exp = 2·atan(w_exp / 2r) / dφ                                    # 예상 히트 수
```

게이트는 절대값이 아니라 이 기대치 대비 비율이다.

| 게이트 | 내용 | 무엇을 죽이는가 |
|--------|------|-----------------|
| 점수 | `0.30·n_exp ≤ n ≤ 3.20·n_exp` | 근거리 1점 노이즈 / 벽·사람 |
| 폭 | `0.25·w_exp ≤ chord ≤ 1.80·w_exp` | 큰 물체 |
| 적응형 분할 | Dietmayer `d_thr = r·sinφ/sin(λ−φ) + 3σ` | 근거리 과분할 / 원거리 과병합 |
| 고립성 | 양옆 이웃 빔이 `isolation_jump` 이상 멀어야 | 벽·연석·차체 |
| 볼록성 | 센서쪽으로 볼록한 짧은 원호 (sagitta) | 평면 벽 조각 |
| 반경 두께 | radial spread ≤ 콘 깊이 수준 | 깊은 물체 |

기본값(콘 0.45 m / 밑동 0.28 m / 스캔 0.16 m, A3M1 0.25°)에서:

```
w_exp = 0.180 m   →  3점 이상 13.8 m / 2점 이상 20.7 m 까지 검출 가능
```

즉 **검출 한계는 거리가 아니라 판정 로직**이었다. 콘 실측값을 넣는 것이 가장 중요하다.
`scan_height ≥ cone_height` 가 되면 `w_exp = 0` 이 되어 아무것도 못 잡는다 (노드가 시작 시 에러 로그).

## 각도 규약 검산

```
유효각 theta_v = laser_yaw + laser_yaw_extra + laser_angle_sign · theta_raw
                 (laser_yaw = π,  laser_angle_sign = −1)

raw 180° → 0°    차량 앞   ✓
raw  90° → +90°  차량 좌측 ✓
raw 270° → −90°  차량 우측 ✓
raw   0° → 180°  차량 뒤   ✓
```

제공된 실측 규약과 정확히 일치한다. 뒤집힘(`Rx(π)`)이 부호 반전으로 들어간 형태다.

## 빌드 / 실행

```bash
cd ~/ros2_ws/src && cp -r t_parking .
cd ~/ros2_ws && colcon build --packages-select t_parking --symlink-install
source install/setup.bash

sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

ros2 launch t_parking parking_left_oneshot.launch.py
# 준비되면 다른 터미널에서
ros2 topic pub -1 /parking_start std_msgs/Bool "{data: true}"
```

## 실차 투입 전 반드시 할 것 (순서대로)

ROS 없이 검출·계획 전체를 먼저 검증할 수 있다.

```bash
python3 tools/offline_check.py
```

**1) 각도 규약 확인 (차량 정지)**
`scan_parking_filter` 가 2초마다 정면/좌/우/후방 최소거리를 찍는다.
차 왼쪽에만 물체를 두고 `좌(+90)` 값이 줄어드는지 본다.
좌우가 뒤바뀌면 `laser_angle_sign` 을 `+1.0` 로, 앞뒤가 뒤바뀌면 `laser_yaw` 를 `0.0` 으로.

**2) 콘 실측값 입력** — `cone_height`, `cone_base_diameter`, `scan_height`.
지면부터 스캔면까지를 자로 재서 넣는다. 이 세 값이 검출 성능을 사실상 결정한다.

**3) 콘 검출 확인** — RViz 에서 `/parking/markers`.
`APPROACH(scan)` 로그의 `rej[...]` 카운터로 어느 게이트가 죽이는지 바로 보인다.
잘 안 잡히면 `n_exp_min_ratio` ↓ / `isolation_jump` ↓, 헛것이 잡히면 `min_track_hits` ↑.

**4) 회전반경 측정** — 최대 조향으로 좌/우 각각 정타 원선회 1회.
`max_steer_angle_*_deg` 가 실제 조향각과 맞는지 확인. bicycle 계산과 다르면
`radius_mode: table` 로 바꾸고 **뒤 차축 기준** 반경표를 넣는다.
미니카 반경표는 절대 재사용하면 안 된다.

**5) 조향 부호** — `forward_turn_sign`. 전진 중 조향 음수(좌)에서 yaw 가 증가해야 한다.
반대면 `+1.0`.

**6) CAN 매핑** — `dump_frames: true` 로 띄워 프레임을 보고
`encoder_can_id`, `count_byte_offset`, `count_byte_len` 을 채운다.
채운 뒤 차를 10 m 직진시켜 `/wheel_odom` 의 x 가 10 m 인지 확인.

**7) yaw 소스** — IMU 가 있으면 `yaw_source: fused` 유지를 권장한다.
90° 선회를 조향각 적분만으로 판정하면 조향 캘리브레이션 오차가 그대로 각도 오차가 된다.

**8) 첫 시도는 콘 없이** — 계획 로그(`plan.csv`)의 `required_aisle_width` 와
`sim_min_clearance` 를 먼저 확인한다. 기본 수치에서 필요 통로폭은 약 **1.77 m** 다.
통로가 그보다 좁으면 콘을 세우기 전에 `setup_steer_abs` / 접근 차선 위치를 조정해야 한다.

## 안전 장치

- `lock_require_feasible` — 실현 불가능한 계획으로는 lock 하지 않는다
- `lock_require_start_ahead` — 시작점을 이미 지난 후보는 버린다
- `validate_plan` — lock 직후 계획 전체를 이산 적분해 콘맵과 스윕 충돌 검사, 충돌이면 abort
- `interior_raw_reject` — 슬롯 내부에 점군이 있으면(다른 차가 주차됨) 후보에서 제외
- `rear_safety_stop_margin` — 후진 중 후방 여유 소진 시 정지

첫 시도는 `reverse_rpm`, `final_rpm` 을 절반으로 낮추고 비상정지를 손에 들고 진행할 것.

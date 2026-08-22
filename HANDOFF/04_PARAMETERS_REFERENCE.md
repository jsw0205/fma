# 파라미터 레퍼런스

토픽 이름 리매핑용 파라미터(`*_topic`류)는 대부분 생략 — 실제 튜닝
대상이 되는 값들 위주로 정리. 전체 목록은 각 노드 파일의
`declare_parameter(...)` 호출부에서 직접 확인 가능
(`grep -n 'declare_parameter(' <파일>`).

## `arbiter_node.py` (`control_arbiter`)

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `control_rate_hz` | 20.0 | 메인 루프 주기 |
| `steer_limit_deg` | 14.3 (`TRUE_STEER_MAX_ANGLE_DEG`) | 물리 최대 조향각 클램프 |
| `base_steer_lowpass_alpha` | **1.0**(코드 기본, 꺼짐) / **0.3**(`post_gps_drive.launch.py` 런치 기본) | 평상주행(`base_steer`) + `gps_priority`류 조향 EMA 필터. `filtered=alpha*현재+(1-alpha)*이전`. 1.0=필터 없음, 작을수록 부드럽지만 반응 느려짐(20Hz 기준 0.3≈시상수 0.2초 추정) |
| `camera_timeout_sec` | 1.0 | 카메라 steer 토픽 dead-man's switch |
| `camera_bad_frames_to_disable` / `camera_good_frames_to_enable` | 10 / 3 | lane_valid 프레임 히스테리시스 (순수 프레임 카운트, 시간 무관) |
| `camera_max_deviation_m` | 2.5 | GPS 대비 카메라 이탈 veto 진입 임계값 |
| `camera_deviation_reenter_m` / `_streak` | 2.5 / 20 | veto 해제 재진입 조건 (스트릭 필요) |
| `camera_deviation_lockout_count`/`_window_sec`/`_sec` | 3 / 20.0 / 15.0 | 반복 veto 시 완전 락아웃 |
| `camera_mode_rpm` | 100.0 (코드) / 130.0 (launch 기본) | **폴백 seed 값** — 실제로는 `camera_rpm_topic`(`~/rpm_target`)의 실측값을 씀 (2026-08-17부터) |
| `gps_priority_slow_rpm` | 80.0 | `gps_priority_slow` 존 속도 상한 |
| `gps_priority_check_traffic_light` | True(코드) / False(`post_gps_drive.launch.py` 기본) | OAK-D 없이 테스트할 땐 false로 — true면 신호등 미확인 시 fail-safe로 무조건 정지해버림 |
| `gps_priority_settle_sec` | 2.0 | **(2026-08-19 신규)** `gps_priority`/`gps_priority_slow` 진입 시 정착 블렌딩 지속시간. 0 이하면 기능 꺼짐 |
| `gps_priority_settle_alpha` | 0.15 | 정착 구간 동안의 블렌딩 세기(`base_steer_lowpass_alpha`보다 훨씬 느림) — 진입 직전 실제로 나가던 값에서 목표값까지 이 alpha로 서서히 수렴. 아직 실차 미검증 |
| `cruise_rpm`(waypoint_follower 쪽 값 참고) | - | GPS 순항 rpm은 `waypoint_follower_node`에 있음 (아래) |
| `event_zones` | `[""]` | 이벤트존 정의 리스트. `[]`(진짜 빈 리스트) 쓰면 파라미터 타입 추론 깨짐 — 꼭 `[""]` |
| `can_channel` | "can0" | |

## `waypoint_follower_node.py` (GPS/Stanley)

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `control_rate_hz` | 20.0 | |
| `stanley_k` | 0.3 | Stanley 횡오차 게인 |
| `stanley_k_boost` / `_duration_sec` | 1.0 / 2.0 | 일시적 게인 부스트 (특정 상황용) |
| `cruise_rpm` | 140 | 직선 순항 rpm |
| `min_curve_rpm` | 50 | 최대 커브 시 최소 rpm |
| `curve_deadzone_angle_deg` | **5.0** (2026-08-18 추가) | 이 각도 이내 커버쳐면 그냥 `cruise_rpm` 유지 (평평), 그 이상부터 `curve_angle_for_min_rpm_deg`까지 선형 감속 |
| `curve_angle_for_min_rpm_deg` | 40.0 | 이 각도(경로 커버쳐, **조향각 아님**) 이상이면 `min_curve_rpm`으로 평평 |
| `curve_lookahead_m` | 6.0 | 커브 감지 거리 (rpm 감속용 + Stanley 예견 blend 둘 다 이 값 공유) |
| `curve_lead_margin` | **1.2** (2026-08-18, 1.1→1.5→1.2로 조정됨) | 코너 예견 반응 배율. >1.0=물리적 최소거리보다 일찍 꺾기 시작. 크면 일찍 꺾음(너무 크면 조기꺾임), 작으면 늦게 반응(너무 작으면 바깥으로 밀림) |
| `heading_lookback_m` | 0.15 | 헤딩 추정 최소 baseline 거리 — 이 이상 움직여야 헤딩 갱신 |
| `heading_buffer_window_mult` | **5.0** (2026-08-19 신규) | `heading_lookback_m`의 몇 배까지 계속 점을 모아 거리가중 버퍼로 헤딩 계산할지. 1.0=예전 방식(점 2~3개만), 클수록 노이즈에 강함(시뮬레이션: 5.0이면 노이즈 표준편차 ~3.5배 감소) |
| `heading_correction_alpha` | 1.0(기존과 동일, 미조정) | GPS 헤딩을 `self.yaw`에 반영하는 EMA 강도 — 1.0=예전처럼 매번 완전히 덮어씀, 작을수록 부드럽게 수렴. `IMU가 실제로 배선 안 돼있음`(아래 참고)이 확인돼서 이게 사실상 유일한 헤딩 안정화 수단 |
| `lowpass_fc_hz` / `lowpass_fs_hz` | 2.0 / 10.0 | 위치+조향 EMA 필터 컷오프/샘플레이트. **`lowpass_fs_hz=10`이 실제 루프(`control_rate_hz=20`)랑 안 맞아서 의도한 2Hz가 아니라 실제로는 ~4Hz로 동작 중 — 버그로 의심되나 미수정** |
| `gps_outlier_threshold_m` / `_streak_accept` | 3.0 / 3 | GPS 이상치 거부 |
| `min_waypoint_distance_m` | 0.5 | 웨이포인트 로드 시 최소 간격 필터 |
| `waypoint_arrival_radius_m` | 0.5 | 도착 판정 반경 |
| `steer_sign` | -1 | 조향 부호 (하드웨어 배선에 따름, 함부로 바꾸지 말 것) |
| `enable_control` | False | true여야 실제 CAN 송신 |
| `publish_can_directly` | True(코드 기본) / **False(모든 통합 launch 파일)** | true면 이 노드가 CAN 직접 씀(단독 테스트용), 통합 실행 시 반드시 false여야 arbiter가 유일한 송신자가 됨 |

## `yolopv2_zed_rpm_node.py` (카메라)

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `can_enable` | (launch에서 항상 False) | **True로 바꾸지 말 것** — arbiter가 유일한 CAN 송신자여야 함. False여도 `rpm_target`은 계산+publish됨(2026-08-17부터) |
| `auto_speed` | True | 커브 기반 rpm 스케일링 on/off |
| `can_target_rpm` | 0 (코드 기본, **launch에서 130으로 오버라이드 필수** — 0이면 rpm 항상 0) | 커브 스케일링의 "직진 시 rpm" 기준값 |
| `steer_deadzone_deg` / `steer_full_deg` | 2.0 / 18.0 (**30°-스케일 기준**, 물리각 환산 시 각각 ~0.95°/~8.58°) | 이 이내면 max rpm, 이상이면 `rpm_turn_scale`까지 선형 감속 |
| `rpm_turn_scale` | 0.8 | 풀커브 시 rpm = `can_target_rpm × 0.8` |
| `rpm_step` | 8 | 한 틱(50ms)당 rpm 변화 제한 (급가감속 방지) |
| `steer_gain` / `steer_min` / `steer_max` | 1.0 / -30 / 30 | 최종 CAN steer 클램프 (30°-스케일) |
| `lane_max_steer_deg` | 30.0 | 내부 LaneTracker의 조향각 기준 스케일 (물리 14.3°가 아니라 30° 기준으로 계산됨) |
| `lane_curvature_gain_deg` / `_max_deg` | 8.0 / 10.0 | 곡률예측항 게인/클램프 |
| `stop_on_lane_lost` | True(추정, 코드 확인) | 차선 유실 확정 시 rpm 0 |
| `max_fps` | 50 | 추론 루프 상한 (실측은 이보다 낮게 나옴, 13~15fps 관측됨) |
| `device` | "0" | **절대 `"cuda"` 같은 문자열 주지 말 것** — `CUDA_VISIBLE_DEVICES`에 그대로 들어가서 GPU 다 숨겨짐, CPU로 떨어짐(~2fps) |

## `traffic_light_node.py`

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `max_fps` | 30.0 | 자체 스레드 루프 상한 (ROS 타이머 아님, OAK-D에서 직접 프레임 뽑음) |

## launch 파일에서만 존재하는 인자 (노드 파라미터 아님, launch 레벨 wrapper)

| 인자 | 기본값 | 설명 |
|---|---|---|
| `camera_can_target_rpm` | "130" | 카메라 노드의 `can_target_rpm`으로 전달 (int 캐스팅 문제로 `camera_mode_rpm`이랑 별도 인자로 분리돼있음 — `"130.0"`을 int로 캐스팅하면 에러남) |

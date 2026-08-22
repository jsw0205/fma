# 미해결/미검증 항목 — 다음에 할 일

우선순위 대충 순서대로.

## 0. IMU가 GPS 헤딩 융합에 실제로 안 쓰이고 있음 (2026-08-19 발견)

`post_gps_drive.launch.py`가 띄우는 IMU는 `mrpt_sensor_imu_taobotics`
(시스템 apt 패키지, 우리 `handsfree_ros2_imu` 아님) - `obstacle_avoid_
node`/`parking_bridge`만 이걸 씀. `waypoint_follower_node`의 `imu_topic`
/`mag_topic`(기본 `/handsfree/imu`/`/handsfree/mag`)엔 아무도 publish를
안 해서 `on_imu()`/`on_mag()`가 안 불림 - GPS 헤딩 융합이 자이로/지자기
보강 없이 순수 GPS 움직임만으로 돌아가고 있었음. 항목 4의 헤딩노이즈
문제의 근본 원인 중 하나. 정식으로 고치려면 `waypoint_follower_node`의
`imu_topic`을 `mrpt_sensor_imu_taobotics`가 실제로 publish하는 토픽에
맞춰 리맵해야 함(단 지자기는 이 드라이버가 아예 안 줄 수 있음, 확인
필요) - 지금은 항목 4의 GPS-only 버퍼링 개선으로 우회함.

## 1. 언덕정지(hill-hold) — 펌웨어 미구현, 실차로 확인됨

`stop_mode=2` + `enable=1`을 CAN에 정상적으로 보내는 것까진 확인됨
(호스트 쪽 코드 문제 없음). **근데 차량이 실제로 안 멈춤** — 언덕에서
관성/중력으로 계속 밀림. 펌웨어의 `Encoder_Motor_SetStopHoldEnable`이
실제로 저항 토크를 걸어주는 로직인지 펌웨어팀 확인 필요. 호스트 쪽에서
더 손볼 여지 없음 (`03_TROUBLESHOOTING.md` 참고).

**진짜 Hill_Stop 기능은 아직 설계만 돼있고 미구현** — 지금 있는 건
"idx에서 N초 정지 후 자동재개"하는 `stop:hold_sec` 타입의 임시
스탠드인일 뿐, 다이어그램에 그려둔 독립된 state로서의 Hill_Stop은
아님. `arbiter_node.py`의 "stop" 존 브랜치 주석에 관련 설명 있음.

## 2. ZED 카메라 방사 EMI가 GPS Fixed를 방해 — 원인/대책 미확정

2026-08-18 밤 발견: 완전히 분리된 노트북(공유 케이블 없음)에서도 ZED
"전원만" 켜면 GPS Fixed에 영향 감. 케이블 근접 커플링 모델로는 설명
안 됨. `gps_quality_monitor` 노드(`02_HOW_TO_RUN.md`) 만들어뒀으니 다음
테스트 때 이걸로 `jam_ind`/`jamming_state`/`h_acc` 값 변화 직접 확인해서
증거 모아둘 것. 대책 후보(검증 안 됨): 카메라 RF 차폐 케이스, 카메라-안테나
물리적 거리 확보, 전원 필터링.

## 3. `lowpass_fs_hz=10.0` vs 실제 루프 20Hz 불일치 — 논의만 하고 미수정

`waypoint_follower_node.py`의 Stanley 조향/위치 필터가 `fc=2Hz,
fs=10Hz`로 계산되는데 실제 루프는 20Hz라, 의도한 2Hz 컷오프가 아니라
실제로는 ~4Hz로 동작 중 (필터가 의도보다 약하게 걸림). 고치면 노이즈는
더 줄지만 반응은 더 느려짐(정착시간 0.12s→0.24s) — 트레이드오프라
사용자 판단 필요해서 보류함. 고치려면
`waypoint_follower_node.py`의 `lowpass_fs_hz` 기본값을 20.0으로.

## 4. 헤딩 노이즈로 인한 조향 풀락 오실레이션 — **수정함(2026-08-19), 실차 미검증**

차가 거의 안 움직일 때뿐 아니라 일반 주행 중에도 `heading_lookback_m
=0.15m`짜리 짧은 baseline이 GPS jitter를 헤딩 오차로 증폭시켜서 조향이
±14.3° 사이를 왔다갔다하는 현상 실차 CAN 로그로 확인됨(2026-08-18,
`cross_track_error_m`은 0.1~0.4m로 작았는데 steer는 풀락 - Stanley의
횡오차 항으론 설명 안 되고 헤딩오차 항이 원인으로 확인).

**추가로 확인된 사실**: IMU가 실제로 배선 안 돼있었음 - `mrpt_sensor_
imu_taobotics`(obstacle_avoid_node/parking_bridge용)만 떠있고,
`waypoint_follower_node`의 `imu_topic`/`mag_topic`엔 아무도 publish 안
해서 `self.yaw`가 자이로/지자기 보강 전혀 없이 순수 GPS 헤딩만으로
결정돼 있었음.

**수정**: (1) `heading_correction_alpha`로 GPS 헤딩 반영을 하드
덮어쓰기에서 EMA 블렌딩으로 변경 (2) `_smoothed_heading()`을 거리가중
버퍼(`heading_buffer_window_mult`)로 재작성 - 시뮬레이션상 노이즈
표준편차 ~3.5배 감소. `04_PARAMETERS_REFERENCE.md` 참고. **실차 테스트
아직 안 함** - 다음 주행 때 확인 필요.

## 5. `gps_priority` 존 idx 경계 폭 — 튜닝 진행 중

너무 좁은 존(예: 원래 30-35)은 GPS 위치 지터로 idx가 경계를 계속
넘나들어서 zone이 제대로 안 걸리는 것처럼 보임(camera/gps_priority
플래핑). 28-40으로 넓혀서 완화 시도함 — 완전히 검증된 건 아니라 재발
가능성 있음, 필요하면 더 넓히거나 idx 판정 자체에 히스테리시스
도입 고려.

## 6. `curve_lead_margin` 튜닝 계속 필요할 수 있음

1.1(너무 늦음) → 1.5(너무 이름) → 1.2로 재조정(2026-08-18). launch
인자로 바로 튜닝 가능하니 코스/속도 바뀌면 다시 맞춰볼 것.

## 7. 카메라 자체 커브감속 로직이 arbiter에서 실제로 쓰이는지 재검증 필요

2026-08-17에 `~/rpm_target` 토픽으로 배선해서 arbiter가 카메라 실제
계산값을 쓰게 고쳤음 — 근데 이게 실차에서 제대로 동작하는지 확인
테스트가 명시적으로 안 됨(카메라가 실제로 커브에서 rpm을 낮추는지
로그로 재확인 권장).

## 8. `yolopv2_zed_rpm_node` SIGABRT 크래시 원인 미조사

2026-08-16 22:23 런에서 302초 후 크래시 확인(`exit code -6`). 재발
시 `~/.ros/log/.../launch.log` 확인할 것.

## 9. 오른쪽 조향 최대각 미검증

`TRUE_STEER_MAX_ANGLE_DEG=14.3`은 **왼쪽 방향으로만 실측 검증됨**
(2026-07-26, GPS 2바퀴 원주행 반경 실측). 오른쪽도 동일한지 확인
안 됨 — `parallel_parking`의 `FINAL_ALIGN` 실패 사례가 이거랑 관련
있을 수 있다는 의심이 있었음(README 2026-08-05 근처 세션 참고).

## 10. `supply_voltage_mV` 항상 0

`0x104 DIAG_STATUS`의 이 필드가 실측 로그에서 항상 0 — 미배선/스텁
의심, 펌웨어팀 확인 필요.

---

## 완료된 것 (참고용, 재작업 불필요)

- `base_steer_lowpass_alpha` 필터 (평상주행 + `gps_priority`류 둘 다 적용됨)
- `gps_priority`/`gps_priority_slow` 진입 시 정착(settle-in) 블렌딩 (`gps_priority_settle_sec`/`_alpha`) — **아직 실차 미검증**, 다음 테스트 때 확인 필요
- `curve_deadzone_angle_deg` (커브감속 데드존)
- 타이밍 정지(`stop:hold_sec`)를 순수 시간기반으로 재작성 (idx 벗어나도 hold_sec 다 채움)
- `0x204 GPS_NAV_STATUS` CAN 메시지 신설
- `gps_quality_monitor` 노드 신설
- ADAS 아키텍처 다이어그램 (Artifact, `🚦` favicon으로 찾기)

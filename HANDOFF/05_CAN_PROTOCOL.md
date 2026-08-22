# CAN 프로토콜 (호스트 ↔ AURIX 펌웨어)

Classic CAN 2.0A, 표준 11비트 ID, 500kbps, 프레임 8바이트 고정. 구현은
`waypoint_follower/can_driver.py` 하나에 다 있음 — CAN 만지는 모든
노드가 이걸 통해서만 접근. 원본 DBC 파일:
`src/waypoint_follower/henes_can.dbc` (CANoe 임포트용).

**호스트 = Jetson/RPi(ROS2 스택), `control_arbiter`가 유일한 실제
제어 송신자.**

## ID 목록

| ID | 방향 | 이름 | 용도 |
|---|---|---|---|
| `0x200` | TX (host→AURIX) | (제어 프레임) | **실제로 차를 움직이는 명령** — rpm/steer/enable/stop_mode |
| `0x101` | RX (AURIX→host) | STEERING_STATUS | 조향 피드백 (포텐셔미터 실측/목표 각도) |
| `0x102` | RX (AURIX→host) | DRIVE_STATUS | 구동 피드백 (엔코더, 실측/목표 rpm, pwm) |
| `0x203` | TX (host→AURIX) | CONTROL_META | 로깅/CANoe 가시성 전용, **실제 제어엔 안 씀** |
| `0x104` | RX (AURIX→host) | DIAG_STATUS | 펌웨어 실측 진단 (실제로 뭘 적용했는지+고장) |
| `0x204` | TX (host→AURIX) | GPS_NAV_STATUS | 로깅 전용, gps_idx + cross_track_error |

`0x200`/`0x101`/`0x102`는 펌웨어 기존 프로토콜 그대로(안 건드림).
`0x203`/`0x104`/`0x204`는 2026-08-15~18에 추가된 신규 필드. **`0x203`/`0x204`는
펌웨어가 파싱 안 해도 차량 동작에 전혀 지장 없음** (CAN은 브로드캐스트,
구독 안 하는 ID는 그냥 무시되는 게 정상).

## 필드 상세 (start bit / bitcount / factor / signed)

### `0x200` (512) — 실제 제어
struct: `<hhBBH`
| 필드 | start bit | bitcount | factor | signed |
|---|---|---|---|---|
| rpm | 0 | 16 | 1 | signed |
| steer | 16 | 16 | 1 | signed (firmware-scale, `CAN_STEER_SCALE`=30/14.3≈2.098 이미 곱해짐) |
| enable | 32 | 8 | 1 | unsigned |
| stop_mode | 40 | 8 | 1 | unsigned (0=normal,1=flat,2=hill) |
| reserved | 48 | 16 | 1 | unsigned |

### `0x101` (257) — STEERING_STATUS
struct: `<HHhh`
| 필드 | start bit | bitcount | factor | signed |
|---|---|---|---|---|
| current_pot | 0 | 16 | 1 | unsigned (raw ADC) |
| target_pot | 16 | 16 | 1 | unsigned |
| current_angle | 32 | 16 | 0.1 | signed (deg, firmware-scale) |
| target_angle | 48 | 16 | 0.1 | signed |

### `0x102` (258) — DRIVE_STATUS
struct: `<hhhh`
| 필드 | start bit | bitcount | factor | signed |
|---|---|---|---|---|
| encoder_count | 0 | 16 | 1 | signed |
| rpm | 16 | 16 | 0.1 | signed (실측) |
| pwm_duty | 32 | 16 | 1 | signed |
| target_rpm | 48 | 16 | 1 | signed (0x200.rpm echo) |

### `0x203` (515) — CONTROL_META (로깅 전용)
struct: `<hhBBBB`
| 필드 | start bit | bitcount | factor | signed |
|---|---|---|---|---|
| target_rpm | 0 | 16 | 1 | signed |
| target_steer | 16 | 16 | 1 | signed |
| req_stop_mode | 32 | 8 | 1 | unsigned |
| controller_id | 40 | 8 | 1 | unsigned (아래 표) |
| seq | 48 | 8 | 1 | unsigned (0-255 롤오버, `0x104.rx_seq_echo`로 왕복확인) |
| reserved | 56 | 8 | 1 | unsigned |

**`controller_id` 값** (`can_driver.py`의 `CONTROLLER_*`):
0=SAFE_STOP, 1=GPS_FALLBACK, 2=CAMERA, 3=EVENT_STOP, 4=EVENT_GPS_PRIORITY,
5=EVENT_GPS_PRIORITY_SLOW, 6=EVENT_AVOID_SCAN, 7=EVENT_AVOID_ACTIVE,
8=EVENT_AVOID_FAILSAFE, 9~11=EVENT_PARKING_LEFT_{MAPPING,ACTIVE,WAIT},
12~14=EVENT_PARKING_RIGHT_{...}, 255=UNKNOWN.

### `0x104` (260) — DIAG_STATUS (펌웨어 실측)
struct: `<BBhHBB`
| 필드 | start bit | bitcount | factor | signed |
|---|---|---|---|---|
| applied_stop_mode | 0 | 8 | 1 | unsigned (0=disabled,1=flat,2=hold — **req_stop_mode랑 숫자는 같아도 의미 다름**, 아래 참고) |
| fault_flags | 8 | 8 | 1 | unsigned (비트필드, 아래) |
| steer_pwm_duty | 16 | 16 | 1 | signed |
| supply_voltage_mV | 32 | 16 | 1 | unsigned (**8/16 로그 기준 항상 0 — 미배선/스텁 의심**) |
| rx_seq_echo | 48 | 8 | 1 | unsigned (`0x203.seq` 그대로 echo) |
| reserved | 56 | 8 | 1 | unsigned |

**`applied_stop_mode` 재정의 이유**: 펌웨어 액추에이션 레이어가
`Encoder_Motor_SetStopHoldEnable`(boolean 하나)뿐이라 원 스펙 3단계를
그대로 못 씀 → 0=disabled(모터 꺼짐)/1=flat(coast)/2=hold(액티브홀드)로
재정의. **`0x200.stop_mode`(0=normal/1=flat/2=hill)랑 `0`의 의미가
다름** — CANoe에서 두 신호 나란히 볼 때 헷갈리기 쉬우니 주의.

**`fault_flags` 비트**: bit0=COMM_TIMEOUT(구현됨), bit1=POT_SENSOR(**드롭됨,
오탐 위험으로 요청 취소, 항상 0**), bit2=ENCODER_SENSOR(미구현), bit3=WATCHDOG_TRIP(미구현),
bit4=UNDERVOLTAGE(미구현) — bit1~4는 항상 0으로 옴.

**`applied_stop_mode`가 제일 중요한 필드** — 요청한 `stop_mode`랑 이게
다르면 "명령은 보냈는데 펌웨어가 안 따른" 걸 CAN 로그 하나로 바로 잡아낼
수 있음.

### `0x204` (516) — GPS_NAV_STATUS (로깅 전용, 2026-08-18 신규)
struct: `<HhHH`
| 필드 | start bit | bitcount | factor | signed |
|---|---|---|---|---|
| gps_idx | 0 | 16 | 1 | unsigned |
| cross_track_error_cm | 16 | 16 | 1 | signed (`-32768`=값없음/NaN sentinel, 0이랑 구분) |
| reserved | 32 | 16 | 1 | unsigned |
| reserved | 48 | 16 | 1 | unsigned |

## 호스트 쪽 구현 현황 (2026-08-19 기준)

- `can_driver.py`: 5개 메시지 전부 pack/parse 함수 구현됨
  (`make_control_data`/`send_control_true_deg`, `parse_drive_status`,
  `parse_steering_status`, `make_control_meta_data`/`send_control_meta`,
  `parse_diag_status`, `make_gps_nav_status_data`/`send_gps_nav_status`)
- `arbiter_node.py`: `_log_can()`이 실제 CAN 송신마다 CSV 로깅 +
  `0x203`/`0x204` 같이 송신. `_poll_diag_status()`가 `0x104` 받아서
  `applied_stop_mode` mismatch 감지 + 경고 로그.
- **펌웨어 쪽 `0x104` 실제 송신은 2026-08-16 CANoe 로그로 확인됨** (팀원이
  구현 완료한 걸로 보임) — `supply_voltage_mV`만 항상 0으로 나옴, 확인 필요.

## CANoe 채널 구성

- **채널1**: 실제 버스 액티브 참여
- **채널2**: 같은 물리버스에 **리슨온리**로 순수 트레이스/로깅만 —
  채널1 설정실수가 실제 버스에 영향 안 주게 분리

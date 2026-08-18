# HENES CAN 프로토콜 — CANoe 마이그레이션용 정리 (2026-08-15)

Classic CAN 2.0A, 표준 11비트 ID, 500kbps, 프레임당 8바이트 고정.
소켓 레벨 구현은 [`waypoint_follower/can_driver.py`](waypoint_follower/can_driver.py)
하나에 다 있고, 실제로 CAN을 쓰는 모든 노드(`arbiter_node.py`,
`parking_bridge/wheel_odom_pcan_node.py` 등)가 이걸 통해서만 CAN을 건드림.

## 전체 ID 목록

| ID | 방향 | 이름 | 상태 | 용도 |
|---|---|---|---|---|
| `0x200` | host → AURIX | (제어 프레임, 이름 없음) | **기존, 안 건드림** | 실제로 차를 움직이는 명령 (rpm/steer/enable/stop_mode) |
| `0x101` | AURIX → host | STEERING_STATUS | **기존, 안 건드림** | 조향 피드백 (포텐셔미터, 실측/목표 각도) |
| `0x102` | AURIX → host | DRIVE_STATUS | **기존, 안 건드림** | 구동 피드백 (엔코더, 실측/목표 rpm, pwm) |
| `0x203` | host → AURIX | **CONTROL_META** | **호스트 쪽 구현 완료** | 로깅/CANoe 가시성 전용 — 실제 제어엔 안 씀 |
| `0x104` | AURIX → host | **DIAG_STATUS** | **호스트 파싱 완료, 8/16 로그부터 펌웨어 실송신 확인됨** | 펌웨어가 실제로 뭘 적용했는지 + 고장 플래그 |
| `0x204` | host → AURIX | **GPS_NAV_STATUS** | **호스트 쪽 구현 완료 (2026-08-18)** | 로깅/CANoe 가시성 전용 — gps_idx + cross_track_error |

`0x200`/`0x101`/`0x102`는 펌웨어가 이미 파싱하는 프로토콜이라 **그대로 둠** —
새로 필요한 건 전부 새 ID들(`0x203`, `0x104`, `0x204`)로 추가.

---

## 왜 새 프레임이 필요했나

기존 3개 프레임 다 8바이트 꽉 차있어서 필드 추가할 자리가 없었고,
특히 이번 세션에서 실차 디버깅하면서 CAN만 보고는 못 풀고 매번 다른 방법
(펌웨어 C소스 직접 읽기, `candump` 수동 확인, 로그 파일 대조)으로 우회해야
했던 것들이 있었음:

- **"stop_mode를 보냈는데 펌웨어가 진짜 적용했는지"** — 판막(둥가둥가) 버그를
  펌웨어 소스(`Can_Comms.c`/`App_Control.c`) 직접 읽어서야 알아냄. CAN에
  그 결과가 안 실려있었음.
- **"지금 이 순간 누가 운전권 쥐고 있는지"** — 로그 파일끼리 시간 맞춰가며
  대조해야 알 수 있었음. CAN 자체엔 그 정보가 없었음.
- **CAN 프레임 드롭 여부** — 타임스탬프로 추정만 했지 정확히 셀 방법이 없었음.

## `0x203` CONTROL_META (host → AURIX)

**실제 제어에는 안 쓰임** — `arbiter_node.py`가 진짜 명령(`0x200`)을 보낼 때마다
**같은 값으로 이 프레임도 같이 쏨.** CANoe/로깅 쪽에서 "지금 왜 이 명령이
나갔는지"를 CAN만 보고 바로 알 수 있게 하는 게 목적.

```
byte0-1 : target_rpm       (int16)
byte2-3 : target_steer     (int16, 0x200의 steer 필드와 같은 raw 스케일)
byte4   : stop_mode        (uint8, 0=normal/1=flat/2=hill)
byte5   : controller_id    (uint8, 아래 표)
byte6   : seq              (uint8, 0~255 롤오버, DIAG_STATUS.rx_seq_echo로 왕복확인용)
byte7   : (미사용)
```
struct 포맷: `<hhBBBB`

### `controller_id` 값 (`can_driver.py`의 `CONTROLLER_*` 상수)

| 값 | 이름 | 의미 |
|---|---|---|
| 0 | SAFE_STOP | 아무것도 유효하지 않아 fail-safe |
| 1 | GPS_FALLBACK | GPS 웨이포인트 추종 (기본) |
| 2 | CAMERA | 카메라 차선추종 |
| 3 | EVENT_STOP | 강제정지 존 |
| 4 | EVENT_GPS_PRIORITY | GPS 전용 존 |
| 5 | EVENT_GPS_PRIORITY_SLOW | GPS 전용 + 속도제한 존 |
| 6 | EVENT_AVOID_SCAN | 회피존 진입, 아직 스캔만(GPS가 몲) |
| 7 | EVENT_AVOID_ACTIVE | 실제 회피 기동 중 |
| 8 | EVENT_AVOID_FAILSAFE | 회피 fail-safe 정지 |
| 9 | EVENT_PARKING_LEFT_MAPPING | T자 주차 슬롯 탐색 중 |
| 10 | EVENT_PARKING_LEFT_ACTIVE | T자 주차 실제 기동 중 |
| 11 | EVENT_PARKING_LEFT_WAIT | T자 주차 노드 응답대기 fail-safe |
| 12-14 | (위 9-11과 동일, 평행주차) | |
| 255 | UNKNOWN | 매핑 안 된 카테고리 (버그 아니면 안 나와야 함) |

세분화된 원인(어느 avoid 서브상태, 어느 쪽 parking인지 등)은 이 1바이트로는
다 못 담아서 `~/.ros/arbiter_logs/arbiter_can_<시각>.csv`에 원본 문자열
카테고리로 계속 남음 — CAN 프레임은 "한눈에 볼 요약"용, CSV가 "정밀 분석"용.

## `0x204` GPS_NAV_STATUS (host → AURIX)

**실제 제어에는 안 쓰임** — `0x203 CONTROL_META`와 같은 취지, 같은 틱에
같이 쏨. AURIX가 이걸 파싱 안 해도 차량 동작엔 전혀 지장 없음 (CAN은
브로드캐스트라 구독 안 하는 ID는 그냥 무시됨). "카메라 주행 중 GPS
경로에서 너무 멀어지면 복귀한다"는 그 판단 근거(지금 몇 번 웨이포인트를
추종 중인지, 경로에서 얼마나 벗어났는지)를 CANoe에서도 바로 보이게
하려고 2026-08-18 추가.

```
byte0-1 : gps_idx               (uint16) - 지금 추종 중인 웨이포인트 인덱스
byte2-3 : cross_track_error_cm  (int16, cm 단위) - 경로 대비 부호 있는 횡오차
byte4-7 : (미사용)
```
struct 포맷: `<HhHH`

`cross_track_error_cm`이 `-32768`(int16 최솟값)이면 "값 없음"(NaN/GPS
이탈/stale) - `0`으로 보내면 "완벽하게 라인 위"랑 구분이 안 돼서 별도
sentinel로 분리해둠. 범위는 ±327.67m (그 이상 벗어나면 클램프).

## `0x104` DIAG_STATUS (AURIX → host) — **호스트 파싱 완료, 8/16 로그부터 실제 송신 확인됨**

```
byte0   : applied_stop_mode   (uint8) - 펌웨어가 실제로 적용한 StopHoldEnable 파생 모드
byte1   : fault_flags         (uint8, 비트필드, 아래)
byte2-3 : steer_pwm_duty      (int16) - 조향 모터 부하 지표
byte4-5 : supply_voltage_mV   (uint16)
byte6   : rx_seq_echo         (uint8) - CONTROL_META.seq를 그대로 되돌려줌 (왕복 확인)
byte7   : (미사용)
```
struct 포맷: `<BBhHBB`

**`applied_stop_mode` 값 (펌웨어 담당자와 협의 후 재정의, 2026-08-15):**
액추에이션 레이어(`Encoder_Motor_SetStopHoldEnable`)가 boolean 하나뿐이라
원래 스펙의 0=normal/1=flat/2=hill 3단계를 그대로 복원할 수 없어서, 실제로
구분 가능한 3가지로 재정의함:

| 값 | 이름 | 조건 |
|---|---|---|
| 0 | disabled | 모터 자체가 꺼진 상태 (`SetEnable(FALSE)`) |
| 1 | flat | enable + `StopHoldEnable==FALSE` (코스팅) |
| 2 | hold | enable + `StopHoldEnable==TRUE` (액티브 홀드) |

**주의**: 이 enum은 `0x200.stop_mode`(0=normal/1=flat/2=hill)와 **숫자는 같아도
뜻이 다름** — 특히 `0`이 한쪽은 "normal(홀드 상태)", 다른 쪽은 "disabled(모터
꺼짐)"라 CANoe에서 두 신호를 나란히 놓고 볼 때 헷갈릴 수 있음. DBC value
table에 이름을 명확히 구분해서 달아둘 것.

normal(0)과 hill(2)이 이 enum에서 구분이 안 되는 건 확인/의도된 것 —
자율주행 소프트웨어(`arbiter_node.py` 등)는 `stop_mode=2`(hill)를 아예 안
씀(수동 조작용 `pcan_tools`에서만 씀), 나중에 hill 실제 사용 시점에 액추에이션
로직 자체를 분리하면 그때 이 enum도 3단계로 다시 나누면 됨 - 지금은 안 함.

### `fault_flags` 비트 (`can_driver.py`의 `FAULT_*` 상수)

| 비트 | 이름 | 의미 |
|---|---|---|
| 0 | FAULT_COMM_TIMEOUT | 펌웨어가 자체적으로 CAN 통신 끊김 감지 |
| 1 | ~~FAULT_POT_SENSOR~~ | **드롭 (2026-08-15)** — 급커브에서 정상적으로 풀락 근처까지 자주 가는 코스 특성상, 마진을 감으로 잡아 오탐 위험이 있어 요청 취소. 상수는 `can_driver.py`에 남겨두되 항상 0으로 보내면 됨 |
| 2 | FAULT_ENCODER_SENSOR | 엔코더 이상 (미구현, 항상 0) |
| 3 | FAULT_WATCHDOG_TRIP | 워치독 발동 (미구현 - 워치독 자체가 disable돼있음, 항상 0) |
| 4 | FAULT_UNDERVOLTAGE | 공급전압 저하 (미구현 - 전압 측정 ADC 채널 없음, 항상 0) |

**`applied_stop_mode`가 제일 중요한 필드** — `0x200`으로 보낸 `stop_mode`와
이 값이 다르면, "명령은 보냈는데 펌웨어가 안 따른" 경우를 CAN 로그 하나로
바로 잡아낼 수 있음 (이번 세션 판막 버그가 딱 이 케이스였음).

**펌웨어 쪽 구현 필요**: AURIX가 `0x104`를 주기적으로(예: 20ms마다,
`0x101`/`0x102`랑 같은 주기) 쏘도록 `Can_Comms.c`에 추가해야 함. 이건 별도
Windows 레포(펌웨어)라 이 세션에서 못 건드림 — 위 레이아웃 그대로 펌웨어
담당자한테 넘기면 됨.

---

## 호스트 쪽 구현 현황

- **`can_driver.py`**: `make_control_meta_data`/`send_control_meta`(TX 0x203),
  `parse_diag_status`(RX 0x104 파싱, 아직 실제로 들어오는 데이터는 없음) 추가됨.
- **`arbiter_node.py`**: 실제 CAN을 보내는 모든 지점(`_send_true_deg`,
  avoid 직접전송, parking relay)에서 `_log_can()`을 거치는데, 이 함수가
  이제 (a) `~/.ros/arbiter_logs/arbiter_can_<시각>.csv`에 CSV로 남기고
  (b) 동시에 `0x203` CONTROL_META를 실제로 CAN에 쏨. 카테고리 문자열 →
  `controller_id` 매핑은 `_controller_id_for_category()`.
- **`0x104` 수신**: 아직 host 쪽에서 안 읽음 — 펌웨어가 실제로 쏘기 시작하면
  `parse_diag_status()`로 바로 파싱 가능하지만, 그걸 누가 폴링해서 뭘 할지
  (arbiter가 직접 읽을지, 별도 로거 노드를 만들지)는 아직 미정.

## CANoe 채널 구성 (참고)

- **채널1**: 실제 버스에 물려서 액티브로 참여 (필요하면 진단요청/CAPL 등)
- **채널2**: 같은 물리 버스에 **리슨온리(listen-only)**로 별도로 물려서
  순수 트레이스/로깅만 — 채널1 설정 실수가 실제 버스에 영향 안 주게 분리.
  지금 리눅스 쪽에서 SocketCAN 여러 소켓이 `can0` 하나를 나눠 읽는 거랑
  같은 목적, Vector 하드웨어는 물리적으로 채널을 나눠야 그게 됨.

---
작성일 2026-08-15 · 관련 코드: `waypoint_follower/can_driver.py`,
`waypoint_follower/waypoint_follower/arbiter_node.py`

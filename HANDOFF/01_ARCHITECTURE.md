# 시스템 아키텍처

## 전체 데이터 흐름

```
[GPS/RTK]                     [카메라(ZED2i)]              [LiDAR]        [OAK-D]
u-blox ZED-F9P                yolopv2_zed_rpm_node          obstacle_      traffic_
  ↓ 시리얼                     (YOLOPv2 차선인식)            avoid_node     light_node
rtk_bridge.py (NTRIP RTCM)      ↓                             ↓              ↓
  ↓                          ~/steering_deg                 (steer/rpm    /traffic_light
ublox_gps_node                ~/lane_valid                   모니터링       (GO/STOP)
  ↓ /fix, /navpvt, /monhw     ~/rpm_target                    토픽)
waypoint_follower_node          │                              │              │
  ↓ gps_control/{steer_deg,    │                              │              │
     rpm,valid,cross_track_    │                              │              │
     error_m,target_idx}       │                              │              │
     └──────────────┬──────────┴──────────────┬───────────────┴──────────────┘
                     ↓                         ↓
              ┌─────────────────────────────────────┐
              │         control_arbiter              │  ← 유일한 CAN 송신자
              │        (arbiter_node.py)             │
              └─────────────────────────────────────┘
                     ↓ CAN (0x200 제어 + 0x203/0x204 로깅용)
              [AURIX TC275 펌웨어] → 모터/조향 액추에이터
                     ↓ CAN (0x101/0x102/0x104 피드백)
              control_arbiter가 다시 받아서 로깅/진단
```

**핵심 원칙: `control_arbiter`(`arbiter_node.py`)가 CAN 버스에 실제로
쓰는 유일한 노드**. 다른 모든 소스(GPS, 카메라, LiDAR회피)는 CAN을
직접 안 건드리고 ROS 토픽으로 "나라면 이렇게 하겠다"만 publish하고,
arbiter가 그중 뭘 실제로 CAN에 실을지 매 틱(20Hz)마다 결정함.
(`publish_can_directly:=false`로 항상 실행 — launch 파일에 이미 고정돼있음)

## `control_arbiter`의 우선순위 로직 (매 20Hz 틱마다)

```python
# 1. 이벤트존이 있는지 먼저 확인 (idx 기반)
zone = _zone_at(gps_idx)  # "stop"/"gps_priority"/"gps_priority_slow"
                          # /"avoid"/"traffic_light"/"parking_left"/"parking_right"
                          # / None(zone 없음)

# 2. 기본 주행원 결정 (zone이 없거나, traffic_light처럼 steer만 빌려쓰는 zone용)
if camera_ok:       # 카메라가 신뢰할 수 있는 상태면
    base_source = "camera"
elif gps_ok:         # 카메라가 안 되면 GPS로
    base_source = "gps_fallback"
else:
    base_source = None  # 둘 다 안 되면 안전정지

# 3. 우선순위 순서대로 분기 (첫 매치가 이김)
if <타이밍 정지 hold 진행중>:      # 시간기반, zone 벗어나도 계속
    stop_mode=2로 강제 정지
elif engaged_side is not None:     # 주차 중(맞물림)이면 최우선
    주차 노드 명령 그대로 릴레이
elif zone == "stop":
    정지 (무기한 또는 타이밍)
elif zone == "gps_priority"/"gps_priority_slow":
    GPS만 운전 (카메라 배제) — 신호등 체크 있음
elif zone == "avoid":
    LiDAR 회피 상태머신 결과 릴레이
elif zone == "traffic_light":
    base_source 그대로 쓰되 속도만 신호등 상태 따라 조절
elif zone == "parking_left"/"parking_right":
    주차 존 로직 (mapping 중엔 그냥 통과, 슬롯 찾으면 맞물림)
elif base_source is not None:
    평상시 주행 (camera 또는 gps_fallback)
else:
    safe_stop
```

**`camera_ok` 판정 3조건** (하나라도 깨지면 GPS로 강등):
1. `camera_active` — `/yolopv2_zed_node/lane_valid` 프레임 히스테리시스 (연속 10프레임 False면 꺼짐, 3프레임 True면 켜짐)
2. Freshness — `~/steering_deg` 토픽이 `camera_timeout_sec`(1.0s) 안에 안 오면 dead-man's switch로 꺼짐
3. GPS cross-track veto — 카메라 주행 중에도 GPS 실측 위치가 기록 경로에서 `camera_max_deviation_m`(2.5m) 넘게 벗어나면 강제로 GPS로 스왑 (재진입은 더 까다로움, `04_PARAMETERS_REFERENCE.md` 참고)

## 이벤트존 (`event_zones` 파라미터)

`'start:end:type'` 또는 `'start:end:type:extra'` 형식 문자열 리스트.
`start`/`end`는 GPS 웨이포인트 인덱스(idx), `type`은:

| type | 의미 | `extra` 필드 |
|---|---|---|
| `stop` | 정지 | 없으면 무기한 정지, 있으면 초 단위 타이밍 정지 후 자동 재개 |
| `gps_priority` | GPS 전용 (카메라 배제) | 없음 |
| `gps_priority_slow` | GPS 전용 + 감속 (`gps_priority_slow_rpm`로 캡) | 없음 |
| `avoid` | LiDAR 장애물회피 존 (`obstacle_avoid_node` 무장) | 없음 |
| `traffic_light` | 신호등 대기 | 정지선 idx (없으면 `end`로 폴백) |
| `parking_left` | T자 후진주차 | 없음 |
| `parking_right` | 평행주차 | 없음 |

idx가 여러 zone에 걸치면 리스트에서 **먼저 매치되는 것**이 이김
(zone들끼리 idx 범위가 겹치면 안 됨 — 실수로 겹치게 만들지 말 것).

## `obstacle_avoid` 상태머신 (LiDAR)

`CLEAR → AVOID → PASS → RETURN → CLEAR` 순환. `avoid` 존 안에서만
무장되고, 존을 벗어나면 상태 리셋. `control_arbiter`가
`can_bridge_enable_topic`으로 무장/해제.

## 카메라 차선인식 알고리즘 (Stanley 아님)

GPS 쪽(Stanley)과 근본적으로 다른 구조 — **경로를 미리 만들지 않고
매 프레임 반응형으로 계산**:
1. YOLOPv2 모델 → 차선 세그멘테이션 마스크
2. BEV(조감도) 변환
3. 슬라이딩 윈도우로 차선중심 추정
4. **위치항**(`position_deg` = 픽셀오차 × `max_steer_deg`) + **곡률항**(`curvature_deg`, 여러 밴드 다항식 피팅의 근접점 기울기) 합산 → 조향각

이전 프레임 기억 없음(매 프레임 완전 독립 계산) — 그래서 필터링이
카메라 자체엔 없고, `control_arbiter`의 `base_steer_lowpass_alpha`가
그 역할을 일부 대신함 (`04_PARAMETERS_REFERENCE.md` 참고).

## Stanley 컨트롤러 (GPS 쪽)

순정 Stanley + 예견(anticipatory) 보정:
- 매 사이클 **가장 가까운 웨이포인트** 기준으로 헤딩오차+횡오차 계산 (pure pursuit 아님, lookahead point 조준 안 함)
- `curve_lookahead_m`(6.0m) 앞의 경로 커버쳐(누적 헤딩변화량, **조향각이 아님**)를 미리 감지해서 헤딩목표를 blend — 코너 진입 전부터 서서히 꺾기 시작
- 코너가 차량 최소회전반경보다 급하면(물리적으로 못 돎) blend 없이 그냥 풀락

## CAN 통신

호스트(Jetson/RPi) ↔ AURIX(펌웨어) 간 5개 메시지. 상세는
`05_CAN_PROTOCOL.md` 참고. 핵심만: **`control_arbiter`가 유일한
TX(0x200) 송신자**, 나머지 TX(`0x203`,`0x204`)는 로깅/CANoe 가시성
전용이라 펌웨어가 안 봐도 차량 동작엔 지장 없음.

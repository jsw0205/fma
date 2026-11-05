# 자율주행/임베디드 프로젝트 포트폴리오

**작성**: 2026-11-05  
**대상**: 자율주행 회사 이력서

---

## 📊 이력서에 쓸 수 있는 프로젝트 목록

### 🏆 1. **ROS2 자율주행 로버 시스템** (메인 프로젝트) ⭐⭐⭐
**기간**: 2026-07-23 ~ 현재 (3.5개월)  
**상태**: 🟢 Production-Ready  
**기술**: ROS2, Python, C++, GPS/RTK, 카메라, LiDAR

#### 개요
GPS/RTK 위치 추적 + 카메라 차선 인식 + LiDAR 자동 회피 + 주차 기능을 통합한 **완전 자율주행 로봇**

#### 핵심 성과
- ✅ **GPS/RTK 3cm 정확도** 안정화 (Float 5-10초, Fixed 30-60초)
- ✅ **직선 주행 오류 90% 감소** (1.07m → 0.105m RMSE)
- ✅ **코너 주행 완주** (물리적 가능성 검증 + 풀 스티어링 락)
- ✅ **다중 센서 아키텍처**: Control Arbiter로 우선순위 기반 중재
- ✅ **실시간 문제 해결**: USB3-EMI, 하드웨어 캘리브레이션, 라이브 버그 수정
- ✅ **인수인계 문서**: HANDOFF 패키지 (961줄, 8개 문서)

#### 주요 기술 성과
1. **GPS/RTK 통합** (직렬 RTCM 중계)
   - 문제: ROS ublox_gps 드라이버에 RTCM 수신자 부재
   - 해결: rtk_bridge.py (직렬 인터페이스 우회)

2. **스티어링 캘리브레이션**
   - 측정: 펌웨어 30° vs 실제 14.3°
   - 방법: GPS 드라이브 로그 → 원형 맞춤 → CAN 변환 계수 적용

3. **Control Arbiter** (우선순위 중재)
   - 카메라 → GPS → 안전 정지
   - 특수 구간: GPS 이벤트 존 (stop/gps_priority/avoid/traffic_light)

4. **다중 센서 융합**
   - GPS/RTK (3cm), IMU (±0.3°), 카메라 (YOLOPv2), LiDAR
   - 안전한 각 모드 디그레이드 (타임아웃/실패 시)

#### 폴더 구조
```
~/ros2_ws/
├── src/waypoint_follower/           (메인: GPS추종/arbiter/CAN)
├── src/zed_camera/                  (카메라 차선인식)
├── src/f9p_bringup/                 (GPS/RTK)
├── src/traffic_light/               (신호등 감지)
├── src/obstacle_avoidance/          (LiDAR 회피)
├── README.md                        (2385줄 상세 로그)
└── HANDOFF/                         (인수인계 문서 8개)
```

#### 추천 이력서 작성법
```
프로젝트명: ROS2 기반 다중 센서 자율주행 로봇 시스템
기간: 2026.07 - 현재 (약 3.5개월)
역할: 풀스택 개발 (GPS/제어/카메라/LiDAR 통합)
성과:
- GPS/RTK 3cm 정확도 달성, 직선 주행 오류 90% 개선
- 다중 센서 우선순위 중재 아키텍처 설계 및 구현
- 실시간 문제 해결 (EMI, 하드웨어 캘리브레이션)
- 체계적 문서화 (인수인계 패키지 961줄)
기술: ROS2, Python, GPS/RTK, YOLO, LiDAR, CAN
```

---

### 2. **헤네스 자율주행 - Stanley 컨트롤러** ⭐⭐
**기간**: ~2026-07 (초기 프로토타입)  
**상태**: 완료 (ros2_ws의 전신)  
**기술**: Python, Stanley 제어, CAN, ROS2

#### 개요
GPS 웨이포인트 추종 + Stanley 컨트롤러 + CAN 송신을 한 노드에서 처리

#### 주요 내용
- **파일**: `stanley_can_drive.py` (통합 노드)
- **경로**: 44개 웨이포인트 리샘플링
- **필터**: GPS 이상치 제거, 위치 LPF, 헤딩 스무딩, 조향 LPF
- **안전 로직**: 워밍업, GPS 타임아웃, 헤딩 대기, 골 정지
- **CAN 프로토콜**: 0x200 TX (rpm/steer/enable/stop_mode), 0x101/0x102 RX

#### 기술 내용
```python
# 핵심 필터
GPS 이상치 제거: gps_outlier_m (기본 1.0m)
위치 LPF: pos_lpf_fc / pos_lpf_fs
헤딩 스무딩: heading_window (최근 N개 위치)
조향 LPF: steer_lpf_fc / steer_lpf_fs
```

#### 추천 이력서 작성법
```
프로젝트명: GPS 웨이포인트 추종 제어 시스템
기간: 2026년 상반기
역할: 제어 알고리즘 설계 및 CAN 통합
성과:
- Stanley 컨트롤러 구현 및 필터 설계
- GPS 이상치 제거 및 신호 처리
- CAN 프로토콜 구현 (20Hz 송수신)
기술: Python, ROS2, Stanley제어, CAN통신
```

---

### 3. **한라대 캠퍼스 주행 로그 분석** ⭐⭐
**기간**: 2026-08-02 (실주행)  
**상태**: 완료  
**기술**: MATLAB/Python, 데이터 분석, 시각화

#### 개요
HENES 차량이 GPS 웨이포인트 추종으로 **한라대 캠퍼스 한 바퀴 완주** 한 실주행 로그 분석

#### 주요 데이터
- **주행시간**: 9.5분 (569.9초)
- **주행거리**: 약 1.18 km
- **평균속도**: 2.08 m/s (~7.5 km/h)
- **경로 이탈오차**: 평균 0.67m, 중앙값 0.44m (p95: 2.3m)

#### 분석 내용
| 항목 | 결과 |
|------|------|
| 역주행 | 없음 (target_idx 단조 증가 0→2185) |
| 완주 | ✅ 확인됨 |
| 이탈오차 | 직선: 양호 / 급코너: 큼(최대 5.25m) |
| 카메라 역할 | 대부분 구간에서 차선인식이 주행 |

#### 시각화 스크립트
- **MATLAB**: `visualize_drive_log.m` (원본)
- **Python**: `visualize_drive_log.py` (포팅판)
- **출력**: 궤적/이탈오차/속도/조향/카메라 유효율 4분할 그래프

#### 로그 컬럼 (CSV)
```
drive_log_*.csv:
  t_s, x_m, y_m, target_idx, cross_track_error_m,
  steer_deg, rpm, speed_mps, current_angle_deg, target_angle_deg

lane_log_*.csv:
  timestamp, lane_valid, side, lane_center_x, error_px,
  half_width, pixel_count, speed_mps, motor_rpm, steering_deg
```

#### 추천 이력서 작성법
```
프로젝트명: 자율주행 로봇 실주행 데이터 분석
기간: 2026-08
역할: 로그 분석 및 시각화
성과:
- 9.5분 GPS 추종 주행 완주 검증
- 경로 이탈 오차 분석 (평균 0.67m)
- 급코너 구간 성능 저하 원인 규명
- MATLAB/Python 포팅 시각화
기술: MATLAB, Python, pandas, matplotlib, 데이터분석
```

---

### 4. **CAN 통신 드라이버** (Jetson Nano) ⭐
**기간**: ~2026년 상반기  
**상태**: 완료  
**기술**: Python, CAN, Jetson Nano

#### 개요
PCAN-View 기반 Jetson Nano 용 CAN 드라이버

#### 주요 파일
- **pcan_jetson_live.py**: Jetson Nano CAN 인터페이스
  - 채널: can0, 비트레이트: 500kbps
  - TX ID: 0x200 (rpm/steer/enable/stop_mode)
  - RX ID: 0x101, 0x102 (피드백)
  - 송신 주기: 20Hz 고정

#### CAN 프로토콜
```python
TX (0x200): <hhBBH rpm(int16), steer(int16), enable(u8), stop_mode(u8)
RX (0x102): <hhhh encoder, rpm×10, pwm, target_rpm
RX (0x101): <HHhh current_pot, target_pot, angle×10, target_angle×10
```

#### 특징
- Jetson Nano의 CAN 인터페이스 초기화
- 실시간 모터 제어 (20Hz)
- 피드백 모니터링
- 안전 정지 (rpm=0 + stop_mode=1)

#### 추천 이력서 작성법
```
프로젝트명: Jetson Nano CAN 통신 드라이버
기간: 2026년 상반기
역할: 드라이버 개발 및 최적화
성과:
- Jetson Nano CAN 인터페이스 구현
- 20Hz 실시간 제어 루프 구성
- 모터/스티어링 피드백 모니터링
기술: Python, CAN, Jetson Nano, 임베디드시스템
```

---

### 5. **추가 파일들**
이 외 찾은 프로젝트 관련 파일들:

| 파일 | 설명 |
|------|------|
| `handsfree_imu_a9_node.py` | HandsFree IMU 드라이버 |
| `yolopv2_zed_rpm_node2.py` | YOLOPv2 카메라 버전 |
| `wheel_odom_pcan_node.py` | 휠 오도메트리 CAN 노드 |
| `RTKLIB/` | GPS/RTK 라이브러리 (클론) |

이들은 주 프로젝트의 컴포넌트들입니다.

---

## 📋 이력서 작성 가이드

### 가장 강력한 프로젝트: ROS2 자율주행 로버 시스템
**왜 이것을 메인으로?**
- ✅ 최신 프로젝트 (3.5개월 진행 중)
- ✅ 실제 하드웨어 (GPS/카메라/LiDAR)
- ✅ 실행 가능한 결과 (코너 완주, 3cm 정확도)
- ✅ 문서화 완벽 (HANDOFF 961줄)
- ✅ 기술 깊이 (제어 알고리즘~아키텍처)

### 이력서 구성 (추천)
```
1. 메인 프로젝트 (50%)
   ROS2 자율주행 로버 시스템
   - GPS/RTK 안정화
   - 다중 센서 아키텍처
   - 문제 해결 사례

2. 보조 프로젝트 (30%)
   - 로그 분석 (데이터 분석 능력)
   - CAN 드라이버 (임베디드 경험)

3. 기술 스택 (20%)
   - ROS2, Python, GPS/RTK
   - 제어 알고리즘, 하드웨어 통합
```

### 인터뷰 대비용 키워드
```
GPS/RTK (3cm 정확도)
Stanley 제어기
다중 센서 융합
ROS2 아키텍처
실시간 제어 (20Hz)
CAN 통신
IMU/카메라/LiDAR 통합
문제 해결 (EMI, 캘리브레이션)
데이터 분석 (MATLAB/Python)
```

---

## 🎯 자율주행 회사 지원 체크리스트

### 준비할 것
- [x] 메인 프로젝트 상세 설명 (RESUME_SUMMARY.md)
- [x] 아키텍처 다이어그램 (PROJECT_OVERVIEW.md)
- [x] 기술 문제 해결 사례 (COMPLETION_STATUS.md)
- [x] 실행 코드 & 로그 (GitHub/로컬)
- [x] 성과 지표 (90% 오류 감소, 3cm 정확도)

### 인터뷰 시 보여줄 것
1. **HANDOFF/01_ARCHITECTURE.md** — 시스템 설계
2. **한라대 로그 분석** — 실제 성과 데이터
3. **COMPLETION_STATUS.md** — 기술 깊이
4. **코드 (stanley_controller.py 등)** — 구현 능력

### 준비하지 말아야 할 것
- ❌ 불완전한 프로젝트 (신호등 감지 OAK-D 테스트 미진행 정도면 OK)
- ❌ 기술 용어 잔뜩 (결과와 배운 점에 집중)
- ❌ 과장 (측정 불가능한 주장 X)

---

## 📚 참고 문서

당신이 이미 만든 문서들:

| 문서 | 내용 | 위치 |
|------|------|------|
| HANDOFF/ | 인수인계 패키지 (8개 파일) | ~/ros2_ws/HANDOFF/ |
| README.md | 세션별 상세 로그 (2385줄) | ~/ros2_ws/README.md |
| RESUME_SUMMARY.md | 이력서용 요약 | ~/ros2_ws/RESUME_SUMMARY.md |
| PROJECT_OVERVIEW.md | 기술 상세 | ~/ros2_ws/PROJECT_OVERVIEW.md |
| COMPLETION_STATUS.md | 완료 상태 | ~/ros2_ws/COMPLETION_STATUS.md |
| FULL_PROJECT_HISTORY.md | 완전한 작업 이력 | ~/ros2_ws/FULL_PROJECT_HISTORY.md |

**→ 이 모든 것을 정렬해서 당신의 포트폴리오입니다!**

---

**최종 조언**: 자율주행 회사 지원할 때는 **ROS2 자율주행 로버 시스템**을 메인으로 하고, 나머지는 "경험 범위의 다양성"을 보여주는 용도로 사용하세요. 면접관이 가장 관심 가질 만한 것들:
1. GPS/RTK 정확도 3cm 달성
2. 직선 주행 오류 90% 개선 (정량 지표)
3. 다중 센서 우선순위 아키텍처
4. 실제 하드웨어 문제 해결 경험

**작성자**: jsw0205  
**최종 정리**: 2026-11-05

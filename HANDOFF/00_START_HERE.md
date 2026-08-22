# HENES 자율주행 로버 — 인수인계 문서 (시작점)

_정리일: 2026-08-19_

> **AI 어시스턴트가 이 폴더를 처음 읽는 거라면**: 아래 "AI에게" 섹션부터 읽어.
> **사람이 인수인계 받는 거라면**: "프로젝트 한눈에 보기"부터 순서대로 읽으면 돼.

---

## AI에게

이 폴더(`HANDOFF/`)는 HENES 자율주행 로버 프로젝트(GPS+카메라+LiDAR
융합 주행, ROS2 Humble)의 인수인계 패키지야. 아래 순서로 읽으면 이
프로젝트를 새로 맡아도 바로 작업 가능한 수준까지 파악돼:

1. **`00_START_HERE.md`** (이 파일) — 프로젝트 개요, 폴더 구조
2. **`01_ARCHITECTURE.md`** — 시스템 구성(노드/패키지/데이터 흐름), 제어권 우선순위 로직
3. **`02_HOW_TO_RUN.md`** — 실행 명령어 전부 (GPS만/통합주행/카메라단독/웨이포인트 기록 등)
4. **`03_TROUBLESHOOTING.md`** — 알려진 문제 + 해결법 (겪었던 실제 버그들)
5. **`04_PARAMETERS_REFERENCE.md`** — 튜닝 파라미터 전체 목록과 의미
6. **`05_CAN_PROTOCOL.md`** — CAN 메시지 프로토콜 (호스트↔펌웨어)
7. **`06_KNOWN_LIMITATIONS_TODO.md`** — 미해결/미검증 항목, 다음에 뭘 해야 하는지
8. **`07_FILE_MAP.md`** — 패키지별 파일 목록 + 한 줄 설명

**더 깊은 배경/추론 과정이 필요하면** `../README.md`(2200줄 넘는 세션별
연대기 로그, 이 워크스페이스 루트에 있음)를 참고해 - 거긴 "왜 이렇게
했는지"에 대한 상세한 시행착오 기록이고, 이 `HANDOFF/` 폴더는 그걸
주제별로 압축·정리한 "지금 상태의 스냅샷"이야. 둘이 내용 충돌하면
**`../README.md` 쪽 최신 날짜 항목이 진실**이야 (이 HANDOFF 폴더는
2026-08-19 기준 스냅샷이라 그 이후 변경사항은 안 담겨 있을 수 있음).

코드 자체를 고치기 전에 `03_TROUBLESHOOTING.md`랑
`06_KNOWN_LIMITATIONS_TODO.md`를 먼저 읽어 — 이미 겪고 해결(또는 보류)한
문제를 처음부터 다시 파는 걸 피할 수 있어.

---

## 프로젝트 한눈에 보기

**뭘 만드는 거냐**: GPS 웨이포인트 추종 + 카메라(ZED2i) 차선인식 + LiDAR
장애물회피 + T자/평행주차 + 신호등 인식을 하나의 `control_arbiter`
노드가 우선순위대로 중재해서, 실제 로버(HENES)를 CAN 버스로 제어하는
시스템. ROS2 Humble 기반.

**제어권 우선순위** (자세한 건 `01_ARCHITECTURE.md`):
```
카메라(차선인식) > GPS 웨이포인트 추종 > safe_stop(정지)
```
단, 특정 구간(`event_zones`)에 들어가면 이 기본 우선순위를 무시하고
GPS 전용/장애물회피/주차/신호등 대기 같은 특수 동작으로 전환됨.

**하드웨어**: u-blox ZED-F9P(GPS/RTK), ZED2i(스테레오카메라), OAK-D(신호등
인식용, 아직 실측 안 됨), RPLiDAR, AURIX TC275(펌웨어, 별도 레포).

**소프트웨어 구조**: ROS2 Humble, 14개 패키지(`../src/` 아래),
`waypoint_follower` 패키지가 핵심(GPS 추종 + arbiter + CAN 드라이버).

**코드 위치 (git)**:
- 저장소: `git@github.com:jsw0205/fma.git` (펌웨어 레포랑 같은 저장소, 브랜치로 구분)
- **이 ROS2 워크스페이스 코드는 `ros2-software` 브랜치**에 있음 (main 아님!)
- 클론: `git clone git@github.com:jsw0205/fma.git && cd fma && git checkout ros2-software`
- 커밋 내역이 사실상 이 프로젝트의 두 번째 상세 로그임 (`git log`로 확인)

**지금 상태 (2026-08-19 기준) 한 줄 요약**: GPS+카메라 통합주행/이벤트존
(정지/GPS전용/주차/신호등)까지 코드는 다 있고 실차로 여러 번 테스트됨.
**언덕정지(hill-hold)만 펌웨어 미구현으로 실제 정지가 안 됨** (호스트
쪽 CAN 명령은 정상 전송 확인됨). GPS-카메라 동시운용 시 **ZED 카메라가
GPS RTK Fixed 판정에 방사 EMI로 영향을 준다는 게 최근에 확인**됐고
원인/대책은 아직 조사 중.

---

## 폴더 구조 요약

```
~/ros2_ws/
├── README.md              ← 세션별 연대기 로그 (제일 상세함, 날짜순)
├── HANDOFF/                ← 이 폴더 (주제별 정리본)
├── src/                    ← 14개 ROS2 패키지 (07_FILE_MAP.md 참고)
│   ├── waypoint_follower/  ← 핵심: GPS추종/arbiter/CAN/런치파일
│   ├── zed_camera/         ← 카메라 차선인식 (YOLOPv2)
│   ├── f9p_bringup/        ← GPS/RTK 구동
│   ├── rtk_bridge/         ← NTRIP→시리얼 RTCM 중계 (ROS 패키지 아님)
│   ├── traffic_light/      ← OAK-D 신호등 인식
│   ├── obstacle_avoidance/ ← LiDAR 회피
│   ├── t_parking/          ← T자 후진주차
│   ├── parallel_parking/   ← 평행주차
│   ├── parking_bridge/     ← 주차 관련 브릿지
│   └── fma/                ← 펌웨어 (별도 gitlink, AURIX 소스)
└── (기타 로그: ~/.ros/arbiter_logs/, ~/logs/drive/, ~/logs/lane/)
```

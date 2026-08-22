# 트러블슈팅 — 겪었던 문제들과 해결법

## GPS/RTK 관련

### RTCM이 안 들어감 (`navpvt` flags가 계속 1)
apt로 설치한 `ros-humble-ublox-gps`(2.3.0) 드라이버는 **RTCM을 ROS
토픽으로 받는 구독자가 아예 없음** (`rtcm_msgs` 의존성 자체가 없음).
`ntrip_client` → `/rtcm` 토픽 발행해도 아무도 안 들음. **해결**:
`rtk_bridge.py`(순수 스크립트, ROS 노드 아님)가 NTRIP에서 RTCM 받아서
시리얼 포트에 직접 바이트로 씀. `f9p_bringup`이 이미 이렇게 구성돼있음
— 다시 "제대로" ROS화하려고 하지 말 것, 구조적으로 안 됨.

주의: `rtk_bridge.py`는 시작할 때 GGA 한 줄 읽으려고 시리얼 포트를
잠깐 직접 여는데, `ublox_gps_node`가 자기 시작 핸드셰이크(MonVER/설정)
하는 도중에 같이 열면 충돌남 — `ublox_gps_node`가 "configured
successfully" 로그 찍은 뒤에만 `rtk_bridge.py` 시작해야 함
(`f9p_rover.launch.py`가 이미 이 순서로 딜레이 넣어놨음).

### GPS `num_sv=0` (위성 0개)
2026-07-28에 한 번 발생, 원인 특정은 못 했지만 **전원 재연결(USB
뽑았다 꽂기)로 해결됨**. 재발하면 안테나/RF 하드웨어 의심하기 전에
먼저 이것부터 시도.

### ZED 카메라가 GPS Fixed를 방해함 (USB3-EMI, **아직 완전히 해결 안 됨**)
- **1차 확인(2026-07-29)**: ZED2i가 USB3 SuperSpeed라 1-6GHz EMI 방사,
  GPS L1(1575.42MHz)이랑 겹침. GPS 단독으론 Fixed 잘 잡히는 지점에서
  ZED 같이 켜면 Float에서 못 올라감. **확인된 해결책**: GPS 안테나
  케이블 ↔ ZED USB 케이블 물리적으로 멀리 배선(평행 구간 피하고 교차
  시 직각으로) → Fixed 정상 회복.
- **2차 확인(2026-08-18 밤, 훨씬 심각함)**: 노트북 2대로 분리 테스트
  (하나는 ZED 전원만, 다른 하나는 GPS) — **완전히 분리된 노트북인데도
  ZED "전원만" 켜도 GPS Fixed에 영향 감**. 코드 돌려서 실제
  스트리밍하면 영향이 더 심해짐. **이건 케이블 근접 커플링 모델로는
  설명 안 됨 — 방사(radiated) EMI가 원인일 가능성.** 대기전력 노이즈
  (전원회로/스위칭 레귤레이터) + 활동량 비례 노이즈 두 가지가 겹쳐있는
  걸로 추정. **아직 대책 미확정** — `06_KNOWN_LIMITATIONS_TODO.md` 참고.
- **진단 도구**: `ros2 run waypoint_follower gps_quality_monitor`로
  `jam_ind`/`jamming_state`/`h_acc` 실시간 확인 가능 — ZED 전원 on/off
  하면서 이 값들 변화 보면 EMI 영향 직접 증명 가능.

### 시작 시 위치가 지구 반대편(~14,000km)에 찍힘
`on_fix()`가 첫 `NavSatFix` 메시지에 로컬 원점(`LocalFrame`)을 고정하는데,
`ublox_gps_node`는 fix 안 잡혀도 lat=lon=0.0으로 계속 publish함 — 그
메시지가 먼저 오면 원점이 (0,0)에 영구 고정됨. **해결**: `on_fix()`가
`status.status < STATUS_FIX`면 그냥 return하도록 수정됨 (이미 코드에
반영돼있음, 재발하면 이 부분 확인).

## 카메라/GPU 관련

### ZED가 "NO GPU DETECTED"라고 나옴 (드라이버는 멀쩡한데)
`nvidia-smi`는 정상인데 `python3 -c "import torch; print(torch.cuda.is_available())"`가
`False`거나 "CUDA unknown error"면 — **노트북 절전모드(sleep)/뚜껑
닫았다 열기 후유증**인 경우가 대부분 (하이브리드 그래픽 노트북 흔한
증상). **해결**:
```bash
sudo rmmod nvidia_uvm
sudo modprobe nvidia_uvm
python3 -c "import torch; print(torch.cuda.is_available())"
```
안 되면 `sudo lsof /dev/nvidia* | grep -v Xorg`로 뭐가 물고 있는지 확인.
그래도 안 되면 **재부팅이 제일 확실함** (거의 100% 해결).

### 카메라가 아예 안 켜짐 (`zed_wrapper`가 조용히 죽음)
`zed_wrapper`는 카메라가 시작 시점에 물리적으로 안 꽂혀 있으면 **~28초
재시도하다 죽고 자동 재시작 안 함**. `lsusb | grep -i stereolabs`로
하드웨어 인식 확인 후, 안 떠 있으면(`ros2 node list`) 그냥 다시 launch.

### 카메라가 CPU로만 돎 (~2fps)
`-p device:=cuda` 파라미터 주면 `select_device()`가 `CUDA_VISIBLE_DEVICES`에
그 문자열을 그대로 넣어버려서 GPU가 전부 숨겨짐. **`device` 파라미터
아예 주지 말 것** (기본값 `"0"`이 실제 GPU 인덱스라 정상 작동함).

### `yolopv2_zed_rpm_node`가 몇 분 후 SIGABRT로 죽음
2026-08-16 22:23 런에서 302초 후 `exit code -6`로 크래시 확인됨
(`~/.ros/log/.../launch.log`에 `terminate called without an active
exception`). **원인 미조사** — 재발하면 그 로그부터 볼 것.

## CAN/제어 관련

### `colcon build`가 `--uninstall` 에러로 실패
`torch` 설치가 `setuptools`를 새 버전(83.0.0)으로 조용히 올려버려서
발생. **해결**: `pip3 install "setuptools<80"`.

### 조향각이 이상하게 큼/작음 (30도 vs 14.3도)
펌웨어의 `STEER_MAX_ANGLE` 상수가 30.0인데 실제 물리적 최대 조향각은
**14.3도**(2026-07-26, 2바퀴 GPS 원 주행으로 반경 실측해서 역산한 값,
왼쪽 방향만 검증됨 — 오른쪽은 아직 별도 검증 안 됨, 평행주차 쪽에서
이 오차가 실제 정렬 실패로 이어진 사례 있음). 호스트 쪽에서
`CAN_STEER_SCALE = 30.0/14.3`로 보정해서 보냄
(`can_driver.py`의 `send_control_true_deg()` 사용 — raw
`send_control()` 직접 쓰지 말 것, 이중변환/누락 버그남).

### 언덕에서 정지가 안 됨 (`stop_mode=2` 보내도 차가 계속 밀림)
**펌웨어에 진짜 홀드(hill-hold) 액추에이션 로직이 없음.**
`rpm=0`으로 보내는 건 "가속하지 마라"는 뜻이지 "브레이크를 걸어라"는
뜻이 아님 — 실제로 저항 토크를 걸어줘야 하는데 그게 미구현. 호스트
쪽에서 `stop_mode=2`+`enable=1`을 3초 동안 시간 기반으로 강제 전송하는
것까지는 확인했지만(코드는 정상), **차량이 물리적으로 안 멈춤 —
펌웨어 팀 확인 필요**, 호스트 쪽에서 더 손 볼 여지 없음.

### 웨이포인트 1개짜리 좁은 이벤트존(예: `44:44:stop:3`)이 순간적으로만 걸림
고속 주행 중 관성으로 idx가 0.4초 만에 그 존을 지나가버려서 원래
의도한 hold 시간(3초)을 못 채우는 문제. **해결됨**: 정지 hold를
"idx가 그 존 안에 있는 동안만"이 아니라 **순수 시간 기반**으로 재작성
(`_stop_hold_active_until`) — 한 번 시작되면 idx/zone 상관없이 그
시각까지 무조건 정지 명령 계속 보냄. (2026-08-18 커밋, `arbiter_node.py`)

### `gps_priority` 존에서 조향이 갑자기 풀락으로 확 꺾임
`gps_priority`/`gps_priority_slow`는 `self.gps_steer`(Stanley 원본값)를
필터 없이 그대로 CAN에 보내고 있었음 — 실차 로그로 ±14.3° 스윙이
실제로 여러 번 나간 것 확인됨. **해결됨**: `base_steer_lowpass_alpha`
필터를 이 두 zone에도 적용 (`_filtered_gps_steer`, 별도 필터 상태).

### 커브에서 너무 일찍/너무 늦게 꺾임
`curve_lead_margin`(예견 반응 배율) 튜닝 문제. 1.1(너무 늦음, 코너
바깥으로 밀림) → 1.5(너무 일찍) → **1.2로 재조정**(2026-08-18). 실차
피드백 따라 더 조정 필요할 수 있음. launch 인자로 재빌드 없이 바로
튜닝 가능.

### `event_zones` launch 인자 관련
`[""]`(빈 문자열 하나 담긴 리스트)를 써야 함 — 진짜 빈 리스트 `[]`는
ROS2 파라미터 타입 추론이 깨짐(`Expected 'value' to be one of [...],
but got '()' of type 'tuple'` 에러로 노드 자체가 안 뜸).

## 기타

### `parking_parallel_oneshot.launch.py` / `parking_left_oneshot.launch.py`
원본 참조 zip에서 그대로 가져온 게 존재하지 않는 패키지(`sllidar_ros2`,
`my_first_pkg`)를 참조함 — **고장난 상태, 쓰지 말 것**. 대신
`02_HOW_TO_RUN.md`의 "주차 단독 테스트" 명령어 쓸 것.

### NTRIP 계정정보
`rtk_bridge.py`엔 실제 평문 NTRIP 계정정보가 들어있음 — git엔 안
올라가게 `.gitignore` 처리돼있고 `rtk_bridge.py.example`만 커밋됨.
새 환경에서 세팅할 땐 `.example` 복사해서 실제 계정정보 채워넣을 것.

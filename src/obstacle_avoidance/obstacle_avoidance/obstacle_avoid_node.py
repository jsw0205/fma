#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
obstacle_avoid_node.py (v3: HENES 실차, CAN 통신 내장 5단계 상태머신)

v2(별도 can_bridge_node + 토픽 중계)를 하나의 프로세스로 합친 버전.
스캔 판단 -> 상태머신 -> CAN 프레임 송수신까지 이 노드 하나가 직접 처리한다
(더 이상 can_bridge_node 가 필요 없음. /avoid/cmd_steer 등은 모니터링용으로
계속 발행하지만, 실제 제어는 이 프로세스 안에서 바로 CAN 으로 나간다).

TX 0x200 (<hhBBH>): rpm(int16), steer(int16), enable(uint8),
                     stop_mode(uint8, 0=normal/1=flat/2=hill), unused(uint16)
RX 0x102 (drive_status,  <hhhh>): encoder_count, rpm_x10, pwm_duty, target_rpm
RX 0x101 (steering_status,<HHhh>): current_pot, target_pot,
                                    current_angle_x10, target_angle_x10
(pcan_jetson_live.py 프로토콜 그대로)

우려1 (회피 중 연석이 다시 장애물로 잡힘):
  -> AVOID/PASS/RETURN 상태에서는 scan_callback 이 상태 전이에 관여하지
     않음 (self.latest 로 로깅만 함). 오직 IMU yaw/엔코더 기반 진행률로만
     다음 단계로 넘어가므로 회피 도중 연석이 찍혀도 무시된다.

우려2 (장애물 뒷부분에서 복귀 중 충돌):
  -> 회피각(alpha) 만큼 꺾은 뒤 바로 복귀하지 않고, PASS 단계에서
     (obstacle_length + pass_margin) 만큼 더 직진(엔코더 거리 누적)한
     뒤에야 RETURN(반대 조향으로 원래 헤딩 복귀)을 시작한다.

상태: CLEAR (cruise_rpm 순항) -> AVOID_LEFT/RIGHT (yaw 변화량 alpha 도달) -> PASS (거리 도달)
      -> RETURN (yaw 가 시작 헤딩으로 복귀) -> CLEAR
      필요시 STOP (너무 가깝거나 넓어서 회피 불가 / CAN·IMU 피드백 유실, rpm 항상 0)

S자 코스에 장애물이 여러 개(2~4개) 있어도 RETURN 완료 시 CLEAR + 스캔 재판단이
다시 켜지므로 다음 장애물을 이어서 회피한다 (별도 처리 불필요).

AVOID 진입 시 IMU yaw(yaw_start)를 기록해두고, AVOID/RETURN 은 실측 yaw 변화량
(|현재yaw - yaw_start|)으로 진행을 판단한다 (TAOBOTICS HFI IMU,
/taobotics/sensor, sensor_msgs/Imu). PASS 는 여전히 CAN 엔코더 거리로 판단.
회전반경(R, 로그 출력/기하 계산용)은 축거 기반 이론값 사용:
  R = wheelbase / tan(steer_angle)  (bicycle model)

발행: /avoid/state (String), /avoid/cmd_steer (Int16, deg), /avoid/cmd_rpm (Int16, 모니터링용)
      /can/encoder_count, /can/rpm_actual, /can/pwm_duty,
      /can/steer_current_angle_deg, /can/steer_target_angle_deg (모니터링용)
구독: ~scan_topic (LaserScan), ~imu_topic (Imu, yaw 추적),
      ~can_enable_topic (Bool, 실제 구동 arm/disarm)

안전상 CAN enable 기본값은 False. ~can_enable_topic 에 True 가 들어와야
실제로 구동 가능한 프레임(enable=1)을 내보낸다.
"""

import math
import struct
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32, Int16, Int32, String

try:
    import can
except ImportError:
    can = None

ACTIVE_STATES = ('AVOID_LEFT', 'AVOID_RIGHT', 'PASS', 'RETURN')

# avoid_steer_left/right (-30/+30) are RAW firmware-scale values, sent
# straight to CAN unscaled (see _send_can_control) - the firmware's own
# "30" only produces ~14.3deg of true physical wheel angle (measured via
# circle-fit, see waypoint_follower/can_driver.py's
# TRUE_STEER_MAX_ANGLE_DEG/CAN_STEER_SCALE, the source of truth for this
# number). Duplicated here rather than importing across packages, purely
# for the diagnostic R_theory log line below - actual control math never
# uses this (it relies on turn_radius_override, the empirically measured
# radius at that raw command, not a tan()-derived theoretical one).
FIRMWARE_STEER_MAX_ANGLE_DEG = 30.0
TRUE_STEER_MAX_ANGLE_DEG = 14.3


def norm(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def unwrap_int16_delta(prev: int, cur: int) -> int:
    """CAN 프레임의 엔코더 카운트는 signed 16bit 라 랩어라운드될 수 있다."""
    delta = cur - prev
    if delta > 32768:
        delta -= 65536
    elif delta < -32768:
        delta += 65536
    return delta


class ObstacleAvoidNode(Node):

    def __init__(self) -> None:
        super().__init__('obstacle_avoid_node')

        # ============================================================
        # 스캔 / 정면 인식 범위 (HENES 그릴 장착, 초기값 - 실차 튜닝 예정)
        # ============================================================
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('front_angle_deg', 35.0)
        self.declare_parameter('min_range', 0.20)
        self.declare_parameter('max_consider_range', 1.8)

        # 라이다 장착 보정 - HENES 실차. 마운트를 만지면 값이 바뀔 수 있으니
        # 재장착 후엔 반드시 재검증 (2026-07-24 최종 실측: lateral_sign=+1,
        # 통행인 없는 상태에서 좌/우 각각 재확인함).
        self.declare_parameter('angle_offset_deg', 180.0)
        self.declare_parameter('invert_angle', False)
        self.declare_parameter('lateral_sign', 1.0)

        self.declare_parameter('cluster_range_tol', 0.30)
        self.declare_parameter('min_points', 2)

        # ============================================================
        # 차량 제원 (HENES 실차)
        # ============================================================
        self.declare_parameter('vehicle_width', 0.8)
        self.declare_parameter('wheelbase', 0.735)
        # 0 이하면 축거 기반 이론값(bicycle model) 사용. 실측 R(_enter_pass 로그의
        # R_measured) 을 알면 여기에 넣어서 이론값 대신 쓸 수 있음 (더 정확함).
        self.declare_parameter('turn_radius_override', 0.0)
        # 안전마진 - 실차 테스트하며 조정 예정 (임시값)
        self.declare_parameter('safety_margin', 0.15)
        self.declare_parameter('reaction_margin', 0.15)

        self.declare_parameter('avoid_steer_left', -30)
        self.declare_parameter('avoid_steer_right', 30)
        self.declare_parameter('avoid_rpm', 30)
        # CLEAR(장애물 없음) 상태일 때 순항 속도. STOP 은 항상 0 (안전정지 유지).
        self.declare_parameter('cruise_rpm', 15)

        self.declare_parameter('alpha_max_deg', 60.0)
        # 회피호 여유각 (계산된 alpha 보다 조금 더 돌아 확실히 지나침)
        self.declare_parameter('alpha_extra_deg', 5.0)

        # ============================================================
        # 장애물(HENES 동일 차종) 통과거리 - 우려2 대응
        # ============================================================
        self.declare_parameter('obstacle_length', 1.43)
        self.declare_parameter('pass_margin', 0.4)   # 임시값, 실험하며 조정

        # ============================================================
        # 엔코더 환산 (바퀴둘레 / 카운트)
        # ============================================================
        self.declare_parameter('wheel_diameter', 0.27)
        self.declare_parameter('encoder_counts_per_rev', 300)

        # ============================================================
        # 액추에이터 이상감지 (선택, 조향 피드백이 명령과 너무 다르면 STOP)
        # ============================================================
        self.declare_parameter('enable_steer_fault_check', True)
        self.declare_parameter('steer_fault_tol_deg', 15.0)
        self.declare_parameter('steer_fault_hold_sec', 2.5)

        # CAN 피드백이 끊기면(엔코더 콜백 유실) 안전하게 STOP
        self.declare_parameter('feedback_timeout_sec', 1.0)

        # ============================================================
        # IMU (TAOBOTICS HFI, mrpt_sensor_imu_taobotics) - AVOID/RETURN
        # 진행 판단용 yaw 추적
        # ============================================================
        self.declare_parameter('imu_topic', '/taobotics/sensor')
        # RETURN 이 끝났다고 볼 yaw 오차 허용치 (시작 헤딩과 이 이내면 CLEAR)
        self.declare_parameter('yaw_tol_deg', 4.0)
        # IMU 피드백이 끊기면(AVOID/RETURN 중) 안전하게 STOP
        self.declare_parameter('imu_feedback_timeout_sec', 1.0)

        # ============================================================
        # GPS 웨이포인트 방향 가중치 (2026-07-31) - 좌/우 어느 쪽으로 피할지
        # 순수 라이다 포인트 개수(nl/nr)만으로 정하면 GPS 웨이포인트 라인이
        # 어느 방향에 있는지는 전혀 고려 안 함. waypoint_follower_node가
        # 발행하는 cross_track_error_m(양수=차가 라인 오른쪽, 음수=왼쪽 -
        # geo_utils/stanley_controller와 동일 부호규약)를 받아서, 애매한
        # 상황(라이다만으로는 확실하지 않을 때)엔 웨이포인트에 가까워지는
        # 방향을 우선하도록 nl/nr에 가상의 포인트를 얹어준다. 안전이
        # 최우선이라 라이다가 명확히 한쪽을 가리키면(포인트 차이가
        # gps_side_bias_pts보다 크면) 이 가중치로 뒤집히지 않는다.
        self.declare_parameter('gps_cross_track_topic', 'gps_control/cross_track_error_m')
        self.declare_parameter('gps_side_bias_pts', 3)
        self.declare_parameter('gps_bias_timeout_sec', 1.0)

        # ============================================================
        # CAN 통신 (구 can_bridge_node 병합)
        # ============================================================
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('tx_id', 0x200)
        self.declare_parameter('drive_status_id', 0x102)
        self.declare_parameter('steering_status_id', 0x101)
        self.declare_parameter('can_enable_topic', '/can_bridge/enable')
        self.declare_parameter('auto_setup_interface', False)
        self.declare_parameter('reconnect_period_sec', 5.0)
        # When an arbiter node (camera/GPS/avoid priority switch) owns the
        # actual CAN bus, set this false - this node still connects to CAN
        # to *read* feedback (0x101/0x102, needed for pass_progress/
        # steer_fault_check) but no longer *writes* control frames itself;
        # /avoid/cmd_steer and /avoid/cmd_rpm (already published below for
        # monitoring) become the arbiter's input instead. Two nodes both
        # writing 0x200 fight each other - same class of problem as the GPS
        # serial-port conflicts worked through earlier.
        self.declare_parameter('write_can_directly', True)

        # ============================================================
        # 디바운스 / 히스테리시스 (CLEAR/STOP 상태에서만 적용)
        # ============================================================
        self.declare_parameter('decision_hold', 3)
        self.declare_parameter('side_switch_pts', 5)

        self.declare_parameter('rate_hz', 15.0)
        self.declare_parameter('log_period_sec', 1.0)

        scan_topic = str(self.get_parameter('scan_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        can_enable_topic = str(self.get_parameter('can_enable_topic').value)

        # ---- 상태머신 ----
        self.state = 'CLEAR'
        self.side = None          # "LEFT" / "RIGHT"
        self.latest = None
        self._cand = 'CLEAR'
        self._cand_n = 0

        self.alpha_target = 0.0   # 이번 회피에 필요한 회전각 (rad, 여유 포함)
        self.pass_progress = 0.0  # PASS 중 누적 직진거리 (m)
        self.avoid_distance = 0.0  # AVOID 중 누적 이동거리 (m) - 실측 R 계산용

        self._last_encoder = None
        self._last_feedback_ns = None
        self._steer_feedback = None
        self._steer_fault_since_ns = None

        # ---- IMU yaw 상태 ----
        self.cur_yaw = None        # 최신 IMU yaw (rad)
        self.yaw_start = None      # AVOID 진입 시점 yaw (rad)
        self.dyaw = 0.0            # 로깅용: |norm(cur_yaw - yaw_start)|
        self._last_imu_ns = None
        # 실제로 CAN enable=1 프레임을 내보내는 중인지. False 면 차량이 명령을
        # 안 따르는 게 정상이므로 조향 이상감지를 적용하지 않는다.
        self._can_enabled = False

        # ---- GPS 웨이포인트 방향 가중치 상태 ----
        self.gps_cross_track_error = None
        self._last_gps_bias_ns = None

        # ---- CAN 상태 ----
        self.bus = None
        self._last_reconnect_attempt = 0.0
        self.tx_id = int(self.get_parameter('tx_id').value)
        self.drive_status_id = int(self.get_parameter('drive_status_id').value)
        self.steering_status_id = int(self.get_parameter('steering_status_id').value)

        # ---- 퍼블리셔 / 서브스크라이버 ----
        self.pub_state = self.create_publisher(String, '/avoid/state', 10)
        self.pub_steer = self.create_publisher(Int16, '/avoid/cmd_steer', 10)
        self.pub_rpm = self.create_publisher(Int16, '/avoid/cmd_rpm', 10)

        self.pub_encoder = self.create_publisher(Int32, '/can/encoder_count', 10)
        self.pub_rpm_actual = self.create_publisher(Float32, '/can/rpm_actual', 10)
        self.pub_pwm_duty = self.create_publisher(Int32, '/can/pwm_duty', 10)
        self.pub_steer_current = self.create_publisher(
            Float32, '/can/steer_current_angle_deg', 10)
        self.pub_steer_target = self.create_publisher(
            Float32, '/can/steer_target_angle_deg', 10)

        self.sub_scan = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data
        )
        self.sub_imu = self.create_subscription(
            Imu, imu_topic, self.imu_callback, 10
        )
        self.sub_can_enable = self.create_subscription(
            Bool, can_enable_topic, self.can_enable_callback, 10
        )
        self.sub_gps_cross_track = self.create_subscription(
            Float32, self.get_parameter('gps_cross_track_topic').value,
            self.gps_cross_track_callback, 10
        )

        self._try_connect_can(initial=True)

        rate_hz = float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)

        self._last_log_ns = 0

        self.get_logger().info(
            '========================================\n'
            'ObstacleAvoidNode v3 (HENES 실차, CAN 통신 내장)\n'
            f'scan_topic     : {scan_topic}\n'
            f'can_channel    : {self.get_parameter("can_channel").value} '
            f'@ {self.get_parameter("can_bitrate").value} bps\n'
            f'vehicle_width  : {self.get_parameter("vehicle_width").value} m, '
            f'wheelbase: {self.get_parameter("wheelbase").value} m\n'
            f'avoid steer    : {self.get_parameter("avoid_steer_left").value}/'
            f'{self.get_parameter("avoid_steer_right").value} deg -> R='
            f'{self._turn_radius(float(self.get_parameter("avoid_steer_right").value)):.3f} m (이론값)\n'
            f'obstacle_length: {self.get_parameter("obstacle_length").value} m + '
            f'pass_margin {self.get_parameter("pass_margin").value} m\n'
            f'encoder        : {self.get_parameter("encoder_counts_per_rev").value} '
            f'count/rev, wheel_d={self.get_parameter("wheel_diameter").value} m '
            f'-> {self._meters_per_count()*1000:.3f} mm/count\n'
            f'imu_topic      : {imu_topic} (AVOID/RETURN 진행 판단, yaw_tol='
            f'{self.get_parameter("yaw_tol_deg").value}deg)\n'
            f'can_enable_topic: {can_enable_topic} (True 안 들어오면 enable=0 프레임만 송신)\n'
            '회피/통과/복귀 중에는 스캔 재판단을 잠그고 IMU yaw/엔코더 진행률로만 전이함\n'
            '========================================'
        )

    # ================================================================
    # 기하 계산 헬퍼
    # ================================================================
    def _turn_radius(self, steer_deg: float) -> float:
        override = float(self.get_parameter('turn_radius_override').value)
        if override > 0.0:
            return override
        wheelbase = float(self.get_parameter('wheelbase').value)
        steer_rad = math.radians(abs(steer_deg))
        if steer_rad < 1e-6:
            return float('inf')
        return wheelbase / math.tan(steer_rad)

    def _meters_per_count(self) -> float:
        wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        counts_per_rev = float(self.get_parameter('encoder_counts_per_rev').value)
        return (math.pi * wheel_diameter) / counts_per_rev

    @staticmethod
    def _solve_avoid_alpha(C: float, R: float, D: float, alpha_max: float):
        """AVOID 회전(R) 만이 아니라 그 뒤 PASS 직진(D, 꺾인 헤딩 유지)까지
        합쳐서 회피폭 C 를 채우는 데 필요한 최소 alpha(rad) 를 구한다.

        clearance(alpha) = R*(1-cos(alpha))  [AVOID 호가 만드는 옆거리]
                          + D*sin(alpha)       [PASS 가 그 헤딩으로 직진하며
                                                 추가로 벌어지는 옆거리]

        clearance 는 alpha 에 대해 단조증가이므로 이분탐색으로 최소 alpha 를
        찾는다. alpha_max 까지 써도 C 를 못 채우면 None (회피 불가 -> STOP).
        """
        def clearance(a: float) -> float:
            return R * (1 - math.cos(a)) + D * math.sin(a)

        if clearance(alpha_max) < C:
            return None

        lo, hi = 0.0, alpha_max
        for _ in range(40):
            mid = (lo + hi) / 2.0
            if clearance(mid) < C:
                lo = mid
            else:
                hi = mid
        return hi

    # ================================================================
    # 각도 보정
    # ================================================================
    def _adjust(self, raw: float) -> float:
        angle_offset_deg = float(self.get_parameter('angle_offset_deg').value)
        invert_angle = bool(self.get_parameter('invert_angle').value)

        a = norm(raw)
        if invert_angle:
            a = -a
        return norm(a - math.radians(angle_offset_deg))

    # ================================================================
    # LaserScan 콜백: CLEAR/STOP 상태에서만 상태 전이에 관여 (우려1 대응)
    # ================================================================
    def scan_callback(self, scan: LaserScan) -> None:
        front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        min_range = float(self.get_parameter('min_range').value)
        max_consider = float(self.get_parameter('max_consider_range').value)
        lateral_sign = float(self.get_parameter('lateral_sign').value)
        cluster_range_tol = float(self.get_parameter('cluster_range_tol').value)
        min_points = int(self.get_parameter('min_points').value)

        front = math.radians(front_angle_deg)
        pts = []
        n_left = 0
        n_right = 0

        for i, r in enumerate(scan.ranges):
            if math.isnan(r) or math.isinf(r) or r < min_range or r > max_consider:
                continue

            a = self._adjust(scan.angle_min + i * scan.angle_increment)
            if -front <= a <= front:
                s = lateral_sign * r * math.sin(a)   # + 오른쪽 / - 왼쪽
                pts.append((s, r * math.cos(a), r))
                if s < 0:
                    n_left += 1
                elif s > 0:
                    n_right += 1

        if len(pts) < min_points:
            self.latest = None
            self._maybe_feed(('CLEAR', None))
            return

        r_n = min(p[2] for p in pts)
        cluster = [p for p in pts if p[2] <= r_n + cluster_range_tol]
        if len(cluster) < min_points:
            self.latest = None
            self._maybe_feed(('CLEAR', None))
            return

        s_min = min(p[0] for p in cluster)
        s_max = max(p[0] for p in cluster)
        d = min(p[1] for p in cluster)

        info = dict(d=d, s_min=s_min, s_max=s_max, n=len(cluster),
                    n_left=n_left, n_right=n_right)
        self.latest = info

        self._maybe_feed(self._decide(info))

    def _maybe_feed(self, candidate) -> None:
        # AVOID/PASS/RETURN 중에는 스캔 재판단이 상태를 바꾸지 않는다.
        # (latest 는 위에서 이미 갱신되어 로그에는 계속 보인다)
        if self.state in ACTIVE_STATES:
            return
        self._feed(candidate)

    # ================================================================
    # 후보 상태 계산
    # ================================================================
    def _decide(self, info: dict):
        side_switch_pts = int(self.get_parameter('side_switch_pts').value)
        vehicle_width = float(self.get_parameter('vehicle_width').value)
        veh_half = vehicle_width / 2.0
        safety_margin = float(self.get_parameter('safety_margin').value)
        reaction_margin = float(self.get_parameter('reaction_margin').value)
        steer_left = float(self.get_parameter('avoid_steer_left').value)
        steer_right = float(self.get_parameter('avoid_steer_right').value)
        alpha_max = math.radians(float(self.get_parameter('alpha_max_deg').value))

        nl, nr = info['n_left'], info['n_right']

        # GPS 웨이포인트 방향 가중치: cross_track_error>0(차가 라인 오른쪽에
        #있음, 라인은 왼쪽) 이면 LEFT 회피가 라인에 가까워지는 방향이라
        # nr 에, error<0 이면 RIGHT 가 라인 방향이라 nl 에 가상 포인트를
        # 얹는다. 라이다 데이터가 뚜렷하면(포인트 차이가 이 가중치를
        # 넘으면) 그대로 안전 우선 - 이건 어디까지나 애매할 때의 타이브레이커.
        bias_pts = int(self.get_parameter('gps_side_bias_pts').value)
        bias_timeout = float(self.get_parameter('gps_bias_timeout_sec').value)
        gps_bias_fresh = (
            self._last_gps_bias_ns is not None
            and (self.get_clock().now().nanoseconds - self._last_gps_bias_ns) / 1e9 <= bias_timeout
        )
        nl_eff, nr_eff = nl, nr
        if gps_bias_fresh and self.gps_cross_track_error is not None and bias_pts > 0:
            if self.gps_cross_track_error > 0:
                nr_eff = nr + bias_pts
            elif self.gps_cross_track_error < 0:
                nl_eff = nl + bias_pts

        if self.side == 'LEFT':
            side = 'RIGHT' if (nl_eff > nr_eff + side_switch_pts) else 'LEFT'
        elif self.side == 'RIGHT':
            side = 'LEFT' if (nr_eff > nl_eff + side_switch_pts) else 'RIGHT'
        else:
            side = 'LEFT' if nr_eff >= nl_eff else 'RIGHT'

        s_min, s_max, d = info['s_min'], info['s_max'], info['d']
        if side == 'LEFT':
            C = veh_half + safety_margin - s_min
            R = self._turn_radius(steer_left)
        else:
            C = veh_half + safety_margin + s_max
            R = self._turn_radius(steer_right)

        if C <= 0.0:
            return ('CLEAR', None)

        # AVOID 회전만으로 C 를 다 채우지 않는다 - 그 뒤 PASS 가 꺾인 헤딩
        # 그대로 직진(obstacle_length+pass_margin)하는 동안 옆으로 더 벌어지는
        # 만큼도 같이 계산해서, 필요한 최소 alpha 를 구한다 (덜 크게 돎).
        pass_distance = (float(self.get_parameter('obstacle_length').value)
                          + float(self.get_parameter('pass_margin').value))
        alpha = self._solve_avoid_alpha(C, R, pass_distance, alpha_max)
        if alpha is None:
            return ('STOP', None)

        d_min = R * math.sin(alpha) + reaction_margin
        info['C'] = C
        info['R'] = R
        info['alpha'] = alpha
        info['side'] = side
        info['d_min'] = d_min

        if d < d_min:
            return ('STOP', None)

        return ('AVOID_' + side, (side, alpha, R))

    # ================================================================
    # 디바운스: 같은 후보가 decision_hold 번 연속되어야 상태 확정
    # ================================================================
    def _feed(self, candidate) -> None:
        raw, payload = candidate
        decision_hold = int(self.get_parameter('decision_hold').value)

        if raw == self._cand:
            self._cand_n += 1
        else:
            self._cand = raw
            self._cand_n = 1

        if self._cand_n < decision_hold:
            return

        if raw.startswith('AVOID'):
            side, alpha, _R = payload
            self._enter_avoid(side, alpha)
        elif raw == 'CLEAR':
            self.state = 'CLEAR'
            self.side = None
        elif raw == 'STOP':
            self.state = 'STOP'
            # side 는 유지 (다음 스캔의 방향 히스테리시스에 사용)

    def _enter_avoid(self, side: str, alpha: float) -> None:
        alpha_extra = math.radians(float(self.get_parameter('alpha_extra_deg').value))
        self.side = side
        self.alpha_target = alpha + alpha_extra
        self.avoid_distance = 0.0  # 실측 R 계산용 (AVOID 중 누적 이동거리)
        # AVOID 시작 시점 yaw 기록 - PASS 를 거쳐 RETURN 이 끝날 때까지 그대로
        # 유지되며, RETURN 은 이 값으로 되돌아왔는지로 완료를 판단한다.
        self.yaw_start = self.cur_yaw
        self.state = 'AVOID_' + side
        if self.yaw_start is None:
            self.get_logger().warn(
                'IMU yaw 없음 - AVOID 진행 판단 불가 (IMU 연결 확인 필요)'
            )
        self.get_logger().info(
            f'[진입] AVOID_{side} alpha_target={math.degrees(self.alpha_target):.1f}deg'
        )

    # ================================================================
    # 엔코더/조향 피드백 처리: PASS 통과거리 누적 (우려2 핵심)
    # (CAN RX 0x102/0x101 을 직접 파싱해서 호출한다 - 더 이상 ROS 토픽 경유 안 함)
    # ================================================================
    def _handle_drive_status(self, data: bytes) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self._last_feedback_ns = now_ns

        encoder_count, rpm_x10, pwm_duty, _target_rpm = struct.unpack('<hhhh', data)
        self.pub_encoder.publish(Int32(data=int(encoder_count)))
        self.pub_rpm_actual.publish(Float32(data=rpm_x10 / 10.0))
        self.pub_pwm_duty.publish(Int32(data=int(pwm_duty)))

        cur = int(encoder_count)
        if self._last_encoder is None:
            self._last_encoder = cur
            return

        delta_counts = unwrap_int16_delta(self._last_encoder, cur)
        self._last_encoder = cur

        delta_m = delta_counts * self._meters_per_count()

        if self.state in ('AVOID_LEFT', 'AVOID_RIGHT'):
            self.avoid_distance += delta_m

        elif self.state == 'PASS':
            self.pass_progress += delta_m
            obstacle_length = float(self.get_parameter('obstacle_length').value)
            pass_margin = float(self.get_parameter('pass_margin').value)
            if self.pass_progress >= (obstacle_length + pass_margin):
                self._enter_return()

    def _handle_steering_status(self, data: bytes) -> None:
        _cur_pot, _tgt_pot, cur_angle_x10, tgt_angle_x10 = struct.unpack('<HHhh', data)
        self._steer_feedback = cur_angle_x10 / 10.0
        self.pub_steer_current.publish(Float32(data=cur_angle_x10 / 10.0))
        self.pub_steer_target.publish(Float32(data=tgt_angle_x10 / 10.0))

    def _enter_pass(self) -> None:
        # 실측 회전반경 = AVOID 중 이동거리 / 실제 회전각(dyaw, IMU 실측).
        # R_theory(축거 기반 이론값)와 비교해서 alpha 계산이 맞는지 검증하는 용도.
        if self.dyaw > 1e-3:
            r_measured = self.avoid_distance / self.dyaw
            steer_deg_raw = float(self.get_parameter(
                'avoid_steer_right' if self.side == 'RIGHT' else 'avoid_steer_left'
            ).value)
            # steer_deg_raw is firmware-scale (e.g. 30), not the true
            # physical angle it actually produces (~14.3deg) - convert
            # before the tan() so this log's "이론값" is directly
            # comparable to R_measured/turn_radius_override instead of
            # silently assuming raw==true and printing a ~2x-off number.
            steer_deg_true = steer_deg_raw * (
                TRUE_STEER_MAX_ANGLE_DEG / FIRMWARE_STEER_MAX_ANGLE_DEG
            )
            wheelbase = float(self.get_parameter('wheelbase').value)
            r_theory = wheelbase / math.tan(math.radians(abs(steer_deg_true)))
            self.get_logger().info(
                f'[R 실측] 이동거리={self.avoid_distance:.2f}m / dyaw={math.degrees(self.dyaw):.1f}deg '
                f'-> R_measured={r_measured:.3f}m (이론값 R_theory={r_theory:.3f}m, '
                f'steer_raw={steer_deg_raw:.0f} -> true={steer_deg_true:.1f}deg)'
            )
        self.pass_progress = 0.0
        self.state = 'PASS'
        self.get_logger().info('[진입] PASS (통과거리 누적 시작)')

    def _enter_return(self) -> None:
        self.state = 'RETURN'
        self.get_logger().info(
            f'[진입] RETURN yaw_start 로 복귀 (허용오차 '
            f'{self.get_parameter("yaw_tol_deg").value}deg)'
        )

    # ================================================================
    # IMU 콜백: yaw 추적 (AVOID/RETURN 진행 판단, 우려1,2 핵심)
    # ================================================================
    def imu_callback(self, msg: Imu) -> None:
        self._last_imu_ns = self.get_clock().now().nanoseconds
        q = msg.orientation
        self.cur_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def gps_cross_track_callback(self, msg: Float32) -> None:
        self.gps_cross_track_error = msg.data
        self._last_gps_bias_ns = self.get_clock().now().nanoseconds

    def _check_yaw_transitions(self) -> None:
        if self.cur_yaw is None or self.yaw_start is None:
            return
        self.dyaw = abs(norm(self.cur_yaw - self.yaw_start))

        if self.state in ('AVOID_LEFT', 'AVOID_RIGHT'):
            if self.dyaw >= self.alpha_target:
                self._enter_pass()
        elif self.state == 'RETURN':
            yaw_tol = math.radians(float(self.get_parameter('yaw_tol_deg').value))
            if self.dyaw <= yaw_tol:
                self.state = 'CLEAR'
                self.side = None
                self.yaw_start = None
                self._cand = 'CLEAR'
                self._cand_n = 0

    def can_enable_callback(self, msg: Bool) -> None:
        self._can_enabled = bool(msg.data)
        if not self._can_enabled:
            self._steer_fault_since_ns = None
            # disarm 은 다음 재무장 때 항상 CLEAR 부터 새로 시작하도록 상태
            # 머신을 리셋한다 (안 그러면 PASS/AVOID 도중 disarm 했다가 다시
            # enable 했을 때 pass_progress/avoid_distance 가 disarm 이전
            # 값을 그대로 이어받아, 예를 들어 PASS 상태가 새로 시작한 것처럼
            # 보여도 실제로는 남은 누적거리가 왜곡되어 CLEAR/RETURN 전이가
            # 안 되는 문제가 있었다).
            self.state = 'CLEAR'
            self.side = None
            self.yaw_start = None
            self.pass_progress = 0.0
            self.avoid_distance = 0.0
            self._cand = 'CLEAR'
            self._cand_n = 0

    # ================================================================
    # CAN 연결 (구 can_bridge_node)
    # ================================================================
    def _setup_can_interface(self, channel: str, bitrate: int) -> None:
        subprocess.run(['sudo', 'ip', 'link', 'set', channel, 'down'], check=False)
        result = subprocess.run(
            ['sudo', 'ip', 'link', 'set', channel, 'up', 'type', 'can',
             'bitrate', str(bitrate)],
            check=False
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"'{channel}' 인터페이스를 올리지 못했습니다. "
                '어댑터가 꽂혀 있는지, sudo 권한이 있는지 확인하세요.'
            )

    def _try_connect_can(self, initial: bool = False) -> None:
        if can is None:
            if initial:
                self.get_logger().error(
                    "python-can 이 설치되어 있지 않습니다. "
                    "'sudo apt install python3-can' 후 재실행하세요."
                )
            return

        channel = str(self.get_parameter('can_channel').value)
        bitrate = int(self.get_parameter('can_bitrate').value)
        auto_setup = bool(self.get_parameter('auto_setup_interface').value)

        try:
            if auto_setup:
                self._setup_can_interface(channel, bitrate)
            self.bus = can.interface.Bus(interface='socketcan', channel=channel)
            # 인터페이스를 올린 직후 바로 보내면 커널 큐가 아직 준비 안 돼
            # ENOBUFS 로 첫 전송이 실패하는 경우가 있어 짧게 안정화 대기.
            time.sleep(0.3)
            self.get_logger().info(f'CAN 연결 성공: {channel}')
        except Exception as error:
            self.bus = None
            if initial:
                self.get_logger().error(f'CAN 연결 실패 ({channel}): {error}')
            else:
                self.get_logger().warn(f'CAN 연결 실패 ({channel}): {error}')

    @staticmethod
    def _make_control_data(rpm, steer, motor_enable, motor_stop_mode) -> bytes:
        return struct.pack(
            '<hhBBH',
            int(rpm), int(steer), int(motor_enable), int(motor_stop_mode), 0
        )

    def _send_can_control(self, rpm: int, steer: int, enable: bool, stop_mode: int) -> None:
        if self.bus is None:
            return
        data = self._make_control_data(rpm, steer, 1 if enable else 0, stop_mode)
        message = can.Message(arbitration_id=self.tx_id, data=data, is_extended_id=False)

        # ENOBUFS 는 순간적인 큐 정체로도 나서(특히 링크업 직후), 한 번 실패했다고
        # 바로 연결을 끊지 않고 몇 번 짧게 재시도한다.
        max_retries = 3
        for attempt in range(max_retries):
            try:
                self.bus.send(message)
                return
            except can.CanError as error:
                if attempt < max_retries - 1:
                    time.sleep(0.01)
                    continue
                self.get_logger().warn(f'CAN 송신 실패 ({max_retries}회 재시도 후): {error}')
                self.bus = None

    def _drain_can_feedback(self) -> None:
        if self.bus is None:
            return
        while True:
            try:
                message = self.bus.recv(timeout=0.0)
            except can.CanError as error:
                self.get_logger().warn(f'CAN 수신 실패: {error}')
                self.bus = None
                return

            if message is None:
                break

            if message.arbitration_id == self.drive_status_id and len(message.data) == 8:
                self._handle_drive_status(message.data)
            elif message.arbitration_id == self.steering_status_id and len(message.data) == 8:
                self._handle_steering_status(message.data)

    def _can_maintain_connection(self) -> None:
        if self.bus is not None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        period = float(self.get_parameter('reconnect_period_sec').value)
        if now - self._last_reconnect_attempt >= period:
            self._last_reconnect_attempt = now
            self._try_connect_can(initial=False)

    def send_stop_and_close(self) -> None:
        if self.bus is None:
            return
        try:
            self._send_can_control(0, 0, False, 1)
        finally:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

    # ================================================================
    # 출력 타이머
    # ================================================================
    def on_timer(self) -> None:
        steer_left = int(self.get_parameter('avoid_steer_left').value)
        steer_right = int(self.get_parameter('avoid_steer_right').value)
        avoid_rpm = int(self.get_parameter('avoid_rpm').value)
        cruise_rpm = int(self.get_parameter('cruise_rpm').value)

        self._can_maintain_connection()
        self._drain_can_feedback()
        self._check_yaw_transitions()

        self._check_feedback_timeout()
        self._check_imu_feedback_timeout()
        self._check_steer_fault(steer_left, steer_right)

        steer, rpm = 0, 0
        if self.state == 'CLEAR':
            steer, rpm = 0, cruise_rpm
        elif self.state == 'AVOID_LEFT':
            steer, rpm = steer_left, avoid_rpm
        elif self.state == 'AVOID_RIGHT':
            steer, rpm = steer_right, avoid_rpm
        elif self.state == 'PASS':
            steer, rpm = 0, avoid_rpm
        elif self.state == 'RETURN':
            steer = steer_right if self.side == 'LEFT' else steer_left
            rpm = avoid_rpm
        # STOP -> 0, 0 (안전정지 유지)

        stop_mode = 1 if rpm == 0 else 0
        if bool(self.get_parameter('write_can_directly').value):
            self._send_can_control(rpm, steer, self._can_enabled, stop_mode)

        self.pub_state.publish(String(data=self.state))
        self.pub_steer.publish(Int16(data=int(steer)))
        self.pub_rpm.publish(Int16(data=int(rpm)))

        self._log_throttled(steer, rpm)

    def _check_feedback_timeout(self) -> None:
        if self.state not in ACTIVE_STATES:
            return
        # CAN 이 아예 연결된 적 없으면(피드백을 한 번도 못 받음) 이 안전장치는
        # 건너뛴다 - CAN 없이 회피 판단(조향값)만 확인하는 개발/테스트를 위함.
        # 이 경우 진행률이 누적되지 않아 AVOID 상태에 머무르게 된다.
        if self._last_feedback_ns is None:
            return
        timeout = float(self.get_parameter('feedback_timeout_sec').value)
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_feedback_ns) > timeout * 1e9:
            self.get_logger().warn(
                f'[{self.state}] CAN 엔코더 피드백 유실 ({timeout:.1f}s 이상) -> STOP'
            )
            self.state = 'STOP'
            self.side = None
            self._cand = 'STOP'
            self._cand_n = 0

    def _check_imu_feedback_timeout(self) -> None:
        # AVOID/RETURN 만 yaw 로 진행을 판단한다 (PASS 는 엔코더만 씀).
        if self.state not in ('AVOID_LEFT', 'AVOID_RIGHT', 'RETURN'):
            return
        # IMU 를 한 번도 못 받았으면 건너뛴다 - IMU 없이 개발/테스트할 때를 위함
        # (이 경우 yaw_start 도 None 이라 _check_yaw_transitions 가 진행을 안 시킴).
        if self._last_imu_ns is None:
            return
        timeout = float(self.get_parameter('imu_feedback_timeout_sec').value)
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_imu_ns) > timeout * 1e9:
            self.get_logger().warn(
                f'[{self.state}] IMU 피드백 유실 ({timeout:.1f}s 이상) -> STOP'
            )
            self.state = 'STOP'
            self.side = None
            self.yaw_start = None
            self._cand = 'STOP'
            self._cand_n = 0

    def _check_steer_fault(self, steer_left: int, steer_right: int) -> None:
        if not bool(self.get_parameter('enable_steer_fault_check').value):
            return
        if self.state not in ('AVOID_LEFT', 'AVOID_RIGHT', 'RETURN'):
            self._steer_fault_since_ns = None
            return
        # enable=0 이면 차량이 명령을 안 따르는 게 정상이므로 이상감지를 하지 않는다.
        if not self._can_enabled:
            self._steer_fault_since_ns = None
            return
        if self._steer_feedback is None:
            return

        if self.state == 'AVOID_LEFT':
            target = steer_left
        elif self.state == 'AVOID_RIGHT':
            target = steer_right
        else:  # RETURN
            target = steer_right if self.side == 'LEFT' else steer_left

        tol = float(self.get_parameter('steer_fault_tol_deg').value)
        now_ns = self.get_clock().now().nanoseconds

        if abs(self._steer_feedback - target) > tol:
            if self._steer_fault_since_ns is None:
                self._steer_fault_since_ns = now_ns
            hold = float(self.get_parameter('steer_fault_hold_sec').value)
            if (now_ns - self._steer_fault_since_ns) > hold * 1e9:
                self.get_logger().warn(
                    f'[{self.state}] 조향 피드백({self._steer_feedback:.1f}deg)이 '
                    f'명령({target}deg)과 {tol:.0f}deg 이상 차이 -> STOP'
                )
                self.state = 'STOP'
                self.side = None
                self._cand = 'STOP'
                self._cand_n = 0
        else:
            self._steer_fault_since_ns = None

    def _log_throttled(self, steer: int, rpm: int) -> None:
        log_period_sec = float(self.get_parameter('log_period_sec').value)
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns < log_period_sec * 1e9:
            return
        self._last_log_ns = now_ns

        L = self.latest
        if self.state in ('AVOID_LEFT', 'AVOID_RIGHT'):
            self.get_logger().info(
                f'[{self.state}] side={self.side} steer={steer} rpm={rpm} | '
                f'dyaw={math.degrees(self.dyaw):.0f}/'
                f'{math.degrees(self.alpha_target):.0f}deg'
            )
        elif self.state == 'RETURN':
            yaw_tol = float(self.get_parameter('yaw_tol_deg').value)
            self.get_logger().info(
                f'[RETURN] side={self.side} steer={steer} rpm={rpm} | '
                f'dyaw={math.degrees(self.dyaw):.0f}deg (목표<={yaw_tol:.0f}deg)'
            )
        elif self.state == 'PASS':
            obstacle_length = float(self.get_parameter('obstacle_length').value)
            pass_margin = float(self.get_parameter('pass_margin').value)
            self.get_logger().info(
                f'[PASS] steer={steer} rpm={rpm} | '
                f'{self.pass_progress:.2f}/{obstacle_length + pass_margin:.2f} m'
            )
        elif L and 'alpha' in L:
            self.get_logger().info(
                f'[{self.state}] side={self.side} steer={steer} rpm={rpm} | '
                f"d={L['d']:.2f} C={L['C']:.2f} alpha={math.degrees(L['alpha']):.0f}deg | "
                f"L{L['n_left']}/R{L['n_right']} pts | s[{L['s_min']:.2f}~{L['s_max']:.2f}]"
            )
        elif L:
            self.get_logger().info(
                f'[{self.state}] steer={steer} rpm={rpm} | '
                f"L{L['n_left']}/R{L['n_right']} pts | s[{L['s_min']:.2f}~{L['s_max']:.2f}]"
            )
        else:
            self.get_logger().info(f'[{self.state}] steer={steer} rpm={rpm}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_stop_and_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

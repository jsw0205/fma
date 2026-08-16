#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rule_based_t_parking_node.py  (ROS2 / 실차판)

전방 장착 LiDAR 차량의 좌측 T자(직각) 후진 주차 노드.
ROS1 미니카 성공판의 설계 원칙과 상태 흐름을 그대로 유지하고,
플랫폼 차이만 반영했다.

[유지한 것]
  - 후방 블라인드 전제: 슬롯이 옆으로 보이는 접근 구간에서만 관측하고 odom 에 고정.
  - 시간/순서 패턴 금지. 세그먼트 전환은 '측정 yaw 도달'로만 판정.
  - 2원호 닫힌 공식 (theta1 + dtheta2 = 90deg).
  - 상태 흐름: IDLE -> APPROACH -> SETTLE -> SETUP_ARC -> SETTLE
               -> REVERSE_ARC -> SETTLE -> REVERSE_STRAIGHT -> PARKED
               -> SETTLE -> EXIT_STRAIGHT -> DONE
    (PARKED/EXIT_STRAIGHT: 2026-08-05 추가. T자는 후진 진입 후 앞이 차로
    쪽을 향하므로 출차는 parallel_parking처럼 S자 경로가 필요 없고 직진
    전진만 하면 된다. REVERSE_STRAIGHT 완료 시 바로 DONE(제어권 반환)하지
    않고 PARKED에서 대기 - /parking_exit_start 또는 auto_exit로
    EXIT_STRAIGHT 진입, entrance_clearance()가 뒷범퍼의 lock된(odom 고정)
    슬롯 입구선 통과를 확인하면 DONE + /parking_exit_done=true. 이 시점엔
    슬롯이 차량 전방/측방 시야 밖이라 라이다 재확인이 불가능 - 이미 lock
    해둔 슬롯 기하에 뒷범퍼 코너를 투영하는 것만으로 충분히 정확하고
    독립적으로 판단 가능하다는 게 이 설계의 핵심.)

[바꾼 것]
  1. ROS1 -> ROS2 (rclpy). tf.transformations 제거, 쿼터니언 직접 구현.
  2. 차량 제원 교체 (L=1.410, W=0.800, WB=0.735, base_link=뒤 차축,
     뒤축->앞범퍼 1.175, 뒤축->뒤범퍼 0.230).
  3. 주차 방향 우측 -> 좌측 (parking_side=left, ROI y>0).
  4. LiDAR 뒤집힘 반영.
     p_base = Rz(laser_yaw) * Rx(pi) * p_sensor  =>  유효각 = laser_yaw - theta_raw
     raw 180=앞 / 90=좌 / 270=우 / 0=뒤 규약이 laser_yaw=pi, angle_sign=-1 과 정확히 일치.
  5. 엔코더 구동축이 앞 차축 -> odom 노드에서 전축 자전거모델로 적분(별도 노드).
     이 노드는 base_link(뒤 차축) 기준 odom 을 받는다는 계약만 유지.
  6. 콘 검출 / 슬롯 검출 전면 재작성 (cone_detector.py, slot_detector.py).
  7. 2원호 목표깊이 분해 (two_arc_planner.py).
     실차 R_rev(1.27 m) < z_goal(1.57 m) 이라 단발 2원호가 불가능하므로
     z_arc(원호 담당) + z_straight(직진후진 담당) 으로 나눈다.

입력
  /scan_parking   sensor_msgs/LaserScan
  /wheel_odom     nav_msgs/Odometry      (base_link = 뒤 차축)
  /parking_start  std_msgs/Bool
  /parking_reset  std_msgs/Bool
  /parking_exit_start  std_msgs/Bool  (PARKED 상태일 때만 반응, 2026-08-05)

출력
  /parking/cmd_rpm, /parking/cmd_steer, /parking/cmd_enable   std_msgs/Int16
  direct_cmd_output=true 이면 /cmd_rpm, /cmd_steer, /cmd_enable 에도 발행
  /parking_request_stop, /parking_active, /parking_mapping, /parking_done  Bool
  /parking_status String, /parking/cones PoseArray,
  /parking/goal_pose PoseStamped, /parking/markers MarkerArray
  /parking_exit_done  Bool  (EXIT_STRAIGHT 완료 시 - 2026-08-05)
"""

import csv
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int16, String
from visualization_msgs.msg import Marker, MarkerArray

from .cone_detector import ConeDetector, ConeParams, ConeTracker
from .geometry import (clamp, interp_table, normalize_angle, parse_table_param,
                       quat_from_yaw, transform_to_local, transform_to_world,
                       unit, yaw_from_quat)
from .slot_detector import SlotDetector, SlotParams
from .temporal_gap_detector import TemporalSideGapDetector
from .two_arc_planner import TwoArcPlanner

VEHICLE_MAX_STEER_CMD = 30


def latched_qos(depth=1):
    return QoSProfile(depth=depth,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=QoSReliabilityPolicy.RELIABLE)


class CsvLogger(object):
    def __init__(self, enabled, base_dir):
        self.enabled = bool(enabled)
        self.base_dir = base_dir
        self.files, self.writers = {}, {}
        self.closed = False
        if self.enabled:
            try:
                os.makedirs(self.base_dir, exist_ok=True)
            except Exception:
                self.enabled = False

    def open_writer(self, name, header):
        if not self.enabled or self.closed:
            return
        try:
            f = open(os.path.join(self.base_dir, name), 'w', newline='')
            w = csv.writer(f)
            w.writerow(header)
            self.files[name], self.writers[name] = f, w
        except Exception:
            pass

    def row(self, name, values):
        if not self.enabled or self.closed:
            return
        w = self.writers.get(name)
        if w is None:
            return
        try:
            w.writerow(values)
        except Exception:
            pass

    def close(self):
        self.closed = True
        for f in list(self.files.values()):
            try:
                f.flush()
                f.close()
            except Exception:
                pass


class RuleBasedTParkingNode(Node):

    # ==================================================================
    def __init__(self):
        super().__init__('rule_based_t_parking_node')
        self._throttle = {}

        # ---------------- topics / frames ----------------
        self.scan_topic = self.p('scan_topic', '/scan_parking')
        self.odom_topic = self.p('odom_topic', '/wheel_odom')
        self.odom_frame = self.p('odom_frame', 'odom')
        self.base_frame = self.p('base_frame', 'base_link')

        # ---------------- LiDAR 장착 (뒤집힘 포함) ----------------
        # 유효각 = laser_yaw + laser_yaw_extra + laser_angle_sign * theta_raw
        # 뒤집힘(바닥이 천장) => Rx(pi) => angle_sign = -1
        # raw 180deg = 차량 앞    => laser_yaw = pi
        self.laser_x = float(self.p('laser_x', 1.175))
        self.laser_y = float(self.p('laser_y', 0.0))
        self.laser_yaw = float(self.p('laser_yaw', math.pi))
        self.laser_yaw_extra = float(self.p('laser_yaw_extra', 0.0))
        self.laser_angle_sign = float(self.p('laser_angle_sign', -1.0))
        self.laser_angle_sign = 1.0 if self.laser_angle_sign >= 0.0 else -1.0

        # ---------------- 차량 제원 ----------------
        self.vehicle_length = float(self.p('vehicle_length', 1.410))
        self.vehicle_width = float(self.p('vehicle_width', 0.800))
        self.wheel_base = float(self.p('wheel_base', 0.735))
        self.track_width = float(self.p('track_width', 0.670))
        self.base_to_front_bumper = float(self.p('base_to_front_bumper', 1.175))
        self.base_to_rear_bumper = float(self.p('base_to_rear_bumper', 0.230))

        self.side_margin = float(self.p('side_margin', 0.15))
        self.back_margin = float(self.p('back_margin', 0.20))
        self.min_goal_inside_depth = float(self.p('min_goal_inside_depth', 0.10))
        self.collision_margin = float(self.p('collision_margin', 0.10))

        # ---------------- ROI / side ----------------
        self.side = str(self.p('parking_side', 'left')).strip().lower()
        self.roi_x_min = float(self.p('roi_x_min', -0.60))
        self.roi_x_max = float(self.p('roi_x_max', 8.00))
        self.roi_y_min = float(self.p('roi_y_min', 0.20))
        self.roi_y_max = float(self.p('roi_y_max', 4.50))
        self.self_mask_extra = float(self.p('self_mask_extra', 0.08))

        # ---------------- 콘 검출 ----------------
        self.cone_params = ConeParams(
            cone_height=float(self.p('cone_height', 0.450)),
            cone_base_diameter=float(self.p('cone_base_diameter', 0.280)),
            cone_plate_diameter=float(self.p('cone_plate_diameter', 0.0)),
            scan_height=float(self.p('scan_height', 0.160)),
            min_range=float(self.p('min_range', 0.25)),
            max_range=float(self.p('max_range', 8.0)),
            range_sigma=float(self.p('range_sigma', 0.020)),
            cluster_lambda_deg=float(self.p('cluster_lambda_deg', 10.0)),
            cluster_gap_min=float(self.p('cluster_gap_min', 0.030)),
            cluster_gap_max=float(self.p('cluster_gap_max', 0.220)),
            max_index_gap=int(self.p('max_index_gap', 3)),
            min_points_abs=int(self.p('min_cluster_points', 1)),
            max_points_abs=int(self.p('max_cluster_points', 400)),
            n_exp_min_ratio=float(self.p('n_exp_min_ratio', 0.30)),
            n_exp_max_ratio=float(self.p('n_exp_max_ratio', 3.20)),
            width_min_ratio=float(self.p('width_min_ratio', 0.25)),
            width_max_ratio=float(self.p('width_max_ratio', 1.80)),
            width_abs_max=float(self.p('max_cluster_width', 0.45)),
            isolation_enable=bool(self.p('isolation_enable', True)),
            isolation_jump=float(self.p('isolation_jump', 0.22)),
            isolation_search=int(self.p('isolation_search', 4)),
            isolation_require_both=bool(self.p('isolation_require_both', False)),
            convexity_enable=bool(self.p('convexity_enable', True)),
            convexity_min_points=int(self.p('convexity_min_points', 4)),
            sagitta_min=float(self.p('sagitta_min', 0.004)),
            sagitta_max_ratio=float(self.p('sagitta_max_ratio', 0.75)),
            radial_spread_max_ratio=float(self.p('radial_spread_max_ratio', 1.30)),
            use_intensity=bool(self.p('use_intensity', False)),
            min_intensity=float(self.p('min_intensity', 0.0)),
            track_merge_dist=float(self.p('track_merge_dist', 0.28)),
            track_alpha=float(self.p('track_alpha', 0.45)),
            min_track_hits=int(self.p('min_track_hits', 3)),
            max_track_miss_sec=float(self.p('max_track_miss_sec', 1.2)),
            far_track_range=float(self.p('far_track_range', 3.0)),
            far_min_track_hits=int(self.p('far_min_track_hits', 2)),
            far_track_miss_sec=float(self.p('far_track_miss_sec', 2.5)),
        )
        self.cone_detector = ConeDetector(self.cone_params)
        self.cone_tracker = ConeTracker(self.cone_params)

        # ---------------- 슬롯 검출 ----------------
        self.slot_params = SlotParams(
            lane_fit_enable=bool(self.p('lane_fit_enable', True)),
            lane_ransac_tol=float(self.p('lane_ransac_tol', 0.18)),
            lane_ransac_iters=int(self.p('lane_ransac_iters', 150)),
            lane_max_angle_deg=float(self.p('lane_max_angle_deg', 25.0)),
            lane_min_inliers=int(self.p('lane_min_inliers', 3)),
            entry_min_width=float(self.p('entry_min_width', 1.05)),
            entry_max_width=float(self.p('entry_max_width', 2.60)),
            entry_expected_width=float(self.p('entry_expected_width', 1.30)),
            entry_pair_max_depth_diff=float(self.p('entry_pair_max_depth_diff', 0.30)),
            entry_line_tol=float(self.p('entry_line_tol', 0.30)),
            entry_require_adjacent=bool(
                self.p('entry_require_adjacent', True)),
            entry_adjacent_margin=float(
                self.p('entry_adjacent_margin', 0.15)),
            max_candidate_entry_dist=float(self.p('max_candidate_entry_dist', 7.0)),
            candidate_min_lateral_abs=float(
                self.p('candidate_min_lateral_abs', 0.0)),
            candidate_max_lateral_abs=float(
                self.p('candidate_max_lateral_abs', 0.0)),
            entrance_gap_mode=bool(self.p('entrance_gap_mode', False)),
            entrance_default_depth=float(self.p('entrance_default_depth', 2.00)),
            slot_min_depth=float(self.p('slot_min_depth', 1.50)),
            slot_max_depth=float(self.p('slot_max_depth', 3.50)),
            slot_expected_depth=float(self.p('slot_expected_depth', 2.00)),
            depth_percentile=float(self.p('depth_percentile', 88.0)),
            rear_cone_band=float(self.p('rear_cone_band', 0.30)),
            wall_lateral_extra=float(self.p('wall_lateral_extra', 0.25)),
            depth_min_positive_v=float(self.p('depth_min_positive_s', 0.15)),
            goal_depth_override=float(self.p('goal_depth_override', 0.0)),
            interior_cone_reject=bool(self.p('interior_cone_reject', True)),
            interior_margin_u=float(self.p('interior_margin_u', 0.12)),
            interior_back_band=float(self.p('interior_back_band', 0.35)),
            interior_front_band=float(self.p('interior_front_band', 0.10)),
            max_interior_cones=int(self.p('max_interior_cones', 0)),
            interior_raw_reject=bool(self.p('interior_raw_reject', True)),
            interior_raw_min_points=int(self.p('interior_raw_min_points', 6)),
            interior_raw_clear_ratio=float(self.p('interior_raw_clear_ratio', 0.75)),
            entry_boundary_check_enable=bool(self.p('entry_boundary_check_enable', True)),
            entry_front_tolerance=float(self.p('entry_front_tolerance', 0.20)),
            max_entry_front_count=int(self.p('max_entry_front_count', 1)),
            entry_offset=float(self.p('entry_offset', 0.05)),
            min_goal_inside_depth=self.min_goal_inside_depth,
            min_cones_for_plan=int(self.p('min_cones_for_plan', 3)),
        )
        self.slot_detector = SlotDetector(self.slot_params, {
            'vehicle_width': self.vehicle_width,
            'side_margin': self.side_margin,
            'back_margin': self.back_margin,
            'base_to_rear_bumper': self.base_to_rear_bumper,
            'collision_margin': self.collision_margin,
        })

        # ---------------- 주행 전이 기반 입구 검출 ----------------
        # 순간 콘쌍 대신 측면 경계의 막힘->열림->막힘을 odom 거리로 잰다.
        self.temporal_gap_enable = bool(
            self.p('temporal_gap_enable', True))
        self.gap_side_window_along = max(
            0.05, float(self.p('gap_side_window_along', 0.22)))
        # 차폭 안쪽 반사는 외부 경계일 수 없다. 이전 실차 로그의 0.31 m
        # 고정값(차량 반폭 0.40 m 안쪽)을 제거하는 하한이다.
        self.gap_side_min_lateral = max(
            self.cone_params.min_range,
            float(self.p(
                'gap_side_min_lateral',
                0.5 * self.vehicle_width + self.self_mask_extra)))
        self.gap_detector = TemporalSideGapDetector(
            min_width=max(
                self.slot_params.entry_min_width,
                self.vehicle_width + 2.0 * self.side_margin),
            max_width=self.slot_params.entry_max_width,
            laser_x=self.laser_x,
            laser_y=self.laser_y,
            side=self.side,
            open_jump=float(self.p('gap_open_jump', 0.60)),
            close_jump=float(self.p('gap_close_jump', 0.30)),
            confirm_frames=int(self.p('gap_confirm_frames', 3)),
            baseline_alpha=float(self.p('gap_baseline_alpha', 0.25)),
        )

        # ---------------- 출차 (2026-08-05) ----------------
        # T자는 후진으로 들어간 뒤 앞이 차로 쪽을 향하므로 직진 전진만
        # 하면 나온다 - parallel_parking처럼 S자 경로를 새로 짤 필요가
        # 없음. REVERSE_STRAIGHT 완료 시점에 바로 제어권을 넘기지 않고
        # PARKED에서 대기했다가, /parking_exit_start(또는 auto_exit)로
        # EXIT_STRAIGHT에 들어가 approach_cmd()와 동일한 시작 헤딩
        # 유지 컨트롤로 전진, entrance_clearance()가 뒷범퍼가 입구선
        # 바깥으로 나갔다고 판단하면 DONE + /parking_exit_done=true.
        self.auto_exit = bool(self.p('auto_exit', False))
        self.parking_hold_sec = float(self.p('parking_hold_sec', 3.0))
        self.exit_forward_rpm = int(self.p('exit_forward_rpm', 8))
        # 뒷범퍼가 입구선(M)을 이 여유만큼 지나야 완전히 빠져나온 걸로
        # 판단 - 라이다로 직접 재확인하는 대신(전방 카메라 시야 밖) 이미
        # lock 해둔 슬롯 기하(odom 고정)에 뒷범퍼 코너를 투영해서 판단.
        # 값을 키우면 더 확실히 빠져나온 뒤에 핸드오프, 너무 크게 잡으면
        # 옆 차로까지 밀고 나가서 GPS로 넘기기 전에 더 진행함.
        self.exit_clear_margin = float(self.p('exit_clear_margin', 0.30))
        self.exit_timeout_sec = float(self.p('exit_timeout_sec', 30.0))

        # ---------------- ABORT 자동 복귀 (2026-08-11) ----------------
        # ABORT는 원래 자동복구를 안 함(사람이 /parking_reset 해줘야 함) -
        # "더 가면 위험할 수도 있어 안전상 멈춘" 상태라 알아서 다시 GPS로
        # 튀어나가면 위험할 수 있어서 의도적으로 그렇게 설계했었음. 근데
        # 실제로 도달 가능한 ABORT 원인이 지금은 사실상 rear_clearance
        # safety stop 하나뿐이고(APPROACH 타임아웃은 pre_straight_
        # timeout_sec=0으로 꺼져있고, plan collision abort도
        # continue_on_plan_collision=true라 안 불림) - 이 경우는 T자
        # 특성상 코가 이미 차로 쪽을 향하고 있어서(후진 진입 상태), 그냥
        # 앞으로 조금 나가는 게 EXIT_STRAIGHT와 방향상 동일하게 안전한
        # 회피 동작임. abort_hold_sec 대기 후 abort_forward_sec 동안
        # exit_forward_rpm으로 직진하고 DONE 처리 - 시간 기반(거리/
        # entrance_clearance 재확인 없음), 사용자 명시적 요청("abort뜨고
        # 3초 뒤에 앞으로 2초 나오고 복귀").
        self.auto_recover_from_abort = bool(self.p('auto_recover_from_abort', False))
        self.abort_hold_sec = float(self.p('abort_hold_sec', 3.0))
        self.abort_forward_sec = float(self.p('abort_forward_sec', 2.0))
        self.abort_start_time = None

        # ---------------- 조향 / 반경 모델 ----------------
        req = abs(int(self.p('max_steer_cmd', VEHICLE_MAX_STEER_CMD)))
        self.max_steer_cmd = min(req, VEHICLE_MAX_STEER_CMD)
        self.radius_mode = str(self.p('radius_mode', 'bicycle')).strip().lower()
        self.max_steer_angle_left_deg = float(self.p('max_steer_angle_left_deg', 30.0))
        self.max_steer_angle_right_deg = float(self.p('max_steer_angle_right_deg', 30.0))
        self.left_radius_table = parse_table_param(
            self.p('left_radius_table', ''), [])
        self.right_radius_table = parse_table_param(
            self.p('right_radius_table', ''), [])
        self.radius_is_center = bool(self.p('radius_is_center', False))
        self.forward_turn_sign = float(self.p('forward_turn_sign', -1.0))
        self.setup_steer_abs = int(self.p('setup_steer_abs', self.max_steer_cmd))
        self.reverse_steer_abs = int(self.p('reverse_steer_abs', self.max_steer_cmd))
        self.steer_deadband_cmd = int(self.p('steer_deadband_cmd', 2))

        # ---------------- 속도 명령 ----------------
        self.pre_straight_rpm = int(self.p('pre_straight_rpm', 10))
        self.setup_forward_rpm = int(self.p('setup_forward_rpm', 10))
        self.reverse_rpm = int(self.p('reverse_rpm', -10))
        self.final_rpm = int(self.p('final_rpm', -8))

        # ---------------- 플래너 ----------------
        self.planner = TwoArcPlanner(
            {'wheel_base': self.wheel_base,
             'vehicle_width': self.vehicle_width,
             'base_to_front_bumper': self.base_to_front_bumper,
             'base_to_rear_bumper': self.base_to_rear_bumper,
             'collision_margin': self.collision_margin},
            {'arc_depth_margin': float(self.p('arc_depth_margin', 0.10)),
             'min_z_arc': float(self.p('min_z_arc', -1.20)),
             'max_straight_back': float(self.p('max_straight_back', 3.00)),
             'sim_ds': float(self.p('sim_ds', 0.03)),
             'validate_plan': bool(self.p('validate_plan', True))})
        self.plan_min_clearance = float(self.p('plan_min_clearance', 0.05))
        self.abort_on_plan_collision = bool(self.p('abort_on_plan_collision', True))
        self.continue_on_plan_collision = bool(
            self.p('continue_on_plan_collision', True))

        # ---------------- APPROACH ----------------
        self.cmd_rate = float(self.p('cmd_rate', 20.0))
        self.stop_speed_thresh = float(self.p('stop_speed_thresh', 0.05))
        self.stop_hold_sec = float(self.p('stop_hold_sec', 1.0))
        # See parallel_parking's identical fix (2026-08-12) - SETTLE
        # otherwise waits forever for vx to drop below stop_speed_thresh,
        # but stop_mode=1 (flat stop, sent whenever cmd_rpm=0) releases the
        # closed-loop hold PID entirely, so on even a slight slope the
        # vehicle can creep one direction indefinitely and vx never
        # settles. Past this timeout SETTLE advances anyway.
        self.settle_timeout_sec = float(self.p('settle_timeout_sec', 5.0))
        self.approach_yaw_kp = float(self.p('pre_straight_yaw_kp', 12.0))
        self.approach_wz_kd = float(self.p('pre_straight_wz_kd', 12.0))
        self.approach_steer_sign = int(self.p('pre_straight_steer_sign', 1))
        self.approach_max_steer = int(self.p('pre_straight_max_steer', 8))
        self.approach_min_steer = max(
            0, int(self.p('pre_straight_min_steer', 0)))
        self.approach_yaw_deadband = math.radians(max(
            0.0, float(self.p('pre_straight_yaw_deadband_deg', 0.3))))
        self.approach_min_lateral_abs = float(self.p('pre_straight_min_lateral_abs', 0.60))
        self.approach_arm_x_min = float(self.p('pre_straight_arm_x_min', 1.00))
        self.lock_stable_window = max(1, int(self.p('pre_straight_stable_window', 4)))
        self.lock_width_tol = float(self.p('pre_straight_width_stable_tol', 0.15))
        self.lock_depth_tol = float(self.p('pre_straight_depth_stable_tol', 0.20))
        self.lock_modom_tol = float(self.p('lock_modom_stable_tol', 0.20))
        self.approach_timeout_sec = float(self.p('pre_straight_timeout_sec', 0.0))
        self.lock_require_feasible = bool(self.p('lock_require_feasible', True))
        self.lock_require_start_ahead = bool(self.p('lock_require_start_ahead', True))
        self.lock_start_ahead_margin = float(self.p('lock_start_ahead_margin', 0.25))

        # ---------------- 아크 / 종단 ----------------
        self.setup_yaw_tolerance_deg = float(self.p('setup_yaw_tolerance_deg', 3.0))
        self.reverse_yaw_tolerance_deg = float(self.p('reverse_yaw_tolerance_deg', 3.0))
        self.arc_absolute_timeout_sec = float(self.p('arc_absolute_timeout_sec', 60.0))
        self.setup_max_forward_dist = float(self.p('setup_max_forward_dist', 2.50))
        self.reverse_max_dist = float(self.p('reverse_max_dist', 6.00))
        self.reverse_align_yaw_kp = float(self.p('reverse_yaw_kp', 6.0))
        self.reverse_align_center_kp = float(self.p('reverse_center_kp', 9.0))
        self.reverse_align_max_steer = int(self.p('reverse_align_max_steer', 10))
        self.recompute_at_start = bool(self.p('recompute_at_start', True))

        # REVERSE_STRAIGHT 에서 전방 입구콘 재관측으로 중앙선 보정 (선택)
        self.rs_use_live_entry = bool(self.p('reverse_straight_use_live_entry', False))
        self.rs_live_weight = float(self.p('reverse_straight_live_weight', 0.4))

        # ---------------- 완료 조건 ----------------
        self.centerline_tolerance = float(self.p('centerline_tolerance', 0.08))
        self.rear_goal_tolerance = float(self.p('rear_goal_tolerance', 0.10))
        self.rear_safety_stop_margin = float(
            self.p('rear_safety_stop_margin', self.back_margin + 0.05))
        self.final_yaw_tolerance_deg = float(self.p('final_yaw_tolerance_deg', 6.0))
        self.final_wz_tolerance = float(self.p('final_wz_tolerance', 0.12))
        self.final_start_yaw_tolerance_deg = float(
            self.p('final_start_yaw_tolerance_deg', 8.0))

        # ---------------- 출력 / 로깅 ----------------
        self.auto_start = bool(self.p('auto_start', False))
        self.direct_cmd_output = bool(self.p('direct_cmd_output', False))
        self.log_enable = bool(self.p('log_enable', True))
        log_root = os.path.expanduser(
            str(self.p('log_root', '~/.ros/parking_logs')))
        self.log_dir = os.path.join(log_root, 'rule_t_%s' % time.strftime('%Y%m%d_%H%M%S'))
        self.csv = CsvLogger(self.log_enable, self.log_dir)
        self.csv.open_writer('control.csv', [
            't', 'state', 'x', 'y', 'yaw_deg', 'yaw_from_start_deg',
            'along_lane', 'lateral', 'center_err', 'rear_clearance', 'rpm', 'steer'])
        self.csv.open_writer('plan.csv', [
            't', 'y_a', 'z_goal', 'z_arc', 'z_straight', 'R_setup', 'R_rev',
            'theta1_deg', 'dtheta2_deg', 'Sx', 'setup_steer', 'reverse_steer',
            'required_aisle', 'sim_min_clear', 'sim_collision', 'feasible'])
        self.csv.open_writer('cones.csv', ['t', 'n_raw', 'n_track', 'n_confirmed'])

        # ---------------- runtime ----------------
        self.state = 'IDLE'
        self.next_state = None
        self.settle_start = None
        self.odom_received = False
        self.x = self.y = self.yaw = 0.0
        self.vx = self.wz = 0.0
        self.last_scan = None
        self.pending_start_reason = None

        self.sequence_start_yaw = 0.0
        self.approach_start_time = None
        self.lock_hist = deque(maxlen=self.lock_stable_window)
        self.armed = False
        self.locked = False
        self.fixed_slot_meta = None
        self.fixed_goal_odom = None
        self.fixed_yaw_goal_odom = 0.0
        self.expected_start_yaw_delta = math.pi / 2.0
        self.plan = None
        self.setup_start_yaw = None
        self.setup_start_pose = None
        self.reverse_start_pose = None
        self.phase_start_time = None
        self.best_candidate = None
        self.done = False
        self.abort_reason = ''
        self.live_center_corr = 0.0
        self.gap_detector.reset(0.0)
        self.parked_start_time = None
        self.exit_requested = False
        self.exit_start_time = None
        self.abort_start_time = None

        # ---------------- publishers ----------------
        self.pub_cmd_rpm = self.create_publisher(Int16, '/parking/cmd_rpm', 10)
        self.pub_cmd_steer = self.create_publisher(Int16, '/parking/cmd_steer', 10)
        self.pub_cmd_enable = self.create_publisher(Int16, '/parking/cmd_enable', 10)
        self.pub_active = self.create_publisher(Bool, '/parking_active', latched_qos())
        self.pub_mapping = self.create_publisher(Bool, '/parking_mapping', latched_qos())
        self.pub_done = self.create_publisher(Bool, '/parking_done', latched_qos())
        self.pub_exit_done = self.create_publisher(Bool, '/parking_exit_done', latched_qos())
        self.pub_stop_req = self.create_publisher(Bool, '/parking_request_stop', latched_qos())
        self.pub_status = self.create_publisher(String, '/parking_status', 10)
        self.pub_cones = self.create_publisher(PoseArray, '/parking/cones', 5)
        self.pub_goal = self.create_publisher(PoseStamped, '/parking/goal_pose', 3)
        self.pub_markers = self.create_publisher(MarkerArray, '/parking/markers', 3)
        self.pub_direct_rpm = self.pub_direct_steer = self.pub_direct_enable = None
        if self.direct_cmd_output:
            self.pub_direct_rpm = self.create_publisher(Int16, '/cmd_rpm', 10)
            self.pub_direct_steer = self.create_publisher(Int16, '/cmd_steer', 10)
            self.pub_direct_enable = self.create_publisher(Int16, '/cmd_enable', 10)

        # ---------------- subscribers ----------------
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)
        self.create_subscription(Bool, '/parking_start', self.start_cb, 5)
        self.create_subscription(Bool, '/parking_reset', self.reset_cb, 5)
        self.create_subscription(Bool, '/parking_exit_start', self.exit_start_cb, 5)

        self.create_timer(1.0 / max(self.cmd_rate, 1.0), self.on_timer)

        self._banner()
        if self.auto_start:
            self.create_timer(0.5, self._auto_start_once)

    # ==================================================================
    def p(self, name, default):
        self.declare_parameter(name, default)
        v = self.get_parameter(name).value
        return default if v is None else v

    def _auto_start_once(self):
        if self.state == 'IDLE':
            self.begin_sequence('auto_start')

    def _banner(self):
        cp = self.cone_params
        w_exp = cp.expected_chord_width()
        dphi = math.radians(float(self.p('assumed_angle_increment_deg', 0.1125)))
        r1 = cp.max_detect_range(dphi, 1.0)
        r2 = cp.max_detect_range(dphi, 2.0)
        r3 = cp.max_detect_range(dphi, 3.0)
        r_setup = self.radius_for_cmd(-self.setup_steer_abs)
        r_rev = self.radius_for_cmd(self.reverse_steer_abs)
        log = self.get_logger()
        log.warn('=========== rule_based_t_parking (ROS2 / real vehicle) ===========')
        log.warn(' side=%s  steer_limit=+/-%d  radius_mode=%s'
                 % (self.side, self.max_steer_cmd, self.radius_mode))
        log.warn(' vehicle L=%.3f W=%.3f WB=%.3f  front=%.3f rear=%.3f'
                 % (self.vehicle_length, self.vehicle_width, self.wheel_base,
                    self.base_to_front_bumper, self.base_to_rear_bumper))
        log.warn(' laser x=%.3f y=%.3f yaw=%.3frad angle_sign=%+.0f (뒤집힘=-1)'
                 % (self.laser_x, self.laser_y, self.laser_yaw, self.laser_angle_sign))
        log.warn(' R_setup=%.3f  R_rev=%.3f  (뒤 차축 기준)' % (r_setup, r_rev))
        log.warn(' 콘 단면폭 예상=%.3f m @scan_h=%.3f  (h>=cone_h 면 0 -> 검출 불가)'
                 % (w_exp, cp.scan_height))
        log.warn(' 콘 최대검출거리 @%.2fdeg : 1pt=%.2fm  2pt=%.2fm  3pt=%.2fm'
                 % (math.degrees(dphi), r1, r2, r3))
        log.warn(' 2원호 한계 : y_a + z_arc <= R_rev = %.3f m' % r_rev)
        log.warn(' log_dir=%s' % self.log_dir)
        log.warn('=================================================================')
        if w_exp < 0.03:
            log.error(' [경고] 스캔높이에서 콘 단면폭이 %.3f m 뿐입니다. '
                      '라이다를 낮추거나 더 큰 콘을 쓰지 않으면 검출이 불가능합니다.' % w_exp)

    # ---------------- time / log helpers ----------------
    def now(self):
        return self.get_clock().now()

    @staticmethod
    def dt(t1, t0):
        return float((t1 - t0).nanoseconds) * 1e-9

    def tsec(self):
        return float(self.now().nanoseconds) * 1e-9

    def warn(self, msg):
        self.get_logger().warn(msg)

    def status(self, text, period=0.5):
        self.pub_status.publish(String(data=text))
        key = text.split(':')[0]
        t = self.tsec()
        if t - self._throttle.get(key, -1e9) >= period:
            self._throttle[key] = t
            self.get_logger().info(text)

    # ==================================================================
    #                          callbacks
    # ==================================================================
    def start_cb(self, msg):
        if bool(msg.data) and self.state in ('IDLE', 'DONE', 'ABORT'):
            self.begin_sequence('/parking_start')

    def reset_cb(self, msg):
        if bool(msg.data):
            self.reset_all('/parking_reset')

    def exit_start_cb(self, msg):
        if bool(msg.data) and self.state == 'PARKED':
            self.exit_requested = True

    def odom_cb(self, msg):
        self.odom_received = True
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.vx = float(msg.twist.twist.linear.x)
        self.wz = float(msg.twist.twist.angular.z)
        if self.pending_start_reason is not None and self.state in ('IDLE', 'DONE', 'ABORT'):
            reason = self.pending_start_reason
            self.pending_start_reason = None
            self.begin_sequence('%s after_first_odom' % reason)

    def scan_cb(self, msg):
        self.last_scan = msg
        if self.state == 'APPROACH':
            self.process_scan_approach(msg)
        elif self.state in (
            'SETUP_ARC', 'REVERSE_ARC', 'REVERSE_STRAIGHT', 'SETTLE',
            'PARKED', 'EXIT_STRAIGHT',
        ):
            if self.state == 'REVERSE_STRAIGHT' and self.rs_use_live_entry:
                self.refine_center_from_live_entry(msg)
            cand = self.reproject_fixed_candidate()
            if cand is not None:
                self.publish_cones(cand.get('cones', []))
                self.publish_markers(cand.get('cones', []), cand)

    # ==================================================================
    #                       sequence control
    # ==================================================================
    def begin_sequence(self, reason):
        if not self.odom_received:
            self.pending_start_reason = reason
            self.warn('[SEQ] START deferred: waiting first %s' % self.odom_topic)
            return
        self.pending_start_reason = None
        t = self.now()
        self.done = False
        self.abort_reason = ''
        self.sequence_start_yaw = self.yaw
        self.approach_start_time = t
        self.cone_tracker.reset()
        self.lock_hist.clear()
        self.armed = False
        self.locked = False
        self.fixed_slot_meta = None
        self.fixed_goal_odom = None
        self.expected_start_yaw_delta = math.pi / 2.0
        self.plan = None
        self.setup_start_yaw = None
        self.setup_start_pose = None
        self.reverse_start_pose = None
        self.phase_start_time = t
        self.best_candidate = None
        self.live_center_corr = 0.0
        self.gap_detector.reset(self.sequence_start_yaw)
        self.parked_start_time = None
        self.exit_requested = False
        self.exit_start_time = None
        self.abort_start_time = None
        self.state = 'APPROACH'
        self.warn('[SEQ] START by %s -> APPROACH start_yaw=%.1fdeg'
                  % (reason, math.degrees(self.sequence_start_yaw)))

    def reset_all(self, reason):
        self.state = 'IDLE'
        self.pending_start_reason = None
        self.done = False
        self.locked = False
        self.plan = None
        self.cone_tracker.reset()
        self.gap_detector.reset(self.yaw)
        self.parked_start_time = None
        self.exit_requested = False
        self.exit_start_time = None
        self.abort_start_time = None
        self.publish_zero(False, False, False, False)
        self.warn('[SEQ] RESET by %s' % reason)

    def abort(self, reason):
        self.state = 'ABORT'
        self.abort_reason = reason
        self.abort_start_time = self.now()
        self.publish_zero(True, False, False, True)
        self.get_logger().error('[SEQ] ABORT: %s' % reason)

    def go_settle(self, next_state):
        self.state = 'SETTLE'
        self.next_state = next_state
        self.settle_start = self.now()
        self.publish_stop()

    # ==================================================================
    #                          transforms
    # ==================================================================
    def laser_angles_vehicle(self, scan):
        """raw 스캔각 -> 차량프레임 유효각. 뒤집힘(angle_sign=-1) 반영."""
        n = len(scan.ranges)
        raw = scan.angle_min + np.arange(n, dtype=float) * scan.angle_increment
        base = self.laser_yaw + self.laser_yaw_extra
        return base + self.laser_angle_sign * raw

    def scan_to_base_xy(self, scan):
        r = np.asarray(scan.ranges, dtype=float)
        ang = self.laser_angles_vehicle(scan)
        xb = self.laser_x + r * np.cos(ang)
        yb = self.laser_y + r * np.sin(ang)
        return r, ang, np.stack([xb, yb], axis=1)

    def valid_mask(self, r, xy):
        cp = self.cone_params
        m = np.isfinite(r) & (r >= cp.min_range) & (r <= cp.max_range)
        x, y = xy[:, 0], xy[:, 1]
        m &= (x >= self.roi_x_min) & (x <= self.roi_x_max)
        m &= (y >= self.roi_y_min) & (y <= self.roi_y_max)
        if self.side == 'left':
            m &= (y > 0.0)
        else:
            m &= (y < 0.0)
        # 자차 풋프린트 마스크 (라이다가 앞범퍼에 있어 차체를 본다)
        e = self.self_mask_extra
        half = 0.5 * self.vehicle_width + e
        self_hit = ((x >= -self.base_to_rear_bumper - e) &
                    (x <= self.base_to_front_bumper + e) &
                    (np.abs(y) <= half))
        return m & (~self_hit)

    def base_to_odom(self, p_base, pose=None):
        if pose is None:
            pose = (self.x, self.y, self.yaw)
        return transform_to_world(p_base, pose)

    def odom_to_base(self, p_odom, pose=None):
        if pose is None:
            pose = (self.x, self.y, self.yaw)
        return transform_to_local(p_odom, pose)

    def lane_axis_base(self):
        """출발 시 진행방향을 현재 base_link 에서 본 단위벡터."""
        return unit(normalize_angle(self.sequence_start_yaw - self.yaw))

    # ==================================================================
    #                    scan -> cones -> slot
    # ==================================================================
    def process_scan_approach(self, scan):
        r, ang, xy = self.scan_to_base_xy(scan)
        mask = self.valid_mask(r, xy)
        inten = None
        if self.cone_params.use_intensity and len(scan.intensities) == len(scan.ranges):
            inten = np.asarray(scan.intensities, dtype=float)

        dets = self.cone_detector.detect(r, ang, xy, mask, inten,
                                        sensor_origin=(self.laser_x, self.laser_y))

        pose = (self.x, self.y, self.yaw)
        dets_odom = [self.base_to_odom(d.center, pose=pose) for d in dets]
        det_ranges = [
            math.hypot(float(d.center[0]) - self.laser_x,
                       float(d.center[1]) - self.laser_y)
            for d in dets]
        confirmed_odom = self.cone_tracker.update(
            dets_odom, self.tsec(), det_ranges)
        confirmed_base = [self.odom_to_base(c, pose=pose) for c in confirmed_odom]
        # 트래커는 odom 프레임에서 관리하지만 슬롯 기하는 현재 차량에서
        # 실제로 관측 가능한 ROI의 콘만 사용한다. 지나가서 뒤쪽으로 벗어난
        # 콘이나 반대편 콘이 입구/내부 콘으로 재사용되는 것을 막는다.
        slot_cones_base = [
            c for c in confirmed_base
            if (self.roi_x_min <= float(c[0]) <= self.roi_x_max
                and self.roi_y_min <= float(c[1]) <= self.roi_y_max
                and ((self.side == 'left' and float(c[1]) > 0.0)
                     or (self.side == 'right' and float(c[1]) < 0.0)))
        ]

        self.csv.row('cones.csv', [self.tsec(), len(dets),
                                   len(self.cone_tracker.tracks), len(slot_cones_base)])
        self.publish_cones(slot_cones_base)

        if self.locked:
            self.publish_markers(slot_cones_base, self.best_candidate)
            return

        if self.temporal_gap_enable:
            side_distance = self.measure_side_boundary(xy, r, mask)
            event = self.gap_detector.update(side_distance, pose)
            if event == 'GAP_OPEN':
                self.warn('[GAP] OPEN 시작: boundary=%.3fm'
                          % self.gap_detector.baseline)
            elif event == 'GAP_COMPLETE':
                self.warn('[GAP] COMPLETE: 실측 입구폭=%.3fm'
                          % self.gap_detector.width)
            elif event == 'GAP_REJECT_WIDTH':
                self.warn('[GAP] 폭 범위 밖 -> 계속 탐색: measured=%.3fm '
                          'allowed=[%.3f,%.3f]'
                          % (self.gap_detector.provisional_width,
                             self.gap_detector.min_width,
                             self.gap_detector.max_width))

            cand = self.temporal_gap_candidate(slot_cones_base, pose)
            self.best_candidate = cand
            self.publish_markers(slot_cones_base, cand)
            if cand is None:
                self.lock_hist.clear()
                sd = ('n/a' if side_distance is None
                      else '%.3f' % side_distance)
                base = ('n/a' if self.gap_detector.baseline is None
                        else '%.3f' % self.gap_detector.baseline)
                self.status(
                    'APPROACH(gap): state=%s side=%sm baseline=%sm '
                    'open_width=%.3fm cones=%d/%d tracks=%d '
                    'rej[n_lo=%d n_hi=%d w=%d iso=%d cvx=%d]'
                    % (self.gap_detector.state, sd, base,
                       self.gap_detector.provisional_width,
                       len(slot_cones_base), len(dets),
                       len(self.cone_tracker.tracks),
                       self.cone_detector.last_reject.get('n_low', 0),
                       self.cone_detector.last_reject.get('n_high', 0),
                       self.cone_detector.last_reject.get('width', 0),
                       self.cone_detector.last_reject.get('isolation', 0),
                       self.cone_detector.last_reject.get('convex', 0)))
                return
            self.try_lock(cand, pose)
            return

        raw_pts = xy[mask] if bool(self.slot_params.interior_raw_reject) else None
        cand = self.slot_detector.detect(slot_cones_base, self.lane_axis_base(),
                                         self.side, raw_pts)
        self.best_candidate = cand
        self.publish_markers(slot_cones_base, cand)

        if cand is None:
            self.lock_hist.clear()
            rej = self.cone_detector.last_reject
            sd = self.slot_detector.debug
            self.status(
                'APPROACH(scan): cones=%d/%d tracks=%d '
                'cone_rej[n_lo=%d n_hi=%d w=%d iso=%d cvx=%d] '
                'slot[pairs=%d width=%d diag=%d line=%d adj=%d depth=%d '
                'center=%d lateral=%d int_cone=%d int_raw=%d front=%d '
                'veh=%d accepted=%d]'
                % (
                    len(slot_cones_base), len(dets), len(self.cone_tracker.tracks),
                    rej.get('n_low', 0), rej.get('n_high', 0),
                    rej.get('width', 0), rej.get('isolation', 0),
                    rej.get('convex', 0),
                    sd.get('pairs', 0), sd.get('rej_width', 0),
                    sd.get('rej_diag', 0), sd.get('rej_line', 0),
                    sd.get('rej_adjacent', 0), sd.get('rej_depth', 0),
                    sd.get('rej_center', 0), sd.get('rej_lateral', 0),
                    sd.get('rej_interior_cone', 0),
                    sd.get('rej_interior_raw', 0), sd.get('rej_front', 0),
                    sd.get('rej_vehicle', 0), sd.get('accepted', 0),
                ))
            return

        self.try_lock(cand, pose)

    def measure_side_boundary(self, xy, ranges, base_valid_mask=None):
        """
        라이다 바로 옆의 진행축 수직 창에서 최근접 경계 거리를 구한다.
        절대 횡거리 임계값을 쓰지 않고 gap_detector가 이 값을 실행마다
        baseline으로 학습한다.
        """
        P = np.asarray(xy, dtype=float).reshape(-1, 2)
        r = np.asarray(ranges, dtype=float)
        if base_valid_mask is None:
            base_valid = np.ones(len(r), dtype=bool)
        else:
            base_valid = np.asarray(base_valid_mask, dtype=bool)
        lane = self.lane_axis_base()
        side_sign = 1.0 if self.side == 'left' else -1.0
        side_dir = np.array(
            [-side_sign * lane[1], side_sign * lane[0]], dtype=float)
        rel = P - np.array([self.laser_x, self.laser_y], dtype=float)
        along = rel @ lane
        lateral = rel @ side_dir
        valid = (
            base_valid
            & np.isfinite(r)
            & np.isfinite(P[:, 0]) & np.isfinite(P[:, 1])
            & (r >= self.cone_params.min_range)
            & (r <= self.cone_params.max_range)
            & (np.abs(along) <= self.gap_side_window_along)
            & (lateral >= self.gap_side_min_lateral)
            & (lateral <= self.roi_y_max))
        vals = lateral[valid]
        if vals.size == 0:
            return None
        # 전이 확인을 여러 프레임 연속으로 하므로 가장 가까운 실제 경계를
        # 그대로 사용해 콘 사이를 먼 벽으로 평균내지 않는다.
        return float(np.min(vals))

    def temporal_gap_candidate(self, slot_cones_base, pose):
        """odom에 저장한 두 경계 전이를 현재 base_link 슬롯 후보로 만든다."""
        gd = self.gap_detector
        if not gd.completed or gd.start_edge_odom is None or gd.end_edge_odom is None:
            return None

        A = self.odom_to_base(gd.start_edge_odom, pose=pose)
        B = self.odom_to_base(gd.end_edge_odom, pose=pose)
        lane = self.lane_axis_base()
        if float(np.dot(B - A, lane)) < 0.0:
            A, B = B, A
        side_sign = 1.0 if self.side == 'left' else -1.0
        d = np.array([-side_sign * lane[1],
                      side_sign * lane[0]], dtype=float)
        M = 0.5 * (A + B)
        W = abs(float(np.dot(B - A, lane)))
        D = float(self.slot_params.entrance_default_depth)
        D_goal = (self.slot_params.goal_depth_override
                  if self.slot_params.goal_depth_override > 0.0 else D)
        z_goal = float(
            D_goal - self.base_to_rear_bumper - self.back_margin)
        P_entry = M - self.slot_params.entry_offset * d
        P_goal = M + z_goal * d
        P_mid = M + max(
            0.5 * z_goal, self.min_goal_inside_depth) * d
        yaw_goal_base = normalize_angle(math.atan2(d[1], d[0]) + math.pi)

        # 전이로 얻은 입구 양끝도 충돌 검증에 포함한다. 현재 추적 중인 콘과
        # 함께 쓰되 특정 콘 개수/간격 패턴을 입구 인식 조건으로 요구하지 않는다.
        cones = [np.asarray(c, dtype=float) for c in slot_cones_base]
        cones.extend([np.asarray(A, dtype=float), np.asarray(B, dtype=float)])
        return {
            'PL': np.asarray(A, dtype=float),
            'PR': np.asarray(B, dtype=float),
            'M': M, 'w': lane, 'd': d, 'lane_dir': lane,
            'entry_width': W, 'depth': D, 'z_goal': z_goal,
            'inside_count': 0, 'score': 0.0, 'heading_err': 0.0,
            'P_entry': P_entry, 'P_mid': P_mid, 'P_goal': P_goal,
            'yaw_goal_base': yaw_goal_base, 'center_origin': M,
            'cones': cones, 'temporal_gap': True,
        }

    # ------------------------------------------------------------------
    def try_lock(self, cand, pose):
        mx = float(cand['M'][0])
        my = float(cand['M'][1])
        if cand.get('temporal_gap', False) or mx >= self.approach_arm_x_min:
            self.armed = True

        m_odom = self.base_to_odom(cand['M'], pose=pose)
        self.lock_hist.append((cand['entry_width'], cand['depth'],
                               float(m_odom[0]), float(m_odom[1])))
        self.status(
            'APPROACH(candidate): W=%.3f M=(%.3f,%.3f) y_a=%.3f '
            'stable=%d/%d raw_rej=%d'
            % (cand['entry_width'], cand['M'][0], cand['M'][1],
               float(np.dot(cand['M'], cand['d'])),
               len(self.lock_hist), self.lock_stable_window,
               self.slot_detector.debug.get('rej_interior_raw', 0)))
        if len(self.lock_hist) < self.lock_stable_window:
            return

        ws = [h[0] for h in self.lock_hist]
        ds = [h[1] for h in self.lock_hist]
        mxs = [h[2] for h in self.lock_hist]
        mys = [h[3] for h in self.lock_hist]
        # odom 프레임에서의 안정성: 차가 움직여도 정지한 슬롯은 좌표가 상수여야 한다
        spread = math.hypot(max(mxs) - min(mxs), max(mys) - min(mys))
        stable_ok = ((max(ws) - min(ws) <= self.lock_width_tol) and
                     (max(ds) - min(ds) <= self.lock_depth_tol) and
                     (spread <= self.lock_modom_tol))
        lateral_ok = abs(my) >= self.approach_min_lateral_abs
        if not (self.armed and stable_ok and lateral_ok and self.odom_received):
            return

        # lock 전에 실현가능성 / 시작점 여유를 먼저 본다 (원본은 lock 후 abort)
        trial = self.build_plan(cand)
        if self.lock_require_feasible and not trial['feasible']:
            self.status('APPROACH(gate): plan infeasible -> %s' % trial['reason'])
            return
        if self.lock_require_start_ahead:
            along = float(np.dot(cand['M'], self.lane_axis_base()))
            if along <= -trial['Sx'] + self.lock_start_ahead_margin:
                self.status('APPROACH(gate): 시작점 이미 지남 along=%.3f Sx=%.3f'
                            % (along, trial['Sx']))
                return

        self.lock_slot_and_plan(cand, pose, trial)

    # ------------------------------------------------------------------
    def build_plan(self, cand):
        side_sign = 1.0 if self.expected_start_yaw_delta >= 0.0 else -1.0
        cand_yaw_goal_odom = normalize_angle(self.yaw + cand['yaw_goal_base'])
        raw_delta = normalize_angle(cand_yaw_goal_odom - self.sequence_start_yaw)
        side_sign = 1.0 if raw_delta >= 0.0 else -1.0

        fts = 1.0 if self.forward_turn_sign >= 0.0 else -1.0
        setup_steer = int(round(side_sign * fts * self.setup_steer_abs))
        reverse_steer = int(round(-side_sign * fts * self.reverse_steer_abs))
        r_setup = self.radius_for_cmd(setup_steer)
        r_rev = self.radius_for_cmd(reverse_steer)

        y_a = float(np.dot(cand['M'], cand['d']))
        z_goal = float(cand['z_goal'])
        return self.planner.plan(y_a, z_goal, r_setup, r_rev,
                                 side_sign, setup_steer, reverse_steer)

    def lock_slot_and_plan(self, cand, pose, plan):
        # 먼저 검증하고, 통과한 후보만 고정한다. 예전 코드는 첫 후보가
        # 충돌이면 locked=True로 만든 뒤 전체 시퀀스를 영구 ABORT했다.
        cones_sf = self.cones_in_slot_frame(cand)
        self.planner.simulate(plan, cones_sf)
        self.log_plan(plan)

        if not plan['feasible']:
            self.lock_hist.clear()
            self.warn('[PLAN REJECT] W=%.3f M=(%.3f,%.3f): %s'
                      % (cand['entry_width'], cand['M'][0], cand['M'][1],
                         plan['reason']))
            return
        if plan['sim_collision'] and self.abort_on_plan_collision:
            detail = self.collision_detail(plan)
            if self.continue_on_plan_collision:
                # 안전하지 않은 후보만 버리고 APPROACH를 계속한다. 실제 폭이
                # 넓은 입구가 충분히 관측되면 그 후보로 다시 안정성 검사를 한다.
                self.lock_hist.clear()
                self.warn('[PLAN REJECT] 충돌 후보만 제외하고 탐색 계속: '
                          'W=%.3f M=(%.3f,%.3f) aisle=%.2f %s'
                          % (cand['entry_width'], cand['M'][0], cand['M'][1],
                             plan['required_aisle_width'], detail))
                return
            self.abort('계획 궤적이 콘과 충돌 (sim). required_aisle=%.2fm %s'
                       % (plan['required_aisle_width'], detail))
            return

        self.fixed_slot_meta = self.candidate_to_meta(cand, pose=pose)
        self.fixed_goal_odom = self.base_to_odom(cand['P_goal'], pose=pose)
        cand_yaw_goal_odom = normalize_angle(self.yaw + cand['yaw_goal_base'])
        raw_delta = normalize_angle(cand_yaw_goal_odom - self.sequence_start_yaw)
        self.expected_start_yaw_delta = (
            (math.pi / 2.0) if raw_delta >= 0.0 else (-math.pi / 2.0))
        self.fixed_yaw_goal_odom = normalize_angle(
            self.sequence_start_yaw + self.expected_start_yaw_delta)
        self.plan = plan
        self.locked = True
        self.best_candidate = cand
        self.publish_goal(self.fixed_goal_odom, self.fixed_yaw_goal_odom)

        mc = plan['sim_min_clearance']
        if mc is not None and mc < self.plan_min_clearance:
            self.warn('[PLAN] 여유 %.3f m 가 작습니다 (기준 %.3f). 저속 유지 권장.'
                      % (mc, self.plan_min_clearance))
        self.warn('[SEQ] SLOT LOCKED -> 시작점으로 Sx=%.3f (z_arc=%.3f z_straight=%.3f)'
                  % (plan['Sx'], plan['z_arc'], plan['z_straight']))

    @staticmethod
    def collision_detail(plan):
        """계획 검증의 첫 접촉점을 사람이 바로 확인할 수 있는 한 줄로."""
        if not plan.get('sim_collision'):
            return 'contact=n/a'
        return ('contact[%s cone=(u%.2f,v%.2f) '
                'car=(u%.2f,v%.2f,phi%.1fdeg) local=(x%.2f,y%.2f)]'
                % (plan.get('sim_collision_phase', '?'),
                   plan.get('sim_collision_cone_u', float('nan')),
                   plan.get('sim_collision_cone_v', float('nan')),
                   plan.get('sim_collision_pose_u', float('nan')),
                   plan.get('sim_collision_pose_v', float('nan')),
                   plan.get('sim_collision_pose_phi_deg', float('nan')),
                   plan.get('sim_collision_cone_local_x', float('nan')),
                   plan.get('sim_collision_cone_local_y', float('nan'))))

    def cones_in_slot_frame(self, cand):
        """콘들을 (u=lane, v=슬롯안쪽) 슬롯 프레임 좌표로."""
        cones = cand.get('cones', [])
        if not len(cones):
            return None
        M = cand['M']
        u_dir = cand.get('lane_dir', self.lane_axis_base())
        v_dir = cand['d']
        out = []
        for c in cones:
            rel = np.asarray(c, dtype=float) - M
            out.append([float(np.dot(rel, u_dir)), float(np.dot(rel, v_dir))])
        return np.asarray(out, dtype=float)

    def log_plan(self, plan):
        self.csv.row('plan.csv', [
            self.tsec(), plan['y_a'], plan['z_goal'], plan['z_arc'],
            plan['z_straight'], plan['R_setup'], plan['R_rev'],
            math.degrees(plan['theta1']), math.degrees(plan['dtheta2']),
            plan['Sx'], plan['setup_steer'], plan['reverse_steer'],
            plan['required_aisle_width'], plan['sim_min_clearance'],
            int(bool(plan['sim_collision'])), int(plan['feasible'])])
        self.warn('[PLAN] y_a=%.3f z_goal=%.3f -> z_arc=%.3f + z_straight=%.3f | '
                  'R_setup=%.3f R_rev=%.3f theta1=%.1f dtheta2=%.1f Sx=%.3f | '
                  'aisle_need=%.2f sim_clear=%s collide=%s feasible=%s'
                  % (plan['y_a'], plan['z_goal'], plan['z_arc'], plan['z_straight'],
                     plan['R_setup'], plan['R_rev'], math.degrees(plan['theta1']),
                     math.degrees(plan['dtheta2']), plan['Sx'],
                     plan['required_aisle_width'],
                     ('%.3f' % plan['sim_min_clearance'])
                     if plan['sim_min_clearance'] is not None else 'n/a',
                     plan['sim_collision'], plan['feasible']))
        if plan['sim_collision']:
            self.warn('[PLAN COLLISION] %s' % self.collision_detail(plan))

    # ------------------------------------------------------------------
    def candidate_to_meta(self, cand, pose=None):
        if pose is None:
            pose = (self.x, self.y, self.yaw)
        cones_odom = [self.base_to_odom(c, pose=pose) for c in cand.get('cones', [])]
        return {
            'M_odom': self.base_to_odom(cand['M'], pose=pose),
            'w_yaw0': float(math.atan2(cand['w'][1], cand['w'][0]) + pose[2]),
            'd_yaw0': float(math.atan2(cand['d'][1], cand['d'][0]) + pose[2]),
            'W': float(cand['entry_width']), 'D': float(cand['depth']),
            'z_goal': float(cand['z_goal']), 'cones_odom': cones_odom,
        }

    def reproject_fixed_candidate(self):
        """odom 에 고정한 슬롯을 현재 base_link 로 재투영. 블라인드 구간 제어 기준."""
        if self.fixed_slot_meta is None or self.fixed_goal_odom is None:
            return None
        meta = self.fixed_slot_meta
        M = self.odom_to_base(meta['M_odom'])
        w = unit(normalize_angle(meta['w_yaw0'] - self.yaw))
        d = unit(normalize_angle(meta['d_yaw0'] - self.yaw))
        z_goal = meta['z_goal']
        origin = M
        return {
            'PL': M - 0.5 * meta['W'] * w, 'PR': M + 0.5 * meta['W'] * w,
            'M': M, 'w': w, 'd': d, 'lane_dir': w,
            'entry_width': meta['W'], 'depth': meta['D'], 'z_goal': z_goal,
            'center_origin': origin,
            'P_entry': origin - self.slot_params.entry_offset * d,
            'P_mid': origin + max(0.5 * z_goal, self.min_goal_inside_depth) * d,
            'P_goal': self.odom_to_base(self.fixed_goal_odom),
            'yaw_goal_base': normalize_angle(self.fixed_yaw_goal_odom - self.yaw),
            'cones': [self.odom_to_base(c) for c in meta.get('cones_odom', [])],
        }

    # ------------------------------------------------------------------
    def refine_center_from_live_entry(self, scan):
        """
        REVERSE_STRAIGHT 에서 차량은 슬롯과 평행하고 라이다는 슬롯 밖(입구 방향)을 본다.
        입구 콘 2개가 다시 보이면 그 중점으로 중앙선 오차를 직접 보정할 수 있다.
        (선택 기능. reverse_straight_use_live_entry=true 일 때만 동작)
        """
        cand = self.reproject_fixed_candidate()
        if cand is None:
            return
        r, ang, xy = self.scan_to_base_xy(scan)
        cp = self.cone_params
        m = np.isfinite(r) & (r >= cp.min_range) & (r <= cp.max_range)
        x, y = xy[:, 0], xy[:, 1]
        e = self.self_mask_extra
        half = 0.5 * self.vehicle_width + e
        m &= ~((x >= -self.base_to_rear_bumper - e) &
               (x <= self.base_to_front_bumper + e) & (np.abs(y) <= half))
        # 전방 밴드만 사용
        m &= (x > self.base_to_front_bumper) & (x < self.base_to_front_bumper + 4.0)
        dets = self.cone_detector.detect(r, ang, xy, m, None,
                                        sensor_origin=(self.laser_x, self.laser_y))
        if len(dets) < 2:
            return
        W = cand['entry_width']
        best = None
        for i in range(len(dets)):
            for j in range(i + 1, len(dets)):
                a, b = dets[i].center, dets[j].center
                if abs(float(np.linalg.norm(b - a)) - W) > 0.30:
                    continue
                mid = 0.5 * (a + b)
                if best is None or abs(mid[1]) < abs(best[1]):
                    best = mid
        if best is None:
            return
        self.live_center_corr = ((1.0 - self.rs_live_weight) * self.live_center_corr
                                + self.rs_live_weight * float(best[1]))

    # ==================================================================
    #                       vehicle geometry
    # ==================================================================
    def steer_angle_for_cmd(self, steer_cmd):
        """cmd -> 조향각 크기 [rad]. cmd<0 = 좌, cmd>0 = 우 (원본 규약 유지)."""
        cmd = float(steer_cmd)
        if abs(cmd) < 1e-6 or self.max_steer_cmd <= 0:
            return 0.0
        frac = clamp(abs(cmd) / float(self.max_steer_cmd), 0.0, 1.0)
        dmax = self.max_steer_angle_left_deg if cmd < 0.0 else self.max_steer_angle_right_deg
        return math.radians(dmax) * frac

    def radius_for_cmd(self, steer_cmd):
        """뒤 차축 기준 회전반경."""
        cmd = int(round(steer_cmd))
        if cmd == 0:
            return float('inf')
        if self.radius_mode == 'table':
            table = self.right_radius_table if cmd > 0 else self.left_radius_table
            if table:
                r = interp_table(table, abs(cmd))
                if self.radius_is_center:
                    half = 0.5 * self.wheel_base
                    r = math.sqrt(max(r * r - half * half, 1e-6))
                return r
            self.warn('[GEOM] radius_mode=table 인데 표가 비었습니다 -> bicycle 사용')
        d = self.steer_angle_for_cmd(cmd)
        t = abs(math.tan(d))
        if t < 1e-9:
            return float('inf')
        return self.wheel_base / t

    def rear_clearance(self, cand, pose=(0.0, 0.0, 0.0)):
        half_w = 0.5 * self.vehicle_width
        corners = [np.array([-self.base_to_rear_bumper, -half_w]),
                   np.array([-self.base_to_rear_bumper, half_w])]
        pts = [transform_to_world(c, pose) for c in corners]
        s_vals = [float(np.dot(p - cand['center_origin'], cand['d'])) for p in pts]
        clear = float(cand['depth']) - max(s_vals)
        return clear, clear - self.back_margin

    def entrance_clearance(self, cand, pose=(0.0, 0.0, 0.0)):
        """Rear-bumper corners' position along the slot depth axis (d),
        relative to the entrance midpoint (M/center_origin) - same corner
        math as rear_clearance(), just measured from the entrance instead
        of the far wall. Positive = still inside the slot (past the
        entrance, deeper in); <= 0 once the rear bumper has cleared the
        entrance line. Used to judge EXIT_STRAIGHT completion - the
        vehicle's own locked (odom-fixed) slot geometry is already exact,
        so this needs no fresh LiDAR read (the slot isn't even in view
        once driving forward out of it)."""
        half_w = 0.5 * self.vehicle_width
        corners = [np.array([-self.base_to_rear_bumper, -half_w]),
                   np.array([-self.base_to_rear_bumper, half_w])]
        pts = [transform_to_world(c, pose) for c in corners]
        s_vals = [float(np.dot(p - cand['center_origin'], cand['d'])) for p in pts]
        return max(s_vals)

    def slot_errors(self, cand, pose):
        p = np.array([pose[0], pose[1]], dtype=float)
        q_err = float(np.dot(p - cand['center_origin'], cand['w']))
        s_val = float(np.dot(p - cand['center_origin'], cand['d']))
        yaw_err = normalize_angle(cand['yaw_goal_base'] - pose[2])
        return q_err, s_val, yaw_err

    # ==================================================================
    #                          commands
    # ==================================================================
    def approach_cmd(self):
        yaw_err = normalize_angle(self.sequence_start_yaw - self.yaw)
        wz_err = 0.0 - self.wz
        control_sign = self.approach_steer_sign * self.forward_turn_sign
        raw = control_sign * (
            self.approach_yaw_kp * yaw_err + self.approach_wz_kd * wz_err)

        # 실차 조향의 중립 오차/데드존 때문에 steer=+/-1 정도는 차체
        # 편향을 이기지 못할 수 있다. 시작 헤딩 오차가 데드밴드를 넘으면
        # 비례항이 요구하는 복귀 방향으로 최소 유효 조향을 보장한다.
        if (self.approach_min_steer > 0
                and abs(yaw_err) > self.approach_yaw_deadband
                and abs(raw) < self.approach_min_steer):
            return_direction = control_sign * yaw_err
            if abs(return_direction) > 1e-9:
                raw = math.copysign(float(self.approach_min_steer),
                                    return_direction)
        steer = int(round(clamp(raw, -self.approach_max_steer, self.approach_max_steer)))
        return int(self.pre_straight_rpm), steer

    def exit_cmd(self):
        """Same start-heading-hold controller as approach_cmd() (exiting a
        T-slot forward re-enters the same lane heading the vehicle backed
        in from - sequence_start_yaw is exactly that), just at
        exit_forward_rpm instead of pre_straight_rpm - keep this slower/
        more cautious since it's driving out towards a lane that may have
        moved since APPROACH."""
        rpm, steer = self.approach_cmd()
        return int(self.exit_forward_rpm), steer

    def publish_cmd(self, rpm, steer, enable=1):
        steer = int(round(clamp(steer, -self.max_steer_cmd, self.max_steer_cmd)))
        self.pub_cmd_rpm.publish(Int16(data=int(rpm)))
        self.pub_cmd_steer.publish(Int16(data=steer))
        self.pub_cmd_enable.publish(Int16(data=int(enable)))
        if self.direct_cmd_output:
            self.pub_direct_rpm.publish(Int16(data=int(rpm)))
            self.pub_direct_steer.publish(Int16(data=steer))
            self.pub_direct_enable.publish(Int16(data=int(enable)))

    def publish_stop(self):
        self.publish_cmd(0, 0, 1)

    def publish_zero(self, active=False, mapping=False, done=False, stop_req=False):
        self.publish_cmd(0, 0, 0 if done else 1)
        self.pub_active.publish(Bool(data=bool(active)))
        self.pub_mapping.publish(Bool(data=bool(mapping)))
        self.pub_done.publish(Bool(data=bool(done)))
        self.pub_stop_req.publish(Bool(data=bool(stop_req)))

    def done_check(self, cand):
        q_err, _s, yaw_err = self.slot_errors(cand, (0.0, 0.0, 0.0))
        clear, rear_err = self.rear_clearance(cand)
        start90 = normalize_angle(
            normalize_angle(self.yaw - self.sequence_start_yaw)
            - self.expected_start_yaw_delta)
        ok = (rear_err <= self.rear_goal_tolerance
              and clear >= self.rear_safety_stop_margin
              and abs(q_err) <= self.centerline_tolerance
              and abs(math.degrees(yaw_err)) <= self.final_yaw_tolerance_deg
              and abs(self.wz) <= self.final_wz_tolerance
              and abs(math.degrees(start90)) <= self.final_start_yaw_tolerance_deg)
        return ok, {'center_err': q_err, 'rear_clearance': clear,
                    'rear_err': rear_err, 'yaw_err': yaw_err, 'start90': start90}

    def log_control(self, rpm, steer, cand):
        along = lateral = center = clear = 0.0
        if cand is not None:
            along = float(np.dot(cand['M'], self.lane_axis_base()))
            lateral = float(np.dot(cand['M'], cand['d']))
            center, _, _ = self.slot_errors(cand, (0.0, 0.0, 0.0))
            clear, _ = self.rear_clearance(cand)
        self.csv.row('control.csv', [
            self.tsec(), self.state, self.x, self.y, math.degrees(self.yaw),
            math.degrees(normalize_angle(self.yaw - self.sequence_start_yaw)),
            along, lateral, center, clear, rpm, steer])

    # ==================================================================
    #                          main loop
    # ==================================================================
    def on_timer(self):
        t = self.now()
        st = self.state

        if st == 'IDLE':
            self.publish_zero(False, False, False, False)
            self.status('IDLE: waiting /parking_start')

        elif st == 'APPROACH':
            self.pub_stop_req.publish(Bool(data=False))
            # mapping means "still searching, don't trust this node's own
            # approach controller yet" (see control_arbiter's
            # _handle_parking_zone docstring) - that's only true before
            # self.locked. This used to be a hardcoded True for the whole
            # APPROACH state regardless of lock status (2026-08-07 bug,
            # same family as SETUP_ARC/REVERSE_ARC/REVERSE_STRAIGHT's fix
            # above): once locked, this state still drives itself toward
            # the arc's start point via approach_cmd() below, then calls
            # publish_stop()+go_settle('SETUP_ARC') - but with mapping
            # stuck True, control_arbiter kept driving via GPS straight
            # through all of that (ignoring this node's own stop command
            # entirely), so SETTLE's own vx<threshold condition could never
            # be satisfied and the vehicle never actually stopped or
            # reached SETUP_ARC ("SETUP_ARC 뜰 때 멈춰야 하는거 아님?" -
            # confirmed 2026-08-07: it never even got there). active must
            # flip True at the same time - the node needs actual control,
            # not just mapping=False with nobody driving.
            self.pub_mapping.publish(Bool(data=not self.locked))
            self.pub_active.publish(Bool(data=self.locked))
            if (self.approach_timeout_sec > 0.0 and self.approach_start_time is not None
                    and self.dt(t, self.approach_start_time) >= self.approach_timeout_sec
                    and not self.locked):
                self.abort('APPROACH timeout: slot not locked')
                return
            if self.locked and self.plan is not None and self.plan['feasible']:
                cand = self.reproject_fixed_candidate()
                along = float(np.dot(cand['M'], self.lane_axis_base()))
                if along <= -self.plan['Sx']:
                    self.publish_stop()
                    self.log_control(0, 0, cand)
                    self.warn('[SEQ] APPROACH reached start point along=%.3f (<= -Sx=%.3f)'
                              % (along, -self.plan['Sx']))
                    self.go_settle('SETUP_ARC')
                else:
                    rpm, steer = self.approach_cmd()
                    self.publish_cmd(rpm, steer, 1)
                    self.log_control(rpm, steer, cand)
                    self.status('APPROACH(to_start): along=%.3f target=%.3f steer=%d'
                                % (along, -self.plan['Sx'], steer))
            else:
                rpm, steer = self.approach_cmd()
                self.publish_cmd(rpm, steer, 1)

        elif st == 'SETTLE':
            self.pub_stop_req.publish(Bool(data=True))
            self.publish_cmd(0, 0, 1)
            if self.settle_start is None:
                self.settle_start = t
            settle_elapsed = self.dt(t, self.settle_start)
            vx_settled = abs(self.vx) < self.stop_speed_thresh
            settle_timed_out = settle_elapsed >= self.settle_timeout_sec
            if settle_elapsed >= self.stop_hold_sec and (vx_settled or settle_timed_out):
                if settle_timed_out and not vx_settled:
                    self.warn(
                        '[SETTLE] timeout %.1fs forcing advance to %s '
                        'despite vx=%.3f (slope creep?)'
                        % (settle_elapsed, self.next_state, self.vx))
                self.settle_start = None
                self.phase_start_time = t
                if self.next_state == 'SETUP_ARC':
                    if self.recompute_at_start:
                        self.recompute_arc_at_start()
                    self.setup_start_yaw = self.yaw
                    self.setup_start_pose = (self.x, self.y, self.yaw)
                elif self.next_state == 'REVERSE_ARC':
                    self.reverse_start_pose = (self.x, self.y, self.yaw)
                self.state = self.next_state
                self.warn('[SEQ] SETTLE -> %s' % self.next_state)
            else:
                self.status(
                    'SETTLE: vx=%.3f timeout=%.1f/%.1fs -> %s'
                    % (self.vx, settle_elapsed, self.settle_timeout_sec,
                       self.next_state))

        elif st == 'SETUP_ARC':
            self.pub_stop_req.publish(Bool(data=False))
            # Slot is already locked by this point (SETTLE only transitions
            # here once self.locked/plan['feasible']) - actively maneuvering
            # now, not searching. mapping=True here was a real bug
            # (2026-08-07): control_arbiter's _handle_parking_zone drives
            # via plain GPS the entire time mapping reads True, regardless
            # of state - with this still True through the whole SETUP_ARC/
            # REVERSE_ARC/REVERSE_STRAIGHT maneuver, the arbiter never once
            # relayed this node's own cmd_rpm/cmd_steer to CAN, so the
            # vehicle just kept driving straight via GPS at approach_rpm
            # the entire time even though this node's own state/logged
            # steer angle were progressing correctly ("reverse arc 뜨고
            # 각도 떠도 걍 rpm 30으로 앞으로만 가는" - confirmed on the real
            # vehicle).
            self.pub_mapping.publish(Bool(data=False))
            # THE bug (2026-08-07, found by direct topic capture -
            # "parking_active가... 멈추는 순간 다시 false되고 안돌아와"):
            # this was hardcoded False here even though SETUP_ARC is
            # actively maneuvering (computing/publishing real cmd_rpm/
            # cmd_steer below) - not something the mapping fix above
            # touched, a separate line. With active=False,
            # control_arbiter's _handle_parking_zone always fell to its
            # fail-safe branch (mapping already False too, so not that
            # path either) and sent a flat stop instead of relaying this
            # state's real commands - matches exactly what was observed:
            # /parking_t/cmd_rpm=30, cmd_steer=30, cmd_enable=1 all
            # correct and live, but /parking_t/parking_active=false the
            # whole time, so none of it ever reached CAN.
            self.pub_active.publish(Bool(data=True))
            plan = self.plan
            turned = normalize_angle(self.yaw - self.setup_start_yaw) * plan['side_sign']
            moved = math.hypot(self.x - self.setup_start_pose[0],
                               self.y - self.setup_start_pose[1])
            elapsed = self.dt(t, self.phase_start_time)
            reached = turned >= (plan['theta1'] - math.radians(self.setup_yaw_tolerance_deg))
            if reached or moved >= self.setup_max_forward_dist \
                    or elapsed >= self.arc_absolute_timeout_sec:
                self.publish_stop()
                self.warn('[SEQ] SETUP_ARC done turned=%.1f/%.1fdeg moved=%.3f'
                          % (math.degrees(turned), math.degrees(plan['theta1']), moved))
                self.go_settle('REVERSE_ARC')
            else:
                self.publish_cmd(self.setup_forward_rpm, plan['setup_steer'], 1)
                self.log_control(self.setup_forward_rpm, plan['setup_steer'],
                                 self.reproject_fixed_candidate())
                self.status('SETUP_ARC: steer=%d turned=%.1f/%.1fdeg moved=%.3f'
                            % (plan['setup_steer'], math.degrees(turned),
                               math.degrees(plan['theta1']), moved))

        elif st == 'REVERSE_ARC':
            self.pub_stop_req.publish(Bool(data=False))
            self.pub_mapping.publish(Bool(data=False))  # see SETUP_ARC's comment
            self.pub_active.publish(Bool(data=True))
            plan = self.plan
            cand = self.reproject_fixed_candidate()
            total = normalize_angle(self.yaw - self.sequence_start_yaw) * plan['side_sign']
            clear, _ = self.rear_clearance(cand)
            elapsed = self.dt(t, self.phase_start_time)
            moved = math.hypot(self.x - self.reverse_start_pose[0],
                               self.y - self.reverse_start_pose[1]) \
                if self.reverse_start_pose else 0.0
            reached = total >= (0.5 * math.pi - math.radians(self.reverse_yaw_tolerance_deg))
            if clear <= self.rear_safety_stop_margin:
                self.publish_stop()
                self.warn('[SEQ] REVERSE_ARC rear safety stop clear=%.3f' % clear)
                self.go_settle('REVERSE_STRAIGHT')
            elif reached or elapsed >= self.arc_absolute_timeout_sec \
                    or moved >= self.reverse_max_dist:
                self.publish_stop()
                self.warn('[SEQ] REVERSE_ARC done total=%.1f/90deg clear=%.3f moved=%.3f'
                          % (math.degrees(total), clear, moved))
                self.go_settle('REVERSE_STRAIGHT')
            else:
                self.publish_cmd(self.reverse_rpm, plan['reverse_steer'], 1)
                self.log_control(self.reverse_rpm, plan['reverse_steer'], cand)
                self.status('REVERSE_ARC: steer=%d total=%.1f/90deg clear=%.3f'
                            % (plan['reverse_steer'], math.degrees(total), clear))

        elif st == 'REVERSE_STRAIGHT':
            self.pub_stop_req.publish(Bool(data=False))
            self.pub_mapping.publish(Bool(data=False))  # see SETUP_ARC's comment
            self.pub_active.publish(Bool(data=True))
            cand = self.reproject_fixed_candidate()
            done_ok, info = self.done_check(cand)
            if done_ok:
                # Backed all the way in - but do NOT hand control back to
                # camera/GPS yet (done stays False here on purpose, see
                # PARKED below): the vehicle is sitting nose-out inside the
                # slot, off the recorded line, and needs to drive straight
                # back out first (2026-08-05 exit logic).
                self.publish_zero(False, False, False, True)
                self.state = 'PARKED'
                self.parked_start_time = self.now()
                self.warn('[SEQ] REVERSE_STRAIGHT -> PARKED rear=%.3f center=%.3f '
                          'yawerr=%.1f start90=%.1f'
                          % (info['rear_clearance'], info['center_err'],
                             math.degrees(info['yaw_err']), math.degrees(info['start90'])))
                return
            if info['rear_clearance'] <= self.rear_safety_stop_margin:
                self.publish_stop()
                self.abort('rear clearance safety stop clear=%.3f center=%.3f yawerr=%.1f'
                           % (info['rear_clearance'], info['center_err'],
                              math.degrees(info['yaw_err'])))
                return
            center_err = info['center_err']
            if self.rs_use_live_entry and abs(self.live_center_corr) > 1e-6:
                center_err = ((1.0 - self.rs_live_weight) * center_err
                              + self.rs_live_weight * self.live_center_corr)
            raw = (self.reverse_align_yaw_kp * info['yaw_err']
                   - self.reverse_align_center_kp * center_err)
            steer = int(round(clamp(raw, -self.reverse_align_max_steer,
                                    self.reverse_align_max_steer)))
            if abs(steer) <= self.steer_deadband_cmd:
                steer = 0
            rpm = int(self.final_rpm)
            self.publish_cmd(rpm, steer, 1)
            self.log_control(rpm, steer, cand)
            self.status('REVERSE_STRAIGHT: steer=%d rear=%.3f rear_err=%.3f '
                        'center=%.3f yawerr=%.1f'
                        % (steer, info['rear_clearance'], info['rear_err'],
                           center_err, math.degrees(info['yaw_err'])))

        elif st == 'PARKED':
            # Parked and stopped, waiting for the exit trigger - mirrors
            # parallel_parking's PARKED->exit pattern. done stays False so
            # control_arbiter keeps relaying (and fails safe if this node
            # stops publishing) instead of handing back to camera/GPS with
            # the vehicle still nose-in on a slot off the recorded line.
            self.publish_zero(False, False, False, True)
            if self.parked_start_time is None:
                self.parked_start_time = t
            hold_elapsed = self.dt(t, self.parked_start_time) >= self.parking_hold_sec
            if self.exit_requested or (self.auto_exit and hold_elapsed):
                self.exit_requested = False
                self.exit_start_time = t
                self.go_settle('EXIT_STRAIGHT')
                self.warn('[SEQ] PARKED -> EXIT_STRAIGHT (auto=%s)'
                          % (self.auto_exit and hold_elapsed and not self.exit_requested))
                return
            self.status('PARKED: waiting /parking_exit_start (auto_exit=%s hold=%.1f/%.1fs)'
                        % (self.auto_exit, self.dt(t, self.parked_start_time),
                           self.parking_hold_sec))

        elif st == 'EXIT_STRAIGHT':
            self.pub_stop_req.publish(Bool(data=False))
            self.pub_mapping.publish(Bool(data=False))
            self.pub_active.publish(Bool(data=True))
            if (self.exit_timeout_sec > 0.0 and self.exit_start_time is not None
                    and self.dt(t, self.exit_start_time) >= self.exit_timeout_sec):
                self.publish_stop()
                self.abort('EXIT_STRAIGHT timeout: never cleared entrance')
                return
            cand = self.reproject_fixed_candidate()
            if cand is None:
                # Locked slot geometry gone (shouldn't happen without a
                # reset) - can't judge exit progress, fail safe.
                self.publish_stop()
                self.abort('EXIT_STRAIGHT: lost fixed slot geometry')
                return
            clearance = self.entrance_clearance(cand)
            if clearance <= -self.exit_clear_margin:
                self.publish_zero(False, False, True, True)
                self.pub_exit_done.publish(Bool(data=True))
                self.state = 'DONE'
                self.done = True
                self.warn('[SEQ] EXIT_STRAIGHT -> DONE entrance_clearance=%.3f'
                          % clearance)
                return
            rpm, steer = self.exit_cmd()
            self.publish_cmd(rpm, steer, 1)
            self.status('EXIT_STRAIGHT: steer=%d rpm=%d entrance_clearance=%.3f (need<=%.3f)'
                        % (steer, rpm, clearance, -self.exit_clear_margin))

        elif st == 'DONE':
            self.publish_zero(False, False, True, True)
            self.status('DONE: parking complete log_dir=%s' % self.log_dir)

        elif st == 'ABORT':
            if not self.auto_recover_from_abort or self.abort_start_time is None:
                self.publish_zero(True, False, False, True)
                self.status('ABORT: %s' % self.abort_reason)
                return
            elapsed = self.dt(t, self.abort_start_time)
            if elapsed < self.abort_hold_sec:
                self.publish_zero(True, False, False, True)
                self.status('ABORT: %s (auto-recover in %.1fs)'
                            % (self.abort_reason, self.abort_hold_sec - elapsed))
                return
            if elapsed < self.abort_hold_sec + self.abort_forward_sec:
                # Same direction/rationale as EXIT_STRAIGHT (T자 already
                # nose-out toward the lane after backing in) - just time-
                # based instead of entrance_clearance-gated, since ABORT
                # can fire before a full lock/depth picture is trustworthy
                # enough to re-check geometrically.
                self.pub_mapping.publish(Bool(data=False))
                self.pub_active.publish(Bool(data=True))
                rpm, steer = self.exit_cmd()
                self.publish_cmd(rpm, steer, 1)
                self.status('ABORT: %s (auto-recover driving forward %.1f/%.1fs)'
                            % (self.abort_reason, elapsed - self.abort_hold_sec,
                               self.abort_forward_sec))
                return
            self.publish_zero(False, False, True, True)
            self.state = 'DONE'
            self.done = True
            self.warn('[SEQ] ABORT -> DONE (auto-recover forward-drive complete)')

    # ------------------------------------------------------------------
    def recompute_arc_at_start(self):
        """
        시작점(정지)에서 실제 횡거리로 theta1 / z_arc 를 재계산한다.
        블라인드이므로 슬롯 재관측은 하지 않고 odom 고정 슬롯을 현재 pose 로 재투영.
        """
        if self.plan is None:
            return
        cand = self.reproject_fixed_candidate()
        if cand is None:
            return
        y_a = float(np.dot(cand['M'], cand['d']))
        ok, old = self.planner.recompute_theta1(self.plan, y_a)
        if not ok:
            self.warn('[PLAN] recompute@start infeasible y_a=%.3f -> lock 시점 계획 유지' % y_a)
            return
        self.warn('[PLAN] recompute@start: y_a=%.3f theta1 %.1f->%.1fdeg '
                  'z_arc=%.3f z_straight=%.3f'
                  % (y_a, math.degrees(old), math.degrees(self.plan['theta1']),
                     self.plan['z_arc'], self.plan['z_straight']))
        self.log_plan(self.plan)

    # ==================================================================
    #                             viz
    # ==================================================================
    def publish_cones(self, cones):
        pa = PoseArray()
        pa.header.stamp = self.now().to_msg()
        pa.header.frame_id = self.base_frame
        for c in cones:
            p = Pose()
            p.position.x, p.position.y, p.position.z = float(c[0]), float(c[1]), 0.10
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_cones.publish(pa)

    def _marker(self, ns, mid, mtype):
        m = Marker()
        m.header.stamp = self.now().to_msg()
        m.header.frame_id = self.base_frame
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime.sec = 0
        m.lifetime.nanosec = 400000000
        return m

    def _sphere(self, ma, mid, p, rgb, scale, ns):
        m = self._marker(ns, mid, Marker.SPHERE)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            float(p[0]), float(p[1]), 0.12
        m.scale.x = m.scale.y = m.scale.z = float(scale)
        m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 0.92
        ma.markers.append(m)

    def _line(self, ma, mid, pts, rgb, width, ns):
        m = self._marker(ns, mid, Marker.LINE_STRIP)
        m.scale.x = float(width)
        m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 0.95
        for p in pts:
            pt = Point()
            pt.x, pt.y, pt.z = float(p[0]), float(p[1]), 0.05
            m.points.append(pt)
        ma.markers.append(m)

    def publish_markers(self, cones, cand):
        ma = MarkerArray()
        mid = 0
        roi = [[self.roi_x_min, self.roi_y_min], [self.roi_x_max, self.roi_y_min],
               [self.roi_x_max, self.roi_y_max], [self.roi_x_min, self.roi_y_max],
               [self.roi_x_min, self.roi_y_min]]
        self._line(ma, mid, roi, (0.4, 0.4, 1.0), 0.02, 'roi'); mid += 1
        for c in cones:
            self._sphere(ma, mid, c, (1.0, 0.45, 0.0), 0.20, 'cones'); mid += 1
        if cand is not None:
            self._line(ma, mid, [cand['PL'], cand['PR']], (1.0, 0.0, 0.0), 0.05, 'entry'); mid += 1
            self._line(ma, mid, [cand['P_entry'], cand['P_mid'], cand['P_goal']],
                       (0.0, 1.0, 0.0), 0.04, 'goal_line'); mid += 1
            self._sphere(ma, mid, cand['P_goal'], (0.0, 1.0, 0.0), 0.25, 'goal'); mid += 1
        self.pub_markers.publish(ma)

    def publish_goal(self, p_odom, yaw):
        ps = PoseStamped()
        ps.header.stamp = self.now().to_msg()
        ps.header.frame_id = self.odom_frame
        ps.pose.position.x, ps.pose.position.y = float(p_odom[0]), float(p_odom[1])
        q = quat_from_yaw(yaw)
        ps.pose.orientation.x, ps.pose.orientation.y = q[0], q[1]
        ps.pose.orientation.z, ps.pose.orientation.w = q[2], q[3]
        self.pub_goal.publish(ps)

    # ------------------------------------------------------------------
    def on_shutdown(self):
        try:
            self.publish_cmd(0, 0, 0)
            self.pub_active.publish(Bool(data=False))
            self.pub_mapping.publish(Bool(data=False))
            self.pub_stop_req.publish(Bool(data=True))
        except Exception:
            pass
        self.csv.close()


def main(args=None):
    rclpy.init(args=args)
    node = RuleBasedTParkingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

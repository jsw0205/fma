#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rule_based_parallel_parking_node.py

Compact rule-based parallel parking node.

Flow:
  IDLE
    -> APPROACH         scan cones, estimate a parallel slot, lock it in odom
    -> SETTLE           stop before each motion phase
    -> REVERSE_IN_ARC   calculated first steering arc into the slot
    -> REVERSE_STRAIGHT calculated straight reverse between arcs
    -> REVERSE_OUT_ARC  calculated opposite steering arc to align with slot axis
    -> FINAL_ALIGN      direction-aware longitudinal centering only
    -> PARKED           hold inside the slot; wait for exit request
    -> EXIT_MANEUVER    execute a swept-path-validated lane exit
    -> EXIT_COMPLETE    verify lane clearance and start heading
    -> DONE             release control to the waypoint controller

Like the successful T-parking node, the slot is detected once and then held in
odom. LiDAR is not used to keep changing the plan during the reverse phases.
"""

import csv
import math
import os
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Int16, String
from visualization_msgs.msg import Marker, MarkerArray

from t_parking.temporal_gap_detector import TemporalSideGapDetector


VEHICLE_MAX_STEER_CMD = 30


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def unit_or_zero(v):
    n = float(np.linalg.norm(v))
    return np.asarray(v, dtype=float) / n if n > 1e-9 else np.zeros_like(v, dtype=float)


def robust_percentile(values, percentile, default=0.0):
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    return float(np.percentile(arr, percentile)) if arr.size else float(default)


def latched_qos(depth=1):
    return QoSProfile(depth=depth,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=QoSReliabilityPolicy.RELIABLE)


class CsvLogger:
    def __init__(self, enabled, base_dir):
        self.enabled = bool(enabled)
        self.base_dir = base_dir
        self.files = {}
        self.writers = {}
        if self.enabled:
            os.makedirs(base_dir, exist_ok=True)

    def open_writer(self, name, header):
        if not self.enabled:
            return
        handle = open(os.path.join(self.base_dir, name), 'w', newline='')
        writer = csv.writer(handle)
        writer.writerow(header)
        self.files[name] = handle
        self.writers[name] = writer

    def row(self, name, values):
        if self.enabled and name in self.writers:
            self.writers[name].writerow(values)

    def close(self):
        for handle in self.files.values():
            handle.flush()
            handle.close()
        self.files.clear()
        self.writers.clear()



class RuleBasedParallelParkingNode(Node):
    def __init__(self):
        super().__init__('rule_based_parallel_parking_node')
        self._throttle = {}

        # Topics / frames
        self.scan_topic = self.p('scan_topic', '/scan_parking')
        self.odom_topic = self.p('odom_topic', '/wheel_odom')
        self.imu_topic = self.p('imu_topic', '/imu/data')
        self.odom_frame = self.p('odom_frame', 'odom')
        self.base_frame = self.p('base_frame', 'base_link')

        # LiDAR mounting: base_link -> laser.
        self.laser_x = float(self.p('laser_x', 1.175))
        self.laser_y = float(self.p('laser_y', 0.000))
        self.laser_yaw = float(self.p('laser_yaw', math.pi))
        self.laser_yaw_extra = float(self.p('laser_yaw_extra', 0.0))
        self.laser_angle_sign = 1.0 if float(self.p('laser_angle_sign', -1.0)) >= 0.0 else -1.0

        # Scan / clustering
        self.min_range = float(self.p('min_range', 0.15))
        self.max_range = float(self.p('max_range', 8.0))
        self.cluster_gap = float(self.p('cluster_gap', 0.22))
        self.min_cluster_points = int(self.p('min_cluster_points', 1))
        self.max_cluster_points = int(self.p('max_cluster_points', 80))
        self.max_cluster_width = float(self.p('max_cluster_width', 0.45))
        self.cone_min_base_r = float(self.p('cone_min_base_r', 0.20))

        # ROI / slot geometry
        self.side = str(self.p('parking_side', 'right')).strip().lower()
        self.roi_x_min = float(self.p('roi_x_min', -1.00))
        self.roi_x_max = float(self.p('roi_x_max', 8.00))
        self.roi_y_min = float(self.p('roi_y_min', -4.50))
        self.roi_y_max = float(self.p('roi_y_max', -0.20))
        self.use_polar_roi = bool(self.p('use_polar_roi', False))
        self.theta_min_deg = float(self.p('theta_min_deg', -120.0))
        self.theta_max_deg = float(self.p('theta_max_deg', 120.0))

        self.min_cones_for_plan = int(self.p('min_cones_for_plan', 4))
        self.parallel_min_length = float(self.p('parallel_min_length', 1.85))
        self.parallel_max_length = float(self.p('parallel_max_length', 5.00))
        self.parallel_expected_length = float(self.p('parallel_expected_length', 2.20))
        self.parallel_min_width = float(self.p('parallel_min_width', 1.10))
        self.parallel_max_width = float(self.p('parallel_max_width', 2.50))
        self.parallel_expected_width = float(self.p('parallel_expected_width', 1.40))
        self.slot_axis_prior_max_error_deg = float(self.p('slot_axis_prior_max_error_deg', 40.0))
        self.use_start_axis_for_slot = bool(self.p('use_start_axis_for_slot', True))
        self.slot_percentile_low = float(self.p('slot_percentile_low', 5.0))
        self.slot_percentile_high = float(self.p('slot_percentile_high', 95.0))
        self.parallel_goal_lateral_ratio = float(self.p('parallel_goal_lateral_ratio', 0.50))
        self.parallel_goal_lateral_offset = float(self.p('parallel_goal_lateral_offset', 0.0))
        self.parallel_goal_longitudinal_offset = float(self.p('parallel_goal_longitudinal_offset', 0.0))
        self.temporal_gap_enable = bool(self.p('temporal_gap_enable', True))
        self.gap_side_window_along = float(
            self.p('gap_side_window_along', 0.45))
        # 측면 창은 경계를 안정적으로 보기 위해 진행축 앞/뒤를 함께 본다.
        # 따라서 GAP_OPEN은 실제 뒤쪽 경계를 지난 뒤, GAP_CLOSE는 실제
        # 앞쪽 경계에 도달하기 전에 발생한다. 잠긴 슬롯 양 끝을 창 반폭만큼
        # 되돌려 실제 경계 위치로 복원한다.
        self.gap_edge_longitudinal_compensation = max(0.0, float(
            self.p('gap_edge_longitudinal_compensation',
                   self.gap_side_window_along)))
        self.gap_side_min_distance = float(
            self.p('gap_side_min_distance', 0.55))
        self.gap_open_depth_samples = []
        self.gap_detector = TemporalSideGapDetector(
            min_width=self.parallel_min_length,
            max_width=self.parallel_max_length,
            laser_x=self.laser_x,
            laser_y=self.laser_y,
            side=self.side,
            open_jump=float(self.p('gap_open_jump', 0.60)),
            close_jump=float(self.p('gap_close_jump', 0.30)),
            confirm_frames=int(self.p('gap_confirm_frames', 2)),
            baseline_alpha=float(self.p('gap_baseline_alpha', 0.25)),
        )

        # Vehicle
        self.vehicle_length = float(self.p('vehicle_length', 1.410))
        self.vehicle_width = float(self.p('vehicle_width', 0.800))
        self.wheel_base = float(self.p('wheel_base', 0.735))
        self.laser_at_front_center = bool(self.p('laser_at_front_center', True))
        self.laser_to_front_bumper = float(self.p('laser_to_front_bumper', 0.0))
        self.base_to_front_bumper = float(self.p('base_to_front_bumper', 1.175))
        self.base_to_rear_bumper = float(self.p('base_to_rear_bumper', 0.230))
        if self.laser_at_front_center:
            self.base_to_front_bumper = max(0.0, self.laser_x + self.laser_to_front_bumper)
            self.base_to_rear_bumper = max(0.0, self.vehicle_length - self.base_to_front_bumper)
        # base_link가 차량 형상 중심이 아니라 뒤차축에 있으므로, base_link
        # 자체를 슬롯 중앙에 두면 긴 전방 오버행만큼 차량이 앞쪽으로
        # 치우친다. 차량 형상 중심이 슬롯 중앙에 오도록 목표를 보정한다.
        self.base_to_vehicle_center = 0.5 * (
            self.base_to_front_bumper - self.base_to_rear_bumper)
        self.side_margin = float(self.p('side_margin', 0.15))
        self.front_margin = float(self.p('front_margin', 0.20))
        self.rear_margin = float(self.p('rear_margin', 0.20))
        self.collision_margin = float(self.p('collision_margin', 0.10))

        # Sequence / command
        self.auto_start = bool(self.p('auto_start', False))
        self.cmd_rate = float(self.p('cmd_rate', 20.0))
        self.stop_hold_sec = float(self.p('stop_hold_sec', 0.8))
        self.stop_speed_thresh = float(self.p('stop_speed_thresh', 0.03))
        # SETTLE otherwise waits forever for vx to drop below
        # stop_speed_thresh - on even a slight slope, stop_mode=1 (flat
        # stop) releases the closed-loop hold PID entirely (see
        # can_driver.py's stop_mode docstring), so the vehicle can creep
        # downhill indefinitely and vx never settles (2026-08-12, found
        # live: encoder_count kept climbing one direction at a real stop).
        # Past this timeout SETTLE advances anyway rather than hanging.
        self.settle_timeout_sec = float(self.p('settle_timeout_sec', 5.0))
        self.pre_straight_rpm = int(self.p('pre_straight_rpm', 15))
        self.reverse_rpm = int(self.p('reverse_rpm', -15))
        self.final_rpm = int(self.p('final_rpm', -8))
        self.maneuver_forward_rpm = int(
            self.p('maneuver_forward_rpm', 10))
        self.maneuver_reverse_rpm = int(
            self.p('maneuver_reverse_rpm', -10))
        self.exit_forward_rpm = max(1, abs(int(
            self.p('exit_forward_rpm', 8))))
        self.exit_reverse_rpm = -max(1, abs(int(
            self.p('exit_reverse_rpm', -8))))
        self.maneuver_slow_rpm = max(1, abs(int(
            self.p('maneuver_slow_rpm', 8))))
        self.maneuver_decel_time_sec = max(0.1, float(
            self.p('maneuver_decel_time_sec', 1.0)))
        self.maneuver_arc_slowdown_min = math.radians(max(0.0, float(
            self.p('maneuver_arc_slowdown_min_deg', 8.0))))
        self.maneuver_arc_slowdown_margin = math.radians(max(0.0, float(
            self.p('maneuver_arc_slowdown_margin_deg', 2.0))))
        self.maneuver_straight_slowdown_min = max(0.0, float(
            self.p('maneuver_straight_slowdown_min_distance', 0.40)))
        self.maneuver_straight_slowdown_margin = max(0.0, float(
            self.p('maneuver_straight_slowdown_margin', 0.10)))
        self.sim_speed_forward = float(self.p('sim_speed_forward', 0.08))
        self.sim_speed_reverse = float(self.p('sim_speed_reverse', -0.08))
        self.max_steer_cmd = min(abs(int(self.p('max_steer_cmd', VEHICLE_MAX_STEER_CMD))),
                                 VEHICLE_MAX_STEER_CMD)
        self.steer_deadband_cmd = int(self.p('steer_deadband_cmd', 3))
        # 실차 조향각 기반 자전거 모델. 실측 반경을 확보하면 좌/우 최대
        # 조향각을 보정해 같은 계산 경로를 그대로 사용할 수 있다.
        self.max_steer_angle_left_deg = float(
            self.p('max_steer_angle_left_deg', 18.84))
        self.max_steer_angle_right_deg = float(
            self.p('max_steer_angle_right_deg', 18.05))
        self.forward_turn_sign = float(self.p('forward_turn_sign', -1.0))
        self.pre_straight_steer_sign = int(self.p('pre_straight_steer_sign', 1))
        self.pre_straight_yaw_kp = float(self.p('pre_straight_yaw_kp', 12.0))
        self.pre_straight_wz_kd = float(self.p('pre_straight_wz_kd', 12.0))
        self.pre_straight_use_imu = bool(
            self.p('pre_straight_use_imu', True))
        self.pre_straight_yaw_ki = float(
            self.p('pre_straight_yaw_ki', 20.0))
        self.pre_straight_i_limit = abs(float(
            self.p('pre_straight_i_limit', 4.0)))
        self.pre_straight_steer_trim = float(
            self.p('pre_straight_steer_trim', 0.0))
        self.pre_straight_abort_yaw = math.radians(abs(float(
            self.p('pre_straight_abort_yaw_deg', 4.0))))
        self.pre_straight_max_steer = int(self.p('pre_straight_max_steer', 6))
        self.pre_straight_min_steer = int(
            self.p('pre_straight_min_steer', 3))
        self.pre_straight_yaw_deadband = math.radians(float(
            self.p('pre_straight_yaw_deadband_deg', 0.3)))
        self.pre_straight_timeout_sec = float(self.p('pre_straight_timeout_sec', 0.0))
        # For parallel parking, start reversing while the locked goal is still
        # in front of base_link. Waiting until it passes x=0 drives too deep.
        self.approach_stop_x = float(self.p('pre_straight_front_stop_x', 1.50))
        self.approach_stop_tol = float(self.p('pre_straight_front_stop_tol', 0.10))
        self.approach_stop_ref = str(self.p('approach_stop_ref', 'front')).strip().lower()
        self.approach_after_lock_max_dist = float(
            self.p('approach_after_lock_max_dist', 1.80))
        self.lock_stable_window = max(1, int(self.p('pre_straight_stable_window', 3)))
        self.lock_required_hits = max(1, int(self.p(
            'pre_straight_required_hits', self.lock_stable_window)))
        self.min_cones_for_lock = max(self.min_cones_for_plan, int(self.p(
            'min_cones_for_lock', self.min_cones_for_plan)))
        self.lock_min_travel = float(self.p('pre_straight_min_travel_before_lock', 0.15))
        self.lock_front_max_x = float(self.p('pre_straight_lock_front_max_x', 1.25))
        self.length_stable_tol = float(self.p('pre_straight_length_stable_tol', 0.12))
        self.width_stable_tol = float(self.p('pre_straight_width_stable_tol', 0.10))
        self.yaw_stable_tol_deg = float(self.p('pre_straight_yaw_stable_tol_deg', 8.0))
        self.min_lateral_abs = float(self.p('pre_straight_min_lateral_abs', 0.15))

        # Calculated parallel-parking motion from calibrated steering radii.
        self.reverse_into_steer = int(self.p('reverse_into_steer', self.max_steer_cmd))
        self.reverse_out_steer = int(self.p('reverse_out_steer', -self.max_steer_cmd))
        self.reverse_into_yaw_deg = float(self.p('reverse_into_yaw_deg', 8.0))
        self.parallel_min_entry_yaw_deg = float(self.p('parallel_min_entry_yaw_deg', 8.0))
        self.parallel_max_entry_yaw_deg = float(self.p('parallel_max_entry_yaw_deg', 55.0))
        self.parallel_plan_tolerance = float(self.p('parallel_plan_tolerance', 0.08))
        self.parallel_max_straight_reverse = float(self.p('parallel_max_straight_reverse', 0.85))
        self.parallel_overshoot_tolerance = float(self.p('parallel_overshoot_tolerance', 0.25))
        self.parallel_min_steer_cmd = int(self.p('parallel_min_steer_cmd', 10))
        self.parallel_steer_step_cmd = max(1, int(self.p('parallel_steer_step_cmd', 1)))
        self.parallel_allow_front_start = bool(self.p('parallel_allow_front_start', True))
        self.parallel_front_start_yaw_deg = float(self.p('parallel_front_start_yaw_deg', 28.0))
        self.parallel_front_start_straight_len = float(self.p('parallel_front_start_straight_len', 0.08))
        self.reverse_out_yaw_tolerance_deg = float(self.p('reverse_out_yaw_tolerance_deg', 5.0))
        self.reverse_arc_timeout_sec = float(self.p('reverse_arc_timeout_sec', 5.0))
        self.final_align_max_steer = int(self.p('final_align_max_steer', 10))
        self.final_align_yaw_kp = float(self.p('final_align_yaw_kp', 9.0))
        self.final_align_lateral_kp = float(self.p('final_align_lateral_kp', 12.0))
        # The model is refreshed from odometry immediately before the long
        # diagonal reverse.  The remaining empirical bias covers tyre slip
        # and motion while the vehicle is settling after the three tail
        # segments.  A positive value makes the calculated stop shallower.
        self.maneuver_lateral_execution_bias = max(0.0, float(
            self.p('maneuver_lateral_execution_bias', 0.0)))
        self.final_center_rpm = max(1, abs(int(
            self.p('final_center_rpm', abs(self.final_rpm)))))
        self.final_center_max_distance = max(0.05, float(
            self.p('final_center_max_distance', 0.30)))
        self.auto_exit = bool(self.p('auto_exit', False))
        self.parking_hold_sec = max(
            0.0, float(self.p('parking_hold_sec', 3.0)))
        # 통합 주행에서는 주차 노드가 슬롯 안에서 후진 여유만 확보한 뒤
        # 제어권을 반환한다. 이후 좌조향 출차는 웨이포인트 추종기가 맡는다.
        self.exit_reverse_only = bool(self.p('exit_reverse_only', True))
        self.exit_heading_tolerance_deg = max(
            0.5, float(self.p('exit_heading_tolerance_deg', 7.0)))
        self.exit_outer_clearance_margin = max(
            0.0, float(self.p('exit_outer_clearance_margin', 0.05)))
        # Safety / done
        self.goal_tolerance = float(self.p('goal_tolerance', 0.055))
        self.lateral_tolerance = float(self.p('lateral_tolerance', 0.035))
        self.longitudinal_tolerance = float(self.p('longitudinal_tolerance', 0.060))
        self.final_yaw_tolerance_deg = float(self.p('final_yaw_tolerance_deg', 5.0))
        self.final_wz_tolerance = float(self.p('final_wz_tolerance', 0.10))
        # lateral_tolerance is declared just above this block in older
        # configurations, so refresh the final acceptance limit after loading
        # all safety/done parameters.
        self.final_center_lateral_tolerance = max(
            self.lateral_tolerance,
            float(self.p('final_center_lateral_tolerance', 0.10)))
        self.end_safety_margin = float(self.p('end_safety_margin', 0.035))
        self.side_safety_margin = float(self.p('side_safety_margin', 0.030))
        self.cone_radius = float(self.p('cone_radius', 0.055))
        self.cone_safety_stop_margin = float(self.p('cone_safety_stop_margin', 0.060))
        self.reverse_cone_safety_stop_margin = float(self.p(
            'reverse_cone_safety_stop_margin', 0.10))
        self.swept_path_check_enable = bool(
            self.p('swept_path_check_enable', True))
        self.swept_path_sample_step = max(
            0.01, float(self.p('swept_path_sample_step', 0.03)))
        self.swept_path_boundary_margin = max(
            0.0, float(self.p('swept_path_boundary_margin', 0.10)))
        self.swept_path_end_wall_check_enable = bool(
            self.p('swept_path_end_wall_check_enable', False))
        self.swept_path_max_candidates = max(
            1, int(self.p('swept_path_max_candidates', 120)))
        # An exact reverse replay is short, but real tyre slip and settling
        # mean that the parked pose is not the ideal end of the planned path.
        # When replay is unsafe, search a conventional three-part exit:
        # reverse inside the slot, forward away from it, then counter-steer
        # forward until the vehicle is parallel with the lane again.
        self.exit_reverse_clear_max = max(
            0.0, float(self.p('exit_reverse_clear_max', 0.85)))
        self.exit_reverse_clear_step = max(
            0.01, float(self.p('exit_reverse_clear_step', 0.025)))
        self.exit_rear_end_margin = max(
            self.swept_path_boundary_margin,
            float(self.p('exit_rear_end_margin', 0.18)))
        self.exit_arc_min_deg = max(
            5.0, float(self.p('exit_arc_min_deg', 35.0)))
        self.exit_arc_max_deg = max(
            self.exit_arc_min_deg,
            float(self.p('exit_arc_max_deg', 75.0)))
        self.exit_arc_step_deg = max(
            0.5, float(self.p('exit_arc_step_deg', 1.0)))
        self.exit_reverse_align_max_steer = max(
            0, min(self.max_steer_cmd, int(
                self.p('exit_reverse_align_max_steer', 10))))
        self.exit_preferred_clearance = max(
            self.reverse_cone_safety_stop_margin,
            float(self.p('exit_preferred_clearance', 0.14)))
        self.exit_feedback_max_adjust = math.radians(max(
            0.0, float(self.p(
                'exit_feedback_max_adjust_deg', 5.0))))
        self.exit_reverse_brake_buffer = max(
            0.0, float(self.p('exit_reverse_brake_buffer', 0.07)))

        # Logging
        self.direct_cmd_output = bool(self.p('direct_cmd_output', False))
        self.log_enable = bool(self.p('log_enable', True))
        log_root = self.p('log_root', os.path.expanduser('~/.ros/parking_logs'))
        self.log_dir = os.path.join(log_root, 'rule_parallel_simple_%s' % time.strftime('%Y%m%d_%H%M%S'))
        self.csv = CsvLogger(self.log_enable, self.log_dir)
        self.csv.open_writer('parallel_candidates.csv', [
            't', 'length', 'width', 'cx', 'cy', 'axis_yaw_deg', 'goal_x', 'goal_y', 'score', 'result'])
        self.csv.open_writer('parallel_control.csv', [
            't', 'state', 'x_odom', 'y_odom', 'yaw_deg',
            'dist', 's_err', 'q_err', 'yaw_err_deg',
            'front_clear', 'rear_clear', 'inner_clear', 'rpm', 'steer'])

        # Runtime state
        self.state = 'IDLE'
        self.next_state = None
        self.phase_start_time = None
        self.phase_start_pose = None
        self.settle_start = None
        self.approach_start_time = None
        self.sequence_start_pose = None
        self.sequence_start_yaw = 0.0
        self.pre_ref_yaw = 0.0
        self.pre_ref_wz = 0.0
        self.pre_ref_imu_yaw = 0.0
        self.pre_heading_i_cmd = 0.0
        self.pre_heading_control_time = None
        self.last_heading_error = 0.0
        self.last_heading_source = 'odom'
        self.pre_hist = deque(maxlen=self.lock_stable_window)
        self.lock_hit_count = 0
        self.locked_slot = None
        self.plan = None
        self.maneuver_plan = None
        self.maneuver_segments = None
        self.maneuver_segment_index = 0
        self.exit_segments = None
        self.exit_segment_index = 0
        self.exit_feedback_pending_segment = None
        self.exit_feedback_start_pose = None
        self.exit_requested = False
        self.parked_start_time = None
        self.lock_pose = None
        self.done = False
        self.abort_reason = ''

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.odom_received = False
        self.imu_yaw = 0.0
        self.imu_wz = 0.0
        self.imu_received = False
        self.pending_start_reason = None
        # Publishers / subscribers
        self.pub_stop_req = self.create_publisher(Bool, '/parking_request_stop', latched_qos())
        self.pub_mapping = self.create_publisher(Bool, '/parking_mapping', latched_qos())
        self.pub_active = self.create_publisher(Bool, '/parking_active', latched_qos())
        self.pub_done = self.create_publisher(Bool, '/parking_done', latched_qos())
        self.pub_parked = self.create_publisher(
            Bool, '/parking_parked', latched_qos())
        self.pub_exit_done = self.create_publisher(
            Bool, '/parking_exit_done', latched_qos())
        self.pub_status = self.create_publisher(String, '/parking_status', 10)
        self.pub_cones = self.create_publisher(PoseArray, '/parking/cones', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/parking/goal_pose', latched_qos())
        self.pub_markers = self.create_publisher(MarkerArray, '/parking/markers', 10)
        self.pub_cmd_rpm = self.create_publisher(Int16, '/parking/cmd_rpm', 10)
        self.pub_cmd_steer = self.create_publisher(Int16, '/parking/cmd_steer', 10)
        self.pub_cmd_enable = self.create_publisher(Int16, '/parking/cmd_enable', 10)
        if self.direct_cmd_output:
            self.pub_direct_rpm = self.create_publisher(Int16, '/cmd_rpm', 10)
            self.pub_direct_steer = self.create_publisher(Int16, '/cmd_steer', 10)
            self.pub_direct_enable = self.create_publisher(Int16, '/cmd_enable', 10)

        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)
        if self.pre_straight_use_imu:
            self.create_subscription(
                Imu, self.imu_topic, self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, '/parking_start', self.start_cb, 2)
        self.create_subscription(
            Bool, '/parking_exit_start', self.exit_start_cb, 2)
        self.create_subscription(Bool, '/parking_reset', self.reset_cb, 2)
        if self.auto_start:
            self.create_timer(1.0, self.auto_start_once)

        self.loginfo('[PARALLEL] one-shot calculated node ready side=%s steer_limit=%d',
                      self.side, self.max_steer_cmd)

    def p(self, name, default):
        self.declare_parameter(name, default)
        value = self.get_parameter(name).value
        return default if value is None else value

    def now(self):
        return self.get_clock().now()

    @staticmethod
    def dt(t1, t0):
        return float((t1 - t0).nanoseconds) * 1e-9

    def tsec(self):
        return float(self.now().nanoseconds) * 1e-9

    @staticmethod
    def _fmt(msg, args):
        return msg % args if args else msg

    def loginfo(self, msg, *args):
        self.get_logger().info(self._fmt(msg, args))

    def logwarn(self, msg, *args):
        self.get_logger().warn(self._fmt(msg, args))

    def logerr(self, msg, *args):
        self.get_logger().error(self._fmt(msg, args))

    def _log_throttle(self, level, period, msg, args):
        now = self.tsec()
        key = (level, msg)
        if now - self._throttle.get(key, -1e30) >= float(period):
            self._throttle[key] = now
            text = self._fmt(msg, args)
            # Humble rclpy는 동일한 Python 호출 위치에서 로그 severity가
            # 바뀌면 ValueError를 낸다. getattr 한 줄로 INFO/WARN을 함께
            # 호출하지 말고 severity별 호출 위치를 분리한다.
            if level == 'info':
                self.get_logger().info(text)
            elif level in ('warn', 'warning'):
                self.get_logger().warning(text)
            elif level == 'error':
                self.get_logger().error(text)
            else:
                self.get_logger().debug(text)

    def loginfo_throttle(self, period, msg, *args):
        self._log_throttle('info', period, msg, args)

    def logwarn_throttle(self, period, msg, *args):
        self._log_throttle('warn', period, msg, args)

    # ---------------- callbacks / state ----------------
    def auto_start_once(self):
        if self.auto_start and self.state == 'IDLE':
            self.auto_start = False
            self.begin_sequence('auto_start')

    def start_cb(self, msg):
        if bool(msg.data) and self.state in ['IDLE', 'DONE', 'ABORT']:
            self.pending_start_reason = 'topic_start'

    def exit_start_cb(self, msg):
        if bool(msg.data):
            self.exit_requested = True
            self.logwarn(
                '[PARALLEL_EXIT] request received state=%s', self.state)

    def reset_cb(self, msg):
        if bool(msg.data):
            self.reset_all('topic_reset')

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.vx = msg.twist.twist.linear.x
        self.wz = msg.twist.twist.angular.z
        self.odom_received = True
        if self.pending_start_reason is not None and self.state in ['IDLE', 'DONE', 'ABORT']:
            reason = self.pending_start_reason
            self.pending_start_reason = None
            self.begin_sequence(reason)

    def imu_cb(self, msg):
        self.imu_yaw = yaw_from_quat(msg.orientation)
        self.imu_wz = float(msg.angular_velocity.z)
        self.imu_received = True
        if (self.pending_start_reason is not None
                and self.state in ['IDLE', 'DONE', 'ABORT']
                and self.odom_received):
            reason = self.pending_start_reason
            self.pending_start_reason = None
            self.begin_sequence(reason)

    def scan_cb(self, msg):
        if self.state == 'APPROACH':
            self.process_scan_approach(msg)
        elif self.state in ['SETTLE', 'MANEUVER', 'REVERSE_IN_ARC',
                            'REVERSE_STRAIGHT', 'REVERSE_OUT_ARC',
                            'FINAL_ALIGN', 'PARKED', 'EXIT_MANEUVER',
                            'EXIT_COMPLETE']:
            cand = self.reproject_locked_slot()
            if cand is not None:
                self.publish_cones(cand.get('cones', []))
                self.publish_markers(cand.get('cones', []), cand)

    def begin_sequence(self, reason):
        if not self.odom_received:
            self.pending_start_reason = reason
            self.status('waiting odom before start')
            return
        if self.pre_straight_use_imu and not self.imu_received:
            self.pending_start_reason = reason
            self.status('waiting IMU before start')
            return
        self.state = 'APPROACH'
        self.sequence_start_pose = (self.x, self.y, self.yaw)
        self.sequence_start_yaw = self.yaw
        self.pre_ref_yaw = self.yaw
        self.pre_ref_wz = self.wz
        self.pre_ref_imu_yaw = self.imu_yaw
        self.pre_heading_i_cmd = self.pre_straight_steer_trim
        self.pre_heading_control_time = None
        self.last_heading_error = 0.0
        self.last_heading_source = (
            'imu' if self.pre_straight_use_imu else 'odom')
        self.gap_detector.reset(self.sequence_start_yaw)
        self.gap_open_depth_samples = []
        self.approach_start_time = self.now()
        self.pre_hist.clear()
        self.lock_hit_count = 0
        self.locked_slot = None
        self.plan = None
        self.maneuver_plan = None
        self.maneuver_segments = None
        self.maneuver_segment_index = 0
        self.exit_segments = None
        self.exit_segment_index = 0
        self.exit_feedback_pending_segment = None
        self.exit_feedback_start_pose = None
        self.exit_requested = False
        self.parked_start_time = None
        self.lock_pose = None
        self.phase_start_pose = None
        self.done = False
        self.abort_reason = ''
        self.publish_zero(active=True, mapping=True, done=False, stop_req=False)
        self.pub_parked.publish(Bool(data=False))
        self.pub_exit_done.publish(Bool(data=False))
        self.logwarn('[PARALLEL_SEQ] START reason=%s yaw=%.1fdeg', reason, math.degrees(self.yaw))

    def reset_all(self, reason):
        self.state = 'IDLE'
        self.next_state = None
        self.phase_start_time = None
        self.phase_start_pose = None
        self.settle_start = None
        self.sequence_start_pose = None
        self.locked_slot = None
        self.plan = None
        self.maneuver_plan = None
        self.maneuver_segments = None
        self.maneuver_segment_index = 0
        self.exit_segments = None
        self.exit_segment_index = 0
        self.exit_feedback_pending_segment = None
        self.exit_feedback_start_pose = None
        self.exit_requested = False
        self.parked_start_time = None
        self.lock_pose = None
        self.done = False
        self.abort_reason = ''
        self.pre_hist.clear()
        self.lock_hit_count = 0
        self.publish_zero(active=False, mapping=False, done=False, stop_req=False)
        self.pub_parked.publish(Bool(data=False))
        self.pub_exit_done.publish(Bool(data=False))
        self.logwarn('[PARALLEL_SEQ] RESET: %s', reason)

    def abort(self, reason):
        self.state = 'ABORT'
        self.abort_reason = reason
        self.publish_zero(active=True, mapping=False, done=False, stop_req=True)
        self.logerr('[PARALLEL_SEQ] ABORT: %s', reason)

    def go_settle(self, next_state):
        self.state = 'SETTLE'
        self.next_state = next_state
        self.settle_start = None
        self.publish_stop()

    # ---------------- transforms / scan ----------------
    def laser_point_to_base(self, r, theta):
        xl = r * math.cos(theta)
        yl = r * math.sin(theta)
        vehicle_angle = self.laser_yaw + self.laser_yaw_extra + self.laser_angle_sign * theta
        c = math.cos(vehicle_angle - theta)
        s = math.sin(vehicle_angle - theta)
        return np.array([self.laser_x + c * xl - s * yl,
                         self.laser_y + s * xl + c * yl], dtype=float)

    def base_to_odom(self, p_base, pose=None):
        if pose is None:
            pose = (self.x, self.y, self.yaw)
        c = math.cos(pose[2])
        s = math.sin(pose[2])
        return np.array([pose[0] + c * p_base[0] - s * p_base[1],
                         pose[1] + s * p_base[0] + c * p_base[1]], dtype=float)

    def odom_to_base(self, p_odom):
        dx = p_odom[0] - self.x
        dy = p_odom[1] - self.y
        c = math.cos(-self.yaw)
        s = math.sin(-self.yaw)
        return np.array([c * dx - s * dy, s * dx + c * dy], dtype=float)

    def in_roi(self, x, y, r=None, theta=None):
        if self.use_polar_roi and r is not None and theta is not None:
            deg = math.degrees(theta)
            return self.theta_min_deg <= deg <= self.theta_max_deg
        return self.roi_x_min <= x <= self.roi_x_max and self.roi_y_min <= y <= self.roi_y_max

    def scan_to_points(self, scan):
        pts = []
        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and self.min_range <= r <= self.max_range:
                p = self.laser_point_to_base(float(r), angle)
                in_roi = self.in_roi(p[0], p[1], float(r), angle)
                pts.append((p[0], p[1], float(r), angle, in_roi))
            angle += scan.angle_increment
        return pts

    def cluster_points(self, pts):
        clusters = []
        cur = []
        prev = None
        for p in pts:
            xy = np.array([p[0], p[1]], dtype=float)
            if prev is None or np.linalg.norm(xy - prev) <= self.cluster_gap:
                cur.append(p)
            else:
                if cur:
                    clusters.append(cur)
                cur = [p]
            prev = xy
        if cur:
            clusters.append(cur)
        return clusters

    def clusters_to_cones(self, clusters):
        cones = []
        for c in clusters:
            if len(c) < self.min_cluster_points or len(c) > self.max_cluster_points:
                continue
            arr = np.array([[p[0], p[1]] for p in c], dtype=float)
            width = float(np.max(np.linalg.norm(arr - np.mean(arr, axis=0), axis=1))) if len(arr) > 1 else 0.0
            if width > self.max_cluster_width:
                continue
            xy = np.mean(arr, axis=0)
            if np.linalg.norm(xy) >= self.cone_min_base_r:
                cones.append(xy)
        return cones

    # ---------------- slot estimate / fixed plan ----------------
    def estimate_parallel_slot(self, cones):
        if len(cones) < self.min_cones_for_plan:
            return None
        arr = np.array(cones, dtype=float)
        center = np.mean(arr, axis=0)
        cov = np.cov((arr - center).T) if arr.shape[0] >= 3 else None
        if cov is not None:
            vals, vecs = np.linalg.eig(cov)
            axis = np.real(vecs[:, int(np.argmax(vals))])
        else:
            axis = arr[-1] - arr[0]
        axis = unit_or_zero(axis)

        start_axis = np.array([math.cos(self.sequence_start_yaw - self.yaw),
                               math.sin(self.sequence_start_yaw - self.yaw)], dtype=float)
        if float(np.dot(axis, start_axis)) < 0.0:
            axis = -axis
        axis_yaw = math.atan2(axis[1], axis[0])
        prior_err = abs(math.degrees(normalize_angle(axis_yaw - math.atan2(start_axis[1], start_axis[0]))))
        prior_err = min(prior_err, 180.0 - prior_err)
        if self.use_start_axis_for_slot:
            axis = unit_or_zero(start_axis)
            axis_yaw = math.atan2(axis[1], axis[0])

        side_vec = np.array([axis[1], -axis[0]], dtype=float) if self.side == 'right' else np.array([-axis[1], axis[0]], dtype=float)
        side_vec = unit_or_zero(side_vec)
        rel = arr - center
        s_vals = np.dot(rel, axis)
        q_vals = np.dot(rel, side_vec)
        s_low = robust_percentile(s_vals, self.slot_percentile_low, default=float(np.min(s_vals)))
        s_high = robust_percentile(s_vals, self.slot_percentile_high, default=float(np.max(s_vals)))
        q_low = robust_percentile(q_vals, self.slot_percentile_low, default=float(np.min(q_vals)))
        q_high = robust_percentile(q_vals, self.slot_percentile_high, default=float(np.max(q_vals)))
        length = float(s_high - s_low)
        width = float(q_high - q_low)

        result = 'ok'
        if prior_err > self.slot_axis_prior_max_error_deg:
            result = 'reject_axis'
        elif length < self.parallel_min_length or length > self.parallel_max_length:
            result = 'reject_length'
        elif width < self.parallel_min_width or width > self.parallel_max_width:
            result = 'reject_width'
        elif width < self.vehicle_width + 2.0 * self.side_margin:
            result = 'reject_vehicle_width'
        elif length < self.vehicle_length + self.front_margin + self.rear_margin:
            result = 'reject_vehicle_length'

        q_goal = q_low + self.parallel_goal_lateral_ratio * width + self.parallel_goal_lateral_offset
        q_goal = clamp(q_goal, q_low + 0.5 * self.vehicle_width + self.side_margin,
                       q_high - 0.5 * self.vehicle_width - self.side_margin)
        s_goal = (
            0.5 * (s_low + s_high)
            - self.base_to_vehicle_center
            + self.parallel_goal_longitudinal_offset)
        s_goal = clamp(s_goal, s_low + self.base_to_rear_bumper + self.rear_margin,
                       s_high - self.base_to_front_bumper - self.front_margin)
        p_goal = center + s_goal * axis + q_goal * side_vec
        slot_center = center + 0.5 * (s_low + s_high) * axis + 0.5 * (q_low + q_high) * side_vec
        front_ref = center + s_high * axis + q_goal * side_vec

        score = 10.0 * len(arr) - 12.0 * abs(length - self.parallel_expected_length) - 10.0 * abs(width - self.parallel_expected_width)
        self.csv.row('parallel_candidates.csv', [
            self.tsec(), length, width, slot_center[0], slot_center[1],
            math.degrees(axis_yaw), p_goal[0], p_goal[1], score if result == 'ok' else -999, result])
        if result != 'ok':
            return None
        return {
            'axis': axis, 'side_vec': side_vec, 'origin': center,
            'slot_center': slot_center, 'length': length, 'width': width,
            's_low': float(s_low), 's_high': float(s_high),
            'q_low': float(q_low), 'q_high': float(q_high),
            's_goal': float(s_goal), 'q_goal': float(q_goal),
            'P_goal': p_goal, 'P_front_ref': front_ref,
            'yaw_goal_base': normalize_angle(axis_yaw),
            'cones': [np.array(c, dtype=float) for c in arr],
            'score': score, 'prior_err': prior_err,
        }

    def candidate_stable(self, cand):
        self.pre_hist.append((cand['length'], cand['width'], cand['yaw_goal_base']))
        if len(self.pre_hist) < self.lock_stable_window:
            return False
        lengths = [v[0] for v in self.pre_hist]
        widths = [v[1] for v in self.pre_hist]
        yaws = [v[2] for v in self.pre_hist]
        yaw_range = max(abs(math.degrees(normalize_angle(y - yaws[0]))) for y in yaws)
        return (max(lengths) - min(lengths) <= self.length_stable_tol
                and max(widths) - min(widths) <= self.width_stable_tol
                and yaw_range <= self.yaw_stable_tol_deg)

    def lock_candidate(self, cand):
        pose = (self.x, self.y, self.yaw)
        self.lock_pose = pose
        self.locked_slot = {
            # 평행주차 완료 헤딩은 슬롯 PCA가 아니라 시작 IMU 헤딩과 같다.
            'axis_yaw_odom': normalize_angle(self.sequence_start_yaw),
            'side_yaw_odom': normalize_angle(pose[2] + math.atan2(cand['side_vec'][1], cand['side_vec'][0])),
            'origin_odom': self.base_to_odom(cand['origin'], pose),
            'slot_center_odom': self.base_to_odom(cand['slot_center'], pose),
            'P_goal_odom': self.base_to_odom(cand['P_goal'], pose),
            'P_front_ref_odom': self.base_to_odom(cand['P_front_ref'], pose),
            'length': cand['length'], 'width': cand['width'],
            's_low': cand['s_low'], 's_high': cand['s_high'],
            'q_low': cand['q_low'], 'q_high': cand['q_high'],
            's_goal': cand['s_goal'], 'q_goal': cand['q_goal'],
            'cones_odom': [self.base_to_odom(c, pose) for c in cand.get('cones', [])],
        }
        self.plan = None
        self.publish_goal(self.reproject_locked_slot())
        front_along = float(np.dot(cand['P_front_ref'], cand['axis']))
        self.logwarn('[PARALLEL_SEQ] SLOT LOCKED length=%.3f width=%.3f yaw=%.1f '
                      'cones=%d hits=%d/%d front=%.3f moved=%.3f goal=(%.3f, %.3f)',
                      cand['length'], cand['width'], math.degrees(self.yaw + cand['yaw_goal_base']),
                      len(cand.get('cones', [])), self.lock_hit_count, self.lock_required_hits,
                      front_along, self.approach_distance_from_start(),
                      cand['P_goal'][0], cand['P_goal'][1])

    def approach_stop_point(self, cand):
        if self.approach_stop_ref == 'front':
            return cand['P_front_ref'], 'front'
        if self.approach_stop_ref == 'center':
            return cand['slot_center'], 'center'
        return cand['P_goal'], 'goal'

    def approach_distance_after_lock(self):
        if self.lock_pose is None:
            return 0.0
        return math.hypot(self.x - self.lock_pose[0], self.y - self.lock_pose[1])

    def approach_distance_from_start(self):
        if self.sequence_start_pose is None:
            return 0.0
        return math.hypot(self.x - self.sequence_start_pose[0],
                          self.y - self.sequence_start_pose[1])

    def slot_ready_for_lock(self, cand, stable, lateral_ok):
        cone_count = len(cand.get('cones', []))
        front_along = float(np.dot(cand['P_front_ref'], cand['axis']))
        moved = self.approach_distance_from_start()
        info = {
            'cones': cone_count,
            'front_along': front_along,
            'moved': moved,
        }
        if not stable:
            return False, 'unstable', info
        if not lateral_ok:
            return False, 'lateral', info
        # 시간축 gap 검출은 한 프레임의 콘 개수 대신 막힘→열림→막힘을
        # 여러 프레임과 odom으로 확인한다. 입구 양 끝점만 보관하므로
        # 기존 사각형 콘 검출의 6개 조건을 적용하면 영원히 lock되지 않는다.
        if (not cand.get('temporal_gap', False)
                and cone_count < self.min_cones_for_lock):
            return False, 'cones', info
        if moved < self.lock_min_travel:
            return False, 'travel', info
        if front_along > self.lock_front_max_x:
            return False, 'front_far', info
        return True, 'ready', info

    def reproject_locked_slot(self):
        if self.locked_slot is None:
            return None
        snap = self.locked_slot
        axis_yaw = normalize_angle(snap['axis_yaw_odom'] - self.yaw)
        side_yaw = normalize_angle(snap['side_yaw_odom'] - self.yaw)
        return {
            'axis': np.array([math.cos(axis_yaw), math.sin(axis_yaw)], dtype=float),
            'side_vec': np.array([math.cos(side_yaw), math.sin(side_yaw)], dtype=float),
            'origin': self.odom_to_base(snap['origin_odom']),
            'slot_center': self.odom_to_base(snap['slot_center_odom']),
            'P_goal': self.odom_to_base(snap['P_goal_odom']),
            'P_front_ref': self.odom_to_base(snap['P_front_ref_odom']),
            'yaw_goal_base': axis_yaw,
            'length': snap['length'], 'width': snap['width'],
            's_low': snap['s_low'], 's_high': snap['s_high'],
            'q_low': snap['q_low'], 'q_high': snap['q_high'],
            's_goal': snap['s_goal'], 'q_goal': snap['q_goal'],
            'cones': [self.odom_to_base(c) for c in snap.get('cones_odom', [])],
        }

    def radius_for_cmd(self, steer_cmd):
        cmd = int(round(steer_cmd))
        if cmd == 0:
            return float('inf')
        max_angle_deg = (self.max_steer_angle_right_deg if cmd > 0
                         else self.max_steer_angle_left_deg)
        steer_angle = math.radians(max_angle_deg) * (
            min(abs(cmd), self.max_steer_cmd) / float(self.max_steer_cmd))
        if abs(steer_angle) < 1e-6:
            return float('inf')
        return self.wheel_base / math.tan(abs(steer_angle))

    def compute_parallel_plan(self, cand):
        """Compute one frozen reverse plan from the current stopped pose.

        The slot is already locked in odom. This uses the calibrated steering
        radius table, searches allowed steering commands, then returns one
        plan to execute without recalculating during reverse.
        """
        side_sign = 1.0 if self.side == 'right' else -1.0
        goal_along = float(np.dot(cand['P_goal'], cand['axis']))
        dY = float(np.dot(cand['P_goal'], cand['side_vec']))
        stop_point, stop_ref = self.approach_stop_point(cand)
        stop_along = float(np.dot(stop_point, cand['axis']))
        front_target_along = self.approach_stop_x
        front_ready = stop_along <= front_target_along + self.approach_stop_tol
        back_needed_now = max(0.0, -goal_along)
        speed_rev = max(abs(self.sim_speed_reverse), 0.01)

        into_sign = 1 if self.reverse_into_steer >= 0 else -1
        out_sign = 1 if self.reverse_out_steer >= 0 else -1
        max_mag = min(abs(int(self.reverse_into_steer)),
                      abs(int(self.reverse_out_steer)),
                      int(self.max_steer_cmd))
        min_mag = max(1, min(int(self.parallel_min_steer_cmd), max_mag))
        steer_values = list(range(min_mag, max_mag + 1, self.parallel_steer_step_cmd))
        if max_mag not in steer_values:
            steer_values.append(max_mag)

        best = None
        solutions = []
        theta_min = math.radians(max(1.0, self.parallel_min_entry_yaw_deg))
        theta_max = math.radians(max(self.parallel_min_entry_yaw_deg, self.parallel_max_entry_yaw_deg))
        if dY > 0.0 and theta_max > theta_min:
            for mag in steer_values:
                into_steer = int(round(clamp(into_sign * mag,
                                             -self.max_steer_cmd, self.max_steer_cmd)))
                out_steer = int(round(clamp(out_sign * mag,
                                            -self.max_steer_cmd, self.max_steer_cmd)))
                R_in = self.radius_for_cmd(into_steer)
                R_out = self.radius_for_cmd(out_steer)
                Rsum = R_in + R_out
                if not math.isfinite(Rsum):
                    continue
                for theta in np.linspace(theta_min, theta_max, 160):
                    sn = math.sin(theta)
                    cs = math.cos(theta)
                    if abs(sn) < 1e-4:
                        continue
                    arc_side = Rsum * (1.0 - cs)
                    straight_len = (dY - arc_side) / sn
                    if straight_len < -self.parallel_plan_tolerance:
                        continue
                    straight_len = max(0.0, straight_len)
                    if straight_len > self.parallel_max_straight_reverse:
                        continue
                    pred_back = Rsum * sn + straight_len * cs
                    err = pred_back - back_needed_now
                    score = abs(err) + 0.02 * straight_len + 0.002 * (max_mag - mag)
                    option = {
                        'theta': theta,
                        'straight_len': straight_len,
                        'pred_back': pred_back,
                        'err': err,
                        'score': score,
                        'into_steer': into_steer,
                        'out_steer': out_steer,
                        'R_in': R_in,
                        'R_out': R_out,
                    }
                    solutions.append(option)
                    if best is None or score < best['score']:
                        best = option

        # ------------------------------------------------------------------
        # front_start 폴백은 제거했다.
        # 그 플랜은 '종방향 후진거리'만 맞추고 '횡거리 dY'는 전혀 만족시키지
        # 못한다. 예) theta=28deg, straight=0.08 이면 실제 횡이동은
        #   Rsum*(1-cos28) + 0.08*sin28 ~= 0.22m 뿐인데,
        # 실측 dY 는 0.53~0.69m 였다. 그래서 차가 슬롯 옆에 그대로 남았다.
        # 유일하게 올바른 플랜은 아래 exact solver(best) 뿐이다.
        # 대신 '이 슬롯/접근거리로 도달 가능한 최대 횡거리'를 명시 검사한다.
        # ------------------------------------------------------------------
        swept_check = {
            'safe': False, 'reason': 'not_checked',
            'min_cone_clear': 999.0, 'min_inner_clear': 999.0,
            'min_end_clear': 999.0, 'phase': 'NONE',
            'pose': (0.0, 0.0, 0.0),
        }
        if best is None:
            theta = theta_min
            straight_len = 0.0
            pred_back = 0.0
            plan_error = 999.0
            into_steer = int(round(clamp(self.reverse_into_steer,
                                         -self.max_steer_cmd, self.max_steer_cmd)))
            out_steer = int(round(clamp(self.reverse_out_steer,
                                        -self.max_steer_cmd, self.max_steer_cmd)))
            R_in = self.radius_for_cmd(into_steer)
            R_out = self.radius_for_cmd(out_steer)
            exact = False
            feasible = False
            status = 'no_solution'
            # 왜 해가 없는지 진단: 만타 기준 도달 가능한 최대 횡거리와 비교.
            rsum_max = R_in + R_out
            if math.isfinite(rsum_max):
                dy_max = (rsum_max * (1.0 - math.cos(theta_max))
                          + self.parallel_max_straight_reverse * math.sin(theta_max))
                self.logwarn_throttle(
                    2.0,
                    '[PARALLEL_PLAN] no_solution: dY=%.3f 필요, 최대 도달 %.3f '
                    '(theta_max=%.0fdeg, Rsum=%.3f, max_straight=%.2f). '
                    '슬롯에 더 붙여 접근해 dY 를 줄이거나 '
                    'parallel_max_entry_yaw_deg 를 키우세요.',
                    dY, dy_max, math.degrees(theta_max), rsum_max,
                    self.parallel_max_straight_reverse)
        else:
            theta = best['theta']
            straight_len = best['straight_len']
            pred_back = best['pred_back']
            plan_error = best['err']
            into_steer = best['into_steer']
            out_steer = best['out_steer']
            R_in = best['R_in']
            R_out = best['R_out']
            exact = abs(plan_error) <= self.parallel_plan_tolerance
            if exact:
                status = 'ready'
                feasible = True
            elif plan_error > 0.0:
                status = 'need_forward'
                feasible = False
            elif abs(plan_error) <= self.parallel_overshoot_tolerance:
                status = 'slightly_past'
                feasible = True
            else:
                status = 'overshot'
                feasible = False

            # 끝점 오차만 맞는 플랜을 실차에 보내지 않는다. 동일한 끝점
            # 허용범위의 후보들을 차량 사각형 전체로 미리 주행시켜,
            # 입구 콘/앞뒤 경계/안쪽 경계 중 하나라도 여유가 부족하면
            # 그 후보를 폐기한다.
            if feasible and self.swept_path_check_enable:
                safe_best = None
                first_failure = None
                eligible = []
                for option in solutions:
                    err = option['err']
                    if (abs(err) <= self.parallel_plan_tolerance
                            or (-self.parallel_overshoot_tolerance
                                <= err < -self.parallel_plan_tolerance)):
                        eligible.append(option)
                eligible.sort(key=lambda item: item['score'])
                for option in eligible[:self.swept_path_max_candidates]:
                    check = self.validate_swept_path(cand, option)
                    if first_failure is None:
                        first_failure = check
                    if check['safe']:
                        safe_best = option
                        swept_check = check
                        break

                if safe_best is None:
                    feasible = False
                    exact = False
                    status = 'unsafe_swept_path'
                    if first_failure is not None:
                        swept_check = first_failure
                    self.logwarn_throttle(
                        1.0,
                        '[PARALLEL_PLAN] unsafe_swept_path reason=%s '
                        'phase=%s pose=(%.2f,%.2f,%.1fdeg) '
                        'clear[cone=%.3f inner=%.3f end=%.3f]',
                        swept_check['reason'], swept_check['phase'],
                        swept_check['pose'][0], swept_check['pose'][1],
                        math.degrees(swept_check['pose'][2]),
                        swept_check['min_cone_clear'],
                        swept_check['min_inner_clear'],
                        swept_check['min_end_clear'])
                else:
                    best = safe_best
                    theta = best['theta']
                    straight_len = best['straight_len']
                    pred_back = best['pred_back']
                    plan_error = best['err']
                    into_steer = best['into_steer']
                    out_steer = best['out_steer']
                    R_in = best['R_in']
                    R_out = best['R_out']
                    exact = abs(plan_error) <= self.parallel_plan_tolerance
                    status = 'ready' if exact else 'slightly_past'
            elif status == 'need_forward' and self.swept_path_check_enable:
                # 현재 최적 플랜이 요구하는 위치까지 더 전진했을 때의 슬롯을
                # 미리 옮겨 검사한다. 그 위치에서도 충돌하는 경로라면 차량을
                # 그곳까지 보내지 않고 잠금 직후 정지한다.
                preview_cand = self.shift_candidate_for_forward(
                    cand, max(0.0, plan_error))
                swept_check = self.validate_swept_path(preview_cand, best)
                if not swept_check['safe']:
                    status = 'unsafe_swept_path'
                    self.logwarn_throttle(
                        1.0,
                        '[PARALLEL_PLAN] future start is unsafe: forward=%.3f '
                        'reason=%s phase=%s clear[cone=%.3f inner=%.3f end=%.3f]',
                        max(0.0, plan_error), swept_check['reason'],
                        swept_check['phase'], swept_check['min_cone_clear'],
                        swept_check['min_inner_clear'],
                        swept_check['min_end_clear'])

        Rsum = R_in + R_out
        L_arc = Rsum * math.sin(theta) if math.isfinite(Rsum) else 0.0
        reverse_start_target = -pred_back
        return {
            'side_sign': side_sign, 'into_steer': into_steer, 'out_steer': out_steer,
            'R_in': R_in, 'R_out': R_out, 'delta_y': dY, 'into_yaw': theta,
            'L_arc': L_arc, 'straight_len': straight_len,
            'target_along': reverse_start_target,
            'front_target_along': front_target_along,
            'stop_along': stop_along, 'stop_ref': stop_ref,
            'goal_along': goal_along, 'back_needed': back_needed_now,
            'pred_back': pred_back, 'plan_error': plan_error,
            'status': status, 'feasible': feasible, 'exact': exact,
            'swept_safe': bool(swept_check['safe']),
            'swept_reason': swept_check['reason'],
            'swept_min_cone_clear': swept_check['min_cone_clear'],
            'swept_min_inner_clear': swept_check['min_inner_clear'],
            'swept_min_end_clear': swept_check['min_end_clear'],
            't_in': R_in * theta / speed_rev,
            't_straight': straight_len / speed_rev,
            't_out': R_out * theta / speed_rev,
        }

    def measure_side_boundary(self, pts):
        """LiDAR 바로 옆의 우측 수직 창에서 가장 가까운 경계를 측정한다."""
        if not pts:
            return None
        lane = np.array([
            math.cos(self.sequence_start_yaw - self.yaw),
            math.sin(self.sequence_start_yaw - self.yaw),
        ], dtype=float)
        side_sign = 1.0 if self.side == 'left' else -1.0
        side_dir = np.array(
            [-side_sign * lane[1], side_sign * lane[0]], dtype=float)
        sensor = np.array([self.laser_x, self.laser_y], dtype=float)
        values = []
        for p in pts:
            xy = np.array([p[0], p[1]], dtype=float)
            rel = xy - sensor
            along = float(np.dot(rel, lane))
            lateral = float(np.dot(rel, side_dir))
            if (abs(along) <= self.gap_side_window_along
                    and self.gap_side_min_distance <= lateral <= self.max_range):
                values.append(lateral)
        return min(values) if values else None

    def temporal_gap_candidate(self, cones):
        """주행 중 측정한 우측 막힘→열림→막힘 구간을 슬롯으로 만든다."""
        gd = self.gap_detector
        if not gd.completed:
            return None
        a = self.odom_to_base(gd.start_edge_odom)
        b = self.odom_to_base(gd.end_edge_odom)
        axis_yaw = normalize_angle(self.sequence_start_yaw - self.yaw)
        axis = np.array([math.cos(axis_yaw), math.sin(axis_yaw)], dtype=float)
        if float(np.dot(b - a, axis)) < 0.0:
            a, b = b, a
        edge_comp = self.gap_edge_longitudinal_compensation
        a = a - edge_comp * axis
        b = b + edge_comp * axis
        side_vec = (np.array([axis[1], -axis[0]], dtype=float)
                    if self.side == 'right'
                    else np.array([-axis[1], axis[0]], dtype=float))
        length = abs(float(np.dot(b - a, axis)))
        # 열린 구간에서 관측한 먼 경계와, 열리기 전 가까운 경계 사이의
        # 거리로 슬롯 깊이를 구한다. 특정 2.4 m 패턴에 고정하지 않는다.
        if self.gap_open_depth_samples:
            width = float(np.percentile(
                np.asarray(self.gap_open_depth_samples, dtype=float), 25.0))
            width = clamp(
                width, self.parallel_min_width, self.parallel_max_width)
        else:
            width = self.parallel_expected_width
        origin = np.asarray(a, dtype=float)
        s_low, s_high = 0.0, length
        q_low, q_high = 0.0, width
        s_goal = clamp(
            0.5 * length - self.base_to_vehicle_center
            + self.parallel_goal_longitudinal_offset,
            self.base_to_rear_bumper + self.rear_margin,
            length - self.base_to_front_bumper - self.front_margin)
        q_goal = clamp(
            self.parallel_goal_lateral_ratio * width
            + self.parallel_goal_lateral_offset,
            0.5 * self.vehicle_width + self.side_margin,
            width - 0.5 * self.vehicle_width - self.side_margin)
        p_goal = origin + s_goal * axis + q_goal * side_vec
        slot_center = origin + 0.5 * length * axis + 0.5 * width * side_vec
        front_ref = origin + length * axis + q_goal * side_vec
        edge_cones = [np.asarray(a), np.asarray(b)]
        return {
            'axis': axis, 'side_vec': side_vec, 'origin': origin,
            'slot_center': slot_center, 'length': length, 'width': width,
            's_low': s_low, 's_high': s_high,
            'q_low': q_low, 'q_high': q_high,
            's_goal': s_goal, 'q_goal': q_goal,
            'P_goal': p_goal, 'P_front_ref': front_ref,
            'yaw_goal_base': axis_yaw,
            'cones': edge_cones,
            'score': 100.0, 'prior_err': 0.0,
            'temporal_gap': True,
        }

    def process_scan_approach(self, scan):
        if self.locked_slot is not None:
            cand = self.reproject_locked_slot()
            self.publish_cones(cand.get('cones', []))
            self.publish_markers(cand.get('cones', []), cand)
            return
        pts = self.scan_to_points(scan)
        pts_roi = [p for p in pts if p[4]]
        cones = self.clusters_to_cones(self.cluster_points(pts_roi))
        gap_event = None
        side_distance = None
        if self.temporal_gap_enable:
            side_distance = self.measure_side_boundary(pts)
            gap_event = self.gap_detector.update(
                side_distance, (self.x, self.y, self.yaw))
            baseline = self.gap_detector.baseline
            if (self.gap_detector.state in ['IN_GAP', 'COMPLETE']
                    and side_distance is not None
                    and baseline is not None):
                measured_depth = float(side_distance) - float(baseline)
                if (self.parallel_min_width
                        <= measured_depth
                        <= self.parallel_max_width):
                    self.gap_open_depth_samples.append(measured_depth)
            cand = self.temporal_gap_candidate(cones)
        else:
            cand = None
        if cand is None and not self.temporal_gap_enable:
            cand = self.estimate_parallel_slot(cones)
        self.publish_cones(cones)
        self.publish_markers(cones, cand)
        if cand is None:
            self.pre_hist.clear()
            self.lock_hit_count = 0
            gap_state = self.gap_detector.state if self.temporal_gap_enable else 'off'
            gap_width = (self.gap_detector.provisional_width
                         if self.temporal_gap_enable else 0.0)
            self.status(
                'APPROACH(gap): state=%s event=%s side=%s width=%.2f '
                'roi=%d cones=%d'
                % (gap_state, gap_event or self.gap_detector.last_event,
                   'none' if side_distance is None else '%.2f' % side_distance,
                   gap_width, len(pts_roi), len(cones)))
            self.loginfo_throttle(
                1.0,
                '[PARALLEL_GAP] state=%s event=%s side=%s width=%.2f',
                gap_state, gap_event or self.gap_detector.last_event,
                'none' if side_distance is None else '%.2f' % side_distance,
                gap_width)
            return
        stable = self.candidate_stable(cand)
        lateral_ok = abs(float(cand['slot_center'][1])) >= self.min_lateral_abs
        ready, reason, info = self.slot_ready_for_lock(cand, stable, lateral_ok)
        if ready and self.odom_received:
            self.lock_hit_count += 1
        else:
            self.lock_hit_count = 0
        if self.lock_hit_count >= self.lock_required_hits:
            self.lock_candidate(cand)
        self.status('APPROACH(scan): length=%.2f width=%.2f yaw=%.1f stable=%s '
                    'ready=%s/%s hits=%d/%d cones=%d/%d front=%.2f/%.2f moved=%.2f/%.2f' %
                    (cand['length'], cand['width'], math.degrees(cand['yaw_goal_base']),
                     stable, ready, reason, self.lock_hit_count, self.lock_required_hits,
                     info['cones'], self.min_cones_for_lock,
                     info['front_along'], self.lock_front_max_x,
                     info['moved'], self.lock_min_travel))

    # ---------------- geometry / checks ----------------
    def slot_coords(self, cand, p):
        rel = np.array(p, dtype=float) - cand['origin']
        return float(np.dot(rel, cand['axis'])), float(np.dot(rel, cand['side_vec']))

    def slot_errors_for_pose(self, cand, pose):
        p = np.array([pose[0], pose[1]], dtype=float)
        s, q = self.slot_coords(cand, p)
        return (float(np.linalg.norm(cand['P_goal'] - p)),
                s - cand['s_goal'], q - cand['q_goal'],
                normalize_angle(cand['yaw_goal_base'] - pose[2]))

    def footprint_corners(self, pose):
        px, py, yaw = pose
        c = math.cos(yaw)
        s = math.sin(yaw)
        local = [
            [self.base_to_front_bumper, 0.5 * self.vehicle_width],
            [self.base_to_front_bumper, -0.5 * self.vehicle_width],
            [-self.base_to_rear_bumper, 0.5 * self.vehicle_width],
            [-self.base_to_rear_bumper, -0.5 * self.vehicle_width],
        ]
        return [np.array([px + c * lx - s * ly, py + s * lx + c * ly], dtype=float) for lx, ly in local]

    def footprint_slot_clearance(self, cand, pose):
        ss, qs = [], []
        for corner in self.footprint_corners(pose):
            s, q = self.slot_coords(cand, corner)
            ss.append(s)
            qs.append(q)
        return {
            'front_clear': cand['s_high'] - max(ss),
            'rear_clear': min(ss) - cand['s_low'],
            'inner_clear': cand['q_high'] - max(qs),
            'outer_clear': min(qs) - cand['q_low'],
        }

    def footprint_cone_clearance(self, cand, pose):
        px, py, yaw = pose
        c = math.cos(-yaw)
        s = math.sin(-yaw)
        half_w = 0.5 * self.vehicle_width + self.collision_margin
        front = self.base_to_front_bumper + self.collision_margin
        rear = self.base_to_rear_bumper + self.collision_margin
        min_clear = 999.0
        collision = False
        for cone in cand.get('cones', []) or []:
            dx = float(cone[0]) - px
            dy = float(cone[1]) - py
            lx = c * dx - s * dy
            ly = s * dx + c * dy
            if -rear <= lx <= front and abs(ly) <= half_w:
                collision = True
                min_clear = 0.0
                continue
            cx = clamp(lx, -rear, front)
            cy = clamp(ly, -half_w, half_w)
            clear = math.hypot(lx - cx, ly - cy) - self.cone_radius
            if clear <= 0.0:
                collision = True
            min_clear = min(min_clear, clear)
        return collision, min_clear

    def advance_reverse_pose(self, pose, distance, steer_cmd):
        """자전거 모델로 후진 distance만큼 진행한 다음 pose를 반환한다."""
        return self.advance_motion_pose(pose, distance, steer_cmd, -1)

    def advance_motion_pose(self, pose, distance, steer_cmd, direction):
        """전진(+1)/후진(-1) 자전거 모델의 다음 pose를 반환한다."""
        px, py, yaw = pose
        if distance <= 0.0:
            return (px, py, yaw)
        radius = self.radius_for_cmd(steer_cmd)
        if steer_cmd == 0 or not math.isfinite(radius):
            return (
                px + direction * distance * math.cos(yaw),
                py + direction * distance * math.sin(yaw),
                yaw,
            )
        steer_sign = 1.0 if steer_cmd >= 0 else -1.0
        if direction > 0:
            turn_sign = self.forward_turn_sign * steer_sign
        else:
            turn_sign = steer_sign
        dyaw = turn_sign * distance / radius
        mid_yaw = yaw + 0.5 * dyaw
        return (
            px + direction * distance * math.cos(mid_yaw),
            py + direction * distance * math.sin(mid_yaw),
            normalize_angle(yaw + dyaw),
        )

    def shift_candidate_for_forward(self, cand, distance):
        """차량이 슬롯 축으로 distance 전진한 뒤 보일 후보 좌표를 만든다."""
        shifted = dict(cand)
        delta = max(0.0, float(distance)) * np.asarray(
            cand['axis'], dtype=float)
        for key in ('origin', 'slot_center', 'P_goal', 'P_front_ref'):
            if key in cand:
                shifted[key] = np.asarray(cand[key], dtype=float) - delta
        shifted['cones'] = [
            np.asarray(cone, dtype=float) - delta
            for cone in (cand.get('cones', []) or [])
        ]
        return shifted

    def reverse_path_samples(self, plan):
        """첫 원호-직선-복귀 원호의 차체 기준 샘플을 만든다."""
        pose = (0.0, 0.0, 0.0)
        yield 'START', pose
        phases = [
            ('IN_ARC', plan['R_in'] * plan['theta'],
             plan['into_steer']),
            ('STRAIGHT', plan['straight_len'], 0),
            ('OUT_ARC', plan['R_out'] * plan['theta'],
             plan['out_steer']),
        ]
        for phase, length, steer in phases:
            count = max(1, int(math.ceil(
                max(0.0, length) / self.swept_path_sample_step)))
            step = max(0.0, length) / float(count)
            for _ in range(count):
                pose = self.advance_reverse_pose(pose, step, steer)
                yield phase, pose

    def swept_pose_clearance(self, cand, pose):
        """한 pose에서 입구와 슬롯의 물리 경계까지 여유를 계산한다."""
        collision, cone_clear = self.footprint_cone_clearance(cand, pose)
        slot_clear = self.footprint_slot_clearance(cand, pose)

        # q_low 쪽은 차로와 연결된 열린 입구이므로 경계로 막지 않는다.
        # 차체 모서리가 슬롯 안(q>=q_low)으로 들어온 부분에 대해서만
        # 앞/뒤 끝벽(s_low/s_high)을 적용한다.
        end_clear = 999.0
        inside_count = 0
        for corner in self.footprint_corners(pose):
            s_coord, q_coord = self.slot_coords(cand, corner)
            if q_coord >= cand['q_low']:
                inside_count += 1
                end_clear = min(
                    end_clear,
                    s_coord - cand['s_low'],
                    cand['s_high'] - s_coord,
                )
        return {
            'cone_collision': collision,
            'cone_clear': cone_clear,
            'inner_clear': slot_clear['inner_clear'],
            'end_clear': end_clear,
            'inside_count': inside_count,
        }

    def validate_swept_path(self, cand, plan):
        """명령을 내리기 전에 후진 전 구간의 차량 footprint를 검사한다."""
        result = {
            'safe': True, 'reason': 'ok',
            'min_cone_clear': 999.0, 'min_inner_clear': 999.0,
            'min_end_clear': 999.0, 'phase': 'START',
            'pose': (0.0, 0.0, 0.0),
        }
        for phase, pose in self.reverse_path_samples(plan):
            clear = self.swept_pose_clearance(cand, pose)
            result['min_cone_clear'] = min(
                result['min_cone_clear'], clear['cone_clear'])
            result['min_inner_clear'] = min(
                result['min_inner_clear'], clear['inner_clear'])
            if clear['inside_count'] > 0:
                result['min_end_clear'] = min(
                    result['min_end_clear'], clear['end_clear'])

            reason = None
            if (clear['cone_collision']
                    or clear['cone_clear']
                    < self.reverse_cone_safety_stop_margin):
                reason = 'entry_obstacle'
            elif clear['inner_clear'] < self.swept_path_boundary_margin:
                reason = 'inner_boundary'
            elif (self.swept_path_end_wall_check_enable
                  and clear['inside_count'] > 0
                  and clear['end_clear']
                  < self.swept_path_boundary_margin):
                reason = 'front_rear_boundary'
            if reason is not None:
                result.update({
                    'safe': False, 'reason': reason,
                    'phase': phase, 'pose': pose,
                })
                return result
        return result

    def maneuver_path_samples(self, segments):
        pose = (0.0, 0.0, 0.0)
        yield 'START', pose
        for index, segment in enumerate(segments):
            length = max(0.0, float(segment['distance']))
            count = max(1, int(math.ceil(
                length / self.swept_path_sample_step)))
            step = length / float(count)
            for _ in range(count):
                pose = self.advance_motion_pose(
                    pose, step, segment['steer'],
                    segment['direction'])
                yield 'M%d' % (index + 1), pose

    def validate_maneuver_path(self, cand, segments):
        result = {
            'safe': True, 'reason': 'ok',
            'min_cone_clear': 999.0, 'min_inner_clear': 999.0,
            'min_end_clear': 999.0, 'phase': 'START',
            'pose': (0.0, 0.0, 0.0),
        }
        for phase, pose in self.maneuver_path_samples(segments):
            clear = self.swept_pose_clearance(cand, pose)
            result['min_cone_clear'] = min(
                result['min_cone_clear'], clear['cone_clear'])
            result['min_inner_clear'] = min(
                result['min_inner_clear'], clear['inner_clear'])
            if clear['inside_count'] > 0:
                result['min_end_clear'] = min(
                    result['min_end_clear'], clear['end_clear'])
            reason = None
            if (clear['cone_collision']
                    or clear['cone_clear']
                    < self.reverse_cone_safety_stop_margin):
                reason = 'entry_obstacle'
            elif clear['inner_clear'] < self.swept_path_boundary_margin:
                reason = 'inner_boundary'
            elif (clear['inside_count'] > 0
                  and clear['end_clear']
                  < self.swept_path_boundary_margin):
                reason = 'front_rear_boundary'
            if reason:
                result.update({
                    'safe': False, 'reason': reason,
                    'phase': phase, 'pose': pose,
                })
                return result
        return result

    def compute_multipoint_plan(self, cand):
        """입구 모서리를 피하는 전진-후진-전진 다단 평행주차 계획."""
        steer = self.max_steer_cmd
        # 우측 슬롯은 먼저 차두를 좌측으로 빼기 위해 전진 음(-)조향,
        # 이후 후진 양(+)조향으로 후미를 슬롯에 넣는다. 좌측은 대칭이다.
        setup_steer = -steer if self.side == 'right' else steer
        entry_steer = -setup_steer
        setup_radius = self.radius_for_cmd(setup_steer)
        entry_radius = self.radius_for_cmd(entry_steer)
        if (not math.isfinite(setup_radius)
                or not math.isfinite(entry_radius)):
            return {'feasible': False, 'reason': 'invalid_radius'}

        best = None
        for setup_deg in np.arange(12.0, 32.1, 2.0):
            setup_yaw = math.radians(float(setup_deg))
            setup_len = setup_radius * setup_yaw
            for entry_deg in np.arange(28.0, 70.1, 2.0):
                entry_yaw = math.radians(float(entry_deg))
                entry_len = entry_radius * entry_yaw
                exit_len = setup_radius * entry_yaw
                align_len = entry_radius * setup_yaw

                fixed = [
                    {'direction': 1, 'steer': setup_steer,
                     'distance': setup_len, 'yaw': setup_yaw,
                     'name': 'SETUP_FORWARD_ARC'},
                    {'direction': -1, 'steer': entry_steer,
                     'distance': entry_len, 'yaw': entry_yaw,
                     'name': 'ENTRY_REVERSE_ARC'},
                ]
                pose = (0.0, 0.0, 0.0)
                for segment in fixed:
                    pose = self.advance_motion_pose(
                        pose, segment['distance'], segment['steer'],
                        segment['direction'])

                reverse_vec = np.array(
                    [-math.cos(pose[2]), -math.sin(pose[2])],
                    dtype=float)
                q_coeff = float(np.dot(reverse_vec, cand['side_vec']))

                tail_zero = [
                    {'direction': -1, 'steer': setup_steer,
                     'distance': exit_len, 'yaw': entry_yaw,
                     'name': 'EXIT_REVERSE_ARC'},
                    {'direction': 1, 'steer': entry_steer,
                     'distance': align_len, 'yaw': setup_yaw,
                     'name': 'ALIGN_FORWARD_ARC'},
                ]
                zero_pose = pose
                for segment in tail_zero:
                    zero_pose = self.advance_motion_pose(
                        zero_pose, segment['distance'], segment['steer'],
                        segment['direction'])
                zero_q = float(np.dot(zero_pose[:2], cand['side_vec']))
                goal_q = float(np.dot(cand['P_goal'], cand['side_vec']))
                if abs(q_coeff) < 1e-4:
                    continue
                middle_len = (goal_q - zero_q) / q_coeff
                if not 0.10 <= middle_len <= 2.50:
                    continue

                segments = fixed + [
                    {'direction': -1, 'steer': 0,
                     'distance': middle_len, 'yaw': 0.0,
                     'name': 'DEEP_REVERSE_STRAIGHT'},
                ] + tail_zero

                pose = (0.0, 0.0, 0.0)
                for segment in segments:
                    pose = self.advance_motion_pose(
                        pose, segment['distance'], segment['steer'],
                        segment['direction'])
                final_heading = np.array(
                    [math.cos(pose[2]), math.sin(pose[2])], dtype=float)
                final_signed = float(np.dot(
                    cand['P_goal'] - np.asarray(pose[:2]), final_heading))
                if abs(final_signed) > 1.00:
                    continue
                final_len = abs(final_signed)
                if final_len > 0.02:
                    final_direction = 1 if final_signed >= 0.0 else -1
                    segments.append({
                        'direction': final_direction, 'steer': 0,
                        'distance': final_len, 'yaw': 0.0,
                        'name': ('CENTER_FORWARD_STRAIGHT'
                                 if final_direction > 0
                                 else 'CENTER_REVERSE_STRAIGHT'),
                    })

                check = self.validate_maneuver_path(cand, segments)
                if not check['safe']:
                    continue
                final_pose = list(self.maneuver_path_samples(segments))[-1][1]
                dist, s_err, q_err, yaw_err = self.slot_errors_for_pose(
                    cand, final_pose)
                final_clear = self.footprint_slot_clearance(
                    cand, final_pose)
                if (abs(s_err) > self.longitudinal_tolerance
                        or abs(q_err) > self.lateral_tolerance
                        or abs(math.degrees(yaw_err))
                        > self.final_yaw_tolerance_deg):
                    continue
                total_len = sum(seg['distance'] for seg in segments)
                score = dist + 0.03 * total_len + 0.002 * (
                    setup_deg + entry_deg)
                option = {
                    'feasible': True, 'segments': segments,
                    'score': score, 'final_pose': final_pose,
                    'dist': dist, 's_err': s_err, 'q_err': q_err,
                    'yaw_err': yaw_err, 'check': check,
                    'final_clear': final_clear,
                }
                if best is None or score < best['score']:
                    best = option
        return best if best is not None else {
            'feasible': False, 'reason': 'no_safe_multipoint_path'}

    def retarget_deep_reverse_from_odometry(self, cand):
        """Recalculate the long diagonal reverse from the stopped real pose.

        The initial plan is intentionally frozen for safety, but small arc
        overshoots accumulate before DEEP_REVERSE_STRAIGHT.  Recalculating
        only this straight segment preserves the already validated maneuver
        shape while removing the measured lateral error.  The final straight
        is also rebuilt because changing the diagonal distance changes the
        longitudinal center.
        """
        index = self.maneuver_segment_index
        if (cand is None or self.maneuver_segments is None
                or not 0 <= index < len(self.maneuver_segments)):
            return False, 'missing_current_segment'
        current = self.maneuver_segments[index]
        if current.get('name') != 'DEEP_REVERSE_STRAIGHT':
            return True, 'not_deep_segment'
        if current.get('feedback_retargeted', False):
            return True, 'already_retargeted'

        tail_core = [
            dict(segment)
            for segment in self.maneuver_segments[index + 1:]
            if not str(segment.get('name', '')).startswith('CENTER_')
        ]

        def evaluate(deep_length):
            segments = [{
                'direction': -1, 'steer': 0,
                'distance': max(0.0, float(deep_length)), 'yaw': 0.0,
                'name': 'DEEP_REVERSE_STRAIGHT',
            }] + tail_core
            pose = (0.0, 0.0, 0.0)
            for segment in segments:
                pose = self.advance_motion_pose(
                    pose, segment['distance'], segment['steer'],
                    segment['direction'])
            heading = np.array(
                [math.cos(pose[2]), math.sin(pose[2])], dtype=float)
            center_signed = float(np.dot(
                np.asarray(cand['P_goal']) - np.asarray(pose[:2]),
                heading))
            final_xy = np.asarray(pose[:2]) + center_signed * heading
            q_error = float(np.dot(
                final_xy - np.asarray(cand['P_goal']),
                np.asarray(cand['side_vec'])))
            return pose, center_signed, q_error

        _, _, q_zero = evaluate(0.0)
        _, _, q_one = evaluate(1.0)
        q_per_meter = q_one - q_zero
        if abs(q_per_meter) < 0.05:
            return False, 'lateral_coefficient_too_small'

        # Latest real run accumulated about +0.09 m additional inward motion
        # after this point.  Aim outward by that calibrated amount so the
        # stopped vehicle, rather than the ideal bicycle model, is centered.
        target_q_error = -self.maneuver_lateral_execution_bias
        deep_length = (target_q_error - q_zero) / q_per_meter
        if not 0.05 <= deep_length <= 2.50:
            return False, 'deep_length_out_of_range=%.3f' % deep_length

        _, center_signed, _ = evaluate(deep_length)
        if abs(center_signed) > 1.00:
            return False, 'center_length_out_of_range=%.3f' % center_signed

        new_deep = dict(current)
        old_length = float(new_deep['distance'])
        new_deep['distance'] = float(deep_length)
        new_deep['feedback_retargeted'] = True
        remaining = [new_deep] + tail_core
        if abs(center_signed) > 0.02:
            remaining.append({
                'direction': 1 if center_signed >= 0.0 else -1,
                'steer': 0,
                'distance': abs(center_signed),
                'yaw': 0.0,
                'name': ('CENTER_FORWARD_STRAIGHT'
                         if center_signed >= 0.0
                         else 'CENTER_REVERSE_STRAIGHT'),
            })

        check = self.validate_maneuver_path(cand, remaining)
        if not check['safe']:
            return False, (
                'retarget_unsafe=%s cone=%.3f inner=%.3f end=%.3f' %
                (check['reason'], check['min_cone_clear'],
                 check['min_inner_clear'], check['min_end_clear']))

        self.maneuver_segments = (
            self.maneuver_segments[:index] + remaining)
        final_pose = list(self.maneuver_path_samples(remaining))[-1][1]
        _, final_s, final_q, final_yaw = self.slot_errors_for_pose(
            cand, final_pose)
        self.logwarn(
            '[PARALLEL_MULTI_FEEDBACK] deep %.3f -> %.3f m '
            'bias=%.3f predicted[s=%.3f q=%.3f yaw=%.1fdeg] '
            'clear[cone=%.3f inner=%.3f end=%.3f]',
            old_length, deep_length,
            self.maneuver_lateral_execution_bias,
            final_s, final_q, math.degrees(final_yaw),
            check['min_cone_clear'], check['min_inner_clear'],
            check['min_end_clear'])
        return True, 'retargeted'

    def exit_pose_check(self, cand, pose):
        """Check that the whole vehicle is outside and parallel to the lane."""
        q_values = [
            self.slot_coords(cand, corner)[1]
            for corner in self.footprint_corners(pose)
        ]
        max_q = max(q_values)
        outside_limit = (
            cand['q_low'] - self.exit_outer_clearance_margin)
        yaw_error = normalize_angle(
            cand['yaw_goal_base'] - pose[2])
        cone_collision, cone_clear = self.footprint_cone_clearance(
            cand, pose)
        ok = (
            max_q <= outside_limit
            and abs(math.degrees(yaw_error))
            <= self.exit_heading_tolerance_deg
            and not cone_collision
            and cone_clear >= self.reverse_cone_safety_stop_margin)
        return ok, {
            'max_q': max_q,
            'outside_limit': outside_limit,
            'yaw_error': yaw_error,
            'cone_clear': cone_clear,
            'cone_collision': cone_collision,
        }

    def build_reverse_handoff_plan(self, cand):
        """Reverse safely inside the slot, then hand control to waypoints."""
        if cand is None:
            return {
                'feasible': False,
                'reason': 'missing_locked_slot',
            }

        start_clear = self.footprint_slot_clearance(
            cand, (0.0, 0.0, 0.0))
        # Reserve both the geometric end margin and measured braking coast.
        # The waypoint controller receives control only after the vehicle has
        # settled at the end of this straight reverse segment.
        reverse_distance = min(
            self.exit_reverse_clear_max,
            max(0.0, start_clear['rear_clear']
                - self.exit_rear_end_margin
                - self.exit_reverse_brake_buffer))
        if reverse_distance < self.exit_reverse_clear_step:
            return {
                'feasible': False,
                'reason': (
                    'insufficient_reverse_clearance rear=%.3f required=%.3f'
                    % (start_clear['rear_clear'],
                       self.exit_rear_end_margin
                       + self.exit_reverse_brake_buffer
                       + self.exit_reverse_clear_step)),
            }

        segments = [{
            'direction': -1,
            'steer': 0,
            'distance': reverse_distance,
            'yaw': 0.0,
            'name': 'EXIT_REVERSE_CLEAR',
        }]
        check = self.validate_maneuver_path(cand, segments)
        if not check['safe']:
            return {
                'feasible': False,
                'reason': (
                    'unsafe_reverse_handoff_%s cone=%.3f inner=%.3f end=%.3f'
                    % (check['reason'], check['min_cone_clear'],
                       check['min_inner_clear'], check['min_end_clear'])),
                'check': check,
            }
        final_pose = list(self.maneuver_path_samples(segments))[-1][1]
        # A reverse-only handoff deliberately remains inside the slot, so the
        # full-exit boolean is ignored. Keep its diagnostics for logging.
        _, ready_info = self.exit_pose_check(cand, final_pose)
        return {
            'feasible': True,
            'strategy': 'reverse_only_waypoint_handoff',
            'segments': segments,
            'check': check,
            'final_pose': final_pose,
            'ready_info': ready_info,
            'reverse_distance': reverse_distance,
            'reverse_steer': 0,
        }

    def build_exit_plan(self, cand):
        """Build a swept-path-validated exit from the *actual* parked pose.

        Prefer the exact inverse only when it remains safe after reprojection.
        Parking execution error can make that inverse skim the entrance cones,
        so a conventional S-curve exit is searched as the safe fallback.
        """
        if cand is None:
            return {
                'feasible': False,
                'reason': 'missing_locked_parking_path',
            }
        if self.exit_reverse_only:
            return self.build_reverse_handoff_plan(cand)
        if not self.maneuver_segments:
            return {
                'feasible': False,
                'reason': 'missing_locked_parking_path',
            }

        inverse_segments = []
        for source in reversed(self.maneuver_segments):
            segment = dict(source)
            segment['direction'] = -int(source['direction'])
            segment['name'] = 'EXIT_RETRACE_' + str(source['name'])
            segment.pop('feedback_retargeted', None)
            inverse_segments.append(segment)

        inverse_check = self.validate_maneuver_path(
            cand, inverse_segments)
        if inverse_check['safe']:
            inverse_final_pose = list(
                self.maneuver_path_samples(inverse_segments))[-1][1]
            inverse_ready, inverse_ready_info = self.exit_pose_check(
                cand, inverse_final_pose)
            if inverse_ready:
                return {
                    'feasible': True,
                    'strategy': 'validated_retrace',
                    'segments': inverse_segments,
                    'check': inverse_check,
                    'final_pose': inverse_final_pose,
                    'ready_info': inverse_ready_info,
                }

        # Right-side slot: first forward arc turns the nose left (negative
        # command with this vehicle calibration), then positive counter-steer.
        # Mirror both commands for a left-side slot.
        away_steer = (
            -self.max_steer_cmd
            if self.side == 'right' else self.max_steer_cmd)
        align_steer = -away_steer
        away_radius = self.radius_for_cmd(away_steer)
        align_radius = self.radius_for_cmd(align_steer)
        if (not math.isfinite(away_radius)
                or not math.isfinite(align_radius)):
            return {
                'feasible': False,
                'reason': 'invalid_exit_steering_radius',
                'check': inverse_check,
            }

        start_clear = self.footprint_slot_clearance(
            cand, (0.0, 0.0, 0.0))
        max_reverse = min(
            self.exit_reverse_clear_max,
            max(0.0, start_clear['rear_clear']
                - self.exit_rear_end_margin))
        reverse_distances = [0.0]
        if max_reverse >= self.exit_reverse_clear_step:
            reverse_distances.extend(
                float(value) for value in np.arange(
                    self.exit_reverse_clear_step,
                    max_reverse + 0.5 * self.exit_reverse_clear_step,
                    self.exit_reverse_clear_step))
            if abs(reverse_distances[-1] - max_reverse) > 1e-3:
                reverse_distances.append(max_reverse)
        # Start with the largest validated rearward clearance.  It gives the
        # front overhang the most room to pass the entrance cone.
        reverse_distances = sorted(set(reverse_distances), reverse=True)

        best = None
        first_failure = None
        desired_yaw = normalize_angle(cand['yaw_goal_base'])
        correction_sign = 1 if desired_yaw >= 0.0 else -1
        if abs(math.degrees(desired_yaw)) >= 0.5:
            reverse_steer_magnitudes = list(range(
                self.exit_reverse_align_max_steer, -1, -1))
        else:
            reverse_steer_magnitudes = [0] + list(range(
                self.exit_reverse_align_max_steer, 0, -1))
        away_turn_sign = (
            self.forward_turn_sign
            * (1.0 if away_steer >= 0 else -1.0))

        # The vehicle can finish parking a few degrees off the slot axis.
        # Correct that heading during the initial low-speed reverse, then use
        # independently sized forward arcs.  Solving the second angle from
        # the signed yaw equation returns exactly to the locked slot heading:
        #   desired = reverse_yaw + A * (away_angle - align_angle)
        for reverse_distance in reverse_distances:
            for reverse_steer_mag in reverse_steer_magnitudes:
                reverse_steer = correction_sign * reverse_steer_mag
                if reverse_steer_mag == 0:
                    reverse_yaw_signed = 0.0
                else:
                    reverse_radius = self.radius_for_cmd(reverse_steer)
                    if not math.isfinite(reverse_radius):
                        continue
                    reverse_yaw_signed = (
                        correction_sign * reverse_distance
                        / reverse_radius)
                combo_best = None
                for away_arc_deg in np.arange(
                        self.exit_arc_min_deg,
                        self.exit_arc_max_deg
                        + 0.5 * self.exit_arc_step_deg,
                        self.exit_arc_step_deg):
                    align_arc_deg = (
                        float(away_arc_deg)
                        + math.degrees(away_turn_sign * (
                            reverse_yaw_signed - desired_yaw)))
                    if (align_arc_deg < self.exit_arc_min_deg
                            or align_arc_deg
                            > self.exit_arc_max_deg + 20.0):
                        continue
                    away_arc_yaw = math.radians(float(away_arc_deg))
                    align_arc_yaw = math.radians(align_arc_deg)
                    segments = []
                    if reverse_distance > 0.01:
                        segments.append({
                            'direction': -1,
                            'steer': reverse_steer,
                            'distance': reverse_distance,
                            'yaw': abs(reverse_yaw_signed),
                            'name': (
                                'EXIT_REVERSE_HEADING_ALIGN'
                                if reverse_steer_mag > 0
                                else 'EXIT_REVERSE_CLEAR'),
                        })
                    segments.extend([
                        {
                            'direction': 1,
                            'steer': away_steer,
                            'distance': away_radius * away_arc_yaw,
                            'yaw': away_arc_yaw,
                            'name': 'EXIT_FORWARD_AWAY_ARC',
                        },
                        {
                            'direction': 1,
                            'steer': align_steer,
                            'distance': align_radius * align_arc_yaw,
                            'yaw': align_arc_yaw,
                            'name': 'EXIT_FORWARD_ALIGN_ARC',
                        },
                    ])
                    check = self.validate_maneuver_path(cand, segments)
                    if not check['safe']:
                        if first_failure is None:
                            first_failure = check
                        continue
                    final_pose = list(
                        self.maneuver_path_samples(segments))[-1][1]
                    ready, ready_info = self.exit_pose_check(
                        cand, final_pose)
                    if not ready:
                        continue
                    total_distance = sum(
                        float(segment['distance'])
                        for segment in segments)
                    score = (
                        total_distance
                        + 0.50 * reverse_distance
                        + 0.05 * abs(math.degrees(
                            ready_info['yaw_error'])))
                    option = {
                        'score': score,
                        'segments': segments,
                        'check': check,
                        'final_pose': final_pose,
                        'ready_info': ready_info,
                        'reverse_distance': reverse_distance,
                        'reverse_steer': reverse_steer,
                        'away_arc_deg': float(away_arc_deg),
                        'align_arc_deg': align_arc_deg,
                    }
                    if (combo_best is None
                            or score < combo_best['score']):
                        combo_best = option
                    if (check['min_cone_clear']
                            >= self.exit_preferred_clearance
                            and check['min_end_clear']
                            >= self.exit_preferred_clearance):
                        best = option
                        break
                if best is not None:
                    break
                # A safe solution in this reverse pose is preferable to a
                # long exhaustive search.  The collision model already
                # inflates the vehicle footprint by collision_margin.
                if combo_best is not None:
                    best = combo_best
                    break
            if best is not None:
                break

        if best is None:
            inverse_reason = (
                'unsafe_%s cone=%.3f inner=%.3f end=%.3f' %
                (inverse_check['reason'],
                 inverse_check['min_cone_clear'],
                 inverse_check['min_inner_clear'],
                 inverse_check['min_end_clear']))
            search_reason = 'no_s_curve_solution'
            if first_failure is not None:
                search_reason += (
                    ' first=%s cone=%.3f inner=%.3f end=%.3f' %
                    (first_failure['reason'],
                     first_failure['min_cone_clear'],
                     first_failure['min_inner_clear'],
                     first_failure['min_end_clear']))
            return {
                'feasible': False,
                'reason': '%s; %s' % (inverse_reason, search_reason),
                'check': inverse_check,
            }

        return {
            'feasible': True,
            'strategy': 'searched_heading_corrected_s_curve',
            'segments': best['segments'],
            'check': best['check'],
            'final_pose': best['final_pose'],
            'ready_info': best['ready_info'],
            'reverse_distance': best['reverse_distance'],
            'reverse_steer': best['reverse_steer'],
            'away_arc_deg': best['away_arc_deg'],
            'align_arc_deg': best['align_arc_deg'],
        }

    def enter_parked(self, info):
        self.state = 'PARKED'
        self.done = False
        self.parked_start_time = self.now()
        self.exit_requested = False
        self.publish_zero(
            active=True, mapping=False, done=False, stop_req=True)
        self.pub_parked.publish(Bool(data=True))
        self.pub_exit_done.publish(Bool(data=False))
        self.logwarn(
            '[PARALLEL_SEQ] FINAL_ALIGN -> PARKED '
            's=%.3f q=%.3f yaw=%.1f '
            'clear[f=%.3f r=%.3f in=%.3f out=%.3f] '
            'auto_exit=%s hold=%.1fs',
            info['s_err'], info['q_err'],
            math.degrees(info['yaw_err']),
            info['front_clear'], info['rear_clear'],
            info['inner_clear'], info['outer_clear'],
            self.auto_exit, self.parking_hold_sec)

    def retarget_exit_after_reverse_feedback(
            self, cand, reverse_segment, actual_turned):
        """Replan both forward arcs from the measured reverse end pose.

        A low steering command has a noticeably larger real turning radius
        than the linear command-to-angle model.  The reverse segment must be
        distance-capped near the rear boundary.  From that actual pose,
        search the two forward arc angles again and validate their complete
        swept path instead of merely correcting the last angle.
        """
        if not self.exit_segments:
            return False, 'missing_exit_segments'
        away_index = None
        align_index = None
        # When feedback is deferred until SETTLE, the completed reverse
        # segment has already advanced exit_segment_index from 0 to 1.  Start
        # at that current index so the first forward-away arc is not skipped.
        for index in range(
                self.exit_segment_index,
                len(self.exit_segments)):
            name = self.exit_segments[index].get('name')
            if name == 'EXIT_FORWARD_AWAY_ARC':
                away_index = index
            elif (name
                    == 'EXIT_FORWARD_ALIGN_ARC'):
                align_index = index
                break
        if away_index is None or align_index is None:
            return False, 'missing_exit_forward_arcs'

        old_away = self.exit_segments[away_index]
        old_align = self.exit_segments[align_index]
        away_radius = self.radius_for_cmd(old_away['steer'])
        align_radius = self.radius_for_cmd(old_align['steer'])
        away_steer_sign = (
            1.0 if old_away['steer'] >= 0 else -1.0)
        away_turn_sign = (
            self.forward_turn_sign * away_steer_sign)
        if (not math.isfinite(away_radius)
                or not math.isfinite(align_radius)
                or abs(away_turn_sign) < 1e-6):
            return False, 'invalid_feedback_arc_geometry'

        old_away_yaw = float(old_away['yaw'])
        old_align_yaw = float(old_align['yaw'])
        search_low = max(
            math.radians(self.exit_arc_min_deg),
            old_away_yaw - self.exit_feedback_max_adjust)
        search_high = min(
            math.radians(self.exit_arc_max_deg),
            old_away_yaw + self.exit_feedback_max_adjust)
        best = None
        first_failure = None
        step = math.radians(self.exit_arc_step_deg)
        for away_yaw in np.arange(
                search_low, search_high + 0.5 * step, step):
            # Starting from the current pose, require:
            # desired = A*away - A*align.
            align_yaw = (
                float(away_yaw)
                - cand['yaw_goal_base'] / away_turn_sign)
            if align_yaw < math.radians(5.0):
                continue
            away_segment = dict(old_away)
            align_segment = dict(old_align)
            away_segment['yaw'] = float(away_yaw)
            away_segment['distance'] = away_radius * float(away_yaw)
            away_segment['feedback_retargeted'] = True
            align_segment['yaw'] = align_yaw
            align_segment['distance'] = align_radius * align_yaw
            align_segment['feedback_retargeted'] = True
            remaining = [away_segment, align_segment]
            check = self.validate_maneuver_path(cand, remaining)
            if not check['safe']:
                if first_failure is None:
                    first_failure = check
                continue
            final_pose = list(
                self.maneuver_path_samples(remaining))[-1][1]
            ready, ready_info = self.exit_pose_check(
                cand, final_pose)
            if not ready:
                continue
            option = {
                'away': away_segment,
                'align': align_segment,
                'check': check,
                'ready_info': ready_info,
                'distance': (away_segment['distance']
                             + align_segment['distance']),
            }
            if best is None or option['distance'] < best['distance']:
                best = option
            if (check['min_cone_clear']
                    >= self.exit_preferred_clearance
                    and check['min_end_clear']
                    >= self.exit_preferred_clearance):
                best = option
                break

        if best is None:
            reason = 'no_safe_forward_replan'
            if first_failure is not None:
                reason += (
                    ' first=%s cone=%.3f inner=%.3f end=%.3f' %
                    (first_failure['reason'],
                     first_failure['min_cone_clear'],
                     first_failure['min_inner_clear'],
                     first_failure['min_end_clear']))
            return False, reason
        self.exit_segments[away_index] = best['away']
        self.exit_segments[align_index] = best['align']
        check = best['check']
        self.logwarn(
            '[PARALLEL_EXIT_FEEDBACK] reverse yaw %.1f/%.1fdeg '
            'away %.1f -> %.1fdeg align %.1f -> %.1fdeg remaining_clear'
            '[cone=%.3f inner=%.3f end=%.3f]',
            math.degrees(actual_turned),
            math.degrees(reverse_segment['yaw']),
            math.degrees(old_away_yaw),
            math.degrees(best['away']['yaw']),
            math.degrees(old_align_yaw),
            math.degrees(best['align']['yaw']),
            check['min_cone_clear'], check['min_inner_clear'],
            check['min_end_clear'])
        return True, 'retargeted'

    def complete_exit(self, info):
        self.state = 'DONE'
        self.done = True
        # Release both the parking stop request and parking ownership.  A
        # waypoint command source may take control only after this point.
        self.publish_zero(
            active=False, mapping=False, done=True, stop_req=False)
        self.pub_parked.publish(Bool(data=False))
        self.pub_exit_done.publish(Bool(data=True))
        self.logwarn(
            '[PARALLEL_EXIT] %s -> DONE/HANDOFF '
            'q=%.3f/%.3f yaw=%.1f cone=%.3f',
            ('REVERSE_CLEAR_COMPLETE'
             if self.exit_reverse_only else 'EXIT_COMPLETE'),
            info['max_q'], info['outside_limit'],
            math.degrees(info['yaw_error']), info['cone_clear'])

    def done_check(self, cand):
        if cand is None:
            return False, {}
        dist, s_err, q_err, yaw_err = self.slot_errors_for_pose(cand, (0.0, 0.0, 0.0))
        clear = self.footprint_slot_clearance(cand, (0.0, 0.0, 0.0))
        cone_collision, cone_clear = self.footprint_cone_clearance(cand, (0.0, 0.0, 0.0))
        ok = (dist <= self.goal_tolerance
              and abs(s_err) <= self.longitudinal_tolerance
              and abs(q_err) <= self.lateral_tolerance
              and abs(math.degrees(yaw_err)) <= self.final_yaw_tolerance_deg
              and abs(self.wz) <= self.final_wz_tolerance
              and clear['front_clear'] >= self.end_safety_margin
              and clear['rear_clear'] >= self.end_safety_margin
              and clear['inner_clear'] >= self.side_safety_margin
              and not cone_collision
              and cone_clear >= self.cone_safety_stop_margin)
        clear.update({'dist': dist, 's_err': s_err, 'q_err': q_err, 'yaw_err': yaw_err})
        clear.update({'cone_collision': cone_collision, 'cone_clear': cone_clear})
        return ok, clear

    def forward_segment_escapes_end_boundary(
            self, cand, segment, current_end_clear):
        """Allow a forward command only when it increases rear clearance."""
        if segment is None or int(segment.get('direction', 0)) <= 0:
            return False
        previous = current_end_clear
        for distance in (0.03, 0.06, 0.10):
            pose = self.advance_motion_pose(
                (0.0, 0.0, 0.0), distance,
                segment['steer'], segment['direction'])
            clear = self.swept_pose_clearance(cand, pose)
            if (clear['cone_collision']
                    or clear['cone_clear']
                    <= self.reverse_cone_safety_stop_margin
                    or clear['inner_clear']
                    <= self.swept_path_boundary_margin):
                return False
            if clear['inside_count'] > 0:
                if clear['end_clear'] < previous - 0.005:
                    return False
                previous = clear['end_clear']
        return previous > current_end_clear + 0.005

    def reverse_safety_stop(self, cand, phase, segment=None):
        collision, cone_clear = self.footprint_cone_clearance(
            cand, (0.0, 0.0, 0.0))
        swept_clear = self.swept_pose_clearance(
            cand, (0.0, 0.0, 0.0))
        margin = self.reverse_cone_safety_stop_margin
        cone_unsafe = collision or cone_clear <= margin
        inner_unsafe = (
            swept_clear['inner_clear']
            <= self.swept_path_boundary_margin)
        end_unsafe = (
            self.swept_path_end_wall_check_enable
            and swept_clear['inside_count'] > 0
            and swept_clear['end_clear']
            <= self.swept_path_boundary_margin)
        escaping_end = (
            end_unsafe and not cone_unsafe and not inner_unsafe
            and self.forward_segment_escapes_end_boundary(
                cand, segment, swept_clear['end_clear']))
        too_close = cone_unsafe or inner_unsafe or (
            end_unsafe and not escaping_end)
        if escaping_end:
            self.logwarn_throttle(
                0.5,
                '[PARALLEL_EXIT_ESCAPE] allow forward motion away from '
                'rear boundary end=%.3f steer=%d',
                swept_clear['end_clear'], segment['steer'])
        if too_close:
            self.publish_stop()
            # cone/end/inner each check against a *different* margin - see
            # this function's own cone_unsafe/inner_unsafe/end_unsafe above
            # (2026-08-12 fix: this used to print a single shared `margin`
            # value for all three, which only ever matched the cone check
            # and made inner/end aborts look like they'd cleared their real
            # threshold).
            self.abort(
                '%s safety stop cone=%.3f/%.3f end=%.3f/%.3f inner=%.3f/%.3f '
                'collision=%s' %
                (phase, cone_clear, margin,
                 swept_clear['end_clear'], self.swept_path_boundary_margin,
                 swept_clear['inner_clear'], self.swept_path_boundary_margin,
                 collision))
            return True
        return False

    def maneuver_speed_command(self, segment, moved, turned,
                               exit_mode=False):
        """Select cruise/slow RPM from remaining distance and current speed."""
        direction = 1 if segment['direction'] > 0 else -1
        if exit_mode:
            cruise = abs(
                self.exit_forward_rpm
                if direction > 0 else self.exit_reverse_rpm)
        else:
            cruise = abs(
                self.maneuver_forward_rpm
                if direction > 0 else self.maneuver_reverse_rpm)
        cruise = max(1, cruise)

        if segment['steer'] == 0:
            remaining = max(0.0, segment['distance'] - moved)
            slow_window = max(
                self.maneuver_straight_slowdown_min,
                abs(self.vx) * self.maneuver_decel_time_sec
                + self.maneuver_straight_slowdown_margin)
            unit = 'm'
        else:
            remaining = max(0.0, segment['yaw'] - turned)
            slow_window = max(
                self.maneuver_arc_slowdown_min,
                abs(self.wz) * self.maneuver_decel_time_sec
                + self.maneuver_arc_slowdown_margin)
            unit = 'rad'

        slowing = (
            cruise > self.maneuver_slow_rpm
            and remaining <= slow_window)
        magnitude = (
            self.maneuver_slow_rpm if slowing else cruise)
        return direction * magnitude, slowing, remaining, slow_window, unit

    # ---------------- command / visualization ----------------
    def approach_cmd(self):
        if self.pre_straight_use_imu and self.imu_received:
            measured_yaw = self.imu_yaw
            measured_wz = self.imu_wz
            reference_yaw = self.pre_ref_imu_yaw
            source = 'imu'
        else:
            measured_yaw = self.yaw
            measured_wz = self.wz
            reference_yaw = self.pre_ref_yaw
            source = 'odom'

        yaw_err = normalize_angle(reference_yaw - measured_yaw)
        wz_err = -measured_wz
        control_sign = self.pre_straight_steer_sign * self.forward_turn_sign

        now_sec = self.tsec()
        if self.pre_heading_control_time is None:
            dt_sec = 0.0
        else:
            dt_sec = clamp(
                now_sec - self.pre_heading_control_time, 0.0, 0.10)
        self.pre_heading_control_time = now_sec

        # steer=0에서도 한쪽으로 흐르는 기계적 중립 편차는 적분항이
        # 비영점 조향 trim으로 학습한다. 작은 yaw 오차에서 출력을 0으로
        # 강제하지 않아 시작 IMU heading을 계속 유지한다.
        self.pre_heading_i_cmd = clamp(
            self.pre_heading_i_cmd
            + control_sign * self.pre_straight_yaw_ki * yaw_err * dt_sec,
            -self.pre_straight_i_limit,
            self.pre_straight_i_limit,
        )
        raw = (
            control_sign * (
                self.pre_straight_yaw_kp * yaw_err
                + self.pre_straight_wz_kd * wz_err)
            + self.pre_heading_i_cmd
        )
        raw = clamp(
            raw, -self.pre_straight_max_steer,
            self.pre_straight_max_steer)
        if (abs(yaw_err) > self.pre_straight_yaw_deadband
                and 0.0 < abs(raw) < self.pre_straight_min_steer):
            raw = math.copysign(float(self.pre_straight_min_steer), raw)
        steer = int(round(raw))
        self.last_heading_error = yaw_err
        self.last_heading_source = source
        return self.pre_straight_rpm, steer

    def approach_heading_unsafe(self):
        return (self.pre_straight_abort_yaw > 0.0
                and abs(self.last_heading_error)
                > self.pre_straight_abort_yaw)

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

    def publish_flags(self, active=False, mapping=False, done=False,
                      stop_req=False):
        self.pub_active.publish(Bool(data=bool(active)))
        self.pub_mapping.publish(Bool(data=bool(mapping)))
        self.pub_done.publish(Bool(data=bool(done)))
        self.pub_stop_req.publish(Bool(data=bool(stop_req)))

    def publish_zero(self, active=False, mapping=False, done=False, stop_req=False):
        self.publish_cmd(0, 0, 0 if done else 1)
        self.publish_flags(active, mapping, done, stop_req)

    def status(self, text):
        self.pub_status.publish(String(data=str(text)))

    def publish_cones(self, cones):
        pa = PoseArray()
        pa.header.stamp = self.now().to_msg()
        pa.header.frame_id = self.base_frame
        for c in cones or []:
            p = Pose()
            p.position.x = float(c[0])
            p.position.y = float(c[1])
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_cones.publish(pa)

    def marker_base(self, ns, mid, mtype):
        m = Marker()
        m.header.stamp = self.now().to_msg()
        m.header.frame_id = self.base_frame
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def add_sphere(self, ma, mid, p, rgb, scale, ns):
        m = self.marker_base(ns, mid, Marker.SPHERE)
        m.pose.position.x = float(p[0])
        m.pose.position.y = float(p[1])
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 1.0
        ma.markers.append(m)

    def add_line(self, ma, mid, pts, rgb, width, ns):
        m = self.marker_base(ns, mid, Marker.LINE_STRIP)
        m.scale.x = width
        m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 1.0
        for p in pts:
            q = Point()
            q.x = float(p[0])
            q.y = float(p[1])
            m.points.append(q)
        ma.markers.append(m)

    def publish_markers(self, cones, cand):
        ma = MarkerArray()
        mid = 0
        for c in cones or []:
            self.add_sphere(ma, mid, c, (1.0, 0.55, 0.0), 0.055, 'cones')
            mid += 1
        if cand is not None:
            self.add_sphere(ma, mid, cand['P_goal'], (0.1, 0.9, 0.2), 0.075, 'goal')
            mid += 1
            a0 = cand['origin'] + cand['s_low'] * cand['axis'] + cand['q_low'] * cand['side_vec']
            a1 = cand['origin'] + cand['s_high'] * cand['axis'] + cand['q_low'] * cand['side_vec']
            b1 = cand['origin'] + cand['s_high'] * cand['axis'] + cand['q_high'] * cand['side_vec']
            b0 = cand['origin'] + cand['s_low'] * cand['axis'] + cand['q_high'] * cand['side_vec']
            self.add_line(ma, mid, [a0, a1, b1, b0, a0], (0.2, 0.8, 1.0), 0.02, 'slot')
        self.pub_markers.publish(ma)

    def publish_goal(self, cand):
        if cand is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.now().to_msg()
        msg.header.frame_id = self.odom_frame
        p = self.base_to_odom(cand['P_goal'])
        msg.pose.position.x = float(p[0])
        msg.pose.position.y = float(p[1])
        yaw_odom = normalize_angle(self.yaw + cand['yaw_goal_base'])
        q = quat_from_yaw(yaw_odom)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        self.pub_goal.publish(msg)

    # ---------------- main loop ----------------
    def run(self):
        # rclpy Rate.sleep()은 executor가 같은 스레드에서 spin_once 되는 이
        # 구조에서 첫 주기 뒤 깨어나지 못할 수 있다. ROS1 루프를 그대로
        # 이식한 형태이므로 wall-clock 주기로 spin_once와 제어를 반복한다.
        period = 1.0 / max(self.cmd_rate, 1.0)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            now = self.now()

            if self.state == 'IDLE':
                # Do not overwrite waypoint /cmd_* while parking is idle.
                self.publish_flags(
                    active=False, mapping=False, done=False,
                    stop_req=False)
                self.pub_parked.publish(Bool(data=False))
                self.pub_exit_done.publish(Bool(data=False))
                self.status('IDLE: waiting /parking_start')

            elif self.state == 'APPROACH':
                self.pub_stop_req.publish(Bool(data=bool(False)))
                # mapping = "still searching, don't trust this node's own
                # controller yet" - was hardcoded True the whole APPROACH
                # state regardless of lock status (2026-08-11 bug, same
                # family as t_parking's identical fix earlier this
                # session): once locked, this state drives itself toward
                # the plan's stop point via approach_cmd() below, but with
                # mapping stuck True control_arbiter kept driving via GPS
                # straight through all of that instead of relaying this
                # node's own commands. active must flip True at the same
                # time - mapping=False with nobody driving is worse than
                # either alone.
                self.pub_mapping.publish(Bool(data=bool(self.locked_slot is None)))
                self.pub_active.publish(Bool(data=bool(self.locked_slot is not None)))
                if self.approach_start_time and self.pre_straight_timeout_sec > 0.0:
                    if self.dt(now, self.approach_start_time) >= self.pre_straight_timeout_sec and self.locked_slot is None:
                        self.abort('APPROACH timeout: slot not locked')
                        time.sleep(period)
                        continue
                if self.locked_slot is not None:
                    cand = self.reproject_locked_slot()
                    self.plan = self.compute_parallel_plan(cand)
                    stop_point, stop_ref = self.approach_stop_point(cand)
                    stop_along = float(np.dot(stop_point, cand['axis']))
                    front_target_along = self.plan['front_target_along']
                    target_along = self.plan['target_along']
                    moved_after_lock = self.approach_distance_after_lock()
                    front_reached = stop_along <= front_target_along + self.approach_stop_tol
                    reached = self.plan['feasible']
                    too_far = moved_after_lock >= self.approach_after_lock_max_dist
                    if reached:
                        self.logwarn(
                            '[PARALLEL_SEQ] APPROACH stop ref=%s front=%.3f/%.3f '
                            'goal=%.3f target=%.3f err=%.3f moved=%.3f status=%s',
                            stop_ref, stop_along, front_target_along,
                            self.plan['goal_along'], target_along,
                            self.plan['plan_error'], moved_after_lock,
                            self.plan['status'])
                        self.go_settle('REVERSE_IN_ARC')
                    elif self.plan['status'] == 'unsafe_swept_path':
                        self.maneuver_plan = self.compute_multipoint_plan(cand)
                        if self.maneuver_plan.get('feasible', False):
                            self.logwarn(
                                '[PARALLEL_MULTI] safe plan found segments=%d '
                                'final=(%.3f,%.3f,%.1fdeg) '
                                'clear[cone=%.3f inner=%.3f end=%.3f] '
                                'center[f=%.3f r=%.3f in=%.3f out=%.3f]',
                                len(self.maneuver_plan['segments']),
                                self.maneuver_plan['s_err'],
                                self.maneuver_plan['q_err'],
                                math.degrees(self.maneuver_plan['yaw_err']),
                                self.maneuver_plan['check']['min_cone_clear'],
                                self.maneuver_plan['check']['min_inner_clear'],
                                self.maneuver_plan['check']['min_end_clear'],
                                self.maneuver_plan['final_clear']['front_clear'],
                                self.maneuver_plan['final_clear']['rear_clear'],
                                self.maneuver_plan['final_clear']['inner_clear'],
                                self.maneuver_plan['final_clear']['outer_clear'])
                            self.maneuver_segments = None
                            self.go_settle('MANEUVER')
                        else:
                            self.abort(
                                'APPROACH no safe one-shot or multipoint path')
                            time.sleep(period)
                            continue
                    elif too_far:
                        self.abort('APPROACH max_dist before valid reverse start '
                                   'goal=%.3f target=%.3f err=%.3f status=%s' %
                                   (self.plan['goal_along'], target_along,
                                    self.plan['plan_error'], self.plan['status']))
                        time.sleep(period)
                        continue
                    else:
                        rpm, steer = self.approach_cmd()
                        if self.approach_heading_unsafe():
                            self.abort(
                                'APPROACH heading safety stop source=%s '
                                'error=%.1fdeg limit=%.1fdeg' %
                                (self.last_heading_source,
                                 math.degrees(self.last_heading_error),
                                 math.degrees(self.pre_straight_abort_yaw)))
                            time.sleep(period)
                            continue
                        self.publish_cmd(rpm, steer, 1)
                        self.loginfo_throttle(
                            1.0,
                            '[PARALLEL_APPROACH] ref=%s front=%.3f/%.3f front_ok=%s '
                            'goal=%.3f target=%.3f err=%.3f status=%s moved=%.3f '
                            'heading[%s]=%.2fdeg i=%.2f steer=%d',
                            stop_ref, stop_along, front_target_along, front_reached,
                            self.plan['goal_along'], target_along,
                            self.plan['plan_error'], self.plan['status'],
                            moved_after_lock, self.last_heading_source,
                            math.degrees(self.last_heading_error),
                            self.pre_heading_i_cmd, steer)
                        self.status(
                            'APPROACH(locked): ref=%s front=%.3f/%.3f front_ok=%s '
                            'goal=%.3f target=%.3f err=%.3f status=%s moved=%.3f steer=%d' %
                            (stop_ref, stop_along, front_target_along, front_reached,
                             self.plan['goal_along'], target_along,
                             self.plan['plan_error'], self.plan['status'],
                             moved_after_lock, steer))
                else:
                    rpm, steer = self.approach_cmd()
                    if self.approach_heading_unsafe():
                        self.abort(
                            'APPROACH heading safety stop source=%s '
                            'error=%.1fdeg limit=%.1fdeg' %
                            (self.last_heading_source,
                             math.degrees(self.last_heading_error),
                             math.degrees(self.pre_straight_abort_yaw)))
                        time.sleep(period)
                        continue
                    self.publish_cmd(rpm, steer, 1)
                    self.loginfo_throttle(
                        1.0,
                        '[PARALLEL_HEADING] source=%s error=%.2fdeg '
                        'i=%.2f cmd=(%d,%d)',
                        self.last_heading_source,
                        math.degrees(self.last_heading_error),
                        self.pre_heading_i_cmd, rpm, steer)
                    self.status(
                        'APPROACH(scan): heading[%s]=%.2fdeg i=%.2f '
                        'steer=%d waiting slot lock' %
                        (self.last_heading_source,
                         math.degrees(self.last_heading_error),
                         self.pre_heading_i_cmd, steer))

            elif self.state == 'SETTLE':
                self.pub_stop_req.publish(Bool(data=bool(True)))
                # See APPROACH's comment (2026-08-11) - SETTLE is only ever
                # entered post-lock, so mapping/active should already be
                # False/True from there, but republish explicitly rather
                # than relying on inherited state (matches t_parking's
                # SETUP_ARC/REVERSE_ARC/REVERSE_STRAIGHT fix).
                self.pub_mapping.publish(Bool(data=bool(False)))
                self.pub_active.publish(Bool(data=bool(True)))
                self.publish_stop()
                if self.settle_start is None:
                    self.settle_start = now
                settle_elapsed = self.dt(now, self.settle_start)
                vx_settled = abs(self.vx) < self.stop_speed_thresh
                settle_timed_out = settle_elapsed >= self.settle_timeout_sec
                if settle_elapsed >= self.stop_hold_sec and (vx_settled or settle_timed_out):
                    if settle_timed_out and not vx_settled:
                        self.logwarn(
                            '[PARALLEL_SETTLE] timeout %.1fs forcing advance '
                            'to %s despite vx=%.3f (slope creep?)',
                            settle_elapsed, self.next_state, self.vx)
                    if self.next_state == 'REVERSE_IN_ARC':
                        cand = self.reproject_locked_slot()
                        if cand is None:
                            self.abort('SETTLE: no locked slot for final plan')
                            time.sleep(period)
                            continue
                        self.plan = self.compute_parallel_plan(cand)
                        if not self.plan['feasible']:
                            self.abort('SETTLE: no calculated reverse path dY=%.3f status=%s' %
                                       (self.plan['delta_y'], self.plan['status']))
                            time.sleep(period)
                            continue
                        self.logwarn(
                            '[PARALLEL_PLAN_FINAL] ref=%s stop=%.3f target=%.3f '
                            'goal=%.3f back=%.3f pred=%.3f err=%.3f '
                            'dY=%.3f R=(%.3f,%.3f) steer=(%d,%d) theta=%.1fdeg '
                            'straight=%.3f status=%s exact=%s '
                            'swept=%s clear[cone=%.3f inner=%.3f end=%.3f]',
                            self.plan['stop_ref'], self.plan['stop_along'],
                            self.plan['target_along'], self.plan['goal_along'],
                            self.plan['back_needed'], self.plan['pred_back'],
                            self.plan['plan_error'], self.plan['delta_y'],
                            self.plan['R_in'], self.plan['R_out'],
                            self.plan['into_steer'], self.plan['out_steer'],
                            math.degrees(self.plan['into_yaw']),
                            self.plan['straight_len'], self.plan['status'],
                            self.plan['exact'], self.plan['swept_safe'],
                            self.plan['swept_min_cone_clear'],
                            self.plan['swept_min_inner_clear'],
                            self.plan['swept_min_end_clear'])
                    elif self.next_state == 'MANEUVER':
                        if self.maneuver_segments is None:
                            cand = self.reproject_locked_slot()
                            self.maneuver_plan = self.compute_multipoint_plan(
                                cand) if cand is not None else None
                            if (not self.maneuver_plan
                                    or not self.maneuver_plan.get(
                                        'feasible', False)):
                                self.abort(
                                    'SETTLE: no safe multipoint path')
                                time.sleep(period)
                                continue
                            self.maneuver_segments = (
                                self.maneuver_plan['segments'])
                            self.maneuver_segment_index = 0
                            self.logwarn(
                                '[PARALLEL_MULTI_FINAL] segments=%s',
                                ', '.join(
                                    '%s:%.2fm' %
                                    (seg['name'], seg['distance'])
                                    for seg in self.maneuver_segments))
                        if (self.maneuver_segments is not None
                                and self.maneuver_segment_index
                                < len(self.maneuver_segments)
                                and self.maneuver_segments[
                                    self.maneuver_segment_index].get(
                                        'name')
                                == 'DEEP_REVERSE_STRAIGHT'):
                            cand = self.reproject_locked_slot()
                            retarget_ok, retarget_reason = (
                                self.retarget_deep_reverse_from_odometry(
                                    cand))
                            if not retarget_ok:
                                self.abort(
                                    'SETTLE: deep reverse feedback failed: %s'
                                    % retarget_reason)
                                time.sleep(period)
                                continue
                    elif (self.next_state == 'EXIT_MANEUVER'
                          and self.exit_feedback_pending_segment
                          is not None):
                        cand = self.reproject_locked_slot()
                        start_pose = self.exit_feedback_start_pose
                        pending = self.exit_feedback_pending_segment
                        if cand is None or start_pose is None:
                            self.abort(
                                'SETTLE: missing exit feedback pose')
                            time.sleep(period)
                            continue
                        steer_sign = (
                            1.0 if pending['steer'] >= 0 else -1.0)
                        actual_turned = steer_sign * normalize_angle(
                            self.yaw - start_pose[2])
                        retarget_ok, retarget_reason = (
                            self.retarget_exit_after_reverse_feedback(
                                cand, pending, actual_turned))
                        if not retarget_ok:
                            self.abort(
                                'SETTLE: exit feedback failed: %s' %
                                retarget_reason)
                            time.sleep(period)
                            continue
                        self.exit_feedback_pending_segment = None
                        self.exit_feedback_start_pose = None
                    self.phase_start_time = now
                    self.phase_start_pose = (self.x, self.y, self.yaw)
                    self.settle_start = None
                    self.state = self.next_state
                    self.logwarn('[PARALLEL_SEQ] SETTLE -> %s', self.state)
                else:
                    self.status(
                        'SETTLE: vx=%.3f timeout=%.1f/%.1fs -> %s' %
                        (self.vx, settle_elapsed, self.settle_timeout_sec,
                         self.next_state))

            elif self.state in ('MANEUVER', 'EXIT_MANEUVER'):
                # See APPROACH's comment (2026-08-11).
                self.pub_mapping.publish(Bool(data=bool(False)))
                self.pub_active.publish(Bool(data=bool(True)))
                is_exit = self.state == 'EXIT_MANEUVER'
                segments = (
                    self.exit_segments if is_exit
                    else self.maneuver_segments)
                segment_index = (
                    self.exit_segment_index if is_exit
                    else self.maneuver_segment_index)
                phase_name = (
                    'PARALLEL_EXIT' if is_exit
                    else 'PARALLEL_MULTI')
                cand = self.reproject_locked_slot()
                if (cand is None or not segments
                        or segment_index >= len(segments)):
                    self.abort(
                        '%s: no locked slot or segments' % self.state)
                    time.sleep(period)
                    continue
                segment = segments[segment_index]
                if self.reverse_safety_stop(
                        cand, self.state, segment=segment):
                    time.sleep(period)
                    continue
                elapsed = self.dt(now, self.phase_start_time)
                if self.phase_start_pose is None:
                    moved = 0.0
                    turned = 0.0
                else:
                    moved = math.hypot(
                        self.x - self.phase_start_pose[0],
                        self.y - self.phase_start_pose[1])
                    steer_sign = (
                        1.0 if segment['steer'] >= 0 else -1.0)
                    if segment['direction'] > 0:
                        turn_sign = self.forward_turn_sign * steer_sign
                    else:
                        turn_sign = steer_sign
                    turned = turn_sign * normalize_angle(
                        self.yaw - self.phase_start_pose[2])
                exit_reverse_distance_cap = (
                    is_exit
                    and segment['direction'] < 0
                    and segment.get('name')
                    == 'EXIT_REVERSE_HEADING_ALIGN'
                    and moved >= max(
                        0.05, segment['distance']
                        - self.exit_reverse_brake_buffer))
                if segment['steer'] == 0:
                    completed = moved >= segment['distance']
                else:
                    completed = (
                        turned >= segment['yaw']
                        or exit_reverse_distance_cap)
                speed = max(
                    abs(self.sim_speed_forward
                        if segment['direction'] > 0
                        else self.sim_speed_reverse), 0.03)
                timeout = max(
                    3.0, 2.0 * segment['distance'] / speed + 3.0)
                if completed or elapsed >= timeout:
                    if (is_exit
                            and segment.get('name')
                            == 'EXIT_REVERSE_HEADING_ALIGN'):
                        # Replan only after SETTLE has absorbed drivetrain
                        # coast; otherwise the next path starts several cm
                        # behind the pose that was just validated.
                        self.exit_feedback_pending_segment = dict(segment)
                        self.exit_feedback_start_pose = self.phase_start_pose
                    self.logwarn(
                        '[%s] segment %d/%d %s done '
                        'moved=%.3f/%.3f turned=%.1f/%.1fdeg limit=%s',
                        phase_name, segment_index + 1,
                        len(segments), segment['name'],
                        moved, segment['distance'],
                        math.degrees(turned),
                        math.degrees(segment['yaw']),
                        ('distance' if exit_reverse_distance_cap
                         else ('timeout' if elapsed >= timeout
                               else 'yaw')))
                    if is_exit:
                        self.exit_segment_index += 1
                        finished = (
                            self.exit_segment_index >= len(segments))
                    else:
                        self.maneuver_segment_index += 1
                        finished = (
                            self.maneuver_segment_index >= len(segments))
                    if finished:
                        self.go_settle(
                            'EXIT_COMPLETE' if is_exit
                            else 'FINAL_ALIGN')
                    else:
                        self.go_settle(
                            'EXIT_MANEUVER' if is_exit
                            else 'MANEUVER')
                else:
                    rpm, slowing, remaining, slow_window, unit = (
                        self.maneuver_speed_command(
                            segment, moved, turned,
                            exit_mode=is_exit))
                    self.publish_cmd(rpm, segment['steer'], 1)
                    if slowing:
                        if unit == 'rad':
                            remaining_text = (
                                'yaw %.1f/%.1fdeg' %
                                (math.degrees(remaining),
                                 math.degrees(slow_window)))
                        else:
                            remaining_text = (
                                'dist %.2f/%.2fm' %
                                (remaining, slow_window))
                        self.loginfo_throttle(
                            0.5,
                            '[PARALLEL_SPEED] %s cruise=(%d,%d) -> '
                            'slow rpm=%d remaining[%s] vx=%.3f wz=%.1fdeg/s',
                            segment['name'],
                            (self.exit_forward_rpm if is_exit
                             else self.maneuver_forward_rpm),
                            (self.exit_reverse_rpm if is_exit
                             else self.maneuver_reverse_rpm),
                            rpm, remaining_text, self.vx,
                            math.degrees(self.wz))
                    self.status(
                        '%s %d/%d %s %s rpm=%d steer=%d '
                        'moved=%.2f/%.2f yaw=%.1f/%.1fdeg' %
                        ('EXIT' if is_exit else 'MANEUVER',
                         segment_index + 1,
                         len(segments), segment['name'],
                         'SLOW' if slowing else 'CRUISE',
                         rpm, segment['steer'], moved,
                         segment['distance'], math.degrees(turned),
                         math.degrees(segment['yaw'])))

            elif self.state == 'REVERSE_IN_ARC':
                # See APPROACH's comment (2026-08-11).
                self.pub_mapping.publish(Bool(data=bool(False)))
                self.pub_active.publish(Bool(data=bool(True)))
                cand = self.reproject_locked_slot()
                if cand is None:
                    self.abort('REVERSE_IN_ARC: no locked slot')
                    time.sleep(period)
                    continue
                if self.reverse_safety_stop(cand, 'REVERSE_IN_ARC'):
                    time.sleep(period)
                    continue
                elapsed = self.dt(now, self.phase_start_time)
                # 진입각은 '고정된 슬롯 축' 기준으로 잰다(접근 드리프트에 강함).
                start_yaw = self.phase_start_pose[2] if self.phase_start_pose is not None else self.yaw
                turn_dir = 1.0 if self.plan['into_steer'] >= 0 else -1.0
                turned = turn_dir * normalize_angle(self.yaw - start_yaw)
                timeout = max(self.reverse_arc_timeout_sec, 1.6 * self.plan['t_in'] + 3.0)
                if turned >= self.plan['into_yaw'] or elapsed >= timeout:
                    self.logwarn('[PARALLEL_SEQ] REVERSE_IN_ARC done turned=%.1f/%.1fdeg elapsed=%.1f/%.1fs',
                                  math.degrees(turned), math.degrees(self.plan['into_yaw']), elapsed, timeout)
                    if self.plan.get('straight_len', 0.0) > 0.03:
                        self.go_settle('REVERSE_STRAIGHT')
                    else:
                        self.go_settle('REVERSE_OUT_ARC')
                else:
                    self.publish_cmd(self.reverse_rpm, self.plan['into_steer'], 1)
                    self.status('REVERSE_IN_ARC: steer=%d turned=%.1f/%.1fdeg elapsed=%.1f/%.1fs' %
                                (self.plan['into_steer'], math.degrees(turned),
                                 math.degrees(self.plan['into_yaw']), elapsed, timeout))

            elif self.state == 'REVERSE_STRAIGHT':
                # See APPROACH's comment (2026-08-11).
                self.pub_mapping.publish(Bool(data=bool(False)))
                self.pub_active.publish(Bool(data=bool(True)))
                cand = self.reproject_locked_slot()
                if cand is None:
                    self.abort('REVERSE_STRAIGHT: no locked slot')
                    time.sleep(period)
                    continue
                if self.reverse_safety_stop(cand, 'REVERSE_STRAIGHT'):
                    time.sleep(period)
                    continue
                elapsed = self.dt(now, self.phase_start_time)
                if self.phase_start_pose is None:
                    moved = 0.0
                else:
                    moved = math.hypot(self.x - self.phase_start_pose[0],
                                       self.y - self.phase_start_pose[1])
                target = max(0.0, self.plan.get('straight_len', 0.0))
                timeout = max(1.0, 1.6 * self.plan.get('t_straight', 0.0) + 2.0)
                if moved >= target or elapsed >= timeout:
                    self.logwarn('[PARALLEL_SEQ] REVERSE_STRAIGHT done moved=%.3f/%.3f elapsed=%.1f/%.1fs',
                                  moved, target, elapsed, timeout)
                    self.go_settle('REVERSE_OUT_ARC')
                else:
                    self.publish_cmd(self.reverse_rpm, 0, 1)
                    self.status('REVERSE_STRAIGHT: moved=%.3f/%.3f elapsed=%.1f/%.1fs' %
                                (moved, target, elapsed, timeout))

            elif self.state == 'REVERSE_OUT_ARC':
                # See APPROACH's comment (2026-08-11).
                self.pub_mapping.publish(Bool(data=bool(False)))
                self.pub_active.publish(Bool(data=bool(True)))
                cand = self.reproject_locked_slot()
                if cand is None:
                    self.abort('REVERSE_OUT_ARC: no locked slot')
                    time.sleep(period)
                    continue
                if self.reverse_safety_stop(cand, 'REVERSE_OUT_ARC'):
                    time.sleep(period)
                    continue
                _, _, _, yaw_err = self.slot_errors_for_pose(cand, (0.0, 0.0, 0.0))
                elapsed = self.dt(now, self.phase_start_time)
                timeout = max(self.reverse_arc_timeout_sec, 1.6 * self.plan['t_out'] + 3.0)
                if abs(math.degrees(yaw_err)) <= self.reverse_out_yaw_tolerance_deg or elapsed >= timeout:
                    self.logwarn('[PARALLEL_SEQ] REVERSE_OUT_ARC done yawerr=%.1fdeg elapsed=%.1f/%.1fs',
                                  math.degrees(yaw_err), elapsed, timeout)
                    self.go_settle('FINAL_ALIGN')
                else:
                    self.publish_cmd(self.reverse_rpm, self.plan['out_steer'], 1)
                    self.status('REVERSE_OUT_ARC: steer=%d yawerr=%.1fdeg elapsed=%.1f/%.1fs' %
                                (self.plan['out_steer'], math.degrees(yaw_err), elapsed, timeout))

            elif self.state == 'FINAL_ALIGN':
                # See APPROACH's comment (2026-08-11).
                self.pub_mapping.publish(Bool(data=bool(False)))
                self.pub_active.publish(Bool(data=bool(True)))
                cand = self.reproject_locked_slot()
                if cand is None:
                    self.abort('FINAL_ALIGN: no locked slot')
                    time.sleep(period)
                    continue
                self.publish_goal(cand)
                done_ok, info = self.done_check(cand)
                center_done = (
                    abs(info['s_err']) <= self.longitudinal_tolerance
                    and abs(info['q_err'])
                    <= self.final_center_lateral_tolerance
                    and abs(math.degrees(info['yaw_err']))
                    <= self.final_yaw_tolerance_deg
                    and abs(self.wz) <= self.final_wz_tolerance
                    and info['front_clear'] >= self.end_safety_margin
                    and info['rear_clear'] >= self.end_safety_margin
                    and info['inner_clear'] >= self.side_safety_margin
                    and not info['cone_collision']
                    and info['cone_clear']
                    >= self.cone_safety_stop_margin)
                if done_ok or center_done:
                    self.enter_parked(info)
                    time.sleep(period)
                    continue

                # A straight motion cannot remove lateral error.  The previous
                # implementation nevertheless kept reversing here, turning a
                # nearly centered 3 cm longitudinal error into a rear-wall
                # stop.  Stop safely and expose the lateral miss instead.
                if (abs(info['q_err'])
                        > self.final_center_lateral_tolerance):
                    self.abort(
                        'FINAL_ALIGN lateral miss q=%.3f exceeds %.3f; '
                        'straight motion blocked' %
                        (info['q_err'],
                         self.final_center_lateral_tolerance))
                    time.sleep(period)
                    continue

                if abs(self.wz) > self.final_wz_tolerance:
                    self.publish_stop()
                    self.status(
                        'FINAL_ALIGN: waiting yaw rate %.3f rad/s' %
                        self.wz)
                    time.sleep(period)
                    continue

                if (abs(info['s_err']) <= self.longitudinal_tolerance
                        and abs(math.degrees(info['yaw_err']))
                        > self.final_yaw_tolerance_deg):
                    self.abort(
                        'FINAL_ALIGN heading miss %.1fdeg at longitudinal '
                        'center; extra straight motion blocked' %
                        math.degrees(info['yaw_err']))
                    time.sleep(period)
                    continue

                required = abs(info['s_err'])
                if required > (
                        self.final_center_max_distance
                        + self.longitudinal_tolerance):
                    self.abort(
                        'FINAL_ALIGN longitudinal correction %.3f exceeds '
                        'limit %.3f' %
                        (required, self.final_center_max_distance))
                    time.sleep(period)
                    continue

                if self.phase_start_pose is None:
                    moved = 0.0
                else:
                    moved = math.hypot(
                        self.x - self.phase_start_pose[0],
                        self.y - self.phase_start_pose[1])
                if moved >= self.final_center_max_distance:
                    self.abort(
                        'FINAL_ALIGN correction travel %.3f reached '
                        'limit %.3f' %
                        (moved, self.final_center_max_distance))
                    time.sleep(period)
                    continue

                # s_err < 0 means the vehicle body is behind the slot center,
                # therefore move forward.  s_err > 0 selects reverse.
                direction = 1 if info['s_err'] < 0.0 else -1
                end_clear = (
                    info['front_clear'] if direction > 0
                    else info['rear_clear'])
                if (info['cone_collision']
                        or info['cone_clear']
                        <= self.reverse_cone_safety_stop_margin
                        or info['inner_clear']
                        <= self.swept_path_boundary_margin
                        or end_clear
                        <= self.end_safety_margin):
                    self.abort(
                        'FINAL_ALIGN %s safety stop end=%.3f cone=%.3f '
                        'inner=%.3f' %
                        ('forward' if direction > 0 else 'reverse',
                         end_clear, info['cone_clear'],
                         info['inner_clear']))
                    time.sleep(period)
                    continue

                turn_factor = (
                    self.forward_turn_sign if direction > 0 else 1.0)
                if abs(turn_factor) < 1e-6:
                    turn_factor = -1.0 if direction > 0 else 1.0
                raw = (
                    self.final_align_yaw_kp * info['yaw_err']
                    / turn_factor)
                steer = int(round(clamp(raw, -self.final_align_max_steer, self.final_align_max_steer)))
                if abs(steer) <= self.steer_deadband_cmd:
                    steer = 0
                rpm = direction * self.final_center_rpm
                self.publish_cmd(rpm, steer, 1)
                self.status(
                    'FINAL_ALIGN: %s rpm=%d steer=%d moved=%.3f/%.3f '
                    's=%.3f q=%.3f yaw=%.1f '
                    'clear[f=%.3f r=%.3f in=%.3f]' %
                    ('forward' if direction > 0 else 'reverse',
                     rpm, steer, moved, self.final_center_max_distance,
                     info['s_err'], info['q_err'],
                     math.degrees(info['yaw_err']),
                     info['front_clear'], info['rear_clear'],
                     info['inner_clear']))
                self.csv.row('parallel_control.csv', [
                    self.tsec(), self.state, self.x, self.y, math.degrees(self.yaw),
                    info['dist'], info['s_err'], info['q_err'], math.degrees(info['yaw_err']),
                    info['front_clear'], info['rear_clear'],
                    info['inner_clear'], rpm, steer])

            elif self.state == 'PARKED':
                self.publish_zero(
                    active=True, mapping=False, done=False,
                    stop_req=True)
                self.pub_parked.publish(Bool(data=True))
                elapsed = (
                    self.dt(now, self.parked_start_time)
                    if self.parked_start_time is not None else 0.0)
                auto_ready = (
                    self.auto_exit
                    and elapsed >= self.parking_hold_sec)
                if self.exit_requested or auto_ready:
                    cand = self.reproject_locked_slot()
                    exit_plan = self.build_exit_plan(cand)
                    if not exit_plan.get('feasible', False):
                        self.abort(
                            'PARKED: no safe exit plan: %s' %
                            exit_plan.get('reason', 'unknown'))
                        time.sleep(period)
                        continue
                    self.exit_segments = exit_plan['segments']
                    self.exit_segment_index = 0
                    self.exit_requested = False
                    self.pub_parked.publish(Bool(data=False))
                    check = exit_plan['check']
                    ready_info = exit_plan['ready_info']
                    self.logwarn(
                        '[PARALLEL_EXIT_PLAN] strategy=%s segments=%s '
                        'predicted[q=%.3f/%.3f yaw=%.1f] '
                        'clear[cone=%.3f inner=%.3f end=%.3f]',
                        exit_plan.get('strategy', 'unknown'),
                        ', '.join(
                            '%s:%.2fm' %
                            (segment['name'], segment['distance'])
                            for segment in self.exit_segments),
                        ready_info['max_q'],
                        ready_info['outside_limit'],
                        math.degrees(ready_info['yaw_error']),
                        check['min_cone_clear'],
                        check['min_inner_clear'],
                        check['min_end_clear'])
                    self.go_settle('EXIT_MANEUVER')
                else:
                    self.status(
                        'PARKED: hold %.1fs auto_exit=%s; '
                        'waiting /parking_exit_start' %
                        (elapsed, self.auto_exit))

            elif self.state == 'EXIT_COMPLETE':
                cand = self.reproject_locked_slot()
                if cand is None:
                    self.abort('EXIT_COMPLETE: no locked slot')
                    time.sleep(period)
                    continue
                ready, info = self.exit_pose_check(
                    cand, (0.0, 0.0, 0.0))
                # Reverse-only integration intentionally hands off while the
                # car is still in the slot. The waypoint controller performs
                # the subsequent left-steer exit and heading tracking.
                if not self.exit_reverse_only and not ready:
                    self.abort(
                        'EXIT_COMPLETE verification failed '
                        'q=%.3f/%.3f yaw=%.1f cone=%.3f collision=%s' %
                        (info['max_q'], info['outside_limit'],
                         math.degrees(info['yaw_error']),
                         info['cone_clear'],
                         info['cone_collision']))
                    time.sleep(period)
                    continue
                self.complete_exit(info)

            elif self.state == 'DONE':
                # complete_exit() sent one final disabled zero command.
                # From here on, publish only ownership flags so waypoint
                # /cmd_* messages are never overwritten by this node.
                self.publish_flags(
                    active=False, mapping=False, done=True,
                    stop_req=False)
                self.pub_parked.publish(Bool(data=False))
                self.pub_exit_done.publish(Bool(data=True))
                self.status(
                    'DONE: exit complete; waypoint control released '
                    'log_dir=%s' % self.log_dir)

            elif self.state == 'ABORT':
                self.publish_zero(active=True, mapping=False, done=False, stop_req=True)
                self.status('ABORT: %s log_dir=%s' % (self.abort_reason, self.log_dir))

            time.sleep(period)

    def on_shutdown(self):
        try:
            self.publish_cmd(0, 0, 0)
        except Exception:
            pass
        self.csv.close()


def main(args=None):
    rclpy.init(args=args)
    node = RuleBasedParallelParkingNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

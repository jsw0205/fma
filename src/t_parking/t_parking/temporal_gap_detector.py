#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""주행 중 측면 경계의 막힘->열림->막힘 전이로 주차 입구를 찾는다."""

import math

import numpy as np

from .geometry import unit


class TemporalSideGapDetector(object):
    """
    순간 프레임의 콘쌍을 맞추지 않는다.

    차량이 진행하면서 측면의 가까운 경계가 사라진 첫 위치와 다시 나타난
    위치를 odom에 저장하고, 두 위치의 진행축 거리를 실제 입구 폭으로 쓴다.
    경계까지의 횡거리는 최초 실행값에 고정하지 않고 매 실행에서 학습한다.
    """

    def __init__(self, min_width, max_width, laser_x, laser_y, side,
                 open_jump=0.60, close_jump=0.30, confirm_frames=3,
                 baseline_alpha=0.25):
        self.min_width = float(min_width)
        self.max_width = float(max_width)
        self.laser_x = float(laser_x)
        self.laser_y = float(laser_y)
        self.side_sign = 1.0 if str(side).lower() == 'left' else -1.0
        self.open_jump = max(0.05, float(open_jump))
        self.close_jump = max(0.02, min(float(close_jump), self.open_jump))
        self.confirm_frames = max(1, int(confirm_frames))
        self.baseline_alpha = min(1.0, max(0.01, float(baseline_alpha)))
        self.reset(0.0)

    def reset(self, start_yaw):
        self.start_yaw = float(start_yaw)
        self.lane_odom = unit(self.start_yaw)
        self.side_odom = np.array(
            [-self.side_sign * self.lane_odom[1],
             self.side_sign * self.lane_odom[0]], dtype=float)
        self.state = 'SEEK_BOUNDARY'
        self.baseline = None
        self.boundary_confirmed = False
        self.blocked_count = 0
        self.open_count = 0
        self.close_count = 0
        self.open_first = None
        self.close_first = None
        self.start_edge_odom = None
        self.end_edge_odom = None
        self.width = None
        self.completed = False
        self.last_event = 'RESET'
        self.last_side_distance = None
        self.provisional_width = 0.0

    def _progress(self, pose):
        return float(np.dot(np.asarray(pose[:2], dtype=float), self.lane_odom))

    def _edge_point(self, pose, lateral):
        """전이 순간 라이다의 진행 위치와 학습한 경계 횡거리의 교점."""
        pos = np.asarray(pose[:2], dtype=float)
        heading = unit(float(pose[2]))
        sensor = pos + self.laser_x * heading + self.laser_y * self.side_odom
        return sensor + float(lateral) * self.side_odom

    def _update_baseline(self, distance):
        d = float(distance)
        if self.baseline is None or d < self.baseline:
            # 시작이 열린 구간이어도 이후 가까운 경계를 만나면 즉시 내려간다.
            self.baseline = d
        elif d <= self.baseline + self.close_jump:
            a = self.baseline_alpha
            self.baseline = (1.0 - a) * self.baseline + a * d

    def update(self, side_distance, pose):
        """
        side_distance: 차량 진행축에 수직인 좁은 측면 창의 최근접 거리.
                       관측점이 없으면 None.
        pose: (odom_x, odom_y, odom_yaw)
        return: 발생한 이벤트 문자열 또는 None.
        """
        if self.completed:
            return None

        finite = side_distance is not None and math.isfinite(float(side_distance))
        d = float(side_distance) if finite else None
        self.last_side_distance = d

        if self.baseline is None and finite:
            self.baseline = d

        if self.state == 'SEEK_BOUNDARY':
            if self.baseline is None:
                self.last_event = 'NO_SIDE_RETURN'
                return None

            is_open = (not finite) or (d > self.baseline + self.open_jump)
            if not is_open:
                self._update_baseline(d)
                self.blocked_count += 1
                self.open_count = 0
                self.open_first = None
                if self.blocked_count >= self.confirm_frames:
                    self.boundary_confirmed = True
                    self.last_event = 'BOUNDARY'
                return None

            if not self.boundary_confirmed:
                # 시작부터 열린 공간이면 입구로 세지 않는다. 먼저 막힌 경계를
                # 실제로 지나야 다음 열림이 주차공간 시작이 된다.
                self.last_event = 'WAIT_BOUNDARY'
                return None

            if self.open_count == 0:
                self.open_first = {
                    'pose': tuple(pose),
                    'progress': self._progress(pose),
                    'lateral': float(self.baseline),
                }
            self.open_count += 1
            if self.open_count >= self.confirm_frames:
                self.state = 'IN_GAP'
                self.start_edge_odom = self._edge_point(
                    self.open_first['pose'], self.open_first['lateral'])
                self.close_count = 0
                self.last_event = 'GAP_OPEN'
                return 'GAP_OPEN'
            self.last_event = 'OPEN_CONFIRM'
            return None

        # IN_GAP: 가까운 경계가 다시 나타나면 입구의 두 번째 끝이다.
        is_closed = finite and d <= self.baseline + self.close_jump
        if not is_closed:
            self.close_count = 0
            self.close_first = None
            self.provisional_width = max(
                0.0, self._progress(pose) - self.open_first['progress'])
            self.last_event = 'IN_GAP'
            return None

        if self.close_count == 0:
            self.close_first = {
                'pose': tuple(pose),
                'progress': self._progress(pose),
                'lateral': float(d),
            }
        self.close_count += 1
        if self.close_count < self.confirm_frames:
            self.last_event = 'CLOSE_CONFIRM'
            return None

        self.end_edge_odom = self._edge_point(
            self.close_first['pose'], self.close_first['lateral'])
        self.width = abs(float(np.dot(
            self.end_edge_odom - self.start_edge_odom, self.lane_odom)))
        self.provisional_width = self.width
        self._update_baseline(self.close_first['lateral'])

        if self.min_width <= self.width <= self.max_width:
            self.completed = True
            self.state = 'COMPLETE'
            self.last_event = 'GAP_COMPLETE'
            return 'GAP_COMPLETE'

        # 폭 범위 밖의 열린 구간은 버리고, 방금 다시 만난 경계를 다음
        # 탐색의 선행 경계로 사용한다.
        event = 'GAP_REJECT_WIDTH'
        self.state = 'SEEK_BOUNDARY'
        self.boundary_confirmed = True
        self.blocked_count = self.confirm_frames
        self.open_count = 0
        self.close_count = 0
        self.open_first = None
        self.close_first = None
        self.start_edge_odom = None
        self.end_edge_odom = None
        self.last_event = event
        return event

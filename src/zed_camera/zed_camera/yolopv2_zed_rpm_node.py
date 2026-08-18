#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZED 2i + YOLOPv2 lane & stop-line node (ROS2 / Humble) + CAN

라인트래킹을 OAK-D 노드에서 쓰던 방식으로 교체한 버전.

교체 내용
---------
기존: sliding_window_with_boxes_and_angle (BEV 슬라이딩 윈도우 + polyfit)
      -> 한쪽 차선만 보이면 left_fit/right_fit 이 [0,0,0] 으로 남아
         lane_center=0, offset=-w/2 가 되어 최대 좌조향이 나가는 버그가 있었음.

신규: LaneTracker (이 파일 안에 내장)
      1) BEV 마스크 하단 ROI만 사용
      2) connectedComponents 로 '차선 덩어리' 단위 좌/우 분류
         -> 한 줄이 화면 중앙을 걸쳐도 가짜 양쪽 차선으로 오인하지 않음
      3) 양쪽 보이면 lane_center 계산 + 반차선폭(half width)을 EMA 로 학습
      4) 한쪽만 보이면 학습된 반차선폭으로 중심 추정 (좌/우 hysteresis)
      5) error_px -> 정규화 -> steering_deg (EMA + step limit)
      6) 차선 유실 확정 시 steer=0 + CAN rpm=0 (정지)

모든 임계값이 '비율' 이라 해상도(VGA/HD720/HD1080)에 무관하게 동작한다.
"""
import os, time, threading, csv, struct
from dataclasses import dataclass
from datetime import datetime, timezone
from collections import deque

import numpy as np, cv2, torch

# python-can: 하위 MCU로 steer/rpm 제어 프레임 전송용.
# 미설치여도 can_enable=false 면 노드는 정상 동작(경고만).
try:
    import can
    _HAS_CAN = True
except Exception:
    _HAS_CAN = False

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSHistoryPolicy, QoSReliabilityPolicy)
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import Image as RosImage, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import message_filters

# === utils (keep your existing package module) ===
from zed_camera.utils.utils import (
    select_device, scale_coords, non_max_suppression, split_for_trace_model,
    lane_line_mask, plot_one_box, AverageMeter, letterbox
)


def time_synchronized() -> float:
    if torch.cuda.is_available():
        try: torch.cuda.synchronize()
        except Exception: pass
    return time.time()


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


# ============================================================
# BEV 변환
# ============================================================
def build_perspective_from_pct(pct_list, w, h):
    px = lambda p, total: float(p) * total * 0.01
    src = np.float32([
        [px(pct_list[0], w), px(pct_list[1], h)],
        [px(pct_list[2], w), px(pct_list[3], h)],
        [px(pct_list[4], w), px(pct_list[5], h)],
        [px(pct_list[6], w), px(pct_list[7], h)],
    ])
    # 좌우 대칭. 비대칭이면 조향에 상시 바이어스가 생긴다.
    dst = np.float32([
        [w * 0.10, 3],
        [w * 0.90, 3],
        [w * 0.10, h],
        [w * 0.90, h]
    ])
    return cv2.getPerspectiveTransform(src, dst), src


def bird_eye_view_transform(img: np.ndarray, M: np.ndarray):
    if img is None or img.size == 0 or M is None or not (isinstance(M, np.ndarray) and M.shape == (3, 3)):
        return None
    h, w = img.shape[:2]
    try:
        return cv2.warpPerspective(img, M, (w, h))
    except cv2.error:
        return None


def detect_stop_line(binary_warped: np.ndarray,
                     horizontal_line_y_offset: int = 50,
                     segment_count: int = 10,
                     threshold: int = 5,
                     min_segments: int = 5) -> bool:
    h, w = binary_warped.shape
    y = max(0, min(h - 1, h - horizontal_line_y_offset))
    row = binary_warped[int(y), :]
    seg_w = max(1, w // max(1, segment_count))
    hits = sum(np.sum(row[i * seg_w:(i + 1) * seg_w] > 0) >= threshold
               for i in range(segment_count))
    return hits >= min_segments


# ============================================================
# LaneTracker : OAK-D 노드에서 쓰던 라인트래킹 로직
# ============================================================
@dataclass
class LaneResult:
    valid: bool = False              # 조향에 쓸 수 있는 결과인지
    steer_deg: float = 0.0           # 평활화/스텝제한 적용된 최종 조향각[deg]
    raw_steer_deg: float = 0.0       # 평활화 전 원시 조향각[deg]
    error_px: float = 0.0            # lane_center - image_center [px]
    lane_center_x: float = 0.0       # 추정된 차선 중심 x [px]
    left_x: float = float('nan')
    right_x: float = float('nan')
    pixel_count: int = 0
    side: str = "NONE"               # BOTH / LEFT / RIGHT / NONE
    half_width: float = 0.0          # 학습된 반차선폭 [px]
    roi_y0: int = 0
    reason: str = ""
    curvature_deg: float = 0.0       # 2차 피팅 기반 앞쪽 곡률 예측분 [deg] (raw_steer_deg 에 이미 포함됨)


class LaneTracker:
    """이진 차선 마스크 -> 조향각[deg]. BEV 마스크를 입력으로 쓴다."""

    def __init__(self,
                 roi_y_ratio=0.55,
                 min_mask_ratio=0.0024,
                 max_mask_ratio=0.35,
                 min_component_ratio=0.0004,
                 min_side_ratio=0.0006,
                 half_width_ratio=0.40,
                 half_width_min_ratio=0.08,
                 half_width_max_ratio=0.45,
                 half_width_smooth=0.2,
                 max_candidates=4,
                 max_steer_deg=30.0,
                 steer_smooth=0.4,
                 max_step_deg=4.0,
                 lost_hold_frames=3,
                 morph=True,
                 num_bands=4,
                 min_fit_bands=3,
                 curvature_gain_deg=8.0,
                 curvature_max_deg=10.0):
        self.roi_y_ratio = roi_y_ratio
        self.min_mask_ratio = min_mask_ratio
        self.max_mask_ratio = max_mask_ratio
        self.min_component_ratio = min_component_ratio
        self.min_side_ratio = min_side_ratio
        self.half_width_ratio = half_width_ratio
        self.half_width_min_ratio = half_width_min_ratio
        self.half_width_max_ratio = half_width_max_ratio
        self.half_width_smooth = half_width_smooth
        self.max_candidates = max_candidates
        self.max_steer_deg = max_steer_deg
        self.steer_smooth = steer_smooth
        self.max_step_deg = max_step_deg
        self.lost_hold_frames = lost_hold_frames
        self.morph = morph
        # 2차 피팅(곡률 예측) - ROI를 세로로 num_bands개 밴드로 나눠 각각에서
        # (근처 밴드와 동일한) 후보 탐지/폭검증 로직을 재사용해 (y, lane_center_x)
        # 점을 모으고, 그 점들에 np.polyfit(deg=2)로 곡선을 피팅한다. 근처(하단)
        # 지점의 기울기(dcenter/dy)를 "앞쪽에서 차선이 휘는 방향"으로 보고
        # 조향에 선반영(anticipation)한다 - GPS handoff 직후처럼 각도가
        # 급격할 때 위치 오차 하나만 보고 확 꺾었다가 놓치는 문제를 완화하기
        # 위함. 점이 min_fit_bands 개 미만이면 곡률항은 0 (기존 동작과 동일).
        self.num_bands = num_bands
        self.min_fit_bands = min_fit_bands
        self.curvature_gain_deg = curvature_gain_deg
        self.curvature_max_deg = curvature_max_deg

        self._k_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self._k_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        self.reset()

    def reset(self):
        self._half_width = None
        self._last_side = None
        self._steer = 0.0
        self._last_valid_steer = 0.0
        self._lost_count = 0

    # --------------------------------------------------------
    def update(self, mask) -> LaneResult:
        res = LaneResult()

        if mask is None or not isinstance(mask, np.ndarray) or mask.size == 0:
            return self._finish(res, False, 0.0, "no mask")

        if mask.ndim == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

        m = np.where(mask > 0, 255, 0).astype(np.uint8)
        h, w = m.shape[:2]

        if h < 8 or w < 8:
            return self._finish(res, False, 0.0, "mask too small")

        if self._half_width is None:
            self._half_width = float(self.half_width_ratio * w)

        hw_min = self.half_width_min_ratio * w
        hw_max = self.half_width_max_ratio * w
        image_center_x = w / 2.0

        # ---- ROI (아래쪽만) ----
        y0 = int(clamp(int(h * self.roi_y_ratio), 0, h - 2))
        roi = m[y0:h, :]
        res.roi_y0 = y0

        if self.morph:
            roi = cv2.morphologyEx(roi, cv2.MORPH_OPEN, self._k_open)
            roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, self._k_close)

        roi_area = float(roi.size)
        nz = cv2.countNonZero(roi)

        if nz < self.min_mask_ratio * roi_area:
            return self._finish(res, False, 0.0, "too few px (%d)" % nz)

        if nz > self.max_mask_ratio * roi_area:
            res.pixel_count = int(nz)
            return self._finish(res, False, 0.0,
                                "mask blown (%.2f)" % (nz / roi_area))

        # ---- 연결요소 단위 후보 추출 (좌/우로 미리 나누지 않음) ----
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            roi, connectivity=8)

        ys, xs_all = np.nonzero(roi)
        lab = labels[ys, xs_all]
        min_comp = max(8, int(self.min_component_ratio * roi_area))

        components = []  # (median_x, area, xs)
        total_px = 0

        for i in range(1, num_labels):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < min_comp:
                continue
            total_px += area
            xs_i = xs_all[lab == i]
            components.append((float(np.median(xs_i)), area, xs_i))

        res.pixel_count = int(total_px)

        if total_px < self.min_mask_ratio * roi_area:
            return self._finish(res, False, 0.0, "no valid component")

        components.sort(key=lambda c: c[0])

        full_w_min = 2.0 * hw_min
        full_w_max = 2.0 * hw_max
        expected_full_w = 2.0 * self._half_width

        # ---- 인접 후보 사전 정리: 너무 붙어있으면 차선 페어가 아니다 ----
        # 연석과 차선은 서로 딱 붙어있어서 간격이 진짜 차선폭(full_w_min)보다
        # 훨씬 좁다. 정렬된 후보를 왼쪽부터 훑으면서, 바로 옆 후보와의 간격이
        # full_w_min보다 좁으면 "이 둘 중 하나는 연석"으로 보고, 화면 중앙에
        # 더 가까운 쪽만 살리고 바깥쪽(연석 쪽)은 버린다. 페어 탐색은 이렇게
        # 정리된 후보로만 한다.
        cleaned = []
        i = 0
        while i < len(components):
            if i + 1 < len(components):
                gap = components[i + 1][0] - components[i][0]
                if gap < full_w_min:
                    a, b = components[i], components[i + 1]
                    keep = a if abs(a[0] - image_center_x) <= abs(b[0] - image_center_x) else b
                    cleaned.append(keep)
                    i += 2
                    continue
            cleaned.append(components[i])
            i += 1
        components = cleaned

        # 정리 후에도 "그럴듯한 차선 범위" 안에 후보가 너무 많으면(횡단보도/
        # 방지턱처럼 줄이 여러 개) 그중 뭐가 진짜 차선인지 폭만으로 안전하게
        # 판단하기 어렵다 - 페어 탐색 자체를 생략하고 바로 LOST. 범위 밖(화면
        # 가장자리 등, 애초에 페어가 될 수 없는 곳)에 있는 후보는 카운트에서
        # 제외 - 거기 뭐가 찍히든 우리 차로 판단이랑은 무관하므로.
        in_range = [
            c for c in components
            if abs(c[0] - image_center_x) <= full_w_max
        ]
        if len(in_range) >= self.max_candidates:
            return self._finish(res, False, 0.0,
                                 f"too many candidates in range ({len(in_range)})")

        # ---- 폭 검증 페어 탐색 (횡단보도/방지턱 등 남은 오검출 필터링) ----
        # 정리된 후보들 중 중앙을 사이에 둔 두 선의 간격이 그럴듯한 차선폭
        # 범위(full_w_min~full_w_max) 안에 들 때만 페어로 인정한다. 후보가
        # 여러 개면 학습된(EMA) 반차선폭에 제일 가까운 폭의 페어를 채택 -
        # "평소 차선 폭에 제일 가까운 조합"을 신뢰하는 것.
        best_pair = None
        best_diff = None
        for i, (lx, _larea, _lxs) in enumerate(components):
            if lx >= image_center_x:
                continue
            for rx, _rarea, _rxs in components[i + 1:]:
                if rx < image_center_x:
                    continue
                width = rx - lx
                if not (full_w_min <= width <= full_w_max):
                    continue
                diff = abs(width - expected_full_w)
                if best_diff is None or diff < best_diff:
                    best_diff = diff
                    best_pair = (lx, rx)

        min_side = max(10, int(self.min_side_ratio * roi_area))

        if best_pair is not None:
            left_x, right_x = best_pair
            lane_center_x = (left_x + right_x) / 2.0

            measured = (right_x - left_x) / 2.0
            a = self.half_width_smooth
            self._half_width = (1.0 - a) * self._half_width + a * measured

            self._last_side = None
            res.side = "BOTH"
            res.left_x, res.right_x = left_x, right_x
        else:
            # 유효한 폭의 페어가 없음. 정리된 후보가 한쪽에 정확히 하나뿐이면
            # (=진짜 한쪽 차선만 보이는 정상 상황) 기존처럼 학습된 반폭으로
            # 단일 차선 추정. 여전히 여러 개(예: 2차로처럼 진짜 차선이 여러
            # 개 있는 상황)면 어느 게 우리 차로 경계인지 애매하므로 억지로
            # 고르지 않고 LOST - 틀린 걸 확신하는 것보다 안전.
            left_comps = [c for c in components if c[0] < image_center_x]
            right_comps = [c for c in components if c[0] >= image_center_x]
            left_xs = left_comps[0][2] if left_comps else np.empty(0)
            right_xs = right_comps[0][2] if right_comps else np.empty(0)
            has_left = len(left_comps) == 1 and left_xs.size >= min_side
            has_right = len(right_comps) == 1 and right_xs.size >= min_side

            if has_left and not has_right:
                line_x = float(np.median(left_xs))
                lane_center_x = line_x + self._half_width
                res.side = "LEFT"
                res.left_x = line_x
                self._last_side = "left"
            elif has_right and not has_left:
                line_x = float(np.median(right_xs))
                lane_center_x = line_x - self._half_width
                res.side = "RIGHT"
                res.right_x = line_x
                self._last_side = "right"
            else:
                return self._finish(res, False, 0.0,
                                     "no valid pair, ambiguous candidates")

        # ---- 조향각 (위치항 + 곡률 예측항) ----
        error_px = lane_center_x - image_center_x
        error_norm = clamp(error_px / image_center_x, -1.0, 1.0)
        position_deg = error_norm * self.max_steer_deg
        curvature_deg = self._compute_curvature_term(roi, image_center_x, hw_min, hw_max, expected_full_w)
        raw_deg = clamp(position_deg + curvature_deg, -self.max_steer_deg, self.max_steer_deg)

        res.lane_center_x = lane_center_x
        res.error_px = error_px
        res.curvature_deg = curvature_deg
        return self._finish(res, True, raw_deg, "ok")

    # --------------------------------------------------------
    def _scan_region(self, region, image_center_x, hw_min, hw_max, expected_full_w):
        """update() 본문의 '연결요소 후보 추출 -> 인접 병합 -> 폭검증 페어
        탐색 -> 단일측 폴백' 로직을 임의의 이진 영역(전체 ROI 또는 밴드 하나)에
        재사용할 수 있게 뽑아낸 버전. (side, left_x, right_x, lane_center_x)를
        반환하고, 아무 것도 못 찾으면 (None, nan, nan, nan)."""
        region_area = float(region.size)
        if region_area <= 0:
            return None, float('nan'), float('nan'), float('nan')

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(region, connectivity=8)
        ys, xs_all = np.nonzero(region)
        if xs_all.size == 0:
            return None, float('nan'), float('nan'), float('nan')
        lab = labels[ys, xs_all]
        min_comp = max(5, int(self.min_component_ratio * region_area))
        components = []
        for i in range(1, num_labels):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < min_comp:
                continue
            xs_i = xs_all[lab == i]
            components.append((float(np.median(xs_i)), area, xs_i))
        if not components:
            return None, float('nan'), float('nan'), float('nan')
        components.sort(key=lambda c: c[0])

        full_w_min = 2.0 * hw_min
        full_w_max = 2.0 * hw_max

        cleaned = []
        i = 0
        while i < len(components):
            if i + 1 < len(components):
                gap = components[i + 1][0] - components[i][0]
                if gap < full_w_min:
                    a, b = components[i], components[i + 1]
                    keep = a if abs(a[0] - image_center_x) <= abs(b[0] - image_center_x) else b
                    cleaned.append(keep)
                    i += 2
                    continue
            cleaned.append(components[i])
            i += 1
        components = cleaned

        in_range = [c for c in components if abs(c[0] - image_center_x) <= full_w_max]
        if len(in_range) >= self.max_candidates:
            return None, float('nan'), float('nan'), float('nan')

        best_pair = None
        best_diff = None
        for i, (lx, _larea, _lxs) in enumerate(components):
            if lx >= image_center_x:
                continue
            for rx, _rarea, _rxs in components[i + 1:]:
                if rx < image_center_x:
                    continue
                width = rx - lx
                if not (full_w_min <= width <= full_w_max):
                    continue
                diff = abs(width - expected_full_w)
                if best_diff is None or diff < best_diff:
                    best_diff = diff
                    best_pair = (lx, rx)

        if best_pair is not None:
            left_x, right_x = best_pair
            return "BOTH", left_x, right_x, (left_x + right_x) / 2.0

        min_side = max(5, int(self.min_side_ratio * region_area))
        left_comps = [c for c in components if c[0] < image_center_x]
        right_comps = [c for c in components if c[0] >= image_center_x]
        left_xs = left_comps[0][2] if left_comps else np.empty(0)
        right_xs = right_comps[0][2] if right_comps else np.empty(0)
        has_left = len(left_comps) == 1 and left_xs.size >= min_side
        has_right = len(right_comps) == 1 and right_xs.size >= min_side

        if has_left and not has_right:
            return "LEFT", float(np.median(left_xs)), float('nan'), float('nan')
        if has_right and not has_left:
            return "RIGHT", float('nan'), float(np.median(right_xs)), float('nan')
        return None, float('nan'), float('nan'), float('nan')

    # --------------------------------------------------------
    def _compute_curvature_term(self, roi, image_center_x, hw_min, hw_max, expected_full_w):
        """ROI를 num_bands개 밴드로 나눠 각 밴드의 차선 중심을 추정하고,
        2차(점 3개 이상) 또는 1차(점 2개) 다항식으로 피팅해 하단(근처) 지점의
        기울기를 곡률 예측 조향각[deg]으로 변환한다. 점이 모자라면 0.0
        (기존 단일 위치항만 쓰던 동작과 동일)."""
        h_roi = roi.shape[0]
        if self.num_bands < 2 or h_roi < self.num_bands * 4 or self._half_width is None:
            return 0.0

        band_h = h_roi // self.num_bands
        ys, centers = [], []
        for b in range(self.num_bands):
            y_start = b * band_h
            y_end = h_roi if b == self.num_bands - 1 else (b + 1) * band_h
            if y_end - y_start < 4:
                continue
            band = roi[y_start:y_end, :]
            side, lx, rx, center = self._scan_region(band, image_center_x, hw_min, hw_max, expected_full_w)
            if side == "BOTH":
                c = center
            elif side == "LEFT":
                c = lx + self._half_width
            elif side == "RIGHT":
                c = rx - self._half_width
            else:
                continue
            ys.append((y_start + y_end) / 2.0)
            centers.append(c)

        if len(centers) < self.min_fit_bands:
            return 0.0

        ys_arr = np.asarray(ys, dtype=np.float64)
        cs_arr = np.asarray(centers, dtype=np.float64)
        deg = 2 if len(centers) >= 3 else 1
        try:
            coeffs = np.polyfit(ys_arr, cs_arr, deg)
        except (np.linalg.LinAlgError, ValueError):
            return 0.0

        near_y = float(h_roi - 1)
        if deg == 2:
            a, b, _c = coeffs
            slope = 2.0 * a * near_y + b
        else:
            a, _b = coeffs
            slope = a
        # y는 아래로(=차량 쪽) 증가하므로 slope>0 은 "앞쪽(위)으로 갈수록
        # 차선 중심이 왼쪽으로 이동" = 전방이 좌커브라는 뜻 -> 미리 좌조향
        # (음수) 로 선반영. gain/max 둘 다 다른 튜닝 파라미터들처럼 실차에서
        # 조절 가능 (lane_curvature_gain_deg / lane_curvature_max_deg).
        return clamp(-slope * self.curvature_gain_deg, -self.curvature_max_deg, self.curvature_max_deg)

    # --------------------------------------------------------
    def _finish(self, res, valid, raw_deg, reason):
        if valid:
            self._lost_count = 0
            target = raw_deg
            self._last_valid_steer = raw_deg
            out_valid = True
        else:
            self._lost_count += 1
            if self._lost_count <= self.lost_hold_frames:
                # 한두 프레임 놓친 것 -> 직전 조향 유지 (아직 LOST 아님)
                target = self._last_valid_steer
                out_valid = True
            else:
                # 확정 LANE LOST -> 조향 0, 호출부에서 rpm 도 0
                target = 0.0
                out_valid = False
                self._last_side = None

        a = clamp(self.steer_smooth, 0.01, 1.0)
        smoothed = (1.0 - a) * self._steer + a * target
        delta = clamp(smoothed - self._steer, -self.max_step_deg, self.max_step_deg)
        self._steer = self._steer + delta

        res.valid = out_valid
        res.raw_steer_deg = float(raw_deg)
        res.steer_deg = float(self._steer)
        res.half_width = float(self._half_width) if self._half_width else 0.0
        res.reason = reason
        if not out_valid:
            res.side = "NONE"
        return res

    # --------------------------------------------------------
    def draw_debug(self, vis_bgr, res):
        """BEV 시각화 위에 ROI/중앙선/추정중심/상태 표시."""
        if vis_bgr is None or vis_bgr.ndim != 3:
            return vis_bgr
        h, w = vis_bgr.shape[:2]

        cv2.line(vis_bgr, (0, res.roi_y0), (w, res.roi_y0), (255, 255, 0), 1)
        cv2.line(vis_bgr, (w // 2, 0), (w // 2, h), (0, 0, 255), 1)

        if res.valid:
            cx = int(clamp(res.lane_center_x, 0, w - 1))
            cv2.line(vis_bgr, (cx, res.roi_y0), (cx, h), (0, 255, 0), 2)
            for x, col in ((res.left_x, (255, 128, 0)), (res.right_x, (255, 0, 255))):
                if not np.isnan(x):
                    xi = int(clamp(x, 0, w - 1))
                    cv2.line(vis_bgr, (xi, res.roi_y0), (xi, h), col, 2)

        status = res.side if res.valid else "LANE LOST"
        color = (0, 255, 0) if res.valid else (0, 0, 255)
        cv2.putText(vis_bgr, "%s  steer=%6.2fdeg" % (status, res.steer_deg),
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(vis_bgr, "err=%6.1fpx  halfW=%.0f  px=%d" % (
                        res.error_px, res.half_width, res.pixel_count),
                    (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
        if not res.valid and res.reason:
            cv2.putText(vis_bgr, res.reason[:48], (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        return vis_bgr


# ============================================================
# 모델 초기화 (CPU/CUDA 안전)
# ============================================================
def initialize_model(weights: str, device: torch.device, half: bool):
    map_loc = 'cpu' if device.type == 'cpu' else None
    model = torch.jit.load(weights, map_location=map_loc)
    model = model.to(device).eval()
    if half and device.type != 'cpu':
        model.half()
    if device.type != 'cpu':
        torch.backends.cudnn.benchmark = True
    return model


# ============================================================
# CAN 브리지 (pcan_jetson_live.py 프레임 규격과 동일)
# ============================================================
class CanBridge:
    """
    TX 0x200 (제어):  <hhBBH> = rpm, steer, enable, stop_mode, reserved(0)
    RX 0x102 (구동):  <hhhh>  = enc, rpm*10, pwm_duty, target_rpm
    RX 0x101 (조향):  <HHhh>  = cur_pot, tgt_pot, cur_angle*10, tgt_angle*10
    """
    DRIVE_STATUS_ID    = 0x102
    STEERING_STATUS_ID = 0x101

    def __init__(self, channel="can0", bitrate=500000, tx_id=0x200,
                 setup_interface=False, logger=None):
        if not _HAS_CAN:
            raise RuntimeError("python-can 미설치 (pip install python-can)")
        self.tx_id  = int(tx_id)
        self.logger = logger
        if setup_interface:
            self._setup_interface(channel, bitrate)
        self.bus = can.interface.Bus(interface="socketcan", channel=channel)
        self.drive_status    = None
        self.steering_status = None

    @staticmethod
    def _setup_interface(channel, bitrate):
        subprocess_run = __import__("subprocess").run
        subprocess_run(["sudo", "ip", "link", "set", channel, "down"], check=False)
        result = subprocess_run(
            ["sudo", "ip", "link", "set", channel, "up",
             "type", "can", "bitrate", str(int(bitrate))],
            check=False)
        if result.returncode != 0:
            raise RuntimeError(
                f"'{channel}' 인터페이스를 올리지 못했습니다. "
                "어댑터 연결/sudo 권한을 확인하세요.")

    def send_control(self, rpm, steer, enable, stop_mode):
        data = struct.pack("<hhBBH",
                           int(rpm), int(steer),
                           int(enable), int(stop_mode), 0)
        msg = can.Message(arbitration_id=self.tx_id, data=data,
                          is_extended_id=False)
        self.bus.send(msg)

    def poll_feedback(self):
        while True:
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                break
            if msg.arbitration_id == self.DRIVE_STATUS_ID and len(msg.data) == 8:
                enc, rpm_x10, pwm, trpm = struct.unpack("<hhhh", msg.data)
                self.drive_status = {"encoder_count": enc, "rpm": rpm_x10 / 10.0,
                                     "pwm_duty": pwm, "target_rpm": trpm}
            elif msg.arbitration_id == self.STEERING_STATUS_ID and len(msg.data) == 8:
                cp, tp, ca, ta = struct.unpack("<HHhh", msg.data)
                self.steering_status = {"current_pot": cp, "target_pot": tp,
                                        "current_angle": ca / 10.0,
                                        "target_angle": ta / 10.0}

    def shutdown(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass


# ============================================================
# 노드
# ============================================================
class YoloPv2ZedNode(Node):
    def __init__(self):
        super().__init__("yolopv2_zed_node")
        self.bridge = CvBridge()

        # ------- 파라미터 -------
        self.weights   = self._param("weights",   "")
        self.img_size  = int(self._param("img_size", 640))
        self.conf_th   = float(self._param("conf_thres", 0.3))
        self.iou_th    = float(self._param("iou_thres", 0.45))
        self.agnostic  = bool(self._param("agnostic_nms", False))

        self.rgb_topic   = self._param("rgb_topic",   "/zed/zed_node/rgb/color/rect/image")
        self.depth_topic = self._param("depth_topic", "/zed/zed_node/depth/depth_registered")
        self.cinfo_topic = self._param("cinfo_topic", "/zed/zed_node/rgb/color/rect/camera_info")
        self.odom_topic  = self._param("odom_topic",  "/zed/zed_node/odom")

        # ------- GUI/디버그 창 -------
        self.show_windows = bool(self._param("show_windows", True))
        if self.show_windows:
            try:
                if not os.environ.get("DISPLAY", ""):
                    raise RuntimeError("DISPLAY not set")
                cv2.namedWindow("Original + HUD", cv2.WINDOW_NORMAL)
                cv2.namedWindow("Bird-eye view", cv2.WINDOW_NORMAL)
                cv2.namedWindow("Stop Line", cv2.WINDOW_NORMAL)
            except Exception as e:
                self.get_logger().warn(f"[gui] disabling windows ({e}); still publishing debug images")
                self.show_windows = False

        latched_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_img_hud      = self.create_publisher(RosImage, "~/dbg/hud",      latched_qos)
        self.pub_img_bev      = self.create_publisher(RosImage, "~/dbg/bev",      latched_qos)
        self.pub_img_stop     = self.create_publisher(RosImage, "~/dbg/stop",     latched_qos)
        self.pub_img_lanemask = self.create_publisher(RosImage, "~/dbg/lanemask", latched_qos)
        self.pub_img_bevmask  = self.create_publisher(RosImage, "~/dbg/bev_mask", latched_qos)

        # ------- 디바이스 -------
        raw_dev = str(self._param("device", "0")).lower()
        if raw_dev.startswith("cuda:"):
            raw_dev = raw_dev.split(":", 1)[1]
        self.device_id = raw_dev
        try:
            self.dev = select_device(self.device_id)
        except AssertionError as e:
            self.get_logger().warn(f"[Device] {e} -> falling back to CPU")
            self.dev = select_device("cpu")

        self.get_logger().info(
            f"[Device] using='{self.device_id}', torch_cuda={torch.cuda.is_available()}, "
            f"cuda_count={torch.cuda.device_count()}, torch_ver={torch.__version__}")
        self.get_logger().info(
            f"[Topics] rgb={self.rgb_topic}, depth={self.depth_topic}, "
            f"cinfo={self.cinfo_topic}, odom={self.odom_topic}")

        assert os.path.isfile(self.weights), f"weights not found: {self.weights}"
        self.half = (self.dev.type != 'cpu')
        self.model = initialize_model(self.weights, self.dev, self.half)
        if torch.cuda.is_available(): torch.cuda.empty_cache()

        # ------- state -------
        self.t_inf = AverageMeter(); self.t_nms = AverageMeter(); self.t_split = AverageMeter()
        self.persp_M = None; self.frame_wh = None; self.K = None
        self.quit = False

        self.pub_steer = self.create_publisher(Float32, "~/steering_deg", 10)
        self.pub_speed = self.create_publisher(Float32, "~/speed_mps",    10)
        self.pub_rpm   = self.create_publisher(Float32, "~/motor_rpm",    10)
        # rpm_target: the curve-scaled command from _speed_for_steer() -
        # NOT the same as pub_rpm above (motor_rpm is a speed-feedback
        # estimate from odometry, not a command). Published unconditionally
        # (2026-08-17) regardless of can_enable so control_arbiter can
        # subscribe to this instead of using a fixed camera_mode_rpm
        # parameter - see arbiter_node.py's camera_rpm_topic.
        self.pub_rpm_target = self.create_publisher(Float32, "~/rpm_target", 10)
        self.pub_lane_valid = self.create_publisher(Bool, "~/lane_valid", 10)
        self.speed_mps = float('nan')
        self.steering_deg = float('nan')

        self.wheel_radius = float(self._param("wheel_radius", 0.05))
        self.gear_ratio   = float(self._param("gear_ratio",   1.0))
        self._warned_no_odom = False

        # ------- CAN -------
        self.can_enable = bool(self._param("can_enable", True))
        self.can_channel = str(self._param("can_channel", "can0"))
        self.can_bitrate = int(self._param("can_bitrate", 500000))
        self.can_tx_id   = int(self._param("can_tx_id", 0x200))
        self.can_setup_interface = bool(self._param("can_setup_interface", True))
        self.can_required = bool(self._param("can_required", False))
        self.can_read_feedback = bool(self._param("can_read_feedback", True))

        self.motor_enable = int(self._param("motor_enable", 1))
        self.stop_mode    = int(self._param("stop_mode", 0))
        # 직진 기준 rpm. 커브에서는 아래 설정에 따라 자동 감속된다.
        self.can_target_rpm = int(self._param("can_target_rpm", 0))
        # 차선 유실 시 rpm 0 으로 정지시킬지
        self.stop_on_lane_lost = bool(self._param("stop_on_lane_lost", True))

        # --- 커브 자동 감속 (OAK-D 코드의 SPEED_STRAIGHT / SPEED_TURN 대응) ---
        # false 면 can_target_rpm 고정값만 송신.
        self.auto_speed = bool(self._param("auto_speed", True))
        # 커브 최대 시 rpm = can_target_rpm * rpm_turn_scale
        #   OAK 기준 24/30 = 0.8
        self.rpm_turn_scale = float(self._param("rpm_turn_scale", 0.8))
        # |steer| 가 이 값 이하면 완전 직진 취급 (감속 없음)
        self.steer_deadzone_deg = float(self._param("steer_deadzone_deg", 2.0))
        # |steer| 가 이 값 이상이면 최대 감속. 사이는 선형 보간.
        self.steer_full_deg = float(self._param("steer_full_deg", 18.0))
        # 한 틱(50ms)당 rpm 변화 제한. 급가감속 방지.
        self.rpm_step = int(self._param("rpm_step", 8))
        self._rpm_cmd = 0   # 현재 송신 중인 rpm (스텝 제한용)

        self.steer_sign = int(self._param("steer_sign", 1))
        self.steer_gain = float(self._param("steer_gain", 1.0))

        self.rpm_min   = int(self._param("rpm_min",   -150))
        self.rpm_max   = int(self._param("rpm_max",    150))
        self.steer_min = int(self._param("steer_min", -30))
        self.steer_max = int(self._param("steer_max",  30))

        self.can = None
        if self.can_enable:
            if not _HAS_CAN:
                msg = "[can] python-can 미설치 -> CAN 비활성 (pip install python-can)"
                if self.can_required:
                    raise RuntimeError(msg)
                self.get_logger().error(msg)
            else:
                try:
                    self.can = CanBridge(
                        channel=self.can_channel,
                        bitrate=self.can_bitrate,
                        tx_id=self.can_tx_id,
                        setup_interface=self.can_setup_interface,
                        logger=self.get_logger(),
                    )
                    self.get_logger().info(
                        f"[can] up: ch={self.can_channel}@{self.can_bitrate} "
                        f"tx_id=0x{self.can_tx_id:03X} enable={self.motor_enable} "
                        f"target_rpm={self.can_target_rpm}")
                except Exception as e:
                    if self.can_required:
                        raise
                    self.get_logger().error(f"[can] init 실패 -> CAN 비활성: {e}")
                    self.can = None
        else:
            self.get_logger().info("[can] can_enable=false -> CAN 비활성")

        # ------- CSV -------
        # 2026-08-05: was defaulting into the git-tracked src/ tree (82
        # loose files, 34MB) - now under ~/logs/lane/ instead.
        default_csv_dir = os.path.expanduser("~/logs/lane")
        csv_dir = str(self._param("csv_dir", default_csv_dir))
        os.makedirs(csv_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(csv_dir, f"lane_log_{ts}.csv")
        self._csv_flush_every = int(self._param("csv_flush_every", 20))
        self._csv_count = 0
        self.log_lock = threading.Lock()
        self.csv_fh, self.csv_writer = None, None
        try:
            self.csv_fh = open(self.csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_fh)
            self.csv_writer.writerow([
                "timestamp", "lane_valid", "side",
                "lane_center_x", "error_px", "half_width", "pixel_count",
                "speed_mps", "motor_rpm", "steering_deg"
            ])
            self.csv_fh.flush()
            self.get_logger().info(f"[csv] logging to {self.csv_path}")
        except Exception as e:
            self.get_logger().error(f"[csv] failed to open log file: {e}")
            self.csv_fh, self.csv_writer = None, None

        # ------- BEV / 정지선 튜닝 파라미터 -------
        self.src_pct = [25.0, 75.0, 75.0, 75.0, 15.0, 100.0, 85.0, 100.0]
        self.src_pct = [
            float(self._param("src_p1_x", self.src_pct[0])), float(self._param("src_p1_y", self.src_pct[1])),
            float(self._param("src_p2_x", self.src_pct[2])), float(self._param("src_p2_y", self.src_pct[3])),
            float(self._param("src_p3_x", self.src_pct[4])), float(self._param("src_p3_y", self.src_pct[5])),
            float(self._param("src_p4_x", self.src_pct[6])), float(self._param("src_p4_y", self.src_pct[7])),
        ]
        self.stop_y_offset  = int(self._param("stop_y_offset",  50))
        self.stop_segments  = int(self._param("stop_segments",  10))
        self.stop_threshold = int(self._param("stop_threshold", 5))
        self.stop_min_hits  = int(self._param("stop_min_hits",  5))
        self.src_pts = []

        # ------- 라인트래킹 (OAK-D 방식) -------
        self.lane = LaneTracker(
            roi_y_ratio         = float(self._param("lane_roi_y_ratio", 0.55)),
            half_width_ratio    = float(self._param("lane_half_width_ratio", 0.40)),
            half_width_smooth   = float(self._param("lane_half_width_smooth", 0.2)),
            max_candidates      = int(self._param("lane_max_candidates", 4)),
            max_steer_deg       = float(self._param("lane_max_steer_deg", 30.0)),
            steer_smooth        = float(self._param("lane_steer_smooth", 0.4)),
            max_step_deg        = float(self._param("lane_max_step_deg", 4.0)),
            lost_hold_frames    = int(self._param("lane_lost_hold_frames", 3)),
            max_mask_ratio      = float(self._param("lane_max_mask_ratio", 0.35)),
            num_bands           = int(self._param("lane_num_bands", 4)),
            min_fit_bands       = int(self._param("lane_min_fit_bands", 3)),
            curvature_gain_deg  = float(self._param("lane_curvature_gain_deg", 8.0)),
            curvature_max_deg   = float(self._param("lane_curvature_max_deg", 10.0)),
        )
        self.lane_valid = False
        # _publish_timer_cb runs on a fixed 20Hz ROS timer completely
        # independent of _infer_loop's actual GPU work - if the inference
        # thread stalls (USB3/EMI frame drop, GPU hitch, thermal throttle),
        # self.steering_deg/self.lane_valid just hold their last value and
        # the timer keeps re-publishing that stale result on schedule, so
        # nothing downstream (arbiter's camera_timeout_sec freshness check,
        # lane_valid-driven GPS fallback) ever sees a gap - a frozen frame
        # looks identical to a healthy one from the topic's perspective.
        # This tracks wall-clock time of the last _infer_loop iteration
        # that actually completed; _publish_timer_cb forces lane_valid to
        # False if that goes stale, independent of the last computed value.
        self._last_infer_wall_time = None
        self.infer_stale_timeout_sec = float(self._param("infer_stale_timeout_sec", 0.5))

        # Speed-based steering damping (2026-08-01) - unlike GPS's Stanley
        # law (atan2(k*cross_track_error, v) - correction shrinks as speed
        # rises), the camera's raw_deg = error_norm * max_steer_deg had NO
        # speed term at all, same gain at any speed. At rpm~150 this showed
        # up as a real problem: the camera would snap hard toward the lane
        # and overshoot past it, losing track. self.lane.max_steer_deg is
        # rescaled every frame from this fixed base value using current
        # self.speed_mps (already subscribed via _on_odom for the rpm
        # display/log) - higher speed -> smaller effective max_steer_deg,
        # same shape of damping as GPS's v-denominator, just written as a
        # simple 1/(1+gain*v) instead of atan2 since error_norm here is
        # already a normalized proportional term, not Stanley's raw
        # cross-track distance.
        self._camera_max_steer_deg_base = float(self._param("lane_max_steer_deg", 30.0))
        self.speed_damp_gain = float(self._param("speed_damp_gain", 0.15))
        self.speed_damp_min_scale = float(self._param("speed_damp_min_scale", 0.4))
        # Prefer GPS-derived speed (waypoint_follower_node's
        # gps_control/speed_mps, from consecutive fix positions) over
        # ZED's own visual odometry for the damping calc specifically -
        # more reliable outdoors on featureless roads where VIO can drift.
        # Falls back to self.speed_mps (ZED odom) if GPS speed hasn't
        # published recently (dead-man's switch, same pattern as
        # elsewhere in this codebase) - e.g. GPS node not running, so the
        # camera can still be tested standalone.
        self.gps_speed_topic = str(self._param("gps_speed_topic", "gps_control/speed_mps"))
        self.gps_speed_timeout_sec = float(self._param("gps_speed_timeout_sec", 1.0))
        self.gps_speed_mps = float('nan')
        self._gps_speed_last_ns = None

        self.add_on_set_parameters_callback(self._on_param_change)
        self.create_timer(0.05, self._publish_timer_cb)  # 20Hz

        # Caps the inference loop's throughput so it doesn't burn GPU
        # cycles racing ahead faster than needed (e.g. 64fps capacity ->
        # capped to 50). Purely a resource-usage concern - does not affect
        # arbiter reaction time, which is already fixed at 20Hz via the
        # publish timer above, independent of inference speed.
        self.max_fps = float(self._param("max_fps", 50.0))
        self._min_frame_interval = 1.0 / self.max_fps if self.max_fps > 0 else 0.0

        # ------- queue/worker -------
        self.q = deque(maxlen=2)
        self.lock = threading.Lock()
        self.worker = threading.Thread(target=self._infer_loop, daemon=True)
        self.worker.start()

        # ------- 동기화 구독 -------
        sensor_qos = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,   # 필요시 BEST_EFFORT
        )
        sub_rgb = message_filters.Subscriber(self, RosImage, self.rgb_topic,   qos_profile=sensor_qos)
        sub_dep = message_filters.Subscriber(self, RosImage, self.depth_topic, qos_profile=sensor_qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_rgb, sub_dep], queue_size=30, slop=0.30)
        self.sync.registerCallback(self._on_sync)

        self.create_subscription(CameraInfo, self.cinfo_topic, self._on_caminfo, 1)
        self.create_subscription(Odometry,   self.odom_topic,  self._on_odom,    5)
        self.create_subscription(
            Float32, self.gps_speed_topic, self._on_gps_speed, 10
        )

        self.get_logger().info("yolopv2_zed_node ready (LaneTracker).")

    # ---------- 파라미터 ----------
    def _param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _on_param_change(self, params):
        for p in params:
            n, v = p.name, p.value
            if   n == "src_p1_x": self.src_pct[0] = float(v)
            elif n == "src_p1_y": self.src_pct[1] = float(v)
            elif n == "src_p2_x": self.src_pct[2] = float(v)
            elif n == "src_p2_y": self.src_pct[3] = float(v)
            elif n == "src_p3_x": self.src_pct[4] = float(v)
            elif n == "src_p3_y": self.src_pct[5] = float(v)
            elif n == "src_p4_x": self.src_pct[6] = float(v)
            elif n == "src_p4_y": self.src_pct[7] = float(v)
            elif n == "stop_y_offset":  self.stop_y_offset = int(v)
            elif n == "stop_segments":  self.stop_segments = int(v)
            elif n == "stop_threshold": self.stop_threshold = int(v)
            elif n == "stop_min_hits":  self.stop_min_hits = int(v)
            elif n == "wheel_radius":   self.wheel_radius = float(v)
            elif n == "gear_ratio":     self.gear_ratio = float(v)
            # --- CAN ---
            elif n == "motor_enable":   self.motor_enable = int(v)
            elif n == "stop_mode":      self.stop_mode = int(v)
            elif n == "can_target_rpm": self.can_target_rpm = int(v)
            elif n == "stop_on_lane_lost": self.stop_on_lane_lost = bool(v)
            elif n == "auto_speed":         self.auto_speed = bool(v)
            elif n == "rpm_turn_scale":     self.rpm_turn_scale = float(v)
            elif n == "steer_deadzone_deg": self.steer_deadzone_deg = float(v)
            elif n == "steer_full_deg":     self.steer_full_deg = float(v)
            elif n == "rpm_step":           self.rpm_step = int(v)
            elif n == "steer_sign":     self.steer_sign = int(v)
            elif n == "steer_gain":     self.steer_gain = float(v)
            elif n == "rpm_min":        self.rpm_min = int(v)
            elif n == "rpm_max":        self.rpm_max = int(v)
            elif n == "steer_min":      self.steer_min = int(v)
            elif n == "steer_max":      self.steer_max = int(v)
            # --- 라인트래킹 ---
            elif n == "lane_roi_y_ratio":         self.lane.roi_y_ratio = float(v)
            elif n == "lane_half_width_ratio":    self.lane.half_width_ratio = float(v)
            elif n == "lane_half_width_smooth":   self.lane.half_width_smooth = float(v)
            elif n == "lane_max_candidates":      self.lane.max_candidates = int(v)
            # Sets the undamped base - self.lane.max_steer_deg itself is
            # overwritten every frame by the speed-damping calc, not here.
            elif n == "lane_max_steer_deg":       self._camera_max_steer_deg_base = float(v)
            elif n == "speed_damp_gain":          self.speed_damp_gain = float(v)
            elif n == "speed_damp_min_scale":     self.speed_damp_min_scale = float(v)
            elif n == "lane_steer_smooth":        self.lane.steer_smooth = float(v)
            elif n == "lane_max_step_deg":        self.lane.max_step_deg = float(v)
            elif n == "lane_lost_hold_frames":    self.lane.lost_hold_frames = int(v)
            elif n == "lane_max_mask_ratio":      self.lane.max_mask_ratio = float(v)
            elif n == "lane_num_bands":           self.lane.num_bands = int(v)
            elif n == "lane_min_fit_bands":       self.lane.min_fit_bands = int(v)
            elif n == "lane_curvature_gain_deg":  self.lane.curvature_gain_deg = float(v)
            elif n == "lane_curvature_max_deg":   self.lane.curvature_max_deg = float(v)
            elif n == "infer_stale_timeout_sec":  self.infer_stale_timeout_sec = float(v)

        if self.frame_wh is not None:
            w0, h0 = self.frame_wh
            self.persp_M, self.src_pts = build_perspective_from_pct(self.src_pct, w0, h0)
        return SetParametersResult(successful=True)

    # ---------- ROS 콜백 ----------
    def _on_caminfo(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float32).reshape(3, 3)

    def _on_odom(self, msg: Odometry):
        self.speed_mps = float(msg.twist.twist.linear.x)

    def _on_gps_speed(self, msg: Float32):
        self.gps_speed_mps = float(msg.data)
        self._gps_speed_last_ns = time.time_ns()

    def _damping_speed_mps(self):
        """GPS speed if it's published recently, else fall back to ZED's
        own VIO speed_mps - see gps_speed_topic declaration."""
        if self._gps_speed_last_ns is not None:
            age_sec = (time.time_ns() - self._gps_speed_last_ns) / 1e9
            if age_sec <= self.gps_speed_timeout_sec and not np.isnan(self.gps_speed_mps):
                return self.gps_speed_mps
        return self.speed_mps if not np.isnan(self.speed_mps) else 0.0

    def _mps_to_motor_rpm(self, speed_mps):
        if speed_mps is None or np.isnan(speed_mps):
            return float('nan')
        circ = 2.0 * np.pi * self.wheel_radius
        if circ <= 1e-9:
            return float('nan')
        return (float(speed_mps) / circ) * 60.0 * self.gear_ratio

    def _on_sync(self, rgb_msg: RosImage, depth_msg: RosImage):
        if self.quit: return
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')

        if self.frame_wh is None:
            h0, w0 = rgb.shape[:2]
            self.frame_wh = (w0, h0)
            self.persp_M, self.src_pts = build_perspective_from_pct(self.src_pct, w0, h0)

        img = letterbox(rgb, self.img_size, stride=32)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR->RGB, HWC->CHW
        img = np.ascontiguousarray(img)

        try:
            tstamp = Time.from_msg(rgb_msg.header.stamp).nanoseconds * 1e-9
        except Exception:
            tstamp = time.time()

        with self.lock:
            if not self.quit:
                self.q.append((rgb, img, tstamp))

    def _log_row(self, tstamp, res, speed_mps):
        if self.csv_writer is None:
            return
        try:
            ts_iso = datetime.fromtimestamp(float(tstamp), tz=timezone.utc).isoformat()
        except Exception:
            ts_iso = ""

        sp = 0.0 if (speed_mps is None or np.isnan(speed_mps)) else float(speed_mps)
        rpm = self._mps_to_motor_rpm(speed_mps)
        rpm = 0.0 if np.isnan(rpm) else float(rpm)

        with self.log_lock:
            try:
                self.csv_writer.writerow([
                    ts_iso, int(res.valid), res.side,
                    round(res.lane_center_x, 2), round(res.error_px, 2),
                    round(res.half_width, 1), res.pixel_count,
                    sp, rpm, round(res.steer_deg, 3)
                ])
                self._csv_count += 1
                if (self._csv_count % self._csv_flush_every) == 0:
                    self.csv_fh.flush()
            except Exception as e:
                self.get_logger().warn(f"[csv] write failed: {e}", throttle_duration_sec=5.0)

    # ---------- 추론 루프 (워커 스레드) ----------
    def _infer_loop(self):
        while rclpy.ok() and not self.quit:
            loop_start = time.time()
            item = None
            with self.lock:
                if self.q: item = self.q.popleft()
            if item is None:
                time.sleep(0.002); continue
            rgb, img_np, tstamp = item

            img_t = torch.from_numpy(img_np).to(self.dev)
            img_t = img_t.half() if self.half else img_t.float()
            img_t /= 255.0
            if img_t.ndim == 3: img_t = img_t.unsqueeze(0)

            t1 = time_synchronized()
            with torch.no_grad():
                (pred_raw, anchor_grid), seg, ll = self.model(img_t)
            t2 = time_synchronized(); self.t_inf.update(t2 - t1)

            ts1 = time_synchronized()
            pred = split_for_trace_model(pred_raw, anchor_grid)
            ts2 = time_synchronized(); self.t_split.update(ts2 - ts1)

            t3 = time_synchronized()
            pred = non_max_suppression(pred, self.conf_th, self.iou_th,
                                       classes=None, agnostic=self.agnostic)
            t4 = time_synchronized(); self.t_nms.update(t4 - t3)

            # 차선 마스크
            ll_mask = cv2.resize(lane_line_mask(ll), rgb.shape[1::-1],
                                 interpolation=cv2.INTER_NEAREST).astype(np.uint8) * 255

            sp = self.speed_mps if not np.isnan(self.speed_mps) else 0.0

            vis = rgb.copy()
            bev = None
            stop_flag = False

            if np.any(ll_mask):
                bev = bird_eye_view_transform(ll_mask, self.persp_M)

            # ===== 라인트래킹 =====
            damp_scale = clamp(
                1.0 / (1.0 + self.speed_damp_gain * max(self._damping_speed_mps(), 0.0)),
                self.speed_damp_min_scale, 1.0,
            )
            self.lane.max_steer_deg = self._camera_max_steer_deg_base * damp_scale
            lane_res = self.lane.update(bev)
            self.lane_valid = lane_res.valid
            self.steering_deg = float(lane_res.steer_deg) if lane_res.valid else float('nan')
            self._last_infer_wall_time = time.time()

            if bev is not None:
                stop_flag = detect_stop_line(
                    bev, horizontal_line_y_offset=self.stop_y_offset,
                    segment_count=self.stop_segments, threshold=self.stop_threshold,
                    min_segments=self.stop_min_hits)

            self._log_row(tstamp, lane_res, sp)

            # ===== HUD =====
            cv2.putText(vis, f"Steer: {lane_res.steer_deg:6.2f} deg [{lane_res.side}]",
                        (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (0, 255, 0) if lane_res.valid else (0, 0, 255), 2)
            cv2.putText(vis, f"Speed: {sp:5.2f} m/s", (30, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            rpm_disp = self._mps_to_motor_rpm(sp)
            rpm_disp = 0.0 if np.isnan(rpm_disp) else rpm_disp
            cv2.putText(vis, f"Motor: {rpm_disp:6.1f} rpm  CMD:{self._rpm_cmd}", (30, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            for x, y in getattr(self, "src_pts", []):
                cv2.circle(vis, (int(x), int(y)), 5, (0, 255, 255), -1)

            lanemask_bgr = cv2.cvtColor(ll_mask, cv2.COLOR_GRAY2BGR)
            if bev is None:
                bev = ll_mask.copy()
            bev_u8 = bev if bev.dtype == np.uint8 else np.clip(bev, 0, 255).astype(np.uint8)
            bev_vis = cv2.cvtColor(bev_u8, cv2.COLOR_GRAY2BGR)
            self.lane.draw_debug(bev_vis, lane_res)

            stop_img = cv2.cvtColor(bev_u8, cv2.COLOR_GRAY2BGR)
            y_line = stop_img.shape[0] - int(self.stop_y_offset)
            y_line = max(0, min(stop_img.shape[0] - 1, y_line))
            cv2.line(stop_img, (0, y_line), (stop_img.shape[1], y_line), (0, 0, 255), 2)
            txt = 'Stop Line Detected' if stop_flag else 'No Stop Line'
            cv2.putText(stop_img, txt, (50, max(20, y_line - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (0, 0, 255) if stop_flag else (200, 200, 200), 2)

            self._show(vis, bev_vis, stop_img, lanemask_bgr, bev_u8, self._fps_est())

            elapsed = time.time() - loop_start
            remaining = self._min_frame_interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

    # ---------- 20Hz 퍼블리시 + CAN ----------
    def _publish_timer_cb(self):
        steer = self.steering_deg if not np.isnan(self.steering_deg) else 0.0
        lane_ok = bool(self.lane_valid)

        infer_stale = (
            self._last_infer_wall_time is None
            or (time.time() - self._last_infer_wall_time) > self.infer_stale_timeout_sec
        )
        if infer_stale:
            lane_ok = False
            steer = 0.0

        try:
            self.pub_steer.publish(Float32(data=float(steer)))
            self.pub_lane_valid.publish(Bool(data=lane_ok))
        except Exception:
            pass

        if np.isnan(self.speed_mps):
            if not self._warned_no_odom:
                self.get_logger().warn(
                    f"[odom] no data from {self.odom_topic}; publishing 0.0 m/s",
                    throttle_duration_sec=5.0)
                self._warned_no_odom = True
            sp = 0.0
        else:
            sp = float(self.speed_mps)

        rpm = self._mps_to_motor_rpm(self.speed_mps)
        rpm = 0.0 if np.isnan(rpm) else float(rpm)
        try:
            self.pub_speed.publish(Float32(data=sp))
            self.pub_rpm.publish(Float32(data=rpm))
        except Exception:
            pass

        # ------- 커브 기반 rpm 계산 + publish (can_enable 무관, 2026-08-17) -------
        # 예전엔 이 블록 전체가 "if self.can is not None:" 안에 있어서
        # can_enable=false(= integrated/post_gps 런처의 정상 설정, arbiter가
        # 유일한 CAN 송신자여야 함)일 땐 rpm_target 계산 자체가 통째로 안
        # 돌았음 - 그래서 arbiter가 카메라 주행 rpm으로 자기 고정 파라미터
        # (camera_mode_rpm)만 쓸 수밖에 없었음. 이제 계산+publish는 항상
        # 하고, 실제 CAN 전송만 can_enable에 따름.
        steer_cmd = int(round(steer * self.steer_sign * self.steer_gain))
        steer_cmd = clamp(steer_cmd, self.steer_min, self.steer_max)

        # 차선 유실 확정 시 rpm 0 (정지)
        if self.stop_on_lane_lost and not lane_ok:
            rpm_target = 0
            steer_cmd = 0
        else:
            rpm_target = self._speed_for_steer(steer)

        # 스텝 제한 (급가감속 방지)
        step = max(1, int(self.rpm_step))
        if rpm_target > self._rpm_cmd + step:
            self._rpm_cmd += step
        elif rpm_target < self._rpm_cmd - step:
            self._rpm_cmd -= step
        else:
            self._rpm_cmd = rpm_target
        rpm_cmd = clamp(int(self._rpm_cmd), self.rpm_min, self.rpm_max)

        try:
            self.pub_rpm_target.publish(Float32(data=float(rpm_cmd)))
        except Exception:
            pass

        # ------- CAN 송신 (can_enable=true일 때만) -------
        if self.can is not None:
            try:
                self.can.send_control(rpm_cmd, steer_cmd, self.motor_enable, 0)
                if self.can_read_feedback:
                    self.can.poll_feedback()
            except Exception as e:
                self.get_logger().warn(f"[can] send 실패: {e}",
                                       throttle_duration_sec=2.0)

    def _speed_for_steer(self, steer_deg):
        """
        조향각 -> 목표 rpm.

        |steer| <= steer_deadzone_deg          -> can_target_rpm (직진)
        |steer| >= steer_full_deg              -> can_target_rpm * rpm_turn_scale
        그 사이                                 -> 선형 보간

        auto_speed=false 면 can_target_rpm 고정.
        """
        base = int(self.can_target_rpm)

        if not self.auto_speed or base == 0:
            return base

        dz = max(0.0, float(self.steer_deadzone_deg))
        full = max(dz + 1e-3, float(self.steer_full_deg))
        a = abs(float(steer_deg))

        t = clamp((a - dz) / (full - dz), 0.0, 1.0)   # 0=직진, 1=최대커브
        scale = 1.0 - (1.0 - float(self.rpm_turn_scale)) * t

        return int(round(base * scale))

    def _fps_est(self):
        total = self.t_inf.avg + self.t_split.avg + self.t_nms.avg + 1e-6
        return 1.0 / total

    def _show(self, hud_bgr, bev_bgr, stop_bgr, lanemask_bgr, bev_mask_u8, fps):
        hud = hud_bgr.copy()
        cv2.putText(hud, f"FPS: {fps:.1f}", (30, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        try:
            self.pub_img_hud.publish(self.bridge.cv2_to_imgmsg(hud, "bgr8"))
            self.pub_img_bev.publish(self.bridge.cv2_to_imgmsg(bev_bgr, "bgr8"))
            self.pub_img_stop.publish(self.bridge.cv2_to_imgmsg(stop_bgr, "bgr8"))
            self.pub_img_lanemask.publish(self.bridge.cv2_to_imgmsg(lanemask_bgr, "bgr8"))
            self.pub_img_bevmask.publish(self.bridge.cv2_to_imgmsg(bev_mask_u8, encoding="mono8"))
        except Exception as e:
            self.get_logger().warn(f"[pub] image publish failed: {e}", throttle_duration_sec=2.0)

        if self.show_windows:
            try:
                cv2.imshow("Original + HUD", hud)
                cv2.imshow("Bird-eye view", bev_bgr)
                cv2.imshow("Stop Line", stop_bgr)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.quit = True
                    try: cv2.destroyAllWindows()
                    except Exception: pass
                    rclpy.shutdown()
            except Exception as e:
                self.get_logger().warn(f"[gui] imshow failed: {e}", throttle_duration_sec=2.0)

    def on_shutdown(self):
        self.quit = True

        # CAN 안전 정지
        if getattr(self, "can", None) is not None:
            try:
                self.can.send_control(0, 0, 0, 0)
            except Exception:
                pass
            try:
                self.can.shutdown()
                self.get_logger().info("[can] safe-stop 전송 후 종료")
            except Exception:
                pass

        try: cv2.destroyAllWindows()
        except Exception: pass
        try:
            if getattr(self, "csv_fh", None):
                self.csv_fh.flush()
                self.csv_fh.close()
                self.get_logger().info(f"[csv] saved to {self.csv_path}")
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = YoloPv2ZedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slot_detector.py  -  주차공간(슬롯) 검출 재작성판.

[왜 재작성했는가]
ROS1 미니카 판(estimate_u_slot)은 다음 구조였다.
  - 진행축(road_axis)을 sequence_start_yaw(= 출발 시 odom yaw)로 고정
  - 모든 콘쌍을 완전탐색하며 폭/깊이차 게이트
  - 깊이는 전체 점의 percentile
문제:
  1) 축이 odom 에만 의존 -> 접근 중 yaw 드리프트가 그대로 슬롯 각도 오차가 된다.
     미니카는 이동거리가 짧아 무시됐지만 실차는 접근거리가 길어 누적된다.
  2) 슬롯 내부가 "비었는지"를 콘 개수로만 간접 확인 -> 슬롯에 차가 서 있으면 통과한다.
  3) 폭 게이트가 절대값(0.68~0.92) -> 차폭 0.800 m 차량에는 그대로 못 쓴다.

[재작성 원리]
  1. 차선축을 콘에서 직접 구한다 (RANSAC + 총최소제곱)
     주차면 쪽 콘들은 도로와 평행한 한 줄(바깥 열)을 이룬다.
     그 직선에 적합해 lane_dir 을 얻고, odom 진행축을 seed 로 주어
     lane_max_angle_deg 이내 해만 채택한다. -> 드리프트에 둔감.

  2. 게이트를 모두 "차선 프레임"에서 수행
     u = lane_dir 방향(도로 진행), v = 슬롯 안쪽 방향(주차면 쪽).
     입구 콘쌍은 (a) 둘 다 차선 직선 근처, (b) u 간격이 슬롯폭 범위,
     (c) v 편차가 작음(대각선쌍 배제).

  3. 깊이는 "슬롯 내부 콘"에서 구한다
     입구 폭 밴드 안쪽의 콘 중 v 가 가장 큰 군집을 뒷벽으로 보고 median.
     콘이 없으면 raw 스캔점 percentile 로 폴백.

  4. 내부 공실 검사 2중화
     (a) 콘 기반: 입구~뒷벽 사이 내부에 콘이 있으면 탈락
     (b) raw 스캔 기반: 슬롯 사각형 내부에 뒷벽보다 앞선 점군이 있으면 탈락
         -> 슬롯에 주차된 차/사람을 걸러낸다. 미니카 판에는 없던 검사.

  5. 차량 제원 기반 하드 게이트
     entry_width >= vehicle_width + 2*side_margin
     depth       >= base_to_rear_bumper + back_margin + min_goal_inside_depth
"""

import math

import numpy as np

from .geometry import (normalize_angle, ransac_line, robust_percentile,
                       robust_median, unit)


class SlotParams(object):
    def __init__(self, **kw):
        # --- 차선축 적합 ---
        self.lane_fit_enable = True
        self.lane_ransac_tol = 0.18
        self.lane_ransac_iters = 150
        self.lane_max_angle_deg = 25.0
        self.lane_min_inliers = 3

        # --- 입구 게이트 (실차: 차폭 0.800) ---
        self.entry_min_width = 1.05
        self.entry_max_width = 2.20
        self.entry_expected_width = 1.30
        self.entry_pair_max_depth_diff = 0.30
        self.entry_line_tol = 0.30          # 입구 콘이 차선직선에서 벗어난 허용치
        self.max_candidate_entry_dist = 7.0
        # 차량 진행축에서 주차 입구까지의 횡거리 범위. 0이면 비활성.
        # 가까운 차선 경계 콘 사이의 틈을 주차 입구로 오인하는 것을 막는다.
        self.candidate_min_lateral_abs = 0.0
        self.candidate_max_lateral_abs = 0.0
        # 입구 콘쌍 사이에 다른 '차선 콘'이 끼어 있으면 그건 입구가 아니다.
        # (차선 콘 두 개를 건너뛰어 만든 가짜 슬롯을 제거하는 결정적 게이트)
        self.entry_require_adjacent = True
        self.entry_adjacent_margin = 0.15
        # 촘촘한 콘 경계에서 최소~최대 폭 사이의 실제 빈 간격만 입구로
        # 사용한다. 뒷벽 콘 배치는 인식 필수조건으로 삼지 않는다.
        self.entrance_gap_mode = False
        self.entrance_default_depth = 2.00
        # 뒷벽 깊이 근거가 폭 밴드 가장자리 콘 하나뿐이면 가짜다.
        # 중앙 근처(|q| <= ratio*W)에 최소 1개의 근거가 있어야 한다.
        self.depth_center_band_ratio = 0.35
        self.require_depth_center_support = True

        # --- 깊이 ---
        self.slot_min_depth = 1.50
        self.slot_max_depth = 3.50
        self.slot_expected_depth = 2.00
        self.depth_percentile = 88.0
        self.rear_cone_band = 0.30
        self.wall_lateral_extra = 0.25
        self.depth_min_positive_v = 0.15
        self.goal_depth_override = 0.0

        # --- 내부 공실 검사 ---
        self.interior_cone_reject = True
        self.interior_margin_u = 0.12       # 입구 폭에서 좌우로 줄일 여유
        self.interior_back_band = 0.35      # 뒷벽으로 인정할 밴드
        self.interior_front_band = 0.10     # 입구선 바로 안쪽 무시 밴드
        self.max_interior_cones = 0

        self.interior_raw_reject = True
        self.interior_raw_min_points = 6
        self.interior_raw_clear_ratio = 0.75  # 뒷벽*0.75 보다 앞에 점군 -> 점유

        # --- 입구 앞쪽 경계 검사 (원본 유지) ---
        self.entry_boundary_check_enable = True
        self.entry_front_tolerance = 0.20
        self.max_entry_front_count = 1

        # --- 목표점 ---
        self.entry_offset = 0.05
        self.min_goal_inside_depth = 0.10

        # --- 점수 ---
        self.score_width_w = 12.0
        self.score_depth_w = 6.0
        self.score_dist_w = 0.6
        self.score_heading_w = 25.0
        self.score_inside_w = 3.0

        # --- 최소 콘 수 ---
        self.min_cones_for_plan = 3

        for k, v in kw.items():
            if hasattr(self, k):
                cur = getattr(self, k)
                setattr(self, k, v if isinstance(cur, bool) else type(cur)(v))


class SlotDetector(object):
    def __init__(self, params, vehicle):
        """
        vehicle : dict(vehicle_width, side_margin, back_margin,
                       base_to_rear_bumper)
        """
        self.p = params
        self.veh = vehicle
        self.debug = {}

    # ------------------------------------------------------------------
    def detect(self, cones_base, road_axis_base, side, raw_points_base=None):
        """
        cones_base     : list of (2,) base_link 콘 좌표 (확정 트랙)
        road_axis_base : (2,) base_link 에서 본 도로 진행방향 단위벡터
        side           : 'left' | 'right'
        raw_points_base: (M,2) 내부 공실검사용 원시 스캔점 (없으면 생략)

        return candidate dict 또는 None
        """
        p = self.p
        dbg = {'n_cones': len(cones_base), 'pairs': 0,
               'rej_width': 0, 'rej_diag': 0, 'rej_line': 0, 'rej_depth': 0,
               'rej_adjacent': 0, 'rej_center': 0,
               'rej_lateral': 0,
               'rej_interior_cone': 0, 'rej_interior_raw': 0,
               'rej_front': 0, 'rej_vehicle': 0, 'accepted': 0}
        self.debug = dbg

        if len(cones_base) < p.min_cones_for_plan:
            return None

        arr = np.asarray(cones_base, dtype=float).reshape(-1, 2)
        side_sign = +1.0 if side == 'left' else -1.0

        # ---------- 1) 차선축 적합 ----------
        lane_dir, lane_c, lane_mask = self._fit_lane_axis(arr, road_axis_base)
        dbg['lane_inliers'] = int(np.count_nonzero(lane_mask))
        dbg['lane_angle_deg'] = math.degrees(
            normalize_angle(math.atan2(lane_dir[1], lane_dir[0])
                            - math.atan2(road_axis_base[1], road_axis_base[0])))

        # 슬롯 안쪽 방향 v : 차선축을 side 방향으로 90도 회전
        v_dir = np.array([-side_sign * lane_dir[1],
                          side_sign * lane_dir[0]], dtype=float)
        u_dir = lane_dir

        # ---------- 2) 입구 콘쌍 탐색 ----------
        u_all = arr @ u_dir
        v_all = arr @ v_dir

        best, best_score = None, -1e18
        best_boundary_lateral = None
        n = arr.shape[0]
        for i in range(n):
            for j in range(i + 1, n):
                dbg['pairs'] += 1
                PA, PB = arr[i], arr[j]
                du = float(u_all[j] - u_all[i])
                dv = float(v_all[j] - v_all[i])
                W = abs(du)

                if W < p.entry_min_width or W > p.entry_max_width:
                    dbg['rej_width'] += 1
                    continue
                if abs(dv) > p.entry_pair_max_depth_diff:
                    dbg['rej_diag'] += 1
                    continue

                # 비-gap 모드에서만 RANSAC 주 경계선 소속을 요구한다.
                # gap 모드는 접근로/입구/측벽 콘이 동시에 보이는 T자 환경이라
                # 전체 콘 RANSAC 결과를 입구 판정에 사용하면 축이 기울 수 있다.
                have_lane_fit = bool(np.any(lane_mask))
                if (not p.entrance_gap_mode and have_lane_fit
                        and not (lane_mask[i] and lane_mask[j])):
                    dbg['rej_line'] += 1
                    continue

                # 두 콘 사이에 같은 경계선의 다른 콘이 있으면 실제 빈 입구가
                # 아니다. 이 검사는 RANSAC을 끈 gap 모드에서도 반드시 수행한다.
                # 따라서 특정 입구 폭/차량 횡거리를 외우는 패턴 검출이 아니라,
                # 매 프레임 실측된 인접 콘 간격을 그대로 사용한다.
                if p.entry_require_adjacent:
                    ulo = min(float(u_all[i]), float(u_all[j]))
                    uhi = max(float(u_all[i]), float(u_all[j]))
                    pair_v = 0.5 * (float(v_all[i]) + float(v_all[j]))
                    if p.entrance_gap_mode or not have_lane_fit:
                        same_boundary = (
                            np.abs(v_all - pair_v) <= p.entry_line_tol)
                    else:
                        same_boundary = lane_mask
                    between = (
                        same_boundary
                        & (u_all > ulo + p.entry_adjacent_margin)
                        & (u_all < uhi - p.entry_adjacent_margin))
                    # i, j 자신은 엄격한 u 범위 밖이라 별도 제외할 필요가 없다.
                    if bool(np.any(between)):
                        dbg['rej_adjacent'] += 1
                        continue

                M = 0.5 * (PA + PB)
                if float(np.linalg.norm(M)) > p.max_candidate_entry_dist:
                    continue
                lateral_abs = abs(float(np.dot(M, v_dir)))
                if (p.candidate_min_lateral_abs > 0.0
                        and lateral_abs < p.candidate_min_lateral_abs):
                    dbg['rej_lateral'] += 1
                    continue
                if (p.candidate_max_lateral_abs > 0.0
                        and lateral_abs > p.candidate_max_lateral_abs):
                    dbg['rej_lateral'] += 1
                    continue

                # 차량 제원 하드 게이트
                if W < self.veh['vehicle_width'] + 2.0 * self.veh['side_margin']:
                    dbg['rej_vehicle'] += 1
                    continue

                # 입구 콘쌍이 이루는 선이 차선축과 평행해야 한다
                heading_err = math.atan2(abs(dv), max(W, 1e-6))

                # w : 입구선 방향(폭 방향) 단위벡터
                w_dir = u_dir if du >= 0.0 else -u_dir

                rel = arr - M
                s_proj = rel @ v_dir          # 슬롯 안쪽(+) 깊이
                q_proj = rel @ w_dir          # 폭 방향

                # ---------- 3) 깊이 ----------
                if p.entrance_gap_mode:
                    # 실제 코스는 입구 양끝 콘 사이의 빈 간격으로 슬롯을
                    # 정의한다. 폭은 실측 W를 그대로 쓰고, 깊이는 경로 생성을
                    # 위한 설정값을 사용한다. 내부 안전은 아래 raw scan
                    # 공실 검사로 별도 확인한다.
                    D = float(p.entrance_default_depth)
                    inside_lat = 0
                    back_cnt = 0
                    center_ok = True
                else:
                    D, inside_lat, back_cnt, center_ok = self._measure_depth(
                        s_proj, q_proj, W, raw_points_base, M, v_dir, w_dir)
                    if D is None or D < p.slot_min_depth or D > p.slot_max_depth:
                        dbg['rej_depth'] += 1
                        continue
                    # 뒷벽 근거가 폭 밴드 가장자리 하나뿐이면 가짜 슬롯이다
                    if p.require_depth_center_support and not center_ok:
                        dbg['rej_center'] += 1
                        continue

                if D < p.slot_min_depth or D > p.slot_max_depth:
                    dbg['rej_depth'] += 1
                    continue

                # 목표 깊이 (뒤 범퍼 + 후방여유)
                D_goal = p.goal_depth_override if p.goal_depth_override > 0.0 else D
                z_goal = float(D_goal - self.veh['base_to_rear_bumper']
                               - self.veh['back_margin'])
                if z_goal < p.min_goal_inside_depth:
                    dbg['rej_depth'] += 1
                    continue

                # ---------- 4) 입구 앞쪽 경계 ----------
                lat = 0.5 * W + p.wall_lateral_extra
                if p.entry_boundary_check_enable and not p.entrance_gap_mode:
                    front_cnt = int(np.count_nonzero(
                        (s_proj < -p.entry_front_tolerance) & (np.abs(q_proj) <= lat)))
                    if front_cnt > p.max_entry_front_count:
                        dbg['rej_front'] += 1
                        continue

                # ---------- 5) 내부 공실 (콘) ----------
                if p.interior_cone_reject and not p.entrance_gap_mode:
                    lat_in = max(0.5 * W - p.interior_margin_u, 0.05)
                    interior = ((s_proj > p.interior_front_band) &
                                (s_proj < D - p.interior_back_band) &
                                (np.abs(q_proj) <= lat_in))
                    n_int = int(np.count_nonzero(interior))
                    if n_int > p.max_interior_cones:
                        dbg['rej_interior_cone'] += 1
                        continue

                # ---------- 6) 내부 공실 (raw scan) ----------
                if p.interior_raw_reject and raw_points_base is not None \
                        and len(raw_points_base) > 0:
                    if self._interior_occupied_raw(raw_points_base, M, v_dir,
                                                   w_dir, W, D):
                        dbg['rej_interior_raw'] += 1
                        continue

                # ---------- 7) 점수 ----------
                inside_cnt = int(np.count_nonzero(s_proj > p.depth_min_positive_v))
                entry_dist = float(np.linalg.norm(M))
                if p.entrance_gap_mode:
                    # 정해진 한 폭에 맞추지 않고 허용 범위 안의 가장 큰
                    # 인접 빈 간격을 사용한다. 다만 T자 코스에는 입구 뒤쪽
                    # 측벽/후방 콘 열에도 큰 틈이 생길 수 있으므로, 서로
                    # 다른 경계선끼리는 차량에서 처음 만나는 경계선을 우선한다.
                    # 같은 경계선(entry_line_tol 이내) 안에서만 더 넓고
                    # 안정적인 빈 간격을 고른다.
                    score = (10.0 * W
                             - p.score_dist_w * entry_dist
                             - p.score_heading_w * heading_err)
                else:
                    score = (p.score_inside_w * inside_cnt
                             + 1.5 * inside_lat
                             + 1.0 * back_cnt
                             - p.score_width_w * abs(W - p.entry_expected_width)
                             - p.score_depth_w * abs(D - p.slot_expected_depth)
                             - p.score_dist_w * entry_dist
                             - p.score_heading_w * heading_err)

                if p.entrance_gap_mode and best is not None:
                    # 횡거리 차이가 line_tol 보다 크면 별개의 평행 경계선이다.
                    # 가장 가까운 유효 경계선은 실제 입구이고, 그 뒤 경계의
                    # 더 큰 간격이 점수로 입구를 빼앗지 못하게 한다.
                    if lateral_abs > best_boundary_lateral + p.entry_line_tol:
                        continue
                    same_boundary_band = (
                        abs(lateral_abs - best_boundary_lateral)
                        <= p.entry_line_tol)
                    if same_boundary_band and score <= best_score:
                        continue
                    # 충분히 가까운 새 경계선이면 폭 점수와 무관하게 교체한다.
                elif score <= best_score:
                    continue

                origin = M
                P_entry = origin - p.entry_offset * v_dir
                P_goal = origin + z_goal * v_dir
                P_mid = origin + max(0.5 * z_goal, p.min_goal_inside_depth) * v_dir
                # 최종 자세는 슬롯 밖을 향한다 (후진 진입이므로)
                yaw_goal_base = normalize_angle(
                    math.atan2(v_dir[1], v_dir[0]) + math.pi)

                best_score = score
                best_boundary_lateral = lateral_abs
                best = {
                    'PL': PA if du < 0 else PB,
                    'PR': PB if du < 0 else PA,
                    'M': origin, 'w': w_dir, 'd': v_dir,
                    'lane_dir': u_dir,
                    'entry_width': W, 'depth': D, 'z_goal': z_goal,
                    'inside_count': inside_cnt, 'score': score,
                    'heading_err': heading_err,
                    'P_entry': P_entry, 'P_mid': P_mid, 'P_goal': P_goal,
                    'yaw_goal_base': yaw_goal_base, 'center_origin': origin,
                    'cones': [np.array(c, dtype=float) for c in arr],
                }
                dbg['accepted'] += 1
        return best

    # ------------------------------------------------------------------
    def _fit_lane_axis(self, arr, road_axis_base):
        """
        콘 열에 직선을 적합해 차선축을 콘에서 직접 얻는다.
        return (lane_dir, lane_centroid, lane_inlier_mask)
        적합 실패 시 odom 진행축으로 폴백하고 마스크는 전부 False.
        """
        p = self.p
        fallback = (road_axis_base, np.zeros(2, dtype=float),
                    np.zeros(arr.shape[0], dtype=bool))
        if not p.lane_fit_enable or arr.shape[0] < p.lane_min_inliers:
            return fallback
        res = ransac_line(arr, tol=p.lane_ransac_tol,
                          iters=p.lane_ransac_iters,
                          seed_dir=road_axis_base,
                          max_angle=math.radians(p.lane_max_angle_deg))
        if res is None:
            return fallback
        c, d, mask, _rms = res
        if int(np.count_nonzero(mask)) < p.lane_min_inliers:
            return fallback
        ang = abs(normalize_angle(math.atan2(d[1], d[0])
                                  - math.atan2(road_axis_base[1], road_axis_base[0])))
        if ang > math.radians(p.lane_max_angle_deg):
            return fallback
        nrm = float(np.linalg.norm(d))
        if nrm <= 1e-9:
            return fallback
        d = d / nrm
        # 적합 직선에서 entry_line_tol 안에 있는 콘을 '차선 콘' 으로 확정
        nvec = np.array([-d[1], d[0]], dtype=float)
        lane_mask = np.abs((arr - c) @ nvec) <= p.entry_line_tol
        return d, c, lane_mask

    # ------------------------------------------------------------------
    def _measure_depth(self, s_proj, q_proj, W, raw_points_base, M, v_dir, w_dir):
        """슬롯 깊이 D. 콘 우선, 없으면 raw 스캔 percentile."""
        p = self.p
        lat = 0.5 * W + p.wall_lateral_extra
        band = (s_proj > p.depth_min_positive_v) & (np.abs(q_proj) <= lat)
        sv = s_proj[band]
        qv = q_proj[band]
        inside_lat = int(sv.size)

        if sv.size >= 1:
            D_pct = robust_percentile(sv, p.depth_percentile,
                                      default=float(np.max(sv)))
            back = sv >= max(p.depth_min_positive_v, D_pct - p.rear_cone_band)
            s_back = sv[back]
            D = robust_median(s_back, default=D_pct) if s_back.size >= 2 else float(D_pct)
            # 뒷벽 근거 중 중앙 근처(|q| <= ratio*W)에 최소 1개가 있는지
            center_ok = bool(np.any(np.abs(qv[back]) <= p.depth_center_band_ratio * W))
            return float(D), inside_lat, int(s_back.size), center_ok

        # 콘이 안쪽에 없으면 raw 스캔으로 뒷벽 추정
        if raw_points_base is None or len(raw_points_base) == 0:
            return None, 0, 0, False
        R = np.asarray(raw_points_base, dtype=float).reshape(-1, 2)
        rel = R - M
        s_r = rel @ v_dir
        q_r = rel @ w_dir
        m = (s_r > p.depth_min_positive_v) & (np.abs(q_r) <= 0.5 * W)
        if int(np.count_nonzero(m)) < p.interior_raw_min_points:
            return None, 0, 0, False
        D = robust_percentile(s_r[m], p.depth_percentile, default=None)
        if D is None:
            return None, 0, 0, False
        back = m & (s_r >= D - p.rear_cone_band)
        center_ok = bool(np.any(np.abs(q_r[back]) <= p.depth_center_band_ratio * W))
        return float(D), 0, int(np.count_nonzero(m)), center_ok

    # ------------------------------------------------------------------
    def _interior_occupied_raw(self, raw_points_base, M, v_dir, w_dir, W, D):
        """
        슬롯 사각형 내부에 뒷벽보다 확실히 앞선 점군이 있으면 점유로 판정.
        슬롯에 다른 차량이 주차되어 있는 경우를 걸러낸다.

        entrance_gap_mode의 콘 코스에서는 입구 폭 거의 전체를 검사하면 측면
        경계 콘의 센서쪽 표면까지 내부 장애물로 세게 된다. 이 모드에서는
        실제 차량 풋프린트가 통과할 중앙 회랑만 검사한다.
        """
        p = self.p
        R = np.asarray(raw_points_base, dtype=float).reshape(-1, 2)
        rel = R - M
        s_r = rel @ v_dir
        q_r = rel @ w_dir
        lat_in = max(0.5 * W - p.interior_margin_u, 0.05)
        if p.entrance_gap_mode:
            vehicle_corridor = (
                0.5 * float(self.veh['vehicle_width'])
                + float(self.veh.get('collision_margin', 0.0)))
            lat_in = min(lat_in, vehicle_corridor)
        s_lo = p.interior_front_band
        s_hi = p.interior_raw_clear_ratio * D
        if s_hi <= s_lo:
            return False
        occ = (s_r > s_lo) & (s_r < s_hi) & (np.abs(q_r) <= lat_in)
        return bool(int(np.count_nonzero(occ)) >= p.interior_raw_min_points)

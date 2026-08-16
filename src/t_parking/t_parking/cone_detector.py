#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cone_detector.py  -  콘 검출 재작성판.

[왜 재작성했는가]
ROS1 미니카 판은 고정 임계값 방식이었다.
    cluster_gap=0.07 / min_cluster_points=1 / max_cluster_width=0.28
차량이 4배 커지고 라이다 장착 높이가 올라간 지금 이 방식은 두 방향으로 동시에 깨진다.

  (a) 라이다를 높이 달아 콘이 "잘 안 잡힘"
      스캔면이 0.15~0.17 m 이므로 콘을 밑동이 아니라 중간 높이에서 자른다.
      잘린 단면(chord)이 밑동보다 훨씬 좁고, 거리가 멀면 점이 1~3개만 맞는다.
      -> min_points 를 올려 잡으면 먼 콘이 전멸하고, 1로 내리면 노이즈가 다 통과한다.

  (b) 인식을 "타이트하게" 해야 함
      고정 폭/점수 임계값은 거리에 따라 의미가 완전히 달라진다.
      6 m 에서 0.28 m 폭 임계값은 벽 조각도 통과시킨다.

[해결 원리 : 임계값을 조이지 않고 물리 모델로 판정한다]
거리 r, 각분해능 dphi 가 주어지면 "콘이라면 반드시 이렇게 보여야 한다"가 계산된다.

  1. 스캔높이에서의 예상 단면폭
         w_exp = cone_base_d * (1 - scan_h / cone_h)
     (원뿔 선형 테이퍼. 밑동 원판이 있으면 plate 옵션으로 하한 보정)

  2. 예상 히트 수
         n_exp = 2*atan(w_exp / 2r) / dphi
     이걸로 min/max 점수를 거리마다 자동 생성한다.
     -> 먼 콘은 1~2점도 통과, 가까운 곳에서 점 1개짜리 노이즈는 탈락,
        점이 n_exp 의 몇 배로 많으면 벽/사람이므로 탈락. (핵심 tight 필터)

  3. 적응형 분할 임계 (Dietmayer breakpoint)
         d_thr = r * sin(dphi)/sin(lambda - dphi) + 3*sigma_r
     고정 7 cm 대신 거리에 비례. 가까이서 과분할, 멀리서 과병합을 동시에 막는다.

  4. 고립성(isolation)
     콘은 주변이 비어 있다. 클러스터 양옆 이웃 빔이 무효이거나 반경차가
     isolation_jump 이상 나야 한다. 벽/연석/차체는 여기서 대부분 죽는다.

  5. 볼록성(convexity)
     점이 4개 이상이면 콘은 센서쪽으로 볼록한 짧은 원호다.
     현(chord) 대비 중앙 부풀음(sagitta)이 양수이고 반경이 콘 반경 근방이어야 한다.
     평면 벽 조각과 콘을 가르는 결정적 조건.

  6. 반경 두께
     클러스터 내 range 폭(radial spread)이 콘 깊이(~w/2) 수준이어야 한다.

[추적]
ROS1 판은 base_link 에서 추적해 차가 움직이면 트랙이 끌려갔다.
여기서는 트랙을 odom 프레임에 저장하므로 주행 중에도 누적 확인이 가능하다.
hits >= min_track_hits 인 트랙만 슬롯 검출에 넘긴다.
"""

import math

import numpy as np


class ConeParams(object):
    """콘 검출 파라미터 묶음."""

    def __init__(self, **kw):
        # --- 콘 물리 모델 (실측해서 넣을 것) ---
        self.cone_height = 0.450          # 콘 전체 높이 [m]
        self.cone_base_diameter = 0.280   # 콘 밑동 지름 [m]
        self.cone_plate_diameter = 0.0    # 밑동 사각판 폭. 0 이면 미사용
        self.scan_height = 0.160          # 지면 기준 스캔면 높이 [m]

        # --- range 게이트 ---
        self.min_range = 0.20
        self.max_range = 8.0
        self.range_sigma = 0.020          # 라이다 거리 노이즈 1sigma

        # --- 적응형 분할 ---
        self.cluster_lambda_deg = 10.0    # Dietmayer lambda
        self.cluster_gap_min = 0.030      # 분할 임계 하한
        self.cluster_gap_max = 0.220      # 분할 임계 상한
        self.max_index_gap = 3            # 빔 인덱스 불연속 허용

        # --- 점수 게이트 (n_exp 기반) ---
        self.min_points_abs = 1
        # S2 DenseBoost는 근거리 콘 하나에도 수백 개 빔이 맞을 수 있다.
        # 해상도와 무관한 너무 낮은 상한은 정상 콘을 벽으로 오판한다.
        # 실제 벽 제거는 아래 n_exp 비율과 물리 폭 게이트가 담당한다.
        self.max_points_abs = 400
        self.n_exp_min_ratio = 0.30       # n >= 0.30 * n_exp
        self.n_exp_max_ratio = 3.20       # n <= 3.20 * n_exp  (초과 -> 벽)

        # --- 폭 게이트 (w_exp 기반) ---
        self.width_min_ratio = 0.25
        self.width_max_ratio = 1.80
        self.width_abs_max = 0.45         # 어떤 경우에도 이 폭 넘으면 콘 아님

        # --- 고립성 ---
        self.isolation_enable = True
        self.isolation_jump = 0.22        # 이웃과 반경차 [m]
        self.isolation_search = 4         # 이웃 탐색 빔 수
        self.isolation_require_both = False  # True 면 양쪽 모두 고립 요구

        # --- 볼록성 ---
        self.convexity_enable = True
        self.convexity_min_points = 4
        self.sagitta_min = 0.004
        # 기하학적 상한: 원호의 현 c, 반경 rho 이면 s = rho - sqrt(rho^2-(c/2)^2)
        # <= rho = c/2 (반원이 다 보일 때 등호). 즉 s/(c/2) <= 1.0 이 참값이고
        # 노이즈 여유를 준 1.25 가 상한이다. (0.75 로 두면 가까운 콘이 전멸한다)
        self.sagitta_max_ratio = 1.25     # sagitta <= 1.25 * (c/2)
        self.radial_spread_max_ratio = 1.30  # spread <= 1.30 * (w_exp/2)

        # --- intensity (반사 테이프 있을 때만) ---
        self.use_intensity = False
        self.min_intensity = 0.0

        # --- 추적 (odom 프레임) ---
        self.track_merge_dist = 0.28
        self.track_alpha = 0.45
        self.min_track_hits = 3
        self.max_track_miss_sec = 1.2
        # 원거리 콘은 가림/약한 반사로 매 프레임 검출되지 않는다. 가까운
        # 노이즈 기준은 그대로 두고, 지정 거리 밖의 트랙만 빨리 확정하고
        # 더 오래 유지한다.
        self.far_track_range = 3.0
        self.far_min_track_hits = 2
        self.far_track_miss_sec = 2.5
        self.max_tracks = 200

        for k, v in kw.items():
            if hasattr(self, k):
                setattr(self, k, type(getattr(self, k))(v) if not isinstance(v, bool) else v)

    # ---------------- 물리 모델 ----------------
    def expected_chord_width(self, unused_range=None):
        """스캔면에서 자른 콘 단면폭."""
        if self.scan_height >= self.cone_height:
            return 0.0
        w = self.cone_base_diameter * (1.0 - self.scan_height / self.cone_height)
        if self.cone_plate_diameter > 0.0 and self.scan_height <= 0.02:
            w = max(w, self.cone_plate_diameter)
        return max(w, 0.0)

    def expected_hits(self, r, dphi):
        """거리 r 에서 콘에 맞는 예상 빔 수."""
        w = self.expected_chord_width()
        if w <= 0.0 or r <= 1e-6 or dphi <= 1e-9:
            return 0.0
        return 2.0 * math.atan2(0.5 * w, r) / dphi

    def max_detect_range(self, dphi, min_hits=1.0):
        """min_hits 개 이상 맞을 수 있는 최대 거리 (물리 한계 진단용)."""
        w = self.expected_chord_width()
        if w <= 0.0 or dphi <= 1e-9:
            return 0.0
        half = 0.5 * min_hits * dphi
        if half <= 1e-9 or half >= 0.5 * math.pi:
            return 0.0
        return 0.5 * w / math.tan(half)


class ConeDetection(object):
    __slots__ = ('center', 'width', 'n', 'range_mean', 'sagitta',
                 'radial_spread', 'i0', 'i1', 'n_exp', 'score')

    def __init__(self, center, width, n, range_mean, sagitta,
                 radial_spread, i0, i1, n_exp, score):
        self.center = center
        self.width = width
        self.n = n
        self.range_mean = range_mean
        self.sagitta = sagitta
        self.radial_spread = radial_spread
        self.i0 = i0
        self.i1 = i1
        self.n_exp = n_exp
        self.score = score


class ConeDetector(object):
    """단일 스캔 -> 콘 후보. 프레임 누적은 ConeTracker 가 담당."""

    def __init__(self, params):
        self.p = params
        self.last_reject = {}

    # ---------------- 메인 ----------------
    def detect(self, ranges, angles_vehicle, xy_base, valid_mask,
               intensities=None, sensor_origin=(0.0, 0.0)):
        """
        ranges          : (N,) 원본 거리
        angles_vehicle  : (N,) 차량프레임 각도 (뒤집힘/장착 yaw 이미 반영)
        xy_base         : (N,2) base_link 좌표
        valid_mask      : (N,) ROI + range + 자차마스크 통과 여부
        sensor_origin   : base_link 에서의 라이다 위치. 콘 중심 보정에 필요하다.
                          (앞범퍼 장착이라 base_link 원점과 1.175 m 다르다)
        """
        p = self.p
        N = int(len(ranges))
        rej = {'n_low': 0, 'n_high': 0, 'width': 0, 'isolation': 0,
               'convex': 0, 'spread': 0, 'intensity': 0}
        if N < 2:
            self.last_reject = rej
            return []

        dphi = self._angle_increment(angles_vehicle)
        idx = np.nonzero(valid_mask)[0]
        if idx.size == 0:
            self.last_reject = rej
            return []

        origin = np.asarray(sensor_origin, dtype=float).reshape(2)
        segments = self._segment(idx, ranges, xy_base, dphi)

        out = []
        for seg in segments:
            det = self._evaluate(seg, ranges, angles_vehicle, xy_base, dphi, N,
                                 intensities, origin, rej)
            if det is not None:
                out.append(det)
        self.last_reject = rej
        return out

    # ---------------- 각분해능 ----------------
    @staticmethod
    def _angle_increment(angles):
        if len(angles) < 2:
            return math.radians(0.1125)
        d = float(abs(angles[1] - angles[0]))
        if not math.isfinite(d) or d <= 1e-9:
            return math.radians(0.1125)
        return d

    # ---------------- 적응형 분할 ----------------
    def _segment(self, idx, ranges, xy_base, dphi):
        p = self.p
        if idx.size == 1:
            return [idx]

        pts = xy_base[idx]
        r_prev = ranges[idx][:-1]
        step = np.diff(pts, axis=0)
        dist = np.hypot(step[:, 0], step[:, 1])

        lam = math.radians(max(p.cluster_lambda_deg, dphi * 180.0 / math.pi + 1.0))
        denom = math.sin(lam - dphi)
        if abs(denom) < 1e-9:
            denom = 1e-9
        d_thr = r_prev * (math.sin(dphi) / denom) + 3.0 * p.range_sigma
        d_thr = np.clip(d_thr, p.cluster_gap_min, p.cluster_gap_max)

        # 빔 인덱스가 많이 건너뛰면(중간에 무효빔 다수) 무조건 분할
        beam_gap = np.diff(idx)
        brk = (dist > d_thr) | (beam_gap > p.max_index_gap)

        cut = np.nonzero(brk)[0] + 1
        return [s for s in np.split(idx, cut) if s.size > 0]

    # ---------------- 클러스터 판정 ----------------
    def _evaluate(self, seg, ranges, angles, xy_base, dphi, N, intensities,
                  origin, rej):
        p = self.p
        n = int(seg.size)
        if n > p.max_points_abs:
            rej['n_high'] += 1
            return None
        if n < p.min_points_abs:
            rej['n_low'] += 1
            return None

        pts = xy_base[seg]
        rs = ranges[seg]
        center = pts.mean(axis=0)
        r_mean = float(np.mean(rs))
        radial_spread = float(np.max(rs) - np.min(rs))

        # --- intensity ---
        if p.use_intensity and intensities is not None:
            try:
                if float(np.mean(intensities[seg])) < p.min_intensity:
                    rej['intensity'] += 1
                    return None
            except Exception:
                pass

        # --- 물리 기대치 ---
        w_exp = p.expected_chord_width()
        n_exp = p.expected_hits(r_mean, dphi)
        quant = r_mean * dphi  # 각분해능에 의한 폭 양자화 오차

        # --- 점수 게이트 ---
        n_min = max(p.min_points_abs, int(math.floor(p.n_exp_min_ratio * n_exp)))
        n_max = max(p.min_points_abs + 1,
                    int(math.ceil(p.n_exp_max_ratio * n_exp + 2.0)))
        if n < n_min:
            rej['n_low'] += 1
            return None
        if n > n_max:
            rej['n_high'] += 1
            return None

        # --- 폭 게이트 ---
        chord = float(np.linalg.norm(pts[-1] - pts[0])) if n >= 2 else 0.0
        w_lo = max(0.0, p.width_min_ratio * w_exp - quant)
        w_hi = min(p.width_abs_max, p.width_max_ratio * w_exp + quant)
        if chord > w_hi:
            rej['width'] += 1
            return None
        if n >= 3 and chord < w_lo:
            rej['width'] += 1
            return None

        # --- 반경 두께 ---
        if n >= 3:
            spread_max = p.radial_spread_max_ratio * max(0.5 * w_exp, 0.02) + quant
            if radial_spread > spread_max:
                rej['spread'] += 1
                return None

        # --- 볼록성 ---
        sag = 0.0
        if p.convexity_enable and n >= p.convexity_min_points and chord > 1e-6:
            sag = self._sagitta_toward_sensor(pts)
            sag_max = p.sagitta_max_ratio * (0.5 * max(chord, w_exp)) + quant
            if sag < p.sagitta_min or sag > sag_max:
                rej['convex'] += 1
                return None

        # --- 고립성 ---
        if p.isolation_enable:
            left_free = self._isolated_side(ranges, int(seg[0]), -1, float(rs[0]), N)
            right_free = self._isolated_side(ranges, int(seg[-1]), +1, float(rs[-1]), N)
            ok = (left_free and right_free) if p.isolation_require_both \
                else (left_free or right_free)
            if not ok:
                rej['isolation'] += 1
                return None

        # --- 콘 중심 보정 ---
        # 스캔은 센서를 향한 표면만 본다. 중심은 반경 rho = w_exp/2 만큼 더 멀다.
        # 주의 1: 방향은 반드시 '센서 원점' 기준이어야 한다. 라이다가 앞범퍼
        #         (x=1.175)에 있으므로 base_link 원점을 쓰면 방향이 크게 틀린다.
        # 주의 2: 점들의 산술평균은 원호 안쪽으로 치우친다. 최근접 거리에 rho 를
        #         더하는 편이 편향이 없다. (먼 콘에서는 둘이 같아진다)
        rho = 0.5 * w_exp
        if rho > 1e-6:
            th_c = float(np.mean(angles[seg]))
            r_c = float(np.min(rs)) + rho
            center = origin + r_c * np.array([math.cos(th_c), math.sin(th_c)],
                                             dtype=float)

        # --- 점수 (n_exp 부합도 + 폭 부합도) ---
        n_fit = 1.0 - min(1.0, abs(n - n_exp) / max(n_exp, 1.0))
        w_fit = 1.0 - min(1.0, abs(chord - w_exp) / max(w_exp, 1e-3))
        score = 2.0 * n_fit + 1.5 * w_fit

        return ConeDetection(center=center, width=chord, n=n, range_mean=r_mean,
                             sagitta=sag, radial_spread=radial_spread,
                             i0=int(seg[0]), i1=int(seg[-1]),
                             n_exp=n_exp, score=score)

    # ---------------- 보조 ----------------
    @staticmethod
    def _sagitta_toward_sensor(pts):
        """현 대비 최대 부풀음. 센서(원점)쪽으로 볼록하면 양수."""
        a, b = pts[0], pts[-1]
        v = b - a
        L = float(np.linalg.norm(v))
        if L < 1e-9:
            return 0.0
        nrm = np.array([-v[1], v[0]], dtype=float) / L
        mid = 0.5 * (a + b)
        # 원점을 향하는 부호로 법선 정렬
        if float(np.dot(nrm, -mid)) < 0.0:
            nrm = -nrm
        d = (pts - a) @ nrm
        return float(np.max(d))

    def _isolated_side(self, ranges, i_edge, direction, r_edge, N):
        """i_edge 바깥쪽 이웃 빔이 비어 있거나 충분히 멀면 True."""
        p = self.p
        for k in range(1, p.isolation_search + 1):
            j = i_edge + direction * k
            if j < 0 or j >= N:
                return True
            rj = ranges[j]
            if not math.isfinite(rj) or rj < p.min_range or rj > p.max_range:
                continue  # 무효빔 -> 계속 바깥 확인
            return bool((rj - r_edge) >= p.isolation_jump)
        return True


class ConeTracker(object):
    """
    odom 프레임 콘 트랙.

    ROS1 판은 base_link 에서 추적했기 때문에 차가 전진하면 트랙 좌표가
    실제로 이동해 hits 가 쌓이기 전에 track_merge_dist 를 벗어났다.
    odom 에 저장하면 정지한 콘의 좌표는 상수이므로 이동 중에도 안정 누적된다.
    """

    def __init__(self, params):
        self.p = params
        self.tracks = []

    def reset(self):
        self.tracks = []

    def update(self, detections_odom, now_sec, detection_ranges=None):
        p = self.p
        if detection_ranges is None or len(detection_ranges) != len(detections_odom):
            detection_ranges = [0.0] * len(detections_odom)
        # --- 매칭 (가까운 쌍 우선) ---
        pairs = []
        for di, d in enumerate(detections_odom):
            for ti, tr in enumerate(self.tracks):
                dist = math.hypot(d[0] - tr['x'], d[1] - tr['y'])
                if dist <= p.track_merge_dist:
                    pairs.append((dist, di, ti))
        pairs.sort(key=lambda it: it[0])

        used_d, used_t = set(), set()
        for _, di, ti in pairs:
            if di in used_d or ti in used_t:
                continue
            d = detections_odom[di]
            tr = self.tracks[ti]
            a = p.track_alpha
            tr['x'] = a * float(d[0]) + (1.0 - a) * tr['x']
            tr['y'] = a * float(d[1]) + (1.0 - a) * tr['y']
            rr = float(detection_ranges[di])
            if math.isfinite(rr) and rr >= 0.0:
                tr['range'] = rr
            tr['hits'] += 1
            tr['last'] = now_sec
            used_d.add(di)
            used_t.add(ti)

        for di, d in enumerate(detections_odom):
            if di in used_d:
                continue
            if len(self.tracks) >= p.max_tracks:
                break
            rr = float(detection_ranges[di])
            self.tracks.append({
                'x': float(d[0]), 'y': float(d[1]),
                'range': rr if math.isfinite(rr) and rr >= 0.0 else 0.0,
                'hits': 1, 'last': now_sec})

        # --- 오래 안 보인 트랙 제거 ---
        # 확정된 트랙도 영구 보관하면 차량이 지나간 콘과 순간 오검출이 계속
        # 슬롯 검출에 누적된다. 마지막 관측 후 max_track_miss_sec가 지나면
        # hits 수와 관계없이 제거한다.
        keep = []
        for tr in self.tracks:
            far = tr.get('range', 0.0) >= p.far_track_range
            timeout = p.far_track_miss_sec if far else p.max_track_miss_sec
            aged = (now_sec - tr['last']) > timeout
            if aged:
                continue
            keep.append(tr)
        self.tracks = keep
        return self.confirmed()

    def confirmed(self):
        out = []
        for tr in self.tracks:
            far = tr.get('range', 0.0) >= self.p.far_track_range
            need = self.p.far_min_track_hits if far else self.p.min_track_hits
            if tr['hits'] >= need:
                out.append(np.array([tr['x'], tr['y']], dtype=float))
        return out

    def all_points(self):
        return [np.array([tr['x'], tr['y']], dtype=float) for tr in self.tracks]

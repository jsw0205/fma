#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
offline_check.py  -  ROS 없이 콘검출 -> 슬롯검출 -> 2원호계획 을 검증한다.

실차에 올리기 전에 이걸 먼저 돌려서
  (1) 뒤집힌 라이다 각도 규약이 맞는지
  (2) 콘이 몇 m 까지 검출 가능한지 (라이다를 높이 달아 생기는 물리 한계)
  (3) 슬롯이 잡히는지
  (4) 2원호 계획이 실현가능한지 (z_arc / z_straight 분배)
를 숫자로 확인한다.

    python3 tools/offline_check.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from t_parking.cone_detector import ConeDetector, ConeParams, ConeTracker
from t_parking.slot_detector import SlotDetector, SlotParams
from t_parking.two_arc_planner import TwoArcPlanner

# ---------------- 실차 제원 ----------------
LASER_X = 1.175
LASER_Y = 0.0
LASER_YAW = math.pi
ANGLE_SIGN = -1.0
DPHI = math.radians(0.25)          # A3M1 대략치
VEH = dict(vehicle_width=0.800, vehicle_length=1.410, wheel_base=0.735,
           base_to_front_bumper=1.175, base_to_rear_bumper=0.230,
           side_margin=0.15, back_margin=0.20, collision_margin=0.10)


def cone_scan(cones_xy, cone_radius, n_beams=1440, max_r=8.0):
    """
    콘들을 원으로 두고 뒤집힌 라이다 raw 스캔을 만든다.
    유효각 theta_v = LASER_YAW + ANGLE_SIGN * raw  이므로
    raw = ANGLE_SIGN * (theta_v - LASER_YAW) (ANGLE_SIGN=-1 이면 대칭)
    """
    raw = -math.pi + np.arange(n_beams) * (2.0 * math.pi / n_beams)
    theta_v = LASER_YAW + ANGLE_SIGN * raw
    d = np.stack([np.cos(theta_v), np.sin(theta_v)], axis=1)
    origin = np.array([LASER_X, LASER_Y])
    r = np.full(n_beams, np.inf)
    for c in cones_xy:
        f = np.asarray(c, dtype=float) - origin
        # 광선-원 교점
        b = d @ f
        c2 = float(f @ f) - cone_radius * cone_radius
        disc = b * b - c2
        ok = (disc >= 0.0) & (b > 0.0)
        t = np.where(ok, b - np.sqrt(np.maximum(disc, 0.0)), np.inf)
        r = np.minimum(r, np.where(t > 0.0, t, np.inf))
    r[r > max_r] = np.inf
    return raw, theta_v, r


def main():
    # ---------------- 씬 구성 ----------------
    # 좌측(y>0)에 콘 열. 입구폭 1.30, 슬롯깊이 2.00, 뒷벽 콘 3개.
    W, D = 1.30, 2.00
    lane_y = 1.00                       # 차량 좌측 1.0 m 에 콘 열
    cones = []
    for u in (-1.6, -0.3):              # 입구 앞쪽 차선 콘
        cones.append((u, lane_y))
    cones.append((0.0, lane_y))         # 입구 콘 A
    cones.append((W, lane_y))           # 입구 콘 B
    for u in (W + 1.3, W + 2.6):        # 입구 뒤쪽 차선 콘
        cones.append((u, lane_y))
    for q in (0.15, 0.5 * W, W - 0.15):  # 뒷벽 콘
        cones.append((q, lane_y + D))

    cp = ConeParams(cone_height=0.450, cone_base_diameter=0.280,
                    scan_height=0.160, min_range=0.25, max_range=8.0,
                    min_track_hits=1, sagitta_max_ratio=1.25)
    w_exp = cp.expected_chord_width()
    cone_radius = 0.5 * w_exp

    print('=== 콘 물리 모델 ===')
    print(' 스캔면 단면폭 w_exp        = %.4f m  (밑동 %.3f, 높이 %.3f, 스캔 %.3f)'
          % (w_exp, cp.cone_base_diameter, cp.cone_height, cp.scan_height))
    for mh in (1.0, 2.0, 3.0):
        print(' %d점 이상 맞는 최대거리     = %.2f m' % (mh, cp.max_detect_range(DPHI, mh)))
    print(' -> 이 거리 밖의 콘은 파라미터를 어떻게 만져도 물리적으로 안 잡힌다.')

    raw, theta_v, r = cone_scan(cones, cone_radius)
    xb = LASER_X + r * np.cos(theta_v)
    yb = LASER_Y + r * np.sin(theta_v)
    xy = np.stack([xb, yb], axis=1)

    valid = np.isfinite(r) & (r >= cp.min_range) & (r <= cp.max_range)
    valid &= (yb > 0.20) & (yb < 4.50) & (xb > -0.60) & (xb < 8.00)
    half = 0.5 * VEH['vehicle_width'] + 0.08
    self_hit = ((xb >= -VEH['base_to_rear_bumper'] - 0.08) &
                (xb <= VEH['base_to_front_bumper'] + 0.08) &
                (np.abs(yb) <= half))
    valid &= ~self_hit

    det = ConeDetector(cp)
    dets = det.detect(r, theta_v, xy, valid, None, (LASER_X, LASER_Y))
    print('\n=== 콘 검출 ===')
    print(' 유효빔 %d,  검출 클러스터 %d / 실제 콘 %d,  reject=%s'
          % (int(valid.sum()), len(dets), len(cones), det.last_reject))
    for d in sorted(dets, key=lambda z: z.center[0]):
        print('  center=(%+.3f, %+.3f) n=%2d n_exp=%4.1f w=%.3f r=%.2f'
              % (d.center[0], d.center[1], d.n, d.n_exp, d.width, d.range_mean))

    trk = ConeTracker(cp)
    confirmed = trk.update([d.center for d in dets], 0.0)

    # ---------------- 슬롯 검출 ----------------
    sp = SlotParams(entry_min_width=1.05, entry_max_width=2.20,
                    entry_expected_width=1.30, slot_min_depth=1.50,
                    slot_max_depth=3.50, slot_expected_depth=2.00,
                    min_cones_for_plan=3)
    sd = SlotDetector(sp, VEH)
    cand = sd.detect(confirmed, np.array([1.0, 0.0]), 'left', xy[valid])
    print('\n=== 슬롯 검출 ===')
    print(' debug=%s' % sd.debug)
    if cand is None:
        print(' 슬롯 검출 실패')
        return 1
    print(' 입구중점 M=(%.3f, %.3f)  폭=%.3f  깊이=%.3f  z_goal=%.3f  '
          'lane각오차=%.2fdeg'
          % (cand['M'][0], cand['M'][1], cand['entry_width'], cand['depth'],
             cand['z_goal'], math.degrees(cand['heading_err'])))
    print(' 정답과 비교: 폭 %.3f(오차 %+.3f), 깊이 %.3f(오차 %+.3f)'
          % (W, cand['entry_width'] - W, D, cand['depth'] - D))

    # ---------------- 2원호 계획 ----------------
    planner = TwoArcPlanner(VEH, dict(arc_depth_margin=0.10, min_z_arc=-1.20,
                                      max_straight_back=3.00, sim_ds=0.03,
                                      validate_plan=True))
    r_rear = VEH['wheel_base'] / math.tan(math.radians(30.0))
    y_a = float(np.dot(cand['M'], cand['d']))
    plan = planner.plan(y_a, cand['z_goal'], r_rear, r_rear, +1.0, -30, +30)

    cones_sf = []
    for c in cand['cones']:
        rel = np.asarray(c) - cand['M']
        cones_sf.append([float(rel @ cand['lane_dir']), float(rel @ cand['d'])])
    planner.simulate(plan, np.asarray(cones_sf))

    print('\n=== 2원호 계획 (최대조향 30deg -> 뒤차축 R=%.3f m) ===' % r_rear)
    print(' y_a=%.3f  z_goal=%.3f' % (plan['y_a'], plan['z_goal']))
    print(' z_arc=%.3f (원호 담당)   z_straight=%.3f (직진후진 담당)'
          % (plan['z_arc'], plan['z_straight']))
    print(' theta1=%.2fdeg  dtheta2=%.2fdeg  (합 %.2f)'
          % (math.degrees(plan['theta1']), math.degrees(plan['dtheta2']),
             math.degrees(plan['theta1'] + plan['dtheta2'])))
    print(' Sx=%.3f  실현가능=%s %s'
          % (plan['Sx'], plan['feasible'], plan['reason']))
    print(' 필요 통로폭=%.2f m   스윕 최소여유=%s  충돌=%s'
          % (plan['required_aisle_width'], plan['sim_min_clearance'],
             plan['sim_collision']))
    print(' 시뮬 종료자세: u=%+.3f v=%+.3f |phi|=%.2fdeg  (목표 v=%.3f, |phi|=90)'
          % (plan['sim_end_u'], plan['sim_end_v'],
             abs(plan['sim_end_phi_deg']), plan['z_goal']))

    # 단발 2원호(원본 공식)로는 되는지 대조
    print('\n=== 대조: 원본 단발 2원호 (분해 없이) ===')
    need = y_a + cand['z_goal']
    print(' 조건 y_a + z_goal <= R_rev : %.3f <= %.3f -> %s'
          % (need, r_rear, '가능' if need <= r_rear else '불가능(그래서 분해가 필요)'))

    # ---------------- 시나리오 2 : 슬롯에 차가 서 있는 경우 ----------------
    # 미니카 판에는 없던 검사다. 슬롯 내부에 점군이 있으면 후보에서 빼야 한다.
    print('\n=== 시나리오 2 : 슬롯 점유 (다른 차가 주차됨) ===')
    occ = list(cones)
    wall = [(0.15 + 0.1 * k, lane_y + 0.9) for k in range(11)]  # 슬롯 안쪽 차체면
    raw2, th2, r2 = cone_scan(occ + wall, cone_radius)
    x2 = LASER_X + r2 * np.cos(th2)
    y2 = LASER_Y + r2 * np.sin(th2)
    xy2 = np.stack([x2, y2], axis=1)
    v2 = (np.isfinite(r2) & (r2 >= cp.min_range) & (r2 <= cp.max_range)
          & (y2 > 0.20) & (y2 < 4.50) & (x2 > -0.60) & (x2 < 8.00))
    v2 &= ~((x2 >= -VEH['base_to_rear_bumper'] - 0.08)
            & (x2 <= VEH['base_to_front_bumper'] + 0.08)
            & (np.abs(y2) <= half))
    d2 = det.detect(r2, th2, xy2, v2, None, (LASER_X, LASER_Y))
    trk2 = ConeTracker(cp)
    conf2 = trk2.update([d.center for d in d2], 0.0)
    sd2 = SlotDetector(sp, VEH)
    cand2 = sd2.detect(conf2, np.array([1.0, 0.0]), 'left', xy2[v2])
    print(' 결과: %s  (rej_interior_raw=%d, rej_depth=%d)'
          % ('거부됨(정상)' if cand2 is None else
             '통과됨(문제!) M=%s' % cand2['M'],
             sd2.debug.get('rej_interior_raw', 0),
             sd2.debug.get('rej_depth', 0)))
    return 0 if cand2 is None else 2


if __name__ == '__main__':
    sys.exit(main())

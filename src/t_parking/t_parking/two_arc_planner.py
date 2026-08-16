#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
two_arc_planner.py  -  2원호 닫힌 공식 + 실차 대응 확장.

[원본 공식 (미니카에서 성공한 그대로)]
  불변량 : theta1 + dtheta2 = 90deg
  공식   : cos(theta1) = (y_a + z_arc + R_setup) / (R_rev + R_setup)
           dtheta2     = 90deg - theta1
           Sx          = R_rev - (R_rev + R_setup) * sin(theta1)
  유도 (v = 슬롯 안쪽, u = 도로 진행, 회전은 모두 같은 방향으로 90deg):
      전진 원호 : v1 = -y_a - R_setup + R_setup*cos(theta1)
      후진 원호 : v2 = v1 + R_rev*cos(theta1)
      v2 = z_arc  ->  위 공식.  u 성분에서 Sx 가 나온다.

[실차에서 반드시 필요한 확장]
공식의 실현가능 조건은
      y_a + z_arc <= R_rev
이다. 미니카는 R_rev=0.83, z_goal~0.2 로 여유가 있었다. 그런데 새 차량은

      wheel_base = 0.735  ->  R_rev(30deg 조향) = 0.735/tan(30) = 1.273 m
      슬롯깊이 2.0 m      ->  z_goal = 2.0 - 0.230 - 0.20 = 1.570 m

이미 z_goal 하나로 R_rev 를 넘는다. y_a(접근 시 횡거리, 보통 0.8~1.0 m)를
더하면 필요 R_rev 가 2.4 m 를 넘어 2원호 단발 해가 수학적으로 존재하지 않는다.

따라서 목표 깊이를 두 개로 분해한다.

      z_arc      = min(z_goal, R_rev - y_a - arc_depth_margin)   <- 원호가 담당
      z_straight = z_goal - z_arc                                <- 직진 후진이 담당

원호 종료 시점에 차량은 이미 슬롯과 평행(90deg 회전 완료)하므로,
남은 깊이는 조향 0 의 직진 후진으로 채우면 된다. 이 구간은 원본에도 있던
REVERSE_STRAIGHT 상태가 그대로 담당한다(중앙선/헤딩 P 보정 포함).
즉 상태 흐름과 판정 방식은 원본과 동일하고, 목표 분배만 바뀐다.

[추가: 계획 궤적 충돌 검증]
차량이 커지면서 원호 스윕이 커져 입구 콘/맞은편 경계를 스치기 쉽다.
lock 직후 계획 전체(전진원호 -> 후진원호 -> 직진후진)를 이산 적분해
차량 풋프린트와 고정 콘맵의 최소 여유를 계산하고, 필요 통로폭도 함께 보고한다.
"""

import math

import numpy as np

from .geometry import clamp, normalize_angle


class TwoArcPlan(dict):
    """dict 상속: 원본 코드의 plan['...'] 접근 방식을 그대로 유지."""
    pass


class TwoArcPlanner(object):
    def __init__(self, vehicle, opts):
        """
        vehicle : dict(wheel_base, vehicle_width, base_to_front_bumper,
                       base_to_rear_bumper, collision_margin)
        opts    : dict(arc_depth_margin, max_straight_back, min_z_arc,
                       sim_ds, validate_plan)
        """
        self.veh = vehicle
        self.o = opts

    # ------------------------------------------------------------------
    def plan(self, y_a, z_goal, r_setup, r_rev, side_sign,
             setup_steer, reverse_steer):
        """
        y_a         : 뒤 차축에서 입구선까지의 횡거리 (슬롯 안쪽 방향 성분)
        z_goal      : 입구선에서 최종 목표까지의 깊이
        r_setup     : 전진 setup 원호 반경 (뒤 차축 기준)
        r_rev       : 후진 진입 원호 반경 (뒤 차축 기준)
        side_sign   : +1 / -1 (회전 방향)
        """
        o = self.o
        arc_margin = float(o.get('arc_depth_margin', 0.10))
        min_z_arc = float(o.get('min_z_arc', -1.20))
        max_straight = float(o.get('max_straight_back', 3.00))

        z_arc_max = r_rev - y_a - arc_margin
        z_arc = min(float(z_goal), float(z_arc_max))
        z_straight = float(z_goal) - z_arc

        denom = r_rev + r_setup
        feasible = True
        reason = ''

        if not math.isfinite(denom) or denom <= 1e-6:
            feasible = False
            reason = 'R_setup/R_rev invalid (steer=0?)'
            arg = 1.0
        else:
            arg = (y_a + z_arc + r_setup) / denom

        if feasible and z_arc < min_z_arc:
            feasible = False
            reason = ('z_arc=%.3f < min_z_arc=%.3f : R_rev(%.3f) 대비 y_a(%.3f)가 '
                      '너무 큼. 접근 차선을 콘 열에 더 붙이거나 조향 한계를 키워야 함.'
                      % (z_arc, min_z_arc, r_rev, y_a))
        if feasible and z_straight > max_straight:
            feasible = False
            reason = ('z_straight=%.3f > max_straight_back=%.3f : 슬롯이 너무 깊음.'
                      % (z_straight, max_straight))
        if feasible and not (-1.0 <= arg <= 1.0):
            feasible = False
            reason = 'arccos arg=%.3f out of range' % arg

        theta1 = math.acos(clamp(arg, -1.0, 1.0))
        dtheta2 = 0.5 * math.pi - theta1
        sx = r_rev - denom * math.sin(theta1) if math.isfinite(denom) else 0.0

        # 필요 통로폭: 후진 원호 중 외측 앞모서리가 그리는 최대 반경
        aisle = self.required_aisle_width(r_rev, y_a)

        return TwoArcPlan({
            'side_sign': float(side_sign),
            'setup_steer': int(setup_steer),
            'reverse_steer': int(reverse_steer),
            'R_setup': float(r_setup), 'R_rev': float(r_rev),
            'y_a': float(y_a),
            'z_goal': float(z_goal),
            'z_arc': float(z_arc),
            'z_straight': float(z_straight),
            'theta1': float(theta1), 'dtheta2': float(dtheta2),
            'Sx': float(sx),
            'feasible': bool(feasible), 'reason': reason,
            'required_aisle_width': float(aisle),
            'sim_min_clearance': None, 'sim_collision': None,
            'sim_collision_phase': None,
            'sim_collision_cone_u': None, 'sim_collision_cone_v': None,
            'sim_collision_pose_u': None, 'sim_collision_pose_v': None,
            'sim_collision_pose_phi_deg': None,
        })

    # ------------------------------------------------------------------
    def recompute_theta1(self, plan, y_a):
        """
        시작점(정지)에서 실제 횡거리로 theta1 을 재계산한다.
        z_arc 도 새 y_a 로 다시 분배해야 하므로 함께 갱신한다.
        """
        o = self.o
        arc_margin = float(o.get('arc_depth_margin', 0.10))
        r_setup, r_rev = plan['R_setup'], plan['R_rev']
        z_goal = plan['z_goal']

        z_arc = min(z_goal, r_rev - y_a - arc_margin)
        denom = r_rev + r_setup
        arg = (y_a + z_arc + r_setup) / denom
        if not (-1.0 <= arg <= 1.0):
            return False, plan['theta1']
        theta1 = math.acos(arg)
        old = plan['theta1']
        plan['theta1'] = float(theta1)
        plan['dtheta2'] = float(0.5 * math.pi - theta1)
        plan['y_a'] = float(y_a)
        plan['z_arc'] = float(z_arc)
        plan['z_straight'] = float(z_goal - z_arc)
        plan['Sx'] = float(r_rev - denom * math.sin(theta1))
        return True, old

    # ------------------------------------------------------------------
    def required_aisle_width(self, r_rev, y_a):
        """
        후진 원호 시 외측(입구 반대쪽) 앞모서리가 회전중심에서 갖는 최대 반경.
        통로에 이만큼의 공간이 필요하다. 사전 경고용.
        """
        veh = self.veh
        half_w = 0.5 * veh['vehicle_width']
        front = veh['base_to_front_bumper']
        r_out = math.hypot(r_rev + half_w, front)
        # 회전중심은 차량 기준 슬롯 반대쪽 r_rev 위치.
        # 필요 통로폭 = r_out - r_rev + (차량이 입구선에서 떨어진 거리)
        return max(0.0, r_out - r_rev) + max(0.0, y_a)

    # ------------------------------------------------------------------
    def simulate(self, plan, cones_slotframe):
        """
        계획 전체를 슬롯 프레임에서 이산 적분하고 콘맵과의 최소 여유를 구한다.

        슬롯 프레임 정의 (플래너 유도와 동일)
            원점 = 입구 중점 M, +v = 슬롯 안쪽, +u = 도로 진행
            시작 자세 = (u = Sx, v = -y_a, phi = 0)   [phi 는 회전방향 기준 부호]
        cones_slotframe : (K,2) 콘들의 (u, v) 좌표
        """
        if not plan['feasible']:
            return None
        o = self.o
        if not bool(o.get('validate_plan', True)):
            return None

        ds = float(o.get('sim_ds', 0.03))
        veh = self.veh
        cones = np.asarray(cones_slotframe, dtype=float).reshape(-1, 2) \
            if cones_slotframe is not None and len(cones_slotframe) else None

        r_setup, r_rev = plan['R_setup'], plan['R_rev']
        theta1, dtheta2 = plan['theta1'], plan['dtheta2']

        u, v, phi = plan['Sx'], -plan['y_a'], 0.0
        poses = []

        # 스텝 수는 반올림하고 실제 스텝길이를 거리/스텝수로 되맞춘다.
        # (버림으로 하면 회전이 덜 되어 검증 결과가 실제보다 나쁘게 나온다)
        def run(phase, dist, radius, reverse):
            nonlocal u, v, phi
            if dist <= 1e-9:
                return
            n = max(1, int(round(dist / ds)))
            step = dist / n
            for _ in range(n):
                dphi = -(step / radius) if (radius and math.isfinite(radius)) else 0.0
                sgn = -1.0 if reverse else 1.0
                u += sgn * step * math.cos(phi + 0.5 * dphi)
                v += sgn * step * math.sin(phi + 0.5 * dphi)
                phi += dphi
                poses.append((phase, u, v, phi))

        run('SETUP_ARC', r_setup * theta1, r_setup, False)
        run('REVERSE_ARC', r_rev * dtheta2, r_rev, True)
        run('REVERSE_STRAIGHT', max(0.0, plan['z_straight']), None, True)

        min_clear = 1e9
        collision = False
        collision_info = None
        if cones is not None and cones.shape[0] > 0:
            half_w = 0.5 * veh['vehicle_width'] + veh['collision_margin']
            front = veh['base_to_front_bumper'] + veh['collision_margin']
            rear = veh['base_to_rear_bumper'] + veh['collision_margin']
            for (phase, pu, pv, pphi) in poses:
                c, s = math.cos(-pphi), math.sin(-pphi)
                dx = cones[:, 0] - pu
                dy = cones[:, 1] - pv
                lx = c * dx - s * dy
                ly = s * dx + c * dy
                inside = (lx >= -rear) & (lx <= front) & (np.abs(ly) <= half_w)
                if bool(np.any(inside)):
                    collision = True
                    # 첫 접촉만 남긴다. 다음 실차 로그 한 번으로 경로 자체의
                    # 문제인지 잘못 남은 콘 트랙인지 구분할 수 있다.
                    hit = int(np.flatnonzero(inside)[0])
                    collision_info = {
                        'phase': phase,
                        'cone_u': float(cones[hit, 0]),
                        'cone_v': float(cones[hit, 1]),
                        'pose_u': float(pu),
                        'pose_v': float(pv),
                        'pose_phi_deg': float(math.degrees(pphi)),
                        'cone_local_x': float(lx[hit]),
                        'cone_local_y': float(ly[hit]),
                    }
                cx = np.clip(lx, -rear, front)
                cy = np.clip(ly, -half_w, half_w)
                d = np.hypot(lx - cx, ly - cy)
                m = float(np.min(d))
                if m < min_clear:
                    min_clear = m
                if collision:
                    break

        end_pose = poses[-1][1:] if poses else (u, v, phi)
        plan['sim_min_clearance'] = None if min_clear > 1e8 else float(min_clear)
        plan['sim_collision'] = bool(collision)
        plan['sim_collision_phase'] = (
            collision_info['phase'] if collision_info is not None else None)
        plan['sim_collision_cone_u'] = (
            collision_info['cone_u'] if collision_info is not None else None)
        plan['sim_collision_cone_v'] = (
            collision_info['cone_v'] if collision_info is not None else None)
        plan['sim_collision_pose_u'] = (
            collision_info['pose_u'] if collision_info is not None else None)
        plan['sim_collision_pose_v'] = (
            collision_info['pose_v'] if collision_info is not None else None)
        plan['sim_collision_pose_phi_deg'] = (
            collision_info['pose_phi_deg'] if collision_info is not None else None)
        plan['sim_collision_cone_local_x'] = (
            collision_info['cone_local_x'] if collision_info is not None else None)
        plan['sim_collision_cone_local_y'] = (
            collision_info['cone_local_y'] if collision_info is not None else None)
        plan['sim_end_u'] = float(end_pose[0])
        plan['sim_end_v'] = float(end_pose[1])
        plan['sim_end_phi_deg'] = float(math.degrees(end_pose[2]))
        return plan

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
geometry.py  -  t_parking 공통 수학 유틸.

ROS1 판에서는 tf.transformations 를 썼지만 ROS2 에서는 tf_transformations 가
기본 설치가 아니므로 쿼터니언 변환을 직접 구현한다(외부 의존성 0).
"""

import math

import numpy as np


# ----------------------------- scalar -----------------------------
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def normalize_angle(a):
    """(-pi, pi] 로 정규화."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def deg(a):
    return math.degrees(a)


def rad(a):
    return math.radians(a)


# ----------------------------- quaternion -----------------------------
def yaw_from_quat(q):
    """geometry_msgs/Quaternion -> yaw (rad)."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quat_from_yaw(yaw):
    """yaw (rad) -> (x, y, z, w)."""
    h = 0.5 * yaw
    return (0.0, 0.0, math.sin(h), math.cos(h))


def set_quat_yaw(msg_quat, yaw):
    x, y, z, w = quat_from_yaw(yaw)
    msg_quat.x, msg_quat.y, msg_quat.z, msg_quat.w = x, y, z, w
    return msg_quat


# ----------------------------- 2D transforms -----------------------------
def rot2(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]], dtype=float)


def transform_to_world(p_local, pose):
    """pose=(x, y, yaw) 로컬점 -> 전역점."""
    x0, y0, yaw0 = pose
    c, s = math.cos(yaw0), math.sin(yaw0)
    px, py = float(p_local[0]), float(p_local[1])
    return np.array([x0 + c * px - s * py, y0 + s * px + c * py], dtype=float)


def transform_to_local(p_world, pose):
    """pose=(x, y, yaw) 전역점 -> 로컬점."""
    x0, y0, yaw0 = pose
    dx, dy = float(p_world[0]) - x0, float(p_world[1]) - y0
    c, s = math.cos(yaw0), math.sin(yaw0)
    return np.array([c * dx + s * dy, -s * dx + c * dy], dtype=float)


def unit(yaw):
    return np.array([math.cos(yaw), math.sin(yaw)], dtype=float)


# ----------------------------- statistics -----------------------------
def robust_percentile(values, pct, default=None):
    vals = np.asarray(values, dtype=float)
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return default
    return float(np.percentile(vals, pct))


def robust_median(values, default=None):
    vals = np.asarray(values, dtype=float)
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return default
    return float(np.median(vals))


# ----------------------------- ackermann -----------------------------
def rear_radius_from_steer_angle(wheel_base, steer_angle_rad):
    """뒤 차축 기준 회전반경. R_rear = L / tan(delta)."""
    t = abs(math.tan(steer_angle_rad))
    if t < 1e-9:
        return float('inf')
    return wheel_base / t


def front_radius_from_rear(wheel_base, r_rear):
    """앞 차축(구동/엔코더 축)이 그리는 반경. R_front = sqrt(R_rear^2 + L^2)."""
    if not math.isfinite(r_rear):
        return float('inf')
    return math.sqrt(r_rear * r_rear + wheel_base * wheel_base)


def steer_angle_from_rear_radius(wheel_base, r_rear):
    if not math.isfinite(r_rear) or r_rear <= 0.0:
        return 0.0
    return math.atan2(wheel_base, r_rear)


def parse_table_param(value, fallback):
    """'10:2.19,20:1.095,30:0.73' 또는 [[10,2.19],...] -> [(cmd, val), ...]."""
    table = []
    try:
        if isinstance(value, str):
            for part in value.split(','):
                part = part.strip()
                if not part:
                    continue
                if ':' in part:
                    a, b = part.split(':', 1)
                elif '=' in part:
                    a, b = part.split('=', 1)
                else:
                    continue
                table.append((abs(int(float(a.strip()))), float(b.strip())))
        elif isinstance(value, (list, tuple)):
            for item in value:
                if isinstance(item, (list, tuple)) and len(item) >= 2:
                    table.append((abs(int(float(item[0]))), float(item[1])))
    except Exception:
        table = []
    if not table:
        table = list(fallback)
    table = [(abs(int(c)), float(r)) for c, r in table
             if abs(int(c)) > 0 and float(r) > 0.0]
    return sorted(set(table), key=lambda x: x[0])


def interp_table(table, cmd_abs):
    """cmd 크기에 대한 표 보간. 표 밖은 1/cmd 비례 외삽 / 마지막 값 고정."""
    if not table:
        return float('inf')
    cmds = [c for c, _ in table]
    vals = [v for _, v in table]
    mag = abs(float(cmd_abs))
    if mag < 1e-6:
        return float('inf')
    if mag <= cmds[0]:
        return vals[0] * (float(cmds[0]) / max(mag, 1e-6))
    if mag >= cmds[-1]:
        return vals[-1]
    return float(np.interp(mag, cmds, vals))


# ----------------------------- line fitting -----------------------------
def fit_line_tls(points):
    """총최소제곱 직선적합. return (centroid(2,), direction(2, unit), rms_residual)."""
    P = np.asarray(points, dtype=float).reshape(-1, 2)
    if P.shape[0] < 2:
        return None
    c = P.mean(axis=0)
    Q = P - c
    # 2x2 공분산의 최대 고유벡터 = 주방향
    cov = Q.T @ Q
    w, v = np.linalg.eigh(cov)
    d = v[:, int(np.argmax(w))]
    d = d / max(float(np.linalg.norm(d)), 1e-12)
    n = np.array([-d[1], d[0]], dtype=float)
    resid = Q @ n
    rms = float(math.sqrt(float(np.mean(resid * resid)))) if resid.size else 0.0
    return c, d, rms


def ransac_line(points, tol, iters=120, seed_dir=None, max_angle=None, rng=None):
    """
    콘 열(row)에 직선을 맞춘다.

    seed_dir/max_angle 이 주어지면 그 방향에서 max_angle 이내인 해만 채택한다.
    (차선 축은 odom 기준 진행방향에서 크게 벗어날 수 없다는 사전지식 주입)

    return (centroid, direction, inlier_mask, rms) 또는 None
    """
    P = np.asarray(points, dtype=float).reshape(-1, 2)
    n = P.shape[0]
    if n < 2:
        return None
    if rng is None:
        rng = np.random.default_rng(12345)

    best = None
    best_cnt = -1
    for _ in range(int(iters)):
        i, j = rng.integers(0, n, size=2)
        if i == j:
            continue
        v = P[j] - P[i]
        nv = float(np.linalg.norm(v))
        if nv < 1e-6:
            continue
        d = v / nv
        if seed_dir is not None and max_angle is not None:
            # 방향은 부호 무관(직선) -> |cos| 로 비교
            cosv = abs(float(np.dot(d, seed_dir)))
            if cosv < math.cos(max_angle):
                continue
        nrm = np.array([-d[1], d[0]], dtype=float)
        dist = np.abs((P - P[i]) @ nrm)
        mask = dist <= tol
        cnt = int(np.count_nonzero(mask))
        if cnt > best_cnt:
            best_cnt = cnt
            best = mask
    if best is None or best_cnt < 2:
        return None
    fit = fit_line_tls(P[best])
    if fit is None:
        return None
    c, d, rms = fit
    if seed_dir is not None:
        if float(np.dot(d, seed_dir)) < 0.0:
            d = -d
    return c, d, best, rms

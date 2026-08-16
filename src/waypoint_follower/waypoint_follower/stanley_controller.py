import math

from waypoint_follower.geo_utils import distance_m


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def lookahead_path_curve(waypoints_x, waypoints_y, idx, lookahead_m):
    """Walks forward from idx+1, accumulating heading change over the next
    lookahead_m of path - however many waypoints that spans. Distance-based
    on purpose: a wide, gentle bend spread over many closely spaced points
    covers little ground per degree turned, so it stays "not sharp" here;
    only a real corner packed into a short distance registers as sharp.
    (A fixed point-count lookahead doesn't have this property - more
    points ahead inflates the angle for gentle bends too, not just real
    corners.)

    Stops accumulating as soon as the turn direction reverses (S-curve) -
    without this, a right-then-left bend would sum both as if they were
    the same direction, wildly overstating how sharp "the corner right in
    front of the vehicle" actually is and picking the wrong full-lock
    direction. Whatever comes after the reversal belongs to the next bend,
    measured on a later call once idx has advanced past this one.

    Returns (total_turn_angle_deg, end_heading_rad, total_dist_m) using
    whatever ground was actually covered (may be less than lookahead_m at
    the end of the path, or if the direction reversed first), or None if
    idx+1 is already the last waypoint."""
    n = len(waypoints_x)
    if idx + 1 >= n - 1:
        return None

    prev_heading = math.atan2(
        waypoints_y[idx + 1] - waypoints_y[idx], waypoints_x[idx + 1] - waypoints_x[idx]
    )
    end_heading = prev_heading

    total_angle = 0.0
    total_dist = 0.0
    turn_sign = None
    i = idx + 1
    while i < n - 1 and total_dist < lookahead_m:
        x0, y0 = waypoints_x[i], waypoints_y[i]
        x1, y1 = waypoints_x[i + 1], waypoints_y[i + 1]
        heading = math.atan2(y1 - y0, x1 - x0)
        step_turn = normalize_angle(heading - prev_heading)

        if turn_sign is None and abs(step_turn) > 1e-6:
            turn_sign = 1.0 if step_turn > 0.0 else -1.0
        elif turn_sign is not None and step_turn * turn_sign < 0.0:
            break

        total_angle += abs(step_turn)
        total_dist += distance_m(x0, y0, x1, y1)
        prev_heading = heading
        end_heading = heading
        i += 1

    return math.degrees(total_angle), end_heading, total_dist


def stanley_control(cx, cy, heading, waypoints_x, waypoints_y, v, k, steer_limit_deg=45.0,
                     turning_radius_m=None, curve_lookahead_m=2.0, curve_lead_margin=1.1,
                     v_min=0.5):
    """Ported from the validated Windows GPS-driving code: picks the
    globally nearest waypoint each cycle (self-corrects if the vehicle
    drifts, rather than only ever advancing an index forward), then
    applies the classic Stanley law against the segment starting there.

    If turning_radius_m is given (vehicle's minimum turning radius at
    steer_limit_deg), this anticipates upcoming corners: the required
    distance to roll out a turn of angle theta at that radius is
    roughly radius*theta, so once the vehicle gets within that distance
    of a sharp bend, the heading-error target is blended towards the
    post-turn direction instead of only reacting once actually at the
    corner (plain Stanley has no preview and turns "too late" at speed).
    curve_lead_margin scales that required distance up (>1.0 = start
    turning earlier than the bare physics minimum) to leave room for
    actuator lag/discretization - without it the vehicle tends to run
    wide on the outside of corners since it starts turning exactly as
    late as physically possible. Cross-track error still uses the true
    current-segment heading, only the heading-error term gets the
    anticipatory blend. curve_lookahead_m is how far ahead (in meters,
    not waypoint count) to measure the upcoming turn - see
    lookahead_path_curve for why distance-based.

    If the upcoming corner's own curvature is tighter than the vehicle can
    physically achieve at turning_radius_m (radius = arc distance covered
    / angle turned), no amount of anticipation closes that gap - the
    vehicle cannot hit the recorded line through that corner regardless of
    when it starts turning. In that case this skips the proportional
    Stanley law entirely and commands full steering lock towards the turn
    as soon as the corner is close enough to matter, which is provably the
    best achievable response and naturally results in cutting to the
    inside of the corner rather than fighting an unreachable target.

    Returns (steer_deg, nearest_idx, cross_track_error_m)."""
    dists = [distance_m(cx, cy, wx, wy) for wx, wy in zip(waypoints_x, waypoints_y)]
    idx = min(range(len(dists)), key=lambda i: dists[i])

    if idx < len(waypoints_x) - 1:
        dx = waypoints_x[idx + 1] - waypoints_x[idx]
        dy = waypoints_y[idx + 1] - waypoints_y[idx]
        path_heading = math.atan2(dy, dx)
    else:
        path_heading = heading

    heading_target = path_heading
    full_lock_sign = None
    if turning_radius_m is not None and turning_radius_m > 0:
        curve = lookahead_path_curve(waypoints_x, waypoints_y, idx, curve_lookahead_m)
        if curve is not None and curve[0]:
            turn_angle_deg, upcoming_heading, arc_dist_m = curve
            required_lead_m = (
                turning_radius_m * math.radians(turn_angle_deg) * curve_lead_margin
            )
            dist_to_corner = distance_m(cx, cy, waypoints_x[idx + 1], waypoints_y[idx + 1])

            turn_angle_rad = math.radians(turn_angle_deg)
            path_radius_m = arc_dist_m / turn_angle_rad if turn_angle_rad > 1e-6 else math.inf
            infeasible = path_radius_m < turning_radius_m

            if infeasible and dist_to_corner <= required_lead_m:
                full_lock_sign = 1.0 if normalize_angle(upcoming_heading - path_heading) >= 0 else -1.0
            else:
                blend = (
                    clamp(1.0 - dist_to_corner / required_lead_m, 0.0, 1.0)
                    if required_lead_m > 0 else 0.0
                )
                if blend > 0.0:
                    heading_target = path_heading + blend * normalize_angle(
                        upcoming_heading - path_heading
                    )

    theta_e = normalize_angle(heading_target - heading)

    dx = cx - waypoints_x[idx]
    dy = cy - waypoints_y[idx]
    cross_track_error = dx * math.sin(path_heading) - dy * math.cos(path_heading)

    if full_lock_sign is not None:
        steer_deg = full_lock_sign * steer_limit_deg
    else:
        # v_min (not just a tiny epsilon) floors the denominator so the
        # correction term stays proportional at low/zero speed instead of
        # saturating to +-90 deg (then clamping to full lock) for *any*
        # nonzero cross_track_error the instant v is near 0 - confirmed
        # 2026-07-29: a stationary vehicle just 1cm off the recorded line
        # would otherwise always compute full steering lock, reproduced
        # live during a stopped mode-switch test. Effectively treats the
        # correction as if crawling at v_min, capping how aggressively a
        # small error gets amplified while nearly stationary.
        delta = theta_e + math.atan2(k * cross_track_error, max(v, v_min))
        steer_deg = clamp(math.degrees(delta), -steer_limit_deg, steer_limit_deg)

    return steer_deg, idx, cross_track_error

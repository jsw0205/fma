import csv
import datetime
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, ColorRGBA, Float32, Int32, String
from visualization_msgs.msg import Marker, MarkerArray

from waypoint_follower import can_driver
from waypoint_follower.geo_utils import LocalFrame, distance_m
from waypoint_follower.stanley_controller import (
    clamp,
    lookahead_path_curve,
    normalize_angle,
    stanley_control,
)


def load_waypoints_csv(path):
    """Loads a Lat,Lon-header CSV. Minimum-spacing decluttering happens
    later once these are projected to ENU meters (see on_fix)."""
    latlon = []
    with open(path, "r") as f:
        for row in csv.DictReader(f):
            latlon.append((float(row["Lat"]), float(row["Lon"])))
    return latlon


def yaw_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__("waypoint_follower_node")

        self.declare_parameter("gps_topic", "/fix")
        self.declare_parameter("use_imu", True)
        self.declare_parameter("imu_topic", "/handsfree/imu")
        self.declare_parameter("use_magnetometer", True)
        self.declare_parameter("mag_topic", "/handsfree/mag")
        # South Korea is roughly -8 deg (magnetic north sits west of true
        # north) - verify for the exact test location/date (e.g. NOAA's
        # declination calculator) rather than trusting this default.
        self.declare_parameter("magnetic_declination_deg", -8.0)
        # Hard-iron calibration offsets, and a catch-all for mounting
        # misalignment/axis convention - all need to be determined
        # empirically once the IMU is mounted in its final spot: do a
        # hard-iron calibration (rotate through many orientations in
        # place), then compare the resulting heading against a known
        # bearing and adjust mag_mount_offset_deg until it matches.
        self.declare_parameter("mag_offset_x", 0.0)
        self.declare_parameter("mag_offset_y", 0.0)
        self.declare_parameter("mag_mount_offset_deg", 0.0)
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("stanley_k", 0.3)
        # Temporary stanley_k boost right after obstacle_avoid_node's
        # RETURN state finishes (avoid maneuver complete, back to normal
        # driving) - snaps back onto the recorded line faster instead of
        # the slow crawl-in a low, tuned-for-cruising k would give. Only
        # active for stanley_k_boost_duration_sec after the RETURN->CLEAR
        # transition, then decays back to the normal stanley_k - doesn't
        # affect regular driving at all.
        self.declare_parameter("stanley_k_boost", 1.0)
        self.declare_parameter("stanley_k_boost_duration_sec", 2.0)
        self.declare_parameter("avoid_state_topic", "/avoid/state")
        # Same boost window, second trigger (2026-08-11): right after
        # exiting a parking slot (t_parking/parallel_parking's
        # /parking_exit_done=true), the vehicle is off the recorded line
        # by whatever the maneuver left it at - same "snap back onto the
        # line faster" need as the avoid-RETURN case above, reusing the
        # exact same stanley_k_boost/stanley_k_boost_duration_sec params
        # rather than adding separate ones.
        self.declare_parameter("parking_left_exit_done_topic", "/parking_t/parking_exit_done")
        self.declare_parameter("parking_right_exit_done_topic", "/parking_r/parking_exit_done")
        # Floors the speed used in Stanley's cross-track correction term
        # so it doesn't saturate to full steering lock for any nonzero
        # error while nearly stationary - see stanley_control's v_min.
        self.declare_parameter("stanley_v_min", 0.5)
        self.declare_parameter("cruise_rpm", 140)
        self.declare_parameter("min_curve_rpm", 50)
        # Below this lookahead-turn-angle, hold cruise_rpm (max speed) - no
        # point easing off for a curve too gentle to actually need it.
        # Above curve_angle_for_min_rpm_deg, hold min_curve_rpm (already
        # true via _curvature_scaled_rpm's frac clamp, unaffected by this
        # param). Between the two: linear ramp. 5.0 default per the
        # professor's guidance (2026-08-17) - previously 0.0 (ramp started
        # immediately off any nonzero angle, no flat "max speed" region at
        # all).
        self.declare_parameter("curve_deadzone_angle_deg", 5.0)
        self.declare_parameter("curve_angle_for_min_rpm_deg", 40.0)
        self.declare_parameter("curve_lookahead_m", 6.0)
        # 1.5 (from a 2026-07-30ish tuning session, "reacts too late/runs
        # wide on corners") lowered to 1.2 on 2026-08-18 - live testing
        # today found the opposite symptom, turning noticeably too early.
        # See its use in stanley_control()'s docstring for what this
        # actually scales.
        self.declare_parameter("curve_lead_margin", 1.2)
        self.declare_parameter("wheelbase_m", 0.735)
        self.declare_parameter("drive_log_file", "")
        self.declare_parameter("min_waypoint_distance_m", 0.5)
        self.declare_parameter("waypoint_arrival_radius_m", 0.5)
        # 닫힌 루프 경로(예: halla1_closed.csv)용 - 마지막 웨이포인트 도달 시
        # 정지시키지 않고 계속 주행. stanley_control() 이 매 사이클 전체
        # 웨이포인트 중 최근접점을 다시 찾는 방식이라(고정 인덱스 증가가
        # 아님) 별다른 처리 없이 idx가 자연스럽게 0 근처로 넘어간다 - 여기서
        # 할 일은 도착 시 self.finished=True 를 세우지 않는 것뿐.
        self.declare_parameter("loop_waypoints", False)
        self.declare_parameter("gps_outlier_threshold_m", 3.0)
        self.declare_parameter("gps_outlier_streak_accept", 3)
        self.declare_parameter("lowpass_fc_hz", 2.0)
        self.declare_parameter("lowpass_fs_hz", 10.0)
        self.declare_parameter("heading_lookback_m", 0.15)
        # 2026-08-19: see _smoothed_heading()'s rewrite comment - extends
        # the weighted-buffer collection window to heading_lookback_m *
        # this multiple, instead of stopping right at heading_lookback_m
        # (which, at real driving speed, only collects 2-3 points - barely
        # better than the old single-bearing method). 1.0 = old-rewrite
        # behavior (stop right at lookback_m). Simulated: 5.0 cut heading
        # noise std ~3.5x vs 1.0, at both lookback_m=0.15 and 1.0.
        self.declare_parameter("heading_buffer_window_mult", 5.0)
        # 2026-08-19: on_fix() used to do `self.yaw = heading` - a hard
        # overwrite, every single time a fresh GPS-movement heading became
        # available (which, at heading_lookback_m=0.15m and real driving
        # speeds of ~1-1.6 m/s, is almost every tick). That threw away
        # on_imu()'s gyro-integrated continuity completely and let raw GPS
        # jitter inject directly into self.yaw at full weight - confirmed
        # as the dominant noise source behind the +-14deg steer swings
        # seen in real CAN logs while cross_track_error_m stayed small
        # (0.1-0.4m, nowhere near enough to explain the swing via Stanley's
        # cross-track term alone - see README's 2026-08-19 session notes).
        # heading_correction_alpha blends toward the fresh GPS heading
        # instead of snapping to it: new_yaw = yaw + alpha*angle_diff(GPS
        # heading, yaw), circular-safe via normalize_angle. 1.0 = old
        # behavior (hard snap, unchanged default - opt-in fix, doesn't
        # change anything until tuned down). Smaller = more damped/trusts
        # gyro continuity more, but slower to correct real gyro drift.
        self.declare_parameter("heading_correction_alpha", 1.0)
        self.declare_parameter("steer_limit_deg", can_driver.TRUE_STEER_MAX_ANGLE_DEG)
        self.declare_parameter("steer_sign", -1)
        self.declare_parameter("enable_control", False)
        self.declare_parameter("control_rate_hz", 20.0)
        # When an arbiter node (e.g. camera/GPS priority switch) owns the
        # actual CAN bus, set this false so this node only publishes its
        # computed steer/rpm/target_idx as topics instead of also sending
        # CAN directly - two nodes both writing to the same CAN bus fight
        # each other (same class of problem as the GPS serial port
        # conflicts from earlier this session).
        self.declare_parameter("publish_can_directly", True)

        waypoints_file = self.get_parameter("waypoints_file").value
        if not waypoints_file:
            waypoints_file = (
                get_package_share_directory("waypoint_follower")
                + "/waypoints/sample_waypoints.csv"
            )

        self.waypoints_latlon = load_waypoints_csv(waypoints_file)
        if len(self.waypoints_latlon) < 2:
            raise RuntimeError(
                f"Need at least 2 waypoints in {waypoints_file}, "
                f"got {len(self.waypoints_latlon)}"
            )
        self.get_logger().info(
            f"Loaded {len(self.waypoints_latlon)} waypoints from {waypoints_file}"
        )

        self.frame = None
        self.waypoints_enu = None
        self.target_idx = 0
        self.finished = False

        drive_log_file = self.get_parameter("drive_log_file").value
        if not drive_log_file:
            # 2026-08-05: logs were piling up loose in the home directory
            # (79 files, 31MB) - now under ~/logs/drive/ instead.
            log_dir = os.path.expanduser("~/logs/drive")
            os.makedirs(log_dir, exist_ok=True)
            drive_log_file = os.path.join(
                log_dir, "drive_log_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
            )
        self.drive_log_file = drive_log_file
        self.drive_log_rows = []
        self._drive_log_start = self.get_clock().now()

        # Ported from the Windows GPS-driving code's filter_gps_signal():
        # rejects single-sample outlier jumps, then EMA-smooths what's left.
        self._raw_prev = None
        self._filtered_prev = None
        # How many consecutive samples have been rejected as "outliers".
        # Without this, a single jump bigger than gps_outlier_threshold_m
        # freezes _raw_prev forever - every real position after a genuine
        # jump (mode-switch handoff, GPS catching up after being stale
        # while the camera was driving, etc.) is *also* far from that
        # stale frozen point, so it gets rejected too, in a loop the
        # filter can never escape. Confirmed 2026-07-29: this is exactly
        # what happened on a real drive - the reported position and speed
        # froze while the vehicle kept actually moving, and steering kept
        # re-applying the same stale correction (a hard-left, in that
        # case) indefinitely because on_fix never saw the real, updated
        # position. See waypoint_arrival note in the README for the
        # writeup.
        self._outlier_streak = 0

        # Position history for get_smoothed_heading()-style bearing estimation.
        self.position_history = []

        fc = self.get_parameter("lowpass_fc_hz").value
        fs = self.get_parameter("lowpass_fs_hz").value
        self._lowpass_alpha = (2 * math.pi * fc) / (2 * math.pi * fc + fs)
        self._filtered_steer_deg = 0.0

        self.pos = None
        self.yaw = None
        self.speed = 0.0
        self._last_fix_time = None
        self._last_imu_time = None

        self.drive_status = None
        self.steering_status = None

        self._avoid_state_prev = None
        self._stanley_k_boost_until = None

        try:
            self.bus = can_driver.open_bus(self.get_parameter("can_channel").value)
        except Exception as exc:
            self.get_logger().warn(f"CAN bus not available ({exc}); running visualization-only")
            self.bus = None

        self.create_subscription(
            NavSatFix, self.get_parameter("gps_topic").value, self.on_fix, 10
        )
        if self.get_parameter("use_imu").value:
            self.create_subscription(
                Imu, self.get_parameter("imu_topic").value, self.on_imu, 10
            )
        if self.get_parameter("use_magnetometer").value:
            self.create_subscription(
                MagneticField, self.get_parameter("mag_topic").value, self.on_mag, 10
            )
        self.create_subscription(
            String, self.get_parameter("avoid_state_topic").value, self.on_avoid_state, 10
        )
        self.create_subscription(
            Bool, self.get_parameter("parking_left_exit_done_topic").value,
            self.on_parking_exit_done, 10,
        )
        self.create_subscription(
            Bool, self.get_parameter("parking_right_exit_done_topic").value,
            self.on_parking_exit_done, 10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "waypoint_follower/markers", 10
        )

        # For an arbiter node to consume (see publish_can_directly param).
        # valid=False until a GPS fix + heading are available - mirrors the
        # same self.pos/self.yaw check used to gate control below.
        self.steer_pub = self.create_publisher(Float32, "gps_control/steer_deg", 10)
        self.rpm_pub = self.create_publisher(Float32, "gps_control/rpm", 10)
        self.idx_pub = self.create_publisher(Int32, "gps_control/target_idx", 10)
        self.valid_pub = self.create_publisher(Bool, "gps_control/valid", 10)
        # Published so control_arbiter can sanity-check the camera against
        # the recorded GPS path even while the camera is actively driving
        # (not just used for the drive log CSV like before) - see
        # arbiter_node's camera_max_deviation_m.
        self.cross_track_pub = self.create_publisher(
            Float32, "gps_control/cross_track_error_m", 10
        )

        control_rate = self.get_parameter("control_rate_hz").value
        self.create_timer(1.0 / control_rate, self.on_timer)

    def _filter_gps_signal(self, x, y):
        outlier_threshold = self.get_parameter("gps_outlier_threshold_m").value
        # After this many consecutive rejections, stop treating the jump
        # as noise and accept it - a *single* big jump is more likely a
        # bad sample, but several in a row all pointing the same way is a
        # real, sustained move the filter shouldn't keep fighting forever.
        max_outlier_streak = self.get_parameter("gps_outlier_streak_accept").value

        if self._raw_prev is not None:
            if (
                distance_m(*self._raw_prev, x, y) > outlier_threshold
                and self._outlier_streak < max_outlier_streak
            ):
                self._outlier_streak += 1
                x, y = self._raw_prev
            else:
                self._outlier_streak = 0
                self._raw_prev = (x, y)
        else:
            self._raw_prev = (x, y)

        if self._filtered_prev is None:
            self._filtered_prev = (x, y)

        alpha = self._lowpass_alpha
        fx = (1 - alpha) * self._filtered_prev[0] + alpha * x
        fy = (1 - alpha) * self._filtered_prev[1] + alpha * y
        self._filtered_prev = (fx, fy)
        return fx, fy

    def _smoothed_heading(self):
        # Distance-based, not sample-count-based (same reasoning as
        # curve_lookahead_m): walk back through position_history until at
        # least heading_lookback_m of movement is covered. A fixed sample
        # count needs a minimum speed to ever trigger at all (e.g. old n=4
        # + 1m min needed 5m/s just to produce a heading); a fixed distance
        # instead updates in however long that distance actually takes at
        # the current speed - no minimum speed requirement, and it's still
        # real movement, not GPS noise, as long as the distance is bigger
        # than GPS jitter.
        #
        # 2026-08-19 rewrite: used to be a single two-point bearing (now
        # vs. the one point exactly heading_lookback_m back) - noisy,
        # since 2 GPS fixes a few cm off from their "true" position can
        # swing that single bearing by tens of degrees, especially at real
        # driving speeds where heading_lookback_m=0.15m is covered in just
        # 2-3 ticks. Confirmed as the dominant source of the +-14deg steer
        # swings seen in real CAN logs (cross_track_error stayed small the
        # whole time, nowhere near enough to explain the swing via
        # Stanley's cross-track term alone - see README's 2026-08-19
        # session notes).
        #
        # Now: a distance-weighted buffer - every point passed while
        # walking back gets its own now->point bearing, weighted by that
        # point's own distance from "now" (farther = more weight, since a
        # longer baseline is proportionally less sensitive to a few cm of
        # GPS noise on either end than a short one is). Combined via
        # unit-vector sum (circular-safe - can't just average angles
        # numerically, e.g. 179deg and -179deg should average to 180deg,
        # not ~0deg) into one heading.
        #
        # Still requires covering heading_lookback_m before returning
        # anything (same minimum as before - "is there enough real
        # movement to trust this at all"), BUT keeps collecting further
        # points past that, out to heading_lookback_m *
        # heading_buffer_window_mult, instead of stopping the instant the
        # minimum is satisfied. This turned out to matter far more than
        # the weighting scheme itself: simulated against synthetic 3cm GPS
        # jitter at real driving speed, stopping right at lookback_m only
        # collects ~2-3 points (heading_lookback_m=0.15m takes 2-3 ticks
        # at ~1.3 m/s) - barely more than the old 2-point method, near-zero
        # improvement. Extending the window to 5x pulled the heading
        # estimate's error std down ~3.5x (9.0deg -> 2.6deg at
        # lookback_m=0.15; 1.4deg -> 0.3deg at lookback_m=1.0 - bigger
        # lookback_m helps independently too, on top of this).
        #
        # Purely GPS-distance-based, no IMU/magnetometer needed.
        lookback_m = self.get_parameter("heading_lookback_m").value
        window_mult = self.get_parameter("heading_buffer_window_mult").value
        max_d = lookback_m * max(window_mult, 1.0)
        if len(self.position_history) < 2:
            return None

        x1, y1 = self.position_history[-1]
        sum_x, sum_y, total_weight = 0.0, 0.0, 0.0
        reached_lookback = False
        for i in range(len(self.position_history) - 2, -1, -1):
            x0, y0 = self.position_history[i]
            d = distance_m(x0, y0, x1, y1)
            if d <= 0.0:
                continue
            bearing = math.atan2(y1 - y0, x1 - x0)
            weight = d
            sum_x += weight * math.cos(bearing)
            sum_y += weight * math.sin(bearing)
            total_weight += weight
            if d >= lookback_m:
                reached_lookback = True
            if d >= max_d:
                break

        if not reached_lookback or total_weight <= 0.0:
            return None
        return math.atan2(sum_y, sum_x)

    def _apply_lowpass(self, value):
        alpha = self._lowpass_alpha
        self._filtered_steer_deg = (
            (1 - alpha) * self._filtered_steer_deg + alpha * value
        )
        return self._filtered_steer_deg

    def _curvature_scaled_rpm(self, idx):
        """|angle| <= curve_deadzone_angle_deg -> cruise_rpm (max, flat).
        |angle| >= curve_angle_for_min_rpm_deg -> min_curve_rpm (min, flat,
        via frac's clamp to 1.0). Between the two: linear ramp down. See
        curve_deadzone_angle_deg's declaration comment."""
        cruise_rpm = self.get_parameter("cruise_rpm").value
        min_rpm = self.get_parameter("min_curve_rpm").value
        deadzone_deg = self.get_parameter("curve_deadzone_angle_deg").value
        max_angle = self.get_parameter("curve_angle_for_min_rpm_deg").value
        lookahead_m = self.get_parameter("curve_lookahead_m").value

        wx, wy = zip(*self.waypoints_enu)
        curve = lookahead_path_curve(wx, wy, idx, lookahead_m)
        if curve is None or max_angle <= deadzone_deg:
            return cruise_rpm

        turn_angle, _, _ = curve
        frac = clamp(
            (turn_angle - deadzone_deg) / (max_angle - deadzone_deg), 0.0, 1.0
        )
        return cruise_rpm - frac * (cruise_rpm - min_rpm)

    def _turning_radius_m(self):
        """Minimum turning radius at steer_limit_deg, from the bicycle
        model (wheelbase / tan(max steer)). Used to figure out how far
        before a corner steering needs to start anticipating it - see
        stanley_control's turning_radius_m."""
        steer_limit = self.get_parameter("steer_limit_deg").value
        if steer_limit <= 0:
            return None
        wheelbase = self.get_parameter("wheelbase_m").value
        return wheelbase / math.tan(math.radians(steer_limit))

    def on_imu(self, msg):
        # Gyro-only yaw integration: fills in the fast updates between GPS
        # fixes. Needs self.yaw already seeded once by GPS movement (can't
        # bootstrap an absolute heading from gyro rate alone) - on_fix's
        # GPS-derived heading then keeps correcting this against drift
        # whenever it's available, so this never has to be drift-free on
        # its own (bench-tested at <0.3 deg/5min stationary, but real
        # driving vibration will be worse than that).
        now = self.get_clock().now()
        if self._last_imu_time is not None and self.yaw is not None:
            dt = (now - self._last_imu_time).nanoseconds / 1e9
            if 0.0 < dt < 1.0:
                self.yaw = normalize_angle(self.yaw + msg.angular_velocity.z * dt)
        self._last_imu_time = now

    def on_mag(self, msg):
        # Absolute heading straight from the magnetometer - available
        # immediately (no movement needed, unlike GPS heading), so this
        # bootstraps self.yaw on the very first reading instead of needing
        # the straight-driving fallback. Also keeps correcting on_imu's
        # gyro-integrated yaw against drift continuously, the same way
        # on_fix's GPS heading does, just faster/available at standstill.
        #
        # mag_offset_x/y are hard-iron calibration (subtract this
        # receiver's/frame's fixed magnetic offset - determine via a
        # rotate-through-many-orientations calibration done with the IMU
        # already mounted in its final spot). mag_mount_offset_deg is a
        # catch-all for IMU mounting misalignment/axis convention -
        # tune empirically by facing a known bearing and adjusting this
        # until self.yaw matches it.
        mx = msg.magnetic_field.x - self.get_parameter("mag_offset_x").value
        my = msg.magnetic_field.y - self.get_parameter("mag_offset_y").value
        if mx == 0.0 and my == 0.0:
            return

        compass_bearing = math.atan2(mx, my)  # clockwise from magnetic north
        declination = math.radians(self.get_parameter("magnetic_declination_deg").value)
        mount_offset = math.radians(self.get_parameter("mag_mount_offset_deg").value)
        # convert compass bearing (CW from north) to this codebase's math
        # convention (CCW from east, matching atan2(dy, dx) elsewhere)
        self.yaw = normalize_angle(
            math.pi / 2 - (compass_bearing + declination) + mount_offset
        )

    def on_avoid_state(self, msg):
        # Start the temporary stanley_k boost window the instant
        # obstacle_avoid_node leaves RETURN (avoid maneuver complete) -
        # see stanley_k_boost/stanley_k_boost_duration_sec declaration.
        if self._avoid_state_prev == "RETURN" and msg.data != "RETURN":
            self._start_stanley_k_boost()
        self._avoid_state_prev = msg.data

    def on_parking_exit_done(self, msg):
        # Same boost, second trigger (2026-08-11) - t_parking/
        # parallel_parking's /parking_exit_done=true means the vehicle
        # just drove itself out of a slot and handed control back; it's
        # off the recorded line by whatever the exit maneuver left it at,
        # same "snap back faster" need as leaving avoid-RETURN.
        if bool(msg.data):
            self._start_stanley_k_boost()

    def _start_stanley_k_boost(self):
        duration = self.get_parameter("stanley_k_boost_duration_sec").value
        self._stanley_k_boost_until = self.get_clock().now() + Duration(seconds=duration)

    def _effective_stanley_k(self):
        if self._stanley_k_boost_until is not None:
            if self.get_clock().now() < self._stanley_k_boost_until:
                return self.get_parameter("stanley_k_boost").value
            self._stanley_k_boost_until = None
        return self.get_parameter("stanley_k").value

    def on_fix(self, msg):
        if self.frame is None:
            # Don't anchor the local ENU origin on a message that has no
            # real fix yet (status.status < STATUS_FIX) - NavSatFix
            # messages get published continuously even before the
            # receiver has locked onto any satellites, with
            # lat/lon defaulting to 0.0/0.0 (Null Island). Locking the
            # origin there once made every real waypoint plot ~14,000km
            # away in mpl_viz forever (self.frame is only ever set once).
            if msg.status.status < NavSatStatus.STATUS_FIX:
                return
            self.frame = LocalFrame(msg.latitude, msg.longitude)
            min_spacing = self.get_parameter("min_waypoint_distance_m").value
            enu_all = [self.frame.to_enu(lat, lon) for lat, lon in self.waypoints_latlon]
            self.waypoints_enu = [enu_all[0]]
            for px, py in enu_all[1:]:
                if distance_m(*self.waypoints_enu[-1], px, py) >= min_spacing:
                    self.waypoints_enu.append((px, py))

        x, y = self.frame.to_enu(msg.latitude, msg.longitude)
        fx, fy = self._filter_gps_signal(x, y)

        # Use the GPS message's own timestamp, not wall-clock time the
        # callback happened to run at - ROS callback scheduling jitter
        # (fixes processed back-to-back vs spread out) made dt too small
        # for some samples, inflating speed by up to ~40% (confirmed: 34%
        # of samples in a real run read >8km/h on a vehicle whose real max
        # is ~8km/h).
        now = Time.from_msg(msg.header.stamp)
        if self._last_fix_time is not None:
            dt = (now - self._last_fix_time).nanoseconds / 1e9
            if dt > 1e-3 and self.position_history:
                dist = distance_m(*self.position_history[-1], fx, fy)
                self.speed = dist / dt
        self._last_fix_time = now

        if not self.position_history or distance_m(*self.position_history[-1], fx, fy) >= 0.001:
            self.position_history.append((fx, fy))

        self.pos = (fx, fy)

        # Bootstraps and periodically re-corrects self.yaw from GPS movement.
        # This is the only way to get an initial heading (gyro rate alone
        # can't bootstrap an absolute yaw) and it also keeps correcting
        # on_imu's integrated yaw against drift whenever the vehicle has
        # actually moved enough to measure a bearing.
        heading = self._smoothed_heading()
        if heading is not None:
            if self.yaw is None:
                self.yaw = heading  # bootstrap - no prior estimate to blend with
            else:
                alpha = self.get_parameter("heading_correction_alpha").value
                # Circular-safe EMA toward the fresh GPS heading (not a
                # hard overwrite) - see heading_correction_alpha's
                # declaration comment. normalize_angle() on the diff
                # handles the +-180deg wraparound correctly (e.g. yaw=179deg,
                # heading=-179deg is really only a 2deg turn, not 358deg).
                self.yaw = normalize_angle(
                    self.yaw + alpha * normalize_angle(heading - self.yaw)
                )

    def on_timer(self):
        if self.bus is not None:
            self.drive_status, self.steering_status = can_driver.poll_feedback(
                self.bus, self.drive_status, self.steering_status
            )

        steer_deg = 0.0
        rpm = 0
        enable = 0
        stop_mode = 1
        cross_track_error = None

        if self.pos is not None and self.yaw is not None and not self.finished:
            x, y = self.pos
            wx, wy = zip(*self.waypoints_enu)

            raw_steer_deg, idx, cross_track_error = stanley_control(
                x, y, self.yaw, wx, wy, self.speed,
                self._effective_stanley_k(),
                steer_limit_deg=self.get_parameter("steer_limit_deg").value,
                turning_radius_m=self._turning_radius_m(),
                curve_lookahead_m=self.get_parameter("curve_lookahead_m").value,
                curve_lead_margin=self.get_parameter("curve_lead_margin").value,
                v_min=self.get_parameter("stanley_v_min").value,
            )
            self.target_idx = idx

            last_idx = len(self.waypoints_enu) - 1
            if (
                not self.get_parameter("loop_waypoints").value
                and idx == last_idx
                and distance_m(x, y, *self.waypoints_enu[last_idx]) <= (
                    self.get_parameter("waypoint_arrival_radius_m").value
                )
            ):
                self.finished = True
            else:
                filtered_steer = self._apply_lowpass(raw_steer_deg)
                steer_deg = filtered_steer * self.get_parameter("steer_sign").value
                rpm = self._curvature_scaled_rpm(idx)
                enable = 1
                stop_mode = 0

        elif self.pos is not None and self.yaw is None and not self.finished:
            # No heading yet (needs GPS movement history to compute) - drive
            # straight so the vehicle actually moves and heading can be
            # established, instead of sitting stopped forever waiting for a
            # heading that only comes from movement. Falls through to normal
            # Stanley control above once self.yaw gets set.
            rpm = self.get_parameter("cruise_rpm").value
            enable = 1
            stop_mode = 0

        if (
            self.bus is not None
            and self.get_parameter("enable_control").value
            and self.get_parameter("publish_can_directly").value
        ):
            can_driver.send_control_true_deg(self.bus, rpm, steer_deg, enable, stop_mode)

        self.steer_pub.publish(Float32(data=float(steer_deg)))
        self.rpm_pub.publish(Float32(data=float(rpm)))
        self.idx_pub.publish(Int32(data=int(self.target_idx)))
        self.valid_pub.publish(Bool(data=self.pos is not None and self.yaw is not None))
        if cross_track_error is not None:
            self.cross_track_pub.publish(Float32(data=float(cross_track_error)))

        self.get_logger().info(
            f"target_idx={self.target_idx} steer={steer_deg:.1f}deg speed={self.speed:.2f}m/s"
            f" | CAN: rpm={rpm:.1f} enable={enable} stop_mode={stop_mode}",
            throttle_duration_sec=1.0,
        )

        if self.pos is not None:
            t = (self.get_clock().now() - self._drive_log_start).nanoseconds / 1e9
            ss = self.steering_status
            self.drive_log_rows.append([
                f"{t:.2f}",
                f"{self.pos[0]:.3f}",
                f"{self.pos[1]:.3f}",
                self.target_idx,
                "" if cross_track_error is None else f"{cross_track_error:.3f}",
                f"{steer_deg:.1f}",
                rpm,
                f"{self.speed:.2f}",
                int(self.finished),
                "" if ss is None else ss["current_angle"],
                "" if ss is None else ss["target_angle"],
                "" if ss is None else ss["current_pot"],
                "" if ss is None else ss["target_pot"],
            ])

        self.publish_markers(steer_deg)

    def save_drive_log(self):
        if not self.drive_log_rows:
            return
        with open(self.drive_log_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "t_s", "x_m", "y_m", "target_idx", "cross_track_error_m",
                "steer_deg", "rpm", "speed_mps", "finished",
                "current_angle_deg", "target_angle_deg", "current_pot", "target_pot",
            ])
            writer.writerows(self.drive_log_rows)
        self.get_logger().info(
            f"Saved {len(self.drive_log_rows)} drive log rows to {self.drive_log_file}"
        )

    def publish_markers(self, steer_deg=0.0):
        if self.waypoints_enu is None:
            return

        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = stamp
        path_marker.ns = "waypoint_follower"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale = Vector3(x=0.2, y=0.0, z=0.0)
        path_marker.color = ColorRGBA(r=0.2, g=0.5, b=1.0, a=1.0)
        path_marker.points = [Point(x=px, y=py, z=0.0) for px, py in self.waypoints_enu]
        markers.markers.append(path_marker)

        waypoints_marker = Marker()
        waypoints_marker.header.frame_id = "map"
        waypoints_marker.header.stamp = stamp
        waypoints_marker.ns = "waypoint_follower"
        waypoints_marker.id = 1
        waypoints_marker.type = Marker.SPHERE_LIST
        waypoints_marker.action = Marker.ADD
        waypoints_marker.scale = Vector3(x=0.5, y=0.5, z=0.5)
        waypoints_marker.color = ColorRGBA(r=1.0, g=0.9, b=0.2, a=1.0)
        waypoints_marker.points = [
            Point(x=px, y=py, z=0.0) for px, py in self.waypoints_enu
        ]
        markers.markers.append(waypoints_marker)

        target_marker = Marker()
        target_marker.header.frame_id = "map"
        target_marker.header.stamp = stamp
        target_marker.ns = "waypoint_follower"
        target_marker.id = 2
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.scale = Vector3(x=0.7, y=0.7, z=0.7)
        target_marker.color = ColorRGBA(r=0.2, g=1.0, b=0.3, a=1.0)
        tx, ty = self.waypoints_enu[self.target_idx]
        target_marker.pose = Pose(position=Point(x=tx, y=ty, z=0.0))
        markers.markers.append(target_marker)

        vehicle_marker = Marker()
        vehicle_marker.header.frame_id = "map"
        vehicle_marker.header.stamp = stamp
        vehicle_marker.ns = "waypoint_follower"
        vehicle_marker.id = 3
        vehicle_marker.action = Marker.ADD

        if self.pos is not None:
            vx, vy = self.pos
            if self.yaw is not None:
                vehicle_marker.type = Marker.ARROW
                vehicle_marker.scale = Vector3(x=1.5, y=0.3, z=0.3)
                vehicle_marker.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
                vehicle_marker.pose = Pose(
                    position=Point(x=vx, y=vy, z=0.0),
                    orientation=yaw_to_quaternion(self.yaw),
                )
            else:
                vehicle_marker.type = Marker.SPHERE
                vehicle_marker.scale = Vector3(x=0.6, y=0.6, z=0.6)
                vehicle_marker.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
                vehicle_marker.pose = Pose(position=Point(x=vx, y=vy, z=0.0))
            markers.markers.append(vehicle_marker)

            if self.yaw is not None:
                steer_marker = Marker()
                steer_marker.header.frame_id = "map"
                steer_marker.header.stamp = stamp
                steer_marker.ns = "waypoint_follower"
                steer_marker.id = 4
                steer_marker.type = Marker.ARROW
                steer_marker.action = Marker.ADD
                steer_marker.scale = Vector3(x=1.0, y=0.2, z=0.2)
                steer_marker.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=1.0)
                steer_marker.pose = Pose(
                    position=Point(x=vx, y=vy, z=0.0),
                    orientation=yaw_to_quaternion(self.yaw + math.radians(steer_deg)),
                )
                markers.markers.append(steer_marker)

                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.header.stamp = stamp
                text_marker.ns = "waypoint_follower"
                text_marker.id = 5
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.scale = Vector3(x=0.0, y=0.0, z=0.5)
                text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                text_marker.pose = Pose(position=Point(x=vx, y=vy, z=1.5))
                text_marker.text = f"steer={steer_deg:.1f}°"
                markers.markers.append(text_marker)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save_drive_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

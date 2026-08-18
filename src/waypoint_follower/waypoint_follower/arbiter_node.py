#!/usr/bin/env python3
"""Picks which control source (LiDAR avoidance > GPS idx-triggered event
zones > camera lane-following > GPS waypoint-following fallback) actually
drives the vehicle, and is the *only* node that writes to the CAN bus.

Camera and GPS nodes both just publish their computed steer/rpm as topics
(see waypoint_follower_node's publish_can_directly=False and
yolopv2_zed_node's can_enable=false) instead of talking to CAN directly -
two nodes independently writing control frames to the same CAN bus fight
each other, the same class of problem as the GPS serial-port conflicts
worked through earlier this session.

Priority (highest first):
  1. GPS idx event zones (event_zones param) - intersections, parking, a
     forced stop, LiDAR obstacle avoidance (obstacle_avoid_node), traffic
     lights (traffic_light_node) - all just "do something specific at this
     idx range", so they're one flat priority tier, not ranked against
     each other. Active whenever the GPS node's current target_idx falls
     in a configured [start, end] range, regardless of whether the camera
     is currently driving.
     "avoid" zones arm obstacle_avoid_node (via can_bridge_enable_topic).
     While its state is CLEAR (scanning, nothing found yet), GPS keeps
     driving (curvature-aware speed/steering preserved) - control only
     actually switches to obstacle_avoid_node's own steer/avoid_rpm once
     it enters an active avoidance state (AVOID_LEFT/RIGHT/PASS/RETURN).
     Fails safe (stop) if the node isn't running/publishing or reports
     STOP.
     "traffic_light" zones are different from the others: they don't
     dictate their own steer, they modify whatever camera/GPS would
     already be doing (steering untouched). `end` doubles as the
     stop-line idx - red light + before the stop line -> slow down
     (traffic_light_approach_rpm_scale); red light + at/past the stop
     line -> full stop; green -> normal speed regardless of position.
     Fails safe to RED if traffic_light_node isn't publishing.
     "parking_left" (t_parking)/"parking_right" (parallel_parking) zones
     (2026-08-05): a course can have both, since each side's topics are
     remapped under its own prefix (/parking_t/, /parking_r/) precisely so
     both parking nodes can be running at once without colliding - see
     parking_t_left.launch.py/parking_parallel_right.launch.py. While that
     side's parking node reports mapping=true (still scanning for/locking
     onto a slot), GPS drives straight through the zone same as "avoid"'s
     CLEAR-state GPS driving - not the parking node's own approach
     controller. Once it locks a slot (mapping=false) and is actively
     maneuvering (active=true), control switches to relaying its
     cmd_rpm/cmd_steer/cmd_enable straight to CAN (raw firmware-scale, like
     avoid's steer). done=true hands
     control back regardless of GPS idx (the vehicle leaves the recorded
     line to pull into/out of the slot, so idx isn't a reliable "done"
     signal here). Fails safe (stop) if the parking node isn't publishing.
  2. Camera lane-following, if its steering topic has published a
     non-NaN value within camera_timeout_sec, AND it isn't currently
     vetoed by the GPS cross-track sanity check (see below) - the
     camera's own lane_valid signal only tells you it found something
     that looks like a lane, not that it's the *correct* one, so this is
     an external check the camera can't fake by just being confident.
  3. GPS waypoint-following, if the GPS node reports itself valid
     (self.pos/self.yaw both known).
  4. Safe stop (rpm=0, enable=0, stop_mode=1) if nothing above is valid.

GPS cross-track veto (2026-07-29): even while camera_ok, if the vehicle's
actual position (per GPS, regardless of who's driving) drifts more than
camera_max_deviation_m from the recorded waypoint line, that's treated as
the camera confidently steering somewhere wrong, and control force-swaps
to GPS. Uses separate enter/re-enter thresholds
(camera_max_deviation_m > camera_deviation_reenter_m) plus a re-entry
streak count, the same hysteresis pattern as the lane_valid frame
counter, so it doesn't flap right at one boundary. If the same vehicle
trips this veto camera_deviation_lockout_count times within
camera_deviation_lockout_window_sec, camera is locked out entirely for
camera_deviation_lockout_sec - a camera that keeps drifting out and
snapping back the instant GPS corrects it is treated as unreliable for a
while rather than immediately re-trusted every time it happens to be
momentarily back in bounds.
"""
import csv
import math
import os
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32, Int16, Int32, String

from waypoint_follower import can_driver

# category string (as passed to _note_source/_log_can) -> can_driver's coarse
# CONTROLLER_* enum, for the CONTROL_META CAN frame (0x203). Ordered
# roughly by priority tier (see module docstring) since that's also
# roughly the order these get hit in on_timer. Falls back to
# CONTROLLER_UNKNOWN for anything not listed here (see
# _controller_id_for_category below) rather than raising - a category
# string this doesn't recognize should show up as UNKNOWN on the CAN bus,
# not crash the arbiter.
_CATEGORY_TO_CONTROLLER_ID = {
    "safe_stop": can_driver.CONTROLLER_SAFE_STOP,
    "gps_fallback": can_driver.CONTROLLER_GPS_FALLBACK,
    "camera": can_driver.CONTROLLER_CAMERA,
    "event_zone_stop": can_driver.CONTROLLER_EVENT_STOP,
    "event_zone_gps_priority": can_driver.CONTROLLER_EVENT_GPS_PRIORITY,
    "event_zone_gps_priority_slow": can_driver.CONTROLLER_EVENT_GPS_PRIORITY_SLOW,
    "event_zone_avoid_scanning": can_driver.CONTROLLER_EVENT_AVOID_SCAN,
    "event_zone_avoid_stop": can_driver.CONTROLLER_EVENT_AVOID_FAILSAFE,
}
# avoid_state-suffixed categories (event_zone_avoid_CLEAR/AVOID_LEFT/...)
# and parking-side-suffixed categories (event_zone_parking_left/right...)
# are dynamic strings (see ObstacleAvoid's ACTIVE_STATES / the parking
# relay's f-strings) - matched by prefix in _controller_id_for_category
# instead of listed individually here.
_AVOID_ACTIVE_STATES = ("AVOID_LEFT", "AVOID_RIGHT", "PASS", "RETURN")


def _controller_id_for_category(category):
    if category in _CATEGORY_TO_CONTROLLER_ID:
        return _CATEGORY_TO_CONTROLLER_ID[category]
    if category.startswith("event_zone_avoid_"):
        state = category[len("event_zone_avoid_"):]
        if state in _AVOID_ACTIVE_STATES:
            return can_driver.CONTROLLER_EVENT_AVOID_ACTIVE
        return can_driver.CONTROLLER_EVENT_AVOID_SCAN  # CLEAR or unrecognized
    for side, mapping_id, active_id, wait_id in (
        ("left", can_driver.CONTROLLER_EVENT_PARKING_LEFT_MAPPING,
         can_driver.CONTROLLER_EVENT_PARKING_LEFT_ACTIVE,
         can_driver.CONTROLLER_EVENT_PARKING_LEFT_WAIT),
        ("right", can_driver.CONTROLLER_EVENT_PARKING_RIGHT_MAPPING,
         can_driver.CONTROLLER_EVENT_PARKING_RIGHT_ACTIVE,
         can_driver.CONTROLLER_EVENT_PARKING_RIGHT_WAIT),
    ):
        prefix = f"event_zone_parking_{side}"
        if category == prefix:
            return active_id
        if category.startswith(prefix + "_wait"):
            return wait_id
        if category.startswith(prefix + "_mapping"):
            return mapping_id
    return can_driver.CONTROLLER_UNKNOWN


# /parking_start is edge-triggered - published exactly once, the instant
# gps_idx first enters a parking zone (see on_timer's trigger loop) - not
# repeated. With the default VOLATILE durability, if the parking node's
# subscription hadn't finished DDS discovery/matching yet at that exact
# moment (e.g. its TimerAction startup delay, or - confirmed live
# 2026-08-12 - a transient nearest-waypoint idx touching the zone right at
# launch), that one message is gone forever and the node never learns to
# start, even though the arbiter's own side (engaged latch, cached mapping
# default of True) carries on as if it had. TRANSIENT_LOCAL retains the
# last published value so a late-joining/late-matching subscriber still
# gets it - a plain VOLATILE subscriber (the node's default, unchanged) is
# QoS-compatible with a TRANSIENT_LOCAL publisher.
_LATCHED_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def parse_event_zones(specs):
    """Each entry: 'start:end:type' or 'start:end:type:extra', e.g.
    '40:42:stop', '60:70:gps_priority', '80:85:avoid',
    '90:95:traffic_light', '90:110:traffic_light:95' (stopline=95, zone
    stays live through idx 110 - see below for why these need to differ),
    '100:100:parking_left', '120:120:parking_right'. type is 'stop',
    'gps_priority', 'avoid' (placeholder, see _handle_avoid_zone),
    'traffic_light', 'parking_left' (t_parking), or 'parking_right'
    (parallel_parking) - see _handle_parking_zone.

    For 'traffic_light', `extra` is the stop-line idx (defaults to `end`
    if omitted, matching the old 3-field behavior). IMPORTANT: `end` and
    the stop-line idx are NOT the same thing and conflating them is a real
    bug - if `end` itself were used as the stop-line, then the instant
    gps_idx ticks past it (which can happen before the red-light debounce
    buffer has actually confirmed STOP, since that takes real frames to
    fill), _zone_at() stops matching this zone at all and the stop logic
    never gets evaluated again - the vehicle just sails through on
    whatever camera/GPS would otherwise do. Give `end` real margin past
    the intended stop-line idx (e.g. stopline=95, end=110) so the zone -
    and the stop check - stays active for a while after the vehicle
    reaches the stop-line while still waiting for a red confirmation."""
    zones = []
    for spec in specs:
        parts = spec.split(":")
        if len(parts) not in (3, 4):
            raise ValueError(
                f"bad event_zones entry {spec!r}, expected 'start:end:type' "
                "or 'start:end:type:extra'"
            )
        try:
            start = int(parts[0])
            end = int(parts[1])
            kind = parts[2].strip()
            extra = int(parts[3]) if len(parts) == 4 else None
        except ValueError:
            raise ValueError(
                f"bad event_zones entry {spec!r}, expected 'start:end:type' "
                "or 'start:end:type:extra'"
            )
        zones.append((start, end, kind, extra))
    return zones


class ArbiterNode(Node):
    def __init__(self):
        super().__init__("control_arbiter")

        self.declare_parameter("camera_steer_topic", "/yolopv2_zed_node/steering_deg")
        # Dead-man's switch: if the camera node stops publishing entirely
        # (crashed, never launched) frame counters below never move and
        # would otherwise freeze in whatever state they were last in - this
        # catches that case independent of frame counting.
        self.declare_parameter("camera_timeout_sec", 1.0)
        # Frame-count hysteresis (camera runs 30-70fps, so counting actual
        # frames is more meaningful than a fixed wall-clock timeout): needs
        # this many consecutive no-lane frames to drop out of camera mode,
        # and this many consecutive lane-detected frames to re-enter it.
        # Counted per camera message, not per arbiter control-loop tick.
        # Driven by camera_lane_valid_topic, NOT by NaN-checking the steer
        # value - yolopv2_zed_rpm_node's publish timer replaces NaN with
        # 0.0 before publishing steering_deg (so downstream consumers never
        # see NaN on the wire), so lane_valid is the only reliable signal
        # for "is the camera's steering trustworthy right now" (confirmed
        # 2026-07-29: without this, a real LaneLost condition kept
        # publishing steer=0.0, arbiter's old NaN check never tripped, and
        # it silently kept driving straight on a dead camera).
        self.declare_parameter("camera_lane_valid_topic", "/yolopv2_zed_node/lane_valid")
        self.declare_parameter("camera_bad_frames_to_disable", 10)
        self.declare_parameter("camera_good_frames_to_enable", 3)
        # GPS cross-track veto (see module docstring). Radius, not
        # diameter - "1m off the line" trips it either side.
        self.declare_parameter("cross_track_topic", "gps_control/cross_track_error_m")
        self.declare_parameter("cross_track_timeout_sec", 1.0)
        self.declare_parameter("camera_max_deviation_m", 2.5)
        self.declare_parameter("camera_deviation_reenter_m", 2.5)
        self.declare_parameter("camera_deviation_reenter_streak", 20)
        self.declare_parameter("camera_deviation_lockout_count", 3)
        self.declare_parameter("camera_deviation_lockout_window_sec", 20.0)
        self.declare_parameter("camera_deviation_lockout_sec", 15.0)
        # camera_mode_rpm: fallback/seed value only as of 2026-08-17 - real
        # camera-driving rpm now comes from camera_rpm_topic (the camera
        # node's own curve-scaled _speed_for_steer() output, published on
        # ~/rpm_target regardless of its can_enable setting). Used to seed
        # self.camera_rpm before the first message arrives, so an early
        # camera-driven tick doesn't command 0 rpm just because nothing's
        # been received yet.
        self.declare_parameter("camera_mode_rpm", 100.0)
        self.declare_parameter("camera_rpm_topic", "/yolopv2_zed_node/rpm_target")
        self.declare_parameter("gps_timeout_sec", 1.0)
        # obstacle_avoidance package (obstacle_avoid_node): only armed
        # while the GPS idx is inside an "avoid" event zone, via
        # can_bridge_enable_topic (matches obstacle_avoid_node's own
        # can_enable_topic param, which resets its state machine to CLEAR
        # on disarm). Its steer/rpm are read from the monitoring topics it
        # already publishes - it must be run with write_can_directly=false
        # so it doesn't also write CAN itself. Its steer values are RAW
        # firmware-scale (avoid_steer_left/right params, e.g. -30/30), NOT
        # true degrees - do not run them through send_control_true_deg.
        self.declare_parameter("avoid_state_topic", "/avoid/state")
        self.declare_parameter("avoid_steer_topic", "/avoid/cmd_steer")
        self.declare_parameter("avoid_rpm_topic", "/avoid/cmd_rpm")
        self.declare_parameter("can_bridge_enable_topic", "/can_bridge/enable")
        self.declare_parameter("avoid_timeout_sec", 1.0)
        # rpm used for the WHOLE "avoid" zone, not just the actual dodge
        # maneuver (2026-08-11) - previously the CLEAR sub-state (still
        # scanning, no obstacle found yet) drove at full GPS cruise rpm
        # (~130-140) and only dropped to obstacle_avoidance's own
        # avoid_rpm once an obstacle was actually found - "구간 rpm 자체를
        # 80으로" asks for the slower speed from the moment idx enters the
        # zone, not just during the dodge itself (more reaction margin the
        # whole way through, not just mid-maneuver).
        self.declare_parameter("avoid_zone_scan_rpm", 80.0)
        # "gps_priority_slow" zone type (2026-08-12) - same GPS-only/no-
        # camera behavior as "gps_priority", just capped slower - for
        # transit stretches between two parking zones (e.g. T자 exit ->
        # 평행 entry) where full cruise speed isn't wanted but it's not an
        # "avoid" zone either.
        self.declare_parameter("gps_priority_slow_rpm", 80.0)
        # traffic_light package (traffic_light_node): publishes GO/STOP on
        # this topic from OAK-D + YOLO red-light detection. Only consulted
        # inside "traffic_light" event zones - end idx of the zone is
        # treated as the stop line (see on_timer).
        self.declare_parameter("traffic_light_topic", "/traffic_light")
        self.declare_parameter("traffic_light_timeout_sec", 1.0)
        # Speed multiplier applied while approaching a red light (before
        # the stop line idx is reached) - full stop only happens AT the
        # stop line, not the instant red is seen.
        self.declare_parameter("traffic_light_approach_rpm_scale", 0.5)
        # parallel_parking (right)/t_parking (left) packages (2026-08-05):
        # both hardcode the SAME absolute topic names in source
        # (/parking_start, /parking/cmd_rpm, ...), so running both at once
        # (a course with both a T-zone and a parallel-zone needs both up
        # simultaneously, only one triggered at a time - see
        # parking_t_left.launch.py's docstring) requires remapping each to
        # its own prefix at launch time: parking_t_left.launch.py ->
        # /parking_t/..., parking_parallel_right.launch.py -> /parking_r/...
        # Two independent parameter/state sets below, one per side, so the
        # arbiter can track and relay each independently. direct_cmd_output
        # stays false on both nodes so /parking/cmd_* are relayed here
        # instead of written directly - same publish/relay pattern as
        # camera/GPS/obstacle_avoid. cmd_steer is already RAW firmware-scale
        # (their own max_steer_cmd=30 matches FIRMWARE_STEER_MAX_ANGLE_DEG),
        # so it goes through can_driver.send_control directly, NOT
        # send_control_true_deg.
        for side, prefix, approach_rpm_default in (
            ("left", "parking_t", 30.0), ("right", "parking_r", 30.0)
        ):
            self.declare_parameter(f"parking_{side}_cmd_rpm_topic", f"/{prefix}/cmd_rpm")
            self.declare_parameter(f"parking_{side}_cmd_steer_topic", f"/{prefix}/cmd_steer")
            self.declare_parameter(f"parking_{side}_cmd_enable_topic", f"/{prefix}/cmd_enable")
            self.declare_parameter(f"parking_{side}_active_topic", f"/{prefix}/parking_active")
            # True while the parking node is still scanning for/locking
            # onto a slot (its own APPROACH state) - both packages publish
            # this (latched) around the exact same entry/lock points, see
            # on_timer.
            self.declare_parameter(f"parking_{side}_mapping_topic", f"/{prefix}/parking_mapping")
            self.declare_parameter(f"parking_{side}_done_topic", f"/{prefix}/parking_done")
            self.declare_parameter(f"parking_{side}_start_topic", f"/{prefix}/parking_start")
            # rpm used while GPS drives straight through the "mapping"
            # phase (see on_timer) - matches this side's own
            # pre_straight_rpm (its cone/slot detection was calibrated at
            # this speed), NOT self.gps_rpm's normal curvature-scaled
            # cruise speed. t_parking(left)=30, parallel_parking(right)=20
            # per each package's own config default.
            self.declare_parameter(f"parking_{side}_approach_rpm", approach_rpm_default)
            # Pre-zone smooth deceleration (2026-08-07): without this, base_rpm
            # goes straight from cruise (gps_rpm/camera_mode_rpm, ~130) to
            # approach_rpm (30/20) the instant gps_idx hits the zone's own
            # start idx - a near-emergency-stop feel on the real vehicle
            # ("rpm이 130에서 30으로 바로 확 죽거든"). Ramp window starts this
            # many idx before the zone's start - within it, rpm is a
            # position-based blend toward approach_rpm (see
            # _parking_ramped_rpm), so it gives the right answer even if
            # gps_idx enters mid-window, not just at the window's first idx.
            # Default 7 -> for parking_left's current zone start (idx 58)
            # that's idx 51, i.e. right after the traffic_light zone ends at
            # idx 50 ("신호등 뒤부터 바로 줄이고" - 2026-08-07).
            self.declare_parameter(f"parking_{side}_ramp_idx_margin", 7)
        # 0.5 -> 1.0 (2026-08-11): integrated-launch-only judder while
        # stopped ("멈출 때 덜컹덜컹") - standalone (direct_cmd_output=true,
        # no arbiter relay at all) never had this, only the full stack
        # with camera/traffic_light inference + everything else competing
        # for CPU/GPU. Theory: system load occasionally pushes the parking
        # node's 20Hz cmd_rpm publish past this timeout, so the arbiter's
        # fresh-check flaps between relaying the real command and forcing
        # a fail-safe rpm=0 - looks exactly like intermittent judder.
        # Widening the window first since it's the cheap, safe fix; if it
        # doesn't fully resolve it, the real fix is reducing system load
        # (or the parking node's per-tick processing time) rather than
        # widening this further and further.
        self.declare_parameter("parking_cmd_timeout_sec", 1.0)
        # See on_timer's retrigger-guard comment - how far outside a
        # completed zone's [start, end] gps_idx must go (padded by this
        # many idx) before a fresh entry into that same zone is allowed to
        # fire /parking_start again.
        self.declare_parameter("parking_retrigger_clear_idx_margin", 30)
        # Stuck-engaged escape hatch (2026-08-12) - see the on_timer usage
        # site's comment. Both conditions must hold before force-releasing:
        # idx this far past the side's own zone bounds, sustained for this
        # long.
        self.declare_parameter("parking_engaged_stuck_idx_margin", 30)
        self.declare_parameter("parking_engaged_stuck_timeout_sec", 90.0)
        # Debounce for the "engaged but now in a different zone entirely"
        # release (2026-08-12) - consecutive on_timer ticks (20Hz by
        # default) required before releasing, so a single noisy tick mid-
        # maneuver can't false-trigger it. 10 ticks =~ 0.5s at 20Hz.
        self.declare_parameter("parking_other_zone_confirm_ticks", 10)
        # 'start:end:type' strings, type is "stop", "gps_priority", "avoid",
        # "traffic_light", "parking_left", or "parking_right".
        self.declare_parameter("event_zones", [""])
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("steer_limit_deg", can_driver.TRUE_STEER_MAX_ANGLE_DEG)
        # base_steer EMA lowpass (2026-08-17): smooths camera/gps_fallback's
        # per-tick steer before it reaches CAN, filtered = alpha*current +
        # (1-alpha)*previous. 1.0 = filtering off (bit-identical to pre-this-
        # change behavior) - opt-in via launch config, doesn't change
        # anything until tuned down. Lower alpha = heavier smoothing but more
        # lag; at 20Hz, alpha=0.3 has roughly a ~0.2s time constant. Only
        # applied to base_steer (normal camera/gps_fallback driving, incl.
        # while a traffic_light zone or a "still GPS-driving-through, not
        # yet engaged" parking zone reuses it) - NOT to avoid/parking-engaged/
        # event-stop's own raw steer, which come from dedicated maneuver
        # logic that needs to track its own commanded value exactly.
        self.declare_parameter("base_steer_lowpass_alpha", 1.0)
        self._filtered_base_steer = None

        zone_specs = [s for s in self.get_parameter("event_zones").value if s]
        self.event_zones = parse_event_zones(zone_specs)
        if self.event_zones:
            self.get_logger().info(f"event zones: {self.event_zones}")

        # Timed "stop" zone hold state - see the "stop" branch in on_timer.
        self._stop_hold_key = None
        self._stop_hold_start_time = None
        self._stop_hold_done = False

        self.camera_steer = float("nan")
        # Seeded from camera_mode_rpm (not 0.0/nan) so an early camera-
        # driven tick, before the first ~/rpm_target message arrives,
        # doesn't command a dead stop - see camera_rpm_topic's declaration
        # comment above.
        self.camera_rpm = self.get_parameter("camera_mode_rpm").value
        self.camera_last_time = None
        # Starts inactive - has to earn a good streak before ever being
        # trusted, rather than defaulting to "on" before any frame arrives.
        self.camera_active = False
        self.camera_bad_streak = 0
        self.camera_good_streak = 0
        self.gps_steer = 0.0
        self.gps_rpm = 0.0
        self.gps_idx = 0
        self.gps_valid = False
        self.gps_last_time = None
        self.avoid_state = "CLEAR"
        self.avoid_steer = 0
        self.avoid_rpm = 0
        self.avoid_last_time = None
        self.traffic_light_state = "GO"
        self.traffic_light_last_time = None
        # One state dict per side (left=t_parking, right=parallel_parking) -
        # see the param declaration loop above for why this is duplicated
        # instead of a single shared set.
        self.parking = {
            side: {
                "cmd_rpm": 0, "cmd_steer": 0, "cmd_enable": 0,
                "cmd_last_time": None,
                "active": False,
                # Defaults True (not False) on purpose: right at zone
                # entry, before the parking node has published anything
                # yet, this should read as "still mapping" so the zone
                # immediately drives straight via GPS (see on_timer)
                # instead of a safe-stop wait for the first message.
                "mapping": True,
                "done": False,
                "zone_was_active": False,
                # See on_timer's retrigger-guard comment.
                "completed_pending_clear": False,
                "clear_bounds": None,
                # See on_timer's dispatch-loop comment (2026-08-07 fix) -
                # latches "this side owns control" from trigger until done,
                # independent of whether gps_idx numerically stays inside
                # the zone's [start, end] the whole time.
                "engaged": False,
                # See on_timer's stuck-engaged escape hatch comment (2026-08-12).
                "engaged_stuck_since": None,
                "other_zone_streak": 0,
            }
            for side in ("left", "right")
        }
        self.cross_track_error = float("nan")
        self.cross_track_last_time = None
        self._deviation_overridden = False
        self._deviation_reenter_streak = 0
        self._deviation_trip_times = []
        self._deviation_locked_until = None
        self._last_category = None

        try:
            self.bus = can_driver.open_bus(self.get_parameter("can_channel").value)
        except Exception as exc:
            self.get_logger().warn(f"CAN bus not available ({exc}); arbiter is log-only")
            self.bus = None

        # CSV log of every CAN frame the arbiter actually sends (2026-08-15
        # - until now nothing recorded what the arbiter itself relayed to
        # CAN; drive_log.csv/lane_log.csv only capture what GPS/camera each
        # independently *computed*, not what actually went out once
        # arbiter picked a source and relayed it - see the module
        # docstring's priority list). `category` is the same string
        # `_note_source` already tracks (gps/camera/event_zone_*/safe_stop
        # etc) so this answers "who had control" for every row too. `steer`
        # is in whatever scale that category actually sends in - true
        # physical degrees for the _send_true_deg() path (gps/camera/
        # safe_stop/parking-wait), raw firmware-scale for avoid and the
        # parking active-relay path (see their own send_control comments).
        self._can_meta_seq = 0
        self._can_log_fh = None
        self._can_log_writer = None
        # DIAG_STATUS (0x104) RX - firmware side confirmed implemented
        # 2026-08-16 (Can_Comms_SendDiagStatus). See README_CAN_PROTOCOL.md.
        self.diag_status = None
        self._last_requested_stop_mode = None
        self._diag_log_fh = None
        self._diag_log_writer = None
        # Set unconditionally (not inside the try below) - _poll_diag_status
        # uses this for its own CSV's timestamps too, regardless of whether
        # either CSV file actually managed to open.
        self._can_log_start = self.get_clock().now()
        try:
            log_dir = os.path.expanduser("~/.ros/arbiter_logs")
            os.makedirs(log_dir, exist_ok=True)
            log_path = os.path.join(
                log_dir, "arbiter_can_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
            self._can_log_fh = open(log_path, "w", newline="")
            self._can_log_writer = csv.writer(self._can_log_fh)
            self._can_log_writer.writerow([
                "t_s", "category", "rpm", "steer", "enable", "stop_mode",
            ])
            self.get_logger().info(f"[csv] logging actual CAN sends to {log_path}")
        except OSError as exc:
            self.get_logger().warn(f"arbiter CAN CSV logging disabled ({exc})")
            self._can_log_fh = None
            self._can_log_writer = None

        try:
            diag_log_path = os.path.join(
                log_dir, "arbiter_diag_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
            self._diag_log_fh = open(diag_log_path, "w", newline="")
            self._diag_log_writer = csv.writer(self._diag_log_fh)
            self._diag_log_writer.writerow([
                "t_s", "requested_stop_mode", "applied_stop_mode", "mismatch",
                "fault_flags", "steer_pwm_duty", "supply_voltage_mV", "rx_seq_echo",
            ])
            self.get_logger().info(f"[csv] logging DIAG_STATUS(0x104) to {diag_log_path}")
        except OSError as exc:
            self.get_logger().warn(f"arbiter DIAG CSV logging disabled ({exc})")
            self._diag_log_fh = None
            self._diag_log_writer = None

        self.create_subscription(
            Float32, self.get_parameter("camera_steer_topic").value, self._on_camera_steer, 10
        )
        self.create_subscription(
            Float32, self.get_parameter("camera_rpm_topic").value, self._on_camera_rpm, 10
        )
        self.create_subscription(
            Bool, self.get_parameter("camera_lane_valid_topic").value,
            self._on_camera_lane_valid, 10,
        )
        self.create_subscription(
            Float32, self.get_parameter("cross_track_topic").value,
            self._on_cross_track, 10,
        )
        self.create_subscription(Float32, "gps_control/steer_deg", self._on_gps_steer, 10)
        self.create_subscription(Float32, "gps_control/rpm", self._on_gps_rpm, 10)
        self.create_subscription(Int32, "gps_control/target_idx", self._on_gps_idx, 10)
        self.create_subscription(Bool, "gps_control/valid", self._on_gps_valid, 10)
        self.create_subscription(
            String, self.get_parameter("avoid_state_topic").value, self._on_avoid_state, 10
        )
        self.create_subscription(
            Int16, self.get_parameter("avoid_steer_topic").value, self._on_avoid_steer, 10
        )
        self.create_subscription(
            Int16, self.get_parameter("avoid_rpm_topic").value, self._on_avoid_rpm, 10
        )
        self.create_subscription(
            String, self.get_parameter("traffic_light_topic").value,
            self._on_traffic_light, 10,
        )
        self.parking_start_pub = {}
        for side in ("left", "right"):
            self.create_subscription(
                Int16, self.get_parameter(f"parking_{side}_cmd_rpm_topic").value,
                self._parking_cb("cmd_rpm", side), 10,
            )
            self.create_subscription(
                Int16, self.get_parameter(f"parking_{side}_cmd_steer_topic").value,
                self._parking_cb("cmd_steer", side), 10,
            )
            self.create_subscription(
                Int16, self.get_parameter(f"parking_{side}_cmd_enable_topic").value,
                self._parking_cb("cmd_enable", side), 10,
            )
            self.create_subscription(
                Bool, self.get_parameter(f"parking_{side}_active_topic").value,
                self._parking_cb("active", side), 10,
            )
            self.create_subscription(
                Bool, self.get_parameter(f"parking_{side}_mapping_topic").value,
                self._parking_cb("mapping", side), 10,
            )
            self.create_subscription(
                Bool, self.get_parameter(f"parking_{side}_done_topic").value,
                self._parking_cb("done", side), 10,
            )
            self.parking_start_pub[side] = self.create_publisher(
                Bool, self.get_parameter(f"parking_{side}_start_topic").value,
                _LATCHED_QOS,
            )
        self.avoid_enable_pub = self.create_publisher(
            Bool, self.get_parameter("can_bridge_enable_topic").value, 10
        )
        # So "what's actually driving right now" is one `ros2 topic echo`
        # away instead of having to read the throttled log line.
        self.active_source_pub = self.create_publisher(String, "control_arbiter/active_source", 10)

        rate = self.get_parameter("control_rate_hz").value
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info("control_arbiter ready.")

    def _on_camera_steer(self, msg):
        # Freshness (dead-man's switch) only - do NOT infer lane validity
        # from NaN here, yolopv2_zed_rpm_node's publish timer always sends
        # a real float (0.0 when the lane is lost), never NaN. See
        # camera_lane_valid_topic declaration above.
        self.camera_steer = msg.data
        self.camera_last_time = self.get_clock().now()

    def _on_camera_rpm(self, msg):
        # No separate freshness tracking - trusted only via camera_ok's
        # existing gate (camera_active + camera_last_time freshness, keyed
        # off the steer topic), same as camera_steer above. Already
        # curve-scaled + step-rate-limited by the camera node itself
        # (_speed_for_steer + rpm_step), so this is used as-is.
        self.camera_rpm = msg.data

    def _on_camera_lane_valid(self, msg):
        if msg.data:
            self.camera_good_streak += 1
            self.camera_bad_streak = 0
            if self.camera_good_streak >= self.get_parameter("camera_good_frames_to_enable").value:
                self.camera_active = True
        else:
            self.camera_bad_streak += 1
            self.camera_good_streak = 0
            if self.camera_bad_streak >= self.get_parameter("camera_bad_frames_to_disable").value:
                self.camera_active = False

    def _on_gps_steer(self, msg):
        self.gps_steer = msg.data

    def _on_gps_rpm(self, msg):
        self.gps_rpm = msg.data
        self.gps_last_time = self.get_clock().now()

    def _on_gps_idx(self, msg):
        self.gps_idx = msg.data

    def _on_gps_valid(self, msg):
        self.gps_valid = msg.data

    def _on_avoid_state(self, msg):
        self.avoid_state = msg.data
        self.avoid_last_time = self.get_clock().now()

    def _on_avoid_steer(self, msg):
        self.avoid_steer = msg.data

    def _on_avoid_rpm(self, msg):
        self.avoid_rpm = msg.data

    def _parking_cb(self, field, side):
        """Returns a bound callback that stores msg.data into
        self.parking[side][field] - cmd_rpm also stamps cmd_last_time
        (the freshness dead-man's switch for that side's relay)."""
        def _cb(msg):
            self.parking[side][field] = msg.data
            if field == "cmd_rpm":
                self.parking[side]["cmd_last_time"] = self.get_clock().now()
        return _cb

    def _on_cross_track(self, msg):
        self.cross_track_error = msg.data
        self.cross_track_last_time = self.get_clock().now()

    def _on_traffic_light(self, msg):
        self.traffic_light_state = msg.data
        self.traffic_light_last_time = self.get_clock().now()

    def _fresh(self, last_time, timeout_sec):
        if last_time is None:
            return False
        age = (self.get_clock().now() - last_time).nanoseconds / 1e9
        return age <= timeout_sec

    def _traffic_light_is_red(self):
        """Shared by the 'traffic_light' zone (which also has a stopline)
        and 'gps_priority'/'gps_priority_slow' (which don't - see their
        own branches). Fails safe to RED if traffic_light_node isn't
        running/publishing, same reasoning in both cases: better to
        crawl/stop through an unconfigured intersection than guess GO and
        drive through blind."""
        light_fresh = self._fresh(
            self.traffic_light_last_time,
            self.get_parameter("traffic_light_timeout_sec").value,
        )
        return (not light_fresh) or (self.traffic_light_state == "STOP")

    def _zone_at(self, idx):
        """Returns (start, end, kind, extra) - extra is the RAW 4th field,
        possibly None if the entry didn't have one. Each zone kind that
        uses it interprets None its own way at the call site (they mean
        different things: traffic_light defaults it to `end`, stop
        defaults it to "hold indefinitely" - see their handling in
        on_timer)."""
        for start, end, kind, extra in self.event_zones:
            if start <= idx <= end:
                return start, end, kind, extra
        return None, None, None, None

    def _camera_deviation_ok(self):
        """GPS cross-track veto state machine - see module docstring.
        Updates self._deviation_overridden/_deviation_locked_until as a
        side effect (called once per on_timer tick) and returns whether
        the camera is currently trusted from this check's perspective."""
        now = self.get_clock().now()

        if self._deviation_locked_until is not None:
            if now < self._deviation_locked_until:
                return False
            self._deviation_locked_until = None

        fresh = self._fresh(
            self.cross_track_last_time, self.get_parameter("cross_track_timeout_sec").value
        )
        if not fresh or math.isnan(self.cross_track_error):
            # No recent GPS cross-track data to check against - can't
            # veto on unknown information, but don't clear an existing
            # override either (stay conservative until data resumes).
            return not self._deviation_overridden

        dev = abs(self.cross_track_error)
        enter_m = self.get_parameter("camera_max_deviation_m").value
        reenter_m = self.get_parameter("camera_deviation_reenter_m").value
        reenter_streak_needed = self.get_parameter("camera_deviation_reenter_streak").value

        if not self._deviation_overridden:
            if dev > enter_m:
                self._deviation_overridden = True
                self._deviation_reenter_streak = 0
                window_sec = self.get_parameter("camera_deviation_lockout_window_sec").value
                self._deviation_trip_times = [
                    t for t in self._deviation_trip_times
                    if (now - t).nanoseconds / 1e9 <= window_sec
                ]
                self._deviation_trip_times.append(now)
                if len(self._deviation_trip_times) >= self.get_parameter(
                    "camera_deviation_lockout_count"
                ).value:
                    lockout_sec = self.get_parameter("camera_deviation_lockout_sec").value
                    self._deviation_locked_until = now + Duration(seconds=lockout_sec)
                    self.get_logger().warn(
                        f"camera deviation veto tripped {len(self._deviation_trip_times)}x "
                        f"in {window_sec}s - locking camera out for {lockout_sec}s"
                    )
        else:
            if dev < reenter_m:
                self._deviation_reenter_streak += 1
                if self._deviation_reenter_streak >= reenter_streak_needed:
                    self._deviation_overridden = False
            else:
                self._deviation_reenter_streak = 0

        return not self._deviation_overridden

    def on_timer(self):
        self._poll_diag_status()

        # camera_active: frame-count hysteresis, updated per camera frame
        # in _on_camera_steer. The freshness check is a separate dead-man's
        # switch for "camera node stopped publishing entirely" - frame
        # counters can't detect that on their own since no callbacks fire.
        camera_ok = (
            self.camera_active
            and self._fresh(self.camera_last_time, self.get_parameter("camera_timeout_sec").value)
            and self._camera_deviation_ok()
        )
        gps_ok = self.gps_valid and self._fresh(
            self.gps_last_time, self.get_parameter("gps_timeout_sec").value
        )
        avoid_ok = self._fresh(
            self.avoid_last_time, self.get_parameter("avoid_timeout_sec").value
        )
        zone_start, zone_end, zone, zone_extra = self._zone_at(self.gps_idx)

        # Timed "stop" zone hold state reset - see the "stop" branch below.
        # Cleared whenever idx isn't inside a "stop" zone at all, so a
        # later re-entry (this zone again, or a different one) starts a
        # fresh hold instead of remembering "already done" from last time.
        if zone != "stop":
            self._stop_hold_key = None
            self._stop_hold_start_time = None
            self._stop_hold_done = False

        # Edge-triggers /parking_start (on this zone's side only) exactly
        # once per zone entry (not every cycle - the parking node treats
        # each True as "begin a fresh attempt") and clears any stale
        # "done" from a previous run so a new attempt doesn't immediately
        # skip its own maneuver. GPS idx during the maneuver itself may
        # wander (the vehicle physically leaves the recorded line to pull
        # into/out of the slot) - self.parking[side]["done"], not idx
        # leaving the zone, is what actually ends this zone's special
        # handling below.
        # zone_was_active alone isn't quite enough of a guard: during the
        # maneuver the vehicle intentionally leaves the recorded line, so
        # stanley_control's global-nearest-waypoint search can momentarily
        # jump gps_idx just outside [start, end] and back (e.g. a slot
        # geometrically closer to some other waypoint than to anything
        # inside the zone) - that flips zone_was_active False then True
        # again within a cycle or two, which would otherwise look exactly
        # like "vehicle left and re-entered the zone" and fire a second
        # /parking_start while the first attempt is still finishing (or
        # right after DONE). completed_pending_clear closes that gap: once
        # a side finishes (done=True seen), no new trigger fires until
        # gps_idx has been solidly outside that zone's own [start, end]
        # (padded by parking_retrigger_clear_idx_margin) - not just
        # "not currently matching" for one cycle - so a real second lap
        # through a looping course still re-triggers normally once the
        # vehicle has actually moved away and come back.
        #
        # "engaged" (2026-08-07 fix): a REAL bug, not just the retrigger-
        # guard's - the dispatch chain below used to gate
        # _handle_parking_zone() purely on the CURRENT idx-based `zone`
        # match, contradicting this very comment block's own claim that
        # "self.parking[side]['done'], not idx leaving the zone" ends the
        # zone's handling. In practice: GPS-driven entry into a parking zone
        # (as opposed to a manual /parking_start at a standstill) can lock
        # onto a slot and start actually maneuvering, then the same idx-
        # wander described above carries gps_idx OUTSIDE the zone's
        # [start, end] mid-maneuver - at which point `zone` stopped being
        # "parking_left"/"parking_right" and control silently fell back to
        # plain GPS/camera driving right in the middle of the slot approach
        # ("공간 잡고 틀려고 하다가 그냥 GPS로 쭉 가는" - confirmed 2026-08-07).
        # `engaged` latches at the same trigger point as /parking_start and
        # stays true regardless of idx afterward, until state["done"]
        # (or an ABORT that a manual /parking_reset clears) - see the
        # dispatch chain below.
        retrigger_margin = self.get_parameter("parking_retrigger_clear_idx_margin").value
        for side in ("left", "right"):
            zone_now = (zone == f"parking_{side}")
            state = self.parking[side]

            if state["done"]:
                state["completed_pending_clear"] = True
                state["engaged"] = False
            # Unambiguous release (2026-08-12, widened from "only the
            # other parking side's zone" after this exact scenario showed
            # up live: idx briefly touched this side's zone at a run's
            # very first tick - likely a transient nearest-waypoint match
            # while GPS was still settling - latching engaged even though
            # the vehicle was never actually going to attempt this side's
            # slot this run, then sitting in some OTHER zone type
            # (gps_priority_slow) that the narrow check didn't cover).
            # Now fires on ANY other concrete zone, not just the sibling
            # parking zone. Debounced over parking_other_zone_confirm_ticks
            # consecutive ticks (not instant) specifically so a real
            # maneuver's normal brief idx-wander (stanley_control's
            # nearest-waypoint search jumping momentarily while the
            # vehicle is physically off the recorded line backing into a
            # slot) can't false-trigger this on a single noisy tick - that
            # was the original 2026-08-07 bug this whole mechanism exists
            # to prevent; a short sustained confirm keeps both fixes true.
            other_zone_now = zone is not None and zone != f"parking_{side}"
            if state["engaged"] and other_zone_now:
                state["other_zone_streak"] = state.get("other_zone_streak", 0) + 1
                confirm_ticks = self.get_parameter("parking_other_zone_confirm_ticks").value
                if state["other_zone_streak"] >= confirm_ticks:
                    self.get_logger().warn(
                        f"parking_{side}: still engaged at gps_idx={self.gps_idx}, "
                        f"which is now inside a different zone ({zone!r}) for "
                        f"{state['other_zone_streak']} consecutive ticks - releasing "
                        f"(node was likely abandoned mid-maneuver, or engaged never "
                        f"should have latched this run)"
                    )
                    state["engaged"] = False
                    state["engaged_stuck_since"] = None
                    state["other_zone_streak"] = 0
            else:
                state["other_zone_streak"] = 0
            if state["completed_pending_clear"] and state["clear_bounds"] is not None:
                b_start, b_end = state["clear_bounds"]
                if self.gps_idx < b_start - retrigger_margin or self.gps_idx > b_end + retrigger_margin:
                    state["completed_pending_clear"] = False
                    state["clear_bounds"] = None

            if zone_now and not state["zone_was_active"] and not state["completed_pending_clear"]:
                state["done"] = False
                state["clear_bounds"] = (zone_start, zone_end)
                state["engaged"] = True
                state["engaged_stuck_since"] = None
                state["other_zone_streak"] = 0
                self.parking_start_pub[side].publish(Bool(data=True))
            state["zone_was_active"] = zone_now

            # Stuck-engaged escape hatch (2026-08-12): `engaged` was only
            # ever designed to be released by state["done"] - deliberately,
            # so a brief idx-wander mid-maneuver can't silently hand
            # control back to GPS/camera (the original 2026-08-07 bug this
            # whole mechanism exists to fix). But if the parking node
            # itself gets abandoned mid-way (crashes, gets stuck in a state
            # that never reaches DONE/ABORT-with-recovery, or is simply
            # never re-triggered again), there was previously NO way to
            # recover short of restarting control_arbiter - confirmed live
            # 2026-08-12: idx was 30+ past this side's own zone bounds,
            # deep in the *other* side's zone, and engaged still held this
            # side's node's stale/absent commands. Real maneuvers finish
            # well within a minute (see this side's own arc/exit timeout
            # params) - so require BOTH a large idx excursion (well beyond
            # the ordinary mid-maneuver wander retrigger_margin already
            # tolerates) AND that excursion persisting for a full timeout
            # window, not just one tick, before concluding this is
            # abandonment rather than a real maneuver still in progress.
            if state["engaged"] and state["clear_bounds"] is not None:
                b_start, b_end = state["clear_bounds"]
                stuck_margin = self.get_parameter("parking_engaged_stuck_idx_margin").value
                far_outside = (
                    self.gps_idx < b_start - stuck_margin
                    or self.gps_idx > b_end + stuck_margin
                )
                if far_outside:
                    if state["engaged_stuck_since"] is None:
                        state["engaged_stuck_since"] = self.get_clock().now()
                    stuck_for = (
                        self.get_clock().now() - state["engaged_stuck_since"]
                    ).nanoseconds / 1e9
                    timeout = self.get_parameter("parking_engaged_stuck_timeout_sec").value
                    if stuck_for >= timeout:
                        self.get_logger().error(
                            f"parking_{side}: engaged stuck for {stuck_for:.0f}s with "
                            f"gps_idx={self.gps_idx} far outside its own zone "
                            f"{state['clear_bounds']} (margin={stuck_margin}) - force-"
                            f"releasing engaged, node was likely abandoned mid-maneuver"
                        )
                        state["engaged"] = False
                        state["engaged_stuck_since"] = None
                else:
                    state["engaged_stuck_since"] = None

        engaged_side = next(
            (s for s in ("left", "right") if self.parking[s]["engaged"]), None
        )

        # What camera/GPS would drive with if no event zone were active -
        # "traffic_light" zones use this as their steer/rpm base (they
        # modify speed, not steering) instead of dictating their own like
        # stop/gps_priority/avoid do.
        if camera_ok:
            base_steer = self.camera_steer
            # 2026-08-17: was a fixed camera_mode_rpm parameter (curvature-
            # blind - same rpm whether the camera was driving straight or
            # at full steering lock). Now the camera's own curve-scaled
            # rpm (see camera_rpm_topic's declaration comment + camera_ok's
            # freshness gate above, which already covers this topic too).
            base_rpm = self.camera_rpm
            base_source = "camera"
        elif gps_ok:
            base_steer = self.gps_steer
            base_rpm = self.gps_rpm
            base_source = "gps_fallback"
        else:
            base_steer, base_rpm, base_source = 0.0, 0.0, None

        # base_steer EMA lowpass - see base_steer_lowpass_alpha's declaration
        # comment above. Snaps (no blend-in lag) the first tick a source
        # becomes valid, and resets to None whenever base_source goes back
        # to None (both camera and gps invalid) so a later re-entry starts
        # fresh instead of blending from a steer value that's now stale by
        # however long the vehicle was in safe_stop/an event zone.
        if base_source is None:
            self._filtered_base_steer = None
        else:
            alpha = self.get_parameter("base_steer_lowpass_alpha").value
            if self._filtered_base_steer is None:
                self._filtered_base_steer = base_steer
            else:
                self._filtered_base_steer = (
                    alpha * base_steer + (1.0 - alpha) * self._filtered_base_steer
                )
            base_steer = self._filtered_base_steer

        # Pre-zone smooth deceleration into a parking zone (2026-08-07) -
        # see _parking_ramped_rpm() and parking_{side}_ramp_idx_margin's
        # declaration comment for why. Applied to base_rpm here so the
        # normal camera/GPS-driving fallback path eases down; the
        # "gps_priority" zone below needs the exact same treatment
        # separately since it sends self.gps_rpm directly instead of
        # routing through base_rpm (today's course has parking_left's ramp
        # window starting inside the gps_priority zone, at the user's
        # request to start slowing right after the traffic_light zone ends
        # rather than waiting for gps_priority's own end - if only base_rpm
        # were patched, that zone's branch would silently ignore the ramp
        # entirely and it would look like nothing happened while inside it).
        if base_source is not None:
            base_rpm = self._parking_ramped_rpm(base_rpm)

        # Arms/disarms obstacle_avoid_node - only while idx is in an "avoid"
        # zone. Published every cycle so it also disarms the instant the
        # vehicle exits the zone (obstacle_avoid_node resets its own state
        # machine to CLEAR on disarm).
        self.avoid_enable_pub.publish(Bool(data=(zone == "avoid")))

        if engaged_side is not None:
            # Takes priority over everything below, including a "stop"/
            # "gps_priority"/etc zone that idx may have wandered into mid-
            # maneuver - see the "engaged" comment above.
            self._handle_parking_zone(engaged_side, base_source, base_steer, base_rpm, gps_ok)
        elif zone == "stop":
            # zone_extra here = optional hold duration in seconds
            # ('start:end:stop:hold_sec', e.g. '44:44:stop:3') - None (old
            # 3-field 'start:end:stop') means hold indefinitely, the
            # original plain-stop behavior, unchanged (stop_mode=1/flat).
            # The timed branch below is the Hill_Stop stand-in specifically,
            # so it sends stop_mode=2 (hill) instead - not the "correct"
            # design (that's a real Hill_Stop state, not a stop-zone
            # variant), but close enough to test the CAN-level behavior
            # per the user's call, 2026-08-18.
            hold_sec = zone_extra
            if hold_sec is None:
                self._send_true_deg(
                    0.0, 0.0, 0, 1, "event_zone_stop", f"event_zone(stop, idx={self.gps_idx})"
                )
            else:
                key = (zone_start, zone_end)
                if self._stop_hold_key != key:
                    self._stop_hold_key = key
                    self._stop_hold_start_time = self.get_clock().now()
                    self._stop_hold_done = False

                if not self._stop_hold_done:
                    elapsed = (
                        self.get_clock().now() - self._stop_hold_start_time
                    ).nanoseconds / 1e9
                    if elapsed >= hold_sec:
                        self._stop_hold_done = True

                if not self._stop_hold_done:
                    self._send_true_deg(
                        0.0, 0.0, 0, 2, "event_zone_stop_timed",
                        f"event_zone(stop, idx={self.gps_idx}) holding "
                        f"{elapsed:.1f}/{hold_sec:.1f}s (stop_mode=2/hill)",
                    )
                elif base_source is not None:
                    # Hold's over - resume normal driving even though idx
                    # is technically still inside [zone_start, zone_end]
                    # (it hasn't moved while stopped). Vehicle moving again
                    # will naturally push idx past zone_end shortly.
                    self._send_true_deg(
                        base_steer, base_rpm, 1, 0, base_source,
                        f"event_zone(stop, idx={self.gps_idx}) hold done, resuming via {base_source}",
                    )
                else:
                    self._send_true_deg(
                        0.0, 0.0, 0, 1, "safe_stop",
                        f"event_zone(stop, idx={self.gps_idx}) hold done but no valid source to resume with",
                    )
        elif zone == "gps_priority" and gps_ok:
            # 2026-08-16: gps_priority excludes the ZED lane-following
            # camera (so slot detection right before a parking search
            # isn't fighting the camera for steering) - but that's a
            # completely separate pipeline from the OAK-D traffic_light
            # camera, which keeps running/detecting regardless. This zone
            # used to never check traffic_light_state at all, so a red
            # light sitting inside a gps_priority stretch (e.g. right
            # before a parking zone) was silently ignored. No stopline
            # concept here (unlike "traffic_light" zones), so red just
            # means a full stop for as long as it's red, not a graceful
            # approach-and-stop-at-line.
            if self._traffic_light_is_red():
                self._send_true_deg(
                    self.gps_steer, 0.0, 0, 1, "event_zone_gps_priority_red",
                    f"event_zone(gps_priority, idx={self.gps_idx}) RED - stopped",
                )
            else:
                self._send_true_deg(
                    self.gps_steer, self._parking_ramped_rpm(self.gps_rpm), 1, 0,
                    "event_zone_gps_priority",
                    f"event_zone(gps_priority, idx={self.gps_idx})",
                )
        elif zone == "gps_priority_slow" and gps_ok:
            # Same as gps_priority (camera excluded, GPS drives) but capped
            # at gps_priority_slow_rpm instead of full cruise - for transit
            # stretches between two parking zones. Same red-light gap fix
            # as gps_priority above.
            slow_rpm = min(
                self._parking_ramped_rpm(self.gps_rpm),
                self.get_parameter("gps_priority_slow_rpm").value,
            )
            if self._traffic_light_is_red():
                self._send_true_deg(
                    self.gps_steer, 0.0, 0, 1, "event_zone_gps_priority_slow_red",
                    f"event_zone(gps_priority_slow, idx={self.gps_idx}) RED - stopped",
                )
            else:
                self._send_true_deg(
                    self.gps_steer, slow_rpm, 1, 0,
                    "event_zone_gps_priority_slow",
                    f"event_zone(gps_priority_slow, idx={self.gps_idx})",
                )
        elif zone == "avoid" and avoid_ok and self.avoid_state == "CLEAR" and gps_ok:
            # In the zone, obstacle_avoid_node is scanning but hasn't found
            # anything yet - drive normally via GPS (keeps the curvature-
            # aware anticipatory-steering behavior instead of dropping to
            # obstacle_avoidance's own flat cruise_rpm) but capped at
            # avoid_zone_scan_rpm (2026-08-11 - "구간 rpm 자체를 80으로",
            # slower for the whole zone, more reaction margin, not just
            # once an obstacle is actually found below) - min() so a tight
            # curve's own curvature-based slowdown can still go below 80,
            # this only caps the ceiling.
            zone_rpm = min(
                self._parking_ramped_rpm(self.gps_rpm),
                self.get_parameter("avoid_zone_scan_rpm").value,
            )
            self._send_true_deg(
                self.gps_steer, zone_rpm, 1, 0,
                "event_zone_avoid_scanning",
                f"event_zone(avoid-clear->gps, idx={self.gps_idx})",
            )
        elif zone == "avoid" and avoid_ok and self.avoid_state in (
            "AVOID_LEFT", "AVOID_RIGHT", "PASS", "RETURN"
        ):
            # obstacle_avoid_node's steer is RAW firmware-scale (its own
            # avoid_steer_left/right params, e.g. -30/30) - must NOT go
            # through send_control_true_deg's scaling, that's only correct
            # for values already in true physical degrees.
            stop_mode = 1 if self.avoid_rpm == 0 else 0
            if self.bus is not None:
                can_driver.send_control(self.bus, self.avoid_rpm, self.avoid_steer, 1, stop_mode)
            self._log_can(
                f"event_zone_avoid_{self.avoid_state}",
                self.avoid_rpm, self.avoid_steer, 1, stop_mode)
            # Sub-state (AVOID_LEFT vs PASS vs RETURN, ...) is its own
            # category so each transition within the maneuver gets logged
            # too, not just the jump into/out of "avoid" as a whole.
            self._note_source(
                f"event_zone_avoid_{self.avoid_state}",
                f"event_zone(avoid, idx={self.gps_idx}, state={self.avoid_state}) "
                f"steer={self.avoid_steer}(raw) rpm={self.avoid_rpm}",
            )
        elif zone == "avoid":
            # Either obstacle_avoid_node isn't publishing (not running, or
            # hasn't caught up yet), it reported STOP, or it's CLEAR but
            # GPS isn't valid to drive with - fail safe rather than drive
            # through blind.
            self._send_true_deg(
                0.0, 0.0, 0, 1, "event_zone_avoid_stop",
                f"event_zone(avoid-stop, idx={self.gps_idx}, state={self.avoid_state}, avoid_ok={avoid_ok})",
            )
        elif zone == "traffic_light":
            is_red = self._traffic_light_is_red()
            # zone_stopline, NOT zone_end - see parse_event_zones' docstring.
            # Using zone_end here was a real bug: the instant gps_idx ticked
            # past it, this whole zone stopped matching in _zone_at() (zone
            # was None again) before the red-light debounce buffer had even
            # confirmed STOP, so the vehicle just drove through on whatever
            # camera/GPS was already doing - confirmed 2026-08-06, showed as
            # "GO" sailing through a red light right at the zone boundary.
            # zone_extra defaults to zone_end here (old 3-field behavior) -
            # "stop"'s branch below defaults the same raw field differently.
            zone_stopline = zone_extra if zone_extra is not None else zone_end
            at_stopline = self.gps_idx >= zone_stopline

            if base_source is None:
                self._send_true_deg(
                    0.0, 0.0, 0, 1, "traffic_light_no_driver",
                    f"event_zone(traffic_light, idx={self.gps_idx}) no camera/GPS to steer with",
                )
            elif at_stopline and is_red:
                self._send_true_deg(
                    base_steer, 0.0, 0, 1, "traffic_light_stop",
                    f"event_zone(traffic_light, idx={self.gps_idx}, stopline={zone_stopline}) RED - stopped",
                )
            elif is_red:
                scale = self.get_parameter("traffic_light_approach_rpm_scale").value
                self._send_true_deg(
                    base_steer, base_rpm * scale, 1, 0, "traffic_light_approach_slow",
                    f"event_zone(traffic_light, idx={self.gps_idx}, stopline={zone_stopline}) "
                    f"RED ahead - slowing ({base_source})",
                )
            else:
                self._send_true_deg(
                    base_steer, base_rpm, 1, 0, "traffic_light_go",
                    f"event_zone(traffic_light, idx={self.gps_idx}, stopline={zone_stopline}) "
                    f"GO ({base_source})",
                )
        elif zone in ("parking_left", "parking_right"):
            # Defensive fallback only - engaged_side above already covers
            # this exact case (it latches True on the same tick zone_now
            # first matches), so this branch should be unreachable in
            # practice. Kept in case engaged is ever cleared unexpectedly
            # while idx is still numerically inside the zone.
            side = zone.split("_", 1)[1]
            self._handle_parking_zone(side, base_source, base_steer, base_rpm, gps_ok)
        elif base_source is not None:
            self._send_true_deg(base_steer, base_rpm, 1, 0, base_source)
        else:
            self._send_true_deg(0.0, 0.0, 0, 1, "safe_stop", "none(safe-stop)")

    def _parking_ramped_rpm(self, raw_rpm):
        """Blend raw_rpm toward a not-yet-active parking zone's approach_rpm
        based on gps_idx's position inside that side's ramp window
        [zone_start-margin, zone_start) - see parking_{side}_ramp_idx_margin's
        declaration comment. Called from every place that would otherwise
        send an unramped rpm while gps_idx is approaching a parking zone
        (base_rpm's normal fallback, the "gps_priority" zone, the "avoid"
        zone's CLEAR-state GPS driving) - each needs its own call since none
        of them share a common code path. Idempotent/side-effect-free (no
        stored ramp state) so calling it multiple times or from multiple
        zones in the same tick is safe."""
        if raw_rpm is None:
            return raw_rpm
        out = raw_rpm
        for side in ("left", "right"):
            zone_start_for_side = next(
                (z_start for z_start, _, z_kind, _ in self.event_zones
                 if z_kind == f"parking_{side}"),
                None,
            )
            if zone_start_for_side is None:
                continue
            margin = self.get_parameter(f"parking_{side}_ramp_idx_margin").value
            if margin <= 0:
                continue
            ramp_lo = zone_start_for_side - margin
            if ramp_lo <= self.gps_idx < zone_start_for_side:
                frac = (self.gps_idx - ramp_lo) / float(margin)
                approach_rpm = self.get_parameter(f"parking_{side}_approach_rpm").value
                out = out * (1.0 - frac) + approach_rpm * frac
        return out

    def _handle_parking_zone(self, side, base_source, base_steer, base_rpm, gps_ok):
        """Shared logic for both "parking_left" (t_parking) and
        "parking_right" (parallel_parking) zones - see module docstring's
        "parking" zones paragraph for the full state-machine rationale."""
        state = self.parking[side]

        if state["done"]:
            # Maneuver already completed this attempt - hand back to
            # whatever would normally be driving, same as if this zone
            # weren't here. Only state["done"] gates this, not idx, since
            # idx may not have moved on/off the zone boundary the way a
            # normal drive-through zone would.
            if base_source is not None:
                self._send_true_deg(base_steer, base_rpm, 1, 0, base_source)
            else:
                self._send_true_deg(0.0, 0.0, 0, 1, "safe_stop", "none(safe-stop)")
            return

        if state["mapping"]:
            # Parking node is still scanning for/locking onto a slot (its
            # own APPROACH state) - drive straight via GPS instead of
            # trusting the parking node's own approach controller, same
            # posture as the "avoid" zone's CLEAR-state GPS driving. The
            # parking node keeps observing /scan_parking + /wheel_odom and
            # planning regardless of who's actually driving - only the
            # actuation source changes here, not whether it's working.
            if gps_ok:
                approach_rpm = self.get_parameter(f"parking_{side}_approach_rpm").value
                self._send_true_deg(
                    self.gps_steer, approach_rpm, 1, 0, f"event_zone_parking_{side}_mapping",
                    f"event_zone(parking_{side}-mapping->gps, idx={self.gps_idx}) "
                    f"rpm={approach_rpm}(parking approach speed, not cruise)",
                )
            else:
                self._send_true_deg(
                    0.0, 0.0, 0, 1, f"event_zone_parking_{side}_mapping_stop",
                    f"event_zone(parking_{side}-mapping, idx={self.gps_idx}) "
                    f"no GPS to drive straight with",
                )
            return

        fresh = self._fresh(
            state["cmd_last_time"], self.get_parameter("parking_cmd_timeout_sec").value
        )
        if fresh and state["active"]:
            # stop_mode=1 ("flat stop") whenever the commanded rpm is 0,
            # not just when disabled (2026-08-11 - found by reading the
            # AURIX firmware, Can_Comms.c/App_Control.c): the firmware only
            # sets its internal flatStopMode=TRUE on an explicit stop_mode
            # ==1 byte; stop_mode==0 with rpm==0 (what every parking-stop
            # relay used to send) leaves flatStopMode at whatever it was
            # last set to while actively driving (FALSE) - so
            # StopHoldEnable ends up TRUE, a closed-loop "hold exactly this
            # position/rpm" controller that hunts against drivetrain
            # backlash/momentum ("덜컹덜컹" judder, integrated-launch-only
            # since standalone direct_cmd_output bypasses this relay
            # entirely). The PS2 controller's own square-button stop
            # explicitly sets flatStopMode=TRUE and stops cleanly - this
            # mirrors that exact behavior instead of guessing at software-
            # side causes (timeout widening, enable/stop_mode unification)
            # that didn't fully fix it.
            stop_mode = 1 if (state["cmd_enable"] == 0 or state["cmd_rpm"] == 0) else 0
            if self.bus is not None:
                can_driver.send_control(
                    self.bus, state["cmd_rpm"], state["cmd_steer"], state["cmd_enable"], stop_mode,
                )
            self._log_can(
                f"event_zone_parking_{side}", state["cmd_rpm"], state["cmd_steer"],
                state["cmd_enable"], stop_mode)
            self._note_source(
                f"event_zone_parking_{side}",
                f"event_zone(parking_{side}, idx={self.gps_idx}) "
                f"steer={state['cmd_steer']}(raw) rpm={state['cmd_rpm']} "
                f"enable={state['cmd_enable']}",
            )
        else:
            # Parking node either isn't running yet (just triggered, hasn't
            # published a command yet), stopped publishing (crashed), or
            # reports itself not active - fail safe rather than drive
            # through a parking maneuver blind. enable=1/stop_mode=1
            # (2026-08-11): enable=1 so a borderline-stale-timestamp flap
            # between this branch and the real relay above doesn't toggle
            # enable back and forth on every flap; stop_mode=1 ("flat
            # stop") for the actual judder fix - see the relay branch
            # above's comment (firmware only disables its closed-loop
            # position/rpm-hold controller on an explicit stop_mode==1
            # byte, not merely rpm==0).
            self._send_true_deg(
                0.0, 0.0, 1, 1, f"event_zone_parking_{side}_wait",
                f"event_zone(parking_{side}, idx={self.gps_idx}) waiting "
                f"(active={state['active']}, fresh={fresh})",
            )

    def _send_true_deg(self, steer, rpm, enable, stop_mode, category, detail=None):
        steer_limit = self.get_parameter("steer_limit_deg").value
        steer = max(-steer_limit, min(steer_limit, steer))

        if self.bus is not None:
            can_driver.send_control_true_deg(self.bus, rpm, steer, enable, stop_mode)
        self._log_can(category, rpm, steer, enable, stop_mode)

        self._note_source(
            category,
            f"{detail or category} steer={steer:.1f}deg rpm={rpm:.0f} "
            f"enable={enable} stop_mode={stop_mode}",
        )

    def _log_can(self, category, rpm, steer, enable, stop_mode):
        """Appends one row per actual CAN send - call this at every real
        `can_driver.send_control*` call site, not inside `_note_source`
        (which also fires on the parking-wait fail-safe branch and other
        paths that don't always map 1:1 to a fresh CAN frame with known
        numeric rpm/steer here)."""
        if self._can_log_writer is not None:
            t = (self.get_clock().now() - self._can_log_start).nanoseconds / 1e9
            try:
                self._can_log_writer.writerow([
                    f"{t:.3f}", category, f"{rpm:.1f}", f"{steer:.2f}",
                    enable, stop_mode,
                ])
                self._can_log_fh.flush()
            except OSError:
                pass

        # Also send this same info out as a real CONTROL_META CAN frame
        # (0x203, see can_driver.py / README_CAN_PROTOCOL.md) so it shows
        # up on a CANoe listen-only channel too, not just this CSV - this
        # was the whole point of adding it (2026-08-15).
        self._last_requested_stop_mode = stop_mode
        if self.bus is not None:
            controller_id = _controller_id_for_category(category)
            self._can_meta_seq = (self._can_meta_seq + 1) & 0xFF
            try:
                can_driver.send_control_meta(
                    self.bus, rpm, steer, stop_mode, controller_id,
                    self._can_meta_seq)
            except Exception as exc:
                self.get_logger().warn(
                    f"CONTROL_META send failed ({exc})", throttle_duration_sec=5.0)

    def _poll_diag_status(self):
        """Drains DIAG_STATUS(0x104) off the bus - firmware confirmed
        sending it as of 2026-08-16 (Can_Comms_SendDiagStatus), see
        README_CAN_PROTOCOL.md. Logs every frame to its own CSV and warns
        loudly the instant applied_stop_mode disagrees with the last
        stop_mode we actually requested (0x200/CONTROL_META), or any
        fault_flags bit is set - this is the whole reason 0x104 exists
        (catch "we asked for flat, firmware held" without needing to read
        firmware source again)."""
        if self.bus is None:
            return
        prev = self.diag_status
        try:
            self.diag_status = can_driver.poll_diag_status(self.bus, self.diag_status)
        except Exception as exc:
            self.get_logger().warn(
                f"DIAG_STATUS poll failed ({exc})", throttle_duration_sec=5.0)
            return
        if self.diag_status is None or self.diag_status is prev:
            return

        diag = self.diag_status
        requested = self._last_requested_stop_mode
        # DIAG_STATUS's applied_stop_mode enum (0=disabled/1=flat/2=hold)
        # is NOT the same enum as the requested stop_mode
        # (0=normal/1=flat/2=hill) - see README_CAN_PROTOCOL.md's
        # applied_stop_mode section. Only 1(flat) lines up directly; a
        # requested 0(normal) is expected to come back as 2(hold) (both
        # mean "closed-loop hold" at the actuation layer), so that specific
        # combination is NOT a mismatch.
        mismatch = False
        if requested is not None:
            if requested == 1 and diag["applied_stop_mode"] != 1:
                mismatch = True
            elif requested in (0, 2) and diag["applied_stop_mode"] not in (2,):
                mismatch = True

        if self._diag_log_writer is not None:
            t = (self.get_clock().now() - self._can_log_start).nanoseconds / 1e9
            try:
                self._diag_log_writer.writerow([
                    f"{t:.3f}", requested, diag["applied_stop_mode"], int(mismatch),
                    diag["fault_flags"], diag["steer_pwm_duty"],
                    diag["supply_voltage_mV"], diag["rx_seq_echo"],
                ])
                self._diag_log_fh.flush()
            except OSError:
                pass

        if mismatch:
            self.get_logger().warn(
                f"[DIAG_STATUS] stop_mode mismatch: requested={requested} "
                f"applied={diag['applied_stop_mode']}",
                throttle_duration_sec=1.0)
        if diag["fault_flags"] != 0:
            self.get_logger().warn(
                f"[DIAG_STATUS] fault_flags=0x{diag['fault_flags']:02x}",
                throttle_duration_sec=1.0)

    def _note_source(self, category, detail):
        """Publishes the current control source every cycle (for
        status_check.sh / topic echo), but only WRITES a log line either
        (a) once per second regardless (so you can see it's alive), or
        (b) immediately, un-throttled, whenever the category actually
        changes - so mode switches show up right away instead of waiting
        for the next throttled tick, and are visually distinct ([MODE
        CHANGE]) from the routine heartbeat lines."""
        self.active_source_pub.publish(String(data=detail))

        if category != self._last_category:
            self.get_logger().info(
                f"[MODE CHANGE] {self._last_category} -> {category} | {detail}"
            )
            self._last_category = category
        else:
            self.get_logger().info(detail, throttle_duration_sec=1.0)

    def shutdown(self):
        if self.bus is not None:
            try:
                can_driver.send_control(self.bus, 0, 0, 0, 0)
            except Exception:
                pass
        if self._can_log_fh is not None:
            try:
                self._can_log_fh.close()
            except OSError:
                pass
        if self._diag_log_fh is not None:
            try:
                self._diag_log_fh.close()
            except OSError:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ArbiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

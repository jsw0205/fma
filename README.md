# ros2_ws

_Last updated: 2026-08-03_

GPS/RTK waypoint following for the rover. u-blox ZED-F9P + NTRIP RTK, IMU
(gyro+magnetometer) heading fusion, Stanley controller with anticipatory
cornering + physical-feasibility corner-cutting, CAN motor interface, and
(2026-07-26/27) a camera/GPS/LiDAR-avoidance priority arbiter so multiple
control sources can share one vehicle safely. As of 2026-07-28 the camera
side is real: ZED SDK + `zed_camera` (YOLOPv2 lane following) is installed,
built, and integrated into one launch file with GPS+RTK and the arbiter,
and a new `traffic_light` package (OAK-D + YOLO) is wired into
`control_arbiter` as a code-only integration (no OAK-D connected/tested
yet) — see "2026-07-28 session" below. GPS briefly hit `num_sv=0` (zero
satellites) on 2026-07-28 but fully recovered later the same day
(Float confirmed standalone *and* in the combined GPS+ZED launch,
antenna/RF path verified healthy via `UBX-MON-HW`) — treated as a
transient glitch, likely resolved by an incidental power-cycle, not a
lasting hardware or software issue. See "GPS `num_sv=0`" below.

## Packages

- **f9p_bringup** — GPS/RTK bring-up (`ublox_gps_node` + `rtk_bridge.py` + odometry/TF).
- **waypoint_follower** — waypoint recording/following, Stanley controller, CAN driver, `control_arbiter` (camera/GPS/avoidance priority switch), visualization, drive logging. Two integrated launch files: `integrated_drive.launch.py` (everything together) and `post_gps_drive.launch.py` (NEW 2026-07-29, everything except GPS - for starting GPS standalone first to avoid ZED USB3-EMI degrading the fix, see "How to run everything" below).
- **rtk_bridge** — standalone NTRIP→serial RTCM relay script (not a ROS package, no `package.xml`). Used by `f9p_bringup`.
- **obstacle_avoidance** — LiDAR (RPLiDAR A3M1) obstacle avoidance state machine for the real HENES vehicle (CLEAR→AVOID→PASS→RETURN→CLEAR), copied in from `~/Downloads/obstacle_avoidance` and integrated with `control_arbiter` on 2026-07-27 (see below). Its own `README.md` has full hardware/run instructions.
- **handsfree_ros2_imu** — driver for the HandsFree HFI-A9 IMU, cloned from
  [3bdul1ah/handsfree_ros2_imu](https://github.com/3bdul1ah/handsfree_ros2_imu). Publishes
  `sensor_msgs/Imu` on `/handsfree/imu` and `sensor_msgs/MagneticField` on `/handsfree/mag`.
- **zed_camera** (NEW 2026-07-28) — ZED2i + YOLOPv2 lane-following node. `yolopv2_zed_rpm_node` (current, `LaneTracker`-based: EMA half-lane-width learning, left/right hysteresis, handles a single visible lane, curve auto-slowdown, `~/lane_valid` + `~/steering_deg` published on a fixed 20Hz timer) is the one actually run; `yolopv2_zed_node` (older sliding-window version) is kept but unused. Weights live at `src/zed_camera/weights/yolopv2.pt`. See "2026-07-28 session" below for the install/build story.
- **traffic_light** (NEW 2026-07-28) — OAK-D + YOLO traffic-light detector, ported from a standalone script (`~/Downloads/test__sunny/test_sunny.py`) into a proper ROS2 node (`test_sunny_node`, from `traffic_light_node.py`). Publishes `/traffic_light` (`std_msgs/String`, `"GO"`/`"STOP"`) for `control_arbiter`'s new `traffic_light` event-zone type to consume. **Not hardware-tested yet** — no OAK-D connected this session, this was a code-only integration (depthai/ultralytics not installed, node fails gracefully into an idle state if either is missing or the camera isn't found, doesn't crash the launch). See "2026-07-28 session" below.
- **zed-ros2-wrapper** — official Stereolabs ROS2 driver for the ZED2i, cloned from [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) and built from source. Publishes the camera topics `zed_camera` subscribes to.
- **pcan_tools**, **fma**, **ydlidar_ros2_driver** — untouched this session.

## How to run everything

Each piece works fine run standalone too (see "individually" below) — pick
whichever gives you the visibility you need.

### GPS + RTK (required)

```bash
source ~/ros2_ws/install/setup.bash
ros2 run ublox_gps ublox_gps_node --ros-args \
  --params-file /home/a/ros2_ws/src/f9p_bringup/config/ublox_rover.yaml
```
Wait for `configured successfully` in the log (a cold reset + reconnect is
normal and can take ~15s) before starting the next piece.

```bash
python3 /home/a/ros2_ws/src/rtk_bridge/rtk_bridge.py
```
Publishes NavSatFix on plain `/fix` (no remap) when run this way.

Or both together via `f9p_bringup`'s launch (starts `rtk_bridge` 2s after
`ublox_gps_node` - remaps `/fix` → `/ublox_gps_node/fix`):
```bash
ros2 launch f9p_bringup f9p_rover.launch.py
```

### IMU (optional, adds fast heading updates + instant bootstrap)

```bash
ros2 run handsfree_ros2_imu hfi_a9_ros2 --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=921600
```
`waypoint_follower_node` uses it automatically if running (`use_imu:=true`,
`use_magnetometer:=true` by default) — see the IMU section below. Not
required; without it, `waypoint_follower_node` falls back to pure
GPS-movement heading exactly like before the IMU existed.

### Waypoint recording

```bash
ros2 run waypoint_follower waypoint_recorder_node --ros-args \
  -p gps_topic:=/fix \
  -p output_file:=/home/a/ros2_ws/src/waypoint_follower/waypoints/new_path.csv \
  -p min_waypoint_distance_m:=1.0
```
Ctrl+C saves. Overwrites `output_file` with no backup if it already exists.

Current recorded courses in `waypoints/`: `my_path.csv` (67 points, first
real recording), `new_path.csv` (25 points, small L-shaped corner course,
used for most of this session's controller-tuning tests), `sample_waypoints.csv`
(dummy placeholder values, not a real place - don't drive this one).

### Waypoint following

```bash
ros2 run waypoint_follower waypoint_follower_node --ros-args \
  -p gps_topic:=/fix \
  -p waypoints_file:=/home/a/ros2_ws/src/waypoint_follower/waypoints/new_path.csv \
  -p enable_control:=true
```
(`gps_topic` is `/ublox_gps_node/fix` instead if GPS was started via
`f9p_bringup`'s launch, which remaps it.) `enable_control:=false` (default)
skips sending CAN commands - visualization-only. Prints, throttled 1/sec:
`target_idx=... steer=...deg speed=...m/s | CAN: rpm=... enable=... stop_mode=...`.
Saves a drive log CSV to `~/drive_log_<timestamp>.csv` on shutdown.

### Visualization (optional)

```bash
ros2 run waypoint_follower mpl_viz_node
```
Plain-matplotlib alternative to RViz — path, waypoints, target, vehicle
position/heading, steer angle. Reads `waypoint_follower/markers`.

### Bundled launch

`ros2 launch waypoint_follower gps_rtk_waypoint.launch.py` starts
`f9p_bringup`'s GPS+RTK stack + `waypoint_follower_node` + `mpl_viz_node`
together (not the IMU driver - start that separately, see above). Args:
`waypoints_file:=...` (default `waypoints/my_path.csv`), `enable_control:=true`.

**For the multi-source (camera/GPS/avoidance) setup** (2026-07-27, see
below for details), use `integrated_drive.launch.py` instead — it adds
`control_arbiter` and runs the GPS node with `publish_can_directly:=false`
so the arbiter is the only thing writing CAN:
```bash
ros2 launch waypoint_follower integrated_drive.launch.py
```

**If GPS needs a clean run at Float/Fixed before the ZED's USB3 link
comes up** (confirmed EMI issue, see "USB3-EMI concern" below) — start
GPS standalone first, then bring up everything else once it has a fix:
```bash
ros2 launch f9p_bringup f9p_rover.launch.py
# wait for Float/Fixed, then in another terminal (leave GPS running):
ros2 launch waypoint_follower post_gps_drive.launch.py
```
`post_gps_drive.launch.py` (NEW 2026-07-29) is `integrated_drive.launch.py`
minus the GPS/RTK bring-up - same args, same nodes otherwise. GPS never
gets restarted so there's no fix to lose (the u-blox receiver keeps
tracking regardless of whether a host has the serial port open). Make
sure the ZED is physically plugged in before running it - `zed_wrapper`
retries opening the camera for ~28s then dies for good with no
auto-respawn.

### CAN (only needed for `enable_control:=true`)

```bash
sudo ip link set can0 up type can bitrate 500000
```
`can_driver.open_bus()` assumes this is already up, doesn't configure it.

### Checking status

```bash
ros2 topic hz /ublox_gps_node/fix        # should be 20Hz (or /fix if run standalone)
ros2 topic echo /navpvt --field flags    # 1=no RTK, 64=Float, 128/131=Fixed
ros2 topic echo /navpvt --field num_sv   # satellite count
ros2 topic hz /rxmrtcm                   # confirms RTCM is actually reaching the receiver
```

## GPS/RTK pipeline — root causes found this session

**The stock `ros-humble-ublox-gps` driver has no RTCM ROS subscriber at all**
(no `rtcm_msgs` dependency, checked its `package.xml` and headers directly) —
any ROS-native NTRIP client publishing to a `/rtcm` topic talks to nobody.
Fix: `rtk_bridge.py`, a plain script that fetches RTCM from NTRIP and writes
bytes straight to the serial port - a previously-proven combo (confirmed RTK
Fixed, `flags=131`, before this session). `f9p_bringup/f9p_rover.launch.py`
runs `ublox_gps_node` + `rtk_bridge.py` together, `rtk_bridge.py` started
via a `TimerAction` 2s after `ublox_gps_node` - not waiting for
"configured successfully" (confirmed unnecessary), but launching both in
the exact same instant (0 delay) reliably corrupts `ublox_gps_node`'s
serial read (confirmed: massive `End of file` error spam within seconds),
so a short fixed delay is still needed to avoid the simultaneous-open
collision. `rtk_bridge.py`'s initial GGA read
(`read_latest_gga_from_gps`) also now catches `SerialException` and falls
back to the hardcoded `INIT_LAT`/`INIT_LON` position instead of crashing,
since a transient serial read failure there shouldn't be fatal.

In `f9p_bringup/config/ublox_rover.yaml`:
- `rate: 20.0` + a `save:` block so it persists to EEPROM (was missing -
  that's why 20Hz kept reverting to 1Hz after power cycles).
- `usb.in`/`usb.out` protocol mask - the receiver connects over USB, which
  has separate protocol config from `uart1`; only `uart1.in` had RTCM3
  enabled, so RTCM was silently dropped at the USB port level too.
- Confirmed working baud is **19200**.
- **`device` now points at the stable by-id symlink**
  (`/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00`,
  same fix applied in `rtk_bridge.py`'s `GPS_PORT` and
  `f9p_rover.launch.py`'s `device` arg default) instead of a plain
  `/dev/ttyACM0` - when the USB connection drops (see "EOF" below) it can
  reconnect as `ttyACM1`, `ttyACM2`, etc., and the plain path then points
  at nothing while the by-id symlink keeps following the real device. The
  IMU (`/dev/ttyUSB0`) has the same class of risk but hasn't needed this
  fix yet - if it starts happening, its by-id path is
  `/dev/serial/by-id/usb-Silicon_Labs_HandsFree_IMU_USB_to_UART_Bridge_Controller_0001-if00-port0`.
- **`U-Blox ASIO input buffer read error: End of file, 0`**: confirmed
  cause - the USB connection actually drops and reconnects under a new
  `/dev/ttyACM*` number (verified via `lsusb`/`ls /dev/serial/by-id/`
  during a live failure). The by-id fix above means the *next* run finds
  the device again automatically; it does not stop the drop from
  happening in the first place - if this keeps recurring, it's a cable/
  connection/power issue worth chasing physically (try a direct USB
  connection with no hub).

## Waypoint follower — controller changes

Current defaults as of this update (2026-07-25; **superseded 2026-07-26 —
see the "2026-07-26/27 session" section below for `steer_limit_deg=14.3`,
`curve_lookahead_m=6.0`, and why**): `stanley_k=0.3`, `cruise_rpm=140`,
`min_curve_rpm=50`, `steer_limit_deg=30`, `steer_sign=-1`,
`wheelbase_m=0.735`, `curve_lookahead_m=2.0`, `curve_lead_margin=1.5`,
`curve_angle_for_min_rpm_deg=40`, `heading_lookback_m=0.15`.

- `steer_limit_deg` → **30** (vehicle's actual max steering lock, confirmed
  by the user; was a conservative 20 before that). **Turned out to be
  wrong** — the vehicle's *true* max lock is 14.3°, "30" was the
  firmware's own (miscalibrated) notion of full lock, not the real wheel
  angle. See below.
- `steer_sign` → **-1** (steering was inverted on the real vehicle; this
  param exists to flip it).
- `waypoint_arrival_radius_m` → **0.5** (only affects the *final*
  waypoint's arrival/stop check, not intermediate tracking precision).
- Added throttled (1/sec) terminal logging of `target_idx`/`steer`/`speed`
  plus the actual CAN-bound values (`rpm`/`enable`/`stop_mode`).
- **Straight-driving fallback**: when `self.yaw` is still `None` (no
  heading yet), drives straight (`steer=0`, `cruise_rpm`) instead of
  sitting at `stop_mode=1` forever - previously a stationary vehicle could
  never move at all, since heading only came from GPS movement and driving
  required heading first (a deadlock). Now largely moot once IMU/
  magnetometer bootstrap the heading instantly (see IMU section), but kept
  as the fallback for when they're not running.
- Added `mpl_viz_node.py` (plain-matplotlib RViz alternative) and
  `gps_rtk_waypoint.launch.py` rewritten to bundle everything (see "How to
  run" above).

### Curvature-based speed + anticipatory steering

`stanley_controller.py`'s `lookahead_path_curve()` walks forward from the
current target **a fixed distance** (`curve_lookahead_m`, meters - not a
fixed waypoint count) accumulating heading change, so a real corner packed
into a short distance reads as sharp regardless of how many closely-spaced
waypoints it's split across, while a wide gentle bend spread over the same
distance does not - a fixed point-count lookahead doesn't have this
property (tested: inflates angle for gentle bends too as you increase the
count, defeating the point).

Two independent things use that curve measurement:
- **`_curvature_scaled_rpm()`** (waypoint_follower_node.py): scales
  `cruise_rpm` down towards `min_curve_rpm` as the upcoming angle
  approaches `curve_angle_for_min_rpm_deg`.
- **Anticipatory heading blend** (`stanley_control()`'s `turning_radius_m`
  param, computed from `wheelbase_m`/`tan(steer_limit_deg)` -
  `_turning_radius_m()`): required lead distance to roll out a turn of
  angle θ at that radius is `radius * θ`, scaled up by `curve_lead_margin`
  (>1.0 = start turning earlier than the bare physics minimum, to leave
  room for actuator/filter lag - see below). Once within that distance of
  a sharp bend, the **heading-error term only** blends towards the
  post-turn direction (cross-track error still uses the true current-
  segment heading, unchanged) - plain Stanley has no preview and turns
  "too late" at speed, which shows up as running wide on the outside of
  corners.

**Analyzed a real run** (`drive_log_20260724_171240.csv`, cruise_rpm=140,
stanley_k=0.5 at the time, curve_lead_margin=1.1): finished the course but
ran 17% longer than planned, cross-track RMSE 0.57m concentrated at the
corner (0.63m mean there vs 0.39-0.49m elsewhere), steering pinned at the
±30° limit 10.5% of the time - a clean "running wide, reacting too late"
signature, *not* oscillation (steer changed ≤9°/sample, cross-track error
flipped sign only once the whole run). Response: `curve_lead_margin`
1.1→1.5, `curve_angle_for_min_rpm_deg` 60→40 (this course's corner only
reaches ~40-57° depending on lookahead distance, so 60 rarely triggered
the full `min_curve_rpm` floor), `stanley_k` back down to 0.3 (0.5 was
untested at this speed and gain only affects the cross-track term, not
heading error, so it wasn't strictly the fix for "reacts too late" -
re-test needed to see if the margin/angle changes alone fixed the
wide-cornering, independent of gain).

**Known real-world gap, not yet handled**: `required_lead_m` is pure
geometry (`radius * angle * margin`) with no speed term, but *reaching* a
commanded steering angle takes real time - both `_apply_lowpass()`'s
software smoothing (~0.24s to ~95% settled, from `lowpass_fc_hz=2.0`) and
the physical actuator's own response lag. At higher speed the same time
lag consumes more distance before the wheels actually reach the target
angle, so higher speed should widen the lead distance too
(`required_lead_m += v * settling_time`), not just via the fixed margin.
Not implemented yet. Related: confirmed (via a live no-slip/kinematics
discussion) that turning radius itself is speed-independent under the
bicycle model - it's specifically this reaction-time-to-distance
conversion that makes speed matter in practice, not the geometry.

**Also known-missing**: no feasibility check - nothing detects "this
corner literally can't be made at this speed/radius" and compensates (e.g.
by deliberately cutting the corner as a recovery strategy); it just
applies the same proportional law regardless and will track poorly if
truly infeasible. `_curvature_scaled_rpm()`'s slowdown is heuristic
(chosen threshold angle/floor), not derived from an actual minimum-speed-
to-complete-this-corner calculation.

**Also not yet fixed**: `stanley_control()`'s nearest-waypoint search is
still global (whole path, every cycle) - if the course self-intersects
(e.g. an intersection crossed twice), `target_idx` can jump to the
geometrically-nearest point regardless of route order. Relevant because
this node is meant as a fallback when a camera/lane-following system loses
tracking, so it needs to self-locate from an arbitrary mid-course
position - discussed fix: full search only when there's no previous
`target_idx` (first activation), then a window around the previous
`target_idx` on subsequent cycles.

### Heading estimation - distance-based, not sample-count-based

`_smoothed_heading()` used to compare position exactly `heading_window_n`
GPS samples back (fixed count) and require ≥`heading_min_movement_m` net
displacement to trust it - at the original defaults (n=4, 1.0m) that
needed ~5 m/s just to ever trigger at all. Replaced both params with a
single **`heading_lookback_m`** (default 0.15m): walks back through
`position_history` until it finds a point at least that far away, then
uses that bearing - same reasoning as `curve_lookahead_m` above. No
minimum-speed requirement; updates every GPS fix while genuinely moving
(the "lookback" is a sliding distance window recomputed fresh each cycle,
not a "wait N cm then update once" throttle). While stationary it keeps
recomputing to the same answer each cycle (frozen `position_history[-1]`
+ frozen lookback point → same result), which is practically equivalent
to "frozen" but is worth knowing is a *recompute*, not a skip.

Note: `position_history`'s own append threshold (`on_fix`, 1mm) is far
below real GPS noise (RTK Fixed is realistically a few mm to ~1-2cm) - it
only dedupes exact-duplicate points, it is not a noise filter. The actual
noise rejection comes from `heading_lookback_m` (15cm) being much larger
than that noise floor, not from the 1mm append threshold.

## IMU (heading fusion)

Heading was previously GPS-movement-based only. An IMU now provides fast
updates between GPS fixes, plus (new) instant heading at standstill via
magnetometer:

- Hardware: **HandsFree HFI-A9**, `/dev/ttyUSB0` (identifies as
  `HandsFree_IMU_USB_to_UART_Bridge_Controller`, has a stable by-id path
  too - see GPS section above). Baud **921600**.
- Driver: cloned `handsfree_ros2_imu` from GitHub (no apt package exists).
  Reverse-verified its frame protocol (`0xAA 0x55` header, length byte
  doubles as type ID, CRC-16/MODBUS checksum) against raw bytes off the
  port - matches the driver source exactly.
- Needed to fix `python3-transforms3d` (apt, 0.3.1, uses removed NumPy 1.x
  APIs) breaking under this machine's pip NumPy 2.2.6 - same class of
  issue as the matplotlib fix. Fixed with
  `pip install --user --upgrade transforms3d` (→0.4.2) +
  `sudo apt install ros-humble-tf-transformations` (wasn't installed).
- **Gyro** (`angular_velocity.z`, integrated in `on_imu()`): fills in
  between GPS fixes. Needs `self.yaw` already seeded (gyro rate alone
  can't bootstrap an absolute heading). Bench-tested stationary: ~10min
  run stayed within ±0.3°, a second ~5min run within -0.16° to +0.13° -
  raw rate at rest sits near zero (~±0.001-0.005 rad/s), no visible
  constant bias. **Not tested while actually driving** (vibration will
  likely be worse).
- **Magnetometer** (`on_mag()`, new): gives an *absolute* heading
  immediately, even at standstill - unlike GPS movement heading, no
  bootstrap problem. The IMU's own onboard fused `orientation` in the Imu
  message is magnetometer-referenced too but was deliberately *not* used
  for this - using the raw `MagneticField` directly instead, so this
  codebase's own declination correction and calibration params apply
  cleanly. Magnetic north ≠ true north (GPS/waypoint frame), so
  `magnetic_declination_deg` (default -8.0, South Korea ballpark -
  **verify for the exact location/date**, e.g. NOAA's declination
  calculator) corrects for it. `mag_offset_x`/`mag_offset_y` are hard-iron
  calibration (see `mag_calibrate.py` below); `mag_mount_offset_deg`
  absorbs IMU mounting misalignment/axis-convention error, tuned
  empirically by facing a known bearing and adjusting until `self.yaw`
  matches.
  - Confirmed with the user: this IMU is mounted **fully isolated from the
    motors** (no dynamic current-EMI concern). Sitting on/near a **steel
    frame** is a different, mostly-static distortion (hard-iron offset +
    soft-iron warping from the ferrous material, not current-driven) that
    a proper calibration *can* largely correct, as long as the IMU stays
    rigidly fixed relative to the frame and calibration is done with it
    already in its final mounted position. Soft-iron (elliptical
    distortion, needs a rotation/scale matrix) is not implemented yet,
    only hard-iron (offset).
  - `mag_calibrate.py` (`ros2 run waypoint_follower mag_calibrate`): spin/
    drive the vehicle through one full 360° turn (just yaw - no need to
    tip/tilt, the vehicle stays level, so no figure-8 needed, that's for
    handheld 3-axis calibration) while it records min/max x/y from
    `/handsfree/mag`; Ctrl+C prints the offset to plug into
    `mag_offset_x`/`mag_offset_y`, plus a sanity warning if the x/y spread
    ratio suggests the turn wasn't completed or there's soft-iron
    distortion.
  - `imu_drift_test.py` (`ros2 run waypoint_follower imu_drift_test`):
    standalone pure-gyro-integration drift bench test, no GPS correction
    at all - used for the stationary bench numbers above.
- `use_imu`/`use_magnetometer` (both default `true`), `imu_topic`
  (`/handsfree/imu`), `mag_topic` (`/handsfree/mag`) control it all;
  setting either `false` falls back cleanly to whatever's left (pure GPS
  heading if both off) - confirmed the node runs fine with no IMU/
  magnetometer connected at all (subscriptions just never fire, no error).

## 2026-07-26/27 session: steering calibration, corner physics, multi-source arbiter

### Steering was miscalibrated — true max lock is ~14.3°, not 30°

`Steering.c` (AURIX firmware, `STM_Interrupt_1_KIT_TC275_LK` project — lives
on a separate Windows machine, edited in AURIX Development Studio, **not in
this repo**) has `STEER_MAX_ANGLE = 30.0f` hardcoded, and maps that
1:1 across the full `POT_LEFT..POT_CENTER..POT_RIGHT` potentiometer range.
`POT_CENTER` was also off (was `2048`, not the true center).

Diagnosed and fixed in this order:
1. **`POT_CENTER` recalibrated** (`2048` → `~1700`, exact value set by the
   user in `Steering.c` on the Windows side) using the drive-log method:
   drove straight with the old center, found the steady-state P-controller
   correction needed to hold the line (~-6.33° in old units), computed
   `new_center = angleToPot(-6.33)` with the *old* constants, and used that.
   Confirmed working: a follow-up straight-line drive log came back with
   cross-track RMSE **0.105m** (was ~1.07m before recentering).
2. **True max steering angle measured empirically**: drove two laps at full
   lock, logged GPS via `waypoint_recorder_node`, fit a circle to the
   points (Kasa algebraic fit) — radius **2.874m** (residuals ~1cm, two
   laps agreed to within 0.5%). `atan(wheelbase_m / radius) = atan(0.735 /
   2.874) = 14.35°` — i.e. commanding "30" (the firmware's own notion of
   full lock) only produces **14.3° of real wheel angle**, not 30°. This
   was only verified for the LEFT direction (`POT_LEFT` wasn't touched
   during recentering) — **RIGHT has not been separately verified** and
   could differ if the two spans (`POT_LEFT-POT_CENTER` vs
   `POT_CENTER-POT_RIGHT`) ended up asymmetric after recentering.
3. **Attempted to properly swap `POT_LEFT`/`POT_RIGHT` labels** (they're
   backwards vs the real vehicle — confirmed) but doing so broke
   `angleToPot`/`potToAngle`'s branch logic (assumes `LEFT > CENTER >
   RIGHT` numerically) and made the wheel spin immediately on connect.
   **Left as-is**: labels are swapped from reality, but the values work,
   so don't "fix" the labels without also fixing the branching logic in
   the same change.

**Fix applied on the ROS/Linux side** (`can_driver.py`):
```python
FIRMWARE_STEER_MAX_ANGLE_DEG = 30.0
TRUE_STEER_MAX_ANGLE_DEG = 14.3
CAN_STEER_SCALE = FIRMWARE_STEER_MAX_ANGLE_DEG / TRUE_STEER_MAX_ANGLE_DEG  # ≈2.098
```
`waypoint_follower_node`'s `steer_limit_deg` default is now
`TRUE_STEER_MAX_ANGLE_DEG` (14.3, was 30) so all the Stanley/turning-radius
math reasons in **true physical degrees**. Every CAN send goes through
`can_driver.send_control_true_deg(bus, rpm, true_steer_deg, enable,
stop_mode)`, which multiplies by `CAN_STEER_SCALE` right before packing the
frame — this is the *only* place that conversion happens. **Any new code
that talks to this vehicle's steering must use `send_control_true_deg`,
not `send_control` directly with a raw angle**, or it'll silently be
commanding roughly half the angle it thinks it is. Exception: code whose
steer values are already known to be in raw firmware-scale (e.g.
`obstacle_avoidance`'s `avoid_steer_left/right = -30/30`) — those go
through `send_control` unscaled, on purpose, and must **not** be run
through `send_control_true_deg` either (would double-apply/wrongly apply
the correction). See "obstacle_avoidance integration" below.

**Once `Steering.c` is reflashed with the correct `STEER_MAX_ANGLE`**
(14.3, or whatever RIGHT turns out to be if asymmetric), all of this
CAN_STEER_SCALE machinery becomes a no-op and should be deleted rather than
left as dead weight — search for `CAN_STEER_SCALE`/`TRUE_STEER_MAX_ANGLE_DEG`.

**Known-real, not-yet-fixed steering roughness**: `POT_TOLERANCE=30` +
`MIN_PWM_PERCENT=15.0f` in `Steering.c` together cause a visible "bang-bang"
jitter right at the deadband edge (a tiny excess error snaps straight to
15% PWM instead of ramping smoothly). Diagnosis: lower `MIN_PWM_PERCENT`
first (more direct fix for jitter *severity*), only touch `POT_TOLERANCE`
if jitter *frequency* is still a problem after that. Not applied yet.

### GPS speed calculation bug (fixed)

`waypoint_follower_node.on_fix()` computed `dt` from
`self.get_clock().now()` (wall-clock time the ROS callback happened to
run) instead of the GPS message's own `header.stamp`. ROS callback
scheduling jitter (fixes processed back-to-back vs spread out) made `dt`
too small for some samples, inflating `self.speed` — confirmed in a real
log: **34% of samples read >8km/h on a vehicle whose real max is ~8km/h**,
peaking at an impossible 11.5km/h. This directly weakens Stanley's
cross-track correction (`v` is in the denominator of the correction term),
so it wasn't just a cosmetic logging bug. Fixed: `now =
Time.from_msg(msg.header.stamp)`.

### Cornering: physical-feasibility check + full-lock override + S-curve fix

The existing anticipatory-steering blend (see "Curvature-based speed +
anticipatory steering" above) assumes the corner is achievable and just
starts turning earlier — it doesn't ask "can the vehicle even make this
turn." Added a real answer, in `stanley_controller.py`:

- `lookahead_path_curve()` now also returns the arc distance it walked, so
  callers can compute `path_radius_m = arc_dist_m / turn_angle_rad` — the
  recorded path's own local curvature.
- If `path_radius_m < turning_radius_m` (vehicle's real minimum, from the
  now-correct `steer_limit_deg=14.3`), the corner is **physically
  infeasible at any anticipation timing** — no amount of "start turning
  earlier" fixes a turn tighter than the vehicle can make. In that case,
  once within the anticipation lead distance, `stanley_control()` skips
  the proportional law entirely and commands **full steering lock** toward
  the turn — provably the best achievable response, and it naturally
  results in cutting to the inside of the corner rather than fighting an
  unreachable target. Verified in a real drive log: the full-lock branch
  triggered exactly at corner entry (idx 11-14, 18-19), *before*
  cross-track error had grown large (started at 0.02-0.14m) — i.e. it
  commits early rather than reacting late, as designed.
- **S-curve bug found and fixed**: the turn-angle accumulation used to sum
  `abs()` of every segment's heading change regardless of direction, so a
  right-20°-then-left-20° bend read as "40° the same direction" instead of
  netting to ~0 — both overstating sharpness and potentially picking the
  wrong full-lock direction for the *next* bend. Fixed: accumulation now
  stops the instant the turn direction reverses (verified with a unit
  test: a 30°-then-(-40)° synthetic bend now correctly reports 30°, not
  70°).
- `curve_lookahead_m` raised **2.0 → 6.0**: the scan window has to be at
  least as long as how far a corner is physically *spread out* in the
  recorded waypoints, or it undercounts the total turn angle. Checked
  against `new_path.csv`'s actual corner: ~89° spread over ~5.6m (a fairly
  gradual curve, not a sharp point-turn) — 2.0m was nowhere near enough to
  see the whole thing.
- Real-world result after all of the above (`new_path.csv`, full course):
  **finished for the first time** (previous attempts got stuck at the
  final waypoint). Cross-track error still swings to ~1.2-1.3m mid-corner
  but now *recovers* smoothly afterward (down to ~0.2m by the end) instead
  of plateauing at a permanent offset like before recentering.

### RTK mountpoint: stick with `RTK-RTCM32`, not VRS

Tried switching `rtk_bridge.py`'s `MOUNTPOINT` to `VRS-RTCM34` (NGII
caster) to fix slow initial Fixed acquisition — worked for that, but
introduced a new problem: VRS synthesizes corrections around the GGA
position we last sent (`GGA_INTERVAL_SEC=10.0`), so a *moving* rover
outruns its own last-reported position and loses Fixed almost immediately
after getting it. Reverted to `RTK-RTCM32` (single physical base station,
slower initial Fix but doesn't degrade while driving). If VRS is
revisited, lowering `GGA_INTERVAL_SEC` to 2-3s is the thing to try first.

### `control_arbiter` — camera / GPS / idx-event-zone / LiDAR-avoidance priority switch

New node (`waypoint_follower/arbiter_node.py`), because multiple things
now want to drive the same vehicle and **only one process may ever write
CAN control frames** — two independent writers fight each other, discovered
the hard way with the GPS serial port earlier this session and it's the
same class of bug. Everything else just publishes its intent as topics;
`control_arbiter` is the only thing that calls `can_driver.send_control*`.

**Priority (highest first):**
1. **GPS idx event zones** (`event_zones` ROS param, list of
   `"idx_start:idx_end:type"` strings — `idx` is the row number, 0-based
   header-excluded, in whatever `waypoints_file` the GPS node is using).
   `stop` / `gps_priority` / `avoid` are one flat tier (not ranked against
   each other) — they're all just "do something specific at this idx",
   whether that's a forced stop, an intersection GPS should drive through
   itself, or a LiDAR-avoidance zone. Active regardless of whether the
   camera is currently driving.
2. **Camera lane-following** — only if `camera_steer_topic` (default
   `/yolopv2_zed_node/steering_deg`) has both (a) a message within
   `camera_timeout_sec` (dead-man's switch for the node being down
   entirely) **and** (b) frame-count hysteresis: `camera_bad_frames_to_disable`
   (10) consecutive NaN frames drops out, `camera_good_frames_to_enable`
   (3) consecutive good frames re-enters — counted per actual camera frame
   in the subscription callback (camera runs 30-70fps), not per arbiter
   tick, and verified with a unit test covering all four transitions.
3. **GPS waypoint-following fallback** — if the GPS node reports itself
   valid (`gps_control/valid` topic, mirrors `self.pos`/`self.yaw` both
   known).
4. **Safe stop** (`rpm=0, enable=0, stop_mode=1`) if nothing above is valid.

**GPS side** (`waypoint_follower_node.py`): new `publish_can_directly`
param (default `true`, so standalone runs are unaffected) — set `false`
when running under the arbiter, and it publishes `gps_control/steer_deg`,
`gps_control/rpm`, `gps_control/target_idx`, `gps_control/valid` (Float32/
Float32/Int32/Bool) every cycle instead of also writing CAN itself.

**Camera side**: not testable yet (package doesn't exist in this
workspace — see Packages above), but the arbiter is already wired to
`/yolopv2_zed_node/steering_deg` and ready.

**obstacle_avoidance integration** (2026-07-27): `obstacle_avoid_node.py`
got the same treatment — new `write_can_directly` param (default `true`),
set `false` under the arbiter. It keeps reading CAN feedback itself
(0x101/0x102, needed for its own `pass_progress`/`steer_fault_check` —
multiple simultaneous *readers* of a CAN bus is fine, only writers
conflict) but stops *writing*; its existing monitoring topics
(`/avoid/state`, `/avoid/cmd_steer`, `/avoid/cmd_rpm`) become the arbiter's
input instead. The arbiter publishes `/can_bridge/enable` (True only while
GPS idx is inside an `avoid` zone — this matches
`obstacle_avoid_node`'s own arm/disarm topic and its existing behavior of
resetting to `CLEAR` on disarm) and relays its `cmd_steer`/`cmd_rpm`
**unscaled** (raw firmware-scale, e.g. -30/30 — see the CAN_STEER_SCALE
warning above) straight into `can_driver.send_control`. If the avoid node
isn't publishing (not running, or `avoid_timeout_sec`=1.0 exceeded), fails
safe to a stop rather than driving through the zone blind.

**Not yet applied, flagged during review**:
- `obstacle_avoidance`'s `turn_radius_override` param (default `0`, meaning
  it uses a *theoretical* radius from `wheelbase/tan(30°)=1.273m`) should
  be set to the **measured** `2.874m` — it's currently assuming the
  vehicle can turn more than twice as tight as it actually can, which
  understates the steering angle (`alpha`) needed to clear an obstacle.
- `obstacle_avoidance`'s `avoid_rpm=30` is currently *faster* than
  `cruise_rpm=15` — backwards from a safety standpoint (slower during the
  actual avoidance maneuver = more reaction margin, matches the
  reaction-time-consumes-distance point in the cornering section above).
  Discussed unifying both to one slow constant speed throughout the whole
  avoidance state machine, since it only ever runs in a known S-curve test
  course anyway — not applied yet.
- `obstacle_avoidance/README.md`'s documented "negative/left steer doesn't
  work on the real vehicle" issue (dated 2026-07-24, before this session's
  recalibration) — **this session's own drive logs already show real
  negative `current_angle` feedback varying properly** (e.g. `-23.7~13.2`,
  `-30.0~12.9`), so it's very likely already resolved as a side effect of
  the `POT_CENTER` recentering. Not independently re-verified with the
  obstacle_avoidance code specifically.

**New launch file**: `waypoint_follower/launch/integrated_drive.launch.py`
— GPS+RTK + `waypoint_follower_node` (`publish_can_directly:=false`) +
`control_arbiter` + `mpl_viz_node` in one launch. `EVENT_ZONES` list near
the top is the place to configure idx zones (empty by default); the
camera `Node(...)` block is fully commented out with instructions, ready
to uncomment once the `zed_camera` package exists.

### Second vehicle ("yellow car") — steering center calibration requested, not done

Same `POT_CENTER` miscalibration is suspected on a second, different
vehicle. Requested procedure: drive straight with steering commanded to
0° (however that vehicle's low-level control is exercised) while logging
GPS via `waypoint_recorder_node`, then send the resulting CSV — same
circle/curvature-fit method as above will back out how far off center is.
**Blocked on**: that vehicle's `POT_LEFT`/`POT_CENTER`/`POT_RIGHT`/
`wheelbase_m` values (not yet provided) and the actual test drive/log.

## 2026-07-28 session: ZED SDK install, camera integration, launch-crash fix, GPS `num_sv=0` blocker

### NVIDIA driver detour

ZED SDK 5.4.1 requires CUDA ≥13.0, which needed a driver bump. Went to
driver 595, which broke `gdm3` (was on `prime-select on-demand`, display
manager wouldn't start). Recovered by downgrading back to driver **580**,
which turned out to already support CUDA 13.0/13.3 fine — no 595 needed
after all. If this ever needs redoing: don't jump straight to 595, try
whatever the current driver already supports first.

### ZED SDK + camera pipeline install

- ZED SDK 5.4.1 installed, diagnostic tool confirmed the ZED2i is detected
  and CUDA is working.
- `torch`/`torchvision` 2.13.0+cu130 installed for YOLOPv2 inference,
  confirmed running on GPU (was stuck on CPU at first — see fix below).
- `zed-ros2-wrapper` cloned from the official Stereolabs repo and built
  from source (see Packages above).
- **NumPy pinned to 1.26.4 (downgraded from 2.x)**: `cv_bridge` +
  `python3-opencv` need numpy<2, but `pyzed`/`transforms3d` (IMU stack)
  want numpy≥2 — a real conflict, not fixable by picking one version that
  satisfies both. Resolved in favor of numpy<2 per explicit instruction to
  deprioritize IMU work for now ("IMU는 나중에 생각하자 GPS만 잘 되면
  됨") — **this means the IMU/heading-fusion stack (`handsfree_ros2_imu`,
  `transforms3d`) is likely broken until numpy is bumped back**, not
  re-verified.
- `setuptools` regression: installing torch silently bumped `setuptools`
  to 83.0.0, which broke `colcon build` (`error: option --uninstall not
  recognized`). Fixed: `pip3 install "setuptools<80"` (landed on 79.0.1).
  Any future `pip install` of a package that pulls in a newer setuptools
  as a dependency can reintroduce this — if `colcon build` suddenly starts
  failing with an `--uninstall` error, check `pip3 show setuptools` first.

### `zed_camera` package assembly

Source code came from a user-supplied zip (`zed_camera(3).zip` in
`~/Downloads` — two earlier copies, `zed_camera.zip` and `zed_camera
(2).zip`, were corrupted/truncated downloads, confirmed via `unzip -t`
reporting "End-of-central-directory signature not found"; `(3)` was the
valid, most-current one). Assembled into a proper ROS2 package:

- `zed_camera/utils/utils.py` was missing from the zip — fetched verbatim
  from the upstream [CAIC-AD/YOLOPv2](https://github.com/CAIC-AD/YOLOPv2)
  repo (`utils/utils.py`), byte-identical to what should have shipped.
- **`setup.cfg` was missing** — root cause of `ros2 run zed_camera
  yolopv2_zed_rpm_node` reporting "No executable found": without it,
  colcon installs console_scripts to `install/zed_camera/bin/` instead of
  `install/zed_camera/lib/zed_camera/` where `ros2 run` looks. Added the
  standard two-line fix (`[develop] script_dir=...` / `[install]
  install_scripts=...`, both pointing at `$base/lib/zed_camera`) and
  clean-rebuilt.
- **Camera stuck on CPU (~2fps instead of ~70)**: caused by passing `-p
  device:=cuda` on the command line. `select_device()` in the YOLOPv2
  utils sets `CUDA_VISIBLE_DEVICES` to the literal parameter string, so
  `"cuda"` isn't a valid device index and silently hides all GPUs. Fixed
  by omitting the param entirely (default is `"0"`, a real GPU index).
- `weights/yolopv2.pt` moved (not copied) from `~/Downloads/yolopv2.pt`
  into `src/zed_camera/weights/` per explicit request.
- Added an FPS cap (`max_fps` param, default 50) inside `_infer_loop()`:
  times each loop iteration and sleeps out the remainder of
  `1/max_fps` if inference finished early, so the node doesn't spin faster
  than needed even on a fast GPU.
- Single-lane-visible + camera-dropout handling (both were open questions
  discussed before implementation): the `LaneTracker` approach in
  `yolopv2_zed_rpm_node` already handles one lane being invisible (learns
  each lane's half-width via EMA and infers the missing side from the
  visible one + that width, rather than requiring both lanes every frame).
  Camera-dropout is handled one layer up, in `control_arbiter`'s existing
  frame-count hysteresis (10 consecutive bad frames → falls back off
  camera; at ~50fps that's ~0.2s, closer to instant than the ~0.5s
  originally asked about — left as-is since faster fallback is strictly
  safer, not slower).

### `integrated_drive.launch.py` — full stack in one launch

Final structure (see the file itself,
[integrated_drive.launch.py](src/waypoint_follower/launch/integrated_drive.launch.py),
for the authoritative version): ZED wrapper starts immediately, GPS+RTK
(`f9p_bringup`) is wrapped in `TimerAction(period=20.0)` so it starts
after, `waypoint_follower_node` runs with `publish_can_directly:=false`,
`camera_node` runs `yolopv2_zed_rpm_node` with `can_enable:=false`, and
`control_arbiter` + `mpl_viz_node` tie it together — mirrors the
2026-07-26/27 arbiter design, just with a real camera node instead of a
commented-out placeholder now.

**Bug found and fixed: `ParameterValue` needed for typed launch args.**
`LaunchConfiguration(...)` always resolves to a string. Passed directly
into a parameters dict, a launch arg meant for a DOUBLE or BOOL parameter
arrives as a STRING and rclpy rejects it at startup
(`InvalidParameterTypeException`) — the node dies within 1s of launch.
Hit this for `camera_mode_rpm` (float, `arbiter_node.py`) — reproduced
directly with a hand-crafted `--params-file` to confirm the exact
exception before fixing. While fixing it, found the identical latent bug
for `enable_control` (bool, `waypoint_follower_node.py`) — it happened
not to have crashed yet in the one real run observed, but the underlying
STRING-vs-BOOL mismatch was there regardless, so fixed it proactively
rather than waiting for it to actually crash. Both now wrapped:
```python
"camera_mode_rpm": ParameterValue(LaunchConfiguration("camera_mode_rpm"), value_type=float),
"enable_control": ParameterValue(LaunchConfiguration("enable_control"), value_type=bool),
```
Verified via an 8s real launch test: no `died`/`InvalidParameterType`/
`Traceback` in the log, `arbiter_node` logged `[MODE CHANGE] None ->
safe_stop` then `[MODE CHANGE] safe_stop -> camera` as expected.
**Any new `LaunchConfiguration` feeding a non-string ROS parameter needs
the same `ParameterValue(..., value_type=...)` wrapping** — this class of
bug is easy to reintroduce.

### `status_check.sh` — one-shot status snapshot

New script,
[scripts/status_check.sh](src/waypoint_follower/scripts/status_check.sh):
prints RTK fix status (`/navpvt` flags), GPS waypoint-follower validity/
target idx, camera lane validity/steering angle, obstacle-avoidance state,
and the arbiter's `active_source` — everything that used to require 5+
terminals, in one `ros2 topic echo --once` sweep. Filters `ros2 topic
echo`'s own "WARNING: topic [...] does not appear to be published yet"
(goes to stdout, not stderr) into a clean `(no data)`. Also added to
`control_arbiter`: an `active_source` topic (String, published every
cycle) and un-throttled `[MODE CHANGE] {old} -> {new} | {detail}` logging
whenever the active source actually changes (throttled 1/sec heartbeat
otherwise) — this is what made the launch-crash verification above
possible to confirm quickly.

### GPS `num_sv=0` — found, not yet solved

While testing the integrated launch, GPS wasn't getting a fix. Initial
hypothesis: same class of bug as the earlier GPS/RTK-bridge simultaneous-
USB-start issue, this time GPS vs. the ZED's own USB
enumeration/CUDA init — "fixed" by staggering the launch (`f9p_bringup`
delayed 20s after ZED, per explicit request to swap the order from an
earlier ZED-delayed attempt; this launch-file change is still in place).

**This hypothesis was then disproven**: testing GPS completely standalone
(no camera, no launch file, just `ublox_gps_node` + `rtk_bridge.py` run
directly) still showed `num_sv=0` outdoors with clear sky. Confirmed not
a software/USB-contention issue this time:
- USB device path/by-id symlink: fine.
- `ublox_gps_node` and `rtk_bridge.py`: both running, `/navpvt` and `/fix`
  actually publishing at 20Hz (not frozen/hung).
- RTCM corrections: flowing from the NTRIP caster at ~7Hz (reaching the
  receiver).
- Antenna cable: user confirmed not loose.
- Still: `num_sv=0` throughout.

**Left unresolved** — user stepped away to charge the laptop before a
suggested power-cycle of the receiver could be tried. Next step when
resuming: power-cycle the u-blox receiver first (cheapest thing to rule
out); if that doesn't help, suspect the antenna/connector itself (despite
"nothing loose", a marginal connection wouldn't necessarily present as
obviously loose) or a receiver-internal glitch. The `integrated_drive.launch.py`
GPS-delay-20s change should probably stay (harmless, and was a reasonable
fix for a different real bug earlier this session) but is **not** the fix
for this problem.

**Theory floated, then ruled out same day**: reverse current / a short on
the antenna bias line (the receiver supplies 3-5V to power an active
antenna over the same SMA coax) could have fried the antenna's LNA or the
receiver's antenna-power output stage - would've explained the symptom
pattern seen (comms healthy, only RF/satellite-tracking dead) and lined
up with the receiver having been dropped at some point. Checkable via
u-blox's self-reported `UBX-MON-HW` message (`a_status`/`a_power`) -
enabled explicitly in `f9p_bringup/config/ublox_rover.yaml`
(`publish.mon.hw: true`) and added to `status_check.sh`. **First attempt
used the wrong topic name** (`/ublox_gps_node/monhw` - ROS2 doesn't
auto-prefix topics with the node name, only the namespace; fixed to the
correct `/monhw`, matching how `/navpvt` etc. are already unprefixed).

**Result once fixed**: `a_status=1` (UNKNOWN), `a_power=2` (UNKNOWN) -
*while simultaneously getting a real Float fix* (`flags=67`). This
combination actually rules the reverse-current/damage theory back out:
`UNKNOWN` here means the Antenna Supervisor Monitor feature itself isn't
enabled (`UBX-CFG-ANT` was never configured - common on boards without
antenna-detect circuitry, or passive-antenna setups), not that anything
is damaged - and if the RF front-end were actually fried, Float
(which requires real carrier-phase signal reception) would be physically
impossible to reach at all, damaged or not. **So as of this test, the
antenna/RF path is confirmed healthy** - whatever caused yesterday's
`num_sv=0` was very likely transient (cold-start TTFF, sky view at that
moment, or a receiver-internal glitch), not permanent hardware damage.
Left as-is (not chasing `UBX-CFG-ANT` enablement further - no evidence it's needed).

**USB3-EMI concern (ZED interfering with GPS) - CONFIRMED, 2026-07-29**:
the ZED2i is a USB3 SuperSpeed camera and a known real source of 1-6GHz
electromagnetic interference overlapping GPS L1 (1575.42MHz), flagged as
a real risk for the *combined* launch even once GPS worked standalone.
Initial same-day (2026-07-28) test seemed to clear it - GPS reached Float
in the combined launch, no worse than standalone - but that test only
checked "does GPS get *a* fix", not whether it reaches the same quality.
**Follow-up test (2026-07-29) found the real effect**: at a spot where
GPS standalone reliably reaches Fixed (`flags=131`), the combined
GPS+ZED launch got stuck at Float (`flags=67`) and would not progress to
Fixed - i.e. EMI wasn't blocking a fix outright, just degrading SNR
enough to prevent carrier-phase ambiguity resolution (Fixed) while Float
(more tolerant of noise) still worked fine. **Fix confirmed**: physically
routing the GPS antenna cable and the ZED's USB cable further apart
resolved it - Fixed acquired normally once separated. **Action item**:
keep these two cables physically separated as far as practical in the
final vehicle wiring/harness layout, not just during testing.

**Closing summary on `num_sv=0` (2026-07-28)**: root cause was never
directly captured (no live diagnostic was running at the moment it
happened), but circumstantial evidence points to **an incidental power
cycle** as the actual fix - the last suggestion before the user stepped
away to charge their laptop was "try power-cycling the receiver," and
disconnecting/reconnecting the GPS's USB cable around that break would
have done exactly that. Everything re-tested clean afterward: standalone
Float (near-instant outdoors), antenna/RF hardware confirmed healthy via
`UBX-MON-HW`, and Float again in the combined GPS+ZED launch. Treated as
resolved. **If `num_sv=0` recurs**: unplug/replug the GPS's USB
connection first (cheap, matches what circumstantially worked here)
before re-investigating antenna hardware or EMI.

### 2026-07-29 follow-up: two real bugs found and fixed during live testing

- **`mpl_viz` showing the vehicle/waypoints ~14,000km apart**: root cause
  in `waypoint_follower_node.on_fix()` - `self.frame` (the local ENU
  origin) was anchored on the *first* `NavSatFix` message received,
  regardless of whether it had a real fix. `ublox_gps_node` publishes
  `NavSatFix` continuously even before any satellites are locked, with
  `lat=lon=0.0` (Null Island) and `status.status=STATUS_NO_FIX`. If that
  first message arrived before GPS had a fix (common - GPS needs time
  after node start), the origin got permanently locked at (0,0), and
  since `self.frame` is only ever set once, every waypoint plotted
  relative to it landed ~14,000km away for the rest of the run - while
  the vehicle marker looked "fine" only because its own position was
  *also* still (0,0) at that moment (coincidence, not correctness).
  **Fix**: `on_fix()` now returns early (doesn't touch `self.frame`) if
  `msg.status.status < NavSatStatus.STATUS_FIX`, so the origin only ever
  gets set from a message with a real fix. Verified this accepts both
  Float and Fixed (ublox's `ublox_firmware7plus.hpp` maps Float RTK to
  `STATUS_FIX` and Fixed RTK to `STATUS_GBAS_FIX`, both `>= STATUS_FIX` -
  only genuine `STATUS_NO_FIX` is rejected), so the plot starts as soon
  as Float is acquired, not just Fixed.
- **`control_arbiter` never actually dropped out of camera mode on
  `LaneLost`**: the frame-count hysteresis in `arbiter_node.py` checked
  `math.isnan()` on the camera's `steering_deg` to decide "is the lane
  lost", but `yolopv2_zed_rpm_node`'s `_publish_timer_cb()` always
  substitutes `0.0` for `NaN` before publishing (so downstream doesn't
  see NaN on the wire) - meaning the arbiter's NaN check could never
  trip, `camera_bad_streak` never incremented, and it kept trusting +
  driving on the camera (steer=0.0, i.e. straight) indefinitely even with
  a confirmed dead lane. Caught live: user held the camera by hand,
  watched `/yolopv2_zed_node/lane_valid` (a separate, correctly-reported
  Bool topic) go `False`, but `active_source` stayed `camera`. **Fix**:
  arbiter now subscribes to `camera_lane_valid_topic` (new param, default
  `/yolopv2_zed_node/lane_valid`) directly and drives the
  good/bad-streak hysteresis off that Bool instead of inferring it from
  the steer value; `_on_camera_steer` now only updates the freshness
  dead-man's-switch timestamp. Verified after the fix by the same
  hand-held test - mode switch away from camera now happens correctly.
- **USB3-EMI (ZED vs. GPS) confirmed real** - see "GPS `num_sv=0`"
  section above for the full writeup: cable separation fixed a
  Float-stuck-can't-reach-Fixed symptom that the 2026-07-28 A/B test had
  missed (it only checked "gets *a* fix", not fix quality).

### `traffic_light` package — OAK-D + YOLO red-light detection, integrated with `control_arbiter` (code-only, not hardware-tested)

User had a separate standalone script already written and downloaded
(`~/Downloads/test__sunny/test_sunny.py` + `test_sunny.pt` weights +
a usage doc) — OAK-D camera (via `depthai`, not the ZED) + `ultralytics`
YOLO, detects red lights, debounces over a 30-frame buffer (STOP if ≥25 of
the last 30 frames saw high-confidence red), was going to be run with
`cv2.imshow` + CSV logging in a blocking `while True` loop, no ROS
involved at all.

**No OAK-D was connected this session** — this was explicitly a
code-integration-only pass (`depthai`/`ultralytics` aren't installed
either), not a hardware test. Ported into a new package:

- **New package `traffic_light`** (`src/traffic_light/`), not folded into
  `zed_camera` — different camera hardware (OAK-D vs ZED2i) and a
  different pip dependency set (`depthai`, `ultralytics` vs `pyzed`/
  `torch`), kept separate on purpose.
- `traffic_light_node.py` (executable `test_sunny_node`, matching the
  original doc's naming): same buffer/threshold logic as the original
  script, but runs the depthai frame-grab + YOLO inference loop in a
  background thread (mirrors `yolopv2_zed_rpm_node`'s `_infer_loop`
  pattern) instead of blocking the ROS spin thread, and publishes
  `/traffic_light` (`std_msgs/String`, `"GO"`/`"STOP"`) every frame
  instead of just drawing to a `cv2.imshow` window. `cv2.imshow` is still
  there but gated behind a `show_debug` param (default `false`) so it
  runs headless by default. CSV logging (`log_csv` param, default `true`)
  is unchanged from the original. Model weights copied into
  `src/traffic_light/weights/test_sunny.pt`.
- **Fails soft, not hard, when hardware/deps are missing**: if
  `ultralytics` or `depthai` aren't importable, or `dai.Device(...)`
  can't find an OAK-D, the node logs a warning and stays idle (never
  publishes) instead of crashing — same reasoning as `can_driver`'s
  "CAN bus not available, log-only" fallback in `arbiter_node.py`. This
  is why it was safe to wire straight into `integrated_drive.launch.py`
  even with no camera plugged in and the pip deps not installed yet.
  **Deps now installed** (2026-07-28, still no OAK-D physically
  connected): `depthai==2.32.0.0`, `ultralytics==8.4.110`. Two gotchas hit
  while installing, both fixed:
  - `pip3 install depthai ultralytics` installs **depthai 3.x by
    default**, which is a breaking API change from the v2 API the
    original script (and this node) is written against —
    `dai.Pipeline()` in v3 defaults to `createImplicitDevice=True` and
    tries to connect to a device *at construction time*, so even
    building the pipeline description (no device needed in v2) throws
    `RuntimeError: No available devices` immediately. Fixed: pinned
    `pip3 install "depthai<3"` (landed on 2.32.0.0) to match the v2
    `dai.Pipeline()` → `dai.Device(pipeline)` two-step flow this code
    uses. **If depthai ever gets reinstalled/upgraded, re-pin to `<3`**
    or the v2-style pipeline code needs a rewrite for the v3 API.
  - `ultralytics` pulls in `opencv-python` (its own pip-installed cv2,
    currently a totally different major version) and bumps `numpy` back
    to 2.x as transitive dependencies - both **directly conflict** with
    the numpy<2/apt-cv2 pin `cv_bridge` needs (see "ZED SDK + camera
    pipeline install" above). Fixed by, after installing
    depthai/ultralytics: `pip3 uninstall -y opencv-python` (so `import
    cv2` falls back to apt's `python3-opencv` at
    `/usr/lib/python3/dist-packages/cv2...so`, version 4.5.4) and
    `pip3 install "numpy<2"` again to re-pin back to 1.26.4. Verified
    afterward that `cv_bridge`, `pyzed`, and `torch`+CUDA all still
    import fine with this combination - **this exact sequence
    (depthai/ultralytics install → strip pip's opencv-python → re-pin
    numpy<2) needs to be repeated any time these get reinstalled.**
  - Confirmed working end-to-end without hardware: pipeline builds,
    `dai.Device(pipeline)` fails cleanly with `RuntimeError: Cannot find
    any device with given deviceInfo` after ~5.6s (device search
    timeout) when nothing's plugged in, caught by
    `traffic_light_node.py`'s try/except, logs the warning and stays
    idle - no crash. Model (`test_sunny.pt`) loads fine via `ultralytics
    YOLO(...)`, one class (`{0: 'red'}`), matching the `label.lower() ==
    "red"` check in the code.

  **Still needed before a real test**: connect the OAK-D, confirm with
  `python3 -c "import depthai as dai;
  print(dai.Device.getAllAvailableDevices())"` (should show a device
  instead of `[]`).

**`control_arbiter` got a new event-zone type, `"traffic_light"`**
(`arbiter_node.py`), designed around exactly what was asked for: not a
simple "stop the instant idx is in range" zone like the existing `stop`
type, but idx-range-aware behavior —
- Zone spec is still `"idx_start:idx_end:traffic_light"` (same 3-field
  format as the other zone types), but `idx_end` is reused as the **stop
  line** idx specifically for this type.
- `idx_start <= gps_idx < idx_end` (**approaching**): if
  `/traffic_light` says `STOP`, speed is scaled down
  (`traffic_light_approach_rpm_scale` param, default `0.5`×) but steering
  is untouched — camera or GPS (whichever `control_arbiter` would
  otherwise be driving with) keeps steering normally, only the speed is
  modified.
- `gps_idx >= idx_end` (**at/past the stop line**): if still `STOP`, full
  stop (`rpm=0`, `enable=0`, `stop_mode=1`); if `GO`, resume full normal
  speed — matches "그 특정 idx에서 빨간불이 아니면 다시 원래 속도로"
  exactly (no different than driving through the zone normally once the
  light is green).
- This is architecturally different from the other event-zone types:
  `stop`/`gps_priority`/`avoid` fully dictate their own steer/rpm, but
  `traffic_light` needs to know what camera/GPS *would have* commanded
  first (to keep steering working through the zone) — added a
  `base_steer`/`base_rpm`/`base_source` computation at the top of
  `on_timer()` (camera_ok → camera, elif gps_ok → gps_fallback, else
  `None`) that `traffic_light` reads from, and the old plain
  camera/GPS-fallback branches at the bottom were simplified to reuse it
  too.
- **Fails safe to RED**, not GO, if `traffic_light_node` isn't
  publishing (`traffic_light_timeout_sec`, default 1.0s, dead-man's
  switch) — deliberate: an unconfigured or crashed traffic-light node
  means the vehicle crawls and stops at every configured zone rather than
  silently driving through possibly-red intersections at full speed.
  **Flagging this explicitly**: this makes any `traffic_light` zone
  unusable for testing without the node actually running and connected
  (it'll just permanently slow-then-stop) — intentional for safety, but
  worth knowing if it looks like the zone is "stuck."
- Verified with `ros2 topic pub` (no real camera/hardware involved, just
  exercising the state machine): confirmed all three transitions log
  correctly —
  `RED ahead - slowing (gps_fallback) steer=2.0deg rpm=40` (half of the
  fake 80 rpm), `RED - stopped ... rpm=0 enable=0 stop_mode=1` at the
  stop-line idx, and `GO (gps_fallback) ... rpm=80` resuming full speed —
  and that it fails through the priority chain correctly to
  `traffic_light_no_driver` → safe stop when GPS validity also expires
  mid-zone.
- Wired into `integrated_drive.launch.py`: new `traffic_light_node` +
  `traffic_light_model_arg` (default
  `src/traffic_light/weights/test_sunny.pt`), added to the
  `LaunchDescription`. `EVENT_ZONES` stays empty by default — add an
  entry like `"90:95:traffic_light"` once there's a real intersection idx
  to test against (and the OAK-D is actually connected).

### `control_arbiter` — GPS cross-track veto on the camera (2026-07-29)

Motivating problem, raised during real-car testing: the camera/GPS
priority switch only ever asked "does the camera *think* it sees a lane"
(`lane_valid`) - it had no way to catch the camera being *confidently
wrong* (mis-detecting something as a lane and driving off the recorded
line with high self-reported confidence). `lane_valid=True` just means
"found lane-shaped pixels," not "this is the correct lane."

**Fix**: `waypoint_follower_node` now also publishes
`gps_control/cross_track_error_m` (previously computed every cycle for
the drive-log CSV, but never put on a topic). `control_arbiter` watches
this continuously - even while the camera is actively driving - and
force-switches to GPS if the actual vehicle position drifts more than
`camera_max_deviation_m` (default 1.0m radius) from the recorded
waypoint line, regardless of what the camera itself claims.

- **Hysteresis, not a single threshold** (`_camera_deviation_ok()` in
  `arbiter_node.py`): re-trusting the camera requires shrinking back
  inside `camera_deviation_reenter_m` (default 0.5m - tighter than the
  1.0m enter threshold) and holding it there for
  `camera_deviation_reenter_streak` consecutive
  cycles (default 20 @ 20Hz = ~1s) - without this it would flap right at
  the boundary the same way the old NaN-based camera check could have.
- **Repeat-offense lockout**: if the veto trips
  `camera_deviation_lockout_count` times (default 3) within
  `camera_deviation_lockout_window_sec` (default 20s), the camera is
  locked out entirely for `camera_deviation_lockout_sec` (default 15s) -
  a camera whose underlying error is persistent would otherwise just get
  pulled back in bounds by GPS and immediately drift out again every
  time control is handed back, at a slower but still-repeating cadence.
  The lockout stops that sawtooth pattern instead of only closing the
  single-cycle flap.
- **Doesn't over-trust GPS either** (explicitly raised as a concern
  during design - GPS itself is only Float, not Fixed, in some stretches,
  so its own position has real error too): if `cross_track_error_m`
  hasn't been published recently (`cross_track_timeout_sec`, default
  1.0s) or is NaN, `_camera_deviation_ok()` does NOT clear an existing
  override on missing information, but also doesn't trip a *new* one -
  stays conservative rather than judging off stale/absent data either
  way. Considered (not implemented, flagged for later if needed): scaling
  the thresholds by GPS's own reported accuracy (`h_acc` from
  `NavSatFix`'s `position_covariance`, or the Fixed/Float flag) rather
  than fixed constants - decided against for now to keep this simple;
  revisit if false-veto/false-trust in Float stretches turns out to be a
  real problem in practice.
- Verified with `ros2 topic pub` fakes (no real camera/GPS involved):
  confirmed `gps_fallback -> camera` on good lane data, then
  `camera -> gps_fallback` the instant a faked 1.5m cross-track error was
  published (>1.0m enter threshold) - the mode-change log line fired
  immediately as expected.

## 2026-07-30 ~ 08-03 session: obstacle-avoidance live tuning, GPS/camera hardening, lane-detection redesign, curvature fitting

Large real-vehicle-testing session, spans several days. Grouped by area
rather than chronologically.

### `obstacle_avoidance` — final tuned values (live-tuned on the real HENES vehicle)

Current [config](src/obstacle_avoidance/config/obstacle_avoid.yaml):
`front_angle_deg=50.0`, `max_consider_range=2.5`, `vehicle_width=0.8`,
`turn_radius_override=2.874` (precise circle-fit measurement),
`safety_margin=0.0`, `reaction_margin=0.0`, `avoid_rpm=80`, `cruise_rpm=30`,
`alpha_extra_deg=0.0`, `obstacle_length=1.43`, `pass_margin=0.0`,
`gps_side_bias_pts=5`.

- **`write_can_directly: false` added** — was missing entirely, defaulting
  to a dangerous `true` (this node writing CAN directly *and* the arbiter
  also writing CAN would fight each other). Only set `true` when running
  `obstacle_avoidance` standalone without the arbiter in the loop.
- **`vehicle_width=0.0` bug (found and fixed)**: an earlier "set everything
  to 0" experiment left this at 0, which makes `_decide()`'s clearance
  check always pass (`C <= 0`), so the state machine never leaves CLEAR —
  avoidance silently never triggers even with an obstacle dead ahead.
  Restored to the real 0.8m.
- **`obstacle_length=0.0` bug (found and fixed)**: `_solve_avoid_alpha`'s
  `clearance(alpha) = R*(1-cosα) + D*sinα` uses `D = obstacle_length +
  pass_margin` as the real-world obstacle footprint the turn has to clear.
  With `obstacle_length` left at 0, increasing `pass_margin` alone
  counterintuitively computed a *weaker* turn (banks on late straight-line
  drift during PASS that doesn't help at the moment of actually passing the
  obstacle). Restored to the real 1.43m.
- **GPS-side tie-breaker for AVOID_LEFT vs AVOID_RIGHT** (`gps_side_bias_pts`,
  `gps_cross_track_topic`, `gps_bias_timeout_sec` in
  [obstacle_avoid_node.py](src/obstacle_avoidance/obstacle_avoidance/obstacle_avoid_node.py)):
  adds `gps_side_bias_pts` phantom LiDAR points to whichever side the GPS
  cross-track error says is "back toward the recorded line", only as a
  tie-breaker when the raw LiDAR left/right point counts are close — never
  overrides a clear LiDAR signal.
- RPLiDAR S2 brought up via `rplidar_ros` (apt) — a wrapper launch file,
  [rplidar_s2.launch.py](src/obstacle_avoidance/launch/rplidar_s2.launch.py),
  pins `serial_port` to the by-id path since the default `/dev/ttyUSB0` is
  actually the AURIX steering board on this vehicle (LiDAR is
  `/dev/ttyUSB1`).

### GPS/waypoint follower hardening

- **`stanley_v_min` (default 0.5)**: floors the speed used in Stanley's
  cross-track term (`atan2(k*cte, max(v, v_min))`), fixing a bug where a
  stationary vehicle even 1cm off the line would compute full steering
  lock for *any* nonzero error (denominator near 0).
- **GPS outlier-rejection streak-accept** (`gps_outlier_streak_accept`,
  default 3): a single GPS jump bigger than `gps_outlier_threshold_m` used
  to freeze the filtered position forever — every real position after a
  genuine jump (mode-switch handoff, GPS catching back up) was *also* far
  from the stale frozen point, so it kept getting rejected too, a loop the
  filter could never escape. Now accepts the jump as real after this many
  consecutive rejections.
- **`stanley_k` boost after obstacle avoidance** (`stanley_k_boost=1.0`,
  `stanley_k_boost_duration_sec=2.0`): temporarily raises the cross-track
  gain right after `obstacle_avoid_node`'s RETURN state ends, to snap back
  onto the recorded line faster instead of a slow cruise-tuned crawl-in.
- **`loop_waypoints` param** (default `false`): for closed-loop courses
  (`halla1_closed.csv`) — skips the arrival-stop check at the last
  waypoint so the vehicle keeps driving. `stanley_control()` already
  re-finds the globally nearest waypoint every cycle (not a monotonic
  index), so idx naturally wraps back toward 0 with no other change
  needed.
- **`halla1_closed.csv`**: the real recorded lap (`halla1.csv`, 2440pts)
  had a ~55m gap where the loop's start/end didn't connect. Stitched a
  user-recorded real segment (`halla1_gap.csv`) across the gap instead of
  a straight-line guess — verified the seam distances are small (1.96m /
  0.41m) and there are no abnormal jumps elsewhere in the merged path.
- **`gps_priority` event zones for a confirmed camera-drift stretch**: a
  post-run cross-track analysis (comparing the recorded course against a
  drive log) found idx 1427-1465 and 1535-1557 on `halla1_closed.csv` are
  where the camera confidently follows a lane that leads somewhere other
  than the recorded line, pulling cross-track error out to 5.25m at the
  worst point — not a LOST/no-lane situation, the camera was actively
  driving on a real but wrong lane. `post_gps_drive.launch.py`'s
  `EVENT_ZONES` now forces GPS-only through `1440:1464` and `1520:1560` to
  cover it.

### `zed_camera` — lane-detection false-positive rejection + curvature fitting

[yolopv2_zed_rpm_node.py](src/zed_camera/zed_camera/yolopv2_zed_rpm_node.py)'s
`LaneTracker.update()` was redesigned to stop mistaking curbs, crosswalk
stripes, and speed-bump lines for real lane edges:
1. Per-connected-component candidates (not pre-split left/right pools).
2. Adjacent-candidate merge when the gap is smaller than a plausible lane
   width — keeps whichever candidate is closer to image center (curbs sit
   just outside the real lane, closer to the edge of frame).
3. Width-validated pair search among the cleaned candidates (closest to
   the EMA-learned half-lane-width wins).
4. Single-side fallback only when exactly one candidate remains on that
   side; otherwise LOST rather than guessing.
5. `lane_max_candidates` (default 4) — if this many candidates remain
   within plausible lane range, treat it as clutter (crosswalk/speed bump)
   and go straight to LOST instead of trying to pick one.

**Speed-based steering damping** (`speed_damp_gain=0.15`,
`speed_damp_min_scale=0.4`): `self.lane.max_steer_deg` is rescaled every
frame from a fixed base value using current speed (GPS speed preferred
over ZED VIO, falls back if GPS speed goes stale) — mirrors GPS's Stanley
v-denominator damping, fixes the camera snapping hard toward the lane and
overshooting at higher rpm.

**2nd-order curve fitting / curvature anticipation** (`lane_num_bands=4`,
`lane_min_fit_bands=3`, `lane_curvature_gain_deg=8.0`,
`lane_curvature_max_deg=10.0`): the single-band candidate-detection logic
above was factored into `_scan_region()` so it can run per horizontal band
of the ROI. `_compute_curvature_term()` scans `lane_num_bands` bands,
collects (y, lane_center_x) points, fits a degree-2 (or degree-1 with only
2 points) polynomial, and turns the near-field slope into a small
anticipatory steering term added on top of the normal position term —
aimed at the "GPS hands off to camera at a sharp angle, camera turns hard
and loses the lane" failure mode, by leaning on the trend across bands
instead of reacting to a single noisy near-field point. Falls back to 0
(identical to the old single-term behavior) when too few bands have a
valid point.

### Hardware bring-up

- **RPLiDAR S2** via `rplidar_ros` (apt) — see obstacle_avoidance section
  above for the port mixup.
- **Taobotics/HandsFree IMU** (`mrpt_sensor_imu_taobotics`) brought up,
  publishing on `/taobotics/sensor` (consumed by `obstacle_avoid_node`'s
  AVOID/RETURN yaw-progress judgment).
- **OAK-D udev permissions**: `dai.Device.getAllAvailableDevices()`
  returned empty despite `lsusb` showing the device (`03e7:2485`) —
  fixed with a manual udev rule,
  `/etc/udev/rules.d/80-movidius.rules`:
  `SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"`.

### First git backup

Entire `src/` was untracked until this session — committed for the first
time (`a155558`). Added `__pycache__`/`*.pyc` to `.gitignore`.
`fma`/`handsfree_ros2_imu`/`ydlidar_ros2_driver`/`zed-ros2-wrapper` are
committed as gitlinks (external clones, not original work — expected).
Note `rtk_bridge.py` contains real plaintext NTRIP credentials, committed
as-is per explicit instruction.

## 2026-08-05: parking integration (parallel_parking / t_parking) — code-only, NOT hardware-tested

Two pre-built rule-based one-shot parking packages (`parallel_parking` -
right-side parallel, `t_parking` - left-side T/perpendicular reverse),
already calibrated for HENES's real vehicle specs (L=1.410, W=0.800,
WB=0.735 - matches `obstacle_avoid.yaml` exactly), integrated into the
arbiter/CAN-ownership architecture instead of run standalone.

**What changed vs. how they arrived** (both wanted their own LiDAR driver
(`sllidar_ros2`), their own IMU node, and a `my_first_pkg` package that
doesn't exist in this workspace, plus a CAN bridge that writes CAN
directly - all three conflict with "control_arbiter is the only CAN
writer" and "don't spin up duplicate sensor nodes"):
- **New package `parking_bridge`** (`src/parking_bridge/`) -
  `wheel_odom_pcan_node`: CAN-*feedback-only* (encoder 0x102 + steering
  0x101), fuses with the already-running taobotics IMU (`/taobotics/sensor`)
  for yaw, publishes `nav_msgs/Odometry` on `/wheel_odom` (what both
  parking nodes expect). `enable_command_tx` defaults `false` - never
  writes CAN itself. Reuses `waypoint_follower.can_driver` for the actual
  CAN frame layout (same TX 0x200/RX 0x101/0x102 already validated
  elsewhere in this codebase) instead of a separate reimplementation, and
  `t_parking.geometry`'s angle/quaternion helpers instead of duplicating
  those either - only `blend_angle` (circular yaw fusion) and
  `wrapped_delta` (encoder-wraparound-safe delta) needed writing.
  `encoder_meter_per_count` defaults to `obstacle_avoid.yaml`'s own
  measured wheel_diameter/encoder_counts_per_rev, not the placeholder the
  original script shipped with.
- **No duplicate LiDAR/IMU**: `t_parking`'s `scan_parking_filter` node
  already just remaps `/scan` -> `/scan_parking` with a vehicle-frame angle
  window, and its `input_scan` default is already `/scan` - the existing
  RPLiDAR S2 feed obstacle_avoidance already uses. Both parking nodes'
  `imu_topic` is pointed at `/taobotics/sensor` in the new launch files.
  Neither `sllidar_ros2` nor `my_first_pkg` need to exist.
- **Both packages hardcode absolute topic names in source**
  (`/parking_start`, `/parking/cmd_rpm`, `/parking_active`, ...) - fine for
  one at a time, but a course with *both* a T-zone and a parallel-zone
  needs both parking nodes running simultaneously for the whole drive
  (only one actually triggered depending on which zone the vehicle is in
  at a given moment - see "why not launch on demand" below). Running two
  instances with the same absolute names would collide, so each launch
  file remaps its parking node's (and its `scan_parking_filter`'s
  `output_scan`) topics under its own prefix via `remappings=` -
  `parking_t_left.launch.py` -> `/parking_t/...`,
  `parking_parallel_right.launch.py` -> `/parking_r/...` - without editing
  either package's source.
- **`control_arbiter` gets two new event-zone types, `"parking_left"`
  (t_parking) and `"parking_right"` (parallel_parking)** (`arbiter_node.py`,
  `_handle_parking_zone()`): both packages' `direct_cmd_output` stays
  `false` (their `/parking/cmd_rpm` etc. already exist for exactly
  "something else relays this to CAN") - no source changes needed in
  either parking package itself. The arbiter tracks each side's state
  independently (`self.parking["left"]`/`self.parking["right"]`, reading
  from that side's remapped topics). On entering a zone (`event_zones`
  entry `"start:end:parking_left"` or `"...:parking_right"`), the arbiter
  edge-triggers that side's `parking_start=true` once. **While that side
  reports `mapping=true`** (its own APPROACH state - still scanning for/
  locking onto a slot), **GPS drives straight through the zone** instead of
  trusting the parking node's own approach controller (same posture as the
  `"avoid"` zone's CLEAR-state GPS driving - the parking node keeps
  observing `/scan_parking`+`/wheel_odom` and planning regardless of who's
  actually driving, only actuation changes; rpm during this phase is that
  side's `parking_{left,right}_approach_rpm` param - each package's own
  `pre_straight_rpm`, t_parking=30/parallel_parking=20 by default - not
  GPS's normal cruise rpm, since slot detection was calibrated at that
  speed). Once it locks a slot (`mapping=false`) and is actively
  maneuvering (`active=true`), control switches to relaying that side's
  `cmd_rpm`/`cmd_steer`/`cmd_enable` straight to CAN (raw firmware-scale,
  like `obstacle_avoid`'s steer - NOT through `send_control_true_deg`) as
  long as commands stay fresh; fails safe (stop) if that side's parking
  node isn't publishing either signal. `done=true` hands control back to
  camera/GPS - gated on that flag specifically, not on GPS idx leaving the
  zone range, since the vehicle physically leaves the recorded line to
  pull into/out of the slot during the maneuver.
- **New launch files**:
  [parking_t_left.launch.py](src/waypoint_follower/launch/parking_t_left.launch.py),
  [parking_parallel_right.launch.py](src/waypoint_follower/launch/parking_parallel_right.launch.py) -
  each starts only `scan_parking_filter` + `parking_bridge`'s odom node +
  the parking node itself (each with its own `remappings=`), assuming the
  rest of the stack (`post_gps_drive.launch.py`: RPLiDAR S2, taobotics IMU,
  CAN, arbiter) is already running. Both are meant to run *together* for a
  course that has both zone types - launching a parking node on-demand
  right as the vehicle enters its zone was considered and rejected: it
  takes a few seconds to initialize (the existing `TimerAction(period=2.0)`
  delay before the parking node subscribes/starts is already accounting
  for this), so an on-demand launch risks not being ready the instant the
  zone is entered, and dynamic process start/stop is its own extra failure
  mode (a stop that doesn't land in time re-creates the exact topic
  collision this remapping was meant to avoid). Both nodes idling
  unpublished/untriggered costs essentially nothing, so pre-starting both
  is the simpler and safer choice.

**NOT hardware-tested yet** - this was a code-integration pass only, no
real parking attempt has been run. Before ever running either with the
drive wheels on the ground, verify (per each package's own README "실차
투입 전 반드시 할 것" / "Safety" sections, which this integration didn't
change):
- LiDAR mount pose assumptions (`laser_yaw`/`laser_angle_sign`/static TF
  `x=1.175 y=0.0 z=scan_height yaw=pi roll=pi`) match the real RPLiDAR S2
  mount - these came from whatever vehicle the packages were tuned on,
  reconcile against `obstacle_avoid.yaml`'s own mount description ("그릴
  장착, 지상고 12.5cm") before trusting them.
- Steering sign/max-angle calibration (`forward_turn_sign`,
  `max_steer_angle_left/right_deg`, the radius tables) against a fresh
  real-vehicle arc test - the shipped values cite specific past HENES
  test logs, but should be reconfirmed.
- `parking_bridge`'s `encoder_meter_per_count`/`wheel_base`/
  `steer_to_yaw_sign` against a real straight-line/rotation-in-place test
  (drive 10m straight, check `/wheel_odom`'s x).
- First attempt with wheels off the ground or in a wide-open, obstacle-free
  space, per both packages' own safety sections.

### `t_parking` gets its own straight-out exit logic

T-slot parking backs in nose-out, so unlike `parallel_parking` (which needs
a verified-safe reverse-retrace or a new S-curve to get back out),
exiting is just driving forward - no new path search needed. New states in
`rule_based_t_parking_node.py`:

- **`REVERSE_STRAIGHT`'s completion** no longer goes straight to `DONE`
  (which used to hand control back to camera/GPS immediately, with the
  vehicle still nose-in inside the slot, off the recorded line) - it goes
  to **`PARKED`** instead: stopped, `done` stays `False` so
  `control_arbiter` keeps relaying/failing-safe rather than handing back.
- **`PARKED`** waits for `/parking_exit_start` (or `auto_exit:=true` +
  `parking_hold_sec`, same pattern as `parallel_parking`), then
  `go_settle('EXIT_STRAIGHT')`.
- **`EXIT_STRAIGHT`** drives forward using the exact same start-heading-hold
  controller as `APPROACH` (`approach_cmd()`/new `exit_cmd()`, just at
  `exit_forward_rpm` instead of `pre_straight_rpm` - `sequence_start_yaw`
  is already the lane heading from before backing in, so no new heading
  target is needed). Completion is judged by a new **`entrance_clearance()`**
  helper: projects the rear-bumper corners onto the *already-locked*
  (odom-fixed) slot's depth axis relative to its entrance midpoint - the
  same corner math `rear_clearance()` already used against the far wall,
  just measured from the entrance instead. Once negative (past the
  entrance) by `exit_clear_margin`, `DONE` + `/parking_exit_done=true`.
  **This does NOT need a fresh LiDAR read** - the slot isn't even in
  view once driving forward out of it, and the geometry from the original
  lock is already exact, so re-projecting it is both sufficient and the
  only option.
- New topics: `/parking_exit_start` (in), `/parking_exit_done` (out) -
  same names `parallel_parking` already used, now shared by both packages;
  `parking_t_left.launch.py`'s topic remap list updated to include them.

### `control_arbiter` retrigger guard (parking zones)

Plain zone-based edge-triggering (`/parking_start` fires once when
`gps_idx` enters a `"parking_left"`/`"parking_right"` zone) isn't quite
enough on its own: mid-maneuver the vehicle intentionally leaves the
recorded line, so `stanley_control`'s global-nearest-waypoint search can
briefly jump `gps_idx` just outside the zone's `[start, end]` and back
(some other waypoint momentarily closer than anything inside the zone) -
without a guard, that looks identical to "left and re-entered the zone"
and would fire a second `/parking_start` while the first attempt is still
finishing, or right after `DONE`. Fixed with a `completed_pending_clear`
flag per side: once a side finishes (`done=True`), no new trigger fires
until `gps_idx` has gone solidly outside that specific completed zone's
`[start, end]` (padded by `parking_retrigger_clear_idx_margin`, default 30
idx) - not just "not currently matching" for one cycle - so a real second
lap through a looping course (`loop_waypoints:=true`) still re-triggers
normally once the vehicle has actually moved away and come back, but a
boundary flicker right after finishing can't restart it immediately. (The
parking nodes themselves also independently ignore `/parking_start` unless
`state in (IDLE, DONE, ABORT)` - this is a second, redundant layer on top
of that, not the only one.)

### Full stack now starts from one launch file

`post_gps_drive.launch.py` previously assumed RPLiDAR S2/taobotics
IMU/`obstacle_avoidance` were already running (per README "individually") -
they're folded in now (each toggleable: `enable_lidar`, `enable_imu`,
`enable_obstacle_avoid`, `enable_parking_left`, `enable_parking_right`),
so a course exercising `traffic_light` + `parking_left`/`parking_right` +
`avoid` together needs only:
```bash
ros2 launch waypoint_follower post_gps_drive.launch.py enable_control:=true \
  waypoints_file:=...
```
`imu_serial_port` defaults to the by-id path already used elsewhere in
this README - verify it matches the real device before trusting it.

## 2026-08-07: merged `t_parking_yellow.zip` (known-working reference from a second vehicle)

User supplied `~/Downloads/t_parking_yellow.zip` - a `t_parking` build
already confirmed working on a different ("yellow") vehicle, with the
explicit ask: keep the original as intact as possible, only swap what's
vehicle-specific (steering geometry).

**What was NOT adopted**: the zip's `CMakeLists.txt`/`ament_cmake` build
(it vendors and compiles its own copy of the `sllidar_ros2` C++ SDK,
bundles its own `handsfree_imu_a9_node.py`/`wheel_odom_pcan_node.py`/
`pcan_protocol.py`/`static_tf_publisher_node.py`, and ships a standalone
`launch/parking_left_oneshot.launch.py`) - that's this other vehicle's own
self-contained single-file test rig. HENES already has all of this running
as part of the main stack (RPLiDAR S2 via `rplidar_ros`, taobotics IMU,
and `parking_bridge`'s `wheel_odom_pcan_node` - CAN-feedback-only, already
proven working) - swapping in a second, untested CAN-writing/LiDAR-driving
implementation alongside the existing one would only add risk with no
benefit. `t_parking` here stays the plain `ament_python` package it already
was; `pcan_protocol.py` was checked and confirmed byte-identical to
`waypoint_follower/can_driver.py`'s struct layout either way (shared
firmware protocol across both vehicles, so no translation was ever needed).

**What WAS adopted** (pure-algorithm files/values, no build-system
changes): `geometry.py`, `two_arc_planner.py`, `temporal_gap_detector.py`,
`scan_parking_filter_node.py`, `slot_detector.py` were already byte-
identical between our copy and the zip's (already merged in an earlier
session). `cone_detector.py` and `rule_based_t_parking_node.py` had real
differences - taken wholesale from the zip, then this session's own
2026-08-05 exit-logic addition (`PARKED`/`EXIT_STRAIGHT` states,
`entrance_clearance()`, `exit_cmd()` - see above) was re-applied on top,
since the zip predates that feature and doesn't have it. Diffed the merge
result against the pre-merge file afterward to confirm the only changes
were the intended tuning improvements plus a clean re-application of the
exit logic - nothing silently lost.

Tuning/algorithm improvements pulled in from the zip (`config/parking_left.yaml`):
- `max_cluster_points: 60` → **400** - S2 DenseBoost can put hundreds of
  beams on one near cone; a low cap misreads a real cone as a wall. Actual
  wall rejection is still handled by the `n_exp_*_ratio`/width gates.
- New `far_track_range`(3.0m)/`far_min_track_hits`(2)/`far_track_miss_sec`(2.5) -
  distant cones aren't seen every frame (occlusion/weak return); confirm
  faster and hold longer past this range instead of using the same
  near-field noise thresholds.
- `entry_max_width: 2.20` → **2.60**.
- New `gap_side_min_lateral` (defaults to `0.5*vehicle_width +
  self_mask_extra` = 0.48m here) - returns inside the vehicle's own body
  width can't be an external boundary; replaces an old fixed 0.31m floor.
- `assumed_angle_increment_deg: 0.25` → **0.1125** (RPLiDAR S2's actual
  DenseBoost angular resolution - the old value was a generic/wrong-LiDAR
  placeholder, only used for the startup banner's detection-range estimate).
- `cone_detector.py`'s `ConeTracker.update()` now takes each detection's
  range and applies the far-track hit/timeout overrides above.

**Steering geometry correction** (the actual point of the merge - the
zip's `max_steer_angle_left/right_deg: 30.0` were explicitly marked `[!!]`
placeholders for the *other* vehicle): set to HENES's real measured
values, matching what `parallel_parking/config/parking_parallel_right.yaml`
already uses - `max_steer_angle_left_deg: 18.84`, `max_steer_angle_right_deg:
18.05` (cmd -30 → R=2.154m, cmd +30 → R=2.255m). Also fixed
`wheel_odom_front_axle`'s copy of the same two params (unused by our
current launch - we use `parking_bridge`'s node instead - but kept
consistent for anyone who switches back), and `log_root` (was hardcoded to
the other vehicle's `/home/jetson/...` path - changed to `~/.ros/parking_logs`).

Verified: `colcon build --symlink-install --packages-select t_parking`
succeeds, `rule_based_t_parking_node.py` imports cleanly under `rclpy`,
YAML parses. **Not yet hardware-tested** with these specific values - same
"실차 투입 전 반드시 할 것" caveats as the original 2026-08-05 integration
still apply (LiDAR mount pose, steering sign, radius calibration).

## 2026-08-07: parking-zone speed ramp (smooth pre-zone deceleration)

Confirmed on the real vehicle: `control_arbiter` was cutting rpm straight
from cruise (`gps_rpm`, ~130) to a parking zone's `approach_rpm` (30 for
`parking_left`/t_parking, 20 for `parking_right`/parallel_parking) the
instant `gps_idx` reached the zone's own start idx - felt like a near
emergency stop ("rpm이 130에서 30으로 바로 확 죽거든").

New `_parking_ramped_rpm(raw_rpm)` helper in `arbiter_node.py` blends a raw
rpm value toward a not-yet-active parking zone's `approach_rpm`, purely as
a function of `gps_idx`'s *current position* inside that side's ramp
window `[zone_start-margin, zone_start)` - no "ramp started at time T"
state at all. (An earlier same-day version snapshotted rpm once on window
entry and decayed it over a fixed wall-clock duration - dropped per
follow-up feedback ("구간 해서 그 안에 계속... 중간에 하게 해도 되게") since
that gave the wrong answer if `gps_idx` ever entered mid-window instead of
exactly at its first idx, e.g. after a restart. Position-based instead:
wherever `gps_idx` sits in the window, rpm is exactly the corresponding
blend - correct no matter when/where the vehicle entered it.)

- New param `parking_{side}_ramp_idx_margin` (default **7**) - ramp window
  starts this many idx before the zone's start. For `parking_left`'s
  current zone start (idx 58) that's idx 51 - right after the
  `traffic_light` zone ends at idx 50, per explicit request ("신호등
  뒤부터 바로 줄이고"). Plain `declare_parameter`, so it's live-tunable
  without relaunching: `ros2 param set /control_arbiter
  parking_left_ramp_idx_margin 7`.
- **Called from every place that would otherwise send an unramped rpm**
  while `gps_idx` is inside the ramp window - not just the normal
  camera/GPS fallback path (`base_rpm`). This mattered concretely: today's
  course has `parking_left`'s ramp window (idx 51-57) starting *inside*
  the `gps_priority` zone (idx 49-57), and that zone's branch sends
  `self.gps_rpm` directly rather than routing through `base_rpm` - so it
  needed its own `_parking_ramped_rpm()` call, or the ramp would have
  silently done nothing for the first several idx. Same fix applied to the
  `avoid` zone's CLEAR-state GPS-driving branch for the same reason
  (doesn't matter for today's course - no idx overlap there - but would
  otherwise silently break if a future course's zones overlapped).
- Only touches rpm - steering keeps coming from whatever's normally driving
  (camera or GPS), unchanged.
- Once `gps_idx` actually reaches the zone, `_handle_parking_zone` takes
  over completely (forces `approach_rpm` directly, ignoring all of the
  above) - the ramp having already eased speed down by then just means
  that handoff is no longer a hard step, not a functional dependency.

**Not yet re-tested live** with this specific margin/window placement.

## 2026-08-07: parking control-authority bug (GPS took back over mid-maneuver)

**Confirmed on the real vehicle**: GPS-driven entry into `parking_left`
would find a slot and start maneuvering, then partway through just
resumed straight GPS driving instead of finishing ("공간 잡고 틀려고 하고
하는데 그냥 gps쭉 하는듯"). Manually triggering `/parking_start` at a
standstill (any distance from the slot) worked fine every time - the
difference was GPS-driven entry specifically.

**Root cause**: `on_timer`'s dispatch chain gated `_handle_parking_zone()`
purely on `_zone_at(self.gps_idx)`'s CURRENT zone match - despite the
retrigger-guard comment right above it explicitly saying
"`self.parking[side]["done"]`, not idx leaving the zone, is what actually
ends this zone's special handling" - a real contradiction between the
documented intent and the actual code. During an active maneuver the
vehicle intentionally leaves the recorded line (backing into a slot), so
`stanley_control`'s global-nearest-waypoint search can carry `gps_idx`
outside the zone's `[start, end]` for real (not just the brief flicker the
retrigger-guard already handled) - the instant that happened, `zone`
stopped being `"parking_left"`, and control silently fell through to plain
GPS/camera driving mid-maneuver, abandoning it. A manual trigger at a
standstill never has this problem because the vehicle isn't following a
recorded GPS line - there's no `gps_idx` to wander at all until the
maneuver itself starts moving it via the parking node's own odometry-based
control.

**Fix**: new per-side `engaged` latch in `self.parking[side]`. Set `True`
at the exact same tick `/parking_start` fires, `False` only when
`state["done"]` is observed (mirrors the retrigger-guard's own
`completed_pending_clear` trigger point). `on_timer` now checks
`engaged_side` **first**, ahead of every other zone type (`stop`,
`gps_priority`, `avoid`, `traffic_light`) - once a side is engaged, it
owns control regardless of what `gps_idx` numerically does, until the
parking node itself reports done. The old idx-gated
`elif zone in ("parking_left", "parking_right")` branch is kept as a dead
defensive fallback (should be unreachable - `engaged` is set on the exact
same tick that branch's condition first becomes true).

Checked the failure-mode safety of latching through an `ABORT`: the
node's `ABORT` state keeps calling `publish_zero(True, False, False,
True)` every tick, which internally calls `publish_cmd(0, 0, 1)` - so
`cmd_rpm`/`cmd_steer` stay `0` and keep refreshing `cmd_last_time`,
`active` stays `True`, `done` stays `False`. Under `engaged`, that means
the arbiter keeps relaying a live `rpm=0, steer=0, enable=1` (motor
engaged, held stopped) via CAN indefinitely - safe by construction (never
silently falls back to blind GPS driving through a stalled/aborted
maneuver), but also means recovery needs a manual `/parking_reset` (the
side won't resume GPS driving on its own after an abort). Considered
acceptable - a parking abort should halt and wait for a human, not
quietly continue.

**Not yet re-tested live** - and per the very next section, this specific
fix turned out NOT to be the (whole) story - see below.

## 2026-08-07: THE actual root cause - `mapping` stayed True through the whole maneuver

The `engaged` fix above was real but not sufficient - next real-vehicle
test still showed the exact same symptom even past that fix ("reverse
arc 뜨고 각도 떠도 걍 rpm 30으로 앞으로만 가는데?" - the node's own state
was progressing correctly, SETUP_ARC → REVERSE_ARC, angles logging
correctly, but the actual vehicle just kept driving straight at
`approach_rpm`=30 the entire time).

**Root cause, found by reading `rule_based_t_parking_node.py`'s own
`on_timer` line by line**: `SETUP_ARC`, `REVERSE_ARC`, and
`REVERSE_STRAIGHT` were *all* calling
`self.pub_mapping.publish(Bool(data=True))` on every tick - i.e. `mapping`
never actually went `False` once a slot locked and the real 2-arc maneuver
began. `control_arbiter`'s `_handle_parking_zone` checks `mapping` *before*
`active` and drives via plain GPS at `approach_rpm` unconditionally
whenever it's `True`, regardless of anything else the node is doing - so
every one of this node's own `cmd_rpm`/`cmd_steer` values computed during
SETUP_ARC/REVERSE_ARC/REVERSE_STRAIGHT (correctly, per its own logged
state/angles) was silently discarded, every single tick, for the entire
maneuver. Not something the 2026-08-05 exit-logic merge or the
2026-08-07 `t_parking_yellow.zip` merge introduced - present in both, an
original oversight in the node's own state machine (mapping should mean
"still searching for a slot", but it was left `True` well past that
point).

**Fix**: `SETUP_ARC`/`REVERSE_ARC`/`REVERSE_STRAIGHT` now publish
`pub_mapping.publish(Bool(data=False))` - only `APPROACH` still publishes
`True` (correctly - it's the only state still searching for/tracking
toward a slot). Confirmed no other state publishes stray `True`s
(`grep pub_mapping.publish`).

Between this and the `engaged` fix above, the intended flow should now be:
`APPROACH` (mapping=True, GPS drives straight at approach_rpm) → slot
locks → `SETUP_ARC`/`REVERSE_ARC`/`REVERSE_STRAIGHT` (mapping=False,
`engaged` keeps the arbiter locked onto this side regardless of idx, node's
own `cmd_rpm`/`cmd_steer` relayed to CAN) → `PARKED`/`EXIT_STRAIGHT` (exit
logic) → `DONE` (`engaged` clears, control handed back).

**Not yet re-tested live** - see the next section, this still wasn't the
full fix either.

## 2026-08-07: same `mapping` bug, one state earlier than the fix above

Live re-test after the `SETUP_ARC`/`REVERSE_ARC`/`REVERSE_STRAIGHT` fix
still just drove straight the whole way, never even stopping ("setup arc
뜰 때 멈춰야 하는거 아님? 지금 주차 공간 탐지해서 멈추는 기준이 뭐야"). A
stationary capture at the zone's start idx confirmed cone
detection/tracking itself is healthy (`cones=5/5 tracks=5`,
`state=SEEK_BOUNDARY` - expected while not moving, since the temporal gap
detector needs real forward motion to see the boundary distance open/close).

**Root cause - same bug, one state earlier**: `APPROACH` was *also*
publishing `pub_mapping.publish(Bool(data=True))` unconditionally, for the
*entire* state, regardless of `self.locked`. But the real design intent
(and `_handle_parking_zone`'s own docstring) is "mapping = still
searching, don't trust this node's own approach controller yet" - which
should stop being true the moment a slot locks, not only once the state
name changes to `SETUP_ARC`. Once locked, `APPROACH` doesn't immediately
transition anyway - it keeps driving itself (via its own `approach_cmd()`)
toward the arc plan's start point, then calls `publish_stop()` and goes
through `SETTLE` (which waits for `vx` to actually drop below
`stop_speed_thresh` before advancing to `SETUP_ARC`). With `mapping` stuck
`True` through all of that, `control_arbiter` kept driving via GPS at
`approach_rpm` straight through the lock, the drive-to-start-point, and
the attempted stop - so `vx` never actually dropped low enough for
`SETTLE` to ever advance, and the vehicle could never reach `SETUP_ARC` at
all. This is exactly why the vehicle "just drove straight and never
stopped" - a fully correct lock happening under the hood couldn't matter,
because nothing after it could ever get relayed to CAN.

**Fix**: `APPROACH` now publishes `pub_mapping.publish(Bool(data=not
self.locked))` and `pub_active.publish(Bool(data=self.locked))` instead of
hardcoded `True`/`False` - both now track lock status directly rather than
state name. `SETTLE` itself still doesn't touch either topic (no change
needed there - it correctly just retains whatever `APPROACH` last
published, which by the time `SETTLE` is reached is already
`mapping=False, active=True` since `self.locked` only ever goes back to
`False` on a fresh `begin_sequence`/`reset_all`).

Combined with the earlier two fixes, the full intended handoff is now:
`APPROACH` unlocked (mapping=True, active=False, GPS drives) → lock →
`APPROACH` locked/driving-to-start-point AND `SETTLE` (mapping=False,
active=True, node's own `approach_cmd()`/stop relayed to CAN) →
`SETUP_ARC`/`REVERSE_ARC`/`REVERSE_STRAIGHT` (same, node's arc/reverse
commands relayed) → `PARKED`/`EXIT_STRAIGHT` → `DONE`.

**Not yet re-tested live** - and still wasn't quite it, see below.

## 2026-08-07: fourth bug in the same family - `SETUP_ARC` hardcoded `active=False`

Live re-test after the fix above: vehicle now actually stopped correctly
(SETTLE working), state advanced to `SETUP_ARC` (confirmed via
`/parking_t/parking_status`: `SETUP_ARC: steer=30 turned=-0.1/12.2deg
moved=0.003`), but then just sat there - `moved` never grew. Direct
per-topic capture nailed it in one shot: `/parking_t/parking_active=false`
while `/parking_t/cmd_rpm=30`, `cmd_steer=30`, `cmd_enable=1` were all
simultaneously live and correct. User's own follow-up capture confirmed
the exact transition: `parking_active` reads `false` while driving/
searching (correct - unlocked), flips `true` right as the vehicle nears
the slot (correct - `APPROACH`'s lock-triggered fix above), then **flips
back to `false` the instant it stops and never returns**.

**Root cause**: `SETUP_ARC` was calling
`self.pub_active.publish(Bool(data=False))` - hardcoded, not something the
`mapping` fix touched (different line, easy to miss scrolling past it
quickly). With `mapping` now correctly `False` too, `control_arbiter`'s
`_handle_parking_zone` no longer took the "mapping" branch, but
`active=False` meant it also failed the `fresh and state["active"]` check
and fell straight to the fail-safe stop branch - so every `cmd_rpm`/
`cmd_steer` this state computed (correctly, per its own log) got a plain
`rpm=0, enable=0` sent to CAN instead. `REVERSE_ARC`/`REVERSE_STRAIGHT`/
`EXIT_STRAIGHT` were already correct (`active=True`) - this was isolated
to `SETUP_ARC` alone, found only because the user captured the raw topics
directly rather than trusting the status log's text (which never mentions
`active` at all, so this was invisible in every log capture so far).

**Fix**: `SETUP_ARC` now publishes `pub_active.publish(Bool(data=True))`.

This is the fourth bug in the same family this session (`engaged`
idx-latch, `SETUP_ARC`/`REVERSE_ARC`/`REVERSE_STRAIGHT` mapping,
`APPROACH` mapping/active tied to lock, and now this) - all variations of
"the arbiter's mapping/active gate didn't match what the node's state
machine was actually doing." Recommend a full state-by-state audit of
`pub_mapping`/`pub_active` calls before the next real-vehicle test rather
than fixing these one at a time as each gets discovered - grep
`pub_mapping.publish\|pub_active.publish` and check every state's pair
against what it should be per `_handle_parking_zone`'s three cases
(searching → mapping=True; actively maneuvering → mapping=False,
active=True; done/aborted/parked-waiting → both False, or as
`publish_zero`'s call site dictates).

**Not yet re-tested live.**

## Known state / what's NOT done

- **RTK**: confirmed reaching Float (`flags=67`) and Fixed (`flags=131`,
  `flags=131` = Fixed + DGPS + valid) with real-world testing outdoors as
  of 2026-07-27, and confirmed Float again later on 2026-07-28 after a
  same-day `num_sv=0` scare (see "2026-07-28 session" above - not a
  software/USB-contention issue, and antenna/RF hardware confirmed
  healthy via `UBX-MON-HW`, so treated as transient). Re-tested standalone
  outdoors later the same day - Float acquired almost immediately,
  matching normal pre-2026-07-28 behavior - and again with GPS+ZED both
  running via `integrated_drive.launch.py`, same result. Depends on sky
  view and NTRIP network stability (caster connection has
  dropped/reconnected and shown checksum errors in testing - that's the
  caster/network, not this code). **Considered resolved** as of
  2026-07-28 - if `num_sv=0` recurs, try unplugging/replugging the GPS's
  USB connection first (see "Closing summary" in the 2026-07-28 session
  section above).
- **traffic_light package — deps installed, still zero hardware
  testing**: `depthai==2.32.0.0` (pinned <3, see gotcha above) and
  `ultralytics==8.4.110` are now installed and confirmed importable
  alongside `cv_bridge`/`pyzed`/`torch` without conflicts, and the node
  starts cleanly and idles when no OAK-D is found. But no OAK-D has
  actually been connected this session — the node itself and the
  `control_arbiter` `traffic_light` event-zone logic were only exercised
  with fake `ros2 topic pub` data (see "2026-07-28 session" above), never
  run against a real camera or real red/green light. Before trusting it
  on the vehicle: connect the OAK-D, verify detection accuracy/latency in
  the field, and check the default `traffic_light_approach_rpm_scale=0.5`
  / `traffic_light_timeout_sec=1.0` values against real behavior.
- **USB disconnect/renumbering — confirmed hardware, not fixed**: by-id
  symlinks (see GPS section) fix the device-path-changes-after-reconnect
  symptom, but the underlying drops themselves are real and unresolved.
  `sudo dmesg | grep -iE "usb|reset"` during a live failure showed repeated
  `device descriptor read/64, error -71` and `usb usbN-portM: attempt
  power cycle` on the GPS receiver's port - classic marginal-connection
  signatures (bad cable, damaged/loose connector, or a failing port),
  cycling every ~10-50s. User mentioned dropping this GPS unit at some
  point, which lines up with an intermittently-failing connector (fails
  sometimes, works other times, rather than a clean always-on/always-off
  break - consistent with what happened: it flooded with `End of file`
  errors during one test and ran clean for the next). Not something more
  code can fix - try a different cable, a different USB port, no hub,
  and physically inspect the receiver's port. In the meantime the *device
  path* problem (post-reconnect) is solved by the by-id symlinks; a
  mid-session drop still kills whatever's holding the port and needs a
  manual restart (an auto-retry wrapper was offered but not built - ask
  for it if the drops keep being disruptive).
- **Heading**: gyro + magnetometer + GPS fusion, see IMU section for
  calibration status and caveats. Untested while actually driving (both
  gyro drift and magnetometer soft-iron/interference).
- **Cornering**: actively being tuned this session (curvature slowdown +
  anticipatory steering) - see "Waypoint follower — controller changes"
  above for the latest analysis and open gaps (no speed term in lead
  distance, no infeasibility/recovery handling).
- **CAN control**: packet format (ID `0x200`, `<hhBBH`
  rpm/steer/enable/stop_mode, feedback IDs `0x101`/`0x102`) confirmed
  correct against real hardware (matches `pcan_tools/pcan_jetson_live.py`,
  a working manual keyboard-teleop script). Run with `enable_control:=true`
  on the real vehicle multiple times this session. `cruise_rpm`→real m/s
  calibration still not measured.
- **NTRIP credentials** (`rtk_bridge.py`, `f9p_bringup/launch/f9p_rover.launch.py`)
  are plaintext in-repo. Fine for now since `ros2_ws/src` isn't committed to
  git yet, but flag this before pushing anywhere.
- `waypoint_follower/gps_node.py`, `fake_gps_node.py`, and
  `f9p_bringup/f9p_bringup/rtcm_serial_bridge.py` are dead code from
  earlier attempts this session (custom NMEA parsing and a hand-rolled
  ROS-side RTCM bridge, both superseded) - left in place but unused, not
  deleted.

## 2026-08-12 session: re-recorded course, `parallel_parking.zip` re-merge,
## params_file launch collision, stop_mode judder fix, live parallel-parking
## debugging (vx noise, SETTLE hang, safety margins, lateral miss)

Long live-debugging session on a newly re-recorded test course. Summary of
what actually changed in code/config - see git history / diffs for exact
lines.

**New course + EVENT_ZONES**: re-recorded waypoints (201 points, idx 0-200).
`waypoint_follower/launch/post_gps_drive.launch.py`'s `EVENT_ZONES` rebuilt:
`10:29:traffic_light:18`, `30:37:gps_priority`, `38:89:parking_left`,
`90:114:gps_priority_slow` (new zone type - GPS-only like `gps_priority` but
rpm-capped via new `gps_priority_slow_rpm` param, added for the T-parking-
exit-to-parallel-parking-entry transit stretch), `115:142:parking_right`,
`143:200:avoid`. Built a one-off interactive waypoint-idx-viewer artifact to
pin these visually since exact idx boundaries weren't known ahead of time -
not saved into the repo, ask if it's needed again (quick to rebuild from
`recorded_waypoints.csv`).

**Critical bug found and fixed - `params_file` launch-argument name
collision**: `parking_t_left.launch.py` and `parking_parallel_right.launch.py`
both used to declare a plain `DeclareLaunchArgument('params_file', ...)`.
`DeclareLaunchArgument`/`LaunchConfiguration` names are global across the
*entire* launch tree, not scoped per included file - since
`post_gps_drive.launch.py` includes both, whichever got included first
silently "won" for both nodes. Confirmed live via `ps aux | grep
rule_based_parallel` showing the actual running process's `--params-file`
pointed at t_parking's `parking_left.yaml` instead of parallel_parking's own
config - this is why a `pre_straight_abort_yaw_deg: 0.0` config override
silently wasn't taking effect. **Fixed** by renaming to package-scoped arg
names: `t_parking_params_file` / `parallel_parking_params_file`.

**Parking-stop judder ("덜컹덜컹") root cause found in AURIX firmware
source** (`STM_Interrupt_1_KIT_TC275_LK_1.zip`, a separate Windows-side
repo, read-only reference): CAN `0x200` byte5 `stop_mode` isn't cosmetic -
`0` (normal) enables a closed-loop `StopHoldEnable=TRUE` hold-position PID
that can hunt/oscillate at a stop; `1` (flat) disables it
(`StopHoldEnable=FALSE`) for a clean release, matching the PS2 controller's
own 네모(square)-button behavior. `control_arbiter`'s parking relay used to
only send `stop_mode=1` when `cmd_enable==0`; now also sends it whenever
relayed `cmd_rpm==0` (`arbiter_node.py`). `parking_bridge/
wheel_odom_pcan_node.py`'s own direct-CAN path already had equivalent logic
(`auto_flat_stop_on_zero_rpm`, default True) so standalone-mode testing
wasn't affected by this specific gap.

**`parallel_parking.zip` re-merged** (new 2026-08-12 version, no-underscore
filename, confirmed working on a second vehicle) on top of this session's
own fixes: `max_steer_angle_left/right_deg` code defaults 30.0/30.0 ->
18.84/18.05 (config already had the real HENES values, unaffected), new
`exit_reverse_only` param (default True) + `build_reverse_handoff_plan()` -
exit now just reverses enough to clear the slot and hands off to the
waypoint follower instead of a full self-driven S-curve exit (matters for
standalone testing: a standalone run will look like it "doesn't finish
exiting" since nothing is left to drive it out to the lane - this is
by design, not a bug), `parallel_max_length` 4.00 -> 5.00.

**`wheel_odom_pcan_node.py` (shared by both t_parking and parallel_parking -
one fix covers both sides) - reported `vx` was single-sample instantaneous
`ds/dt` with no filtering**: at slow parking speeds a single ~20ms sample
often moves under 1 encoder count, so `vx` was dominated by count-
quantization noise and could sit pinned away from 0 even at a real stop
(`SETTLE` then waits forever). Fixed by averaging `vx` over the last
`vx_window_samples` (default 7) `(ds, dt)` pairs instead of one sample -
position/yaw integration (`self.x`/`self.y`/`self.yaw`) is untouched, only
the *reported* `vx` is smoothed. **Still not fully explained** - live
testing after this fix showed `vx`-at-rest varying run to run (0.081 / 0.099
/ 0.16 m/s) even on close-to-flat ground with the wheel barely visibly
moving, which doesn't cleanly fit either "software noise" (candump showed
`encoder_count`'s first 2 bytes genuinely, steadily changing in one
direction while "stopped" - a real signal, not stale data) or a consistent
"slope creep" story (magnitude too inconsistent run to run). Likely encoder
electrical/wiring noise rather than a pure software bug - not root-caused,
just worked around (see below).

**Practical workaround for the above - `SETTLE` now has a hard timeout**
(new `settle_timeout_sec` param, default 5.0, currently 3.0 in
`parking_parallel_right.yaml` for live testing) in both
`rule_based_parallel_parking_node.py` and `rule_based_t_parking_node.py`:
past this many seconds in `SETTLE`, advance to the next state regardless of
what `vx` is doing, instead of hanging forever. `stop_hold_sec` (1.0s)
already elapses before this even starts counting, so accuracy loss from not
waiting for a "clean" vx reading is expected to be small. Also raised
`stop_speed_thresh` 0.05 -> 0.10 in both parking configs as the "fast path"
(pass promptly without needing the timeout) - given the vx magnitude keeps
varying, don't expect this exact number to be final.

**Safety-margin tuning during live testing** (`parking_parallel_right.yaml`):
`reverse_cone_safety_stop_margin` 0.10 -> 0.07 (real `MANEUVER` reverse
aborted at `cone=0.099`, 1mm inside the old margin, `collision=False`) and
`swept_path_boundary_margin` 0.10 -> 0.07 (same maneuver next aborted on
`inner=0.093` against this *different* margin - the two are easy to
conflate; `reverse_safety_stop()`'s abort log used to print one shared
`margin=` value that was only ever accurate for the cone check, now fixed
to print each check's own margin next to its measured value). These are
real safety thresholds, not bugs - lowered because the actual geometry
legitimately runs that close, but they're a live-tuning guess, not
re-verified against a fresh full run yet.

**Open, unresolved after this session (rain stopped live testing)**:
- **`FINAL_ALIGN lateral miss q=0.425 exceeds 0.100`**: a real run got all
  the way through `MANEUVER` to `FINAL_ALIGN` (the last centering step)
  and aborted there because the vehicle ended up 42.5cm laterally off the
  slot center - `FINAL_ALIGN` deliberately refuses to correct lateral
  error with straight-line motion (a past version tried to and turned a
  near-miss into a rear-wall stop instead), so this needs the earlier
  arc/reverse segments to be more accurate, not a bigger tolerance here.
  Leading suspect: the same steering-angle calibration gap noted below -
  every arc segment's planned turn radius comes from
  `max_steer_angle_left/right_deg` (18.84/18.05, from a less-rigorous
  earlier arc-log test), while `can_driver.py`'s `TRUE_STEER_MAX_ANGLE_DEG`
  (14.3, from a 2-lap GPS circle-fit test) is the more trustworthy
  measurement - but only verified for the *left* turn direction, and
  parallel_parking's arcs use both directions. Next step: a dedicated
  full-lock circle-fit test for the *right* turn before trusting a swap to
  14.3 in `parallel_parking`'s config. `parallel_control.csv`/
  `parallel_candidates.csv` per-run logs under `~/.ros/parking_logs/
  rule_parallel_simple_<timestamp>/` were checked as a way to see which
  segment the error came from, but were empty (header row only, no data
  rows) for the runs checked this session - logger's write condition
  needs a look before it's useful for this kind of post-mortem.
- `parking_parallel_oneshot.launch.py` / `t_parking/launch/
  parking_left_oneshot.launch.py` are **broken, unadapted leftovers** from
  the original reference zips - they reference packages that don't exist
  in this workspace (`sllidar_ros2`, `my_first_pkg`) instead of this
  workspace's real ones (`rplidar_ros` via `obstacle_avoidance/launch/
  rplidar_s2.launch.py`, `mrpt_sensor_imu_taobotics`, `parking_bridge`).
  Don't use them as-is. For a real standalone-without-arbiter test, launch
  the real lidar/imu launch files plus `waypoint_follower/launch/
  parking_parallel_right.launch.py direct_cmd_output:=true auto_start:=true`
  instead (reuses the already-validated real components, just skips
  waiting for GPS/arbiter).
- `reverse_cone_safety_stop_margin`/`swept_path_boundary_margin`/
  `settle_timeout_sec`/`stop_speed_thresh` values above are all from one
  evening of live tuning cut short by weather - re-verify on the next dry
  run rather than assuming they're final.
- `vx` noise root cause (see above) not actually found - only worked
  around via `SETTLE`'s timeout. If it turns out to matter somewhere other
  than `SETTLE` (anywhere else that gates on `abs(vx) < threshold`), the
  same timeout-backstop pattern may need to be applied there too.

## 2026-08-17 session: CAN 진단 프로토콜(0x104/0x203) 설계, `arbiter_node.py` `base_steer` 로우패스 필터 추가

- **CAN 진단 프로토콜 신규 설계** (`can_driver.py`, `arbiter_node.py`, `README_CAN_PROTOCOL.md`,
  `henes_can.dbc`): `0x203 CONTROL_META`(TX, 로깅/CANoe 가시성 전용, 실제 제어엔 안 쓰임 -
  `0x200`과 같은 틱에 rpm/steer/stop_mode 미러링 + `controller_id`/`seq` 추가)와
  `0x104 DIAG_STATUS`(RX, 펌웨어 실측 - `applied_stop_mode`/`fault_flags`/`steer_pwm_duty`/
  `supply_voltage_mV`/`rx_seq_echo`). 설계 당시엔 펌웨어 미구현이었으나, 8/16 저녁 CANoe
  로그(`Logging_2.asc`) 확인 결과 팀원이 이미 구현해서 실제로 송신 중임을 확인함
  (`can_driver.py`의 "NOT YET SENT BY FIRMWARE" 주석은 갱신 필요, 아직 안 함).
  8/16 로그 분석 중 발견한 것들:
  - `supply_voltage_mV`가 17623개 프레임 전부 0 - 미배선/스텁 의심, 펌웨어팀 확인 필요
  - `rx_seq_echo` 라운드트립 0건 이상 없음 - seq echo 메커니즘 정상 동작 확인
  - `applied_stop_mode`가 대부분 1(flat)/2(hold)이고 0(disabled)은 거의 안 잡힘 - 요청
    (`0x200.stop_mode`)은 계속 0(normal)이었는데, 의미가 다르다는 건 알지만(README 참고)
    한 번 팀원 확인 필요

- **8/16 22:23 실주행 로그 분석** (`arbiter_can_20260816_222302.csv` +
  `drive_log_20260816_222303.csv`, `Logging_2.asc`와 시간대 겹침 확인:
  CAN 로그 절대시간 22:23:27~22:24:55 = ROS 로그 t=25.1~113.2s, 두 로그가 카메라→
  GPS_FALLBACK 전환 시점에서 서로 일치함):
  - **카메라 주행 중 rpm이 항상 130.0 고정, 분산 0** (커브에서 최대 -14.3°까지 꺾여도
    안 바뀜) - `yolopv2_zed_rpm_node.py` 자체엔 커브 감속 로직(`_speed_for_steer`,
    `auto_speed`/`steer_deadzone_deg=2.0`/`steer_full_deg=18.0`(30°-스케일 기준,
    물리각 환산 시 약 8.58°)/`rpm_turn_scale=0.8`)이 있지만, 이 런에서
    `can_enable=false`였어서 그 블록 자체가 실행 안 됐고(카메라는 steer 토픽만 publish),
    실제 CAN을 쏘는 `arbiter_node.py`가 카메라 주행 시 `base_rpm`을 자기 파라미터
    `camera_mode_rpm` 고정값으로만 쓰기 때문 - 카메라의 곡률 계산 결과 자체가 rpm에는
    전혀 반영 안 되는 구조. 필요하면 이거 arbiter에서 걸 수 있음 (현재는 보류).
  - `yolopv2_zed_rpm_node` 프로세스가 시작 302.0초 후 **`exit code -6`(SIGABRT)로 크래시**
    (`~/.ros/log/2026-08-16-22-23-02-*/launch.log`) - drive_log 마지막 t_s(300.96s)와
    거의 일치. "로그를 한참 뒤에 껐다"가 아니라 크래시로 끝난 것. 원인 미조사.
  - GPS `cruise_rpm=140`/`min_curve_rpm=50` 커브 기반 감속 확인 (`_curvature_scaled_rpm`) -
    직선 구간에서 135~140대 찍히는 게 정상 동작.
  - `camera_ok` 판정 3조건 정리: ① `camera_active`(lane_valid 프레임 히스테리시스,
    `camera_bad_frames_to_disable=10`/`camera_good_frames_to_enable=3`, 순수 프레임
    카운트로 시간 무관 - 실측 20Hz 설계 대비 어제 실제 13~15fps로 밀림, 10프레임이면
    설계상 500ms인데 실측 기준 670~770ms), ② freshness dead-man's switch
    (`camera_timeout_sec=1.0s`), ③ GPS cross-track veto(`camera_max_deviation_m=2.5m`
    넘으면 강제 스왑, 재진입은 `camera_deviation_reenter_m=2.5m` 안으로
    `camera_deviation_reenter_streak=20`틱 연속 필요, `camera_deviation_lockout_count=3`번
    /`camera_deviation_lockout_window_sec=20s` 안에 반복 트립되면
    `camera_deviation_lockout_sec=15s` 강제 락아웃 - 이땐 차선 바로 보여도 안 풀림).

- **`arbiter_node.py`에 `base_steer` EMA 로우패스 필터 추가** (신규 파라미터
  `base_steer_lowpass_alpha`, 기본값 `1.0` = 필터 끔, 기존 동작과 100% 동일 - opt-in):
  `filtered = alpha*현재값 + (1-alpha)*이전값`. `base_steer`(평상시 카메라/gps_fallback
  주행값 - traffic_light 구간, 아직 안 맞물린 상태로 GPS가 주차존 그냥 통과하는 구간에서도
  재사용됨)가 계산되는 딱 한 지점에 적용해서 하위 소비처 전부에 자동 반영되게 함. 소스가
  camera<->gps_fallback로 전환될 때도 그대로 블렌딩됨(의도된 동작 - 전환 시 튀는 것도
  같이 완화하려는 목적). `base_source`가 `None`(카메라·GPS 둘 다 무효)이 되면 필터 상태
  리셋 - safe_stop/이벤트존 통과 후 재진입 시 오래된 값에서 블렌딩 시작하는 걸 방지.
  avoid/parking-engaged/event-stop 등 자체 정밀 조향 로직을 쓰는 구간은 이 필터 영향 안 받음
  (그쪽은 각자 이미 별도로 값 관리함). 아직 실차 테스트 안 됨 - alpha 튜닝값 미정.

## 2026-08-17 (이어서): GPS 커브 감속에 데드존 추가 (`curve_deadzone_angle_deg`)

교수님 피드백: 각도(커브 곡률) 따라 속도 줄이는 로직에 "특정 각도 이내면
그냥 최대 속도, 그 이후 선형적으로 내리다 최소값 되면 유지"하는 형태로
만들라는 지시. 기존 `_curvature_scaled_rpm()`은 `turn_angle > 0`이면 바로
선형 감속이 시작되는 구조였음(데드존 없음) - `curve_deadzone_angle_deg`
파라미터(기본값 **5.0**, 사용자 판단 근거로 채택) 추가해서
`0~5° -> cruise_rpm(140, 평평) -> 선형 감속 -> curve_angle_for_min_rpm_deg
(40°) 이상 -> min_curve_rpm(50, 평평)` 구조로 수정. 기존 대비: `frac`
계산식이 `turn_angle / max_angle`에서 `(turn_angle - deadzone) /
(max_angle - deadzone)`로 바뀜. 실차 테스트 아직 안 함.

## 2026-08-17 (이어서): 카메라 곡률기반 rpm을 arbiter가 실제로 쓰도록 배선

이전까지 카메라 노드(`yolopv2_zed_rpm_node.py`)엔 곡률기반 rpm 스케일링
로직(`_speed_for_steer`, `auto_speed`/`steer_deadzone_deg=2.0`/
`steer_full_deg=18.0`/`rpm_turn_scale=0.8` - 상한(데드존, 각도 작으면
그냥 max)/하한(풀커브, `rpm_turn_scale`로 clamp) 둘 다 이미 있었음)이
있었지만 두 겹으로 막혀서 실제로는 전혀 안 쓰이고 있었음:
1. `integrated_drive.launch.py`/`post_gps_drive.launch.py`가 `can_enable:
   False`로 카메라 노드를 띄우는데(arbiter가 유일한 CAN 송신자여야 해서
   맞는 설정), `_speed_for_steer` 계산 자체가 `if self.can is not None:`
   블록 안에 있어서 통째로 안 돌았음
2. 설령 돌았어도 arbiter가 카메라 주행 시 rpm은 자기 고정 파라미터
   `camera_mode_rpm`만 썼음 - 카메라 계산 결과 자체를 안 받음

**수정:**
- `yolopv2_zed_rpm_node.py`: rpm_target 계산(+스텝 제한)을 `can_enable`
  게이트 밖으로 빼서 항상 실행, `~/rpm_target` 토픽으로 항상 publish
  (`~/motor_rpm`은 그대로 둠 - 그건 오도메트리 기반 속도추정값이라 다른
  의미)
- `arbiter_node.py`: `camera_rpm_topic`(기본
  `/yolopv2_zed_node/rpm_target`) 구독 추가, `_on_camera_rpm` 콜백,
  카메라 주행 시 `base_rpm = self.camera_rpm`으로 교체 (기존
  `camera_mode_rpm` 파라미터는 첫 메시지 오기 전 seed 값/폴백으로만 남음)
- `integrated_drive.launch.py`/`post_gps_drive.launch.py`: 카메라 노드에
  `auto_speed: true`, `can_target_rpm`(신규 인자 `camera_can_target_rpm`,
  기본 130 - `camera_mode_rpm`이랑 값은 맞췄지만 int 타입 캐스팅 문제
  때문에 별도 인자로 분리, `int("130.0")`이 에러나서) 추가

`can_enable: False`는 그대로 유지 - CAN은 여전히 arbiter만 씀, 이번
수정은 "계산은 하되 CAN 전송은 안 함, 결과만 토픽으로 넘김" 구조.
아직 실차 테스트 안 함.

## 2026-08-18: "stop" 이벤트존에 타이밍 정지(hold_sec) 지원 추가

Hill_Stop(언덕정지, idx 정지 + stop_mode 전환)이 아직 미구현이라, 그
자리에 우선 "idx에서 N초 정지 후 자동 재출발"만 되는 간단 버전을 넣어서
테스트해보려는 목적. `stop_mode` 전환은 안 함 - 순수 타이밍 정지만.

- `parse_event_zones`/`_zone_at`: 4번째 필드(`extra`)를 이제 zone
  kind별로 다르게 해석 - `traffic_light`는 여전히 stopline(없으면
  `end`로 폴백, 기존과 동일), `stop`은 hold_sec(없으면 무한정지, 기존과
  동일 - 하위호환).
- `"stop"` 브랜치: `hold_sec`이 있으면 그 zone에 처음 진입한 시각을
  기록해두고, 경과시간이 `hold_sec` 넘으면 `base_steer`/`base_rpm`(카메라
  또는 gps_fallback, 그 순간 뭐가 유효하냐에 따라)으로 자동 재개. idx가
  실제로 그 존을 벗어나면(차가 다시 움직이면서) 상태 리셋 - 다음 랩에
  같은 존 다시 만나면 또 처음부터 정지.
- 포맷: `"start:end:stop:hold_sec"`, 예: `"44:44:stop:3"` (idx 44에서
  3초 정지 후 재출발). `hold_sec` 생략하면 기존처럼 무한정지.
- `post_gps_drive.launch.py`의 `EVENT_ZONES`를 `["44:44:stop:3"]`으로
  임시 설정 (테스트용, 8/18 저녁 새로 기록한 코스
  `path_20260818_145848.csv` 기준). 실차 테스트 예정.

## 2026-08-18 (이어서): 타이밍 정지에 stop_mode=2(hill) 반영

바로 위 hold_sec 타이밍정지 테스트에서, 정지 중 stop_mode를 1(flat)
대신 **2(hill)**로 보내도록 수정 - 사용자 요청("의도완 다른데 암튼
작동하는거아니까"). 정확한 설계는 아님(진짜 Hill_Stop은 stop 존의
변형이 아니라 별도 state가 돼야 함, 다이어그램에도 그렇게 그려둠) -
CAN 레벨에서 stop_mode=2 동작 자체를 실차로 확인해보려는 임시 테스트.
정지 중(hold 안 끝난 동안)만 stop_mode=2, hold_sec 없는 기존 무한정지
zone은 그대로 stop_mode=1 유지.

## 2026-08-18 (이어서): 커브 조기 꺾임 - `curve_lead_margin` 1.5→1.2, launch 인자로 노출

실차 테스트 중 커브에서 너무 일찍 꺾이는 증상 발견 - Stanley의 예견
(anticipatory) blend(`stanley_control()`의 `curve_lead_margin`,
`required_lead_m = turning_radius_m * radians(turn_angle_deg) *
curve_lead_margin`)가 원인으로 추정됨. 이 값은 7/30ish 세션에 "반응이
너무 늦어서 코너 바깥으로 밀린다"는 반대 증상을 고치려고 1.1→1.5로
올렸던 건데, 지금은 과하게 일찍 반응하는 쪽으로 넘어간 상태로 보임.

- `waypoint_follower_node.py`의 `curve_lead_margin` 기본값 1.5→**1.2**로
  낮춤
- `integrated_drive.launch.py`/`post_gps_drive.launch.py`에 `
  curve_lead_margin` launch 인자로 노출 - 재빌드 없이 실차에서 바로
  튜닝 가능 (`curve_lead_margin:=1.0`, 더 낮춰보거나 `curve_lead_margin:
  =0.0`으로 예견 blend 자체를 꺼볼 수도 있음 - 0이면 `required_lead_m`도
  0이 돼서 무조건 blend=0, 순수 반응형 Stanley로 돌아감. 단, "코너가
  회전반경보다 급해서 물리적으로 못 돈다"는 별도 안전장치(full-lock
  폴백)는 이 값과 무관하게 계속 작동함)

stop_mode=2(hill)는 CAN 인코딩 자체(호스트 쪽 struct.pack)는 문제없음
재확인 - 펌웨어가 hill 모드 액추에이션을 아직 구현 안 했을 가능성이
높음(기존에 StopHoldEnable 하나로 normal/hill 구분 못한다고 기록해둔
내용과 일치), 펌웨어팀 확인 필요.

## 2026-08-18 (이어서): 타이밍 정지 무한반복 버그 수정 (`_stop_hold_fired_once`)

실차 테스트 중 발견: stop_mode=2가 실제로 언덕에서 차를 못 붙잡아서
정지 중 차가 밀려 내려감(펌웨어 hill-hold 미구현 - 위 항목 참고) ->
GPS idx가 그 여파로 다시 zone 안으로 들어옴 -> 정지 재발동 ->
또 밀림 -> 무한반복.

`_stop_hold_fired_once`(zone (start,end) 집합, 이번 실행 동안 영구
유지 - idx가 zone을 벗어나도 리셋 안 됨, 기존 `_stop_hold_key` 등
3개와 다름) 추가 - 한 번 hold_sec 다 채우고 정상 완료된 zone은
그 세션 동안 다시는 안 걸리고 그냥 통과함. 재출발 안전장치와 별개로,
"한 번 시도했으면 그걸로 끝"이라는 사용자 요청 반영.

## 2026-08-18 (이어서): 타이밍 정지 무한반복 버그 재수정 - fired_once를 완료 시점이 아니라 진입 시점에 마킹

바로 위 수정(`_stop_hold_fired_once`)이 실차에서 여전히 무한반복함 -
"밀렸다 앞으로 갔다"를 계속 반복. 원인: `fired_once`를 hold_sec **다
채운 뒤에만** 추가하고 있었는데, 언덕에서 진짜로 안 붙잡혀서(펌웨어
hill-hold 미구현) hold_sec 채우기 전에 idx가 zone 밖으로 밀려나가면
`_stop_hold_key`/`_stop_hold_done`이 리셋되면서 `fired_once`엔 아무것도
안 남고, 다시 idx 44 들어오면 완전히 처음부터 재시도 - 못 버티는 상황
자체가 반복되니 똑같이 계속 반복됨.

**수정**: zone에 **처음 진입하는 순간** 바로 `fired_once`에 추가하도록
변경(성공적으로 hold_sec 다 채웠는지와 무관). 이제 "한 번 시도(끝까지
버텼든 밀려서 중간에 빠졌든)하면 이번 실행 동안은 그걸로 끝" - 사용자가
원한 정확히 그 동작.

## 2026-08-18 (이어서): 타이밍 정지 hold 중 enable=1로 시험

hold 중(정지 유지하는 3초 동안) `enable=0`으로 보내던 걸 **`enable=1`**로
바꿔서 시험 - 사용자 가설: hill-hold는 모터를 완전히 꺼서(freewheel)
버티는 게 아니라, 모터를 켠 채로 target rpm=0을 유지하면서 중력에
반하는 토크를 실제로 걸어야(액티브 홀드) 되는 거 아니냐는 것.
`stop_mode=2`는 그대로. hold 끝나고 재출발/스킵 분기는 안 건드림.
실차 테스트 예정 - 이것도 효과 없으면 펌웨어 액추에이션 자체를
다시 봐야 함.

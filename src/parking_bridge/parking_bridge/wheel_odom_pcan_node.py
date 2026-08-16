#!/usr/bin/env python3
"""CAN wheel-odometry bridge for the parallel_parking / t_parking packages.

Ported from a standalone `wheel_odom_pcan_node.py` (originally part of a
`my_first_pkg` package this workspace doesn't have) - same job (read CAN
encoder + steering feedback, fuse with IMU yaw, publish nav_msgs/Odometry
on /wheel_odom) but rewired onto pieces this workspace already has instead
of the missing `.geometry`/`.pcan_protocol` sibling modules:
  - CAN frame layout (TX 0x200 / RX 0x101 / RX 0x102) reuses
    `waypoint_follower.can_driver` (the exact same parsing/packing already
    validated against the real HENES CAN bus) instead of a separate
    reimplementation, so this can't drift from the rest of the stack.
  - Angle/quaternion/table-interp helpers reuse `t_parking.geometry`
    (`yaw_from_quat`/`quat_from_yaw`/`clamp`/`normalize_angle`/
    `parse_table_param`/`interp_table`), since t_parking already ships
    dependency-free versions of all of them.
  - `blend_angle` (circular blend for yaw fusion) and `wrapped_delta`
    (encoder-counter wraparound-safe delta) aren't in either of those, so
    they're implemented directly below - both are small, standard.

IMPORTANT: `enable_command_tx` defaults to False. When running alongside
control_arbiter (the sole CAN writer in the full stack - see
waypoint_follower/README.md), this node must stay read-only: it opens its
own python-can bus purely to listen for 0x101/0x102 feedback frames and
never calls bus.send(). The parking node's own /parking/cmd_rpm etc. get
relayed to CAN by control_arbiter's "parking" priority mode instead - same
publish/relay pattern as everything else in this codebase (camera, GPS,
obstacle_avoid all publish; arbiter is the only one that writes CAN).
"""
from __future__ import annotations

import math
import traceback
from collections import deque
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Int16, Int32
from tf2_ros import TransformBroadcaster

from waypoint_follower import can_driver
from t_parking.geometry import (
    clamp,
    interp_table,
    normalize_angle,
    parse_table_param,
    quat_from_yaw,
    yaw_from_quat,
)

try:
    import can
except ImportError:  # pragma: no cover - reported at runtime on the target
    can = None


def blend_angle(current, target, weight):
    """Circular blend from current towards target by weight in [0,1] - used
    to fuse the encoder/steering-predicted yaw with the IMU yaw without a
    +-180deg wraparound glitch."""
    weight = clamp(weight, 0.0, 1.0)
    return normalize_angle(current + weight * normalize_angle(target - current))


def wrapped_delta(current, previous, modulus):
    """Signed delta between two counter readings that wrap at `modulus`
    (e.g. a 16-bit encoder count) - picks whichever direction (forward or
    backward through the wrap) is shorter, same as an angle wraparound."""
    delta = (current - previous) % modulus
    if delta > modulus // 2:
        delta -= modulus
    return delta


class WheelOdomPcanNode(Node):
    """SocketCAN command-bridge feedback reader and wheel odometry.

    Uses the already-validated CAN layout from waypoint_follower/can_driver.py:
      TX 0x200: <hhBBH  rpm, steer, enable, stop_mode, reserved
      RX 0x102: <hhhh   encoder, rpm_x10, pwm, target_rpm
      RX 0x101: <HHhh   current_pot, target_pot, angle_x10, target_angle_x10

    base_link is assumed to be at the rear-axle centre (matches
    obstacle_avoid.yaml / arbiter_node.py's wheelbase convention).
    """

    def __init__(self) -> None:
        super().__init__('wheel_odom_pcan_node')
        defaults = {
            'can_channel': 'can0',
            'can_reconnect_period': 1.0,
            'max_rx_messages_per_tick': 200,
            'tx_id': 0x200,
            'drive_status_id': 0x102,
            'steering_status_id': 0x101,
            # False by default - see module docstring. Only set true when
            # running this node standalone with no arbiter in the loop.
            'enable_command_tx': False,
            'publish_rate': 50.0,
            'tx_period': 0.05,
            'cmd_timeout': 0.30,
            'feedback_timeout': 0.30,
            'require_steering_feedback_for_connected': True,
            'max_rpm_cmd': 300,
            'max_steer_cmd': 30,
            'run_stop_mode': 0,
            'flat_stop_mode': 1,
            'hill_stop_mode': 2,
            'timeout_stop_mode': 1,
            'disabled_stop_mode': 1,
            'auto_flat_stop_on_zero_rpm': True,
            # pi * wheel_diameter(0.27m) / encoder_counts_per_rev(300) -
            # the same real-measured HENES calibration obstacle_avoid.yaml
            # uses (see wheel_diameter/encoder_counts_per_rev there), not
            # the placeholder value from the original standalone script.
            'encoder_meter_per_count': math.pi * 0.27 / 300.0,
            'encoder_sign': 1.0,
            'encoder_modulus': 65536,
            'max_encoder_delta_count': 500,
            'max_delta_distance_m': 0.25,
            # Reported vx is averaged over this many recent encoder samples
            # instead of a single-sample instantaneous ds/dt (2026-08-12).
            # At slow parking-maneuver speeds (e.g. maneuver_slow_rpm=8 ->
            # ~0.11m/s) a single ~20ms sample often moves under 1 encoder
            # count, so per-sample ds/dt is dominated by count-quantization
            # noise and never settles to 0 even when the vehicle is truly
            # stopped (SETTLE then waits forever for vx to drop below
            # stop_speed_thresh). Averaging ds/dt over a short window lets
            # genuine slow motion still accumulate a measurable distance
            # while random +-1 count noise cancels out at true rest.
            # Position/yaw integration (self.x/self.y/self.yaw) still uses
            # the raw per-sample ds unchanged - only the reported self.vx
            # is windowed.
            'vx_window_samples': 7,
            'wheel_base': 0.735,
            'steering_model': 'angle',  # angle | radius_table
            'steering_angle_scale': 1.0,
            'steering_angle_offset_deg': 0.0,
            'steering_deadband_deg': 0.5,
            # Matches waypoint_follower_node's steer_sign=-1 convention.
            'steer_to_yaw_sign': -1.0,
            'left_radius_table': '10:4.1684,20:2.0194,30:1.2731',
            'right_radius_table': '10:4.1684,20:2.0194,30:1.2731',
            'yaw_source': 'fused',  # steering | imu | fused
            'use_imu': True,
            'imu_topic': '/taobotics/sensor',
            'imu_sign': 1.0,
            'imu_gain': 1.0,
            'imu_yaw_alpha': 1.0,
            'yaw_deadband_deg': 0.0,
            'fused_imu_weight': 0.35,
            'cmd_rpm_topic': '/cmd_rpm',
            'cmd_steer_topic': '/cmd_steer',
            'cmd_enable_topic': '/cmd_enable',
            'cmd_stop_mode_topic': '/cmd_stop_mode',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'odom_topic': '/wheel_odom',
            'publish_tf': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in defaults:
            setattr(self, name, self.get_parameter(name).value)

        self.validate_parameters()

        self.left_radius_table = parse_table_param(
            str(self.left_radius_table), [(30, 1.2731)])
        self.right_radius_table = parse_table_param(
            str(self.right_radius_table), [(30, 1.2731)])

        self.bus = None
        self.last_reconnect_attempt = -math.inf
        self.last_tx_time = -math.inf
        self.last_rx_time = -math.inf
        self.last_drive_rx_time = -math.inf
        self.last_steer_rx_time = -math.inf

        self.cmd_rpm = 0
        self.cmd_steer = 0
        self.cmd_enable = 0
        self.cmd_stop_mode = int(self.flat_stop_mode)
        self.cmd_stamps = {'rpm': -math.inf, 'steer': -math.inf, 'enable': -math.inf}
        self.stop_mode_stamp = -math.inf

        self.encoder_count: Optional[int] = None
        self.previous_encoder_count: Optional[int] = None
        self.encoder_sample_seq = 0
        self.last_processed_encoder_seq = 0
        self.encoder_sample_time = -math.inf
        self.previous_encoder_sample_time = -math.inf

        self.actual_rpm = 0.0
        self.pwm_duty = 0
        self.feedback_target_rpm = 0
        self.current_pot = 0
        self.target_pot = 0
        self.current_angle_deg: Optional[float] = None
        self.target_angle_deg = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        # (ds, dt) of the last vx_window_samples encoder samples - see the
        # 'vx_window_samples' default's comment for why this exists.
        self.vx_window = deque(maxlen=max(1, int(self.vx_window_samples)))

        self.imu_ready = False
        self.imu_zero_yaw: Optional[float] = None
        self.imu_yaw = 0.0

        self.odom_pub = self.create_publisher(Odometry, str(self.odom_topic), 10)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if bool(self.publish_tf) else None
        )
        self.connected_pub = self.create_publisher(Bool, '/can/connected', 5)
        self.encoder_pub = self.create_publisher(Int32, '/can/encoder_count', 10)
        self.actual_rpm_pub = self.create_publisher(Float32, '/can/actual_rpm', 10)
        self.pwm_pub = self.create_publisher(Int16, '/can/pwm_duty', 10)
        self.steer_angle_pub = self.create_publisher(Float32, '/can/steering_angle_deg', 10)
        self.steer_pot_pub = self.create_publisher(Int32, '/can/steering_pot', 10)

        self.create_subscription(Int16, str(self.cmd_rpm_topic), self.cmd_rpm_cb, 10)
        self.create_subscription(Int16, str(self.cmd_steer_topic), self.cmd_steer_cb, 10)
        self.create_subscription(Int16, str(self.cmd_enable_topic), self.cmd_enable_cb, 10)
        self.create_subscription(Int16, str(self.cmd_stop_mode_topic), self.cmd_stop_mode_cb, 10)
        if bool(self.use_imu):
            self.create_subscription(Imu, str(self.imu_topic), self.imu_cb, 10)

        self.timer = self.create_timer(1.0 / max(float(self.publish_rate), 1.0), self.control_tick)
        self._last_log = -math.inf
        self._python_can_error_logged = False
        self.open_bus(force=True)

    def validate_parameters(self) -> None:
        if float(self.publish_rate) <= 0.0:
            raise ValueError('publish_rate must be positive')
        if float(self.tx_period) <= 0.0:
            raise ValueError('tx_period must be positive')
        if float(self.cmd_timeout) <= 0.0:
            raise ValueError('cmd_timeout must be positive')
        if float(self.feedback_timeout) <= 0.0:
            raise ValueError('feedback_timeout must be positive')
        if int(self.max_rx_messages_per_tick) <= 0:
            raise ValueError('max_rx_messages_per_tick must be positive')
        if int(self.encoder_modulus) <= 0:
            raise ValueError('encoder_modulus must be positive')
        if float(self.encoder_meter_per_count) <= 0.0:
            raise ValueError('encoder_meter_per_count must be positive')
        if float(self.wheel_base) <= 0.0:
            raise ValueError('wheel_base must be positive')
        if str(self.steering_model).strip().lower() not in ('angle', 'radius_table'):
            raise ValueError('steering_model must be angle or radius_table')
        if str(self.yaw_source).strip().lower() not in ('steering', 'imu', 'fused'):
            raise ValueError('yaw_source must be steering, imu, or fused')

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def cmd_rpm_cb(self, message: Int16) -> None:
        self.cmd_rpm = int(clamp(message.data, -int(self.max_rpm_cmd), int(self.max_rpm_cmd)))
        self.cmd_stamps['rpm'] = self.now_sec()

    def cmd_steer_cb(self, message: Int16) -> None:
        self.cmd_steer = int(clamp(message.data, -int(self.max_steer_cmd), int(self.max_steer_cmd)))
        self.cmd_stamps['steer'] = self.now_sec()

    def cmd_enable_cb(self, message: Int16) -> None:
        self.cmd_enable = int(clamp(message.data, 0, 1))
        self.cmd_stamps['enable'] = self.now_sec()

    def cmd_stop_mode_cb(self, message: Int16) -> None:
        self.cmd_stop_mode = int(clamp(message.data, 0, 2))
        self.stop_mode_stamp = self.now_sec()

    def imu_cb(self, message: Imu) -> None:
        raw = float(self.imu_sign) * float(self.imu_gain) * yaw_from_quat(message.orientation)
        if self.imu_zero_yaw is None:
            self.imu_zero_yaw = raw
        relative = normalize_angle(raw - self.imu_zero_yaw)
        if abs(math.degrees(normalize_angle(relative - self.imu_yaw))) < float(self.yaw_deadband_deg):
            relative = self.imu_yaw
        self.imu_yaw = blend_angle(self.imu_yaw, relative, float(self.imu_yaw_alpha))
        self.imu_ready = True

    def open_bus(self, force: bool = False) -> None:
        if can is None:
            if not self._python_can_error_logged:
                self._python_can_error_logged = True
                self.get_logger().error(
                    'python-can is not installed; install package python3-can'
                )
            return

        now = self.now_sec()
        if self.bus is not None:
            return
        if (
            not force
            and now - self.last_reconnect_attempt < float(self.can_reconnect_period)
        ):
            return

        self.last_reconnect_attempt = now
        try:
            self.bus = can_driver.open_bus(str(self.can_channel))
            self.get_logger().info(f'CAN opened: channel={self.can_channel}')
        except Exception as exc:
            self.bus = None
            self.get_logger().warning(f'CAN open failed: {exc}')

    def close_bus(self) -> None:
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
        self.bus = None

    def read_can(self) -> None:
        if self.bus is None:
            self.open_bus()
            return

        try:
            for _ in range(int(self.max_rx_messages_per_tick)):
                message = self.bus.recv(timeout=0.0)
                if message is None:
                    break

                now = self.now_sec()
                if (
                    message.arbitration_id == int(self.drive_status_id)
                    and len(message.data) == 8
                ):
                    status = can_driver.parse_drive_status(bytes(message.data))
                    self.encoder_count = status['encoder_count']
                    self.actual_rpm = status['rpm']
                    self.pwm_duty = status['pwm_duty']
                    self.feedback_target_rpm = status['target_rpm']
                    self.encoder_sample_seq += 1
                    self.encoder_sample_time = now
                    self.last_drive_rx_time = now
                    self.last_rx_time = now
                elif (
                    message.arbitration_id == int(self.steering_status_id)
                    and len(message.data) == 8
                ):
                    status = can_driver.parse_steering_status(bytes(message.data))
                    self.current_pot = status['current_pot']
                    self.target_pot = status['target_pot']
                    self.current_angle_deg = (
                        float(self.steering_angle_scale) * status['current_angle']
                        + float(self.steering_angle_offset_deg)
                    )
                    self.target_angle_deg = status['target_angle']
                    self.last_steer_rx_time = now
                    self.last_rx_time = now
        except Exception as exc:
            self.get_logger().warning(f'CAN receive failed: {exc}')
            self.close_bus()

    def command_is_fresh(self, now: float) -> bool:
        return all(now - stamp <= float(self.cmd_timeout) for stamp in self.cmd_stamps.values())

    def safe_command(self, now: float) -> tuple[int, int, int, int]:
        if not self.command_is_fresh(now):
            return 0, 0, 0, int(self.timeout_stop_mode)

        rpm = int(self.cmd_rpm)
        steer = int(self.cmd_steer)
        enable = int(self.cmd_enable)
        stop_mode = int(self.cmd_stop_mode)

        if now - self.stop_mode_stamp > float(self.cmd_timeout):
            stop_mode = int(self.run_stop_mode)
        if enable == 0:
            rpm = 0
            steer = 0
            stop_mode = int(self.disabled_stop_mode)
        elif bool(self.auto_flat_stop_on_zero_rpm) and rpm == 0:
            stop_mode = int(self.flat_stop_mode)
        return rpm, steer, enable, int(clamp(stop_mode, 0, 2))

    def send_command(self) -> None:
        # Stays a no-op unless explicitly opted into standalone mode - see
        # module docstring. control_arbiter is what actually writes CAN
        # commands for parking when running the full stack.
        if not bool(self.enable_command_tx):
            return
        now = self.now_sec()
        if now - self.last_tx_time < float(self.tx_period):
            return
        self.last_tx_time = now
        if self.bus is None:
            self.open_bus()
            return

        rpm, steer, enable, stop_mode = self.safe_command(now)
        try:
            can_driver.send_control(self.bus, rpm, steer, enable, stop_mode)
        except Exception as exc:
            self.get_logger().warning(f'CAN send failed: {exc}')
            self.close_bus()

    def steering_curvature(self) -> float:
        angle_or_command = (
            self.current_angle_deg
            if self.current_angle_deg is not None
            and self.now_sec() - self.last_steer_rx_time <= float(self.feedback_timeout)
            else float(self.cmd_steer)
        )
        model = str(self.steering_model).strip().lower()
        if model == 'angle':
            if abs(angle_or_command) <= float(self.steering_deadband_deg):
                return 0.0
            wheel_base = float(self.wheel_base)
            if wheel_base <= 0.0:
                return 0.0
            return (
                float(self.steer_to_yaw_sign)
                * math.tan(math.radians(angle_or_command))
                / wheel_base
            )

        if abs(angle_or_command) <= float(self.steering_deadband_deg):
            return 0.0
        radius = interp_table(
            self.left_radius_table if angle_or_command < 0 else self.right_radius_table,
            angle_or_command,
        )
        if not math.isfinite(radius) or radius <= 0.0:
            return 0.0
        return (
            float(self.steer_to_yaw_sign)
            * math.copysign(1.0, angle_or_command)
            / radius
        )

    def update_odometry(self) -> None:
        now = self.now_sec()
        if self.encoder_sample_seq == self.last_processed_encoder_seq:
            if now - self.last_drive_rx_time > float(self.feedback_timeout):
                self.vx = 0.0
                self.wz = 0.0
                self.vx_window.clear()
            return

        self.last_processed_encoder_seq = self.encoder_sample_seq
        current = self.encoder_count
        sample_time = self.encoder_sample_time
        if current is None:
            return
        if self.previous_encoder_count is None:
            self.previous_encoder_count = current
            self.previous_encoder_sample_time = sample_time
            return

        dt = sample_time - self.previous_encoder_sample_time
        delta = wrapped_delta(current, self.previous_encoder_count, int(self.encoder_modulus))
        self.previous_encoder_count = current
        self.previous_encoder_sample_time = sample_time
        if dt <= 0.0:
            return
        if abs(delta) > int(self.max_encoder_delta_count):
            self.get_logger().warning(f'encoder jump skipped: {delta}')
            self.vx = 0.0
            self.wz = 0.0
            self.vx_window.clear()
            return

        ds = float(self.encoder_sign) * delta * float(self.encoder_meter_per_count)
        if abs(ds) > float(self.max_delta_distance_m):
            self.get_logger().warning(f'distance jump skipped: {ds:.3f} m')
            self.vx = 0.0
            self.wz = 0.0
            self.vx_window.clear()
            return

        predicted_yaw = normalize_angle(self.yaw + ds * self.steering_curvature())
        source = str(self.yaw_source).strip().lower()
        if source == 'imu' and self.imu_ready:
            new_yaw = self.imu_yaw
        elif source == 'fused' and self.imu_ready:
            new_yaw = blend_angle(predicted_yaw, self.imu_yaw, float(self.fused_imu_weight))
        else:
            new_yaw = predicted_yaw

        delta_yaw = normalize_angle(new_yaw - self.yaw)
        mid_yaw = normalize_angle(self.yaw + 0.5 * delta_yaw)
        self.x += ds * math.cos(mid_yaw)
        self.y += ds * math.sin(mid_yaw)
        self.yaw = new_yaw
        # Position/yaw above already integrated the raw per-sample ds/
        # delta_yaw - only the *reported* self.vx is smoothed, over the
        # last vx_window_samples (ds, dt) pairs, so single-count
        # quantization noise at near-zero speed cancels out instead of
        # holding self.vx pinned away from 0 (see vx_window_samples'
        # default comment).
        self.vx_window.append((ds, dt))
        window_dt = sum(item[1] for item in self.vx_window)
        self.vx = (
            sum(item[0] for item in self.vx_window) / window_dt
            if window_dt > 0.0 else 0.0)
        self.wz = delta_yaw / dt

    def publish_feedback(self) -> None:
        if self.encoder_count is not None:
            message = Int32()
            message.data = int(self.encoder_count)
            self.encoder_pub.publish(message)

        rpm_message = Float32()
        rpm_message.data = float(self.actual_rpm)
        self.actual_rpm_pub.publish(rpm_message)

        pwm_message = Int16()
        pwm_message.data = int(clamp(self.pwm_duty, -32768, 32767))
        self.pwm_pub.publish(pwm_message)

        if self.current_angle_deg is not None:
            angle_message = Float32()
            angle_message.data = float(self.current_angle_deg)
            self.steer_angle_pub.publish(angle_message)

        pot_message = Int32()
        pot_message.data = int(self.current_pot)
        self.steer_pot_pub.publish(pot_message)

        now = self.now_sec()
        drive_fresh = (
            self.bus is not None
            and now - self.last_drive_rx_time <= float(self.feedback_timeout)
        )
        steer_fresh = now - self.last_steer_rx_time <= float(self.feedback_timeout)
        connected = Bool()
        connected.data = bool(
            drive_fresh
            and (steer_fresh or not bool(self.require_steering_feedback_for_connected))
        )
        self.connected_pub.publish(connected)

    def publish_odometry(self) -> None:
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quat_from_yaw(self.yaw)
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = str(self.odom_frame)
        message.child_frame_id = str(self.base_frame)
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = self.vx
        message.twist.twist.angular.z = self.wz
        self.odom_pub.publish(message)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = str(self.odom_frame)
            transform.child_frame_id = str(self.base_frame)
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

    def send_emergency_stop(self) -> None:
        if self.bus is None or not bool(self.enable_command_tx):
            return
        try:
            can_driver.send_control(self.bus, 0, 0, 0, int(self.disabled_stop_mode))
        except Exception:
            pass

    def control_tick(self) -> None:
        try:
            self._control_tick()
        except Exception as exc:
            self.get_logger().error(
                f'control_tick failed: {type(exc).__name__}: {exc}\n'
                f'{traceback.format_exc()}'
            )
            self.send_emergency_stop()
            self.close_bus()

    def _control_tick(self) -> None:
        self.read_can()
        self.update_odometry()
        self.publish_odometry()
        self.publish_feedback()
        self.send_command()
        now = self.now_sec()
        if now - self._last_log >= 0.5:
            self._last_log = now
            rpm, steer, enable, stop_mode = self.safe_command(now)
            self.get_logger().info(
                f'CAN={self.bus is not None} enc={self.encoder_count} rpm_fb={self.actual_rpm:.1f} '
                f'steer_fb={self.current_angle_deg} cmd=({rpm},{steer},{enable},{stop_mode}) '
                f'odom=({self.x:.3f},{self.y:.3f},{math.degrees(self.yaw):.1f}deg)'
            )

    def destroy_node(self):
        self.send_emergency_stop()
        self.close_bus()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[WheelOdomPcanNode] = None
    try:
        node = WheelOdomPcanNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

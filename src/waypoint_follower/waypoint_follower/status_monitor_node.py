#!/usr/bin/env python3
"""One clean, continuously-refreshing status line instead of scrolling
through every node's own throttled log output (waypoint_follower_node,
control_arbiter, camera, obstacle_avoid, parking, traffic_light all log
independently at ~1Hz - combined that's a lot of noise to read through
during a live test).

Subscribes to the topics that already exist for exactly this purpose
(gps_control/*, ~/steering_deg, ~/lane_valid, control_arbiter's own
active_source) and prints one line at print_rate_hz, e.g.:

  idx=123 v=2.14m/s cte=0.18m | gps: steer=-3.2deg rpm=118 valid=1
  | camera: steer=1.1deg valid=1 | ARBITER: camera steer=1.1deg rpm=100
  enable=1 stop_mode=0

Doesn't replace the individual nodes' own logs (still useful for actually
debugging a specific one) - this is for "what's happening right now" at a
glance during a drive.
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String

# Also published on status_monitor/summary (std_msgs/String) - `ros2 topic
# echo /status_monitor/summary` reads it from any terminal, no launch
# output/piping/buffering to fight with (that's the print()-to-screen path
# below, kept for when this node IS the launch's only screen output).


def _fmt(value, digits=2, unit=""):
    if value is None:
        return "?"
    return f"{value:.{digits}f}{unit}"


class StatusMonitorNode(Node):
    def __init__(self):
        super().__init__("status_monitor_node")

        self.declare_parameter("print_rate_hz", 2.0)
        self.declare_parameter("stale_sec", 1.0)

        self.gps_steer = None
        self.gps_rpm = None
        self.gps_speed = None
        self.gps_idx = None
        self.gps_valid = None
        self.cross_track = None
        self.camera_steer = None
        self.camera_valid = None
        self.active_source = "?"
        self._last_update = {}

        def _stamp(name):
            self._last_update[name] = time.time()

        def on_gps_steer(msg):
            self.gps_steer = msg.data
            _stamp("gps")

        def on_gps_rpm(msg):
            self.gps_rpm = msg.data

        def on_gps_speed(msg):
            self.gps_speed = msg.data

        def on_gps_idx(msg):
            self.gps_idx = msg.data

        def on_gps_valid(msg):
            self.gps_valid = msg.data

        def on_cross_track(msg):
            self.cross_track = msg.data

        def on_camera_steer(msg):
            self.camera_steer = msg.data
            _stamp("camera")

        def on_camera_valid(msg):
            self.camera_valid = msg.data

        def on_active_source(msg):
            self.active_source = msg.data
            _stamp("arbiter")

        self.create_subscription(Float32, "gps_control/steer_deg", on_gps_steer, 10)
        self.create_subscription(Float32, "gps_control/rpm", on_gps_rpm, 10)
        self.create_subscription(Float32, "gps_control/speed_mps", on_gps_speed, 10)
        self.create_subscription(Int32, "gps_control/target_idx", on_gps_idx, 10)
        self.create_subscription(Bool, "gps_control/valid", on_gps_valid, 10)
        self.create_subscription(
            Float32, "gps_control/cross_track_error_m", on_cross_track, 10
        )
        self.create_subscription(
            Float32, "/yolopv2_zed_node/steering_deg", on_camera_steer, 10
        )
        self.create_subscription(
            Bool, "/yolopv2_zed_node/lane_valid", on_camera_valid, 10
        )
        self.create_subscription(
            String, "/control_arbiter/active_source", on_active_source, 10
        )
        self.summary_pub = self.create_publisher(String, "status_monitor/summary", 10)

        rate = float(self.get_parameter("print_rate_hz").value)
        self.create_timer(1.0 / rate if rate > 0 else 0.5, self._print_status)

    def _fresh(self, name):
        t = self._last_update.get(name)
        if t is None:
            return False
        return (time.time() - t) <= float(self.get_parameter("stale_sec").value)

    def _print_status(self):
        idx = self.gps_idx if self.gps_idx is not None else "?"
        gps_tag = "" if self._fresh("gps") else " (STALE)"
        cam_tag = "" if self._fresh("camera") else " (STALE)"
        arb_tag = "" if self._fresh("arbiter") else " (STALE)"

        line = (
            f"idx={idx} v={_fmt(self.gps_speed, 2, 'm/s')} "
            f"cte={_fmt(self.cross_track, 2, 'm')} | "
            f"gps: steer={_fmt(self.gps_steer, 1, 'deg')} "
            f"rpm={_fmt(self.gps_rpm, 0)} valid={self.gps_valid}{gps_tag} | "
            f"camera: steer={_fmt(self.camera_steer, 1, 'deg')} "
            f"valid={self.camera_valid}{cam_tag} | "
            f"ARBITER: {self.active_source}{arb_tag}"
        )
        self.summary_pub.publish(String(data=line))
        # flush=True: stdout is block-buffered (not line-buffered) once
        # piped into anything (e.g. `| grep`), so without this the lines
        # sit in the buffer and arrive in laggy bursts instead of live.
        print(line, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

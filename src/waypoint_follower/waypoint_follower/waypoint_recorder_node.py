import csv
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from waypoint_follower.geo_utils import distance_m, latlon_to_mercator

# Same mapping ublox_gps's driver uses (ublox_firmware7plus.hpp): a basic
# fix and Float RTK both come through as STATUS_FIX, Fixed RTK comes
# through one level up as STATUS_GBAS_FIX - see waypoint_follower_node's
# on_fix() for the same convention.
_STATUS_LABELS = {
    NavSatStatus.STATUS_NO_FIX: "NONE",
    NavSatStatus.STATUS_FIX: "FLOAT",
    NavSatStatus.STATUS_SBAS_FIX: "SBAS",
    NavSatStatus.STATUS_GBAS_FIX: "FIXED",
}


class WaypointRecorderNode(Node):
    """Subscribes to a NavSatFix topic and records a Lat,Lon waypoint
    every time the fix moves at least `min_waypoint_distance_m` from the
    last recorded point. Writes the CSV on shutdown (Ctrl+C).

    Also records each point's RTK status (Float/Fixed/etc, from the same
    status.status field waypoint_follower_node uses) - added 2026-07-29
    so recorded paths can later be visualized/audited by fix quality
    instead of just as a bare lat/lon polyline. Points with no fix at all
    (status.status < STATUS_FIX) are skipped entirely rather than
    recorded as garbage 0,0 coordinates."""

    def __init__(self):
        super().__init__("waypoint_recorder_node")

        self.declare_parameter("gps_topic", "/fix")
        self.declare_parameter("output_file", os.path.expanduser("~/recorded_waypoints.csv"))
        self.declare_parameter("min_waypoint_distance_m", 0.5)

        self.output_file = self.get_parameter("output_file").value
        self.min_dist = self.get_parameter("min_waypoint_distance_m").value

        self.points = []
        self._last_xy = None

        self.create_subscription(
            NavSatFix, self.get_parameter("gps_topic").value, self._fix_cb, 10
        )

        self.get_logger().info(
            f"Recording waypoints from {self.get_parameter('gps_topic').value} "
            f"(min spacing {self.min_dist} m). Ctrl+C to stop and save to "
            f"{self.output_file}"
        )

    def _fix_cb(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return  # no fix yet - don't record garbage 0,0 coordinates

        lat, lon = msg.latitude, msg.longitude
        x, y = latlon_to_mercator(lat, lon)

        if self._last_xy is not None and distance_m(*self._last_xy, x, y) < self.min_dist:
            return

        self._last_xy = (x, y)
        status = msg.status.status
        self.points.append((lat, lon, status))
        label = _STATUS_LABELS.get(status, str(status))
        self.get_logger().info(f"Waypoint {len(self.points)}: {lat:.7f}, {lon:.7f} [{label}]")

    def save(self):
        if not self.points:
            self.get_logger().warn("No waypoints recorded, nothing to save.")
            return

        with open(self.output_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Lat", "Lon", "FixStatus", "FixLabel"])
            writer.writerows([
                [f"{lat:.7f}", f"{lon:.7f}", status, _STATUS_LABELS.get(status, str(status))]
                for lat, lon, status in self.points
            ])

        self.get_logger().info(f"Saved {len(self.points)} waypoints to {self.output_file}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

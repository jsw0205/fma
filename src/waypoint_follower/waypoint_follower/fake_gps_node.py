import csv

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from waypoint_follower.geo_utils import distance_m, latlon_to_mercator


def load_waypoints_csv(path):
    latlon = []
    with open(path, "r") as f:
        for row in csv.DictReader(f):
            latlon.append((float(row["Lat"]), float(row["Lon"])))
    return latlon


class FakeGpsNode(Node):
    """Publishes NavSatFix that walks along a waypoint CSV at a fixed
    speed, so the waypoint follower can be exercised end-to-end without
    real GPS/CAN hardware attached."""

    def __init__(self):
        super().__init__("fake_gps_node")

        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("gps_topic", "/fix")
        self.declare_parameter("speed_mps", 1.0)
        self.declare_parameter("publish_rate_hz", 5.0)

        waypoints_file = self.get_parameter("waypoints_file").value
        if not waypoints_file:
            waypoints_file = (
                get_package_share_directory("waypoint_follower")
                + "/waypoints/sample_waypoints.csv"
            )

        self.waypoints = load_waypoints_csv(waypoints_file)
        self.get_logger().info(
            f"fake_gps_node walking {len(self.waypoints)} waypoints from {waypoints_file}"
        )

        self.seg_lengths = []
        for i in range(len(self.waypoints) - 1):
            x0, y0 = latlon_to_mercator(*self.waypoints[i])
            x1, y1 = latlon_to_mercator(*self.waypoints[i + 1])
            self.seg_lengths.append(distance_m(x0, y0, x1, y1))
        self.total_length = sum(self.seg_lengths)

        self.traveled_m = 0.0
        self.done = False

        self.fix_pub = self.create_publisher(
            NavSatFix, self.get_parameter("gps_topic").value, 10
        )

        rate = self.get_parameter("publish_rate_hz").value
        self.create_timer(1.0 / rate, self.on_timer)

    def _position_at(self, traveled_m):
        if traveled_m <= 0.0:
            return self.waypoints[0]
        if traveled_m >= self.total_length:
            return self.waypoints[-1]

        remaining = traveled_m
        for i, seg_len in enumerate(self.seg_lengths):
            if remaining <= seg_len:
                frac = remaining / seg_len if seg_len > 1e-9 else 0.0
                lat0, lon0 = self.waypoints[i]
                lat1, lon1 = self.waypoints[i + 1]
                return (lat0 + frac * (lat1 - lat0), lon0 + frac * (lon1 - lon0))
            remaining -= seg_len

        return self.waypoints[-1]

    def on_timer(self):
        rate = self.get_parameter("publish_rate_hz").value
        speed = self.get_parameter("speed_mps").value

        if not self.done:
            self.traveled_m += speed / rate
            if self.traveled_m >= self.total_length:
                self.traveled_m = self.total_length
                self.done = True
                self.get_logger().info("fake_gps_node reached the last waypoint")

        lat, lon = self._position_at(self.traveled_m)

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 0.0
        self.fix_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeGpsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

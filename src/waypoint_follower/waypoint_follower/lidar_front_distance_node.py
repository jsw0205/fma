import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class LidarFrontDistanceNode(Node):
    """Subscribes to a LaserScan and publishes the range at 0 degrees
    (straight ahead, +x axis) as a Float32, averaged over a small window
    to reduce single-ray noise."""

    def __init__(self):
        super().__init__("lidar_front_distance_node")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("output_topic", "/front_distance")
        self.declare_parameter("window_deg", 2.0)

        self.window_rad = math.radians(self.get_parameter("window_deg").value)

        self.pub = self.create_publisher(
            Float32, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.on_scan,
            qos_profile_sensor_data,
        )

    def on_scan(self, msg: LaserScan):
        if not (msg.angle_min <= 0.0 <= msg.angle_max):
            return

        half_window = self.window_rad / 2.0
        lo = max(0.0 - half_window, msg.angle_min)
        hi = min(0.0 + half_window, msg.angle_max)
        i_lo = round((lo - msg.angle_min) / msg.angle_increment)
        i_hi = round((hi - msg.angle_min) / msg.angle_increment)
        i_lo = max(0, min(i_lo, len(msg.ranges) - 1))
        i_hi = max(0, min(i_hi, len(msg.ranges) - 1))

        valid = [
            r
            for r in msg.ranges[i_lo : i_hi + 1]
            if msg.range_min <= r <= msg.range_max and not math.isnan(r)
        ]
        if not valid:
            self.get_logger().warn("No valid returns near 0 deg")
            return

        distance = sum(valid) / len(valid)
        self.pub.publish(Float32(data=distance))
        self.get_logger().info(f"Front distance (0 deg): {distance:.3f} m")


def main(args=None):
    rclpy.init(args=args)
    node = LidarFrontDistanceNode()
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

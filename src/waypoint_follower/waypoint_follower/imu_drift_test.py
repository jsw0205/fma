import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuDriftTest(Node):
    """Pure gyro-only yaw integration, no GPS correction at all - just to
    see how much heading drifts over time with nothing pulling it back."""

    def __init__(self):
        super().__init__("imu_drift_test")
        self.declare_parameter("imu_topic", "/handsfree/imu")

        self.yaw = 0.0
        self.start_time = None
        self.last_time = None

        self.create_subscription(
            Imu, self.get_parameter("imu_topic").value, self.on_imu, 10
        )
        self.create_timer(10.0, self.report)

        self.get_logger().info(
            "Integrating angular_velocity.z only, starting at yaw=0. "
            "Keep the IMU still (or move it back to the same spot) to see "
            "pure drift."
        )

    def on_imu(self, msg):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        if self.last_time is not None:
            dt = (now - self.last_time).nanoseconds / 1e9
            if 0 < dt < 1.0:
                self.yaw += msg.angular_velocity.z * dt
        self.last_time = now

    def report(self):
        if self.start_time is None:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.get_logger().info(
            f"t={elapsed:6.1f}s  integrated_yaw={math.degrees(self.yaw):+7.2f} deg"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuDriftTest()
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

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField


class MagCalibrate(Node):
    """Drive/spin the vehicle through one full 360-degree turn while this
    runs (car stays level - only yaw needs to sweep through all headings,
    no need to tip/tilt it). Ctrl+C when you've completed the full circle;
    prints the hard-iron offset to plug into mag_offset_x/mag_offset_y."""

    def __init__(self):
        super().__init__("mag_calibrate")
        self.declare_parameter("mag_topic", "/handsfree/mag")

        self.min_x = math.inf
        self.max_x = -math.inf
        self.min_y = math.inf
        self.max_y = -math.inf
        self.count = 0

        self.create_subscription(
            MagneticField, self.get_parameter("mag_topic").value, self.on_mag, 10
        )
        self.create_timer(2.0, self.report)

        self.get_logger().info(
            "지금부터 차를 완전히 한 바퀴(360도) 천천히 돌리세요. "
            "다 돌았으면 Ctrl+C로 종료 - 오프셋 결과가 출력됩니다."
        )

    def on_mag(self, msg):
        x, y = msg.magnetic_field.x, msg.magnetic_field.y
        self.min_x = min(self.min_x, x)
        self.max_x = max(self.max_x, x)
        self.min_y = min(self.min_y, y)
        self.max_y = max(self.max_y, y)
        self.count += 1

    def report(self):
        if self.count == 0:
            return
        offset_x = (self.min_x + self.max_x) / 2.0
        offset_y = (self.min_y + self.max_y) / 2.0
        spread_x = self.max_x - self.min_x
        spread_y = self.max_y - self.min_y
        self.get_logger().info(
            f"[진행중, {self.count}개 샘플] "
            f"현재 offset_x={offset_x:.4f} offset_y={offset_y:.4f} "
            f"(x범위={spread_x:.4f} y범위={spread_y:.4f} - 한바퀴 다 돌면 이 두 범위가 "
            f"비슷해져야 정상)"
        )

    def final_report(self):
        if self.count == 0:
            self.get_logger().warn("샘플을 하나도 못 받았습니다 - mag_topic 확인하세요.")
            return
        offset_x = (self.min_x + self.max_x) / 2.0
        offset_y = (self.min_y + self.max_y) / 2.0
        print("\n=== 캘리브레이션 결과 ===")
        print(f"샘플 수: {self.count}")
        print(f"x: min={self.min_x:.4f} max={self.max_x:.4f} -> offset_x={offset_x:.4f}")
        print(f"y: min={self.min_y:.4f} max={self.max_y:.4f} -> offset_y={offset_y:.4f}")
        print(
            "\n이 값을 waypoint_follower_node 실행할 때 넣으세요:\n"
            f"  -p mag_offset_x:={offset_x:.4f} -p mag_offset_y:={offset_y:.4f}"
        )
        x_spread = self.max_x - self.min_x
        y_spread = self.max_y - self.min_y
        if x_spread > 0 and y_spread > 0:
            ratio = max(x_spread, y_spread) / min(x_spread, y_spread)
            if ratio > 1.3:
                print(
                    f"\n주의: x범위/y범위 비율이 {ratio:.2f}로 차이가 좀 큽니다 "
                    "(1.0에 가까워야 정상) - 한 바퀴를 다 못 돌았거나, 소프트아이언 "
                    "왜곡(찌그러짐)이 있을 수 있어요. 한 바퀴 다시 확실하게 돌려서 "
                    "재측정 해보세요."
                )


def main(args=None):
    rclpy.init(args=args)
    node = MagCalibrate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.final_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

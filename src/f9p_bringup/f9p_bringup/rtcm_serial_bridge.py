import rclpy
import serial
from rclpy.node import Node
from rtcm_msgs.msg import Message


class RtcmSerialBridge(Node):
    """Writes incoming rtcm_msgs/Message bytes straight to the GPS serial
    port. The stock ublox_gps driver has no RTCM ROS subscriber at all
    (no rtcm_msgs dependency), so ntrip_client's corrections never reached
    the receiver without this. Write-only, so it doesn't fight
    ublox_gps_node for the read side of the same port."""

    def __init__(self):
        super().__init__('rtcm_serial_bridge')

        self.declare_parameter('device', '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00')
        self.declare_parameter('baud', 19200)
        self.declare_parameter('rtcm_topic', 'rtcm')

        device = self.get_parameter('device').value
        baud = self.get_parameter('baud').value

        self.ser = serial.Serial(device, baud, timeout=0.1)
        self.get_logger().info(f'RTCM serial bridge writing to {device}@{baud}')

        self.create_subscription(
            Message, self.get_parameter('rtcm_topic').value, self.on_rtcm, 10
        )

    def on_rtcm(self, msg):
        self.ser.write(bytes(msg.message))

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RtcmSerialBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

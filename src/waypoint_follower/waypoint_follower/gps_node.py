import threading

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

# GGA fix quality codes: 0=no fix, 1=GPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float.
def dm_to_dec(dm, direction):
    try:
        d = int(float(dm) / 100)
        m = float(dm) - d * 100
        dec = d + m / 60
        return -dec if direction in ("S", "W") else dec
    except (ValueError, TypeError):
        return None


class GpsNode(Node):
    def __init__(self):
        super().__init__("gps_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("gps_topic", "/fix")
        # Default: RTK Fixed/Float only, same as the validated Windows
        # GPS-driving code this was ported from. Loosen this (e.g. to
        # include "1") to test without RTK corrections / rtk_bridge running,
        # such as a handheld walk test where cm-level accuracy doesn't matter.
        self.declare_parameter("accepted_fix_qualities", ["4", "5"])

        self.fix_pub = self.create_publisher(
            NavSatFix, self.get_parameter("gps_topic").value, 10
        )

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value

        try:
            ser = serial.Serial(port, baud, timeout=0.1)
        except Exception as exc:
            self.get_logger().error(f"Could not open GPS serial port {port}: {exc}")
            return

        self.get_logger().info(f"GPS serial connected: {port}@{baud}")
        accepted_fix_qualities = set(self.get_parameter("accepted_fix_qualities").value)

        with ser:
            while not self._stop.is_set():
                try:
                    line = ser.readline().decode("ascii", errors="ignore").strip()
                except Exception as exc:
                    self.get_logger().warn(f"GPS serial read error: {exc}")
                    continue

                if not (line.startswith("$GNGGA") or line.startswith("$GPGGA")):
                    continue

                parts = line.split(",")
                if len(parts) <= 6 or not (parts[2] and parts[3] and parts[4] and parts[5]):
                    continue

                if parts[6] not in accepted_fix_qualities:
                    continue

                lat = dm_to_dec(parts[2], parts[3])
                lon = dm_to_dec(parts[4], parts[5])
                if lat is None or lon is None:
                    continue

                altitude = 0.0
                if len(parts) > 9 and parts[9]:
                    try:
                        altitude = float(parts[9])
                    except ValueError:
                        pass

                self._publish_fix(lat, lon, altitude)

    def _publish_fix(self, lat, lon, altitude):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = altitude
        self.fix_pub.publish(msg)

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class GpsToOdom(Node):
    def __init__(self):
        super().__init__('gps_to_odom')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.gps_callback,
            10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.path_publisher = self.create_publisher(Path, '/path', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.origin_lat = None
        self.origin_lon = None
        
        # WGS-84 Earth semimajor axis (m)
        self.a = 6378137.0
        # WGS-84 Earth eccentricity squared
        self.esq = 6.69437999014e-3
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'
        self.get_logger().info("GpsToOdom node initialized and waiting for GPS fix...")

    def gps_callback(self, msg: NavSatFix):
        # Ignore invalid fixes (status.status < 0 means no fix)
        if msg.status.status < 0:
            return
            
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f"GPS Origin set to Lat: {self.origin_lat:.8f}, Lon: {self.origin_lon:.8f}")
            
        # Convert lat/lon difference to local meters using flat earth approximation around origin
        lat_rad = math.radians(msg.latitude)
        origin_lat_rad = math.radians(self.origin_lat)
        delta_lat = math.radians(msg.latitude - self.origin_lat)
        delta_lon = math.radians(msg.longitude - self.origin_lon)
        
        # Radius of curvature in prime vertical
        N = self.a / math.sqrt(1.0 - self.esq * math.sin(origin_lat_rad)**2)
        # Radius of curvature in meridian
        M = self.a * (1.0 - self.esq) / (1.0 - self.esq * math.sin(origin_lat_rad)**2)**(1.5)
        
        x = delta_lon * N * math.cos(origin_lat_rad)
        y = delta_lat * M
        
        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        
        # Identity orientation
        odom.pose.pose.orientation.w = 1.0
        
        # Fill covariance from GPS covariance (approximate diagonal)
        odom.pose.covariance[0] = msg.position_covariance[0] # x var
        odom.pose.covariance[7] = msg.position_covariance[4] # y var
        odom.pose.covariance[14] = msg.position_covariance[8] # z var
        
        self.odom_publisher.publish(odom)
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
        # Publish Path
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = msg.header.stamp
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose = odom.pose.pose
        
        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(pose_stamped)
        
        # Limit path length to prevent memory bloat (last 2000 points)
        if len(self.path_msg.poses) > 2000:
            self.path_msg.poses.pop(0)
            
        self.path_publisher.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

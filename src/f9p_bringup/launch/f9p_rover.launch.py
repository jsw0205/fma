import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    pkg_f9p_bringup = get_package_share_directory('f9p_bringup')

    # Declare arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00',
        description='ublox USB device path (stable by-id symlink - a plain '
                     '/dev/ttyACM0 shifts to ttyACM1/2/... after a USB drop/reconnect)'
    )

    rtk_bridge_script_arg = DeclareLaunchArgument(
        'rtk_bridge_script',
        default_value='/home/a/ros2_ws/src/rtk_bridge/rtk_bridge.py',
        description='Path to the standalone NTRIP RTCM relay script (not a ROS package)'
    )

    # Ublox GPS Node configuration path
    ublox_config_path = os.path.join(pkg_f9p_bringup, 'config', 'ublox_rover.yaml')

    # Ublox GPS Node
    ublox_gps_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='screen',
        parameters=[
            ublox_config_path,
            {
                'device': LaunchConfiguration('device')
            }
        ],
        remappings=[
            ('/ublox_gps_node/nmea', '/nmea'),
            ('/fix', '/ublox_gps_node/fix')
        ]
    )

    # RTCM relay: fetches corrections from NTRIP and writes them straight to
    # the GPS serial port. The stock ublox_gps driver has no RTCM ROS
    # subscriber at all (confirmed - no rtcm_msgs dependency in its
    # package.xml), so ntrip_client + a ROS-side bridge never actually
    # reaches the receiver. This raw script writes directly to the serial
    # port instead. No need to wait for ublox_gps_node to finish
    # configuring, but launching both in the exact same instant (0 action
    # delay) reliably corrupts ublox_gps_node's serial read - a short fixed
    # delay avoids the simultaneous-open collision without needing to
    # parse ublox_gps_node's log output.
    rtk_bridge_action = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=['python3', LaunchConfiguration('rtk_bridge_script')],
            name='rtk_bridge',
            output='screen',
        )],
    )

    # GPS to Odometry Node for visualization
    gps_to_odom_node = Node(
        package='f9p_bringup',
        executable='gps_to_odom',
        name='gps_to_odom_node',
        output='screen'
    )

    # Static TF node from base_link to gps frame
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base_to_gps',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps']
    )

    return LaunchDescription([
        device_arg,
        rtk_bridge_script_arg,
        ublox_gps_node,
        rtk_bridge_action,
        gps_to_odom_node,
        static_tf_node
    ])

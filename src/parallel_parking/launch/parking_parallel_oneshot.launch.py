#!/usr/bin/env python3
"""Launch the complete right-side parallel-parking stack.

LiDAR + static TF + IMU + PCAN command/odometry + scan filter + parking node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('parallel_parking')
    default_params = os.path.join(
        package_share, 'config', 'parking_parallel_right.yaml')

    params_file = LaunchConfiguration('params_file')

    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
        }],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        output='screen',
        arguments=[
            '--x', '1.175', '--y', '0.0',
            '--z', LaunchConfiguration('scan_height'),
            '--yaw', '3.14159265',
            '--pitch', '0.0', '--roll', '3.14159265',
            '--frame-id', 'base_link', '--child-frame-id', 'laser',
        ],
    )

    imu = Node(
        package='my_first_pkg',
        executable='handsfree_imu_a9_node',
        name='handsfree_imu_a9_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_imu')),
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baudrate': 921600,
            'frame_id': 'imu_link',
            'imu_topic': '/imu/data',
        }],
    )

    pcan = Node(
        package='my_first_pkg',
        executable='wheel_odom_pcan_node',
        name='wheel_odom_pcan_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'can_interface': 'socketcan',
            'can_channel': LaunchConfiguration('can_channel'),
            'can_bitrate': 500000,
            'configure_can_interface': False,
            'enable_command_tx': ParameterValue(
                LaunchConfiguration('enable_command_tx'), value_type=bool),
            'tx_id': 0x200,
            'drive_status_id': 0x102,
            'steering_status_id': 0x101,
            'cmd_rpm_topic': '/cmd_rpm',
            'cmd_steer_topic': '/cmd_steer',
            'cmd_enable_topic': '/cmd_enable',
            'cmd_stop_mode_topic': '/cmd_stop_mode',
            'cmd_timeout': 0.30,
            'feedback_timeout': 0.30,
            'max_rpm_cmd': 300,
            'max_steer_cmd': 30,
            'encoder_meter_per_count': 0.002930016494,
            'encoder_sign': 1.0,
            'encoder_modulus': 65536,
            'wheel_base': 0.735,
            'steering_model': 'angle',
            'steer_to_yaw_sign': -1.0,
            'yaw_source': 'fused',
            'use_imu': ParameterValue(
                LaunchConfiguration('use_imu'), value_type=bool),
            'imu_topic': '/imu/data',
            'fused_imu_weight': 0.65,
            'odom_topic': '/wheel_odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True,
        }],
    )

    scan_filter = TimerAction(period=3.0, actions=[Node(
        package='t_parking',
        executable='scan_parking_filter',
        name='scan_parking_filter',
        output='screen',
        parameters=[params_file],
    )])

    parking = TimerAction(period=5.0, actions=[Node(
        package='parallel_parking',
        executable='rule_based_parallel_parking_node',
        name='rule_based_parallel_parking_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {
                'parking_side': ParameterValue(
                    LaunchConfiguration('parking_side'), value_type=str),
                'auto_start': ParameterValue(
                    LaunchConfiguration('auto_start'), value_type=bool),
                'auto_exit': ParameterValue(
                    LaunchConfiguration('auto_exit'), value_type=bool),
                'parking_hold_sec': ParameterValue(
                    LaunchConfiguration('parking_hold_sec'),
                    value_type=float),
                'direct_cmd_output': ParameterValue(
                    LaunchConfiguration('direct_cmd_output'), value_type=bool),
                'pre_straight_rpm': ParameterValue(
                    LaunchConfiguration('pre_straight_rpm'),
                    value_type=int),
                'maneuver_forward_rpm': ParameterValue(
                    LaunchConfiguration('maneuver_forward_rpm'),
                    value_type=int),
                'maneuver_reverse_rpm': ParameterValue(
                    LaunchConfiguration('maneuver_reverse_rpm'),
                    value_type=int),
                'exit_forward_rpm': ParameterValue(
                    LaunchConfiguration('exit_forward_rpm'),
                    value_type=int),
                'exit_reverse_rpm': ParameterValue(
                    LaunchConfiguration('exit_reverse_rpm'),
                    value_type=int),
            },
        ],
    )])

    start_trigger = TimerAction(
        period=LaunchConfiguration('start_delay'),
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '-1', '/parking_start',
                    'std_msgs/msg/Bool', '{data: true}',
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_on_launch')),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('parking_side', default_value='right'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('auto_exit', default_value='false'),
        DeclareLaunchArgument('parking_hold_sec', default_value='3.0'),
        DeclareLaunchArgument('direct_cmd_output', default_value='true'),
        DeclareLaunchArgument('pre_straight_rpm', default_value='20'),
        DeclareLaunchArgument('maneuver_forward_rpm', default_value='20'),
        DeclareLaunchArgument('maneuver_reverse_rpm', default_value='-20'),
        DeclareLaunchArgument('exit_forward_rpm', default_value='8'),
        DeclareLaunchArgument('exit_reverse_rpm', default_value='-8'),
        DeclareLaunchArgument('start_on_launch', default_value='false'),
        DeclareLaunchArgument('start_delay', default_value='12.0'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='256000'),
        DeclareLaunchArgument('scan_height', default_value='0.16'),
        DeclareLaunchArgument('can_channel', default_value='can0'),
        DeclareLaunchArgument('enable_command_tx', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument(
            'imu_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Silicon_Labs_HandsFree_IMU_USB_to_UART_Bridge_'
                'Controller_0001-if00-port0')),
        lidar,
        static_tf,
        imu,
        pcan,
        scan_filter,
        parking,
        start_trigger,
    ])

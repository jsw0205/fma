#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
parking_left_oneshot.launch.py

ROS1 parking_right_oneshot.launch 의 ROS2 좌측판.

  1) rplidar_ros            : RPLIDAR A3M1 (256000 baud)
  2) static TF              : base_link -> laser
  3) handsfree_imu_a9       : 실제 yaw 측정 (선택, 기본 사용)
  4) wheel_odom_pcan        : 주차 명령 -> PCAN, 엔코더+IMU -> odom
  5) scan_parking_filter    : /scan -> /scan_parking (뒤집힘 반영 각도창)
  6) rule_based_t_parking   : 주차 노드 (auto_start=false 면 IDLE 대기)
  7) (선택) /parking_start 자동 발행

사용법 (안전: 수동 시작 권장)
  ros2 launch t_parking parking_left_oneshot.launch.py
  준비되면 다른 터미널에서:
  ros2 topic pub -1 /parking_start std_msgs/Bool "{data: true}"

완전 자동 (start_delay 초 뒤 실제로 움직입니다)
  ros2 launch t_parking parking_left_oneshot.launch.py \
      start_on_launch:=true start_delay:=12.0

CAN 은 미리 올려둘 것:
  sudo ip link set can0 type can bitrate 500000
  sudo ip link set can0 up
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
    pkg = get_package_share_directory('t_parking')
    params = os.path.join(pkg, 'config', 'parking_left.yaml')

    args = [
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('parking_side', default_value='left'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('direct_cmd_output', default_value='true'),
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
    ]

    pf = LaunchConfiguration('params_file')

    # ---------------- 1) LiDAR ----------------
    # A3M1 : 256000 baud. 드라이버는 원시 센서 각도를 그대로 발행하고,
    # 아래 static TF와 주차 노드의 수동 좌표변환이 동일한 장착 자세
    # Rz(pi)*Rx(pi)를 표현한다.
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

    # ---------------- 2) static TF : base_link -> laser ----------------
    # 실측 규약 raw180=차량 전방, raw90=차량 좌측을 만족하려면
    # Rz(pi)*Rx(pi), 즉 yaw=pi + roll=pi 여야 한다.
    # 기존 yaw=0, roll=pi 는 RViz에서 base_link 마커의 좌우를 뒤집었다.
    # 주차 노드의 theta_vehicle=pi-theta_raw 변환과 정확히 같은 자세다.
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_laser', output='screen',
        arguments=['--x', '1.175', '--y', '0.0',
                   '--z', LaunchConfiguration('scan_height'),
                   '--yaw', '3.14159265',
                   '--pitch', '0.0', '--roll', '3.14159265',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
    )

    # ---------------- 3) HandsFree IMU ----------------
    imu = Node(
        package='my_first_pkg', executable='handsfree_imu_a9_node',
        name='handsfree_imu_a9_node', output='screen', emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_imu')),
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baudrate': 921600,
            'frame_id': 'imu_link',
            'imu_topic': '/imu/data',
        }],
    )

    # ---------------- 4) PCAN command bridge + wheel odom ----------------
    # /cmd_rpm, /cmd_steer, /cmd_enable -> CAN 0x200
    # CAN 0x101/0x102 -> steering/encoder feedback -> /wheel_odom
    # 기존 wheel_odom_front_axle와 동시에 실행하면 /wheel_odom 및 TF가
    # 중복되므로 PCAN 통합 노드만 실행한다.
    pcan = Node(
        package='my_first_pkg', executable='wheel_odom_pcan_node',
        name='wheel_odom_pcan_node', output='screen', emulate_tty=True,
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
            # 조향각 적분만으로는 steer=0일 때의 실차 편향을 알 수 없다.
            # IMU 상대 yaw를 융합해 시작 헤딩 직선 추종과 원호 종료에 사용한다.
            'yaw_source': 'fused',
            'use_imu': ParameterValue(
                LaunchConfiguration('use_imu'), value_type=bool),
            'imu_topic': '/imu/data',
            'fused_imu_weight': 0.35,
            'odom_topic': '/wheel_odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True,
        }],
    )

    # ---------------- 4) 주차용 스캔 필터 ----------------
    scan_filter = TimerAction(period=3.0, actions=[Node(
        package='t_parking', executable='scan_parking_filter',
        name='scan_parking_filter', output='screen',
        parameters=[pf],
    )])

    # ---------------- 5) 주차 노드 ----------------
    parking = TimerAction(period=5.0, actions=[Node(
        package='t_parking', executable='rule_based_t_parking_node',
        name='rule_based_t_parking_node', output='screen',
        parameters=[pf, {
            # 노드가 bool 로 선언한 파라미터에 launch 문자열을 그대로 주면
            # InvalidParameterTypeException 이 난다. 반드시 캐스팅한다.
            'parking_side': ParameterValue(LaunchConfiguration('parking_side'),
                                           value_type=str),
            'auto_start': ParameterValue(LaunchConfiguration('auto_start'),
                                         value_type=bool),
            'direct_cmd_output': ParameterValue(
                LaunchConfiguration('direct_cmd_output'), value_type=bool),
        }],
    )])

    # ---------------- 6) (선택) 자동 시작 ----------------
    auto_trigger = TimerAction(
        period=LaunchConfiguration('start_delay'),
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-1', '/parking_start',
                 'std_msgs/msg/Bool', '{data: true}'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_on_launch')),
        )],
    )

    return LaunchDescription(args + [lidar, static_tf, imu, pcan, scan_filter,
                                     parking, auto_trigger])

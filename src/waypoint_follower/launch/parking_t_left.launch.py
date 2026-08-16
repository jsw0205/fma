"""Left-side T(perpendicular) reverse parking - t_parking package.

Starts ONLY what t_parking needs beyond what's already running in the main
stack: `scan_parking_filter` (re-filters the existing RPLiDAR S2 /scan into
a private /scan_parking with a vehicle-frame angle window) + `parking_bridge`'s
wheel_odom_pcan_node (CAN-feedback-only encoder+IMU odometry, enable_command_tx
stays false) + `rule_based_t_parking_node` itself (direct_cmd_output stays
false - control_arbiter relays /parking/cmd_* to CAN, same publish/relay
pattern as camera and GPS).

All of t_parking's own topics (/parking_start, /parking_active,
/parking_mapping, /parking_done, /parking/cmd_*, etc. - the package
hardcodes these as absolute names in source, not parameters, so `remappings`
is the only way to retarget them without editing the package) are remapped
under a `parking_t/` prefix so this can run alongside
parking_parallel_right.launch.py without the two parking nodes fighting over
the same topics - control_arbiter's "parking_left" event-zone type (as
opposed to "parking_right") reads from this prefix specifically. scan_topic/
odom_topic are real parameters, so those are overridden directly instead of
remapped.

Assumes the rest of the stack (RPLiDAR S2 -> /scan, taobotics IMU ->
/taobotics/sensor, control_arbiter, CAN up) is already running via
post_gps_drive.launch.py.

Trigger manually for the first tests:
    ros2 launch waypoint_follower parking_t_left.launch.py
    ros2 topic pub -1 /parking_t/parking_start std_msgs/msg/Bool "{data: true}"

See src/t_parking/README.md "실차 투입 전 반드시 할 것" before ever running
this with the drive wheels on the ground - LiDAR mount pose (laser_yaw/
laser_angle_sign), steering sign, and radius calibration all need real-
vehicle verification first.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Every absolute topic name t_parking/parallel_parking hardcode in source -
# both packages share the identical set (parallel_parking has a few extra
# t_parking got its own straight-out exit logic 2026-08-05 too (T-slot
# exit is just a forward drive, no S-curve needed - see
# rule_based_t_parking_node.py's PARKED/EXIT_STRAIGHT states), same
# /parking_exit_start,/parking_exit_done topics as parallel_parking.
_PARKING_TOPICS = [
    'parking_start', 'parking_reset', 'parking_active', 'parking_mapping',
    'parking_done', 'parking_request_stop', 'parking_status',
    'parking_exit_start', 'parking_exit_done',
]
_PARKING_SUBTOPICS = ['cmd_rpm', 'cmd_steer', 'cmd_enable', 'cones', 'goal_pose', 'markers']


def _remappings(prefix):
    remaps = [(f'/{t}', f'/{prefix}/{t}') for t in _PARKING_TOPICS]
    remaps += [(f'/parking/{t}', f'/{prefix}/{t}') for t in _PARKING_SUBTOPICS]
    return remaps


def generate_launch_description():
    pkg_share = get_package_share_directory('t_parking')
    # NOT just 'params_file' - see parking_parallel_right.launch.py's
    # identical comment (2026-08-12 bug, found live via `ps aux`):
    # DeclareLaunchArgument/LaunchConfiguration names are global across the
    # whole launch tree, so a plain 'params_file' here collided with
    # parking_parallel_right.launch.py's own arg of the same name once
    # post_gps_drive.launch.py included both - whichever got included
    # first silently won for both nodes.
    params_file = LaunchConfiguration('t_parking_params_file')
    prefix = 'parking_t'

    params_file_arg = DeclareLaunchArgument(
        't_parking_params_file',
        default_value=pkg_share + '/config/parking_left.yaml',
    )
    auto_start_arg = DeclareLaunchArgument('auto_start', default_value='false')
    # false when running the full stack (control_arbiter relays /parking/
    # cmd_* to CAN instead) - set true for isolated testing of this launch
    # file alone (no arbiter running to do that relay, so nothing would
    # otherwise reach CAN and the vehicle wouldn't move even with a
    # correctly-computed plan).
    direct_cmd_output_arg = DeclareLaunchArgument('direct_cmd_output', default_value='false')

    scan_filter = Node(
        package='t_parking',
        executable='scan_parking_filter',
        name='scan_parking_filter_t',
        output='log',
        parameters=[params_file, {
            'output_scan': f'/{prefix}/scan_parking',
        }],
    )

    # CAN-feedback-only (enable_command_tx defaults false, see
    # parking_bridge's module docstring) - reads 0x101/0x102, fuses with
    # the already-running taobotics IMU, publishes odom under this side's
    # own prefix (each side's wheel_odom_pcan_node still safely opens the
    # same can0 socket independently - SocketCAN allows multiple readers).
    odom_bridge = Node(
        package='parking_bridge',
        executable='wheel_odom_pcan_node',
        name='wheel_odom_pcan_node_t',
        output='log',
        parameters=[{
            'imu_topic': '/taobotics/sensor',
            'odom_topic': f'/{prefix}/wheel_odom',
            'odom_frame': f'{prefix}/odom',
            'base_frame': f'{prefix}/base_link',
            # direct_cmd_output on the parking node above publishes to
            # plain /cmd_rpm etc, but this node is what actually turns
            # that into a CAN frame - both need to agree, or commands get
            # computed/published correctly and just never reach CAN.
            'enable_command_tx': ParameterValue(
                LaunchConfiguration('direct_cmd_output'), value_type=bool),
        }],
    )

    parking_node = TimerAction(period=2.0, actions=[Node(
        package='t_parking',
        executable='rule_based_t_parking_node',
        name='rule_based_t_parking_node',
        output='log',
        parameters=[
            params_file,
            {
                'parking_side': 'left',
                'scan_topic': f'/{prefix}/scan_parking',
                'odom_topic': f'/{prefix}/wheel_odom',
                'auto_start': ParameterValue(
                    LaunchConfiguration('auto_start'), value_type=bool),
                'direct_cmd_output': ParameterValue(
                    LaunchConfiguration('direct_cmd_output'), value_type=bool),
            },
        ],
        remappings=_remappings(prefix),
    )])

    return LaunchDescription([
        params_file_arg,
        auto_start_arg,
        direct_cmd_output_arg,
        scan_filter,
        odom_bridge,
        parking_node,
    ])

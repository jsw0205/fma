"""Right-side parallel parking - parallel_parking package.

Same shape as parking_t_left.launch.py (see that file's docstring for the
full rationale) - starts scan_parking_filter + parking_bridge's
wheel_odom_pcan_node + rule_based_parallel_parking_node, reusing the
already-running RPLiDAR S2 (/scan) and taobotics IMU (/taobotics/sensor)
instead of spinning up duplicate sensor nodes. direct_cmd_output stays
false - control_arbiter relays /parking/cmd_* to CAN.

All topics remapped under a `parking_r/` prefix (parallel_parking has one
extra topic t_parking doesn't: /parking_parked - both now have
/parking_exit_start,/parking_exit_done, see t_parking's 2026-08-05 exit
logic) so this can run alongside
parking_t_left.launch.py in the same drive - a course with both a T-zone
and a parallel-zone needs both parking nodes up simultaneously, only one
actually triggered at a time depending on which zone the vehicle is in.
control_arbiter's "parking_right" event-zone type reads from this prefix.

Trigger manually for the first tests:
    ros2 launch waypoint_follower parking_parallel_right.launch.py
    ros2 topic pub -1 /parking_r/parking_start std_msgs/msg/Bool "{data: true}"

See src/parallel_parking/README.md "Safety" before ever running this with
the drive wheels on the ground.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_PARKING_TOPICS = [
    'parking_start', 'parking_reset', 'parking_active', 'parking_mapping',
    'parking_done', 'parking_request_stop', 'parking_status',
    # parallel_parking-only (exit maneuver after parking, t_parking has no
    # equivalent - see rule_based_parallel_parking_node.py).
    'parking_parked', 'parking_exit_start', 'parking_exit_done',
]
_PARKING_SUBTOPICS = ['cmd_rpm', 'cmd_steer', 'cmd_enable', 'cones', 'goal_pose', 'markers']


def _remappings(prefix):
    remaps = [(f'/{t}', f'/{prefix}/{t}') for t in _PARKING_TOPICS]
    remaps += [(f'/parking/{t}', f'/{prefix}/{t}') for t in _PARKING_SUBTOPICS]
    return remaps


def generate_launch_description():
    pkg_share = get_package_share_directory('parallel_parking')
    # NOT just 'params_file' - see parking_t_left.launch.py's identical
    # arg name (2026-08-12 bug, found live): both launch files used to
    # declare a plain 'params_file' argument, and since post_gps_drive.
    # launch.py includes both in the same parent launch tree,
    # DeclareLaunchArgument/LaunchConfiguration share one global namespace
    # across the whole tree - whichever gets included first "wins" and the
    # second file's own default is silently ignored, so this node was
    # loading t_parking's parking_left.yaml instead of its own config the
    # entire time (confirmed via `ps aux` showing the actual --params-file
    # argument the process was launched with). Scoped, package-specific
    # name here so this can never collide with another parking package's
    # own params_file argument again.
    params_file = LaunchConfiguration('parallel_parking_params_file')
    prefix = 'parking_r'

    params_file_arg = DeclareLaunchArgument(
        'parallel_parking_params_file',
        default_value=pkg_share + '/config/parking_parallel_right.yaml',
    )
    auto_start_arg = DeclareLaunchArgument('auto_start', default_value='false')
    auto_exit_arg = DeclareLaunchArgument('auto_exit', default_value='false')
    # false when running the full stack (control_arbiter relays /parking/
    # cmd_* to CAN instead) - set true for isolated testing of this launch
    # file alone, see parking_t_left.launch.py's identical arg.
    direct_cmd_output_arg = DeclareLaunchArgument('direct_cmd_output', default_value='false')

    # t_parking's filter node is reused as-is (parallel_parking depends on
    # the t_parking package already, see package.xml) - same /scan ->
    # /scan_parking job, just a different keep_min_deg/keep_max_deg window
    # per config file (right-side vs left-side).
    scan_filter = Node(
        package='t_parking',
        executable='scan_parking_filter',
        name='scan_parking_filter_r',
        output='log',
        parameters=[params_file, {
            'output_scan': f'/{prefix}/scan_parking',
        }],
    )

    odom_bridge = Node(
        package='parking_bridge',
        executable='wheel_odom_pcan_node',
        name='wheel_odom_pcan_node_r',
        output='log',
        parameters=[{
            'imu_topic': '/taobotics/sensor',
            'odom_topic': f'/{prefix}/wheel_odom',
            'odom_frame': f'{prefix}/odom',
            'base_frame': f'{prefix}/base_link',
            # See parking_t_left.launch.py's identical comment - this is
            # what actually turns direct_cmd_output's /cmd_rpm etc into a
            # CAN frame.
            'enable_command_tx': ParameterValue(
                LaunchConfiguration('direct_cmd_output'), value_type=bool),
        }],
    )

    parking_node = TimerAction(period=2.0, actions=[Node(
        package='parallel_parking',
        executable='rule_based_parallel_parking_node',
        name='rule_based_parallel_parking_node',
        output='log',
        parameters=[
            params_file,
            {
                'parking_side': 'right',
                'imu_topic': '/taobotics/sensor',
                'scan_topic': f'/{prefix}/scan_parking',
                'odom_topic': f'/{prefix}/wheel_odom',
                'auto_start': ParameterValue(
                    LaunchConfiguration('auto_start'), value_type=bool),
                'auto_exit': ParameterValue(
                    LaunchConfiguration('auto_exit'), value_type=bool),
                'direct_cmd_output': ParameterValue(
                    LaunchConfiguration('direct_cmd_output'), value_type=bool),
            },
        ],
        remappings=_remappings(prefix),
    )])

    return LaunchDescription([
        params_file_arg,
        auto_start_arg,
        auto_exit_arg,
        direct_cmd_output_arg,
        scan_filter,
        odom_bridge,
        parking_node,
    ])

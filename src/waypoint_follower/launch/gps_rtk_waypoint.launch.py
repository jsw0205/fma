from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    waypoints_file_arg = DeclareLaunchArgument(
        "waypoints_file",
        default_value="/home/a/ros2_ws/src/waypoint_follower/waypoints/path_20260728_164603.csv",
    )
    enable_control_arg = DeclareLaunchArgument("enable_control", default_value="false")
    cruise_rpm_arg = DeclareLaunchArgument("cruise_rpm", default_value="140")

    # GPS + RTK stack: ublox_gps_node + rtk_bridge.py (proven combo, see
    # f9p_bringup/launch/f9p_rover.launch.py) + gps_to_odom + static_tf.
    # Included rather than duplicated so there's one source of truth for it.
    f9p_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("f9p_bringup") + "/launch/f9p_rover.launch.py"
        )
    )

    waypoint_follower_node = Node(
        package="waypoint_follower",
        executable="waypoint_follower_node",
        name="waypoint_follower_node",
        output="screen",
        parameters=[{
            "gps_topic": "/ublox_gps_node/fix",
            "waypoints_file": LaunchConfiguration("waypoints_file"),
            # LaunchConfiguration always resolves to a STRING - without
            # ParameterValue(value_type=...) this crashes the node
            # instantly (InvalidParameterTypeException) against enable_control's
            # BOOL / cruise_rpm's INT declared types. Same trap fixed in
            # integrated_drive.launch.py/post_gps_drive.launch.py earlier
            # this session - missed here until 2026-07-29 (this file wasn't
            # touched in that pass), confirmed as the actual cause of
            # waypoint_follower_node crashing silently (nothing plotted at
            # all) when run with enable_control:=true.
            "enable_control": ParameterValue(
                LaunchConfiguration("enable_control"), value_type=bool
            ),
            "cruise_rpm": ParameterValue(LaunchConfiguration("cruise_rpm"), value_type=int),
        }],
    )

    mpl_viz_node = Node(
        package="waypoint_follower",
        executable="mpl_viz_node",
        name="mpl_viz_node",
        output="screen",
    )

    return LaunchDescription([
        waypoints_file_arg,
        enable_control_arg,
        cruise_rpm_arg,
        f9p_bringup_launch,
        waypoint_follower_node,
        mpl_viz_node,
    ])

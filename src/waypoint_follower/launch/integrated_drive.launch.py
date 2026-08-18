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
    weights_arg = DeclareLaunchArgument(
        "weights", default_value="/home/a/ros2_ws/src/zed_camera/weights/yolopv2.pt"
    )
    camera_mode_rpm_arg = DeclareLaunchArgument("camera_mode_rpm", default_value="130.0")
    # int-formatted (no decimal point) on purpose - yolopv2_zed_rpm_node.py
    # declares can_target_rpm with an int default (0), so casting the
    # LaunchConfiguration string via ParameterValue(value_type=int) needs a
    # string int() can actually parse ("130", not "130.0" - int("130.0")
    # raises). Kept as a separate arg from camera_mode_rpm (which IS
    # "130.0", float-typed for arbiter's own camera_mode_rpm param) rather
    # than reusing it, to avoid that trap. Should track the same intended
    # cruise value as camera_mode_rpm if that's ever tuned.
    camera_can_target_rpm_arg = DeclareLaunchArgument("camera_can_target_rpm", default_value="130")
    cruise_rpm_arg = DeclareLaunchArgument("cruise_rpm", default_value="140")
    # 2026-08-18: see post_gps_drive.launch.py's matching comment - live
    # field tuning, default lowered 1.5->1.2 after today's testing found
    # the vehicle turning too early on corners.
    curve_lead_margin_arg = DeclareLaunchArgument("curve_lead_margin", default_value="1.2")
    traffic_light_model_arg = DeclareLaunchArgument(
        "traffic_light_model",
        default_value="/home/a/ros2_ws/src/traffic_light/weights/best.pt",
    )
    # ZED camera driver (publishes /zed/zed_node/rgb/..., depth/..., odom -
    # yolopv2_zed_node below just subscribes to these, doesn't touch the
    # camera directly).
    #
    # NOTE (2026-07-29): the ZED causes real USB3 EMI that degrades GPS
    # fix quality when both run together (confirmed - see README "USB3-EMI
    # concern"), and zed_wrapper permanently dies if the camera isn't
    # plugged in within ~28s of starting with no auto-respawn. If you need
    # GPS to get a clean Fixed/Float before the ZED's USB3 link comes up,
    # don't use this combined launch file - start GPS standalone first
    # (`f9p_bringup f9p_rover.launch.py`), then once it has a fix, run
    # `waypoint_follower post_gps_drive.launch.py` for everything else
    # (ZED/camera/traffic_light/arbiter/mpl_viz). This file is for the
    # simple "start everything together" case.
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("zed_wrapper") + "/launch/zed_camera.launch.py"
        ),
        launch_arguments={"camera_model": "zed2i"}.items(),
    )

    # GPS + RTK stack (ublox_gps_node + rtk_bridge.py + odometry/TF).
    f9p_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("f9p_bringup") + "/launch/f9p_rover.launch.py"
        )
    )

    # GPS/waypoint follower: publishes gps_control/{steer_deg,rpm,target_idx,
    # valid} for the arbiter instead of writing CAN itself
    # (publish_can_directly=false) - the arbiter below is the only thing
    # that talks to CAN.
    waypoint_follower_node = Node(
        package="waypoint_follower",
        executable="waypoint_follower_node",
        name="waypoint_follower_node",
        output="screen",
        parameters=[{
            "gps_topic": "/ublox_gps_node/fix",
            "waypoints_file": LaunchConfiguration("waypoints_file"),
            # Same STRING-vs-BOOL trap as camera_mode_rpm above.
            "enable_control": ParameterValue(
                LaunchConfiguration("enable_control"), value_type=bool
            ),
            "cruise_rpm": ParameterValue(LaunchConfiguration("cruise_rpm"), value_type=int),
            "curve_lead_margin": ParameterValue(
                LaunchConfiguration("curve_lead_margin"), value_type=float
            ),
            "publish_can_directly": False,
        }],
    )

    # Camera node (yolopv2_zed_node): can_enable MUST stay false - the
    # arbiter is the only thing that talks to CAN. Publishes
    # ~/steering_deg (-> /yolopv2_zed_node/steering_deg), which is what
    # control_arbiter's camera_steer_topic already points at by default.
    # auto_speed/can_target_rpm (2026-08-17): rpm_target computation runs
    # unconditionally now regardless of can_enable (see
    # yolopv2_zed_rpm_node.py's publish-timer comment) and is published on
    # ~/rpm_target for control_arbiter's camera_rpm_topic to consume -
    # can_target_rpm needs a real base (default is 0, which would make
    # _speed_for_steer always return 0) so it's set to match
    # camera_mode_rpm here, same "straight" cruise value either arbiter
    # falls back to.
    camera_node = Node(
        package="zed_camera",
        executable="yolopv2_zed_rpm_node",  # newer LaneTracker-based version
        name="yolopv2_zed_node",
        output="screen",
        parameters=[{
            "can_enable": False,
            "weights": LaunchConfiguration("weights"),
            "auto_speed": True,
            "can_target_rpm": ParameterValue(
                LaunchConfiguration("camera_can_target_rpm"), value_type=int
            ),
        }],
    )

    # Traffic light node (OAK-D + YOLO, package: traffic_light). Publishes
    # /traffic_light (GO/STOP), consumed by control_arbiter's
    # "traffic_light" event zones below. Requires depthai + ultralytics
    # (pip) and an OAK-D physically connected - if either is missing the
    # node just idles/logs a warning instead of crashing (see
    # traffic_light_node.py), so it's safe to leave in this launch file
    # even before the camera is hooked up.
    traffic_light_node = Node(
        package="traffic_light",
        executable="test_sunny_node",
        name="traffic_light_node",
        output="screen",
        parameters=[{
            "model_path": LaunchConfiguration("traffic_light_model"),
        }],
    )

    # ------------------------------------------------------------------
    # Event zones: "idx_start:idx_end:type" strings, type is "stop",
    # "gps_priority", "avoid" (placeholder until LiDAR avoidance is
    # built), or "traffic_light" (idx_end doubles as the stop-line idx -
    # see arbiter_node.py). idx is the row number (0-based, header
    # excluded) in whatever CSV waypoints_file above points at. Find idx
    # for a real-world spot either by reading the CSV directly, or by
    # driving to that spot with just the GPS node running and reading the
    # printed target_idx=... value.
    #
    # Uncomment and fill in as needed, e.g.:
    #   EVENT_ZONES = [
    #       "40:42:stop",           # forced stop (e.g. a known intersection)
    #       "60:70:gps_priority",   # GPS drives this stretch, not the camera
    #       "80:85:avoid",          # obstacle avoidance zone (placeholder)
    #       "90:95:traffic_light",  # slow on red idx 90-94, stop at 95 if still red
    #   ]
    # ------------------------------------------------------------------
    EVENT_ZONES = [""]  # empty = no event zones active (obstacle_avoid_node not running right now)

    arbiter_node = Node(
        package="waypoint_follower",
        executable="arbiter_node",
        name="control_arbiter",
        output="screen",
        parameters=[{
            "event_zones": EVENT_ZONES,
            # LaunchConfiguration always resolves to a string - without
            # ParameterValue(value_type=float) it gets passed as a STRING
            # to a parameter declared as a DOUBLE (camera_mode_rpm=100.0
            # in arbiter_node.py), which rclpy rejects at startup
            # (InvalidParameterTypeException) - confirmed this crashed the
            # node instantly (exit code 1, <1s after launch).
            "camera_mode_rpm": ParameterValue(
                LaunchConfiguration("camera_mode_rpm"), value_type=float
            ),
            # "camera_steer_topic": "/yolopv2_zed_node/steering_deg",
            # "camera_timeout_sec": 1.0,
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
        weights_arg,
        camera_mode_rpm_arg,
        camera_can_target_rpm_arg,
        cruise_rpm_arg,
        curve_lead_margin_arg,
        traffic_light_model_arg,
        f9p_bringup_launch,
        zed_wrapper_launch,
        waypoint_follower_node,
        camera_node,
        traffic_light_node,
        arbiter_node,
        mpl_viz_node,
    ])

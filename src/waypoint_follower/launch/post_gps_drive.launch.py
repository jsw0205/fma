from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Everything EXCEPT GPS+RTK: ZED, camera, traffic_light,
    waypoint_follower_node, control_arbiter, mpl_viz.

    Use this after starting GPS standalone
    (`ros2 launch f9p_bringup f9p_rover.launch.py`) and letting it reach
    Float/Fixed on its own first - avoids the ZED's confirmed USB3 EMI
    (see README "USB3-EMI concern") degrading the fix while it's still
    converging, and avoids ever having to kill/restart the GPS node (which
    is unnecessary anyway - the u-blox receiver keeps tracking
    independently of whether a host has the serial port open, so there's
    no fix to lose by leaving it running across a restart of everything
    else). waypoint_follower_node subscribes to GPS's /ublox_gps_node/fix
    topic like normal, it doesn't care which launch file started GPS.

    For the "just start everything together" case, use
    integrated_drive.launch.py instead.
    """
    waypoints_file_arg = DeclareLaunchArgument(
        "waypoints_file",
        default_value="/home/a/ros2_ws/src/waypoint_follower/waypoints/path_20260728_164603.csv",
    )
    enable_control_arg = DeclareLaunchArgument("enable_control", default_value="false")
    loop_waypoints_arg = DeclareLaunchArgument("loop_waypoints", default_value="false")
    weights_arg = DeclareLaunchArgument(
        "weights", default_value="/home/a/ros2_ws/src/zed_camera/weights/yolopv2.pt"
    )
    camera_mode_rpm_arg = DeclareLaunchArgument("camera_mode_rpm", default_value="130.0")
    # int-formatted on purpose - see integrated_drive.launch.py's matching
    # comment (int("130.0") would raise, can_target_rpm is int-typed).
    camera_can_target_rpm_arg = DeclareLaunchArgument("camera_can_target_rpm", default_value="130")
    cruise_rpm_arg = DeclareLaunchArgument("cruise_rpm", default_value="140")
    traffic_light_model_arg = DeclareLaunchArgument(
        "traffic_light_model",
        default_value="/home/a/ros2_ws/src/traffic_light/weights/best.pt",
    )
    # cv2.imshow debug window - off by default (headless), see
    # traffic_light_node.py.
    traffic_light_show_debug_arg = DeclareLaunchArgument(
        "traffic_light_show_debug", default_value="true"
    )
    traffic_light_conf_threshold_arg = DeclareLaunchArgument(
        "traffic_light_conf_threshold", default_value="0.2"
    )
    traffic_light_manual_exposure_arg = DeclareLaunchArgument(
        "traffic_light_manual_exposure", default_value="false"
    )
    # parking_t_left.launch.py/parking_parallel_right.launch.py started
    # alongside this file - both idle until an event_zones "parking_left"/
    # "parking_right" entry triggers them (see arbiter_node.py's
    # _handle_parking_zone). Toggle off if a course has no parking zones,
    # or if only one side is ever used on this course.
    enable_parking_left_arg = DeclareLaunchArgument("enable_parking_left", default_value="true")
    enable_parking_right_arg = DeclareLaunchArgument("enable_parking_right", default_value="true")
    # rpm used while GPS drives straight through a parking zone's mapping
    # phase - matches that side's own pre_straight_rpm (its cone/slot
    # detection was calibrated at this speed), not GPS's normal cruise rpm.
    # Defaults match each package's own config (t_parking=30,
    # parallel_parking=20).
    parking_left_approach_rpm_arg = DeclareLaunchArgument(
        "parking_left_approach_rpm", default_value="30.0"
    )
    parking_right_approach_rpm_arg = DeclareLaunchArgument(
        "parking_right_approach_rpm", default_value="30.0"
    )
    # RPLiDAR S2 + taobotics IMU + obstacle_avoidance (2026-08-05): needed
    # by both the parking packages (scan_parking_filter/wheel_odom_pcan_node
    # need real /scan+/taobotics/sensor, not just idle nodes with nothing to
    # read) and by any "avoid" event_zones entry. Previously always started
    # separately by hand (see README "individually") - folded in here so a
    # course exercising traffic_light+parking+avoid together needs only
    # this one launch file. Toggle off individually if running without that
    # hardware attached (e.g. indoors, camera-only testing).
    enable_lidar_arg = DeclareLaunchArgument("enable_lidar", default_value="true")
    enable_imu_arg = DeclareLaunchArgument("enable_imu", default_value="true")
    enable_obstacle_avoid_arg = DeclareLaunchArgument(
        "enable_obstacle_avoid", default_value="true"
    )
    imu_serial_port_arg = DeclareLaunchArgument(
        "imu_serial_port",
        default_value=(
            "/dev/serial/by-id/usb-Silicon_Labs_HandsFree_IMU_USB_to_UART_Bridge_"
            "Controller_0001-if00-port0"
        ),
    )

    # ZED camera driver. Make sure it's physically plugged in before
    # running this launch file - zed_wrapper retries opening the camera
    # 5x (~28s total) then dies for good with no auto-respawn.
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("zed_wrapper") + "/launch/zed_camera.launch.py"
        ),
        launch_arguments={"camera_model": "zed2i"}.items(),
    )

    # GPS/waypoint follower: publishes gps_control/{steer_deg,rpm,target_idx,
    # valid} for the arbiter instead of writing CAN itself
    # (publish_can_directly=false) - the arbiter below is the only thing
    # that talks to CAN. Reads GPS fixes from whatever f9p_bringup launch
    # is already running externally.
    waypoint_follower_node = Node(
        package="waypoint_follower",
        executable="waypoint_follower_node",
        name="waypoint_follower_node",
        output="log",
        parameters=[{
            "gps_topic": "/ublox_gps_node/fix",
            "waypoints_file": LaunchConfiguration("waypoints_file"),
            "enable_control": ParameterValue(
                LaunchConfiguration("enable_control"), value_type=bool
            ),
            # Same STRING-vs-typed trap as camera_mode_rpm - cruise_rpm is
            # declared as an int in waypoint_follower_node.
            "cruise_rpm": ParameterValue(LaunchConfiguration("cruise_rpm"), value_type=int),
            "publish_can_directly": False,
            "loop_waypoints": ParameterValue(
                LaunchConfiguration("loop_waypoints"), value_type=bool
            ),
        }],
    )

    # Camera node (yolopv2_zed_node): can_enable MUST stay false - the
    # arbiter is the only thing that talks to CAN. auto_speed/
    # can_target_rpm (2026-08-17): see integrated_drive.launch.py's
    # matching comment - rpm_target now published on ~/rpm_target
    # regardless of can_enable, for control_arbiter's camera_rpm_topic.
    camera_node = Node(
        package="zed_camera",
        executable="yolopv2_zed_rpm_node",
        name="yolopv2_zed_node",
        output="log",
        parameters=[{
            "can_enable": False,
            "weights": LaunchConfiguration("weights"),
            "auto_speed": True,
            "can_target_rpm": ParameterValue(
                LaunchConfiguration("camera_can_target_rpm"), value_type=int
            ),
        }],
    )

    # Traffic light node (OAK-D + YOLO, package: traffic_light). Idles
    # harmlessly if deps/hardware aren't present - see traffic_light_node.py.
    traffic_light_node = Node(
        package="traffic_light",
        executable="test_sunny_node",
        name="traffic_light_node",
        output="log",
        parameters=[{
            "model_path": LaunchConfiguration("traffic_light_model"),
            "show_debug": ParameterValue(
                LaunchConfiguration("traffic_light_show_debug"), value_type=bool
            ),
            "conf_threshold": ParameterValue(
                LaunchConfiguration("traffic_light_conf_threshold"), value_type=float
            ),
            "use_manual_exposure": ParameterValue(
                LaunchConfiguration("traffic_light_manual_exposure"), value_type=bool
            ),
        }],
    )

    # Same EVENT_ZONES format as integrated_drive.launch.py - see that
    # file's comment for the "start:end:type" spec.
    # 2026-08-12: re-recorded course on ~/recorded_waypoints.csv (201
    # waypoints, target_idx 0-200) - traffic_light -> gps_priority (camera
    # excluded before T-parking search) -> T-parking (left) ->
    # gps_priority_slow (GPS-only transit from T-parking exit to the
    # parallel slot, capped at gps_priority_slow_rpm=80 instead of full
    # cruise - see arbiter_node.py) -> parallel parking (right) -> obstacle
    # avoidance through to the end of the course. idx numbers this time
    # were given directly as 0-based target_idx (confirmed via the
    # waypoint_idx_viewer artifact, not the recorder's 1-based "Waypoint N"
    # log lines - no -1 conversion needed here, unlike the 2026-08-05 course).
    # 2026-08-16: temporarily emptied - a brand new course was just
    # recorded to ~/recorded_waypoints.csv and this list's idx ranges are
    # all from the OLD ~201-waypoint course above. Left in a comment below
    # rather than deleted so it's easy to bring back once the new course's
    # zone idx are known (walk it with the waypoint_idx_viewer artifact
    # again, same as last time). With EVENT_ZONES empty, control_arbiter
    # just falls through to its normal camera>GPS priority the whole
    # course - exactly "GPS+카메라만" as requested.
    # [""] not [] - arbiter_node.py's own event_zones param default is [""]
    # and it filters blank entries itself (see its declare_parameter call);
    # a truly empty [] breaks ROS2 parameter type inference at launch time
    # ("Expected 'value' to be one of [...], but got '()' of type 'tuple'"
    # - confirmed live 2026-08-16, this is why control_arbiter didn't even
    # start last run).
    # 2026-08-18: 언덕정지 자리 테스트용 - idx 44에서 3초 정지 후 자동 재출발
    # (진짜 Hill_Stop, stop_mode 전환은 아직 미구현 - 그냥 타이밍만 시험).
    # '44:44:stop:3' 형식: start:end:type:hold_sec (hold_sec 있으면 그 시간
    # 지난 뒤 base_steer/base_rpm으로 자동 재개, 없으면 예전처럼 무한정지).
    EVENT_ZONES = ["44:44:stop:3"]
    # EVENT_ZONES = [""]
    # EVENT_ZONES = [
    #     "10:29:traffic_light:18",
    #     "30:37:gps_priority",
    #     "38:89:parking_left",
    #     "90:114:gps_priority_slow",
    #     "115:142:parking_right",
    #     "143:200:avoid",
    # ]

    arbiter_node = Node(
        package="waypoint_follower",
        executable="arbiter_node",
        name="control_arbiter",
        output="log",
        parameters=[{
            "event_zones": EVENT_ZONES,
            "camera_mode_rpm": ParameterValue(
                LaunchConfiguration("camera_mode_rpm"), value_type=float
            ),
            "parking_left_approach_rpm": ParameterValue(
                LaunchConfiguration("parking_left_approach_rpm"), value_type=float
            ),
            "parking_right_approach_rpm": ParameterValue(
                LaunchConfiguration("parking_right_approach_rpm"), value_type=float
            ),
        }],
    )

    mpl_viz_node = Node(
        package="waypoint_follower",
        executable="mpl_viz_node",
        name="mpl_viz_node",
        output="log",
    )

    # Only thing that actually prints to the terminal - see
    # status_monitor_node.py's docstring. Every other node's own log
    # output above is set to output="log"/'log' (file only, ~/.ros/log/)
    # instead of "screen" so this one clean line is all that shows up.
    status_monitor_node = Node(
        package="waypoint_follower",
        executable="status_monitor_node",
        name="status_monitor_node",
        output="screen",
    )

    # Idle until event_zones triggers the matching side - see arbiter_node.
    # py's module docstring ("parking_left"/"parking_right" zones) and
    # parking_t_left.launch.py's docstring for why both run the whole time
    # instead of being started on demand.
    parking_left_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("waypoint_follower") + "/launch/parking_t_left.launch.py"
        ),
        condition=IfCondition(LaunchConfiguration("enable_parking_left")),
    )
    parking_right_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("waypoint_follower")
            + "/launch/parking_parallel_right.launch.py"
        ),
        condition=IfCondition(LaunchConfiguration("enable_parking_right")),
    )

    # RPLiDAR S2 (by-id serial path, see obstacle_avoidance/launch/
    # rplidar_s2.launch.py's own comment for why not the default port).
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("obstacle_avoidance") + "/launch/rplidar_s2.launch.py"
        ),
        condition=IfCondition(LaunchConfiguration("enable_lidar")),
    )

    # taobotics HFI IMU - /taobotics/sensor, consumed by obstacle_avoid_node
    # (AVOID/RETURN yaw progress) and parking_bridge's odom fusion.
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("mrpt_sensor_imu_taobotics")
            + "/launch/mrpt_sensor_imu_taobotics.launch.py"
        ),
        launch_arguments={"serial_port": LaunchConfiguration("imu_serial_port")}.items(),
        condition=IfCondition(LaunchConfiguration("enable_imu")),
    )

    # obstacle_avoid_node itself - only actually takes over from GPS/camera
    # inside an "avoid" event_zones entry (arbiter arms/disarms it via
    # /can_bridge/enable), otherwise idles in CLEAR. write_can_directly is
    # already false in its own config (obstacle_avoid.yaml) - arbiter is
    # still the only CAN writer.
    obstacle_avoid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("obstacle_avoidance") + "/launch/obstacle_avoid.launch.py"
        ),
        condition=IfCondition(LaunchConfiguration("enable_obstacle_avoid")),
    )

    return LaunchDescription([
        waypoints_file_arg,
        enable_control_arg,
        loop_waypoints_arg,
        weights_arg,
        camera_mode_rpm_arg,
        camera_can_target_rpm_arg,
        cruise_rpm_arg,
        traffic_light_model_arg,
        traffic_light_show_debug_arg,
        traffic_light_conf_threshold_arg,
        traffic_light_manual_exposure_arg,
        enable_parking_left_arg,
        enable_parking_right_arg,
        parking_left_approach_rpm_arg,
        parking_right_approach_rpm_arg,
        enable_lidar_arg,
        enable_imu_arg,
        enable_obstacle_avoid_arg,
        imu_serial_port_arg,
        zed_wrapper_launch,
        waypoint_follower_node,
        camera_node,
        traffic_light_node,
        arbiter_node,
        mpl_viz_node,
        status_monitor_node,
        lidar_launch,
        imu_launch,
        obstacle_avoid_launch,
        parking_left_launch,
        parking_right_launch,
    ])

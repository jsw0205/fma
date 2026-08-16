from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Thin wrapper around rplidar_ros's own rplidar_s2_launch.py - just pins
# serial_port to the by-id path instead of the default /dev/ttyUSB0, which
# on this vehicle is the AURIX steering board, not the LiDAR (the RPLiDAR
# S2 enumerates as /dev/ttyUSB1 - by-id avoids depending on that ordering,
# same reasoning as the GPS by-id symlink used elsewhere in this workspace).

LIDAR_BY_ID = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
    "56d802b9c06def11b21ddec2c169b110-if00-port0"
)


def generate_launch_description():
    # rplidar_s2_launch.py's own default is DenseBoost - exposed here as a
    # passthrough arg (2026-08-11) since our wrapper previously swallowed
    # it silently (IncludeLaunchDescription only forwards args explicitly
    # listed below, so `scan_mode:=Standard` on this wrapper's own command
    # line used to be a no-op). Standard mode has fewer points/lower
    # near-field density than DenseBoost - useful to try if DenseBoost's
    # health-status-2 dropouts are suspected to be point-rate related (see
    # README's RPLiDAR S2 troubleshooting notes).
    scan_mode_arg = DeclareLaunchArgument("scan_mode", default_value="DenseBoost")
    return LaunchDescription([
        scan_mode_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("rplidar_ros") + "/launch/rplidar_s2_launch.py"
            ),
            launch_arguments={
                "serial_port": LIDAR_BY_ID,
                "scan_mode": LaunchConfiguration("scan_mode"),
            }.items(),
        ),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    jetson_port = LaunchConfiguration(
        "jetson_port", default="/dev/ttyACM0"
    )
    rplidar_port = LaunchConfiguration(
        "rplidar_port",
        default="/dev/ttyUSB0",
    )
    robot_localization_file_path = get_package_share_directory("robot-bringup")
    # share_dir = get_package_share_directory("mpu6050driver")
    remappings = [("/odometry/filtered", "/odometry/filtered"), ("/odom", "/odom")]

    _mpu6050 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('mpu6050driver'),'launch','mpu6050driver_launch.py'
                )])
    )
    _rpLidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('robot-bringup'),'launch','rp_lidar.launch.py'
                )])
    )
    return LaunchDescription(
        [
            _mpu6050, 

            DeclareLaunchArgument(
                "jetson_port",
                default_value=jetson_port,
                description="Serial port for communication w microcontroller",
            ),
            _rpLidar,
            Node(
                package="robot-bringup",
                executable="jetson_robot_control2.py",

                # executable="robot_Kinematics.py",
                # parameters=[{"jetson_port": jetson_port}],
                arguments=[],
                output="screen",
            ),
            # static transform for laser scan
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0.0", "0", "0.18", "0", "0", "0", "base_link", "laser"],
                output="screen",
            ),
            # static transform from link to footprint
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0",
                    "0",
                    "-0.05",
                    "0",
                    "0",
                    "0",
                    "base_link",
                    "base_footprint",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0.0", "0", "0.0", "0", "0", "0", "map", "odom"],
                output="screen",
            ),
            Node(
                package = 'robot_localization',
                executable = 'ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[os.path.join(robot_localization_file_path, 'config', 'ekf.yaml'),
                {'use_sim_time': False}],
                remappings=remappings,
            )
        ]
    )

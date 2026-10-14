"""Launch the C++ viewer with one central YAML configuration."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")
    default_config = PathJoinSubstitution([FindPackageShare("rm_auto_aim_sim"), "config", "sim.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        Node(package="rm_auto_aim_sim", executable="rm_auto_aim_sim", name="rm_auto_aim_sim",
             arguments=["--config", config], output="screen", emulate_tty=True),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    local_ip = LaunchConfiguration("local_ip")
    dog_ip = LaunchConfiguration("dog_ip")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="d1w"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("agibot_d1w_driver"), "config", "driver.yaml"
            ]),
        ),
        DeclareLaunchArgument("local_ip", default_value="192.168.168.120"),
        DeclareLaunchArgument("dog_ip", default_value="192.168.168.168"),
        Node(
            package="agibot_d1w_driver",
            executable="agibot_d1w_driver_node",
            name="driver",
            namespace=namespace,
            output="screen",
            parameters=[
                params_file,
                {"local_ip": local_ip, "dog_ip": dog_ip},
            ],
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = "surface_follower"
    pkg_share = FindPackageShare(package=pkg).find(pkg)
    cfg_file = PathJoinSubstitution([pkg_share, "config", "surface_follower.yaml"])
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package=pkg,
            executable="surface_follower_node",
            name="surface_follower_node",
            # parameters=[cfg_file, {"use_sim_time": use_sim_time}],
            parameters=[{"use_sim_time": False}],
            output="screen"
        )
    ])

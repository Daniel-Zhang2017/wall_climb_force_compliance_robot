from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = "surface_follower"
    pkg_share = FindPackageShare(package=pkg).find(pkg)

    # 配置文件路径
    cfg_file = PathJoinSubstitution([pkg_share, "config", "surface_follower.yaml"])
    rviz_cfg = PathJoinSubstitution([pkg_share, "config", "sf_demo.rviz"])

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        # static TF: base_link -> ft_sensor_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'ft_sensor_link'],
            output='screen'
        ),

        # surface follower node, load yaml only
        Node(
            package="surface_follower",
            executable="surface_follower_node2",
            name="surface_follower_node2",
            parameters=[cfg_file],
            output="screen",
        ),

        # rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg],
            output="screen"
        )
    ])

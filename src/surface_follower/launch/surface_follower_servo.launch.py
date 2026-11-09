from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="moveit_resources_panda_moveit_config")
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    pkg_share = FindPackageShare(package="surface_follower").find("surface_follower")
    servo_param_file = PathJoinSubstitution([pkg_share, "config", "servo_panda.yaml"])
    follower_param_file = PathJoinSubstitution([pkg_share, "config", "surface_follower.yaml"])

    # world -> panda_link0 static TF
    static_tf_world_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--qx", "0.0",
            "--qy", "0.0",
            "--qz", "0.0",
            "--qw", "1.0",
            "--frame-id", "world",
            "--child-frame-id", "panda_link0"
        ],
        output="screen",
    )

    # ft_sensor_link attached to panda_link8
    static_tf_sensor = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.05",
            "--qx", "0.0",
            "--qy", "0.0",
            "--qz", "0.0",
            "--qw", "1.0",
            "--frame-id", "panda_link8",
            "--child-frame-id", "ft_sensor_link"
        ],
        output="screen",
    )

    container = ComposableNodeContainer(
        name="servo_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    moveit_config.to_dict(),
                    servo_param_file,
                ],
            ),
        ],
        output="screen",
    )

    follower_node = Node(
        package="surface_follower",
        executable="surface_follower_node3",
        name="surface_follower_node3",
        remappings=[
            ("/ft_sensor/wrench", "/franka_robot_state/wrench")
        ],
        parameters=[follower_param_file],
        output="screen",
    )

    return LaunchDescription([
        static_tf_world_base,
        static_tf_sensor,
        container,
        follower_node
    ])

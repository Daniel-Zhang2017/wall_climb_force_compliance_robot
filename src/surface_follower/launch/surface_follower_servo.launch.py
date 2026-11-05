from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory("surface_follower")
    servo_config_path = os.path.join(pkg, "config", "servo_panda.yaml")
    follower_config_path = os.path.join(pkg, "config", "surface_follower.yaml")

    # 加载 moveit_resources_panda 的全套URDF/SRDF/kinematics
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_config_path,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ],
        output="screen"
    )

    follower_node = Node(
        package="surface_follower",
        executable="surface_follower_node3",
        name="surface_follower_node3",
        parameters=[follower_config_path],
        output="screen"
    )

    return LaunchDescription([
        servo_node,
        follower_node
    ])

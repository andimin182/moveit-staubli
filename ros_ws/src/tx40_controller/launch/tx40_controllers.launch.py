from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robot_description = ParameterValue(Command(
        [
            "xacro ", 
            os.path.join(get_package_share_directory("tx40_simulation"), "urdf", "tx40.urdf.xacro")
        ]
    ),
        value_type=str,
        )

    robot_controllers = os.path.join(get_package_share_directory("tx40_controller"), "config", "tx40_controllers.yaml")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_controllers
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file", robot_controllers
        ]
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file", robot_controllers
        ]
    )

    return LaunchDescription([
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    is_sim = LaunchConfiguration("is_sim")

    moveit_config = (
        MoveItConfigsBuilder("tx40", 
                                         package_name="tx40_moveit")
                                         .robot_description(file_path= os.path.join(get_package_share_directory("tx40_simulation"), "urdf", "tx40.urdf.xacro"))
                                         .robot_description_semantic(file_path="config/tx40.srdf")
                                         .trajectory_execution(file_path="config/moveit_controllers.yaml")
                                         .to_moveit_configs()
                                         
                                         )
    
    moveit_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), 
                    {"use_sim_time": is_sim},
                    {"publish_robot_description_semantic": True}],

        arguments= [
            "--ros-args", "--log-level", "info"
        ]
    )

    rviz_config = os.path.join(get_package_share_directory("tx40_moveit"), "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d", rviz_config
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ]

    )

    return LaunchDescription([
        is_sim_arg,
        moveit_node,
        rviz_node
    ])
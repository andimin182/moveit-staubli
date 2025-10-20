from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import  PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_simulation_dir = get_package_share_directory("tx40_simulation")
    # First launch the robot state publisher to see the robot_description
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value = os.path.join(get_package_share_directory("tx40_simulation"), "urdf", "tx40.urdf.xacro"),
        description=" Absolute path to URDF model file"
        )
    
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(robot_simulation_dir).parent.resolve())

        ]
    )
    
    # Full plain URDF robot model (converted from xacro)
    robot_description = ParameterValue(Command([
        "xacro ", 
        LaunchConfiguration("model")]),
        value_type=str
    )

    # Launch the node robot state pub to publish the robot states thanks to the URDF model
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                    "use_sim_time": True}]
    )

    # Launch the Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
            "/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", [" -r -v 4 -r empty.sdf"])]

    )

    # Spawn the robot from the URDF model
    gz_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "tx40_robot"]
    )

    # LAunch the bridge between Ros2 & Gz to synchronize the clock
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        gz_spawn,
        gz_ros2_bridge
    ])
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the gazebo launch file
    launch_gazebo = os.path.join(
        get_package_share_directory('tx40_simulation'),
        'launch',
        'tx40_gazebo.launch.py'
    )

    # Path to the ros2 control launch file
    launch_controllers = os.path.join(
        get_package_share_directory('tx40_controller'),
        'launch',
        'tx40_controllers.launch.py'
    )

    # Path to the moveit launch file
    launch_moveit = os.path.join(
        get_package_share_directory('tx40_moveit'),
        'launch',
        'moveit.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_gazebo)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_controllers)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_moveit)
        )
    ])

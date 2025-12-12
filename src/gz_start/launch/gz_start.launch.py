from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('gz_start')
    urdf_file_env = os.path.join(pkg_share, 'urdf', 'env.urdf')
    urdf_file_robot = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(
    #             get_package_share_directory('ros_gz_sim'),
    #             'launch',
    #             'gz_sim.launch.py'
    #         )]
    #     ),
    #     launch_arguments={
    #         'gz_args': '-r empty.sdf'
    #     }.items(),
    # )

    spawn_env = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'env',
            '-file', urdf_file_env
        ]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'robot',
            '-file', urdf_file_robot
        ]
    )

    return LaunchDescription([
        #gazebo,
        spawn_env,
        spawn_robot
    ])
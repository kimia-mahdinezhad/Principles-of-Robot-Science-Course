import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    gz_ros2_control_demos_path = os.path.join(
        get_package_share_directory('laser_scan'))

    xacro_file = os.path.join(gz_ros2_control_demos_path,
                              'maze/'
                              'maze.sdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    launchDescription = LaunchDescription()

    launchDescription.add_action(
        ExecuteProcess(cmd=[['ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan']], shell=True)
    )

    launchDescription.add_action(
        ExecuteProcess(cmd=[['ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist']], shell=True)
    )

    launchDescription.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r -v 4 ' + xacro_file
        }.items(),
        ),
    )

    launchDescription.add_action(Node(
        package='laser_scan',
        executable='main',
        name='main'
    ))

    return launchDescription

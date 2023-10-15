import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments=[
            ('gz_args', "/home/kimia/Desktop/Robotics/Lab_Assignment/S09/WS/src/lab9_2/maze/maze.sdf")
        ]
    )

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        arguments=[
            ['/cmd_vel' +
             '@geometry_msgs/msg/Twist' +
             '[gz.msgs.Twist'],
        ]
    )

    laser_scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='laser_scan_bridge',
        output='screen',
        arguments=[
            ['/lidar' +
             '@sensor_msgs/msg/LaserScan' +
             '[gz.msgs.LaserScan'],
        ]
    )

    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        output='screen',
        arguments=[
            ['/vehicle_blue/chassis/tf' +
             '@tf2_msgs/msg/TFMessage' +
             '[gz.msgs.Pose_V'],
        ],
        remappings=[
            ('/vehicle_blue/chassis/tf', '/tf')
        ]
    )
  
    lidar_chasis = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_chasis',
        arguments=[
            "0", "0", "0", "0", "0", "0", "vehicle_blue/chassis", "vehicle_blue/chassis/gpu_lidar",
        ]
    )

    odom_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_world',
        arguments=[
            "0", "0", "0", "0", "0", "0", "world", "vehicle_blue/chassis/odom",
        ]
    )

    launchDescription = LaunchDescription()
    launchDescription.add_action(gz_sim)
    launchDescription.add_action(cmd_vel_bridge)
    launchDescription.add_action(laser_scan_bridge)
    launchDescription.add_action(tf_bridge)
    launchDescription.add_action(lidar_chasis)
    launchDescription.add_action(odom_world)

    return launchDescription

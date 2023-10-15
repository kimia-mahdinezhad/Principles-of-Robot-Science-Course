from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster3',
            parameters=[
                {'turtlename': 'turtle3'}
            ]
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster4',
            parameters=[
                {'turtlename': 'turtle4'}
            ]
        ),
        DeclareLaunchArgument(
            'current_frame_value_2', default_value='t2',
            description='Current Frame is definied.'
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_listener',
            name='listener12',
            parameters=[
                {'current_frame_value': LaunchConfiguration('current_frame_value_2')},
            ]
        ),
        DeclareLaunchArgument(
            'current_frame_value_3', default_value='t3',
            description='Current Frame is definied.'
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_listener',
            name='listener23',
            parameters=[
                {'current_frame_value': LaunchConfiguration('current_frame_value_3')},
            ]
        ),
        DeclareLaunchArgument(
            'current_frame_value_4', default_value='t4',
            description='Current Frame is definied.'
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_listener',
            name='listener34',
            parameters=[
                {'current_frame_value': LaunchConfiguration('current_frame_value_4')},
            ]
        ),
    ])

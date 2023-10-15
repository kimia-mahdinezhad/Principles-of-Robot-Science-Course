from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-entity', 'my_robot', '-file', './src/lab5_urdf_package/urdf/model.urdf.xacro', '-x', '0', '-y', '0', '-z', '0'],
            output='screen'
        )
    ])
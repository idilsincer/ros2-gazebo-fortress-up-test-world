import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('up_week1')
    world = os.path.join(share, 'worlds', 'test_world.sdf')

    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', world], output='screen')

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen',
    )

    sim_time = {'use_sim_time': True}
    return LaunchDescription([
        gazebo,
        bridge,
        Node(package='up_week1', executable='camera_monitor', output='screen'),
        Node(package='up_week1', executable='heartbeat_publisher', output='screen'),
        Node(package='up_week1', executable='heartbeat_subscriber', output='screen'),
    ])
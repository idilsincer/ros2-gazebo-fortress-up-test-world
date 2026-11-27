import os

from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

WORLD_NAME = 'up_test_world'


def _find_package(*names):
    for name in names:
        try:
            get_package_share_directory(name)
            return name
        except PackageNotFoundError:
            continue
    return None


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context):
    arguments = context.launch_configurations
    share = get_package_share_directory('up_week2')

    lighting = arguments.get('lighting', 'normal').strip().lower()
    if lighting not in ('normal', 'dim'):
        raise RuntimeError(
            f'lighting must be "normal" or "dim", got "{lighting}"')

    use_mover = _as_bool(arguments.get('use_mover', 'true'))

    params_file = arguments.get('params_file', '').strip()
    if not params_file:
        params_file = os.path.join(
            share, 'config',
            'detector_dim.yaml' if lighting == 'dim' else 'detector.yaml')
    params_file = os.path.expanduser(params_file)
    if not os.path.isfile(params_file):
        raise RuntimeError(f'params_file does not exist: {params_file}')

    world_file = os.path.join(
        share, 'worlds',
        'test_world_w2_dim.sdf' if lighting == 'dim'
        else 'test_world_w2.sdf')

    bridge_package = _find_package('ros_ign_bridge', 'ros_gz_bridge')
    interface_package = _find_package('ros_ign_interfaces',
                                      'ros_gz_interfaces')
    if bridge_package is None:
        raise RuntimeError(
            'Neither ros_ign_bridge nor ros_gz_bridge is installed.')
    if interface_package is None:
        raise RuntimeError(
            'Neither ros_ign_interfaces nor ros_gz_interfaces is installed.')

    try:
        week1_share = get_package_share_directory('up_week1')
    except PackageNotFoundError as exc:
        raise RuntimeError(
            'up_week1 package not found. Build it first: it is a documented '
            'dependency of up_week2.') from exc

    week1_launch = os.path.join(week1_share, 'launch', 'week1.launch.py')
    try:
        with open(week1_launch, 'r', encoding='utf-8') as handle:
            week1_source = handle.read()
    except OSError:
        week1_source = ''
    week1_supports_world = ('"world"' in week1_source
                            or "'world'" in week1_source)

    set_pose_bridge = (
        f'/world/{WORLD_NAME}/set_pose'
        f'@{interface_package}/srv/SetEntityPose')
    camera_info_bridge = ('/camera/camera_info'
                          '@sensor_msgs/msg/CameraInfo'
                          '[ignition.msgs.CameraInfo')
    image_bridge = ('/camera/image_raw'
                    '@sensor_msgs/msg/Image[ignition.msgs.Image')
    clock_bridge = '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
    # Actual Gazebo model poses (ground truth of the evaluation tool).
    pose_info_bridge = (f'/world/{WORLD_NAME}/pose/info'
                        '@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V')

    if week1_supports_world:
        simulation = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(week1_launch),
                launch_arguments={'world': world_file}.items(),
            )
        ]
        bridge_arguments = [camera_info_bridge, set_pose_bridge]
    else:
        # Backward compatible path for an unpatched week-1 package.  The
        # week-1 *executables* are used; their launch file is not copied.
        simulation = [
            ExecuteProcess(
                cmd=['ign', 'gazebo', '-r', world_file], output='screen'),
            Node(
                package='up_week1',
                executable='camera_monitor',
                output='screen',
            ),
            Node(
                package='up_week1',
                executable='heartbeat_publisher',
                output='screen',
            ),
            Node(
                package='up_week1',
                executable='heartbeat_subscriber',
                output='screen',
            ),
        ]
        bridge_arguments = [image_bridge, clock_bridge,
                            camera_info_bridge, set_pose_bridge]

    bridge = Node(
        package=bridge_package,
        executable='parameter_bridge',
        arguments=bridge_arguments,
        output='screen',
    )

    pose_bridge = Node(
        package=bridge_package,
        executable='parameter_bridge',
        arguments=[pose_info_bridge],
        output='screen',
    )

    sim_time = {'use_sim_time': True}

    target_mover = Node(
        package='up_week2',
        executable='target_mover',
        parameters=[
            sim_time,
            {
                'use_mover': use_mover,
                'path_file': os.path.join(share, 'config',
                                          'target_path.yaml'),
                'set_pose_service': f'/world/{WORLD_NAME}/set_pose',
                'use_gazebo_pose': True,
                'pose_info_topic': f'/world/{WORLD_NAME}/pose/info',
            },
        ],
        output='screen',
    )

    detector = Node(
        package='up_week2',
        executable='red_target_detector',
        parameters=[params_file, sim_time],
        output='screen',
    )

    return simulation + [bridge, pose_bridge, target_mover, detector]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lighting',
            default_value='normal',
            description='Lighting configuration: normal or dim',
        ),
        DeclareLaunchArgument(
            'use_mover',
            default_value='true',
            description='Move the red target along config/target_path.yaml',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description=('Detector parameter file. Empty = automatically '
                         'select config/detector.yaml (normal) or '
                         'config/detector_dim.yaml (dim).'),
        ),
        OpaqueFunction(function=_launch_setup),
    ])

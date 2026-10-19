"""Mapping: point_lio provides odometry, FreeDOM optionally provides the static map.

    ros2 launch point_lio map.launch.py                        # Point-LIO only
    ros2 launch point_lio map.launch.py enable_freedom:=true   # Point-LIO + FreeDOM
    ros2 launch point_lio map.launch.py scene:=outdoor         # outdoor (Point-LIO only)
    ros2 launch point_lio map.launch.py enable_freedom:=true scene:=outdoor
    ros2 launch point_lio map.launch.py map_dir:=/data/maps    # change the map output root
    ros2 topic pub --once /save_map std_msgs/msg/Empty "{}"    # save the map

"""
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup_log_dir(base: str) -> str:
    """Put ROS logs under base/<date>, falling back to the default when base is
    not writable (a plain user has no write access to /var/log).

    Without this the logs land in ~/.ros/log with one directory per run and no
    date in the name, which makes finding the run that matters after the fact a
    matter of matching timestamps by hand.
    """
    day = os.path.join(base, datetime.now().strftime('%Y-%m-%d'))
    try:
        os.makedirs(day, exist_ok=True)
        probe = os.path.join(day, '.write_test')
        with open(probe, 'w'):
            pass
        os.remove(probe)
        return day
    except OSError as e:
        fallback = os.path.expanduser('~/.ros/log')
        print(f'[launch] log dir {day} not writable ({e}), falling back to {fallback}')
        return fallback

# Absolute, so a map never follows whatever directory the node was launched from.
_DEFAULT_MAP_DIR = '/var/opt/map'


def _default_map_dir():
    return _DEFAULT_MAP_DIR


MAP_NAME = 'map.pcd'

OUTDOOR = {
    'sensor.max_range': 60.0,
    'sensor.min_z': -10.0,
    'sensor.max_z': 25.0,
    'map.raycast_max_range': 30.0,
    'map.raycast_min_z': -5.0,
    'map.raycast_max_z': 10.0,
    'map.local_map_range': 80.0,
    'map.local_map_max_z': 30.0,
}


def launch_setup(context, *args, **kwargs):
    log_dir = _setup_log_dir(LaunchConfiguration('log_dir').perform(context))
    os.environ['ROS_LOG_DIR'] = log_dir
    print(f'[launch] log dir: {log_dir}')

    point_lio_dir = get_package_share_directory('point_lio')
    freedom_dir = get_package_share_directory('freedom')

    cfg_file = os.path.join(point_lio_dir, 'config', 'mid360s.yaml')
    freedom_cfg = os.path.join(freedom_dir, 'config', 'mid360_pointlio.yaml')
    rviz_cfg = os.path.join(point_lio_dir, 'rviz_cfg', 'loam_livox.rviz')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    scene = LaunchConfiguration('scene').perform(context).lower()
    if scene not in ('indoor', 'outdoor'):
        raise RuntimeError(f"scene must be indoor or outdoor, got '{scene}'")

    map_dir = LaunchConfiguration('map_dir').perform(context)
    run_dir = os.path.join(map_dir, datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(run_dir, exist_ok=True)
    sc_path = os.path.join(run_dir, MAP_NAME + '.sc')

    # Point-LIO maintains this symlink itself only when it makes the run directory.
    # run_subdir is off here, so the launch has to keep it, or downstream configs
    # that hard-code one path (nav.launch.py, relocalization) lose their anchor.
    latest = os.path.join(map_dir, 'latest')
    try:
        if os.path.islink(latest) or os.path.exists(latest):
            os.remove(latest)
        os.symlink(run_dir, latest)
    except OSError as e:
        print(f'[map.launch] failed to update the latest symlink: {e}')
    print(f'[map.launch] map output dir: {run_dir}  (map.pcd, map.pcd.sc and tiles all live here)')

    enable_freedom = LaunchConfiguration('enable_freedom').perform(context).lower() == 'true'
    freedom_params = [freedom_cfg, {'save_map_path': run_dir,
                                    'save_map_name': MAP_NAME}]
    if scene == 'outdoor':
        freedom_params.append(OUTDOOR)

    user_config_path = PathJoinSubstitution(
        [FindPackageShare('livox_ros_driver2'), 'config', 'MID360s_config.json'])


    nodes = [
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            output='screen',
            parameters=[{
                'xfer_format': 1,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'lvx_file_path': '/home/livox/livox_test.lvx',
                'user_config_path': user_config_path,
                'cmdline_input_bd_code': 'livox0000000001',
            }],
        ),

        Node(
            package='point_lio',
            executable='pointlio_mapping',
            parameters=[cfg_file, {
                'pcd_save.output_dir': run_dir,
                'pcd_save.run_subdir': False,   # run_dir already carries the stamp
                'pcd_save.export_patches': LaunchConfiguration('export_patches'),
                'pcd_save.pcd_save_en': not enable_freedom,
                'pcd_save.file_name': MAP_NAME,
                'scan_context.save_path': sc_path,
            }],
            remappings=remappings,
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_camera_init',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            remappings=remappings,
            output='screen',
        ),
    ]

    if enable_freedom:
        nodes.append(
            Node(
                package='freedom',
                executable='freedom_node',
                name='freedom',
                parameters=freedom_params,
                remappings=remappings,
                output='screen',
            )
        )
        print('[map.launch] FreeDOM enabled')
    else:
        print('[map.launch] FreeDOM disabled (Point-LIO only)')

    nodes.append(
        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_cfg],
            remappings=remappings,
            output='screen',
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'enable_freedom', default_value='false',
            description='true: run FreeDOM for dynamic-point removal | false: Point-LIO only'
        ),
        DeclareLaunchArgument(
            'scene', default_value='indoor',
            description='indoor | outdoor - affects only FreeDOM range and height limits'),
        DeclareLaunchArgument(
            'export_patches', default_value='false',
            description='Also write poses.txt and patches/ for open3d_loc localization'),
        DeclareLaunchArgument(
            'map_dir', default_value=_default_map_dir(),
            description='map output root; each launch creates a timestamped subdirectory under it'),
        DeclareLaunchArgument(
            'log_dir', default_value='/var/log/slam',
            description='Log root; a <date> folder is created under it'),
        OpaqueFunction(function=launch_setup),
    ])

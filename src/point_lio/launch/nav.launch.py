import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

def launch_setup(context, *args, **kwargs):
    log_dir = _setup_log_dir(LaunchConfiguration('log_dir').perform(context))
    os.environ['ROS_LOG_DIR'] = log_dir
    print(f'[launch] log dir: {log_dir}')

    point_lio_dir      = get_package_share_directory('point_lio')
    small_gicp_dir     = get_package_share_directory('small_gicp_relocalization')

    cfg_file  = os.path.join(point_lio_dir, 'config', 'mid360.yaml')
    rviz_cfg  = os.path.join(point_lio_dir, 'rviz_cfg', 'loam_livox.rviz')
    
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    use_rviz = LaunchConfiguration('rviz').perform(context).lower() in ('true', '1', 'yes')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    body_frame = LaunchConfiguration('body_frame')

    xfer_format = 1
    multi_topic = 0
    data_src = 0
    publish_freq = 10.0
    output_type = 0
    frame_id = 'livox_frame'
    lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'
    user_config_path = PathJoinSubstitution(
        [FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json']
    )

    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        output='screen',
        parameters=[{
            'xfer_format': xfer_format, 'multi_topic': multi_topic,
            'data_src': data_src, 'publish_freq': publish_freq,
            'output_data_type': output_type, 'frame_id': frame_id,
            'lvx_file_path': lvx_file_path, 'user_config_path': user_config_path,
            'cmdline_input_bd_code': cmdline_bd_code,
        }],
    )

    actions = [
        livox_driver_node,

        Node(
            package='point_lio',
            executable='pointlio_mapping',
            parameters=[cfg_file, {
                'pcd_save.pcd_save_en': False,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=remappings,
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(small_gicp_dir, 'launch', 'small_gicp_relocalization_launch.py')
            ),
            launch_arguments={
                'use_sim_time':      LaunchConfiguration('use_sim_time'),
                'map_frame':         map_frame,
                'odom_frame':        odom_frame,
                'base_frame':        body_frame,
                'lidar_frame':       body_frame,
                'input_cloud_topic': '/cloud_registered', 
                'prior_pcd_file':    LaunchConfiguration('prior_pcd_file'),
                'max_iterations':    LaunchConfiguration('max_iterations'),
                'max_dist_sq':       LaunchConfiguration('max_dist_sq'),
                'global_init':       LaunchConfiguration('global_init'),
                'initial_guess':     LaunchConfiguration('initial_guess'),
            }.items(),
        ),
    ]

    if use_rviz:
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=['-d', rviz_cfg],
                remappings=remappings,
                output='screen',
            )
        )

    return actions


def _default_prior_pcd() -> str:
    share = get_package_share_directory('point_lio')
    ws = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    return os.path.join(ws, 'PCD', 'latest', 'temp_louti.pcd')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('body_frame', default_value='body', 
                              description='Point-LIO body frame name (body)'),
        
        DeclareLaunchArgument(
            'prior_pcd_file', default_value=_default_prior_pcd(),
            description='Prior map. Defaults to the latest run of map.launch.py; '
                        'its .sc descriptor is expected alongside it'),
        DeclareLaunchArgument('max_iterations', default_value='64'),
        DeclareLaunchArgument('max_dist_sq', default_value='0.25', description='tighten the correspondence distance to 0.5 m to prevent sliding on the ground'),
        
        DeclareLaunchArgument('global_init', default_value='true', description='enable blind global search'),
        DeclareLaunchArgument('initial_guess', default_value='[0.0, 0.0, 0.0, 0.0]', 
                              description='format: [x, y, z, yaw_rad]'),

        DeclareLaunchArgument(
            'log_dir', default_value='/var/log/slam',
            description='Log root; a <date> folder is created under it'),
        OpaqueFunction(function=launch_setup),
    ])
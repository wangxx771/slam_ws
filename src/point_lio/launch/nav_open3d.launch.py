"""
Usage:
    ros2 launch point_lio nav_open3d.launch.py
    ros2 launch point_lio nav_open3d.launch.py prior_pcd_file:=/var/opt/map/active/map.pcd
    ros2 launch point_lio nav_open3d.launch.py rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _default_prior_pcd() -> str:
    share = get_package_share_directory('point_lio')
    ws = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    return os.path.join(ws, 'PCD', 'latest', 'map.pcd')


def _setup_log_dir() -> str:
    """Mirror nav.launch.py: date-stamped directory under /var/log/slam."""
    from datetime import date
    base = os.environ.get('SLAM_LOG_DIR', '/var/log/slam')
    day = os.path.join(base, date.today().isoformat())
    try:
        os.makedirs(day, exist_ok=True)
        probe = os.path.join(day, '.w')
        with open(probe, 'w'):
            pass
        os.remove(probe)
    except OSError as e:
        fallback = os.path.expanduser('~/.ros/log')
        os.makedirs(fallback, exist_ok=True)
        print(f'[launch] log dir {day} not writable ({e}), falling back to {fallback}')
        day = fallback
    os.environ['ROS_LOG_DIR'] = day
    print(f'[launch] log dir: {day}')
    return day


def launch_setup(context, *args, **kwargs):
    point_lio_dir = get_package_share_directory('point_lio')
    cfg_name = LaunchConfiguration('config').perform(context)
    cfg_file = cfg_name if os.path.isabs(cfg_name) else os.path.join(
        point_lio_dir, 'config', cfg_name)
    rviz_cfg = os.path.join(point_lio_dir, 'rviz_cfg', 'loam_livox.rviz')
    log_dir = _setup_log_dir()

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    def _flag(name):
        return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')

    use_rviz = _flag('rviz')
    use_livox = _flag('livox')

    map_frame = LaunchConfiguration('map_frame').perform(context)
    odom_frame = LaunchConfiguration('odom_frame').perform(context)
    body_frame = LaunchConfiguration('body_frame').perform(context)
    prior_pcd = LaunchConfiguration('prior_pcd_file').perform(context)
    poses_file = LaunchConfiguration('poses_file').perform(context)
    patch_file = LaunchConfiguration('patch_file').perform(context)

    map_dir = os.path.dirname(prior_pcd)
    if not poses_file:
        cand = os.path.join(map_dir, 'poses.txt')
        poses_file = cand if os.path.isfile(cand) else ''
    if not patch_file:
        cand = os.path.join(map_dir, 'patches')
        patch_file = cand if os.path.isdir(cand) else ''
    if not poses_file or not patch_file:
        print('[nav_open3d] WARNING: no poses.txt/patches beside the map; '
              'global localization cannot initialize. Rebuild the map with '
              'pcd_save.export_patches:=true')

    if not os.path.isfile(prior_pcd):
        print(f'[nav_open3d] WARNING: prior map not found: {prior_pcd}')

    livox_driver_node = Node(
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
            'user_config_path': PathJoinSubstitution(
                [FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json']
            ),
            'cmdline_input_bd_code': 'livox0000000001',
        }],
    )

    point_lio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        parameters=[cfg_file, {
            'pcd_save.pcd_save_en': False,
            'log.dir': log_dir,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=remappings,
        output='screen',
    )

    open3d_loc_node = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='screen',
        parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'map_frame': map_frame,
            'odom_frame': odom_frame,
            'base_frame': body_frame,

            'path_map': prior_pcd,
            'poses_file': poses_file,
            'patch_file': patch_file,

            'pcd_queue_maxsize': 10,
            'voxelsize_coarse': 0.01,
            'voxelsize_fine': 0.2,
            'threshold_fitness': LaunchConfiguration('threshold_fitness'),
            'threshold_fitness_init': LaunchConfiguration('threshold_fitness_init'),
            'fitness_inlier_dist': LaunchConfiguration('fitness_inlier_dist'),
            'kiss_min_inliers': LaunchConfiguration('kiss_min_inliers'),
            'loc_frequence': 2.5,
            'save_scan': False,
            'hidden_removal': False,
            'maxpoints_source': 80000,
            'maxpoints_target': 400000,
            'filter_odom2map': False,
            'kalman_processVar2': 0.001,
            'kalman_estimatedMeasVar2': 0.02,
            'confidence_loc_th': 0.7,
            'dis_updatemap': 3.5,
            'leaf_size': LaunchConfiguration('leaf_size'),
            'log_dir': log_dir,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=remappings,
    )

    actions = [point_lio_node, open3d_loc_node]
    if use_livox:
        actions.insert(0, livox_driver_node)

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


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'livox', default_value='true',
            description='Start the Livox driver. Set false when replaying a bag or running MuJoCo'),
        DeclareLaunchArgument(
            'config', default_value='mid360.yaml',
            description='Point-LIO config: a name under point_lio/config or an absolute path'),

        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument(
            'body_frame', default_value='body',
            description="Point-LIO body frame name; open3d_loc's own default is base_link"),

        DeclareLaunchArgument(
            'odom_topic', default_value='/lio/odom',
            description="Point-LIO odometry; open3d_loc's own default is /Odometry"),
        DeclareLaunchArgument('cloud_topic', default_value='/cloud_registered'),

        DeclareLaunchArgument(
            'prior_pcd_file', default_value=_default_prior_pcd(),
            description='Prior map for localization'),
        DeclareLaunchArgument('threshold_fitness', default_value='0.5'),
        DeclareLaunchArgument('threshold_fitness_init', default_value='0.5'),
        DeclareLaunchArgument(
            'fitness_inlier_dist', default_value='0.2',
            description='A scan point counts as matched within this distance of a map point'),
        DeclareLaunchArgument('kiss_min_inliers', default_value='10'),
        DeclareLaunchArgument(
            'leaf_size', default_value='0.05',
            description='Map downsample for the fitness check; see the note in launch_setup'),
        DeclareLaunchArgument(
            'poses_file', default_value='',
            description='Optional keyframe poses next to the map'),
        DeclareLaunchArgument(
            'patch_file', default_value='',
            description='Optional map patch directory'),

        OpaqueFunction(function=launch_setup),
    ])

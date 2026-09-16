# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_bool(name, context):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")


def launch_setup(context, *args, **kwargs):
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    parameters = [
        {

            "use_sim_time": _parse_bool("use_sim_time", context),
            "num_threads": 4,
            "num_neighbors": 10,
            "global_leaf_size": 0.25,
            "registered_leaf_size": 0.25,
            "max_dist_sq": float(LaunchConfiguration("max_dist_sq").perform(context)),
            "max_iterations": int(LaunchConfiguration("max_iterations").perform(context)),
            "map_frame": LaunchConfiguration("map_frame").perform(context),
            "odom_frame": LaunchConfiguration("odom_frame").perform(context),
            "base_frame": LaunchConfiguration("base_frame").perform(context),
            "lidar_frame": LaunchConfiguration("lidar_frame").perform(context),
            "prior_pcd_file": LaunchConfiguration("prior_pcd_file").perform(context),
            "input_cloud_topic": LaunchConfiguration("input_cloud_topic").perform(context),
            "min_startup_time": float(LaunchConfiguration("min_startup_time").perform(context)),
            "min_source_points": int(LaunchConfiguration("min_source_points").perform(context)),
            "min_inlier_ratio": float(LaunchConfiguration("min_inlier_ratio").perform(context)),
            "min_inliers": int(LaunchConfiguration("min_inliers").perform(context)),
            "init_consistency_trans": float(
                LaunchConfiguration("init_consistency_trans").perform(context)
            ),
            "init_consistency_rot": float(
                LaunchConfiguration("init_consistency_rot").perform(context)
            ),
            # Continuous gated correction (problem 2)
            "continuous_correction": _parse_bool("continuous_correction", context),
            "max_correction_trans": float(
                LaunchConfiguration("max_correction_trans").perform(context)
            ),
            "max_correction_rot": float(
                LaunchConfiguration("max_correction_rot").perform(context)
            ),
            # Automatic global relocalization front-end (FPFH+RANSAC) + manual /initialpose
            "feature_leaf_size": float(
                LaunchConfiguration("feature_leaf_size").perform(context)
            ),
            "feature_normal_radius": float(
                LaunchConfiguration("feature_normal_radius").perform(context)
            ),
            "feature_fpfh_radius": float(
                LaunchConfiguration("feature_fpfh_radius").perform(context)
            ),
            "feature_vertical_nz_max": float(
                LaunchConfiguration("feature_vertical_nz_max").perform(context)
            ),
            "ransac_max_iterations": int(
                LaunchConfiguration("ransac_max_iterations").perform(context)
            ),
            "ransac_inlier_fraction": float(
                LaunchConfiguration("ransac_inlier_fraction").perform(context)
            ),
            "ransac_similarity_threshold": float(
                LaunchConfiguration("ransac_similarity_threshold").perform(context)
            ),
            "ransac_correspondence_randomness": int(
                LaunchConfiguration("ransac_correspondence_randomness").perform(context)
            ),
            "ransac_max_corr_dist": float(
                LaunchConfiguration("ransac_max_corr_dist").perform(context)
            ),
            "ransac_min_inliers": int(
                LaunchConfiguration("ransac_min_inliers").perform(context)
            ),
            "ransac_max_abs_z": float(
                LaunchConfiguration("ransac_max_abs_z").perform(context)
            ),
            "sc_fallback_attempts": int(
                LaunchConfiguration("sc_fallback_attempts").perform(context)
            ),
            "verify_ratio": float(LaunchConfiguration("verify_ratio").perform(context)),
            "verify_margin": float(LaunchConfiguration("verify_margin").perform(context)),
            "ransac_max_tilt_deg": float(
                LaunchConfiguration("ransac_max_tilt_deg").perform(context)
            ),
            "enable_initialpose": _parse_bool("enable_initialpose", context),
            "initialpose_topic": LaunchConfiguration("initialpose_topic").perform(
                context
            ),
        }
    ]

    node = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        namespace="",
        output="screen",
        remappings=remappings,
        parameters=parameters,
    )
    return [node]


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="body"),
        DeclareLaunchArgument("lidar_frame", default_value="body"),
        DeclareLaunchArgument("max_dist_sq", default_value="4.0"),
        DeclareLaunchArgument("max_iterations", default_value="64"),
        DeclareLaunchArgument(
            "prior_pcd_file",
            default_value="/root/map.pcd",
        ),
        DeclareLaunchArgument("input_cloud_topic", default_value="cloud_registered"),
        DeclareLaunchArgument("min_startup_time", default_value="2.0"),
        DeclareLaunchArgument("min_source_points", default_value="300"),
        DeclareLaunchArgument("min_inlier_ratio", default_value="0.5"),
        DeclareLaunchArgument("min_inliers", default_value="100"),
        DeclareLaunchArgument("init_consistency_trans", default_value="0.15"),
        DeclareLaunchArgument("init_consistency_rot", default_value="0.05"),
        DeclareLaunchArgument("continuous_correction", default_value="False"),
        DeclareLaunchArgument("max_correction_trans", default_value="0.30"),
        DeclareLaunchArgument("max_correction_rot", default_value="0.08"),
        DeclareLaunchArgument("feature_leaf_size", default_value="0.3"),
        DeclareLaunchArgument("feature_normal_radius", default_value="1.0"),
        DeclareLaunchArgument("feature_fpfh_radius", default_value="2.5"),
        DeclareLaunchArgument("feature_vertical_nz_max", default_value="0.8"),
        DeclareLaunchArgument("ransac_max_iterations", default_value="50000"),
        DeclareLaunchArgument("ransac_inlier_fraction", default_value="0.25"),
        DeclareLaunchArgument("ransac_similarity_threshold", default_value="0.9"),
        DeclareLaunchArgument("ransac_correspondence_randomness", default_value="5"),
        DeclareLaunchArgument("ransac_max_corr_dist", default_value="1.0"),
        DeclareLaunchArgument("ransac_min_inliers", default_value="200"),
        DeclareLaunchArgument("ransac_max_abs_z", default_value="1.5"),
        DeclareLaunchArgument("sc_fallback_attempts", default_value="10"),
        DeclareLaunchArgument("verify_margin", default_value="0.15"),
        DeclareLaunchArgument("verify_ratio", default_value="0.6"),
        DeclareLaunchArgument("ransac_max_tilt_deg", default_value="25.0"),
        DeclareLaunchArgument("enable_initialpose", default_value="True"),
        DeclareLaunchArgument("initialpose_topic", default_value="initialpose"),
    ]

    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])

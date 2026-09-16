
#include "slam_log/log.h"
#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/search/kdtree.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

namespace
{


pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFpfh(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, double normal_radius, double fpfh_radius,
  double vertical_nz_max, int num_threads, pcl::PointCloud<pcl::PointXYZ>::Ptr & out_cloud)
{
  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

  auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNumberOfThreads(num_threads);
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(normal_radius);
  ne.compute(*normals);

  out_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto kept_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  out_cloud->reserve(cloud->size());
  kept_normals->reserve(cloud->size());
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const float nz = normals->points[i].normal_z;
    if (vertical_nz_max >= 1.0 || (std::isfinite(nz) && std::fabs(nz) < vertical_nz_max)) {
      out_cloud->push_back(cloud->points[i]);
      kept_normals->push_back(normals->points[i]);
    }
  }

  auto kept_tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  auto features = std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fe;
  fe.setNumberOfThreads(num_threads);
  fe.setInputCloud(out_cloud);
  fe.setInputNormals(kept_normals);
  fe.setSearchMethod(kept_tree);
  fe.setRadiusSearch(fpfh_radius);
  fe.compute(*features);
  return features;
}
}  // namespace

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 4.0);
  this->declare_parameter("max_iterations", 64);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("input_cloud_topic", "registered_scan");
  this->declare_parameter("min_startup_time", 2.0);
  this->declare_parameter("min_source_points", 300);
  this->declare_parameter("min_inlier_ratio", 0.5);
  this->declare_parameter("min_inliers", 100);
  this->declare_parameter("degeneracy_warn_ratio", 30.0);
  this->declare_parameter("init_consistency_trans", 0.15);
  this->declare_parameter("init_consistency_rot", 0.05);
  this->declare_parameter("continuous_correction", false);
  this->declare_parameter("max_correction_trans", 0.30);
  this->declare_parameter("max_correction_rot", 0.08);
  this->declare_parameter("max_consecutive_rejects", 10);
  this->declare_parameter("feature_leaf_size", 0.5);
  this->declare_parameter("feature_normal_radius", 1.0);
  this->declare_parameter("feature_fpfh_radius", 2.5);
  this->declare_parameter("feature_vertical_nz_max", 0.8);
  this->declare_parameter("ransac_max_iterations", 50000);
  this->declare_parameter("ransac_inlier_fraction", 0.25);
  this->declare_parameter("ransac_similarity_threshold", 0.9);
  this->declare_parameter("ransac_correspondence_randomness", 5);
  this->declare_parameter("ransac_max_corr_dist", 1.0);
  this->declare_parameter("ransac_min_inliers", 200);
  this->declare_parameter("ransac_max_abs_z", 1.5);
  this->declare_parameter("ransac_max_tilt_deg", 15.0);
  this->declare_parameter("sc_enable", true);
  this->declare_parameter("sc_top_k", 30);
  this->declare_parameter("sc_fallback_attempts", 240);
  this->declare_parameter("sc_verify_candidates", 5);
  this->declare_parameter("verify_dist", 0.5);
  // Recalibrated for the normal-aware strictFitRatio. Requiring matched points to
// also agree in orientation lowers every score, so the old 0.60 - set when the
// check was proximity only - rejected every candidate at two of three start
// points. At 0.40 all six test cases lock, and the score still tracks accuracy:
// 0.64 gave 0.10 m, 0.51 gave 1.71 m, 0.48 gave 1.92 m.
this->declare_parameter("verify_ratio", 0.40);
  this->declare_parameter("verify_nz_max", 0.7);
  this->declare_parameter("sc_max_height", 1e9);
  this->declare_parameter("verify_min_pts", 200);
  this->declare_parameter("verify_margin", 0.15);
  this->declare_parameter("sc_query_accum_sec", 2.0);
  this->declare_parameter("manual_seed_max_trans", 6.0);
  this->declare_parameter("manual_seed_max_rot", 0.6);
  this->declare_parameter("enable_initialpose", true);
  this->declare_parameter("initialpose_topic", "initialpose");
  this->declare_parameter("relocalize_service", "/relocalize");

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("max_iterations", max_iterations_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);
  this->get_parameter("min_startup_time", min_startup_time_);
  this->get_parameter("min_source_points", min_source_points_);
  this->get_parameter("min_inlier_ratio", min_inlier_ratio_);
  this->get_parameter("min_inliers", min_inliers_);
  this->get_parameter("degeneracy_warn_ratio", degeneracy_warn_ratio_);
  this->get_parameter("init_consistency_trans", init_consistency_trans_);
  this->get_parameter("init_consistency_rot", init_consistency_rot_);
  this->get_parameter("continuous_correction", continuous_correction_);
  this->get_parameter("max_correction_trans", max_correction_trans_);
  this->get_parameter("max_correction_rot", max_correction_rot_);
  this->get_parameter("max_consecutive_rejects", max_consecutive_rejects_);
  this->get_parameter("feature_leaf_size", feature_leaf_size_);
  this->get_parameter("feature_normal_radius", feature_normal_radius_);
  this->get_parameter("feature_fpfh_radius", feature_fpfh_radius_);
  this->get_parameter("feature_vertical_nz_max", feature_vertical_nz_max_);
  this->get_parameter("ransac_max_iterations", ransac_max_iterations_);
  this->get_parameter("ransac_inlier_fraction", ransac_inlier_fraction_);
  this->get_parameter("ransac_similarity_threshold", ransac_similarity_threshold_);
  this->get_parameter("ransac_correspondence_randomness", ransac_correspondence_randomness_);
  this->get_parameter("ransac_max_corr_dist", ransac_max_corr_dist_);
  this->get_parameter("ransac_min_inliers", ransac_min_inliers_);
  this->get_parameter("ransac_max_abs_z", ransac_max_abs_z_);
  double ransac_max_tilt_deg = 15.0;
  this->get_parameter("ransac_max_tilt_deg", ransac_max_tilt_deg);
  ransac_max_tilt_ = ransac_max_tilt_deg * M_PI / 180.0;
  this->get_parameter("sc_enable", sc_enable_);
  this->get_parameter("sc_top_k", sc_top_k_);
  this->get_parameter("sc_fallback_attempts", sc_fallback_attempts_);
  this->get_parameter("sc_verify_candidates", sc_verify_candidates_);
  this->get_parameter("verify_dist", verify_dist_);
  this->get_parameter("verify_ratio", verify_ratio_);
  this->get_parameter("verify_nz_max", verify_nz_max_);
  {
    double sc_h = 1e9;
    this->get_parameter("sc_max_height", sc_h);
    sc_db_.cfg.max_height = static_cast<float>(sc_h);
  }
  this->get_parameter("verify_min_pts", verify_min_pts_);
  rival_min_sep_ = this->declare_parameter<double>("rival_min_sep", 8.0);
  sc_score_good_ = this->declare_parameter<double>("sc_score_good", 0.15);
  // On the test bag the descriptor score predicted the outcome directly:
  //   0.131 -> fit 0.64 -> 0.096 m
  //   0.166 -> fit 0.57 -> 1.92 m
  //   0.245 -> fit 0.51 -> 1.71 m
  // At 2.0 the last is rejected rather than locked at 1.7 m of error.
  sc_score_penalty_ = this->declare_parameter<double>("sc_score_penalty", 2.0);
  verify_normal_deg_ = this->declare_parameter<double>("verify_normal_deg", 25.0);
  this->get_parameter("verify_margin", verify_margin_);
  this->get_parameter("sc_query_accum_sec", sc_query_accum_sec_);
  this->get_parameter("manual_seed_max_trans", manual_seed_max_trans_);
  this->get_parameter("manual_seed_max_rot", manual_seed_max_rot_);
  this->get_parameter("enable_initialpose", enable_initialpose_);
  this->get_parameter("initialpose_topic", initialpose_topic_);
  this->get_parameter("relocalize_service", relocalize_service_);

  feature_map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  map_features_ = std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);
  rebuildTarget();

  icp_finish_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/icp_finish", rclcpp::QoS(1).transient_local());

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  if (enable_initialpose_) {
    initialpose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initialpose_topic_, 10,
        std::bind(
          &SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));
    PL_INFO(
      this->get_logger(), "Manual seeding enabled: listening on '%s' (RViz 2D Pose Estimate).",
      initialpose_topic_.c_str());
  }

  relocalize_srv_ = this->create_service<interface::srv::Relocalize>(
    relocalize_service_,
    std::bind(
      &SmallGicpRelocalizationNode::relocalizeCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  PL_INFO(this->get_logger(), "Relocalize service ready on '%s'.", relocalize_service_.c_str());

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

/// Rebuild the registration target (downsampled cloud, covariances, kd-tree) from
/// the current global_map_.
///
/// Swapping the prior map needs the whole set rebuilt; calling loadGlobalMap alone
/// leaves target_ / target_tree_ on the old map, so registration matches new-map
/// seeds against old geometry. FPFH features must be invalidated too — they are
/// map-relative, and keeping them drops the blind search onto the old map.
void SmallGicpRelocalizationNode::rebuildTarget()
{
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  map_features_built_ = false;
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    PL_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  PL_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  if (!global_map_->empty() && sc_db_.cfg.max_height > 1e8f) {
    std::vector<float> zs;
    zs.reserve(global_map_->size());
    for (const auto & p : global_map_->points) {
      if (std::isfinite(p.z)) {zs.push_back(p.z);}
    }
    if (zs.size() > 1000) {
      std::vector<float> tmp(zs);
      auto pct = [&tmp](double q) {
        const std::size_t k = static_cast<std::size_t>(tmp.size() * q);
        std::nth_element(tmp.begin(), tmp.begin() + k, tmp.end());
        return tmp[std::min(k, tmp.size() - 1)];
      };
      const float z_lo = pct(0.02), z_hi = pct(0.98);
      const float z_top_start = pct(0.85);
      std::size_t n_top = 0;
      double mean_top = 0.0;
      for (float z : zs) {
        if (z >= z_top_start) {++n_top; mean_top += z;}
      }
      if (n_top > 0) {
        mean_top /= n_top;
        double var = 0.0;
        for (float z : zs) {
          if (z >= z_top_start) {const double d = z - mean_top; var += d * d;}
        }
        const double sd_top = std::sqrt(var / n_top);
        const float span = z_hi - z_lo;
        const double top_frac = static_cast<double>(n_top) / zs.size();
        if (span > 0.5f && sd_top < 0.15 * span && top_frac > 0.05) {
          const double suggest = mean_top - 3.0 * sd_top - 0.3;
          PL_WARN(
            this->get_logger(),
            "possible ceiling in the prior map: the top %.0f%% of points cluster at z~%.2f m (thickness sigma=%.2f m, "
            "map height span %.2f m). Scan-context descriptors use the max height per bin, "
            "so a ceiling pins every nearby bin to the same height and places stop being distinguishable; global relocalization degrades. "
            "Suggest sc_max_height~%.2f (currently unclipped).",
            top_frac * 100.0, mean_top, sd_top, span, suggest);
        } else {
          PL_INFO(
            this->get_logger(),
            "prior-map ceiling check passed: height span %.2f m, no thin dense layer on top (sigma=%.2f m).",
            span, sd_top);
        }
      }
    }
  }

  if (sc_enable_) {
    const std::string scf = file_name + ".sc";
    if (sc_db_.load(scf)) {
      PL_INFO(
        this->get_logger(),
        "Loaded Scan Context database: %zu entries, %dx%d bins, %.0f m radius (%s)",
        sc_db_.entries.size(), sc_db_.cfg.num_ring, sc_db_.cfg.num_sector,
        sc_db_.cfg.max_radius, scf.c_str());
    } else {
      PL_WARN(
        this->get_logger(),
        "no scan-context DB '%s' next to the map; global relocalization will not work - "
        "rebuild the map with point_lio to produce it, or give an initial guess via /initialpose.", scf.c_str());
    }
  }

  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      PL_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      PL_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
}

void SmallGicpRelocalizationNode::buildMapFeatures()
{
  map_features_built_ = true;

  if (!global_map_ || global_map_->empty()) {
    PL_WARN(
      this->get_logger(),
      "Global map empty; cannot build FPFH features for global relocalization.");
    return;
  }

  const auto t0 = std::chrono::steady_clock::now();

  auto voxelized = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(feature_leaf_size_, feature_leaf_size_, feature_leaf_size_);
  vg.setInputCloud(global_map_);
  vg.filter(*voxelized);

  map_features_ = computeFpfh(
    voxelized, feature_normal_radius_, feature_fpfh_radius_, feature_vertical_nz_max_,
    num_threads_, feature_map_cloud_);

  const double secs =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  PL_INFO(
    this->get_logger(),
    "Built map FPFH features: %zu voxels -> %zu vertical points, %zu descriptors (%.2f s).",
    voxelized->size(), feature_map_cloud_->size(), map_features_->size(), secs);
}

void SmallGicpRelocalizationNode::ensureMapFeatures()
{
  if (map_features_built_) return;
  PL_INFO(
    this->get_logger(), "Scan Context gave no usable seed; building map FPFH features now.");
  buildMapFeatures();
}

Eigen::Isometry3d SmallGicpRelocalizationNode::featureGlobalSeed(bool & found)
{
  found = false;
  Eigen::Isometry3d best = previous_result_t_;

  ensureMapFeatures();

  if (!feature_map_cloud_ || feature_map_cloud_->empty() || !map_features_ ||
      map_features_->empty()) {
    PL_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Map FPFH features unavailable; cannot run feature-based global init.");
    return best;
  }

  auto voxelized = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(feature_leaf_size_, feature_leaf_size_, feature_leaf_size_);
  vg.setInputCloud(accumulated_cloud_);
  vg.filter(*voxelized);
  if (voxelized->size() < 10) {
    return best;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan;
  auto scan_features = computeFpfh(
    voxelized, feature_normal_radius_, feature_fpfh_radius_, feature_vertical_nz_max_,
    num_threads_, scan);
  if (scan->size() < 20) {
    return best;
  }

  pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> align;
  align.setInputSource(scan);
  align.setSourceFeatures(scan_features);
  align.setInputTarget(feature_map_cloud_);
  align.setTargetFeatures(map_features_);
  align.setMaximumIterations(ransac_max_iterations_);
  align.setNumberOfSamples(3);
  align.setCorrespondenceRandomness(ransac_correspondence_randomness_);
  align.setSimilarityThreshold(ransac_similarity_threshold_);
  align.setMaxCorrespondenceDistance(ransac_max_corr_dist_);
  align.setInlierFraction(ransac_inlier_fraction_);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  align.align(aligned);

  if (!align.hasConverged()) {
    PL_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "FPFH+RANSAC global init did not converge (%zu inliers).", align.getInliers().size());
    return best;
  }

  const Eigen::Isometry3d cand(align.getFinalTransformation().cast<double>());
  const std::size_t n_inliers = align.getInliers().size();

  const Eigen::Vector3d t = cand.translation();
  const double yaw = std::atan2(cand.linear()(1, 0), cand.linear()(0, 0));
  const Eigen::Vector3d z_axis = cand.linear().col(2);
  const double tilt = std::acos(std::min(1.0, std::max(-1.0, z_axis.z())));
  if (std::fabs(t.z()) > ransac_max_abs_z_ || tilt > ransac_max_tilt_ ||
      n_inliers < static_cast<std::size_t>(ransac_min_inliers_)) {
    PL_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "FPFH+RANSAC seed rejected: t=[%.2f, %.2f, %.2f], tilt=%.1f deg, %zu inliers "
      "(limits: |z|<=%.2f m, tilt<=%.1f deg, inliers>=%d).",
      t.x(), t.y(), t.z(), tilt * 180.0 / M_PI, n_inliers, ransac_max_abs_z_,
      ransac_max_tilt_ * 180.0 / M_PI, ransac_min_inliers_);
    return best;
  }

  best = cand;
  found = true;
  PL_INFO(
    this->get_logger(),
    "Global feature seed (FPFH+RANSAC): t=[%.3f, %.3f, %.3f], yaw=%.3f rad, tilt=%.1f deg, "
    "%zu inliers.",
    t.x(), t.y(), t.z(), yaw, tilt * 180.0 / M_PI, n_inliers);
  return best;
}

int SmallGicpRelocalizationNode::scanContextCandidates(
  std::vector<Eigen::Isometry3d> & seeds) const
{
  seeds.clear();
  if (sc_db_.entries.empty() || !accumulated_cloud_ || accumulated_cloud_->empty()) return 0;

  Eigen::Isometry3d t_odom_base = Eigen::Isometry3d::Identity();
  try {
    t_odom_base = tf2::transformToEigen(
      tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero,
                                  tf2::durationFromSec(0.2)).transform);
  } catch (const tf2::TransformException & ex) {
    static rclcpp::Clock throttle_clock{RCL_ROS_TIME};
    PL_WARN_THROTTLE(
      this->get_logger(), throttle_clock, 5000,
      "Scan Context: odom->base lookup failed (%s).", ex.what());
    return 0;
  }

  const Eigen::Vector3d c = t_odom_base.translation();
  std::vector<float> rel;
  rel.reserve(accumulated_cloud_->size() * 3);
  for (const auto & p : accumulated_cloud_->points) {
    rel.push_back(static_cast<float>(p.x - c.x()));
    rel.push_back(static_cast<float>(p.y - c.y()));
    rel.push_back(static_cast<float>(p.z - c.z()));
  }
  const auto desc = scan_context::make(rel, sc_db_.cfg);
  const auto cands = sc_db_.queryTopK(desc, sc_verify_candidates_, sc_top_k_);
  cand_scores_.clear();
  for (const auto & m : cands) {
    cand_scores_.push_back(m.score);
    const Eigen::Matrix3d R_mo =
      Eigen::AngleAxisd(m.yaw_map_odom, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Isometry3d s = Eigen::Isometry3d::Identity();
    s.linear() = R_mo;
    s.translation() = Eigen::Vector3d(m.x, m.y, m.z) - R_mo * t_odom_base.translation();
    seeds.push_back(s);
  }
  if (!cands.empty()) {
    static rclcpp::Clock throttle_clock{RCL_ROS_TIME};
    PL_INFO_THROTTLE(
      this->get_logger(), throttle_clock, 3000,
      "scan context: %zu candidates to verify, best score %.3f @ [%.1f, %.1f]",
      cands.size(), cands.front().score, cands.front().x, cands.front().y);
  }
  return static_cast<int>(seeds.size());
}
double SmallGicpRelocalizationNode::strictFitRatio(const Eigen::Isometry3d & T) const
{
  if (!source_ || source_->empty() || !target_tree_) return 0.0;
  const double d2 = verify_dist_ * verify_dist_;
  const double cos_min = std::cos(verify_normal_deg_ * M_PI / 180.0);
  const bool check_normal = verify_normal_deg_ > 0.0 && verify_normal_deg_ < 90.0 && target_;
  std::size_t hit = 0, used = 0;
  const std::size_t n = source_->size();
  for (std::size_t i = 0; i < n; ++i) {
    const auto & cov = source_->points[i].cov;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(
      cov.template block<3, 3>(0, 0).template cast<double>());
    const double nz = std::fabs(es.eigenvectors().col(0).z());
    if (nz > verify_nz_max_) continue;         
    ++used;
    const auto & sp = source_->points[i];
    Eigen::Vector4d p;
    p.head<3>() = T * Eigen::Vector3d(sp.x, sp.y, sp.z);
    p(3) = 1.0;
    std::size_t idx = 0;
    double sq = 0.0;
    if (!target_tree_->nearest_neighbor_search(p, &idx, &sq) || sq >= d2) continue;
    if (check_normal && idx < target_->size()) {
      // The scan normal, carried into the map frame, must agree with the map's
      // local normal. Sign is arbitrary for a plane, hence the absolute value.
      const Eigen::Vector3d ns = T.linear() * es.eigenvectors().col(0);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> et(
        target_->points[idx].cov.template block<3, 3>(0, 0).template cast<double>());
      const Eigen::Vector3d nt = et.eigenvectors().col(0);
      const double c = std::fabs(ns.normalized().dot(nt.normalized()));
      if (c < cos_min) continue;
    }
    ++hit;
  }
  if (used < static_cast<std::size_t>(verify_min_pts_)) return -1.0;
  return static_cast<double>(hit) / static_cast<double>(used);
}

void SmallGicpRelocalizationNode::resetAccumulation()
{
  accumulated_cloud_->clear();
  accum_started_ = false;
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  Eigen::Isometry3d t_map_base;
  tf2::fromMsg(msg->pose.pose, t_map_base);
  applyManualSeed(t_map_base);
}

void SmallGicpRelocalizationNode::applyManualSeed(const Eigen::Isometry3d & t_map_base)
{
  Eigen::Isometry3d t_odom_base = Eigen::Isometry3d::Identity();
  try {
    auto tf_stamped = tf_buffer_->lookupTransform(
      odom_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.2));
    t_odom_base = tf2::transformToEigen(tf_stamped.transform);
  } catch (const tf2::TransformException & ex) {
    PL_WARN(
      this->get_logger(),
      "manual seed: odom->base lookup failed (%s); assuming odom is at origin.", ex.what());
  }

  manual_seed_map_odom_ = t_map_base * t_odom_base.inverse();
  have_manual_seed_ = true;
  initialized_ = false;
  have_candidate_ = false;
  if (register_timer_ && register_timer_->is_canceled()) {
    register_timer_->reset();
  }

  const Eigen::Vector3d t = manual_seed_map_odom_.translation();
  const double yaw =
    std::atan2(manual_seed_map_odom_.linear()(1, 0), manual_seed_map_odom_.linear()(0, 0));
  PL_INFO(
    this->get_logger(),
    "Manual seed received: map<-odom t=[%.3f, %.3f, %.3f], yaw=%.3f rad. Re-localizing.",
    t.x(), t.y(), t.z(), yaw);
}

void SmallGicpRelocalizationNode::relocalizeCallback(
  const std::shared_ptr<interface::srv::Relocalize::Request> req,
  std::shared_ptr<interface::srv::Relocalize::Response> res)
{
  if (!req->pcd_path.empty() && req->pcd_path != prior_pcd_file_) {
    PL_INFO(this->get_logger(), "Relocalize: switching prior map to '%s'.",
                req->pcd_path.c_str());
    prior_pcd_file_ = req->pcd_path;
    loadGlobalMap(prior_pcd_file_);
    if (global_map_->empty()) {
      res->success = false;
      res->message = "failed to load prior map: " + req->pcd_path;
      PL_ERROR(this->get_logger(), "%s", res->message.c_str());
      return;
    }
    rebuildTarget();
  }

  Eigen::Isometry3d t_map_base = Eigen::Isometry3d::Identity();
  t_map_base.translation() = Eigen::Vector3d(req->x, req->y, req->z);
  t_map_base.linear() =
    (Eigen::AngleAxisd(req->yaw, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(req->pitch, Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(req->roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();

  applyManualSeed(t_map_base);

  res->success = true;
  char buf[192];
  std::snprintf(
    buf, sizeof(buf), "seeded at map<-base (%.3f, %.3f, %.3f) rpy(%.3f, %.3f, %.3f), re-localizing",
    req->x, req->y, req->z, req->roll, req->pitch, req->yaw);
  res->message = buf;
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  if (!has_first_scan_) {
    first_scan_stamp_ = msg->header.stamp;
    has_first_scan_ = true;
  }

  if (!accum_started_) {
    accum_start_stamp_ = msg->header.stamp;
    accum_started_ = true;
  }

  if (!accum_started_) {
    accum_start_stamp_ = msg->header.stamp;
    accum_started_ = true;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  *accumulated_cloud_ += *scan;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  // After the initial lock: if continuous correction is disabled, behave like
  // the old one-shot mode (keep broadcasting the locked result_t_).
  if (initialized_ && !continuous_correction_) {
    resetAccumulation();
    return;
  }
  if (accumulated_cloud_->empty()) {
    PL_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "No accumulated points to process.");
    return;
  }
  if (!target_ || target_->empty() || !target_tree_) {
    PL_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Target map is empty (PCD failed to load?). Skipping registration.");
    resetAccumulation();
    return;
  }

  if (!initialized_ && accum_started_) {
    if ((last_scan_time_ - accum_start_stamp_).seconds() < sc_query_accum_sec_) {
      return;   
    }
  }

  const std::size_t raw_points = accumulated_cloud_->size();
  if (!initialized_) {
    const double elapsed =
      has_first_scan_ ? (last_scan_time_ - first_scan_stamp_).seconds() : 0.0;
    if (elapsed < min_startup_time_) {
      resetAccumulation();
      return;
    }
    if (raw_points < static_cast<std::size_t>(min_source_points_)) {
      PL_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for a denser scan before first relocalization (%zu < %d points).", raw_points,
        min_source_points_);
      resetAccumulation();
      return;
    }
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    resetAccumulation();
    return;
  }

  register_->reduction.num_threads = num_threads_;
  register_->optimizer.max_iterations = max_iterations_;

  small_gicp::RegistrationResult result;
  bool manual_seed_used = false;
  if (!initialized_) {
    Eigen::Isometry3d seed;

    if (have_manual_seed_) {

      seed = manual_seed_map_odom_;
      manual_seed_used = true;
    } else {

      std::vector<Eigen::Isometry3d> seeds;
      const int n_cand = scanContextCandidates(seeds);
      bool found = false;
      Eigen::Isometry3d feat = previous_result_t_;

      // A rival only counts if it is a different place. Scan context returns
      // several keyframes from the same area, and those legitimately fit about
      // equally well; treating them as rivals made the relative test reject
      // every attempt at one start point (7 for 7, best margin 0.130 against a
      // 0.15 threshold) while the candidates sat within a few metres of each
      // other.
      double best = -1.0, second = -1.0;
      int best_i = -1;
      for (int ci = 0; ci < n_cand; ++ci) {
        register_->rejector.max_dist_sq = max_dist_sq_ * 4.0f;
        const auto coarse = register_->align(*target_, *source_, *target_tree_, seeds[ci]);
        register_->rejector.max_dist_sq = max_dist_sq_;
        const auto fine = register_->align(
          *target_, *source_, *target_tree_,
          coarse.converged ? coarse.T_target_source : seeds[ci]);
        const std::size_t ss = source_ ? source_->size() : 0;
        const bool loose_ok = ss > 0 &&
          fine.num_inliers >= static_cast<std::size_t>(min_inliers_) &&
          fine.num_inliers >= static_cast<std::size_t>(min_inlier_ratio_ * ss);
        if (!loose_ok) continue;
        const double r = strictFitRatio(fine.T_target_source);
        auto far_from = [&](int a, int b) {
          if (a < 0 || b < 0) return false;
          return (seeds[a].translation() - seeds[b].translation()).norm() >
                 rival_min_sep_;
        };
        if (r > best) {
          if (far_from(ci, best_i)) second = best;
          best = r;
          best_i = ci;
        } else if (far_from(ci, best_i) && r > second) {
          second = r;
        }
      }

      if (best_i >= 0) {
        // Raise the bar when the descriptor match was weak.
        double need = verify_ratio_;
        if (best_i < static_cast<int>(cand_scores_.size())) {
          const double excess = cand_scores_[best_i] - sc_score_good_;
          if (excess > 0.0) need += sc_score_penalty_ * excess;
        }
        const bool pass_abs = best >= need;
        const bool has_rival = second >= 0.0;
        const bool pass_rel = !has_rival || best >= second + verify_margin_;
        if (pass_abs && pass_rel) {
          PL_INFO(
            this->get_logger(), "candidate %d verified: fit ratio %.2f (runner-up %.2f)%s",
            best_i + 1, best, second,
            has_rival ? "" : " - no runner-up to compare against; needs one more consistent hit to lock");
          feat = seeds[best_i];
          found = true;
          sc_had_rival_ = has_rival;
        } else {
          PL_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "rejected: best fit ratio %.2f runner-up %.2f (threshold %.2f, min margin %.2f) - %s",
            best, second, need, verify_margin_,
            pass_abs ? "candidates indistinguishable" : "fit ratio too low");
        }
      }
      if (found) {
        manual_seed_map_odom_ = feat;
        // With one candidate the relative test is absent, not passed, and the
        // absolute threshold alone is unreliable: the stairwell bag locked on a
        // single candidate at fit ratio 0.97 with the pose flipped 176 deg and
        // 4.8 m out, while the correct lock scored 1.00 - no usable margin.
        // Without a runner-up, fall back to requiring two consistent fixes.
        manual_seed_used = sc_had_rival_;
      }
      if (!found) ++sc_fail_count_;

      if (!found && sc_fail_count_ >= sc_fallback_attempts_) {
        bool fp = false;
        const Eigen::Isometry3d fseed = featureGlobalSeed(fp);
        if (fp) {
          register_->rejector.max_dist_sq = max_dist_sq_ * 4.0f;
          const auto coarse = register_->align(*target_, *source_, *target_tree_, fseed);
          register_->rejector.max_dist_sq = max_dist_sq_;
          const auto fine = register_->align(
            *target_, *source_, *target_tree_,
            coarse.converged ? coarse.T_target_source : fseed);
          const double strict = strictFitRatio(fine.T_target_source);
          if (strict >= verify_ratio_ + verify_margin_) {
            PL_INFO(
              this->get_logger(), "FPFH seed verified: vertical-structure fit ratio %.2f", strict);
            feat = fseed;
            found = true;
            manual_seed_map_odom_ = feat;
            manual_seed_used = true;
          } else {
            PL_INFO(
              this->get_logger(), "FPFH seed fit ratio only %.2f (<%.2f), rejected",
              strict, verify_ratio_ + verify_margin_);
          }
        }
      }

      if (!found) {
        PL_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Feature global init found no usable candidate; retrying with the next scan.");
        have_candidate_ = false;
        resetAccumulation();
        return;
      }
      seed = feat;
    }

    register_->rejector.max_dist_sq = max_dist_sq_ * 4.0f;
    auto coarse = register_->align(*target_, *source_, *target_tree_, seed);
    const Eigen::Isometry3d init_for_fine = coarse.converged ? coarse.T_target_source : seed;
    register_->rejector.max_dist_sq = max_dist_sq_;
    result = register_->align(*target_, *source_, *target_tree_, init_for_fine);
  } else {
    register_->rejector.max_dist_sq = max_dist_sq_;
    result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);
  }
  const std::size_t src_size = source_ ? source_->size() : 0;
  const bool good_fit = src_size > 0 &&
    result.num_inliers >= static_cast<std::size_t>(min_inliers_) &&
    result.num_inliers >= static_cast<std::size_t>(min_inlier_ratio_ * src_size);

  if (!good_fit) {
    PL_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "GICP rejected: converged=%d inliers=%zu/%zu.", result.converged ? 1 : 0,
      result.num_inliers, src_size);
    // Break the consistency streak so a stale candidate can't lock later.
    have_candidate_ = false;
    resetAccumulation();
    return;
  }

  const Eigen::Isometry3d raw = result.T_target_source;

  if (!initialized_) {
    const bool consistent = have_candidate_ &&
      poseClose(raw, candidate_raw_, init_consistency_trans_, init_consistency_rot_);
    if (manual_seed_used &&
        !poseClose(raw, manual_seed_map_odom_, manual_seed_max_trans_, manual_seed_max_rot_)) {
      const Eigen::Isometry3d d = manual_seed_map_odom_.inverse() * raw;
      PL_WARN(
        this->get_logger(),
        "Manual-seed fit is too far from the seed (dt=%.2f m, dr=%.3f rad); "
        "keeping the seed and retrying.", d.translation().norm(),
        Eigen::AngleAxisd(d.rotation()).angle());
      have_candidate_ = false;
      resetAccumulation();
      return;
    }

    if (manual_seed_used || consistent) {
      have_manual_seed_ = false;      
      previous_result_t_ = raw;
      result_t_ = raw;
      initialized_ = true;
      if (!continuous_correction_) {
        register_timer_->cancel();
      }
      auto finish_msg = std_msgs::msg::Bool();
      finish_msg.data = true;
      icp_finish_pub_->publish(finish_msg);
      const Eigen::Vector3d t = result_t_.translation();
      const double yaw = std::atan2(result_t_.linear()(1, 0), result_t_.linear()(0, 0));
      const Eigen::Matrix3d Ht = result.H.block<3, 3>(3, 3);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(Ht);
      const double lmin = es.eigenvalues()(0), lmax = es.eigenvalues()(2);
      const Eigen::Vector3d weak = es.eigenvectors().col(0);
      const double cond = lmin > 1e-9 ? lmax / lmin : std::numeric_limits<double>::infinity();
      if (cond > degeneracy_warn_ratio_) {
        PL_WARN(
          this->get_logger(),
          "pose weakly constrained along [%.2f %.2f %.2f]: information-matrix condition number %.0f "
          "(threshold %.0f). Error along that direction is not constrained by registration; downstream must not assume isotropic accuracy.",
          weak.x(), weak.y(), weak.z(), cond, degeneracy_warn_ratio_);
      } else {
        PL_INFO(
          this->get_logger(), "translation observability normal: condition number %.0f, weakest direction [%.2f %.2f %.2f]",
          cond, weak.x(), weak.y(), weak.z());
      }
      PL_INFO(
        this->get_logger(),
        "Relocalization LOCKED (%s). map<-odom: t=[%.3f, %.3f, %.3f], yaw=%.3f rad. "
        "Continuous correction: %s.",
        manual_seed_used ? (have_manual_seed_ ? "manual seed" : "scan-context relative test")
                         : "2 consistent fits", t.x(), t.y(), t.z(), yaw,
        continuous_correction_ ? "ON" : "OFF");
    } else {
      candidate_raw_ = raw;
      have_candidate_ = true;
      previous_result_t_ = raw;
      PL_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Relocalization candidate found; awaiting a consistent confirmation.");
    }
  } else {
    if (poseClose(raw, previous_result_t_, max_correction_trans_, max_correction_rot_)) {
      previous_result_t_ = raw;
      result_t_ = raw;
      consecutive_rejects_ = 0;
    } else {
      const Eigen::Isometry3d d = previous_result_t_.inverse() * raw;
      ++consecutive_rejects_;
      PL_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Rejected large correction (outlier): dt=%.3f m, dr=%.3f rad. (%d in a row)",
        d.translation().norm(), Eigen::AngleAxisd(d.rotation()).angle(), consecutive_rejects_);

      // Outlier filtering needs a failure criterion.
      if (consecutive_rejects_ >= max_consecutive_rejects_) {
        const double strict = strictFitRatio(raw);
        if (strict >= verify_ratio_) {
          PL_WARN(
            this->get_logger(),
            "%d consecutive rejects and the new solution passes geometric verification (fit ratio %.2f); treating it as a real offset "
            "and accepting the correction.", consecutive_rejects_, strict);
          previous_result_t_ = raw;
          result_t_ = raw;
        } else {
          PL_ERROR(
            this->get_logger(),
            "%d consecutive rejects and the new solution is also untrustworthy (fit ratio %.2f < %.2f); localization has failed, "
            "falling back to global relocalization.", consecutive_rejects_, strict, verify_ratio_);
          initialized_ = false;
          have_candidate_ = false;
          have_manual_seed_ = false;
          sc_fail_count_ = 0;
        }
        consecutive_rejects_ = 0;
      }
    }
  }

  resetAccumulation();
}

bool SmallGicpRelocalizationNode::poseClose(
  const Eigen::Isometry3d & a, const Eigen::Isometry3d & b, double trans_tol, double rot_tol)
{
  const Eigen::Isometry3d d = a.inverse() * b;
  const double dt = d.translation().norm();
  const double dr = Eigen::AngleAxisd(d.rotation()).angle();
  return dt <= trans_tol && dr <= rot_tol;
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (!initialized_) {
    return;
  }
  if (last_tf_stamp_ == last_scan_time_) {
    return;
  }
  last_tf_stamp_ = last_scan_time_;

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)

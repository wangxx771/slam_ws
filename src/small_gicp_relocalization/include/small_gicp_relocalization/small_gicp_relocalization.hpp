// Copyright 2025 Lihan Chen
//
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "pcl/io/pcd_io.h"
#include "pcl/features/fpfh_omp.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/registration/sample_consensus_prerejective.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "interface/srv/relocalize.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "small_gicp_relocalization/scan_context.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace small_gicp_relocalization
{

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void buildMapFeatures();
  void ensureMapFeatures();
  Eigen::Isometry3d featureGlobalSeed(bool & found);
  void resetAccumulation();
  void performRegistration();
  void publishTransform();
  int scanContextCandidates(std::vector<Eigen::Isometry3d> & seeds) const;
  double strictFitRatio(const Eigen::Isometry3d & T) const;
  static bool poseClose(
    const Eigen::Isometry3d & a, const Eigen::Isometry3d & b, double trans_tol, double rot_tol);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_sub_;
  rclcpp::Service<interface::srv::Relocalize>::SharedPtr relocalize_srv_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr icp_finish_pub_;

  int num_threads_;
  int num_neighbors_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;
  int max_iterations_;

  double min_startup_time_;

  int min_source_points_;
  double min_inlier_ratio_;
  int min_inliers_;
  double init_consistency_trans_;
  double init_consistency_rot_;
  bool enable_initialpose_;
  std::string initialpose_topic_;
  std::string relocalize_service_;

  void applyManualSeed(const Eigen::Isometry3d & t_map_base);

  void rebuildTarget();

  void relocalizeCallback(
    const std::shared_ptr<interface::srv::Relocalize::Request> req,
    std::shared_ptr<interface::srv::Relocalize::Response> res);

  bool continuous_correction_;
  double max_correction_trans_;
  double max_correction_rot_;
  // Consecutive rejection count. Outlier filtering needs a failure criterion, or a
  // stray bad match and a real drift look alike and lock the system on a wrong pose.
  int consecutive_rejects_ = 0;
  int max_consecutive_rejects_;

  // Consistency streak bookkeeping for the initial lock.
  bool have_candidate_ = false;
  Eigen::Isometry3d candidate_raw_ = Eigen::Isometry3d::Identity();
  bool have_manual_seed_ = false;
  double manual_seed_max_trans_;
  double manual_seed_max_rot_;
  Eigen::Isometry3d manual_seed_map_odom_ = Eigen::Isometry3d::Identity();
  // Elapsed-time gate reference (message stamp of the first received scan).
  bool has_first_scan_ = false;
  rclcpp::Time first_scan_stamp_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string current_scan_frame_id_;
  std::string input_cloud_topic_;
  rclcpp::Time last_scan_time_;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;
  bool initialized_ = false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  scan_context::Database sc_db_;
  bool sc_enable_ = true;
  int sc_top_k_ = 30;
  int sc_verify_candidates_ = 5;
  double feature_leaf_size_;
  double feature_normal_radius_;
  double feature_fpfh_radius_;
  double feature_vertical_nz_max_;
  int ransac_max_iterations_;
  double ransac_inlier_fraction_;
  double ransac_similarity_threshold_;
  int ransac_correspondence_randomness_;
  double ransac_max_corr_dist_;
  int ransac_min_inliers_;
  double ransac_max_abs_z_;
  double ransac_max_tilt_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr feature_map_cloud_;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr map_features_;
  bool map_features_built_{false};
  double verify_dist_ = 0.5;
  double verify_ratio_ = 0.6;
  double verify_nz_max_ = 0.7;
  bool sc_had_rival_ = false;
  double degeneracy_warn_ratio_ = 30.0;
  double verify_margin_ = 0.15;    ///< Best fit must lead the runner-up by this much
  int verify_min_pts_ = 200;
  /// Max angle between a scan surface normal and the map normal it lands on.
  /// Proximity alone cannot tell a correct pose from a rotated one indoors: a
  /// scan yawed 76 deg still puts 95% of its wall points near *some* map wall.
  double verify_normal_deg_ = 25.0;
  /// Minimum separation for a candidate to count as a rival in the relative test.
  double rival_min_sep_ = 8.0;
  /// A weak descriptor match is asked for more geometric agreement. The
  /// scan-context score predicts the outcome: 0.131 gave 0.10 m, 0.245 gave
  /// 1.71 m because the retrieved keyframe was already ~1.5 m off and GICP
  /// faithfully aligned to the wrong place. Threshold becomes
  /// verify_ratio + sc_score_penalty * (score - sc_score_good).
  double sc_score_good_ = 0.15;
  double sc_score_penalty_ = 2.0;
  mutable std::vector<double> cand_scores_;
  int sc_fallback_attempts_ = 240;
  int sc_fail_count_ = 0;
  double sc_query_accum_sec_ = 2.0;
  rclcpp::Time accum_start_stamp_{0, 0, RCL_ROS_TIME};
  bool accum_started_ = false;
  rclcpp::Time last_tf_stamp_{0, 0, RCL_ROS_TIME};
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;


  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#pragma once


#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <cmath>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nanoflann/KDTreeVectorOfVectorsAdaptor.h"
#include "small_gicp_relocalization/scan_context.hpp"

#include "pose_graph_optimizer.h"
#include "preprocess.h"  // PointType / PointCloudXYZI

namespace point_lio
{

struct LoopClosureConfig
{
  bool enable = false;

  bool detect_loops = false;

  float keyframe_add_dist = 1.0f;    // m
  float keyframe_add_angle = 0.2f;   // rad
  int max_keyframes_per_cell = 0;
  float revisit_cell_size = 2.0f;

  float search_radius = 10.0f;
  float search_radius_z = 2.0f;

  float search_time_diff = 30.0f;
  int search_num = 25;
  float fitness_score = 0.3f;
  float icp_leaf_size = 0.4f;
  float icp_max_corr_dist = 150.0f;
  int cur_search_num = 0;
  float max_correction = 0.0f;
  float max_correction_rot = 0.0f;
  float min_travel_between = 0.0f;
  float loop_sigma_floor = 0.0f;

  double prior_sigma = 1e-6;
  double odom_sigma_rot = 0.05;
  double odom_sigma_trans = 0.1;

  bool keep_factors = true;
  int max_window = 500;
  int opt_iters = 6;
  int opt_iters_on_loop = 10;

  int gnss_opt_every = 10;
  // Evaluation only: runs the backend inline so a replay is bit-reproducible.
  // Production leaves this false - inline ICP and optimization cost the LIO
  // loop 4.26 ms per frame and push the peak to 60.6 ms, against 0.13 ms and
  // 18.5 ms on the backend thread, for the same accuracy (ATE 1.422 / 1.439 m).
  bool sync_detect = false;
  // Cap on closures between the same pair of revisit_cell_size cells. At 1 the
  // pair locks out permanently and closures covered only 20% of a two-lap run;
  // 3 gives 99% coverage and took ATE from 12.98 m to 1.42 m.
  int max_loops_per_cell_pair = 3;
  // Loop candidates per attempt, kept radius_candidate_sep apart. Left at 1: on
  // the 929 m ring, 3 and 6 produced 3x and 6x the candidates but the extra
  // ones fail ICP, so ATE RMSE stayed 1.437 / 1.452 / 1.444 m - inside the
  // 0.006 m run-to-run spread. Raise it only where revisits are dense.
  int radius_num_candidates = 1;
  float radius_candidate_sep = 3.0f;

  // Appearance-based detection (scan context). It needs no odometry, so unlike
  // radius search it can still find a revisit after drift exceeds search_radius.
  //
  // Retrieval quality is a property of the site, not of the algorithm, so
  // measure before enabling. tools/sc_auc.cpp on exported keyframes gave:
  //     outdoor VLP-16 (bags/park_ros2)   AUC 0.997   100% precision, 93% recall
  //     indoor Mid-360 corridor           AUC 0.68     28% precision, 48% recall
  // Off here because a corridor leaves two of three detections false with only
  // ICP behind them, and on the self-similar ring enabling it took ATE from
  // 1.42 m to 80.8 m. Outdoors it is worth turning on.
  bool sc_detect = false;
  // SC-PGO's SC_DIST_THRES range is 0.13-0.4. At 0.2 almost nothing passes:
  // on park that was 3 detections against 37 at 0.3, with recall 7.5% vs 93%.
  float sc_dist_thresh = 0.3f;
  // best < sc_ratio * second_best, an addition of ours, not part of Scan Context.
  // It assumes a true revisit stands clear of its rivals, which is false whenever
  // several consecutive keyframes all see the same place: on park at 0.75 it
  // rejected every single correct detection, taking 37/37 down to 0. Off (1.0)
  // by default; only raise it where revisits are isolated.
  float sc_ratio = 1.0f;
  int sc_num_candidates = 3;
  // Rebuild the ring-key index every this many keyframes, as SC-PGO does.
  // Rebuilding per query made search cost grow with trajectory length.
  int sc_tree_period = 30;
  int sc_exclude_recent = 30;
  // Must cover the structure the site actually has. The indoor corridor returns
  // almost nothing past 7 m, and sweeping this was worth more than any other
  // scan-context parameter: AUC 0.58 at 10 m, 0.62 at 20 m, 0.68 at 40 m, back
  // to 0.59 at 60 m, with recall 15% / 27% / 48% / 30%.
  float sc_max_radius = 60.0f;
  // 20x60 is the paper's grid and stays the default. Indoors more rings trade
  // recall for precision at a fixed threshold - at sc_max_radius 20 m the real
  // corridor gave 37%/27% at 20 rings, 61%/17% at 30 and 82%/11% at 40 - but
  // ranking AUC does not improve (0.62 to 0.60), so the extra rings only move
  // the operating point. Sector count barely matters: 120 was worse than 60
  // everywhere measured.
  int sc_num_ring = 20;
  int sc_num_sector = 60;
  float sc_max_height = 1e9f;
  // A single Livox frame covers only part of the 360 deg field because the scan
  // is non-repetitive, so one frame may be too sparse for a stable descriptor.
  // >0 accumulates that many earlier keyframe clouds into the descriptor.
  int sc_accum_frames = 0;
  // Seed ICP with the yaw implied by the scan-context shift. Off falls back to
  // the odometry-relative pose, which is only valid while drift is small.
  bool sc_use_yaw_guess = true;
  // Retain the dense scan per keyframe so ExportKeyframePatches can write it.
  bool keep_dense = false;

  // Backend loop period and the cap on frames waiting to be consumed.
  int backend_interval_ms = 50;
  int max_pending = 64;
};

struct KeyFrame
{
  double time = 0.0;
  Pose3 pose;
  Pose3 raw;
  PointCloudXYZI::Ptr cloud;
  // Full-resolution undistorted scan, kept only when patch export is on. The
  // registration cloud above is decimated to 500-900 points, two orders of
  // magnitude below what a keypoint extractor like LinK3D needs.
  PointCloudXYZI::Ptr dense;
  double travel = 0.0;
  std::vector<float> sc_desc;
  std::vector<float> sc_rkey;
};

// One frame handed to the backend. The caller only fills this in and returns.
struct PendingFrame
{
  double stamp = 0.0;
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  PointCloudXYZI::Ptr cloud;
  PointCloudXYZI::Ptr dense;
  bool has_gnss = false;
  Eigen::Vector3d gnss_enu = Eigen::Vector3d::Zero();
  Eigen::Vector3d gnss_sigma = Eigen::Vector3d::Zero();
};

class LoopClosure
{
public:
  ~LoopClosure() { Stop(); }

  void Init(const LoopClosureConfig & cfg);
  void Start();
  void Stop();

  // Enqueue a frame. Never blocks on graph work: the keyframe test and a cloud
  // copy are all that run on the caller's thread. Returns true when the frame
  // was taken as a keyframe.
  bool AddFrame(
    double stamp, const Eigen::Vector3d & pos, const Eigen::Matrix3d & rot,
    const PointCloudXYZI::Ptr & body_cloud,
    const Eigen::Vector3d * gnss_enu = nullptr,
    const Eigen::Vector3d * gnss_sigma = nullptr,
    const PointCloudXYZI::Ptr & dense_cloud = nullptr);

  // Returns true and writes the current correction when the backend has
  // published a new one since the last call.
  bool PollCorrection(Pose3 * out);

  Pose3 LatestPose() const;

  Pose3 Correction() const;

  int NumLoops() const { return num_loops_; }

  int NumKeyframes() const
  {
    std::lock_guard<std::mutex> lk(kf_mutex_);
    return static_cast<int>(keyframes_.size());
  }

  std::vector<std::tuple<double, Pose3, Pose3>> AllKeyframePosesTimed() const
  {
    std::lock_guard<std::mutex> lk(kf_mutex_);
    std::vector<std::tuple<double, Pose3, Pose3>> out;
    out.reserve(keyframes_.size());
    for (const auto & kf : keyframes_) out.emplace_back(kf.time, kf.raw, kf.pose);
    return out;
  }
  struct LoopInfo
  {
    int from = -1, to = -1;
    double t_from = 0, t_to = 0;
    double dist = 0;
    double fitness = -1;
  };
  LoopInfo LastLoop() const
  {
    std::lock_guard<std::mutex> lk(loop_mutex_);
    return last_loop_;
  }
  struct Stats
  {
    std::size_t keyframes = 0;
    std::size_t clouds_held = 0;
    std::size_t cloud_points = 0;
    std::size_t poses = 0;
    std::size_t betweens = 0;
    std::size_t cells = 0;
  };
  Stats GetStats() const;

  // Write the keyframe database open3d_loc expects: one body-frame PCD per
  // keyframe under <dir>/patches, and <dir>/poses.txt listing
  // "<file> x y z qw qx qy qz" with the optimized pose. Its global
  // initialization matches the live scan against these patches, so a map
  // without them leaves it with zero candidates. Returns the number written;
  // requires loop closure to have been enabled, since that is what holds the
  // keyframes.
  std::size_t ExportKeyframePatches(const std::string & dir) const;

private:
  bool NeedKeyFrame(const Eigen::Vector3d & pos, const Eigen::Matrix3d & rot) const;
  void BackendThread();
  // Consume every queued frame, then optimize and detect once. Also the body of
  // the synchronous path, so both modes run identical logic.
  void BackendStep();
  void IngestFrame(const PendingFrame & f);
  void PublishCorrection();
  std::vector<float> MakeDescriptor(
    const PointCloudXYZI::Ptr & body_cloud) const;
  bool DetectBySc(int cur, int * pre) const;
  bool DetectByRadius(int cur, std::vector<int> * pre) const;
  void AddOdomFactor(const Pose3 & cur_raw);
  bool DrainLoopQueue();
  void DetectOnce();
  // A loop candidate plus where it came from. The origin decides the ICP seed:
  // a scan-context hit carries a reliable relative yaw but no translation, while
  // a radius hit means odometry is still trustworthy enough to seed both.
  struct Candidate
  {
    int cur = -1;
    int pre = -1;
    bool from_sc = false;
    double sc_yaw = 0.0;
    float sc_dist = 2.0f;
  };
  // Ordered by preference: scan context first, radius as fallback. Both are
  // returned so a candidate rejected by ICP does not shadow the other channel.
  void DetectLoopPair(std::vector<Candidate> * out);
  bool TryCandidate(const Candidate & cand);
  // Submap around `key`, expressed in that keyframe's own frame. Building it in
  // the world frame instead would make the ICP starting error equal to the
  // accumulated drift, which is exactly what a loop is supposed to remove.
  void BuildLocalSubmap(PointCloudXYZI::Ptr & out, int key, int search_num) const;
  std::uint64_t CellPairKey(const Eigen::Vector3d & a, const Eigen::Vector3d & b) const;
  bool CellPairSaturated(const Eigen::Vector3d & a, const Eigen::Vector3d & b) const;

  LoopClosureConfig cfg_;

  std::vector<KeyFrame> keyframes_;
  mutable std::mutex kf_mutex_;

  struct CellHash
  {
    std::size_t operator()(const Eigen::Vector3i & c) const noexcept
    {
      return (static_cast<std::size_t>(c.x()) * 73856093u) ^
             (static_cast<std::size_t>(c.y()) * 19349663u) ^
             (static_cast<std::size_t>(c.z()) * 83492791u);
    }
  };
  std::unordered_map<Eigen::Vector3i, std::deque<int>, CellHash> cell_clouds_;

  PointCloudXYZI::Ptr latest_cloud_;
  int latest_cloud_idx_ = -1;
  Eigen::Vector3i PosToCell(const Eigen::Vector3d & p) const
  {
    const double s = cfg_.revisit_cell_size > 1e-3 ? cfg_.revisit_cell_size : 1.0;
    return Eigen::Vector3i(
      static_cast<int>(std::floor(p.x() / s)),
      static_cast<int>(std::floor(p.y() / s)),
      static_cast<int>(std::floor(p.z() / s)));
  }

  PoseGraphOptimizer opt_;
  std::unordered_map<int, int> loop_pairs_;
  std::unordered_map<std::uint64_t, int> cell_pair_loops_;
  mutable std::mutex pairs_mutex_;
  struct LoopConstraint
  {
    int from;
    int to;
    Pose3 relative;
    DiagNoise6 noise;
  };
  std::deque<LoopConstraint> loop_queue_;
  mutable std::mutex loop_mutex_;
  LoopInfo last_loop_;

  std::deque<PendingFrame> pending_;
  mutable std::mutex pending_mutex_;
  // Keyframe spacing is decided on the caller's thread, so it needs its own copy
  // of the last accepted pose rather than reaching into keyframes_.
  Pose3 last_enqueued_;
  bool has_last_enqueued_ = false;
  std::size_t dropped_pending_ = 0;

  Pose3 correction_;
  std::uint64_t correction_version_ = 0;
  std::uint64_t correction_seen_ = 0;
  mutable std::mutex correction_mutex_;

  scan_context::Config sc_cfg_;

  // Ring-key index over keyframes, excluding the most recent sc_exclude_recent.
  // Coarse retrieval only; the shortlist it returns is then scored with the full
  // descriptor and verified by ICP.
  using RingKeyTree =
    KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<float>>, float>;
  mutable std::vector<std::vector<float>> rkey_index_;
  mutable std::shared_ptr<RingKeyTree> rkey_tree_;
  mutable int rkey_tree_size_ = 0;
  mutable int since_tree_build_ = 0;

  std::thread thread_;
  double last_opt_ms_ = 0.0;
  int last_opt_iters_ = 0;

public:
  double LastOptMs() const {return last_opt_ms_;}
  int LastOptIters() const {return last_opt_iters_;}

private:
  std::atomic<bool> running_{false};
  int attempts_ = 0, accepted_ = 0, rej_pair_ = 0, rej_range_ = 0, rej_cell_ = 0;
  int rej_empty_ = 0, rej_icp_ = 0, rej_corr_ = 0, rej_rot_ = 0;
  mutable int rej_z_ = 0;
  int hit_sc_ = 0, hit_radius_ = 0;
  mutable int rej_sc_abs_ = 0, rej_sc_ratio_ = 0;
  mutable double sc_last_yaw_ = 0.0;
  mutable float sc_last_dist_ = 2.0f;
  std::atomic<int> num_loops_{0};
  int gnss_since_opt_ = 0;

  pcl::VoxelGrid<PointType> icp_filter_;
};

}  // namespace point_lio

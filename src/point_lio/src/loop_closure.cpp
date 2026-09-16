#include "slam_log/log.h"
#include "loop_closure.h"

#include <omp.h>

#include <pcl/registration/icp.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <cmath>
#include <limits>

namespace point_lio
{

namespace
{

void TransformCloud(
  const PointCloudXYZI::Ptr & in, const Pose3 & pose, PointCloudXYZI & out)
{
  out.points.reserve(out.points.size() + in->points.size());
  const Eigen::Matrix3f R = pose.R.cast<float>();
  const Eigen::Vector3f t = pose.t.cast<float>();
  for (const auto & p : in->points) {
    const Eigen::Vector3f q = R * Eigen::Vector3f(p.x, p.y, p.z) + t;
    PointType o = p;
    o.x = q.x();
    o.y = q.y();
    o.z = q.z();
    out.points.push_back(o);
  }
}

DiagNoise6 MakeNoise6(double rot_sigma, double trans_sigma)
{
  DiagNoise6 n;
  n.sigma = {{rot_sigma, rot_sigma, rot_sigma, trans_sigma, trans_sigma, trans_sigma}};
  return n;
}

}  // namespace

void LoopClosure::Init(const LoopClosureConfig & cfg)
{
  cfg_ = cfg;
  opt_.setKeepFactors(cfg_.keep_factors);
  opt_.setMaxWindow(static_cast<std::size_t>(std::max(1, cfg_.max_window)));
  icp_filter_.setLeafSize(cfg_.icp_leaf_size, cfg_.icp_leaf_size, cfg_.icp_leaf_size);
  sc_cfg_.max_radius = cfg_.sc_max_radius;
  sc_cfg_.num_ring = cfg_.sc_num_ring;
  sc_cfg_.num_sector = cfg_.sc_num_sector;
  sc_cfg_.max_height = cfg_.sc_max_height;
}

void LoopClosure::Start()
{
  // Nothing runs in the background in synchronous mode: AddFrame does the whole
  // backend step inline so an offline replay stays bit-reproducible.
  if (!cfg_.enable || running_ || cfg_.sync_detect) return;
  running_ = true;
  thread_ = std::thread(&LoopClosure::BackendThread, this);
}

void LoopClosure::Stop()
{
  running_ = false;
  if (thread_.joinable()) thread_.join();
}

bool LoopClosure::NeedKeyFrame(const Eigen::Vector3d & pos, const Eigen::Matrix3d & rot) const
{
  if (!has_last_enqueued_) return true;
  const Pose3 & last = last_enqueued_;

  const bool moved = (pos - last.t).norm() > cfg_.keyframe_add_dist;
  const Eigen::Vector3d dw = Log(Eigen::Matrix3d(last.R.transpose() * rot));
  const bool turned = dw.norm() > cfg_.keyframe_add_angle;
  if (!moved && !turned) return false;
  return true;
}

void LoopClosure::AddOdomFactor(const Pose3 & cur_raw)
{
  if (keyframes_.empty()) {
    const int idx = opt_.addPose(cur_raw);
    opt_.addPrior(idx, cur_raw, MakeNoise6(cfg_.prior_sigma, cfg_.prior_sigma));
    return;
  }

  const KeyFrame & last = keyframes_.back();
  const Pose3 rel = last.raw.inverse().compose(cur_raw);
  const int idx_from = static_cast<int>(keyframes_.size()) - 1;

  const int idx_to = opt_.addPose(last.pose.compose(rel));
  opt_.addBetween(
    idx_from, idx_to, rel, MakeNoise6(cfg_.odom_sigma_rot, cfg_.odom_sigma_trans));
}

bool LoopClosure::DrainLoopQueue()
{
  std::deque<LoopConstraint> pending;
  {
    std::lock_guard<std::mutex> lk(loop_mutex_);
    pending.swap(loop_queue_);
  }
  if (pending.empty()) return false;
  for (const auto & c : pending) {
    opt_.addBetween(c.from, c.to, c.relative, c.noise);
  }
  num_loops_ += static_cast<int>(pending.size());
  return true;
}

bool LoopClosure::AddFrame(
  double stamp, const Eigen::Vector3d & pos, const Eigen::Matrix3d & rot,
  const PointCloudXYZI::Ptr & body_cloud,
  const Eigen::Vector3d * gnss_enu, const Eigen::Vector3d * gnss_sigma,
  const PointCloudXYZI::Ptr & dense_cloud)
{
  if (!cfg_.enable) return false;
  if (!NeedKeyFrame(pos, rot)) return false;

  PendingFrame f;
  f.stamp = stamp;
  f.pos = pos;
  f.rot = rot;
  f.cloud = std::make_shared<PointCloudXYZI>(*body_cloud);
  if (cfg_.keep_dense && dense_cloud && !dense_cloud->empty()) {
    f.dense = std::make_shared<PointCloudXYZI>(*dense_cloud);
  }
  if (gnss_enu && gnss_sigma) {
    f.has_gnss = true;
    f.gnss_enu = *gnss_enu;
    f.gnss_sigma = *gnss_sigma;
  }

  last_enqueued_ = Pose3(rot, pos);
  has_last_enqueued_ = true;

  if (cfg_.sync_detect) {
    // Deterministic path: no queue, no thread, identical ordering every run.
    IngestFrame(f);
    BackendStep();
    return true;
  }

  {
    std::lock_guard<std::mutex> lk(pending_mutex_);
    // A bounded queue is the whole point: if the backend cannot keep up the
    // frames are dropped rather than growing until memory runs out. Dropping a
    // keyframe costs graph density, never the LIO loop.
    if (cfg_.max_pending > 0 &&
        static_cast<int>(pending_.size()) >= cfg_.max_pending) {
      pending_.pop_front();
      ++dropped_pending_;
    }
    pending_.push_back(std::move(f));
  }
  return true;
}

bool LoopClosure::PollCorrection(Pose3 * out)
{
  std::lock_guard<std::mutex> lk(correction_mutex_);
  if (correction_version_ == correction_seen_) return false;
  correction_seen_ = correction_version_;
  if (out) *out = correction_;
  return true;
}

void LoopClosure::PublishCorrection()
{
  Pose3 c;
  {
    std::lock_guard<std::mutex> lk(kf_mutex_);
    if (keyframes_.empty()) return;
    const KeyFrame & k = keyframes_.back();
    c = k.pose.compose(k.raw.inverse());
  }
  std::lock_guard<std::mutex> lk(correction_mutex_);
  correction_ = c;
  ++correction_version_;
}

std::vector<float> LoopClosure::MakeDescriptor(
  const PointCloudXYZI::Ptr & body_cloud) const
{
  if (!cfg_.sc_detect || !body_cloud || body_cloud->empty()) return {};
  std::vector<float> pts;
  pts.reserve(body_cloud->points.size() * 3);
  for (const auto & p : body_cloud->points) {
    pts.push_back(p.x);
    pts.push_back(p.y);
    pts.push_back(p.z);
  }
  return scan_context::make(pts, sc_cfg_);
}

void LoopClosure::IngestFrame(const PendingFrame & f)
{
  const Pose3 cur_raw(f.rot, f.pos);
  AddOdomFactor(cur_raw);

  if (f.has_gnss) {
    DiagNoise3 n;
    n.sigma = {{f.gnss_sigma.x(), f.gnss_sigma.y(), f.gnss_sigma.z()}};
    opt_.addGps(static_cast<int>(opt_.numPoses()) - 1, f.gnss_enu, n);
    ++gnss_since_opt_;
  }

  KeyFrame kf;
  kf.time = f.stamp;
  kf.raw = cur_raw;
  kf.pose = opt_.getPose(static_cast<int>(opt_.numPoses()) - 1);
  kf.cloud = f.cloud;
  kf.dense = f.dense;
  kf.sc_desc = MakeDescriptor(f.cloud);
  if (cfg_.sc_accum_frames > 0 && !kf.sc_desc.empty()) {
    // Fold earlier keyframes into the descriptor, expressed in this keyframe's
    // frame. Occupancy of a lone Livox frame is low enough that the ring key is
    // dominated by which sectors happened to be scanned.
    std::vector<float> pts;
    {
      std::lock_guard<std::mutex> lk(kf_mutex_);
      const int n = static_cast<int>(keyframes_.size());
      const Pose3 anchor_inv = cur_raw.inverse();
      for (int j = std::max(0, n - cfg_.sc_accum_frames); j < n; ++j) {
        const auto & c = keyframes_[j].cloud;
        if (!c) continue;
        const Pose3 rel = anchor_inv.compose(keyframes_[j].raw);
        const Eigen::Matrix3f R = rel.R.cast<float>();
        const Eigen::Vector3f t = rel.t.cast<float>();
        for (const auto & p : c->points) {
          const Eigen::Vector3f q = R * Eigen::Vector3f(p.x, p.y, p.z) + t;
          pts.push_back(q.x()); pts.push_back(q.y()); pts.push_back(q.z());
        }
      }
    }
    for (const auto & p : f.cloud->points) {
      pts.push_back(p.x); pts.push_back(p.y); pts.push_back(p.z);
    }
    kf.sc_desc = scan_context::make(pts, sc_cfg_);
  }
  if (!kf.sc_desc.empty()) {
    kf.sc_rkey = scan_context::ringKey(kf.sc_desc, sc_cfg_);
  }

  std::lock_guard<std::mutex> lk(kf_mutex_);
  if (!keyframes_.empty()) {
    kf.travel = keyframes_.back().travel + (cur_raw.t - keyframes_.back().raw.t).norm();
  }
  keyframes_.push_back(std::move(kf));

  latest_cloud_ = f.cloud;
  latest_cloud_idx_ = static_cast<int>(keyframes_.size()) - 1;

  if (cfg_.max_keyframes_per_cell > 0) {
    auto & lst = cell_clouds_[PosToCell(cur_raw.t)];
    lst.push_back(latest_cloud_idx_);
    while (static_cast<int>(lst.size()) > cfg_.max_keyframes_per_cell) {
      const int old_idx = lst.front();
      lst.pop_front();
      if (old_idx >= 0 && old_idx < static_cast<int>(keyframes_.size())) {
        keyframes_[old_idx].cloud.reset();
      }
    }
  }
}

void LoopClosure::BackendStep()
{
  const bool has_loop = DrainLoopQueue();
  const bool has_gnss =
    gnss_since_opt_ > 0 && gnss_since_opt_ >= std::max(1, cfg_.gnss_opt_every);

  if (has_loop || has_gnss) {
    const double t0 = omp_get_wtime();
    last_opt_iters_ =
      opt_.optimize(has_loop ? cfg_.opt_iters_on_loop : cfg_.opt_iters, 1e-4);
    last_opt_ms_ = (omp_get_wtime() - t0) * 1000.0;
    gnss_since_opt_ = 0;
    {
      std::lock_guard<std::mutex> lk(kf_mutex_);
      const int n = static_cast<int>(keyframes_.size());
      for (int i = 0; i < n; ++i) keyframes_[i].pose = opt_.getPose(i);
    }
    PublishCorrection();
  }

  if (cfg_.detect_loops) DetectOnce();
}

void LoopClosure::BackendThread()
{
  while (running_) {
    std::deque<PendingFrame> batch;
    {
      std::lock_guard<std::mutex> lk(pending_mutex_);
      batch.swap(pending_);
    }
    for (const auto & f : batch) IngestFrame(f);

    if (!batch.empty() || cfg_.detect_loops) BackendStep();

    std::this_thread::sleep_for(
      std::chrono::milliseconds(std::max(1, cfg_.backend_interval_ms)));
  }
}

std::uint64_t LoopClosure::CellPairKey(
  const Eigen::Vector3d & a, const Eigen::Vector3d & b) const
{
  const Eigen::Vector3i ca = PosToCell(a);
  const Eigen::Vector3i cb = PosToCell(b);
  auto pack = [](const Eigen::Vector3i & c) {
    return (static_cast<std::uint64_t>(static_cast<std::uint16_t>(c.x())) * 73856093ull) ^
           (static_cast<std::uint64_t>(static_cast<std::uint16_t>(c.y())) * 19349663ull) ^
           (static_cast<std::uint64_t>(static_cast<std::uint16_t>(c.z())) * 83492791ull);
  };
  const std::uint64_t ka = pack(ca), kb = pack(cb);
  const std::uint64_t lo = ka < kb ? ka : kb;
  const std::uint64_t hi = ka < kb ? kb : ka;
  return (hi << 32) | lo;
}

bool LoopClosure::CellPairSaturated(
  const Eigen::Vector3d & a, const Eigen::Vector3d & b) const
{
  if (cfg_.max_loops_per_cell_pair <= 0) return false;
  const std::uint64_t k = CellPairKey(a, b);
  std::lock_guard<std::mutex> lk(pairs_mutex_);
  const auto it = cell_pair_loops_.find(k);
  return it != cell_pair_loops_.end() && it->second >= cfg_.max_loops_per_cell_pair;
}

LoopClosure::Stats LoopClosure::GetStats() const
{
  Stats s;
  {
    std::lock_guard<std::mutex> lk(kf_mutex_);
    s.keyframes = keyframes_.size();
    for (const auto & kf : keyframes_) {
      if (kf.cloud) {
        ++s.clouds_held;
        s.cloud_points += kf.cloud->points.size();
      }
    }
    s.cells = cell_clouds_.size();
  }
  s.poses = opt_.numPoses();
  s.betweens = opt_.numBetweens();
  return s;
}

std::size_t LoopClosure::ExportKeyframePatches(const std::string & dir) const
{
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::create_directories(dir + "/patches", ec);
  if (ec) {
    PL_ERROR(
      rclcpp::get_logger("LoopClosure"), "cannot create %s/patches: %s", dir.c_str(),
      ec.message().c_str());
    return 0;
  }

  struct Item { Pose3 pose; PointCloudXYZI::Ptr cloud; };
  std::vector<Item> items;
  {
    std::lock_guard<std::mutex> lk(kf_mutex_);
    items.reserve(keyframes_.size());
    for (const auto & kf : keyframes_) {
      // Clouds dropped by the per-cell budget cannot be exported.
      // Prefer the dense scan: a decimated one yields too few keypoints for the
      // extractors that consume these patches.
      const auto & c = (kf.dense && !kf.dense->empty()) ? kf.dense : kf.cloud;
      if (!c || c->empty()) continue;
      items.push_back(Item{kf.pose, c});
    }
  }
  if (items.empty()) return 0;

  std::ofstream poses(dir + "/poses.txt");
  if (!poses) {
    PL_ERROR(rclcpp::get_logger("LoopClosure"), "cannot write %s/poses.txt", dir.c_str());
    return 0;
  }
  poses << std::fixed << std::setprecision(6);

  std::size_t n = 0;
  for (const auto & it : items) {
    char name[32];
    std::snprintf(name, sizeof(name), "kf_%06zu.pcd", n);
    PointCloudXYZI out = *it.cloud;
    out.width = out.points.size();
    out.height = 1;
    out.is_dense = true;
    if (pcl::io::savePCDFileBinary(dir + "/patches/" + name, out) != 0) continue;
    const Eigen::Quaterniond q(it.pose.R);
    poses << name << ' ' << it.pose.t.x() << ' ' << it.pose.t.y() << ' ' << it.pose.t.z()
          << ' ' << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << '\n';
    ++n;
  }
  PL_INFO(
    rclcpp::get_logger("LoopClosure"), "exported %zu keyframe patches to %s/patches", n,
    dir.c_str());
  return n;
}

Pose3 LoopClosure::LatestPose() const
{
  std::lock_guard<std::mutex> lk(kf_mutex_);
  return keyframes_.empty() ? Pose3() : keyframes_.back().pose;
}

Pose3 LoopClosure::Correction() const
{
  std::lock_guard<std::mutex> lk(kf_mutex_);
  if (keyframes_.empty()) return Pose3();
  const KeyFrame & k = keyframes_.back();
  return k.pose.compose(k.raw.inverse());
}

void LoopClosure::BuildLocalSubmap(
  PointCloudXYZI::Ptr & out, int key, int search_num) const
{
  out->clear();
  struct NearKf { Pose3 rel; PointCloudXYZI::Ptr cloud; };
  std::vector<NearKf> snap;
  snap.reserve(static_cast<std::size_t>(2 * search_num + 1));
  {
    std::lock_guard<std::mutex> lk(kf_mutex_);
    const int n = static_cast<int>(keyframes_.size());
    if (key < 0 || key >= n) return;
    const Pose3 anchor_inv = keyframes_[key].pose.inverse();
    for (int i = -search_num; i <= search_num; ++i) {
      const int j = key + i;
      if (j < 0 || j >= n) continue;
      PointCloudXYZI::Ptr c = keyframes_[j].cloud;
      // The cloud may have been reclaimed by the space budget. When this is the
      // newest frame, fall back to latest_cloud_ - the loop origin must always be
      // available or a revisited area never triggers a closure.
      if (!c && j == latest_cloud_idx_) c = latest_cloud_;
      if (!c || c->empty()) continue;
      snap.push_back(NearKf{anchor_inv.compose(keyframes_[j].pose), c});
    }
  }
  for (const auto & kf : snap) TransformCloud(kf.cloud, kf.rel, *out);
  if (out->points.empty()) return;
  out->width = out->points.size();
  out->height = 1;

  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(cfg_.icp_leaf_size, cfg_.icp_leaf_size, cfg_.icp_leaf_size);
  vg.setInputCloud(out);
  PointCloudXYZI::Ptr tmp(new PointCloudXYZI());
  vg.filter(*tmp);
  out.swap(tmp);
}

bool LoopClosure::DetectBySc(int cur, int * pre) const
{
  std::vector<float> q_desc, q_rkey;
  int newest = -1;
  {
    std::lock_guard<std::mutex> lk(kf_mutex_);
    const int n = static_cast<int>(keyframes_.size());
    if (cur < 0 || cur >= n) return false;
    if (keyframes_[cur].sc_desc.empty()) return false;
    q_desc = keyframes_[cur].sc_desc;
    q_rkey = keyframes_[cur].sc_rkey;
    newest = n - 1 - std::max(0, cfg_.sc_exclude_recent);

    // Stage 0: keep the ring-key index in step with the keyframes, rebuilt on a
    // period rather than per query. The ring key is rotation invariant, so this
    // survives approaching a place from any direction.
    if (newest >= 0 &&
        (!rkey_tree_ || newest + 1 != rkey_tree_size_) &&
        (since_tree_build_ >= std::max(1, cfg_.sc_tree_period) || !rkey_tree_)) {
      rkey_index_.clear();
      rkey_index_.reserve(static_cast<std::size_t>(newest + 1));
      for (int i = 0; i <= newest; ++i) {
        if (keyframes_[i].sc_rkey.size() ==
            static_cast<std::size_t>(cfg_.sc_num_ring)) {
          rkey_index_.push_back(keyframes_[i].sc_rkey);
        } else {
          rkey_index_.emplace_back(cfg_.sc_num_ring, 0.0f);
        }
      }
      rkey_tree_ = std::make_shared<RingKeyTree>(cfg_.sc_num_ring, rkey_index_, 10);
      rkey_tree_size_ = newest + 1;
      since_tree_build_ = 0;
    } else {
      ++since_tree_build_;
    }
  }
  if (!rkey_tree_ || rkey_tree_size_ <= 0) return false;

  // Stage 1: shortlist by ring key.
  const std::size_t k =
    std::min<std::size_t>(std::max(1, cfg_.sc_num_candidates), rkey_tree_size_);
  std::vector<std::size_t> idx(k);
  std::vector<float> dist(k);
  nanoflann::KNNResultSet<float> res(k);
  res.init(idx.data(), dist.data());
  rkey_tree_->index->findNeighbors(res, q_rkey.data(), nanoflann::SearchParams(10));

  // Stage 2: score the shortlist with the full descriptor, and require the best
  // to stand clear of the runner-up. Absolute distance alone cannot separate a
  // true revisit from self-similar geometry.
  float best = std::numeric_limits<float>::max();
  float second = std::numeric_limits<float>::max();
  int best_i = -1, best_shift = 0;
  {
    std::lock_guard<std::mutex> lk(kf_mutex_);
    const int n = static_cast<int>(keyframes_.size());
    for (std::size_t m = 0; m < k; ++m) {
      const int j = static_cast<int>(idx[m]);
      if (j < 0 || j >= n || j > newest) continue;
      if (keyframes_[j].sc_desc.empty()) continue;
      int sh = 0;
      const float d = scan_context::align(q_desc, keyframes_[j].sc_desc, sc_cfg_, &sh);
      if (d < best) {
        if (best_i >= 0 && std::abs(j - best_i) > cfg_.sc_exclude_recent) second = best;
        best = d;
        best_i = j;
        best_shift = sh;
      } else if (best_i >= 0 && std::abs(j - best_i) > cfg_.sc_exclude_recent &&
                 d < second) {
        second = d;
      }
    }
  }
  if (best_i < 0 || best >= cfg_.sc_dist_thresh) { ++rej_sc_abs_; return false; }
  if (cfg_.sc_ratio < 1.0f && second < std::numeric_limits<float>::max() &&
      best >= cfg_.sc_ratio * second) {
    ++rej_sc_ratio_;
    return false;
  }

  *pre = best_i;
  sc_last_dist_ = best;
  // align() compares query bin j against candidate bin (j + shift), and a yaw of
  // +dtheta moves a point's bin by -dtheta*S/2pi, so the shift is the yaw of cur
  // relative to pre.
  sc_last_yaw_ = best_shift * 2.0 * M_PI / cfg_.sc_num_sector;
  while (sc_last_yaw_ > M_PI) sc_last_yaw_ -= 2.0 * M_PI;
  return true;
}

bool LoopClosure::DetectByRadius(int cur, std::vector<int> * pre) const
{
  struct PoseSnap { Eigen::Vector3d t; double time; double travel; };
  std::vector<PoseSnap> snap;
  {
    std::lock_guard<std::mutex> lk(kf_mutex_);
    snap.reserve(keyframes_.size());
    for (const auto & kf : keyframes_) {
      snap.push_back(PoseSnap{kf.pose.t, kf.time, kf.travel});
    }
  }
  const int n = static_cast<int>(snap.size());
  if (cur < 0 || cur >= n) return false;

  const double t_cur = snap[cur].time;
  const double travel_cur = snap[cur].travel;
  const double z_cur = snap[cur].t.z();
  const double r2 = static_cast<double>(cfg_.search_radius) * cfg_.search_radius;

  // Linear scan, then thin: with at most a few thousand keyframes, O(N) once
  // beats rebuilding a kd-tree per attempt. The height test comes after the
  // radius test so its counter reports candidates actually lost to height and
  // not every keyframe on another floor.
  std::vector<std::pair<double, int>> in_range;
  for (int i = 0; i < n; ++i) {
    if (i == cur) continue;
    if (std::fabs(snap[i].time - t_cur) <= cfg_.search_time_diff) continue;
    if (cfg_.min_travel_between > 0.0f &&
        std::fabs(travel_cur - snap[i].travel) < cfg_.min_travel_between) {
      continue;
    }
    const double d2 = (snap[i].t - snap[cur].t).squaredNorm();
    if (d2 >= r2) continue;
    if (cfg_.search_radius_z > 0.0f &&
        std::fabs(snap[i].t.z() - z_cur) > cfg_.search_radius_z) {
      ++rej_z_;
      continue;
    }
    in_range.push_back({d2, i});
  }
  if (in_range.empty()) return false;
  std::sort(in_range.begin(), in_range.end());

  const int want = std::max(1, cfg_.radius_num_candidates);
  const double sep2 =
    static_cast<double>(cfg_.radius_candidate_sep) * cfg_.radius_candidate_sep;
  pre->clear();
  for (const auto & cand : in_range) {
    bool too_close = false;
    for (int taken : *pre) {
      if ((snap[cand.second].t - snap[taken].t).squaredNorm() < sep2) {
        too_close = true;
        break;
      }
    }
    if (too_close) continue;
    pre->push_back(cand.second);
    if (static_cast<int>(pre->size()) >= want) break;
  }
  return !pre->empty();
}

void LoopClosure::DetectLoopPair(std::vector<Candidate> * out)
{
  out->clear();
  int n = 0;
  {
    std::lock_guard<std::mutex> lk(kf_mutex_);
    n = static_cast<int>(keyframes_.size());
  }
  if (n < 2) return;
  const int cur = n - 1;
  {
    std::lock_guard<std::mutex> lk(pairs_mutex_);
    if (loop_pairs_.count(cur)) return;
  }

  // Scan context first: it is the only channel that still works once drift
  // exceeds search_radius, which is the regime loop closure is needed in.
  int pre = -1;
  if (cfg_.sc_detect && DetectBySc(cur, &pre)) {
    Candidate c;
    c.cur = cur;
    c.pre = pre;
    c.from_sc = true;
    c.sc_yaw = sc_last_yaw_;
    c.sc_dist = sc_last_dist_;
    out->push_back(c);
    ++hit_sc_;
  }
  std::vector<int> pre_r;
  if (DetectByRadius(cur, &pre_r)) {
    for (int p : pre_r) {
      if (p == pre) continue;
      Candidate c;
      c.cur = cur;
      c.pre = p;
      c.from_sc = false;
      out->push_back(c);
      ++hit_radius_;
    }
  }
}

void LoopClosure::DetectOnce()
{
  ++attempts_;
  if ((accepted_ == 0 && attempts_ % 50 == 0) || attempts_ % 200 == 0) {
    PL_INFO(
      rclcpp::get_logger("LoopClosure"),
      "[loop diag] attempts %d success %d (sc %d radius %d) | rejected: no candidate %d "
      "out of range %d pair saturated %d empty cloud %d ICP failed %d "
      "translation over limit %d rotation over limit %d height over limit %d | dropped %zu",
      attempts_, accepted_, hit_sc_, hit_radius_, rej_pair_, rej_range_, rej_cell_,
      rej_empty_, rej_icp_, rej_corr_, rej_rot_, rej_z_, rej_sc_abs_, rej_sc_ratio_,
      dropped_pending_);
  }

  std::vector<Candidate> cands;
  DetectLoopPair(&cands);
  if (cands.empty()) { ++rej_pair_; return; }
  for (const auto & c : cands) {
    if (TryCandidate(c)) return;
  }
}

bool LoopClosure::TryCandidate(const Candidate & cand)
{
  const int cur = cand.cur, pre = cand.pre;

  Eigen::Vector3d raw_cur, raw_pre;
  Pose3 pose_cur, pose_pre;
  {
    std::lock_guard<std::mutex> lk(kf_mutex_);
    const int n = static_cast<int>(keyframes_.size());
    if (cur >= n || pre >= n) { ++rej_range_; return false; }
    raw_cur = keyframes_[cur].raw.t;
    raw_pre = keyframes_[pre].raw.t;
    pose_cur = keyframes_[cur].pose;
    pose_pre = keyframes_[pre].pose;
  }
  if (CellPairSaturated(raw_cur, raw_pre)) { ++rej_cell_; return false; }

  PointCloudXYZI::Ptr src(new PointCloudXYZI());
  PointCloudXYZI::Ptr tgt(new PointCloudXYZI());
  BuildLocalSubmap(src, cur, cfg_.cur_search_num);
  BuildLocalSubmap(tgt, pre, cfg_.search_num);
  if (src->points.empty() || tgt->points.empty()) { ++rej_empty_; return false; }

  // Seeds, tried in order. Both submaps sit in their own keyframe frame, so a
  // seed is the relative pose of cur seen from pre. Odometry goes first: it
  // carries translation as well as rotation and is exact while drift is small
  // (33 closures against 25 when seeding with the scan-context yaw instead).
  // The scan-context yaw stays as a fallback for when drift is large.
  std::vector<Eigen::Matrix4f> guesses;
  {
    const Pose3 rel = pose_pre.inverse().compose(pose_cur);
    Eigen::Matrix4f g = Eigen::Matrix4f::Identity();
    g.block<3, 3>(0, 0) = rel.R.cast<float>();
    g.block<3, 1>(0, 3) = rel.t.cast<float>();
    guesses.push_back(g);
  }
  if (cand.from_sc && cfg_.sc_use_yaw_guess) {
    const double c = std::cos(cand.sc_yaw), sn = std::sin(cand.sc_yaw);
    Eigen::Matrix4f g = Eigen::Matrix4f::Identity();
    g(0, 0) = static_cast<float>(c);
    g(0, 1) = static_cast<float>(-sn);
    g(1, 0) = static_cast<float>(sn);
    g(1, 1) = static_cast<float>(c);
    guesses.push_back(g);
  }

  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(cfg_.icp_max_corr_dist);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);
  icp.setInputSource(src);
  icp.setInputTarget(tgt);

  Eigen::Matrix4f best_m = Eigen::Matrix4f::Identity();
  double best_fit = std::numeric_limits<double>::max();
  for (const auto & g : guesses) {
    PointCloudXYZI unused;
    icp.align(unused, g);
    if (!icp.hasConverged()) continue;
    const double fit = icp.getFitnessScore();
    if (fit < best_fit) { best_fit = fit; best_m = icp.getFinalTransformation(); }
    if (fit <= cfg_.fitness_score) break;
  }

  if (best_fit > cfg_.fitness_score) { ++rej_icp_; return false; }

  // ICP maps cur-frame points into pre-frame, i.e. T_pre_cur.
  const Eigen::Matrix4f m = best_m;
  Pose3 t_pre_cur;
  t_pre_cur.R = m.block<3, 3>(0, 0).cast<double>();
  t_pre_cur.t = m.block<3, 1>(0, 3).cast<double>();

  // betweenResidual takes measured = from^-1 * to, so the factor cur -> pre
  // carries T_cur_pre.
  const Pose3 relative = t_pre_cur.inverse();
  const Pose3 corrected_cur = pose_pre.compose(t_pre_cur);

  if (cfg_.max_correction > 0.0f &&
      (corrected_cur.t - pose_cur.t).norm() > cfg_.max_correction) {
    ++rej_corr_;
    return false;
  }
  if (cfg_.max_correction_rot > 0.0f) {
    const Eigen::Matrix3d dR = pose_cur.R.transpose() * corrected_cur.R;
    if (Eigen::AngleAxisd(dR).angle() > cfg_.max_correction_rot) { ++rej_rot_; return false; }
  }

  LoopConstraint c;
  c.from = cur;
  c.to = pre;
  c.relative = relative;
  const double sigma =
    std::max(std::sqrt(best_fit), static_cast<double>(cfg_.loop_sigma_floor));
  c.noise = MakeNoise6(sigma, sigma);

  // Accounted here rather than at the decision point: only pairs that actually
  // pass ICP and every gate count, otherwise one failed attempt blocks the pair
  // permanently.
  ++accepted_;
  {
    std::lock_guard<std::mutex> lk(pairs_mutex_);
    if (cfg_.max_loops_per_cell_pair > 0) {
      ++cell_pair_loops_[CellPairKey(raw_cur, raw_pre)];
    }
    // Claim the pair here, not when the backend drains the queue: a keyframe
    // that stays newest across two backend ticks used to be enqueued twice and
    // the same measurement entered the graph with double weight.
    loop_pairs_[cur] = pre;
  }

  std::lock_guard<std::mutex> lk(loop_mutex_);
  loop_queue_.push_back(c);
  last_loop_.from = cur;
  last_loop_.to = pre;
  last_loop_.fitness = best_fit;
  last_loop_.dist = (pose_cur.t - pose_pre.t).norm();
  {
    std::lock_guard<std::mutex> lk2(kf_mutex_);
    if (cur < static_cast<int>(keyframes_.size()) &&
        pre < static_cast<int>(keyframes_.size())) {
      last_loop_.t_from = keyframes_[cur].time;
      last_loop_.t_to = keyframes_[pre].time;
    }
  }
  return true;
}

}  // namespace point_lio

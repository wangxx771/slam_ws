#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>
#include <utility>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>

namespace point_lio
{
template <typename PointT>
class TileMapWriter
{
public:
  using OutCloud = pcl::PointCloud<pcl::PointXYZI>;

  struct Config
  {
    double tile_size = 30.0;
    double flush_idle_s = 10.0;
    std::size_t max_buffer_pts = 500000;
    float voxel = 0.05f;
    enum class Policy
    {
      kFirst,
      kLast,
      kCentroid,
    };
    Policy policy = Policy::kCentroid;
  };

  using SaveFn = std::function<void(const std::filesystem::path &, const OutCloud &)>;

  void Init(const std::filesystem::path & dir, const Config & cfg, SaveFn save)
  {
    cfg_ = cfg;
    save_ = std::move(save);
    tile_size_ = cfg_.tile_size > 1e-3 ? cfg_.tile_size : 30.0;
    if (dir.empty() || !save_) return;

    std::error_code ec;
    const auto tdir = dir / "tiles";
    std::filesystem::create_directories(tdir, ec);
    if (ec) return;
    dir_ = tdir;
  }

  bool enabled() const { return !dir_.empty(); }
  void SetStampAsIntensity(bool on) { stamp_as_intensity_ = on; }

  void Add(const pcl::PointCloud<PointT> & cloud, double stamp)
  {
    if (dir_.empty() || cloud.points.empty()) return;
    if (t0_ < 0.0) t0_ = stamp;
    const float rel_t = static_cast<float>(stamp - t0_);

    const bool dedup = cfg_.voxel > 1e-4f;
    const double inv_v = dedup ? 1.0 / cfg_.voxel : 0.0;

    for (const auto & p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      Tile & t = tiles_[KeyOf(p.x, p.y)];
      t.last_stamp = stamp;

      const CompactPt cp{p.x, p.y, p.z, stamp_as_intensity_ ? rel_t : intensity_of(p), 1};
      if (!dedup) {
        t.pts.push_back(cp);
        ++live_pts_;
        continue;
      }
      const VoxKey k{
        static_cast<std::int32_t>(std::floor(p.x * inv_v)),
        static_cast<std::int32_t>(std::floor(p.y * inv_v)),
        static_cast<std::int32_t>(std::floor(p.z * inv_v))};
      const auto ins = t.vox.emplace(k, cp);
      if (ins.second) {
        ++live_pts_;
        continue;
      }
      CompactPt & cur = ins.first->second;
      switch (cfg_.policy) {
        case Config::Policy::kFirst:
          break;
        case Config::Policy::kLast:
          cur = cp;
          break;
        case Config::Policy::kCentroid:
          cur.x += cp.x; cur.y += cp.y; cur.z += cp.z;
          cur.intensity += cp.intensity;
          ++cur.n;
          break;
      }
    }

    for (auto it = tiles_.begin(); it != tiles_.end();) {
      if (stamp - it->second.last_stamp > cfg_.flush_idle_s && !it->second.empty()) {
        FlushTile(it->first, it->second);
        it = tiles_.erase(it);
      } else {
        ++it;
      }
    }
    EvictUntilUnderCap();
  }

  void FlushAll()
  {
    if (dir_.empty()) return;
    for (auto & kv : tiles_) FlushTile(kv.first, kv.second);
    tiles_.clear();
    live_pts_ = 0;
  }

  std::size_t FinalizeToFile(
    const std::filesystem::path & out, float merge_voxel,
    const Eigen::Matrix3f & Rf = Eigen::Matrix3f::Identity(),
    const Eigen::Vector3f & tf = Eigen::Vector3f::Zero(),
    bool consume = true)
  {
    const bool xform = !Rf.isIdentity(1e-9f) || !tf.isZero(1e-9f);
    FlushAll();
    if (dir_.empty()) return 0;

    OutCloud merged;
    std::unordered_map<VoxKey, char, VoxHash> seen;
    const bool dedup = merge_voxel > 1e-4f;
    const double inv_v = dedup ? 1.0 / merge_voxel : 0.0;

    std::error_code ec;
    std::vector<std::filesystem::path> shards;
    for (const auto & e : std::filesystem::directory_iterator(dir_, ec)) {
      if (e.path().extension() == ".pcd" &&
          e.path().filename().string().rfind("tile_", 0) == 0) {
        shards.push_back(e.path());
      }
    }
    std::sort(shards.begin(), shards.end());
    for (const auto & f : shards) {
      OutCloud c;
      if (pcl::io::loadPCDFile(f.string(), c) < 0) continue;


      for (auto p : c.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (xform) {
          const Eigen::Vector3f q = Rf * Eigen::Vector3f(p.x, p.y, p.z) + tf;
          p.x = q.x(); p.y = q.y(); p.z = q.z();
        }
        if (dedup) {
          const VoxKey k{
            static_cast<std::int32_t>(std::floor(p.x * inv_v)),
            static_cast<std::int32_t>(std::floor(p.y * inv_v)),
            static_cast<std::int32_t>(std::floor(p.z * inv_v))};
          if (!seen.emplace(k, 0).second) continue;
        }
        merged.points.push_back(p);
      }
    }
    if (merged.points.empty()) return 0;
    merged.width = merged.points.size();
    merged.height = 1;
    merged.is_dense = false;
    if (pcl::io::savePCDFileBinary(out.string(), merged) < 0) return 0;

    if (consume) {
      std::filesystem::remove_all(dir_, ec);
    }
    return merged.points.size();
  }

  struct Stats
  {
    std::size_t tiles_live = 0;
    std::size_t points_live = 0;
    std::size_t files_written = 0;
    std::size_t points_written = 0;
  };
  Stats GetStats() const
  {
    Stats s = stats_;
    s.tiles_live = tiles_.size();
    s.points_live = live_pts_;
    return s;
  }

private:
  template <typename P>
  static float intensity_of(const P & p)
  {
    if constexpr (pcl::traits::has_intensity_v<P>) {
      return static_cast<float>(p.intensity);
    } else {
      (void)p;
      return 0.0f;
    }
  }

  struct Key
  {
    int i = 0, j = 0;
    bool operator==(const Key & o) const { return i == o.i && j == o.j; }
  };
  struct KeyHash
  {
    std::size_t operator()(const Key & k) const noexcept
    {
      return (static_cast<std::size_t>(static_cast<std::uint32_t>(k.i)) * 73856093u) ^
             (static_cast<std::size_t>(static_cast<std::uint32_t>(k.j)) * 19349663u);
    }
  };
  struct VoxKey
  {
    std::int32_t x = 0, y = 0, z = 0;
    bool operator==(const VoxKey & o) const { return x == o.x && y == o.y && z == o.z; }
  };
  struct VoxHash
  {
    std::size_t operator()(const VoxKey & k) const noexcept
    {
      return (static_cast<std::size_t>(static_cast<std::uint32_t>(k.x)) * 73856093u) ^
             (static_cast<std::size_t>(static_cast<std::uint32_t>(k.y)) * 19349663u) ^
             (static_cast<std::size_t>(static_cast<std::uint32_t>(k.z)) * 83492791u);
    }
  };

  struct CompactPt
  {
    float x = 0, y = 0, z = 0, intensity = 0;
    std::uint32_t n = 1;
  };
  struct Tile
  {
    std::unordered_map<VoxKey, CompactPt, VoxHash> vox;
    std::vector<CompactPt> pts;
    double last_stamp = 0.0;
    std::size_t size() const { return vox.empty() ? pts.size() : vox.size(); }
    bool empty() const { return vox.empty() && pts.empty(); }
  };

  Key KeyOf(float x, float y) const
  {
    return Key{
      static_cast<int>(std::floor(x / tile_size_)),
      static_cast<int>(std::floor(y / tile_size_))};
  }

  void FlushTile(const Key & k, Tile & t)
  {
    if (t.empty()) return;
    live_pts_ -= std::min(live_pts_, t.size());

    OutCloud out;
    out.points.reserve(t.size());
    auto emit = [&out](const CompactPt & c) {
      const float inv_n = c.n > 1 ? 1.0f / static_cast<float>(c.n) : 1.0f;
      pcl::PointXYZI p;
      p.x = c.x * inv_n; p.y = c.y * inv_n; p.z = c.z * inv_n;
      p.intensity = c.intensity * inv_n;
      out.points.push_back(p);
    };
    if (!t.vox.empty()) {
      for (const auto & kv : t.vox) emit(kv.second);
    } else {
      for (const auto & c : t.pts) emit(c);
    }
    t.vox.clear();
    t.pts.clear();
    if (out.points.empty()) return;

    const std::string name = "tile_" + std::to_string(k.i) + "_" + std::to_string(k.j) +
                             "_" + std::to_string(seq_[k]++) + ".pcd";
    out.width = out.points.size();
    out.height = 1;
    out.is_dense = false;
    save_(dir_ / name, out);


    ++stats_.files_written;
    stats_.points_written += out.points.size();
  }

  void EvictUntilUnderCap()
  {
    while (live_pts_ > cfg_.max_buffer_pts && !tiles_.empty()) {
      auto oldest = tiles_.begin();
      double t_min = std::numeric_limits<double>::max();
      for (auto it = tiles_.begin(); it != tiles_.end(); ++it) {
        if (it->second.last_stamp < t_min) {
          t_min = it->second.last_stamp;
          oldest = it;
        }
      }
      if (oldest->second.empty()) {
        tiles_.erase(oldest);
        continue;
      }
      FlushTile(oldest->first, oldest->second);
      tiles_.erase(oldest);
    }
  }

  std::filesystem::path dir_;
  Config cfg_;
  SaveFn save_;
  double tile_size_ = 30.0;
  std::unordered_map<Key, Tile, KeyHash> tiles_;
  std::unordered_map<Key, int, KeyHash> seq_;
  std::size_t live_pts_ = 0;
  bool stamp_as_intensity_ = false;
  double t0_ = -1.0;
  Stats stats_;
};

}  // namespace point_lio

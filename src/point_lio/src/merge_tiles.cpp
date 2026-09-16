
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <unordered_set>
#include <vector>

namespace
{

struct VoxKey
{
  std::int32_t x, y, z;
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

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::printf("usage: merge_tiles <tiles_dir> <out.pcd> [voxel=0.05, 0=no dedup]\n");
    return 1;
  }
  const std::filesystem::path dir = argv[1];
  const std::filesystem::path out = argv[2];
  const double voxel = argc > 3 ? std::atof(argv[3]) : 0.05;

  std::error_code ec;
  if (!std::filesystem::is_directory(dir, ec)) {
    std::printf("directory does not exist: %s\n", dir.c_str());
    return 1;
  }

  std::vector<std::filesystem::path> files;
  for (const auto & e : std::filesystem::directory_iterator(dir, ec)) {
    const auto n = e.path().filename().string();
    if (n.rfind("tile_", 0) == 0 && e.path().extension() == ".pcd") {
      files.push_back(e.path());
    }
  }
  if (files.empty()) {
    std::printf("no tile shards found: %s/tile_*.pcd\n", dir.c_str());
    return 1;
  }
  std::sort(files.begin(), files.end());

  pcl::PointCloud<pcl::PointXYZI> merged;
  std::unordered_set<VoxKey, VoxHash> seen;
  const bool dedup = voxel > 1e-4;
  const double inv_v = dedup ? 1.0 / voxel : 0.0;
  std::size_t raw_total = 0;

  for (std::size_t i = 0; i < files.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZI> tile;
    if (pcl::io::loadPCDFile(files[i].string(), tile) < 0) {
      std::printf("  skipping unreadable shard: %s\n", files[i].filename().c_str());
      continue;
    }
    raw_total += tile.points.size();
    for (const auto & p : tile.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (dedup) {
        const VoxKey k{
          static_cast<std::int32_t>(std::floor(p.x * inv_v)),
          static_cast<std::int32_t>(std::floor(p.y * inv_v)),
          static_cast<std::int32_t>(std::floor(p.z * inv_v))};
        if (!seen.insert(k).second) continue;
      }
      merged.points.push_back(p);
    }
    if ((i + 1) % 50 == 0) {
      std::printf("  read %zu/%zu shards, %zu points so far\n", i + 1, files.size(), raw_total);
      std::fflush(stdout);
    }
  }

  if (merged.points.empty()) {
    std::printf("all shards are empty\n");
    return 1;
  }
  merged.width = merged.points.size();
  merged.height = 1;
  merged.is_dense = false;

  if (pcl::io::savePCDFileBinary(out.string(), merged) < 0) {
    std::printf("write failed: %s\n", out.c_str());
    return 1;
  }
  const double mb = std::filesystem::file_size(out, ec) / 1048576.0;
  std::printf(
    "\n%zu shards, %zu raw points -> %zu merged (dedup %.1f%%)\nwrote %s (%.1f MB)\n",
    files.size(), raw_total, merged.points.size(),
    100.0 * (1.0 - static_cast<double>(merged.points.size()) /
                     static_cast<double>(std::max<std::size_t>(raw_total, 1))),
    out.c_str(), mb);
  return 0;
}

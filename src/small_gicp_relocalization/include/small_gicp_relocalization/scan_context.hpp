#ifndef SMALL_GICP_RELOCALIZATION__SCAN_CONTEXT_HPP_
#define SMALL_GICP_RELOCALIZATION__SCAN_CONTEXT_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace scan_context
{

struct Config
{
  float max_radius = 60.0f;
  int num_ring = 20;
  int num_sector = 60;
  float max_height = 1e9f;
  // Added to every point's z before max-pooling, as SC_PGO's LIDAR_HEIGHT does,
  // so that cells store a positive value and an empty cell stays distinguishable
  // from one whose points all sit below the sensor. Only the sign matters, not
  // the exact mounting height; the cosine distance is scale-free per column.
  float lidar_height = 2.0f;
};

struct Entry
{
  float x = 0, y = 0, z = 0;
  float yaw = 0;
  std::vector<float> desc;
  std::vector<float> rkey;
};

struct Match
{
  bool found = false;
  int index = -1;
  float x = 0, y = 0, z = 0;
  float yaw_map_odom = 0;     
  float score = 1.0f;
};

inline std::vector<float> make(const std::vector<float> & pts, const Config & c)
{
  // Cells start below any real height and are zeroed at the end, so "no points
  // here" and "points here, all of them low" stay different. Initialising to 0
  // and max-pooling raw z instead silently dropped everything below the sensor:
  // on this site that erased the floor and left only ceiling, which is the least
  // distinctive part of a corridor. Following SC_PGO Scancontext.cpp:158.
  constexpr float kNoPoint = -1000.0f;
  std::vector<float> d(static_cast<std::size_t>(c.num_ring) * c.num_sector, kNoPoint);
  const float ring_w = c.max_radius / c.num_ring;
  for (std::size_t i = 0; i + 2 < pts.size(); i += 3) {
    const float px = pts[i], py = pts[i + 1], pz = pts[i + 2];
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;
    const float r = std::hypot(px, py);
    if (r < 1e-3f || r >= c.max_radius) continue;
    int ri = static_cast<int>(r / ring_w);
    if (ri >= c.num_ring) continue;
    float az = std::atan2(py, px);
    if (az < 0) az += 2.0f * static_cast<float>(M_PI);
    int si = static_cast<int>(az / (2.0f * static_cast<float>(M_PI)) * c.num_sector);
    if (si >= c.num_sector) si = c.num_sector - 1;
    // max_height is tested on the raw height above the sensor so its calibration
    // is unaffected by the shift below, which is only a storage convention.
    if (pz > c.max_height) continue;
    const float v = pz + c.lidar_height;
    float & cell = d[static_cast<std::size_t>(ri) * c.num_sector + si];
    if (v > cell) cell = v;
  }
  for (float & v : d) {
    if (v == kNoPoint) v = 0.0f;
  }
  return d;
}

inline std::vector<float> ringKey(const std::vector<float> & d, const Config & c)
{
  std::vector<float> k(c.num_ring, 0.0f);
  for (int r = 0; r < c.num_ring; ++r) {
    double s = 0.0;
    for (int j = 0; j < c.num_sector; ++j) s += d[static_cast<std::size_t>(r) * c.num_sector + j];
    k[r] = static_cast<float>(s / c.num_sector);
  }
  return k;
}

inline float align(
  const std::vector<float> & a, const std::vector<float> & b, const Config & c, int * shift)
{
  const int R = c.num_ring, S = c.num_sector;
  float best = 2.0f;
  int best_s = 0;
  for (int s = 0; s < S; ++s) {
    double sum = 0.0;
    int used = 0;
    for (int j = 0; j < S; ++j) {
      const int jb = (j + s) % S;
      double dot = 0, na = 0, nb = 0;
      for (int r = 0; r < R; ++r) {
        const double va = a[static_cast<std::size_t>(r) * S + j];
        const double vb = b[static_cast<std::size_t>(r) * S + jb];
        dot += va * vb; na += va * va; nb += vb * vb;
      }
      if (na < 1e-9 || nb < 1e-9) continue;
      sum += 1.0 - dot / (std::sqrt(na) * std::sqrt(nb));
      ++used;
    }
    if (used < S / 4) continue;
    const float dist = static_cast<float>(sum / used);
    if (dist < best) { best = dist; best_s = s; }
  }
  *shift = best_s;
  return best;
}

class Database
{
public:
  Config cfg;
  std::vector<Entry> entries;


  static int occupancy(const std::vector<float> & d)
  {
    int n = 0;
    for (float v : d) if (v > 0.3f) ++n;
    return n;
  }

  bool add(float x, float y, float z, float yaw, std::vector<float> desc, int min_cells = 60)
  {
    if (occupancy(desc) < min_cells) return false;
    Entry e;
    e.x = x; e.y = y; e.z = z; e.yaw = yaw;
    e.rkey = ringKey(desc, cfg);
    e.desc = std::move(desc);
    entries.push_back(std::move(e));
    return true;
  }

  bool save(const std::string & path) const
  {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) return false;
    // 0002 stores lidar_height. A 0001 file holds descriptors built with a
    // different height convention, so it is rejected rather than mixed in.
    const char magic[8] = {'S', 'C', 'T', 'X', '0', '0', '0', '2'};
    f.write(magic, 8);
    const int32_t hdr[3] = {cfg.num_ring, cfg.num_sector,
                            static_cast<int32_t>(entries.size())};
    f.write(reinterpret_cast<const char *>(hdr), sizeof(hdr));
    f.write(reinterpret_cast<const char *>(&cfg.max_radius), sizeof(float));
    f.write(reinterpret_cast<const char *>(&cfg.lidar_height), sizeof(float));
    for (const auto & e : entries) {
      const float p[4] = {e.x, e.y, e.z, e.yaw};
      f.write(reinterpret_cast<const char *>(p), sizeof(p));
      f.write(
        reinterpret_cast<const char *>(e.desc.data()),
        static_cast<std::streamsize>(e.desc.size() * sizeof(float)));
    }
    return f.good();
  }

  bool load(const std::string & path)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    char magic[8];
    f.read(magic, 8);
    if (std::memcmp(magic, "SCTX0002", 8) != 0) return false;
    int32_t hdr[3];
    f.read(reinterpret_cast<char *>(hdr), sizeof(hdr));
    if (hdr[0] <= 0 || hdr[0] > 1024 || hdr[1] <= 0 || hdr[1] > 4096 ||
        hdr[2] < 0 || hdr[2] > 10000000) {
      return false;
    }
    cfg.num_ring = hdr[0];
    cfg.num_sector = hdr[1];
    f.read(reinterpret_cast<char *>(&cfg.max_radius), sizeof(float));
    if (!(cfg.max_radius > 0.0f) || cfg.max_radius > 1e4f) return false;
    f.read(reinterpret_cast<char *>(&cfg.lidar_height), sizeof(float));
    if (!std::isfinite(cfg.lidar_height)) return false;
    entries.clear();
    entries.reserve(static_cast<std::size_t>(hdr[2]));
    const std::size_t n = static_cast<std::size_t>(cfg.num_ring) * cfg.num_sector;
    for (int32_t i = 0; i < hdr[2]; ++i) {
      float p[4];
      f.read(reinterpret_cast<char *>(p), sizeof(p));
      Entry e;
      e.x = p[0]; e.y = p[1]; e.z = p[2]; e.yaw = p[3];
      e.desc.resize(n);
      f.read(
        reinterpret_cast<char *>(e.desc.data()),
        static_cast<std::streamsize>(n * sizeof(float)));
      if (!f) return false;
      e.rkey = ringKey(e.desc, cfg);
      entries.push_back(std::move(e));
    }
    return true;
  }
  std::vector<Match> queryTopK(const std::vector<float> & desc, int n_out,
                               int top_k = 30, int exclude_idx = -1) const
  {
    std::vector<Match> out;
    if (entries.empty()) return out;
    const std::vector<float> qk = ringKey(desc, cfg);

    std::vector<std::pair<float, int>> cand;
    cand.reserve(entries.size());
    for (std::size_t i = 0; i < entries.size(); ++i) {
      if (static_cast<int>(i) == exclude_idx) continue;
      float s = 0;
      for (int r = 0; r < cfg.num_ring; ++r) {
        const float d = qk[r] - entries[i].rkey[r];
        s += d * d;
      }
      cand.emplace_back(s, static_cast<int>(i));
    }
    if (cand.empty()) return out;
    const std::size_t k = std::min<std::size_t>(top_k, cand.size());
    std::partial_sort(cand.begin(), cand.begin() + k, cand.end());

    std::vector<std::pair<float, std::pair<int, int>>> scored;
    scored.reserve(k);
    for (std::size_t j = 0; j < k; ++j) {
      int sh = 0;
      const float d = align(desc, entries[cand[j].second].desc, cfg, &sh);
      scored.push_back({d, {cand[j].second, sh}});
    }
    std::sort(scored.begin(), scored.end(),
              [](const auto & a, const auto & b) { return a.first < b.first; });

    const float delta = 2.0f * static_cast<float>(M_PI) / cfg.num_sector;
    for (std::size_t j = 0; j < scored.size() && static_cast<int>(out.size()) < n_out; ++j) {
      const Entry & e = entries[scored[j].second.first];
      bool dup = false;
      for (const auto & m : out) {
        if (std::hypot(e.x - m.x, e.y - m.y) < 5.0f) { dup = true; break; }
      }
      if (dup) continue;
      float th = scored[j].second.second * delta;
      while (th > M_PI) th -= 2.0f * static_cast<float>(M_PI);
      while (th < -M_PI) th += 2.0f * static_cast<float>(M_PI);
      Match m;
      m.found = true;
      m.index = scored[j].second.first;
      m.x = e.x; m.y = e.y; m.z = e.z;
      m.yaw_map_odom = th;
      m.score = scored[j].first;
      out.push_back(m);
    }
    return out;
  }

};

}  // namespace scan_context

#endif  // SMALL_GICP_RELOCALIZATION__SCAN_CONTEXT_HPP_

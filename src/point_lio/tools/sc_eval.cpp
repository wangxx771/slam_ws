// Offline Scan Context retrieval evaluation over exported keyframe patches.
// Replays exactly the retrieval loop_closure.cpp runs (ringkey prefilter ->
// column-shift align), then adds a sequence-consistency variant for comparison.
#include "small_gicp_relocalization/scan_context.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

struct Pose { double x, y, z; };

static bool loadPcdXYZ(const std::string & path, std::vector<float> & out)
{
  FILE * f = fopen(path.c_str(), "rb");
  if (!f) return false;
  char line[512];
  long npts = 0; int stride = 0; std::vector<int> sizes; bool binary = false;
  while (fgets(line, sizeof(line), f)) {
    if (!strncmp(line, "POINTS", 6)) npts = atol(line + 6);
    else if (!strncmp(line, "SIZE", 4)) {
      char * p = line + 4; while (*p) { while (*p == ' ') ++p; if (*p < '0' || *p > '9') break;
        sizes.push_back(atoi(p)); while (*p > ' ') ++p; }
    } else if (!strncmp(line, "DATA", 4)) { binary = strstr(line, "binary") != nullptr; break; }
  }
  for (int s : sizes) stride += s;
  if (!binary || npts <= 0 || stride < 12) { fclose(f); return false; }
  std::vector<char> buf(static_cast<size_t>(npts) * stride);
  size_t got = fread(buf.data(), 1, buf.size(), f);
  fclose(f);
  if (got < buf.size()) npts = static_cast<long>(got / stride);
  out.resize(static_cast<size_t>(npts) * 3);
  for (long i = 0; i < npts; ++i) memcpy(&out[i * 3], &buf[i * stride], 12);
  return true;
}

int main(int argc, char ** argv)
{
  const std::string dir = argc > 1 ? argv[1] : "/home/toe/slam_ws/PCD/ring_map";
  const int K = argc > 2 ? atoi(argv[2]) : 10;
  const int EXCL = argc > 3 ? atoi(argv[3]) : 30;
  const float THR = argc > 4 ? atof(argv[4]) : 0.35f;
  const float RATIO = argc > 5 ? atof(argv[5]) : 0.75f;
  const float GT = argc > 6 ? atof(argv[6]) : 4.0f;   // true revisit radius

  scan_context::Config cfg;
  cfg.max_radius = getenv("SC_R") ? atof(getenv("SC_R")) : 60.0f;
  cfg.num_ring = getenv("SC_NR") ? atoi(getenv("SC_NR")) : 20;
  cfg.num_sector = getenv("SC_NS") ? atoi(getenv("SC_NS")) : 60;
  cfg.max_height = getenv("SC_H") ? atof(getenv("SC_H")) : 1e9f;

  std::vector<Pose> poses; std::vector<std::vector<float>> desc, rkey;
  FILE * pf = fopen((dir + "/poses.txt").c_str(), "r");
  if (!pf) { fprintf(stderr, "no poses.txt in %s\n", dir.c_str()); return 1; }
  char name[256]; double x, y, z, qw, qx, qy, qz;
  while (fscanf(pf, "%255s %lf %lf %lf %lf %lf %lf %lf", name, &x, &y, &z, &qw, &qx, &qy, &qz) == 8) {
    std::vector<float> pts;
    if (!loadPcdXYZ(dir + "/patches/" + name, pts)) continue;
    poses.push_back({x, y, z});
    auto d = scan_context::make(pts, cfg);
    rkey.push_back(scan_context::ringKey(d, cfg));
    desc.push_back(std::move(d));
  }
  fclose(pf);
  const int N = static_cast<int>(poses.size());
  printf("keyframes: %d\n", N);

  auto dist2 = [&](int a, int b) {
    const double dx = poses[a].x - poses[b].x, dy = poses[a].y - poses[b].y;
    return std::sqrt(dx * dx + dy * dy);
  };
  std::map<long long, float> memo;
  auto alignD = [&](int a, int b) {
    const long long key = static_cast<long long>(a) * 100000 + b;
    auto it = memo.find(key);
    if (it != memo.end()) return it->second;
    int sh; const float v = scan_context::align(desc[a], desc[b], cfg, &sh);
    memo[key] = v; return v;
  };

  // How many queries actually have a revisit available at all.
  int revisitable = 0;
  for (int i = EXCL; i < N; ++i)
    for (int j = 0; j <= i - EXCL; ++j) if (dist2(i, j) <= GT) { ++revisitable; break; }

  struct Res { int det = 0, cor = 0; };
  Res plain, seq;
  const int W = 5, STEP = 4;   // sequence window: 5 queries spaced 4 keyframes apart

  for (int i = EXCL; i < N; ++i) {
    const int hi = i - EXCL;
    if (hi < 1) continue;
    // ringkey prefilter (brute force here; the node uses a KD-tree over the same key)
    std::vector<std::pair<float, int>> pre;
    for (int j = 0; j <= hi; ++j) {
      double s = 0; for (int r = 0; r < cfg.num_ring; ++r) {
        const double d = rkey[i][r] - rkey[j][r]; s += d * d; }
      pre.push_back({static_cast<float>(s), j});
    }
    const int k = std::min<int>(K, pre.size());
    std::partial_sort(pre.begin(), pre.begin() + k, pre.end());

    float b1 = 1e9f, b2 = 1e9f; int bi = -1;
    std::vector<std::pair<float, int>> scored;
    for (int c = 0; c < k; ++c) {
      const float d = alignD(i, pre[c].second);
      scored.push_back({d, pre[c].second});
      if (d < b1) { b2 = b1; b1 = d; bi = pre[c].second; }
      else if (d < b2) b2 = d;
    }
    if (bi >= 0 && b1 < THR && b1 < RATIO * b2) {
      ++plain.det; if (dist2(i, bi) <= GT) ++plain.cor;
    }

    // Sequence consistency: a candidate must stay the best explanation as both
    // query and candidate walk backwards together. Aliased corridor matches do
    // not survive because the false partner drifts out of step.
    float sb1 = 1e9f, sb2 = 1e9f; int sbi = -1;
    for (auto & sc : scored) {
      const int j = sc.second;
      double sum = 0; int used = 0;
      for (int w = 0; w < W; ++w) {
        const int qi = i - w * STEP, cj = j - w * STEP;
        if (qi < 0 || cj < 0) break;
        sum += alignD(qi, cj); ++used;
      }
      if (used < W) continue;
      const float s = static_cast<float>(sum / used);
      if (s < sb1) { sb2 = sb1; sb1 = s; sbi = j; }
      else if (s < sb2) sb2 = s;
    }
    if (sbi >= 0 && sb1 < THR && sb1 < RATIO * sb2) {
      ++seq.det; if (dist2(i, sbi) <= GT) ++seq.cor;
    }
  }

  auto rep = [&](const char * tag, Res r) {
    printf("%-10s detections %3d  correct %3d  precision %5.1f%%  recall %5.1f%%\n",
      tag, r.det, r.cor, r.det ? 100.0 * r.cor / r.det : 0.0,
      revisitable ? 100.0 * r.cor / revisitable : 0.0);
  };
  printf("queries with a real revisit: %d\n", revisitable);
  rep("plain", plain);
  rep("seq", seq);
  return 0;
}
// Build:
//   g++ -O2 -o sc_eval sc_eval.cpp -I../../small_gicp_relocalization/include
// Run (keyframe dir holding poses.txt + patches/, as exported by
// pcd_save.export_patches):
//   ./sc_eval /home/toe/slam_ws/PCD/ring_map

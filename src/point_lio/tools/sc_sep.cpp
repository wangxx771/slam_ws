// Separability of the Scan Context distance on this dataset: for every query
// with a real revisit, compare the distance to its true partner against the
// distance to the best non-revisit frame. If the false one is routinely closer,
// no threshold or candidate count can make retrieval work.
#include "small_gicp_relocalization/scan_context.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
struct Pose { double x, y, z; };
static bool loadPcdXYZ(const std::string & p, std::vector<float> & out) {
  FILE * f = fopen(p.c_str(), "rb"); if (!f) return false;
  char l[512]; long n = 0; int st = 0; std::vector<int> sz; bool bin = false;
  while (fgets(l, sizeof(l), f)) {
    if (!strncmp(l, "POINTS", 6)) n = atol(l + 6);
    else if (!strncmp(l, "SIZE", 4)) { char*q=l+4; while(*q){while(*q==' ')++q; if(*q<'0'||*q>'9')break; sz.push_back(atoi(q)); while(*q>' ')++q;} }
    else if (!strncmp(l, "DATA", 4)) { bin = strstr(l, "binary"); break; } }
  for (int s : sz) st += s;
  if (!bin || n <= 0 || st < 12) { fclose(f); return false; }
  std::vector<char> b((size_t)n * st); size_t g = fread(b.data(), 1, b.size(), f); fclose(f);
  if (g < b.size()) n = (long)(g / st);
  out.resize((size_t)n * 3);
  for (long i = 0; i < n; ++i) memcpy(&out[i * 3], &b[i * st], 12);
  return true;
}
int main(int argc, char ** argv) {
  const std::string dir = argc > 1 ? argv[1] : "/home/toe/slam_ws/PCD/ring_map";
  const int EXCL = 30; const float GT = 4.0f, FAR = 15.0f;
  scan_context::Config cfg; cfg.max_radius = 60; cfg.num_ring = 20; cfg.num_sector = 60;
  cfg.max_height = getenv("SC_H") ? atof(getenv("SC_H")) : 1e9f;
  std::vector<Pose> P; std::vector<std::vector<float>> D;
  FILE * pf = fopen((dir + "/poses.txt").c_str(), "r");
  char nm[256]; double x,y,z,qw,qx,qy,qz;
  while (fscanf(pf,"%255s %lf %lf %lf %lf %lf %lf %lf",nm,&x,&y,&z,&qw,&qx,&qy,&qz)==8) {
    std::vector<float> pts; if (!loadPcdXYZ(dir + "/patches/" + nm, pts)) continue;
    P.push_back({x,y,z}); D.push_back(scan_context::make(pts, cfg)); }
  fclose(pf);
  const int N = P.size();
  auto d2=[&](int a,int b){ double dx=P[a].x-P[b].x, dy=P[a].y-P[b].y; return std::sqrt(dx*dx+dy*dy); };
  int n = 0, wins = 0; double sTrue = 0, sFalse = 0;
  std::vector<float> tv, fv;
  for (int i = EXCL; i < N; ++i) {
    float bt = 1e9f, bf = 1e9f; int sh;
    for (int j = 0; j <= i - EXCL; ++j) {
      const double dd = d2(i, j);
      if (dd > GT && dd < FAR) continue;            // ambiguous band, skip
      const float v = scan_context::align(D[i], D[j], cfg, &sh);
      if (dd <= GT) bt = std::min(bt, v); else bf = std::min(bf, v);
    }
    if (bt > 1e8f || bf > 1e8f) continue;
    ++n; sTrue += bt; sFalse += bf; tv.push_back(bt); fv.push_back(bf);
    if (bt < bf) ++wins;
  }
  std::sort(tv.begin(), tv.end()); std::sort(fv.begin(), fv.end());
  printf("max_height %.2g | queries %d\n", (double)cfg.max_height, n);
  printf("  true-revisit  SC distance  mean %.3f  median %.3f\n", sTrue/n, tv[n/2]);
  printf("  best far-away SC distance  mean %.3f  median %.3f\n", sFalse/n, fv[n/2]);
  printf("  true partner beats every far frame: %d/%d (%.1f%%)\n", wins, n, 100.0*wins/n);
  return 0;
}
// Build:
//   g++ -O2 -o sc_sep sc_sep.cpp -I../../small_gicp_relocalization/include
// Run: ./sc_sep <keyframe dir>   (SC_H caps descriptor height)

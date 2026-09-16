// Two questions the precision number alone cannot answer:
//   1. How well does the descriptor distance rank true revisits at all (AUC)?
//   2. Is the ring-key prefilter the bottleneck, or the descriptor itself?
#include "small_gicp_relocalization/scan_context.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
struct Pose{double x,y,z;};
static bool load(const std::string&p,std::vector<float>&o){
  FILE*f=fopen(p.c_str(),"rb"); if(!f)return false;
  char l[512]; long n=0; int st=0; std::vector<int> sz; bool bin=false;
  while(fgets(l,sizeof(l),f)){
    if(!strncmp(l,"POINTS",6))n=atol(l+6);
    else if(!strncmp(l,"SIZE",4)){char*q=l+4;while(*q){while(*q==' ')++q;if(*q<'0'||*q>'9')break;sz.push_back(atoi(q));while(*q>' ')++q;}}
    else if(!strncmp(l,"DATA",4)){bin=strstr(l,"binary");break;}}
  for(int s:sz)st+=s;
  if(!bin||n<=0||st<12){fclose(f);return false;}
  std::vector<char> b((size_t)n*st); size_t g=fread(b.data(),1,b.size(),f); fclose(f);
  if(g<b.size())n=(long)(g/st);
  o.resize((size_t)n*3);
  for(long i=0;i<n;++i)memcpy(&o[i*3],&b[i*st],12);
  return true;
}
static std::vector<float> makeRef(const std::vector<float>&pts,const scan_context::Config&c,float lh){
  constexpr float NP=-1000.0f;
  std::vector<float> d((size_t)c.num_ring*c.num_sector,NP);
  const float rw=c.max_radius/c.num_ring;
  for(size_t i=0;i+2<pts.size();i+=3){
    const float px=pts[i],py=pts[i+1],pz=pts[i+2]+lh;
    const float r=std::hypot(px,py); if(r<1e-3f||r>=c.max_radius)continue;
    int ri=(int)(r/rw); if(ri>=c.num_ring)continue;
    float az=std::atan2(py,px); if(az<0)az+=2.0f*(float)M_PI;
    int si=(int)(az/(2.0f*(float)M_PI)*c.num_sector); if(si>=c.num_sector)si=c.num_sector-1;
    float&cl=d[(size_t)ri*c.num_sector+si]; if(pz>cl)cl=pz;
  }
  for(float&v:d) if(v==NP)v=0.0f;
  return d;
}
int main(int argc,char**argv){
  const std::string dir=argc>1?argv[1]:"/home/toe/slam_ws/PCD/ring_map";
  const int EXCL=30; const float GT=4.0f;
  scan_context::Config c;
  c.num_ring=getenv("SC_NR")?atoi(getenv("SC_NR")):20;
  c.num_sector=getenv("SC_NS")?atoi(getenv("SC_NS")):60;
  c.max_radius=getenv("SC_R")?atof(getenv("SC_R")):6.0f;
  std::vector<Pose> P; std::vector<std::vector<float>> D,K;
  FILE*pf=fopen((dir+"/poses.txt").c_str(),"r");
  char nm[256]; double x,y,z,qw,qx,qy,qz;
  while(fscanf(pf,"%255s %lf %lf %lf %lf %lf %lf %lf",nm,&x,&y,&z,&qw,&qx,&qy,&qz)==8){
    std::vector<float> pts; if(!load(dir+"/patches/"+nm,pts))continue;
    P.push_back({x,y,z}); auto d=makeRef(pts,c,2.0f);
    K.push_back(scan_context::ringKey(d,c)); D.push_back(std::move(d));
  }
  fclose(pf);
  const int N=P.size();
  auto sep=[&](int a,int b){double dx=P[a].x-P[b].x,dy=P[a].y-P[b].y;return std::sqrt(dx*dx+dy*dy);};
  auto dist=[&](int a,int b){int sh;return scan_context::align(D[a],D[b],c,&sh);};

  // AUC over every eligible pair: can the distance rank a true revisit above an
  // impostor? 0.5 is chance.
  std::vector<float> pos,neg;
  for(int i=EXCL;i<N;++i)
    for(int j=0;j<=i-EXCL;++j)
      (sep(i,j)<=GT?pos:neg).push_back(dist(i,j));
  std::sort(neg.begin(),neg.end());
  double auc=0;
  for(float v:pos) auc += (double)(neg.end()-std::upper_bound(neg.begin(),neg.end(),v));
  auc/=(double)pos.size()*neg.size();
  printf("R=%.0fm %dx%d  真回访 %zu 非回访 %zu  AUC=%.3f\n",
         c.max_radius,c.num_ring,c.num_sector,pos.size(),neg.size(),auc);

  // SC_THR follows SC-PGO's SC_DIST_THRES (0.13-0.4). SC_RATIO is our own
  // best-vs-runner-up test, not part of Scan Context; 1.0 disables it, which is
  // the reference behaviour.
  const float THR = getenv("SC_THR") ? atof(getenv("SC_THR")) : 0.30f;
  const float RATIO = getenv("SC_RATIO") ? atof(getenv("SC_RATIO")) : 1.0f;
  // Retrieval: ring-key prefilter (as the node does) vs brute force over all
  // eligible frames. If brute force is much better the prefilter is the fault.
  for (int mode=0; mode<2; ++mode) {
    int det=0,cor=0,revis=0;
    for(int i=EXCL;i<N;++i){
      const int hi=i-EXCL; if(hi<1) continue;
      bool has=false; for(int j=0;j<=hi;++j) if(sep(i,j)<=GT){has=true;break;}
      if(has) ++revis;
      std::vector<int> cand;
      if(mode==0){
        std::vector<std::pair<float,int>> pre;
        for(int j=0;j<=hi;++j){ double s=0; for(int r=0;r<c.num_ring;++r){double d=K[i][r]-K[j][r]; s+=d*d;} pre.push_back({(float)s,j}); }
        const int k=std::min<int>(10,pre.size());
        std::partial_sort(pre.begin(),pre.begin()+k,pre.end());
        for(int q=0;q<k;++q) cand.push_back(pre[q].second);
      } else {
        for(int j=0;j<=hi;++j) cand.push_back(j);
      }
      float b1=1e9f,b2=1e9f; int bi=-1;
      for(int j:cand){ const float d=dist(i,j); if(d<b1){b2=b1;b1=d;bi=j;} else if(d<b2) b2=d; }
      if(bi>=0 && b1<THR && (RATIO>=1.0f || b1<RATIO*b2)){ ++det; if(sep(i,bi)<=GT) ++cor; }
    }
    printf("  thr=%.2f ratio=%.2f %-10s 检测 %3d  正确 %3d  精度 %5.1f%%  召回 %5.1f%%\n",
           THR, RATIO, mode?"全量比对":"ringkey预筛", det,cor, det?100.0*cor/det:0.0, revis?100.0*cor/revis:0.0);
  }
  return 0;
}
// Build:
//   g++ -O2 -o sc_auc sc_auc.cpp -I../../small_gicp_relocalization/include
// Run: ./sc_auc <keyframe dir>
//   SC_R max_radius, SC_NR/SC_NS ring and sector counts,
//   SC_THR distance threshold, SC_RATIO best-vs-runner-up (1.0 = off)

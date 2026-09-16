#pragma once
#include <Eigen/Core>
#include <cmath>
#include <deque>

namespace point_lio
{

struct GnssConfig
{
  bool enable = false;
  bool use_elevation = false;
  float cov_threshold = 2.0f;
  float min_travel = 1.0f;
  double time_tol = 0.05;
  Eigen::Vector3d extrin_t = Eigen::Vector3d::Zero();
  Eigen::Matrix3d extrin_r = Eigen::Matrix3d::Identity();

  double sigma_floor = 0.05;
  double sigma_scale = 1.0;
  double sigma_max = 1e3;
};

struct GnssSample
{
  double time = 0.0;
  Eigen::Vector3d enu = Eigen::Vector3d::Zero();
  Eigen::Vector3d sigma = Eigen::Vector3d::Ones();
};

class GnssProcess
{
public:
  void Init(const GnssConfig & cfg) { cfg_ = cfg; }
  const GnssConfig & cfg() const { return cfg_; }
  void Add(double stamp, double lat, double lon, double alt, const Eigen::Vector3d & cov,
           int status);


  bool PopAligned(double stamp, GnssSample * out);

  bool HasOrigin() const { return has_origin_; }
  size_t Pending() const { return buf_.size(); }

private:
  static void Wgs84ToEcef(double lat_deg, double lon_deg, double alt, Eigen::Vector3d * ecef);

  GnssConfig cfg_;
  std::deque<GnssSample> buf_;

  bool has_origin_ = false;
  double origin_lat_ = 0.0, origin_lon_ = 0.0, origin_alt_ = 0.0;
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d ecef_to_enu_ = Eigen::Matrix3d::Identity();
};

}  // namespace point_lio

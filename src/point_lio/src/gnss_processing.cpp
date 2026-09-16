#include "gnss_processing.h"

#include <algorithm>

#include <Eigen/Dense>  

namespace point_lio
{

namespace
{

constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr double kE2 = kF * (2.0 - kF);
constexpr double kDeg2Rad = M_PI / 180.0;
}  // namespace

void GnssProcess::Wgs84ToEcef(double lat_deg, double lon_deg, double alt, Eigen::Vector3d * ecef)
{
  const double lat = lat_deg * kDeg2Rad;
  const double lon = lon_deg * kDeg2Rad;
  const double s = std::sin(lat);
  const double N = kA / std::sqrt(1.0 - kE2 * s * s);  
  const double c = std::cos(lat);
  (*ecef)[0] = (N + alt) * c * std::cos(lon);
  (*ecef)[1] = (N + alt) * c * std::sin(lon);
  (*ecef)[2] = (N * (1.0 - kE2) + alt) * s;
}

void GnssProcess::Add(
  double stamp, double lat, double lon, double alt, const Eigen::Vector3d & cov, int status)
{
  if (!cfg_.enable) return;
  if (status < 0) return;                                 
  if (!std::isfinite(lat) || !std::isfinite(lon)) return;

  if (!has_origin_) {
    origin_lat_ = lat;
    origin_lon_ = lon;
    origin_alt_ = alt;
    Wgs84ToEcef(lat, lon, alt, &origin_ecef_);
    const double la = lat * kDeg2Rad, lo = lon * kDeg2Rad;
    const double sla = std::sin(la), cla = std::cos(la);
    const double slo = std::sin(lo), clo = std::cos(lo);
    ecef_to_enu_ << -slo,        clo,       0.0,
                    -sla * clo, -sla * slo, cla,
                     cla * clo,  cla * slo, sla;
    has_origin_ = true;
  }

  Eigen::Vector3d ecef;
  Wgs84ToEcef(lat, lon, alt, &ecef);

  GnssSample s;
  s.time = stamp;
  s.enu = ecef_to_enu_ * (ecef - origin_ecef_);
  s.enu = cfg_.extrin_r * s.enu + cfg_.extrin_t;
  s.sigma = Eigen::Vector3d(
    std::sqrt(std::max(cov[0], 1e-6)), std::sqrt(std::max(cov[1], 1e-6)),
    std::sqrt(std::max(cov[2], 1e-6)));
  for (int i = 0; i < 3; ++i) {
    s.sigma[i] = std::clamp(s.sigma[i] * cfg_.sigma_scale, cfg_.sigma_floor, cfg_.sigma_max);
  }

  buf_.push_back(s);
  while (buf_.size() > 2000) buf_.pop_front();
}

bool GnssProcess::PopAligned(double stamp, GnssSample * out)
{
  if (!cfg_.enable) return false;
  while (!buf_.empty()) {
    const GnssSample & s = buf_.front();
    if (s.time < stamp - cfg_.time_tol) {
      buf_.pop_front();        
      continue;
    }
    if (s.time > stamp + cfg_.time_tol) return false;
    const double var = cfg_.use_elevation ? s.sigma.squaredNorm() : s.sigma.head<2>().squaredNorm();
    const double lim = (cfg_.use_elevation ? 3.0 : 2.0) * cfg_.cov_threshold * cfg_.cov_threshold;
    if (var > lim) {
      buf_.pop_front();
      continue;
    }
    *out = s;
    buf_.pop_front();
    return true;
  }
  return false;
}

}  // namespace point_lio

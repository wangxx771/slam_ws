#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include "so3_math.h"

namespace point_lio
{

struct Pose3
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();

  Pose3() = default;
  Pose3(const Eigen::Matrix3d & R_in, const Eigen::Vector3d & t_in) : R(R_in), t(t_in) {}

  static Pose3 from_rpy_xyz(double roll, double pitch, double yaw, double x, double y, double z)
  {
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    Eigen::Matrix3d Rz = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Ry = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Rx = Eigen::Matrix3d::Identity();
    Rz(0, 0) = cy; Rz(0, 1) = -sy; Rz(1, 0) = sy; Rz(1, 1) = cy;
    Ry(0, 0) = cp; Ry(0, 2) = sp;  Ry(2, 0) = -sp; Ry(2, 2) = cp;
    Rx(1, 1) = cr; Rx(1, 2) = -sr; Rx(2, 1) = sr;  Rx(2, 2) = cr;
    Pose3 p;
    p.R = Rz * Ry * Rx;
    p.t = Eigen::Vector3d(x, y, z);
    return p;
  }

  double roll() const { return std::atan2(R(2, 1), R(2, 2)); }
  double pitch() const
  {
    const double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    return std::atan2(-R(2, 0), sy);
  }
  double yaw() const { return std::atan2(R(1, 0), R(0, 0)); }

  Pose3 compose(const Pose3 & o) const { return Pose3(R * o.R, t + R * o.t); }

  Pose3 inverse() const
  {
    const Eigen::Matrix3d Rt = R.transpose();
    return Pose3(Rt, -(Rt * t));
  }

  Pose3 boxplus(const std::array<double, 6> & d) const
  {
    Pose3 r;
    r.R = R * Exp(Eigen::Vector3d(d[0], d[1], d[2]));
    r.t = t + R * Eigen::Vector3d(d[3], d[4], d[5]);
    return r;
  }

  std::array<double, 6> log() const
  {
    const Eigen::Vector3d w = Log(R);
    return {w(0), w(1), w(2), t(0), t(1), t(2)};
  }
};

struct DiagNoise6
{
  std::array<double, 6> sigma{{1, 1, 1, 1, 1, 1}};  // [rot_x,rot_y,rot_z, t_x,t_y,t_z]
};
struct DiagNoise3
{
  std::array<double, 3> sigma{{1, 1, 1}};
};

class PoseGraphOptimizer
{
public:
  // Poses older than this are held fixed. A run with more keyframes than the
  // window has its graph truncated: a closure spanning the boundary pulls the
  // optimized poses toward anchors that cannot move, and the solve diverges. On
  // an 898-keyframe run at 500 the optimizer displaced keyframes by a median of
  // 106 m; the same code on a 190-keyframe run moved them 0.23 m.
  void setMaxWindow(std::size_t w) { max_window_ = w > 0 ? w : 1; }

  int addPose(const Pose3 & init)
  {
    poses_.push_back(init);
    return static_cast<int>(poses_.size()) - 1;
  }

  void setPose(int idx, const Pose3 & p)
  {
    if (idx >= 0 && static_cast<std::size_t>(idx) < poses_.size()) poses_[idx] = p;
  }

  void addPrior(int idx, const Pose3 & measured, const DiagNoise6 & noise)
  {
    priors_.push_back({idx, measured, noise});
  }

  void addBetween(int from, int to, const Pose3 & relative, const DiagNoise6 & noise)
  {
    betweens_.push_back({from, to, relative, noise});
  }

  void addGps(int idx, const Eigen::Vector3d & position, const DiagNoise3 & noise)
  {
    gps_.push_back({idx, position, noise});
  }

  Pose3 getPose(int idx) const { return poses_[idx]; }
  std::size_t numPoses() const { return poses_.size(); }
  std::size_t numBetweens() const { return betweens_.size(); }
  std::size_t numGpsFactors() const { return gps_.size(); }

  void setKeepFactors(bool keep) { keep_factors_ = keep; }


  int optimize(int max_iters = 10, double tol = 1e-4)
  {
    const std::size_t N = poses_.size();
    if (N == 0) return 0;

    const std::size_t W = N <= max_window_ ? N : max_window_;
    const std::size_t first = N - W;
    const std::size_t dim = 6 * W;

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt;
    bool pattern_analyzed = false;

    int iter_done = 0;
    for (int iter = 0; iter < max_iters; ++iter) {
      Trips trips;
      trips.reserve(36 * (4 * betweens_.size() + priors_.size() + gps_.size()) + dim);
      Eigen::VectorXd b = Eigen::VectorXd::Zero(dim);

      accumulatePriors(trips, b, first, W);
      accumulateBetweens(trips, b, first, W);
      accumulateGps(trips, b, first, W);

      Eigen::VectorXd diag = Eigen::VectorXd::Zero(dim);
      for (const auto & t : trips) {
        if (t.row() == t.col()) diag(t.row()) += t.value();
      }
      for (std::size_t d = 0; d < dim; ++d) {
        const double hii = diag(static_cast<Eigen::Index>(d));
        trips.emplace_back(
          static_cast<int>(d), static_cast<int>(d), (hii > 0.0 ? hii : 1.0) * 1e-6);
      }

      Eigen::SparseMatrix<double> H(
        static_cast<Eigen::Index>(dim), static_cast<Eigen::Index>(dim));
      H.setFromTriplets(trips.begin(), trips.end()); 

      if (!pattern_analyzed) {
        ldlt.analyzePattern(H);
        if (ldlt.info() != Eigen::Success) break;
        pattern_analyzed = true;
      }
      ldlt.factorize(H);
      if (ldlt.info() != Eigen::Success) break;
      const Eigen::VectorXd dx = ldlt.solve(b);
      if (ldlt.info() != Eigen::Success || !dx.allFinite()) break;

      double max_abs = 0.0;
      for (std::size_t k = 0; k < W; ++k) {
        std::array<double, 6> d{};
        for (int j = 0; j < 6; ++j) {
          d[j] = dx(6 * k + j);
          max_abs = std::max(max_abs, std::fabs(d[j]));
        }
        poses_[first + k] = poses_[first + k].boxplus(d);
      }

      iter_done = iter + 1;
      if (max_abs < tol) break;
    }

    if (!keep_factors_) {
      priors_.clear();
      betweens_.clear();
      gps_.clear();
    }
    return iter_done;
  }

private:
  std::size_t max_window_ = 500;

  using Trips = std::vector<Eigen::Triplet<double>>;

  struct PriorFactor
  {
    int idx;
    Pose3 measured;
    DiagNoise6 noise;
  };
  struct BetweenFactor
  {
    int from;
    int to;
    Pose3 measured;
    DiagNoise6 noise;
  };
  struct GpsFactor
  {
    int idx;
    Eigen::Vector3d position;
    DiagNoise3 noise;
  };

  std::vector<Pose3> poses_;
  std::vector<PriorFactor> priors_;
  std::vector<BetweenFactor> betweens_;
  std::vector<GpsFactor> gps_;
  bool keep_factors_ = false;

  static std::array<double, 6> priorResidual(const Pose3 & pose, const Pose3 & measured)
  {
    return measured.inverse().compose(pose).log();
  }
  static std::array<double, 6> betweenResidual(
    const Pose3 & from, const Pose3 & to, const Pose3 & measured)
  {
    return from.inverse().compose(to).compose(measured.inverse()).log();
  }
  static std::array<double, 3> gpsResidual(const Pose3 & pose, const Eigen::Vector3d & position)
  {
    return {pose.t(0) - position(0), pose.t(1) - position(1), pose.t(2) - position(2)};
  }

  template <int Rows, typename Fn>
  static void numericJacobianPose(const Pose3 & base, Fn && eval, double J[Rows][6])
  {
    constexpr double eps = 1e-6;
    const std::array<double, Rows> r0 = eval(base);
    for (int j = 0; j < 6; ++j) {
      std::array<double, 6> d{};
      d[j] = eps;
      const std::array<double, Rows> r1 = eval(base.boxplus(d));
      for (int i = 0; i < Rows; ++i) J[i][j] = (r1[i] - r0[i]) / eps;
    }
  }

  int windowIdx(int node, std::size_t first, std::size_t W) const
  {
    if (node < static_cast<int>(first)) return -1;
    if (node >= static_cast<int>(first + W)) return -1;
    return node - static_cast<int>(first);
  }

  void accumulatePriors(Trips & trips, Eigen::VectorXd & b, std::size_t first, std::size_t W)
  {
    for (const auto & f : priors_) {
      const int wi = windowIdx(f.idx, first, W);
      if (wi < 0) continue;
      const Pose3 & base = poses_[f.idx];
      auto eval = [&](const Pose3 & p) { return priorResidual(p, f.measured); };
      double J[6][6];
      numericJacobianPose<6>(base, eval, J);
      const std::array<double, 6> r = priorResidual(base, f.measured);

      std::array<double, 6> w{};
      for (int i = 0; i < 6; ++i) w[i] = 1.0 / (f.noise.sigma[i] * f.noise.sigma[i]);

      const int off = 6 * wi;
      for (int a = 0; a < 6; ++a) {
        for (int c = 0; c < 6; ++c) {
          double s = 0.0;
          for (int k = 0; k < 6; ++k) s += J[k][a] * w[k] * J[k][c];
          trips.emplace_back(off + a, off + c, s);
        }
        double bs = 0.0;
        for (int k = 0; k < 6; ++k) bs += J[k][a] * w[k] * r[k];
        b(off + a) -= bs;
      }
    }
  }

  void accumulateBetweens(Trips & trips, Eigen::VectorXd & b, std::size_t first, std::size_t W)
  {
    for (const auto & f : betweens_) {
      const int wi_from = windowIdx(f.from, first, W);
      const int wi_to = windowIdx(f.to, first, W);
      if (wi_from < 0 && wi_to < 0) continue;

      const Pose3 base_from = poses_[f.from];
      const Pose3 base_to = poses_[f.to];

      double Jf[6][6] = {{0}};
      double Jt[6][6] = {{0}};
      if (wi_from >= 0) {
        auto eval = [&](const Pose3 & p) { return betweenResidual(p, base_to, f.measured); };
        numericJacobianPose<6>(base_from, eval, Jf);
      }
      if (wi_to >= 0) {
        auto eval = [&](const Pose3 & p) { return betweenResidual(base_from, p, f.measured); };
        numericJacobianPose<6>(base_to, eval, Jt);
      }
      const std::array<double, 6> r = betweenResidual(base_from, base_to, f.measured);

      std::array<double, 6> w{};
      for (int i = 0; i < 6; ++i) w[i] = 1.0 / (f.noise.sigma[i] * f.noise.sigma[i]);

      auto stampBlock = [&](int wi_r, int wi_c, double Jr[6][6], double Jc[6][6]) {
        if (wi_r < 0 || wi_c < 0) return;
        const int orow = 6 * wi_r, ocol = 6 * wi_c;
        for (int a = 0; a < 6; ++a) {
          for (int c = 0; c < 6; ++c) {
            double s = 0.0;
            for (int k = 0; k < 6; ++k) s += Jr[k][a] * w[k] * Jc[k][c];
            trips.emplace_back(orow + a, ocol + c, s);
          }
        }
      };
      stampBlock(wi_from, wi_from, Jf, Jf);
      stampBlock(wi_to, wi_to, Jt, Jt);
      stampBlock(wi_from, wi_to, Jf, Jt);
      stampBlock(wi_to, wi_from, Jt, Jf);

      auto stampB = [&](int wi_r, double Jr[6][6]) {
        if (wi_r < 0) return;
        const std::size_t off = 6 * wi_r;
        for (int a = 0; a < 6; ++a) {
          double bs = 0.0;
          for (int k = 0; k < 6; ++k) bs += Jr[k][a] * w[k] * r[k];
          b(off + a) -= bs;
        }
      };
      stampB(wi_from, Jf);
      stampB(wi_to, Jt);
    }
  }

  void accumulateGps(Trips & trips, Eigen::VectorXd & b, std::size_t first, std::size_t W)
  {
    for (const auto & f : gps_) {
      const int wi = windowIdx(f.idx, first, W);
      if (wi < 0) continue;
      const Pose3 & base = poses_[f.idx];
      auto eval = [&](const Pose3 & p) -> std::array<double, 3> {
        return gpsResidual(p, f.position);
      };
      double J[3][6];
      numericJacobianPose<3>(base, eval, J);
      const std::array<double, 3> r = gpsResidual(base, f.position);

      std::array<double, 3> w{};
      for (int i = 0; i < 3; ++i) w[i] = 1.0 / (f.noise.sigma[i] * f.noise.sigma[i]);

      const int off = 6 * wi;
      for (int a = 0; a < 6; ++a) {
        for (int c = 0; c < 6; ++c) {
          double s = 0.0;
          for (int k = 0; k < 3; ++k) s += J[k][a] * w[k] * J[k][c];
          trips.emplace_back(off + a, off + c, s);
        }
        double bs = 0.0;
        for (int k = 0; k < 3; ++k) bs += J[k][a] * w[k] * r[k];
        b(off + a) -= bs;
      }
    }
  }
};

}  // namespace point_lio

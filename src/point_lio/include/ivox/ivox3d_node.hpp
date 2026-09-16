#include <pcl/common/centroid.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace faster_lio
{

// squared distance of two pcl points
template <typename PointT>
inline double distance2(const PointT & pt1, const PointT & pt2)
{
  Eigen::Vector3f d = pt1.getVector3fMap() - pt2.getVector3fMap();
  return d.squaredNorm();
}

// convert from pcl point to eigen
template <typename T, int dim, typename PointType>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType & pt)
{
  std::cout << "line 21" << '\n';
  return Eigen::Matrix<T, dim, 1>(pt.x, pt.y, pt.z);
  std::cout << "line 23" << '\n';
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZ>(const pcl::PointXYZ & pt)
{
  return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZI>(const pcl::PointXYZI & pt)
{
  return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZINormal>(
  const pcl::PointXYZINormal & pt)
{
  return pt.getVector3fMap();
}

template <typename PointT, int dim = 3>
class IVoxNode
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  struct DistPoint;

  IVoxNode() = default;
  IVoxNode(const PointT & center, const float & side_length) {}  /// same with phc

  inline void SetMaxPoints(const std::size_t max_pts) { max_pts_ = max_pts; }

  void InsertPoint(const PointT & pt);

  inline bool Empty() const;

  inline std::size_t Size() const;

  inline PointT GetPoint(const std::size_t idx) const;

  int KNNPointByCondition(
    std::vector<DistPoint> & dis_points, const PointT & point, const int & K,
    const double & max_range);

private:
  std::vector<PointT> points_;
  std::size_t max_pts_ = 0;
  std::size_t write_idx_ = 0;
};

template <typename PointT, int dim>
struct IVoxNode<PointT, dim>::DistPoint
{
  double dist = 0;
  IVoxNode * node = nullptr;
  int idx = 0;

  DistPoint() = default;
  DistPoint(const double d, IVoxNode * n, const int i) : dist(d), node(n), idx(i) {}

  PointT Get() { return node->GetPoint(idx); }

  inline bool operator()(const DistPoint & p1, const DistPoint & p2) { return p1.dist < p2.dist; }

  inline bool operator<(const DistPoint & rhs) { return dist < rhs.dist; }
};

template <typename PointT, int dim>
void IVoxNode<PointT, dim>::InsertPoint(const PointT & pt)
{
  if (max_pts_ == 0 || points_.size() < max_pts_) {
    points_.template emplace_back(pt);
    return;
  }
  points_[write_idx_] = pt;
  write_idx_ = (write_idx_ + 1) % max_pts_;
}

template <typename PointT, int dim>
bool IVoxNode<PointT, dim>::Empty() const
{
  return points_.empty();
}

template <typename PointT, int dim>
std::size_t IVoxNode<PointT, dim>::Size() const
{
  return points_.size();
}

template <typename PointT, int dim>
PointT IVoxNode<PointT, dim>::GetPoint(const std::size_t idx) const
{
  return points_[idx];
}

template <typename PointT, int dim>
int IVoxNode<PointT, dim>::KNNPointByCondition(
  std::vector<DistPoint> & dis_points, const PointT & point, const int & K,
  const double & max_range)
{
  std::size_t old_size = dis_points.size();
// #define INNER_TIMER
#ifdef INNER_TIMER
  static std::unordered_map<std::string, std::vector<int64_t>> stats;
  if (stats.empty()) {
    stats["dis"] = std::vector<int64_t>();
    stats["put"] = std::vector<int64_t>();
    stats["nth"] = std::vector<int64_t>();
  }
#endif

  for (const auto & pt : points_) {
#ifdef INNER_TIMER
    auto t0 = std::chrono::high_resolution_clock::now();
#endif
    double d = distance2(pt, point);
#ifdef INNER_TIMER
    auto t1 = std::chrono::high_resolution_clock::now();
#endif
    if (d < max_range * max_range) {
      dis_points.template emplace_back(DistPoint(d, this, &pt - points_.data()));
    }
#ifdef INNER_TIMER
    auto t2 = std::chrono::high_resolution_clock::now();

    auto dis = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
    stats["dis"].emplace_back(dis);
    auto put = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
    stats["put"].emplace_back(put);
#endif
  }

#ifdef INNER_TIMER
  auto t1 = std::chrono::high_resolution_clock::now();
#endif
  // sort by distance
  if (old_size + K >= dis_points.size()) {
  } else {
    std::nth_element(
      dis_points.begin() + old_size, dis_points.begin() + old_size + K - 1, dis_points.end());
    dis_points.resize(old_size + K);
  }

#ifdef INNER_TIMER
  auto t2 = std::chrono::high_resolution_clock::now();
  auto nth = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
  stats["nth"].emplace_back(nth);

  constexpr int STAT_PERIOD = 100000;
  if (!stats["nth"].empty() && stats["nth"].size() % STAT_PERIOD == 0) {
    for (auto & it : stats) {
      const std::string & key = it.first;
      std::vector<int64_t> & stat = it.second;
      int64_t sum_ = std::accumulate(stat.begin(), stat.end(), 0);
      int64_t num_ = stat.size();
      stat.clear();
      std::cout << "inner_" << key << "(ns): sum=" << sum_ << " num=" << num_
                << " ave=" << 1.0 * sum_ / num_ << " ave*n=" << 1.0 * sum_ / STAT_PERIOD << '\n';
    }
  }
#endif

  return dis_points.size();
}

}  // namespace faster_lio

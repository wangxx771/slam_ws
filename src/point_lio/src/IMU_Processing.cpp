#include "slam_log/log.h"
#include "IMU_Processing.h"
#include "parameters.h"

bool time_list(PointType & x, PointType & y) { return (x.curvature < y.curvature); };

void ImuProcess::set_gyr_cov(const V3D & scaler) { cov_gyr_scale = scaler; }

void ImuProcess::set_acc_cov(const V3D & scaler) { cov_vel_scale = scaler; }

ImuProcess::ImuProcess()
: b_first_frame_(true), imu_need_init_(true), logger(rclcpp::get_logger("ImuProcess"))
{
  imu_en = true;
  init_iter_num = 1;
  mean_acc = V3D(0, 0, 0.0);
  mean_gyr = V3D(0, 0, 0);
  after_imu_init_ = false;
  state_cov.setIdentity();
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
  PL_WARN(logger, "reset ImuProcess");
  mean_acc = V3D(0, 0, 0.0);
  mean_gyr = V3D(0, 0, 0);
  imu_need_init_ = true;
  init_iter_num = 1;
  after_imu_init_ = false;

  time_last_scan = 0.0;
}

void ImuProcess::Set_init(Eigen::Vector3d & tmp_gravity, Eigen::Matrix3d & rot)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  M3D hat_grav;
  hat_grav << 0.0, gravity_(2), -gravity_(1), -gravity_(2), 0.0, gravity_(0), gravity_(1),
    -gravity_(0), 0.0;
  double align_norm = (hat_grav * tmp_gravity).norm() / gravity_.norm() / tmp_gravity.norm();
  double align_cos = gravity_.transpose() * tmp_gravity;
  align_cos = align_cos / gravity_.norm() / tmp_gravity.norm();
  if (align_norm < 1e-6) {
    if (align_cos > 1e-6) {
      rot = Eye3d;
    } else {
      rot = -Eye3d;
    }
  } else {
    V3D align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * acos(align_cos);
    rot = Exp(align_angle(0), align_angle(1), align_angle(2));
  }
}

void ImuProcess::IMU_init(const MeasureGroup & meas, int & N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  V3D cur_acc, cur_gyr;

  if (b_first_frame_) {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto & imu_acc = meas.imu.front()->linear_acceleration;
    const auto & gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
  }

  for (const auto & imu : meas.imu) {
    const auto & imu_acc = imu->linear_acceleration;
    const auto & gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    const double t = get_time_sec(imu->header.stamp);
    if (init_t0_ < 0.0) init_t0_ = t;


    const bool still = std::fabs(cur_acc.norm() - acc_norm) < imu_init_acc_ratio * acc_norm &&
                       cur_gyr.norm() < imu_init_gyr_tol;
    if (!still && !init_relaxed_) {
      ++init_reject_;
      continue;
    }

    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;
    N++;
  }
}

void ImuProcess::Process(const MeasureGroup & meas, PointCloudXYZI::Ptr cur_pcl_un_)
{
  if (imu_en) {
    if (meas.imu.empty()) return;

    if (imu_need_init_) {
      {
        /// The very first lidar frame
        IMU_init(meas, init_iter_num);

        imu_need_init_ = true;

        const double elapsed = init_t0_ < 0.0 ? 0.0 :
          get_time_sec(meas.imu.back()->header.stamp) - init_t0_;
        const bool by_time = elapsed >= imu_init_sec && init_iter_num > 20;
        const bool by_count = init_iter_num > MAX_INI_COUNT;
        if (!init_relaxed_ && elapsed > imu_init_timeout && init_iter_num <= 20) {
          init_relaxed_ = true;
          PL_ERROR(
            logger,
            "IMU init waited %.1f s without enough stationary samples (%d rejected); "
            "continuing without filtering. The platform was not stationary at power-on, so the estimated gravity and bias are unreliable "
            "and the world vertical axis may be tilted. For a good initialization, keep it still for %.1f s before starting.",
            elapsed, init_reject_, imu_init_sec);
        }
        if (by_time || by_count) {
          const double gnorm = mean_acc.norm();
          PL_INFO(
            logger,
            "IMU init done: %.2f s, %d stationary samples (%d rejected), "
            "gravity norm %.4f (configured acc_norm %.4f), gyro bias [%.5f %.5f %.5f]",
            elapsed, init_iter_num - 1, init_reject_, gnorm, acc_norm,
            mean_gyr(0), mean_gyr(1), mean_gyr(2));
          if (std::fabs(gnorm - acc_norm) > imu_init_acc_ratio * acc_norm) {
            PL_ERROR(
              logger, "estimated gravity norm %.4f differs from the configured %.4f by %.4f, beyond tolerance - "
              "the init window probably did not fall on a genuinely stationary period", gnorm, acc_norm, std::fabs(gnorm - acc_norm));
          }
          imu_need_init_ = false;
          *cur_pcl_un_ = *(meas.lidar);
        }
      }
      return;
    }
    if (!after_imu_init_) after_imu_init_ = true;
    *cur_pcl_un_ = *(meas.lidar);
    return;
  } else {
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
}
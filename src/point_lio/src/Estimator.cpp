// #include <../include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>
#include "Estimator.h"


PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
std::vector<int> time_seq;
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI(10000, 1));
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI(10000, 1));
std::vector<V3D> pbody_list;
std::vector<PointVector> Nearest_Points;
std::shared_ptr<IVoxType> ivox_ = nullptr;  // localmap in ivox
bool point_selected_surf[100000] = {0};
std::vector<M3D> crossmat_list;
int effct_feat_num = 0;
int k = 0;
int idx = -1;
esekfom::esekf<state_input, 24, input_ikfom> kf_input;
esekfom::esekf<state_output, 30, input_ikfom> kf_output;
input_ikfom input_in;
V3D angvel_avr, acc_avr, acc_avr_norm;
int feats_down_size = 0;
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);
double G_m_s2 = 9.81;

Eigen::Matrix<double, 24, 24> process_noise_cov_input()
{
  Eigen::Matrix<double, 24, 24> cov;
  cov.setZero();
  cov.block<3, 3>(3, 3).diagonal() << gyr_cov_input, gyr_cov_input, gyr_cov_input;
  cov.block<3, 3>(12, 12).diagonal() << acc_cov_input, acc_cov_input, acc_cov_input;
  cov.block<3, 3>(15, 15).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
  cov.block<3, 3>(18, 18).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
  return cov;
}

Eigen::Matrix<double, 30, 30> process_noise_cov_output()
{
  Eigen::Matrix<double, 30, 30> cov;
  cov.setZero();
  cov.block<3, 3>(12, 12).diagonal() << vel_cov, vel_cov, vel_cov;
  cov.block<3, 3>(15, 15).diagonal() << gyr_cov_output, gyr_cov_output, gyr_cov_output;
  cov.block<3, 3>(18, 18).diagonal() << acc_cov_output, acc_cov_output, acc_cov_output;
  cov.block<3, 3>(24, 24).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
  cov.block<3, 3>(27, 27).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
  return cov;
}

Eigen::Matrix<double, 24, 1> get_f_input(state_input & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  vect3 a_inertial = s.rot * (in.acc - s.ba);  // .normalized()
  for (int i = 0; i < 3; i++) {
    res(i) = s.vel[i];
    res(i + 3) = omega[i];
    res(i + 12) = a_inertial[i] + s.gravity[i];
  }
  return res;
}

Eigen::Matrix<double, 30, 1> get_f_output(state_output & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 30, 1> res = Eigen::Matrix<double, 30, 1>::Zero();
  vect3 a_inertial = s.rot * s.acc;  // .normalized()
  for (int i = 0; i < 3; i++) {
    res(i) = s.vel[i];
    res(i + 3) = s.omg[i];
    res(i + 12) = a_inertial[i] + s.gravity[i];
  }
  return res;
}

Eigen::Matrix<double, 24, 24> df_dx_input(state_input & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
  cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  vect3 acc_;
  in.acc.boxminus(acc_, s.ba);
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  cov.template block<3, 3>(12, 3) = -s.rot * MTK::hat(acc_);  // .normalized().toRotationMatrix()
  cov.template block<3, 3>(12, 18) = -s.rot;                  //.normalized().toRotationMatrix();
  cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();  // grav_matrix;
  cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
  return cov;
}

Eigen::Matrix<double, 30, 30> df_dx_output(state_output & s, const input_ikfom & in)
{
  Eigen::Matrix<double, 30, 30> cov = Eigen::Matrix<double, 30, 30>::Zero();
  cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(12, 3) = -s.rot * MTK::hat(s.acc);  // .normalized().toRotationMatrix()
  cov.template block<3, 3>(12, 18) = s.rot;                    //.normalized().toRotationMatrix();
  cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();  // grav_matrix;
  cov.template block<3, 3>(3, 15) = Eigen::Matrix3d::Identity();
  return cov;
}
namespace {

double zc_nz2_thresh = 0.02;
double zc_weight = 0.15;
double zc_max_hold_s = 60.0;   ///< Latch hold limit; released on timeout
double zc_max_tilt = 0.105;
bool   zc_enable = false;

constexpr double kZcDecay = 0.99;
double zc_nz2_acc = 0.0, zc_n_acc = 0.0;
constexpr double kZcTiltDecay = 0.9995;
double zc_pitch_ema = 0.0;

double zc_diag_ratio = 0.0, zc_diag_tilt = 0.0;
double zc_z_ref = 0.0;
bool   zc_z_ref_valid = false;
double zc_z_ref_time = 0.0;
long   zc_degen_frames = 0, zc_total_frames = 0;
bool zc_update(double nz2_sum, int npt, double z_cur, double now, double sin_pitch)
{
  zc_nz2_acc = kZcDecay * zc_nz2_acc + nz2_sum;
  zc_n_acc = kZcDecay * zc_n_acc + npt;
  zc_pitch_ema = kZcTiltDecay * zc_pitch_ema + (1.0 - kZcTiltDecay) * sin_pitch;
  ++zc_total_frames;
  zc_diag_ratio = (zc_n_acc > 1e-6) ? zc_nz2_acc / zc_n_acc : 0.0;
  zc_diag_tilt = std::asin(std::min(1.0, std::max(-1.0, zc_pitch_ema))) * 180.0 / M_PI;
  if (!zc_enable || zc_n_acc < 50.0) {return false;}

  if (zc_nz2_acc / zc_n_acc >= zc_nz2_thresh) {
    zc_z_ref = z_cur;
    zc_z_ref_valid = true;
    zc_z_ref_time = now;
    return false;
  }
  if (std::fabs(zc_pitch_ema) > std::sin(zc_max_tilt)) {
    zc_z_ref = z_cur;
    zc_z_ref_valid = true;
    zc_z_ref_time = now;
    return false;
  }
  ++zc_degen_frames;
  if (!zc_z_ref_valid) {return false;}
  if (now - zc_z_ref_time > zc_max_hold_s) {
    zc_z_ref_valid = false;
    return false;
  }
  return true;
}

}  // namespace

void zc_configure(
  bool enable, double nz2_thresh, double weight, double max_hold_s, double max_tilt_deg)
{
  zc_max_tilt = max_tilt_deg * M_PI / 180.0;
  zc_enable = enable;
  zc_nz2_thresh = nz2_thresh;
  zc_weight = weight;
  zc_max_hold_s = max_hold_s;
  zc_z_ref_valid = false;
  zc_nz2_acc = zc_n_acc = 0.0;
  zc_pitch_ema = 0.0;
  zc_degen_frames = zc_total_frames = 0;
}

void zc_stats(long & degen, long & total) {degen = zc_degen_frames; total = zc_total_frames;}

void zc_diag(double & nz2_ratio, double & tilt_deg, double & z_ref, bool & ref_valid)
{
  nz2_ratio = zc_diag_ratio;
  tilt_deg = zc_diag_tilt;
  z_ref = zc_z_ref;
  ref_valid = zc_z_ref_valid;
}

void h_model_input(
  state_input & s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R,
  esekfom::dyn_share_modified<double> & ekfom_data)
{
  VF(4) pabcd;
  pabcd.setZero();
  int effect_num_k = 0;

  normvec->resize(time_seq[k]);
  for (int j = 0; j < time_seq[k]; j++) {
    const int gi = idx + j + 1;
    PointType & point_body_j = feats_down_body->points[gi];
    PointType & point_world_j = feats_down_world->points[gi];
    pointBodyToWorld(&point_body_j, &point_world_j);
    V3D p_body = pbody_list[gi];
    double p_norm = p_body.norm();
    V3D p_world;
    p_world << point_world_j.x, point_world_j.y, point_world_j.z;
    {
      auto & points_near = Nearest_Points[gi];
      ivox_->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS, 5.0);
      if ((points_near.size() <
           NUM_MATCH_POINTS))
      {
        point_selected_surf[gi] = false;
      } else {
        point_selected_surf[gi] = false;
        if (esti_plane(pabcd, points_near, plane_thr))  //(planeValid)
        {
          float pd2 = fabs(
            pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z +
            pabcd(3));
          if (p_norm > match_s * pd2 * pd2) {
            point_selected_surf[gi] = true;
            normvec->points[j].x = pabcd(0);
            normvec->points[j].y = pabcd(1);
            normvec->points[j].z = pabcd(2);
            normvec->points[j].intensity = pabcd(3);
            effect_num_k++;

          }
        }
      }
    }
  }
  if (effect_num_k == 0) {
    ekfom_data.valid = false;
    return;
  }
  ekfom_data.M_Noise = laser_point_cov;
  ekfom_data.h_x.resize(effect_num_k, 12);
  ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
  ekfom_data.z.resize(effect_num_k);
  int m = 0;
  double nz2_sum = 0.0;

  for (int j = 0; j < time_seq[k]; j++) {
    if (point_selected_surf[idx + j + 1]) {
      V3D norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);

      if (extrinsic_est_en) {
        V3D p_body = pbody_list[idx + j + 1];
        M3D p_crossmat, p_imu_crossmat;
        p_crossmat << SKEW_SYM_MATRX(p_body);
        V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
        p_imu_crossmat << SKEW_SYM_MATRX(point_imu);
        V3D C(s.rot.transpose() * norm_vec);
        V3D A(p_imu_crossmat * C);
        V3D B(p_crossmat * s.offset_R_L_I.transpose() * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2),
          VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
      } else {
        M3D point_crossmat = crossmat_list[idx + j + 1];
        V3D C(s.rot.transpose() * norm_vec);  // conjugate().normalized()
        V3D A(point_crossmat * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2),
          VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      }
      ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx + j + 1].x -
                        norm_vec(1) * feats_down_world->points[idx + j + 1].y -
                        norm_vec(2) * feats_down_world->points[idx + j + 1].z -
                        normvec->points[j].intensity;
      nz2_sum += norm_vec(2) * norm_vec(2);

      m++;
    }
  }

  if (m > 0 && zc_update(nz2_sum, m, s.pos(2), lidar_end_time, s.rot(2, 0))) {
    const double k = std::sqrt(zc_weight * m);
    ekfom_data.h_x.conservativeResize(m + 1, 12);
    ekfom_data.h_x.row(m).setZero();
    ekfom_data.h_x(m, 2) = k;
    ekfom_data.z.conservativeResize(m + 1);
    ekfom_data.z(m) = k * (zc_z_ref - s.pos(2));
  }
  effct_feat_num += effect_num_k;
}

void h_model_output(
  state_output & s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R,
  esekfom::dyn_share_modified<double> & ekfom_data)
{
  VF(4) pabcd;
  pabcd.setZero();
  int effect_num_k = 0;

  normvec->resize(time_seq[k]);
  for (int j = 0; j < time_seq[k]; j++) {
    const int gi = idx + j + 1;
    PointType & point_body_j = feats_down_body->points[gi];
    PointType & point_world_j = feats_down_world->points[gi];
    pointBodyToWorld(&point_body_j, &point_world_j);
    V3D p_body = pbody_list[gi];
    double p_norm = p_body.norm();
    V3D p_world;
    p_world << point_world_j.x, point_world_j.y, point_world_j.z;
    {
      auto & points_near = Nearest_Points[gi];
      ivox_->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS, 5.0);

      if ((points_near.size() <
           NUM_MATCH_POINTS))
      {
        point_selected_surf[gi] = false;
      } else {
        point_selected_surf[gi] = false;
        if (esti_plane(pabcd, points_near, plane_thr))  //(planeValid)
        {
          float pd2 = fabs(
            pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z +
            pabcd(3));
          if (p_norm > match_s * pd2 * pd2) {
            point_selected_surf[gi] = true;
            normvec->points[j].x = pabcd(0);
            normvec->points[j].y = pabcd(1);
            normvec->points[j].z = pabcd(2);
            normvec->points[j].intensity = pabcd(3);
            effect_num_k++;

          }
        }
      }
    }
  }
  if (effect_num_k == 0) {
    ekfom_data.valid = false;
    return;
  }
  ekfom_data.M_Noise = laser_point_cov;
  ekfom_data.h_x.resize(effect_num_k, 12);
  ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
  ekfom_data.z.resize(effect_num_k);
  int m = 0;
  double nz2_sum = 0.0;   
  for (int j = 0; j < time_seq[k]; j++) {
    if (point_selected_surf[idx + j + 1]) {
      V3D norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);
      if (extrinsic_est_en) {
        V3D p_body = pbody_list[idx + j + 1];
        M3D p_crossmat, p_imu_crossmat;
        p_crossmat << SKEW_SYM_MATRX(p_body);
        V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
        p_imu_crossmat << SKEW_SYM_MATRX(point_imu);
        V3D C(s.rot.transpose() * norm_vec);
        V3D A(p_imu_crossmat * C);
        V3D B(p_crossmat * s.offset_R_L_I.transpose() * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2),
          VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
      } else {
        M3D point_crossmat = crossmat_list[idx + j + 1];
        V3D C(s.rot.transpose() * norm_vec);  // conjugate().normalized()
        V3D A(point_crossmat * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2),
          VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      }
      ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx + j + 1].x -
                        norm_vec(1) * feats_down_world->points[idx + j + 1].y -
                        norm_vec(2) * feats_down_world->points[idx + j + 1].z -
                        normvec->points[j].intensity;
      nz2_sum += norm_vec(2) * norm_vec(2);

      m++;
    }
  }
  if (m > 0 && zc_update(nz2_sum, m, s.pos(2), lidar_end_time, s.rot(2, 0))) {
    const double k = std::sqrt(zc_weight * m);
    ekfom_data.h_x.conservativeResize(m + 1, 12);
    ekfom_data.h_x.row(m).setZero();
    ekfom_data.h_x(m, 2) = k;
    ekfom_data.z.conservativeResize(m + 1);
    ekfom_data.z(m) = k * (zc_z_ref - s.pos(2));
  }
  effct_feat_num += effect_num_k;
}

void h_model_IMU_output(state_output & s, esekfom::dyn_share_modified<double> & ekfom_data)
{
  std::memset(ekfom_data.satu_check, false, 6);
  ekfom_data.z_IMU.block<3, 1>(0, 0) = angvel_avr - s.omg - s.bg;
  ekfom_data.z_IMU.block<3, 1>(3, 0) = acc_avr * G_m_s2 / acc_norm - s.acc - s.ba;
  ekfom_data.R_IMU << imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_acc_cov,
    imu_meas_acc_cov, imu_meas_acc_cov;
  if (check_satu) {
    if (fabs(angvel_avr(0)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[0] = true;
      ekfom_data.z_IMU(0) = 0.0;
    }

    if (fabs(angvel_avr(1)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[1] = true;
      ekfom_data.z_IMU(1) = 0.0;
    }

    if (fabs(angvel_avr(2)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[2] = true;
      ekfom_data.z_IMU(2) = 0.0;
    }

    if (fabs(acc_avr(0)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[3] = true;
      ekfom_data.z_IMU(3) = 0.0;
    }

    if (fabs(acc_avr(1)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[4] = true;
      ekfom_data.z_IMU(4) = 0.0;
    }

    if (fabs(acc_avr(2)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[5] = true;
      ekfom_data.z_IMU(5) = 0.0;
    }
  }
}

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
  V3D p_body(pi->x, pi->y, pi->z);

  V3D p_global;
  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      p_global =
        kf_output.x_.rot * (kf_output.x_.offset_R_L_I * p_body + kf_output.x_.offset_T_L_I) +
        kf_output.x_.pos;
    } else {
      p_global = kf_input.x_.rot * (kf_input.x_.offset_R_L_I * p_body + kf_input.x_.offset_T_L_I) +
                 kf_input.x_.pos;
    }
  } else {
    if (!use_imu_as_input) {
      p_global = kf_output.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) +
                 kf_output.x_.pos;  // .normalized()
    } else {
      p_global = kf_input.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) +
                 kf_input.x_.pos;  // .normalized()
    }
  }

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}
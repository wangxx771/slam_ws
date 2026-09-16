#ifndef Estimator_H
#define Estimator_H

#include "common_lib.h"
#include "parameters.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
// #include <ikd-Tree/ikd_Tree.h>
#include <pcl/io/pcd_io.h>
#include <unordered_set>



extern PointCloudXYZI::Ptr normvec; //(new PointCloudXYZI(100000, 1));
extern std::vector<int> time_seq;
extern PointCloudXYZI::Ptr feats_down_body; //(new PointCloudXYZI());
extern PointCloudXYZI::Ptr feats_down_world; //(new PointCloudXYZI());
extern std::vector<V3D> pbody_list;
extern std::vector<PointVector> Nearest_Points; 
extern std::shared_ptr<IVoxType> ivox_;                    // localmap in ivox
extern bool point_selected_surf[100000]; // = {0};
extern std::vector<M3D> crossmat_list;
extern int effct_feat_num;
extern int k;
extern int idx;
extern V3D angvel_avr, acc_avr, acc_avr_norm;
extern int feats_down_size;
extern V3D Lidar_T_wrt_IMU; //(Zero3d);
extern M3D Lidar_R_wrt_IMU; //(Eye3d);
extern double G_m_s2;
extern input_ikfom input_in;

Eigen::Matrix<double, 24, 24> process_noise_cov_input();

Eigen::Matrix<double, 30, 30> process_noise_cov_output();

Eigen::Matrix<double, 24, 1> get_f_input(state_input &s, const input_ikfom &in);

Eigen::Matrix<double, 30, 1> get_f_output(state_output &s, const input_ikfom &in);

Eigen::Matrix<double, 24, 24> df_dx_input(state_input &s, const input_ikfom &in);


Eigen::Matrix<double, 30, 30> df_dx_output(state_output &s, const input_ikfom &in);


void h_model_input(state_input &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data);

void h_model_output(state_output &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data);

void h_model_IMU_output(state_output &s, esekfom::dyn_share_modified<double> &ekfom_data);

void zc_configure(bool enable, double nz2_thresh, double weight, double max_hold_s,
                  double max_tilt_deg);
void zc_stats(long &degen, long &total);
void zc_diag(double &nz2_ratio, double &tilt_deg, double &z_ref, bool &ref_valid);

void pointBodyToWorld(PointType const * const pi, PointType * const po);

#endif
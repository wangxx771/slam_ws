#include "slam_log/log.h"
#include <algorithm>
#include <cstdint>
#include <vector>
#include <malloc.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <ctime>
#include <map>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <cerrno>
#include <cstring>
#include <sstream>
#include <filesystem>

#include "gnss_processing.h"
#include <interface/srv/save_maps.hpp>
#include <std_msgs/msg/empty.hpp>

#include "li_initialization.h"
#include "loop_closure.h"
#include "tile_map_writer.h"
#include "small_gicp_relocalization/scan_context.hpp"

using namespace std;

#define PUBFRAME_PERIOD (20)

const float MOV_THRESHOLD = 1.5f;


int time_log_counter = 0;

bool init_map = false, flg_first_scan = true;

// Time Log Variables
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

bool flg_reset = false, flg_exit = false;

//surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomImu;
geometry_msgs::msg::PoseStamped msg_body_pose;


struct ImuOdomSnapshot {
  bool   valid = false;
  double t_stamp = 0.0;
  Eigen::Vector3d    p   = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q   = Eigen::Quaterniond::Identity();
  Eigen::Vector3d    v   = Eigen::Vector3d::Zero();
  Eigen::Vector3d    ba  = Eigen::Vector3d::Zero();
  Eigen::Vector3d    bg  = Eigen::Vector3d::Zero();
  Eigen::Vector3d    g   = Eigen::Vector3d::Zero();
  Eigen::Vector3d    last_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d    last_gyr = Eigen::Vector3d::Zero();
  bool   has_last_imu = false;
};
ImuOdomSnapshot imu_odom_snap;

double last_pub_odom_t = -1.0;

int sleep_time = 0;

auto LOGGER = rclcpp::get_logger("laserMapping");

void SigHandle(int sig)
{
  flg_exit = true;
  PL_WARN(LOGGER, "catch sig %d", sig);
  sig_buffer.notify_all();
}

static std::filesystem::path expand_path(const std::string & raw)
{
  std::string p = raw;
  if (!p.empty() && p[0] == '~') {
    const char * home = std::getenv("HOME");
    if (home) p = std::string(home) + p.substr(1);
  }
  std::string out;
  for (std::size_t i = 0; i < p.size();) {
    if (p[i] == '$') {
      std::size_t j = i + 1;
      bool brace = (j < p.size() && p[j] == '{');
      if (brace) ++j;
      std::size_t k = j;
      while (k < p.size() && (std::isalnum(static_cast<unsigned char>(p[k])) || p[k] == '_')) ++k;
      const std::string name = p.substr(j, k - j);
      const char * val = name.empty() ? nullptr : std::getenv(name.c_str());
      if (val) out += val;
      i = (brace && k < p.size() && p[k] == '}') ? k + 1 : k;
    } else {
      out += p[i++];
    }
  }
  std::filesystem::path fp(out);
  if (fp.is_relative()) fp = std::filesystem::current_path() / fp;
  return fp.lexically_normal();
}

static std::filesystem::path g_pcd_dir;
static bool export_patches = false;
static point_lio::TileMapWriter<PointType> g_tile_map;

// Absolute, so a map never follows whatever directory the node was launched from.
static constexpr const char * kDefaultMapDir = "/var/opt/map";

static std::filesystem::path prepare_pcd_dir()
{
  const std::filesystem::path dir =
    pcd_save_dir.empty() ? std::filesystem::path(kDefaultMapDir) : expand_path(pcd_save_dir);

  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec && !std::filesystem::is_directory(dir)) {
    PL_ERROR(
      LOGGER, "cannot create cloud output dir '%s': %s", dir.c_str(), ec.message().c_str());
    return {};
  }
  const std::filesystem::path probe = dir / ".pointlio_write_test";
  {
    std::ofstream f(probe, std::ios::binary);
    if (!f) {
      PL_ERROR(LOGGER, "cloud output dir not writable: %s", dir.c_str());
      return {};
    }
    f.put('\0');
  }
  std::filesystem::remove(probe, ec);

  if (!pcd_run_subdir) {
    PL_INFO(LOGGER, "cloud output dir: %s (run_subdir off, writing directly)", dir.c_str());
    return dir;
  }

  const std::time_t now = std::time(nullptr);
  std::tm tmv{};
  localtime_r(&now, &tmv);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "map_%Y%m%d_%H%M%S", &tmv);
  const std::filesystem::path run_dir = dir / stamp;
  std::filesystem::create_directories(run_dir, ec);
  if (ec && !std::filesystem::is_directory(run_dir)) {
    PL_ERROR(LOGGER, "cannot create run output dir '%s'", run_dir.c_str());
    return {};
  }

  // A latest symlink points at this run so downstream (prior_pcd and friends) can
  // still hard-code one path. Timestamped directories fixed overwriting but made
  // "where is the map" a lookup every time; latest brings that back.
  const std::filesystem::path link = dir / "latest";
  std::filesystem::remove(link, ec);
  std::filesystem::create_directory_symlink(stamp, link, ec);
  if (ec) {
    PL_WARN(LOGGER, "cannot create 'latest' symlink (%s); map saving unaffected", ec.message().c_str());
  }

  PL_INFO(LOGGER, "cloud output dir: %s  (also %s)", run_dir.c_str(), link.c_str());
  return run_dir;
}

static void save_pcd(const std::filesystem::path & file, const PointCloudXYZI & cloud)
{
  std::vector<pcl::PCLPointField> fields;
  pcl::getFields<PointType>(fields);
  fields.erase(
    std::remove_if(
      fields.begin(), fields.end(),
      [](const pcl::PCLPointField & f) { return f.name == "_"; }),
    fields.end());

  std::size_t rec = 0;
  for (const auto & f : fields) rec += pcl::getFieldSize(f.datatype) * f.count;

  std::ostringstream hs;
  hs << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS";
  for (const auto & f : fields) hs << " " << f.name;
  hs << "\nSIZE";
  for (const auto & f : fields) hs << " " << pcl::getFieldSize(f.datatype);
  hs << "\nTYPE";
  for (const auto & f : fields) hs << " " << pcl::getFieldType(f.datatype);
  hs << "\nCOUNT";
  for (const auto & f : fields) hs << " " << f.count;
  hs << "\nWIDTH " << cloud.size() << "\nHEIGHT 1"
     << "\nVIEWPOINT 0 0 0 1 0 0 0"
     << "\nPOINTS " << cloud.size() << "\nDATA binary\n";
  const std::string header = hs.str();

  const std::filesystem::path tmp = file.string() + ".part";
  std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
  if (!out) {
    PL_ERROR(LOGGER, "cannot open '%s' for writing", tmp.c_str());
    return;
  }
  out.write(header.data(), static_cast<std::streamsize>(header.size()));

  std::vector<char> buf(rec * std::min<std::size_t>(cloud.size(), 65536));
  std::size_t n = 0;
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    char * dst = buf.data() + n * rec;
    const char * src = reinterpret_cast<const char *>(&cloud.points[i]);
    for (const auto & f : fields) {
      const std::size_t sz = pcl::getFieldSize(f.datatype) * f.count;
      std::memcpy(dst, src + f.offset, sz);
      dst += sz;
    }
    if (++n * rec == buf.size()) {
      out.write(buf.data(), static_cast<std::streamsize>(buf.size()));
      n = 0;
    }
  }
  if (n) out.write(buf.data(), static_cast<std::streamsize>(n * rec));
  out.flush();
  const bool ok = out.good();
  out.close();

  std::error_code ec;
  if (!ok) {
    PL_ERROR(LOGGER, "write to '%s' failed (disk full or network share dropped?)", tmp.c_str());
    std::filesystem::remove(tmp, ec);
    return;
  }
  std::filesystem::rename(tmp, file, ec);
  if (ec) {
    std::filesystem::copy_file(
      tmp, file, std::filesystem::copy_options::overwrite_existing, ec);
    std::filesystem::remove(tmp, ec);
  }
  PL_INFO(LOGGER, "map saved: %s (%zu points)", file.c_str(), cloud.size());
}

PointCloudXYZI::Ptr loadPointcloudFromPcd(const std::string & file_path)
{
  auto pcd_ptr = std::make_shared<PointCloudXYZI>();

  if (pcl::io::loadPCDFile(file_path, *pcd_ptr) == -1) {
    PL_ERROR(LOGGER, "Couldn't read pcd file %s", file_path.c_str());
    return nullptr;
  }

  PL_INFO(LOGGER, "Loaded %zu points from %s", pcd_ptr->size(), file_path.c_str());
  return pcd_ptr;
}

inline void dump_lio_state_to_log(FILE * fp)
{
  if (fp == nullptr) {
    return;
  }

  V3D rot_ang;
  if (!use_imu_as_input) {
    rot_ang = SO3ToEuler(kf_output.x_.rot);
  } else {
    rot_ang = SO3ToEuler(kf_input.x_.rot);
  }

  fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
  fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));  // Angle
  if (use_imu_as_input) {
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2));  // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // omega
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2));  // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                               // Acc
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));  // Bias_g
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));  // Bias_a
    fprintf(
      fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1),
      kf_input.x_.gravity(2));  // Bias_a
  } else {
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2));  // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                            // omega
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2));  // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                            // Acc
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));  // Bias_g
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));  // Bias_a
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1),
      kf_output.x_.gravity(2));  // Bias_a
  }
  fprintf(fp, "\r\n");
  fflush(fp);
}

void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu;
  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
    } else {
      p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
    }
  } else {
    p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
  }
  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}


void MapIncremental()
{
  PointVector points_to_add;
  int cur_pts = feats_down_world->size();
  points_to_add.reserve(cur_pts);

  for (size_t i = 0; i < cur_pts; ++i) {
    /* decide if need add to map */
    PointType & point_world = feats_down_world->points[i];
    if (!Nearest_Points[i].empty()) {
      const PointVector & points_near = Nearest_Points[i];

      Eigen::Vector3f center =
        ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) *
        filter_size_map_min;
      bool need_add = true;
      for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
        Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
        if (
          fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
          fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
          fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
          need_add = false;
          break;
        }
      }
      if (need_add) {
        points_to_add.emplace_back(point_world);
      }
    } else {
      points_to_add.emplace_back(point_world);
    }
  }
  ivox_->AddPoints(points_to_add);
}


std::ofstream replay_odom_csv;
static std::string g_replay_kf_csv;

point_lio::LoopClosure loop_backend;
point_lio::GnssProcess gnss_proc;

static void get_filter_pose(Eigen::Vector3d & pos, Eigen::Matrix3d & rot)
{
  if (use_imu_as_input) {
    pos = kf_input.x_.pos;
    rot = kf_input.x_.rot;
  } else {
    pos = kf_output.x_.pos;
    rot = kf_output.x_.rot;
  }
}


static point_lio::Pose3 pg_correct;
static bool pg_correct_valid = false;

static scan_context::Database sc_db;
static std::string sc_save_path;
static double sc_dedup_dist = 1.5;  
static std::vector<float> sc_accum;
static Eigen::Vector3d sc_last_pos(1e9, 1e9, 1e9);
static bool sc_enable = false;
static double sc_stride = 2.0;

static void update_scan_context(const Eigen::Vector3d & pos, const Eigen::Matrix3d & rot)
{
  if (!sc_enable || feats_down_world->empty()) return;


  constexpr std::size_t kMaxAccumFloats = 3000000;  
  if (sc_accum.size() < kMaxAccumFloats) {
    sc_accum.reserve(sc_accum.size() + feats_down_world->size() * 3);
    for (const auto & p : feats_down_world->points) {
      sc_accum.push_back(p.x);
      sc_accum.push_back(p.y);
      sc_accum.push_back(p.z);
    }
  }
  if ((pos - sc_last_pos).norm() < sc_stride) return;

  const Eigen::Vector3d gpos = pos;
  const Eigen::Matrix3d grot = rot;

  std::vector<float> rel;
  rel.resize(sc_accum.size());
  for (std::size_t i = 0; i + 2 < sc_accum.size(); i += 3) {
    rel[i]     = static_cast<float>(sc_accum[i] - pos.x());
    rel[i + 1] = static_cast<float>(sc_accum[i + 1] - pos.y());
    rel[i + 2] = static_cast<float>(sc_accum[i + 2] - pos.z());
  }
  if (sc_dedup_dist > 0.0) {
    for (const auto & e : sc_db.entries) {
      const double dx = e.x - gpos.x(), dy = e.y - gpos.y(), dz = e.z - gpos.z();
      if (dx * dx + dy * dy + dz * dz < sc_dedup_dist * sc_dedup_dist) {
        sc_accum.clear();
        sc_last_pos = pos;
        return;
      }
    }
  }

  const double yaw = std::atan2(grot(1, 0), grot(0, 0));
  sc_db.add(
    static_cast<float>(gpos.x()), static_cast<float>(gpos.y()), static_cast<float>(gpos.z()),
    static_cast<float>(yaw), scan_context::make(rel, sc_db.cfg));
  sc_accum.clear();
  sc_last_pos = pos;
}

// Write the map, its scan-context database and optionally the keyframe patches.
// dir empty means the configured output directory. Shared by the shutdown path
// and the SaveMaps service so an operator gets the same artefacts either way.
struct SaveReport
{
  bool ok{false};
  std::string message;
};

static SaveReport save_maps_to(const std::filesystem::path & dir, bool want_patches);
extern PointCloudXYZI::Ptr pcl_wait_save;

static bool save_scan_context(const std::filesystem::path & dir)
{
  if (!sc_enable || sc_db.entries.empty()) return false;
  std::filesystem::path scf = sc_save_path;
  if (scf.empty()) {
    if (dir.empty()) return false;
    scf = dir / ((pcd_save_name.empty() ? "map.pcd" : pcd_save_name) + ".sc");
  }
  if (pg_correct_valid && !sc_db.entries.empty()) {
    const Eigen::Matrix3d & R = pg_correct.R;
    const double dyaw = std::atan2(R(1, 0), R(0, 0));
    const int ns = sc_db.cfg.num_sector;
    const int nr = sc_db.cfg.num_ring;
    int shift = static_cast<int>(std::lround(dyaw / (2.0 * M_PI) * ns)) % ns;
    if (shift < 0) shift += ns;
    for (auto & e : sc_db.entries) {
      const Eigen::Vector3d p = R * Eigen::Vector3d(e.x, e.y, e.z) + pg_correct.t;
      e.x = static_cast<float>(p.x());
      e.y = static_cast<float>(p.y());
      e.z = static_cast<float>(p.z());
      e.yaw = static_cast<float>(e.yaw + dyaw);
      if (shift != 0 && static_cast<int>(e.desc.size()) == nr * ns) {
        std::vector<float> d(e.desc.size());
        for (int r = 0; r < nr; ++r) {
          for (int s = 0; s < ns; ++s) {
            d[static_cast<std::size_t>(r) * ns + (s + shift) % ns] =
              e.desc[static_cast<std::size_t>(r) * ns + s];
          }
        }
        e.desc.swap(d);
      }
    }
    PL_INFO(
      LOGGER, "scan-context DB aligned to map frame: %.3f m, yaw %.2f deg (descriptor shifted %d/%d bins)",
      pg_correct.t.norm(), dyaw * 180.0 / M_PI, shift, ns);
  }

  std::error_code ec;
  std::filesystem::create_directories(scf.parent_path(), ec);
  if (!sc_db.save(scf.string())) {
    PL_ERROR(LOGGER, "scan-context DB write failed: %s", scf.c_str());
    return false;
  }
  PL_INFO(
    LOGGER, "scan-context DB saved: %s (%zu entries, one per %.1f m)",
    scf.c_str(), sc_db.entries.size(), sc_stride);
  return true;
}


// Last map->odom that resolved, reused when a lookup misses.
//
// Both odometry publishers used to fall through to the uncorrected odom pose on
// a miss while still labelling it `map`, so the stream silently mixed two
// frames: 65% of one run's rows sat near the odom origin and a relocalization
// accurate to 0.109 m measured as 13.92 m. On a miss the pose is now labelled
// `odom`, which lets a consumer tell the two apart.
static bool g_have_m2o = false;
static Eigen::Vector3d g_t_m2o = Eigen::Vector3d::Zero();
static Eigen::Quaterniond g_q_m2o = Eigen::Quaterniond::Identity();

static bool refresh_map_to_odom(const std::shared_ptr<tf2_ros::Buffer> & buf)
{
  try {
    const auto t = buf->lookupTransform("map", "odom", tf2::TimePointZero);
    g_t_m2o = Eigen::Vector3d(
      t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    g_q_m2o = Eigen::Quaterniond(
      t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y,
      t.transform.rotation.z);
    g_have_m2o = true;
  } catch (const tf2::TransformException &) {
  }
  return g_have_m2o;
}


// Write the map, its scan-context database and optionally the keyframe patches.
// Target directory empty means the configured output directory.
static SaveReport save_maps_to_impl(const std::filesystem::path & where, bool want_patches);

// PCL reports a failed write by throwing, and an exception escaping a service
// callback terminates the process - one bad path would take the whole mapping
// run with it.
static SaveReport save_maps_to(const std::filesystem::path & where, bool want_patches)
{
  try {
    return save_maps_to_impl(where, want_patches);
  } catch (const std::exception & e) {
    SaveReport r;
    r.message = "save to '" + (where.empty() ? g_pcd_dir.string() : where.string()) +
      "' failed: " + e.what();
    return r;
  }
}

static SaveReport save_maps_to_impl(const std::filesystem::path & where, bool want_patches)
{
  SaveReport r;
  std::filesystem::path dir = where.empty() ? g_pcd_dir : expand_path(where.string());
  if (dir.empty()) {
    r.message = "no output directory: set pcd_save.output_dir or pass file_path";
    return r;
  }
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec && !std::filesystem::is_directory(dir)) {
    r.message = "cannot create " + dir.string() + ": " + ec.message();
    return r;
  }

  // The loop correction is a rigid transform applied once, here, rather than to
  // the live odometry - see the note on publish_odometry.
  const Eigen::Matrix3f Rf = pg_correct_valid
    ? Eigen::Matrix3f(pg_correct.R.cast<float>()) : Eigen::Matrix3f::Identity();
  const Eigen::Vector3f tf_ = pg_correct_valid
    ? Eigen::Vector3f(pg_correct.t.cast<float>()) : Eigen::Vector3f::Zero();

  const std::string mapname = pcd_save_name.empty() ? "map.pcd" : pcd_save_name;
  const auto out = dir / mapname;
  std::size_t n = 0;
  if (g_tile_map.enabled()) {
    // Keep the shards: the run continues after an on-demand save, and the
    // caller may well save again.
    n = g_tile_map.FinalizeToFile(out, static_cast<float>(pcd_tile_voxel), Rf, tf_, false);
    if (n == 0) {
      r.message = "tile merge failed; shards kept in " + dir.string() + "/tiles";
      return r;
    }
  } else if (!pcl_wait_save->empty()) {
    PointCloudXYZI copy = *pcl_wait_save;
    if (pg_correct_valid) {
      for (auto & pt : copy.points) {
        const Eigen::Vector3f q = Rf * Eigen::Vector3f(pt.x, pt.y, pt.z) + tf_;
        pt.x = q.x(); pt.y = q.y(); pt.z = q.z();
      }
    }
    save_pcd(out, copy);
    n = copy.size();
  } else {
    r.message = "no points accumulated yet";
    return r;
  }

  // The relocalization node needs the descriptor database beside the map, and
  // open3d_loc needs the per-keyframe patches; a map.pcd on its own leaves both
  // without candidates.
  std::string extra;
  if (!save_scan_context(dir)) {extra += " (scan-context database not written)";}
  if (want_patches) {
    if (!loop_en) {
      extra += " (patches need loop_closure.enable)";
    } else {
      const std::size_t k = loop_backend.ExportKeyframePatches(dir.string());
      extra += " (" + std::to_string(k) + " keyframe patches)";
    }
  }

  r.ok = true;
  r.message = out.string() + ": " + std::to_string(n) + " points" + extra;
  return r;
}

static void handle_loop_closure()
{
  Eigen::Vector3d pos;
  Eigen::Matrix3d rot;
  get_filter_pose(pos, rot);

  const bool gnss_for_graph = gnss_proc.cfg().enable;
  point_lio::GnssSample g;
  const bool has_g = gnss_for_graph && gnss_proc.PopAligned(lidar_end_time, &g);
  if (has_g && !gnss_proc.cfg().use_elevation) {
    g.enu.z() = (pg_correct.R * pos + pg_correct.t).z();
    g.sigma.z() = 0.01;
  }

  // Enqueue only. Detection, ICP and pose-graph optimization all run on the
  // backend thread; nothing here may block the LIO loop.
  loop_backend.AddFrame(
    lidar_end_time, pos, rot, feats_down_body,
    has_g ? &g.enu : nullptr, has_g ? &g.sigma : nullptr, feats_undistort);

  point_lio::Pose3 fresh;
  if (!loop_backend.PollCorrection(&fresh)) return;

  const point_lio::Pose3 prev = pg_correct;
  pg_correct = fresh;
  pg_correct_valid = true;

  const Eigen::Vector3d jump =
    (pg_correct.R * pos + pg_correct.t) - (prev.R * pos + prev.t);
  const Eigen::Matrix3d dR = prev.R.transpose() * pg_correct.R;
  const double d_yaw_deg = Eigen::AngleAxisd(dR).angle() * 180.0 / M_PI;
  const double tot_yaw_deg =
    std::atan2(pg_correct.R(1, 0), pg_correct.R(0, 0)) * 180.0 / M_PI;

  const auto li = loop_backend.LastLoop();
  if (li.from < 0) {
    PL_WARN(
      LOGGER, "[GNSS] pose-graph correction %.3f m / %.3f deg  (cumulative T_g_w: %.3f m, yaw %.3f deg)",
      jump.norm(), d_yaw_deg, pg_correct.t.norm(), tot_yaw_deg);
    return;
  }
  PL_WARN(
    LOGGER,
    "[LoopClosure] closure #%d kf%d(t=%.1fs) <-> kf%d(t=%.1fs) dt=%.1fs dist=%.2fm "
    "fitness=%.4f correction %.3fm/%.3fdeg  solve %.1fms/%d iters  (cumulative T_g_w: %.3fm, yaw %.3fdeg)",
    loop_backend.NumLoops(), li.from, li.t_from, li.to, li.t_to,
    li.t_from - li.t_to, li.dist, li.fitness, jump.norm(), d_yaw_deg,
    loop_backend.LastOptMs(), loop_backend.LastOptIters(),
    pg_correct.t.norm(), tot_yaw_deg);
}

void publish_init_map(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  sensor_msgs::msg::PointCloud2 laserCloudmsg;

  pcl::toROSMsg(*init_feats_world, laserCloudmsg);

  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "odom";
  pubLaserCloudFullRes->publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());   // NOLINT
void publish_frame_world(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  if (scan_pub_en) {
    // Raw, in the odom frame, exactly like /odom and /path. Applying the loop
    // correction here but not to the odometry shifted the cloud while the
    // trajectory stayed put, so the two visibly drifted apart. Global consistency
    // is applied once, to the saved map; live topics stay in one frame.
    PointCloudXYZI::Ptr pub_cloud = feats_down_world;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pub_cloud, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom";
    pubLaserCloudFullRes->publish(laserCloudmsg);

    //--------------------------save map-----------------------------------
    // 1. make sure you have enough memories
    // 2. noted that pcd save will influence the real-time performances
    PointCloudXYZI::Ptr save_cloud = feats_down_world;
    if (pcd_save_en && pcd_max_range > 0.0 &&
        feats_down_body->size() == feats_down_world->size())
    {
      save_cloud.reset(new PointCloudXYZI());
      save_cloud->points.reserve(feats_down_world->size());
      const float r2 = static_cast<float>(pcd_max_range * pcd_max_range);
      for (std::size_t i = 0; i < feats_down_world->size(); ++i) {
        const auto & b = feats_down_body->points[i];
        if (b.x * b.x + b.y * b.y + b.z * b.z <= r2) {
          save_cloud->points.push_back(feats_down_world->points[i]);
        }
      }
    }
    if (pcd_save_en) {
      if (pcd_tiled_en) {
        g_tile_map.Add(*save_cloud, lidar_end_time);
      } else {
        *pcl_wait_save += *save_cloud;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (!pcl_wait_save->empty() && pcd_save_interval > 0 &&
            scan_wait_num >= pcd_save_interval) {
          pcd_index++;
          const auto & dir = g_pcd_dir;
          if (!dir.empty()) {
            save_pcd(dir / ("scans_" + to_string(pcd_index) + ".pcd"), *pcl_wait_save);
          }
          pcl_wait_save->clear();
          scan_wait_num = 0;
        }

        constexpr size_t kMaxWaitSavePts = 5000000;
        if (pcl_wait_save->size() > kMaxWaitSavePts) {
          const size_t before = pcl_wait_save->size();
          PointCloudXYZI::Ptr tmp(new PointCloudXYZI());
          downSizeFilterMap.setInputCloud(pcl_wait_save);
          downSizeFilterMap.filter(*tmp);
          pcl_wait_save.swap(tmp);
          PL_WARN(
            LOGGER, "pcd buffer exceeded %zu points, voxel-downsampled %zu -> %zu at %.2f m",
            kMaxWaitSavePts, before, pcl_wait_save->size(), filter_size_map_min);
        }
      }
    }
  }
}
void publish_frame_lidar(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*feats_undistort, msg);
  msg.header.stamp = get_ros_time(lidar_end_time);
  msg.header.frame_id = "lidar";
  pub->publish(msg);
}

void publish_frame_body(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
{
  int size = feats_undistort->points.size();
  PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    pointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
  }

  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "body";
  pubLaserCloudFull_body->publish(laserCloudmsg);
}

template <typename T>
void set_posestamp(T & out)
{
  auto set_output_from_kf = [&](const auto & kf) {
    Eigen::Vector3d p = kf.x_.pos;
    Eigen::Matrix3d R = kf.x_.rot;
    out.position.x = p(0);
    out.position.y = p(1);
    out.position.z = p(2);
    Eigen::Quaterniond q(R);
    q.normalize();
    out.orientation.x = q.coeffs()[0];
    out.orientation.y = q.coeffs()[1];
    out.orientation.z = q.coeffs()[2];
    out.orientation.w = q.coeffs()[3];
  };

  if (!use_imu_as_input) {
    set_output_from_kf(kf_output);
  } else {
    set_output_from_kf(kf_input);
  }
}

static void write_replay_row(double stamp)
{
  if (!replay_odom_csv.is_open()) return;
  const auto & pp = odomImu.pose.pose.position;
  const auto & qq = odomImu.pose.pose.orientation;
  const auto & vv = odomImu.twist.twist.linear;

  const auto & raw = kf_output.x_.pos;
  replay_odom_csv << std::fixed << std::setprecision(9) << stamp << ','
                  << std::setprecision(6) << pp.x << ',' << pp.y << ',' << pp.z << ','
                  << std::setprecision(9) << qq.x << ',' << qq.y << ',' << qq.z << ',' << qq.w
                  << ',' << std::setprecision(6) << vv.x << ',' << vv.y << ',' << vv.z
                  << ',' << raw(0) << ',' << raw(1) << ',' << raw(2) << '\n';
}

void publish_odometry(
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pub,
  std::shared_ptr<tf2_ros::TransformBroadcaster> & tf_br,
  std::shared_ptr<tf2_ros::Buffer> & tf_buf)
{
  geometry_msgs::msg::Pose local_pose;
  set_posestamp(local_pose);

  {
    imu_odom_snap.t_stamp = publish_odometry_without_downsample
                            ? time_current : lidar_end_time;
    if (use_imu_as_input) {
      imu_odom_snap.p  = kf_input.x_.pos;
      imu_odom_snap.q  = Eigen::Quaterniond(kf_input.x_.rot);
      imu_odom_snap.v  = kf_input.x_.vel;
      imu_odom_snap.ba = kf_input.x_.ba;
      imu_odom_snap.bg = kf_input.x_.bg;
      imu_odom_snap.g  = kf_input.x_.gravity;
    } else {
      imu_odom_snap.p  = kf_output.x_.pos;
      imu_odom_snap.q  = Eigen::Quaterniond(kf_output.x_.rot);
      imu_odom_snap.v  = kf_output.x_.vel;
      imu_odom_snap.ba = kf_output.x_.ba;
      imu_odom_snap.bg = kf_output.x_.bg;
      imu_odom_snap.g  = kf_output.x_.gravity;
    }
    imu_odom_snap.valid = true;


    imu_odom_snap.has_last_imu = false;
    std::lock_guard<std::mutex> lk(imu_pub_mutex);
    while (!imu_pub_deque.empty() &&
           get_time_sec(imu_pub_deque.front()->header.stamp) <= imu_odom_snap.t_stamp) {
      const auto & m = imu_pub_deque.front();
      imu_odom_snap.last_acc = Eigen::Vector3d(
        m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
      imu_odom_snap.last_gyr = Eigen::Vector3d(
        m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
      imu_odom_snap.has_last_imu = true;
      imu_pub_deque.pop_front();
    }
  }

  if (tf_send_en) {
    {
      geometry_msgs::msg::TransformStamped bl;
      bl.header.frame_id = "body";
      bl.child_frame_id = "lidar";
      bl.transform.translation.x = Lidar_T_wrt_IMU.x();
      bl.transform.translation.y = Lidar_T_wrt_IMU.y();
      bl.transform.translation.z = Lidar_T_wrt_IMU.z();
      const Eigen::Quaterniond q{Eigen::Matrix3d(Lidar_R_wrt_IMU)};
      bl.transform.rotation.w = q.w(); bl.transform.rotation.x = q.x();
      bl.transform.rotation.y = q.y(); bl.transform.rotation.z = q.z();
      bl.header.stamp = get_ros_time(imu_odom_snap.t_stamp);
      tf_br->sendTransform(bl);
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "body";
    transform.transform.translation.x = local_pose.position.x;
    transform.transform.translation.y = local_pose.position.y;
    transform.transform.translation.z = local_pose.position.z;
    transform.transform.rotation.w = local_pose.orientation.w;
    transform.transform.rotation.x = local_pose.orientation.x;
    transform.transform.rotation.y = local_pose.orientation.y;
    transform.transform.rotation.z = local_pose.orientation.z;
    transform.header.stamp = get_ros_time(imu_odom_snap.t_stamp);
    tf_br->sendTransform(transform);
  }

  if (imu_propagate_en) {
    return;
  }

  odomImu.header.frame_id = "map";
  odomImu.child_frame_id  = "body";
  odomImu.header.stamp    = get_ros_time(imu_odom_snap.t_stamp);
  odomImu.pose.pose       = local_pose;


  const Eigen::Vector3d v_b = imu_odom_snap.q.conjugate() * imu_odom_snap.v;
  odomImu.twist.twist.linear.x = v_b.x();
  odomImu.twist.twist.linear.y = v_b.y();
  odomImu.twist.twist.linear.z = v_b.z();

  Eigen::Vector3d w_b = Eigen::Vector3d::Zero();
  if (imu_odom_snap.has_last_imu) {
    w_b = imu_odom_snap.last_gyr - imu_odom_snap.bg;
  }
  odomImu.twist.twist.angular.x = w_b.x();
  odomImu.twist.twist.angular.y = w_b.y();
  odomImu.twist.twist.angular.z = w_b.z();

  if (refresh_map_to_odom(tf_buf)) {
    const Eigen::Vector3d & t_mc = g_t_m2o;
    const Eigen::Quaterniond & q_mc = g_q_m2o;
    Eigen::Vector3d p_o(
      local_pose.position.x, local_pose.position.y, local_pose.position.z);
    Eigen::Quaterniond q_o(
      local_pose.orientation.w, local_pose.orientation.x,
      local_pose.orientation.y, local_pose.orientation.z);

    Eigen::Vector3d p_mb = q_mc * p_o + t_mc;
    Eigen::Quaterniond q_mb = (q_mc * q_o).normalized();
    odomImu.pose.pose.position.x = p_mb.x();
    odomImu.pose.pose.position.y = p_mb.y();
    odomImu.pose.pose.position.z = p_mb.z();
    odomImu.pose.pose.orientation.w = q_mb.w();
    odomImu.pose.pose.orientation.x = q_mb.x();
    odomImu.pose.pose.orientation.y = q_mb.y();
    odomImu.pose.pose.orientation.z = q_mb.z();
  } else {
    // Never resolved yet: the pose is still in the odom frame, so label it
    // honestly rather than claiming map.
    odomImu.header.frame_id = "odom";
  }

  if (imu_odom_snap.t_stamp <= last_pub_odom_t) return;
  pub->publish(odomImu);
  last_pub_odom_t = imu_odom_snap.t_stamp;
  write_replay_row(imu_odom_snap.t_stamp);
}


struct FrameTimeStats
{
  std::vector<double> samples;
  std::size_t next = 0;
  double max_ms = 0.0;
  double max_ever_ms = 0.0;
  std::uint64_t count = 0;
  double last_report_t = 0.0;
  double eff_ratio_sum = 0.0;
  double eff_ratio_min = 1.0;
  std::uint64_t eff_n = 0;
  struct Breakdown
  {
    double down = 0.0;
    double ekf = 0.0;
    double map = 0.0;
    double loop = 0.0;
    double sc = 0.0;
    double total = 0.0;
  };
  Breakdown worst;

  void add(double ms, const Breakdown & bd)
  {
    if (samples.size() < 2000) {
      samples.push_back(ms);
    } else {
      samples[next] = ms;
      next = (next + 1) % samples.size();
    }
    if (ms > max_ms) {max_ms = ms; worst = bd;}
    if (ms > max_ever_ms) {max_ever_ms = ms;}
    ++count;
  }

  double p99()
  {
    if (samples.empty()) {return 0.0;}
    std::vector<double> tmp(samples);
    const std::size_t k = static_cast<std::size_t>(tmp.size() * 0.99);
    std::nth_element(tmp.begin(), tmp.begin() + k, tmp.end());
    return tmp[std::min(k, tmp.size() - 1)];
  }

  double mean() const
  {
    if (samples.empty()) {return 0.0;}
    double s = 0.0;
    for (double v : samples) {s += v;}
    return s / samples.size();
  }
};
static FrameTimeStats g_frame_stats;

static int g_imu_spike_streak = 0;

static bool propagate_one_imu(
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pub,
  std::shared_ptr<tf2_ros::Buffer> & tf_buf,
  const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg)
{
  const double t_imu = get_time_sec(imu_msg->header.stamp);
  const double dt = t_imu - imu_odom_snap.t_stamp;
  if (dt <= 0.0) return false;
  if (dt > 0.5) return false;  

  const Eigen::Vector3d a_imu(
    imu_msg->linear_acceleration.x,
    imu_msg->linear_acceleration.y,
    imu_msg->linear_acceleration.z);
  const Eigen::Vector3d w_imu(
    imu_msg->angular_velocity.x,
    imu_msg->angular_velocity.y,
    imu_msg->angular_velocity.z);

  const Eigen::Vector3d a_raw =
    imu_odom_snap.has_last_imu ? 0.5 * (imu_odom_snap.last_acc + a_imu) : a_imu;
  const Eigen::Vector3d w_raw =
    imu_odom_snap.has_last_imu ? 0.5 * (imu_odom_snap.last_gyr + w_imu) : w_imu;

  const Eigen::Vector3d a_b = a_raw * G_m_s2 / acc_norm;
  const Eigen::Vector3d w_b = w_raw - imu_odom_snap.bg;

  const Eigen::Vector3d a_w = imu_odom_snap.q * (a_b - imu_odom_snap.ba) + imu_odom_snap.g;

  const Eigen::Vector3d p_new =
    imu_odom_snap.p + imu_odom_snap.v * dt + 0.5 * a_w * dt * dt;
  const Eigen::Vector3d v_new = imu_odom_snap.v + a_w * dt;

  const double theta = w_b.norm() * dt;
  Eigen::Quaterniond dq;
  if (theta > 1e-9) {
    dq = Eigen::AngleAxisd(theta, w_b.normalized());
  } else {
    dq.setIdentity();
  }
  const Eigen::Quaterniond q_new = (imu_odom_snap.q * dq).normalized();

  if (!p_new.allFinite() || !v_new.allFinite() || !q_new.coeffs().allFinite() ||
      v_new.norm() > odom_max_vel) {
    static double last_spike_log_t = 0.0;
    ++g_imu_spike_streak;
    if (omp_get_wtime() - last_spike_log_t > 2.0) {
      last_spike_log_t = omp_get_wtime();
      PL_WARN(
        LOGGER, "IMU propagation spike (v=%.1f m/s > %.1f); dropping this /odom (%d in a row)",
        v_new.allFinite() ? v_new.norm() : -1.0, odom_max_vel, g_imu_spike_streak);
    }
    constexpr int kSpikeAlarm = 100;   // ~0.5 s at 200 Hz
    if (g_imu_spike_streak == kSpikeAlarm) {
      PL_ERROR(
        LOGGER,
        "dropped %d consecutive /odom from IMU propagation - odometry has effectively stopped. "
        "Check that the IMU rate matches imu_time_inte, or whether the body is stuck.",
        g_imu_spike_streak);
    }
    return false;
  }

  g_imu_spike_streak = 0;

  imu_odom_snap.p = p_new;
  imu_odom_snap.v = v_new;
  imu_odom_snap.q = q_new;
  imu_odom_snap.t_stamp = t_imu;
  imu_odom_snap.last_acc = a_imu;
  imu_odom_snap.last_gyr = w_imu;
  imu_odom_snap.has_last_imu = true;

  if (t_imu <= last_pub_odom_t) return true;

  // Loop correction stays out of odom: pg_correct is a time-varying rigid
  // transform updated on every closure, so applying it steps odom at each one
  // (corridor: per-frame jump 3.3 cm -> 23.3 cm). The control loop needs
  // continuity; global consistency is applied once before the map is saved.
  const Eigen::Vector3d & p_out = p_new;
  const Eigen::Quaterniond & q_out = q_new;

  odomImu.header.frame_id = "map";
  odomImu.child_frame_id  = "body";
  odomImu.header.stamp    = get_ros_time(t_imu);
  odomImu.pose.pose.position.x = p_out.x();
  odomImu.pose.pose.position.y = p_out.y();
  odomImu.pose.pose.position.z = p_out.z();
  odomImu.pose.pose.orientation.w = q_out.w();
  odomImu.pose.pose.orientation.x = q_out.x();
  odomImu.pose.pose.orientation.y = q_out.y();
  odomImu.pose.pose.orientation.z = q_out.z();

  const Eigen::Vector3d v_b = q_new.conjugate() * v_new;
  odomImu.twist.twist.linear.x = v_b.x();
  odomImu.twist.twist.linear.y = v_b.y();
  odomImu.twist.twist.linear.z = v_b.z();
  odomImu.twist.twist.angular.x = w_b.x();
  odomImu.twist.twist.angular.y = w_b.y();
  odomImu.twist.twist.angular.z = w_b.z();

  if (refresh_map_to_odom(tf_buf)) {
    const Eigen::Vector3d & t_mc = g_t_m2o;
    const Eigen::Quaterniond & q_mc = g_q_m2o;
    Eigen::Vector3d p_mb = q_mc * p_out + t_mc;
    Eigen::Quaterniond q_mb = (q_mc * q_out).normalized();
    odomImu.pose.pose.position.x = p_mb.x();
    odomImu.pose.pose.position.y = p_mb.y();
    odomImu.pose.pose.position.z = p_mb.z();
    odomImu.pose.pose.orientation.w = q_mb.w();
    odomImu.pose.pose.orientation.x = q_mb.x();
    odomImu.pose.pose.orientation.y = q_mb.y();
    odomImu.pose.pose.orientation.z = q_mb.z();
  } else {
    odomImu.header.frame_id = "odom";
  }

  pub->publish(odomImu);
  last_pub_odom_t = t_imu;
  write_replay_row(t_imu);
  return true;
}

void publish_odom_imu(
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pub,
  std::shared_ptr<tf2_ros::Buffer> & tf_buf)
{
  if (!imu_odom_snap.valid) return;
  // Copy under the lock, publish outside it: the subscriber thread must not
  // block on ROS publishing, and the deque must not shift mid-iteration.
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> batch;
  {
    std::lock_guard<std::mutex> lk(imu_pub_mutex);
    batch.assign(imu_pub_deque.begin(), imu_pub_deque.end());
  }
  for (const auto & imu_msg : batch) {
    if (get_time_sec(imu_msg->header.stamp) <= imu_odom_snap.t_stamp) continue;
    propagate_one_imu(pub, tf_buf, imu_msg);
  }
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
  msg_body_pose.header.frame_id = "odom";
  static int jjj = 0;
  jjj++;
  {
    path.poses.emplace_back(msg_body_pose);

    constexpr size_t kMaxPathPoses = 2000;
    if (path.poses.size() > kMaxPathPoses) {
      path.poses.erase(
        path.poses.begin(),
        path.poses.begin() + static_cast<long>(path.poses.size() - kMaxPathPoses));
    }

    pubPath->publish(path);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("laserMapping");

  const auto log_base = nh->declare_parameter<std::string>("log.dir", "/var/log/slam");
  const auto log_dir = slam_log::init(log_base, argv[0]);
  PL_INFO(nh->get_logger(), "log dir: %s", log_dir.c_str());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh);

  readParameters(nh);
  std::cout << "lidar_type: " << lidar_type << '\n';

  sc_save_path = nh->declare_parameter<std::string>("scan_context.save_path", "");
  sc_enable = nh->declare_parameter<bool>("scan_context.enable", true) &&
              (pcd_save_en || !sc_save_path.empty());
  sc_stride = nh->declare_parameter<double>("scan_context.stride", 2.0);
  sc_dedup_dist = nh->declare_parameter<double>("scan_context.dedup_dist", 1.5);
  sc_db.cfg.max_radius = nh->declare_parameter<double>("scan_context.max_radius", 60.0);
  sc_db.cfg.num_ring = nh->declare_parameter<int>("scan_context.num_ring", 20);
  sc_db.cfg.num_sector = nh->declare_parameter<int>("scan_context.num_sector", 60);
  // Height cap on descriptor points, off by default. Must equal the
  // relocalization node's sc_max_height: cells store max height, so clipping one
  // side only compares different quantities. The .sc file does not carry the
  // value, so the two sides agree through config alone. Needed under ceilings or
  // in shafts, where every cell reaches the same height and positions stop being
  // distinguishable; the relocalization node detects that and suggests a value.
  sc_db.cfg.max_height = nh->declare_parameter<double>("scan_context.max_height", 1e9);

  if (pcd_save_en) {
    g_pcd_dir = prepare_pcd_dir();
    if (g_pcd_dir.empty()) {
      PL_ERROR(
        LOGGER, "cloud saving is on but the output dir is unusable; nothing will be saved. Check pcd_save.output_dir");
    } else if (pcd_tiled_en) {
      point_lio::TileMapWriter<PointType>::Config tc;
      tc.tile_size = pcd_tile_size;
      tc.flush_idle_s = pcd_tile_flush_idle_s;
      tc.max_buffer_pts = static_cast<std::size_t>(std::max(1, pcd_tile_max_buffer_pts));
      tc.voxel = static_cast<float>(pcd_tile_voxel);
      using Pol = point_lio::TileMapWriter<PointType>::Config::Policy;
      tc.policy = pcd_tile_policy == "first" ? Pol::kFirst
                : pcd_tile_policy == "last"  ? Pol::kLast
                                             : Pol::kCentroid;
      g_tile_map.Init(
        g_pcd_dir, tc,
        [](const std::filesystem::path & f,
           const pcl::PointCloud<pcl::PointXYZI> & c) {
          pcl::io::savePCDFileBinary(f.string(), c);
        });
      g_tile_map.SetStampAsIntensity(
        nh->declare_parameter<bool>("pcd_save.stamp_as_intensity", false));
      PL_INFO(
        LOGGER,
        "incremental cloud dump: %s/tiles/  tile %.0f m, finalized after %.0f s idle, resident cap %d points (~%.0f MB)",
        g_pcd_dir.c_str(), tc.tile_size, tc.flush_idle_s, pcd_tile_max_buffer_pts,
        pcd_tile_max_buffer_pts * sizeof(PointType) / 1048576.0);
    } else {
      PL_INFO(
        LOGGER, "cloud will be saved to: %s/%s (single-file mode keeps everything in memory; enable pcd_save.tiled for long runs)",
        g_pcd_dir.c_str(), pcd_save_name.empty() ? "map.pcd" : pcd_save_name.c_str());
    }
  }
  ivox_ = std::make_shared<IVoxType>(ivox_options_);

  {
    point_lio::LoopClosureConfig lc;

    lc.enable = loop_en || gnss_en;
    lc.detect_loops = loop_en;
    lc.keyframe_add_dist = nh->declare_parameter<double>("loop_closure.keyframe_add_dist", 1.0);
    lc.keyframe_add_angle = nh->declare_parameter<double>("loop_closure.keyframe_add_angle", 0.2);
    lc.max_keyframes_per_cell =
      nh->declare_parameter<int>("loop_closure.max_keyframes_per_cell", 0);
    lc.revisit_cell_size =
      nh->declare_parameter<double>("loop_closure.revisit_cell_size", 2.0);
    lc.search_radius = nh->declare_parameter<double>("loop_closure.search_radius", 10.0);
    lc.search_radius_z = nh->declare_parameter<double>("loop_closure.search_radius_z", 2.0);
    lc.search_time_diff = nh->declare_parameter<double>("loop_closure.search_time_diff", 30.0);
    // Required for offline A/B: async detection makes runs irreproducible. Keep
    // false on hardware.
    lc.sync_detect = nh->declare_parameter<bool>("loop_closure.sync_detect", false);
    lc.search_num = nh->declare_parameter<int>("loop_closure.search_num", 25);
    lc.fitness_score = nh->declare_parameter<double>("loop_closure.fitness_score", 0.3);
    lc.icp_leaf_size = nh->declare_parameter<double>("loop_closure.icp_leaf_size", 0.4);
    lc.icp_max_corr_dist =
      nh->declare_parameter<double>("loop_closure.icp_max_corr_dist", 10.0);
    lc.cur_search_num = nh->declare_parameter<int>("loop_closure.cur_search_num", 0);
    lc.max_correction = nh->declare_parameter<double>("loop_closure.max_correction", 0.0);
    lc.max_correction_rot =
      nh->declare_parameter<double>("loop_closure.max_correction_rot", 0.1);
    lc.min_travel_between =
      nh->declare_parameter<double>("loop_closure.min_travel_between", 30.0);
    lc.loop_sigma_floor = nh->declare_parameter<double>("loop_closure.loop_sigma_floor", 0.0);
    lc.prior_sigma = nh->declare_parameter<double>("loop_closure.prior_sigma", 1e-6);
    lc.odom_sigma_rot = nh->declare_parameter<double>("loop_closure.odom_sigma_rot", 0.05);
    lc.odom_sigma_trans = nh->declare_parameter<double>("loop_closure.odom_sigma_trans", 0.1);
    lc.keep_factors = nh->declare_parameter<bool>("loop_closure.keep_factors", true);
    lc.max_window = nh->declare_parameter<int>("loop_closure.max_window", 500);
    lc.opt_iters = nh->declare_parameter<int>("loop_closure.opt_iters", 6);
    lc.opt_iters_on_loop = nh->declare_parameter<int>("loop_closure.opt_iters_on_loop", 10);
    lc.gnss_opt_every = nh->declare_parameter<int>("loop_closure.gnss_opt_every", 10);
    lc.max_loops_per_cell_pair =
      nh->declare_parameter<int>("loop_closure.max_loops_per_cell_pair", 3);
    lc.radius_num_candidates =
      nh->declare_parameter<int>("loop_closure.radius_num_candidates", 1);
    lc.radius_candidate_sep =
      nh->declare_parameter<double>("loop_closure.radius_candidate_sep", 3.0);
    lc.backend_interval_ms =
      nh->declare_parameter<int>("loop_closure.backend_interval_ms", 50);
    lc.max_pending = nh->declare_parameter<int>("loop_closure.max_pending", 64);
    lc.sc_detect = nh->declare_parameter<bool>("loop_closure.sc_detect", false);
    lc.sc_dist_thresh =
      nh->declare_parameter<double>("loop_closure.sc_dist_thresh", 0.3);
    lc.sc_ratio = nh->declare_parameter<double>("loop_closure.sc_ratio", 1.0);
    lc.sc_tree_period =
      nh->declare_parameter<int>("loop_closure.sc_tree_period", 30);
    lc.sc_num_candidates =
      nh->declare_parameter<int>("loop_closure.sc_num_candidates", 3);
    lc.sc_exclude_recent =
      nh->declare_parameter<int>("loop_closure.sc_exclude_recent", 30);
    lc.sc_accum_frames =
      nh->declare_parameter<int>("loop_closure.sc_accum_frames", 0);
    lc.sc_use_yaw_guess =
      nh->declare_parameter<bool>("loop_closure.sc_use_yaw_guess", true);
    export_patches =
      nh->declare_parameter<bool>("pcd_save.export_patches", false);
    lc.keep_dense = export_patches;
    // The descriptor geometry is shared with the relocalization database so a
    // map and the loops built into it stay comparable.
    lc.sc_max_radius = sc_db.cfg.max_radius;
    lc.sc_num_ring = sc_db.cfg.num_ring;
    lc.sc_num_sector = sc_db.cfg.num_sector;
    lc.sc_max_height = sc_db.cfg.max_height;
    loop_backend.Init(lc);
    loop_backend.Start();

    point_lio::GnssConfig gc;
    gc.enable = gnss_en;
    gc.use_elevation = nh->declare_parameter<bool>("gnss.use_elevation", false);
    gc.cov_threshold = nh->declare_parameter<double>("gnss.cov_threshold", 2.0);
    gc.min_travel = nh->declare_parameter<double>("gnss.min_travel", 1.0);
    gc.time_tol = nh->declare_parameter<double>("gnss.time_tol", 0.05);
    const auto gt = nh->declare_parameter<std::vector<double>>(
      "gnss.extrinsic_T", std::vector<double>{0.0, 0.0, 0.0});
    const auto gr = nh->declare_parameter<std::vector<double>>(
      "gnss.extrinsic_R", std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
    if (gt.size() == 3) gc.extrin_t = Eigen::Vector3d(gt[0], gt[1], gt[2]);
    if (gr.size() == 9) gc.extrin_r = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(gr.data());
    gc.sigma_floor = nh->declare_parameter<double>("gnss.sigma_floor", 0.05);
    gc.sigma_scale = nh->declare_parameter<double>("gnss.sigma_scale", 1.0);
    gc.sigma_max = nh->declare_parameter<double>("gnss.sigma_max", 1e3);
    gnss_proc.Init(gc);
    if (gnss_en) {
      PL_INFO(
        LOGGER, "[GNSS] enabled: unary pose-graph factor, elevation=%d, sigma in [%.2f, %.2f]m, gate=%.1f",
        int(gc.use_elevation), gc.sigma_floor, gc.sigma_max, gc.cov_threshold);
    }
    if (loop_en) {
      PL_INFO(
        LOGGER, "[LoopClosure] enabled: keyframe spacing %.1fm/%.2frad, search radius %.1fm (vertical %.1fm), "
        "min dt %.0fs, fitness<%.2f, keep_factors=%d, GNSS=%d",
        lc.keyframe_add_dist, lc.keyframe_add_angle, lc.search_radius, lc.search_radius_z,
        lc.search_time_diff, lc.fitness_score, int(lc.keep_factors), int(gnss_en));
    }
  }

  path.header.stamp = get_ros_time(lidar_end_time);
  path.header.frame_id = "odom";

  /*** variables definition for counting ***/
  int frame_num = 0;
  double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_solve = 0,
         aver_time_propag = 0;

  memset(point_selected_surf, true, sizeof(point_selected_surf));
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    } else {
      kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    }
  }

  p_imu->lidar_type = p_pre->lidar_type = lidar_type;
  p_imu->imu_en = imu_en;

  zc_configure(
    zc_enable_param, zc_nz2_thresh_param, zc_weight_param, zc_max_hold_s_param,
    zc_max_tilt_deg_param);
  kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
  kf_output.init_dyn_share_modified_3h(
    get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
  Eigen::Matrix<double, 24, 24> P_init;  // = MD(18, 18)::Identity() * 0.1;
  reset_cov(P_init);
  kf_input.change_P(P_init);
  Eigen::Matrix<double, 30, 30> P_init_output;  // = MD(24, 24)::Identity() * 0.01;
  reset_cov_output(P_init_output);
  kf_output.change_P(P_init_output);
  Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
  Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();
  /*** debug record ***/
  FILE * fp;
  string pos_log_dir = log_dir + "/pos_log.txt";
  fp = fopen(pos_log_dir.c_str(), "w");
  if (fp == nullptr) {
    PL_WARN(
      LOGGER, "cannot open '%s' (%s); pos log disabled, console timing still works",
      pos_log_dir.c_str(), strerror(errno));
  }
  open_file(log_dir);

  /*** ROS subscribe initialization ***/
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;
  if (p_pre->lidar_type == AVIA) {
    sub_pcl_livox = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lid_topic, rclcpp::SensorDataQoS(),
      [](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { livox_pcl_cbk(msg); });
  } else {
    sub_pcl_pc = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      lid_topic, rclcpp::SensorDataQoS(),
      [](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { standard_pcl_cbk(msg); });
  }
  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(200))
                 .best_effort()
                 .durability_volatile();
  auto sub_imu =
    nh->create_subscription<sensor_msgs::msg::Imu>(imu_topic, imu_qos, imu_cbk);
  auto pub_laser_cloud_full_res =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", 20);
  auto pub_laser_cloud_full_res_body =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_body", 20);
  auto pub_laser_cloud_full_res_lidar =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_lidar", 20);
  auto pub_laser_cloud_effect =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_effected", 20);
  auto pub_laser_cloud_map = nh->create_publisher<sensor_msgs::msg::PointCloud2>("Laser_map", 20);
  auto pub_odom_aft_mapped =
    nh->create_publisher<nav_msgs::msg::Odometry>("lio/odom", 100);
  auto pub_path = nh->create_publisher<nav_msgs::msg::Path>("path", 20);
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss;
  const auto gnss_topic = nh->declare_parameter<std::string>("gnss.topic", "/gnss");
  if (gnss_en) {
    sub_gnss = nh->create_subscription<sensor_msgs::msg::NavSatFix>(
      gnss_topic, rclcpp::SensorDataQoS(),
      [](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        gnss_proc.Add(
          get_time_sec(msg->header.stamp), msg->latitude, msg->longitude, msg->altitude,
          Eigen::Vector3d(
            msg->position_covariance[0], msg->position_covariance[4],
            msg->position_covariance[8]),
          msg->status.status);
      });
    PL_INFO(LOGGER, "[GNSS] subscribing %s", gnss_topic.c_str());
  }
  // Map lifecycle over services: scan with the node running, call SaveMaps when
  // the area is covered, then point the relocalization node at the result. The
  // Empty topic below predates these and is kept because tooling uses it.
  auto save_maps_srv = nh->create_service<interface::srv::SaveMaps>(
    nh->declare_parameter<std::string>("services.save_maps", "~/save_maps"),
    [](const std::shared_ptr<interface::srv::SaveMaps::Request> req,
    std::shared_ptr<interface::srv::SaveMaps::Response> res) {
      const SaveReport r = save_maps_to(req->file_path, req->save_patches);
      res->success = r.ok;
      res->message = r.message;
      if (r.ok) {
        PL_INFO(LOGGER, "SaveMaps: %s", r.message.c_str());
      } else {
        PL_ERROR(LOGGER, "SaveMaps failed: %s", r.message.c_str());
      }
    });

  auto save_map_sub = nh->create_subscription<std_msgs::msg::Empty>(
    "/save_map", 10,
    [](const std_msgs::msg::Empty::SharedPtr) { save_scan_context(g_pcd_dir); });

  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(nh->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);


  const std::string replay_bag = nh->declare_parameter<std::string>("replay.bag", "");
  const std::string replay_csv = nh->declare_parameter<std::string>("replay.odom_csv", "");
  g_replay_kf_csv = nh->declare_parameter<std::string>("replay.keyframe_csv", "");
  if (!replay_csv.empty()) {
    replay_odom_csv.open(replay_csv);
    replay_odom_csv << "t,x,y,z,qx,qy,qz,qw,vx,vy,vz,rx,ry,rz\n";
    PL_INFO(LOGGER, "[Replay] odometry dump: %s", replay_csv.c_str());
  }
  std::unique_ptr<rosbag2_cpp::Reader> replay_reader;
  int replay_idle_rounds = 0;
  bool replay_done = false;
  // Replay matches topic names exactly while configs are inconsistent about the
  // leading slash (mid360.yaml has "livox/lidar", the bag has "/livox/lidar").
  // Without normalisation one wrong character means zero frames processed and
  // exit code 0 — two hardware bags were replayed for nothing before this showed.
  auto norm_topic = [](const std::string & t) {
    return (t.empty() || t.front() == '/') ? t : "/" + t;
  };
  const std::string lid_topic_n = norm_topic(lid_topic);
  const std::string imu_topic_n = norm_topic(imu_topic);
  const std::string gnss_topic_n = norm_topic(gnss_topic);
  std::size_t replay_lidar_frames = 0;
  std::map<std::string, std::size_t> replay_seen;
  if (!replay_bag.empty()) {
    replay_reader = std::make_unique<rosbag2_cpp::Reader>();
    replay_reader->open(replay_bag);
    PL_INFO(
      LOGGER, "[Replay] deterministic offline replay: %s (lidar %s, IMU %s)",
      replay_bag.c_str(), lid_topic_n.c_str(), imu_en ? imu_topic_n.c_str() : "off");
  }
  rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> ser_livox;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser_pc2;
  rclcpp::Serialization<sensor_msgs::msg::Imu> ser_imu;
  rclcpp::Serialization<sensor_msgs::msg::NavSatFix> ser_gnss;

  auto feed_one = [&]() -> bool {
    while (replay_reader->has_next()) {
      auto bm = replay_reader->read_next();
      rclcpp::SerializedMessage sm(*bm->serialized_data);
      replay_seen[bm->topic_name] += 1;
      const std::string topic_n = norm_topic(bm->topic_name);
      if (topic_n == lid_topic_n) {
        ++replay_lidar_frames;
        if (lidar_type == AVIA) {
          auto m = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
          ser_livox.deserialize_message(&sm, m.get());
          livox_pcl_cbk(m);
        } else {
          auto m = std::make_shared<sensor_msgs::msg::PointCloud2>();
          ser_pc2.deserialize_message(&sm, m.get());
          standard_pcl_cbk(m);
        }
        return true;                     
      } else if (topic_n == imu_topic_n) {
        auto m = std::make_shared<sensor_msgs::msg::Imu>();
        ser_imu.deserialize_message(&sm, m.get());
        imu_cbk(m);
      } else if (gnss_en && topic_n == gnss_topic_n) {
        auto m = std::make_shared<sensor_msgs::msg::NavSatFix>();
        ser_gnss.deserialize_message(&sm, m.get());
        gnss_proc.Add(
          get_time_sec(m->header.stamp), m->latitude, m->longitude, m->altitude,
          Eigen::Vector3d(
            m->position_covariance[0], m->position_covariance[4], m->position_covariance[8]),
          m->status.status);
      }
    }
    return false;
  };

  signal(SIGINT, SigHandle);
  rclcpp::Rate rate(500);
  while (rclcpp::ok()) {
    if (flg_exit) break;
    if (replay_reader) {
      if (!feed_one() && !replay_done) {
        replay_done = true;
        if (replay_lidar_frames == 0) {
          std::string avail;
          for (const auto & kv : replay_seen) {
            avail += "  " + kv.first + " x" + std::to_string(kv.second) + "\n";
          }
          PL_ERROR(
            LOGGER,
            "replay read the whole bag but matched zero frames on lidar topic %s; this run produced nothing. "
            "Topics actually present:\n%sCheck common.lid_topic.",
            lid_topic_n.c_str(), avail.c_str());
        }
      }
    } else {
      executor.spin_some();
    }

    if (imu_propagate_en) {
      publish_odom_imu(pub_odom_aft_mapped, tf_buffer);
    }
    if (sync_packages(Measures)) {
      replay_idle_rounds = 0;
      if (flg_reset) {
        PL_WARN(LOGGER, "reset when rosbag play back");
        p_imu->Reset();
        feats_undistort.reset(new PointCloudXYZI());
        if (use_imu_as_input) {
          state_in = state_input();
          kf_input.change_P(P_init);
        } else {
          state_out = state_output();
          kf_output.change_P(P_init_output);
        }
        flg_first_scan = true;
        is_first_frame = true;
        flg_reset = false;
        init_map = false;

        {
          ivox_.reset(new IVoxType(ivox_options_));
        }
      }

      if (flg_first_scan) {
        first_lidar_time = Measures.lidar_beg_time;
        flg_first_scan = false;
        if (first_imu_time < 1) {
          first_imu_time = get_time_sec(imu_next.header.stamp);
          printf("first imu time: %f\n", first_imu_time);
        }
        time_current = 0.0;
        if (imu_en) {
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);

          {
            while (Measures.lidar_beg_time >
                   get_time_sec(imu_next.header.stamp))  // if it is needed for the new map?
            {
              imu_deque.pop_front();
              if (imu_deque.empty()) {
                break;
              }
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
            }
          }
        } else {
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);   // _init);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);  //_init);
          kf_output.x_.acc << VEC_FROM_ARRAY(gravity);      //_init);
          kf_output.x_.acc *= -1;
          p_imu->imu_need_init_ = false;
        }
        G_m_s2 =
          std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
      }

      double t0, t1, t3, t5, solve_start;
      match_time = 0;
      solve_time = 0;
      propag_time = 0;
      update_time = 0;
      t0 = omp_get_wtime();

      /*** downsample the feature points in a scan ***/
      t1 = omp_get_wtime();
      p_imu->Process(Measures, feats_undistort);
      if (space_down_sample) {
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      } else {
        feats_down_body = Measures.lidar;
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      }
      {
        time_seq = time_compressing<int>(feats_down_body);
        feats_down_size = feats_down_body->points.size();
      }
      const double t_down1 = omp_get_wtime();

      if (!p_imu->after_imu_init_)  // !p_imu->UseLIInit &&
      {
        if (!p_imu->imu_need_init_) {
          V3D tmp_gravity;
          if (imu_en && !imu_init_dynamic) {
            tmp_gravity = -p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
          } else if (imu_en && imu_init_dynamic) {
            // Dynamic start: gravity comes from the configured mounting attitude
            // instead of a still phase, which vehicles never satisfy - on the park
            // data the still test rejected 23197 samples and never triggered. Gyro
            // bias stays 0; estimating it while moving is wrong and the EKF carries
            // it as a state anyway.
            tmp_gravity << VEC_FROM_ARRAY(gravity_init);
            p_imu->ZeroGyroBias();
            PL_WARN(
              LOGGER,
              "dynamic start: gravity taken from configured gravity_init [%.4f %.4f %.4f] (norm %.4f), "
              "no static init; gyro bias set to 0 and estimated online by the EKF. "
              "This must match the actual sensor mounting or the world vertical axis will be tilted.",
              gravity_init[0], gravity_init[1], gravity_init[2], tmp_gravity.norm());
          } else {
            tmp_gravity << VEC_FROM_ARRAY(gravity_init);
            p_imu->after_imu_init_ = true;
          }
          if (imu_en && !imu_init_dynamic && gravity_init.size() >= 3) {
            const V3D gi(gravity_init[0], gravity_init[1], gravity_init[2]);
            if (gi.norm() > 1e-3 && tmp_gravity.norm() > 1e-3) {
              const double ang = std::acos(std::clamp(
                gi.dot(tmp_gravity) / gi.norm() / tmp_gravity.norm(), -1.0, 1.0)) * 180.0 / M_PI;
              if (ang > 5.0) {
                PL_ERROR(
                  LOGGER,
                  "configured gravity_init [%.4f %.4f %.4f] differs from the measured gravity by %.2f deg. "
                  "Unused this run (static init used the measured value), but enabling imu_init_dynamic "
                  "or imu_en=false would build the world frame from this wrong value. Re-measure it against the real mounting.",
                  gravity_init[0], gravity_init[1], gravity_init[2], ang);
              }
            }
          }
          M3D rot_init;
          p_imu->Set_init(tmp_gravity, rot_init);
          if (enable_prior_pcd) {
            double rr = 0.0, pp = 0.0, yy = 0.0;
            if (init_pose.size() >= 6) {
              rr = init_pose[3] * M_PI / 180.0;
              pp = init_pose[4] * M_PI / 180.0;
              yy = init_pose[5] * M_PI / 180.0;
            } else if (init_pose.size() >= 4) {
              yy = init_pose[3] * M_PI / 180.0;
            }
            const M3D r_off =
              (Eigen::AngleAxisd(yy, V3D::UnitZ()) * Eigen::AngleAxisd(pp, V3D::UnitY()) *
               Eigen::AngleAxisd(rr, V3D::UnitX())).toRotationMatrix();
            rot_init = r_off * rot_init;
            if (init_pose.size() >= 3) {
              const V3D p0(init_pose[0], init_pose[1], init_pose[2]);
              kf_input.x_.pos = p0;
              kf_output.x_.pos = p0;
            }
            PL_INFO(
              LOGGER, "[Reloc] prior-map initial pose (%.2f %.2f %.2f) yaw=%.1f deg",
              init_pose.size() >= 3 ? init_pose[0] : 0.0,
              init_pose.size() >= 3 ? init_pose[1] : 0.0,
              init_pose.size() >= 3 ? init_pose[2] : 0.0, yy * 180.0 / M_PI);
          }
          kf_input.x_.rot = rot_init;
          kf_output.x_.rot = rot_init;
          kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
        } else {
          continue;
        }
      }
      /*** initialize the map ***/
      if (!init_map) {
        feats_down_world->resize(feats_undistort->size());
        for (int i = 0; i < feats_undistort->size(); i++) {
          {
            pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
          }
        }
        for (const auto & point : *feats_down_world) {
          init_feats_world->points.emplace_back(point);
        }

        if (init_feats_world->size() >= init_map_size) {
          if (enable_prior_pcd) {
            auto map_cloud = loadPointcloudFromPcd(prior_pcd_map_path);
            ivox_->AddPoints(map_cloud->points);
          } else {
            ivox_->AddPoints(init_feats_world->points);
          }
          publish_init_map(pub_laser_cloud_map);
          init_feats_world.reset(new PointCloudXYZI());
          init_map = true;
        } else {
          init_map = false;
        }
        continue;
      }

      /*** ICP and Kalman filter update ***/
      normvec->resize(feats_down_size);
      feats_down_world->resize(feats_down_size);

      Nearest_Points.resize(feats_down_size);



      /*** iterated state estimation ***/
      crossmat_list.resize(feats_down_size);
      pbody_list.resize(feats_down_size);

      for (size_t i = 0; i < feats_down_body->size(); i++) {
        V3D point_this(
          feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z);
        pbody_list[i] = point_this;
        if (!extrinsic_est_en)
        //     else
        // else
        {
          point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
          M3D point_crossmat;
          point_crossmat << SKEW_SYM_MATRX(point_this);
          crossmat_list[i] = point_crossmat;
        }
      }

      if (!use_imu_as_input) {
        effct_feat_num = 0;
        /**** point by point update ****/
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];

            time_current = point_body.curvature / 1000.0 + pcl_beg_time;

            if (is_first_frame) {
              if (imu_en) {
                while (time_current > get_time_sec(imu_next.header.stamp)) {
                  imu_deque.pop_front();
                  if (imu_deque.empty()) break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
              }
              is_first_frame = false;
              time_update_last = time_current;
              time_predict_last_const = time_current;
            }
            if (imu_en && !imu_deque.empty()) {
              bool last_imu = get_time_sec(imu_next.header.stamp) ==
                              get_time_sec(imu_deque.front()->header.stamp);
              while (get_time_sec(imu_next.header.stamp) < time_predict_last_const &&
                     !imu_deque.empty()) {
                if (!last_imu) {
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                  break;
                } else {
                  imu_deque.pop_front();
                  if (imu_deque.empty()) break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
              }
              bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              while (imu_comes) {
                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;

                /*** covariance update ***/
                double dt = get_time_sec(imu_next.header.stamp) - time_predict_last_const;
                kf_output.predict(dt, Q_output, input_in, true, false);
                time_predict_last_const = get_time_sec(imu_next.header.stamp);  // big problem

                {
                  double dt_cov = get_time_sec(imu_next.header.stamp) - time_update_last;

                  if (dt_cov > 0.0) {
                    time_update_last = get_time_sec(imu_next.header.stamp);
                    double propag_imu_start = omp_get_wtime();

                    kf_output.predict(dt_cov, Q_output, input_in, false, true);

                    propag_time += omp_get_wtime() - propag_imu_start;
                    double solve_imu_start = omp_get_wtime();
                    kf_output.update_iterated_dyn_share_IMU();
                    solve_time += omp_get_wtime() - solve_imu_start;
                  }
                }
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
                imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              }
            }
            if (flg_reset) {
              break;
            }

            double dt = time_current - time_predict_last_const;
            double propag_state_start = omp_get_wtime();
            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_output.predict(dt, Q_output, input_in, true, false);
            propag_time += omp_get_wtime() - propag_state_start;
            time_predict_last_const = time_current;
            double t_update_start = omp_get_wtime();

            if (feats_down_size < 1) {
              PL_WARN(LOGGER, "No point, skip this scan!\n");
              idx += time_seq[k];
              continue;
            }
            if (!kf_output.update_iterated_dyn_share_modified()) {
              idx = idx + time_seq[k];
              continue;
            }
            solve_start = omp_get_wtime();

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_aft_mapped, tf_broadcaster, tf_buffer);
              if (runtime_pos_log) {
                euler_cur = SO3ToEuler(kf_output.x_.rot);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                         << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose()
                         << " " << kf_output.x_.acc.transpose() << " "
                         << kf_output.x_.gravity.transpose() << " " << kf_output.x_.bg.transpose()
                         << " " << kf_output.x_.ba.transpose() << " "
                         << feats_undistort->points.size() << '\n';
              }
            }

            for (int j = 0; j < time_seq[k]; j++) {
              const int gi = idx + j + 1;
              PointType & point_body_j = feats_down_body->points[gi];
              PointType & point_world_j = feats_down_world->points[gi];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }

            solve_time += omp_get_wtime() - solve_start;

            update_time += omp_get_wtime() - t_update_start;
            idx += time_seq[k];
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());

            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (get_time_sec(imu_next.header.stamp) <
                           Measures.lidar_beg_time + lidar_time_inte) {
                      imu_deque.pop_front();
                      if (imu_deque.empty()) break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }
                  break;
                }
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;

                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;

                time_update_last = time_current;
                time_predict_last_const = time_current;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - time_predict_last_const;
                {
                  double dt_cov = time_current - time_update_last;
                  if (dt_cov > 0.0) {
                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                    time_update_last = time_current;
                  }
                  kf_output.predict(dt, Q_output, input_in, true, false);
                }

                time_predict_last_const = time_current;

                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                kf_output.update_iterated_dyn_share_IMU();
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      } else {
        effct_feat_num = 0;
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];
            time_current = point_body.curvature / 1000.0 + pcl_beg_time;
            if (is_first_frame) {
              while (time_current > get_time_sec(imu_next.header.stamp)) {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }

              is_first_frame = false;
              t_last = time_current;
              time_update_last = time_current;
              {
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              }
            }

            while (time_current > get_time_sec(imu_next.header.stamp))  // && !imu_deque.empty())
            {
              imu_deque.pop_front();

              input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                imu_last.angular_velocity.z;
              input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                imu_last.linear_acceleration.z;
              input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              double dt = get_time_sec(imu_last.header.stamp) - t_last;

              double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                time_update_last = get_time_sec(imu_last.header.stamp);  //time_current;
              }
              kf_input.predict(dt, Q_input, input_in, true, false);
              t_last = get_time_sec(imu_last.header.stamp);

              if (imu_deque.empty()) break;
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
            }
            if (flg_reset) {
              break;
            }
            double dt = time_current - t_last;
            t_last = time_current;
            double propag_start = omp_get_wtime();

            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_input.predict(dt, Q_input, input_in, true, false);

            propag_time += omp_get_wtime() - propag_start;

            double t_update_start = omp_get_wtime();

            if (feats_down_size < 1) {
              PL_WARN(LOGGER, "No point, skip this scan!\n");

              idx += time_seq[k];
              continue;
            }
            if (!kf_input.update_iterated_dyn_share_modified()) {
              idx = idx + time_seq[k];
              continue;
            }

            solve_start = omp_get_wtime();

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_aft_mapped, tf_broadcaster, tf_buffer);
              if (runtime_pos_log) {
                euler_cur = SO3ToEuler(kf_input.x_.rot);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                         << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                         << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose()
                         << " " << feats_undistort->points.size() << '\n';
              }
            }

            for (int j = 0; j < time_seq[k]; j++) {
              const int gi = idx + j + 1;
              PointType & point_body_j = feats_down_body->points[gi];
              PointType & point_world_j = feats_down_world->points[gi];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }
            solve_time += omp_get_wtime() - solve_start;

            update_time += omp_get_wtime() - t_update_start;
            idx = idx + time_seq[k];
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());
            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (get_time_sec(imu_next.header.stamp) <
                           Measures.lidar_beg_time + lidar_time_inte) {
                      imu_deque.pop_front();
                      if (imu_deque.empty()) break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }

                  break;
                }

                t_last = time_current;
                time_update_last = time_current;
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {

                double dt_cov = time_current - time_update_last;
                if (dt_cov > 0.0) {
                  time_update_last = get_time_sec(imu_next.header.stamp);  //time_current;
                }

                t_last = get_time_sec(imu_next.header.stamp);

                input_in.gyro << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                input_in.acc << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      }

      /******* Publish odometry downsample *******/
      if (!publish_odometry_without_downsample) {
        publish_odometry(pub_odom_aft_mapped, tf_broadcaster, tf_buffer);
      }

      /*** add the feature points to map ***/
      t3 = omp_get_wtime();
      const double t_map0 = t3;
      if (feats_down_size > 4) {
        if (enable_prior_pcd) {
          sleep_time++;
          if (!prior_pcd_freeze && sleep_time > 200) {
            MapIncremental();
          }
        } else {
          MapIncremental();
        }
      }

      const double t_map1 = omp_get_wtime();
      if ((loop_en || gnss_en) && feats_down_size > 0) {
        handle_loop_closure();
      }

      const double t_loop1 = omp_get_wtime();
      if (sc_enable && feats_down_size > 0) {
        Eigen::Vector3d sp; Eigen::Matrix3d sr;
        get_filter_pose(sp, sr);
        update_scan_context(sp, sr);
      }
      const double t_sc1 = omp_get_wtime();


      t5 = omp_get_wtime();
      /******* Publish points *******/
      if (path_en) publish_path(pub_path);
      if (scan_pub_en || pcd_save_en) publish_frame_world(pub_laser_cloud_full_res);
      if (scan_pub_en && scan_body_pub_en) {
        publish_frame_body(pub_laser_cloud_full_res_body);
        publish_frame_lidar(pub_laser_cloud_full_res_lidar);
      }

      FrameTimeStats::Breakdown bd;
      bd.down = (t_down1 - t1) * 1000.0;
      bd.ekf = (t3 - t_down1) * 1000.0;
      bd.map = (t_map1 - t_map0) * 1000.0;
      bd.loop = (t_loop1 - t_map1) * 1000.0;
      bd.sc = (t_sc1 - t_loop1) * 1000.0;
      bd.total = (t5 - t0) * 1000.0;
      g_frame_stats.add(bd.total, bd);
      if (feats_down_size > 0) {
        const double r = static_cast<double>(effct_feat_num) / feats_down_size;
        g_frame_stats.eff_ratio_sum += r;
        if (r < g_frame_stats.eff_ratio_min) {g_frame_stats.eff_ratio_min = r;}
        ++g_frame_stats.eff_n;
      }
      {
        const double now_wall = omp_get_wtime();
        if (g_frame_stats.last_report_t <= 0.0) {g_frame_stats.last_report_t = now_wall;}
        if (now_wall - g_frame_stats.last_report_t >= 10.0) {
          g_frame_stats.last_report_t = now_wall;
          const double eff_avg = g_frame_stats.eff_n
            ? g_frame_stats.eff_ratio_sum / g_frame_stats.eff_n : 0.0;
          PL_INFO(
            LOGGER,
            "[timing] frame mean %.2f ms  P99 %.2f ms  window max %.2f ms  all-time peak %.2f ms "
            "(%lu frames) | valid points mean %.1f%% min %.1f%% | ivox %zu cells %zu points",
            g_frame_stats.mean(), g_frame_stats.p99(), g_frame_stats.max_ms,
            g_frame_stats.max_ever_ms,
            static_cast<unsigned long>(g_frame_stats.count),
            eff_avg * 100.0, g_frame_stats.eff_ratio_min * 100.0,
            ivox_->NumValidGrids(), ivox_->NumPoints());
          {
            double zr = 0.0, zt = 0.0, zref = 0.0;
            bool zv = false;
            zc_diag(zr, zt, zref, zv);
            PL_INFO(
              LOGGER, "[vertical] mean-square n_z %.4f (thr %.2f) smoothed tilt %.2f deg (thr %.1f deg) "
              "reference height %.3f%s  current z %.3f",
              zr, zc_nz2_thresh_param, zt, zc_max_tilt_deg_param, zref,
              zv ? "" : "(invalid)",
              use_imu_as_input ? kf_input.x_.pos(2) : kf_output.x_.pos(2));
          }
          const auto & w = g_frame_stats.worst;
          PL_INFO(
            LOGGER,
            "[timing] slowest frame %.2f ms = downsample %.2f + filter %.2f + mapping %.2f + loop %.2f "
            "+ SC %.2f (other %.2f)",
            w.total, w.down, w.ekf, w.map, w.loop, w.sc,
            w.total - w.down - w.ekf - w.map - w.loop - w.sc);
          if (loop_en || gnss_en) {
            const auto st = loop_backend.GetStats();
            const double cloud_mb =
              st.cloud_points * sizeof(PointType) / (1024.0 * 1024.0);
            PL_INFO(
              LOGGER,
              "[backend] keyframes %zu (clouds held %zu / %zu cells)  %zu points ~%.1f MB  "
              "pose graph %zu nodes %zu factors",
              st.keyframes, st.clouds_held, st.cells,
              st.cloud_points, cloud_mb, st.poses, st.betweens);
          }

          if (g_frame_stats.eff_n > 100 && eff_avg < 0.30) {
            PL_ERROR(
              LOGGER,
              "valid-point ratio is only %.1f%% (normally >60%%). Likely causes: wrong lidar-IMU extrinsics "
              "(lever-arm error shows up most in turns), time-sync offset, the z slice cutting away the main "
              "structure, or geometric degeneracy (long corridor / open area). Pose still publishes but accuracy is degraded.",
              eff_avg * 100.0);
          }
          g_frame_stats.eff_ratio_sum = 0.0;
          g_frame_stats.eff_ratio_min = 1.0;
          g_frame_stats.eff_n = 0;
          g_frame_stats.max_ms = 0.0;
          g_frame_stats.worst = FrameTimeStats::Breakdown{};
        }
      }

      /*** Debug variables Logging ***/
      if (runtime_pos_log) {
        frame_num++;
        aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
        {
          aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num;
        }
        aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
        aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
        aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
        T1[time_log_counter] = Measures.lidar_beg_time;
        s_plot[time_log_counter] = t5 - t0;
        s_plot2[time_log_counter] = feats_undistort->points.size();
        s_plot3[time_log_counter] = aver_time_consu;
        time_log_counter++;
        printf(
          "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: "
          "%0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f "
          "| map grids: %zu pts: %zu (%.1f pts/grid)\n",
          t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu,
          aver_time_icp, aver_time_propag, ivox_->NumValidGrids(), ivox_->NumPoints(),
          ivox_->NumValidGrids() ? double(ivox_->NumPoints()) / ivox_->NumValidGrids() : 0.0);
        if (!publish_odometry_without_downsample) {
          if (!use_imu_as_input) {
            euler_cur = SO3ToEuler(kf_output.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                     << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                     << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose() << " "
                     << kf_output.x_.acc.transpose() << " " << kf_output.x_.gravity.transpose()
                     << " " << kf_output.x_.bg.transpose() << " " << kf_output.x_.ba.transpose()
                     << " " << feats_undistort->points.size() << '\n';
          } else {
            euler_cur = SO3ToEuler(kf_input.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                     << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                     << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                     << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose() << " "
                     << feats_undistort->points.size() << '\n';
          }
        }
        dump_lio_state_to_log(fp);
      }
    }
    if (replay_reader) {
      if (replay_done) {
        if (lidar_buffer.empty()) break;
        if (++replay_idle_rounds > 200) {
          PL_WARN(
            LOGGER, "[Replay] wrap-up: %zu cloud frames left but IMU exhausted; ending replay",
            lidar_buffer.size());
          break;
        }
      }
    } else {
      rate.sleep();
    }
  }
  if (pg_correct_valid) {
    PL_INFO(
      LOGGER, "applying loop correction before saving: %.3f m, yaw %.2f deg", pg_correct.t.norm(),
      std::atan2(pg_correct.R(1, 0), pg_correct.R(0, 0)) * 180.0 / M_PI);
  }
  if (!replay_csv.empty()) {
    std::ofstream pf(replay_csv + ".pgcorrect");
    pf << std::setprecision(12);
    if (pg_correct_valid) {
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) pf << pg_correct.R(r, c) << ' ';
      }
      pf << pg_correct.t.x() << ' ' << pg_correct.t.y() << ' ' << pg_correct.t.z() << '\n';
    } else {
      pf << "1 0 0 0 1 0 0 0 1 0 0 0\n";
    }
  }
  if (loop_en && !g_replay_kf_csv.empty()) {
    const auto kfs = loop_backend.AllKeyframePosesTimed();
    std::ofstream kfo(g_replay_kf_csv);
    kfo << "t,raw_x,raw_y,raw_z,opt_x,opt_y,opt_z,"
           "rqx,rqy,rqz,rqw,oqx,oqy,oqz,oqw\n";
    kfo << std::fixed << std::setprecision(6);
    for (const auto & e : kfs) {
      const auto & raw = std::get<1>(e);
      const auto & opt = std::get<2>(e);
      const Eigen::Quaterniond rq(raw.R), oq(opt.R);
      kfo << std::get<0>(e) << ',' << raw.t.x() << ',' << raw.t.y() << ',' << raw.t.z()
          << ',' << opt.t.x() << ',' << opt.t.y() << ',' << opt.t.z()
          << ',' << rq.x() << ',' << rq.y() << ',' << rq.z() << ',' << rq.w()
          << ',' << oq.x() << ',' << oq.y() << ',' << oq.z() << ',' << oq.w() << '\n';
    }
    PL_INFO(LOGGER, "keyframe poses (pre/post optimization) written: %s", g_replay_kf_csv.c_str());
  }

  if (pcd_save_en) {
    const SaveReport rep = save_maps_to({}, loop_en && export_patches);
    if (rep.ok) {
      PL_INFO(LOGGER, "map saved: %s", rep.message.c_str());
    } else {
      PL_WARN(LOGGER, "map not saved: %s", rep.message.c_str());
    }
  }
  {
    long zc_degen = 0, zc_total = 0;
    zc_stats(zc_degen, zc_total);
    if (zc_total > 0) {
      PL_INFO(
        LOGGER, "[vertical obs] %ld/%ld batches judged degenerate (%.1f%%)%s", zc_degen, zc_total,
        100.0 * zc_degen / zc_total,
        zc_enable_param ? ", height constraint applied" : ", constraint disabled");
    }
  }
  save_scan_context(g_pcd_dir);
  if (extrinsic_est_en) {
    const auto & eT = use_imu_as_input ? kf_input.x_.offset_T_L_I : kf_output.x_.offset_T_L_I;
    const auto & eR = use_imu_as_input ? kf_input.x_.offset_R_L_I : kf_output.x_.offset_R_L_I;
    const Eigen::Vector3d dT = eT - Lidar_T_wrt_IMU;
    const Eigen::Matrix3d dR = Lidar_R_wrt_IMU.transpose() * eR;
    const Eigen::Vector3d rpy = dR.eulerAngles(2, 1, 0) * 180.0 / M_PI;
    PL_WARN(
      LOGGER,
      "online extrinsic estimate converged: T=[%.4f %.4f %.4f] (configured [%.4f %.4f %.4f], diff %.4f m) | "
      "rotated %.3f deg from the configured value",
      eT.x(), eT.y(), eT.z(), Lidar_T_wrt_IMU.x(), Lidar_T_wrt_IMU.y(), Lidar_T_wrt_IMU.z(),
      dT.norm(), Eigen::AngleAxisd(dR).angle() * 180.0 / M_PI);
    PL_WARN(
      LOGGER, "  converged extrinsic_R: [%.6f, %.6f, %.6f,  %.6f, %.6f, %.6f,  %.6f, %.6f, %.6f]",
      eR(0, 0), eR(0, 1), eR(0, 2), eR(1, 0), eR(1, 1), eR(1, 2), eR(2, 0), eR(2, 1), eR(2, 2));
    (void)rpy;
  }
  if (replay_odom_csv.is_open()) replay_odom_csv.close();
  fout_out.close();
  return 0;
}